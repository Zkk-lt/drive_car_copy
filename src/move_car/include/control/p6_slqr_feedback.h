#pragma once
#include "control/reduced_nmpc.h"
#include <Eigen/Dense>

namespace research_control {
// Research adapter only. Original SpatialSLQR and its spatial solve are untouched.
// One steering input, center-error coordinates, explicit published-input delay queue.
class P6PolicyValidator {
 using Core=ReducedNMPC;using Clock=std::chrono::steady_clock;
 enum {D=26,N=124};
 using V=Eigen::Matrix<double,D,1>;using M=Eigen::Matrix<double,D,D>;
 using K=Eigen::Matrix<double,1,D>;
public:
 struct Policy {
  std::array<K,N> gains;int count=0,queue_size=0;double h=1./30.,stamp=0.;
  bool valid=false;std::string reason="not_built";
 };
 struct Validation {SolveResult solve;double feedback=0.,feedback_ms=0.,feedback_scale=1.;bool feedback_nonzero=false,feedback_bounded=false;};
 static State nominalAt(const SolveResult& nominal,const SolveRequest&r,const SolverConfig&c,double t){
  if(t<=0.)return nominal.predicted[0];
  for(int i=0;i+1<nominal.count;++i){const double begin=Core::startTime(i,r,c),end=begin+Core::interval(i,r,c);
   if(t<=end||i+2==nominal.count){const double f=clamp((t-begin)/(end-begin),0.,1.);const auto&a=nominal.predicted[i];const auto&b=nominal.predicted[i+1];
    return {a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),wrap(a.yaw+f*wrap(b.yaw-a.yaw)),a.speed+f*(b.speed-a.speed),a.steering+f*(b.steering-a.steering),a.acceleration+f*(b.acceleration-a.acceleration)};}}
  return nominal.predicted[nominal.count-1];
 }
 static Command held(const std::vector<CommandSample>&h,double t,bool*known=nullptr){
  auto i=std::upper_bound(h.begin(),h.end(),t,[](double time,const CommandSample&s){return time<s.stamp;});
  if(known)*known=i!=h.begin();return i==h.begin()?Command{}:(i-1)->command;
 }
 static void actuatorCoefficients(double delay,double tau,double gain,double h,int&m,double&a,double&bo,double&bn){
  m=static_cast<int>(std::floor(delay/h+1e-10));const double rho=clamp(delay/h-m,0.,1.);
  a=std::exp(-h/tau);const double partial=std::exp(-(1.-rho)*h/tau);
  bo=gain*(partial-a);bn=gain*(1.-partial);
 }
 // The optimization cost observes the CENTER course angle, while the local
 // state contains BODY yaw.  d(course)/d(delta) is generally nonzero (0.5
 // on the nominal straight vehicle).  Use C'QC, not a body-yaw-only penalty.
 static double courseSteeringJacobian(const State&z,const Model&m){
  const double cs=std::cos(z.steering);
  const double rd=m.yaw_gain*z.speed/(m.wheelbase*cs*cs);
  const double r=m.yaw_gain*z.speed*std::tan(z.steering)/m.wheelbase+m.yaw_residual;
  const double vy=m.lateral_residual+m.rear_to_center*r;
  const double den=z.speed*z.speed+vy*vy;
  return den>1e-12?m.rear_to_center*rd*z.speed/den:0.;
 }
 static bool build(const SolveRequest&r,const SolverConfig&c,const SolveResult&nominal,Policy&p,Clock::time_point deadline,bool course_consistent=false){
  p.valid=false;p.stamp=r.now;p.h=1./30.;
  if(!nominal.ok||nominal.count!=c.steps+1||r.state.speed<.015){p.reason="feedback_nominal_invalid_or_low_speed";return false;}
  if(!std::isfinite(c.model.steering_delay)||c.model.steering_delay<0.||c.model.steering_delay>.7||
     !std::isfinite(c.model.steering_tau)||c.model.steering_tau<=0.){p.reason="feedback_delay_capacity";return false;}
  // Exact immutable-model delay support, including its fractional old-input
  // tap. The old fixed 0.7s queue added states that could never affect the
  // plant or cost (21 instead of 10 for 0.325s). Keep the same 0.7s capacity.
  p.queue_size=std::max(1,static_cast<int>(std::ceil(c.model.steering_delay/p.h)));const int dim=3+p.queue_size;
  if(dim>D||c.model.steering_delay>.7){p.reason="feedback_delay_capacity";return false;}
  const double horizon=r.control_dt+(c.steps-1)*c.dt;p.count=std::min(static_cast<int>(N),static_cast<int>(std::ceil(horizon/p.h)));
  M P=M::Zero();P(0,0)=90.;P(1,1)=40.;P(2,2)=.55;
  if(course_consistent){const double b=courseSteeringJacobian(nominalAt(nominal,r,c,horizon),c.model);
   P(1,2)=P(2,1)=40.*b;P(2,2)+=40.*b*b;}
  const double rr=c.weights.smoothness*.01/p.h,ru=.55*p.h;
  int m=0;double a,bo,bn;actuatorCoefficients(c.model.steering_delay,c.model.steering_tau,c.model.steering_gain,p.h,m,a,bo,bn);
  struct TransitionColumn {
   int count=0,rows[3]{};
   double coefficients[3]{};
   void append(int row,double coefficient){
    if(coefficient!=0.){rows[count]=row;coefficients[count++]=coefficient;}
   }
  };
  for(int k=p.count-1;k>=0;--k){
   if(Clock::now()>=deadline){p.reason="feedback_budget_exhausted";return false;}
   const auto z=nominalAt(nominal,r,c,k*p.h);if(z.speed<.015){p.reason="feedback_nominal_low_speed";return false;}
   std::array<TransitionColumn,D> A{};
   M Q=M::Zero();V B=V::Zero(),cross=V::Zero();
   const double sec=1./std::cos(z.steering),rd=c.model.yaw_gain*z.speed*sec*sec/c.model.wheelbase;
   const double nominal_r=c.model.yaw_gain*z.speed*std::tan(z.steering)/c.model.wheelbase+c.model.yaw_residual;
   const double center_vy=c.model.lateral_residual+c.model.rear_to_center*nominal_r;
   const double beta=std::atan2(center_vy,z.speed);
   A[0].append(0,1.);
   A[1].append(0,p.h*std::hypot(z.speed,center_vy));A[1].append(1,1.);
   A[2].append(0,p.h*c.model.rear_to_center*rd*std::cos(beta));
   A[2].append(1,p.h*rd);A[2].append(2,a);
   if(m==0)B(2)=bn;else if(m<=p.queue_size)A[3+m-1].append(2,bn);else {p.reason="feedback_delay_queue_short";return false;}
   if(bo>1e-14){if(m+1>p.queue_size){p.reason="feedback_fractional_queue_short";return false;}A[3+m].append(2,bo);}
   B(3)=1.;for(int j=1;j<p.queue_size;++j)A[3+j-1].append(3+j,1.);
   Q(0,0)=c.weights.lateral*p.h;Q(1,1)=c.weights.heading*p.h;Q(2,2)=.55*p.h;Q(3,3)=rr;cross(3)=-rr;
   if(course_consistent){const double b=courseSteeringJacobian(z,c.model);
    Q(1,2)=Q(2,1)=c.weights.heading*p.h*b;Q(2,2)+=c.weights.heading*p.h*b*b;}
   // A contains the three local states and a shift register: every column
   // has at most three nonzeros. Sparse products are algebraically identical
   // to the dense Riccati recursion; all workspaces stay fixed and on stack.
   M PA=M::Zero();
   for(int j=0;j<dim;++j)for(int entry=0;entry<A[j].count;++entry)
    for(int i=0;i<dim;++i)PA(i,j)+=P(i,A[j].rows[entry])*A[j].coefficients[entry];
   K G=cross.transpose();double H=ru+rr;
   for(int i=0;i<dim;++i)if(B(i)!=0.){
    for(int j=0;j<dim;++j){G(j)+=B(i)*PA(i,j);if(B(j)!=0.)H+=B(i)*P(i,j)*B(j);}
   }
   if(!std::isfinite(H)||H<=1e-12){p.reason="feedback_riccati_invalid";return false;}
   p.gains[k]=G/H;M next=Q;
   for(int i=0;i<dim;++i)for(int entry=0;entry<A[i].count;++entry)
    for(int j=0;j<dim;++j)next(i,j)+=A[i].coefficients[entry]*PA(A[i].rows[entry],j);
   for(int i=0;i<dim;++i)for(int j=0;j<dim;++j)next(i,j)-=G(i)*G(j)/H;
   P=.5*(next+next.transpose()).eval();
   if(!P.allFinite()||!p.gains[k].allFinite()){p.reason="feedback_riccati_nonfinite";return false;}
  }
  p.valid=true;p.reason=course_consistent?"delay_augmented_center_course_local_feedback":"delay_augmented_single_input_local_feedback";return true;
 }
 // One fresh measurement at validation time, expressed in the nominal local frame.
 // All resulting physical inputs are then checked by the SAME nonlinear rollout.
 static Validation validate(const SolveRequest&r,const SolverConfig&c,const SolveResult&nominal,
  const Policy&p,const State&measured,double validation_stamp,const std::vector<CommandSample>&actual_history,
  Clock::time_point deadline,bool enabled=true,bool bounded_feedback=false){
  const auto start=Clock::now();Validation out;out.solve.command=r.previous;out.solve.decision_dimension=nominal.decision_dimension;
  auto fail=[&](const char*why){out.solve.reason=why;out.feedback_ms=Core::milliseconds(start);return out;};
  if(!p.valid||p.count<=0||p.count>N||p.queue_size<1||p.queue_size>D-3||
     !std::isfinite(p.h)||p.h<=0.||!std::isfinite(validation_stamp)||!std::isfinite(r.now)||
     !Core::validState(measured)||validation_stamp<r.now||validation_stamp-r.now>.075)
    return fail("feedback_validation_state_or_time_invalid");
  double last_stamp=-INFINITY;for(const auto&h:actual_history){if(!std::isfinite(h.stamp)||h.stamp<=last_stamp||h.stamp>validation_stamp||!Core::finiteCommand(h.command))return fail("feedback_future_or_invalid_actual_history");last_stamp=h.stamp;}
  if(actual_history.empty()||r.history.empty()||actual_history.back().stamp!=r.history.back().stamp||
    actual_history.back().command.speed!=r.history.back().command.speed||actual_history.back().command.steering!=r.history.back().command.steering)return fail("feedback_history_advanced_requires_new_context");
  const double offset=validation_stamp-r.now;const auto z=nominalAt(nominal,r,c,offset);
  const double l=c.model.rear_to_center,dx=measured.x+l*std::cos(measured.yaw)-z.x-l*std::cos(z.yaw);
  const double dy=measured.y+l*std::sin(measured.yaw)-z.y-l*std::sin(z.yaw);
  const double nominal_r=c.model.yaw_gain*z.speed*std::tan(z.steering)/c.model.wheelbase+c.model.yaw_residual;
  const double center_tangent=z.yaw+std::atan2(c.model.lateral_residual+l*nominal_r,z.speed);
  const double along=std::cos(center_tangent)*dx+std::sin(center_tangent)*dy;
  V error=V::Zero();error(0)=-std::sin(center_tangent)*dx+std::cos(center_tangent)*dy;error(1)=wrap(measured.yaw-z.yaw);error(2)=measured.steering-z.steering;
  // These domain limits are inherited P5 engineering pair-checks, not real calibration.
  if(std::abs(along)>.03||std::abs(error(0))>.03||std::abs(error(1))>.10||std::abs(error(2))>.12)return fail("feedback_outside_engineering_domain");
  for(int j=0;j<p.queue_size;++j){bool good=false;const double time=validation_stamp-(j+1)*p.h;
   const auto real=held(actual_history,time,&good);if(!good)return fail("feedback_actual_queue_incomplete");
   const auto old=held(r.history,time,&good);if(!good)return fail("feedback_nominal_queue_incomplete");error(3+j)=real.steering-old.steering;}
  const int k=std::min(p.count-1,static_cast<int>(offset/p.h));
  out.feedback=enabled?-(p.gains[k]*error)(0,0):0.;out.feedback_nonzero=std::abs(out.feedback)>1e-10;
  auto commands=nominal.controls;commands[0].steering+=out.feedback;
  SolveRequest fresh=r;fresh.state=measured;fresh.now=validation_stamp;fresh.history=actual_history;
  if(offset>0.){
   if(offset+r.control_dt+(c.steps-1)*c.dt>r.reference.points[r.reference.count-1].t)return fail("feedback_shifted_reference_horizon_missing");
   for(int i=0;i<fresh.reference.count;++i){auto point=Core::referenceAt(r.reference,r.reference.points[i].t+offset);point.t=r.reference.points[i].t;fresh.reference.points[i]=point;}
  }
  // Re-anchor state/command verification to the actual validation time, never pretend
  // this measurement existed at the earlier optimization stamp.
  Core core;SolveProfile profile;if(!core.prepare(fresh,c,profile,false))return fail("feedback_schedule_capacity");
  bool stopped=false;auto roll=Core::rollout(Core::Decision{},fresh,c,core.prepared_,deadline,stopped,profile,&commands);
  if(stopped||Clock::now()>=deadline){out.solve.timed_out=true;return fail("feedback_validation_timeout");}
  if(!roll.feasible&&enabled&&bounded_feedback&&out.feedback_nonzero){
    // The Riccati correction minimizes a strictly convex scalar quadratic.
    // Search its segment from zero to the unconstrained correction, retaining
    // ONLY fully feasible nonlinear rollouts. This is constrained local
    // feedback, not actuator clipping or mixing two full controllers.
    // If zero itself is infeasible at the fresh state, refuse the candidate.
    const double requested=out.feedback;commands[0].steering=nominal.controls[0].steering;
    const auto baseline_tick=Clock::now();
    auto feasible=Core::rollout(Core::Decision{},fresh,c,core.prepared_,deadline,stopped,profile,&commands);
    double worst_ms=Core::milliseconds(baseline_tick);
    if(stopped||Clock::now()>=deadline){out.solve.timed_out=true;return fail("feedback_validation_timeout");}
    if(!feasible.feasible)return fail("feedback_nominal_and_combined_input_infeasible");
    double lo=0.,hi=1.;
    // Tighten the scalar search using the issued steering limits at the
    // first two intervals. The second input itself is unchanged, but its
    // rate constraint also depends on the corrected first input.
    const double first=nominal.controls[0].steering;
    const double first_step=c.limits.steering_rate*fresh.control_dt;
    double lower=std::max(-c.limits.steering,fresh.previous.steering-first_step);
    double upper=std::min(c.limits.steering,fresh.previous.steering+first_step);
    if(c.steps>1){
      const double next_step=c.limits.steering_rate*c.dt;
      lower=std::max(lower,nominal.controls[1].steering-next_step);
      upper=std::min(upper,nominal.controls[1].steering+next_step);
    }
    const double issued_scale=clamp(((requested>0.?upper:lower)-first)/requested,0.,1.);
    if(issued_scale>0.&&issued_scale<1.&&Clock::now()+Core::duration(1.5*worst_ms+.02)<deadline){
      hi=issued_scale;
      commands[0].steering=first+hi*requested;
      const auto tick=Clock::now();
      const auto trial=Core::rollout(Core::Decision{},fresh,c,core.prepared_,deadline,stopped,profile,&commands);
      worst_ms=std::max(worst_ms,Core::milliseconds(tick));
      if(stopped||Clock::now()>=deadline){out.solve.timed_out=true;return fail("feedback_validation_timeout");}
      if(trial.feasible){lo=hi;feasible=trial;}
    }
    for(int iteration=0;iteration<4&&hi-lo>1e-9;++iteration){
      if(Clock::now()+Core::duration(1.5*worst_ms+.02)>=deadline)break;
      const double alpha=.5*(lo+hi);commands[0].steering=nominal.controls[0].steering+alpha*requested;
      const auto tick=Clock::now();const auto trial=Core::rollout(Core::Decision{},fresh,c,core.prepared_,deadline,stopped,profile,&commands);
      worst_ms=std::max(worst_ms,Core::milliseconds(tick));
      if(stopped||Clock::now()>=deadline){out.solve.timed_out=true;return fail("feedback_validation_timeout");}
      if(trial.feasible){lo=alpha;feasible=trial;}else hi=alpha;
    }
    roll=feasible;out.feedback_scale=lo;out.feedback=lo*requested;
    out.feedback_nonzero=std::abs(out.feedback)>1e-10;out.feedback_bounded=true;
  }
  if(!roll.feasible)return fail("feedback_combined_input_infeasible");
  out.solve=nominal;out.solve.command=roll.commands[0];out.solve.command_acceleration=roll.command_accelerations[0];
  out.solve.controls=roll.commands;out.solve.predicted=roll.states;out.solve.objective=roll.cost;out.solve.max_violation=roll.violation;
  out.solve.reason=out.feedback_bounded?"feedback_bounded_rollout_feasible":
    (enabled?"feedback_combined_rollout_feasible":"feedback_disabled_same_rollout");
  out.feedback_ms=Core::milliseconds(start);return out;
 }
};
}
