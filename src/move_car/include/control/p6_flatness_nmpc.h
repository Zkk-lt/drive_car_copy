#pragma once
#include "control/reduced_nmpc.h"
#include "control/p6_flat_response_base.h"
#include <memory>

namespace research_control {
// Research-only flat-output NMPC.  A clamped quintic B-spline has seven
// control points in each planar coordinate.  The first three are fixed by the
// measured initial position/velocity/acceleration jet; eight remain free.
//
// The default chart replaces P3.x/P3.y by the FIRST physical input coordinates
// (command acceleration and steering rate) through a locally invertible 2x2
// reconstruction.  P4..P6 are the other six coordinates.  This is an eight-
// dimensional change of coordinates on the spline, not command clipping.  Every
// reconstructed command is checked by the original delayed Core::rollout.
class P6FlatnessNMPC {
  using Core=ReducedNMPC;
  using Clock=std::chrono::steady_clock;
 public:
  using Coefficients=std::array<double,8>;
  struct Curve {
    std::array<double,13> knots{};
    std::array<std::array<double,7>,2> control{};
    std::array<double,2> decision_scale{{0.,0.}};
    std::array<double,8> geometry_scale{}; // implicit-first-input conditioned coordinates
    double horizon=0.,yaw=0.;
    bool delay_aligned=false;
    std::shared_ptr<const P6FlatResponseBase> response_base;
    FlatOutputJet response_anchor;
  };
  struct Basis { std::array<std::array<double,7>,4> d{};FlatOutputJet offset; };
  using BasisGrid=std::array<Basis,kMaxHorizon>;
 private:
  using JetTerms=std::array<std::array<double,4>,2>;
 public:
  struct Diagnostics {
    double interior_knot=0.,reference_fit_rmse=missing(),warm_fit_rmse=missing();
    double first_inverse_error=missing();
    int inverse_failures=0,reference_fit_samples=0;
    bool nonuniform=false,first_input_chart=true;
    bool near_knot_restoration=false;
    bool feasibility_recovery_attempted=false,feasibility_recovered=false;
    bool accuracy_refinement=false,warm_geometry_preserved=false;
    int refinement_improvements=0;
    bool conditioned_geometry=false,baseline_seed_accepted=false,delay_aligned_reconstruction=false;
    bool independent_input_encoding=false;
    double baseline_fit_rmse=missing();
    int tracking_directions=0,tracking_accepted=0;
    double tracking_linearized_objective=missing(),tracking_step_norm=0.;
    bool coarse_poll_skipped=false;
    int fine_poll_evaluations=0,fine_poll_improvements=0;
    int coarse_poll_sweeps=0,local_poll_stop_stage=0; // 1: fine, 2: coarse
    double fine_poll_cost_before=missing(),fine_poll_cost_after=missing();
    bool causal_reference_seed=false,response_knot_monitor=false;
    int rejected_rollouts=0,first_rejected_step=-1;
    const char* first_rejection="none";
    int input_sqp_steps=0;
    double input_merit_before=missing(),input_merit_after=missing();
    bool response_base_used=false,terminal_seed_used=false;double response_base_cost=missing();
    bool response_recovery_seed_attempted=false,response_recovery_seed_used=false;
    bool response_warm_available=false,response_warm_accepted=false,response_warm_reconstruction_failed=false;
    int response_warm_evaluations=0;
    double response_warm_cost_before=missing(),response_warm_cost_after=missing();
    bool steering_rate_history_valid=false,steering_rate_continuity_active=false;
    double steering_rate_prior=missing(),steering_rate_continuity_weight=0.,steering_rate_continuity_cost=0.;
  };
  static constexpr int dimension(){return 8;}
  void setNonuniformKnots(bool value){nonuniform_=value;}
  void setFirstInputChart(bool value){first_input_chart_=value;}
  void setNearKnotRestoration(bool value){near_knot_restoration_=value;}
  void setFeasibilityRecovery(bool value){feasibility_recovery_=value;}
  void setAccuracyRefinement(bool value){accuracy_refinement_=value;}
  void setConditionedGeometry(bool value){conditioned_geometry_=value;}
  void setBaselineSeed(bool value){baseline_seed_=value;}
  void setDelayAlignedReconstruction(bool value){delay_aligned_reconstruction_=value;}
  void setIndependentInputEncoding(bool value){independent_input_encoding_=value;}
  void setCommandProjection(bool value){command_projection_=value;}
  void setCausalReferenceSeed(bool value){causal_reference_seed_=value;}
  void setResponseKnotMonitor(bool value){response_knot_monitor_=value;}
  void setInputFeasibilitySQP(bool value){input_feasibility_sqp_=value;}
  void setResponseBase(bool value){response_base_=value;}
  void setResponseNearSupport(bool value){response_near_support_=value;}
  void setResponseWarmStart(bool value){response_warm_start_=value;}
  void setBudgetedPoll(bool value){budgeted_poll_=value;}
  // Relative performance regularization, not an actuator identification or a
  // physical bound. Zero retains the original flat8 objective exactly.
  void setSteeringRateContinuityWeight(double value){steering_rate_continuity_weight_=
    std::isfinite(value)&&value>0.?value:0.;}
  struct PublishedSteeringRate {
    double value=0.,interval=0.;bool valid=false;
    const char* reason="published_history_unverified";
  };
  static PublishedSteeringRate publishedSteeringRate(const SolveRequest&r){
    PublishedSteeringRate out;
    if(!r.published_history_contiguous||r.history.size()<2)return out;
    const auto& last=r.history.back();const auto& before=r.history[r.history.size()-2];
    if(!std::isfinite(r.now)||!std::isfinite(last.stamp)||!std::isfinite(before.stamp)||
       !std::isfinite(last.command.steering)||!std::isfinite(before.command.steering)||
       !std::isfinite(r.previous.steering)||last.stamp>r.now||before.stamp>=last.stamp){
      out.reason="published_steering_time_or_value_invalid";return out;
    }
    const double dt=last.stamp-before.stamp;
    // These are inherited P5 receipt/publication contracts, not recovered
    // hardware sample times. Never invent an input over a missing sequence.
    if(dt<.001||dt>.12||r.now-last.stamp>.12||
       std::abs(last.command.steering-r.previous.steering)>1e-9){
      out.reason="published_steering_gap_or_context_mismatch";return out;
    }
    out.value=(last.command.steering-before.command.steering)/dt;
    out.interval=dt;out.valid=std::isfinite(out.value);
    out.reason=out.valid?"causal_published_steering_rate":"published_steering_rate_nonfinite";
    return out;
  }
  // Publication jitter can make the measured interval slightly shorter
  // than the interval used for the command transition. Project this causal
  // prior into the admissible rate range for the soft objective only.
  static PublishedSteeringRate steeringRatePrior(const SolveRequest&r,const SolverConfig&c){
    auto rate=publishedSteeringRate(r);
    if(!rate.valid||!std::isfinite(c.limits.steering_rate)||c.limits.steering_rate<=0.){
      rate.valid=false;return rate;
    }
    rate.value=clamp(rate.value,-c.limits.steering_rate,c.limits.steering_rate);
    return rate;
  }
  // The rates belong to interval midpoints. Integrating squared angular
  // acceleration weights a rate difference by inverse midpoint separation.
  // Normalize by dt^2 to retain the tuning scale on the regular grid.
  static double steeringRateChangeWeight(double h,double previous_h,const SolverConfig&c,double weight){
    return .01*c.weights.smoothness*weight*c.dt*c.dt/(.5*(h+previous_h));
  }

  static double steeringRateContinuityCost(const SolveRequest&r,const SolverConfig&c,
      const std::array<Command,kMaxHorizon>& commands,double weight){
    if(!(weight>0.)||!std::isfinite(weight))return 0.;
    const auto prior=steeringRatePrior(r,c);if(!prior.valid)return 0.;
    return steeringRateContinuityCost(r,c,commands,weight,prior);
  }
  static double steeringRateContinuityCost(const SolveRequest&r,const SolverConfig&c,
      const std::array<Command,kMaxHorizon>& commands,double weight,const PublishedSteeringRate& prior){
    double steering=r.previous.steering,rate=prior.value,previous_h=prior.interval,cost=0.;
    for(int i=0;i<c.steps;++i){const double h=Core::interval(i,r,c);
      const double next_rate=(commands[i].steering-steering)/h;
      const double difference=next_rate-rate;
      cost+=steeringRateChangeWeight(h,previous_h,c,weight)*difference*difference;
      steering=commands[i].steering;rate=next_rate;previous_h=h;
    }
    return cost;
  }
  void reset(){warm_=false;}
  const Diagnostics& diagnostics()const{return diagnostics_;}
  const Coefficients& lastCoefficients()const{return selected_;}
  const Curve& lastCurve()const{return previous_curve_;}

  struct LocalRearReference {
    std::array<std::array<double,2>,kMaxHorizon+1> positions{};
    std::array<double,kMaxHorizon+1> times{},body_rates{};
    bool valid=false;
  };
  // The body-center curve is NOT a flat output. Its rear-axle lift requires
  // an initial body heading, supplied by the causal local state, and obeys
  // psi_b_dot = v_c sin(psi_c-psi_b)/ell. Unlike a steady-circle conversion,
  // this nonholonomic transport does not assume ell*curvature < 1 at every
  // future point. It is only a seed; Core still checks the ORIGINAL reference,
  // reconstructed inputs and delayed physical trajectory, without clipping.
  static LocalRearReference localRearReference(const SolveRequest& r,const SolverConfig& c){
    LocalRearReference out;
    if(c.steps<2||c.steps>kMaxHorizon||!std::isfinite(c.dt)||c.dt<=0.||
       !std::isfinite(r.control_dt)||r.control_dt<=0.||
       !Core::validReference(r.reference,c)||!std::isfinite(r.state.yaw)||
       !std::isfinite(c.model.rear_to_center)||c.model.rear_to_center<0.)return out;
    double body=r.state.yaw,last_time=0.;const double ell=c.model.rear_to_center;
    for(int i=0;i<c.steps;++i){
      const double time=Core::startTime(i,r,c)+Core::interval(i,r,c);
      const auto p=Core::referenceAt(r.reference,time);
      if(ell>1e-6){const double h=(time-last_time)/4.;
        for(int j=0;j<4;++j){const auto center=Core::referenceAt(r.reference,last_time+(j+.5)*h);
          const double rate=center.speed*std::sin(center.yaw-body)/ell;
          body=wrap(body+h*center.speed*std::sin(center.yaw-body-.5*h*rate)/ell);}
      }else body=p.yaw;
      out.times[i]=time;
      out.positions[i]={{p.x-ell*std::cos(body),p.y-ell*std::sin(body)}};
      out.body_rates[i]=ell>1e-6?p.speed*std::sin(p.yaw-body)/ell:p.speed*p.curvature;
      if(!std::isfinite(out.positions[i][0])||!std::isfinite(out.positions[i][1])||
         !std::isfinite(out.body_rates[i]))return out;
      last_time=time;
    }
    out.valid=true;return out;
  }
  // Discrete total-variation monitor with units of time. Multiplying an
  // already integrated delta-yaw/delta-rate by dt again makes adaptation
  // disappear as the reference grid is refined. The center-to-rear heading
  // ODE supplies the response rate, rather than differentiating noisy raw
  // curvature. The unique interior knot is still only a chart choice: 8 DOFs.
  static double responseKnotFraction(const SolveRequest& r,const SolverConfig& c,
      double H,const LocalRearReference& rear){
    if(!rear.valid||c.steps<2||c.steps>kMaxHorizon||!std::isfinite(c.dt)||c.dt<=0.||
       !std::isfinite(r.control_dt)||r.control_dt<=0.||!std::isfinite(H)||H<=0.)return .5;
    std::array<double,kMaxHorizon> mass{};double total=0.;
    const auto initial=Core::referenceAt(r.reference,0.);
    double last_yaw=initial.yaw;
    double last_rate=c.model.rear_to_center>1e-6?
      initial.speed*std::sin(initial.yaw-r.state.yaw)/c.model.rear_to_center:initial.speed*initial.curvature;
    for(int i=0;i<c.steps;++i){
      const auto p=Core::referenceAt(r.reference,rear.times[i]);
      mass[i]=Core::interval(i,r,c)+H*std::abs(wrap(p.yaw-last_yaw))+
        H*H*std::abs(rear.body_rates[i]-last_rate);
      total+=mass[i];last_yaw=p.yaw;last_rate=rear.body_rates[i];
    }
    double cumulative=0.,time=.5*H;
    for(int i=0;i<c.steps;++i){if(cumulative+mass[i]>=.5*total){
        time=Core::startTime(i,r,c)+Core::interval(i,r,c)*(.5*total-cumulative)/mass[i];break;}
      cumulative+=mass[i];}
    const double edge=std::min(.25,2.*r.control_dt/H);
    return clamp(time/H,std::max(.05,edge),std::min(.95,1.-edge));
  }

  static Curve anchoredCurve(const State& s,const Model& m,double H,double interior=.5){
    Curve q;q.horizon=H;q.yaw=s.yaw;
    const double knot=clamp(interior,.05,.95)*H;
    for(int i=0;i<13;++i)q.knots[i]=i<6?0.:(i==6?knot:H);
    const double co=std::cos(s.yaw),si=std::sin(s.yaw);
    // Strict nominal bicycle jet.  Effective-model residuals and actuator
    // dynamics are checked in the common forward rollout, not called flat.
    const double normal=s.speed*(m.yaw_gain*s.speed*std::tan(s.steering)/m.wheelbase+m.yaw_residual);
    const double velocity[2]={s.speed*co,s.speed*si};
    const double acceleration[2]={s.acceleration*co-normal*si,s.acceleration*si+normal*co};
    const double position[2]={s.x,s.y};
    // Polynomial blossom: this exactly represents the anchored quadratic.
    for(int i=0;i<7;++i){
      double sum=0.,pairs=0.;
      for(int j=1;j<=5;++j){sum+=q.knots[i+j];for(int k=j+1;k<=5;++k)pairs+=q.knots[i+j]*q.knots[i+k];}
      for(int axis=0;axis<2;++axis)q.control[axis][i]=position[axis]+velocity[axis]*sum/5.+acceleration[axis]*pairs/20.;
    }
    return q;
  }

  // Cox-de Boor recurrence differentiated analytically through order three.
  // All scratch arrays are fixed-size.  The left limit at the final clamped
  // endpoint gives exactly the correct endpoint value and derivatives.
  static Basis basis(const Curve& q,double time){
    Basis out;std::array<std::array<double,13>,4> a{},b{};
    const double t=clamp(time,0.,q.horizon);
    int span=-1;
    for(int i=0;i<12;++i){
      if((q.knots[i]<=t&&t<q.knots[i+1])||
         (t==q.horizon&&q.knots[i]<q.horizon&&q.knots[i+1]==q.horizon)){span=i;break;}
    }
    if(span>=0)a[0][span]=1.;
    for(int degree=1;degree<=5;++degree){
      b={};
      // All derivatives have the same local support. Skip zero terms
      // outside the active span, preserving arithmetic on its support.
      for(int i=std::max(0,span-degree);span>=0&&i<=std::min(span,11-degree);++i){
        const double dl=q.knots[i+degree]-q.knots[i],dr=q.knots[i+degree+1]-q.knots[i+1];
        for(int d=0;d<=3;++d){
          if(dl>0.){b[d][i]+=(t-q.knots[i])*a[d][i]/dl;if(d)b[d][i]+=d*a[d-1][i]/dl;}
          if(dr>0.){b[d][i]+=(q.knots[i+degree+1]-t)*a[d][i+1]/dr;if(d)b[d][i]-=d*a[d-1][i+1]/dr;}
        }
      }a=b;
    }
    for(int d=0;d<4;++d)for(int i=0;i<7;++i)out.d[d][i]=a[d][i];
    if(q.response_base){
      // The model-predicted base is sampled at exact reconstruction times.
      // Never invent intermediate sensor samples or interpolate event jets.
      const double actual=std::max(0.,time);int sample=-1;
      for(int i=0;i<q.response_base->count;++i)
        if(std::abs(q.response_base->times[i]-actual)<1e-9){sample=i;break;}
      if(sample<0){out.offset.dx=missing();return out;}
      if(actual>q.horizon){
        const double h=actual-q.horizon;
        for(int k=0;k<7;++k){const double b0=out.d[0][k],b1=out.d[1][k],b2=out.d[2][k],b3=out.d[3][k];
          out.d[0][k]=b0+h*b1+.5*h*h*b2+h*h*h*b3/6.;
          out.d[1][k]=b1+h*b2+.5*h*h*b3;out.d[2][k]=b2+h*b3;}
      }
      const auto& a=q.response_anchor;const auto& b=q.response_base->jets[sample];
      out.offset={b.x-(a.x+actual*a.dx+.5*actual*actual*a.ddx),
        b.y-(a.y+actual*a.dy+.5*actual*actual*a.ddy),b.dx-(a.dx+actual*a.ddx),
        b.dy-(a.dy+actual*a.ddy),b.ddx-a.ddx,b.ddy-a.ddy,b.dddx,b.dddy};
    }
    return out;
  }
 private:
  template<int Begin,int End>
  static void addJetTerms(JetTerms& v,const Curve& q,const Basis& weights){
    for(int axis=0;axis<2;++axis)for(int d=0;d<4;++d)for(int i=Begin;i<End;++i)
      v[axis][d]+=weights.d[d][i]*q.control[axis][i];
  }
  static FlatOutputJet finishJet(const JetTerms& v,const Basis& weights){
    FlatOutputJet out;out.x=v[0][0];out.y=v[1][0];out.dx=v[0][1];out.dy=v[1][1];
    out.ddx=v[0][2];out.ddy=v[1][2];out.dddx=v[0][3];out.dddy=v[1][3];
    const auto& a=weights.offset;out.x+=a.x;out.y+=a.y;out.dx+=a.dx;out.dy+=a.dy;
    out.ddx+=a.ddx;out.ddy+=a.ddy;out.dddx+=a.dddx;out.dddy+=a.dddy;return out;
  }
 public:
  static FlatOutputJet jet(const Curve& q,const Basis& weights){
    JetTerms v{};addJetTerms<0,7>(v,q,weights);return finishJet(v,weights);
  }
  static FlatOutputJet jet(const Curve& q,double time){return jet(q,basis(q,time));}

  // Exact derivative of the inverse nominal actuators with respect to P3.x/y.
  // This changes only the two-coordinate spline chart inversion. The online
  // optimizer remains bounded pattern search and every input still passes the
  // common delayed nonlinear rollout. No differencing step/tuning is required.
  static bool firstInputJacobian(const FlatOutputJet& q,const Basis& b,
      const Model& m,double J[2][2],int control_index=3){
    return firstInputJacobianCached(q,b,m,FlatnessReferenceModel::reconstructForInputs(q,m),J,control_index);
  }
  // Reuse the identical nominal reconstruction for all four CP derivatives at
  // this knot. The formulas and inverse-actuator model remain unchanged.
  static bool firstInputJacobianCached(const FlatOutputJet& q,const Basis& b,
      const Model& m,const FlatReconstruction& flat,double J[2][2],int control_index=3){
    if(!flat.valid)return false;
    const double v=flat.state.speed,v2=v*v,v3=v2*v,v4=v2*v2,v5=v4*v,v6=v3*v3;
    const double dot=q.dx*q.ddx+q.dy*q.ddy;
    const double cross=q.dx*q.ddy-q.dy*q.ddx;
    const double cross_rate=q.dx*q.dddy-q.dy*q.dddx;
    if(control_index<3||control_index>6)return false;
    const double A=b.d[1][control_index],B=b.d[2][control_index],C=b.d[3][control_index];
    const double k=flat.curvature,kr=cross_rate/v3-3.*cross*dot/v5;
    const double effective=(k-m.yaw_residual/v)/m.yaw_gain;
    const double effective_rate=(kr+m.yaw_residual*flat.state.acceleration/v2)/m.yaw_gain;
    const double D=m.wheelbase*effective,den=1.+D*D;
    for(int axis=0;axis<2;++axis){
      const double p=axis?q.dy:q.dx,a=axis?q.ddy:q.ddx;
      const double dv=A*p/v,ddot=A*a+B*p;
      const double da=ddot/v-dot*dv/v2;
      const double dcross=axis?(-A*q.ddx+B*q.dx):(A*q.ddy-B*q.dy);
      const double dcross_rate=axis?(-A*q.dddx+C*q.dx):(A*q.dddy-C*q.dy);
      const double dk=dcross/v3-3.*cross*dv/v4;
      const double dkr=dcross_rate/v3-3.*cross_rate*dv/v4-
        3.*(dcross*dot+cross*ddot)/v5+15.*cross*dot*dv/v6;
      const double de=(dk+m.yaw_residual*dv/v2)/m.yaw_gain;
      const double der=(dkr+m.yaw_residual*(da/v2-2.*flat.state.acceleration*dv/v3))/m.yaw_gain;
      J[0][axis]=(dv+m.speed_tau*da)/m.speed_gain;
      J[1][axis]=(m.wheelbase*de/den+m.steering_tau*m.wheelbase*
        (der/den-2.*D*m.wheelbase*de*effective_rate/(den*den)))/m.steering_gain;
      if(!std::isfinite(J[0][axis])||!std::isfinite(J[1][axis]))return false;
    }
    return true;
  }

  // Invert the two nominal actuators at their distinct activation times.
  // Beyond the finite spline horizon, hold its endpoint inverse input (ZOH).
  // Delay is still applied by the common physical forward rollout.
  static double inputTime(const Curve& q,double time,double delay){
    const double t=time+(q.delay_aligned?delay:0.);
    return q.response_base?std::max(0.,t):clamp(t,0.,q.horizon);
  }
  static bool commandAt(const Curve& q,double time,const Model& m,Command& command){
    const auto speed=FlatnessReferenceModel::reconstructForInputs(jet(q,inputTime(q,time,m.speed_delay)),m);
    if(!speed.valid)return false;
    if(!q.delay_aligned){command=speed.feedforward;return true;}
    const auto steering=FlatnessReferenceModel::reconstructForInputs(jet(q,inputTime(q,time,m.steering_delay)),m);
    if(!steering.valid)return false;
    command={speed.feedforward.speed,steering.feedforward.steering};return true;
  }
  // A normal displacement adds speed at SECOND order, invisible to the
  // straight-curve first-order speed Jacobian. Couple it with a tangent cubic
  // -beta*t^3/6. This stays in CP3..6, preserving the complete initial jet.
  static bool coupledSpeedRetreat(Curve& q,const SolveRequest& r,const SolverConfig& c,
      const BasisGrid* speed_basis=nullptr){
    double beta=0.;const double cap=std::min(r.speed_cap,c.limits.speed);
    for(int i=0;i<c.steps;++i){const double issue=Core::startTime(i,r,c)+(q.delay_aligned?0.:Core::interval(i,r,c));
      const double t=inputTime(q,issue,c.model.speed_delay);
      const auto flat=FlatnessReferenceModel::reconstructForInputs(
          speed_basis?jet(q,(*speed_basis)[i]):jet(q,t),c.model);
      if(!flat.valid)return false;
      const double sensitivity=(.5*t*t+c.model.speed_tau*t)/c.model.speed_gain;
      if(sensitivity>1e-12)beta=std::max(beta,(flat.feedforward.speed-cap)/sensitivity);
      const double state_sensitivity=.5*t*t;
      if(state_sensitivity>1e-12)beta=std::max(beta,(flat.state.speed-cap)/state_sensitivity);
    }
    if(!(beta>0.)||!std::isfinite(beta))return false;
    beta*=1.1; // nonlinear seed margin only; NOT a physical tolerance or bound
    const double co=std::cos(q.yaw),si=std::sin(q.yaw);
    for(int i=3;i<7;++i){double triples=0.;
      for(int a=1;a<=5;++a)for(int b=a+1;b<=5;++b)for(int d=b+1;d<=5;++d)
        triples+=q.knots[i+a]*q.knots[i+b]*q.knots[i+d];
      const double displacement=-beta*triples/60.;
      q.control[0][i]+=co*displacement;q.control[1][i]+=si*displacement;
    }
    return true;
  }
  static bool commandJacobian(const Curve& q,const Basis& speed,const Basis& steering,
      const Model& m,double J[2][2],int control_index=3){
    if(!firstInputJacobian(jet(q,speed),speed,m,J,control_index))return false;
    if(!q.delay_aligned)return true;
    double other[2][2]{};
    if(!firstInputJacobian(jet(q,steering),steering,m,other,control_index))return false;
    J[1][0]=other[1][0];J[1][1]=other[1][1];return true;
  }


  struct CommandDifferentialCache {
    FlatOutputJet speed_jet,steering_jet;
    FlatReconstruction speed_flat,steering_flat;
  };
  static bool prepareCommandDifferential(const Curve& q,const Basis& speed,
      const Basis& steering,const Model& m,CommandDifferentialCache& cache){
    cache.speed_jet=jet(q,speed);
    cache.speed_flat=FlatnessReferenceModel::reconstructForInputs(cache.speed_jet,m);
    if(!cache.speed_flat.valid)return false;
    if(q.delay_aligned){cache.steering_jet=jet(q,steering);
      cache.steering_flat=FlatnessReferenceModel::reconstructForInputs(cache.steering_jet,m);
      if(!cache.steering_flat.valid)return false;}
    return true;
  }
  static bool cachedCommandJacobian(const Curve& q,const Basis& speed,const Basis& steering,
      const Model& m,const CommandDifferentialCache& cache,double J[2][2],int control_index=3){
    if(!firstInputJacobianCached(cache.speed_jet,speed,m,cache.speed_flat,J,control_index))return false;
    if(!q.delay_aligned)return true;
    double other[2][2]{};
    if(!firstInputJacobianCached(cache.steering_jet,steering,m,cache.steering_flat,other,control_index))return false;
    J[1][0]=other[1][0];J[1][1]=other[1][1];return true;
  }

  // Public for verifiable parameterization and rank tests.  No command is
  // clipped here.  Failed inversions are rejected before the physical rollout.
  static bool curveFromDecision(const Curve& anchor,const Coefficients& z,
      const SolveRequest& r,const SolverConfig& c,Curve& q,bool first_input_chart=true,
      double* inverse_error=nullptr,bool delay_aligned=false,
      const Basis* first_speed_basis=nullptr,const Basis* first_steering_basis=nullptr,
      CommandDifferentialCache* final_differential=nullptr){
    q=anchor;q.delay_aligned=delay_aligned||anchor.delay_aligned;
    const double co=std::cos(anchor.yaw),si=std::sin(anchor.yaw);
    const auto scale=scales(anchor,r,c);
    for(int k=0;k<4;++k){
      const double along=z[k]*(anchor.geometry_scale[k]>0.?anchor.geometry_scale[k]:scale[0]),
        cross=z[k+4]*(anchor.geometry_scale[k+4]>0.?anchor.geometry_scale[k+4]:scale[1]);
      q.control[0][k+3]+=co*along-si*cross;q.control[1][k+3]+=si*along+co*cross;
    }
    if(!first_input_chart){if(inverse_error)*inverse_error=0.;return true;}
    double alo,ahi,dlo,dhi;
    if(!Core::demandBounds(r.previous,r.previous_command_acceleration,r.control_dt,
       std::min(r.speed_cap,c.limits.speed),c.limits,alo,ahi,dlo,dhi))return false;
    const Command target{r.previous.speed+r.control_dt*Core::decode(z[0],alo,ahi),
                         r.previous.steering+r.control_dt*Core::decode(z[4],dlo,dhi)};
    // P3 is determined by the two input coordinates; the six remaining
    // coordinates continue to determine P4..P6 independently.
    q.control[0][3]=anchor.control[0][3];q.control[1][3]=anchor.control[1][3];
    const double first_time=q.delay_aligned?0.:r.control_dt;
    const auto weights=first_speed_basis?*first_speed_basis:basis(q,inputTime(q,first_time,c.model.speed_delay));
    const auto steering_weights=first_steering_basis?*first_steering_basis:
      (q.delay_aligned?basis(q,inputTime(q,first_time,c.model.steering_delay)):weights);
    for(int iteration=0;iteration<10;++iteration){
      CommandDifferentialCache cache;
      if(!prepareCommandDifferential(q,weights,steering_weights,c.model,cache))return false;
      Command reconstructed=cache.speed_flat.feedforward;
      if(q.delay_aligned)reconstructed.steering=cache.steering_flat.feedforward.steering;
      const double ev=target.speed-reconstructed.speed,ed=target.steering-reconstructed.steering;
      const double error=std::max(std::abs(ev),std::abs(ed));
      if(inverse_error)*inverse_error=error;
      if(error<2e-13){if(final_differential)*final_differential=cache;return true;}
      double J[2][2]{};
      if(!cachedCommandJacobian(q,weights,steering_weights,c.model,cache,J))return false;
      const double det=J[0][0]*J[1][1]-J[0][1]*J[1][0];
      const double norm=std::hypot(J[0][0],J[0][1])*std::hypot(J[1][0],J[1][1]);
      if(!std::isfinite(det)||norm<=0.||std::abs(det)<1e-9*norm)return false;
      const double dx=(ev*J[1][1]-ed*J[0][1])/det,dy=(ed*J[0][0]-ev*J[1][0])/det;
      if(!std::isfinite(dx)||!std::isfinite(dy)||std::hypot(dx,dy)>2.*c.limits.speed*anchor.horizon)return false;
      q.control[0][3]+=dx;q.control[1][3]+=dy;
    }
    return false;
  }

  SolveResult solve(const SolveRequest& r,const SolverConfig& c){
    const auto started=Clock::now();SolveResult out;out.decision_dimension=8;out.command=r.previous;
    diagnostics_=Diagnostics{};diagnostics_.nonuniform=nonuniform_;diagnostics_.first_input_chart=first_input_chart_;
    diagnostics_.accuracy_refinement=accuracy_refinement_;
    diagnostics_.conditioned_geometry=conditioned_geometry_;
    diagnostics_.delay_aligned_reconstruction=delay_aligned_reconstruction_;
    diagnostics_.independent_input_encoding=independent_input_encoding_;
    diagnostics_.causal_reference_seed=causal_reference_seed_;
    diagnostics_.response_knot_monitor=response_knot_monitor_&&nonuniform_;
    auto finish=[&](const char* reason){
      out.reason=reason;
      if(out.ok&&diagnostics_.steering_rate_continuity_active)
        diagnostics_.steering_rate_continuity_cost=steeringRateContinuityCost(r,c,out.controls,steering_rate_continuity_weight_);
      out.solve_ms=Core::milliseconds(started);
      if(out.solve_ms>=c.budget_ms&&std::isfinite(c.budget_ms)){
        out.ok=out.feasible=false;out.timed_out=true;out.command=r.previous;out.reason="solver_timeout";warm_=false;
      }
      if(c.profile){const auto& p=out.profile;out.profile.other_ms=std::max(0.,out.solve_ms-p.initialization_ms-p.reference_ms-
        p.prediction_ms-p.constraint_cost_ms-p.direction_ms-p.candidate_selection_ms);}
      if(response_warm_start_&&!out.ok)warm_=false;
      return out;
    };
    if(!Core::validConfig(c))return finish("invalid_config");
    if(!Core::validState(r.state)||!Core::finiteCommand(r.previous)||!Core::finiteCommand(r.baseline)||
       !std::isfinite(r.now)||!std::isfinite(r.control_dt)||r.control_dt<.001||r.control_dt>.25||
       !std::isfinite(r.speed_cap)||r.speed_cap<0.||!std::isfinite(r.previous_command_acceleration))return finish("invalid_state_or_request");
    if(!Core::validReference(r.reference,c))return finish("reference_invalid");
    if(r.force_timeout){out.timed_out=true;return finish("injected_timeout");}
    if(r.force_infeasible)return finish("injected_infeasible");
    if(r.force_numerical_failure)return finish("injected_numerical_failure");
    if(r.state.speed<.015)return finish("flatness_low_speed_invalid");
    if(r.state.speed>std::min(r.speed_cap,c.limits.speed)+1e-7)
      return finish("flatness_initial_speed_above_dynamic_cap");
    double first_alo,first_ahi,first_dlo,first_dhi;
    if(!Core::demandBounds(r.previous,r.previous_command_acceleration,r.control_dt,
       std::min(r.speed_cap,c.limits.speed),c.limits,first_alo,first_ahi,first_dlo,first_dhi))
      return finish("flatness_first_command_speed_acceleration_jerk_bounds_empty");
    out.max_violation=Core::stateViolation(r.state,r.state.acceleration,0.,
      std::min(r.speed_cap,c.limits.speed),c);
    if(out.max_violation>1e-7)return finish("flatness_initial_physical_state_infeasible");
    if(std::abs(c.model.lateral_residual)>1e-9)return finish("flatness_lateral_velocity_unobservable_or_unsupported");
    if(r.history.size()>512)return finish("command_history_too_large");
    double last=-std::numeric_limits<double>::infinity();
    for(const auto& h:r.history){if(!std::isfinite(h.stamp)||h.stamp<last||h.stamp>r.now+1e-6||!Core::finiteCommand(h.command))return finish("invalid_command_history");last=h.stamp;}
    const double delay=std::max(c.model.steering_delay,c.model.speed_delay);
    if(delay>1e-9&&(r.history.empty()||r.history.front().stamp>r.now-delay+1e-6))return finish("history_incomplete");
    const auto rate_memory=steeringRatePrior(r,c);
    const bool rate_history_valid=rate_memory.valid;
    diagnostics_.steering_rate_history_valid=rate_history_valid;
    diagnostics_.steering_rate_prior=rate_history_valid?rate_memory.value:missing();
    const double continuity_weight=rate_history_valid?steering_rate_continuity_weight_:0.;
    diagnostics_.steering_rate_continuity_weight=continuity_weight;
    diagnostics_.steering_rate_continuity_active=continuity_weight>0.;
    auto regularize=[&](Core::Rollout& roll){if(roll.feasible&&continuity_weight>0.)
      roll.cost+=steeringRateContinuityCost(r,c,roll.commands,continuity_weight,rate_memory);};
    const auto deadline=started+Core::duration(c.budget_ms);
    if(delay_aligned_reconstruction_&&c.model.speed_delay<=1e-6)
      return finish("flatness_delay_aligned_speed_channel_singular");
    const auto search_deadline=deadline-Core::duration(std::min(.8,.12*c.budget_ms+.05));
    if(!core_.prepare(r,c,out.profile))return finish("schedule_capacity_exceeded");
    // Commands issued now cannot change the interval before either actuator's
    // delay expires. Prove infeasibility there before any spline fitting/polls.
    // This uses the SAME event partition, exact actuator decays and physical
    // bounds as Core; it never treats a failed controllable seed as a proof.
    Core::FixedPrefix fixed_prefix;
    State fixed=r.state;
    int fixed_stage=0;
    bool terminal_prefix_valid=true;
    for(int j=0;j<core_.prepared_.count;++j){
      const auto& step=core_.prepared_.substeps[j];
      if(step.speed.index>=0||step.steering.index>=0)break;
      while(j>=core_.prepared_.begin[fixed_stage+1])++fixed_stage;
      const auto& uv=step.speed.fixed;const auto& ud=step.steering.fixed;
      const double acc=(c.model.speed_gain*uv.speed-fixed.speed)/c.model.speed_tau+c.model.longitudinal_residual;
      const double dr=(c.model.steering_gain*ud.steering-fixed.steering)/c.model.steering_tau;
      out.max_violation=Core::stateViolation(fixed,acc,dr,std::min(r.speed_cap,c.limits.speed),c);
      if(out.max_violation>1e-7)return finish("flatness_uncontrollable_history_infeasible");
      auto& stage=fixed_prefix.stages[fixed_stage];
      stage.violation=std::max(stage.violation,out.max_violation);
      Core::advanceSubstep(fixed,step,c.model,uv,ud);
      out.max_violation=Core::stateViolation(fixed,fixed.acceleration,
        (c.model.steering_gain*ud.steering-fixed.steering)/c.model.steering_tau,std::min(r.speed_cap,c.limits.speed),c);
      if(!Core::validState(fixed)||out.max_violation>1e-7)return finish("flatness_uncontrollable_history_infeasible");
      stage.violation=std::max(stage.violation,out.max_violation);
      stage.state=fixed;stage.end=j+1;
      fixed_prefix.stage_count=fixed_stage+1;
      fixed_prefix.speed_command=uv.speed;fixed_prefix.steering_command=ud.steering;
      if(core_.prepared_.terminal_approach&&Core::endpointOverrun(fixed,core_.prepared_,c)>1e-7)
        terminal_prefix_valid=false;
      if(Clock::now()>=search_deadline){out.budget_limited=true;return finish("flatness_budget_without_feasible");}
    }
    // Keeping the ordinary rollout on a terminal overrun preserves ordered
    // rejection diagnostics. The prefix is request-local and physically fixed.
    const auto* prefix=terminal_prefix_valid?&fixed_prefix:nullptr;
    const double H=r.control_dt+(c.steps-1)*c.dt;
    LocalRearReference causal_reference;
    bool causal_reference_computed=false;
    auto ensureCausalReference=[&](){if(!causal_reference_computed){
      causal_reference=localRearReference(r,c);causal_reference_computed=true;}};
    // Preserve the existing chart and all its feasible seeds first. A different
    // response-aware knot is a bounded restoration only if this chart fails.
    // On a feasible response base the spline is a correction, not the entire
    // path. Bring its last free CP support to the first post-delay prediction
    // interval. This rule uses only the immutable actuator model/time grid.
    const double knot=response_base_&&response_near_support_?
      clamp((delay+c.dt)/H,.05,.95):interiorFraction(r,c,H,nonuniform_);
    anchor_=anchoredCurve(r.state,c.model,H,knot);
    anchor_.delay_aligned=delay_aligned_reconstruction_;
    std::array<Command,kMaxHorizon> response_warm_target{};bool response_warm_target_valid=false;
    if(response_base_&&anchor_.delay_aligned){
      Core::Rollout base;
      // Deterministic Core seeds, not a second 12-variable optimization.
      for(int kind=0;kind<3&&Clock::now()<search_deadline;++kind){
        bool exhausted=false;
        auto candidate=Core::rollout(core_.makeSeed(r,c,kind),r,c,core_.prepared_,search_deadline,exhausted,out.profile);
        regularize(candidate);
        if(candidate.feasible&&(!base.feasible||candidate.cost<base.cost))base=candidate;
      }
      // Preserve the original feasible seed before trying the timing-aware
      // proposal. Compare them under one model and one current reference.
      if(r.reference.synchronized_preview&&Clock::now()+Core::duration(.35)<search_deadline){
        std::array<Command,kMaxHorizon> scheduled{};bool complete=false,exhausted=false;
        const auto seed=core_.makeSeed(r,c,1,&scheduled,&complete);
        if(complete){
          auto candidate=Core::rollout(seed,r,c,core_.prepared_,search_deadline,exhausted,out.profile,&scheduled);
          regularize(candidate);
          if(!exhausted&&candidate.feasible&&(!base.feasible||candidate.cost<base.cost))base=candidate;
        }
      }
      // A positive issued acceleration can make all cruise/hold seeds run
      // into the FUTURE speed-cap/jerk viability boundary, although a causal
      // speed-matched response remains feasible. Before abandoning the nominal
      // response chart, try ONE conservative state-matched seed. It is only
      // initialization: original reference/cost, actual published history and
      // every Core physical/input constraint still apply to its full rollout,
      // then to the eight-coefficient flat reconstruction. Never replace an
      // existing feasible base or enlarge a limit to admit this seed.
      if(!base.feasible&&Clock::now()<search_deadline){
        diagnostics_.response_recovery_seed_attempted=true;
        auto recovery=r;
        recovery.baseline={std::min(r.state.speed,r.previous.speed),r.previous.steering};
        bool exhausted=false;
        auto candidate=Core::rollout(core_.makeSeed(recovery,c,0),r,c,core_.prepared_,
          search_deadline,exhausted,out.profile);regularize(candidate);
        if(!exhausted&&candidate.feasible){base=candidate;diagnostics_.response_recovery_seed_used=true;}
      }
      if(r.reference.terminal_approach&&Clock::now()+Core::duration(.40)<search_deadline){
        std::array<Command,kMaxHorizon> steer{},issued{};
        bool complete=false;
        core_.makeSeed(r,c,1,&steer,&complete);
        if(complete)for(double scale:{1.,.78}){
          if(Clock::now()+Core::duration(.18)>=search_deadline)break;
          if(!Core::terminalBrakeSeed(r,c,core_.prepared_,steer,scale,issued))continue;
          bool exhausted=false;
          auto candidate=Core::rollout(Core::Decision{},r,c,core_.prepared_,search_deadline,exhausted,out.profile,&issued);
          regularize(candidate);
          if(!exhausted&&candidate.feasible&&(!base.feasible||candidate.cost<base.cost)){
            base=candidate;diagnostics_.terminal_seed_used=true;
          }
        }
      }
      // A previous solution is a prediction seed, NEVER published history.
      // Shift its actual issued-demand ZOH grid by elapsed physical time;
      // no old local-coordinate geometry is carried into the new request.
      // Preserve all three original seed evaluations before this optional
      // extra candidate. Revalidate under the CURRENT model, measured state,
      // exact published history and unchanged physical bounds.
      const double elapsed=r.now-previous_stamp_;
      const double old_horizon=previous_first_interval_+(previous_command_count_-1)*previous_command_dt_;
      diagnostics_.response_warm_available=response_warm_start_&&warm_&&previous_command_count_==c.steps&&
        std::isfinite(elapsed)&&elapsed>=0.&&std::isfinite(previous_first_interval_)&&previous_first_interval_>0.&&
        std::isfinite(previous_command_dt_)&&previous_command_dt_>0.&&elapsed<old_horizon;
      if(diagnostics_.response_warm_available&&Clock::now()<search_deadline){
        std::array<Command,kMaxHorizon> shifted{};Command previous=r.previous;
        double acceleration=r.previous_command_acceleration;bool generated=true;
        for(int i=0;i<c.steps;++i){
          const double time=elapsed+Core::startTime(i,r,c);
          const int old=time<previous_first_interval_?0:
            1+static_cast<int>((time-previous_first_interval_+1e-10)/previous_command_dt_);
          const auto& wanted=previous_commands_[std::min(old,previous_command_count_-1)];
          const double h=Core::interval(i,r,c);double alo,ahi,dlo,dhi;
          if(!Core::demandBounds(previous,acceleration,h,std::min(r.speed_cap,c.limits.speed),
             c.limits,alo,ahi,dlo,dhi)){generated=false;break;}
          // This projection constructs only an initialization seed. It is
          // not clipping an optimized/accepted output: full nonlinear Core
          // rollout below, flat reconstruction and all subsequent validation
          // remain mandatory before any candidate can be accepted.
          acceleration=clamp((wanted.speed-previous.speed)/h,alo,ahi);
          previous.speed+=h*acceleration;
          previous.steering+=h*clamp((wanted.steering-previous.steering)/h,dlo,dhi);
          shifted[i]=previous;
        }
        if(generated&&Clock::now()<search_deadline){
          bool exhausted=false;++diagnostics_.response_warm_evaluations;
          auto candidate=Core::rollout(Core::Decision{},r,c,core_.prepared_,search_deadline,
            exhausted,out.profile,&shifted);regularize(candidate);
          if(!exhausted&&candidate.feasible){response_warm_target=shifted;response_warm_target_valid=true;}
        }
      }
      out.warm_available=diagnostics_.response_warm_available;
      if(base.feasible){
        std::array<double,P6FlatResponseBase::kCapacity> samples{};int count=1;samples[0]=0.;
        for(int i=0;i<c.steps;++i){const double t=Core::startTime(i,r,c);
          samples[count++]=t+Core::interval(i,r,c);
          samples[count++]=t+c.model.speed_delay;samples[count++]=t+c.model.steering_delay;}
        for(auto& storage:response_bases_)if(storage.use_count()==1){
          *storage=P6FlatResponseBase::build(r,c,base.commands,samples,count);
          if(storage->valid){anchor_.response_anchor=jet(anchor_,0.);anchor_.response_base=storage;
            diagnostics_.response_base_used=true;diagnostics_.response_base_cost=base.cost;}
          break;
        }
      }
    }
    prepareChart(r,c);
    Core::Rollout best;Curve best_curve;
    Coefficients selected=anchor_.response_base?encodeCurve(anchor_,anchor_,r,c,true,true):neutralDecision(r,c);bool stopped=false;
    Coefficients closest_z=selected;int closest_step=-1;double closest_violation=INFINITY;
    bool restoring=false;
    double worst_evaluation_ms=0.;
    const double initial_yaw_cos=std::cos(r.state.yaw), initial_yaw_sin=std::sin(r.state.yaw);
    const bool bounded_inverse=FlatnessReferenceModel::boundedInputModel(c.model);
    auto evaluate=[&](const Coefficients& z,Curve* curve=nullptr){
      Core::Rollout roll;const auto tick=Clock::now();
      if(tick>=search_deadline||(best.feasible&&tick+Core::duration(1.2*worst_evaluation_ms)>=search_deadline)){stopped=true;return roll;}
      ++out.evaluations;Curve candidate;double inverse_error=missing();
      const auto* first_speed=anchor_.delay_aligned?&speed_weights_[0]:&weights_[0];
      const auto* first_steering=anchor_.delay_aligned?&steering_weights_[0]:first_speed;
      CommandDifferentialCache first_inputs;
      if(!curveFromDecision(anchor_,z,r,c,candidate,first_input_chart_,&inverse_error,false,first_speed,first_steering,&first_inputs)){
        ++diagnostics_.inverse_failures;worst_evaluation_ms=std::max(worst_evaluation_ms,Core::milliseconds(tick));return roll;
      }
      diagnostics_.first_inverse_error=inverse_error;
      std::array<Command,kMaxHorizon> commands{};
      Core::IssuedInputs issued;
      Command previous=r.previous;
      double previous_acceleration=r.previous_command_acceleration;
      const double speed_cap=std::min(c.limits.speed,r.speed_cap);
      struct Sample {Command command;bool evaluated=false,valid=false;};
      std::array<Sample,3*kMaxHorizon> samples{};
      const auto store=[&](int slot,const FlatOutputJet& q,const FlatReconstruction& flat){
        auto& sample=samples[slot];
        sample.valid=flat.valid&&!(q.dx*initial_yaw_cos+q.dy*initial_yaw_sin<=.015);
        sample.command=flat.feedforward;sample.evaluated=true;
      };
      // The accepted Newton iterate already evaluated these exact input jets.
      // Apply the same forward-motion test used for the other samples.
      if(first_input_chart_){
        store(input_slot_[0][candidate.delay_aligned?1:0],first_inputs.speed_jet,first_inputs.speed_flat);
        if(candidate.delay_aligned)
          store(input_slot_[0][2],first_inputs.steering_jet,first_inputs.steering_flat);
      }
      const auto reconstruct=[&](int slot,const Basis& basis)->const Sample& {
        auto& sample=samples[slot];
        if(!sample.evaluated){
          // The chart copies P0..P2 from the anchor; Newton and search only
          // vary P3..P6. Start with the cached anchored terms and keep the
          // original sum order and exact inverse-input validity checks.
          auto terms=anchored_jet_terms_[slot];
          addJetTerms<3,7>(terms,candidate,basis);
          const auto q=finishJet(terms,basis);
          sample.valid=FlatnessReferenceModel::reconstructSelectedInputs(q,c.model,bounded_inverse,
              sample_input_mask_[slot],sample.command)&&!(q.dx*initial_yaw_cos+q.dy*initial_yaw_sin<=.015);
          sample.evaluated=true;
        }
        return sample;
      };
      for(int i=0;i<c.steps;++i){
        const auto& flat=reconstruct(input_slot_[i][0],weights_[i]);
        if(!flat.valid){
          worst_evaluation_ms=std::max(worst_evaluation_ms,Core::milliseconds(tick));return roll;
        }
        commands[i]=flat.command;
        if(candidate.delay_aligned){
          const auto& speed_flat=reconstruct(input_slot_[i][1],speed_weights_[i]);
          const auto& steering_flat=reconstruct(input_slot_[i][2],steering_weights_[i]);
          if(!speed_flat.valid||!steering_flat.valid){
            worst_evaluation_ms=std::max(worst_evaluation_ms,Core::milliseconds(tick));return roll;}
          commands[i]={speed_flat.command.speed,steering_flat.command.steering};
        }
        // Independent issued-input bounds can reject this candidate before
        // reconstructing later spline samples. The restoration search uses
        // the original state-vs-input rejection ordering to rank its seeds.
        if(!feasibility_recovery_){
          if(!Core::prepareDirectInput(commands[i],previous,previous_acceleration,
              core_.prepared_.h[i],speed_cap,c.limits,issued[i],roll)){
            roll.rejected_step=i;break;
          }
          previous=issued[i].command;
          previous_acceleration=issued[i].acceleration;
        }
      }
      // The prepared inputs are valid only in this request and candidate.
      // Check the entire delayed state trajectory, as before.
      if(roll.rejected_step<0)
        roll=Core::rollout(Core::Decision{},r,c,core_.prepared_,search_deadline,stopped,out.profile,
            feasibility_recovery_?&commands:nullptr,feasibility_recovery_?nullptr:&issued,prefix,
            best.feasible&&!feasibility_recovery_?best.cost:INFINITY);
      regularize(roll);
      if(!roll.feasible&&!roll.dominated&&!stopped){
        ++diagnostics_.rejected_rollouts;
        if(diagnostics_.first_rejected_step<0){diagnostics_.first_rejected_step=roll.rejected_step;
          diagnostics_.first_rejection=roll.rejection;}
      }
      // A rejected trajectory is ONLY a restoration search direction. It is
      // never a command, warm execution history, or feasible incumbent.
      if(feasibility_recovery_&&!roll.feasible&&std::isfinite(roll.violation)&&roll.rejected_step>=0&&
         (roll.rejected_step>closest_step||
          (roll.rejected_step==closest_step&&roll.violation<closest_violation))){
        closest_step=roll.rejected_step;closest_violation=roll.violation;closest_z=z;
        if(restoring&&!best.feasible)selected=z;
      }
      if(curve&&roll.feasible)*curve=candidate;
      worst_evaluation_ms=std::max(worst_evaluation_ms,Core::milliseconds(tick));return roll;
    };
    best=evaluate(selected,&best_curve);out.initial_objective=best.feasible?best.cost:missing();
    if((conditioned_geometry_||baseline_seed_)&&best.feasible&&best.cost<=1e-12){
      // All stage/terminal terms are nonnegative. An already feasible solution
      // at their lower bound cannot benefit from 60+ geometry polls.
      out.ok=out.feasible=out.early_stopped=true;out.objective=best.cost;out.max_violation=best.violation;
      out.command=best.commands[0];out.command_acceleration=best.command_accelerations[0];
      out.controls=best.commands;out.predicted=best.states;out.count=c.steps+1;
      selected_=selected;previous_curve_=best_curve;rememberCommands(best,r,c);previous_stamp_=r.now;warm_=true;
      diagnostics_.reference_fit_samples=0; // no reference fit was needed
      return finish("flat8_feasible_cost_lower_bound");
    }
    auto attempt=[&](const Coefficients& z,bool warm){
      if(stopped)return;
      Curve candidate;const auto roll=evaluate(z,&candidate);
      if(roll.feasible&&!stopped&&(!best.feasible||roll.cost<best.cost)){best=roll;best_curve=candidate;selected=z;out.warm_started=warm;}
    };
    // Preserve the first physical command independently from six geometric
    // coordinates. A distant control point must not scale the command back
    // toward neutral, which invalidates a causally shifted warm start.
    auto encode=[&](const Curve& curve,bool preserve=false){
      return encodeCurve(anchor_,curve,r,c,preserve,independent_input_encoding_);
    };
    const double elapsed=r.now-previous_stamp_;
    // The first interval is the measured callback interval, not a fixed grid.
    // Ordinary ROS jitter changes H. warmSeed already resamples the physical
    // elapsed time and refits to the NEW anchor/horizon before full validation;
    // requiring two horizons to agree to 1 ns disabled real warm starts.
    const bool geometry_warm_available=warm_&&!anchor_.response_base&&!previous_curve_.response_base&&std::isfinite(elapsed)&&elapsed>=0.&&
      std::isfinite(previous_curve_.horizon)&&elapsed<previous_curve_.horizon;
    out.warm_available=geometry_warm_available||diagnostics_.response_warm_available;
    out.warm_started=best.feasible&&diagnostics_.response_warm_accepted;
    auto warmSeed=[&](){
      if(!geometry_warm_available||stopped)return;
      const auto origin=jet(previous_curve_,elapsed);
      const double rotation=r.state.yaw-std::atan2(origin.dy,origin.dx),co=std::cos(rotation),si=std::sin(rotation);
      const double overlap=std::min(H,previous_curve_.horizon-elapsed);Curve shifted=anchor_;
      std::array<std::array<double,2>,kMaxHorizon+1> warm_targets{};
      std::array<double,kMaxHorizon+1> warm_times{};
      for(int i=0;i<c.steps;++i){warm_times[i]=overlap*(i+1)/c.steps;const auto old=jet(previous_curve_,elapsed+warm_times[i]);
        const double dx=old.x-origin.x,dy=old.y-origin.y;
        warm_targets[i]={{r.state.x+co*dx-si*dy,r.state.y+si*dx+co*dy}};}
      if(fit(shifted,warm_targets,warm_times,c.steps,&diagnostics_.warm_fit_rmse)){
        const bool preserve=command_projection_||(accuracy_refinement_&&feasibility_recovery_);
        diagnostics_.warm_geometry_preserved=preserve;
        attempt(encode(shifted,preserve),true);
      }
    };
    // Preserve the ORIGINAL response base and its eight-dimensional affine
    // correction space. Replacing the base with a cheap warm rollout changes
    // that space and can exclude the better cold-chart optimum. Instead fit
    // the old physical input sequence into THIS chart as one extra seed.
    // Projection is only an initialization: encode/reconstruct and the full
    // nonlinear rollout must strictly improve the retained cold incumbent.
    if(anchor_.response_base&&response_warm_target_valid&&best.feasible&&!stopped&&
       Clock::now()+Core::duration(1.2*worst_evaluation_ms)<search_deadline){
      diagnostics_.response_warm_cost_before=best.cost;Curve projected=anchor_;
      if(projectCommands(projected,response_warm_target,r,c,true)){
        for(double alpha:{1.,.5,.25}){
          if(stopped)break;Curve trial=anchor_;
          for(int axis=0;axis<2;++axis)for(int k=3;k<7;++k)
            trial.control[axis][k]+=alpha*(projected.control[axis][k]-anchor_.control[axis][k]);
          const int failed=diagnostics_.inverse_failures;const double before=best.cost;
          attempt(encodeCurve(anchor_,trial,r,c,true,true),true);
          if(diagnostics_.inverse_failures>failed)diagnostics_.response_warm_reconstruction_failed=true;
          if(best.cost<before){diagnostics_.response_warm_accepted=true;break;}
        }
      }else diagnostics_.response_warm_reconstruction_failed=true;
      diagnostics_.response_warm_cost_after=best.cost;
    }
    // Evaluate the real shifted incumbent before spending a finite budget on
    // new reference/baseline fits. Legacy ordering remains unchanged by default.
    // Reusing a feasible incumbent is independent of how its first input is
    // encoded. In the projected strategy it must precede expensive new fits.
    if(independent_input_encoding_||command_projection_)warmSeed();
    // Feasibility restoration remains IN the spline space. An anchored quadratic
    // on a bend increases |position derivative| and may exceed a saturated speed
    // cap. Add a smooth longitudinal cubic (zero initial position/velocity/a)
    // rather than clipping the reconstructed speed or loosening the cap.
    if(!best.feasible){
      const double jerk_scale=.5*c.limits.jerk*r.control_dt/(c.model.speed_tau+r.control_dt);
      for(double fraction:{.05,.1,.25,.5,1.}){
        Curve restore=anchor_;const double co=std::cos(anchor_.yaw),si=std::sin(anchor_.yaw);
        for(int i=3;i<7;++i){double triples=0.;for(int a=1;a<=5;++a)for(int b=a+1;b<=5;++b)for(int d=b+1;d<=5;++d)triples+=anchor_.knots[i+a]*anchor_.knots[i+b]*anchor_.knots[i+d];
          const double displacement=-fraction*jerk_scale*triples/60.;restore.control[0][i]+=co*displacement;restore.control[1][i]+=si*displacement;}
        attempt(encode(restore),false);
      }
    }
    // Fit ONLY the research spline, never alter the original reference message.
    Curve fitted=anchor_;std::array<std::array<double,2>,kMaxHorizon+1> targets{};std::array<double,kMaxHorizon+1> times{};
    const auto initial_reference=Core::referenceAt(r.reference,0.);
    const double initial_lk=c.model.rear_to_center*initial_reference.curvature;
    bool steady_seed_valid=!anchor_.response_base&&std::abs(initial_lk)<1.;
    if(!anchor_.response_base&&!steady_seed_valid&&!causal_reference_seed_)return finish("flatness_reference_center_singularity");
    const double initial_kr=steady_seed_valid?initial_reference.curvature/std::sqrt(1.-initial_lk*initial_lk):0.;
    double reference_body=steady_seed_valid?initial_reference.yaw-std::atan(c.model.rear_to_center*initial_kr):r.state.yaw,last_time=0.;
    for(int i=0;i<c.steps;++i){
      times[i]=Core::startTime(i,r,c)+Core::interval(i,r,c);
      if(anchor_.response_base)continue; // No reference-position fit on this branch.
      const auto p=Core::referenceAt(r.reference,times[i]);
      const double lk=c.model.rear_to_center*p.curvature;
      if(!anchor_.response_base&&std::abs(lk)>=1.){
        if(!causal_reference_seed_)return finish("flatness_reference_center_singularity");
        steady_seed_valid=false; // invalid SEED, not proof every control is infeasible
      }
      // Center position is NOT asserted flat. Transport the body heading with
      // its kinematic ODE; instantaneous constant-curvature conversion is wrong
      // when reference curvature changes.
      if(c.model.rear_to_center>1e-6){
        const double h=(times[i]-last_time)/4.;for(int j=0;j<4;++j){const auto center=Core::referenceAt(r.reference,last_time+(j+.5)*h);
          const double rate=center.speed*std::sin(center.yaw-reference_body)/c.model.rear_to_center;
          reference_body=wrap(reference_body+h*center.speed*std::sin(center.yaw-reference_body-.5*h*rate)/c.model.rear_to_center);}
      }else reference_body=p.yaw;
      last_time=times[i];targets[i]={{p.x-c.model.rear_to_center*std::cos(reference_body),p.y-c.model.rear_to_center*std::sin(reference_body)}};
    }
    diagnostics_.reference_fit_samples=anchor_.response_base?0:c.steps;
    auto fitReferenceSeed=[&](){
     auto tryTargets=[&](const std::array<std::array<double,2>,kMaxHorizon+1>& samples){
     if(stopped)return;
     fitted=anchor_;
     if(fit(fitted,samples,times,c.steps,&diagnostics_.reference_fit_rmse)){
      const auto seed=encode(fitted);const auto neutral=selected;
      // A shape change needs correlated coefficients. Poll its entire ray; a
      // single independently clipped CP or single coarse coordinate step can
      // increase heading/rate cost despite a useful combined path correction.
      for(double fraction:{1.,.5,.25,.125,.0625,.03125,.015625}){
        auto trial=neutral;for(int j=0;j<8;++j)trial[j]+=fraction*(seed[j]-neutral[j]);attempt(trial,false);
      }
     }
     };
     if(steady_seed_valid)tryTargets(targets);
     // Spend recovery work ONLY on the measured bottleneck: an infeasible
     // seed. Do not consume the refinement budget of an already feasible
     // reference/warm incumbent. No second ODE/fit is needed on that path.
     if(causal_reference_seed_&&!best.feasible){ensureCausalReference();
       if(causal_reference.valid)tryTargets(causal_reference.positions);}
    };
    if(!anchor_.response_base)fitReferenceSeed();
    std::array<Command,kMaxHorizon> baseline_commands{};bool baseline_available=false;
    Curve baseline_projection;bool baseline_projection_available=false;
    if(baseline_seed_&&!anchor_.response_base&&!stopped){
      // S8 is an INITIAL GUESS only. Roll its rate/jerk-feasible seed through
      // the same delayed model, fit the eight spline coordinates, and accept
      // only reconstructed controls with a fully feasible, cheaper rollout.
      // Candidate controls are never inserted into real published history.
      const auto seed=core_.makeSeed(r,c,0);
      auto baseline=Core::rollout(seed,r,c,core_.prepared_,search_deadline,stopped,out.profile);regularize(baseline);
      if(baseline.feasible&&!stopped){
        baseline_commands=baseline.commands;baseline_available=true;
        if(command_projection_){
          // Fit the *issued controls*, not only sampled positions. Position
          // least squares has no control over third derivatives and can turn
          // an excellent delayed S8 seed into excessive jerk after inversion.
          // This is an eight-CP local least-squares seed, not a second control
          // optimizer or an unvalidated command output.
          Curve projected=best.feasible?best_curve:anchor_;
          for(int iteration=0;iteration<3&&!stopped;++iteration){
            if(Clock::now()+Core::duration(.25)>=search_deadline)break;
            // Preserve the already-tested amplitude seed before trying the new
            // derivative-aware restoration. A useful new fit must not erase an
            // older feasible incumbent or preempt its ordinary pattern search.
            if(!projectCommands(projected,baseline.commands,r,c,false))break;
            if(iteration==0){baseline_projection=projected;baseline_projection_available=true;}
            const auto z=encodeCurve(anchor_,projected,r,c,true,true);
            Curve trial;const auto roll=evaluate(z,&trial);
            if(roll.feasible&&!stopped&&(!best.feasible||roll.cost<best.cost)){
              best=roll;best_curve=trial;selected=z;diagnostics_.baseline_seed_accepted=true;
            }
          }
        }
        Curve fitted_baseline=anchor_;
        std::array<std::array<double,2>,kMaxHorizon+1> baseline_targets{};
        for(int i=0;i<c.steps;++i)baseline_targets[i]={{baseline.states[i+1].x,baseline.states[i+1].y}};
        if(fit(fitted_baseline,baseline_targets,times,c.steps,&diagnostics_.baseline_fit_rmse)){
          const auto seed_z=encode(fitted_baseline);const auto origin=selected;
          const double before=best.cost;
          for(double fraction:{1.,.5,.25,.125}){auto z=origin;
            for(int j=0;j<8;++j)z[j]+=fraction*(seed_z[j]-origin[j]);
            attempt(z,false);}
          diagnostics_.baseline_seed_accepted=best.feasible&&best.cost<before;
        }
      }
    }
    // Phase I in the SAME eight implicit-first-input spline coordinates.
    // A feasible first 33ms command says nothing about the following 100ms
    // acceleration/jerk interval. Restore ALL issued-input inequalities before
    // spending polls on a tracking objective that has no feasible incumbent.
    // No issued command is clipped and no rejected curve becomes warm history.
    if(input_feasibility_sqp_&&first_input_chart_&&!best.feasible&&!stopped){
      Coefficients phase_z=baseline_projection_available?
        encodeCurve(anchor_,baseline_projection,r,c,true,true):selected;
      Curve phase;
      if(curveFromDecision(anchor_,phase_z,r,c,phase,first_input_chart_)){
        double merit=missing(),maximum=missing();Coefficients direction{};
        if(inputFeasibilityDirection(phase,r,c,merit,maximum,&direction,search_deadline)){
          diagnostics_.input_merit_before=diagnostics_.input_merit_after=merit;
          for(int restoration=0;restoration<3&&!best.feasible&&!stopped;++restoration){
            if(Clock::now()+Core::duration(.3)>=search_deadline)break;
            if(restoration&&!inputFeasibilityDirection(phase,r,c,merit,maximum,&direction,search_deadline))break;
            if(merit<=1e-20){attempt(phase_z,false);break;}
            bool advanced=false;
            for(double alpha:{1.,.5,.25,.125}){
              if(Clock::now()>=search_deadline)break;
              auto z=phase_z;
              for(int n=0;n<8;++n){const double bound=(n==0||n==4)?1.:8.;
                z[n]=clamp(z[n]+alpha*direction[n],-bound,bound);}
              Curve trial;if(!curveFromDecision(anchor_,z,r,c,trial,first_input_chart_))continue;
              double value=missing(),max_value=missing();
              if(!inputFeasibilityDirection(trial,r,c,value,max_value,nullptr,search_deadline))continue;
              if(value+1e-14>=merit||max_value>maximum+1e-10)continue;
              phase=trial;phase_z=z;merit=value;maximum=max_value;advanced=true;
              ++diagnostics_.input_sqp_steps;diagnostics_.input_merit_after=merit;
              attempt(phase_z,false);break;
            }
            if(!advanced)break;
          }
        }
      }
    }
    if(near_knot_restoration_&&!best.feasible&&!stopped&&best.rejected_step>=0&&best.rejected_step<=2){
      // One bounded chart restoration driven by the first failed interval,
      // not by a route/terrain label. Keep the SAME eight free coefficients,
      // initial jet, physical inputs, cost, horizon and validation checks.
      // The old weighted-median knot near 1s can prevent a 33ms command
      // history transition from satisfying the next 100ms jerk interval.
      const double near_fraction=clamp(2.*r.control_dt/H,.05,.25);
      if(std::abs(anchor_.knots[6]/H-near_fraction)>1e-9){
        diagnostics_.near_knot_restoration=true;
        anchor_=anchoredCurve(r.state,c.model,H,near_fraction);anchor_.delay_aligned=delay_aligned_reconstruction_;
        // Coefficients in different knot charts are not interchangeable.
        closest_step=-1;closest_violation=INFINITY;
        prepareChart(r,c);
        selected=neutralDecision(r,c);best=evaluate(selected,&best_curve);fitReferenceSeed();
      }
    }
    if(!independent_input_encoding_&&!command_projection_)warmSeed();
    if(command_projection_&&baseline_projection_available&&baseline_projection.knots==anchor_.knots&&!stopped){
      // At a saturated speed, an independent lateral poll violates |y'|<=cap
      // even though an arbitrarily small coupled (tangent,normal) turn exists.
      // Test the coupled direction as ONE spline seed, never as clipped inputs.
      for(double alpha:{1.,.25,.0625,.015625}){
        if(stopped)break;
        // Use the physically meaningful S8 command projection, not a position
        // LS ray. The latter can initially steer the wrong way while fitting
        // distant positions under its fixed position/velocity/acceleration jet.
        Curve coupled=anchor_;
        for(int axis=0;axis<2;++axis)for(int k=3;k<7;++k)
          coupled.control[axis][k]+=alpha*(baseline_projection.control[axis][k]-anchor_.control[axis][k]);
        if(coupledSpeedRetreat(coupled,r,c,coupled.delay_aligned?&speed_weights_:&weights_))
          attempt(encodeCurve(anchor_,coupled,r,c,true,true),false);
      }
    }
    // Optimize the ACTUAL common tracking objective around a feasible spline,
    // rather than only matching the S8 seed's issued input amplitudes. The
    // analytic direction never becomes a command without chart reconstruction
    // and the complete original delayed-model constraint/objective evaluation.
    if(command_projection_&&best.feasible&&!stopped){
      for(int refinement=0;refinement<2&&!stopped;++refinement){
        if(Clock::now()+Core::duration(.5)>=search_deadline)break;
        Coefficients direction{};double linearized=missing();
        // The chart anchor owns the changing knot / response-base layout. It
        // already evaluates the same issue-time basis for every physical step.
        // Reuse it only when all non-coefficient inputs are exactly identical.
        const bool same_basis=sameBasisLayout(best_curve,anchor_);
        const auto* speed_basis=same_basis?(best_curve.delay_aligned?&speed_weights_:&weights_):nullptr;
        const auto* steering_basis=same_basis&&best_curve.delay_aligned?&steering_weights_:nullptr;
        if(!trackingNormal(best_curve,r,c,core_.prepared_,direction,linearized,nullptr,search_deadline,
           continuity_weight,speed_basis,steering_basis,prefix))break;
        ++diagnostics_.tracking_directions;diagnostics_.tracking_linearized_objective=linearized;
        double norm=0.;for(double value:direction)norm+=value*value;
        diagnostics_.tracking_step_norm=std::sqrt(norm);
        const Curve origin=best_curve;const double before=best.cost;
        for(double alpha:{1.,.5,.25,.125,.0625}){
          if(stopped)break;Curve proposed=origin;
          applyTrackingDirection(proposed,direction,alpha,r,c);
          // First-order speed sensitivities miss the positive second-order
          // norm increment at a saturated straight path. This is only a seed
          // repair; acceptance still uses the complete nonlinear Core cost.
          coupledSpeedRetreat(proposed,r,c,speed_basis);
          attempt(encodeCurve(anchor_,proposed,r,c,true,true),false);
          if(best.cost<before){++diagnostics_.tracking_accepted;break;}
        }
        if(!(best.cost<before))break;
      }
    }
    // Projected seeds already use this wider SHAPE chart. Poll and warm-start
    // in the same chart: clipping selected[k]~3 to 1 during a +/-0.005 poll
    // destroys locality and strands an otherwise feasible seed. The first
    // acceleration/steering-rate coordinates and all physical limits stay fixed.
    auto coordinateBound=[&](int j){return (command_projection_||(accuracy_refinement_&&feasibility_recovery_))&&j!=0&&j!=4?8.:1.;};
    // Fine poll before coarse search: inverse flat dynamics can make the cost
    // very stiff. Three coarse radii alone incorrectly declare a zero-response
    // curve optimal even when a smaller feasible improvement exists.
    diagnostics_.fine_poll_cost_before=best.feasible?best.cost:missing();
    for(int coordinate=0;coordinate<8&&!stopped;++coordinate){const auto base=selected;
      for(int sign:{-1,1}){auto z=base;const double bound=coordinateBound(coordinate);z[coordinate]=clamp(z[coordinate]+sign*.005,-bound,bound);
        const int evaluations=out.evaluations;const double before=best.cost;
        attempt(z,false);diagnostics_.fine_poll_evaluations+=out.evaluations-evaluations;
        if(best.feasible&&best.cost<before)++diagnostics_.fine_poll_improvements;
      }
    }
    diagnostics_.fine_poll_cost_after=best.feasible?best.cost:missing();
    // A complete nonimproving positive-spanning local poll after a successful
    // GN step is an opt-in finite-computation stopping rule, not an optimality
    // certificate. Keep the fully feasible incumbent; failed/incomplete polls
    // and all infeasibility/restoration paths retain the original coarse search.
    diagnostics_.coarse_poll_skipped=budgeted_poll_&&best.feasible&&best_curve.response_base&&
      diagnostics_.tracking_accepted>0&&!stopped&&diagnostics_.fine_poll_evaluations==16&&
      diagnostics_.fine_poll_improvements==0&&
      diagnostics_.fine_poll_cost_before==diagnostics_.fine_poll_cost_after;
    if(diagnostics_.coarse_poll_skipped){out.early_stopped=true;diagnostics_.local_poll_stop_stage=1;}
    int nonimproving=0;
    for(int iteration=0;iteration<c.iterations&&!stopped&&!diagnostics_.coarse_poll_skipped;++iteration){
      const double old_cost=best.cost,radius=.35*std::pow(.5,iteration);
      for(int coordinate=0;coordinate<8&&!stopped;++coordinate){const auto base=selected;
        for(int sign:{-1,1}){auto z=base;const double bound=coordinateBound(coordinate);z[coordinate]=clamp(z[coordinate]+sign*radius,-bound,bound);
          if(std::abs(z[coordinate]-base[coordinate])<1e-14)continue;
          Curve candidate;const auto roll=evaluate(z,&candidate);
          if(roll.feasible&&!stopped&&(!best.feasible||roll.cost<best.cost)){best=roll;selected=z;best_curve=candidate;}
          if(stopped)break;
        }
      }out.iterations=iteration+1;
      if(!stopped)++diagnostics_.coarse_poll_sweeps;
      // If the complete coarse sweep also found no strict feasible decrease,
      // keep the incumbent instead of automatically spending another sweep at
      // a smaller radius. This remains opt-in and requires a successful GN
      // update on an actual response base; it is not a convergence theorem.
      if(budgeted_poll_&&best.feasible&&best_curve.response_base&&diagnostics_.tracking_accepted>0&&
         !stopped&&best.cost==old_cost&&iteration+1<c.iterations){
        diagnostics_.coarse_poll_skipped=true;diagnostics_.local_poll_stop_stage=2;
        out.early_stopped=true;break;
      }
      if(best.feasible&&std::isfinite(old_cost)&&old_cost-best.cost<=1e-7*std::max(1.,old_cost))++nonimproving;else nonimproving=0;
      if(nonimproving>=2&&iteration>=2){out.early_stopped=true;break;}
    }
    // A derivative-aware eight-CP refinement/restoration uses only the residual
    // budget after all original seeds and polls. Keep every feasible incumbent.
    if(command_projection_&&baseline_available&&!stopped){
      Curve projected=best.feasible?best_curve:anchor_;
      for(int iteration=0;iteration<2&&!stopped;++iteration){
        if(Clock::now()+Core::duration(.3)>=search_deadline)break;
        const Curve origin=projected;Curve proposed=origin;
        if(!projectCommands(proposed,baseline_commands,r,c,true))break;
        bool advanced=false;
        for(double alpha:{1.,.5,.25}){
          Curve damped=origin;
          for(int axis=0;axis<2;++axis)for(int k=3;k<7;++k)
            damped.control[axis][k]+=alpha*(proposed.control[axis][k]-origin.control[axis][k]);
          const auto z=encodeCurve(anchor_,damped,r,c,true,true);Curve decoded;
          if(!curveFromDecision(anchor_,z,r,c,decoded,first_input_chart_))continue;
          if(!advanced){projected=decoded;advanced=true;}
          Curve trial;const auto roll=evaluate(z,&trial);
          if(roll.feasible&&!stopped&&(!best.feasible||roll.cost<best.cost)){
            best=roll;best_curve=trial;selected=z;projected=trial;
            diagnostics_.baseline_seed_accepted=true;break;
          }
          if(stopped)break;
        }
        if(!advanced)break;
      }
    }
    // A knot change is not a universal tracking improvement. Try the response
    // chart only after the original chart's seeds AND optimization have failed,
    // and only in the remaining wall budget. No feasible incumbent is discarded.
    if(response_knot_monitor_&&nonuniform_&&!best.feasible&&!stopped&&
       Clock::now()+Core::duration(.3)<search_deadline){
      ensureCausalReference();
      if(causal_reference.valid){
      const double alternate=responseKnotFraction(r,c,H,causal_reference);
      if(std::abs(anchor_.knots[6]/H-alternate)>1e-9){
        anchor_=anchoredCurve(r.state,c.model,H,alternate);anchor_.delay_aligned=delay_aligned_reconstruction_;
        prepareChart(r,c);
        selected=neutralDecision(r,c);closest_step=-1;closest_violation=INFINITY;
        best=evaluate(selected,&best_curve);fitReferenceSeed();
      }
      }
    }
    // Keep the entire original search and all its successful solutions. Only
    // an infeasible result can enter this extra bounded restoration phase,
    // within the SAME original deadline. Enlarging geometric coordinates is
    // not enlarging physical limits: Core checks every reconstructed input
    // and delayed state before attempt() can accept it.
    if(feasibility_recovery_&&!best.feasible&&!stopped&&
       Clock::now()+Core::duration(.2)<search_deadline){
      diagnostics_.feasibility_recovery_attempted=true;restoring=true;
      if(closest_step>=0)selected=closest_z;
      for(int restoration=0;restoration<3&&!stopped&&!best.feasible;++restoration){
        const double radius=4.*std::pow(.5,restoration);
        for(int coordinate=0;coordinate<8&&!stopped&&!best.feasible;++coordinate){const auto base=selected;
          for(int sign:{-1,1}){auto z=base;const double bound=(coordinate==0||coordinate==4)?1.:8.;
            z[coordinate]=clamp(z[coordinate]+sign*radius,-bound,bound);
            attempt(z,false);if(stopped||best.feasible)break;
          }
        }
      }
      diagnostics_.feasibility_recovered=best.feasible;
    }
    // Recovery previously returned the first feasible spline without any
    // tracking-cost refinement. Retain that feasible incumbent and accept
    // only strict objective improvements in the SAME eight-dimensional
    // spline space, within the original deadline and physical constraints.
    if(accuracy_refinement_&&diagnostics_.feasibility_recovered&&!stopped){
      for(double radius:{.125,.03125}){
        for(int coordinate=0;coordinate<8&&!stopped;++coordinate){
          for(int sign:{-1,1}){
            auto z=selected;const double bound=coordinateBound(coordinate);
            z[coordinate]=clamp(z[coordinate]+sign*radius,-bound,bound);
            if(std::abs(z[coordinate]-selected[coordinate])<1e-14)continue;
            const double old=best.cost;attempt(z,false);
            if(best.cost<old)++diagnostics_.refinement_improvements;
            if(stopped)break;
          }
        }
        if(stopped)break;
      }
    }
    out.budget_limited=stopped;
    if(Clock::now()>=deadline)return finish("solver_timeout");
    if(!best.feasible)return finish(stopped?"flatness_budget_without_feasible":"flatness_no_feasible_rollout");
    out.ok=out.feasible=true;out.objective=best.cost;out.max_violation=best.violation;
    out.command=best.commands[0];out.command_acceleration=best.command_accelerations[0];
    out.controls=best.commands;out.predicted=best.states;out.count=c.steps+1;
    selected_=selected;previous_curve_=best_curve;rememberCommands(best,r,c);previous_stamp_=r.now;warm_=true;
    return finish(diagnostics_.feasibility_recovered?"flat8_feasible_bounded_restoration":
      (diagnostics_.near_knot_restoration?"flat8_feasible_near_knot_restoration":
      (stopped?"flat8_feasible_budget_stop":
       (diagnostics_.coarse_poll_skipped?"flat8_feasible_local_poll_stop":"flat8_feasible_pattern_search"))));
  }

 private:
  Core core_;Curve anchor_,previous_curve_;Coefficients selected_{};
  BasisGrid weights_{},speed_weights_{},steering_weights_{};
  std::array<JetTerms,3*kMaxHorizon> anchored_jet_terms_;
  std::array<std::array<int,3>,kMaxHorizon> input_slot_{};
  std::array<unsigned,3*kMaxHorizon> sample_input_mask_{};
  Diagnostics diagnostics_;bool warm_=false,nonuniform_=true,first_input_chart_=true,near_knot_restoration_=false,
    feasibility_recovery_=false,accuracy_refinement_=false,conditioned_geometry_=false,baseline_seed_=false,
    delay_aligned_reconstruction_=false,independent_input_encoding_=false,command_projection_=false,
    causal_reference_seed_=false,response_knot_monitor_=false,input_feasibility_sqp_=false,response_base_=false,response_near_support_=false,
    budgeted_poll_=false,response_warm_start_=false;double previous_stamp_=0.,steering_rate_continuity_weight_=0.;
  std::array<Command,kMaxHorizon> previous_commands_{};
  int previous_command_count_=0;double previous_first_interval_=0.,previous_command_dt_=0.;
  void rememberCommands(const Core::Rollout& roll,const SolveRequest&r,const SolverConfig&c){
    previous_commands_=roll.commands;previous_command_count_=c.steps;
    previous_first_interval_=r.control_dt;previous_command_dt_=c.dt;
  }
  // Allocated once at solver construction. Do not mutate a base still retained
  // by a previous curve/test consumer; two in-flight references freeze updates.
  std::array<std::shared_ptr<P6FlatResponseBase>,2> response_bases_{{
    std::make_shared<P6FlatResponseBase>(),std::make_shared<P6FlatResponseBase>()}};
 public:
  // Full-horizon normalized hinge merit and its GN direction. The Jacobian
  // differentiates the ACTUAL implicit P3 elimination (not unconstrained raw
  // CPs subsequently changed by encode). Limits and measured h_i are Core's.
  // Public only for independent finite-difference/constraint verification.
  static bool inputFeasibilityDirection(const Curve& q,const SolveRequest& r,const SolverConfig& c,
      double& merit,double& maximum,Coefficients* direction=nullptr,
      Clock::time_point deadline=Clock::now()+std::chrono::seconds(1),
      Coefficients* merit_gradient=nullptr){
    using V=Coefficients;
    merit=maximum=0.;double normal[8][8]{},rhs[8]{};
    const auto scale=trackingScales(q,r,c);
    const double co=std::cos(q.yaw),si=std::sin(q.yaw);
    double P3[8][2]{};
    if(direction){
      double alo,ahi,dlo,dhi;
      if(!Core::demandBounds(r.previous,r.previous_command_acceleration,r.control_dt,
          std::min(r.speed_cap,c.limits.speed),c.limits,alo,ahi,dlo,dhi))return false;
      const double issue=q.delay_aligned?0.:r.control_dt;
      const auto B=basis(q,inputTime(q,issue,c.model.speed_delay));
      const auto D=q.delay_aligned?basis(q,inputTime(q,issue,c.model.steering_delay)):B;
      CommandDifferentialCache cache;if(!prepareCommandDifferential(q,B,D,c.model,cache))return false;
      double A[2][2]{};if(!cachedCommandJacobian(q,B,D,c.model,cache,A))return false;
      const double det=A[0][0]*A[1][1]-A[0][1]*A[1][0];
      if(!std::isfinite(det)||std::abs(det)<1e-14)return false;
      auto inverse=[&](int n,double v,double d){
        P3[n][0]=(v*A[1][1]-d*A[0][1])/det;
        P3[n][1]=(d*A[0][0]-v*A[1][0])/det;
      };
      inverse(0,.5*r.control_dt*(ahi-alo),0.);
      inverse(4,0.,.5*r.control_dt*(dhi-dlo));
      for(int k=1;k<4;++k){double J[2][2]{};
        if(!cachedCommandJacobian(q,B,D,c.model,cache,J,k+3))return false;
        inverse(k,-(J[0][0]*co+J[0][1]*si)*scale[k],-(J[1][0]*co+J[1][1]*si)*scale[k]);
        inverse(k+4,-(-J[0][0]*si+J[0][1]*co)*scale[k+4],-(-J[1][0]*si+J[1][1]*co)*scale[k+4]);
      }
    }
    auto row=[&](double value,const V& derivative,double lo,double hi,double unit){
      const double residual=(value-clamp(value,lo,hi))/unit;
      merit+=.5*residual*residual;maximum=std::max(maximum,std::abs(residual));
      if(!direction||residual==0.)return;
      for(int a=0;a<8;++a){const double Ja=derivative[a]/unit;
        rhs[a]-=Ja*residual;
        for(int b=0;b<8;++b)normal[a][b]+=Ja*derivative[b]/unit;}
    };
    Command previous=r.previous;double previous_a=r.previous_command_acceleration;
    V previous_v{},previous_d{},previous_acceleration{};
    for(int i=0;i<c.steps;++i){
      if(Clock::now()>=deadline)return false;
      const double issue=Core::startTime(i,r,c)+(q.delay_aligned?0.:Core::interval(i,r,c));
      const auto B=basis(q,inputTime(q,issue,c.model.speed_delay));
      const auto D=q.delay_aligned?basis(q,inputTime(q,issue,c.model.steering_delay)):B;
      CommandDifferentialCache cache;if(!prepareCommandDifferential(q,B,D,c.model,cache))return false;
      Command command=cache.speed_flat.feedforward;
      if(q.delay_aligned)command.steering=cache.steering_flat.feedforward.steering;
      V Vspeed{},Vsteering{},A{},R{},K{};
      if(direction){
        double J3[2][2]{};if(!cachedCommandJacobian(q,B,D,c.model,cache,J3))return false;
        for(int n=0;n<8;++n){Vspeed[n]=J3[0][0]*P3[n][0]+J3[0][1]*P3[n][1];
          Vsteering[n]=J3[1][0]*P3[n][0]+J3[1][1]*P3[n][1];}
        for(int k=1;k<4;++k){double J[2][2]{};
          if(!cachedCommandJacobian(q,B,D,c.model,cache,J,k+3))return false;
          Vspeed[k]+=(J[0][0]*co+J[0][1]*si)*scale[k];
          Vspeed[k+4]+=(-J[0][0]*si+J[0][1]*co)*scale[k+4];
          Vsteering[k]+=(J[1][0]*co+J[1][1]*si)*scale[k];
          Vsteering[k+4]+=(-J[1][0]*si+J[1][1]*co)*scale[k+4];}
      }
      const double h=Core::interval(i,r,c),a=(command.speed-previous.speed)/h;
      for(int n=0;n<8;++n){A[n]=(Vspeed[n]-previous_v[n])/h;
        R[n]=(Vsteering[n]-previous_d[n])/h;K[n]=(A[n]-previous_acceleration[n])/h;}
      const double cap=std::min(r.speed_cap,c.limits.speed);
      row(command.speed,Vspeed,0.,cap,std::max(.015,cap));
      row(command.steering,Vsteering,-c.limits.steering,c.limits.steering,c.limits.steering);
      row(a,A,-c.limits.deceleration,c.limits.acceleration,std::min(c.limits.deceleration,c.limits.acceleration));
      row((command.steering-previous.steering)/h,R,-c.limits.steering_rate,c.limits.steering_rate,c.limits.steering_rate);
      row((a-previous_a)/h,K,-c.limits.jerk,c.limits.jerk,c.limits.jerk);
      previous=command;previous_a=a;previous_v=Vspeed;previous_d=Vsteering;previous_acceleration=A;
    }
    if(!std::isfinite(merit)||!std::isfinite(maximum))return false;
    if(!direction)return true;
    if(merit_gradient)for(int n=0;n<8;++n)(*merit_gradient)[n]=-rhs[n];
    double largest=0.;for(int n=0;n<8;++n)largest=std::max(largest,normal[n][n]);
    if(largest==0.){*direction={};return true;}
    V columns{};
    for(int n=0;n<8;++n)columns[n]=std::sqrt(std::max(normal[n][n],1e-12*largest));
    for(int a=0;a<8;++a){rhs[a]/=columns[a];
      for(int b=0;b<8;++b)normal[a][b]/=columns[a]*columns[b];
      normal[a][a]+=1e-6;}
    for(int k=0;k<8;++k){int pivot=k;
      for(int i=k+1;i<8;++i)if(std::abs(normal[i][k])>std::abs(normal[pivot][k]))pivot=i;
      if(std::abs(normal[pivot][k])<1e-14)return false;
      for(int j=0;j<8;++j)std::swap(normal[k][j],normal[pivot][j]);std::swap(rhs[k],rhs[pivot]);
      const double diagonal=normal[k][k];for(int j=k;j<8;++j)normal[k][j]/=diagonal;rhs[k]/=diagonal;
      for(int i=0;i<8;++i)if(i!=k){const double factor=normal[i][k];
        for(int j=k;j<8;++j)normal[i][j]-=factor*normal[k][j];rhs[i]-=factor*rhs[k];}
    }
    double trust=1.;for(int n=0;n<8;++n){rhs[n]/=columns[n];
      if(!std::isfinite(rhs[n]))return false;trust=std::max(trust,std::abs(rhs[n])/2.);}
    for(int n=0;n<8;++n)(*direction)[n]=rhs[n]/trust;
    return true;
  }
  // Use one physical CP scale per optimization coordinate in both chain-rule
  // Jacobians and curve increments. A shared P3-dominated scale suppresses far
  // control points under the finite trust box, despite their conditioned chart.
  static Coefficients trackingScales(const Curve& q,const SolveRequest& r,const SolverConfig& c){
    const auto shared=scales(q,r,c);Coefficients result{};
    for(int j=0;j<8;++j)result[j]=q.geometry_scale[j]>0.?q.geometry_scale[j]:shared[j/4];
    return result;
  }
  static void applyTrackingDirection(Curve& q,const Coefficients& direction,double alpha,
      const SolveRequest& r,const SolverConfig& c){
    const auto scale=trackingScales(q,r,c);const double co=std::cos(q.yaw),si=std::sin(q.yaw);
    for(int k=0;k<4;++k){const double x=alpha*direction[k]*scale[k],y=alpha*direction[k+4]*scale[k+4];
      q.control[0][k+3]+=co*x-si*y;q.control[1][k+3]+=si*x+co*y;}
  }
  // Verification interface. Derivatives are with respect to eight scaled CP
  // displacements, not eight extra issued inputs. The caller may verify the
  // gradient against the independently evaluated common Core objective.
  static bool trackingDirection(const Curve& q,const SolveRequest& r,const SolverConfig& c,
      Coefficients& direction,double& objective,Coefficients* gradient=nullptr,double continuity_weight=0.){
    if(!Core::validConfig(c)||!Core::validReference(r.reference,c))return false;
    Core core;SolveProfile profile;if(!core.prepare(r,c,profile))return false;
    return trackingNormal(q,r,c,core.prepared_,direction,objective,gradient,
      Clock::now()+std::chrono::seconds(1),continuity_weight);
  }
 private:
  static bool trackingNormal(const Curve& q,const SolveRequest& r,const SolverConfig& c,
      const Core::Prepared& cache,Coefficients& direction,double& objective,
      Coefficients* gradient,Clock::time_point deadline,double continuity_weight=0.,
      const std::array<Basis,kMaxHorizon>* speed_basis=nullptr,
      const std::array<Basis,kMaxHorizon>* steering_basis=nullptr,const Core::FixedPrefix* fixed=nullptr){
    using V=std::array<double,8>;
    struct Halfspace{V a{};double b=0.,norm_squared=0.;};
    std::array<Halfspace,10*kMaxHorizon+16> constraints{};int count=0;
    std::array<Command,kMaxHorizon> commands{};
    std::array<std::array<V,2>,kMaxHorizon> input{};
    double normal[8][8]{},rhs[8]{};objective=0.;
    const auto scale=trackingScales(q,r,c);const double co=std::cos(q.yaw),si=std::sin(q.yaw);
    auto add=[&](double value,const V& derivative,double weight){
      objective+=weight*value*value;
      for(int a=0;a<8;++a){rhs[a]-=weight*derivative[a]*value;
        for(int b=0;b<8;++b)normal[a][b]+=weight*derivative[a]*derivative[b];}
    };
    auto bounds=[&](double value,const V& derivative,double lo,double hi,double unit){
      auto& upper=constraints[count++];auto& lower=constraints[count++];
      upper.b=(hi-value)/unit;lower.b=(value-lo)/unit;
      for(int a=0;a<8;++a){
        upper.a[a]=derivative[a]/unit;lower.a[a]=-upper.a[a];
        upper.norm_squared+=upper.a[a]*upper.a[a];
      }
      lower.norm_squared=upper.norm_squared;
    };
    Command previous=r.previous;double previous_a=r.previous_command_acceleration;
    V previous_v{},previous_d{},previous_acceleration{},previous_rate_derivative{};
    const auto rate_memory=steeringRatePrior(r,c);
    const bool continuity=std::isfinite(continuity_weight)&&continuity_weight>0.&&rate_memory.valid;
    double previous_rate=rate_memory.value,previous_h=rate_memory.interval;
    std::array<V,5> S{};State state=r.state;
    for(int i=0;i<c.steps;++i){
      if(Clock::now()>=deadline)return false;
      const double issue=Core::startTime(i,r,c)+(q.delay_aligned?0.:Core::interval(i,r,c));
      const auto B=speed_basis?(*speed_basis)[i]:basis(q,inputTime(q,issue,c.model.speed_delay));
      const auto D=steering_basis?(*steering_basis)[i]:(q.delay_aligned?basis(q,inputTime(q,issue,c.model.steering_delay)):B);
      CommandDifferentialCache differential;
      if(!prepareCommandDifferential(q,B,D,c.model,differential))return false;
      commands[i]=differential.speed_flat.feedforward;
      if(q.delay_aligned)commands[i].steering=differential.steering_flat.feedforward.steering;
      for(int k=0;k<4;++k){double J[2][2]{};
        if(!cachedCommandJacobian(q,B,D,c.model,differential,J,k+3))return false;
        for(int channel=0;channel<2;++channel){
          input[i][channel][k]=(J[channel][0]*co+J[channel][1]*si)*scale[k];
          input[i][channel][k+4]=(-J[channel][0]*si+J[channel][1]*co)*scale[k+4];}
      }
      const double h=cache.h[i],a=(commands[i].speed-previous.speed)/h,
        rate=(commands[i].steering-previous.steering)/h,jerk=(a-previous_a)/h;
      V A{},R{},K{};
      for(int n=0;n<8;++n){A[n]=(input[i][0][n]-previous_v[n])/h;
        R[n]=(input[i][1][n]-previous_d[n])/h;K[n]=(A[n]-previous_acceleration[n])/h;}
      const double cap=std::min(r.speed_cap,c.limits.speed);
      bounds(commands[i].speed,input[i][0],0.,cap,std::max(.015,cap));
      bounds(commands[i].steering,input[i][1],-c.limits.steering,c.limits.steering,c.limits.steering);
      bounds(a,A,-c.limits.deceleration,c.limits.acceleration,std::min(c.limits.acceleration,c.limits.deceleration));
      bounds(rate,R,-c.limits.steering_rate,c.limits.steering_rate,c.limits.steering_rate);
      bounds(jerk,K,-c.limits.jerk,c.limits.jerk,c.limits.jerk);
      // Exactly Core's event partition, exponential actuators and midpoint SE2.
      // A historical fixed source has zero sensitivity to all current CPs.
      int first=cache.begin[i];
      if(fixed&&i<fixed->stage_count){
        const auto& stage=fixed->stages[i];
        state=stage.state;first=stage.end;
      }
      for(int j=first;j<cache.begin[i+1];++j){
        if(Clock::now()>=deadline)return false;
        const auto& step=cache.substeps[j];const auto& m=c.model;
        const Command& uv=step.speed.index<0?step.speed.fixed:commands[step.speed.index];
        const Command& ud=step.steering.index<0?step.steering.fixed:commands[step.steering.index];
        const double veq=m.speed_gain*uv.speed+m.speed_tau*m.longitudinal_residual;
        const double next_v=veq+(state.speed-veq)*step.speed_decay;
        const double deq=m.steering_gain*ud.steering;
        const double next_d=deq+(state.steering-deq)*step.steering_decay;
        const double vm=.5*(state.speed+next_v),dm=.5*(state.steering+next_d),td=std::tan(dm);
        const double omega=m.yaw_gain*vm*td/m.wheelbase+m.yaw_residual;
        const double middle=state.yaw+.5*step.dt*omega,cm=std::cos(middle),sm=std::sin(middle);
        std::array<V,5> next=S;
        for(int n=0;n<8;++n){
          const double Jv=step.speed.index<0?0.:input[step.speed.index][0][n];
          const double Jd=step.steering.index<0?0.:input[step.steering.index][1][n];
          next[3][n]=step.speed_decay*S[3][n]+(1.-step.speed_decay)*m.speed_gain*Jv;
          next[4][n]=step.steering_decay*S[4][n]+(1.-step.steering_decay)*m.steering_gain*Jd;
          const double Dv=.5*(S[3][n]+next[3][n]),Dd=.5*(S[4][n]+next[4][n]);
          const double Dw=m.yaw_gain*(td*Dv+vm*(1.+td*td)*Dd)/m.wheelbase;
          const double Dyaw=S[2][n]+.5*step.dt*Dw;
          next[0][n]=S[0][n]+step.dt*(cm*Dv+(-vm*sm-m.lateral_residual*cm)*Dyaw);
          next[1][n]=S[1][n]+step.dt*(sm*Dv+(vm*cm-m.lateral_residual*sm)*Dyaw);
          next[2][n]=S[2][n]+step.dt*Dw;
        }
        state.x+=step.dt*(vm*cm-m.lateral_residual*sm);
        state.y+=step.dt*(vm*sm+m.lateral_residual*cm);
        state.yaw=Core::predictionAngle(state.yaw+step.dt*omega);
        state.speed=next_v;state.steering=next_d;S=next;
      }
      const auto& p=cache.reference[i];const auto&m=c.model;
      const double cy=std::cos(state.yaw),sy=std::sin(state.yaw);
      const double ex=state.x+m.rear_to_center*cy-p.x,ey=state.y+m.rear_to_center*sy-p.y;
      const double lateral=-cache.ref_sin[i]*ex+cache.ref_cos[i]*ey;
      const double along=cache.ref_cos[i]*ex+cache.ref_sin[i]*ey;
      const double td=std::tan(state.steering);
      const double omega=m.yaw_gain*state.speed*td/m.wheelbase+m.yaw_residual;
      const double vy=m.lateral_residual+m.rear_to_center*omega;
      const double speed=std::hypot(state.speed,vy),den=state.speed*state.speed+vy*vy;
      const double heading=Core::predictionAngle(state.yaw+(std::abs(state.speed)>.015?
          std::atan2(vy,state.speed):0.)-p.yaw);
      V L{},T{},H{},Vspeed{},Omega{};
      for(int n=0;n<8;++n){
        const double dx=S[0][n]-m.rear_to_center*sy*S[2][n],dy=S[1][n]+m.rear_to_center*cy*S[2][n];
        L[n]=-cache.ref_sin[i]*dx+cache.ref_cos[i]*dy;
        T[n]=cache.ref_cos[i]*dx+cache.ref_sin[i]*dy;
        const double Domega=m.yaw_gain*(td*S[3][n]+state.speed*(1.+td*td)*S[4][n])/m.wheelbase;
        Omega[n]=Domega;
        const double Dvy=m.rear_to_center*Domega;
        H[n]=S[2][n]+(std::abs(state.speed)>.015?(state.speed*Dvy-vy*S[3][n])/den:0.);
        Vspeed[n]=speed>1e-12?(state.speed*S[3][n]+vy*Dvy)/speed:0.;
      }
      const double stage=h+(i==c.steps-1?.4:0.);
      add(lateral,L,stage*c.weights.lateral);add(along,T,stage*.15*c.weights.lateral);
      add(heading,H,stage*c.weights.heading);add(speed-p.speed,Vspeed,stage*c.weights.speed);
      if(cache.yaw_rate_weight>0.)
        add(omega-cache.body_yaw_rate[i],Omega,stage*cache.yaw_rate_weight);
      add(rate,R,h*.01*c.weights.smoothness);add(a,A,h*.02*c.weights.smoothness);
      add(jerk,K,h*.002*c.weights.smoothness*cache.terminal_jerk_scale);
      if(continuity){V difference_derivative{};
        for(int n=0;n<8;++n)difference_derivative[n]=R[n]-previous_rate_derivative[n];
        add(rate-previous_rate,difference_derivative,steeringRateChangeWeight(h,previous_h,c,continuity_weight));
        previous_rate=rate;previous_rate_derivative=R;previous_h=h;
      }
      previous=commands[i];previous_a=a;previous_v=input[i][0];previous_d=input[i][1];previous_acceleration=A;
    }
    if(!std::isfinite(objective))return false;
    if(gradient)for(int n=0;n<8;++n)(*gradient)[n]=-2.*rhs[n];
    // Column-relative damping preserves the CP preconditioning: using the
    // largest diagonal for every column erased weak far-horizon directions.
    for(int n=0;n<8;++n)normal[n][n]+=1e-6*std::max(1e-8,normal[n][n]);
    for(int k=0;k<8;++k){int pivot=k;for(int i=k+1;i<8;++i)if(std::abs(normal[i][k])>std::abs(normal[pivot][k]))pivot=i;
      if(std::abs(normal[pivot][k])<1e-16)return false;
      for(int j=0;j<8;++j)std::swap(normal[k][j],normal[pivot][j]);std::swap(rhs[k],rhs[pivot]);
      const double diagonal=normal[k][k];for(int j=k;j<8;++j)normal[k][j]/=diagonal;rhs[k]/=diagonal;
      for(int i=0;i<8;++i)if(i!=k){const double factor=normal[i][k];
        for(int j=k;j<8;++j)normal[i][j]-=factor*normal[k][j];rhs[i]-=factor*rhs[k];}
    }
    for(int n=0;n<8;++n){auto& upper=constraints[count++];auto& lower=constraints[count++];
      upper.a[n]=1.;lower.a[n]=-1.;upper.b=lower.b=2.;
      upper.norm_squared=lower.norm_squared=1.;}
    for(int pass=0;pass<6;++pass)for(int i=0;i<count;++i){
      if(Clock::now()>=deadline)return false;
      const auto& constraint=constraints[i];double violation=-constraint.b;
      for(int n=0;n<8;++n)violation+=constraint.a[n]*rhs[n];
      if(violation>0.&&constraint.norm_squared>1e-20)
        for(int n=0;n<8;++n)rhs[n]-=violation*constraint.a[n]/constraint.norm_squared;
    }
    for(int n=0;n<8;++n){if(!std::isfinite(rhs[n]))return false;direction[n]=rhs[n];}
    return true;
  }
 public:
  // A testable eight-coefficient seed projection; callers must still apply
  // the ordinary forward dynamics/constraint validation before accepting it.
  static bool projectCommands(Curve& q,const std::array<Command,kMaxHorizon>& target,
      const SolveRequest& r,const SolverConfig& c,bool rate_aware=true){
    double normal[8][8]{},rhs[8]{};
    struct Halfspace { std::array<double,8> a{};double b=0.; };
    // Five physical quantities, two bounds each, plus an eight-CP trust box.
    std::array<Halfspace,10*kMaxHorizon+16> constraints{};int count=0;
    const auto scale=scales(q,r,c);
    const double co=std::cos(q.yaw),si=std::sin(q.yaw);
    Command previous=r.previous,wanted_previous=r.previous;
    double previous_a=r.previous_command_acceleration,wanted_a=previous_a;
    std::array<double,8> previous_v{},previous_d{},previous_acceleration{};
    auto row=[&](double value,double wanted,double lo,double hi,
                 const std::array<double,8>& derivative,double unit,double h){
      const double residual=(wanted-value)/unit,weight=h;
      for(int a=0;a<8;++a){const double Ja=derivative[a]/unit;
        rhs[a]+=weight*Ja*residual;
        for(int b=0;b<8;++b)normal[a][b]+=weight*Ja*derivative[b]/unit;}
      auto& upper=constraints[count++];auto& lower=constraints[count++];
      upper.b=(hi-value)/unit;lower.b=(value-lo)/unit;
      for(int a=0;a<8;++a){upper.a[a]=derivative[a]/unit;lower.a[a]=-upper.a[a];}
    };
    for(int i=0;i<c.steps;++i){
      const double issue=Core::startTime(i,r,c)+(q.delay_aligned?0.:Core::interval(i,r,c));
      const auto B=basis(q,inputTime(q,issue,c.model.speed_delay));
      const auto steering_basis=q.delay_aligned?basis(q,inputTime(q,issue,c.model.steering_delay)):B;
      CommandDifferentialCache differential;
      if(!prepareCommandDifferential(q,B,steering_basis,c.model,differential))return false;
      Command actual=differential.speed_flat.feedforward;
      if(q.delay_aligned)actual.steering=differential.steering_flat.feedforward.steering;
      double J[2][8]{};
      for(int k=0;k<4;++k){double A[2][2]{};
        if(!cachedCommandJacobian(q,B,steering_basis,c.model,differential,A,k+3))return false;
        for(int channel=0;channel<2;++channel){
          J[channel][k]=(A[channel][0]*co+A[channel][1]*si)*scale[0];
          J[channel][k+4]=(-A[channel][0]*si+A[channel][1]*co)*scale[1];
        }
      }
      const double h=Core::interval(i,r,c),cap=std::min(c.limits.speed,r.speed_cap);
      if(!rate_aware){
        const double residual[2]={target[i].speed-actual.speed,target[i].steering-actual.steering};
        for(int channel=0;channel<2;++channel){
          const double unit=channel?c.limits.steering:cap,weight=h/(unit*unit);
          for(int a=0;a<8;++a){rhs[a]+=weight*J[channel][a]*residual[channel];
            for(int b=0;b<8;++b)normal[a][b]+=weight*J[channel][a]*J[channel][b];}
        }
        continue;
      }
      const double acceleration=(actual.speed-previous.speed)/h,rate=(actual.steering-previous.steering)/h;
      const double target_a=(target[i].speed-wanted_previous.speed)/h;
      const double target_rate=(target[i].steering-wanted_previous.steering)/h;
      std::array<double,8> V{},D{},A{},R{},K{};
      for(int a=0;a<8;++a){V[a]=J[0][a];D[a]=J[1][a];
        A[a]=(V[a]-previous_v[a])/h;R[a]=(D[a]-previous_d[a])/h;
        K[a]=(A[a]-previous_acceleration[a])/h;}
      // Normalize by the unchanged physical limits, not empirically tuned
      // per-route weights. Matching amplitudes alone cannot preserve jerk:
      // the measured first interval is ~33 ms and later intervals are 100 ms.
      row(actual.speed,target[i].speed,0.,cap,V,std::max(.015,cap),h);
      row(actual.steering,target[i].steering,-c.limits.steering,c.limits.steering,D,c.limits.steering,h);
      row(acceleration,target_a,-c.limits.deceleration,c.limits.acceleration,A,
          std::min(c.limits.acceleration,c.limits.deceleration),h);
      row(rate,target_rate,-c.limits.steering_rate,c.limits.steering_rate,R,c.limits.steering_rate,h);
      row((acceleration-previous_a)/h,(target_a-wanted_a)/h,-c.limits.jerk,c.limits.jerk,K,c.limits.jerk,h);
      previous=actual;wanted_previous=target[i];previous_a=acceleration;wanted_a=target_a;
      previous_v=V;previous_d=D;previous_acceleration=A;
    }
    for(int i=0;i<8;++i)normal[i][i]+=1e-6;
    for(int k=0;k<8;++k){int pivot=k;for(int i=k+1;i<8;++i)if(std::abs(normal[i][k])>std::abs(normal[pivot][k]))pivot=i;
      if(std::abs(normal[pivot][k])<1e-14)return false;
      for(int j=0;j<8;++j)std::swap(normal[k][j],normal[pivot][j]);
      std::swap(rhs[k],rhs[pivot]);
      const double v=normal[k][k];for(int j=k;j<8;++j)normal[k][j]/=v;rhs[k]/=v;
      for(int i=0;i<8;++i)if(i!=k){const double f=normal[i][k];
        for(int j=k;j<8;++j)normal[i][j]-=f*normal[k][j];
        rhs[i]-=f*rhs[k];}
    }
    double maximum=1.;for(double x:rhs){if(!std::isfinite(x))return false;maximum=std::max(maximum,std::abs(x)/2.);}
    for(double& x:rhs)x/=maximum;
    if(!rate_aware){
      for(int k=0;k<4;++k){const double a=rhs[k]*scale[0],b=rhs[k+4]*scale[1];
        q.control[0][k+3]+=co*a-si*b;q.control[1][k+3]+=si*a+co*b;}
      return true;
    }
    for(int a=0;a<8;++a){auto& upper=constraints[count++];auto& lower=constraints[count++];
      upper.a[a]=1.;lower.a[a]=-1.;upper.b=lower.b=2.;}
    // A bounded linearized restoration in the SAME eight CP coordinates.
    // Halfspace projection modifies an optimization step, never output commands.
    // Linear feasibility is not nonlinear feasibility: caller round-trips the
    // spline chart and accepts ONLY Core's complete delayed physical rollout.
    for(int pass=0;pass<6;++pass)for(int i=0;i<count;++i){const auto& constraint=constraints[i];
      double violation=-constraint.b,norm=0.;
      for(int a=0;a<8;++a){violation+=constraint.a[a]*rhs[a];norm+=constraint.a[a]*constraint.a[a];}
      if(violation>0.&&norm>1e-20)for(int a=0;a<8;++a)rhs[a]-=violation*constraint.a[a]/norm;
    }
    for(double x:rhs)if(!std::isfinite(x))return false;
    for(int k=0;k<4;++k){const double a=rhs[k]*scale[0],b=rhs[k+4]*scale[1];
      q.control[0][k+3]+=co*a-si*b;q.control[1][k+3]+=si*a+co*b;}
    return true;
  }
 private:
  static bool sameBasisLayout(const Curve& a,const Curve& b){
    if(a.delay_aligned!=b.delay_aligned||a.horizon!=b.horizon||a.knots!=b.knots||
       a.response_base!=b.response_base)return false;
    if(!a.response_base)return true;
    const auto& x=a.response_anchor;const auto& y=b.response_anchor;
    return x.x==y.x&&x.y==y.y&&x.dx==y.dx&&x.dy==y.dy&&x.ddx==y.ddx&&x.ddy==y.ddy;
  }
  void updateInputWeights(const SolveRequest& r,const SolverConfig& c){
    // Reuse exactly equal sample times within the same spline chart. The
    // original times, validation order and model stay identical.
    std::array<double,3*kMaxHorizon> times;
    std::array<const Basis*,3*kMaxHorizon> first_basis;
    int count=0;
    const auto assign=[&](double t,Basis& weight,unsigned mask){
      int slot=0;
      while(slot<count && times[slot]!=t)++slot;
      if(slot==count){
        times[count++]=t;weight=basis(anchor_,t);first_basis[slot]=&weight;sample_input_mask_[slot]=mask;
        auto& anchored=anchored_jet_terms_[slot];
        anchored={};addJetTerms<0,3>(anchored,anchor_,weight);
      }
      else{weight=*first_basis[slot];sample_input_mask_[slot]|=mask;}
      return slot;
    };
    constexpr unsigned kInverseSpeed=FlatnessReferenceModel::kInverseSpeed;
    constexpr unsigned kInverseSteering=FlatnessReferenceModel::kInverseSteering;
    for(int i=0;i<c.steps;++i){const double time=Core::startTime(i,r,c)+Core::interval(i,r,c);
      input_slot_[i][0]=assign(time,weights_[i],anchor_.delay_aligned?0:kInverseSpeed|kInverseSteering);
      if(anchor_.delay_aligned){const double issue_time=Core::startTime(i,r,c);
        input_slot_[i][1]=assign(inputTime(anchor_,issue_time,c.model.speed_delay),speed_weights_[i],kInverseSpeed);
        input_slot_[i][2]=assign(inputTime(anchor_,issue_time,c.model.steering_delay),steering_weights_[i],kInverseSteering);}
    }
  }
  void prepareChart(const SolveRequest& r,const SolverConfig& c){
    // Scale, condition and evaluate in the same chart and activation grid.
    updateInputWeights(r,c);
    const auto& speed=anchor_.delay_aligned?speed_weights_:weights_;
    const auto& steering=anchor_.delay_aligned?steering_weights_:speed;
    anchor_.decision_scale=scales(anchor_,r,c,&speed,&steering);
    if(conditioned_geometry_)conditionGeometry(anchor_,r,c,speed,steering);
    diagnostics_.interior_knot=anchor_.knots[6];
  }
  static std::array<double,2> scales(const Curve& q,const SolveRequest& r,const SolverConfig& c,
      const BasisGrid* speed=nullptr,const BasisGrid* steering=nullptr){
    if(q.decision_scale[0]>0.&&q.decision_scale[1]>0.)return q.decision_scale;
    const double v=std::min(c.limits.speed,r.speed_cap),H=q.horizon;
    // Normalize CP coordinates with conservative nominal input sensitivities.
    // Without this basis-aware scaling, a geometric CP displacement can produce
    // enormous inverse-actuator commands through the third derivative.  This
    // scaling only defines the search chart; feasibility still uses Core.
    double velocity_sensitivity=0.,acceleration_sensitivity=0.,steering_sensitivity=0.;
    const double nominal_speed=std::max(.015,std::abs(r.state.speed));
    for(int i=0;i<c.steps;++i){const double time=Core::startTime(i,r,c)+(q.delay_aligned?0.:Core::interval(i,r,c));
      const auto B=speed?(*speed)[i]:basis(q,inputTime(q,time,c.model.speed_delay));
      const auto D=steering?(*steering)[i]:(q.delay_aligned?basis(q,inputTime(q,time,c.model.steering_delay)):B);
      for(int k=3;k<7;++k){
        velocity_sensitivity=std::max(velocity_sensitivity,(std::abs(B.d[1][k])+c.model.speed_tau*std::abs(B.d[2][k]))/c.model.speed_gain);
        acceleration_sensitivity=std::max(acceleration_sensitivity,(std::abs(B.d[2][k])+c.model.speed_tau*std::abs(B.d[3][k]))/c.model.speed_gain);
        steering_sensitivity=std::max(steering_sensitivity,c.model.wheelbase*(std::abs(D.d[2][k])+c.model.steering_tau*std::abs(D.d[3][k]))/
          (nominal_speed*nominal_speed*c.model.steering_gain));
      }
    }
    return {{std::max(1e-8,std::min({.5*c.limits.acceleration*H*H,v/std::max(1e-8,velocity_sensitivity),
      std::min(c.limits.acceleration,c.limits.deceleration)/std::max(1e-8,acceleration_sensitivity)})),
      std::max(1e-8,std::min(.5*v*v*std::tan(c.limits.steering)/c.model.wheelbase*H*H,
        c.limits.steering/std::max(1e-8,steering_sensitivity)))}};
  }
  // Normalize each remaining geometric DOF after eliminating P3 through
  // the first physical command. A shared maximum dominated by P3's third
  // derivative previously suppressed the far control points by orders of
  // magnitude. This is a nonsingular diagonal chart change, not extra DOFs
  // or relaxed actuator constraints. All nonlinear constraints still use Core.
  static void conditionGeometry(Curve& q,const SolveRequest& r,const SolverConfig& c,
      const BasisGrid& speed_basis,const BasisGrid& steering_basis){
    Curve linearization;
    const auto& first=speed_basis[0];
    const auto& first_steering=steering_basis[0];
    CommandDifferentialCache first_cache;
    if(!curveFromDecision(q,neutralDecision(r,c),r,c,linearization,true,nullptr,false,
        &first,&first_steering,&first_cache))return;
    double A[2][2]{};
    if(!cachedCommandJacobian(linearization,first,first_steering,c.model,first_cache,A))return;
    const double det=A[0][0]*A[1][1]-A[0][1]*A[1][0];
    if(!std::isfinite(det)||std::abs(det)<1e-12)return;
    const double co=std::cos(q.yaw),si=std::sin(q.yaw);
    struct KnotSensitivity {
      double input[4][2][2]{};
      bool valid[4]{};
    };
    std::array<KnotSensitivity,kMaxHorizon> sensitivity{};
    for(int i=0;i<c.steps;++i){
      const auto& speed=speed_basis[i];
      const auto& steering=steering_basis[i];
      CommandDifferentialCache cache;
      if(!prepareCommandDifferential(linearization,speed,steering,c.model,cache))continue;
      for(int k=0;k<4;++k)
        sensitivity[i].valid[k]=cachedCommandJacobian(linearization,speed,steering,c.model,
            cache,sensitivity[i].input[k],k+3);
    }
    for(int axis=0;axis<2;++axis)for(int k=1;k<4;++k){
      double F[2][2]{};
      if(!cachedCommandJacobian(linearization,first,first_steering,c.model,first_cache,F,k+3))continue;
      const double dx=axis?-si:co,dy=axis?co:si;
      const double fx=F[0][0]*dx+F[0][1]*dy,fy=F[1][0]*dx+F[1][1]*dy;
      const double px=(-fx*A[1][1]+fy*A[0][1])/det,py=(-fy*A[0][0]+fx*A[1][0])/det;
      double vs=0.,ds=0.,as=0.,rs=0.,oldv=0.,oldd=0.;
      for(int i=0;i<c.steps;++i){
        const double h=Core::interval(i,r,c);
        const auto& step=sensitivity[i];
        if(!step.valid[0]||!step.valid[k]){vs=ds=INFINITY;break;}
        const auto& J=step.input[0];const auto& K=step.input[k];
        const double v=K[0][0]*dx+K[0][1]*dy+J[0][0]*px+J[0][1]*py;
        const double d=K[1][0]*dx+K[1][1]*dy+J[1][0]*px+J[1][1]*py;
        vs=std::max(vs,std::abs(v));ds=std::max(ds,std::abs(d));
        as=std::max(as,std::abs(v-oldv)/h);rs=std::max(rs,std::abs(d-oldd)/h);oldv=v;oldd=d;
      }
      const double distance=axis?.5*r.state.speed*r.state.speed*std::tan(c.limits.steering)/c.model.wheelbase*q.horizon*q.horizon:
        .5*c.limits.acceleration*q.horizon*q.horizon;
      q.geometry_scale[axis*4+k]=std::max(1e-8,std::min({distance,
        c.limits.speed/std::max(1e-8,vs),c.limits.steering/std::max(1e-8,ds),
        std::min(c.limits.acceleration,c.limits.deceleration)/std::max(1e-8,as),
        c.limits.steering_rate/std::max(1e-8,rs)}));
    }
  }
  static Coefficients neutralDecision(const SolveRequest& r,const SolverConfig& c){
    Coefficients z{};double alo,ahi,dlo,dhi;
    if(Core::demandBounds(r.previous,r.previous_command_acceleration,r.control_dt,std::min(r.speed_cap,c.limits.speed),c.limits,alo,ahi,dlo,dhi)){
      z[0]=Core::encode(r.previous_command_acceleration,alo,ahi);z[4]=Core::encode(0.,dlo,dhi);
    }return z;
  }
 public:
  // Exposed for a direct chart round-trip test; this never validates a command.
  static Coefficients encodeCurve(const Curve& anchor,const Curve& q,const SolveRequest& r,const SolverConfig& c,bool preserve_geometry=false,bool independent_inputs=false){
    auto z=neutralDecision(r,c);const auto scale=scales(anchor,r,c);const double co=std::cos(anchor.yaw),si=std::sin(anchor.yaw);
    for(int k=1;k<4;++k){const double dx=q.control[0][k+3]-anchor.control[0][k+3],dy=q.control[1][k+3]-anchor.control[1][k+3];
      z[k]=(co*dx+si*dy)/(anchor.geometry_scale[k]>0.?anchor.geometry_scale[k]:scale[0]);
      z[k+4]=(-si*dx+co*dy)/(anchor.geometry_scale[k+4]>0.?anchor.geometry_scale[k+4]:scale[1]);
    }
    double alo,ahi,dlo,dhi;
    Command first;
    if(commandAt(q,q.delay_aligned?0.:r.control_dt,c.model,first)&&Core::demandBounds(r.previous,r.previous_command_acceleration,r.control_dt,std::min(r.speed_cap,c.limits.speed),c.limits,alo,ahi,dlo,dhi)){
      z[0]=Core::encode((first.speed-r.previous.speed)/r.control_dt,alo,ahi);
      z[4]=Core::encode((first.steering-r.previous.steering)/r.control_dt,dlo,dhi);
    }
    if(preserve_geometry){
      for(int j=0;j<8;++j){const double bound=(j==0||j==4)?1.:8.;z[j]=clamp(z[j],-bound,bound);}
      return z;
    }
    if(independent_inputs){
      // Only contract the six-dimensional shape ray. z0/z4 already encode the
      // bounded first acceleration and steering rate, so retain them exactly.
      double maximum=1.;for(int j:{1,2,3,5,6,7})maximum=std::max(maximum,std::abs(z[j]));
      for(int j:{1,2,3,5,6,7})z[j]/=maximum;
      return z;
    }
    const auto neutral=neutralDecision(r,c);double maximum=1.;for(int j=0;j<8;++j)maximum=std::max(maximum,std::abs(z[j]-neutral[j]));
    for(int j=0;j<8;++j)z[j]=clamp(neutral[j]+(z[j]-neutral[j])/maximum,-1.,1.);
    return z;
  }
 private:
  static double interiorFraction(const SolveRequest& r,const SolverConfig& c,double H,bool adaptive){
    if(!adaptive)return .5;
    // Fixed dimensionless reference-only density; not a terrain/friction oracle.
    // One interior knot is the weighted time median.  The admissible knot
    // interval also reserves two actual control intervals at either end.
    std::array<double,kMaxHorizon> mass{};double total=0.;
    double last_yaw=r.reference.points[0].yaw,last_k=r.reference.points[0].curvature;
    for(int i=0;i<c.steps;++i){const auto p=Core::referenceAt(r.reference,Core::startTime(i,r,c)+Core::interval(i,r,c));
      mass[i]=Core::interval(i,r,c)*(1.+std::abs(wrap(p.yaw-last_yaw))+H*std::abs(p.speed*(p.curvature-last_k)));
      total+=mass[i];last_yaw=p.yaw;last_k=p.curvature;
    }
    double cumulative=0.,time=0.;
    for(int i=0;i<c.steps;++i){if(cumulative+mass[i]>=.5*total){time=Core::startTime(i,r,c)+Core::interval(i,r,c)*(.5*total-cumulative)/mass[i];break;}cumulative+=mass[i];}
    const double edge=std::min(.25,2.*r.control_dt/H);return clamp(time/H,std::max(.05,edge),std::min(.95,1.-edge));
  }
  static bool fit(Curve& q,const std::array<std::array<double,2>,kMaxHorizon+1>& samples,
      const std::array<double,kMaxHorizon+1>& times,int count,double* rmse){
    if(count<4)return false;
    double normal[4][4]{},rhs[4][2]{};
    for(int i=0;i<count;++i){const auto B=basis(q,times[i]);double residual[2]={samples[i][0]-B.offset.x,samples[i][1]-B.offset.y};
      for(int axis=0;axis<2;++axis)for(int k=0;k<3;++k)residual[axis]-=B.d[0][k]*q.control[axis][k];
      for(int a=0;a<4;++a){for(int b=0;b<4;++b)normal[a][b]+=B.d[0][a+3]*B.d[0][b+3];
        for(int axis=0;axis<2;++axis)rhs[a][axis]+=B.d[0][a+3]*residual[axis];}
    }
    // Pivoted four-variable least squares.  No dynamic allocation, no changes
    // to the immutable original reference and no fictitious sensor samples.
    for(int k=0;k<4;++k){int pivot=k;for(int j=k+1;j<4;++j)if(std::abs(normal[j][k])>std::abs(normal[pivot][k]))pivot=j;
      if(std::abs(normal[pivot][k])<1e-14)return false;
      if(pivot!=k){for(int j=0;j<4;++j)std::swap(normal[pivot][j],normal[k][j]);for(int axis=0;axis<2;++axis)std::swap(rhs[pivot][axis],rhs[k][axis]);}
      const double d=normal[k][k];for(int j=k;j<4;++j)normal[k][j]/=d;for(int axis=0;axis<2;++axis)rhs[k][axis]/=d;
      for(int i=0;i<4;++i)if(i!=k){const double f=normal[i][k];for(int j=k;j<4;++j)normal[i][j]-=f*normal[k][j];for(int axis=0;axis<2;++axis)rhs[i][axis]-=f*rhs[k][axis];}
    }
    for(int axis=0;axis<2;++axis)for(int k=0;k<4;++k){if(!std::isfinite(rhs[k][axis]))return false;q.control[axis][k+3]=rhs[k][axis];}
    double square=0.;for(int i=0;i<count;++i){const auto v=jet(q,times[i]);square+=(v.x-samples[i][0])*(v.x-samples[i][0])+(v.y-samples[i][1])*(v.y-samples[i][1]);}
    if(rmse)*rmse=std::sqrt(square/count);
    return true;
  }
};
}
