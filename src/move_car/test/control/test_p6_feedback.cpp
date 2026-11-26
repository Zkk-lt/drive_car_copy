#include "control/p6_slqr_feedback.h"
#include "control/p6_observation_hold.h"
#include <iostream>
#include <stdexcept>
using namespace research_control;
void check(bool b,const char*s){if(!b)throw std::runtime_error(s);}
// Independent dense frozen Riccati oracle, retaining the original maximum
// queue. This validates exact state elimination and sparse multiplication,
// rather than checking the optimized implementation against itself.
P6PolicyValidator::Policy denseReference(const SolveRequest&r,const SolverConfig&c,const SolveResult&n,bool course=false){
 using M=Eigen::Matrix<double,26,26>;using V=Eigen::Matrix<double,26,1>;
 P6PolicyValidator::Policy out;out.h=1./30.;out.queue_size=21;
 out.count=std::min(124,static_cast<int>(std::ceil((r.control_dt+(c.steps-1)*c.dt)/out.h)));
 M P=M::Zero();P(0,0)=90.;P(1,1)=40.;P(2,2)=.55;
 auto courseDerivative=[&](const State&z){
  auto angle=[&](double d){return std::atan2(c.model.lateral_residual+c.model.rear_to_center*(c.model.yaw_gain*z.speed*std::tan(d)/c.model.wheelbase+c.model.yaw_residual),z.speed);};
  return (angle(z.steering+1e-6)-angle(z.steering-1e-6))/2e-6;
 };
 if(course){const double b=courseDerivative(P6PolicyValidator::nominalAt(n,r,c,r.control_dt+(c.steps-1)*c.dt));P(1,2)=P(2,1)=40.*b;P(2,2)+=40.*b*b;}
 const double rr=c.weights.smoothness*.01/out.h,ru=.55*out.h;
 for(int k=out.count-1;k>=0;--k){
  const auto z=P6PolicyValidator::nominalAt(n,r,c,k*out.h);
  const double sec=1./std::cos(z.steering),rd=c.model.yaw_gain*z.speed*sec*sec/c.model.wheelbase;
  const double yr=c.model.yaw_gain*z.speed*std::tan(z.steering)/c.model.wheelbase+c.model.yaw_residual;
  const double vy=c.model.lateral_residual+c.model.rear_to_center*yr,beta=std::atan2(vy,z.speed);
  M A=M::Zero(),Q=M::Zero();V B=V::Zero(),cross=V::Zero();
  A(0,0)=A(1,1)=1.;A(0,1)=out.h*std::hypot(z.speed,vy);
  A(0,2)=out.h*c.model.rear_to_center*rd*std::cos(beta);A(1,2)=out.h*rd;
  int m=0;double a,bo,bn;P6PolicyValidator::actuatorCoefficients(c.model.steering_delay,c.model.steering_tau,c.model.steering_gain,out.h,m,a,bo,bn);
  A(2,2)=a;if(!m)B(2)=bn;else A(2,3+m-1)+=bn;
  if(bo>1e-14)A(2,3+m)+=bo;
  B(3)=1.;for(int j=1;j<out.queue_size;++j)A(3+j,3+j-1)=1.;
  Q(0,0)=c.weights.lateral*out.h;Q(1,1)=c.weights.heading*out.h;Q(2,2)=.55*out.h;Q(3,3)=rr;cross(3)=-rr;
  if(course){const double b=courseDerivative(z);Q(1,2)=Q(2,1)=c.weights.heading*out.h*b;Q(2,2)+=c.weights.heading*out.h*b*b;}
  const Eigen::Matrix<double,1,26> G=B.transpose()*P*A+cross.transpose();
  const double H=ru+rr+(B.transpose()*P*B)(0,0);out.gains[k]=G/H;
  P=Q+A.transpose()*P*A-G.transpose()*G/H;P=.5*(P+P.transpose()).eval();
 }
 return out;
}
int main(){try{
 // Same IMU identity with a qualified CAN receipt after the frozen state.
 const auto can_only=[](double wheel=10.008,double steering=10.008,double imu=9.998,
                        double source=9.995,double now=10.012,double apply=10.030) {
  return p6CanOnlyTiming(10.,10.005,9.999,9.995,wheel,steering,imu,source,now,apply);
 };
 check(can_only(),"causal CAN-only update rejected");
 check(!can_only(10.,10.),"CAN-only update reused frozen receipt");
 check(!can_only(10.008,10.007),"CAN-only update reused deferred steering");
 check(!can_only(10.014,10.014),"future CAN receipt accepted");
 check(!can_only(10.008,10.008,10.015),"future IMU receipt accepted");
 check(!can_only(10.008,10.008,9.998,9.994),"regressed IMU source accepted");
 check(!can_only(10.008,10.008,9.998,9.996),"new IMU source misclassified as held");
 check(!can_only(10.008,10.008,9.998,9.995,10.031),"late CAN-only candidate accepted");
 check(!p6CanOnlyTiming(10.,10.005,9.911,9.900,10.008,10.008,9.940,9.900,10.012,10.030),
       "skewed IMU accepted with new CAN");
 check(!p6CanOnlyTiming(10.,10.005,9.901,9.880,10.008,10.008,10.001,9.880,10.012,10.030),
       "stale held IMU accepted with new CAN");
 SolverConfig c;SolveRequest r;r.now=10.;r.state.speed=.24;r.previous.speed=.24;r.baseline=r.previous;r.speed_cap=.3;
 for(int i=0;i<45;++i)r.history.push_back({r.now-1.5+i/30.,r.previous});
 r.reference.valid=true;r.reference.count=21;r.reference.stamp=10.;
 for(int i=0;i<21;++i){auto&p=r.reference.points[i];p.t=i*.1;p.x=.3+.24*p.t;p.s=.24*p.t;p.speed=.24;}
 ReducedNMPC core;auto nominal=core.solve(r,c);check(nominal.ok,"nominal solve failed");
 P6PolicyValidator::Policy p;auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(100);
 check(P6PolicyValidator::build(r,c,nominal,p,deadline),"delay Riccati build failed");
 check(p.queue_size>=10,"pure delay queue missing");
 for(double delay:{0.,1./30.-.0001,1./30.,.325,.70}){
  auto model=c;model.model.steering_delay=delay;auto trajectory=nominal;
  for(int i=0;i<trajectory.count;++i){trajectory.predicted[i].steering=.12*std::sin(.2*i);
   trajectory.predicted[i].speed=.20+.002*i;}
  P6PolicyValidator::Policy active;
  check(P6PolicyValidator::build(r,model,trajectory,active,
    std::chrono::steady_clock::now()+std::chrono::seconds(1)),"sparse reference build failed");
  const auto oracle=denseReference(r,model,trajectory);
  check(active.count==oracle.count,"sparse/dense horizon mismatch");
  for(int k=0;k<active.count;++k)
   check((active.gains[k]-oracle.gains[k]).cwiseAbs().maxCoeff()<1e-9,"sparse/dense Riccati gain mismatch");
 }
 // Actual center-course output derivative and an independent dense oracle.
 for(double delta:{-.3,0.,.3}){
  auto z=r.state;z.steering=delta;
  auto angle=[&](double d){return std::atan2(c.model.rear_to_center*c.model.yaw_gain*z.speed*std::tan(d)/c.model.wheelbase,z.speed);};
  const double numeric=(angle(delta+1e-6)-angle(delta-1e-6))/2e-6;
  check(std::abs(numeric-P6PolicyValidator::courseSteeringJacobian(z,c.model))<1e-8,"course output Jacobian mismatch");
 }
 P6PolicyValidator::Policy course;
 check(P6PolicyValidator::build(r,c,nominal,course,std::chrono::steady_clock::now()+std::chrono::seconds(1),true),"course Riccati build failed");
 const auto course_oracle=denseReference(r,c,nominal,true);
 for(int k=0;k<course.count;++k)check((course.gains[k]-course_oracle.gains[k]).cwiseAbs().maxCoeff()<1e-8,"course/dense gain mismatch");
 check((course.gains[0]-p.gains[0]).cwiseAbs().maxCoeff()>1e-6,"course correction has no actual effect");
 // The independent dense oracle above can exceed the earlier build deadline
 // in unoptimized assertion builds. Give this functional validation its own
 // deadline; production budgets and explicit expiry checks below are unchanged.
 deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
 auto zero=P6PolicyValidator::validate(r,c,nominal,p,r.state,r.now,r.history,deadline,true);
 check(zero.solve.ok&&!zero.feedback_nonzero,"zero error did not restore nominal");
 State perturbed=r.state;perturbed.y=.0002;
 auto active=P6PolicyValidator::validate(r,c,nominal,p,perturbed,r.now,r.history,deadline,true);
 check(active.feedback_nonzero&&active.feedback<0.,"local feedback has no/counteracting response");
 check(active.solve.ok&&active.solve.max_violation<=1e-7,"combined command did not pass full original constraints");
 auto disabled=P6PolicyValidator::validate(r,c,nominal,p,perturbed,r.now,r.history,deadline,false);
 check(disabled.solve.ok&&disabled.solve.command.steering==nominal.command.steering,"K0 did not restore nominal command");
 State bad=r.state;bad.y=.1;auto rejected=P6PolicyValidator::validate(r,c,nominal,p,bad,r.now,r.history,deadline,true);
 check(!rejected.solve.ok,"outside feedback domain accepted");
 // A fresh local error can make the unconstrained Riccati input violate the
 // first steering-rate interval while the nominal trajectory remains safe.
 // The constrained scalar solve must retain a verified solution, never clip
 // and assume feasibility, and leave the ordinary valid-feedback path exact.
 auto unchanged=P6PolicyValidator::validate(r,c,nominal,p,perturbed,r.now,r.history,
   std::chrono::steady_clock::now()+std::chrono::seconds(1),true,true);
 check(unchanged.solve.ok&&!unchanged.feedback_bounded&&
   unchanged.solve.command.steering==active.solve.command.steering,"bounded option changed an unconstrained feasible input");
 bool restored=false;auto rate_limited=c;rate_limited.limits.steering_rate=.0001;
 for(double lateral:{.001,.003,.006,.012,.020}){
  auto fresh=r.state;fresh.y=lateral;const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  const auto full=P6PolicyValidator::validate(r,rate_limited,nominal,p,fresh,r.now,r.history,until,true);
  const auto limited=P6PolicyValidator::validate(r,rate_limited,nominal,p,fresh,r.now,r.history,until,true,true);
  if(!full.solve.ok&&limited.solve.ok){
   restored=true;check(limited.feedback_bounded&&limited.feedback_scale>=0.&&limited.feedback_scale<1.,"scalar feedback scale not bounded");
   check(std::abs(limited.feedback)<=std::abs(full.feedback),"constrained feedback exceeds requested correction");
   const auto fresh_request=[&](){auto v=r;v.state=fresh;return v;}();
   check(ReducedNMPC::validateControls(fresh_request,rate_limited,limited.solve.controls,until).feasible,"bounded feedback failed independent physical rollout");
  }
 }
 check(restored,"constrained feedback did not recover any rate-limited local error");
 auto impossible=r.state;impossible.speed=r.speed_cap+.1;
 const auto unsafe=P6PolicyValidator::validate(r,c,nominal,p,impossible,r.now,r.history,
   std::chrono::steady_clock::now()+std::chrono::seconds(1),true,true);
 check(!unsafe.solve.ok,"bounded feedback accepted physically infeasible fresh speed");
 const auto limited_expired=P6PolicyValidator::validate(r,c,nominal,p,perturbed,r.now,r.history,
   std::chrono::steady_clock::now(),true,true);
 check(!limited_expired.solve.ok&&limited_expired.solve.timed_out,"bounded feedback bypassed deadline");
 check(!P6PolicyValidator::validate(r,c,nominal,p,perturbed,missing(),r.history,
   std::chrono::steady_clock::now()+std::chrono::seconds(1),true,true).solve.ok,"NaN feedback timestamp accepted");
 auto no_gain=p;no_gain.count=0;
 check(!P6PolicyValidator::validate(r,c,nominal,no_gain,perturbed,r.now,r.history,
   std::chrono::steady_clock::now()+std::chrono::seconds(1),true,true).solve.ok,"empty feedback policy accepted");
 // Exact fractional delayed first-order step vs independent 10 us integration.
 int m;double a,bo,bn;P6PolicyValidator::actuatorCoefficients(.325,.44,.83,1./30.,m,a,bo,bn);
 const double d0=.04,old=-.09,next=.13,h=1./30.;double numeric=d0,t=0.;
 while(t<h-1e-12){double dt=std::min(1e-5,h-t);const double split=.325-m*h;
  if(t<split&&t+dt>split)dt=split-t;const double u=t<split?old:next;numeric+=(.83*u-numeric)/.44*dt;t+=dt;}
 check(std::abs(numeric-(a*d0+bo*old+bn*next))<1e-6,"fractional delay discretization mismatch");
 // Independent local physical closure about a constant straight nominal, with
 // all queue errors retained. This tests the gain beyond its first sign.
 std::vector<CommandSample> actual=r.history;double y=.002,psi=0.,delta=0.,last_u=0.;
 for(int k=0;k<300;++k){const double now=r.now+k/30.;Eigen::Matrix<double,26,1> e=Eigen::Matrix<double,26,1>::Zero();
  e(0)=y+c.model.rear_to_center*std::sin(psi);e(1)=psi;e(2)=delta;
  for(int j=0;j<p.queue_size;++j)e(3+j)=P6PolicyValidator::held(actual,now-(j+1)/30.).steering;
  const double u=-(p.gains[0]*e)(0,0);check(std::abs(u)<=c.limits.steering&&std::abs(u-last_u)<=c.limits.steering_rate/30.,"local feedback physical input bound");
  actual.push_back({now,{.24,u}});last_u=u;
  for(int j=0;j<40;++j){const double h=1./1200.,t=now+j*h;
   const double delayed=P6PolicyValidator::held(actual,t-c.model.steering_delay).steering;
   delta+=h*(delayed-delta)/c.model.steering_tau;const double omega=.24*std::tan(delta)/c.model.wheelbase;
   y+=h*.24*std::sin(psi+.5*h*omega);psi+=h*omega;
  }
 }
 check(std::abs(y+c.model.rear_to_center*std::sin(psi))<.002,"local delayed feedback did not reduce perturbation");
 auto expired=P6PolicyValidator::validate(r,c,nominal,p,r.state,r.now,r.history,std::chrono::steady_clock::now(),true);
 check(!expired.solve.ok&&expired.solve.timed_out,"feedback timeout not rejected");
 std::cout<<"PASS P6 delay-augmented feedback, nonzero correction="<<active.feedback<<" final_center_error="<<(y+c.model.rear_to_center*std::sin(psi))<<" zero/K0/constraints/fractional-delay/timeout\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
