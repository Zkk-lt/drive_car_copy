#include "control/p6_flatness_nmpc.h"
#include <cassert>
#include <iostream>
using namespace research_control;
using F=P6FlatnessNMPC;
using Clock=std::chrono::steady_clock;
SolveRequest request(){
 SolveRequest r;r.now=100.;r.control_dt=.05;r.previous={.24,0.};r.baseline=r.previous;
 r.speed_cap=.3;r.state={-.3,.015,.025,.24,0.,0.};r.published_history_contiguous=true;
 r.history={{99.,{.24,0.}},{99.9,{.24,-.02}},{99.95,{.24,-.01}},{100.,{.24,0.}}};
 r.reference.valid=true;r.reference.count=21;
 for(int i=0;i<21;++i){auto&p=r.reference.points[i];p.t=.1*i;p.x=.24*p.t;p.s=p.x;p.speed=.24;}
 return r;
}
int main(){
 auto r=request();SolverConfig c;
 auto m=F::publishedSteeringRate(r);assert(m.valid&&std::abs(m.value-.2)<1e-11);
 std::array<Command,kMaxHorizon> controls{};
 c.steps=3;controls[0]={.24,.01};controls[1]={.24,.03};controls[2]={.24,.05};
 assert(F::steeringRateContinuityCost(r,c,controls,1.)<1e-22);
 controls[1].steering=-.01;
 const double expected=.01*c.weights.smoothness*.1*.1*(.4*.4/.075+.8*.8/.1);
 assert(std::abs(F::steeringRateContinuityCost(r,c,controls,1.)-expected)<1e-12);
 assert(F::steeringRateContinuityCost(r,c,controls,0.)==0.);
 auto bad=r;bad.published_history_contiguous=false;assert(!F::publishedSteeringRate(bad).valid);
 assert(F::steeringRateContinuityCost(bad,c,controls,1.)==0.);
 bad=r;bad.history.back().stamp=100.01;assert(!F::publishedSteeringRate(bad).valid);
 bad=r;bad.history.back().stamp=bad.history[bad.history.size()-2].stamp;assert(!F::publishedSteeringRate(bad).valid);
 bad=r;bad.previous.steering=.01;assert(!F::publishedSteeringRate(bad).valid);
 bad=r;bad.now+=.121;assert(!F::publishedSteeringRate(bad).valid);
 bad=r;bad.history[bad.history.size()-2].stamp=99.8;assert(!F::publishedSteeringRate(bad).valid);
 // Measured publish spacing can make the recorded slope slightly larger
 // than the slope on the planning grid. Keep the prior and its exact cost.
 auto jitter=r;
 const double limit=c.limits.steering_rate;
 jitter.history[jitter.history.size()-2].command.steering=.05*limit*1.0004;
 const auto measured=F::publishedSteeringRate(jitter);
 const auto prior=F::steeringRatePrior(jitter,c);
 assert(measured.valid&&measured.value<-limit&&prior.valid&&prior.value==-limit);
 assert(F::steeringRateContinuityCost(jitter,c,controls,1.)>0.);
 assert(F::steeringRateContinuityCost(jitter,c,controls,1.)==
        F::steeringRateContinuityCost(jitter,c,controls,1.,prior));
 // Extra objective and its analytic derivatives must be the same cost, not
 // an unrelated heuristic direction or post-solver output filter.
 c=SolverConfig{};auto q=F::anchoredCurve(r.state,c.model,r.control_dt+(c.steps-1)*c.dt);
 q.delay_aligned=true;q.decision_scale={{.001,.001}};
 q.geometry_scale[1]=.002;q.geometry_scale[2]=.003;q.geometry_scale[3]=.004;
 q.geometry_scale[5]=.0025;q.geometry_scale[6]=.0035;q.geometry_scale[7]=.0045;
 auto commands=[&](const F::Curve& curve){std::array<Command,kMaxHorizon> u{};
  for(int i=0;i<c.steps;++i)assert(F::commandAt(curve,i?r.control_dt+(i-1)*c.dt:0.,c.model,u[i]));return u;};
 auto oracle=[&](const F::Curve& curve){const auto u=commands(curve);
  const auto checked=ReducedNMPC::validateControls(r,c,u,Clock::now()+std::chrono::seconds(1));
  assert(checked.feasible);return checked.cost+F::steeringRateContinuityCost(r,c,u,1.);};
 F::Coefficients direction{},gradient{};double value=missing();
 assert(F::trackingDirection(q,r,c,direction,value,&gradient,1.));
 assert(std::abs(value-oracle(q))<1e-10);
 const double eps=1e-5;
 for(int n=0;n<8;++n){F::Coefficients step{};step[n]=1.;auto lo=q,hi=q;
  F::applyTrackingDirection(lo,step,-eps,r,c);F::applyTrackingDirection(hi,step,eps,r,c);
  const double numerical=(oracle(hi)-oracle(lo))/(2.*eps);
  assert(std::abs(numerical-gradient[n])<1e-6*std::max(1.,std::abs(numerical)));
 }
 // Invalid history must restore the ORIGINAL objective, including gradient.
 bad=r;bad.published_history_contiguous=false;F::Coefficients g0{},g1{},unused{};double v0,v1;
 assert(F::trackingDirection(q,bad,c,unused,v0,&g0,0.));
 assert(F::trackingDirection(q,bad,c,unused,v1,&g1,1.));
 assert(v0==v1&&g0==g1);
 // On a curved reference the optional rate error contributes to the same
 // nonlinear objective and the analytic direction in all eight coordinates.
 c.yaw_rate_tracking_weight=2.;
 for(int i=0;i<r.reference.count;++i){auto& p=r.reference.points[i];p.curvature=.35;p.yaw=p.s*p.curvature;}
 assert(F::trackingDirection(q,r,c,direction,value,&gradient,1.));
 assert(std::abs(value-oracle(q))<1e-10);
 for(int n=0;n<8;++n){F::Coefficients step{};step[n]=1.;auto lo=q,hi=q;
  F::applyTrackingDirection(lo,step,-eps,r,c);F::applyTrackingDirection(hi,step,eps,r,c);
  const double numerical=(oracle(hi)-oracle(lo))/(2.*eps);
  assert(std::abs(numerical-gradient[n])<1e-6*std::max(1.,std::abs(numerical)));
 }
 std::cout<<"PASS causal published-rate memory, actual nonuniform intervals, exact rate-change cost, history gaps/future/duplicate/context/stale rejection, all 8 analytic gradient coordinates and invalid-history exact original objective\n";
}
