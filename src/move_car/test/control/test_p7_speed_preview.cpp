#include "control/p6_flatness_nmpc.h"
#include <cassert>
#include <iostream>
using namespace research_control;
static std::vector<ReferencePoint> path(double end,double start_bend,double kappa){
  std::vector<ReferencePoint> x;
  ReferencePoint p;
  for(int i=0;i<=int(std::round(end/.02));++i){
    p.s=.02*i;p.curvature=p.s>=start_bend-1e-9?kappa:0.;
    x.push_back(p);p.x+=.02*std::cos(p.yaw+.01*p.curvature);
    p.y+=.02*std::sin(p.yaw+.01*p.curvature);p.yaw+=.02*p.curvature;
  }
  return x;
}
int main(){
  SolverConfig c;c.limits.lateral_acceleration=.25;
  const auto straight=path(2.,10.,0.);
  const auto bend=path(1.5,.46,.50);
  const auto finite=path(.64,10.,0.);
  const auto old=FlatnessReferenceModel::buildTimeReference(bend,.4,.4,10.,c);
  const auto pre=FlatnessReferenceModel::buildTimeReference(bend,.4,.4,10.,c,false,true);
  const auto clear=FlatnessReferenceModel::buildTimeReference(straight,.4,.4,10.,c,false,true);
  const auto near=FlatnessReferenceModel::buildTimeReference(finite,.4,.4,10.,c,false,true);
  assert(old.valid&&pre.valid&&clear.valid&&near.valid);
  assert(pre.speed_preview&&!old.speed_preview&&pre.points[0].speed==old.points[0].speed);
  bool slows_in_advance=false;
  for(int i=1;i<pre.count;++i){
    const auto& a=pre.points[i-1];const auto& b=pre.points[i];
    assert(b.s>=a.s&&b.speed>=0.&&b.speed<=.4+1e-9);
    assert(std::abs(b.acceleration-a.acceleration)<=c.limits.jerk*c.dt+1e-9);
    if(b.s<.44&&b.speed<.39)slows_in_advance=true;
    assert(std::abs(clear.points[i].speed-.4)<1e-9);
  }
  assert(slows_in_advance&&near.points[8].speed<.38&&near.points[near.count-1].s<finite.back().s);
  // Continuous curvature can still trigger a curvature acceleration preview.
  auto curv=path(2.,.6,0.);
  for(auto& p:curv)p.curvature=clamp((p.s-.4)*4.,0.,2.);
  const auto high=FlatnessReferenceModel::buildTimeReference(curv,.4,.4,10.,c,false,true);
  assert(high.valid&&high.points[10].speed<.398);
  const auto stop0=FlatnessReferenceModel::buildTimeReference(straight,.2,.4,10.,c,true,false);
  const auto stop1=FlatnessReferenceModel::buildTimeReference(straight,.2,.4,10.,c,true,true,.12);
  assert(stop0.valid&&stop1.valid&&!stop1.speed_preview);
  for(int i=0;i<stop0.count;++i)assert(stop0.points[i].speed==stop1.points[i].speed);
  auto bad=straight;bad[1].s=bad[0].s;
  assert(!FlatnessReferenceModel::buildTimeReference(bad,.4,.4,10.,c,false,true).valid);
  SolveRequest r;r.reference=pre;r.now=10.;r.state.speed=.4;r.speed_cap=.39;r.previous={.39,0.};r.baseline=r.previous;
  P6FlatnessNMPC solver;
  const auto rejected=solver.solve(r,c);
  assert(!rejected.ok&&rejected.reason=="flatness_initial_speed_above_dynamic_cap");
  std::cout<<"PASS straight/corner/curvature-rate/end preview; flags, jerk, emergency compatibility and hard overspeed guard\n";
}
