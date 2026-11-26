#include "control/flatness_reference_model.h"
#include "control/reduced_nmpc.h"
#include <cassert>
#include <iostream>
using namespace research_control;
static std::vector<ReferencePoint> route(double spacing,double end,double bend=.70){
  std::vector<ReferencePoint> p;
  for(int i=0;i<=int(std::round(end/spacing));++i){
    ReferencePoint r;r.s=i*spacing;r.x=r.s;
    r.curvature=r.s>=bend-1e-9?.50:0.;r.yaw=.50*std::max(0.,r.s-bend);
    p.push_back(r);
  }
  return p;
}
static std::vector<ReferencePoint> shortCurvatureTransition(){
  // A short centerline transition can exceed its steady-state curvature
  // bound while remaining reachable with a finite transient steering angle.
  const double ds=.01;
  std::vector<ReferencePoint> path;
  ReferencePoint p;
  for(int i=0;i<=180;++i){
    p.s=i*ds;
    p.curvature=p.s>=.55&&p.s<.61?2.0:0.;
    path.push_back(p);
    p.x+=ds*std::cos(p.yaw+.5*ds*p.curvature);
    p.y+=ds*std::sin(p.yaw+.5*ds*p.curvature);
    p.yaw+=ds*p.curvature;
  }
  return path;
}
int main(){
  SolverConfig c;
  const auto original=route(.01,2.);
  const auto old=FlatnessReferenceModel::buildTimeReference(original,.40,.40,10.,c,false,true);
  const auto fine=FlatnessReferenceModel::buildTimeReference(original,.40,.40,10.,c,false,true,0.,true);
  const auto coarse=FlatnessReferenceModel::buildTimeReference(route(.05,2.),.40,.40,10.,c,false,true,0.,true);
  assert(old.valid&&fine.valid&&coarse.valid&&fine.synchronized_preview);
  assert(old.points[12].speed+.03<fine.points[12].speed);
  for(int i=0;i<fine.count;++i)assert(std::abs(fine.points[i].speed-coarse.points[i].speed)<1e-7);
  auto tight=original;tight.back().curvature=.9;
  const auto safe=FlatnessReferenceModel::buildTimeReference(tight,.40,.40,10.,c,false,true,0.,true);
  const auto prior=FlatnessReferenceModel::buildTimeReference(tight,.40,.40,10.,c,false,true);
  assert(safe.valid&&!safe.synchronized_preview&&prior.valid);
  for(int i=0;i<safe.count;++i)assert(safe.points[i].speed==prior.points[i].speed);
  const auto near=FlatnessReferenceModel::buildTimeReference(route(.01,.64),.4,.4,10.,c,false,true,0.,true);
  assert(near.valid&&near.points[8].speed<.38);
  const auto braking=FlatnessReferenceModel::buildTimeReference(original,.20,.40,10.,c,true,true,0.,true);
  assert(braking.valid&&!braking.synchronized_preview&&!braking.speed_preview);
  const auto transition=shortCurvatureTransition();
  const auto prior_transition=FlatnessReferenceModel::buildTimeReference(transition,.40,.40,10.,c,false,true,0.,true);
  const auto aligned=FlatnessReferenceModel::buildTimeReference(transition,.40,.40,10.,c,false,true,0.,true,false,0.);
  assert(prior_transition.valid&&!prior_transition.synchronized_preview);
  assert(aligned.valid&&aligned.synchronized_preview&&aligned.reason=="synthesized_actuator_aligned_preview");
  bool earlier_braking=false, less_restrictive_profile=false;
  for(int i=1;i<aligned.count;++i){
    const auto& a=aligned.points[i-1];const auto& b=aligned.points[i];
    assert(b.s>a.s&&b.speed<=.40+1e-9);
    assert(std::abs(b.acceleration-a.acceleration)<=c.limits.jerk*c.dt+1e-9);
    earlier_braking|=b.s<.45&&b.speed<.38;
    less_restrictive_profile|=b.speed>prior_transition.points[i].speed+.03;
  }
  assert(earlier_braking&&less_restrictive_profile);
  const auto outside=FlatnessReferenceModel::buildTimeReference(transition,.40,.40,10.,c,false,true,0.,true,false,.4);
  assert(outside.valid&&outside.reason==prior_transition.reason);
  for(int i=0;i<outside.count;++i)assert(outside.points[i].speed==prior_transition.points[i].speed);
  const auto straight=FlatnessReferenceModel::buildTimeReference(route(.01,3.,4.),.4,.4,10.,c,false,true,0.,true,false,.08);
  assert(straight.valid&&!std::isfinite(straight.initial_body_yaw));
  for(int i=0;i<straight.count;++i)
    assert(straight.points[i].y==0.&&straight.points[i].yaw==0.&&std::abs(straight.points[i].speed-.40)<1e-9);
  const auto short_bend=route(.01,.64,.20);
  const auto terminal_baseline=FlatnessReferenceModel::buildTimeReference(short_bend,.4,.4,10.,c,false,true,0.,true,true);
  const auto terminal_aligned=FlatnessReferenceModel::buildTimeReference(short_bend,.4,.4,10.,c,false,true,0.,true,true,0.);
  assert(terminal_aligned.valid&&!std::isfinite(terminal_aligned.initial_body_yaw));
  assert(terminal_aligned.reason==terminal_baseline.reason);
  for(int i=0;i<terminal_aligned.count;++i)
    assert(terminal_aligned.points[i].speed==terminal_baseline.points[i].speed);
  // The unmodified hard envelope still rejects an already overspeed state.
  SolveRequest r;r.reference=aligned;r.now=10.;r.state.speed=.40;r.speed_cap=.39;r.previous={.39,0.};r.baseline=r.previous;
  const auto rejected=ReducedNMPC{}.solve(r,c);
  assert(!rejected.ok);
  const double delay=.16;
  c.model.steering_delay=delay;
  // Isolated seed oracle using GCC's access-check override only for this test.
  // Moving steering latency alone advances the issued steer at a transition,
  // while the same speed-clock demand is bit-identical.
  ReducedNMPC early,late;SolveProfile profile;
  r.reference=fine;
  r.speed_cap=.40;r.previous.speed=.40;r.baseline.speed=.40;
  for(int i=0;i<=50;++i)r.history.push_back({8.+i*.04,{.4,0.}});
  assert(early.prepare(r,c,profile));
  c.model.steering_delay=.325;
  assert(late.prepare(r,c,profile));
  bool advanced=false;
  for(int i=0;i<c.steps-3;++i){
    const auto a=early.prepared_.synchronized_demand[i],b=late.prepared_.synchronized_demand[i];
    assert(early.prepared_.synchronized_demand_valid[i]&&late.prepared_.synchronized_demand_valid[i]);
    assert(a.speed==b.speed);
    if(b.steering>a.steering+.025)advanced=true;
  }
  assert(advanced);
  std::cout<<"PASS transient center lift, bounded prebraking, out-of-domain fallback, overspeed guards, distinct activation times\n";
}
