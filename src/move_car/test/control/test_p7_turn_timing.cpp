#include "control/p7_turn_timing.h"
#include <cassert>
#include <iostream>

using namespace research_control;

template<class Curvature>
static std::vector<ReferencePoint> makePath(Curvature curvature) {
  std::vector<ReferencePoint> out;
  ReferencePoint p;
  for(int i=0;i<=125;++i){
    p.s=i*.02;
    p.curvature=curvature(p.s);
    out.push_back(p);
    p.x+=.02*std::cos(p.yaw+.01*p.curvature);
    p.y+=.02*std::sin(p.yaw+.01*p.curvature);
    p.yaw+=.02*p.curvature;
  }
  return out;
}

static std::vector<ReferencePoint> path(double first,double last,double side) {
  return makePath([=](double s){return side*(first+(last-first)*clamp((s-.12)/.78,0.,1.));});
}

static SolveRequest makeRequest(const std::vector<ReferencePoint>& spatial,const SolverConfig& cfg,double steer) {
  SolveRequest q;
  q.now=100.;q.speed_cap=.38;q.target_speed=.37;q.state.speed=.32;q.state.steering=steer;
  q.published_history_contiguous=true;
  q.reference=FlatnessReferenceModel::buildTimeReference(spatial,q.state.speed,q.target_speed,
      q.now,cfg,false,true,0.,true,true,q.state.yaw);
  assert(q.reference.valid);
  return q;
}

int main() {
  SolverConfig cfg;cfg.limits.speed=.40;
  double right_target=0.,right_weight=0.,right_yaw=0.;
  for(double side:{-1.,1.}){
    auto tight=path(.16,.56,side);auto in=makeRequest(tight,cfg,.14*side);
    const auto before=in;
    auto a=P7TurnTiming::apply(tight,cfg,in,.035,2.);
    assert(a.tightening&&!a.opening&&a.speed_reserve>0.&&a.speed_reserve<=.035);
    assert(in.speed_cap==before.speed_cap&&in.target_speed<before.target_speed);
    assert(in.reference.valid&&in.reference.stamp==before.reference.stamp);
    assert(in.reference.points.front().x==before.reference.points.front().x);
    assert(in.reference.points.front().y==before.reference.points.front().y);
    assert(in.reference.points.back().s==before.reference.points.back().s);
    assert(in.reference.points[in.reference.count-1].speed<before.reference.points[before.reference.count-1].speed);
    if(side<0.)right_target=in.target_speed;else assert(std::abs(right_target-in.target_speed)<1e-10);

    // The archived approach had a short opening between two same-side arcs.
    auto compound=[=](double direction){return makePath([=](double s){
      return side*.33*(1.-clamp((s-.22)/.42,0.,1.))+
             direction*.65*clamp((s-.85)/.36,0.,1.);
    });};
    auto long_curve=compound(side);
    auto early=makeRequest(long_curve,cfg,.19*side);
    assert(P7TurnTiming::apply(long_curve,cfg,early,.035,0.).tightening);
    long_curve.resize(69); // short of the far-chord support
    early=makeRequest(long_curve,cfg,.19*side);
    assert(!P7TurnTiming::apply(long_curve,cfg,early,.035,0.).tightening);
    const auto reverse=compound(-side);
    early=makeRequest(reverse,cfg,.19*side);
    assert(!P7TurnTiming::apply(reverse,cfg,early,.035,0.).tightening);

    auto open=path(.51,.11,side);auto out=makeRequest(open,cfg,.30*side);
    auto b=P7TurnTiming::apply(open,cfg,out,.035,2.);
    assert(b.opening&&!b.tightening&&b.release_weight>0.&&b.release_weight<=2.);
    const auto near=ReferencePoint{0.,.25,0.,0.,0.,0.,.3,0.};
    const auto far=ReferencePoint{0.,.75,0.,0.,0.,0.,.3,0.};
    const double n=P7TurnTiming::releaseYawRate(out.turn_release,near);
    const double f=P7TurnTiming::releaseYawRate(out.turn_release,far);
    assert(side*n>side*f&&side*f>0.);
    if(side<0.){right_weight=b.release_weight;right_yaw=f;}
    else assert(std::abs(right_weight-b.release_weight)<1e-10&&std::abs(right_yaw+f)<1e-10);
    auto weak=makeRequest(open,cfg,.06*side);
    assert(!P7TurnTiming::apply(open,cfg,weak,.035,2.).opening);
  }
  for(double k:{0.,.30,-.30}){
    auto steady=path(k,k,1.);auto request=makeRequest(steady,cfg,.25);
    const double target=request.target_speed;
    auto result=P7TurnTiming::apply(steady,cfg,request,.035,2.);
    assert(!result.tightening&&!result.opening&&request.target_speed==target&&request.turn_release.weight==0.);
  }
  auto steady=path(.16,.56,1.);
  auto terminal=makeRequest(steady,cfg,.14);
  terminal.reference.terminal_approach=true;terminal.reference.endpoint.x=3.;
  assert(!P7TurnTiming::apply(steady,cfg,terminal,.035,2.).tightening&&terminal.reference.endpoint.x==3.);
  terminal.reference.terminal_approach=false;terminal.published_history_contiguous=false;
  assert(!P7TurnTiming::apply(steady,cfg,terminal,.035,2.).tightening);
  assert(!P7TurnTiming::apply(steady,cfg,terminal,0.,0.).tightening);
  std::cout<<"PASS symmetric turn timing, protected geometry and hard limits, straight and steady bypasses\n";
}
