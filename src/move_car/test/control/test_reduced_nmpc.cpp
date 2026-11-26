#include "control/reduced_nmpc.h"
#include <cassert>
#include <iostream>

using namespace research_control;
static SolveRequest request(const SolverConfig& c){
  SolveRequest r;r.now=10.;r.state.speed=.20;r.state.y=.035;
  r.previous.speed=.20;r.baseline.speed=.20;r.baseline.steering=-.08;
  r.speed_cap=.35;r.history.push_back({9.,r.previous});
  std::vector<ReferencePoint> path;
  for(int i=0;i<100;++i){ReferencePoint p;p.s=.02*i;p.x=c.model.rear_to_center+p.s;path.push_back(p);}
  r.reference=FlatnessReferenceModel::buildTimeReference(path,.20,.25,r.now,c);
  return r;
}
static void assertDemandConstraints(const SolveRequest& r,const SolverConfig& c,const SolveResult& result){
  assert(result.ok&&result.feasible);assert(result.max_violation<=1e-7);
  Command last=r.previous;double a_last=r.previous_command_acceleration;
  for(int i=0;i<c.steps;++i){
    const double h=i==0?r.control_dt:c.dt;const auto u=result.controls[i];
    const double a=(u.speed-last.speed)/h,jerk=(a-a_last)/h;
    assert(u.speed>=-1e-9&&u.speed<=std::min(c.limits.speed,r.speed_cap)+1e-9);
    assert(std::abs(u.steering)<=c.limits.steering+1e-9);
    assert(std::abs(u.steering-last.steering)<=c.limits.steering_rate*h+1e-9);
    assert(a<=c.limits.acceleration+1e-9&&a>=-c.limits.deceleration-1e-9);
    assert(std::abs(jerk)<=c.limits.jerk+1e-9);
    const auto s=result.predicted[i+1];
    const double yaw=c.model.yaw_gain*s.speed*std::tan(s.steering)/c.model.wheelbase+c.model.yaw_residual;
    assert(std::abs(s.speed*yaw)<=c.limits.lateral_acceleration+1e-7);
    assert(s.acceleration<=c.limits.acceleration+1e-7&&s.acceleration>=-c.limits.deceleration-1e-7);
    last=u;a_last=a;
  }
}
int main(){
  SolverConfig c;c.budget_ms=1000.;
  { // Exact forward circular jet, nominal no-slip reconstruction.
    FlatOutputJet q;q.x=2.;q.dy=.4;q.ddx=-.08;q.dddy=-.016;
    const auto f=FlatnessReferenceModel::reconstruct(q,c.model);
    assert(f.valid);assert(std::abs(f.state.speed-.4)<1e-12);
    assert(std::abs(f.curvature-.5)<1e-12);
    assert(std::abs(f.state.steering-std::atan(.3))<1e-12);
    assert(std::abs(f.state.yaw-M_PI/2.)<1e-12);
    assert(std::abs(f.steering_rate)<1e-12);assert(std::abs(f.jerk)<1e-12);
    assert(!FlatnessReferenceModel::reconstruct(FlatOutputJet{},c.model).valid);
    q.dx=missing();assert(!FlatnessReferenceModel::reconstruct(q,c.model).valid);
  }
  auto r=request(c);assert(r.reference.valid);
  {
    std::vector<ReferencePoint> short_path(2);short_path[1].s=short_path[1].x=.01;
    const auto short_reference=FlatnessReferenceModel::buildTimeReference(short_path,.3,.3,10.,c);
    assert(!short_reference.valid&&short_reference.reason=="reference_horizon_exhausted_while_moving");
  }
  ReducedNMPC solver;auto solution=solver.solve(r,c);
  if(!solution.ok)std::cerr<<solution.reason<<" violation "<<solution.max_violation<<"\n";
  assertDemandConstraints(r,c,solution);
  assert(solution.decision_dimension==12);assert(solution.evaluations>3);
  if(std::isfinite(solution.initial_objective))assert(solution.objective<=solution.initial_objective+1e-12);
  assert(solution.command.steering<0.); // correct direction for left offset
  {
    auto bad=r;bad.reference.valid=false;assert(solver.solve(bad,c).reason=="reference_invalid");
    bad=r;bad.state.speed=missing();assert(solver.solve(bad,c).reason=="invalid_state_or_request");
    bad=r;bad.history.clear();assert(solver.solve(bad,c).reason=="history_incomplete");
    bad=r;bad.history[0].stamp=11.;assert(solver.solve(bad,c).reason=="invalid_command_history");
    bad=r;bad.history.push_back({8.,bad.previous});assert(solver.solve(bad,c).reason=="invalid_command_history");
    bad=r;bad.force_timeout=true;assert(solver.solve(bad,c).timed_out);
    bad=r;bad.force_numerical_failure=true;assert(!solver.solve(bad,c).ok);
    bad=r;bad.force_infeasible=true;assert(!solver.solve(bad,c).ok);
    auto invalid=c;invalid.knots=0;assert(solver.solve(r,invalid).reason=="invalid_config");
    invalid=c;invalid.model.speed_tau=missing();assert(solver.solve(r,invalid).reason=="invalid_config");
    invalid=c;invalid.budget_ms=.000001;assert(solver.solve(r,invalid).timed_out);
  }
  { // Feasible chart cannot conceal a physically infeasible initial condition.
    auto bad=r;bad.state.speed=c.limits.speed+.1;assert(!solver.solve(bad,c).ok);
    bad=r;bad.previous_command_acceleration=20.;assert(!solver.solve(bad,c).ok);
    auto tight=c;tight.limits.lateral_acceleration=.00001;
    bad=r;bad.state.steering=.3;assert(!solver.solve(bad,tight).ok);
  }
  { // Weights genuinely alter optimization in BOTH channels.
    auto fast=c;fast.weights.speed=300.;fast.weights.smoothness=.1;
    auto gentle=c;gentle.weights.speed=.01;gentle.weights.smoothness=100.;
    ReducedNMPC a,b;const auto x=a.solve(r,fast),y=b.solve(r,gentle);
    assertDemandConstraints(r,fast,x);assertDemandConstraints(r,gentle,y);
    double change=0.;for(int i=0;i<c.steps;++i)change+=std::abs(x.controls[i].speed-y.controls[i].speed);
    assert(change>1e-4);
    auto lateral=c;lateral.weights.lateral=300.;lateral.weights.heading=40.;lateral.weights.smoothness=.1;
    ReducedNMPC d;const auto z=d.solve(r,lateral);assertDemandConstraints(r,lateral,z);
    change=0.;for(int i=0;i<c.steps;++i)change+=std::abs(z.controls[i].steering-y.controls[i].steering);
    assert(change>1e-4);
  }
  { // Delays shorter than 33 ms are split inside the first predicted interval.
    auto short_delay=c;short_delay.model.speed_delay=.015;short_delay.model.steering_delay=.015;
    ReducedNMPC a;const auto x=a.solve(r,short_delay);assertDemandConstraints(r,short_delay,x);
    assert(std::abs(x.predicted[1].steering)>1e-7);
    assert(std::abs(solution.predicted[1].steering)<1e-12);
  }
  { // Closed first-action constraints use control_dt, not the longer horizon dt.
    const double a=(solution.command.speed-r.previous.speed)/r.control_dt;
    assert(std::abs(a-r.previous_command_acceleration)<=c.limits.jerk*r.control_dt+1e-10);
    assert(std::abs(solution.command.steering-r.previous.steering)<=c.limits.steering_rate*r.control_dt+1e-10);
  }
  { // Low-speed nominal flatness singularity does not disable NMPC itself.
    auto stopped=r;stopped.state={};stopped.previous={};stopped.baseline={.2,.03};
    stopped.history={{9.,{}}};
    ReducedNMPC a;auto x=a.solve(stopped,c);assertDemandConstraints(stopped,c,x);
  }
  { // Warm policies are available only while physical time and horizon agree.
    ReducedNMPC warm;
    assert(!warm.solve(r,c).warm_available);
    auto advanced=r;advanced.now+=.10;
    assert(warm.solve(advanced,c).warm_available); // a skipped 30 Hz callback
    advanced.now+=c.steps*c.dt+1.;
    assert(!warm.solve(advanced,c).warm_available);
    auto changed=c;changed.dt=.08;
    assert(!warm.solve(advanced,changed).warm_available);
    advanced.now-=.1;
    assert(!warm.solve(advanced,changed).warm_available); // clock rewind
    warm.reset();assert(!warm.solve(r,c).warm_available);
  }
  { // Profiling reports the operations actually executed, with no derivative fiction.
    auto profiled=c;profiled.profile=true;ReducedNMPC diagnostic;
    const auto x=diagnostic.solve(r,profiled);assertDemandConstraints(r,profiled,x);
    assert(x.profile.derivative_ms==0.&&x.profile.line_search_ms==0.);
    assert(x.profile.initialization_ms>0.&&x.profile.reference_ms>0.);
    assert(x.profile.prediction_ms>0.&&x.profile.constraint_cost_ms>0.);
    const auto& p=x.profile;
    const double sum=p.initialization_ms+p.reference_ms+p.prediction_ms+
      p.constraint_cost_ms+p.direction_ms+p.candidate_selection_ms+p.other_ms;
    assert(std::abs(sum-x.solve_ms)<1e-8);
  }
  { // Reserve remaining budget for a previously completed feasible trajectory.
    // Scale the test budget to measured host speed; never accept an expired result.
    auto bounded=c;bounded.iterations=30;ReducedNMPC probe;
    const auto full=probe.solve(r,bounded);assert(full.ok);
    bounded.budget_ms=std::max(.10,std::min(8.,full.solve_ms*.30));
    bool retained=false;
    for(int attempt=0;attempt<8&&!retained;++attempt){
      ReducedNMPC budgeted;const auto x=budgeted.solve(r,bounded);
      if(x.ok){assert(x.solve_ms<bounded.budget_ms);assertDemandConstraints(r,bounded,x);}
      if(x.timed_out)assert(!x.ok);
      retained=x.ok&&x.budget_limited&&x.reason=="feasible_budget_stop";
    }
    assert(retained);
  }
  solver.reset();assertDemandConstraints(r,c,solver.solve(r,c));
  std::cout<<"test_reduced_nmpc: flat reconstruction, constraints, weights, faults, delays, low speed, warm validity, phase clocks, feasible budget stop PASS\n";
}
