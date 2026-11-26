#include "control/reduced_nmpc.h"
#include <cassert>
#include <chrono>
#include <iostream>
using namespace research_control;
int main(){
  SolverConfig c;c.budget_ms=8.;
  SolveRequest r;r.now=10.;r.state={-.3,0.,0.,.4,0.,0.};r.previous=r.baseline={.4,0.};r.speed_cap=.4;
  r.published_history_contiguous=true;
  for(int i=1;i<=20;++i)r.history.push_back({9.+i*.05,{.4,0.}});
  std::vector<ReferencePoint> spatial;
  for(int i=0;i<=32;++i){ReferencePoint p;p.s=p.x=i*.025;spatial.push_back(p);}
  r.reference=FlatnessReferenceModel::buildTimeReference(spatial,.4,.4,r.now,c,false,true,0.,true,true);
  assert(r.reference.valid&&r.reference.terminal_approach);
  ReducedNMPC solver;SolveProfile profile;
  assert(solver.prepare(r,c,profile));
  std::array<Command,kMaxHorizon> steer{},issued{};for(auto& u:steer)u={.4,0.};
  assert(ReducedNMPC::terminalBrakeSeed(r,c,solver.prepared_,steer,1.,issued));
  auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(100);
  auto pass=ReducedNMPC::validateControls(r,c,issued,deadline);
  assert(pass.feasible&&pass.result.predicted[c.steps].speed<.37&&issued.front().speed<.4);
  const auto old=issued;
  // Stronger than a horizon-only check: a cruise trajectory that would
  // cross the geometric endpoint is rejected independently of the cost.
  r.reference.endpoint.x=.57;
  const auto fail=ReducedNMPC::validateControls(r,c,steer,deadline);
  assert(!fail.feasible&&std::string(fail.reason)=="terminal_prediction_past_reference");
  r.reference.endpoint.x=.80;
  r.published_history_contiguous=false;
  assert(!ReducedNMPC::terminalBrakeSeed(r,c,solver.prepared_,steer,1.,issued));
  r.published_history_contiguous=true;
  r.history.back().command.speed=.35;
  assert(!ReducedNMPC::terminalBrakeSeed(r,c,solver.prepared_,steer,1.,issued));
  r.history.back().command.speed=.4;
  // Old committed higher speed is carried through the model dead time.
  for(auto& h:r.history)if(h.stamp<r.now-.08)h.command.speed=.5;
  assert(solver.prepare(r,c,profile));
  assert(ReducedNMPC::terminalBrakeSeed(r,c,solver.prepared_,steer,1.,issued));
  assert(issued[6].speed<old[6].speed);
  // No endpoint logic in a genuine braking reference.
  const auto stop=FlatnessReferenceModel::buildTimeReference(spatial,.4,.4,r.now,c,true,true,0.,true,true);
  assert(!stop.terminal_approach);
  // Folded geometry does not turn into a false half-plane.
  auto folded=spatial;folded[20].x=-.3;
  assert(!FlatnessReferenceModel::buildTimeReference(folded,.4,.4,r.now,c,false,true,0.,true,true).terminal_approach);
  std::cout<<"PASS terminal brake seed, delayed published history, endpoint validation, folded/reference-stop guards\n";
}
