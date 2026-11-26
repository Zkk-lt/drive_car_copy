#include "control/p5_nominal_propagation.h"
#include "control/p6_controller.h"
#include "control/p7_straight_tracking_reference.h"
#include "control/p7_turn_timing.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace research_control;
using Clock = std::chrono::steady_clock;

static P6Controller controller() {
  P6Controller solver;
  // Exact fixed smooth strategy of the source archive, not a fit.
  solver.setConditionedGeometry(true);
  solver.setBaselineSeed(true);
  solver.setDelayAlignedReconstruction(true);
  solver.setCommandProjection(true);
  solver.setResponseBase(true);
  solver.setResponseNearSupport(true);
  solver.setBoundedFeedback(true);
  solver.setSteeringRateContinuityWeight(.05);
  solver.setTurnAwareRateContinuity(true);
  return solver;
}

static P6Controller::Result solve(const SolveRequest& r, const SolverConfig& config,
                                   double allowance) {
  P6EffectiveParameters parameters;
  parameters.valid=true;
  parameters.solver=config;
  parameters.speed_cap=r.speed_cap;
  parameters.nominal_config_hash="frozen_archive_no_parameter_change";
  parameters.scene="smooth";
  parameters.snapshot_hash=p6SnapshotIdentity(parameters);
  auto fresh=controller(); // no future continuation or previous hypothetical result
  return fresh.solve(r,parameters,"flat_nmpc_slqr",nullptr,0.,nullptr,allowance);
}

static double straighten(const std::vector<ReferencePoint>& spatial,SolveRequest& request) {
  const double blend=P7StraightTrackingReference::condition(spatial,request.reference);
  request.straight_yaw_damping_weight=P7StraightTrackingReference::yawDampingWeight(blend,2.);
  return blend;
}

int main(int argc,char**argv) {
  if(argc!=4)throw std::runtime_error("pass speed reserve, release weight and budget");
  const double reserve=std::stod(argv[1]), release=std::stod(argv[2]), budget=std::stod(argv[3]);
  if(!std::isfinite(reserve)||!std::isfinite(release)||!std::isfinite(budget)||budget<0.1||budget>8.)
    throw std::runtime_error("invalid replay configuration");
  std::ios::sync_with_stdio(false);
  std::cout<<std::setprecision(17)<<"cycle,old_ok,new_ok,old_reason,new_reason,cap_unchanged,"
    "terminal_unchanged,reserve_mps,release_active,release_weight,tightening,opening,"
    "old_speed,new_speed,old_steering,new_steering,old_timeout,new_timeout,"
    "old_validated,new_validated,old_solve_ms,new_solve_ms,old_speed_at_0p6s,new_speed_at_0p6s,"
    "old_steer_at_0p6s,new_steer_at_0p6s\n";

  std::uint64_t cycle;
  std::size_t points,commands;
  std::size_t count=0;
  while(std::cin>>cycle>>points>>commands) {
    if(points<2||points>1024||commands!=256)throw std::runtime_error("invalid fixture dimensions");
    double queued, observed, pu,yu;
    SolveRequest before;
    SolverConfig config;
    auto& m=config.model;
    auto& lim=config.limits;
    auto& w=config.weights;
    auto& s=before.state;
    if(!(std::cin>>queued>>observed>>before.control_dt>>before.speed_cap>>before.target_speed>>pu>>yu
         >>s.x>>s.y>>s.yaw>>s.speed>>s.steering>>s.acceleration
         >>before.previous.speed>>before.previous.steering>>before.previous_command_acceleration
         >>before.baseline.speed>>before.baseline.steering
         >>config.steps>>config.knots>>config.iterations>>config.dt>>config.budget_ms
         >>m.wheelbase>>m.rear_to_center>>m.steering_tau>>m.steering_delay>>m.speed_tau>>m.speed_delay
         >>m.yaw_gain>>m.steering_gain>>m.speed_gain>>m.longitudinal_residual>>m.yaw_residual>>m.lateral_residual
         >>lim.speed>>lim.steering>>lim.steering_rate>>lim.acceleration>>lim.deceleration>>lim.jerk>>lim.lateral_acceleration
         >>w.lateral>>w.heading>>w.speed>>w.smoothness))throw std::runtime_error("truncated fixture scalars");
    std::vector<ReferencePoint> path(points),spatial;
    for(auto& p:path) std::cin>>p.s>>p.x>>p.y>>p.yaw>>p.curvature;
    before.history.resize(commands);
    for(auto& h:before.history)std::cin>>h.stamp>>h.command.speed>>h.command.steering;
    if(!std::cin)throw std::runtime_error("truncated fixture arrays");

    before.now=queued+before.control_dt;
    before.published_history_contiguous=true;
    const auto propagated=p5PropagateNominal(before.state,observed,before.now,m,lim,
                                             before.history,pu,yu,.04,.08,queued);
    FlatnessReferenceModel::recoverMissingSpatialCurvature(path);
    std::string input_reason;
    if(!propagated.valid)input_reason=propagated.reason;
    else if(propagated.position_uncertainty>.03||propagated.yaw_uncertainty>.10)
      input_reason="p6_initialization_uncertainty_limit";
    else if(!FlatnessReferenceModel::projectSpatialSuffix(path,propagated.state,m,spatial))
      input_reason="p6_project_geometry_failed";
    before.state=propagated.state;
    if(input_reason.empty()) {
      before.reference=FlatnessReferenceModel::buildTimeReference(spatial,before.state.speed,
          before.target_speed,before.now,config,false,true,before.state.acceleration,true,true,before.state.yaw);
      if(!before.reference.valid)input_reason=before.reference.reason;
    }
    const auto safety_reference=before.reference;
    const double straight_blend=input_reason.empty()?straighten(spatial,before):0.;
    SolveRequest after=before;
    P7TurnTiming::Result timing;
    if(input_reason.empty()&&straight_blend==0.)
      timing=P7TurnTiming::apply(spatial,config,after,reserve,release);
    const bool same_terminal=before.reference.terminal_approach==after.reference.terminal_approach&&
      before.reference.braking==after.reference.braking&&
      before.reference.endpoint.s==after.reference.endpoint.s&&
      before.reference.endpoint.x==after.reference.endpoint.x&&
      before.reference.endpoint.y==after.reference.endpoint.y&&
      before.reference.endpoint.yaw==after.reference.endpoint.yaw;
    if(before.speed_cap!=after.speed_cap||after.target_speed>before.target_speed||!same_terminal)
      throw std::runtime_error("turn performance objective changed a hard cap or terminal");
    P6Controller::Result lhs,rhs;
    if(input_reason.empty()) {
      // The budget reserve for asynchronous publication, ROS accounting and
      // final controller revalidation is not enlarged by the replay.
      const double allowance=std::min(config.budget_ms,budget)-1.;
      if(cycle%2) {lhs=solve(before,config,allowance);rhs=solve(after,config,allowance);}
      else {rhs=solve(after,config,allowance);lhs=solve(before,config,allowance);}
    } else {
      lhs.solve.reason=input_reason;
      rhs.solve.reason=input_reason;
    }
    SolveRequest check=before;
    check.reference=safety_reference;
    check.straight_yaw_damping_weight=0.;
    const auto validated=[&](const SolveResult& chosen) {
      if(!chosen.ok) return false;
      // Separate feasibility-only validation against original, unshaped
      // geometry. This is not ROS publication revalidation at a future frame.
      const auto test=ReducedNMPC::validateControls(check,config,chosen.controls,
          Clock::now()+std::chrono::milliseconds(50));
      if(!test.feasible)throw std::runtime_error("selected sequence failed original bounds");
      return test.feasible;
    };
    const bool old_checked=validated(lhs.solve),new_checked=validated(rhs.solve);
    const auto value=[&](const SolveResult& picked,bool steering) {
      if(!picked.ok) return std::numeric_limits<double>::quiet_NaN();
      const int at=std::min(picked.count-1,
          std::max(1,1+static_cast<int>(std::ceil((.6-before.control_dt)/config.dt))));
      return steering?picked.predicted[at].steering:picked.predicted[at].speed;
    };
    const auto& a=lhs.solve;const auto& b=rhs.solve;
    std::cout<<cycle<<','<<a.ok<<','<<b.ok<<','<<a.reason<<','<<b.reason<<','
     <<(before.speed_cap==after.speed_cap)<<','<<same_terminal<<','
     <<timing.speed_reserve<<','<<(after.turn_release.weight>0.)<<','<<after.turn_release.weight<<','
     <<timing.tightening<<','<<timing.opening<<','
     <<a.command.speed<<','<<b.command.speed<<','<<a.command.steering<<','<<b.command.steering<<','
     <<a.timed_out<<','<<b.timed_out<<','<<old_checked<<','<<new_checked<<','
     <<a.solve_ms<<','<<b.solve_ms<<','<<value(a,false)<<','<<value(b,false)<<','
     <<value(a,true)<<','<<value(b,true)<<'\n';
    ++count;
  }
  if(!std::cin.eof()||count==0)throw std::runtime_error("empty or malformed fixture stream");
}
