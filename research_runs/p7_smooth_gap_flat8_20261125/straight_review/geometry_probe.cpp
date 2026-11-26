#include "control/flatness_reference_model.h"
#include "control/p7_straight_tracking_reference.h"
#include <cstdint>
#include <iomanip>
#include <iostream>

// One frozen request + archived predicted initial state/snapshot per line.
int main() {
  using namespace research_control;
  std::ios::sync_with_stdio(false);
  std::uint64_t cycle;
  std::size_t count;
  std::cout << std::setprecision(12);
  while (std::cin >> cycle >> count) {
    SolverConfig c;
    State s;
    double speed;
    bool braking;
    std::cin >> speed >> braking
      >> s.x >> s.y >> s.yaw >> s.speed >> s.steering >> s.acceleration
      >> c.steps >> c.dt
      >> c.model.wheelbase >> c.model.rear_to_center >> c.model.steering_tau
      >> c.model.steering_delay >> c.model.speed_tau >> c.model.speed_delay
      >> c.model.yaw_gain >> c.model.steering_gain >> c.model.speed_gain
      >> c.model.longitudinal_residual >> c.model.yaw_residual >> c.model.lateral_residual
      >> c.limits.speed >> c.limits.steering >> c.limits.steering_rate
      >> c.limits.acceleration >> c.limits.deceleration >> c.limits.jerk >> c.limits.lateral_acceleration;
    std::vector<ReferencePoint> spatial(count), suffix;
    for (auto& p : spatial) std::cin >> p.s >> p.x >> p.y >> p.yaw >> p.curvature;
    if (!std::cin) return 2;
    FlatnessReferenceModel::recoverMissingSpatialCurvature(spatial);
    if (!FlatnessReferenceModel::projectSpatialSuffix(spatial,s,c.model,suffix)) {
      std::cout << cycle << ",0,0,0,0,0,0\n";
      continue;
    }
    auto ref = FlatnessReferenceModel::buildTimeReference(
        suffix,s.speed,speed,1.,c,braking,true,s.acceleration,true,true,s.yaw);
    const auto original=ref;
    const double blend=P7StraightTrackingReference::condition(suffix,ref);
    double max_shift=0., max_yaw_delta=0.;
    for (int i=0; i<ref.count; ++i) {
      const auto &a=ref.points[i],&b=original.points[i];
      if (a.s!=b.s || a.t!=b.t || a.speed!=b.speed || a.acceleration!=b.acceleration ||
          ref.endpoint.x!=original.endpoint.x || ref.endpoint.y!=original.endpoint.y ||
          ref.terminal_approach!=original.terminal_approach) return 3;
      max_shift=std::max(max_shift,std::hypot(a.x-b.x,a.y-b.y));
      max_yaw_delta=std::max(max_yaw_delta,std::abs(wrap(a.yaw-b.yaw)));
    }
    std::cout << cycle << ',' << ref.valid << ',' << ref.terminal_approach << ','
              << blend << ',' << max_shift << ',' << max_yaw_delta << ','
              << (suffix.back().s-suffix.front().s) << '\n';
  }
}
