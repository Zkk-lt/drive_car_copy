#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
namespace research_control {
inline double clamp(double x,double lo,double hi){return std::max(lo,std::min(x,hi));}
inline double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
inline double missing(){return std::numeric_limits<double>::quiet_NaN();}
constexpr int kMaxHorizon=40, kMaxKnots=10;
struct Command { double speed=0.,steering=0.; };
struct CommandSample { double stamp=0.; Command command; };
// Rear axle pose, signed rear speed, actual steering and longitudinal acceleration.
struct State { double x=0.,y=0.,yaw=0.,speed=0.,steering=0.,acceleration=0.; };
struct ReferencePoint {
 double t=0.,s=0.,x=0.,y=0.,yaw=0.,curvature=0.,speed=0.,acceleration=0.;
};
struct Reference {
 std::array<ReferencePoint,kMaxHorizon+1> points{};
 int count=0; double stamp=0.; bool valid=false,braking=false,speed_preview=false,synchronized_preview=false;
 // Geometric endpoint of a valid finite suffix, in this exact local frame.
 // Set only when the terminal strategy and its domain checks are active.
 bool terminal_approach=false;
 ReferencePoint endpoint;
 // Body center reference; yaw is tangent direction, NOT body yaw.
 std::string reason="uninitialized";
 // Causal body heading used by the optional transient centerline lift.
 // The regular geometry-only schedule has no measured body anchor.
 double initial_body_yaw=missing();
};
struct Model {
 double wheelbase=.60,rear_to_center=.30,steering_tau=.44,steering_delay=.325;
 double speed_tau=.20,speed_delay=.20,yaw_gain=1.;
 // Input-to-equilibrium gains. One preserves the frozen P5 model exactly.
 // These are response parameters, not tire friction or longitudinal slip.
 double steering_gain=1.,speed_gain=1.;
 double longitudinal_residual=0.,yaw_residual=0.,lateral_residual=0.;
};
struct Limits {
 double speed=.50,steering=.32,steering_rate=1.,acceleration=.28,deceleration=.36,jerk=.80;
 double lateral_acceleration=.25; // Conservative engineering bound, not estimated mu.
};
struct Weights { double lateral=26.,heading=7.,speed=6.,smoothness=24.; };
struct SolverConfig {
 int steps=20,knots=6,iterations=3;
 double dt=.10,budget_ms=8.; Model model; Limits limits; Weights weights;
 // Diagnostic phase clocks are disabled in the normal control path.
 bool profile=false;
 // Offline research only. 0 preserves the original index-uniform arithmetic;
 // 1 front-loaded time grid, 2 continuous curvature density, 3 ordinary holds.
 int research_grid=0;
 bool research_consistent_flat_seed=false;
 double yaw_rate_tracking_weight=0.;
};
struct SolveRequest {
 struct TurnRelease {
  double weight=0., start_s=0.;
  std::array<double,5> station{},curvature{};
 };
 State state; Reference reference; Command previous,baseline;
 double previous_command_acceleration=0.,control_dt=1./30.;
 double now=0.,speed_cap=.30;
 // Nominal speed objective, separate from the immutable safety speed_cap.
 // NaN preserves the legacy speed-cap-as-target behavior at every caller.
 double target_speed=missing();
 // Performance term for a verified near-straight reference. Zero everywhere
 // else. Uses the delayed model's body yaw rate, not a noisy tangent derivative.
 double straight_yaw_damping_weight=0.;
 double supported_course_yaw_weight=0.;
 TurnRelease turn_release;
 bool published_history_contiguous=false;
 std::vector<CommandSample> history;
 bool force_timeout=false,force_infeasible=false,force_numerical_failure=false;
 // Explicit initialization hint, never applied without the normal rollout.
 int warm_hint_count=0;
 std::array<Command,kMaxHorizon> warm_hint{};
};
// Exclusive elapsed-time buckets; derivative/line search stay zero because the
// derivative-free pattern search performs neither operation. Diagnostic clocks
// themselves add overhead: never compare instrumented timings to plain timings.
struct SolveProfile {
 double initialization_ms=0.,reference_ms=0.,prediction_ms=0.,constraint_cost_ms=0.;
 double direction_ms=0.,candidate_selection_ms=0.,other_ms=0.;
 double derivative_ms=0.,line_search_ms=0.;
};
struct SolveResult {
 bool ok=false,feasible=false,timed_out=false,warm_started=false;
 bool budget_limited=false,early_stopped=false,warm_available=false;
 SolveProfile profile;
 Command command; double command_acceleration=0.,objective=missing(),initial_objective=missing();
 double solve_ms=0.,max_violation=missing();
 int iterations=0,evaluations=0,decision_dimension=0;
 std::string reason="not_run";
 std::array<State,kMaxHorizon+1> predicted{};
 std::array<Command,kMaxHorizon> controls{};
 int count=0;
};
struct Estimate {
 double value=missing(),confidence=0.,stamp=0.; bool valid=false;
 std::string reason="unobserved";
};
struct Observation {
 Estimate speed,slip,yaw_rate,yaw_reference,yaw_residual,steering_tau,steering_bias;
 Estimate sideslip,roughness,yaw_gain,longitudinal_residual,lateral_residual;
 Estimate steering; double stamp=0.;
};
struct TerrainEstimate {
 // smooth, grass_bumpy, asphalt, unknown. Probabilities are model scores until calibrated.
 std::array<double,4> probabilities{{0.,0.,0.,1.}};
 std::string label="unknown",reason="unobserved";
 double confidence=0.,stamp=0.; bool valid=false;
};
}
