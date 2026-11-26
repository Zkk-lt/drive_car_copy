# Diagnostic-only, never a chassis command interface. SI unless named _ms.
std_msgs/Header header
uint32 schema_version
string run_id
time reference_stamp
bool tracking_valid
float64 lateral_error
float64 heading_error
float64 measured_speed_fused
float64 target_speed
float64 command_speed
float64 command_steering
float64 command_acceleration
float64 command_jerk
float64 steering_feedback_filtered
float64 predicted_steering
float64 yaw_rate_error
bool solver_attempted
bool solver_ok
uint32 iterations
float64 objective
float64 slqr_steering
float64 solve_ms
float64 compute_ms
# Excludes this telemetry serialization/publication and callback dispatch wait.
float64 callback_before_telemetry_ms
float64 timer_lateness_ms
bool steering_rate_limited
bool steering_saturated
bool safe_stop
bool collision_stop
bool braking_reference
string mode
string stop_reason
string fallback_reason
string speed_cap_reason
# P1: nmpc_active always false; no NMPC estimates fabricated.
bool nmpc_active
