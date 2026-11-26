# P3.5 additive diagnostics; original research messages keep their MD5.
std_msgs/Header header
string run_id
string config_hash
string ground_twist_convention
string ground_frame_status
string imu_frame_status
bool source_timestamps_verified
bool steering_delay_verified
string wheel_freshness_status
string source_time_status
# RLS information is computed only from accepted updates. Rejected innovations
# have separate counters; a small accepted-fit RMS does not erase their events.
uint32 yaw_identification_samples
uint32 yaw_identification_rejected
float64 yaw_identification_information
float64 yaw_regressor_variance
float64 yaw_innovation_rms
string yaw_observability_reason
uint32 steering_identification_samples
uint32 steering_identification_rejected
float64 steering_regressor_variance
float64 steering_innovation_rms
string steering_observability_reason
bool budget_limited
bool early_stopped
bool warm_available
bool model_adaptation_active
bool weight_adaptation_active
float64 adaptation_risk
string adaptation_reason
string[] evidence_names
int32[] evidence_samples
float64[] nominal_mse
float64[] candidate_mse
bool[] adaptation_active
string[] adaptation_reasons
# Profiling is optional and consumes time itself; zero does not mean free work.
bool detailed_profiling_enabled
float64 initialization_ms
float64 reference_ms
float64 prediction_ms
float64 derivative_ms
float64 constraint_cost_ms
float64 direction_ms
float64 candidate_selection_ms
float64 other_ms
