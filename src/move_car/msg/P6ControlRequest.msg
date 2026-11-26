uint64 schema_version
uint64 observation_steady_ns
uint64 request_steady_ns
move_car/P5ControlRequest base
string session_id
string algorithm_mode
string computation_mode
bool synthetic
bool imu_frame_verified
move_car/P6RawMeasurement wheel_speed
move_car/P6RawMeasurement steering
move_car/P6RawMeasurement yaw_rate
move_car/P6RawMeasurement accel_x
move_car/P6RawMeasurement accel_y
move_car/P6RawMeasurement accel_z
# Optional independent motion source: absent/invalid is the normal local mode.
move_car/P6RawMeasurement ground_speed
bool ground_frame_verified
bool ground_lever_arm_verified
# Schema 3 optional local speed policy. base.speed_cap remains the hard safety
# envelope; this objective does not claim a measured planner time law.
bool speed_target_separated
float64 target_speed
string speed_policy_reason
