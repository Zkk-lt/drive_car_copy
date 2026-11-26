#pragma once
#include "adaptive_runtime.h"
#include <ros/ros.h>
#include <stdexcept>

namespace research_control {
template<class T> void required(const ros::NodeHandle& nh,const std::string& key,T& v) {
 if(!nh.getParam(key,v))throw std::runtime_error("missing or mistyped runtime parameter: "+key);
}
inline RuntimeConfig loadRuntime(const ros::NodeHandle& root,std::string& mode,std::string& terrain,std::string& hash) {
 ros::NodeHandle n(root,"research_runtime");int schema=0;std::string deployment;
 required(n,"schema_version",schema);required(n,"deployment",deployment);
 if(schema!=1||deployment!="shadow_only")throw std::runtime_error("only schema 1 shadow_only is supported");
 required(n,"mode",mode);required(n,"terrain",terrain);required(n,"config_hash",hash);
 if(hash.size()!=64||hash.find_first_not_of("0123456789abcdef")!=std::string::npos)
   throw std::runtime_error("invalid runtime snapshot hash");
 RuntimeConfig c;ros::NodeHandle v(n,"values");
#define RC_VALUE(key,member) required(v,key,c.member)
 RC_VALUE("wheelbase",solver.model.wheelbase);RC_VALUE("rear_to_center",solver.model.rear_to_center);
 RC_VALUE("steering_tau",solver.model.steering_tau);RC_VALUE("steering_delay",solver.model.steering_delay);
 RC_VALUE("yaw_gain",solver.model.yaw_gain);RC_VALUE("prediction_dt",solver.dt);
 RC_VALUE("horizon_steps",solver.steps);RC_VALUE("control_knots",solver.knots);
 RC_VALUE("solver_max_iterations",solver.iterations);RC_VALUE("solver_budget_ms",solver.budget_ms);
 RC_VALUE("weight_lateral",solver.weights.lateral);RC_VALUE("weight_heading",solver.weights.heading);
 RC_VALUE("weight_speed",solver.weights.speed);RC_VALUE("weight_smoothness",solver.weights.smoothness);
 RC_VALUE("max_speed",solver.limits.speed);RC_VALUE("solver_max_steer",solver.limits.steering);
 RC_VALUE("max_steer_rate",solver.limits.steering_rate);RC_VALUE("max_accel",solver.limits.acceleration);
 RC_VALUE("max_decel",solver.limits.deceleration);RC_VALUE("max_jerk",solver.limits.jerk);
 RC_VALUE("filter_tau",filter_tau);RC_VALUE("min_confidence",minimum_confidence);
 RC_VALUE("switch_dwell",switch_dwell);RC_VALUE("switch_hysteresis",switch_hysteresis);
 RC_VALUE("yaw_gain_max_change",yaw_gain_max_change);RC_VALUE("tau_max_change",tau_max_change);
#undef RC_VALUE
 for(int i=0;i<4;++i){ros::NodeHandle p(n,"profiles/"+AdaptiveRuntime::name(i));auto& t=c.profiles[i];
   required(p,"speed_cap",t.speed_cap);required(p,"lateral_weight_scale",t.lateral_scale);
   required(p,"heading_weight_scale",t.heading_scale);required(p,"speed_weight_scale",t.speed_scale);
   required(p,"smoothness_weight_scale",t.smoothness_scale);}
 ros::NodeHandle s(n,"sensors");
#define RC_SENSOR(key) required(s,#key,c.observer.key)
 RC_SENSOR(wheel_scale);RC_SENSOR(wheel_sign);RC_SENSOR(wheel_calibrated);
 RC_SENSOR(steering_scale);RC_SENSOR(steering_sign);RC_SENSOR(steering_calibrated);
 RC_SENSOR(ground_scale);RC_SENSOR(ground_lever_x);RC_SENSOR(ground_lever_y);
 RC_SENSOR(ground_calibrated);RC_SENSOR(ground_frame_verified);RC_SENSOR(ground_independent);
 RC_SENSOR(ground_heading_verified);RC_SENSOR(ground_lever_arm_verified);
 RC_SENSOR(gyro_scale);RC_SENSOR(gyro_sign);RC_SENSOR(accel_scale);RC_SENSOR(imu_frame_verified);
 RC_SENSOR(stale_after);RC_SENSOR(sync_tolerance);RC_SENSOR(identify_yaw);RC_SENSOR(identify_steering);
 c.observer.source_timestamps_verified=s.param("source_timestamps_verified",false);
 c.observer.steering_delay_verified=s.param("steering_delay_verified",false);
#undef RC_SENSOR
 required(n,"terrain_estimation_enabled",c.terrain_estimation_enabled);
 required(n,"use_estimated_terrain",c.use_estimated_terrain);
 required(n,"model_adaptation_enabled",c.model_adaptation_enabled);
 required(n,"weight_adaptation_enabled",c.weight_adaptation_enabled);
 required(n,"feature_minimum_confidence",c.feature_minimum_confidence);
 double rate=0.;required(v,"control_rate",rate);
 if(rate!=30.||c.solver.budget_ms>12.||c.solver.budget_ms<1.||!AdaptiveRuntime::validate(c))
   throw std::runtime_error("runtime range/safety validation failed");
 return c;
}
}
