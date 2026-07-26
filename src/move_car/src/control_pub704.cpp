#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_car/car_parameter.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

namespace {

double clampValue(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

double square(double value) {
    return value * value;
}

double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double safeHypot(double x, double y) {
    return std::sqrt(x * x + y * y);
}

double percentile(std::vector<double> values, double q) {
    if (values.empty()) {
        return 0.0;
    }
    q = clampValue(q, 0.0, 1.0);
    const size_t idx = static_cast<size_t>(
        clampValue(std::floor(q * static_cast<double>(values.size() - 1)), 0.0,
                   static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<long>(idx), values.end());
    return values[idx];
}

bool finitePoint(double x, double y) {
    return std::isfinite(x) && std::isfinite(y);
}

struct VehicleState {
    ros::Time stamp;
    double speed = 0.0;
    double steering_feedback = 0.0;
    double yaw_rate = 0.0;
    bool speed_valid = false;
    bool steering_valid = false;
    int chassis_error = 0;
};

struct ControlCmd {
    double speed = 0.0;
    double steering = 0.0;
};

struct LeadTargetState {
    bool valid = false;
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double distance = std::numeric_limits<double>::infinity();
    double rel_vx = 0.0;
    double rel_vy = 0.0;
    double range_rate = 0.0;
    double lead_speed = 0.0;
    double cth_gap_error = 0.0;
    double confidence = 0.0;
};

struct PathPoint {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double curvature = 0.0;
};

struct PathBuffer {
    ros::Time stamp;
    std::vector<PathPoint> points;
    bool valid = false;
    bool held_reference = false;
    std::string source;
    bool conditioned = false;
    double turn_entry_s = -1.0;
    double distance_to_turn_entry = -1.0;
    double turn_blend = 0.0;
    std::string segment_mode = "raw";
};

struct PathTrackingProblem {
    bool valid = false;
    double e_y = 0.0;
    double e_theta = 0.0;
    double projection_s = 0.0;
    double path_span = 0.0;
    double nearest_distance = 0.0;
    double first_curvature = 0.0;
    double fit_rms = 0.0;
    bool fit_valid = false;
    double line_fit_rms = 1e3;
    double line_yaw = 0.0;
    double line_span = 0.0;
    double straight_confidence = 0.0;
    std::vector<double> curvatures;
};

struct ControllerDebug {
    bool path_valid = false;
    bool using_raw_fallback = false;
    bool target_valid = false;
    bool emergency_stop = false;
    bool safe_stop = false;
    std::string stop_reason = "init";
    double e_y = 0.0;
    double e_theta = 0.0;
    double first_curvature = 0.0;
    double gamma_cps = 0.0;
    double steering_residual = 0.0;
    double imu_z_variance = 0.0;
    double gap = -1.0;
    double range_rate = 0.0;
    double gap_error = 0.0;
    double lead_speed = 0.0;
    double cruise_speed = 0.0;
    double planned_speed = 0.0;
    double fit_rms = 0.0;
    double slqr_cost = 0.0;
    double barrier_margin = 1.0;
    int delay_steps = 0;
    double delta_slqr = 0.0;
    double delta_comp = 0.0;
    double delta_cmd = 0.0;
    double path_span = 0.0;
    int path_points = 0;
    double reference_quality = 1.0;
    std::string hold_reason = "none";
    double heading_jump = 0.0;
    double kappa_jump = 0.0;
    bool comp_clipped = false;
    double gamma_steer_component = 0.0;
    double gamma_imu_component = 0.0;
    double steer_feedback_for_control = 0.0;
    double yaw_rate = 0.0;
    double yaw_rate_raw_imu = 0.0;
    double yaw_rate_feedback_est = 0.0;
    bool yaw_rate_valid = false;
    std::string yaw_rate_source = "none";
    double yaw_damping_delta = 0.0;
    bool steer_sign_mismatch = false;
    double barrier_adapt_scale = 1.0;
    double kappa_control = 0.0;
    bool reference_held = false;
    std::string path_stale_reason = "none";
    double control_ramp_scale = 1.0;
    double held_reference_speed_limit = 0.0;
    double candidate_e_y = 0.0;
    double candidate_e_theta = 0.0;
    double candidate_kappa0 = 0.0;
    double candidate_kappa_eval = 0.0;
    double candidate_fit_rms = 0.0;
    double candidate_span = 0.0;
    int candidate_points = 0;
    double last_good_age = -1.0;
    double reference_age = -1.0;
    double raw_age = -1.0;
    std::string reject_stage = "none";
    std::string control_mode = "INIT";
    std::string reference_status_mode = "UNKNOWN";
    std::string reference_status_reason = "none";
    std::string hard_stop_reason = "none";
    std::string soft_recovery_reason = "none";
    double filtered_e_y = 0.0;
    double filtered_e_theta = 0.0;
    double filtered_kappa_eval = 0.0;
    double yaw_rate_bound = 0.0;
    double envelope_slack = 0.0;
    double tracking_slack = 0.0;
    double sqp_jerk_cost = 0.0;
    bool soft_constraint_active = false;
    bool straight_lock = false;
    double straight_confidence = 0.0;
    double line_fit_rms = 0.0;
    double line_yaw = 0.0;
    double kappa_raw_front = 0.0;
    double kappa_trimmed = 0.0;
    double kappa_suppressed = 0.0;
    double delta_ff = 0.0;
    double delta_fb = 0.0;
    double yaw_rate_penalty = 0.0;
    double oscillation_score = 0.0;
    bool straight_speed_limited = false;
    int raw_chassis_error = 0;
    bool heading_suspect = false;
    double e_theta_control = 0.0;
    double delta_rail = 0.0;
    double rail_blend = 0.0;
    double nearest_obstacle = -1.0;
    bool collision_stop_active = false;
    double collision_stop_age = 0.0;
    double collision_resume_distance = 0.0;
    std::string collision_source = "none";
    double course_error = 0.0;
    std::string course_error_source = "none";
    double e_y_rate = 0.0;
    double delta_course = 0.0;
    double delta_bias = 0.0;
    double bias_apply_scale = 0.0;
    bool bias_update_allowed = false;
    std::string bias_block_reason = "none";
    double reference_confidence = 0.0;
    bool bias_learning_active = false;
    bool bias_learning_suspended = false;
    bool course_lock_active = false;
    double course_slqr_blend = 0.0;
    double yaw_rate_ref = 0.0;
    double yaw_rate_error = 0.0;
    double track_signed_e_y = 0.0;
    double track_kappa_ref = 0.0;
    double track_beta_est = 0.0;
    double track_curve_blend = 0.0;
    double delta_ref = 0.0;
    double delta_ref_ff = 0.0;
    double delta_ref_course = 0.0;
    double delta_ref_lateral = 0.0;
    double delta_ref_ey_rate = 0.0;
    double delta_ref_yaw = 0.0;
    double unified_slqr_weight = 0.0;
    double tracking_speed_scale = 1.0;
    double kappa_ref_filtered = 0.0;
    bool kappa_ref_rate_limited = false;
    bool target_lost_slow_track = false;
    std::string collision_latch_source = "none";
    double last_close_target_age = -1.0;
    double kappa_pp = 0.0;
    double kappa_path = 0.0;
    double kappa_cmd = 0.0;
    double kappa_ff = 0.0;
    double kappa_pp_weighted = 0.0;
    double psi_preview = 0.0;
    double psi_preview_used = 0.0;
    double preview_lookahead = 0.0;
    double preview_x = 0.0;
    double preview_y = 0.0;
    bool path_sign_conflict = false;
    double inside_error = 0.0;
    double inside_gain_scale = 1.0;
    double ff_reduce_scale = 1.0;
    double lookahead_adaptive = 0.0;
    double actuator_delay_est = 0.0;
    double nearest_cloud_obstacle = -1.0;
    int collision_point_count = 0;
    std::string cloud_frame = "none";
    bool cloud_tf_ok = false;
    bool cloud_tf_failed = false;
    double cloud_tf_age = -1.0;
    double nearest_cloud_x = 0.0;
    double nearest_cloud_y = 0.0;
    double collision_raw_nearest = -1.0;
    double collision_effective_nearest = -1.0;
    double preview_min_v = 0.0;
    double commanded_v_ref = 0.0;
    double curve_distance = 0.0;
    double preview_curve_speed_limit = 0.0;
    std::string curve_speed_reason = "none";
    bool reference_conditioner_valid = false;
    double conditioned_fit_rms = 0.0;
    double conditioned_kappa = 0.0;
    double conditioned_kappa_rate = 0.0;
    double raw_path_kappa_p95 = 0.0;
    double turn_entry_s = -1.0;
    double distance_to_turn_entry = -1.0;
    double turn_blend = 0.0;
    std::string segment_mode = "raw";
    std::string conditioner_reject_reason = "none";
    std::string speed_limit_reason = "none";
    double curve_speed_cap = 0.0;
    double inside_speed_scale = 1.0;
    double sqp_preview_min_v = 0.0;
    double final_speed_cap = 0.0;
    double visual_gap_urgency = 0.0;
    std::string speed_arbitration_mode = "none";
    double curve_safe_cap = 0.0;
    double catchup_speed_floor = 0.0;
    double risk_speed_floor = 0.0;
    std::string speed_cap_source = "none";
    bool cloud_msg_received = false;
    double cloud_age = -1.0;
    bool cloud_source_active = false;
    bool cloud_parse_failed = false;
    double front_axle_e_ct = 0.0;
    double inside_error_front = 0.0;
    std::string lookahead_mode = "raw";
    bool lookahead_limited_by_turn_entry = false;
    double kappa_ref_delay = 0.0;
    double steering_delay_predicted_e_y = 0.0;
    double corner_cut_score = 0.0;
    double turn_blend_effective = 0.0;
    bool delta_lateral_limited = false;
    double speed_risk_score = 0.0;
    std::string speed_stop_class = "none";
    std::string reference_trackable_mode = "unknown";
    double gap_speed_floor = 0.0;
    bool speed_filter_limited = false;
    bool accel_limited = false;
    std::string ref_tracking_class = "unknown";
    bool straight_guard_active = false;
    std::string pp_suppressed_reason = "none";
    bool preview_x_too_short = false;
    double delta_lateral_raw = 0.0;
    double delta_lateral_guarded = 0.0;
    double reference_status_age = -1.0;
    bool low_confidence_steer_cap_active = false;
    bool curve_evidence_reliable = false;
    bool ignored_short_path_kappa = false;
    bool low_confidence_delta_cap = false;
    bool imu_yaw_rate_unavailable = false;
    double yaw_feedback_source_age = -1.0;
    std::string yaw_rate_damping_source = "none";
};

class CPSStateObserver {
public:
    void configure(ros::NodeHandle& pnh) {
        alpha_steer_ = pnh.param("cps_alpha_steer", 1.35);
        alpha_imu_z_ = pnh.param("cps_alpha_imu_z", 0.035);
        gamma_max_ = pnh.param("cps_gamma_max", 0.80);
        steering_delay_ = pnh.param("cps_steering_delay", 0.16);
        imu_window_seconds_ = pnh.param("cps_imu_window", 0.80);
        command_history_seconds_ = pnh.param("cps_command_history", 2.0);
        residual_deadband_ = pnh.param("cps_residual_deadband", 0.025);
        residual_lpf_alpha_ = clampValue(pnh.param("cps_residual_lpf_alpha", 0.18), 0.0, 1.0);
        max_residual_for_gamma_ = pnh.param("cps_max_steering_residual_for_gamma", 0.35);
    }

    void updateSteeringFeedback(double steering, const ros::Time& now) {
        steering_feedback_ = steering;
        feedback_stamp_ = now;
        have_feedback_ = true;
    }

    void updateImu(const sensor_msgs::Imu& msg, const ros::Time& now) {
        const double z = msg.linear_acceleration.z;
        if (!std::isfinite(z)) {
            return;
        }
        imu_z_samples_.push_back(TimedValue{now, z});
        while (!imu_z_samples_.empty() &&
               (now - imu_z_samples_.front().stamp).toSec() > imu_window_seconds_) {
            imu_z_samples_.pop_front();
        }
        computeImuVariance();
    }

    void recordCommand(double steering_cmd, const ros::Time& now) {
        command_history_.push_back(TimedValue{now, steering_cmd});
        while (!command_history_.empty() &&
               (now - command_history_.front().stamp).toSec() > command_history_seconds_) {
            command_history_.pop_front();
        }
        last_cmd_ = steering_cmd;
    }

    double signedSteeringResidual(const ros::Time& now) const {
        if (!have_feedback_) {
            return 0.0;
        }
        return steering_feedback_ - delayedCommand(now);
    }

    double steeringResidualAbs(const ros::Time& now) const {
        return std::fabs(signedSteeringResidual(now));
    }

    double gamma(const ros::Time& now) {
        const double d_delta = clampValue(std::max(0.0, steeringResidualAbs(now) - residual_deadband_),
                                          0.0,
                                          max_residual_for_gamma_);
        filtered_abs_residual_ =
            (1.0 - residual_lpf_alpha_) * filtered_abs_residual_ + residual_lpf_alpha_ * d_delta;
        gamma_steer_component_ = alpha_steer_ * filtered_abs_residual_;
        gamma_imu_component_ = alpha_imu_z_ * imu_z_variance_;
        const double raw = gamma_steer_component_ + gamma_imu_component_;
        return clampValue(raw, 0.0, gamma_max_);
    }

    double imuZVariance() const {
        return imu_z_variance_;
    }

    double gammaSteerComponent() const {
        return gamma_steer_component_;
    }

    double gammaImuComponent() const {
        return gamma_imu_component_;
    }

private:
    struct TimedValue {
        ros::Time stamp;
        double value = 0.0;
    };

    double delayedCommand(const ros::Time& now) const {
        if (command_history_.empty()) {
            return last_cmd_;
        }
        const ros::Time query = now - ros::Duration(std::max(0.0, steering_delay_));
        double best = command_history_.front().value;
        double best_age = std::fabs((command_history_.front().stamp - query).toSec());
        for (size_t i = 1; i < command_history_.size(); ++i) {
            const double age = std::fabs((command_history_[i].stamp - query).toSec());
            if (age < best_age) {
                best_age = age;
                best = command_history_[i].value;
            }
        }
        return best;
    }

    void computeImuVariance() {
        if (imu_z_samples_.size() < 3) {
            imu_z_variance_ = 0.0;
            return;
        }
        double mean = 0.0;
        for (const TimedValue& sample : imu_z_samples_) {
            mean += sample.value;
        }
        mean /= static_cast<double>(imu_z_samples_.size());
        double variance = 0.0;
        for (const TimedValue& sample : imu_z_samples_) {
            variance += square(sample.value - mean);
        }
        imu_z_variance_ = variance / static_cast<double>(imu_z_samples_.size() - 1);
    }

    std::deque<TimedValue> command_history_;
    std::deque<TimedValue> imu_z_samples_;
    ros::Time feedback_stamp_;
    bool have_feedback_ = false;
    double steering_feedback_ = 0.0;
    double last_cmd_ = 0.0;
    double imu_z_variance_ = 0.0;
    double filtered_abs_residual_ = 0.0;
    double gamma_steer_component_ = 0.0;
    double gamma_imu_component_ = 0.0;
    double alpha_steer_ = 1.35;
    double alpha_imu_z_ = 0.035;
    double gamma_max_ = 0.80;
    double steering_delay_ = 0.16;
    double imu_window_seconds_ = 0.80;
    double command_history_seconds_ = 2.0;
    double residual_deadband_ = 0.025;
    double residual_lpf_alpha_ = 0.18;
    double max_residual_for_gamma_ = 0.35;
};

class LeadTargetObserver {
public:
    void configure(ros::NodeHandle& pnh) {
        min_forward_x_ = pnh.param("target_min_forward_x", -0.10);
        lateral_gate_ = pnh.param("target_lateral_gate", 3.00);
        target_timeout_ = pnh.param("target_timeout", 0.45);
        pose_alpha_ = pnh.param("target_pose_alpha", 0.35);
        yaw_alpha_ = pnh.param("target_yaw_alpha", 0.20);
        velocity_alpha_ = pnh.param("target_velocity_alpha", 0.20);
        range_alpha_ = pnh.param("target_range_alpha", 0.55);
        range_beta_ = pnh.param("target_range_beta", 0.12);
        max_range_rate_ = pnh.param("target_max_range_rate", 1.0);
        lost_range_rate_decay_ = pnh.param("target_lost_range_rate_decay", 0.82);
        hard_stop_distance_ = pnh.param("hard_stop_distance", 0.50);
        hard_stop_forward_x_ = pnh.param("hard_stop_forward_x", -0.05);
    }

    void update(const geometry_msgs::PoseArray& msg, const ros::Time& now) {
        emergency_stop_ = false;
        nearest_obstacle_ = std::numeric_limits<double>::infinity();

        int best_index = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < msg.poses.size(); ++i) {
            const geometry_msgs::Pose& pose = msg.poses[i];
            const double x = pose.position.x;
            const double y = pose.position.y;
            if (!finitePoint(x, y)) {
                continue;
            }
            const double distance = safeHypot(x, y);
            if (x >= hard_stop_forward_x_ && std::fabs(y) <= lateral_gate_) {
                nearest_obstacle_ = std::min(nearest_obstacle_, distance);
                if (distance <= hard_stop_distance_) {
                    emergency_stop_ = true;
                }
            }
            if (x < min_forward_x_ || std::fabs(y) > lateral_gate_) {
                continue;
            }
            const double lateral_cost = 0.55 * std::fabs(y);
            const double continuity_cost =
                state_.valid ? 0.35 * safeHypot(x - state_.x, y - state_.y) : 0.0;
            const double cost = distance + lateral_cost + continuity_cost;
            if (cost < best_cost) {
                best_cost = cost;
                best_index = static_cast<int>(i);
            }
        }

        if (best_index < 0) {
            return;
        }

        const geometry_msgs::Pose& pose = msg.poses[static_cast<size_t>(best_index)];
        const double x = pose.position.x;
        const double y = pose.position.y;
        const double yaw = yawFromQuaternion(pose.orientation);
        const double measured_range = safeHypot(x, y);

        if (!state_.valid) {
            state_.valid = true;
            state_.stamp = now;
            state_.x = x;
            state_.y = y;
            state_.yaw = yaw;
            state_.distance = measured_range;
            state_.range_rate = 0.0;
            state_.confidence = 1.0;
            return;
        }

        const double dt = std::max(0.03, (now - state_.stamp).toSec());
        const double meas_vx = (x - state_.x) / dt;
        const double meas_vy = (y - state_.y) / dt;
        const double predicted_range = state_.distance + state_.range_rate * dt;
        const double range_residual = measured_range - predicted_range;
        state_.distance = std::max(0.0, predicted_range + range_alpha_ * range_residual);
        state_.range_rate = clampValue(
            state_.range_rate + (range_beta_ / dt) * range_residual,
            -max_range_rate_,
            max_range_rate_);
        state_.x += pose_alpha_ * (x - state_.x);
        state_.y += pose_alpha_ * (y - state_.y);
        state_.yaw = wrapAngle(state_.yaw + yaw_alpha_ * wrapAngle(yaw - state_.yaw));
        state_.rel_vx += velocity_alpha_ * (meas_vx - state_.rel_vx);
        state_.rel_vy += velocity_alpha_ * (meas_vy - state_.rel_vy);
        state_.confidence = clampValue(state_.confidence + 0.20 * (1.0 - state_.confidence), 0.0, 1.0);
        state_.stamp = now;
    }

    LeadTargetState state(const ros::Time& now) const {
        LeadTargetState out = state_;
        if (!out.valid) {
            return out;
        }
        const double age = (now - out.stamp).toSec();
        if (age > target_timeout_) {
            out.valid = false;
            out.range_rate *= lost_range_rate_decay_;
            out.confidence = 0.0;
        } else {
            out.x += out.rel_vx * std::max(0.0, age);
            out.y += out.rel_vy * std::max(0.0, age);
            out.distance = std::max(0.0, out.distance + out.range_rate * std::max(0.0, age));
            out.confidence = clampValue(out.confidence * (1.0 - age / std::max(0.05, target_timeout_)), 0.0, 1.0);
        }
        return out;
    }

    bool emergencyStop() const {
        return emergency_stop_;
    }

    double nearestObstacle() const {
        return nearest_obstacle_;
    }

private:
    LeadTargetState state_;
    bool emergency_stop_ = false;
    double nearest_obstacle_ = std::numeric_limits<double>::infinity();
    double min_forward_x_ = -0.10;
    double lateral_gate_ = 3.00;
    double target_timeout_ = 0.45;
    double pose_alpha_ = 0.35;
    double yaw_alpha_ = 0.20;
    double velocity_alpha_ = 0.20;
    double range_alpha_ = 0.55;
    double range_beta_ = 0.12;
    double max_range_rate_ = 1.0;
    double lost_range_rate_decay_ = 0.82;
    double hard_stop_distance_ = 0.50;
    double hard_stop_forward_x_ = -0.05;
};

struct CollisionState {
    bool active = false;
    double nearest = std::numeric_limits<double>::infinity();
    double age = std::numeric_limits<double>::infinity();
    double active_age = 0.0;
    double resume_distance = 0.55;
    std::string source = "none";
    std::string latch_source = "none";
    double last_close_target_age = -1.0;
    double nearest_cloud = -1.0;
    int cloud_point_count = 0;
    std::string cloud_frame = "none";
    bool cloud_tf_ok = false;
    bool cloud_tf_failed = false;
    double cloud_tf_age = -1.0;
    double nearest_cloud_x = 0.0;
    double nearest_cloud_y = 0.0;
    double raw_nearest = -1.0;
    double effective_nearest = -1.0;
    bool cloud_msg_received = false;
    double cloud_age = -1.0;
    bool cloud_source_active = false;
    bool cloud_parse_failed = false;
};

class CollisionSupervisor {
public:
    void configure(ros::NodeHandle& pnh) {
        enabled_ = pnh.param("collision_stop_enabled", true);
        stop_distance_ = pnh.param("collision_stop_distance", 0.50);
        resume_distance_ = std::max(stop_distance_ + 0.01,
                                    pnh.param("collision_resume_distance", 0.55));
        timeout_ = pnh.param("collision_observation_timeout", 0.35);
        min_forward_x_ = pnh.param("collision_min_forward_x", 0.0);
        max_lateral_y_ = pnh.param("collision_lateral_gate", 0.85);
        close_target_latch_enabled_ = pnh.param("collision_close_target_latch_enabled", true);
        close_target_latch_distance_ =
            std::max(stop_distance_, pnh.param("collision_close_target_latch_distance", 0.80));
        close_target_latch_timeout_ =
            std::max(timeout_, pnh.param("collision_close_target_latch_timeout", 0.50));
        cloud_enabled_ = pnh.param("collision_cloud_enabled", true);
        cloud_timeout_ = std::max(timeout_, pnh.param("collision_cloud_timeout", 1.00));
        cloud_stride_ = std::max(1, pnh.param("collision_cloud_stride", 3));
        cloud_min_x_ = pnh.param("collision_cloud_min_x", 0.0);
        cloud_max_x_ = pnh.param("collision_cloud_max_x", 1.50);
        cloud_lateral_gate_ = pnh.param("collision_cloud_lateral_gate", max_lateral_y_);
        cloud_min_z_ = pnh.param("collision_cloud_min_z", -0.40);
        cloud_max_z_ = pnh.param("collision_cloud_max_z", 1.20);
    }

    void updateTracked(const geometry_msgs::PoseArray& msg, const ros::Time& now) {
        updateObservation(msg, now, "tracked_objects", tracked_);
    }

    void updateObstaclePoses(const geometry_msgs::PoseArray& msg, const ros::Time& now) {
        updateObservation(msg, now, "pointpillars_poses", poses_);
    }

    void updateLeadTarget(const LeadTargetState& lead, const ros::Time& now) {
        if (!close_target_latch_enabled_ || !lead.valid) {
            updateLatch(now);
            return;
        }
        if (lead.x >= min_forward_x_ &&
            std::fabs(lead.y) <= max_lateral_y_ &&
            lead.distance <= close_target_latch_distance_) {
            close_target_.stamp = now;
            close_target_.nearest = lead.distance;
            close_target_.source = "lead_latch";
        }
        updateLatch(now);
    }

    void updatePointCloud(const sensor_msgs::PointCloud2& msg,
                          const ros::Time& now,
                          const std::string& source,
                          const Eigen::Affine3d& cloud_to_base,
                          bool tf_ok,
                          const std::string& frame_id,
                          double tf_age) {
        cloud_.stamp = now;
        cloud_.nearest = std::numeric_limits<double>::infinity();
        cloud_.source = source;
        cloud_point_count_ = 0;
        cloud_frame_ = frame_id.empty() ? "unknown" : frame_id;
        cloud_tf_ok_ = tf_ok;
        cloud_tf_failed_ = !tf_ok;
        cloud_tf_age_ = tf_age;
        cloud_parse_failed_ = false;
        cloud_nearest_x_ = 0.0;
        cloud_nearest_y_ = 0.0;
        cloud_raw_nearest_ = std::numeric_limits<double>::infinity();
        if (!cloud_enabled_) {
            updateLatch(now);
            return;
        }
        if (!tf_ok) {
            ROS_WARN_THROTTLE(1.0,
                              "collision cloud TF unavailable from %s frame=%s",
                              source.c_str(),
                              cloud_frame_.c_str());
            updateLatch(now);
            return;
        }
        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
            int idx = 0;
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++idx) {
                if (idx % cloud_stride_ != 0) {
                    continue;
                }
                const double x = static_cast<double>(*iter_x);
                const double y = static_cast<double>(*iter_y);
                const double z = static_cast<double>(*iter_z);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    continue;
                }
                cloud_raw_nearest_ = std::min(cloud_raw_nearest_, safeHypot(x, y));
                const Eigen::Vector3d p_base = cloud_to_base * Eigen::Vector3d(x, y, z);
                const double bx = p_base.x();
                const double by = p_base.y();
                const double bz = p_base.z();
                if (bx < cloud_min_x_ || bx > cloud_max_x_ ||
                    std::fabs(by) > cloud_lateral_gate_ ||
                    bz < cloud_min_z_ || bz > cloud_max_z_) {
                    continue;
                }
                ++cloud_point_count_;
                const double d = safeHypot(bx, by);
                if (d < cloud_.nearest) {
                    cloud_.nearest = d;
                    cloud_nearest_x_ = bx;
                    cloud_nearest_y_ = by;
                }
            }
        } catch (const std::runtime_error& e) {
            cloud_parse_failed_ = true;
            ROS_WARN_THROTTLE(1.0, "collision cloud parse failed from %s: %s", source.c_str(), e.what());
        }
        updateLatch(now);
    }

    CollisionState state(const ros::Time& now) {
        updateLatch(now);
        CollisionState out;
        out.active = active_;
        out.nearest = combinedNearest(now, &out.source);
        out.age = combinedAge(now);
        out.active_age = active_ && !active_since_.isZero() ? (now - active_since_).toSec() : 0.0;
        out.resume_distance = resume_distance_;
        out.latch_source = closeTargetFresh(now) ? close_target_.source : "none";
        out.last_close_target_age = closeTargetFresh(now) ? (now - close_target_.stamp).toSec() : -1.0;
        out.nearest_cloud = freshCloud(now) && std::isfinite(cloud_.nearest) ? cloud_.nearest : -1.0;
        out.cloud_point_count = freshCloud(now) ? cloud_point_count_ : 0;
        out.cloud_frame = cloud_frame_;
        out.cloud_tf_ok = freshCloud(now) && cloud_tf_ok_;
        out.cloud_tf_failed = freshCloud(now) && cloud_tf_failed_;
        out.cloud_tf_age = freshCloud(now) ? cloud_tf_age_ : -1.0;
        out.nearest_cloud_x = freshCloud(now) ? cloud_nearest_x_ : 0.0;
        out.nearest_cloud_y = freshCloud(now) ? cloud_nearest_y_ : 0.0;
        out.raw_nearest = freshCloud(now) && std::isfinite(cloud_raw_nearest_) ? cloud_raw_nearest_ : -1.0;
        out.effective_nearest = out.nearest;
        out.cloud_msg_received = !cloud_.stamp.isZero();
        out.cloud_age = out.cloud_msg_received ? (now - cloud_.stamp).toSec() : -1.0;
        out.cloud_source_active = freshCloud(now);
        out.cloud_parse_failed = freshCloud(now) && cloud_parse_failed_;
        if (!std::isfinite(out.nearest)) {
            out.nearest = -1.0;
        }
        if (!std::isfinite(out.effective_nearest)) {
            out.effective_nearest = -1.0;
        }
        return out;
    }

private:
    struct Observation {
        ros::Time stamp;
        double nearest = std::numeric_limits<double>::infinity();
        std::string source = "none";
    };

    void updateObservation(const geometry_msgs::PoseArray& msg,
                           const ros::Time& now,
                           const std::string& source,
                           Observation& obs) {
        obs.stamp = now;
        obs.nearest = std::numeric_limits<double>::infinity();
        obs.source = source;
        for (const geometry_msgs::Pose& pose : msg.poses) {
            const double x = pose.position.x;
            const double y = pose.position.y;
            if (!finitePoint(x, y)) {
                continue;
            }
            if (x < min_forward_x_ || std::fabs(y) > max_lateral_y_) {
                continue;
            }
            obs.nearest = std::min(obs.nearest, safeHypot(x, y));
        }
        updateLatch(now);
    }

    bool fresh(const Observation& obs, const ros::Time& now) const {
        return !obs.stamp.isZero() && (now - obs.stamp).toSec() <= timeout_;
    }

    bool freshCloud(const ros::Time& now) const {
        return !cloud_.stamp.isZero() && (now - cloud_.stamp).toSec() <= cloud_timeout_;
    }

    bool closeTargetFresh(const ros::Time& now) const {
        return close_target_latch_enabled_ &&
               !close_target_.stamp.isZero() &&
               (now - close_target_.stamp).toSec() <= close_target_latch_timeout_;
    }

    double combinedNearest(const ros::Time& now, std::string* source) const {
        double nearest = std::numeric_limits<double>::infinity();
        std::string nearest_source = "none";
        if (fresh(tracked_, now) && tracked_.nearest < nearest) {
            nearest = tracked_.nearest;
            nearest_source = tracked_.source;
        }
        if (fresh(poses_, now) && poses_.nearest < nearest) {
            nearest = poses_.nearest;
            nearest_source = poses_.source;
        }
        if (closeTargetFresh(now) && close_target_.nearest < nearest) {
            nearest = close_target_.nearest;
            nearest_source = close_target_.source;
        }
        if (freshCloud(now) && cloud_.nearest < nearest) {
            nearest = cloud_.nearest;
            nearest_source = cloud_.source;
        }
        if (source != nullptr) {
            *source = nearest_source;
        }
        return nearest;
    }

    double combinedAge(const ros::Time& now) const {
        double age = std::numeric_limits<double>::infinity();
        if (!tracked_.stamp.isZero()) {
            age = std::min(age, (now - tracked_.stamp).toSec());
        }
        if (!poses_.stamp.isZero()) {
            age = std::min(age, (now - poses_.stamp).toSec());
        }
        if (closeTargetFresh(now)) {
            age = std::min(age, (now - close_target_.stamp).toSec());
        }
        if (!cloud_.stamp.isZero()) {
            age = std::min(age, (now - cloud_.stamp).toSec());
        }
        return std::isfinite(age) ? age : -1.0;
    }

    void updateLatch(const ros::Time& now) {
        if (!enabled_) {
            active_ = false;
            return;
        }
        const double nearest = combinedNearest(now, nullptr);
        if (!active_) {
            if (std::isfinite(nearest) && nearest <= stop_distance_) {
                active_ = true;
                active_since_ = now;
            }
            return;
        }
        if (!std::isfinite(nearest) || nearest > resume_distance_) {
            active_ = false;
            active_since_ = ros::Time();
        }
    }

    Observation tracked_;
    Observation poses_;
    Observation close_target_;
    Observation cloud_;
    ros::Time active_since_;
    int cloud_point_count_ = 0;
    std::string cloud_frame_ = "none";
    bool cloud_tf_ok_ = false;
    bool cloud_tf_failed_ = false;
    bool cloud_parse_failed_ = false;
    double cloud_tf_age_ = -1.0;
    double cloud_nearest_x_ = 0.0;
    double cloud_nearest_y_ = 0.0;
    double cloud_raw_nearest_ = std::numeric_limits<double>::infinity();
    bool active_ = false;
    bool enabled_ = true;
    double stop_distance_ = 0.50;
    double resume_distance_ = 0.55;
    double timeout_ = 0.35;
    double min_forward_x_ = 0.0;
    double max_lateral_y_ = 0.85;
    bool close_target_latch_enabled_ = true;
    double close_target_latch_distance_ = 0.80;
    double close_target_latch_timeout_ = 0.50;
    bool cloud_enabled_ = true;
    double cloud_timeout_ = 1.00;
    int cloud_stride_ = 3;
    double cloud_min_x_ = 0.0;
    double cloud_max_x_ = 1.50;
    double cloud_lateral_gate_ = 0.85;
    double cloud_min_z_ = -0.40;
    double cloud_max_z_ = 1.20;
};

class SteeringBiasObserver {
public:
    void configure(ros::NodeHandle& pnh) {
        enabled_ = pnh.param("bias_learning_enabled", true);
        learning_rate_ = pnh.param("bias_learning_rate", 0.015);
        leak_rate_ = pnh.param("bias_leak_rate", 0.003);
        max_bias_ = std::fabs(pnh.param("bias_max_delta", 0.08));
        min_speed_ = pnh.param("bias_min_speed", 0.08);
        max_speed_ = pnh.param("bias_max_speed", 0.25);
        max_abs_kappa_ = pnh.param("bias_max_abs_kappa", 0.12);
        max_abs_yaw_rate_ = pnh.param("bias_max_abs_yaw_rate", 0.10);
        residual_suspend_gate_ = pnh.param("bias_residual_suspend_gate", 0.16);
        lateral_weight_ = pnh.param("bias_lateral_weight", 0.45);
        course_weight_ = pnh.param("bias_course_weight", 0.55);
        yaw_weight_ = pnh.param("bias_yaw_weight", 0.20);
    }

    void update(double dt,
                bool straight_reference,
                bool collision_active,
                bool chassis_ok,
                double speed,
                double abs_kappa,
                double e_y,
                double course_error,
                double yaw_rate,
                double yaw_rate_ref,
                double steering_residual,
                bool& active,
                bool& suspended) {
        active = false;
        suspended = false;
        const bool can_learn =
            enabled_ &&
            straight_reference &&
            !collision_active &&
            chassis_ok &&
            speed >= min_speed_ &&
            speed <= max_speed_ &&
            abs_kappa <= max_abs_kappa_ &&
            std::fabs(yaw_rate) <= max_abs_yaw_rate_;
        if (!can_learn) {
            applyLeak(dt);
            suspended = enabled_ && straight_reference;
            return;
        }
        if (std::fabs(steering_residual) > residual_suspend_gate_) {
            applyLeak(dt);
            suspended = true;
            return;
        }
        const double lateral_term = std::atan2(-e_y, std::max(0.10, speed));
        const double yaw_term = yaw_rate_ref - yaw_rate;
        const double correction =
            course_weight_ * course_error + lateral_weight_ * lateral_term + yaw_weight_ * yaw_term;
        bias_ += clampValue(dt, 0.001, 0.20) * learning_rate_ * correction;
        applyLeak(dt);
        bias_ = clampValue(bias_, -max_bias_, max_bias_);
        active = true;
    }

    double bias() const {
        return bias_;
    }

private:
    void applyLeak(double dt) {
        const double decay = clampValue(1.0 - leak_rate_ * clampValue(dt, 0.0, 0.20), 0.0, 1.0);
        bias_ *= decay;
    }

    bool enabled_ = true;
    double learning_rate_ = 0.080;
    double leak_rate_ = 0.0005;
    double max_bias_ = 0.12;
    double min_speed_ = 0.08;
    double max_speed_ = 0.25;
    double max_abs_kappa_ = 0.12;
    double max_abs_yaw_rate_ = 0.10;
    double residual_suspend_gate_ = 0.16;
    double lateral_weight_ = 0.45;
    double course_weight_ = 0.55;
    double yaw_weight_ = 0.20;
    double bias_ = 0.0;
};

struct UnifiedTrackingSignal {
    double signed_e_y = 0.0;
    double e_course = 0.0;
    double kappa_ref = 0.0;
    double e_y_rate = 0.0;
    double beta_est = 0.0;
    bool beta_valid = false;
    double yaw_rate_ref = 0.0;
    double yaw_rate_error = 0.0;
    double curve_blend = 0.0;
    std::string source = "none";
};

struct UnifiedControlOutput {
    double delta_ref = 0.0;
    double delta_ff = 0.0;
    double delta_course = 0.0;
    double delta_lateral_raw = 0.0;
    double delta_lateral = 0.0;
    double delta_ey_rate = 0.0;
    double delta_yaw = 0.0;
    double slqr_weight = 0.0;
    bool delta_lateral_limited = false;
};

class UnifiedPathTracker {
public:
    void configure(ros::NodeHandle& pnh) {
        k_course_ = pnh.param("track_k_course", 0.60);
        k_y_ = pnh.param("track_k_y", 0.70);
        k_ey_rate_ = pnh.param("track_k_ey_rate", 0.12);
        k_yaw_rate_ = pnh.param("track_k_yaw_rate", 0.22);
        k_soft_ = pnh.param("track_k_soft", 0.12);
        delta_max_ = std::fabs(pnh.param("track_delta_max", 0.22));
        slqr_max_weight_ = clampValue(pnh.param("track_slqr_max_weight", 0.35), 0.0, 1.0);
        curve_blend_low_ = std::max(0.0, pnh.param("curve_blend_kappa_low", 0.12));
        curve_blend_high_ = std::max(curve_blend_low_ + 1e-3,
                                     pnh.param("curve_blend_kappa_high", 0.35));
        wheelbase_ = pnh.param("wheelbase", 0.60);
    }

    double curveBlend(double abs_kappa) const {
        const double x = clampValue((abs_kappa - curve_blend_low_) /
                                        std::max(1e-3, curve_blend_high_ - curve_blend_low_),
                                    0.0,
                                    1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    UnifiedControlOutput compute(const UnifiedTrackingSignal& signal,
                                 double speed,
                                 double bias,
                                 double slqr_delta) const {
        UnifiedControlOutput out;
        const double speed_abs = std::max(0.02, std::fabs(speed));
        out.delta_ff = std::atan(wheelbase_ * signal.kappa_ref);
        out.delta_course = k_course_ * signal.e_course;
        out.delta_lateral =
            std::atan2(k_y_ * signal.signed_e_y, speed_abs + k_soft_);
        out.delta_ey_rate = k_ey_rate_ * signal.e_y_rate;
        out.delta_yaw = k_yaw_rate_ * signal.yaw_rate_error;
        out.delta_ref =
            clampValue(bias +
                           out.delta_ff +
                           out.delta_course +
                           out.delta_lateral +
                           out.delta_ey_rate +
                           out.delta_yaw,
                       -delta_max_,
                       delta_max_);
        out.slqr_weight = clampValue(slqr_max_weight_ * signal.curve_blend, 0.0, slqr_max_weight_);
        out.delta_ref = (1.0 - out.slqr_weight) * out.delta_ref + out.slqr_weight * slqr_delta;
        out.delta_ref = clampValue(out.delta_ref, -delta_max_, delta_max_);
        return out;
    }

    double deltaMax() const {
        return delta_max_;
    }

private:
    double k_course_ = 0.60;
    double k_y_ = 0.70;
    double k_ey_rate_ = 0.12;
    double k_yaw_rate_ = 0.22;
    double k_soft_ = 0.12;
    double delta_max_ = 0.22;
    double slqr_max_weight_ = 0.35;
    double curve_blend_low_ = 0.12;
    double curve_blend_high_ = 0.35;
    double wheelbase_ = 0.60;
};

struct ConditionedReference {
    PathBuffer path;
    bool valid = false;
};

class ReferenceConditioner {
public:
    void configure(ros::NodeHandle& pnh) {
        start_s_ = std::max(0.0, pnh.param("conditioner_start_s", 0.25));
        end_s_ = std::max(start_s_ + 0.30, pnh.param("conditioner_end_s", 2.00));
        min_points_ = std::max(4, pnh.param("conditioner_min_points", 7));
        huber_delta_ = std::max(1e-4, pnh.param("conditioner_huber_delta", 0.08));
        ridge_ = std::max(1e-8, pnh.param("conditioner_ridge", 1e-4));
        max_rms_ = std::max(0.01, pnh.param("conditioner_max_rms", 0.16));
        max_abs_kappa_ = std::max(0.10, pnh.param("conditioner_max_abs_kappa", 1.80));
        max_kappa_rate_ = std::max(0.10, pnh.param("conditioner_max_kappa_rate", 3.00));
        turn_entry_margin_ = std::max(0.02, pnh.param("conditioner_turn_entry_margin", 0.25));
        turn_blend_distance_ = std::max(0.05, pnh.param("conditioner_turn_blend_distance", 0.25));
        turn_entry_kappa_gate_ = std::max(0.02, pnh.param("conditioner_turn_entry_kappa_gate", 0.28));
        turn_entry_cumulative_yaw_gate_ =
            std::max(0.02, pnh.param("conditioner_turn_entry_cumulative_yaw_gate", 0.16));
        turn_entry_min_curve_length_ =
            std::max(0.05, pnh.param("conditioner_turn_entry_min_curve_length", 0.18));
        straight_force_kappa_gate_ =
            std::max(0.0, pnh.param("conditioner_straight_force_kappa_gate", 0.06));
        curve_fit_window_ = std::max(0.50, pnh.param("conditioner_curve_fit_window", 1.20));
    }

    ConditionedReference condition(const PathBuffer& input, ControllerDebug& debug) const {
        ConditionedReference result;
        result.path = input;
        result.path.valid = input.valid;
        result.path.conditioned = false;
        debug.conditioner_reject_reason = "none";
        debug.raw_path_kappa_p95 = rawCurvatureP95(input.points);
        if (!input.valid || static_cast<int>(input.points.size()) < min_points_) {
            debug.conditioner_reject_reason = "short_or_invalid";
            return result;
        }

        const SegmentPlan segment = chooseSegment(input.points);
        debug.turn_entry_s = segment.turn_entry_s;
        debug.distance_to_turn_entry = segment.distance_to_turn_entry;
        debug.turn_blend = segment.turn_blend;
        debug.segment_mode = segment.mode;
        result.path.turn_entry_s = segment.turn_entry_s;
        result.path.distance_to_turn_entry = segment.distance_to_turn_entry;
        result.path.turn_blend = segment.turn_blend;
        result.path.segment_mode = segment.mode;

        std::vector<const PathPoint*> samples;
        for (const PathPoint& pt : input.points) {
            if (pt.s >= segment.fit_start_s && pt.s <= segment.fit_end_s) {
                samples.push_back(&pt);
            }
        }
        if (static_cast<int>(samples.size()) < min_points_) {
            debug.conditioner_reject_reason = "segment_too_short";
            return result;
        }

        const double center_s = 0.5 * (samples.front()->s + samples.back()->s);
        Eigen::Vector4d cx = Eigen::Vector4d::Zero();
        Eigen::Vector4d cy = Eigen::Vector4d::Zero();
        std::vector<double> weights(samples.size(), 1.0);
        for (int iter = 0; iter < 3; ++iter) {
            Eigen::Matrix4d h = Eigen::Matrix4d::Zero();
            Eigen::Vector4d bx = Eigen::Vector4d::Zero();
            Eigen::Vector4d by = Eigen::Vector4d::Zero();
            for (size_t i = 0; i < samples.size(); ++i) {
                const double u = samples[i]->s - center_s;
                Eigen::Vector4d a;
                a << 1.0, u, u * u, u * u * u;
                const double w = weights[i] / (1.0 + 0.10 * std::fabs(u));
                h += w * (a * a.transpose());
                bx += w * a * samples[i]->x;
                by += w * a * samples[i]->y;
            }
            h(0, 0) += ridge_;
            h(1, 1) += ridge_;
            h(2, 2) += 4.0 * ridge_;
            h(3, 3) += 9.0 * ridge_;
            Eigen::LDLT<Eigen::Matrix4d> ldlt(h);
            if (ldlt.info() != Eigen::Success) {
                debug.conditioner_reject_reason = "ldlt_failed";
                return result;
            }
            cx = ldlt.solve(bx);
            cy = ldlt.solve(by);
            if (!cx.allFinite() || !cy.allFinite()) {
                debug.conditioner_reject_reason = "nonfinite_fit";
                return result;
            }
            for (size_t i = 0; i < samples.size(); ++i) {
                const CurveEval e = evaluate(cx, cy, samples[i]->s - center_s);
                const double r = safeHypot(e.x - samples[i]->x, e.y - samples[i]->y);
                weights[i] = huberWeight(r);
            }
        }

        double err2 = 0.0;
        double wsum = 0.0;
        for (size_t i = 0; i < samples.size(); ++i) {
            const CurveEval e = evaluate(cx, cy, samples[i]->s - center_s);
            const double r = safeHypot(e.x - samples[i]->x, e.y - samples[i]->y);
            const double w = weights[i] / (1.0 + 0.10 * std::fabs(samples[i]->s - center_s));
            err2 += w * r * r;
            wsum += w;
        }
        const double rms = wsum > 1e-8 ? std::sqrt(err2 / wsum) : 0.0;
        debug.conditioned_fit_rms = rms;
        if (rms > max_rms_) {
            debug.conditioner_reject_reason = "fit_rms";
            return result;
        }

        result.path.points = input.points;
        std::vector<double> kappa_rates;
        std::vector<double> signed_kappas;
        bool have_last = false;
        double last_s = 0.0;
        double last_kappa = 0.0;
        for (PathPoint& pt : result.path.points) {
            if (pt.s < segment.fit_start_s || pt.s > segment.fit_end_s) {
                if (segment.mode == "straight" &&
                    segment.turn_entry_s > 0.0 &&
                    pt.s < segment.turn_entry_s - 0.05) {
                    pt.curvature = 0.0;
                }
                continue;
            }
            const CurveEval e = evaluate(cx, cy, pt.s - center_s);
            pt.x = e.x;
            pt.y = e.y;
            pt.yaw = e.yaw;
            double conditioned_kappa = clampValue(e.curvature, -max_abs_kappa_, max_abs_kappa_);
            if (segment.mode == "straight" && std::fabs(conditioned_kappa) < straight_force_kappa_gate_) {
                conditioned_kappa = 0.0;
            }
            if (segment.mode == "straight") {
                conditioned_kappa = 0.0;
            }
            pt.curvature = conditioned_kappa;
            if (pt.s >= 0.40 && pt.s <= 1.40) {
                signed_kappas.push_back(pt.curvature);
            }
            if (have_last) {
                const double ds = std::max(1e-3, pt.s - last_s);
                kappa_rates.push_back(std::fabs(pt.curvature - last_kappa) / ds);
            }
            last_s = pt.s;
            last_kappa = pt.curvature;
            have_last = true;
        }
        debug.conditioned_kappa_rate = percentile(kappa_rates, 0.95);
        if (debug.conditioned_kappa_rate > max_kappa_rate_) {
            debug.conditioner_reject_reason = "kappa_rate";
            return result;
        }
        debug.conditioned_kappa = signedTrimmedMean(signed_kappas);
        debug.reference_conditioner_valid = true;
        result.path.valid = true;
        result.path.conditioned = true;
        result.valid = true;
        return result;
    }

private:
    struct SegmentPlan {
        double fit_start_s = 0.25;
        double fit_end_s = 2.00;
        double turn_entry_s = -1.0;
        double distance_to_turn_entry = -1.0;
        double turn_blend = 0.0;
        std::string mode = "straight";
    };

    struct CurveEval {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double curvature = 0.0;
    };

    CurveEval evaluate(const Eigen::Vector4d& cx, const Eigen::Vector4d& cy, double u) const {
        const double u2 = u * u;
        const double u3 = u2 * u;
        CurveEval e;
        e.x = cx[0] + cx[1] * u + cx[2] * u2 + cx[3] * u3;
        e.y = cy[0] + cy[1] * u + cy[2] * u2 + cy[3] * u3;
        const double dx = cx[1] + 2.0 * cx[2] * u + 3.0 * cx[3] * u2;
        const double dy = cy[1] + 2.0 * cy[2] * u + 3.0 * cy[3] * u2;
        const double ddx = 2.0 * cx[2] + 6.0 * cx[3] * u;
        const double ddy = 2.0 * cy[2] + 6.0 * cy[3] * u;
        e.yaw = std::atan2(dy, dx);
        const double speed2 = dx * dx + dy * dy;
        e.curvature = (dx * ddy - dy * ddx) / std::pow(std::max(1e-6, speed2), 1.5);
        return e;
    }

    SegmentPlan chooseSegment(const std::vector<PathPoint>& pts) const {
        SegmentPlan plan;
        plan.fit_start_s = start_s_;
        plan.fit_end_s = std::min(end_s_, pts.empty() ? end_s_ : pts.back().s);
        const double turn_entry = detectTurnEntry(pts);
        plan.turn_entry_s = turn_entry;
        plan.distance_to_turn_entry = turn_entry >= 0.0 ? turn_entry : -1.0;
        if (turn_entry < 0.0 || turn_entry > start_s_ + turn_entry_margin_) {
            plan.mode = "straight";
            plan.turn_blend = 0.0;
            if (turn_entry > 0.0) {
                plan.fit_end_s = std::min(plan.fit_end_s,
                                          std::max(start_s_ + 0.35,
                                                   turn_entry - 0.06));
            }
            return plan;
        }

        if (turn_entry > start_s_ - turn_blend_distance_) {
            plan.mode = "entry";
            const double distance = std::max(0.0, turn_entry - start_s_);
            plan.turn_blend = clampValue(1.0 - distance / turn_blend_distance_, 0.20, 1.0);
            plan.fit_start_s = std::max(start_s_, turn_entry - 0.10);
            plan.fit_end_s = std::min(plan.fit_start_s + curve_fit_window_,
                                      pts.empty() ? end_s_ : pts.back().s);
            return plan;
        }

        plan.mode = "curve";
        plan.turn_blend = 1.0;
        plan.fit_start_s = start_s_;
        plan.fit_end_s = std::min(start_s_ + curve_fit_window_,
                                  pts.empty() ? end_s_ : pts.back().s);
        return plan;
    }

    double detectTurnEntry(const std::vector<PathPoint>& pts) const {
        if (pts.size() < 3) {
            return -1.0;
        }
        double cumulative_yaw = 0.0;
        double continuous_curve = 0.0;
        double first_curve_s = -1.0;
        for (size_t i = 1; i < pts.size(); ++i) {
            const double ds = std::max(1e-4, pts[i].s - pts[i - 1].s);
            const double dyaw = std::fabs(wrapAngle(pts[i].yaw - pts[i - 1].yaw));
            const double kappa = dyaw / ds;
            if (pts[i].s < start_s_) {
                continue;
            }
            cumulative_yaw += dyaw;
            if (kappa >= turn_entry_kappa_gate_) {
                if (first_curve_s < 0.0) {
                    first_curve_s = pts[i - 1].s;
                }
                continuous_curve += ds;
            } else {
                first_curve_s = -1.0;
                continuous_curve = 0.0;
            }
            if (continuous_curve >= turn_entry_min_curve_length_) {
                return std::max(start_s_, first_curve_s);
            }
            if (cumulative_yaw >= turn_entry_cumulative_yaw_gate_) {
                return std::max(start_s_, pts[i - 1].s);
            }
        }
        return -1.0;
    }

    double huberWeight(double r) const {
        if (r <= huber_delta_) {
            return 1.0;
        }
        return huber_delta_ / std::max(huber_delta_, r);
    }

    double rawCurvatureP95(const std::vector<PathPoint>& pts) const {
        std::vector<double> values;
        values.reserve(pts.size());
        for (size_t i = 1; i < pts.size(); ++i) {
            const double ds = std::max(1e-4, pts[i].s - pts[i - 1].s);
            values.push_back(std::fabs(wrapAngle(pts[i].yaw - pts[i - 1].yaw) / ds));
        }
        return percentile(values, 0.95);
    }

    double signedTrimmedMean(std::vector<double> values) const {
        if (values.empty()) {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        const size_t trim = values.size() > 6 ? values.size() / 6 : 0;
        double sum = 0.0;
        int count = 0;
        for (size_t i = trim; i < values.size() - trim; ++i) {
            sum += values[i];
            ++count;
        }
        return count > 0 ? sum / static_cast<double>(count) : 0.0;
    }

    double start_s_ = 0.25;
    double end_s_ = 2.00;
    int min_points_ = 7;
    double huber_delta_ = 0.08;
    double ridge_ = 1e-4;
    double max_rms_ = 0.16;
    double max_abs_kappa_ = 1.80;
    double max_kappa_rate_ = 3.00;
    double turn_entry_margin_ = 0.25;
    double turn_blend_distance_ = 0.25;
    double turn_entry_kappa_gate_ = 0.28;
    double turn_entry_cumulative_yaw_gate_ = 0.16;
    double turn_entry_min_curve_length_ = 0.18;
    double straight_force_kappa_gate_ = 0.06;
    double curve_fit_window_ = 1.20;
};

struct PreviewGeometrySignal {
    bool valid = false;
    double lookahead = 0.0;
    double x = 0.0;
    double y = 0.0;
    double cross_track_error = 0.0;
    double psi = 0.0;
    double psi_used = 0.0;
    double kappa_pp = 0.0;
    double kappa_path = 0.0;
    double kappa_cmd = 0.0;
    double kappa_ff = 0.0;
    double kappa_pp_weighted = 0.0;
    double curve_blend = 0.0;
    bool sign_conflict = false;
    double inside_error = 0.0;
    double front_axle_e_ct = 0.0;
    double inside_error_front = 0.0;
    double inside_gain_scale = 1.0;
    double ff_reduce_scale = 1.0;
    double turn_entry_s = -1.0;
    double distance_to_turn_entry = -1.0;
    double turn_blend = 0.0;
    double turn_blend_effective = 0.0;
    std::string segment_mode = "raw";
    std::string lookahead_mode = "raw";
    bool lookahead_limited_by_turn_entry = false;
    double corner_cut_score = 0.0;
    bool straight_guard_active = false;
    std::string pp_suppressed_reason = "none";
    bool preview_x_too_short = false;
};

class PreviewGeometryTracker {
public:
    void configure(ros::NodeHandle& pnh) {
        wheelbase_ = pnh.param("wheelbase", 0.60);
        preview_min_ = pnh.param("preview_min", 0.45);
        preview_max_ = std::max(preview_min_, pnh.param("preview_max", 1.20));
        preview_gain_ = pnh.param("preview_lookahead_gain", 1.25);
        curve_preview_ = pnh.param("preview_curve_lookahead", 0.85);
        curve_preview_min_ = pnh.param("preview_curve_lookahead_min", 0.38);
        curve_preview_max_ = std::max(curve_preview_min_, pnh.param("preview_curve_lookahead_max", 0.65));
        curve_preview_kappa_low_ = std::max(0.0, pnh.param("preview_curve_kappa_low", 0.15));
        curve_preview_kappa_high_ =
            std::max(curve_preview_kappa_low_ + 1e-3,
                     pnh.param("preview_curve_kappa_high", 0.65));
        ignore_front_s_ = pnh.param("preview_ignore_front_s", 0.25);
        kappa_path_window_ = std::max(0.08, pnh.param("preview_kappa_path_window", 0.35));
        reference_kappa_start_s_ = std::max(0.02, pnh.param("preview_reference_kappa_start_s", 0.15));
        reference_kappa_end_s_ =
            std::max(reference_kappa_start_s_ + 0.05,
                     pnh.param("preview_reference_kappa_end_s", 0.55));
        reference_kappa_abs_limit_ =
            std::max(0.20, pnh.param("preview_reference_kappa_abs_limit", 2.00));
        kappa_path_weight_ = clampValue(pnh.param("preview_kappa_path_weight", 0.30), 0.0, 0.80);
        yaw_delta_limit_ = std::max(0.0, pnh.param("preview_yaw_delta_limit", 0.10));
        yaw_flip_gate_ = std::max(0.30, pnh.param("preview_yaw_flip_gate", 1.20));
        yaw_valid_kappa_gate_ = std::max(0.0, pnh.param("preview_yaw_valid_kappa_gate", 0.10));
        yaw_valid_lateral_gate_ = std::max(0.0, pnh.param("preview_yaw_valid_lateral_gate", 0.05));
        k_y_ = pnh.param("track_k_pp_y", 0.55);
        k_pp_curve_ = pnh.param("track_k_pp_curve", 0.24);
        inside_gain_ = std::max(0.0, pnh.param("track_k_inside", 1.60));
        inside_error_gate_ = std::max(0.0, pnh.param("inside_error_gate", 0.008));
        inside_error_full_ = std::max(inside_error_gate_ + 1e-3,
                                      pnh.param("inside_error_full", 0.040));
        inside_ff_reduce_max_ =
            clampValue(pnh.param("inside_ff_reduce_max", 0.35), 0.0, 0.80);
        k_preview_yaw_ = pnh.param("track_k_preview_yaw", 0.22);
        k_yaw_rate_ = pnh.param("track_k_yaw_rate", 0.36);
        k_feedback_ = pnh.param("track_k_steering_feedback", 0.12);
        k_soft_ = pnh.param("track_k_soft", 0.12);
        delta_max_ = std::fabs(pnh.param("track_delta_max", 0.32));
        kappa_pp_abs_limit_ = std::max(0.10, pnh.param("preview_kappa_pp_abs_limit", 1.25));
        kappa_pp_ref_margin_ = std::max(0.0, pnh.param("preview_kappa_pp_ref_margin", 0.50));
        kappa_pp_straight_limit_ = std::max(0.05, pnh.param("preview_kappa_pp_straight_limit", 0.35));
        sharp_entry_turn_blend_min_ =
            clampValue(pnh.param("sharp_entry_turn_blend_min", 0.85), 0.0, 1.0);
        sharp_entry_kappa_gate_ = std::max(0.0, pnh.param("sharp_entry_kappa_gate", 1.00));
        lateral_cancel_limit_ratio_ =
            clampValue(pnh.param("lateral_cancel_limit_ratio", 0.45), 0.0, 1.0);
        lateral_cancel_limit_base_ =
            std::max(0.0, pnh.param("lateral_cancel_limit_base", 0.08));
        yaw_oversteer_soft_gate_ = std::max(0.0, pnh.param("yaw_oversteer_soft_gate", 0.070));
        yaw_oversteer_hard_gate_ =
            std::max(yaw_oversteer_soft_gate_ + 1e-3,
                     pnh.param("yaw_oversteer_hard_gate", 0.115));
        yaw_oversteer_delta_scale_ =
            clampValue(pnh.param("yaw_oversteer_delta_scale", 0.70), 0.20, 1.0);
        slqr_max_weight_ = clampValue(pnh.param("track_slqr_max_weight", 0.15), 0.0, 1.0);
        curve_blend_low_ = std::max(0.0, pnh.param("curve_blend_kappa_low", 0.12));
        curve_blend_high_ = std::max(curve_blend_low_ + 1e-3,
                                     pnh.param("curve_blend_kappa_high", 0.35));
        conflict_kappa_gate_ = pnh.param("preview_sign_conflict_kappa_gate", 0.08);
        turn_entry_stop_margin_ =
            std::max(0.02, pnh.param("preview_turn_entry_stop_margin", 0.10));
        entry_preview_min_ = std::max(0.20, pnh.param("preview_entry_min", 0.35));
        entry_preview_max_ = std::max(entry_preview_min_, pnh.param("preview_entry_max", 0.50));
        front_axle_tracking_enabled_ = pnh.param("front_axle_tracking_enabled", true);
        corner_cut_gain_ = std::max(0.0, pnh.param("corner_cut_gain", 1.80));
        curve_lookahead_inside_scale_ =
            clampValue(pnh.param("curve_lookahead_inside_scale", 0.65), 0.35, 1.0);
        straight_guard_kappa_gate_ =
            std::max(0.0, pnh.param("straight_guard_kappa_gate", 0.12));
        straight_guard_curve_gate_ =
            std::max(straight_guard_kappa_gate_,
                     pnh.param("straight_guard_curve_gate", 0.20));
        straight_guard_preview_min_x_ =
            std::max(0.10, pnh.param("straight_guard_preview_min_x", 0.60));
        straight_guard_lateral_deadband_ =
            std::max(0.0, pnh.param("straight_guard_lateral_deadband", 0.035));
        straight_guard_lateral_limit_ =
            std::max(0.0, pnh.param("straight_guard_lateral_limit", 0.06));
        straight_guard_delta_max_ =
            std::max(0.0, pnh.param("straight_guard_delta_max", 0.10));
    }

    double curveBlend(double abs_kappa) const {
        const double x = clampValue((abs_kappa - curve_blend_low_) /
                                        std::max(1e-3, curve_blend_high_ - curve_blend_low_),
                                    0.0,
                                    1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    PreviewGeometrySignal observe(const PathBuffer& path,
                                  const UnifiedTrackingSignal& tracking,
                                  double speed) const {
        PreviewGeometrySignal out;
        if (!path.valid || path.points.size() < 2) {
            return out;
        }
        const double path_turn_blend = clampValue(path.turn_blend, 0.0, 1.0);
        const bool segment_straight = path.segment_mode == "straight";
        const bool segment_entry = path.segment_mode == "entry";
        const bool segment_curve = path.segment_mode == "curve";
        const double abs_ref_kappa = segment_straight ? 0.0 : std::fabs(tracking.kappa_ref);
        const double curve_amount =
            segment_curve ? 1.0 :
            (segment_entry ? std::max(path_turn_blend,
                                      smoothStep(curve_preview_kappa_low_,
                                                 curve_preview_kappa_high_,
                                                 abs_ref_kappa))
                           : 0.0);
        const double speed_preview =
            clampValue(preview_min_ + preview_gain_ * std::fabs(speed), preview_min_, preview_max_);
        double requested = speed_preview;
        if (segment_entry) {
            requested = clampValue(entry_preview_min_ +
                                       (1.0 - path_turn_blend) *
                                           (entry_preview_max_ - entry_preview_min_),
                                   entry_preview_min_,
                                   entry_preview_max_);
        } else if (segment_curve) {
            const double curve_target =
                curve_preview_min_ +
                (1.0 - curve_amount) * (curve_preview_max_ - curve_preview_min_);
            requested = clampValue(curve_target, curve_preview_min_, curve_preview_max_);
        }
        bool limited_by_turn_entry = false;
        if (segment_straight && path.turn_entry_s > 0.0) {
            const double limited = std::min(requested,
                                            std::max(ignore_front_s_,
                                                     path.turn_entry_s - turn_entry_stop_margin_));
            limited_by_turn_entry = limited + 1e-6 < requested;
            requested = limited;
        }
        double target_s =
            clampValue(std::max(ignore_front_s_, requested), path.points.front().s, path.points.back().s);

        auto fill_from_s = [&](double query_s, PreviewGeometrySignal* dst) -> bool {
            PathPoint preview;
            if (!interpolate(path.points, query_s, preview)) {
                return false;
            }
            const double ld2 = std::max(0.04, preview.x * preview.x + preview.y * preview.y);
            dst->valid = true;
            dst->lookahead = std::sqrt(ld2);
            dst->x = preview.x;
            dst->y = preview.y;
            dst->psi = wrapAngle(preview.yaw);
            dst->cross_track_error =
                std::sin(preview.yaw) * preview.x - std::cos(preview.yaw) * preview.y;
            dst->front_axle_e_ct =
                std::sin(preview.yaw) * (preview.x - wheelbase_) - std::cos(preview.yaw) * preview.y;
            dst->kappa_pp = clampValue(2.0 * preview.y / ld2,
                                       -kappa_pp_abs_limit_,
                                       kappa_pp_abs_limit_);
            dst->kappa_path = pathYawCurvature(path.points, query_s);
            return true;
        };

        if (!fill_from_s(target_s, &out)) {
            return PreviewGeometrySignal();
        }

        out.turn_entry_s = path.turn_entry_s;
        out.distance_to_turn_entry = path.distance_to_turn_entry;
        out.turn_blend = segment_straight ? 0.0 : (segment_entry ? path_turn_blend : 1.0);
        out.segment_mode = path.segment_mode;
        out.lookahead_mode = segment_curve ? "curve" : (segment_entry ? "entry" : "straight");
        out.lookahead_limited_by_turn_entry = limited_by_turn_entry;

        const double preliminary_kappa =
            std::fabs(out.kappa_path) > 1e-4 ? out.kappa_path : tracking.kappa_ref;
        const double preliminary_ff = out.turn_blend * preliminary_kappa;
        const double preliminary_sign =
            preliminary_ff > 1e-4 ? 1.0 :
            (preliminary_ff < -1e-4 ? -1.0 : 0.0);
        const double preliminary_inside =
            preliminary_sign == 0.0 ? 0.0 : std::max(0.0, preliminary_sign * out.front_axle_e_ct);
        const double preliminary_inside_amount =
            clampValue((preliminary_inside - inside_error_gate_) /
                           std::max(1e-4, inside_error_full_ - inside_error_gate_),
                       0.0,
                       1.0);
        if (!segment_straight && preliminary_inside_amount > 1e-3) {
            const double scale =
                1.0 - (1.0 - curve_lookahead_inside_scale_) * preliminary_inside_amount;
            const double shortened_s =
                clampValue(ignore_front_s_ + (target_s - ignore_front_s_) * scale,
                           path.points.front().s,
                           path.points.back().s);
            if (shortened_s + 1e-4 < target_s && fill_from_s(shortened_s, &out)) {
                target_s = shortened_s;
                out.lookahead_mode = "corner_cut";
            }
        }

        out.lookahead_limited_by_turn_entry = limited_by_turn_entry;
        out.turn_entry_s = path.turn_entry_s;
        out.distance_to_turn_entry = path.distance_to_turn_entry;
        out.turn_blend = segment_straight ? 0.0 : (segment_entry ? path_turn_blend : 1.0);
        out.segment_mode = path.segment_mode;
        const double near_reference_kappa = pathCurvatureMedian(
            path.points,
            reference_kappa_start_s_,
            std::min(reference_kappa_end_s_, std::max(reference_kappa_start_s_, path.points.back().s)));
        const double raw_kappa_ff =
            segment_straight
                ? 0.0
                : (std::fabs(near_reference_kappa) > 1e-4
                       ? near_reference_kappa
                       : (std::fabs(out.kappa_path) > 1e-4 ? out.kappa_path : tracking.kappa_ref));
        const bool sharp_entry =
            !segment_straight && std::fabs(raw_kappa_ff) > sharp_entry_kappa_gate_;
        out.turn_blend_effective =
            sharp_entry ? std::max(out.turn_blend, sharp_entry_turn_blend_min_) : out.turn_blend;
        out.kappa_ff = out.turn_blend_effective * raw_kappa_ff;
        const double ref_envelope =
            std::max(std::fabs(out.kappa_ff),
                     std::max(std::fabs(tracking.kappa_ref), std::fabs(out.kappa_path)));
        const double pp_limit =
            segment_straight
                ? kappa_pp_straight_limit_
                : std::min(kappa_pp_abs_limit_,
                           std::max(kappa_pp_straight_limit_, ref_envelope + kappa_pp_ref_margin_));
        out.kappa_pp = clampValue(out.kappa_pp, -pp_limit, pp_limit);
        out.kappa_cmd = segment_straight ? 0.0 : out.kappa_ff;
        bool path_kappa_conflict = false;
        if (std::isfinite(out.kappa_path) && std::fabs(out.kappa_path) > 1e-4) {
            path_kappa_conflict =
                std::fabs(out.kappa_pp) > conflict_kappa_gate_ &&
                std::fabs(out.kappa_path) > conflict_kappa_gate_ &&
                out.kappa_pp * out.kappa_path < 0.0;
            if (!segment_straight && !path_kappa_conflict) {
                out.kappa_cmd =
                    clampValue((1.0 - kappa_path_weight_) * out.kappa_ff +
                                   kappa_path_weight_ * out.kappa_pp,
                               -pp_limit,
                               pp_limit);
            }
        }
        out.kappa_pp_weighted = out.kappa_cmd;
        out.curve_blend = curveBlend(std::fabs(out.kappa_cmd));
        const bool kappa_conflict =
            std::fabs(tracking.kappa_ref) > conflict_kappa_gate_ &&
            std::fabs(out.kappa_cmd) > conflict_kappa_gate_ &&
            tracking.kappa_ref * out.kappa_cmd < 0.0;
        const bool yaw_conflict =
            std::fabs(out.psi) > 0.10 &&
            std::fabs(out.kappa_cmd) > conflict_kappa_gate_ &&
            out.psi * out.kappa_cmd < 0.0;
        out.sign_conflict = kappa_conflict || yaw_conflict || path_kappa_conflict;
        if (out.sign_conflict) {
            if (segment_straight) {
                out.kappa_ff = 0.0;
                out.kappa_cmd = 0.0;
            } else {
                out.kappa_cmd =
                    clampValue(out.kappa_ff,
                               -pp_limit,
                               pp_limit);
            }
        }
        if (segment_straight) {
            out.kappa_ff = 0.0;
            out.kappa_pp = clampValue(out.kappa_pp,
                                      -kappa_pp_straight_limit_,
                                      kappa_pp_straight_limit_);
            out.kappa_cmd = 0.0;
            out.curve_blend = 0.0;
        }
        out.psi_used = out.psi;
        const bool straight_yaw_flip =
            std::fabs(out.kappa_cmd) < yaw_valid_kappa_gate_ &&
            std::fabs(out.y) < yaw_valid_lateral_gate_ &&
            std::fabs(out.psi) > yaw_flip_gate_;
        if (straight_yaw_flip) {
            out.psi_used = 0.0;
        } else if (yaw_conflict) {
            out.psi_used *= 0.35;
        }
        const double ksign =
            out.kappa_ff > 1e-4 ? 1.0 :
            (out.kappa_ff < -1e-4 ? -1.0 :
             (out.kappa_pp > 1e-4 ? 1.0 : (out.kappa_pp < -1e-4 ? -1.0 : 0.0)));
        out.inside_error_front = ksign == 0.0 ? 0.0 : std::max(0.0, ksign * out.front_axle_e_ct);
        out.inside_error = out.inside_error_front;
        const double inside_amount =
            clampValue((out.inside_error - inside_error_gate_) /
                           std::max(1e-4, inside_error_full_ - inside_error_gate_),
                       0.0,
                       1.0);
        out.corner_cut_score = inside_amount;
        out.inside_gain_scale = 1.0 + corner_cut_gain_ * inside_amount;
        out.ff_reduce_scale = 1.0 - inside_ff_reduce_max_ * inside_amount;
        out.kappa_pp_weighted = segment_straight ? 0.0 : out.kappa_cmd;
        out.curve_blend = segment_straight ? 0.0 : curveBlend(std::fabs(out.kappa_cmd));
        return out;
    }

    void applyStraightGuard(PreviewGeometrySignal* signal,
                            const std::string& reason) const {
        if (signal == NULL || !signal->valid) {
            return;
        }
        signal->straight_guard_active = true;
        signal->pp_suppressed_reason = reason.empty() ? "straight_guard" : reason;
        signal->kappa_pp = 0.0;
        signal->kappa_cmd = 0.0;
        signal->kappa_ff = 0.0;
        signal->kappa_pp_weighted = 0.0;
        signal->curve_blend = 0.0;
        signal->turn_blend_effective = 0.0;
        signal->ff_reduce_scale = 1.0;
        signal->inside_gain_scale = 1.0;
        signal->inside_error = 0.0;
        signal->inside_error_front = 0.0;
        signal->corner_cut_score = 0.0;
        signal->lookahead_mode = "straight_guard";
    }

    UnifiedControlOutput compute(const UnifiedTrackingSignal& tracking,
                                 const PreviewGeometrySignal& preview,
                                 double speed,
                                 double yaw_rate,
                                 double steering_residual,
                                 double bias,
                                 double slqr_delta) const {
        UnifiedControlOutput out;
        const double speed_abs = std::max(0.02, std::fabs(speed));
        const double kappa = preview.valid ? preview.kappa_cmd : tracking.kappa_ref;
        const double psi = preview.valid ? preview.psi_used : tracking.e_course;
        const double lateral_error =
            preview.valid && front_axle_tracking_enabled_ ? preview.front_axle_e_ct :
            (preview.valid ? preview.cross_track_error : tracking.signed_e_y);
        const double ff_scale = preview.valid ? preview.ff_reduce_scale : 1.0;
        const double lateral_scale = preview.valid ? preview.inside_gain_scale : 1.0;
        out.delta_ff = ff_scale * std::atan(wheelbase_ * kappa);
        out.delta_course = clampValue(k_preview_yaw_ * psi, -yaw_delta_limit_, yaw_delta_limit_);
        double delta_lateral =
            -std::atan2(k_y_ * lateral_scale * lateral_error, speed_abs + k_soft_);
        out.delta_lateral_raw = delta_lateral;
        if (preview.valid && preview.straight_guard_active) {
            const double guarded =
                std::fabs(lateral_error) < straight_guard_lateral_deadband_
                    ? 0.0
                    : clampValue(delta_lateral,
                                 -straight_guard_lateral_limit_,
                                 straight_guard_lateral_limit_);
            out.delta_lateral_limited =
                out.delta_lateral_limited || std::fabs(guarded - delta_lateral) > 1e-5;
            delta_lateral = guarded;
        }
        if (preview.valid && preview.curve_blend > 0.15 &&
            std::fabs(out.delta_ff) > 0.04 &&
            delta_lateral * out.delta_ff < 0.0) {
            const double cancel_limit =
                lateral_cancel_limit_ratio_ * std::fabs(out.delta_ff) + lateral_cancel_limit_base_;
            const double limited = clampValue(delta_lateral, -cancel_limit, cancel_limit);
            out.delta_lateral_limited = std::fabs(limited - delta_lateral) > 1e-5;
            delta_lateral = limited;
        }
        out.delta_lateral = delta_lateral;
        out.delta_ey_rate = 0.0;
        out.delta_yaw = k_yaw_rate_ * (speed_abs * kappa - yaw_rate) -
                        k_feedback_ * steering_residual;
        double active_delta_max =
            preview.valid && preview.straight_guard_active
                ? std::min(delta_max_, straight_guard_delta_max_)
                : delta_max_;
        const double same_direction_yaw =
            (yaw_rate * (bias + out.delta_ff + out.delta_course + out.delta_lateral)) > 0.0
                ? std::fabs(yaw_rate)
                : 0.0;
        if (same_direction_yaw > yaw_oversteer_soft_gate_) {
            const double t = clampValue((same_direction_yaw - yaw_oversteer_soft_gate_) /
                                            std::max(1e-3, yaw_oversteer_hard_gate_ - yaw_oversteer_soft_gate_),
                                        0.0,
                                        1.0);
            active_delta_max *= (1.0 - (1.0 - yaw_oversteer_delta_scale_) * t);
        }
        out.delta_ref = clampValue(bias +
                                       out.delta_ff +
                                       out.delta_course +
                                       out.delta_lateral +
                                       out.delta_yaw,
                                   -active_delta_max,
                                   active_delta_max);
        const double curve_blend = preview.valid ? preview.curve_blend : tracking.curve_blend;
        out.slqr_weight = clampValue(slqr_max_weight_ * curve_blend, 0.0, slqr_max_weight_);
        out.delta_ref = (1.0 - out.slqr_weight) * out.delta_ref + out.slqr_weight * slqr_delta;
        out.delta_ref = clampValue(out.delta_ref, -active_delta_max, active_delta_max);
        return out;
    }

    double deltaMax() const {
        return delta_max_;
    }

    double slqrMaxWeight() const {
        return slqr_max_weight_;
    }

    double straightGuardKappaGate() const {
        return straight_guard_kappa_gate_;
    }

    double straightGuardCurveGate() const {
        return straight_guard_curve_gate_;
    }

    double straightGuardPreviewMinX() const {
        return straight_guard_preview_min_x_;
    }

    double straightGuardDeltaMax() const {
        return straight_guard_delta_max_;
    }

private:
    double smoothStep(double low, double high, double value) const {
        const double x = clampValue((value - low) / std::max(1e-6, high - low), 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    double pathYawCurvature(const std::vector<PathPoint>& pts, double target_s) const {
        if (pts.size() < 3) {
            return 0.0;
        }
        const double s0 = clampValue(target_s - kappa_path_window_, pts.front().s, pts.back().s);
        const double s1 = clampValue(target_s + kappa_path_window_, pts.front().s, pts.back().s);
        if (s1 - s0 < 0.12) {
            return 0.0;
        }
        PathPoint a;
        PathPoint b;
        if (!interpolate(pts, s0, a) || !interpolate(pts, s1, b)) {
            return 0.0;
        }
        return clampValue(wrapAngle(b.yaw - a.yaw) / std::max(1e-3, s1 - s0), -2.0, 2.0);
    }

    double pathCurvatureMedian(const std::vector<PathPoint>& pts, double s0, double s1) const {
        if (pts.empty() || s1 <= s0) {
            return 0.0;
        }
        std::vector<double> values;
        values.reserve(pts.size());
        for (const PathPoint& pt : pts) {
            if (pt.s >= s0 && pt.s <= s1 && std::isfinite(pt.curvature)) {
                values.push_back(clampValue(pt.curvature,
                                            -reference_kappa_abs_limit_,
                                            reference_kappa_abs_limit_));
            }
        }
        if (values.empty()) {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        return values[values.size() / 2];
    }

    bool interpolate(const std::vector<PathPoint>& pts, double s, PathPoint& out) const {
        if (pts.empty()) {
            return false;
        }
        if (s <= pts.front().s) {
            out = pts.front();
            return true;
        }
        for (size_t i = 1; i < pts.size(); ++i) {
            if (s <= pts[i].s) {
                const PathPoint& a = pts[i - 1];
                const PathPoint& b = pts[i];
                const double den = std::max(1e-6, b.s - a.s);
                const double t = clampValue((s - a.s) / den, 0.0, 1.0);
                out.s = s;
                out.x = a.x + t * (b.x - a.x);
                out.y = a.y + t * (b.y - a.y);
                out.yaw = wrapAngle(a.yaw + t * wrapAngle(b.yaw - a.yaw));
                out.curvature = a.curvature + t * (b.curvature - a.curvature);
                return true;
            }
        }
        out = pts.back();
        return true;
    }

    double wheelbase_ = 0.60;
    double preview_min_ = 0.45;
    double preview_max_ = 1.20;
    double preview_gain_ = 1.25;
    double curve_preview_ = 0.85;
    double curve_preview_min_ = 0.38;
    double curve_preview_max_ = 0.65;
    double curve_preview_kappa_low_ = 0.15;
    double curve_preview_kappa_high_ = 0.65;
    double ignore_front_s_ = 0.25;
    double kappa_path_window_ = 0.35;
    double reference_kappa_start_s_ = 0.15;
    double reference_kappa_end_s_ = 0.55;
    double reference_kappa_abs_limit_ = 2.00;
    double kappa_path_weight_ = 0.30;
    double yaw_delta_limit_ = 0.10;
    double yaw_flip_gate_ = 1.20;
    double yaw_valid_kappa_gate_ = 0.10;
    double yaw_valid_lateral_gate_ = 0.05;
    double k_y_ = 0.50;
    double k_pp_curve_ = 0.20;
    double inside_gain_ = 1.60;
    double inside_error_gate_ = 0.008;
    double inside_error_full_ = 0.040;
    double inside_ff_reduce_max_ = 0.35;
    double k_preview_yaw_ = 0.22;
    double k_yaw_rate_ = 0.42;
    double k_feedback_ = 0.12;
    double k_soft_ = 0.12;
    double delta_max_ = 0.28;
    double kappa_pp_abs_limit_ = 1.05;
    double kappa_pp_ref_margin_ = 0.35;
    double kappa_pp_straight_limit_ = 0.35;
    double sharp_entry_turn_blend_min_ = 0.85;
    double sharp_entry_kappa_gate_ = 1.00;
    double lateral_cancel_limit_ratio_ = 0.45;
    double lateral_cancel_limit_base_ = 0.08;
    double yaw_oversteer_soft_gate_ = 0.070;
    double yaw_oversteer_hard_gate_ = 0.115;
    double yaw_oversteer_delta_scale_ = 0.70;
    double slqr_max_weight_ = 0.15;
    double curve_blend_low_ = 0.12;
    double curve_blend_high_ = 0.35;
    double conflict_kappa_gate_ = 0.08;
    double turn_entry_stop_margin_ = 0.10;
    double entry_preview_min_ = 0.35;
    double entry_preview_max_ = 0.50;
    bool front_axle_tracking_enabled_ = true;
    double corner_cut_gain_ = 1.80;
    double curve_lookahead_inside_scale_ = 0.65;
    double straight_guard_kappa_gate_ = 0.12;
    double straight_guard_curve_gate_ = 0.20;
    double straight_guard_preview_min_x_ = 0.60;
    double straight_guard_lateral_deadband_ = 0.035;
    double straight_guard_lateral_limit_ = 0.06;
    double straight_guard_delta_max_ = 0.10;
};

class FlatnessErrorExtractorLegacy {
public:
    void configure(ros::NodeHandle& pnh) {
        horizon_steps_ = pnh.param("horizon_steps", 20);
        ds_ = pnh.param("spatial_step", 0.10);
        max_curvature_ = pnh.param("max_path_curvature", 1.50);
        min_segment_length_ = pnh.param("path_min_segment_length", 0.015);
        use_pose_orientation_ = pnh.param("use_path_pose_orientation", false);
        curvature_smooth_alpha_ = pnh.param("curvature_smooth_alpha", 0.25);
        projection_lateral_weight_ = pnh.param("projection_lateral_weight", 1.0);
    }

    PathBuffer ingestPath(const nav_msgs::Path& msg,
                          const ros::Time& now,
                          const std::string& source) const {
        PathBuffer buffer;
        buffer.stamp = msg.header.stamp.isZero() ? now : msg.header.stamp;
        buffer.source = source;
        buffer.held_reference = (source == "reference" && msg.header.seq == 1u);
        buffer.valid = false;

        std::vector<PathPoint> raw;
        raw.reserve(msg.poses.size());
        for (size_t i = 0; i < msg.poses.size(); ++i) {
            const double x = msg.poses[i].pose.position.x;
            const double y = msg.poses[i].pose.position.y;
            if (!finitePoint(x, y)) {
                continue;
            }
            if (!raw.empty() &&
                safeHypot(x - raw.back().x, y - raw.back().y) < min_segment_length_) {
                continue;
            }
            PathPoint pt;
            pt.x = x;
            pt.y = y;
            pt.yaw = yawFromQuaternion(msg.poses[i].pose.orientation);
            raw.push_back(pt);
        }

        if (raw.size() < 2) {
            buffer.points = raw;
            return buffer;
        }

        double accum_s = 0.0;
        raw.front().s = 0.0;
        for (size_t i = 1; i < raw.size(); ++i) {
            accum_s += safeHypot(raw[i].x - raw[i - 1].x, raw[i].y - raw[i - 1].y);
            raw[i].s = accum_s;
        }

        computeYaw(raw);
        computeCurvature(raw);
        buffer.points = raw;
        buffer.valid = buffer.points.size() >= 2 &&
                       (buffer.points.back().s - buffer.points.front().s) > min_segment_length_;
        return buffer;
    }

    bool freshEnough(const PathBuffer& path, const ros::Time& now, double timeout) const {
        if (!path.valid || path.points.size() < 2) {
            return false;
        }
        return (now - path.stamp).toSec() <= timeout;
    }

    PathTrackingProblem extract(const PathBuffer& path) const {
        PathTrackingProblem problem;
        if (!path.valid || path.points.size() < 2) {
            return problem;
        }

        int best_index = -1;
        double best_ratio = 0.0;
        double best_cost = std::numeric_limits<double>::infinity();
        PathPoint best_projection;

        for (size_t i = 1; i < path.points.size(); ++i) {
            const PathPoint& p0 = path.points[i - 1];
            const PathPoint& p1 = path.points[i];
            const double vx = p1.x - p0.x;
            const double vy = p1.y - p0.y;
            const double len2 = vx * vx + vy * vy;
            if (len2 < square(min_segment_length_)) {
                continue;
            }
            const double ratio = clampValue((-(p0.x * vx + p0.y * vy)) / len2, 0.0, 1.0);
            const double px = p0.x + ratio * vx;
            const double py = p0.y + ratio * vy;
            const double yaw = wrapAngle(p0.yaw + ratio * wrapAngle(p1.yaw - p0.yaw));
            const double lateral = std::fabs(-std::sin(yaw) * (-px) + std::cos(yaw) * (-py));
            const double forward_penalty = 0.20 * std::max(0.0, -px);
            const double cost = safeHypot(px, py) + projection_lateral_weight_ * lateral + forward_penalty;
            if (cost < best_cost) {
                best_cost = cost;
                best_index = static_cast<int>(i - 1);
                best_ratio = ratio;
                best_projection = interpolate(path.points, p0.s + ratio * (p1.s - p0.s));
            }
        }

        if (best_index < 0) {
            return problem;
        }

        const double dx_ego = -best_projection.x;
        const double dy_ego = -best_projection.y;
        problem.e_y =
            -std::sin(best_projection.yaw) * dx_ego + std::cos(best_projection.yaw) * dy_ego;
        problem.e_theta = wrapAngle(-best_projection.yaw);
        problem.projection_s = best_projection.s;
        problem.nearest_distance = best_cost;
        problem.path_span = path.points.back().s - path.points.front().s;
        problem.first_curvature = best_projection.curvature;
        problem.curvatures.reserve(static_cast<size_t>(horizon_steps_ + 1));
        for (int i = 0; i <= horizon_steps_; ++i) {
            const PathPoint sample = interpolate(path.points, best_projection.s + ds_ * static_cast<double>(i));
            problem.curvatures.push_back(sample.curvature);
        }
        problem.valid = problem.curvatures.size() >= 2;
        (void)best_ratio;
        return problem;
    }

private:
    void computeYaw(std::vector<PathPoint>& pts) const {
        if (pts.size() < 2) {
            return;
        }
        double previous_yaw = 0.0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const size_t i0 = (i == 0) ? 0 : i - 1;
            const size_t i1 = std::min(pts.size() - 1, i + 1);
            const double dx = pts[i1].x - pts[i0].x;
            const double dy = pts[i1].y - pts[i0].y;
            double geom_yaw = (std::fabs(dx) + std::fabs(dy) > 1e-6) ? std::atan2(dy, dx) : previous_yaw;
            if (use_pose_orientation_) {
                geom_yaw = wrapAngle(0.35 * pts[i].yaw + 0.65 * geom_yaw);
            }
            if (i > 0) {
                geom_yaw = wrapAngle(previous_yaw + wrapAngle(geom_yaw - previous_yaw));
            }
            pts[i].yaw = geom_yaw;
            previous_yaw = geom_yaw;
        }
    }

    void computeCurvature(std::vector<PathPoint>& pts) const {
        if (pts.size() < 3) {
            for (PathPoint& pt : pts) {
                pt.curvature = 0.0;
            }
            return;
        }
        std::vector<double> raw(pts.size(), 0.0);
        for (size_t i = 1; i + 1 < pts.size(); ++i) {
            const PathPoint& p0 = pts[i - 1];
            const PathPoint& p1 = pts[i];
            const PathPoint& p2 = pts[i + 1];
            const double a = safeHypot(p1.x - p0.x, p1.y - p0.y);
            const double b = safeHypot(p2.x - p1.x, p2.y - p1.y);
            const double c = safeHypot(p2.x - p0.x, p2.y - p0.y);
            const double cross =
                (p1.x - p0.x) * (p2.y - p1.y) -
                (p1.y - p0.y) * (p2.x - p1.x);
            const double area2 = std::fabs(cross);
            raw[i] = (a * b * c > 1e-8)
                         ? clampValue(2.0 * cross / (a * b * c), -max_curvature_, max_curvature_)
                         : 0.0;
            (void)area2;
        }
        raw.front() = raw[1];
        raw.back() = raw[raw.size() - 2];
        pts.front().curvature = raw.front();
        for (size_t i = 1; i < pts.size(); ++i) {
            pts[i].curvature =
                clampValue(curvature_smooth_alpha_ * raw[i] +
                               (1.0 - curvature_smooth_alpha_) * pts[i - 1].curvature,
                           -max_curvature_,
                           max_curvature_);
        }
    }

    PathPoint interpolate(const std::vector<PathPoint>& pts, double query_s) const {
        if (pts.empty()) {
            return PathPoint();
        }
        if (query_s <= pts.front().s) {
            return pts.front();
        }
        if (query_s >= pts.back().s) {
            return pts.back();
        }
        for (size_t i = 1; i < pts.size(); ++i) {
            if (query_s > pts[i].s) {
                continue;
            }
            const PathPoint& p0 = pts[i - 1];
            const PathPoint& p1 = pts[i];
            const double ds = std::max(1e-5, p1.s - p0.s);
            const double ratio = clampValue((query_s - p0.s) / ds, 0.0, 1.0);
            PathPoint out;
            out.s = query_s;
            out.x = p0.x + ratio * (p1.x - p0.x);
            out.y = p0.y + ratio * (p1.y - p0.y);
            out.yaw = wrapAngle(p0.yaw + ratio * wrapAngle(p1.yaw - p0.yaw));
            out.curvature = p0.curvature + ratio * (p1.curvature - p0.curvature);
            return out;
        }
        return pts.back();
    }

    int horizon_steps_ = 20;
    double ds_ = 0.10;
    double max_curvature_ = 1.50;
    double min_segment_length_ = 0.015;
    double curvature_smooth_alpha_ = 0.25;
    double projection_lateral_weight_ = 1.0;
    bool use_pose_orientation_ = false;
};

class FlatnessErrorExtractor {
public:
    void configure(ros::NodeHandle& pnh) {
        horizon_steps_ = pnh.param("horizon_steps", pnh.param("slqr_horizon_steps", 20));
        ds_ = pnh.param("spatial_step", 0.10);
        min_segment_length_ = pnh.param("path_min_segment_length", 0.015);
        max_curvature_ = pnh.param("max_path_curvature", 1.50);
        smooth_back_length_ = pnh.param("smooth_back_length", 0.30);
        smooth_front_length_ = pnh.param("smooth_front_length", 2.40);
        smooth_min_points_ = pnh.param("smooth_min_points", 6);
        smooth_huber_delta_ = pnh.param("smooth_huber_delta", 0.08);
        smooth_ridge_ = pnh.param("smooth_ridge", 1e-4);
        smooth_max_rms_ = pnh.param("smooth_max_rms", 0.35);
        smooth_newton_iterations_ = pnh.param("smooth_newton_iterations", 6);
        smooth_max_curvature_rate_ = pnh.param("smooth_max_curvature_rate", 1.80);
        projection_lateral_weight_ = pnh.param("projection_lateral_weight", 1.0);
        straight_fit_min_s_ = pnh.param("straight_fit_min_s", 0.40);
        straight_fit_max_s_ = pnh.param("straight_fit_max_s", 2.00);
        straight_fit_min_points_ = std::max(3, pnh.param("straight_fit_min_points", 5));
        straight_fit_huber_delta_ = pnh.param("straight_fit_huber_delta", 0.06);
    }

    PathBuffer ingestPath(const nav_msgs::Path& msg,
                          const ros::Time& now,
                          const std::string& source) const {
        PathBuffer buffer;
        buffer.stamp = msg.header.stamp.isZero() ? now : msg.header.stamp;
        buffer.source = source;
        buffer.held_reference = (source == "reference" && msg.header.seq == 1u);

        for (size_t i = 0; i < msg.poses.size(); ++i) {
            const double x = msg.poses[i].pose.position.x;
            const double y = msg.poses[i].pose.position.y;
            if (!finitePoint(x, y)) {
                continue;
            }
            if (!buffer.points.empty() &&
                safeHypot(x - buffer.points.back().x, y - buffer.points.back().y) < min_segment_length_) {
                continue;
            }
            PathPoint pt;
            pt.x = x;
            pt.y = y;
            buffer.points.push_back(pt);
        }

        if (buffer.points.size() < 2) {
            return buffer;
        }

        double accum_s = 0.0;
        buffer.points.front().s = 0.0;
        for (size_t i = 1; i < buffer.points.size(); ++i) {
            accum_s += safeHypot(buffer.points[i].x - buffer.points[i - 1].x,
                                 buffer.points[i].y - buffer.points[i - 1].y);
            buffer.points[i].s = accum_s;
        }
        computeRoughYaw(buffer.points);
        buffer.valid = accum_s > min_segment_length_;
        return buffer;
    }

    bool freshEnough(const PathBuffer& path, const ros::Time& now, double timeout) const {
        if (!path.valid || path.points.size() < 2) {
            return false;
        }
        return (now - path.stamp).toSec() <= timeout;
    }

    PathTrackingProblem extract(const PathBuffer& path) const {
        PathTrackingProblem problem;
        if (!path.valid || path.points.size() < 2) {
            return problem;
        }

        double rough_s = 0.0;
        double rough_cost = 0.0;
        if (!roughProject(path.points, rough_s, rough_cost)) {
            return problem;
        }

        CubicFit fit;
        if (!fitCubic(path.points, rough_s, fit)) {
            return problem;
        }

        double u = clampValue(rough_s - fit.s0, fit.u_min, fit.u_max);
        for (int i = 0; i < smooth_newton_iterations_; ++i) {
            CurveEval e = evaluate(fit, u);
            const double f = e.x * e.dx + e.y * e.dy;
            const double df = e.dx * e.dx + e.x * e.ddx + e.dy * e.dy + e.y * e.ddy;
            if (std::fabs(df) < 1e-8) {
                break;
            }
            const double step = clampValue(f / df, -0.25, 0.25);
            u = clampValue(u - step, fit.u_min, fit.u_max);
            if (std::fabs(step) < 1e-5) {
                break;
            }
        }

        const CurveEval proj = evaluate(fit, u);
        const double dx_ego = -proj.x;
        const double dy_ego = -proj.y;
        problem.e_y = -std::sin(proj.yaw) * dx_ego + std::cos(proj.yaw) * dy_ego;
        problem.e_theta = wrapAngle(-proj.yaw);
        problem.projection_s = fit.s0 + u;
        problem.nearest_distance = rough_cost;
        problem.path_span = path.points.back().s - path.points.front().s;
        problem.first_curvature = proj.curvature;
        problem.fit_rms = fit.rms;
        problem.fit_valid = fit.rms <= smooth_max_rms_ || fit.sample_count < smooth_min_points_;
        computeForwardLineFit(path.points, problem.projection_s, problem);

        problem.curvatures.reserve(static_cast<size_t>(horizon_steps_ + 1));
        double last_kappa = proj.curvature;
        const double max_step = std::max(0.01, smooth_max_curvature_rate_ * ds_);
        for (int i = 0; i <= horizon_steps_; ++i) {
            const double query_u = clampValue(u + ds_ * static_cast<double>(i), fit.u_min, fit.u_max);
            double kappa = evaluate(fit, query_u).curvature;
            if (i > 0) {
                kappa = clampValue(kappa, last_kappa - max_step, last_kappa + max_step);
            }
            kappa = clampValue(kappa, -max_curvature_, max_curvature_);
            problem.curvatures.push_back(kappa);
            last_kappa = kappa;
        }
        problem.valid = problem.curvatures.size() >= 2;
        return problem;
    }

private:
    struct CubicFit {
        bool valid = false;
        double s0 = 0.0;
        double u_min = 0.0;
        double u_max = 0.0;
        double rms = 0.0;
        int sample_count = 0;
        Eigen::Vector4d cx = Eigen::Vector4d::Zero();
        Eigen::Vector4d cy = Eigen::Vector4d::Zero();
    };

    struct CurveEval {
        double x = 0.0;
        double y = 0.0;
        double dx = 0.0;
        double dy = 0.0;
        double ddx = 0.0;
        double ddy = 0.0;
        double yaw = 0.0;
        double curvature = 0.0;
    };

    void computeForwardLineFit(const std::vector<PathPoint>& pts,
                               double projection_s,
                               PathTrackingProblem& problem) const {
        std::vector<const PathPoint*> samples;
        samples.reserve(pts.size());
        const double min_s = std::max(0.0, straight_fit_min_s_);
        const double max_s = std::max(min_s + 0.20, straight_fit_max_s_);
        for (const PathPoint& pt : pts) {
            const double rel_s = pt.s - projection_s;
            if (rel_s >= min_s && rel_s <= max_s) {
                samples.push_back(&pt);
            }
        }
        if (static_cast<int>(samples.size()) < straight_fit_min_points_) {
            samples.clear();
            for (const PathPoint& pt : pts) {
                const double rel_s = pt.s - projection_s;
                if (rel_s >= 0.15 && rel_s <= max_s) {
                    samples.push_back(&pt);
                }
            }
        }
        if (static_cast<int>(samples.size()) < 2) {
            problem.line_fit_rms = 1e3;
            problem.line_yaw = problem.e_theta;
            problem.line_span = 0.0;
            problem.straight_confidence = 0.0;
            return;
        }

        std::vector<double> weights(samples.size(), 1.0);
        Eigen::Vector2d direction(1.0, 0.0);
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (int iter = 0; iter < 4; ++iter) {
            double w_sum = 0.0;
            mean.setZero();
            for (size_t i = 0; i < samples.size(); ++i) {
                mean += weights[i] * Eigen::Vector2d(samples[i]->x, samples[i]->y);
                w_sum += weights[i];
            }
            if (w_sum < 1e-6) {
                break;
            }
            mean /= w_sum;

            Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
            for (size_t i = 0; i < samples.size(); ++i) {
                const Eigen::Vector2d d(samples[i]->x - mean.x(), samples[i]->y - mean.y());
                cov += weights[i] * (d * d.transpose());
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
            if (eig.info() != Eigen::Success) {
                break;
            }
            direction = eig.eigenvectors().col(1).normalized();
            const Eigen::Vector2d chord(samples.back()->x - samples.front()->x,
                                        samples.back()->y - samples.front()->y);
            if (direction.dot(chord) < 0.0) {
                direction = -direction;
            }
            if (direction.x() < -0.05) {
                direction = -direction;
            }
            const Eigen::Vector2d normal(-direction.y(), direction.x());
            for (size_t i = 0; i < samples.size(); ++i) {
                const Eigen::Vector2d d(samples[i]->x - mean.x(), samples[i]->y - mean.y());
                const double residual = std::fabs(normal.dot(d));
                weights[i] = huberWeight(residual, straight_fit_huber_delta_);
            }
        }

        const Eigen::Vector2d normal(-direction.y(), direction.x());
        double err2 = 0.0;
        double w_sum = 0.0;
        for (size_t i = 0; i < samples.size(); ++i) {
            const Eigen::Vector2d d(samples[i]->x - mean.x(), samples[i]->y - mean.y());
            const double residual = normal.dot(d);
            err2 += weights[i] * residual * residual;
            w_sum += weights[i];
        }
        problem.line_fit_rms = w_sum > 1e-6 ? std::sqrt(err2 / w_sum) : 1e3;
        problem.line_yaw = wrapAngle(std::atan2(direction.y(), direction.x()));
        problem.line_span = samples.back()->s - samples.front()->s;
        const double rms_score = clampValue(1.0 - problem.line_fit_rms / 0.08, 0.0, 1.0);
        const double yaw_score = clampValue(1.0 - std::fabs(problem.line_yaw) / 0.35, 0.0, 1.0);
        const double span_score = clampValue(problem.line_span / 0.80, 0.0, 1.0);
        problem.straight_confidence = rms_score * yaw_score * span_score;
    }

    void computeRoughYaw(std::vector<PathPoint>& pts) const {
        double previous_yaw = 0.0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const size_t i0 = (i == 0) ? 0 : i - 1;
            const size_t i1 = std::min(pts.size() - 1, i + 1);
            const double dx = pts[i1].x - pts[i0].x;
            const double dy = pts[i1].y - pts[i0].y;
            double yaw = (std::fabs(dx) + std::fabs(dy) > 1e-6) ? std::atan2(dy, dx) : previous_yaw;
            if (i > 0) {
                yaw = wrapAngle(previous_yaw + wrapAngle(yaw - previous_yaw));
            }
            pts[i].yaw = yaw;
            previous_yaw = yaw;
        }
    }

    bool roughProject(const std::vector<PathPoint>& pts, double& best_s, double& best_cost) const {
        best_cost = std::numeric_limits<double>::infinity();
        best_s = 0.0;
        bool found = false;
        for (size_t i = 1; i < pts.size(); ++i) {
            const PathPoint& p0 = pts[i - 1];
            const PathPoint& p1 = pts[i];
            const double vx = p1.x - p0.x;
            const double vy = p1.y - p0.y;
            const double len2 = vx * vx + vy * vy;
            if (len2 < square(min_segment_length_)) {
                continue;
            }
            const double r = clampValue((-(p0.x * vx + p0.y * vy)) / len2, 0.0, 1.0);
            const double px = p0.x + r * vx;
            const double py = p0.y + r * vy;
            const double yaw = wrapAngle(p0.yaw + r * wrapAngle(p1.yaw - p0.yaw));
            const double lateral = std::fabs(-std::sin(yaw) * (-px) + std::cos(yaw) * (-py));
            const double forward_penalty = 0.20 * std::max(0.0, -px);
            const double cost = safeHypot(px, py) + projection_lateral_weight_ * lateral + forward_penalty;
            if (cost < best_cost) {
                best_cost = cost;
                best_s = p0.s + r * (p1.s - p0.s);
                found = true;
            }
        }
        return found;
    }

    bool fitCubic(const std::vector<PathPoint>& pts, double center_s, CubicFit& fit) const {
        std::vector<const PathPoint*> samples;
        samples.reserve(pts.size());
        for (const PathPoint& pt : pts) {
            if (pt.s >= center_s - smooth_back_length_ &&
                pt.s <= center_s + smooth_front_length_) {
                samples.push_back(&pt);
            }
        }
        if (static_cast<int>(samples.size()) < smooth_min_points_) {
            samples.clear();
            for (const PathPoint& pt : pts) {
                if (pt.s >= center_s - 2.0 * smooth_back_length_ &&
                    pt.s <= center_s + 2.0 * smooth_front_length_) {
                    samples.push_back(&pt);
                }
            }
        }
        if (samples.size() < 2) {
            return false;
        }

        fit.s0 = center_s;
        fit.u_min = samples.front()->s - center_s;
        fit.u_max = samples.back()->s - center_s;
        fit.sample_count = static_cast<int>(samples.size());

        std::vector<double> weights(samples.size(), 1.0);
        for (int iter = 0; iter < 3; ++iter) {
            Eigen::Matrix4d h = Eigen::Matrix4d::Zero();
            Eigen::Vector4d bx = Eigen::Vector4d::Zero();
            Eigen::Vector4d by = Eigen::Vector4d::Zero();
            for (size_t i = 0; i < samples.size(); ++i) {
                const double u = samples[i]->s - center_s;
                Eigen::Vector4d a;
                a << 1.0, u, u * u, u * u * u;
                const double w = weights[i] / (1.0 + 0.15 * std::fabs(u));
                h += w * (a * a.transpose());
                bx += w * a * samples[i]->x;
                by += w * a * samples[i]->y;
            }
            h(1, 1) += smooth_ridge_;
            h(2, 2) += 4.0 * smooth_ridge_;
            h(3, 3) += 9.0 * smooth_ridge_;
            Eigen::LDLT<Eigen::Matrix4d> ldlt(h);
            if (ldlt.info() != Eigen::Success) {
                return false;
            }
            fit.cx = ldlt.solve(bx);
            fit.cy = ldlt.solve(by);
            if (!fit.cx.allFinite() || !fit.cy.allFinite()) {
                return false;
            }

            for (size_t i = 0; i < samples.size(); ++i) {
                const double u = samples[i]->s - center_s;
                const CurveEval e = evaluate(fit, u);
                const double residual = safeHypot(e.x - samples[i]->x, e.y - samples[i]->y);
                weights[i] = huberWeight(residual, smooth_huber_delta_);
            }
        }

        double err2 = 0.0;
        double weight_sum = 0.0;
        for (size_t i = 0; i < samples.size(); ++i) {
            const double u = samples[i]->s - center_s;
            const CurveEval e = evaluate(fit, u);
            const double residual = safeHypot(e.x - samples[i]->x, e.y - samples[i]->y);
            const double w = weights[i] / (1.0 + 0.15 * std::fabs(u));
            err2 += w * residual * residual;
            weight_sum += w;
        }
        fit.rms = weight_sum > 1e-8 ? std::sqrt(err2 / weight_sum) : 0.0;
        fit.valid = true;
        return true;
    }

    double huberWeight(double residual, double delta) const {
        if (delta <= 1e-6 || residual <= delta) {
            return 1.0;
        }
        return delta / std::max(delta, residual);
    }

    CurveEval evaluate(const CubicFit& fit, double u) const {
        CurveEval e;
        const double u2 = u * u;
        const double u3 = u2 * u;
        e.x = fit.cx[0] + fit.cx[1] * u + fit.cx[2] * u2 + fit.cx[3] * u3;
        e.y = fit.cy[0] + fit.cy[1] * u + fit.cy[2] * u2 + fit.cy[3] * u3;
        e.dx = fit.cx[1] + 2.0 * fit.cx[2] * u + 3.0 * fit.cx[3] * u2;
        e.dy = fit.cy[1] + 2.0 * fit.cy[2] * u + 3.0 * fit.cy[3] * u2;
        e.ddx = 2.0 * fit.cx[2] + 6.0 * fit.cx[3] * u;
        e.ddy = 2.0 * fit.cy[2] + 6.0 * fit.cy[3] * u;
        e.yaw = std::atan2(e.dy, e.dx);
        const double speed2 = e.dx * e.dx + e.dy * e.dy;
        const double denom = std::pow(std::max(1e-6, speed2), 1.5);
        e.curvature = clampValue((e.dx * e.ddy - e.dy * e.ddx) / denom, -max_curvature_, max_curvature_);
        return e;
    }

    int horizon_steps_ = 20;
    int smooth_min_points_ = 6;
    int smooth_newton_iterations_ = 6;
    double ds_ = 0.10;
    double min_segment_length_ = 0.015;
    double max_curvature_ = 1.50;
    double smooth_back_length_ = 0.30;
    double smooth_front_length_ = 2.40;
    double smooth_huber_delta_ = 0.08;
    double smooth_ridge_ = 1e-4;
    double smooth_max_rms_ = 0.35;
    double smooth_max_curvature_rate_ = 1.80;
    double projection_lateral_weight_ = 1.0;
    double straight_fit_min_s_ = 0.40;
    double straight_fit_max_s_ = 2.00;
    int straight_fit_min_points_ = 5;
    double straight_fit_huber_delta_ = 0.06;
};

class CustomVelocityPlanner {
public:
    void configure(ros::NodeHandle& pnh) {
        horizon_steps_ = pnh.param("horizon_steps", pnh.param("slqr_horizon_steps", 20));
        ds_ = pnh.param("spatial_step", 0.10);
        v_min_ = pnh.param("v_min", 0.10);
        v_max_ = pnh.param("max_speed", 0.50);
        v_cruise_ = pnh.param("cruise_speed", 0.32);
        friction_mu_ = pnh.param("friction_mu_base", 0.45);
        gravity_ = pnh.param("gravity", 9.80665);
        curvature_epsilon_ = pnh.param("curvature_epsilon", 0.03);
        cps_global_speed_gain_ = pnh.param("cps_global_speed_gain", 0.45);
        sqp_max_iter_ = std::max(1, pnh.param("sqp_max_iter", 4));
        friction_circle_margin_ = clampValue(pnh.param("sqp_friction_circle_margin", 0.82), 0.10, 1.0);
        min_curve_speed_ = clampValue(pnh.param("sqp_min_curve_speed", 0.06), 0.0, v_max_);
        w_time_ = pnh.param("sqp_w_time", 0.25);
        w_v_ = pnh.param("sqp_w_v", 6.0);
        w_accel_ = pnh.param("sqp_w_accel", 1.8);
        w_jerk_ = pnh.param("sqp_w_jerk", 6.0);
        w_initial_ = pnh.param("sqp_w_initial", 9.0);
        desired_follow_distance_ = pnh.param("desired_follow_distance", 2.0);
        cth_time_headway_ = pnh.param("cth_time_headway", 0.50);
        cth_kp_ = pnh.param("cth_kp", 0.22);
        cth_kd_ = pnh.param("cth_kd", 0.10);
        close_gap_slowdown_gain_ = pnh.param("cth_close_gap_slowdown_gain", 0.40);
        hard_stop_distance_ = pnh.param("hard_stop_distance", 0.50);
        lost_target_speed_scale_ = pnh.param("lost_target_speed_scale", 0.75);
        close_gap_min_speed_scale_ = pnh.param("close_gap_min_speed_scale", 0.45);
    }

    std::vector<double> plan(const std::vector<double>& curvatures,
                             double gamma_cps,
                             double current_speed,
                             const LeadTargetState& lead,
                             bool hard_stop,
                             double& target_speed_out,
                             double& lead_speed_out,
                             double& gap_error_out) {
        const int n = std::max(2, std::min(static_cast<int>(curvatures.size()), horizon_steps_ + 1));
        std::vector<double> lower(static_cast<size_t>(n), square(v_min_));
        std::vector<double> upper(static_cast<size_t>(n), square(v_max_));

        const double ego_speed = clampValue(std::fabs(current_speed), 0.0, v_max_);
        double target_speed = v_cruise_;
        lead_speed_out = 0.0;
        gap_error_out = 0.0;
        if (lead.valid) {
            lead_speed_out = clampValue(ego_speed + lead.range_rate, 0.0, v_max_);
            gap_error_out = lead.distance - desired_follow_distance_ - cth_time_headway_ * ego_speed;
            target_speed = lead_speed_out + cth_kp_ * gap_error_out + cth_kd_ * lead.range_rate;
            if (gap_error_out < 0.0) {
                const double scale = clampValue(
                    1.0 + close_gap_slowdown_gain_ * gap_error_out,
                    close_gap_min_speed_scale_,
                    1.0);
                target_speed *= scale;
            }
        } else {
            target_speed = v_cruise_ * lost_target_speed_scale_;
        }
        const double cps_scale = clampValue(1.0 - cps_global_speed_gain_ * gamma_cps, 0.35, 1.0);
        const double cps_speed_cap = v_max_ * cps_scale;
        const double mu_eff = std::max(0.05, friction_mu_ * cps_scale);
        const double accel_bound = std::max(0.05, mu_eff * gravity_ * friction_circle_margin_);
        target_speed = clampValue(target_speed, v_min_, cps_speed_cap);
        target_speed_out = target_speed;
        last_jerk_cost_ = 0.0;

        if (hard_stop) {
            return std::vector<double>(static_cast<size_t>(n), 0.0);
        }

        for (int i = 0; i < n; ++i) {
            const double kappa = std::fabs(curvatures[static_cast<size_t>(i)]);
            double b_max = square(cps_speed_cap);
            if (kappa > curvature_epsilon_) {
                const double lateral_limit = accel_bound / std::max(curvature_epsilon_, kappa);
                b_max = std::min(b_max, lateral_limit);
            }
            b_max = std::max(b_max, square(std::min(v_max_, std::max(0.0, min_curve_speed_))));
            if (lead.valid && lead.distance < desired_follow_distance_) {
                const double closeness = clampValue(
                    (desired_follow_distance_ - lead.distance) /
                        std::max(0.05, desired_follow_distance_ - hard_stop_distance_),
                    0.0,
                    1.0);
                const double close_limit =
                    v_min_ + (v_max_ - v_min_) * (1.0 - 0.75 * closeness);
                b_max = std::min(b_max, square(close_limit));
            }
            upper[static_cast<size_t>(i)] = std::max(square(v_min_), b_max);
        }

        const double current_b = square(clampValue(std::fabs(current_speed), v_min_, v_max_));
        std::vector<double> feasible_upper = upper;
        for (int iter = 0; iter < sqp_max_iter_; ++iter) {
            for (int i = 0; i + 1 < n; ++i) {
                const double ay = std::fabs(feasible_upper[static_cast<size_t>(i)] *
                                            curvatures[static_cast<size_t>(i)]);
                const double a_long =
                    std::sqrt(std::max(0.0, square(accel_bound) - ay * ay));
                feasible_upper[static_cast<size_t>(i + 1)] =
                    std::min(feasible_upper[static_cast<size_t>(i + 1)],
                             feasible_upper[static_cast<size_t>(i)] + 2.0 * ds_ * a_long);
            }
            for (int i = n - 1; i > 0; --i) {
                const double ay = std::fabs(feasible_upper[static_cast<size_t>(i)] *
                                            curvatures[static_cast<size_t>(i)]);
                const double a_long =
                    std::sqrt(std::max(0.0, square(accel_bound) - ay * ay));
                feasible_upper[static_cast<size_t>(i - 1)] =
                    std::min(feasible_upper[static_cast<size_t>(i - 1)],
                             feasible_upper[static_cast<size_t>(i)] + 2.0 * ds_ * a_long);
            }
            for (int i = 1; i + 1 < n; ++i) {
                const double smooth =
                    0.25 * feasible_upper[static_cast<size_t>(i - 1)] +
                    0.50 * feasible_upper[static_cast<size_t>(i)] +
                    0.25 * feasible_upper[static_cast<size_t>(i + 1)];
                feasible_upper[static_cast<size_t>(i)] =
                    clampValue(smooth, lower[static_cast<size_t>(i)], upper[static_cast<size_t>(i)]);
            }
        }
        upper = feasible_upper;

        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n, n);
        Eigen::VectorXd f = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < n; ++i) {
            const double speed_time_bias =
                w_time_ / std::max(square(v_min_), square(target_speed));
            h(i, i) += w_v_ + speed_time_bias;
            f(i) += -(w_v_ * square(target_speed) + speed_time_bias * upper[static_cast<size_t>(i)]);
        }
        h(0, 0) += w_initial_;
        f(0) += -w_initial_ * current_b;

        const double inv_ds = 1.0 / std::max(0.03, ds_);
        for (int i = 0; i + 1 < n; ++i) {
            const double w = w_accel_ * inv_ds * inv_ds;
            h(i, i) += w;
            h(i + 1, i + 1) += w;
            h(i, i + 1) -= w;
            h(i + 1, i) -= w;
        }
        for (int i = 1; i + 1 < n; ++i) {
            const double w = w_jerk_ * inv_ds * inv_ds;
            h(i - 1, i - 1) += w;
            h(i, i) += 4.0 * w;
            h(i + 1, i + 1) += w;
            h(i - 1, i) -= 2.0 * w;
            h(i, i - 1) -= 2.0 * w;
            h(i, i + 1) -= 2.0 * w;
            h(i + 1, i) -= 2.0 * w;
            h(i - 1, i + 1) += w;
            h(i + 1, i - 1) += w;
        }
        for (int i = 0; i < n; ++i) {
            h(i, i) += 1e-6;
        }

        Eigen::VectorXd b(n);
        for (int i = 0; i < n; ++i) {
            b(i) = clampValue(square(target_speed), lower[static_cast<size_t>(i)], upper[static_cast<size_t>(i)]);
        }

        std::vector<bool> fixed(static_cast<size_t>(n), false);
        for (int iter = 0; iter < 8; ++iter) {
            std::vector<int> free_indices;
            free_indices.reserve(static_cast<size_t>(n));
            for (int i = 0; i < n; ++i) {
                if (!fixed[static_cast<size_t>(i)]) {
                    free_indices.push_back(i);
                }
            }
            if (free_indices.empty()) {
                break;
            }

            const int nf = static_cast<int>(free_indices.size());
            Eigen::MatrixXd hff = Eigen::MatrixXd::Zero(nf, nf);
            Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nf);
            for (int r = 0; r < nf; ++r) {
                const int ir = free_indices[static_cast<size_t>(r)];
                rhs(r) = -f(ir);
                for (int j = 0; j < n; ++j) {
                    if (fixed[static_cast<size_t>(j)]) {
                        rhs(r) -= h(ir, j) * b(j);
                    }
                }
                for (int c = 0; c < nf; ++c) {
                    const int ic = free_indices[static_cast<size_t>(c)];
                    hff(r, c) = h(ir, ic);
                }
            }

            Eigen::VectorXd sol = hff.ldlt().solve(rhs);
            bool added_fixed = false;
            for (int r = 0; r < nf; ++r) {
                const int idx = free_indices[static_cast<size_t>(r)];
                const double clipped =
                    clampValue(sol(r), lower[static_cast<size_t>(idx)], upper[static_cast<size_t>(idx)]);
                b(idx) = clipped;
                if (std::fabs(clipped - sol(r)) > 1e-7) {
                    fixed[static_cast<size_t>(idx)] = true;
                    added_fixed = true;
                }
            }
            if (!added_fixed) {
                break;
            }
        }

        std::vector<double> speeds(static_cast<size_t>(n), v_min_);
        for (int i = 0; i < n; ++i) {
            const double bi = clampValue(b(i), lower[static_cast<size_t>(i)], upper[static_cast<size_t>(i)]);
            speeds[static_cast<size_t>(i)] = std::sqrt(std::max(0.0, bi));
        }
        for (int i = 1; i + 1 < n; ++i) {
            const double a_prev =
                (square(speeds[static_cast<size_t>(i)]) - square(speeds[static_cast<size_t>(i - 1)])) /
                (2.0 * std::max(0.03, ds_));
            const double a_next =
                (square(speeds[static_cast<size_t>(i + 1)]) - square(speeds[static_cast<size_t>(i)])) /
                (2.0 * std::max(0.03, ds_));
            last_jerk_cost_ += square((a_next - a_prev) / std::max(0.03, ds_));
        }
        return speeds;
    }

    double lastJerkCost() const {
        return last_jerk_cost_;
    }

private:
    int horizon_steps_ = 20;
    double ds_ = 0.10;
    double v_min_ = 0.10;
    double v_max_ = 0.50;
    double v_cruise_ = 0.32;
    double friction_mu_ = 0.45;
    double gravity_ = 9.80665;
    double curvature_epsilon_ = 0.03;
    double cps_global_speed_gain_ = 0.45;
    int sqp_max_iter_ = 4;
    double friction_circle_margin_ = 0.82;
    double min_curve_speed_ = 0.06;
    double w_time_ = 0.25;
    double w_v_ = 6.0;
    double w_accel_ = 1.8;
    double w_jerk_ = 6.0;
    double w_initial_ = 9.0;
    double desired_follow_distance_ = 2.0;
    double cth_time_headway_ = 0.50;
    double cth_kp_ = 0.22;
    double cth_kd_ = 0.10;
    double close_gap_slowdown_gain_ = 0.40;
    double hard_stop_distance_ = 0.50;
    double lost_target_speed_scale_ = 0.75;
    double close_gap_min_speed_scale_ = 0.45;
    double last_jerk_cost_ = 0.0;
};

class SpatialNMPCLegacy {
public:
    void configure(ros::NodeHandle& pnh) {
        horizon_steps_ = pnh.param("horizon_steps", 20);
        ds_ = pnh.param("spatial_step", 0.10);
        wheelbase_ = pnh.param("wheelbase", 0.60);
        max_steer_ = pnh.param("max_steer", 0.785398);
        max_sequence_delta_step_ = pnh.param("nmpc_max_sequence_delta_step", 0.18);
        iterations_ = pnh.param("nmpc_iterations", 4);
        gradient_epsilon_ = pnh.param("nmpc_gradient_epsilon", 1e-4);
        step_size_ = pnh.param("nmpc_step_size", 0.18);
        q_y_ = pnh.param("nmpc_q_y", 36.0);
        q_theta_ = pnh.param("nmpc_q_theta", 14.0);
        q_terminal_y_ = pnh.param("nmpc_q_terminal_y", 70.0);
        q_terminal_theta_ = pnh.param("nmpc_q_terminal_theta", 28.0);
        r_delta_ = pnh.param("nmpc_r_delta", 0.45);
        r_delta_rate_ = pnh.param("nmpc_r_delta_rate", 8.0);
        r_feedforward_ = pnh.param("nmpc_r_feedforward", 1.8);
        error_guard_y_ = pnh.param("nmpc_error_guard_y", 3.0);
        error_guard_theta_ = pnh.param("nmpc_error_guard_theta", 1.40);
        controls_.assign(static_cast<size_t>(std::max(2, horizon_steps_)), 0.0);
    }

    double solve(double e_y,
                 double e_theta,
                 const std::vector<double>& curvatures,
                 double previous_delta,
                 double& delta_uncompensated) {
        const int n = std::max(2, std::min(static_cast<int>(curvatures.size()), horizon_steps_));
        if (static_cast<int>(controls_.size()) != n) {
            controls_.assign(static_cast<size_t>(n), previous_delta);
        } else {
            for (int i = 0; i + 1 < n; ++i) {
                controls_[static_cast<size_t>(i)] = controls_[static_cast<size_t>(i + 1)];
            }
            controls_.back() = controls_[controls_.size() - 2];
        }

        for (int i = 0; i < n; ++i) {
            const double ff = feedforward(curvatures[static_cast<size_t>(i)]);
            const double blend = (i == 0) ? 0.35 : 0.15;
            controls_[static_cast<size_t>(i)] =
                clampValue((1.0 - blend) * controls_[static_cast<size_t>(i)] + blend * ff,
                           -max_steer_,
                           max_steer_);
        }
        projectControls(controls_, previous_delta);

        double base_cost = rolloutCost(e_y, e_theta, curvatures, controls_, previous_delta);
        for (int iter = 0; iter < iterations_; ++iter) {
            std::vector<double> gradient(static_cast<size_t>(n), 0.0);
            for (int k = 0; k < n; ++k) {
                std::vector<double> trial = controls_;
                trial[static_cast<size_t>(k)] =
                    clampValue(trial[static_cast<size_t>(k)] + gradient_epsilon_, -max_steer_, max_steer_);
                projectControls(trial, previous_delta);
                const double c_plus = rolloutCost(e_y, e_theta, curvatures, trial, previous_delta);
                gradient[static_cast<size_t>(k)] = (c_plus - base_cost) / gradient_epsilon_;
            }

            bool accepted = false;
            double alpha = step_size_;
            for (int ls = 0; ls < 5; ++ls) {
                std::vector<double> candidate = controls_;
                for (int k = 0; k < n; ++k) {
                    candidate[static_cast<size_t>(k)] =
                        clampValue(candidate[static_cast<size_t>(k)] - alpha * gradient[static_cast<size_t>(k)],
                                   -max_steer_,
                                   max_steer_);
                }
                projectControls(candidate, previous_delta);
                const double c = rolloutCost(e_y, e_theta, curvatures, candidate, previous_delta);
                if (c < base_cost) {
                    controls_ = candidate;
                    base_cost = c;
                    accepted = true;
                    break;
                }
                alpha *= 0.45;
            }
            if (!accepted) {
                break;
            }
        }

        delta_uncompensated = clampValue(controls_.front(), -max_steer_, max_steer_);
        return delta_uncompensated;
    }

private:
    double feedforward(double curvature) const {
        return clampValue(std::atan(wheelbase_ * curvature), -max_steer_, max_steer_);
    }

    void stepState(double curvature, double delta, double& e_y, double& e_theta) const {
        const double ey = clampValue(e_y, -error_guard_y_, error_guard_y_);
        const double eth = clampValue(e_theta, -error_guard_theta_, error_guard_theta_);
        const double cos_eth = std::max(0.18, std::fabs(std::cos(eth)));
        const double geometry = clampValue(1.0 - curvature * ey, 0.20, 2.80);
        const double ey_prime = geometry * std::tan(eth);
        const double etheta_prime = std::tan(delta) / std::max(0.05, wheelbase_) *
                                        geometry / cos_eth -
                                    curvature;
        e_y = clampValue(e_y + ds_ * ey_prime, -error_guard_y_, error_guard_y_);
        e_theta = wrapAngle(e_theta + ds_ * etheta_prime);
        e_theta = clampValue(e_theta, -error_guard_theta_, error_guard_theta_);
    }

    double rolloutCost(double e_y,
                       double e_theta,
                       const std::vector<double>& curvatures,
                       const std::vector<double>& controls,
                       double previous_delta) const {
        const int n = static_cast<int>(controls.size());
        double cost = 0.0;
        double last_delta = previous_delta;
        double ey = e_y;
        double eth = e_theta;
        for (int k = 0; k < n; ++k) {
            const double kappa = curvatures[static_cast<size_t>(std::min(k, static_cast<int>(curvatures.size()) - 1))];
            const double delta = controls[static_cast<size_t>(k)];
            stepState(kappa, delta, ey, eth);
            const double ff = feedforward(kappa);
            cost += q_y_ * ey * ey + q_theta_ * eth * eth;
            cost += r_delta_ * delta * delta;
            cost += r_delta_rate_ * square(delta - last_delta);
            cost += r_feedforward_ * square(delta - ff);
            last_delta = delta;
        }
        cost += q_terminal_y_ * ey * ey + q_terminal_theta_ * eth * eth;
        return cost;
    }

    void projectControls(std::vector<double>& controls, double previous_delta) const {
        double last = previous_delta;
        for (double& delta : controls) {
            delta = clampValue(delta, -max_steer_, max_steer_);
            delta = clampValue(delta, last - max_sequence_delta_step_, last + max_sequence_delta_step_);
            delta = clampValue(delta, -max_steer_, max_steer_);
            last = delta;
        }
        for (int pass = 0; pass < 2; ++pass) {
            for (int i = static_cast<int>(controls.size()) - 2; i >= 0; --i) {
                controls[static_cast<size_t>(i)] =
                    clampValue(controls[static_cast<size_t>(i)],
                               controls[static_cast<size_t>(i + 1)] - max_sequence_delta_step_,
                               controls[static_cast<size_t>(i + 1)] + max_sequence_delta_step_);
            }
        }
    }

    std::vector<double> controls_;
    int horizon_steps_ = 20;
    int iterations_ = 4;
    double ds_ = 0.10;
    double wheelbase_ = 0.60;
    double max_steer_ = 0.785398;
    double max_sequence_delta_step_ = 0.18;
    double gradient_epsilon_ = 1e-4;
    double step_size_ = 0.18;
    double q_y_ = 36.0;
    double q_theta_ = 14.0;
    double q_terminal_y_ = 70.0;
    double q_terminal_theta_ = 28.0;
    double r_delta_ = 0.45;
    double r_delta_rate_ = 8.0;
    double r_feedforward_ = 1.8;
    double error_guard_y_ = 3.0;
    double error_guard_theta_ = 1.40;
};

class SpatialSLQR {
public:
    struct SolveInfo {
        double delta = 0.0;
        double cost = 0.0;
        double barrier_margin = 1.0;
        bool used_fallback = false;
        bool solver_ok = true;
        double tracking_slack = 0.0;
        double envelope_slack = 0.0;
        bool soft_constraint_active = false;
    };

    void configure(ros::NodeHandle& pnh) {
        horizon_steps_ = pnh.param("slqr_horizon_steps", pnh.param("horizon_steps", 20));
        ds_ = pnh.param("spatial_step", 0.10);
        wheelbase_ = pnh.param("wheelbase", 0.60);
        max_steer_ = pnh.param("max_steer", 0.785398);
        max_sequence_delta_step_ = pnh.param("slqr_max_delta_step", 0.18);
        if (pnh.param("slqr_rate_from_actuator", true)) {
            const double max_rate = pnh.param("max_steer_rate", 1.20);
            const double control_rate = std::max(1.0, pnh.param("control_rate", 30.0));
            const double rate_scale = pnh.param("slqr_rate_step_scale", 1.50);
            const double actuator_step = std::max(0.01, max_rate / control_rate * rate_scale);
            max_sequence_delta_step_ = std::min(max_sequence_delta_step_, actuator_step);
        }
        q_y_ = pnh.param("slqr_q_y", 42.0);
        q_theta_ = pnh.param("slqr_q_theta", 18.0);
        q_terminal_y_ = pnh.param("slqr_q_terminal_y", 90.0);
        q_terminal_theta_ = pnh.param("slqr_q_terminal_theta", 40.0);
        r_delta_ = pnh.param("slqr_r_delta", 0.55);
        r_delta_rate_ = pnh.param("slqr_r_delta_rate", 9.0);
        r_feedforward_ = pnh.param("slqr_r_feedforward", 1.8);
        barrier_weight_ = pnh.param("barrier_weight", 0.020);
        barrier_rate_weight_ = pnh.param("barrier_rate_weight", 0.010);
        fallback_cost_ratio_ = pnh.param("slqr_fallback_cost_ratio", 1.45);
        error_guard_y_ = pnh.param("slqr_error_guard_y", 3.0);
        error_guard_theta_ = pnh.param("slqr_error_guard_theta", 1.40);
        barrier_adapt_low_margin_ = pnh.param("barrier_adapt_low_margin", 1e-3);
        barrier_adapt_recover_margin_ = pnh.param("barrier_adapt_recover_margin", 0.08);
        barrier_adapt_trigger_count_ = std::max(1, pnh.param("barrier_adapt_trigger_count", 5));
        barrier_adapt_scale_ = clampValue(pnh.param("barrier_adapt_scale", 0.55), 0.10, 1.0);
        barrier_adapt_speed_scale_ = clampValue(pnh.param("barrier_adapt_speed_scale", 0.75), 0.0, 1.0);
        slack_l1_ = pnh.param("slqr_slack_l1", 12.0);
        slack_l2_ = pnh.param("slqr_slack_l2", 22.0);
        soft_y_bound_ = pnh.param("slqr_soft_y", 0.16);
        soft_theta_bound_ = pnh.param("slqr_soft_theta", 0.42);
        yaw_envelope_weight_ = pnh.param("yaw_envelope_weight", 35.0);
        yaw_envelope_mu_scale_ = clampValue(pnh.param("yaw_envelope_mu_scale", 0.75), 0.05, 1.0);
        yaw_envelope_speed_floor_ = pnh.param("yaw_envelope_speed_floor", 0.08);
        straight_q_scale_ = clampValue(pnh.param("slqr_straight_q_scale", 0.45), 0.05, 1.0);
        straight_r_delta_scale_ = std::max(1.0, pnh.param("slqr_straight_r_delta_scale", 2.4));
        straight_r_delta_rate_scale_ = std::max(1.0, pnh.param("slqr_straight_r_delta_rate_scale", 2.2));
        r_feedforward_straight_ = pnh.param("slqr_r_feedforward_straight", 0.15);
        yaw_rate_penalty_ = pnh.param("slqr_yaw_rate_penalty", 2.0);
        straight_yaw_rate_penalty_ = pnh.param("slqr_straight_yaw_rate_penalty", 24.0);
        steering_feedback_penalty_ = pnh.param("slqr_steering_feedback_penalty", 0.25);
        straight_steering_feedback_penalty_ =
            pnh.param("slqr_straight_steering_feedback_penalty", 2.5);
        straight_delta_ff_limit_ = pnh.param("straight_delta_ff_limit", 0.06);
        last_feasible_.clear();
    }

    void setRuntimeEnvelope(double speed, double mu, double gravity) {
        runtime_speed_ = std::max(0.0, std::fabs(speed));
        runtime_mu_ = std::max(0.05, mu);
        runtime_gravity_ = std::max(0.10, gravity);
    }

    void setRuntimeStability(bool straight_mode, double yaw_rate, double steering_feedback) {
        straight_mode_ = straight_mode;
        runtime_yaw_rate_ = std::isfinite(yaw_rate) ? yaw_rate : 0.0;
        runtime_steering_feedback_ =
            clampValue(std::isfinite(steering_feedback) ? steering_feedback : 0.0,
                       -max_steer_,
                       max_steer_);
    }

    double yawRateBound() const {
        return runtime_mu_ * yaw_envelope_mu_scale_ * runtime_gravity_ /
               std::max(yaw_envelope_speed_floor_, runtime_speed_);
    }

    void setAdaptScale(double scale) {
        active_adapt_scale_ = clampValue(scale, 0.10, 1.0);
    }

    double activeAdaptScale() const {
        return active_adapt_scale_;
    }

    double adaptSpeedScale() const {
        return barrier_adapt_speed_scale_;
    }

    double lowMargin() const {
        return barrier_adapt_low_margin_;
    }

    double recoverMargin() const {
        return barrier_adapt_recover_margin_;
    }

    int triggerCount() const {
        return barrier_adapt_trigger_count_;
    }

    double triggeredScale() const {
        return barrier_adapt_scale_;
    }

    double solve(double e_y,
                 double e_theta,
                 const std::vector<double>& curvatures,
                 double previous_delta,
                 SolveInfo& info) {
        const int n = std::max(2, std::min(static_cast<int>(curvatures.size()), horizon_steps_));
        std::vector<double> fallback = shiftedFeasible(curvatures, n, previous_delta);
        std::vector<Eigen::RowVector3d> gains(static_cast<size_t>(n), Eigen::RowVector3d::Zero());
        std::vector<double> feedforward_terms(static_cast<size_t>(n), 0.0);
        double barrier_margin = 1.0;
        bool ok = backwardPass(curvatures, n, fallback, gains, feedforward_terms, barrier_margin);

        std::vector<double> candidate = fallback;
        if (ok) {
            candidate = forwardPass(e_y, e_theta, curvatures, previous_delta, gains, feedforward_terms);
        }

        double fallback_margin = 1.0;
        const double fallback_cost =
            rolloutCost(e_y, e_theta, curvatures, fallback, previous_delta, fallback_margin, nullptr);
        double candidate_margin = 1.0;
        const double candidate_cost =
            rolloutCost(e_y, e_theta, curvatures, candidate, previous_delta, candidate_margin, nullptr);

        const bool candidate_good =
            ok &&
            std::isfinite(candidate_cost) &&
            candidate_cost <= fallback_cost * fallback_cost_ratio_ + 1.0;
        if (candidate_good) {
            last_feasible_ = candidate;
            info.cost = candidate_cost;
            info.barrier_margin = std::min(barrier_margin, candidate_margin);
            info.used_fallback = false;
            info.solver_ok = true;
        } else {
            last_feasible_ = fallback;
            info.cost = fallback_cost;
            info.barrier_margin = fallback_margin;
            info.used_fallback = true;
            info.solver_ok = ok;
        }

        double selected_margin = 1.0;
        RolloutMetrics metrics;
        info.cost = rolloutCost(e_y,
                                e_theta,
                                curvatures,
                                last_feasible_,
                                previous_delta,
                                selected_margin,
                                &metrics);
        info.barrier_margin = std::min(info.barrier_margin, selected_margin);
        info.tracking_slack = metrics.tracking_slack;
        info.envelope_slack = metrics.envelope_slack;
        info.soft_constraint_active =
            metrics.tracking_slack > 1e-4 || metrics.envelope_slack > 1e-4;
        info.delta = clampValue(last_feasible_.front(), -max_steer_, max_steer_);
        return info.delta;
    }

private:
    struct RolloutMetrics {
        double tracking_slack = 0.0;
        double envelope_slack = 0.0;
    };

    std::vector<double> shiftedFeasible(const std::vector<double>& curvatures,
                                        int n,
                                        double previous_delta) const {
        std::vector<double> out(static_cast<size_t>(n), previous_delta);
        if (last_feasible_.empty()) {
            for (int i = 0; i < n; ++i) {
                out[static_cast<size_t>(i)] = feedforward(curvatures[static_cast<size_t>(std::min(i, static_cast<int>(curvatures.size()) - 1))]);
            }
        } else {
            for (int i = 0; i < n; ++i) {
                const size_t src = std::min(static_cast<size_t>(i + 1), last_feasible_.size() - 1);
                out[static_cast<size_t>(i)] = last_feasible_[src];
            }
        }
        projectSequence(out, previous_delta);
        return out;
    }

    bool backwardPass(const std::vector<double>& curvatures,
                      int n,
                      const std::vector<double>& nominal,
                      std::vector<Eigen::RowVector3d>& gains,
                      std::vector<double>& feedforward_terms,
                      double& barrier_margin) const {
        Eigen::Matrix3d p_mat = Eigen::Matrix3d::Zero();
        const double q_scale = straight_mode_ ? straight_q_scale_ : 1.0;
        const double r_delta_rate_eff_base =
            r_delta_rate_ * (straight_mode_ ? straight_r_delta_rate_scale_ : 1.0);
        p_mat(0, 0) = active_adapt_scale_ * q_scale * q_terminal_y_;
        p_mat(1, 1) = active_adapt_scale_ * q_scale * q_terminal_theta_;
        p_mat(2, 2) = 0.10 * r_delta_rate_eff_base;
        Eigen::Vector3d p_vec = Eigen::Vector3d::Zero();
        barrier_margin = 1.0;

        for (int k = n - 1; k >= 0; --k) {
            const double kappa = curvatures[static_cast<size_t>(std::min(k, static_cast<int>(curvatures.size()) - 1))];
            const double delta_ff = feedforward(kappa);
            const double sec2 = 1.0 / std::max(0.08, square(std::cos(delta_ff)));
            const double b_delta = ds_ * sec2 / std::max(0.05, wheelbase_);

            Eigen::Matrix3d a = Eigen::Matrix3d::Zero();
            a(0, 0) = 1.0;
            a(0, 1) = ds_;
            a(1, 0) = -ds_ * kappa * kappa;
            a(1, 1) = 1.0;
            Eigen::Vector3d b;
            b << 0.0, b_delta, 1.0;
            Eigen::Vector3d c;
            c << 0.0, -b_delta * delta_ff, 0.0;

            const double nominal_delta = nominal[static_cast<size_t>(k)];
            const double steer_h = barrierHessian(nominal_delta, max_steer_, barrier_weight_);
            const double steer_g = barrierGradient(nominal_delta, max_steer_, barrier_weight_);
            barrier_margin = std::min(barrier_margin, barrierMargin(nominal_delta, max_steer_));

            const double prev_nominal = (k == 0) ? nominal.front() : nominal[static_cast<size_t>(k - 1)];
            const double rate_nominal = nominal_delta - prev_nominal;
            const double rate_h = barrierHessian(rate_nominal, max_sequence_delta_step_, barrier_rate_weight_);
            barrier_margin = std::min(barrier_margin, barrierMargin(rate_nominal, max_sequence_delta_step_));

            const double r_delta_eff = r_delta_ * (straight_mode_ ? straight_r_delta_scale_ : 1.0);
            const double r_feedforward_eff = straight_mode_ ? r_feedforward_straight_ : r_feedforward_;
            const double yaw_delta_gain = runtime_speed_ / std::max(0.05, wheelbase_);
            const double yaw_delta_weight =
                (straight_mode_ ? straight_yaw_rate_penalty_ : yaw_rate_penalty_) *
                yaw_delta_gain * yaw_delta_gain;
            const double feedback_weight =
                straight_mode_ ? straight_steering_feedback_penalty_ : steering_feedback_penalty_;
            const double r_rate_eff = r_delta_rate_eff_base + 0.5 * rate_h;
            Eigen::Matrix3d q = Eigen::Matrix3d::Zero();
            q(0, 0) = active_adapt_scale_ * q_scale * q_y_;
            q(1, 1) = active_adapt_scale_ * q_scale * q_theta_;
            q(2, 2) = r_rate_eff;
            Eigen::Vector3d s = Eigen::Vector3d::Zero();
            s(2) = -r_rate_eff;
            const double r = r_delta_eff + r_feedforward_eff + r_rate_eff +
                             yaw_delta_weight + feedback_weight + 0.5 * steer_h;
            const double lin_r =
                -(r_delta_eff + r_feedforward_eff) * delta_ff -
                feedback_weight * runtime_steering_feedback_ +
                0.5 * (steer_g - steer_h * nominal_delta);

            const Eigen::Vector3d pc = p_mat * c + p_vec;
            const Eigen::Matrix3d qxx = q + a.transpose() * p_mat * a;
            const Eigen::Vector3d qxu = s + a.transpose() * p_mat * b;
            const double quu = r + b.transpose() * p_mat * b;
            const Eigen::Vector3d qx = a.transpose() * pc;
            const double qu = lin_r + b.dot(pc);

            if (!std::isfinite(quu) || quu <= 1e-8) {
                return false;
            }
            gains[static_cast<size_t>(k)] = (qxu / quu).transpose();
            feedforward_terms[static_cast<size_t>(k)] = qu / quu;
            p_mat = qxx - (qxu * qxu.transpose()) / quu;
            p_mat = 0.5 * (p_mat + p_mat.transpose());
            p_vec = qx - qxu * (qu / quu);
            if (!p_mat.allFinite() || !p_vec.allFinite()) {
                return false;
            }
        }
        return true;
    }

    std::vector<double> forwardPass(double e_y,
                                    double e_theta,
                                    const std::vector<double>& curvatures,
                                    double previous_delta,
                                    const std::vector<Eigen::RowVector3d>& gains,
                                    const std::vector<double>& feedforward_terms) const {
        const int n = static_cast<int>(gains.size());
        std::vector<double> controls(static_cast<size_t>(n), previous_delta);
        Eigen::Vector3d z;
        z << clampValue(e_y, -error_guard_y_, error_guard_y_),
             clampValue(e_theta, -error_guard_theta_, error_guard_theta_),
             clampValue(previous_delta, -max_steer_, max_steer_);
        double last_delta = previous_delta;
        for (int k = 0; k < n; ++k) {
            double delta = -(feedforward_terms[static_cast<size_t>(k)] +
                             gains[static_cast<size_t>(k)].dot(z));
            delta = clampValue(delta, -0.995 * max_steer_, 0.995 * max_steer_);
            delta = clampValue(delta,
                               last_delta - max_sequence_delta_step_,
                               last_delta + max_sequence_delta_step_);
            controls[static_cast<size_t>(k)] = delta;

            double ey = z[0];
            double eth = z[1];
            const double kappa = curvatures[static_cast<size_t>(std::min(k, static_cast<int>(curvatures.size()) - 1))];
            stepState(kappa, delta, ey, eth);
            z << ey, eth, delta;
            last_delta = delta;
        }
        projectSequence(controls, previous_delta);
        return controls;
    }

    void projectSequence(std::vector<double>& controls, double previous_delta) const {
        double last = previous_delta;
        for (double& delta : controls) {
            delta = clampValue(delta, -0.995 * max_steer_, 0.995 * max_steer_);
            delta = clampValue(delta,
                               last - max_sequence_delta_step_,
                               last + max_sequence_delta_step_);
            last = delta;
        }
    }

    double rolloutCost(double e_y,
                       double e_theta,
                       const std::vector<double>& curvatures,
                       const std::vector<double>& controls,
                       double previous_delta,
                       double& barrier_margin,
                       RolloutMetrics* metrics) const {
        double ey = clampValue(e_y, -error_guard_y_, error_guard_y_);
        double eth = clampValue(e_theta, -error_guard_theta_, error_guard_theta_);
        double last_delta = previous_delta;
        double cost = 0.0;
        barrier_margin = 1.0;
        RolloutMetrics local_metrics;
        for (size_t i = 0; i < controls.size(); ++i) {
            const double kappa = curvatures[std::min(i, curvatures.size() - 1)];
            const double delta = controls[i];
            stepState(kappa, delta, ey, eth);
            const double ff = feedforward(kappa);
            const double d_delta = delta - last_delta;
            const double slack_y = std::max(0.0, std::fabs(ey) - soft_y_bound_);
            const double slack_theta = std::max(0.0, std::fabs(eth) - soft_theta_bound_);
            const double tracking_slack = slack_y + slack_theta;
            const double yaw_rate_pred =
                runtime_speed_ * std::tan(delta) / std::max(0.05, wheelbase_);
            const double envelope_slack =
                std::max(0.0, std::fabs(yaw_rate_pred) - yawRateBound());
            local_metrics.tracking_slack =
                std::max(local_metrics.tracking_slack, tracking_slack);
            local_metrics.envelope_slack =
                std::max(local_metrics.envelope_slack, envelope_slack);
            barrier_margin = std::min(barrier_margin, barrierMargin(delta, max_steer_));
            barrier_margin = std::min(barrier_margin, barrierMargin(d_delta, max_sequence_delta_step_));
            const double q_scale = straight_mode_ ? straight_q_scale_ : 1.0;
            const double r_delta_eff = r_delta_ * (straight_mode_ ? straight_r_delta_scale_ : 1.0);
            const double r_delta_rate_eff =
                r_delta_rate_ * (straight_mode_ ? straight_r_delta_rate_scale_ : 1.0);
            const double r_feedforward_eff = straight_mode_ ? r_feedforward_straight_ : r_feedforward_;
            const double yaw_penalty = straight_mode_ ? straight_yaw_rate_penalty_ : yaw_rate_penalty_;
            const double feedback_penalty =
                straight_mode_ ? straight_steering_feedback_penalty_ : steering_feedback_penalty_;
            cost += active_adapt_scale_ * q_scale * (q_y_ * ey * ey + q_theta_ * eth * eth);
            cost += slack_l1_ * tracking_slack + slack_l2_ * tracking_slack * tracking_slack;
            cost += yaw_envelope_weight_ * envelope_slack * envelope_slack;
            cost += yaw_penalty * yaw_rate_pred * yaw_rate_pred;
            cost += feedback_penalty * square(delta - runtime_steering_feedback_);
            cost += r_delta_eff * delta * delta + r_feedforward_eff * square(delta - ff);
            cost += r_delta_rate_eff * d_delta * d_delta;
            cost += barrierPenalty(delta, max_steer_, barrier_weight_);
            cost += barrierPenalty(d_delta, max_sequence_delta_step_, barrier_rate_weight_);
            last_delta = delta;
        }
        const double q_scale = straight_mode_ ? straight_q_scale_ : 1.0;
        cost += active_adapt_scale_ * q_scale *
                (q_terminal_y_ * ey * ey + q_terminal_theta_ * eth * eth);
        if (metrics != nullptr) {
            *metrics = local_metrics;
        }
        return std::isfinite(cost) ? cost : std::numeric_limits<double>::infinity();
    }

    void stepState(double curvature, double delta, double& e_y, double& e_theta) const {
        const double ey = clampValue(e_y, -error_guard_y_, error_guard_y_);
        const double eth = clampValue(e_theta, -error_guard_theta_, error_guard_theta_);
        const double cos_eth = std::max(0.18, std::fabs(std::cos(eth)));
        const double geometry = clampValue(1.0 - curvature * ey, 0.20, 2.80);
        const double ey_prime = geometry * std::tan(eth);
        const double etheta_prime = std::tan(delta) / std::max(0.05, wheelbase_) *
                                        geometry / cos_eth -
                                    curvature;
        e_y = clampValue(e_y + ds_ * ey_prime, -error_guard_y_, error_guard_y_);
        e_theta = clampValue(wrapAngle(e_theta + ds_ * etheta_prime),
                             -error_guard_theta_,
                             error_guard_theta_);
    }

    double feedforward(double curvature) const {
        const double limit = straight_mode_ ? std::min(0.95 * max_steer_, straight_delta_ff_limit_)
                                            : 0.95 * max_steer_;
        return clampValue(std::atan(wheelbase_ * curvature), -limit, limit);
    }

    double barrierMargin(double value, double bound) const {
        const double safe_bound = std::max(1e-4, std::fabs(bound));
        return clampValue(1.0 - square(value / safe_bound), 1e-6, 1.0);
    }

    double barrierPenalty(double value, double bound, double weight) const {
        if (weight <= 0.0) {
            return 0.0;
        }
        return -weight * std::log(barrierMargin(value, bound));
    }

    double barrierGradient(double value, double bound, double weight) const {
        if (weight <= 0.0) {
            return 0.0;
        }
        const double b2 = square(std::max(1e-4, std::fabs(bound)));
        const double margin = std::max(1e-6, b2 - value * value);
        return 2.0 * weight * value / margin;
    }

    double barrierHessian(double value, double bound, double weight) const {
        if (weight <= 0.0) {
            return 0.0;
        }
        const double b2 = square(std::max(1e-4, std::fabs(bound)));
        const double margin = std::max(1e-6, b2 - value * value);
        return 2.0 * weight * (b2 + value * value) / (margin * margin);
    }

    std::vector<double> last_feasible_;
    int horizon_steps_ = 20;
    double ds_ = 0.10;
    double wheelbase_ = 0.60;
    double max_steer_ = 0.785398;
    double max_sequence_delta_step_ = 0.18;
    double q_y_ = 42.0;
    double q_theta_ = 18.0;
    double q_terminal_y_ = 90.0;
    double q_terminal_theta_ = 40.0;
    double r_delta_ = 0.55;
    double r_delta_rate_ = 9.0;
    double r_feedforward_ = 1.8;
    double barrier_weight_ = 0.020;
    double barrier_rate_weight_ = 0.010;
    double fallback_cost_ratio_ = 1.45;
    double error_guard_y_ = 3.0;
    double error_guard_theta_ = 1.40;
    double barrier_adapt_low_margin_ = 1e-3;
    double barrier_adapt_recover_margin_ = 0.08;
    int barrier_adapt_trigger_count_ = 5;
    double barrier_adapt_scale_ = 0.55;
    double barrier_adapt_speed_scale_ = 0.75;
    double active_adapt_scale_ = 1.0;
    double slack_l1_ = 12.0;
    double slack_l2_ = 22.0;
    double soft_y_bound_ = 0.16;
    double soft_theta_bound_ = 0.42;
    double yaw_envelope_weight_ = 35.0;
    double yaw_envelope_mu_scale_ = 0.75;
    double yaw_envelope_speed_floor_ = 0.08;
    double straight_q_scale_ = 0.45;
    double straight_r_delta_scale_ = 2.4;
    double straight_r_delta_rate_scale_ = 2.2;
    double r_feedforward_straight_ = 0.15;
    double yaw_rate_penalty_ = 2.0;
    double straight_yaw_rate_penalty_ = 24.0;
    double steering_feedback_penalty_ = 0.25;
    double straight_steering_feedback_penalty_ = 2.5;
    double straight_delta_ff_limit_ = 0.06;
    bool straight_mode_ = false;
    double runtime_yaw_rate_ = 0.0;
    double runtime_steering_feedback_ = 0.0;
    double runtime_speed_ = 0.0;
    double runtime_mu_ = 0.45;
    double runtime_gravity_ = 9.80665;
};

class VirtualTowingNode {
public:
    VirtualTowingNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
        node_start_stamp_ = ros::Time::now();
        loadParameters();
        initializeDebugLogs();
        cps_.configure(pnh_);
        lead_observer_.configure(pnh_);
        collision_.configure(pnh_);
        steering_bias_.configure(pnh_);
        unified_tracker_.configure(pnh_);
        preview_tracker_.configure(pnh_);
        reference_conditioner_.configure(pnh_);
        extractor_.configure(pnh_);
        velocity_planner_.configure(pnh_);
        slqr_.configure(pnh_);

        reference_sub_ = nh_.subscribe(reference_topic_, 1, &VirtualTowingNode::referenceCallback, this);
        raw_sub_ = nh_.subscribe(raw_topic_, 1, &VirtualTowingNode::rawPathCallback, this);
        reference_status_sub_ =
            nh_.subscribe(reference_status_topic_, 1, &VirtualTowingNode::referenceStatusCallback, this);
        tracked_sub_ = nh_.subscribe(tracked_topic_, 1, &VirtualTowingNode::trackedCallback, this);
        if (!obstacle_poses_topic_.empty()) {
            obstacle_poses_sub_ =
                nh_.subscribe(obstacle_poses_topic_, 1, &VirtualTowingNode::obstaclePosesCallback, this);
        }
        if (!filtered_cloud_topic_.empty()) {
            filtered_cloud_sub_ =
                nh_.subscribe(filtered_cloud_topic_, 1, &VirtualTowingNode::filteredCloudCallback, this);
        }
        if (!cluster_cloud_topic_.empty()) {
            cluster_cloud_sub_ =
                nh_.subscribe(cluster_cloud_topic_, 1, &VirtualTowingNode::clusterCloudCallback, this);
        }
        car_sub_ = nh_.subscribe(car_topic_, 10, &VirtualTowingNode::carCallback, this);
        carvel_sub_ = nh_.subscribe(carvel_topic_, 10, &VirtualTowingNode::carVelCallback, this);
        imu_sub_ = nh_.subscribe(imu_topic_, 50, &VirtualTowingNode::imuCallback, this);
        ned_odom_sub_ = nh_.subscribe(ned_odom_topic_, 10, &VirtualTowingNode::nedOdomCallback, this);
        system_speed_sub_ = nh_.subscribe(system_speed_topic_, 10, &VirtualTowingNode::systemSpeedCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 10);

        const double rate = std::max(1.0, control_rate_);
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &VirtualTowingNode::controlTimer, this);
        last_control_time_ = ros::Time::now();
        cps_.recordCommand(0.0, last_control_time_);
        steering_history_.push_back(TimedSteering{last_control_time_, 0.0});

        ROS_INFO("virtual_towing control_pub ready | ref=%s target=%s cmd=%s rate=%.1fHz L=%.2f v=[%.2f,%.2f] max_steer=%.3f follow=%.2f fb_sign=%.1f cmd_sign=%.1f ref_hold=%.2f held_v=%.2f ramp=%.2f",
                 reference_topic_.c_str(),
                 tracked_topic_.c_str(),
                 cmd_topic_.c_str(),
                 control_rate_,
                 wheelbase_,
                 v_min_,
                 max_speed_,
                 max_steer_,
                 desired_follow_distance_,
                 feedback_steer_sign_,
                 cmd_steer_sign_,
                 reference_hold_timeout_,
                 held_reference_max_speed_,
                 control_recovery_ramp_seconds_);
    }

private:
    struct TimedSteering {
        ros::Time stamp;
        double value = 0.0;
    };

    struct TimedControlSample {
        ros::Time stamp;
        double cmd_delta = 0.0;
        double yaw_rate = 0.0;
        double feedback_delta = 0.0;
    };

    struct ReferenceStatus {
        ros::Time stamp;
        std::string mode = "UNKNOWN";
        std::string reason = "none";
        double quality = 1.0;
        double age = 0.0;
        bool valid = false;
    };

    void loadParameters() {
        reference_topic_ = pnh_.param<std::string>("reference_topic", "/planner/reference_path");
        raw_topic_ = pnh_.param<std::string>("raw_topic", "/planner/raw_path");
        tracked_topic_ = pnh_.param<std::string>("tracked_topic", "/pointpillars/tracked_objects");
        obstacle_poses_topic_ =
            pnh_.param<std::string>("obstacle_poses_topic", "/pointpillars/poses");
        filtered_cloud_topic_ =
            pnh_.param<std::string>("filtered_cloud_topic", "/filtered_cloud");
        cluster_cloud_topic_ =
            pnh_.param<std::string>("cluster_cloud_topic", "/cluster_cloud");
        collision_cloud_target_frame_ =
            pnh_.param<std::string>("collision_cloud_target_frame", "base_link");
        collision_tf_timeout_ = std::max(0.0, pnh_.param("collision_tf_timeout", 0.03));
        car_topic_ = pnh_.param<std::string>("car_topic", "/car_message");
        carvel_topic_ = pnh_.param<std::string>("carvel_topic", "/carvel");
        imu_topic_ = pnh_.param<std::string>("imu_topic", "/imu");
        ned_odom_topic_ = pnh_.param<std::string>("ned_odom_topic", "/NED_odometry");
        system_speed_topic_ = pnh_.param<std::string>("system_speed_topic", "/system_speed");
        cmd_topic_ = pnh_.param<std::string>("cmd_topic", "/cmd_vel");
        reference_status_topic_ =
            pnh_.param<std::string>("reference_status_topic", "/planner/reference_status");

        control_rate_ = pnh_.param("control_rate", 30.0);
        wheelbase_ = pnh_.param("wheelbase", 0.60);
        v_min_ = pnh_.param("v_min", 0.10);
        max_speed_ = pnh_.param("max_speed", 0.50);
        max_accel_ = pnh_.param("max_accel", 0.55);
        max_decel_ = pnh_.param("max_decel", 0.90);
        max_steer_ = pnh_.param("max_steer", 0.785398);
        max_steer_rate_ = pnh_.param("max_steer_rate", 1.20);
        spatial_step_ = pnh_.param("spatial_step", 0.10);
        feedback_steer_sign_ = pnh_.param("feedback_steer_sign", -1.0);
        cmd_steer_sign_ = pnh_.param("cmd_steer_sign", 1.0);
        cmd_steer_scale_ = pnh_.param("cmd_steer_scale", 1.0);
        wheel_speed_scale_ = pnh_.param("wheel_speed_scale", 0.001);
        carvel_speed_scale_ = pnh_.param("carvel_speed_scale", 0.001);
        carvel_angular_is_steer_ = pnh_.param("carvel_angular_is_steer", true);
        carvel_yaw_rate_sign_ = pnh_.param("carvel_yaw_rate_sign", 1.0);
        system_speed_scale_ = pnh_.param("system_speed_scale", 1.0);
        ned_speed_scale_ = pnh_.param("ned_speed_scale", 1.0);
        speed_filter_alpha_ = pnh_.param("speed_filter_alpha", 0.35);
        steering_filter_alpha_ = pnh_.param("steering_filter_alpha", 0.45);
        yaw_rate_filter_alpha_ = pnh_.param("yaw_rate_filter_alpha", 0.35);
        yaw_rate_imu_timeout_ = pnh_.param("yaw_rate_imu_timeout", 0.30);
        yaw_rate_imu_deadband_ = pnh_.param("yaw_rate_imu_deadband", 0.002);
        yaw_rate_feedback_deadband_ = pnh_.param("yaw_rate_feedback_deadband", 0.003);
        yaw_rate_feedback_blend_when_imu_zero_ =
            clampValue(pnh_.param("yaw_rate_feedback_blend_when_imu_zero", 0.85), 0.0, 1.0);
        imu_yaw_zero_unavailable_time_ =
            std::max(0.10, pnh_.param("imu_yaw_zero_unavailable_time", 1.0));
        reference_timeout_ = pnh_.param("reference_timeout", 0.35);
        raw_timeout_ = pnh_.param("raw_timeout", 0.35);
        allow_raw_path_fallback_ = pnh_.param("allow_raw_path_fallback", false);
        min_path_points_ = pnh_.param("min_path_points", 2);
        min_path_span_ = pnh_.param("min_path_span", 0.18);
        desired_follow_distance_ = pnh_.param("desired_follow_distance", 2.0);
        hard_stop_distance_ = pnh_.param("hard_stop_distance", 0.50);
        cps_compensation_gain_ = pnh_.param("cps_compensation_gain", 0.30);
        max_cps_comp_ = pnh_.param("max_cps_comp", 0.06);
        delay_compensation_enabled_ = pnh_.param("delay_compensation_enabled", true);
        steering_delay_seconds_ = pnh_.param("steering_delay_seconds", 0.20);
        steering_delay_steps_ = pnh_.param("steering_delay_steps", 4);
        delay_derivative_gain_ = pnh_.param("delay_derivative_gain", 0.12);
        max_delay_comp_ = pnh_.param("max_delay_comp", 0.08);
        delta_comp_lpf_alpha_ = clampValue(pnh_.param("delta_comp_lpf_alpha", 0.25), 0.0, 1.0);
        comp_deadband_ = pnh_.param("comp_deadband", 0.004);
        steering_history_seconds_ = pnh_.param("steering_history_seconds", 2.0);
        reference_quality_gate_enabled_ = pnh_.param("reference_quality_gate_enabled", true);
        reference_max_abs_e_theta_ = pnh_.param("reference_max_abs_e_theta", 1.20);
        reference_e_theta_reject_gate_ =
            pnh_.param("reference_e_theta_reject_gate", std::max(1.55, reference_max_abs_e_theta_ + 0.25));
        reference_e_theta_reject_persist_ = pnh_.param("reference_e_theta_reject_persist", 0.80);
        reference_max_abs_kappa0_ = pnh_.param("reference_max_abs_kappa0", 1.35);
        reference_max_fit_rms_ = pnh_.param("reference_max_fit_rms", 0.18);
        reference_max_heading_jump_ = pnh_.param("reference_max_heading_jump", 0.75);
        reference_max_kappa_jump_ = pnh_.param("reference_max_kappa_jump", 0.55);
        reference_hold_timeout_ = pnh_.param("reference_hold_timeout", 0.45);
        degraded_reference_min_span_ = pnh_.param("degraded_reference_min_span", 1.20);
        reference_bad_speed_scale_ = clampValue(pnh_.param("reference_bad_speed_scale", 0.36), 0.0, 1.0);
        held_reference_max_speed_ = pnh_.param("held_reference_max_speed", 0.14);
        provisional_tail_curve_min_span_ =
            std::max(0.05, pnh_.param("provisional_tail_curve_min_span", 0.35));
        provisional_tail_short_speed_limit_ =
            std::max(0.0, pnh_.param("provisional_tail_short_speed_limit", 0.14));
        straight_guard_speed_limit_ =
            std::max(0.0, pnh_.param("straight_guard_speed_limit", 0.12));
        low_confidence_speed_limit_ =
            std::max(0.0, pnh_.param("low_confidence_speed_limit", 0.08));
        curve_evidence_min_span_ =
            std::max(0.10, pnh_.param("curve_evidence_min_span", 1.00));
        curve_evidence_min_points_ =
            std::max(2, pnh_.param("curve_evidence_min_points", 8));
        curve_evidence_min_status_age_ =
            std::max(0.0, pnh_.param("curve_evidence_min_status_age", 0.70));
        short_path_guard_span_ =
            std::max(0.10, pnh_.param("short_path_guard_span", 0.80));
        control_recovery_ramp_seconds_ = pnh_.param("control_recovery_ramp_seconds", 0.50);
        kappa_degrade_gate_ = pnh_.param("kappa_degrade_gate", reference_max_abs_kappa0_);
        kappa_reject_gate_ = pnh_.param("kappa_reject_gate", 1.70);
        kappa_reject_persist_ = pnh_.param("kappa_reject_persist", 0.50);
        reference_kappa_jump_reject_gate_ =
            pnh_.param("reference_kappa_jump_reject_gate", 1.60);
        reference_kappa_jump_reject_persist_ =
            pnh_.param("reference_kappa_jump_reject_persist", 0.80);
        degraded_speed_limit_ = pnh_.param("degraded_speed_limit", 0.10);
        degraded_kappa_scale_ = clampValue(pnh_.param("degraded_kappa_scale", 0.55), 0.05, 1.0);
        degraded_max_kappa_control_ = pnh_.param("degraded_max_kappa_control", 0.90);
        degraded_max_steer_ = pnh_.param("degraded_max_steer", 0.34);
        degraded_steer_rate_scale_ = clampValue(pnh_.param("degraded_steer_rate_scale", 0.55), 0.05, 1.0);
        candidate_kappa_eval_window_ = pnh_.param("candidate_kappa_eval_window", 0.80);
        reference_soft_timeout_ = pnh_.param("reference_soft_timeout", reference_timeout_);
        reference_hard_timeout_ = pnh_.param("reference_hard_timeout", 1.20);
        yaw_damping_gain_ = pnh_.param("yaw_damping_gain", 0.10);
        max_yaw_damping_delta_ = pnh_.param("max_yaw_damping_delta", 0.08);
        yaw_rate_slow_gate_ = pnh_.param("yaw_rate_slow_gate", 0.08);
        yaw_rate_speed_scale_ = clampValue(pnh_.param("yaw_rate_speed_scale", 0.65), 0.0, 1.0);
        residual_comp_pause_gate_ = pnh_.param("residual_comp_pause_gate", 0.18);
        kappa_control_alpha_ = clampValue(pnh_.param("kappa_control_alpha", 0.35), 0.0, 1.0);
        kappa_preview_count_ = std::max(1, pnh_.param("kappa_preview_count", 3));
        kappa_preview_mix_ = clampValue(pnh_.param("kappa_preview_mix", 0.40), 0.0, 1.0);
        steer_sign_diag_window_ = pnh_.param("steer_sign_diag_window", 1.0);
        steer_sign_diag_min_corr_ = pnh_.param("steer_sign_diag_min_corr", 0.50);
        tracking_filter_alpha_y_ = clampValue(pnh_.param("tracking_filter_alpha_y", 0.30), 0.0, 1.0);
        tracking_filter_alpha_theta_ = clampValue(pnh_.param("tracking_filter_alpha_theta", 0.24), 0.0, 1.0);
        tracking_filter_alpha_kappa_ = clampValue(pnh_.param("tracking_filter_alpha_kappa", 0.28), 0.0, 1.0);
        tracking_filter_alpha_yaw_rate_ = clampValue(pnh_.param("tracking_filter_alpha_yaw_rate", 0.35), 0.0, 1.0);
        tracking_filter_theta_innovation_gate_ =
            pnh_.param("tracking_filter_theta_innovation_gate", 0.55);
        yaw_envelope_speed_scale_ = clampValue(pnh_.param("yaw_envelope_speed_scale", 0.70), 0.0, 1.0);
        control_curvature_ignore_front_s_ = pnh_.param("control_curvature_ignore_front_s", 0.25);
        control_curvature_trim_start_s_ = pnh_.param("control_curvature_trim_start_s", 0.40);
        control_curvature_trim_end_s_ = pnh_.param("control_curvature_trim_end_s", 1.50);
        straight_lock_enabled_ = pnh_.param("straight_lock_enabled", true);
        straight_lock_fit_rms_gate_ = pnh_.param("straight_lock_fit_rms_gate", 0.04);
        straight_lock_fit_rms_exit_gate_ = pnh_.param("straight_lock_fit_rms_exit_gate", 0.08);
        straight_lock_yaw_gate_ = pnh_.param("straight_lock_yaw_gate", 0.18);
        straight_lock_yaw_exit_gate_ = pnh_.param("straight_lock_yaw_exit_gate", 0.30);
        straight_lock_kappa_gate_ = pnh_.param("straight_lock_kappa_gate", 0.35);
        straight_lock_kappa_exit_gate_ = pnh_.param("straight_lock_kappa_exit_gate", 0.55);
        straight_lock_enter_time_ = pnh_.param("straight_lock_enter_time", 0.30);
        straight_lock_exit_time_ = pnh_.param("straight_lock_exit_time", 0.50);
        straight_lock_max_kappa_ = pnh_.param("straight_lock_max_kappa", 0.18);
        straight_lock_kappa_scale_ = clampValue(pnh_.param("straight_lock_kappa_scale", 0.18), 0.0, 1.0);
        straight_yaw_recovery_gate_ = pnh_.param("straight_yaw_recovery_gate", 0.045);
        straight_yaw_recovery_release_gate_ =
            pnh_.param("straight_yaw_recovery_release_gate", 0.030);
        straight_yaw_recovery_release_time_ =
            pnh_.param("straight_yaw_recovery_release_time", 0.40);
        straight_yaw_recovery_speed_limit_ =
            pnh_.param("straight_yaw_recovery_speed_limit", 0.12);
        straight_yaw_damping_gain_ = pnh_.param("straight_yaw_damping_gain", 0.16);
        straight_delta_deadband_ = pnh_.param("straight_delta_deadband", 0.025);
        straight_delta_ff_limit_ = pnh_.param("straight_delta_ff_limit", 0.06);
        straight_max_steer_ = pnh_.param("straight_max_steer", 0.22);
        straight_steer_rate_scale_ = clampValue(pnh_.param("straight_steer_rate_scale", 0.45), 0.05, 1.0);
        straight_sway_speed_limit_ = pnh_.param("straight_sway_speed_limit", 0.18);
        rail_enabled_ = pnh_.param("rail_stabilizer_enabled", true);
        rail_lateral_gate_ = pnh_.param("rail_lateral_gate", 0.05);
        rail_line_yaw_gate_ = pnh_.param("rail_line_yaw_gate", 0.22);
        rail_heading_suspect_gate_ = pnh_.param("rail_heading_suspect_gate", 0.22);
        rail_theta_filter_gain_ = clampValue(pnh_.param("rail_theta_filter_gain", 0.35), 0.0, 1.0);
        rail_theta_limit_ = pnh_.param("rail_theta_limit", 0.16);
        rail_gain_y_ = pnh_.param("rail_gain_y", 0.80);
        rail_gain_line_yaw_ = pnh_.param("rail_gain_line_yaw", 0.20);
        rail_gain_yaw_rate_ = pnh_.param("rail_gain_yaw_rate", 0.12);
        rail_gain_steering_ = pnh_.param("rail_gain_steering", 0.25);
        rail_max_delta_ = pnh_.param("rail_max_delta", 0.12);
        rail_slqr_weight_ = clampValue(pnh_.param("rail_slqr_weight", 0.15), 0.0, 1.0);
        course_lock_enabled_ = pnh_.param("course_lock_enabled", true);
        course_lock_lateral_gate_ = pnh_.param("course_lock_lateral_gate", 0.07);
        course_lock_line_yaw_gate_ = pnh_.param("course_lock_line_yaw_gate", 0.28);
        course_lock_kappa_gate_ = pnh_.param("course_lock_kappa_gate", 0.18);
        course_lock_gain_course_ = pnh_.param("course_lock_gain_course", 0.55);
        course_lock_gain_lateral_ = pnh_.param("course_lock_gain_lateral", 0.75);
        course_lock_gain_yaw_rate_ = pnh_.param("course_lock_gain_yaw_rate", 0.18);
        course_lock_soft_speed_ = pnh_.param("course_lock_soft_speed", 0.12);
        course_lock_max_delta_ = pnh_.param("course_lock_max_delta", 0.20);
        course_lock_slqr_weight_ = clampValue(pnh_.param("course_lock_slqr_weight", 0.20), 0.0, 1.0);
        course_error_ey_rate_gain_ = pnh_.param("course_error_ey_rate_gain", 0.35);
        course_error_beta_gain_ = pnh_.param("course_error_beta_gain", 0.65);
        course_filter_alpha_ = clampValue(pnh_.param("course_filter_alpha", 0.35), 0.0, 1.0);
        course_ey_rate_alpha_ = clampValue(pnh_.param("course_ey_rate_alpha", 0.25), 0.0, 1.0);
        track_course_filter_alpha_ =
            clampValue(pnh_.param("track_course_filter_alpha", 0.30), 0.0, 1.0);
        track_ey_rate_alpha_ =
            clampValue(pnh_.param("track_ey_rate_alpha", 0.28), 0.0, 1.0);
        track_course_ey_rate_gain_ = pnh_.param("track_course_ey_rate_gain", 0.45);
        track_beta_gain_ = pnh_.param("track_beta_gain", 0.65);
        track_imu_yaw_course_gain_ = pnh_.param("track_imu_yaw_course_gain", 0.10);
        track_course_slow_gate_ = pnh_.param("track_course_slow_gate", 0.25);
        track_course_crawl_gate_ = pnh_.param("track_course_crawl_gate", 0.55);
        track_lateral_slow_gate_ = pnh_.param("track_lateral_slow_gate", 0.05);
        track_lateral_crawl_gate_ = pnh_.param("track_lateral_crawl_gate", 0.10);
        track_min_speed_scale_ = clampValue(pnh_.param("track_min_speed_scale", 0.45), 0.05, 1.0);
        straight_stable_speed_limit_ = pnh_.param("straight_stable_speed_limit", 0.22);
        straight_stable_course_gate_ = pnh_.param("straight_stable_course_gate", 0.10);
        straight_stable_lateral_gate_ = pnh_.param("straight_stable_lateral_gate", 0.025);
        straight_stable_time_ = pnh_.param("straight_stable_time", 1.0);
        kappa_ref_lpf_alpha_ = clampValue(pnh_.param("kappa_ref_lpf_alpha", 0.28), 0.0, 1.0);
        kappa_ref_rate_limit_ = std::max(0.0, pnh_.param("kappa_ref_rate_limit", 0.55));
        curve_speed_kappa_gate_ = std::max(0.0, pnh_.param("curve_speed_kappa_gate", 0.18));
        curve_speed_limit_ = std::max(0.0, pnh_.param("curve_speed_limit", 0.16));
        sharp_curve_kappa_gate_ = std::max(curve_speed_kappa_gate_ + 1e-3,
                                           pnh_.param("sharp_curve_kappa_gate", 0.95));
        sharp_curve_speed_limit_ = std::max(0.0, pnh_.param("sharp_curve_speed_limit", 0.12));
        target_lost_curve_blend_gate_ =
            clampValue(pnh_.param("target_lost_curve_blend_gate", 0.20), 0.0, 1.0);
        target_lost_curve_speed_limit_ =
            std::max(0.0, pnh_.param("target_lost_curve_speed_limit", 0.09));
        preview_curve_speed_kappa_low_ =
            std::max(0.0, pnh_.param("preview_curve_speed_kappa_low", 0.35));
        preview_curve_speed_kappa_high_ =
            std::max(preview_curve_speed_kappa_low_ + 1e-3,
                     pnh_.param("preview_curve_speed_kappa_high", 0.95));
        preview_curve_speed_limit_low_ =
            std::max(0.0, pnh_.param("preview_curve_speed_limit_low", 0.24));
        preview_curve_speed_limit_high_ =
            std::max(0.0, pnh_.param("preview_curve_speed_limit_high", 0.12));
        sign_conflict_speed_limit_ =
            std::max(0.0, pnh_.param("sign_conflict_speed_limit", 0.08));
        conditioner_invalid_speed_limit_ =
            std::max(0.0, pnh_.param("conditioner_invalid_speed_limit", 0.08));
        conditioner_invalid_raw_kappa_gate_ =
            std::max(0.0, pnh_.param("conditioner_invalid_raw_kappa_gate", 4.0));
        curve_recovery_min_speed_ =
            std::max(0.0, pnh_.param("curve_recovery_min_speed", 0.12));
        inside_error_speed_scale_max_ =
            clampValue(pnh_.param("inside_error_speed_scale_max", 0.25), 0.0, 0.90);
        if (pnh_.hasParam("corner_cut_speed_scale")) {
            const double retain =
                clampValue(pnh_.param("corner_cut_speed_scale", 0.65), 0.10, 1.0);
            inside_error_speed_scale_max_ =
                std::max(inside_error_speed_scale_max_, 1.0 - retain);
        }
        tracking_speed_use_kappa_scale_ = pnh_.param("tracking_speed_use_kappa_scale", false);
        preview_speed_start_s_ = std::max(0.0, pnh_.param("preview_speed_start_s", 0.60));
        preview_speed_end_s_ =
            std::max(preview_speed_start_s_ + spatial_step_, pnh_.param("preview_speed_end_s", 1.20));
        visual_gap_start_ = pnh_.param("visual_gap_start", 2.30);
        visual_gap_full_ = std::max(visual_gap_start_ + 0.10, pnh_.param("visual_gap_full", 3.30));
        visual_gap_close_ = pnh_.param("visual_gap_close", 1.80);
        visual_catchup_max_speed_ = std::max(0.0, pnh_.param("visual_catchup_max_speed", 0.30));
        visual_catchup_straight_floor_ =
            std::max(0.0, pnh_.param("visual_catchup_straight_floor", 0.22));
        visual_catchup_curve_floor_ =
            std::max(0.0, pnh_.param("visual_catchup_curve_floor", 0.16));
        visual_catchup_sharp_floor_ =
            std::max(0.0, pnh_.param("visual_catchup_sharp_floor", 0.11));
        visual_catchup_yaw_gate_ =
            std::max(0.0, pnh_.param("visual_catchup_yaw_gate", 0.075));
        visual_catchup_inside_gate_ =
            std::max(0.0, pnh_.param("visual_catchup_inside_gate", 0.060));
        risk_inside_error_gate_ =
            std::max(0.0, pnh_.param("risk_inside_error_gate", 0.080));
        yaw_risk_persist_ = std::max(0.0, pnh_.param("yaw_risk_persist", 0.35));
        gap_speed_floor_start_ = pnh_.param("gap_speed_floor_start", 2.70);
        gap_curve_speed_floor_ = std::max(0.0, pnh_.param("gap_curve_speed_floor", 0.16));
        gap_straight_speed_floor_ = std::max(0.0, pnh_.param("gap_straight_speed_floor", 0.20));
        gap_sharp_speed_floor_ = std::max(0.0, pnh_.param("gap_sharp_speed_floor", 0.13));
        speed_cmd_alpha_up_ = clampValue(pnh_.param("speed_cmd_alpha_up", 0.45), 0.0, 1.0);
        speed_cmd_alpha_down_ = clampValue(pnh_.param("speed_cmd_alpha_down", 0.75), 0.0, 1.0);
        steer_cmd_alpha_cross_ = clampValue(pnh_.param("steer_cmd_alpha_cross", 0.75), 0.0, 1.0);
        max_steer_rate_cross_ = std::max(0.0, pnh_.param("max_steer_rate_cross", 1.80));
        degraded_hold_max_steer_ = std::max(0.0, pnh_.param("degraded_hold_max_steer", 0.10));
        oscillation_window_ = pnh_.param("oscillation_window", 0.80);
        oscillation_deadband_ = pnh_.param("oscillation_deadband", 0.025);
        oscillation_flip_gate_ = std::max(1, pnh_.param("oscillation_flip_gate", 3));
        oscillation_speed_scale_ = clampValue(pnh_.param("oscillation_speed_scale", 0.70), 0.05, 1.0);
        oscillation_steer_rate_scale_ =
            clampValue(pnh_.param("oscillation_steer_rate_scale", 0.55), 0.05, 1.0);
        enable_debug_csv_logs_ = pnh_.param("enable_debug_csv_logs", true);
        debug_log_dir_ = pnh_.param<std::string>("debug_log_dir", "/home/ubuntu/drive_car_copy/cdata");
        runtime_debug_file_ = pnh_.param<std::string>("runtime_debug_file", "runtime_debug.csv");
        path_debug_file_ = pnh_.param<std::string>("path_debug_file", "path_snapshot.csv");
        profile_debug_file_ = pnh_.param<std::string>("profile_debug_file", "profile_debug.csv");
        profile_log_stride_ = std::max(1, pnh_.param("profile_log_stride", 3));
        speed_cmd_alpha_ = pnh_.param("speed_cmd_alpha", 0.32);
        steer_cmd_alpha_ = pnh_.param("steer_cmd_alpha", 0.28);
        stale_path_stop_ = pnh_.param("stale_path_stop", true);
        publish_zero_on_stop_ = pnh_.param("publish_zero_on_stop", false);
    }

    void initializeDebugLogs() {
        if (!enable_debug_csv_logs_) {
            return;
        }
        mkdir(debug_log_dir_.c_str(), 0775);
        runtime_log_.open(joinPath(debug_log_dir_, runtime_debug_file_).c_str(), std::ios::out | std::ios::trunc);
        path_log_.open(joinPath(debug_log_dir_, path_debug_file_).c_str(), std::ios::out | std::ios::trunc);
        profile_log_.open(joinPath(debug_log_dir_, profile_debug_file_).c_str(), std::ios::out | std::ios::trunc);

        if (runtime_log_) {
            runtime_log_ << "stamp,path_valid,raw_fallback,path_points,path_span,e_y,e_theta,kappa0,fit_rms,"
                            "reference_quality,hold_reason,heading_jump,kappa_jump,"
                            "target_valid,gap,gap_error,range_rate,v_lead,gamma_cps,steering_residual,imu_z_variance,"
                            "gamma_steer_component,gamma_imu_component,"
                            "v_target,v_plan,cmd_v,slqr_cost,barrier_margin,delay_steps,delta_slqr,delta_comp,cmd_delta,"
                            "steer_feedback_for_control,yaw_rate,yaw_rate_raw_imu,yaw_rate_feedback_est,"
                            "yaw_rate_valid,yaw_rate_source,yaw_damping_delta,steer_sign_mismatch,"
                            "barrier_adapt_scale,kappa_control,reference_held,path_stale_reason,"
                            "control_ramp_scale,held_reference_speed_limit,"
                            "candidate_e_y,candidate_e_theta,candidate_kappa0,candidate_kappa_eval,"
                            "candidate_fit_rms,candidate_span,candidate_points,last_good_age,reference_age,raw_age,"
                            "reject_stage,control_mode,reference_status_mode,reference_status_reason,"
                            "hard_stop_reason,soft_recovery_reason,"
                            "filtered_e_y,filtered_e_theta,filtered_kappa_eval,yaw_rate_bound,"
                            "envelope_slack,tracking_slack,sqp_jerk_cost,soft_constraint_active,"
                            "straight_lock,straight_confidence,line_fit_rms,line_yaw,"
                            "kappa_raw_front,kappa_trimmed,kappa_suppressed,"
                            "delta_ff,delta_fb,yaw_rate_penalty,oscillation_score,"
                            "straight_speed_limited,raw_chassis_error,"
                            "heading_suspect,e_theta_control,delta_rail,rail_blend,"
                            "nearest_obstacle,collision_stop_active,collision_stop_age,"
                            "collision_resume_distance,collision_source,"
                            "course_error,course_error_source,e_y_rate,delta_course,delta_bias,"
                            "bias_apply_scale,bias_update_allowed,bias_block_reason,reference_confidence,"
                            "bias_learning_active,bias_learning_suspended,course_lock_active,"
                            "course_slqr_blend,yaw_rate_ref,yaw_rate_error,"
                            "track_signed_e_y,track_kappa_ref,track_beta_est,track_curve_blend,"
                            "delta_ref,delta_ref_ff,delta_ref_course,delta_ref_lateral,"
                            "delta_ref_ey_rate,delta_ref_yaw,unified_slqr_weight,tracking_speed_scale,"
                            "kappa_ref_filtered,kappa_ref_rate_limited,target_lost_slow_track,"
                            "collision_latch_source,last_close_target_age,"
                            "kappa_pp,kappa_path,kappa_cmd,kappa_ff,kappa_pp_weighted,"
                            "psi_preview,psi_preview_used,"
                            "preview_lookahead,preview_x,preview_y,path_sign_conflict,"
                            "inside_error,inside_gain_scale,ff_reduce_scale,lookahead_adaptive,"
                            "actuator_delay_est,nearest_cloud_obstacle,collision_point_count,"
                            "cloud_frame,cloud_tf_ok,cloud_tf_failed,cloud_tf_age,nearest_cloud_x,nearest_cloud_y,"
                            "collision_raw_nearest,collision_effective_nearest,"
                            "preview_min_v,commanded_v_ref,curve_distance,preview_curve_speed_limit,curve_speed_reason,"
                            "reference_conditioner_valid,conditioned_fit_rms,conditioned_kappa,"
                            "conditioned_kappa_rate,raw_path_kappa_p95,"
                            "turn_entry_s,distance_to_turn_entry,turn_blend,segment_mode,"
                            "conditioner_reject_reason,speed_limit_reason,curve_speed_cap,"
                            "inside_speed_scale,sqp_preview_min_v,final_speed_cap,"
                            "visual_gap_urgency,speed_arbitration_mode,curve_safe_cap,"
                            "catchup_speed_floor,risk_speed_floor,speed_cap_source,"
                            "cloud_msg_received,cloud_age,cloud_source_active,cloud_parse_failed,"
                            "front_axle_e_ct,inside_error_front,lookahead_mode,lookahead_limited_by_turn_entry,"
                            "kappa_ref_delay,steering_delay_predicted_e_y,corner_cut_score,"
                            "turn_blend_effective,delta_lateral_limited,speed_risk_score,"
                            "speed_stop_class,reference_trackable_mode,gap_speed_floor,"
                            "speed_filter_limited,accel_limited,ref_tracking_class,"
                            "straight_guard_active,pp_suppressed_reason,preview_x_too_short,"
                            "delta_lateral_raw,delta_lateral_guarded,reference_status_age,"
                            "low_confidence_steer_cap_active,"
                            "curve_evidence_reliable,ignored_short_path_kappa,low_confidence_delta_cap,"
                            "imu_yaw_rate_unavailable,yaw_feedback_source_age,yaw_rate_damping_source,"
                            "comp_clipped,safe_stop,stop_reason\n";
        }
        if (path_log_) {
            path_log_ << "stamp,source,path_seq,index,s,x,y,yaw\n";
        }
        if (profile_log_) {
            profile_log_ << "stamp,profile_seq,index,kappa,v_ref,preview_min_v,commanded_v_ref,curve_distance\n";
        }
        ROS_INFO("debug csv logs reset in %s", debug_log_dir_.c_str());
    }

    std::string joinPath(const std::string& dir, const std::string& file) const {
        if (dir.empty()) {
            return file;
        }
        if (dir[dir.size() - 1] == '/') {
            return dir + file;
        }
        return dir + "/" + file;
    }

    void referenceCallback(const nav_msgs::Path::ConstPtr& msg) {
        reference_path_ = extractor_.ingestPath(*msg, ros::Time::now(), "reference");
        writePathSnapshot(reference_path_);
    }

    void rawPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        raw_path_ = extractor_.ingestPath(*msg, ros::Time::now(), "raw");
        writePathSnapshot(raw_path_);
    }

    std::string parseStatusField(const std::string& text, const std::string& key) const {
        const std::string needle = key + "=";
        const size_t pos = text.find(needle);
        if (pos == std::string::npos) {
            return "";
        }
        const size_t begin = pos + needle.size();
        const size_t end = text.find_first_of(" \t\n;", begin);
        return text.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
    }

    void referenceStatusCallback(const std_msgs::String::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        const std::string mode = parseStatusField(msg->data, "mode");
        const std::string reason = parseStatusField(msg->data, "reason");
        const std::string quality = parseStatusField(msg->data, "quality");
        const std::string next_mode = mode.empty() ? "UNKNOWN" : mode;
        if (!reference_status_.valid || next_mode != reference_status_.mode) {
            reference_status_mode_since_ = now;
        }
        reference_status_.stamp = now;
        reference_status_.mode = next_mode;
        reference_status_.reason = reason.empty() ? "none" : reason;
        if (!quality.empty()) {
            try {
                reference_status_.quality = std::atof(quality.c_str());
            } catch (...) {
                reference_status_.quality = 1.0;
            }
        }
        reference_status_.age =
            reference_status_mode_since_.isZero() ? 0.0 : (now - reference_status_mode_since_).toSec();
        reference_status_.valid = true;
    }

    void trackedCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        lead_observer_.update(*msg, now);
        collision_.updateTracked(*msg, now);
    }

    void obstaclePosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        collision_.updateObstaclePoses(*msg, ros::Time::now());
    }

    void filteredCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        Eigen::Affine3d cloud_to_base = Eigen::Affine3d::Identity();
        bool tf_ok = false;
        double tf_age = -1.0;
        resolveCloudTransform(*msg, now, cloud_to_base, tf_ok, tf_age);
        collision_.updatePointCloud(*msg,
                                    now,
                                    "filtered_cloud",
                                    cloud_to_base,
                                    tf_ok,
                                    msg->header.frame_id,
                                    tf_age);
    }

    void clusterCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        Eigen::Affine3d cloud_to_base = Eigen::Affine3d::Identity();
        bool tf_ok = false;
        double tf_age = -1.0;
        resolveCloudTransform(*msg, now, cloud_to_base, tf_ok, tf_age);
        collision_.updatePointCloud(*msg,
                                    now,
                                    "cluster_cloud",
                                    cloud_to_base,
                                    tf_ok,
                                    msg->header.frame_id,
                                    tf_age);
    }

    void resolveCloudTransform(const sensor_msgs::PointCloud2& msg,
                               const ros::Time& now,
                               Eigen::Affine3d& cloud_to_base,
                               bool& tf_ok,
                               double& tf_age) {
        cloud_to_base = Eigen::Affine3d::Identity();
        tf_ok = false;
        tf_age = -1.0;
        const std::string frame = msg.header.frame_id;
        if (frame.empty() ||
            frame == collision_cloud_target_frame_ ||
            frame == "/" + collision_cloud_target_frame_) {
            tf_ok = true;
            tf_age = 0.0;
            return;
        }
        const ros::Time stamp = msg.header.stamp.isZero() ? ros::Time(0) : msg.header.stamp;
        geometry_msgs::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_.lookupTransform(collision_cloud_target_frame_,
                                                frame,
                                                stamp,
                                                ros::Duration(collision_tf_timeout_));
        } catch (const tf2::TransformException&) {
            try {
                tf_msg = tf_buffer_.lookupTransform(collision_cloud_target_frame_,
                                                    frame,
                                                    ros::Time(0),
                                                    ros::Duration(collision_tf_timeout_));
            } catch (const tf2::TransformException& e) {
                ROS_WARN_THROTTLE(1.0,
                                  "collision cloud TF lookup failed target=%s source=%s: %s",
                                  collision_cloud_target_frame_.c_str(),
                                  frame.c_str(),
                                  e.what());
                return;
            }
        }
        const geometry_msgs::Quaternion& qmsg = tf_msg.transform.rotation;
        Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
        if (q.norm() < 1e-6) {
            return;
        }
        q.normalize();
        cloud_to_base = Eigen::Affine3d::Identity();
        cloud_to_base.linear() = q.toRotationMatrix();
        cloud_to_base.translation() = Eigen::Vector3d(tf_msg.transform.translation.x,
                                                      tf_msg.transform.translation.y,
                                                      tf_msg.transform.translation.z);
        tf_ok = true;
        tf_age = tf_msg.header.stamp.isZero() ? 0.0 : std::fabs((now - tf_msg.header.stamp).toSec());
    }

    void carCallback(const move_car::car_parameter::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        updateSpeed(std::fabs(static_cast<double>(msg->back_wheel_speed)) * wheel_speed_scale_, now, true);
        const double steering = feedback_steer_sign_ * static_cast<double>(msg->turn_angle);
        updateSteering(steering, now);
        vehicle_.chassis_error = static_cast<int>(msg->error_flag);
    }

    void carVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        updateSpeed(std::fabs(msg->linear.x) * carvel_speed_scale_, now, false);
        if (carvel_angular_is_steer_) {
            updateSteering(feedback_steer_sign_ * msg->angular.z, now);
        } else {
            const double yaw_rate = carvel_yaw_rate_sign_ * msg->angular.z;
            vehicle_.yaw_rate =
                (1.0 - yaw_rate_filter_alpha_) * vehicle_.yaw_rate + yaw_rate_filter_alpha_ * yaw_rate;
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        cps_.updateImu(*msg, now);
        const double yaw_rate = msg->angular_velocity.z;
        if (std::isfinite(yaw_rate)) {
            last_imu_yaw_rate_raw_ = yaw_rate;
            last_imu_yaw_rate_stamp_ = now;
            has_imu_yaw_rate_ = true;
            if (std::fabs(yaw_rate) > yaw_rate_imu_deadband_) {
                last_imu_nonzero_yaw_rate_stamp_ = now;
            }
            vehicle_.yaw_rate =
                (1.0 - yaw_rate_filter_alpha_) * vehicle_.yaw_rate + yaw_rate_filter_alpha_ * yaw_rate;
        }
    }

    void nedOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        const double speed = safeHypot(vx, vy) * ned_speed_scale_;
        updateSpeed(speed, now, false);
        if (std::isfinite(vx) && std::isfinite(vy) && safeHypot(vx, vy) > 0.03) {
            ned_course_yaw_ = std::atan2(vy, vx);
            ned_course_stamp_ = now;
            ned_course_valid_ = true;
        }
        const double pose_yaw = yawFromQuaternion(msg->pose.pose.orientation);
        if (std::isfinite(pose_yaw)) {
            ned_pose_yaw_ = pose_yaw;
            ned_pose_yaw_stamp_ = now;
            ned_pose_yaw_valid_ = true;
        }
    }

    void systemSpeedCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        updateSpeed(std::fabs(msg->linear.x) * system_speed_scale_, ros::Time::now(), false);
    }

    void updateSpeed(double speed, const ros::Time& now, bool primary) {
        if (!std::isfinite(speed)) {
            return;
        }
        const double clipped = clampValue(speed, 0.0, std::max(1.5, 2.0 * max_speed_));
        if (!vehicle_.speed_valid || primary) {
            vehicle_.speed = (1.0 - speed_filter_alpha_) * vehicle_.speed + speed_filter_alpha_ * clipped;
        } else {
            vehicle_.speed = 0.85 * vehicle_.speed + 0.15 * clipped;
        }
        vehicle_.stamp = now;
        vehicle_.speed_valid = true;
    }

    void updateSteering(double steering, const ros::Time& now) {
        if (!std::isfinite(steering)) {
            return;
        }
        const double clipped = clampValue(steering, -max_steer_ * 1.25, max_steer_ * 1.25);
        if (!vehicle_.steering_valid) {
            vehicle_.steering_feedback = clipped;
        } else {
            vehicle_.steering_feedback =
                (1.0 - steering_filter_alpha_) * vehicle_.steering_feedback + steering_filter_alpha_ * clipped;
        }
        vehicle_.stamp = now;
        vehicle_.steering_valid = true;
        cps_.updateSteeringFeedback(vehicle_.steering_feedback, now);
    }

    const PathBuffer* selectPath(const ros::Time& now, ControllerDebug& debug) const {
        debug.reference_age = reference_path_.stamp.isZero() ? -1.0 : (now - reference_path_.stamp).toSec();
        debug.raw_age = raw_path_.stamp.isZero() ? -1.0 : (now - raw_path_.stamp).toSec();
        const bool ref_fresh = extractor_.freshEnough(reference_path_, now, reference_timeout_) &&
                               static_cast<int>(reference_path_.points.size()) >= min_path_points_ &&
                               pathSpan(reference_path_) >= min_path_span_;
        if (ref_fresh) {
            debug.using_raw_fallback = false;
            return &reference_path_;
        }
        const bool ref_soft_fresh = extractor_.freshEnough(reference_path_, now, reference_hard_timeout_) &&
                                    static_cast<int>(reference_path_.points.size()) >= min_path_points_ &&
                                    pathSpan(reference_path_) >= min_path_span_;
        if (ref_soft_fresh) {
            debug.using_raw_fallback = false;
            debug.reference_held = true;
            debug.hold_reason = "reference_soft_timeout";
            debug.reference_quality = 0.45;
            return &reference_path_;
        }
        const bool raw_fresh = allow_raw_path_fallback_ &&
                               extractor_.freshEnough(raw_path_, now, raw_timeout_) &&
                               static_cast<int>(raw_path_.points.size()) >= min_path_points_ &&
                               pathSpan(raw_path_) >= min_path_span_;
        if (raw_fresh) {
            debug.using_raw_fallback = true;
            debug.reference_held = true;
            debug.hold_reason = "raw_fallback";
            debug.reference_quality = 0.45;
            return &raw_path_;
        }
        return nullptr;
    }

    double pathSpan(const PathBuffer& path) const {
        if (path.points.size() < 2) {
            return 0.0;
        }
        return path.points.back().s - path.points.front().s;
    }

    ControlCmd safeStop(const ros::Time& now, const std::string& reason, ControllerDebug& debug) {
        debug.safe_stop = true;
        debug.stop_reason = reason;
        debug.path_stale_reason = reason;
        const bool hard_stop_reason =
            reason == "chassis_error" ||
            reason == "collision_stop" ||
            reason == "hard_target_gap" ||
            reason == "hard_follow_gap";
        if (hard_stop_reason) {
            debug.control_mode = "HARD_STOP";
            debug.hard_stop_reason = reason;
            debug.soft_recovery_reason = "none";
        } else if (debug.control_mode == "INIT") {
            debug.control_mode = "SOFT_PATH_STOP";
            debug.hard_stop_reason = "none";
            debug.soft_recovery_reason = reason;
        } else if (debug.control_mode == "SOFT_PATH_STOP" ||
                   debug.control_mode == "QUALITY_HOLD" ||
                   debug.control_mode == "SLOW_TRACK_DEGRADED") {
            debug.hard_stop_reason = "none";
            debug.soft_recovery_reason = reason;
        } else {
            debug.hard_stop_reason = reason;
            debug.soft_recovery_reason = "none";
        }
        const double dt = controlDt(now);
        const double target_speed = publish_zero_on_stop_ ? 0.0 : 0.0;
        if (reason == "collision_stop") {
            filtered_cmd_.speed = 0.0;
        } else {
            filtered_cmd_.speed = approachWithLimit(filtered_cmd_.speed, target_speed, max_decel_ * dt);
        }
        filtered_cmd_.steering = approachWithLimit(filtered_cmd_.steering, 0.0, max_steer_rate_ * dt);
        cps_.recordCommand(filtered_cmd_.steering, now);
        recordSteeringCommand(filtered_cmd_.steering, now);
        return filtered_cmd_;
    }

    double controlDt(const ros::Time& now) const {
        if (last_control_time_.isZero()) {
            return 1.0 / std::max(1.0, control_rate_);
        }
        return clampValue((now - last_control_time_).toSec(), 0.001, 0.20);
    }

    double approachWithLimit(double current, double target, double max_step) const {
        return current + clampValue(target - current, -std::fabs(max_step), std::fabs(max_step));
    }

    int curvatureIndexFromS(double s) const {
        return std::max(0, static_cast<int>(std::ceil(std::max(0.0, s) / std::max(0.03, spatial_step_))));
    }

    std::vector<double> curvatureWindow(const std::vector<double>& input,
                                        double start_s,
                                        double end_s) const {
        std::vector<double> values;
        if (input.empty()) {
            return values;
        }
        const int start = std::min(static_cast<int>(input.size()) - 1, curvatureIndexFromS(start_s));
        const int end = std::min(static_cast<int>(input.size()) - 1,
                                 std::max(start, curvatureIndexFromS(end_s)));
        values.reserve(static_cast<size_t>(end - start + 1));
        for (int i = start; i <= end; ++i) {
            const double kappa = input[static_cast<size_t>(i)];
            if (std::isfinite(kappa)) {
                values.push_back(kappa);
            }
        }
        if (values.empty()) {
            values.push_back(input.front());
        }
        return values;
    }

    double signedTrimmedMean(std::vector<double> values) const {
        if (values.empty()) {
            return 0.0;
        }
        std::sort(values.begin(), values.end(), [](double a, double b) {
            return std::fabs(a) < std::fabs(b);
        });
        const size_t trim = values.size() >= 5 ? values.size() / 5 : 0;
        const size_t begin = trim;
        const size_t end = values.size() - trim;
        double sum = 0.0;
        int count = 0;
        for (size_t i = begin; i < end; ++i) {
            sum += values[i];
            ++count;
        }
        return count > 0 ? sum / static_cast<double>(count) : values[values.size() / 2];
    }

    double medianAbs(std::vector<double> values) const {
        if (values.empty()) {
            return 0.0;
        }
        for (double& v : values) {
            v = std::fabs(v);
        }
        const size_t mid = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + static_cast<long>(mid), values.end());
        return values[mid];
    }

    double candidateCurvatureEnvelope(const PathTrackingProblem& candidate,
                                      ControllerDebug* debug) const {
        double envelope = std::fabs(candidate.first_curvature);
        if (candidate.curvatures.empty()) {
            if (debug != nullptr) {
                debug->kappa_raw_front = candidate.first_curvature;
                debug->kappa_trimmed = candidate.first_curvature;
                debug->kappa_suppressed = 0.0;
            }
            return envelope;
        }
        const std::vector<double> eval_window =
            curvatureWindow(candidate.curvatures,
                            control_curvature_trim_start_s_,
                            std::max(control_curvature_trim_end_s_, candidate_kappa_eval_window_));
        const double trimmed = signedTrimmedMean(eval_window);
        const double median_abs_kappa = medianAbs(eval_window);
        envelope = std::max(std::fabs(trimmed), median_abs_kappa);
        if (eval_window.size() < 3) {
            envelope = std::max(envelope, std::fabs(candidate.first_curvature));
        }
        if (has_filtered_kappa_) {
            const double filtered_candidate =
                (1.0 - kappa_control_alpha_) * filtered_kappa0_ +
                kappa_control_alpha_ * trimmed;
            envelope = std::max(envelope, std::fabs(filtered_candidate));
        }
        if (debug != nullptr) {
            debug->kappa_raw_front = candidate.curvatures.front();
            debug->kappa_trimmed = trimmed;
            debug->kappa_suppressed = std::fabs(candidate.curvatures.front()) - envelope;
        }
        return envelope;
    }

    double updateKappaExtremeAge(bool extreme, const ros::Time& now) {
        if (!extreme) {
            kappa_extreme_start_ = ros::Time();
            return 0.0;
        }
        if (kappa_extreme_start_.isZero()) {
            kappa_extreme_start_ = now;
            return 0.0;
        }
        return std::max(0.0, (now - kappa_extreme_start_).toSec());
    }

    double updateKappaJumpExtremeAge(bool extreme, const ros::Time& now) {
        if (!extreme) {
            kappa_jump_extreme_start_ = ros::Time();
            return 0.0;
        }
        if (kappa_jump_extreme_start_.isZero()) {
            kappa_jump_extreme_start_ = now;
            return 0.0;
        }
        return std::max(0.0, (now - kappa_jump_extreme_start_).toSec());
    }

    double updateEThetaExtremeAge(bool extreme, const ros::Time& now) {
        if (!extreme) {
            e_theta_extreme_start_ = ros::Time();
            return 0.0;
        }
        if (e_theta_extreme_start_.isZero()) {
            e_theta_extreme_start_ = now;
            return 0.0;
        }
        return std::max(0.0, (now - e_theta_extreme_start_).toSec());
    }

    bool acceptReference(const PathTrackingProblem& candidate,
                         const ros::Time& now,
                         bool path_is_held,
                         PathTrackingProblem& selected,
                         ControllerDebug& debug) {
        selected = candidate;
        const bool preselected_degraded = debug.reference_held || debug.hold_reason != "none";
        const std::string preselected_reason = debug.hold_reason;
        if (!preselected_degraded) {
            debug.reference_quality = 1.0;
            debug.hold_reason = "none";
        }
        debug.heading_jump = 0.0;
        debug.kappa_jump = 0.0;
        debug.candidate_e_y = candidate.e_y;
        debug.candidate_e_theta = candidate.e_theta;
        debug.candidate_kappa0 = candidate.first_curvature;
        debug.candidate_fit_rms = candidate.fit_rms;
        debug.candidate_span = candidate.path_span;
        debug.candidate_points = static_cast<int>(candidate.curvatures.size());
        debug.last_good_age = has_last_good_problem_ ? (now - last_good_problem_stamp_).toSec() : -1.0;
        debug.reference_age = reference_path_.stamp.isZero() ? -1.0 : (now - reference_path_.stamp).toSec();
        debug.raw_age = raw_path_.stamp.isZero() ? -1.0 : (now - raw_path_.stamp).toSec();
        debug.reference_status_mode = reference_status_.mode;
        debug.reference_status_reason = reference_status_.reason;
        const double kappa_eval = candidateCurvatureEnvelope(candidate, &debug);
        debug.candidate_kappa_eval = kappa_eval;
        debug.line_fit_rms = candidate.line_fit_rms;
        debug.line_yaw = candidate.line_yaw;
        debug.straight_confidence = candidate.straight_confidence;
        debug.reject_stage = "none";
        debug.control_mode = "NORMAL_TRACK";
        const bool status_degraded =
            reference_status_.valid &&
            (now - reference_status_.stamp).toSec() <= 0.60 &&
            reference_status_.mode != "NORMAL" &&
            reference_status_.mode != "TRACK_GOOD" &&
            reference_status_.mode != "TRACK_PROVISIONAL_TAIL" &&
            reference_status_.mode != "TRACK_DEGRADED_RAW" &&
            reference_status_.mode != "UNKNOWN";

        if (!reference_quality_gate_enabled_) {
            if (path_is_held || status_degraded || preselected_degraded) {
                debug.reference_held = true;
                debug.hold_reason = path_is_held ? "reconstructor_held" :
                                    (status_degraded ? "reference_status_degraded" : preselected_reason);
                debug.reference_quality = 0.50;
                debug.control_mode = "SLOW_TRACK_DEGRADED";
                last_reference_hold_time_ = now;
                was_reference_held_ = true;
            } else {
                rememberGoodReference(candidate, now, kappa_eval);
                was_reference_held_ = false;
            }
            return true;
        }

        if (path_is_held || status_degraded || preselected_degraded) {
            debug.reference_held = true;
            debug.hold_reason = path_is_held ? "reconstructor_held" :
                                (status_degraded ? "reference_status_degraded" : preselected_reason);
            debug.reference_quality = 0.50;
            debug.control_mode = "SLOW_TRACK_DEGRADED";
            last_reference_hold_time_ = now;
            was_reference_held_ = true;
            return true;
        }

        std::string reject_reason;
        double quality = 1.0;
        bool e_theta_degraded = false;
        if (!candidate.fit_valid || candidate.fit_rms > reference_max_fit_rms_) {
            reject_reason = "fit_rms";
            quality = std::min(quality, reference_max_fit_rms_ / std::max(1e-6, candidate.fit_rms));
        }
        if (std::fabs(candidate.e_theta) > reference_max_abs_e_theta_) {
            e_theta_degraded = true;
            quality = std::min(quality, reference_max_abs_e_theta_ / std::max(1e-6, std::fabs(candidate.e_theta)));
        }
        updateEThetaExtremeAge(false, now);
        const bool kappa_degraded = kappa_eval > kappa_degrade_gate_;
        const bool kappa_extreme = kappa_eval > kappa_reject_gate_;
        updateKappaExtremeAge(false, now);
        bool kappa_jump_degraded = false;
        if (kappa_extreme) {
            quality = std::min(quality, kappa_reject_gate_ / std::max(1e-6, kappa_eval));
        } else if (kappa_degraded) {
            quality = std::min(quality, kappa_degrade_gate_ / std::max(1e-6, kappa_eval));
        }
        if (has_last_good_problem_) {
            debug.heading_jump = std::fabs(wrapAngle(candidate.e_theta - last_good_e_theta_));
            debug.kappa_jump = std::fabs(kappa_eval - last_good_kappa_eval_);
            if (debug.heading_jump > reference_max_heading_jump_) {
                if (reject_reason.empty()) reject_reason = "heading_jump";
                quality = std::min(quality, reference_max_heading_jump_ / std::max(1e-6, debug.heading_jump));
            }
            kappa_jump_degraded = debug.kappa_jump > reference_max_kappa_jump_;
            updateKappaJumpExtremeAge(false, now);
            if (kappa_jump_degraded) {
                quality = std::min(quality, reference_max_kappa_jump_ / std::max(1e-6, debug.kappa_jump));
            }
        } else {
            updateKappaJumpExtremeAge(false, now);
        }

        debug.reference_quality = clampValue(quality, 0.0, 1.0);
        if (reject_reason.empty()) {
            (void)e_theta_degraded;
            (void)kappa_degraded;
            (void)kappa_jump_degraded;
            rememberGoodReference(candidate, now, kappa_eval);
            debug.reference_held = false;
            if (was_reference_held_ && last_reference_hold_time_.isZero()) {
                last_reference_hold_time_ = now;
            }
            was_reference_held_ = false;
            return true;
        }

        const bool can_hold = has_last_good_problem_ &&
                              (now - last_good_problem_stamp_).toSec() <= reference_hold_timeout_;
        if (can_hold) {
            selected = last_good_problem_;
            debug.hold_reason = reject_reason;
            debug.reference_held = true;
            debug.control_mode = "QUALITY_HOLD";
            last_reference_hold_time_ = now;
            was_reference_held_ = true;
            debug.reference_quality = std::min(debug.reference_quality, 0.50);
            return true;
        }
        debug.hold_reason = reject_reason;
        debug.reject_stage = reject_reason;
        debug.control_mode = "HARD_STOP";
        return false;
    }

    void rememberGoodReference(const PathTrackingProblem& problem,
                               const ros::Time& now,
                               double kappa_eval) {
        last_good_problem_ = problem;
        last_good_problem_stamp_ = now;
        last_good_e_theta_ = problem.e_theta;
        last_good_kappa0_ = problem.first_curvature;
        last_good_kappa_eval_ = kappa_eval;
        has_last_good_problem_ = true;
    }

    std::vector<double> controlCurvatures(const std::vector<double>& input, ControllerDebug& debug) {
        if (input.empty()) {
            return input;
        }
        std::vector<double> out = input;
        const std::vector<double> control_window =
            curvatureWindow(input, control_curvature_trim_start_s_, control_curvature_trim_end_s_);
        const double trimmed = signedTrimmedMean(control_window);
        const int ignore_n =
            std::min(static_cast<int>(input.size()) - 1,
                     curvatureIndexFromS(control_curvature_ignore_front_s_));
        const int preview_begin = std::min(static_cast<int>(input.size()) - 1, ignore_n);
        const int preview_n = std::max(1,
            std::min(kappa_preview_count_,
                     static_cast<int>(input.size()) - preview_begin));
        double preview_mean = 0.0;
        for (int i = 0; i < preview_n; ++i) {
            preview_mean += input[static_cast<size_t>(preview_begin + i)];
        }
        preview_mean /= static_cast<double>(preview_n);
        const double control_seed = 0.65 * trimmed + 0.35 * preview_mean;
        for (size_t i = 0; i < out.size(); ++i) {
            if (static_cast<int>(i) < ignore_n) {
                const double fade = ignore_n > 0 ? static_cast<double>(i) / static_cast<double>(ignore_n) : 1.0;
                out[i] = (1.0 - fade) * control_seed +
                         fade * ((1.0 - kappa_preview_mix_) * input[i] + kappa_preview_mix_ * preview_mean);
            } else {
                out[i] = (1.0 - kappa_preview_mix_) * input[i] + kappa_preview_mix_ * preview_mean;
            }
        }
        if (!has_filtered_kappa_) {
            filtered_kappa0_ = out.front();
            has_filtered_kappa_ = true;
        } else {
            filtered_kappa0_ =
                (1.0 - kappa_control_alpha_) * filtered_kappa0_ + kappa_control_alpha_ * out.front();
        }
        const double offset = filtered_kappa0_ - out.front();
        const double blend_span = std::max(1.0, static_cast<double>(std::min(5, static_cast<int>(out.size()))));
        for (size_t i = 0; i < out.size(); ++i) {
            const double fade = std::max(0.0, 1.0 - static_cast<double>(i) / blend_span);
            out[i] += fade * offset;
        }
        debug.kappa_raw_front = input.front();
        debug.kappa_trimmed = trimmed;
        debug.kappa_suppressed = input.front() - out.front();
        debug.kappa_control = out.front();
        return out;
    }

    void updateStraightLock(const PathTrackingProblem& problem,
                            ControllerDebug& debug,
                            const ros::Time& now) {
        const double median_abs_kappa =
            medianAbs(curvatureWindow(problem.curvatures,
                                      control_curvature_trim_start_s_,
                                      control_curvature_trim_end_s_));
        const bool enter_candidate =
            straight_lock_enabled_ &&
            problem.line_fit_rms <= straight_lock_fit_rms_gate_ &&
            std::fabs(problem.line_yaw) <= straight_lock_yaw_gate_ &&
            median_abs_kappa <= straight_lock_kappa_gate_;
        const bool exit_candidate =
            !straight_lock_enabled_ ||
            problem.line_fit_rms > straight_lock_fit_rms_exit_gate_ ||
            std::fabs(problem.line_yaw) > straight_lock_yaw_exit_gate_ ||
            median_abs_kappa > straight_lock_kappa_exit_gate_;

        if (!straight_lock_active_) {
            straight_exit_start_ = ros::Time();
            if (enter_candidate) {
                if (straight_enter_start_.isZero()) {
                    straight_enter_start_ = now;
                }
                if ((now - straight_enter_start_).toSec() >= straight_lock_enter_time_) {
                    straight_lock_active_ = true;
                    straight_exit_start_ = ros::Time();
                }
            } else {
                straight_enter_start_ = ros::Time();
            }
        } else {
            straight_enter_start_ = ros::Time();
            if (exit_candidate) {
                if (straight_exit_start_.isZero()) {
                    straight_exit_start_ = now;
                }
                if ((now - straight_exit_start_).toSec() >= straight_lock_exit_time_) {
                    straight_lock_active_ = false;
                    straight_enter_start_ = ros::Time();
                }
            } else {
                straight_exit_start_ = ros::Time();
            }
        }

        const double rms_score =
            clampValue(1.0 - problem.line_fit_rms / std::max(1e-3, straight_lock_fit_rms_exit_gate_), 0.0, 1.0);
        const double yaw_score =
            clampValue(1.0 - std::fabs(problem.line_yaw) / std::max(1e-3, straight_lock_yaw_exit_gate_), 0.0, 1.0);
        const double kappa_score =
            clampValue(1.0 - median_abs_kappa / std::max(1e-3, straight_lock_kappa_exit_gate_), 0.0, 1.0);
        debug.straight_confidence = rms_score * yaw_score * kappa_score;
        debug.straight_lock = straight_lock_active_;
        debug.line_fit_rms = problem.line_fit_rms;
        debug.line_yaw = problem.line_yaw;
        if (straight_lock_active_ && debug.control_mode == "NORMAL_TRACK") {
            debug.control_mode = "STRAIGHT_LOCK";
        }
    }

    void applyStraightLockCurvatureSuppression(std::vector<double>& curvatures,
                                               ControllerDebug& debug) const {
        if (!straight_lock_active_ || curvatures.empty()) {
            return;
        }
        for (double& kappa : curvatures) {
            kappa = clampValue(kappa * straight_lock_kappa_scale_,
                               -straight_lock_max_kappa_,
                               straight_lock_max_kappa_);
        }
        debug.kappa_suppressed = debug.kappa_raw_front - curvatures.front();
        debug.kappa_control = curvatures.front();
    }

    bool degradedControlActive(const ControllerDebug& debug) const {
        return debug.reference_held ||
               debug.control_mode == "SLOW_TRACK_DEGRADED" ||
               debug.control_mode == "QUALITY_HOLD" ||
               debug.hold_reason != "none";
    }

    void applyDegradedCurvatureLimits(std::vector<double>& curvatures, ControllerDebug& debug) const {
        if (curvatures.empty()) {
            return;
        }
        const double kappa_limit = std::max(0.05, std::min(kappa_reject_gate_, degraded_max_kappa_control_));
        for (double& kappa : curvatures) {
            kappa = clampValue(kappa * degraded_kappa_scale_, -kappa_limit, kappa_limit);
        }
        debug.kappa_control = curvatures.front();
    }

    void updateTrackingFilters(const PathTrackingProblem& raw_problem,
                               double kappa_eval,
                               ControllerDebug& debug) {
        const ros::Time now = ros::Time::now();
        const double speed_for_feedback =
            vehicle_.speed_valid ? vehicle_.speed : filtered_cmd_.speed;
        const double feedback_yaw_rate =
            (vehicle_.steering_valid && std::fabs(speed_for_feedback) > 1e-4)
                ? speed_for_feedback * std::tan(vehicle_.steering_feedback) / std::max(0.05, wheelbase_)
                : 0.0;
        const double command_pred_yaw_rate =
            std::fabs(speed_for_feedback) > 1e-4
                ? speed_for_feedback * std::tan(filtered_cmd_.steering) / std::max(0.05, wheelbase_)
                : 0.0;
        const bool imu_fresh =
            has_imu_yaw_rate_ && last_imu_yaw_rate_stamp_.isValid() &&
            (now - last_imu_yaw_rate_stamp_).toSec() <= yaw_rate_imu_timeout_;
        const bool imu_nonzero = imu_fresh && std::fabs(last_imu_yaw_rate_raw_) > yaw_rate_imu_deadband_;
        const bool feedback_nonzero = std::fabs(feedback_yaw_rate) > yaw_rate_feedback_deadband_;
        const bool command_pred_nonzero = std::fabs(command_pred_yaw_rate) > yaw_rate_feedback_deadband_;
        const double imu_nonzero_age =
            last_imu_nonzero_yaw_rate_stamp_.isValid()
                ? std::max(0.0, (now - last_imu_nonzero_yaw_rate_stamp_).toSec())
                : std::numeric_limits<double>::infinity();
        debug.imu_yaw_rate_unavailable =
            !imu_fresh || (!imu_nonzero && imu_nonzero_age > imu_yaw_zero_unavailable_time_);
        double raw_yaw_rate = vehicle_.yaw_rate;
        debug.yaw_rate_raw_imu = imu_fresh ? last_imu_yaw_rate_raw_ : 0.0;
        debug.yaw_rate_feedback_est = feedback_yaw_rate;
        debug.yaw_rate_valid = imu_nonzero || feedback_nonzero || command_pred_nonzero;
        debug.yaw_rate_source = "none";
        debug.yaw_rate_damping_source = "none";
        debug.yaw_feedback_source_age = -1.0;
        if (imu_nonzero) {
            raw_yaw_rate = last_imu_yaw_rate_raw_;
            debug.yaw_rate_source = "imu";
            debug.yaw_rate_damping_source = "imu";
            debug.yaw_feedback_source_age =
                last_imu_yaw_rate_stamp_.isValid() ? std::max(0.0, (now - last_imu_yaw_rate_stamp_).toSec()) : -1.0;
        } else if (feedback_nonzero) {
            raw_yaw_rate =
                yaw_rate_feedback_blend_when_imu_zero_ * feedback_yaw_rate +
                (1.0 - yaw_rate_feedback_blend_when_imu_zero_) * vehicle_.yaw_rate;
            debug.yaw_rate_source = imu_fresh ? "feedback_est_imu_zero" : "feedback_est";
            debug.yaw_rate_damping_source = "feedback_est";
            debug.yaw_feedback_source_age =
                vehicle_.stamp.isValid() ? std::max(0.0, (now - vehicle_.stamp).toSec()) : -1.0;
        } else if (command_pred_nonzero) {
            raw_yaw_rate = command_pred_yaw_rate;
            debug.yaw_rate_source = imu_fresh ? "command_pred_imu_zero" : "command_pred";
            debug.yaw_rate_damping_source = "command_pred";
            debug.yaw_feedback_source_age = 0.0;
        } else if (imu_fresh) {
            raw_yaw_rate = vehicle_.yaw_rate;
            debug.yaw_rate_source = "imu_zero";
            debug.yaw_rate_damping_source = "none";
            debug.yaw_feedback_source_age =
                last_imu_yaw_rate_stamp_.isValid() ? std::max(0.0, (now - last_imu_yaw_rate_stamp_).toSec()) : -1.0;
        }
        if (!tracking_filter_initialized_) {
            filtered_e_y_ = raw_problem.e_y;
            filtered_e_theta_ = raw_problem.e_theta;
            filtered_kappa_eval_ = kappa_eval;
            filtered_yaw_rate_ = raw_yaw_rate;
            tracking_filter_initialized_ = true;
        } else {
            filtered_e_y_ += tracking_filter_alpha_y_ * (raw_problem.e_y - filtered_e_y_);
            double theta_alpha = tracking_filter_alpha_theta_;
            const double theta_innovation = std::fabs(wrapAngle(raw_problem.e_theta - filtered_e_theta_));
            if (theta_innovation > tracking_filter_theta_innovation_gate_) {
                theta_alpha *= 0.30;
            }
            filtered_e_theta_ = wrapAngle(filtered_e_theta_ +
                                          theta_alpha * wrapAngle(raw_problem.e_theta - filtered_e_theta_));
            filtered_kappa_eval_ += tracking_filter_alpha_kappa_ * (kappa_eval - filtered_kappa_eval_);
            filtered_yaw_rate_ += tracking_filter_alpha_yaw_rate_ * (raw_yaw_rate - filtered_yaw_rate_);
        }
        debug.filtered_e_y = filtered_e_y_;
        debug.filtered_e_theta = filtered_e_theta_;
        debug.filtered_kappa_eval = filtered_kappa_eval_;
        debug.yaw_rate = filtered_yaw_rate_;
    }

    void updateBarrierAdaptation(double barrier_margin) {
        if (barrier_margin < slqr_.lowMargin()) {
            ++low_barrier_count_;
        } else if (barrier_margin > slqr_.recoverMargin()) {
            low_barrier_count_ = 0;
        }
        const bool active = low_barrier_count_ >= slqr_.triggerCount();
        slqr_.setAdaptScale(active ? slqr_.triggeredScale() : 1.0);
    }

    double correlation(const std::deque<TimedControlSample>& samples,
                       bool use_yaw_rate) const {
        if (samples.size() < 8) {
            return 0.0;
        }
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const TimedControlSample& s : samples) {
            mean_x += s.cmd_delta;
            mean_y += use_yaw_rate ? s.yaw_rate : s.feedback_delta;
        }
        mean_x /= static_cast<double>(samples.size());
        mean_y /= static_cast<double>(samples.size());
        double cov = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        for (const TimedControlSample& s : samples) {
            const double dx = s.cmd_delta - mean_x;
            const double dy = (use_yaw_rate ? s.yaw_rate : s.feedback_delta) - mean_y;
            cov += dx * dy;
            vx += dx * dx;
            vy += dy * dy;
        }
        if (vx < 1e-8 || vy < 1e-8) {
            return 0.0;
        }
        return cov / std::sqrt(vx * vy);
    }

    bool updateSteerSignDiagnostic(double cmd_delta, const ros::Time& now) {
        sign_diag_samples_.push_back(TimedControlSample{now, cmd_delta, vehicle_.yaw_rate, vehicle_.steering_feedback});
        while (!sign_diag_samples_.empty() &&
               (now - sign_diag_samples_.front().stamp).toSec() > steer_sign_diag_window_) {
            sign_diag_samples_.pop_front();
        }
        const double cmd_yaw_corr = correlation(sign_diag_samples_, true);
        const double cmd_feedback_corr = correlation(sign_diag_samples_, false);
        return cmd_yaw_corr > steer_sign_diag_min_corr_ && cmd_feedback_corr < -steer_sign_diag_min_corr_;
    }

    double steeringOscillationScore(const ros::Time& now, double candidate_delta) const {
        int flips = 0;
        int last_sign = 0;
        auto consume = [&](double value) {
            if (std::fabs(value) < oscillation_deadband_) {
                return;
            }
            const int sign = value > 0.0 ? 1 : -1;
            if (last_sign != 0 && sign != last_sign) {
                ++flips;
            }
            last_sign = sign;
        };
        for (const TimedSteering& item : steering_history_) {
            if ((now - item.stamp).toSec() <= oscillation_window_) {
                consume(item.value);
            }
        }
        consume(candidate_delta);
        return clampValue(static_cast<double>(flips) /
                              static_cast<double>(std::max(1, oscillation_flip_gate_)),
                          0.0,
                          2.0);
    }

    bool updateStraightYawRecovery(const ros::Time& now) {
        if (!straight_lock_active_) {
            straight_yaw_recovery_active_ = false;
            straight_yaw_recovery_clear_start_ = ros::Time();
            return false;
        }
        if (std::fabs(filtered_yaw_rate_) > straight_yaw_recovery_gate_) {
            straight_yaw_recovery_active_ = true;
            straight_yaw_recovery_clear_start_ = ros::Time();
            return true;
        }
        if (!straight_yaw_recovery_active_) {
            return false;
        }
        if (std::fabs(filtered_yaw_rate_) < straight_yaw_recovery_release_gate_) {
            if (straight_yaw_recovery_clear_start_.isZero()) {
                straight_yaw_recovery_clear_start_ = now;
            }
            if ((now - straight_yaw_recovery_clear_start_).toSec() >= straight_yaw_recovery_release_time_) {
                straight_yaw_recovery_active_ = false;
                straight_yaw_recovery_clear_start_ = ros::Time();
            }
        } else {
            straight_yaw_recovery_clear_start_ = ros::Time();
        }
        return straight_yaw_recovery_active_;
    }

    double computeRailDelta(double speed_for_control, ControllerDebug& debug) const {
        const double speed = std::max(0.10, std::fabs(speed_for_control));
        const double raw =
            -rail_gain_y_ * filtered_e_y_ -
            rail_gain_line_yaw_ * debug.line_yaw -
            rail_gain_yaw_rate_ * filtered_yaw_rate_ / speed -
            rail_gain_steering_ * vehicle_.steering_feedback;
        return clampValue(raw, -rail_max_delta_, rail_max_delta_);
    }

    UnifiedTrackingSignal updateUnifiedTrackingSignal(const PathTrackingProblem& problem,
                                                      const std::vector<double>& steering_curvatures,
                                                      double speed_for_control,
                                                      const ros::Time& now,
                                                      ControllerDebug& debug) {
        UnifiedTrackingSignal signal;
        signal.signed_e_y = -filtered_e_y_;
        double dt = controlDt(now);
        if (!last_course_stamp_.isZero()) {
            dt = clampValue((now - last_course_stamp_).toSec(), 0.005, 0.20);
        }
        const double raw_e_y_rate =
            last_course_stamp_.isZero() ? 0.0 :
            (signal.signed_e_y - last_course_e_y_) / std::max(0.005, dt);
        filtered_e_y_rate_ += track_ey_rate_alpha_ * (raw_e_y_rate - filtered_e_y_rate_);
        last_course_e_y_ = signal.signed_e_y;
        last_course_stamp_ = now;
        signal.e_y_rate = filtered_e_y_rate_;

        const std::vector<double> eval_window =
            curvatureWindow(steering_curvatures,
                            control_curvature_trim_start_s_,
                            control_curvature_trim_end_s_);
        signal.kappa_ref = eval_window.empty() ? (steering_curvatures.empty() ? 0.0 : steering_curvatures.front())
                                               : signedTrimmedMean(eval_window);
        const double raw_kappa_ref = signal.kappa_ref;
        bool kappa_rate_limited = false;
        if (!filtered_kappa_ref_initialized_) {
            filtered_kappa_ref_ = raw_kappa_ref;
            filtered_kappa_ref_initialized_ = true;
        } else {
            const double lpf_kappa =
                filtered_kappa_ref_ + kappa_ref_lpf_alpha_ * (raw_kappa_ref - filtered_kappa_ref_);
            if (kappa_ref_rate_limit_ > 1e-6) {
                const double max_step = kappa_ref_rate_limit_ * std::max(0.005, dt);
                const double limited =
                    clampValue(lpf_kappa, filtered_kappa_ref_ - max_step, filtered_kappa_ref_ + max_step);
                kappa_rate_limited = std::fabs(limited - lpf_kappa) > 1e-6;
                filtered_kappa_ref_ = limited;
            } else {
                filtered_kappa_ref_ = lpf_kappa;
            }
        }
        signal.kappa_ref = filtered_kappa_ref_;
        const double median_abs_kappa_raw =
            eval_window.empty() ? std::fabs(raw_kappa_ref) : medianAbs(eval_window);
        const double median_abs_kappa =
            0.55 * median_abs_kappa_raw + 0.45 * std::fabs(signal.kappa_ref);
        signal.curve_blend = unified_tracker_.curveBlend(median_abs_kappa);
        signal.yaw_rate_ref = speed_for_control * signal.kappa_ref;
        signal.yaw_rate_error = signal.yaw_rate_ref - filtered_yaw_rate_;

        bool beta_valid = false;
        signal.beta_est = estimateSideslipBeta(now, beta_valid);
        signal.beta_valid = beta_valid;
        const double speed_abs = std::max(0.10, std::fabs(speed_for_control));
        const double course_from_lateral =
            std::atan2(track_course_ey_rate_gain_ * signal.e_y_rate, speed_abs);
        const double raw_course_error =
            wrapAngle(problem.line_yaw +
                      course_from_lateral -
                      (beta_valid ? track_beta_gain_ * signal.beta_est : 0.0) +
                      track_imu_yaw_course_gain_ * signal.yaw_rate_error);
        if (!course_filter_initialized_) {
            filtered_course_error_ = raw_course_error;
            course_filter_initialized_ = true;
        } else {
            filtered_course_error_ = wrapAngle(
                filtered_course_error_ +
                track_course_filter_alpha_ * wrapAngle(raw_course_error - filtered_course_error_));
        }
        signal.e_course = filtered_course_error_;
        signal.source = beta_valid ? "ned_beta_preview_imu" : "preview_eyrate_imu";

        debug.track_signed_e_y = signal.signed_e_y;
        debug.track_kappa_ref = signal.kappa_ref;
        debug.kappa_ref_filtered = signal.kappa_ref;
        debug.kappa_ref_rate_limited = kappa_rate_limited;
        debug.track_beta_est = signal.beta_est;
        debug.track_curve_blend = signal.curve_blend;
        debug.course_error = signal.e_course;
        debug.course_error_source = signal.source;
        debug.e_y_rate = signal.e_y_rate;
        debug.yaw_rate_ref = signal.yaw_rate_ref;
        debug.yaw_rate_error = signal.yaw_rate_error;
        debug.kappa_control = signal.kappa_ref;
        return signal;
    }

    double trackingSpeedScale(const UnifiedTrackingSignal& signal) const {
        double scale = 1.0;
        const double abs_course = std::fabs(signal.e_course);
        const double abs_lateral = std::fabs(signal.signed_e_y);
        if (abs_course > track_course_crawl_gate_) {
            scale = std::min(scale, track_min_speed_scale_);
        } else if (abs_course > track_course_slow_gate_) {
            const double t = clampValue((abs_course - track_course_slow_gate_) /
                                            std::max(1e-3, track_course_crawl_gate_ - track_course_slow_gate_),
                                        0.0,
                                        1.0);
            scale = std::min(scale, 1.0 - 0.35 * t);
        }
        if (abs_lateral > track_lateral_crawl_gate_) {
            scale = std::min(scale, track_min_speed_scale_);
        } else if (abs_lateral > track_lateral_slow_gate_) {
            const double t = clampValue((abs_lateral - track_lateral_slow_gate_) /
                                            std::max(1e-3, track_lateral_crawl_gate_ - track_lateral_slow_gate_),
                                        0.0,
                                        1.0);
            scale = std::min(scale, 1.0 - 0.30 * t);
        }
        if (tracking_speed_use_kappa_scale_) {
            const double abs_kappa = std::fabs(signal.kappa_ref);
            if (abs_kappa > sharp_curve_kappa_gate_) {
                scale = std::min(scale,
                                 sharp_curve_speed_limit_ / std::max(1e-3, max_speed_));
            } else if (abs_kappa > curve_speed_kappa_gate_) {
                const double t = clampValue((abs_kappa - curve_speed_kappa_gate_) /
                                                std::max(1e-3, sharp_curve_kappa_gate_ - curve_speed_kappa_gate_),
                                            0.0,
                                            1.0);
                const double speed_limit =
                    (1.0 - t) * curve_speed_limit_ + t * sharp_curve_speed_limit_;
                scale = std::min(scale, speed_limit / std::max(1e-3, max_speed_));
            }
        }
        const double curve_floor =
            std::min(track_min_speed_scale_,
                     sharp_curve_speed_limit_ / std::max(1e-3, max_speed_));
        return clampValue(scale, std::max(0.05, curve_floor), 1.0);
    }

    double previewCurveSpeedLimit(const PreviewGeometrySignal& preview,
                                  bool conditioner_valid,
                                  double raw_path_kappa_p95,
                                  std::string& reason) const {
        reason = "none";
        if (!preview.valid) {
            reason = "no_preview";
            return max_speed_;
        }
        double limit = max_speed_;
        const double abs_kappa =
            std::max(std::fabs(preview.kappa_ff), 0.55 * std::fabs(preview.kappa_pp));
        if (abs_kappa > preview_curve_speed_kappa_high_) {
            limit = std::min(limit, preview_curve_speed_limit_high_);
            reason = "sharp_curve";
        } else if (abs_kappa > preview_curve_speed_kappa_low_) {
            const double t = clampValue((abs_kappa - preview_curve_speed_kappa_low_) /
                                            std::max(1e-3,
                                                     preview_curve_speed_kappa_high_ -
                                                         preview_curve_speed_kappa_low_),
                                        0.0,
                                        1.0);
            limit = std::min(limit,
                             (1.0 - t) * preview_curve_speed_limit_low_ +
                                 t * preview_curve_speed_limit_high_);
            reason = "curve";
        }
        if (!conditioner_valid && raw_path_kappa_p95 > conditioner_invalid_raw_kappa_gate_) {
            limit = std::min(limit, conditioner_invalid_speed_limit_);
            reason = "conditioner_invalid_raw";
        }
        if (preview.inside_error > 1e-5) {
            const double t = clampValue((preview.inside_gain_scale - 1.0) /
                                            std::max(1e-3, preview.inside_gain_scale),
                                        0.0,
                                        1.0);
            const double scaled = limit * (1.0 - inside_error_speed_scale_max_ * t);
            limit = std::max(curve_recovery_min_speed_, scaled);
            if (reason == "none" || reason == "curve") {
                reason = "inside_error";
            }
        }
        return clampValue(limit, 0.0, max_speed_);
    }

    struct SpeedArbitration {
        double speed = 0.0;
        double visual_gap_urgency = 0.0;
        double curve_safe_cap = 0.0;
        double catchup_speed_floor = 0.0;
        double gap_speed_floor = 0.0;
        double risk_speed_floor = 0.0;
        double risk_score = 0.0;
        std::string stop_class = "none";
        std::string mode = "none";
        std::string cap_source = "none";
    };

    double catchupFloorForKappa(double abs_kappa) const {
        if (abs_kappa >= preview_curve_speed_kappa_high_) {
            return visual_catchup_sharp_floor_;
        }
        if (abs_kappa <= preview_curve_speed_kappa_low_) {
            return visual_catchup_straight_floor_;
        }
        const double t = clampValue((abs_kappa - preview_curve_speed_kappa_low_) /
                                        std::max(1e-3,
                                                 preview_curve_speed_kappa_high_ -
                                                     preview_curve_speed_kappa_low_),
                                    0.0,
                                    1.0);
        return (1.0 - t) * visual_catchup_curve_floor_ +
               t * visual_catchup_sharp_floor_;
    }

    SpeedArbitration arbitrateSpeed(double preview_min_speed,
                                    double curve_safe_cap,
                                    const UnifiedTrackingSignal& tracking,
                                    const PreviewGeometrySignal& preview,
                                    const LeadTargetState& lead,
                                    const CollisionState& collision,
                                    bool degraded_control,
                                    bool slqr_preview_active,
                                    double envelope_slack,
                                    double slqr_adapt_scale,
                                    double ramp_scale,
                                    const ControllerDebug& debug,
                                    const ros::Time& now) {
        SpeedArbitration out;
        out.curve_safe_cap = clampValue(curve_safe_cap, 0.0, max_speed_);
        out.speed = std::min(clampValue(preview_min_speed, 0.0, max_speed_), out.curve_safe_cap);
        out.mode = "curve_safe";
        out.cap_source = debug.curve_speed_reason;

        const double abs_kappa =
            preview.valid
                ? std::max(std::fabs(preview.kappa_ff), 0.55 * std::fabs(preview.kappa_pp))
                : std::fabs(tracking.kappa_ref);
        const double inside_error =
            preview.valid ? std::max(preview.inside_error, preview.inside_error_front)
                          : std::fabs(debug.inside_error_front);
        const double abs_yaw_rate = std::fabs(filtered_yaw_rate_);
        const bool close_gap = lead.valid && lead.distance <= visual_gap_close_;
        const bool target_lost = !lead.valid;
        const double yaw_risk =
            clampValue((abs_yaw_rate - visual_catchup_yaw_gate_) /
                           std::max(1e-3, 0.06),
                       0.0,
                       1.0);
        const double inside_risk =
            clampValue((inside_error - risk_inside_error_gate_) /
                           std::max(1e-3, 0.08),
                       0.0,
                       1.0);
        const bool yaw_risk_instant = yaw_risk > 1e-4;
        if (yaw_risk_instant) {
            if (yaw_risk_start_.isZero()) {
                yaw_risk_start_ = now;
            }
        } else {
            yaw_risk_start_ = ros::Time();
        }
        const bool yaw_risky =
            yaw_risk_instant &&
            (yaw_risk_persist_ <= 1e-4 ||
             (!yaw_risk_start_.isZero() &&
              (now - yaw_risk_start_).toSec() >= yaw_risk_persist_));
        const bool inside_risky = inside_risk > 1e-4;
        const double effective_yaw_risk = yaw_risky ? yaw_risk : 0.35 * yaw_risk;
        out.risk_score = std::max(effective_yaw_risk, inside_risk);
        const bool stable_for_gap_catchup =
            !degraded_control &&
            debug.reference_confidence >= 0.65 &&
            debug.reference_trackable_mode != "provisional_tail_short" &&
            std::fabs(debug.front_axle_e_ct) < 0.08 &&
            std::fabs(debug.yaw_rate_error) < 0.08 &&
            out.risk_score < 0.80;

        if (lead.valid) {
            out.visual_gap_urgency =
                clampValue((lead.distance - visual_gap_start_) /
                               std::max(0.1, visual_gap_full_ - visual_gap_start_),
                           0.0,
                           1.0);
        }

        out.risk_speed_floor = curve_recovery_min_speed_;
        if (degraded_control) {
            out.speed = std::min(out.speed, std::max(0.0, held_reference_max_speed_));
            out.mode = "degraded_reference";
            out.cap_source = "degraded_reference";
            out.stop_class = "soft_reference";
            return out;
        }
        if (target_lost) {
            if (tracking.curve_blend > target_lost_curve_blend_gate_ ||
                collision.last_close_target_age >= 0.0) {
                out.speed = std::min(out.speed, target_lost_curve_speed_limit_);
                out.mode = "target_lost";
                out.cap_source = "target_lost";
                out.stop_class = "target_lost";
            }
            return out;
        }
        if (close_gap) {
            out.mode = "close_gap";
            out.cap_source = "close_gap";
            out.stop_class = "close_gap";
            return out;
        }
        if (yaw_risky || inside_risky) {
            const double risk_cap =
                std::max(out.risk_speed_floor,
                         std::min(out.speed, yaw_risky ? straight_yaw_recovery_speed_limit_
                                                       : out.curve_safe_cap));
            const double t = clampValue(out.risk_score, 0.0, 1.0);
            out.speed = (1.0 - t) * out.speed + t * risk_cap;
            out.mode = yaw_risky ? "risk_yaw" : "risk_inside";
            out.cap_source = out.mode;
        }

        if (out.visual_gap_urgency > 1e-3 && stable_for_gap_catchup) {
            const double base_floor = catchupFloorForKappa(abs_kappa);
            out.catchup_speed_floor =
                std::min(visual_catchup_max_speed_,
                         base_floor + out.visual_gap_urgency *
                                          std::max(0.0, visual_catchup_max_speed_ - base_floor));
            const double catchup_target =
                std::min(std::min(preview_min_speed, visual_catchup_max_speed_),
                         std::max(out.curve_safe_cap, out.catchup_speed_floor));
            out.speed = std::max(out.speed, catchup_target);
            if (out.risk_score < 1e-4) {
                out.mode = out.visual_gap_urgency > 0.65 ? "vision_catchup" : "gap_assist";
                out.cap_source = "visual_gap";
            }
        }

        if (lead.valid && lead.distance > gap_speed_floor_start_ && stable_for_gap_catchup) {
            const double abs_k = abs_kappa;
            const double floor =
                abs_k >= preview_curve_speed_kappa_high_
                    ? gap_sharp_speed_floor_
                    : (abs_k >= preview_curve_speed_kappa_low_ ? gap_curve_speed_floor_
                                                               : gap_straight_speed_floor_);
            out.gap_speed_floor = std::min(visual_catchup_max_speed_, floor);
            out.speed = std::max(out.speed,
                                 std::min(std::min(preview_min_speed, out.curve_safe_cap),
                                          out.gap_speed_floor));
            if (out.risk_score < 1e-4) {
                out.mode = "gap_floor";
                out.cap_source = "gap_floor";
            }
        }

        if (slqr_preview_active && envelope_slack > 1e-4) {
            out.speed *= yaw_envelope_speed_scale_;
            out.mode = "yaw_envelope";
            out.cap_source = "yaw_envelope";
        }
        if (slqr_preview_active && slqr_adapt_scale < 0.999) {
            out.speed *= slqr_.adaptSpeedScale();
            out.mode = "slqr_adapt";
            out.cap_source = "slqr_adapt";
        }
        if (ramp_scale < 0.999) {
            out.speed *= ramp_scale;
            out.cap_source = "ramp";
        }
        return out;
    }

    std::vector<double> curvaturesFromPath(const PathBuffer& path, size_t n) const {
        std::vector<double> out;
        if (!path.valid || path.points.empty() || n == 0) {
            return out;
        }
        out.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const double s = static_cast<double>(i) * spatial_step_;
            out.push_back(interpolatedPathCurvature(path.points, s));
        }
        return out;
    }

    double interpolatedPathCurvature(const std::vector<PathPoint>& pts, double s) const {
        if (pts.empty()) {
            return 0.0;
        }
        if (s <= pts.front().s) {
            return pts.front().curvature;
        }
        for (size_t i = 1; i < pts.size(); ++i) {
            if (s <= pts[i].s) {
                const PathPoint& a = pts[i - 1];
                const PathPoint& b = pts[i];
                const double t = clampValue((s - a.s) / std::max(1e-6, b.s - a.s), 0.0, 1.0);
                return a.curvature + t * (b.curvature - a.curvature);
            }
        }
        return pts.back().curvature;
    }

    double previewMinimumSpeed(const std::vector<double>& speed_profile,
                               const std::vector<double>& curvatures,
                               double& curve_distance) const {
        if (speed_profile.empty()) {
            curve_distance = -1.0;
            return 0.0;
        }
        const int start_idx = std::max(0, static_cast<int>(
            std::floor(preview_speed_start_s_ / std::max(0.03, spatial_step_))));
        const int end_idx = std::max(start_idx, static_cast<int>(
            std::ceil(preview_speed_end_s_ / std::max(0.03, spatial_step_))));
        const int n = static_cast<int>(speed_profile.size());
        double out = speed_profile[0];
        for (int i = start_idx; i <= end_idx && i < n; ++i) {
            out = std::min(out, speed_profile[static_cast<size_t>(i)]);
        }
        curve_distance = -1.0;
        for (size_t i = 0; i < curvatures.size(); ++i) {
            if (std::fabs(curvatures[i]) >= curve_speed_kappa_gate_) {
                curve_distance = static_cast<double>(i) * spatial_step_;
                break;
            }
        }
        return out;
    }

    bool nedCourseFresh(const ros::Time& now) const {
        return ned_course_valid_ &&
               ned_pose_yaw_valid_ &&
               !ned_course_stamp_.isZero() &&
               !ned_pose_yaw_stamp_.isZero() &&
               (now - ned_course_stamp_).toSec() < 0.50 &&
               (now - ned_pose_yaw_stamp_).toSec() < 0.50;
    }

    double estimateSideslipBeta(const ros::Time& now, bool& valid) const {
        valid = nedCourseFresh(now);
        if (!valid) {
            return 0.0;
        }
        return wrapAngle(ned_course_yaw_ - ned_pose_yaw_);
    }

    double updateCourseError(const PathTrackingProblem& problem,
                             double speed_for_control,
                             const ros::Time& now,
                             ControllerDebug& debug) {
        double dt = controlDt(now);
        if (!last_course_stamp_.isZero()) {
            dt = clampValue((now - last_course_stamp_).toSec(), 0.005, 0.20);
        }
        const double raw_e_y_rate =
            last_course_stamp_.isZero() ? 0.0 : (filtered_e_y_ - last_course_e_y_) / std::max(0.005, dt);
        filtered_e_y_rate_ += course_ey_rate_alpha_ * (raw_e_y_rate - filtered_e_y_rate_);
        last_course_e_y_ = filtered_e_y_;
        last_course_stamp_ = now;

        bool beta_valid = false;
        const double beta_est = estimateSideslipBeta(now, beta_valid);
        const double rate_course =
            std::atan2(-course_error_ey_rate_gain_ * filtered_e_y_rate_,
                       std::max(0.10, std::fabs(speed_for_control)));
        const double raw_course_error =
            wrapAngle(problem.line_yaw + rate_course - (beta_valid ? course_error_beta_gain_ * beta_est : 0.0));
        if (!course_filter_initialized_) {
            filtered_course_error_ = raw_course_error;
            course_filter_initialized_ = true;
        } else {
            filtered_course_error_ = wrapAngle(
                filtered_course_error_ +
                course_filter_alpha_ * wrapAngle(raw_course_error - filtered_course_error_));
        }
        debug.course_error = filtered_course_error_;
        debug.course_error_source = beta_valid ? "ned_beta_line_eyrate" : "line_eyrate_imu";
        debug.e_y_rate = filtered_e_y_rate_;
        return filtered_course_error_;
    }

    bool updateStraightStable(bool course_lock_active,
                              double abs_course_error,
                              double abs_e_y,
                              const ros::Time& now) {
        const bool stable =
            course_lock_active &&
            abs_course_error < straight_stable_course_gate_ &&
            abs_e_y < straight_stable_lateral_gate_;
        if (!stable) {
            straight_stable_start_ = ros::Time();
            straight_stable_active_ = false;
            return false;
        }
        if (straight_stable_start_.isZero()) {
            straight_stable_start_ = now;
            straight_stable_active_ = false;
            return false;
        }
        straight_stable_active_ = (now - straight_stable_start_).toSec() >= straight_stable_time_;
        return straight_stable_active_;
    }

    double computeCourseDelta(double speed_for_control,
                              ControllerDebug& debug) const {
        const double speed = std::max(0.02, std::fabs(speed_for_control));
        const double lateral_term =
            std::atan2(-course_lock_gain_lateral_ * filtered_e_y_, speed + course_lock_soft_speed_);
        const double yaw_term = course_lock_gain_yaw_rate_ * debug.yaw_rate_error;
        const double raw =
            steering_bias_.bias() +
            course_lock_gain_course_ * debug.course_error +
            lateral_term +
            yaw_term;
        return clampValue(raw, -course_lock_max_delta_, course_lock_max_delta_);
    }

    ControlCmd computeCommand(const ros::Time& now, ControllerDebug& debug) {
        debug.raw_chassis_error = vehicle_.chassis_error;
        const LeadTargetState lead = lead_observer_.state(now);
        collision_.updateLeadTarget(lead, now);
        const CollisionState collision = collision_.state(now);
        debug.target_valid = lead.valid;
        debug.gap = lead.valid ? lead.distance : -1.0;
        debug.nearest_obstacle = collision.nearest;
        debug.collision_stop_active = collision.active;
        debug.collision_stop_age = collision.active_age;
        debug.collision_resume_distance = collision.resume_distance;
        debug.collision_source = collision.source;
        debug.collision_latch_source = collision.latch_source;
        debug.last_close_target_age = collision.last_close_target_age;
        debug.nearest_cloud_obstacle = collision.nearest_cloud;
        debug.collision_point_count = collision.cloud_point_count;
        debug.cloud_frame = collision.cloud_frame;
        debug.cloud_tf_ok = collision.cloud_tf_ok;
        debug.cloud_tf_failed = collision.cloud_tf_failed;
        debug.cloud_tf_age = collision.cloud_tf_age;
        debug.nearest_cloud_x = collision.nearest_cloud_x;
        debug.nearest_cloud_y = collision.nearest_cloud_y;
        debug.collision_raw_nearest = collision.raw_nearest;
        debug.collision_effective_nearest = collision.effective_nearest;
        debug.cloud_msg_received = collision.cloud_msg_received;
        debug.cloud_age = collision.cloud_age;
        debug.cloud_source_active = collision.cloud_source_active;
        debug.cloud_parse_failed = collision.cloud_parse_failed;
        debug.emergency_stop = lead_observer_.emergencyStop() || collision.active;

        if (vehicle_.chassis_error != 0) {
            return safeStop(now, "chassis_error", debug);
        }
        if (collision.active) {
            return safeStop(now, "collision_stop", debug);
        }
        if (lead_observer_.emergencyStop()) {
            return safeStop(now, "hard_target_gap", debug);
        }
        if (lead.valid && lead.distance <= hard_stop_distance_) {
            return safeStop(now, "hard_follow_gap", debug);
        }

        const PathBuffer* path = selectPath(now, debug);
        if (path == nullptr) {
            return safeStop(now, stale_path_stop_ ? "path_stale" : "path_unavailable", debug);
        }
        debug.path_points = static_cast<int>(path->points.size());

        const PathTrackingProblem extracted_problem = extractor_.extract(*path);
        if (!extracted_problem.valid) {
            return safeStop(now, "path_extract_failed", debug);
        }
        PathTrackingProblem problem;
        if (!acceptReference(extracted_problem, now, path->held_reference, problem, debug)) {
            return safeStop(now, std::string("path_quality_reject_") + debug.hold_reason, debug);
        }
        debug.path_valid = true;
        debug.e_y = problem.e_y;
        debug.e_theta = problem.e_theta;
        debug.first_curvature = problem.first_curvature;
        debug.path_span = problem.path_span;
        debug.fit_rms = problem.fit_rms;
        std::vector<double> steering_curvatures = controlCurvatures(problem.curvatures, debug);
        const ConditionedReference conditioned_reference =
            reference_conditioner_.condition(*path, debug);
        const bool has_conditioner_metadata =
            conditioned_reference.path.valid && conditioned_reference.path.segment_mode != "raw";
        const PathBuffer* control_path =
            (conditioned_reference.valid || has_conditioner_metadata) ? &conditioned_reference.path : path;
        const bool short_reference =
            debug.path_span > 0.0 && debug.path_span < degraded_reference_min_span_;
        const bool conditioner_segment_short =
            debug.conditioner_reject_reason == "segment_too_short" ||
            debug.conditioner_reject_reason == "short_or_invalid";
        const bool trackable_raw_reference =
            debug.reference_status_mode == "TRACK_DEGRADED_RAW" &&
            debug.path_span >= degraded_reference_min_span_;
        const bool trackable_provisional =
            debug.reference_status_mode == "TRACK_PROVISIONAL_TAIL";
        debug.reference_trackable_mode =
            trackable_provisional ? "provisional_tail" :
            (trackable_raw_reference ? "degraded_raw_trackable" :
             (debug.reference_status_mode == "TRACK_GOOD" ? "good" : "degraded"));
        debug.ref_tracking_class = debug.reference_trackable_mode;
        if ((short_reference || conditioner_segment_short) &&
            !trackable_raw_reference &&
            !trackable_provisional &&
            !debug.reference_held) {
            debug.reference_held = true;
            debug.reference_quality = std::min(debug.reference_quality, 0.50);
            debug.control_mode = "SLOW_TRACK_DEGRADED";
            debug.hold_reason = short_reference ? "reference_span_short" : "conditioner_segment_short";
        }
        if (conditioned_reference.valid) {
            const std::vector<double> conditioned_curvatures =
                curvaturesFromPath(conditioned_reference.path, steering_curvatures.size());
            if (conditioned_curvatures.size() == steering_curvatures.size()) {
                steering_curvatures = conditioned_curvatures;
            }
        }
        updateStraightLock(problem, debug, now);
        applyStraightLockCurvatureSuppression(steering_curvatures, debug);
        const bool degraded_control = degradedControlActive(debug);
        if (degraded_control) {
            applyDegradedCurvatureLimits(steering_curvatures, debug);
        }
        updateTrackingFilters(problem, debug.candidate_kappa_eval, debug);
        const double vehicle_speed_for_control =
            vehicle_.speed_valid ? vehicle_.speed : filtered_cmd_.speed;
        UnifiedTrackingSignal tracking_signal =
            updateUnifiedTrackingSignal(problem, steering_curvatures, vehicle_speed_for_control, now, debug);
        PreviewGeometrySignal preview_signal =
            preview_tracker_.observe(*control_path, tracking_signal, vehicle_speed_for_control);
        const bool provisional_tail_short =
            trackable_provisional && debug.path_span < provisional_tail_curve_min_span_;
        if (preview_signal.valid && provisional_tail_short) {
            preview_signal.kappa_ff = 0.0;
            preview_signal.kappa_cmd = 0.0;
            preview_signal.kappa_pp_weighted = 0.0;
            preview_signal.curve_blend = 0.0;
            preview_signal.turn_blend_effective = 0.0;
            preview_signal.ff_reduce_scale = 1.0;
            preview_signal.lookahead_mode = "provisional_short";
            debug.reference_trackable_mode = "provisional_tail_short";
            debug.ref_tracking_class = debug.reference_trackable_mode;
        }
        const double reference_status_age =
            reference_status_.valid ? reference_status_.age : -1.0;
        debug.reference_status_age = reference_status_age;
        const bool reference_age_stable =
            reference_status_age >= curve_evidence_min_status_age_;
        const bool reference_mode_good =
            debug.reference_status_mode == "TRACK_GOOD" &&
            reference_age_stable &&
            debug.path_span >= curve_evidence_min_span_ &&
            debug.path_points >= curve_evidence_min_points_;
        const bool reference_mode_stable_provisional =
            debug.reference_status_mode == "TRACK_PROVISIONAL_TAIL" &&
            reference_status_age >= 1.0 &&
            debug.path_span >= degraded_reference_min_span_ &&
            !provisional_tail_short &&
            debug.path_points >= curve_evidence_min_points_;
        const bool reference_mode_stable_raw =
            debug.reference_status_mode == "TRACK_DEGRADED_RAW" &&
            reference_status_age >= 1.0 &&
            debug.path_span >= degraded_reference_min_span_ &&
            debug.path_points >= curve_evidence_min_points_;
        if (reference_mode_good) {
            debug.reference_confidence = 1.0;
        } else if (reference_mode_stable_provisional || reference_mode_stable_raw) {
            debug.reference_confidence = 0.65;
        } else if (debug.path_valid && debug.path_points >= 2 && debug.path_span > 0.0) {
            debug.reference_confidence = 0.25;
        } else {
            debug.reference_confidence = 0.0;
        }
        const bool low_confidence_reference =
            debug.reference_confidence < 0.50 ||
            debug.reference_held ||
            provisional_tail_short;
        if (preview_signal.valid) {
            const bool status_curve_capable =
                debug.reference_status_mode == "TRACK_GOOD" ||
                debug.reference_status_mode == "TRACK_DEGRADED_RAW" ||
                debug.reference_status_mode == "TRACK_PROVISIONAL_TAIL";
            const bool reference_status_stable =
                debug.reference_confidence >= 0.65;
            const bool short_or_unstable_reference =
                debug.reference_status_mode == "DEGRADED_HELD" ||
                debug.path_span < short_path_guard_span_ ||
                debug.reference_confidence < 0.65;
            const bool reference_curve_strong =
                std::fabs(preview_signal.kappa_path) > preview_tracker_.straightGuardCurveGate() ||
                std::fabs(preview_signal.kappa_ff) > preview_tracker_.straightGuardCurveGate() ||
                std::fabs(tracking_signal.kappa_ref) > preview_tracker_.straightGuardCurveGate();
            debug.curve_evidence_reliable =
                status_curve_capable &&
                reference_status_stable &&
                !debug.reference_held &&
                debug.path_span >= curve_evidence_min_span_ &&
                debug.path_points >= curve_evidence_min_points_ &&
                reference_curve_strong &&
                !preview_signal.sign_conflict;
            debug.ignored_short_path_kappa =
                short_or_unstable_reference &&
                !debug.curve_evidence_reliable &&
                (std::fabs(preview_signal.kappa_path) > preview_tracker_.straightGuardKappaGate() ||
                 std::fabs(preview_signal.kappa_ff) > preview_tracker_.straightGuardKappaGate() ||
                 std::fabs(tracking_signal.kappa_ref) > preview_tracker_.straightGuardKappaGate());
            const bool near_straight_reference =
                std::fabs(preview_signal.kappa_path) < preview_tracker_.straightGuardKappaGate() &&
                std::fabs(preview_signal.kappa_ff) < preview_tracker_.straightGuardKappaGate() &&
                std::fabs(tracking_signal.kappa_ref) < preview_tracker_.straightGuardCurveGate();
            const bool preview_x_too_short =
                preview_signal.x < preview_tracker_.straightGuardPreviewMinX() &&
                !reference_curve_strong;
            const bool low_confidence_straight =
                low_confidence_reference && near_straight_reference;
            if (preview_x_too_short || low_confidence_straight ||
                (short_or_unstable_reference && !debug.curve_evidence_reliable)) {
                preview_signal.preview_x_too_short = preview_x_too_short;
                preview_tracker_.applyStraightGuard(
                    &preview_signal,
                    preview_x_too_short ? "preview_x_too_short" :
                    (debug.ignored_short_path_kappa ? "ignored_short_path_kappa" :
                     (low_confidence_straight ? "low_confidence_straight" : "low_confidence_unstable")));
                tracking_signal.kappa_ref = 0.0;
                tracking_signal.curve_blend = 0.0;
            }
        }
        if (preview_signal.valid) {
            tracking_signal.yaw_rate_ref = vehicle_speed_for_control * preview_signal.kappa_ff;
            tracking_signal.yaw_rate_error = tracking_signal.yaw_rate_ref - filtered_yaw_rate_;
            tracking_signal.curve_blend = std::max(tracking_signal.curve_blend, preview_signal.curve_blend);
            debug.yaw_rate_ref = tracking_signal.yaw_rate_ref;
            debug.yaw_rate_error = tracking_signal.yaw_rate_error;
            debug.track_curve_blend = tracking_signal.curve_blend;
            debug.kappa_pp = preview_signal.kappa_pp;
            debug.kappa_path = preview_signal.kappa_path;
            debug.kappa_cmd = preview_signal.kappa_cmd;
            debug.kappa_ff = preview_signal.kappa_ff;
            debug.kappa_pp_weighted = preview_signal.kappa_pp_weighted;
            debug.psi_preview = preview_signal.psi;
            debug.psi_preview_used = preview_signal.psi_used;
            debug.preview_lookahead = preview_signal.lookahead;
            debug.lookahead_adaptive = preview_signal.lookahead;
            debug.preview_x = preview_signal.x;
            debug.preview_y = preview_signal.y;
            debug.path_sign_conflict = preview_signal.sign_conflict;
            debug.inside_error = preview_signal.inside_error;
            debug.inside_gain_scale = preview_signal.inside_gain_scale;
            debug.ff_reduce_scale = preview_signal.ff_reduce_scale;
            debug.kappa_control = preview_signal.kappa_ff;
            debug.turn_entry_s = preview_signal.turn_entry_s;
            debug.distance_to_turn_entry = preview_signal.distance_to_turn_entry;
            debug.turn_blend = preview_signal.turn_blend;
            debug.turn_blend_effective = preview_signal.turn_blend_effective;
            debug.segment_mode = preview_signal.segment_mode;
            debug.track_signed_e_y = preview_signal.cross_track_error;
            debug.front_axle_e_ct = preview_signal.front_axle_e_ct;
            debug.inside_error_front = preview_signal.inside_error_front;
            debug.lookahead_mode = preview_signal.lookahead_mode;
            debug.lookahead_limited_by_turn_entry = preview_signal.lookahead_limited_by_turn_entry;
            debug.kappa_ref_delay = preview_signal.kappa_ff;
            debug.steering_delay_predicted_e_y = preview_signal.front_axle_e_ct;
            debug.corner_cut_score = preview_signal.corner_cut_score;
            debug.straight_guard_active = preview_signal.straight_guard_active;
            debug.pp_suppressed_reason = preview_signal.pp_suppressed_reason;
            debug.preview_x_too_short = preview_signal.preview_x_too_short;
        }
        debug.heading_suspect = std::fabs(wrapAngle(filtered_e_theta_ - tracking_signal.e_course)) >
                                rail_heading_suspect_gate_;
        debug.e_theta_control = tracking_signal.e_course;

        const double gamma = cps_.gamma(now);
        const bool hard_stop = false;
        double target_speed = 0.0;
        double lead_speed = 0.0;
        double gap_error = 0.0;
        const std::vector<double> speed_profile =
            velocity_planner_.plan(steering_curvatures,
                                   gamma,
                                   vehicle_.speed_valid ? vehicle_.speed : filtered_cmd_.speed,
                                   lead,
                                   hard_stop,
                                   target_speed,
                                   lead_speed,
                                   gap_error);
        if (speed_profile.empty()) {
            return safeStop(now, "sqp_failed", debug);
        }

        const double previous_delta = filtered_cmd_.steering;
        double delayed_e_y = filtered_e_y_;
        double delayed_e_theta = debug.e_theta_control;
        const int delay_steps = predictDelayCompensatedState(
            steering_curvatures,
            vehicle_speed_for_control,
            delayed_e_y,
            delayed_e_theta);
        SpatialSLQR::SolveInfo slqr_info;
        slqr_.setRuntimeEnvelope(vehicle_speed_for_control, 0.45 * (1.0 - 0.35 * gamma), 9.80665);
        const bool slqr_preview_active =
            tracking_signal.curve_blend > 0.02 && preview_tracker_.slqrMaxWeight() > 1e-4;
        slqr_.setRuntimeStability(slqr_preview_active,
                                  filtered_yaw_rate_,
                                  vehicle_.steering_feedback);
        double delta_base = 0.0;
        if (slqr_preview_active) {
            delta_base = slqr_.solve(delayed_e_y, delayed_e_theta, steering_curvatures, previous_delta, slqr_info);
            updateBarrierAdaptation(slqr_info.barrier_margin);
        } else {
            slqr_.setAdaptScale(1.0);
            slqr_info.delta = 0.0;
            slqr_info.cost = 0.0;
            slqr_info.barrier_margin = 1.0;
            slqr_info.used_fallback = false;
            slqr_info.solver_ok = true;
            slqr_info.tracking_slack = 0.0;
            slqr_info.envelope_slack = 0.0;
            slqr_info.soft_constraint_active = false;
        }
        const double residual = cps_.signedSteeringResidual(now);
        const double dt = controlDt(now);
        debug.course_lock_active = true;
        std::string bias_block_reason = "none";
        const double node_age =
            node_start_stamp_.isValid() ? std::max(0.0, (now - node_start_stamp_).toSec()) : 0.0;
        const bool bias_reference_ok = debug.reference_confidence >= 0.65 &&
                                       debug.path_span >= degraded_reference_min_span_ &&
                                       reference_status_age >= 1.0 &&
                                       !debug.reference_held &&
                                       !provisional_tail_short;
        const bool bias_curve_ok =
            tracking_signal.curve_blend < 0.20 &&
            (!preview_signal.valid || std::fabs(preview_signal.kappa_cmd) < 0.12) &&
            std::fabs(tracking_signal.kappa_ref) < 0.12;
        const bool bias_lateral_ok =
            !preview_signal.valid || std::fabs(preview_signal.front_axle_e_ct) < 0.04;
        const bool bias_speed_ok =
            std::fabs(vehicle_speed_for_control) >= 0.08 &&
            std::fabs(vehicle_speed_for_control) <= 0.25;
        const bool bias_yaw_ok = debug.yaw_rate_valid;
        const bool startup_low_confidence =
            node_age < 8.0 && debug.reference_confidence < 0.65;
        if (!bias_reference_ok) {
            bias_block_reason = "reference_confidence";
        } else if (startup_low_confidence) {
            bias_block_reason = "startup_low_confidence";
        } else if (!bias_curve_ok) {
            bias_block_reason = "curve";
        } else if (!bias_lateral_ok) {
            bias_block_reason = "front_axle_error";
        } else if (!bias_speed_ok) {
            bias_block_reason = "speed";
        } else if (!bias_yaw_ok) {
            bias_block_reason = "yaw_feedback";
        } else if (collision.active) {
            bias_block_reason = "collision";
        } else if (vehicle_.chassis_error != 0) {
            bias_block_reason = "chassis";
        }
        debug.bias_update_allowed = bias_block_reason == "none";
        debug.bias_block_reason = bias_block_reason;
        steering_bias_.update(dt,
                              debug.bias_update_allowed,
                              collision.active,
                              vehicle_.chassis_error == 0,
                              std::fabs(vehicle_speed_for_control),
                              std::fabs(tracking_signal.kappa_ref),
                              -tracking_signal.signed_e_y,
                              tracking_signal.e_course,
                              filtered_yaw_rate_,
                              tracking_signal.yaw_rate_ref,
                              residual,
                              debug.bias_learning_active,
                              debug.bias_learning_suspended);
        debug.delta_bias = steering_bias_.bias();
        debug.bias_apply_scale =
            (debug.reference_confidence >= 0.65 && !startup_low_confidence) ? 1.0 : 0.0;
        const double applied_delta_bias = debug.delta_bias * debug.bias_apply_scale;
        double cps_comp = 0.0;
        const bool pause_cps_comp =
            !slqr_preview_active || degraded_control || std::fabs(residual) > residual_comp_pause_gate_;
        if (!pause_cps_comp && std::fabs(residual) > comp_deadband_) {
            cps_comp = cps_compensation_gain_ * residual;
        }
        const double cps_comp_limited = clampValue(cps_comp, -max_cps_comp_, max_cps_comp_);
        const double delta_rate_ff =
            (!slqr_preview_active || degraded_control)
                ? 0.0
                : delay_derivative_gain_ * (delta_base - last_slqr_delta_) / std::max(0.01, dt);
        const double delay_comp_limited = clampValue(delta_rate_ff, -max_delay_comp_, max_delay_comp_);
        const double raw_delta_comp = cps_comp_limited + delay_comp_limited;
        filtered_delta_comp_ =
            (1.0 - delta_comp_lpf_alpha_) * filtered_delta_comp_ + delta_comp_lpf_alpha_ * raw_delta_comp;
        const double comp_bound = std::max(0.0, max_cps_comp_) + std::max(0.0, max_delay_comp_);
        const double delta_comp = clampValue(filtered_delta_comp_, -comp_bound, comp_bound);
        filtered_delta_comp_ = delta_comp;
        const double speed_for_damping =
            std::max(v_min_, std::fabs(vehicle_.speed_valid ? vehicle_.speed : filtered_cmd_.speed));
        const double active_yaw_damping_gain =
            std::max(yaw_damping_gain_, straight_yaw_damping_gain_);
        const double yaw_damping_delta =
            clampValue(-active_yaw_damping_gain * filtered_yaw_rate_ / speed_for_damping,
                       -max_yaw_damping_delta_,
                       max_yaw_damping_delta_);
        double ramp_scale = 1.0;
        if (debug.reference_held) {
            ramp_scale = 0.65;
        } else if (!last_reference_hold_time_.isZero() && control_recovery_ramp_seconds_ > 1e-3) {
            const double since_hold = (now - last_reference_hold_time_).toSec();
            if (since_hold >= 0.0 && since_hold < control_recovery_ramp_seconds_) {
                ramp_scale = clampValue(since_hold / control_recovery_ramp_seconds_, 0.20, 1.0);
            }
        }
        debug.control_ramp_scale = ramp_scale;
        double active_steer_limit =
            degraded_control ? std::min(max_steer_, std::max(0.05, degraded_max_steer_)) : max_steer_;
        active_steer_limit = std::min(active_steer_limit, preview_tracker_.deltaMax());
        const bool straight_low_curvature_reference =
            preview_signal.valid &&
            std::fabs(preview_signal.kappa_cmd) < 0.15 &&
            (debug.segment_mode == "straight" || debug.segment_mode == "raw");
        if (low_confidence_reference && straight_low_curvature_reference) {
            active_steer_limit = std::min(active_steer_limit, degraded_hold_max_steer_);
            debug.low_confidence_steer_cap_active = true;
            debug.low_confidence_delta_cap = true;
            debug.reference_trackable_mode =
                provisional_tail_short ? "provisional_tail_short" : "low_confidence_straight";
            debug.ref_tracking_class = debug.reference_trackable_mode;
        } else if (low_confidence_reference) {
            active_steer_limit = std::min(active_steer_limit, std::max(0.05, degraded_hold_max_steer_));
            debug.low_confidence_steer_cap_active = true;
            debug.low_confidence_delta_cap = true;
            debug.reference_trackable_mode =
                provisional_tail_short ? "provisional_tail_short" : "low_confidence_reference";
            debug.ref_tracking_class = debug.reference_trackable_mode;
        }
        if (debug.straight_guard_active) {
            active_steer_limit = std::min(active_steer_limit, preview_tracker_.straightGuardDeltaMax());
            debug.low_confidence_delta_cap = true;
        }
        const UnifiedControlOutput unified_output =
            preview_tracker_.compute(tracking_signal,
                                     preview_signal,
                                     vehicle_speed_for_control,
                                     filtered_yaw_rate_,
                                     residual,
                                     applied_delta_bias,
                                     delta_base);
        debug.delta_ref = unified_output.delta_ref;
        debug.delta_ref_ff = unified_output.delta_ff;
        debug.delta_ref_course = unified_output.delta_course;
        debug.delta_lateral_raw = unified_output.delta_lateral_raw;
        debug.delta_ref_lateral = unified_output.delta_lateral;
        debug.delta_lateral_guarded = unified_output.delta_lateral;
        debug.delta_lateral_limited = unified_output.delta_lateral_limited;
        debug.delta_ref_ey_rate = unified_output.delta_ey_rate;
        debug.delta_ref_yaw = unified_output.delta_yaw;
        debug.unified_slqr_weight = unified_output.slqr_weight;
        debug.delta_course = unified_output.delta_ref;
        debug.delta_rail = unified_output.delta_ref;
        debug.rail_blend = 1.0 - unified_output.slqr_weight;
        debug.course_slqr_blend = 1.0 - unified_output.slqr_weight;
        double delta_target = unified_output.delta_ref;
        debug.control_mode = tracking_signal.curve_blend > 0.20 ? "UNIFIED_CURVE" : "UNIFIED_TRACK";
        delta_target = clampValue(delta_target, -active_steer_limit, active_steer_limit);
        delta_target = previous_delta + ramp_scale * (delta_target - previous_delta);
        delta_target = clampValue(delta_target, -active_steer_limit, active_steer_limit);
        const bool non_trackable_degraded_control =
            degraded_control && !trackable_raw_reference && !trackable_provisional;
        if (non_trackable_degraded_control) {
            delta_target = clampValue(delta_target,
                                      -degraded_hold_max_steer_,
                                      degraded_hold_max_steer_);
        }
        if (low_confidence_reference && straight_low_curvature_reference) {
            delta_target = clampValue(delta_target,
                                      -degraded_hold_max_steer_,
                                      degraded_hold_max_steer_);
        } else if (low_confidence_reference) {
            delta_target = clampValue(delta_target,
                                      -degraded_hold_max_steer_,
                                      degraded_hold_max_steer_);
        }
        const bool sign_crossing_fast =
            filtered_cmd_.steering * unified_output.delta_ref < -1e-4 &&
            (std::fabs(preview_signal.front_axle_e_ct) > 0.04 ||
             std::fabs(preview_signal.kappa_cmd) > 0.35 ||
             non_trackable_degraded_control);

        double curve_distance = -1.0;
        const double preview_min_speed =
            previewMinimumSpeed(speed_profile, steering_curvatures, curve_distance);
        debug.preview_min_v = preview_min_speed;
        debug.curve_distance = curve_distance;
        debug.preview_curve_speed_limit =
            previewCurveSpeedLimit(preview_signal,
                                   conditioned_reference.valid,
                                   debug.raw_path_kappa_p95,
                                   debug.curve_speed_reason);
        debug.speed_limit_reason = debug.curve_speed_reason;
        debug.curve_speed_cap = debug.preview_curve_speed_limit;
        debug.yaw_rate_bound = slqr_.yawRateBound();
        debug.tracking_speed_scale = trackingSpeedScale(tracking_signal);
        debug.oscillation_score = steeringOscillationScore(now, delta_target);
        SpeedArbitration speed_choice =
            arbitrateSpeed(preview_min_speed,
                           debug.preview_curve_speed_limit,
                           tracking_signal,
                           preview_signal,
                           lead,
                           collision,
                           degraded_control,
                           slqr_preview_active,
                           slqr_info.envelope_slack,
                           slqr_.activeAdaptScale(),
                           ramp_scale,
                           debug,
                           now);
        double planned_speed = clampValue(speed_choice.speed, 0.0, max_speed_);
        if (provisional_tail_short) {
            planned_speed = std::min(planned_speed, provisional_tail_short_speed_limit_);
            speed_choice.mode = "provisional_tail_short";
            speed_choice.cap_source = "provisional_tail_short";
        }
        if (debug.straight_guard_active) {
            planned_speed = std::min(planned_speed, straight_guard_speed_limit_);
            speed_choice.mode = "straight_guard";
            speed_choice.cap_source = "straight_guard";
        }
        if (debug.reference_confidence < 0.50) {
            planned_speed = std::min(planned_speed, low_confidence_speed_limit_);
            speed_choice.mode = "low_confidence_reference";
            speed_choice.cap_source = "low_confidence_reference";
        }
        if (degraded_control) {
            debug.held_reference_speed_limit =
                std::min(std::max(0.0, std::min(held_reference_max_speed_, degraded_speed_limit_)),
                         std::max(0.0, reference_bad_speed_scale_ * max_speed_));
            planned_speed = std::min(planned_speed, debug.held_reference_speed_limit);
            speed_choice.mode = "degraded_reference";
            speed_choice.cap_source = "degraded_reference";
        } else {
            debug.held_reference_speed_limit = 0.0;
        }
        if (!lead.valid &&
            (tracking_signal.curve_blend > target_lost_curve_blend_gate_ ||
             collision.last_close_target_age >= 0.0)) {
            debug.target_lost_slow_track = true;
            debug.straight_speed_limited = true;
        }
        if (debug.oscillation_score >= 1.0) {
            planned_speed *= oscillation_speed_scale_;
            speed_choice.mode = "oscillation";
            speed_choice.cap_source = "oscillation";
        }
        debug.visual_gap_urgency = speed_choice.visual_gap_urgency;
        debug.speed_arbitration_mode = speed_choice.mode;
        debug.curve_safe_cap = speed_choice.curve_safe_cap;
        debug.catchup_speed_floor = speed_choice.catchup_speed_floor;
        debug.risk_speed_floor = speed_choice.risk_speed_floor;
        debug.speed_cap_source = speed_choice.cap_source;
        debug.speed_risk_score = speed_choice.risk_score;
        debug.speed_stop_class = speed_choice.stop_class;
        debug.gap_speed_floor = speed_choice.gap_speed_floor;
        debug.inside_speed_scale =
            debug.curve_safe_cap > 1e-4
                ? clampValue(planned_speed / std::max(1e-4, debug.curve_safe_cap), 0.0, 2.0)
                : 0.0;
        debug.straight_speed_limited =
            debug.straight_speed_limited || debug.speed_arbitration_mode != "none";
        debug.commanded_v_ref = planned_speed;
        debug.sqp_preview_min_v = debug.preview_min_v;
        debug.final_speed_cap = planned_speed;
        const double speed_limited =
            planned_speed >= filtered_cmd_.speed
                ? approachWithLimit(filtered_cmd_.speed, planned_speed, max_accel_ * dt)
                : approachWithLimit(filtered_cmd_.speed, planned_speed, max_decel_ * dt);
        debug.accel_limited = std::fabs(speed_limited - planned_speed) > 1e-4;
        double active_steer_rate =
            max_steer_rate_ * (degraded_control ? degraded_steer_rate_scale_ : 1.0);
        if (sign_crossing_fast) {
            active_steer_rate = std::max(active_steer_rate, max_steer_rate_cross_);
        }
        if (debug.oscillation_score >= 1.0) {
            active_steer_rate *= oscillation_steer_rate_scale_;
        }
        const double steer_limited = approachWithLimit(filtered_cmd_.steering, delta_target, active_steer_rate * dt);

        const double active_speed_alpha =
            speed_limited < filtered_cmd_.speed ? speed_cmd_alpha_down_ : speed_cmd_alpha_up_;
        const double speed_before_filter = filtered_cmd_.speed;
        filtered_cmd_.speed += active_speed_alpha * (speed_limited - filtered_cmd_.speed);
        debug.speed_filter_limited = std::fabs(filtered_cmd_.speed - speed_limited) > 1e-4 &&
                                     std::fabs(speed_limited - speed_before_filter) > 1e-4;
        const double active_steer_alpha =
            sign_crossing_fast ? std::max(steer_cmd_alpha_, steer_cmd_alpha_cross_) : steer_cmd_alpha_;
        filtered_cmd_.steering += active_steer_alpha * (steer_limited - filtered_cmd_.steering);
        filtered_cmd_.speed = clampValue(filtered_cmd_.speed, 0.0, max_speed_);
        filtered_cmd_.steering = clampValue(filtered_cmd_.steering, -max_steer_, max_steer_);

        cps_.recordCommand(filtered_cmd_.steering, now);
        recordSteeringCommand(filtered_cmd_.steering, now);
        last_slqr_delta_ = delta_base;
        debug.gamma_cps = gamma;
        debug.steering_residual = residual;
        debug.imu_z_variance = cps_.imuZVariance();
        debug.gamma_steer_component = cps_.gammaSteerComponent();
        debug.gamma_imu_component = cps_.gammaImuComponent();
        debug.range_rate = lead.valid ? lead.range_rate : 0.0;
        debug.gap_error = gap_error;
        debug.lead_speed = lead_speed;
        debug.cruise_speed = target_speed;
        debug.planned_speed = planned_speed;
        debug.slqr_cost = slqr_info.cost;
        debug.barrier_margin = slqr_info.barrier_margin;
        debug.envelope_slack = slqr_info.envelope_slack;
        debug.tracking_slack = slqr_info.tracking_slack;
        debug.soft_constraint_active = slqr_info.soft_constraint_active;
        debug.sqp_jerk_cost = velocity_planner_.lastJerkCost();
        debug.delay_steps = delay_steps;
        debug.actuator_delay_est = steering_delay_seconds_;
        debug.delta_slqr = delta_base;
        debug.delta_ff = unified_output.delta_ff;
        debug.delta_fb = unified_output.delta_ref - unified_output.delta_ff;
        debug.yaw_rate_penalty =
            std::fabs(tracking_signal.yaw_rate_error) / std::max(1e-3, yaw_rate_slow_gate_);
        debug.delta_comp = delta_comp;
        debug.delta_cmd = filtered_cmd_.steering;
        debug.steer_feedback_for_control = vehicle_.steering_feedback;
        debug.yaw_rate = filtered_yaw_rate_;
        debug.yaw_damping_delta = yaw_damping_delta;
        debug.barrier_adapt_scale = slqr_.activeAdaptScale();
        debug.comp_clipped =
            std::fabs(cps_comp - cps_comp_limited) > 1e-6 ||
            std::fabs(delta_rate_ff - delay_comp_limited) > 1e-6;
        debug.steer_sign_mismatch = updateSteerSignDiagnostic(filtered_cmd_.steering, now);
        if (debug.bias_learning_active && std::fabs(debug.delta_bias) > 0.005) {
            debug.stop_reason = "tracking_bias_learning";
        } else if (tracking_signal.curve_blend > 0.20) {
            debug.stop_reason = "tracking_unified_curve";
        } else if (straight_lock_active_) {
            debug.stop_reason = "tracking_unified_straight";
        } else {
            debug.stop_reason = "tracking_unified";
        }
        if (debug.hold_reason != "none") {
            debug.stop_reason = "path_quality_hold_" + debug.hold_reason;
        }
        writeProfile(now,
                     steering_curvatures,
                     speed_profile,
                     debug.preview_min_v,
                     debug.commanded_v_ref,
                     debug.curve_distance);
        return filtered_cmd_;
    }

    int predictDelayCompensatedState(const std::vector<double>& curvatures,
                                     double speed,
                                     double& e_y,
                                     double& e_theta) const {
        if (!delay_compensation_enabled_ || curvatures.empty()) {
            return 0;
        }
        const double delay_distance = std::max(0.0, steering_delay_seconds_) *
                                      std::max(v_min_, std::fabs(speed));
        const int spatial_steps = static_cast<int>(
            std::ceil(delay_distance / std::max(0.03, spatial_step_)));
        const int steps = std::max(0, std::min(steering_delay_steps_, spatial_steps));
        if (steps <= 0) {
            return 0;
        }
        double delta = filtered_cmd_.steering;
        if (!steering_history_.empty()) {
            const ros::Time query = ros::Time::now() - ros::Duration(steering_delay_seconds_);
            double best_age = std::numeric_limits<double>::infinity();
            for (const TimedSteering& item : steering_history_) {
                const double age = std::fabs((item.stamp - query).toSec());
                if (age < best_age) {
                    best_age = age;
                    delta = item.value;
                }
            }
        }
        for (int i = 0; i < steps; ++i) {
            const double kappa = curvatures[static_cast<size_t>(std::min(i, static_cast<int>(curvatures.size()) - 1))];
            const double ey = clampValue(e_y, -3.0, 3.0);
            const double eth = clampValue(e_theta, -1.40, 1.40);
            const double cos_eth = std::max(0.18, std::fabs(std::cos(eth)));
            const double geometry = clampValue(1.0 - kappa * ey, 0.20, 2.80);
            const double ey_prime = geometry * std::tan(eth);
            const double eth_prime = std::tan(delta) / std::max(0.05, wheelbase_) *
                                         geometry / cos_eth -
                                     kappa;
            e_y = clampValue(e_y + spatial_step_ * ey_prime, -3.0, 3.0);
            e_theta = clampValue(wrapAngle(e_theta + spatial_step_ * eth_prime), -1.40, 1.40);
        }
        return steps;
    }

    void recordSteeringCommand(double steering, const ros::Time& now) {
        steering_history_.push_back(TimedSteering{now, steering});
        while (!steering_history_.empty() &&
               (now - steering_history_.front().stamp).toSec() > steering_history_seconds_) {
            steering_history_.pop_front();
        }
    }

    void writePathSnapshot(const PathBuffer& path) {
        if (!enable_debug_csv_logs_ || !path_log_) {
            return;
        }
        const int seq = ++path_log_seq_;
        for (size_t i = 0; i < path.points.size(); ++i) {
            const PathPoint& pt = path.points[i];
            path_log_ << std::fixed << std::setprecision(6) << path.stamp.toSec() << ","
                      << path.source << ","
                      << seq << ","
                      << i << ","
                      << pt.s << ","
                      << pt.x << ","
                      << pt.y << ","
                      << pt.yaw << "\n";
        }
        path_log_.flush();
    }

    void writeProfile(const ros::Time& now,
                      const std::vector<double>& curvatures,
                      const std::vector<double>& speed_profile,
                      double preview_min_v,
                      double commanded_v_ref,
                      double curve_distance) {
        if (!enable_debug_csv_logs_ || !profile_log_) {
            return;
        }
        ++profile_cycle_count_;
        if (profile_cycle_count_ % profile_log_stride_ != 0) {
            return;
        }
        const int seq = ++profile_log_seq_;
        const size_t n = std::max(curvatures.size(), speed_profile.size());
        for (size_t i = 0; i < n; ++i) {
            const double kappa = i < curvatures.size() ? curvatures[i] : 0.0;
            const double v_ref = i < speed_profile.size() ? speed_profile[i] : 0.0;
            profile_log_ << std::fixed << std::setprecision(6) << now.toSec() << ","
                         << seq << ","
                         << i << ","
                         << kappa << ","
                         << v_ref << ","
                         << preview_min_v << ","
                         << commanded_v_ref << ","
                         << curve_distance << "\n";
        }
        profile_log_.flush();
    }

    void writeRuntimeDebug(const ros::Time& now,
                           const ControllerDebug& debug,
                           const ControlCmd& cmd) {
        if (!enable_debug_csv_logs_ || !runtime_log_) {
            return;
        }
        runtime_log_ << std::fixed << std::setprecision(6) << now.toSec() << ","
                     << (debug.path_valid ? 1 : 0) << ","
                     << (debug.using_raw_fallback ? 1 : 0) << ","
                     << debug.path_points << ","
                     << debug.path_span << ","
                     << debug.e_y << ","
                     << debug.e_theta << ","
                     << debug.first_curvature << ","
                     << debug.fit_rms << ","
                     << debug.reference_quality << ","
                     << debug.hold_reason << ","
                     << debug.heading_jump << ","
                     << debug.kappa_jump << ","
                     << (debug.target_valid ? 1 : 0) << ","
                     << debug.gap << ","
                     << debug.gap_error << ","
                     << debug.range_rate << ","
                     << debug.lead_speed << ","
                     << debug.gamma_cps << ","
                     << debug.steering_residual << ","
                     << debug.imu_z_variance << ","
                     << debug.gamma_steer_component << ","
                     << debug.gamma_imu_component << ","
                     << debug.cruise_speed << ","
                     << debug.planned_speed << ","
                     << cmd.speed << ","
                     << debug.slqr_cost << ","
                     << debug.barrier_margin << ","
                     << debug.delay_steps << ","
                     << debug.delta_slqr << ","
                     << debug.delta_comp << ","
                     << cmd.steering << ","
                     << debug.steer_feedback_for_control << ","
                     << debug.yaw_rate << ","
                     << debug.yaw_rate_raw_imu << ","
                     << debug.yaw_rate_feedback_est << ","
                     << (debug.yaw_rate_valid ? 1 : 0) << ","
                     << debug.yaw_rate_source << ","
                     << debug.yaw_damping_delta << ","
                     << (debug.steer_sign_mismatch ? 1 : 0) << ","
                     << debug.barrier_adapt_scale << ","
                     << debug.kappa_control << ","
                     << (debug.reference_held ? 1 : 0) << ","
                     << debug.path_stale_reason << ","
                     << debug.control_ramp_scale << ","
                     << debug.held_reference_speed_limit << ","
                     << debug.candidate_e_y << ","
                     << debug.candidate_e_theta << ","
                     << debug.candidate_kappa0 << ","
                     << debug.candidate_kappa_eval << ","
                     << debug.candidate_fit_rms << ","
                     << debug.candidate_span << ","
                     << debug.candidate_points << ","
                     << debug.last_good_age << ","
                     << debug.reference_age << ","
                     << debug.raw_age << ","
                     << debug.reject_stage << ","
                     << debug.control_mode << ","
                     << debug.reference_status_mode << ","
                     << debug.reference_status_reason << ","
                     << debug.hard_stop_reason << ","
                     << debug.soft_recovery_reason << ","
                     << debug.filtered_e_y << ","
                     << debug.filtered_e_theta << ","
                     << debug.filtered_kappa_eval << ","
                     << debug.yaw_rate_bound << ","
                     << debug.envelope_slack << ","
                     << debug.tracking_slack << ","
                     << debug.sqp_jerk_cost << ","
                     << (debug.soft_constraint_active ? 1 : 0) << ","
                     << (debug.straight_lock ? 1 : 0) << ","
                     << debug.straight_confidence << ","
                     << debug.line_fit_rms << ","
                     << debug.line_yaw << ","
                     << debug.kappa_raw_front << ","
                     << debug.kappa_trimmed << ","
                     << debug.kappa_suppressed << ","
                     << debug.delta_ff << ","
                     << debug.delta_fb << ","
                     << debug.yaw_rate_penalty << ","
                     << debug.oscillation_score << ","
                     << (debug.straight_speed_limited ? 1 : 0) << ","
                     << debug.raw_chassis_error << ","
                     << (debug.heading_suspect ? 1 : 0) << ","
                     << debug.e_theta_control << ","
                     << debug.delta_rail << ","
                     << debug.rail_blend << ","
                     << debug.nearest_obstacle << ","
                     << (debug.collision_stop_active ? 1 : 0) << ","
                     << debug.collision_stop_age << ","
                     << debug.collision_resume_distance << ","
                     << debug.collision_source << ","
                     << debug.course_error << ","
                     << debug.course_error_source << ","
                     << debug.e_y_rate << ","
                     << debug.delta_course << ","
                     << debug.delta_bias << ","
                     << debug.bias_apply_scale << ","
                     << (debug.bias_update_allowed ? 1 : 0) << ","
                     << debug.bias_block_reason << ","
                     << debug.reference_confidence << ","
                     << (debug.bias_learning_active ? 1 : 0) << ","
                     << (debug.bias_learning_suspended ? 1 : 0) << ","
                     << (debug.course_lock_active ? 1 : 0) << ","
                     << debug.course_slqr_blend << ","
                     << debug.yaw_rate_ref << ","
                     << debug.yaw_rate_error << ","
                     << debug.track_signed_e_y << ","
                     << debug.track_kappa_ref << ","
                     << debug.track_beta_est << ","
                     << debug.track_curve_blend << ","
                     << debug.delta_ref << ","
                     << debug.delta_ref_ff << ","
                     << debug.delta_ref_course << ","
                     << debug.delta_ref_lateral << ","
                     << debug.delta_ref_ey_rate << ","
                     << debug.delta_ref_yaw << ","
                     << debug.unified_slqr_weight << ","
                     << debug.tracking_speed_scale << ","
                     << debug.kappa_ref_filtered << ","
                     << (debug.kappa_ref_rate_limited ? 1 : 0) << ","
                     << (debug.target_lost_slow_track ? 1 : 0) << ","
                     << debug.collision_latch_source << ","
                     << debug.last_close_target_age << ","
                     << debug.kappa_pp << ","
                     << debug.kappa_path << ","
                     << debug.kappa_cmd << ","
                     << debug.kappa_ff << ","
                     << debug.kappa_pp_weighted << ","
                     << debug.psi_preview << ","
                     << debug.psi_preview_used << ","
                     << debug.preview_lookahead << ","
                     << debug.preview_x << ","
                     << debug.preview_y << ","
                     << (debug.path_sign_conflict ? 1 : 0) << ","
                     << debug.inside_error << ","
                     << debug.inside_gain_scale << ","
                     << debug.ff_reduce_scale << ","
                     << debug.lookahead_adaptive << ","
                     << debug.actuator_delay_est << ","
                     << debug.nearest_cloud_obstacle << ","
                     << debug.collision_point_count << ","
                     << debug.cloud_frame << ","
                     << (debug.cloud_tf_ok ? 1 : 0) << ","
                     << (debug.cloud_tf_failed ? 1 : 0) << ","
                     << debug.cloud_tf_age << ","
                     << debug.nearest_cloud_x << ","
                     << debug.nearest_cloud_y << ","
                     << debug.collision_raw_nearest << ","
                     << debug.collision_effective_nearest << ","
                     << debug.preview_min_v << ","
                     << debug.commanded_v_ref << ","
                     << debug.curve_distance << ","
                     << debug.preview_curve_speed_limit << ","
                     << debug.curve_speed_reason << ","
                     << (debug.reference_conditioner_valid ? 1 : 0) << ","
                     << debug.conditioned_fit_rms << ","
                     << debug.conditioned_kappa << ","
                     << debug.conditioned_kappa_rate << ","
                     << debug.raw_path_kappa_p95 << ","
                     << debug.turn_entry_s << ","
                     << debug.distance_to_turn_entry << ","
                     << debug.turn_blend << ","
                     << debug.segment_mode << ","
                     << debug.conditioner_reject_reason << ","
                     << debug.speed_limit_reason << ","
                     << debug.curve_speed_cap << ","
                     << debug.inside_speed_scale << ","
                     << debug.sqp_preview_min_v << ","
                     << debug.final_speed_cap << ","
                     << debug.visual_gap_urgency << ","
                     << debug.speed_arbitration_mode << ","
                     << debug.curve_safe_cap << ","
                     << debug.catchup_speed_floor << ","
                     << debug.risk_speed_floor << ","
                     << debug.speed_cap_source << ","
                     << (debug.cloud_msg_received ? 1 : 0) << ","
                     << debug.cloud_age << ","
                     << (debug.cloud_source_active ? 1 : 0) << ","
                     << (debug.cloud_parse_failed ? 1 : 0) << ","
                     << debug.front_axle_e_ct << ","
                     << debug.inside_error_front << ","
                     << debug.lookahead_mode << ","
                     << (debug.lookahead_limited_by_turn_entry ? 1 : 0) << ","
                     << debug.kappa_ref_delay << ","
                     << debug.steering_delay_predicted_e_y << ","
                     << debug.corner_cut_score << ","
                     << debug.turn_blend_effective << ","
                     << (debug.delta_lateral_limited ? 1 : 0) << ","
                     << debug.speed_risk_score << ","
                     << debug.speed_stop_class << ","
                     << debug.reference_trackable_mode << ","
                     << debug.gap_speed_floor << ","
                     << (debug.speed_filter_limited ? 1 : 0) << ","
                     << (debug.accel_limited ? 1 : 0) << ","
                     << debug.ref_tracking_class << ","
                     << (debug.straight_guard_active ? 1 : 0) << ","
                     << debug.pp_suppressed_reason << ","
                     << (debug.preview_x_too_short ? 1 : 0) << ","
                     << debug.delta_lateral_raw << ","
                     << debug.delta_lateral_guarded << ","
                     << debug.reference_status_age << ","
                     << (debug.low_confidence_steer_cap_active ? 1 : 0) << ","
                     << (debug.curve_evidence_reliable ? 1 : 0) << ","
                     << (debug.ignored_short_path_kappa ? 1 : 0) << ","
                     << (debug.low_confidence_delta_cap ? 1 : 0) << ","
                     << (debug.imu_yaw_rate_unavailable ? 1 : 0) << ","
                     << debug.yaw_feedback_source_age << ","
                     << debug.yaw_rate_damping_source << ","
                     << (debug.comp_clipped ? 1 : 0) << ","
                     << (debug.safe_stop ? 1 : 0) << ","
                     << debug.stop_reason << "\n";
        runtime_log_.flush();
    }

    void controlTimer(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        ControllerDebug debug;
        ControlCmd cmd = computeCommand(now, debug);
        last_control_time_ = now;

        geometry_msgs::Twist msg;
        msg.linear.x = cmd.speed;
        msg.angular.z = cmd_steer_sign_ * cmd_steer_scale_ * cmd.steering;
        cmd_pub_.publish(msg);
        writeRuntimeDebug(now, debug, cmd);

        ROS_INFO_THROTTLE(
            0.5,
            "virtual_towing | path=%d raw_fb=%d pts=%d span=%.2f e_y=%.3f e_th=%.2fdeg k0=%.3f "
            "fit_rms=%.3f ref_q=%.2f hold=%s hj=%.2f kj=%.2f target=%d gap=%.2f gap_error=%.2f range_rate=%.2f v_lead=%.2f "
            "gamma=%.3f(gs=%.3f,gi=%.3f) res=%.3f fb=%.3f yaw_rate=%.3f yaw_damp=%.3f smm=%d imu_var=%.4f "
            "v_target=%.2f v_plan=%.2f cmd_v=%.2f held=%d ramp=%.2f hlim=%.2f slqr_cost=%.2f barrier_margin=%.3f badapt=%.2f "
            "delay_steps=%d delta_slqr=%.3f comp=%.3f cclip=%d cmd_delta=%.3f kctl=%.3f "
            "straight=%d conf=%.2f line_rms=%.3f line_yaw=%.2fdeg ktrim=%.3f course=%.2fdeg cbias=%.3f dc=%.3f cl=%d osc=%.2f stop=%d reason=%s",
            debug.path_valid ? 1 : 0,
            debug.using_raw_fallback ? 1 : 0,
            debug.path_points,
            debug.path_span,
            debug.e_y,
            debug.e_theta * 180.0 / M_PI,
            debug.first_curvature,
            debug.fit_rms,
            debug.reference_quality,
            debug.hold_reason.c_str(),
            debug.heading_jump,
            debug.kappa_jump,
            debug.target_valid ? 1 : 0,
            debug.gap,
            debug.gap_error,
            debug.range_rate,
            debug.lead_speed,
            debug.gamma_cps,
            debug.gamma_steer_component,
            debug.gamma_imu_component,
            debug.steering_residual,
            debug.steer_feedback_for_control,
            debug.yaw_rate,
            debug.yaw_damping_delta,
            debug.steer_sign_mismatch ? 1 : 0,
            debug.imu_z_variance,
            debug.cruise_speed,
            debug.planned_speed,
            msg.linear.x,
            debug.reference_held ? 1 : 0,
            debug.control_ramp_scale,
            debug.held_reference_speed_limit,
            debug.slqr_cost,
            debug.barrier_margin,
            debug.barrier_adapt_scale,
            debug.delay_steps,
            debug.delta_slqr,
            debug.delta_comp,
            debug.comp_clipped ? 1 : 0,
            cmd.steering,
            debug.kappa_control,
            debug.straight_lock ? 1 : 0,
            debug.straight_confidence,
            debug.line_fit_rms,
            debug.line_yaw * 180.0 / M_PI,
            debug.kappa_trimmed,
            debug.course_error * 180.0 / M_PI,
            debug.delta_bias,
            debug.delta_course,
            debug.course_lock_active ? 1 : 0,
            debug.oscillation_score,
            debug.safe_stop ? 1 : 0,
            debug.stop_reason.c_str());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber reference_sub_;
    ros::Subscriber raw_sub_;
    ros::Subscriber reference_status_sub_;
    ros::Subscriber tracked_sub_;
    ros::Subscriber obstacle_poses_sub_;
    ros::Subscriber filtered_cloud_sub_;
    ros::Subscriber cluster_cloud_sub_;
    ros::Subscriber car_sub_;
    ros::Subscriber carvel_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ned_odom_sub_;
    ros::Subscriber system_speed_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    CPSStateObserver cps_;
    LeadTargetObserver lead_observer_;
    CollisionSupervisor collision_;
    SteeringBiasObserver steering_bias_;
    UnifiedPathTracker unified_tracker_;
    PreviewGeometryTracker preview_tracker_;
    ReferenceConditioner reference_conditioner_;
    FlatnessErrorExtractor extractor_;
    CustomVelocityPlanner velocity_planner_;
    SpatialSLQR slqr_;

    VehicleState vehicle_;
    ControlCmd filtered_cmd_;
    PathBuffer reference_path_;
    PathBuffer raw_path_;
    ros::Time last_control_time_;
    std::deque<TimedSteering> steering_history_;
    std::ofstream runtime_log_;
    std::ofstream path_log_;
    std::ofstream profile_log_;

    std::string reference_topic_;
    std::string raw_topic_;
    std::string reference_status_topic_;
    std::string tracked_topic_;
    std::string obstacle_poses_topic_;
    std::string filtered_cloud_topic_;
    std::string cluster_cloud_topic_;
    std::string collision_cloud_target_frame_ = "base_link";
    std::string car_topic_;
    std::string carvel_topic_;
    std::string imu_topic_;
    std::string ned_odom_topic_;
    std::string system_speed_topic_;
    std::string cmd_topic_;
    double collision_tf_timeout_ = 0.03;

    double control_rate_ = 30.0;
    double wheelbase_ = 0.60;
    double v_min_ = 0.10;
    double max_speed_ = 0.50;
    double max_accel_ = 0.55;
    double max_decel_ = 0.90;
    double max_steer_ = 0.785398;
    double max_steer_rate_ = 1.20;
    double spatial_step_ = 0.10;
    double feedback_steer_sign_ = -1.0;
    double cmd_steer_sign_ = 1.0;
    double cmd_steer_scale_ = 1.0;
    double wheel_speed_scale_ = 0.001;
    double carvel_speed_scale_ = 0.001;
    bool carvel_angular_is_steer_ = true;
    double carvel_yaw_rate_sign_ = 1.0;
    double system_speed_scale_ = 1.0;
    double ned_speed_scale_ = 1.0;
    double speed_filter_alpha_ = 0.35;
    double steering_filter_alpha_ = 0.45;
    double yaw_rate_filter_alpha_ = 0.35;
    double yaw_rate_imu_timeout_ = 0.30;
    double yaw_rate_imu_deadband_ = 0.002;
    double yaw_rate_feedback_deadband_ = 0.003;
    double yaw_rate_feedback_blend_when_imu_zero_ = 0.85;
    double imu_yaw_zero_unavailable_time_ = 1.0;
    ros::Time node_start_stamp_;
    double last_imu_yaw_rate_raw_ = 0.0;
    ros::Time last_imu_yaw_rate_stamp_;
    ros::Time last_imu_nonzero_yaw_rate_stamp_;
    bool has_imu_yaw_rate_ = false;
    double reference_timeout_ = 0.35;
    double raw_timeout_ = 0.35;
    bool allow_raw_path_fallback_ = false;
    int min_path_points_ = 2;
    double min_path_span_ = 0.18;
    double desired_follow_distance_ = 2.0;
    double hard_stop_distance_ = 0.65;
    double cps_compensation_gain_ = 0.30;
    double max_cps_comp_ = 0.06;
    bool delay_compensation_enabled_ = true;
    double steering_delay_seconds_ = 0.20;
    int steering_delay_steps_ = 4;
    double delay_derivative_gain_ = 0.12;
    double max_delay_comp_ = 0.08;
    double delta_comp_lpf_alpha_ = 0.25;
    double comp_deadband_ = 0.004;
    double steering_history_seconds_ = 2.0;
    double last_slqr_delta_ = 0.0;
    double filtered_delta_comp_ = 0.0;
    bool reference_quality_gate_enabled_ = true;
    double reference_max_abs_e_theta_ = 1.20;
    double reference_e_theta_reject_gate_ = 1.55;
    double reference_e_theta_reject_persist_ = 0.80;
    double reference_max_abs_kappa0_ = 1.35;
    double reference_max_fit_rms_ = 0.18;
    double reference_max_heading_jump_ = 0.75;
    double reference_max_kappa_jump_ = 0.55;
    double reference_hold_timeout_ = 0.45;
    double degraded_reference_min_span_ = 1.20;
    double reference_bad_speed_scale_ = 0.36;
    double held_reference_max_speed_ = 0.14;
    double provisional_tail_curve_min_span_ = 0.35;
    double provisional_tail_short_speed_limit_ = 0.14;
    double straight_guard_speed_limit_ = 0.12;
    double low_confidence_speed_limit_ = 0.08;
    double curve_evidence_min_span_ = 1.00;
    int curve_evidence_min_points_ = 8;
    double curve_evidence_min_status_age_ = 0.70;
    double short_path_guard_span_ = 0.80;
    double control_recovery_ramp_seconds_ = 0.50;
    double kappa_degrade_gate_ = 1.35;
    double kappa_reject_gate_ = 1.70;
    double kappa_reject_persist_ = 0.50;
    double reference_kappa_jump_reject_gate_ = 1.60;
    double reference_kappa_jump_reject_persist_ = 0.80;
    double degraded_speed_limit_ = 0.10;
    double degraded_kappa_scale_ = 0.55;
    double degraded_max_kappa_control_ = 0.90;
    double degraded_max_steer_ = 0.34;
    double degraded_steer_rate_scale_ = 0.55;
    double candidate_kappa_eval_window_ = 0.80;
    double reference_soft_timeout_ = 0.35;
    double reference_hard_timeout_ = 1.20;
    double yaw_damping_gain_ = 0.10;
    double max_yaw_damping_delta_ = 0.08;
    double yaw_rate_slow_gate_ = 0.08;
    double yaw_rate_speed_scale_ = 0.65;
    double residual_comp_pause_gate_ = 0.18;
    double kappa_control_alpha_ = 0.35;
    int kappa_preview_count_ = 3;
    double kappa_preview_mix_ = 0.40;
    double filtered_kappa0_ = 0.0;
    bool has_filtered_kappa_ = false;
    double tracking_filter_alpha_y_ = 0.30;
    double tracking_filter_alpha_theta_ = 0.24;
    double tracking_filter_alpha_kappa_ = 0.28;
    double tracking_filter_alpha_yaw_rate_ = 0.35;
    double tracking_filter_theta_innovation_gate_ = 0.55;
    double yaw_envelope_speed_scale_ = 0.70;
    double control_curvature_ignore_front_s_ = 0.25;
    double control_curvature_trim_start_s_ = 0.40;
    double control_curvature_trim_end_s_ = 1.50;
    bool straight_lock_enabled_ = true;
    double straight_lock_fit_rms_gate_ = 0.04;
    double straight_lock_fit_rms_exit_gate_ = 0.08;
    double straight_lock_yaw_gate_ = 0.18;
    double straight_lock_yaw_exit_gate_ = 0.30;
    double straight_lock_kappa_gate_ = 0.35;
    double straight_lock_kappa_exit_gate_ = 0.55;
    double straight_lock_enter_time_ = 0.30;
    double straight_lock_exit_time_ = 0.50;
    double straight_lock_max_kappa_ = 0.18;
    double straight_lock_kappa_scale_ = 0.18;
    double straight_yaw_recovery_gate_ = 0.045;
    double straight_yaw_recovery_release_gate_ = 0.030;
    double straight_yaw_recovery_release_time_ = 0.40;
    double straight_yaw_recovery_speed_limit_ = 0.12;
    double straight_yaw_damping_gain_ = 0.16;
    double straight_delta_deadband_ = 0.025;
    double straight_delta_ff_limit_ = 0.06;
    double straight_max_steer_ = 0.22;
    double straight_steer_rate_scale_ = 0.45;
    double straight_sway_speed_limit_ = 0.18;
    bool rail_enabled_ = true;
    double rail_lateral_gate_ = 0.05;
    double rail_line_yaw_gate_ = 0.22;
    double rail_heading_suspect_gate_ = 0.22;
    double rail_theta_filter_gain_ = 0.35;
    double rail_theta_limit_ = 0.16;
    double rail_gain_y_ = 0.80;
    double rail_gain_line_yaw_ = 0.20;
    double rail_gain_yaw_rate_ = 0.12;
    double rail_gain_steering_ = 0.25;
    double rail_max_delta_ = 0.12;
    double rail_slqr_weight_ = 0.15;
    bool course_lock_enabled_ = true;
    double course_lock_lateral_gate_ = 0.07;
    double course_lock_line_yaw_gate_ = 0.28;
    double course_lock_kappa_gate_ = 0.18;
    double course_lock_gain_course_ = 0.55;
    double course_lock_gain_lateral_ = 0.75;
    double course_lock_gain_yaw_rate_ = 0.18;
    double course_lock_soft_speed_ = 0.12;
    double course_lock_max_delta_ = 0.20;
    double course_lock_slqr_weight_ = 0.20;
    double course_error_ey_rate_gain_ = 0.35;
    double course_error_beta_gain_ = 0.65;
    double course_filter_alpha_ = 0.35;
    double course_ey_rate_alpha_ = 0.25;
    double track_course_filter_alpha_ = 0.30;
    double track_ey_rate_alpha_ = 0.28;
    double track_course_ey_rate_gain_ = 0.45;
    double track_beta_gain_ = 0.65;
    double track_imu_yaw_course_gain_ = 0.10;
    double track_course_slow_gate_ = 0.25;
    double track_course_crawl_gate_ = 0.55;
    double track_lateral_slow_gate_ = 0.05;
    double track_lateral_crawl_gate_ = 0.10;
    double track_min_speed_scale_ = 0.45;
    double straight_stable_speed_limit_ = 0.22;
    double straight_stable_course_gate_ = 0.10;
    double straight_stable_lateral_gate_ = 0.025;
    double straight_stable_time_ = 1.0;
    double kappa_ref_lpf_alpha_ = 0.28;
    double kappa_ref_rate_limit_ = 0.55;
    double filtered_kappa_ref_ = 0.0;
    bool filtered_kappa_ref_initialized_ = false;
    double curve_speed_kappa_gate_ = 0.18;
    double curve_speed_limit_ = 0.16;
    double sharp_curve_kappa_gate_ = 0.95;
    double sharp_curve_speed_limit_ = 0.12;
    double target_lost_curve_blend_gate_ = 0.20;
    double target_lost_curve_speed_limit_ = 0.09;
    double preview_curve_speed_kappa_low_ = 0.35;
    double preview_curve_speed_kappa_high_ = 0.95;
    double preview_curve_speed_limit_low_ = 0.24;
    double preview_curve_speed_limit_high_ = 0.12;
    double sign_conflict_speed_limit_ = 0.08;
    double conditioner_invalid_speed_limit_ = 0.08;
    double conditioner_invalid_raw_kappa_gate_ = 4.0;
    double curve_recovery_min_speed_ = 0.12;
    double inside_error_speed_scale_max_ = 0.25;
    bool tracking_speed_use_kappa_scale_ = false;
    double preview_speed_start_s_ = 0.60;
    double preview_speed_end_s_ = 1.20;
    double visual_gap_start_ = 2.30;
    double visual_gap_full_ = 3.30;
    double visual_gap_close_ = 1.80;
    double visual_catchup_max_speed_ = 0.30;
    double visual_catchup_straight_floor_ = 0.22;
    double visual_catchup_curve_floor_ = 0.16;
    double visual_catchup_sharp_floor_ = 0.11;
    double visual_catchup_yaw_gate_ = 0.075;
    double visual_catchup_inside_gate_ = 0.060;
    double yaw_risk_persist_ = 0.35;
    ros::Time yaw_risk_start_;
    double gap_speed_floor_start_ = 2.70;
    double gap_curve_speed_floor_ = 0.16;
    double gap_straight_speed_floor_ = 0.20;
    double gap_sharp_speed_floor_ = 0.13;
    double risk_inside_error_gate_ = 0.080;
    double oscillation_window_ = 0.80;
    double oscillation_deadband_ = 0.025;
    int oscillation_flip_gate_ = 3;
    double oscillation_speed_scale_ = 0.70;
    double oscillation_steer_rate_scale_ = 0.55;
    bool straight_lock_active_ = false;
    bool straight_yaw_recovery_active_ = false;
    ros::Time straight_enter_start_;
    ros::Time straight_exit_start_;
    ros::Time straight_yaw_recovery_clear_start_;
    bool straight_stable_active_ = false;
    ros::Time straight_stable_start_;
    bool tracking_filter_initialized_ = false;
    double filtered_e_y_ = 0.0;
    double filtered_e_theta_ = 0.0;
    double filtered_kappa_eval_ = 0.0;
    double filtered_yaw_rate_ = 0.0;
    double filtered_e_y_rate_ = 0.0;
    double filtered_course_error_ = 0.0;
    bool course_filter_initialized_ = false;
    double last_course_e_y_ = 0.0;
    ros::Time last_course_stamp_;
    double ned_course_yaw_ = 0.0;
    double ned_pose_yaw_ = 0.0;
    ros::Time ned_course_stamp_;
    ros::Time ned_pose_yaw_stamp_;
    bool ned_course_valid_ = false;
    bool ned_pose_yaw_valid_ = false;
    int low_barrier_count_ = 0;
    double steer_sign_diag_window_ = 1.0;
    double steer_sign_diag_min_corr_ = 0.50;
    std::deque<TimedControlSample> sign_diag_samples_;
    PathTrackingProblem last_good_problem_;
    ros::Time last_good_problem_stamp_;
    bool has_last_good_problem_ = false;
    double last_good_e_theta_ = 0.0;
    double last_good_kappa0_ = 0.0;
    double last_good_kappa_eval_ = 0.0;
    ros::Time last_reference_hold_time_;
    ros::Time kappa_extreme_start_;
    ros::Time kappa_jump_extreme_start_;
    ros::Time e_theta_extreme_start_;
    bool was_reference_held_ = false;
    ReferenceStatus reference_status_;
    ros::Time reference_status_mode_since_;
    bool enable_debug_csv_logs_ = true;
    std::string debug_log_dir_ = "/home/ubuntu/drive_car_copy/cdata";
    std::string runtime_debug_file_ = "runtime_debug.csv";
    std::string path_debug_file_ = "path_snapshot.csv";
    std::string profile_debug_file_ = "profile_debug.csv";
    int profile_log_stride_ = 3;
    int path_log_seq_ = 0;
    int profile_log_seq_ = 0;
    int profile_cycle_count_ = 0;
    double speed_cmd_alpha_ = 0.32;
    double speed_cmd_alpha_up_ = 0.45;
    double speed_cmd_alpha_down_ = 0.75;
    double steer_cmd_alpha_ = 0.28;
    double steer_cmd_alpha_cross_ = 0.75;
    double max_steer_rate_cross_ = 1.80;
    double degraded_hold_max_steer_ = 0.10;
    bool stale_path_stop_ = true;
    bool publish_zero_on_stop_ = false;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "virtual_towing_controller");
    VirtualTowingNode node;
    ros::spin();
    return 0;
}
