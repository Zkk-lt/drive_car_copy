#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <move_car/car_parameter.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
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
    double yaw_damping_delta = 0.0;
    bool steer_sign_mismatch = false;
    double barrier_adapt_scale = 1.0;
    double kappa_control = 0.0;
    bool reference_held = false;
    std::string path_stale_reason = "none";
    double control_ramp_scale = 1.0;
    double held_reference_speed_limit = 0.0;
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
        hard_stop_distance_ = pnh.param("hard_stop_distance", 0.65);
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
    double hard_stop_distance_ = 0.65;
    double hard_stop_forward_x_ = -0.05;
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
        w_v_ = pnh.param("sqp_w_v", 6.0);
        w_accel_ = pnh.param("sqp_w_accel", 1.8);
        w_jerk_ = pnh.param("sqp_w_jerk", 6.0);
        w_initial_ = pnh.param("sqp_w_initial", 9.0);
        desired_follow_distance_ = pnh.param("desired_follow_distance", 2.0);
        cth_time_headway_ = pnh.param("cth_time_headway", 0.50);
        cth_kp_ = pnh.param("cth_kp", 0.22);
        cth_kd_ = pnh.param("cth_kd", 0.10);
        close_gap_slowdown_gain_ = pnh.param("cth_close_gap_slowdown_gain", 0.40);
        hard_stop_distance_ = pnh.param("hard_stop_distance", 0.65);
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
                             double& gap_error_out) const {
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
        const double cps_speed_cap = v_max_ * clampValue(1.0 - cps_global_speed_gain_ * gamma_cps, 0.35, 1.0);
        target_speed = clampValue(target_speed, v_min_, cps_speed_cap);
        target_speed_out = target_speed;

        if (hard_stop) {
            return std::vector<double>(static_cast<size_t>(n), 0.0);
        }

        for (int i = 0; i < n; ++i) {
            const double kappa = std::fabs(curvatures[static_cast<size_t>(i)]);
            const double curvature_limit =
                friction_mu_ * gravity_ / (kappa + curvature_epsilon_);
            double b_max = std::min(square(cps_speed_cap), curvature_limit * (1.0 - clampValue(gamma_cps, 0.0, 0.80)));
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
        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n, n);
        Eigen::VectorXd f = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < n; ++i) {
            h(i, i) += w_v_;
            f(i) += -w_v_ * square(target_speed);
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
        return speeds;
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
    double w_v_ = 6.0;
    double w_accel_ = 1.8;
    double w_jerk_ = 6.0;
    double w_initial_ = 9.0;
    double desired_follow_distance_ = 2.0;
    double cth_time_headway_ = 0.50;
    double cth_kp_ = 0.22;
    double cth_kd_ = 0.10;
    double close_gap_slowdown_gain_ = 0.40;
    double hard_stop_distance_ = 0.65;
    double lost_target_speed_scale_ = 0.75;
    double close_gap_min_speed_scale_ = 0.45;
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
        last_feasible_.clear();
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
        const double fallback_cost = rolloutCost(e_y, e_theta, curvatures, fallback, previous_delta, fallback_margin);
        double candidate_margin = 1.0;
        const double candidate_cost = rolloutCost(e_y, e_theta, curvatures, candidate, previous_delta, candidate_margin);

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

        info.delta = clampValue(last_feasible_.front(), -max_steer_, max_steer_);
        return info.delta;
    }

private:
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
        p_mat(0, 0) = active_adapt_scale_ * q_terminal_y_;
        p_mat(1, 1) = active_adapt_scale_ * q_terminal_theta_;
        p_mat(2, 2) = 0.10 * r_delta_rate_;
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

            const double r_rate_eff = r_delta_rate_ + 0.5 * rate_h;
            Eigen::Matrix3d q = Eigen::Matrix3d::Zero();
            q(0, 0) = active_adapt_scale_ * q_y_;
            q(1, 1) = active_adapt_scale_ * q_theta_;
            q(2, 2) = r_rate_eff;
            Eigen::Vector3d s = Eigen::Vector3d::Zero();
            s(2) = -r_rate_eff;
            const double r = r_delta_ + r_feedforward_ + r_rate_eff + 0.5 * steer_h;
            const double lin_r =
                -(r_delta_ + r_feedforward_) * delta_ff +
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
                       double& barrier_margin) const {
        double ey = clampValue(e_y, -error_guard_y_, error_guard_y_);
        double eth = clampValue(e_theta, -error_guard_theta_, error_guard_theta_);
        double last_delta = previous_delta;
        double cost = 0.0;
        barrier_margin = 1.0;
        for (size_t i = 0; i < controls.size(); ++i) {
            const double kappa = curvatures[std::min(i, curvatures.size() - 1)];
            const double delta = controls[i];
            stepState(kappa, delta, ey, eth);
            const double ff = feedforward(kappa);
            const double d_delta = delta - last_delta;
            barrier_margin = std::min(barrier_margin, barrierMargin(delta, max_steer_));
            barrier_margin = std::min(barrier_margin, barrierMargin(d_delta, max_sequence_delta_step_));
            cost += active_adapt_scale_ * (q_y_ * ey * ey + q_theta_ * eth * eth);
            cost += r_delta_ * delta * delta + r_feedforward_ * square(delta - ff);
            cost += r_delta_rate_ * d_delta * d_delta;
            cost += barrierPenalty(delta, max_steer_, barrier_weight_);
            cost += barrierPenalty(d_delta, max_sequence_delta_step_, barrier_rate_weight_);
            last_delta = delta;
        }
        cost += active_adapt_scale_ * (q_terminal_y_ * ey * ey + q_terminal_theta_ * eth * eth);
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
        return clampValue(std::atan(wheelbase_ * curvature), -0.95 * max_steer_, 0.95 * max_steer_);
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
};

class VirtualTowingNode {
public:
    VirtualTowingNode() : nh_(), pnh_("~") {
        loadParameters();
        initializeDebugLogs();
        cps_.configure(pnh_);
        lead_observer_.configure(pnh_);
        extractor_.configure(pnh_);
        velocity_planner_.configure(pnh_);
        slqr_.configure(pnh_);

        reference_sub_ = nh_.subscribe(reference_topic_, 1, &VirtualTowingNode::referenceCallback, this);
        raw_sub_ = nh_.subscribe(raw_topic_, 1, &VirtualTowingNode::rawPathCallback, this);
        tracked_sub_ = nh_.subscribe(tracked_topic_, 1, &VirtualTowingNode::trackedCallback, this);
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

    void loadParameters() {
        reference_topic_ = pnh_.param<std::string>("reference_topic", "/planner/reference_path");
        raw_topic_ = pnh_.param<std::string>("raw_topic", "/planner/raw_path");
        tracked_topic_ = pnh_.param<std::string>("tracked_topic", "/pointpillars/tracked_objects");
        car_topic_ = pnh_.param<std::string>("car_topic", "/car_message");
        carvel_topic_ = pnh_.param<std::string>("carvel_topic", "/carvel");
        imu_topic_ = pnh_.param<std::string>("imu_topic", "/imu");
        ned_odom_topic_ = pnh_.param<std::string>("ned_odom_topic", "/NED_odometry");
        system_speed_topic_ = pnh_.param<std::string>("system_speed_topic", "/system_speed");
        cmd_topic_ = pnh_.param<std::string>("cmd_topic", "/cmd_vel");

        control_rate_ = pnh_.param("control_rate", 30.0);
        wheelbase_ = pnh_.param("wheelbase", 0.60);
        v_min_ = pnh_.param("v_min", 0.10);
        max_speed_ = pnh_.param("max_speed", 0.50);
        max_accel_ = pnh_.param("max_accel", 0.18);
        max_decel_ = pnh_.param("max_decel", 0.35);
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
        reference_timeout_ = pnh_.param("reference_timeout", 0.35);
        raw_timeout_ = pnh_.param("raw_timeout", 0.35);
        allow_raw_path_fallback_ = pnh_.param("allow_raw_path_fallback", false);
        min_path_points_ = pnh_.param("min_path_points", 2);
        min_path_span_ = pnh_.param("min_path_span", 0.18);
        desired_follow_distance_ = pnh_.param("desired_follow_distance", 2.0);
        hard_stop_distance_ = pnh_.param("hard_stop_distance", 0.65);
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
        reference_max_abs_kappa0_ = pnh_.param("reference_max_abs_kappa0", 1.35);
        reference_max_fit_rms_ = pnh_.param("reference_max_fit_rms", 0.18);
        reference_max_heading_jump_ = pnh_.param("reference_max_heading_jump", 0.75);
        reference_max_kappa_jump_ = pnh_.param("reference_max_kappa_jump", 0.55);
        reference_hold_timeout_ = pnh_.param("reference_hold_timeout", 0.45);
        reference_bad_speed_scale_ = clampValue(pnh_.param("reference_bad_speed_scale", 0.30), 0.0, 1.0);
        held_reference_max_speed_ = pnh_.param("held_reference_max_speed", 0.12);
        control_recovery_ramp_seconds_ = pnh_.param("control_recovery_ramp_seconds", 0.50);
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
                            "steer_feedback_for_control,yaw_rate,yaw_damping_delta,steer_sign_mismatch,"
                            "barrier_adapt_scale,kappa_control,reference_held,path_stale_reason,"
                            "control_ramp_scale,held_reference_speed_limit,"
                            "comp_clipped,safe_stop,stop_reason\n";
        }
        if (path_log_) {
            path_log_ << "stamp,source,path_seq,index,s,x,y,yaw\n";
        }
        if (profile_log_) {
            profile_log_ << "stamp,profile_seq,index,kappa,v_ref\n";
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

    void trackedCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        lead_observer_.update(*msg, ros::Time::now());
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
        const bool ref_fresh = extractor_.freshEnough(reference_path_, now, reference_timeout_) &&
                               static_cast<int>(reference_path_.points.size()) >= min_path_points_ &&
                               pathSpan(reference_path_) >= min_path_span_;
        if (ref_fresh) {
            debug.using_raw_fallback = false;
            return &reference_path_;
        }
        const bool raw_fresh = allow_raw_path_fallback_ &&
                               extractor_.freshEnough(raw_path_, now, raw_timeout_) &&
                               static_cast<int>(raw_path_.points.size()) >= min_path_points_ &&
                               pathSpan(raw_path_) >= min_path_span_;
        if (raw_fresh) {
            debug.using_raw_fallback = true;
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
        const double dt = controlDt(now);
        const double target_speed = publish_zero_on_stop_ ? 0.0 : 0.0;
        filtered_cmd_.speed = approachWithLimit(filtered_cmd_.speed, target_speed, max_decel_ * dt);
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

    bool acceptReference(const PathTrackingProblem& candidate,
                         const ros::Time& now,
                         bool path_is_held,
                         PathTrackingProblem& selected,
                         ControllerDebug& debug) {
        selected = candidate;
        debug.reference_quality = 1.0;
        debug.hold_reason = "none";
        debug.heading_jump = 0.0;
        debug.kappa_jump = 0.0;

        if (!reference_quality_gate_enabled_) {
            if (path_is_held) {
                debug.reference_held = true;
                debug.hold_reason = "reconstructor_held";
                debug.reference_quality = 0.50;
                last_reference_hold_time_ = now;
                was_reference_held_ = true;
            } else {
                rememberGoodReference(candidate, now);
                was_reference_held_ = false;
            }
            return true;
        }

        if (path_is_held) {
            debug.reference_held = true;
            debug.hold_reason = "reconstructor_held";
            debug.reference_quality = 0.50;
            last_reference_hold_time_ = now;
            was_reference_held_ = true;
            return true;
        }

        std::string reject_reason;
        double quality = 1.0;
        if (!candidate.fit_valid || candidate.fit_rms > reference_max_fit_rms_) {
            reject_reason = "fit_rms";
            quality = std::min(quality, reference_max_fit_rms_ / std::max(1e-6, candidate.fit_rms));
        }
        if (std::fabs(candidate.e_theta) > reference_max_abs_e_theta_) {
            if (reject_reason.empty()) reject_reason = "e_theta";
            quality = std::min(quality, reference_max_abs_e_theta_ / std::max(1e-6, std::fabs(candidate.e_theta)));
        }
        if (std::fabs(candidate.first_curvature) > reference_max_abs_kappa0_) {
            if (reject_reason.empty()) reject_reason = "kappa0";
            quality = std::min(quality, reference_max_abs_kappa0_ / std::max(1e-6, std::fabs(candidate.first_curvature)));
        }
        if (has_last_good_problem_) {
            debug.heading_jump = std::fabs(wrapAngle(candidate.e_theta - last_good_e_theta_));
            debug.kappa_jump = std::fabs(candidate.first_curvature - last_good_kappa0_);
            if (debug.heading_jump > reference_max_heading_jump_) {
                if (reject_reason.empty()) reject_reason = "heading_jump";
                quality = std::min(quality, reference_max_heading_jump_ / std::max(1e-6, debug.heading_jump));
            }
            if (debug.kappa_jump > reference_max_kappa_jump_) {
                if (reject_reason.empty()) reject_reason = "kappa_jump";
                quality = std::min(quality, reference_max_kappa_jump_ / std::max(1e-6, debug.kappa_jump));
            }
        }

        debug.reference_quality = clampValue(quality, 0.0, 1.0);
        if (reject_reason.empty()) {
            rememberGoodReference(candidate, now);
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
            last_reference_hold_time_ = now;
            was_reference_held_ = true;
            debug.reference_quality = std::min(debug.reference_quality, 0.50);
            return true;
        }
        debug.hold_reason = reject_reason;
        return false;
    }

    void rememberGoodReference(const PathTrackingProblem& problem, const ros::Time& now) {
        last_good_problem_ = problem;
        last_good_problem_stamp_ = now;
        last_good_e_theta_ = problem.e_theta;
        last_good_kappa0_ = problem.first_curvature;
        has_last_good_problem_ = true;
    }

    std::vector<double> controlCurvatures(const std::vector<double>& input, ControllerDebug& debug) {
        if (input.empty()) {
            return input;
        }
        std::vector<double> out = input;
        const int preview_n = std::max(1, std::min(kappa_preview_count_, static_cast<int>(input.size())));
        double preview_mean = 0.0;
        for (int i = 0; i < preview_n; ++i) {
            preview_mean += input[static_cast<size_t>(i)];
        }
        preview_mean /= static_cast<double>(preview_n);
        for (double& kappa : out) {
            kappa = (1.0 - kappa_preview_mix_) * kappa + kappa_preview_mix_ * preview_mean;
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
        debug.kappa_control = out.front();
        return out;
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

    ControlCmd computeCommand(const ros::Time& now, ControllerDebug& debug) {
        const LeadTargetState lead = lead_observer_.state(now);
        debug.target_valid = lead.valid;
        debug.gap = lead.valid ? lead.distance : -1.0;
        debug.emergency_stop = lead_observer_.emergencyStop();

        if (vehicle_.chassis_error != 0) {
            return safeStop(now, "chassis_error", debug);
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

        const double gamma = cps_.gamma(now);
        const bool hard_stop = false;
        double target_speed = 0.0;
        double lead_speed = 0.0;
        double gap_error = 0.0;
        const std::vector<double> speed_profile =
            velocity_planner_.plan(problem.curvatures,
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
        double delayed_e_y = problem.e_y;
        double delayed_e_theta = problem.e_theta;
        const int delay_steps = predictDelayCompensatedState(
            steering_curvatures,
            vehicle_.speed_valid ? vehicle_.speed : filtered_cmd_.speed,
            delayed_e_y,
            delayed_e_theta);
        SpatialSLQR::SolveInfo slqr_info;
        const double delta_base = slqr_.solve(delayed_e_y, delayed_e_theta, steering_curvatures, previous_delta, slqr_info);
        updateBarrierAdaptation(slqr_info.barrier_margin);
        const double residual = cps_.signedSteeringResidual(now);
        const double dt = controlDt(now);
        double cps_comp = 0.0;
        const bool pause_cps_comp = std::fabs(residual) > residual_comp_pause_gate_;
        if (!pause_cps_comp && std::fabs(residual) > comp_deadband_) {
            cps_comp = cps_compensation_gain_ * residual;
        }
        const double cps_comp_limited = clampValue(cps_comp, -max_cps_comp_, max_cps_comp_);
        const double delta_rate_ff =
            delay_derivative_gain_ * (delta_base - last_slqr_delta_) / std::max(0.01, dt);
        const double delay_comp_limited = clampValue(delta_rate_ff, -max_delay_comp_, max_delay_comp_);
        const double raw_delta_comp = cps_comp_limited + delay_comp_limited;
        filtered_delta_comp_ =
            (1.0 - delta_comp_lpf_alpha_) * filtered_delta_comp_ + delta_comp_lpf_alpha_ * raw_delta_comp;
        const double comp_bound = std::max(0.0, max_cps_comp_) + std::max(0.0, max_delay_comp_);
        const double delta_comp = clampValue(filtered_delta_comp_, -comp_bound, comp_bound);
        filtered_delta_comp_ = delta_comp;
        const double speed_for_damping =
            std::max(v_min_, std::fabs(vehicle_.speed_valid ? vehicle_.speed : filtered_cmd_.speed));
        const double yaw_damping_delta =
            clampValue(-yaw_damping_gain_ * vehicle_.yaw_rate / speed_for_damping,
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
        double delta_target = clampValue(delta_base + delta_comp + yaw_damping_delta,
                                        -max_steer_,
                                        max_steer_);
        delta_target = previous_delta + ramp_scale * (delta_target - previous_delta);

        double planned_speed = clampValue(speed_profile.front(), 0.0, max_speed_);
        if (debug.hold_reason != "none") {
            debug.held_reference_speed_limit =
                std::min(std::max(0.0, held_reference_max_speed_),
                         std::max(0.0, reference_bad_speed_scale_ * max_speed_));
            planned_speed = std::min(planned_speed, debug.held_reference_speed_limit);
        } else {
            debug.held_reference_speed_limit = 0.0;
            planned_speed *= ramp_scale;
        }
        if (std::fabs(vehicle_.yaw_rate) > yaw_rate_slow_gate_) {
            planned_speed *= yaw_rate_speed_scale_;
        }
        if (slqr_.activeAdaptScale() < 0.999) {
            planned_speed *= slqr_.adaptSpeedScale();
        }
        const double speed_limited =
            planned_speed >= filtered_cmd_.speed
                ? approachWithLimit(filtered_cmd_.speed, planned_speed, max_accel_ * dt)
                : approachWithLimit(filtered_cmd_.speed, planned_speed, max_decel_ * dt);
        const double steer_limited = approachWithLimit(filtered_cmd_.steering, delta_target, max_steer_rate_ * dt);

        filtered_cmd_.speed += speed_cmd_alpha_ * (speed_limited - filtered_cmd_.speed);
        filtered_cmd_.steering += steer_cmd_alpha_ * (steer_limited - filtered_cmd_.steering);
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
        debug.delay_steps = delay_steps;
        debug.delta_slqr = delta_base;
        debug.delta_comp = delta_comp;
        debug.delta_cmd = filtered_cmd_.steering;
        debug.steer_feedback_for_control = vehicle_.steering_feedback;
        debug.yaw_rate = vehicle_.yaw_rate;
        debug.yaw_damping_delta = yaw_damping_delta;
        debug.barrier_adapt_scale = slqr_.activeAdaptScale();
        debug.comp_clipped =
            std::fabs(cps_comp - cps_comp_limited) > 1e-6 ||
            std::fabs(delta_rate_ff - delay_comp_limited) > 1e-6;
        debug.steer_sign_mismatch = updateSteerSignDiagnostic(filtered_cmd_.steering, now);
        debug.stop_reason = slqr_info.used_fallback ? "tracking_slqr_fallback" : "tracking";
        if (debug.hold_reason != "none") {
            debug.stop_reason = "path_quality_hold_" + debug.hold_reason;
        }
        writeProfile(now, steering_curvatures, speed_profile);
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
                      const std::vector<double>& speed_profile) {
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
                         << v_ref << "\n";
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
                     << debug.yaw_damping_delta << ","
                     << (debug.steer_sign_mismatch ? 1 : 0) << ","
                     << debug.barrier_adapt_scale << ","
                     << debug.kappa_control << ","
                     << (debug.reference_held ? 1 : 0) << ","
                     << debug.path_stale_reason << ","
                     << debug.control_ramp_scale << ","
                     << debug.held_reference_speed_limit << ","
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
            "delay_steps=%d delta_slqr=%.3f comp=%.3f cclip=%d cmd_delta=%.3f kctl=%.3f stop=%d reason=%s",
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
            debug.safe_stop ? 1 : 0,
            debug.stop_reason.c_str());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber reference_sub_;
    ros::Subscriber raw_sub_;
    ros::Subscriber tracked_sub_;
    ros::Subscriber car_sub_;
    ros::Subscriber carvel_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ned_odom_sub_;
    ros::Subscriber system_speed_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer timer_;

    CPSStateObserver cps_;
    LeadTargetObserver lead_observer_;
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
    std::string tracked_topic_;
    std::string car_topic_;
    std::string carvel_topic_;
    std::string imu_topic_;
    std::string ned_odom_topic_;
    std::string system_speed_topic_;
    std::string cmd_topic_;

    double control_rate_ = 30.0;
    double wheelbase_ = 0.60;
    double v_min_ = 0.10;
    double max_speed_ = 0.50;
    double max_accel_ = 0.18;
    double max_decel_ = 0.35;
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
    double reference_max_abs_kappa0_ = 1.35;
    double reference_max_fit_rms_ = 0.18;
    double reference_max_heading_jump_ = 0.75;
    double reference_max_kappa_jump_ = 0.55;
    double reference_hold_timeout_ = 0.45;
    double reference_bad_speed_scale_ = 0.30;
    double held_reference_max_speed_ = 0.12;
    double control_recovery_ramp_seconds_ = 0.50;
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
    int low_barrier_count_ = 0;
    double steer_sign_diag_window_ = 1.0;
    double steer_sign_diag_min_corr_ = 0.50;
    std::deque<TimedControlSample> sign_diag_samples_;
    PathTrackingProblem last_good_problem_;
    ros::Time last_good_problem_stamp_;
    bool has_last_good_problem_ = false;
    double last_good_e_theta_ = 0.0;
    double last_good_kappa0_ = 0.0;
    ros::Time last_reference_hold_time_;
    bool was_reference_held_ = false;
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
    double steer_cmd_alpha_ = 0.28;
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
