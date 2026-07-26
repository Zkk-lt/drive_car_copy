#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <move_car/car_parameter.h>
#include <nav_msgs/Path.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace {

double clampValue(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

double positivePart(double value) {
    return std::max(0.0, value);
}

double huberWeight(double residual, double delta) {
    const double abs_r = std::fabs(residual);
    if (abs_r <= delta || delta <= 1e-6) {
        return 1.0;
    }
    return delta / abs_r;
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

using Vec4 = std::array<double, 4>;
using Vec2 = std::array<double, 2>;
using Mat4 = std::array<std::array<double, 4>, 4>;
using Mat42 = std::array<std::array<double, 2>, 4>;
using Mat24 = std::array<std::array<double, 4>, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

Mat4 zeroMat4() {
    return {{{0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0}}};
}

Mat4 identityMat4() {
    Mat4 m = zeroMat4();
    for (int i = 0; i < 4; ++i) {
        m[i][i] = 1.0;
    }
    return m;
}

Mat2 zeroMat2() {
    return {{{0.0, 0.0}, {0.0, 0.0}}};
}

Mat2 identityMat2() {
    Mat2 m = zeroMat2();
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    return m;
}

Mat4 transpose4(const Mat4& a) {
    Mat4 out = zeroMat4();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = a[j][i];
        }
    }
    return out;
}

Mat24 transpose42(const Mat42& a) {
    Mat24 out{};
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = a[j][i];
        }
    }
    return out;
}

Mat4 addMat4(const Mat4& a, const Mat4& b) {
    Mat4 out = zeroMat4();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = a[i][j] + b[i][j];
        }
    }
    return out;
}

Mat2 addMat2(const Mat2& a, const Mat2& b) {
    Mat2 out = zeroMat2();
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            out[i][j] = a[i][j] + b[i][j];
        }
    }
    return out;
}

Mat4 mul44(const Mat4& a, const Mat4& b) {
    Mat4 out = zeroMat4();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return out;
}

Mat42 mul42(const Mat4& a, const Mat42& b) {
    Mat42 out{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 2; ++j) {
            out[i][j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return out;
}

Mat24 mul24(const Mat24& a, const Mat4& b) {
    Mat24 out{};
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return out;
}

Mat2 mul22(const Mat24& a, const Mat42& b) {
    Mat2 out = zeroMat2();
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 4; ++k) {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return out;
}

Mat4 mul424(const Mat42& a, const Mat24& b) {
    Mat4 out = zeroMat4();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 2; ++k) {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return out;
}

Mat42 mul422(const Mat42& a, const Mat2& b) {
    Mat42 out{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 2; ++j) {
            out[i][j] = 0.0;
            for (int k = 0; k < 2; ++k) {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return out;
}

Mat24 mul224(const Mat2& a, const Mat24& b) {
    Mat24 out{};
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = 0.0;
            for (int k = 0; k < 2; ++k) {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return out;
}

Mat4 subMat4(const Mat4& a, const Mat4& b) {
    Mat4 out = zeroMat4();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = a[i][j] - b[i][j];
        }
    }
    return out;
}

Mat2 invert2(const Mat2& m) {
    const double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    Mat2 out = identityMat2();
    const double safe_det = std::fabs(det) < 1e-8 ? (det >= 0.0 ? 1e-8 : -1e-8) : det;
    out[0][0] =  m[1][1] / safe_det;
    out[0][1] = -m[0][1] / safe_det;
    out[1][0] = -m[1][0] / safe_det;
    out[1][1] =  m[0][0] / safe_det;
    return out;
}

bool solve3x3(double a[3][4], double out[3]) {
    for (int i = 0; i < 3; ++i) {
        int pivot = i;
        for (int r = i + 1; r < 3; ++r) {
            if (std::fabs(a[r][i]) > std::fabs(a[pivot][i])) {
                pivot = r;
            }
        }
        if (std::fabs(a[pivot][i]) < 1e-8) {
            return false;
        }
        if (pivot != i) {
            for (int c = i; c < 4; ++c) {
                std::swap(a[i][c], a[pivot][c]);
            }
        }
        const double div = a[i][i];
        for (int c = i; c < 4; ++c) {
            a[i][c] /= div;
        }
        for (int r = 0; r < 3; ++r) {
            if (r == i) continue;
            const double factor = a[r][i];
            for (int c = i; c < 4; ++c) {
                a[r][c] -= factor * a[i][c];
            }
        }
    }
    for (int i = 0; i < 3; ++i) {
        out[i] = a[i][3];
    }
    return true;
}

struct LocalTargetSample {
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct CandidateTarget {
    LocalTargetSample local;
    double wx = 0.0;
    double wy = 0.0;
    double wyaw = 0.0;
    double distance = 0.0;
};

struct WorldTargetSample {
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct PathPoint {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double curvature = 0.0;
};

struct TargetState {
    bool valid = false;
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double rel_vx = 0.0;
    double rel_vy = 0.0;
    double rel_yaw_rate = 0.0;
    double distance = std::numeric_limits<double>::infinity();
};

struct PlannerState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
};

struct ControlInput {
    double a = 0.0;
    double delta = 0.0;
};

struct ReferencePoint {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double target_x = 0.0;
    double target_y = 0.0;
    double curvature = 0.0;
};

struct RolloutSummary {
    double cost = 0.0;
    double max_speed_violation = 0.0;
    double max_steer_violation = 0.0;
    double max_clearance_violation = 0.0;
    std::vector<PlannerState> states;
};

struct PlannerDebugInfo {
    std::string fit_mode = "constv";
    bool used_polyfit = false;
    bool target_locked = false;
    int fit_samples = 0;
    int inlier_samples = 0;
    double cross_track_error = 0.0;
    double heading_error = 0.0;
    double first_curvature = 0.0;
    double clearance_margin = 0.0;
    double fit_rms = 0.0;
    double fit_confidence = 0.0;
    double history_span = 0.0;
    double speed_target = 0.0;
    double line_search_alpha = 0.0;
    double rollout_cost = 0.0;
    double lambda_speed = 0.0;
    double lambda_steer = 0.0;
    double lambda_clearance = 0.0;
    double mu = 0.0;
    double max_violation = 0.0;
    bool speed_clipped = false;
    bool steer_clipped = false;
    bool stale_hold = false;
};

class FollowerController {
public:
    FollowerController()
        : nh_(), pnh_("~") {
        tracked_topic_ = pnh_.param<std::string>("tracked_topic", "/pointpillars/tracked_objects");
        car_topic_ = pnh_.param<std::string>("car_topic", "/car_message");
        car_vel_topic_ = pnh_.param<std::string>("car_vel_topic", "/carvel");
        cmd_topic_ = pnh_.param<std::string>("cmd_topic", "/cmd_vel");
        reference_topic_ = pnh_.param<std::string>("reference_topic", "/planner/reference_path");

        control_rate_ = pnh_.param("control_rate", 20.0);
        chassis_speed_scale_ = pnh_.param("chassis_speed_scale", 0.001);
        wheelbase_ = pnh_.param("wheelbase", 2.6);
        steer_sign_ = pnh_.param("steer_sign", -1.0);
        carvel_angular_is_steer_ = pnh_.param("carvel_angular_is_steer", false);
        carvel_yaw_rate_sign_ = pnh_.param("carvel_yaw_rate_sign", 1.0);
        compensation_gain_ = pnh_.param("compensation_gain", 0.94);
        yaw_rate_filter_alpha_ = pnh_.param("yaw_rate_filter_alpha", 0.82);
        max_compensation_yaw_rate_ = pnh_.param("max_compensation_yaw_rate", 0.90);
        min_compensation_speed_ = pnh_.param("min_compensation_speed", 0.02);
        max_speed_ = pnh_.param("max_speed", 0.10);
        min_speed_ = pnh_.param("min_speed", 0.0);
        max_steer_ = pnh_.param("max_steer", 0.38);
        max_accel_ = pnh_.param("max_accel", 0.10);
        min_accel_ = pnh_.param("min_accel", -0.22);
        max_steer_rate_ = pnh_.param("max_steer_rate", 0.06);
        emergency_stop_distance_ = pnh_.param("emergency_stop_distance", 0.50);
        desired_follow_distance_ = pnh_.param("desired_follow_distance", 1.00);
        stop_buffer_distance_ = pnh_.param("stop_buffer_distance", 0.50);
        target_timeout_ = pnh_.param("target_timeout", 0.60);
        target_hold_timeout_ = pnh_.param("target_hold_timeout", 1.00);
        reference_timeout_ = pnh_.param("reference_timeout", 0.25);
        reference_hold_timeout_ = pnh_.param("reference_hold_timeout", 0.80);
        reference_min_points_ = pnh_.param("reference_min_points", 5);
        reference_sample_spacing_ = pnh_.param("reference_sample_spacing", 0.12);
        reference_anchor_forward_offset_ = pnh_.param("reference_anchor_forward_offset", 0.10);
        reference_anchor_lateral_weight_ = pnh_.param("reference_anchor_lateral_weight", 1.8);
        reference_quality_span_ = pnh_.param("reference_quality_span", 0.80);
        reference_quality_points_ = pnh_.param("reference_quality_points", 18);
        path_curvature_speed_gain_ = pnh_.param("path_curvature_speed_gain", 1.6);
        path_path_hold_speed_scale_ = pnh_.param("path_hold_speed_scale", 0.85);
        path_confidence_floor_ = pnh_.param("path_confidence_floor", 0.35);
        lateral_gate_ = pnh_.param("lateral_gate", 1.00);
        min_forward_target_x_ = pnh_.param("min_forward_target_x", 0.12);
        pose_alpha_ = pnh_.param("pose_alpha", 0.28);
        yaw_alpha_ = pnh_.param("yaw_alpha", 0.18);
        velocity_alpha_ = pnh_.param("velocity_alpha", 0.14);
        speed_cmd_alpha_ = pnh_.param("speed_cmd_alpha", 0.14);
        steer_cmd_alpha_ = pnh_.param("steer_cmd_alpha", 0.16);

        horizon_steps_ = pnh_.param("horizon_steps", 20);
        horizon_dt_ = pnh_.param("horizon_dt", 0.10);
        reference_history_window_ = pnh_.param("reference_history_window", 1.50);
        fit_min_samples_ = pnh_.param("fit_min_samples", 4);
        fit_max_rms_ = pnh_.param("fit_max_rms", 0.35);
        fit_blend_gain_ = pnh_.param("fit_blend_gain", 2.5);
        outlier_reject_distance_ = pnh_.param("outlier_reject_distance", 0.65);
        outlier_reject_speed_ = pnh_.param("outlier_reject_speed", 1.20);
        huber_delta_ = pnh_.param("huber_delta", 0.18);
        fit_ridge_lambda_ = pnh_.param("fit_ridge_lambda", 0.45);
        low_speed_heading_threshold_ = pnh_.param("low_speed_heading_threshold", 0.12);
        low_speed_span_threshold_ = pnh_.param("low_speed_span_threshold", 0.45);
        heading_memory_alpha_ = pnh_.param("heading_memory_alpha", 0.85);
        curvature_memory_alpha_ = pnh_.param("curvature_memory_alpha", 0.82);
        min_progress_speed_ = pnh_.param("min_progress_speed", 0.12);
        preview_speed_steps_ = pnh_.param("preview_speed_steps", 4);
        max_curvature_ = pnh_.param("max_curvature", 0.22);
        max_curvature_rate_ = pnh_.param("max_curvature_rate", 0.035);
        max_heading_step_ = pnh_.param("max_heading_step", 0.10);
        gap_speed_gain_ = pnh_.param("gap_speed_gain", 0.22);
        fit_bad_speed_scale_ = pnh_.param("fit_bad_speed_scale", 0.55);
        line_search_decay_ = pnh_.param("line_search_decay", 0.5);
        temporal_heading_weight_ = pnh_.param("temporal_heading_weight", 2.4);
        temporal_speed_weight_ = pnh_.param("temporal_speed_weight", 1.2);
        lock_distance_gate_ = pnh_.param("lock_distance_gate", 0.90);
        lock_yaw_gate_ = pnh_.param("lock_yaw_gate", 0.90);
        lock_hold_timeout_ = pnh_.param("lock_hold_timeout", 1.20);
        al_outer_iterations_ = pnh_.param("al_outer_iterations", 3);
        al_violation_tolerance_ = pnh_.param("al_violation_tolerance", 0.02);
        mu_penalty_ = pnh_.param("mu_penalty", 10.0);
        mu_growth_ = pnh_.param("mu_growth", 1.35);
        mu_penalty_max_ = pnh_.param("mu_penalty_max", 120.0);
        lambda_max_ = pnh_.param("lambda_max", 400.0);
        lambda_decay_ = pnh_.param("lambda_decay", 0.80);
        path_curvature_change_speed_gain_ = pnh_.param("path_curvature_change_speed_gain", 2.4);
        path_heading_speed_gain_ = pnh_.param("path_heading_speed_gain", 0.75);

        q_x_ = pnh_.param("q_x", 2.5);
        q_y_ = pnh_.param("q_y", 14.0);
        q_yaw_ = pnh_.param("q_yaw", 4.0);
        q_v_ = pnh_.param("q_v", 2.0);
        r_a_ = pnh_.param("r_a", 0.35);
        r_delta_ = pnh_.param("r_delta", 0.55);
        r_da_ = pnh_.param("r_da", 1.40);
        r_ddelta_ = pnh_.param("r_ddelta", 1.80);
        clearance_penalty_ = pnh_.param("clearance_penalty", 220.0);
        speed_penalty_ = pnh_.param("speed_penalty", 160.0);
        steer_penalty_ = pnh_.param("steer_penalty", 60.0);
        terminal_x_weight_ = pnh_.param("terminal_x_weight", 4.0);
        terminal_y_weight_ = pnh_.param("terminal_y_weight", 18.0);
        terminal_yaw_weight_ = pnh_.param("terminal_yaw_weight", 6.0);
        ilqr_w_curv_ = pnh_.param("ilqr_w_curv", 14.0);
        lag_preview_time_ = pnh_.param("lag_preview_time", 0.10);

        tracked_sub_ = nh_.subscribe(tracked_topic_, 1, &FollowerController::trackedCallback, this);
        car_sub_ = nh_.subscribe(car_topic_, 10, &FollowerController::carCallback, this);
        car_vel_sub_ = nh_.subscribe(car_vel_topic_, 10, &FollowerController::carVelCallback, this);
        reference_sub_ = nh_.subscribe(reference_topic_, 1, &FollowerController::referencePathCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 10);

        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, control_rate_)), &FollowerController::controlLoop, this);

        nominal_controls_.assign(static_cast<size_t>(std::max(2, horizon_steps_)), ControlInput());
        ego_pose_initialized_ = false;
        ROS_INFO(
            "Follower controller started | tracked_topic=%s cmd_topic=%s control_rate=%.1fHz max_speed=%.2f planner=reference_path+al_ilqr follow=%.2f ref_min=%d lambda_max=%.1f mu_max=%.1f carvel_is_steer=%d",
            tracked_topic_.c_str(),
            cmd_topic_.c_str(),
            control_rate_,
            max_speed_,
            desired_follow_distance_,
            reference_min_points_,
            lambda_max_,
            mu_penalty_max_,
            carvel_angular_is_steer_ ? 1 : 0);
    }

private:
    void updateEgoPose(const ros::Time& now) {
        if (!ego_pose_initialized_) {
            last_ego_pose_time_ = now;
            ego_pose_initialized_ = true;
            return;
        }
        double dt = clampValue((now - last_ego_pose_time_).toSec(), 0.0, 0.10);
        last_ego_pose_time_ = now;
        if (dt <= 1e-4) {
            return;
        }
        const double speed =
            std::fabs(ego_speed_mps_) < min_compensation_speed_ ? 0.0 : compensation_gain_ * ego_speed_mps_;
        const double steer = clampValue(steering_feedback_, -0.75, 0.75);
        const double model_yaw_rate = speed * std::tan(steer) / std::max(0.5, wheelbase_);
        const double fused_yaw_rate = carvel_angular_is_steer_
                                          ? model_yaw_rate
                                          : (0.35 * model_yaw_rate + 0.65 * carvel_yaw_rate_);
        const double yaw_rate = clampValue(
            fused_yaw_rate,
            -max_compensation_yaw_rate_,
            max_compensation_yaw_rate_);
        const double heading_mid = ego_world_yaw_ + 0.5 * yaw_rate * dt;
        ego_world_x_ += speed * std::cos(heading_mid) * dt;
        ego_world_y_ += speed * std::sin(heading_mid) * dt;
        ego_world_yaw_ = wrapAngle(ego_world_yaw_ + yaw_rate * dt);
    }

    void localToWorld(double lx, double ly, double lyaw, double& wx, double& wy, double& wyaw) const {
        const double c = std::cos(ego_world_yaw_);
        const double s = std::sin(ego_world_yaw_);
        wx = ego_world_x_ + c * lx - s * ly;
        wy = ego_world_y_ + s * lx + c * ly;
        wyaw = wrapAngle(ego_world_yaw_ + lyaw);
    }

    void worldToLocal(double wx, double wy, double wyaw, double& lx, double& ly, double& lyaw) const {
        const double dx = wx - ego_world_x_;
        const double dy = wy - ego_world_y_;
        const double c = std::cos(ego_world_yaw_);
        const double s = std::sin(ego_world_yaw_);
        lx =  c * dx + s * dy;
        ly = -s * dx + c * dy;
        lyaw = wrapAngle(wyaw - ego_world_yaw_);
    }

    void trackedCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        updateEgoPose(now);

        double min_distance = std::numeric_limits<double>::infinity();
        bool found_emergency = false;
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const geometry_msgs::Pose& pose = msg->poses[i];
            const double x = pose.position.x;
            const double y = pose.position.y;
            const double distance = std::hypot(x, y);
            if (x > -0.05 && std::fabs(y) < 1.0) {
                min_distance = std::min(min_distance, distance);
            }
            if (x > -0.05 && distance <= emergency_stop_distance_) {
                found_emergency = true;
            }
        }
        nearest_obstacle_distance_ = min_distance;
        emergency_stop_active_ = found_emergency;

        LocalTargetSample sample;
        if (selectBestTarget(*msg, now, sample)) {
            updateTargetState(sample, now);
        } else {
            updatePredictedLockedTarget(now);
        }
    }

    void carCallback(const move_car::car_parameter::ConstPtr& msg) {
        ego_speed_mps_ = clampValue(static_cast<double>(msg->back_wheel_speed) * chassis_speed_scale_, -0.5, 0.5);
        steering_feedback_ = steer_sign_ * static_cast<double>(msg->turn_angle);
        chassis_error_flag_ = static_cast<int>(msg->error_flag);
    }

    void carVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        const double speed_hint = clampValue(static_cast<double>(msg->linear.x) * chassis_speed_scale_, -0.5, 0.5);
        ego_speed_mps_ = 0.75 * ego_speed_mps_ + 0.25 * speed_hint;
        if (carvel_angular_is_steer_) {
            steering_feedback_ = steer_sign_ * static_cast<double>(msg->angular.z);
        } else {
            const double yaw_rate_measured = carvel_yaw_rate_sign_ * static_cast<double>(msg->angular.z);
            carvel_yaw_rate_ =
                yaw_rate_filter_alpha_ * carvel_yaw_rate_ +
                (1.0 - yaw_rate_filter_alpha_) * yaw_rate_measured;
        }
    }

    bool updatePredictedLockedTarget(const ros::Time& now) {
        if (!target_lock_active_) {
            return false;
        }
        const double age = std::max(0.0, (now - target_lock_stamp_).toSec());
        if (age > lock_hold_timeout_) {
            return false;
        }
        const double pred_x_w = target_lock_world_x_ + target_lock_world_vx_ * age;
        const double pred_y_w = target_lock_world_y_ + target_lock_world_vy_ * age;
        const double pred_yaw_w = wrapAngle(target_lock_world_yaw_ + target_lock_world_yaw_rate_ * age);
        double lx = 0.0, ly = 0.0, lyaw = 0.0;
        worldToLocal(pred_x_w, pred_y_w, pred_yaw_w, lx, ly, lyaw);
        filtered_target_.valid = true;
        filtered_target_.stamp = now;
        filtered_target_.x = lx;
        filtered_target_.y = ly;
        filtered_target_.yaw = wrapAngle(0.85 * filtered_target_.yaw + 0.15 * lyaw);
        filtered_target_.distance = std::hypot(lx, ly);
        filtered_target_.rel_vx = 0.90 * filtered_target_.rel_vx + 0.10 * target_lock_world_vx_;
        filtered_target_.rel_vy = 0.90 * filtered_target_.rel_vy + 0.10 * target_lock_world_vy_;
        filtered_target_.rel_yaw_rate = 0.90 * filtered_target_.rel_yaw_rate + 0.10 * target_lock_world_yaw_rate_;
        planner_debug_.target_locked = true;
        return true;
    }

    bool selectBestTarget(const geometry_msgs::PoseArray& msg, const ros::Time& now, LocalTargetSample& sample) {
        std::vector<CandidateTarget> candidates;
        candidates.reserve(msg.poses.size());
        for (size_t i = 0; i < msg.poses.size(); ++i) {
            const geometry_msgs::Pose& pose = msg.poses[i];
            const double x = pose.position.x;
            const double y = pose.position.y;
            if (x < min_forward_target_x_) {
                continue;
            }
            if (std::fabs(y) > lateral_gate_) {
                continue;
            }
            CandidateTarget cand;
            cand.local.stamp = now;
            cand.local.x = x;
            cand.local.y = y;
            cand.local.yaw = yawFromQuaternion(pose.orientation);
            cand.distance = std::hypot(x, y);
            localToWorld(cand.local.x, cand.local.y, cand.local.yaw, cand.wx, cand.wy, cand.wyaw);
            candidates.push_back(cand);
        }
        if (candidates.empty()) {
            planner_debug_.target_locked = target_lock_active_;
            return false;
        }

        int chosen = -1;
        if (target_lock_active_) {
            const double dt = std::max(0.0, (now - target_lock_stamp_).toSec());
            const double pred_x = target_lock_world_x_ + target_lock_world_vx_ * dt;
            const double pred_y = target_lock_world_y_ + target_lock_world_vy_ * dt;
            const double pred_yaw = wrapAngle(target_lock_world_yaw_ + target_lock_world_yaw_rate_ * dt);
            double best_lock_cost = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < candidates.size(); ++i) {
                const double pos_err = std::hypot(candidates[i].wx - pred_x, candidates[i].wy - pred_y);
                const double yaw_err = std::fabs(wrapAngle(candidates[i].wyaw - pred_yaw));
                if (pos_err > lock_distance_gate_ || yaw_err > lock_yaw_gate_) {
                    continue;
                }
                const double cost = pos_err + 0.35 * yaw_err + 0.12 * std::fabs(candidates[i].local.y - filtered_target_.y);
                if (cost < best_lock_cost) {
                    best_lock_cost = cost;
                    chosen = static_cast<int>(i);
                }
            }
            if (chosen < 0 && (now - target_lock_stamp_).toSec() <= lock_hold_timeout_) {
                planner_debug_.target_locked = true;
                return false;
            }
        }

        if (chosen < 0) {
            double best_cost = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < candidates.size(); ++i) {
                const double cost = candidates[i].local.x + 0.85 * std::fabs(candidates[i].local.y);
                if (cost < best_cost) {
                    best_cost = cost;
                    chosen = static_cast<int>(i);
                }
            }
        }
        if (chosen < 0) {
            planner_debug_.target_locked = target_lock_active_;
            return false;
        }

        sample = candidates[static_cast<size_t>(chosen)].local;
        planner_debug_.target_locked = target_lock_active_;
        return true;
    }

    void updateTargetState(const LocalTargetSample& sample, const ros::Time& now) {
        if (!filtered_target_.valid) {
            filtered_target_.valid = true;
            filtered_target_.stamp = now;
            filtered_target_.x = sample.x;
            filtered_target_.y = sample.y;
            filtered_target_.yaw = sample.yaw;
            filtered_target_.distance = std::hypot(sample.x, sample.y);
            filtered_target_.rel_vx = 0.0;
            filtered_target_.rel_vy = 0.0;
            filtered_target_.rel_yaw_rate = 0.0;
        } else {
            const double dt = std::max(0.03, (now - filtered_target_.stamp).toSec());
            const double measured_vx = (sample.x - filtered_target_.x) / dt;
            const double measured_vy = (sample.y - filtered_target_.y) / dt;
            const double measured_yaw_rate = wrapAngle(sample.yaw - filtered_target_.yaw) / dt;
            filtered_target_.x += pose_alpha_ * (sample.x - filtered_target_.x);
            filtered_target_.y += pose_alpha_ * (sample.y - filtered_target_.y);
            filtered_target_.yaw = wrapAngle(filtered_target_.yaw + yaw_alpha_ * wrapAngle(sample.yaw - filtered_target_.yaw));
            filtered_target_.rel_vx += velocity_alpha_ * (measured_vx - filtered_target_.rel_vx);
            filtered_target_.rel_vy += velocity_alpha_ * (measured_vy - filtered_target_.rel_vy);
            filtered_target_.rel_yaw_rate += velocity_alpha_ * (measured_yaw_rate - filtered_target_.rel_yaw_rate);
            filtered_target_.distance = std::hypot(filtered_target_.x, filtered_target_.y);
            filtered_target_.stamp = now;
        }

        double wx = 0.0, wy = 0.0, wyaw = 0.0;
        localToWorld(filtered_target_.x, filtered_target_.y, filtered_target_.yaw, wx, wy, wyaw);
        target_lock_active_ = true;
        target_lock_stamp_ = now;
        target_lock_world_x_ = wx;
        target_lock_world_y_ = wy;
        target_lock_world_yaw_ = wyaw;
        target_world_history_.push_back(WorldTargetSample{now, wx, wy, wyaw});
        while (!target_world_history_.empty() && (now - target_world_history_.front().stamp).toSec() > reference_history_window_) {
            target_world_history_.pop_front();
        }

        if (target_world_history_.size() >= 2) {
            const WorldTargetSample& prev = target_world_history_[target_world_history_.size() - 2];
            const WorldTargetSample& curr = target_world_history_.back();
            const double dt = std::max(0.03, (curr.stamp - prev.stamp).toSec());
            const double vx = (curr.x - prev.x) / dt;
            const double vy = (curr.y - prev.y) / dt;
            target_world_vx_ = 0.80 * target_world_vx_ + 0.20 * vx;
            target_world_vy_ = 0.80 * target_world_vy_ + 0.20 * vy;
            target_lock_world_vx_ = target_world_vx_;
            target_lock_world_vy_ = target_world_vy_;
            target_lock_world_yaw_rate_ = 0.85 * target_lock_world_yaw_rate_ + 0.15 * (wrapAngle(curr.yaw - prev.yaw) / dt);
        }
        filtered_target_.valid = true;
        filtered_target_.stamp = now;
    }

    std::deque<WorldTargetSample> buildInlierHistory() {
        std::deque<WorldTargetSample> inliers;
        planner_debug_.inlier_samples = 0;
        if (target_world_history_.empty()) {
            return inliers;
        }

        inliers.push_back(target_world_history_.front());
        for (size_t i = 1; i < target_world_history_.size(); ++i) {
            const WorldTargetSample& prev = inliers.back();
            const WorldTargetSample& curr = target_world_history_[i];
            const double dt = std::max(0.03, (curr.stamp - prev.stamp).toSec());
            const double jump_dist = std::hypot(curr.x - prev.x, curr.y - prev.y);
            const double jump_speed = jump_dist / dt;
            if (jump_dist > outlier_reject_distance_ || jump_speed > outlier_reject_speed_) {
                continue;
            }
            inliers.push_back(curr);
        }
        planner_debug_.inlier_samples = static_cast<int>(inliers.size());
        return inliers;
    }

    double computeHistorySpan(const std::deque<WorldTargetSample>& history) const {
        if (history.size() < 2) {
            return 0.0;
        }
        double min_x = history.front().x;
        double max_x = history.front().x;
        double min_y = history.front().y;
        double max_y = history.front().y;
        for (const auto& sample : history) {
            min_x = std::min(min_x, sample.x);
            max_x = std::max(max_x, sample.x);
            min_y = std::min(min_y, sample.y);
            max_y = std::max(max_y, sample.y);
        }
        return std::hypot(max_x - min_x, max_y - min_y);
    }

    bool fitQuadratic(const std::deque<WorldTargetSample>& history, const ros::Time& now, double coeff_x[3], double coeff_y[3], double& rms_error) {
        if (static_cast<int>(history.size()) < fit_min_samples_) {
            return false;
        }

        std::vector<double> weights(history.size(), 1.0);
        std::vector<double> temporal_weights(history.size(), 1.0);
        if (history.size() >= 3) {
            temporal_weights.front() = 0.85;
            temporal_weights.back() = 1.0;
            for (size_t idx = 1; idx + 1 < history.size(); ++idx) {
                const auto& prev = history[idx - 1];
                const auto& curr = history[idx];
                const auto& next = history[idx + 1];
                const double dt1 = std::max(0.03, (curr.stamp - prev.stamp).toSec());
                const double dt2 = std::max(0.03, (next.stamp - curr.stamp).toSec());
                const double vx1 = (curr.x - prev.x) / dt1;
                const double vy1 = (curr.y - prev.y) / dt1;
                const double vx2 = (next.x - curr.x) / dt2;
                const double vy2 = (next.y - curr.y) / dt2;
                const double h1 = std::atan2(vy1, vx1);
                const double h2 = std::atan2(vy2, vx2);
                const double heading_jump = std::fabs(wrapAngle(h2 - h1));
                const double speed_jump = std::fabs(std::hypot(vx2, vy2) - std::hypot(vx1, vy1));
                temporal_weights[idx] = 1.0 / (1.0 + temporal_heading_weight_ * heading_jump + temporal_speed_weight_ * speed_jump);
            }
        }
        int used = 0;
        for (size_t iter = 0; iter < 3; ++iter) {
            double s00 = 0.0, s01 = 0.0, s02 = 0.0, s11 = 0.0, s12 = 0.0, s22 = 0.0;
            double bx0 = 0.0, bx1 = 0.0, bx2 = 0.0;
            double by0 = 0.0, by1 = 0.0, by2 = 0.0;
            used = 0;
            for (size_t idx = 0; idx < history.size(); ++idx) {
                const auto& sample = history[idx];
                const double t = (sample.stamp - now).toSec();
                const double age = std::fabs(t);
                const double base_w = 1.0 / (1.0 + fit_blend_gain_ * age);
                const double w = base_w * temporal_weights[idx] * weights[idx];
                const double t2 = t * t;
                s00 += w;
                s01 += w * t;
                s02 += w * t2;
                s11 += w * t * t;
                s12 += w * t2 * t;
                s22 += w * t2 * t2;
                bx0 += w * sample.x;
                bx1 += w * sample.x * t;
                bx2 += w * sample.x * t2;
                by0 += w * sample.y;
                by1 += w * sample.y * t;
                by2 += w * sample.y * t2;
                ++used;
            }

            s22 += fit_ridge_lambda_;
            s11 += 0.15 * fit_ridge_lambda_;
            double ax[3][4] = {{s00, s01, s02, bx0}, {s01, s11, s12, bx1}, {s02, s12, s22, bx2}};
            double ay[3][4] = {{s00, s01, s02, by0}, {s01, s11, s12, by1}, {s02, s12, s22, by2}};
            if (!solve3x3(ax, coeff_x) || !solve3x3(ay, coeff_y)) {
                return false;
            }

            for (size_t idx = 0; idx < history.size(); ++idx) {
                const auto& sample = history[idx];
                const double t = (sample.stamp - now).toSec();
                const double pred_x = coeff_x[0] + coeff_x[1] * t + coeff_x[2] * t * t;
                const double pred_y = coeff_y[0] + coeff_y[1] * t + coeff_y[2] * t * t;
                const double err = std::hypot(sample.x - pred_x, sample.y - pred_y);
                weights[idx] = huberWeight(err, huber_delta_);
            }
        }

        double sq_error = 0.0;
        double weight_sum = 0.0;
        for (size_t idx = 0; idx < history.size(); ++idx) {
            const auto& sample = history[idx];
            const double t = (sample.stamp - now).toSec();
            const double age = std::fabs(t);
            const double w = (1.0 / (1.0 + fit_blend_gain_ * age)) * temporal_weights[idx] * weights[idx];
            const double pred_x = coeff_x[0] + coeff_x[1] * t + coeff_x[2] * t * t;
            const double pred_y = coeff_y[0] + coeff_y[1] * t + coeff_y[2] * t * t;
            const double err = std::hypot(sample.x - pred_x, sample.y - pred_y);
            sq_error += w * err * err;
            weight_sum += w;
        }
        rms_error = weight_sum > 1e-6 ? std::sqrt(sq_error / weight_sum) : 0.0;
        planner_debug_.fit_samples = used;
        planner_debug_.used_polyfit = true;
        return true;
    }

    double estimateStablePathHeading(const std::deque<WorldTargetSample>& history) const {
        if (history.size() >= 2) {
            const WorldTargetSample& newest = history.back();
            for (size_t idx = history.size() - 1; idx > 0; --idx) {
                const auto& older = history[idx - 1];
                const double dx = newest.x - older.x;
                const double dy = newest.y - older.y;
                if (std::hypot(dx, dy) > 0.08) {
                    return std::atan2(dy, dx);
                }
            }
        }
        return path_heading_memory_;
    }

    double alignHeadingToReference(double heading, double reference_heading) const {
        double best = heading;
        double best_err = std::fabs(wrapAngle(heading - reference_heading));
        const double flipped = wrapAngle(heading + M_PI);
        const double flipped_err = std::fabs(wrapAngle(flipped - reference_heading));
        if (flipped_err < best_err) {
            best = flipped;
        }
        return best;
    }

    void referencePathCallback(const nav_msgs::Path::ConstPtr& msg) {
        reference_path_points_.clear();
        reference_path_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        has_reference_path_ = !msg->poses.empty();
        if (!has_reference_path_) {
            return;
        }

        reference_path_points_.reserve(msg->poses.size());
        double accum_s = 0.0;
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            PathPoint pt;
            pt.x = msg->poses[i].pose.position.x;
            pt.y = msg->poses[i].pose.position.y;
            pt.yaw = yawFromQuaternion(msg->poses[i].pose.orientation);
            if (i > 0) {
                accum_s += std::hypot(
                    pt.x - reference_path_points_.back().x,
                    pt.y - reference_path_points_.back().y);
            }
            pt.s = accum_s;
            reference_path_points_.push_back(pt);
        }
        smoothReferencePathGeometry();
    }

    void smoothReferencePathGeometry() {
        if (reference_path_points_.size() < 2) {
            return;
        }

        size_t init_front = 0;
        while (init_front + 1 < reference_path_points_.size() &&
               (reference_path_points_[init_front].s - reference_path_points_.front().s) < 0.30) {
            ++init_front;
        }
        const double init_dx = reference_path_points_[init_front].x - reference_path_points_.front().x;
        const double init_dy = reference_path_points_[init_front].y - reference_path_points_.front().y;
        double previous_yaw =
            (std::fabs(init_dx) + std::fabs(init_dy) > 1e-4)
                ? std::atan2(init_dy, init_dx)
                : reference_path_points_.front().yaw;
        for (size_t i = 0; i < reference_path_points_.size(); ++i) {
            const size_t back = (i > 2) ? i - 3 : 0;
            const size_t front = std::min(reference_path_points_.size() - 1, i + 4);
            const double dx = reference_path_points_[front].x - reference_path_points_[back].x;
            const double dy = reference_path_points_[front].y - reference_path_points_[back].y;
            double yaw = std::fabs(dx) + std::fabs(dy) > 1e-4 ? std::atan2(dy, dx) : previous_yaw;
            yaw = alignHeadingToReference(yaw, previous_yaw);
            previous_yaw = yaw;
            reference_path_points_[i].yaw = yaw;
        }

        for (size_t i = 1; i + 1 < reference_path_points_.size(); ++i) {
            const auto& p0 = reference_path_points_[i - 1];
            const auto& p1 = reference_path_points_[i];
            const auto& p2 = reference_path_points_[i + 1];
            const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
            const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
            const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);
            const double area = 0.5 * std::fabs(
                p0.x * (p1.y - p2.y) +
                p1.x * (p2.y - p0.y) +
                p2.x * (p0.y - p1.y));
            const double cross =
                (p1.x - p0.x) * (p2.y - p1.y) -
                (p1.y - p0.y) * (p2.x - p1.x);
            const double sign = cross >= 0.0 ? 1.0 : -1.0;
            reference_path_points_[i].curvature =
                (a * b * c > 1e-6) ? clampValue(sign * (4.0 * area) / (a * b * c), -max_curvature_, max_curvature_) : 0.0;
        }
        reference_path_points_.front().curvature = 0.0;
        reference_path_points_.back().curvature = 0.0;
    }

    bool referencePathFresh(const ros::Time& now, bool& using_hold) const {
        using_hold = false;
        if (!has_reference_path_ || reference_path_points_.size() < 2) {
            return false;
        }
        const double span = reference_path_points_.back().s - reference_path_points_.front().s;
        if (static_cast<int>(reference_path_points_.size()) < reference_min_points_) {
            if (reference_path_points_.size() < 4 || span < 0.35) {
                return false;
            }
        }
        const double age = std::max(0.0, (now - reference_path_stamp_).toSec());
        if (age <= reference_timeout_) {
            return true;
        }
        if (age <= reference_hold_timeout_) {
            using_hold = true;
            return true;
        }
        return false;
    }

    int findReferenceAnchorIndex() const {
        if (reference_path_points_.empty()) {
            return -1;
        }
        int best_index = 0;
        double best_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < reference_path_points_.size(); ++i) {
            const auto& pt = reference_path_points_[i];
            const double forward_penalty = positivePart(-pt.x) * 4.0;
            const double lateral_penalty = reference_anchor_lateral_weight_ * std::fabs(pt.y);
            const double cost = std::hypot(pt.x, pt.y) + forward_penalty + lateral_penalty;
            if (cost < best_cost) {
                best_cost = cost;
                best_index = static_cast<int>(i);
            }
        }
        return best_index;
    }

    PathPoint interpolateReferencePath(double query_s) const {
        if (reference_path_points_.empty()) {
            return PathPoint();
        }
        if (query_s <= reference_path_points_.front().s) {
            return reference_path_points_.front();
        }
        if (query_s >= reference_path_points_.back().s) {
            return reference_path_points_.back();
        }
        for (size_t i = 1; i < reference_path_points_.size(); ++i) {
            const auto& p0 = reference_path_points_[i - 1];
            const auto& p1 = reference_path_points_[i];
            if (query_s > p1.s) {
                continue;
            }
            const double ds = std::max(1e-4, p1.s - p0.s);
            const double ratio = clampValue((query_s - p0.s) / ds, 0.0, 1.0);
            PathPoint out;
            out.s = query_s;
            out.x = p0.x + ratio * (p1.x - p0.x);
            out.y = p0.y + ratio * (p1.y - p0.y);
            out.yaw = wrapAngle(p0.yaw + ratio * wrapAngle(p1.yaw - p0.yaw));
            out.curvature = p0.curvature + ratio * (p1.curvature - p0.curvature);
            return out;
        }
        return reference_path_points_.back();
    }

    double estimateLeaderSpeed() const {
        const double world_speed = std::hypot(target_world_vx_, target_world_vy_);
        const double local_speed = std::max(0.0, ego_speed_mps_ + filtered_target_.rel_vx);
        return clampValue(0.65 * world_speed + 0.35 * local_speed, 0.0, max_speed_);
    }


    bool isTargetFresh(const ros::Time& now) const {
        if (!filtered_target_.valid) {
            return false;
        }
        return (now - filtered_target_.stamp).toSec() <= target_timeout_;
    }

    PlannerState stepDynamics(const PlannerState& state, const ControlInput& control, double dt) const {
        PlannerState next = state;
        const double steer = clampValue(control.delta, -max_steer_, max_steer_);
        const double accel = clampValue(control.a, min_accel_, max_accel_);
        next.x += state.v * std::cos(state.yaw) * dt;
        next.y += state.v * std::sin(state.yaw) * dt;
        next.yaw = wrapAngle(state.yaw + state.v * std::tan(steer) * dt / std::max(0.5, wheelbase_));
        next.v = clampValue(state.v + accel * dt, min_speed_, max_speed_);
        return next;
    }

    std::vector<ReferencePoint> buildReferenceTrajectory(const ros::Time& now) {
        std::vector<ReferencePoint> reference;
        reference.reserve(static_cast<size_t>(std::max(2, horizon_steps_ + 1)));
        planner_debug_.used_polyfit = false;
        planner_debug_.fit_mode = "constv";
        planner_debug_.fit_samples = 0;
        planner_debug_.inlier_samples = 0;
        planner_debug_.fit_rms = 0.0;
        planner_debug_.fit_confidence = 0.0;
        planner_debug_.history_span = 0.0;
        if (!filtered_target_.valid) {
            return reference;
        }
        bool using_path_hold = false;
        if (!referencePathFresh(now, using_path_hold)) {
            return reference;
        }

        const int anchor_index = findReferenceAnchorIndex();
        if (anchor_index < 0) {
            return reference;
        }

        const double path_span = reference_path_points_.back().s - reference_path_points_.front().s;
        const double points_ratio = clampValue(
            static_cast<double>(reference_path_points_.size()) / std::max(1.0, static_cast<double>(reference_quality_points_)),
            0.0, 1.0);
        const double span_ratio = clampValue(path_span / std::max(0.05, reference_quality_span_), 0.0, 1.0);
        const double age = std::max(0.0, (now - reference_path_stamp_).toSec());
        const double freshness = clampValue(1.0 - age / std::max(0.05, reference_hold_timeout_), 0.0, 1.0);
        const double path_confidence = clampValue(0.35 * points_ratio + 0.45 * span_ratio + 0.20 * freshness, 0.0, 1.0);
        planner_debug_.fit_mode = using_path_hold ? "path_hold" : "path";
        planner_debug_.fit_samples = static_cast<int>(reference_path_points_.size());
        planner_debug_.inlier_samples = static_cast<int>(reference_path_points_.size());
        planner_debug_.fit_confidence = path_confidence;
        planner_debug_.history_span = path_span;

        const PathPoint& anchor = reference_path_points_[static_cast<size_t>(anchor_index)];
        const double leader_speed = estimateLeaderSpeed();
        const double gap_error = filtered_target_.distance - desired_follow_distance_;
        double desired_speed = clampValue(
            0.70 * leader_speed + gap_speed_gain_ * gap_error + 0.10 * filtered_target_.rel_vx,
            min_speed_,
            max_speed_);
        if (gap_error > 0.10) {
            desired_speed = std::max(
                desired_speed,
                clampValue(min_progress_speed_ + 0.16 * gap_error, 0.0, max_speed_));
        } else if (gap_error < -0.05) {
            desired_speed *= clampValue(1.0 + 1.4 * gap_error, 0.0, 1.0);
        }
        if (using_path_hold) {
            desired_speed *= path_path_hold_speed_scale_;
        }
        desired_speed *= clampValue(
            fit_bad_speed_scale_ + (1.0 - fit_bad_speed_scale_) * std::max(path_confidence_floor_, path_confidence),
            fit_bad_speed_scale_,
            1.0);
        if (filtered_target_.distance <= stop_buffer_distance_) {
            desired_speed = 0.0;
        }

        const double anchor_s = anchor.s + reference_anchor_forward_offset_;
        const double sample_step = std::max(reference_sample_spacing_, std::max(min_progress_speed_, desired_speed) * horizon_dt_);
        double preview_curvature = 0.0;
        double preview_curvature_max = 0.0;
        double preview_heading = 0.0;
        for (int i = 0; i < std::max(1, preview_speed_steps_); ++i) {
            const PathPoint sample = interpolateReferencePath(anchor_s + sample_step * static_cast<double>(i));
            preview_curvature += std::fabs(sample.curvature);
            preview_curvature_max = std::max(preview_curvature_max, std::fabs(sample.curvature));
            preview_heading += std::fabs(sample.yaw);
        }
        preview_curvature /= static_cast<double>(std::max(1, preview_speed_steps_));
        preview_heading /= static_cast<double>(std::max(1, preview_speed_steps_));

        for (int i = 0; i <= horizon_steps_; ++i) {
            const double t = horizon_dt_ * static_cast<double>(i);
            const PathPoint path_sample = interpolateReferencePath(anchor_s + sample_step * static_cast<double>(i));
            ReferencePoint ref;
            ref.x = path_sample.x;
            ref.y = path_sample.y;
            ref.yaw = path_sample.yaw;
            ref.curvature = clampValue(path_sample.curvature, -max_curvature_, max_curvature_);
            ref.target_x = filtered_target_.x + filtered_target_.rel_vx * t;
            ref.target_y = filtered_target_.y + filtered_target_.rel_vy * t;
            const double curvature_scale = 1.0 / (
                1.0 +
                path_curvature_speed_gain_ * std::max({std::fabs(ref.curvature), preview_curvature, preview_curvature_max}) +
                0.5 * path_heading_speed_gain_ * preview_heading);
            ref.v = clampValue(desired_speed * curvature_scale, min_speed_, max_speed_);
            if (std::hypot(ref.target_x, ref.target_y) <= stop_buffer_distance_) {
                ref.v = 0.0;
            }
            reference.push_back(ref);
        }

        if (!reference.empty()) {
            planner_debug_.cross_track_error = reference.front().y;
            planner_debug_.heading_error = wrapAngle(reference.front().yaw);
            planner_debug_.first_curvature = reference.front().curvature;
            planner_debug_.clearance_margin = std::hypot(reference.front().target_x, reference.front().target_y) - stop_buffer_distance_;
            planner_debug_.speed_target = reference.front().v;
        }
        return reference;
    }

    void buildNominalControls(const std::vector<ReferencePoint>& reference, std::vector<ControlInput>& controls) const {
        controls.assign(static_cast<size_t>(std::max(1, horizon_steps_)), ControlInput());
        for (int k = 0; k < horizon_steps_; ++k) {
            const double v0 = reference[k].v;
            const double v1 = reference[std::min(k + 1, static_cast<int>(reference.size()) - 1)].v;
            controls[static_cast<size_t>(k)].a = clampValue((v1 - v0) / std::max(0.05, horizon_dt_), min_accel_, max_accel_);
            controls[static_cast<size_t>(k)].delta = clampValue(std::atan(wheelbase_ * reference[k].curvature), -max_steer_, max_steer_);
        }
    }

    RolloutSummary rolloutTrajectory(const std::vector<ReferencePoint>& reference,
                                   const std::vector<ControlInput>& controls,
                                   double lambda_speed,
                                   double lambda_steer,
                                   double lambda_clearance,
                                   double mu_penalty) const {
        RolloutSummary summary;
        if (reference.size() < 2 || controls.empty()) {
            return summary;
        }

        summary.states.reserve(reference.size());
        PlannerState state;
        state.v = clampValue(ego_speed_mps_, min_speed_, max_speed_);
        summary.states.push_back(state);

        ControlInput previous = controls.front();
        for (int k = 0; k < horizon_steps_; ++k) {
            const ReferencePoint& ref = reference[std::min(k, static_cast<int>(reference.size()) - 1)];
            const ControlInput& u = controls[static_cast<size_t>(k)];
            state = stepDynamics(state, u, horizon_dt_);
            summary.states.push_back(state);

            const double ex = state.x - ref.x;
            const double ey = state.y - ref.y;
            const double eyaw = wrapAngle(state.yaw - ref.yaw);
            const double ev = state.v - ref.v;
            const double model_curvature = std::tan(clampValue(u.delta, -max_steer_, max_steer_)) / std::max(0.5, wheelbase_);
            const double ecurv = model_curvature - ref.curvature;
            summary.cost += q_x_ * ex * ex + q_y_ * ey * ey + q_yaw_ * eyaw * eyaw + q_v_ * ev * ev;
            summary.cost += ilqr_w_curv_ * ecurv * ecurv;
            summary.cost += r_a_ * u.a * u.a + r_delta_ * u.delta * u.delta;
            if (k > 0) {
                const double da = u.a - previous.a;
                const double ddelta = u.delta - previous.delta;
                summary.cost += r_da_ * da * da + r_ddelta_ * ddelta * ddelta;
            }
            previous = u;

            const double target_gap = std::hypot(ref.target_x - state.x, ref.target_y - state.y);
            const double clearance_violation = positivePart(stop_buffer_distance_ - target_gap);
            const double speed_violation = positivePart(state.v - max_speed_);
            const double steer_violation = positivePart(std::fabs(u.delta) - max_steer_);
            summary.max_clearance_violation = std::max(summary.max_clearance_violation, clearance_violation);
            summary.max_speed_violation = std::max(summary.max_speed_violation, speed_violation);
            summary.max_steer_violation = std::max(summary.max_steer_violation, steer_violation);
            summary.cost += speed_penalty_ * speed_violation * speed_violation;
            summary.cost += steer_penalty_ * steer_violation * steer_violation;
            summary.cost += clearance_penalty_ * clearance_violation * clearance_violation;
            summary.cost += lambda_speed * speed_violation + 0.5 * mu_penalty * speed_violation * speed_violation;
            summary.cost += lambda_steer * steer_violation + 0.5 * mu_penalty * steer_violation * steer_violation;
            summary.cost += lambda_clearance * clearance_violation + 0.5 * mu_penalty * clearance_violation * clearance_violation;
        }
        return summary;
    }

    void linearizeDynamics(const PlannerState& ref_state, const ControlInput& ref_control, Mat4& A, Mat42& B) const {
        A = identityMat4();
        B = {{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}};
        const double v = ref_state.v;
        const double yaw = ref_state.yaw;
        const double delta = clampValue(ref_control.delta, -max_steer_, max_steer_);
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const double sec2 = 1.0 / std::max(0.2, std::cos(delta) * std::cos(delta));

        A[0][2] = -v * s * horizon_dt_;
        A[0][3] = c * horizon_dt_;
        A[1][2] = v * c * horizon_dt_;
        A[1][3] = s * horizon_dt_;
        A[2][3] = std::tan(delta) * horizon_dt_ / std::max(0.5, wheelbase_);

        B[2][1] = v * sec2 * horizon_dt_ / std::max(0.5, wheelbase_);
        B[3][0] = horizon_dt_;
    }

    void backwardPass(const std::vector<ReferencePoint>& reference,
                      const std::vector<ControlInput>& nominal_controls,
                      std::vector<Mat24>& gains) const {
        gains.assign(static_cast<size_t>(horizon_steps_), Mat24{});
        Mat4 P = zeroMat4();
        P[0][0] = terminal_x_weight_;
        P[1][1] = terminal_y_weight_;
        P[2][2] = terminal_yaw_weight_;
        P[3][3] = q_v_ + 1.0;

        Mat4 Q = zeroMat4();
        Q[0][0] = q_x_;
        Q[1][1] = q_y_;
        Q[2][2] = q_yaw_;
        Q[3][3] = q_v_;

        Mat2 R = zeroMat2();
        R[0][0] = r_a_ + r_da_;
        R[1][1] = r_delta_ + r_ddelta_;

        for (int k = horizon_steps_ - 1; k >= 0; --k) {
            const int ref_index = std::min(k, static_cast<int>(reference.size()) - 1);
            PlannerState ref_state;
            ref_state.x = reference[ref_index].x;
            ref_state.y = reference[ref_index].y;
            ref_state.yaw = reference[ref_index].yaw;
            ref_state.v = reference[ref_index].v;
            Mat4 A;
            Mat42 B;
            linearizeDynamics(ref_state, nominal_controls[static_cast<size_t>(k)], A, B);

            const Mat4 At = transpose4(A);
            const Mat42 PB = mul42(P, B);
            const Mat24 BtP = transpose42(PB);
            Mat2 S = addMat2(R, mul22(BtP, B));
            S[0][0] += 1e-4;
            S[1][1] += 1e-4;
            const Mat2 S_inv = invert2(S);
            const Mat24 BtPA = mul24(BtP, A);
            Mat24 K = mul224(S_inv, BtPA);
            for (int r = 0; r < 2; ++r) {
                for (int c = 0; c < 4; ++c) {
                    K[r][c] = -K[r][c];
                }
            }
            gains[static_cast<size_t>(k)] = K;

            const Mat4 AtPA = mul44(At, mul44(P, A));
            const Mat42 AtPB = mul42(At, PB);
            const Mat4 correction = mul424(mul422(AtPB, S_inv), BtPA);
            P = addMat4(Q, subMat4(AtPA, correction));
        }
    }

    ControlInput solveLocalPlan(const std::vector<ReferencePoint>& reference) {
        if (reference.size() < 2) {
            nominal_controls_.assign(static_cast<size_t>(std::max(2, horizon_steps_)), ControlInput());
            return ControlInput();
        }

        buildNominalControls(reference, nominal_controls_);
        planner_debug_.cross_track_error = reference[0].y;
        planner_debug_.heading_error = wrapAngle(reference[0].yaw);
        planner_debug_.first_curvature = reference[0].curvature;
        planner_debug_.clearance_margin = std::hypot(reference[0].target_x, reference[0].target_y) - stop_buffer_distance_;
        planner_debug_.line_search_alpha = 0.0;

        std::vector<ControlInput> current_controls = nominal_controls_;
        double lambda_speed = lambda_speed_;
        double lambda_steer = lambda_steer_;
        double lambda_clearance = lambda_clearance_;
        double mu_penalty = mu_penalty_;
        RolloutSummary best_summary;
        double best_cost = std::numeric_limits<double>::infinity();

        for (int outer = 0; outer < std::max(1, al_outer_iterations_); ++outer) {
            std::vector<Mat24> gains;
            backwardPass(reference, current_controls, gains);

            PlannerState state;
            state.v = clampValue(ego_speed_mps_, min_speed_, max_speed_);
            std::vector<ControlInput> candidate_controls = current_controls;
            for (int k = 0; k < horizon_steps_; ++k) {
                const ReferencePoint& ref = reference[std::min(k, static_cast<int>(reference.size()) - 1)];
                const Vec4 e = {state.x - ref.x, state.y - ref.y, wrapAngle(state.yaw - ref.yaw), state.v - ref.v};
                ControlInput u = candidate_controls[static_cast<size_t>(k)];
                u.a += gains[static_cast<size_t>(k)][0][0] * e[0] + gains[static_cast<size_t>(k)][0][1] * e[1] + gains[static_cast<size_t>(k)][0][2] * e[2] + gains[static_cast<size_t>(k)][0][3] * e[3];
                u.delta += gains[static_cast<size_t>(k)][1][0] * e[0] + gains[static_cast<size_t>(k)][1][1] * e[1] + gains[static_cast<size_t>(k)][1][2] * e[2] + gains[static_cast<size_t>(k)][1][3] * e[3];
                u.a = clampValue(u.a, min_accel_, max_accel_);
                u.delta = clampValue(u.delta, -max_steer_, max_steer_);
                if (k > 0) {
                    const double lower = candidate_controls[static_cast<size_t>(k - 1)].delta - max_steer_rate_;
                    const double upper = candidate_controls[static_cast<size_t>(k - 1)].delta + max_steer_rate_;
                    u.delta = clampValue(u.delta, lower, upper);
                }
                candidate_controls[static_cast<size_t>(k)] = u;
                state = stepDynamics(state, u, horizon_dt_);
            }

            const RolloutSummary nominal_summary = rolloutTrajectory(reference, current_controls, lambda_speed, lambda_steer, lambda_clearance, mu_penalty);
            double outer_best_cost = nominal_summary.cost;
            std::vector<ControlInput> outer_best_controls = current_controls;
            RolloutSummary outer_best_summary = nominal_summary;

            for (double alpha : {1.0, line_search_decay_, line_search_decay_ * line_search_decay_, line_search_decay_ * line_search_decay_ * line_search_decay_}) {
                std::vector<ControlInput> trial = current_controls;
                for (size_t i = 0; i < trial.size() && i < candidate_controls.size(); ++i) {
                    trial[i].a = clampValue(current_controls[i].a + alpha * (candidate_controls[i].a - current_controls[i].a), min_accel_, max_accel_);
                    trial[i].delta = clampValue(current_controls[i].delta + alpha * (candidate_controls[i].delta - current_controls[i].delta), -max_steer_, max_steer_);
                }
                const RolloutSummary trial_summary = rolloutTrajectory(reference, trial, lambda_speed, lambda_steer, lambda_clearance, mu_penalty);
                if (trial_summary.cost <= outer_best_cost) {
                    outer_best_cost = trial_summary.cost;
                    outer_best_controls = trial;
                    outer_best_summary = trial_summary;
                    planner_debug_.line_search_alpha = alpha;
                    planner_debug_.rollout_cost = trial_summary.cost;
                }
            }

            current_controls = outer_best_controls;
            best_summary = outer_best_summary;
            best_cost = outer_best_cost;
            const double max_violation = std::max(best_summary.max_speed_violation, std::max(best_summary.max_steer_violation, best_summary.max_clearance_violation));
            lambda_speed *= (best_summary.max_speed_violation <= al_violation_tolerance_) ? lambda_decay_ : 1.0;
            lambda_steer *= (best_summary.max_steer_violation <= al_violation_tolerance_) ? lambda_decay_ : 1.0;
            lambda_clearance *= (best_summary.max_clearance_violation <= al_violation_tolerance_) ? lambda_decay_ : 1.0;
            if (max_violation <= al_violation_tolerance_) {
                break;
            }
            lambda_speed = clampValue(lambda_speed + mu_penalty * best_summary.max_speed_violation, 0.0, lambda_max_);
            lambda_steer = clampValue(lambda_steer + mu_penalty * best_summary.max_steer_violation, 0.0, lambda_max_);
            lambda_clearance = clampValue(lambda_clearance + mu_penalty * best_summary.max_clearance_violation, 0.0, lambda_max_);
            mu_penalty = std::min(mu_penalty * mu_growth_, mu_penalty_max_);
        }

        nominal_controls_ = current_controls;
        if (planner_debug_.line_search_alpha <= 0.0) {
            planner_debug_.line_search_alpha = 1.0;
            planner_debug_.rollout_cost = best_cost;
        }
        lambda_speed_ = lambda_speed;
        lambda_steer_ = lambda_steer;
        lambda_clearance_ = lambda_clearance;
        mu_penalty_ = std::max(mu_penalty, 1.0);
        planner_debug_.lambda_speed = lambda_speed_;
        planner_debug_.lambda_steer = lambda_steer_;
        planner_debug_.lambda_clearance = lambda_clearance_;
        planner_debug_.mu = mu_penalty_;
        planner_debug_.max_violation = std::max(best_summary.max_speed_violation, std::max(best_summary.max_steer_violation, best_summary.max_clearance_violation));
        planner_debug_.speed_clipped = std::fabs(nominal_controls_.front().a - max_accel_) < 1e-5 || std::fabs(nominal_controls_.front().a - min_accel_) < 1e-5;
        planner_debug_.steer_clipped = std::fabs(nominal_controls_.front().delta - max_steer_) < 1e-5 || std::fabs(nominal_controls_.front().delta + max_steer_) < 1e-5;
        return nominal_controls_.front();
    }

    geometry_msgs::Twist buildSafeStopCommand() {
        geometry_msgs::Twist cmd;
        lambda_speed_ *= lambda_decay_;
        lambda_steer_ *= lambda_decay_;
        lambda_clearance_ *= lambda_decay_;
        mu_penalty_ = std::max(1.0, std::min(mu_penalty_, mu_penalty_max_ * 0.25));
        filtered_speed_cmd_ += speed_cmd_alpha_ * (0.0 - filtered_speed_cmd_);
        filtered_steer_cmd_ += steer_cmd_alpha_ * (0.0 - filtered_steer_cmd_);
        cmd.linear.x = clampValue(filtered_speed_cmd_, 0.0, max_speed_);
        cmd.angular.z = clampValue(filtered_steer_cmd_, -max_steer_, max_steer_);
        return cmd;
    }

    geometry_msgs::Twist computeCommand(const ros::Time& now) {
        if (chassis_error_flag_ != 0) {
            return buildSafeStopCommand();
        }
        if (emergency_stop_active_) {
            return buildSafeStopCommand();
        }
        if (std::isfinite(nearest_obstacle_distance_) && nearest_obstacle_distance_ <= emergency_stop_distance_) {
            return buildSafeStopCommand();
        }
        if (!filtered_target_.valid || !isTargetFresh(now)) {
            planner_debug_.stale_hold = updatePredictedLockedTarget(now);
            if (!planner_debug_.stale_hold) {
                return buildSafeStopCommand();
            }
        } else {
            planner_debug_.stale_hold = false;
        }

        const std::vector<ReferencePoint> reference = buildReferenceTrajectory(now);
        if (reference.size() < 2) {
            return buildSafeStopCommand();
        }

        const ControlInput control = solveLocalPlan(reference);
        const double model_speed = clampValue(ego_speed_mps_ + control.a * lag_preview_time_, min_speed_, max_speed_);
        double preview_curvature = 0.0;
        double preview_curvature_max = 0.0;
        double preview_curvature_delta = 0.0;
        double preview_speed = 0.0;
        double preview_heading_max = 0.0;
        const int preview_count = std::max(1, std::min(preview_speed_steps_, static_cast<int>(reference.size())));
        for (int i = 0; i < preview_count; ++i) {
            preview_curvature += std::fabs(reference[static_cast<size_t>(i)].curvature);
            preview_curvature_max = std::max(preview_curvature_max, std::fabs(reference[static_cast<size_t>(i)].curvature));
            preview_speed += reference[static_cast<size_t>(i)].v;
            preview_heading_max = std::max(preview_heading_max, std::fabs(reference[static_cast<size_t>(i)].yaw));
            if (i > 0) {
                preview_curvature_delta += std::fabs(
                    reference[static_cast<size_t>(i)].curvature -
                    reference[static_cast<size_t>(i - 1)].curvature);
            }
        }
        preview_curvature /= static_cast<double>(preview_count);
        preview_curvature_delta /= static_cast<double>(std::max(1, preview_count - 1));
        preview_speed /= static_cast<double>(preview_count);
        const double curvature_scale = 1.0 / (
            1.0 +
            path_curvature_speed_gain_ * std::max(preview_curvature, preview_curvature_max) +
            path_curvature_change_speed_gain_ * preview_curvature_delta);
        const double heading_scale = 1.0 / (1.0 + path_heading_speed_gain_ * preview_heading_max);
        double desired_speed = clampValue(preview_speed * curvature_scale * heading_scale, min_speed_, max_speed_);
        desired_speed = clampValue(0.25 * model_speed + 0.75 * desired_speed, min_speed_, max_speed_);
        if (filtered_target_.distance - desired_follow_distance_ > 0.12) {
            desired_speed = std::max(
                desired_speed,
                clampValue(min_progress_speed_ + 0.14 * (filtered_target_.distance - desired_follow_distance_), 0.0, max_speed_));
        }
        if (preview_curvature_max > 0.18 || preview_heading_max > 0.45) {
            desired_speed = std::min(desired_speed, clampValue(0.08 + 0.20 / (1.0 + 4.0 * preview_curvature_max), 0.05, 0.18));
        }
        if (filtered_target_.distance <= stop_buffer_distance_) {
            desired_speed = 0.0;
        }
        filtered_speed_cmd_ += speed_cmd_alpha_ * (desired_speed - filtered_speed_cmd_);
        filtered_steer_cmd_ += steer_cmd_alpha_ * (control.delta - filtered_steer_cmd_);

        geometry_msgs::Twist cmd;
        cmd.linear.x = clampValue(filtered_speed_cmd_, 0.0, max_speed_);
        cmd.angular.z = clampValue(filtered_steer_cmd_, -max_steer_, max_steer_);
        return cmd;
    }

    void controlLoop(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        updateEgoPose(now);

        if (filtered_target_.valid) {
            const double hold_age = (now - filtered_target_.stamp).toSec();
            if (hold_age > std::max(target_timeout_, target_hold_timeout_)) {
                filtered_target_.valid = false;
            }
        }

        geometry_msgs::Twist cmd = computeCommand(now);
        cmd_pub_.publish(cmd);

        ROS_INFO_THROTTLE(
            1.0,
            "control_pub | target_valid=%d lock=%d x=%.2f y=%.2f rel_vx=%.2f ego_v=%.2f obs=%.2f cmd_v=%.2f cmd_steer=%.2f | fit=%s samples=%d inliers=%d rms=%.3f conf=%.2f span=%.2f cte=%.2f head=%.1fdeg curv=%.3f clear=%.2f vref=%.2f alpha=%.2f J=%.2f viol=%.3f lam=[%.2f,%.2f,%.2f] mu=%.2f speed_clip=%d steer_clip=%d stale_hold=%d emergency=%d",
            filtered_target_.valid ? 1 : 0,
            planner_debug_.target_locked ? 1 : 0,
            filtered_target_.x,
            filtered_target_.y,
            filtered_target_.rel_vx,
            ego_speed_mps_,
            std::isfinite(nearest_obstacle_distance_) ? nearest_obstacle_distance_ : -1.0,
            cmd.linear.x,
            cmd.angular.z,
            planner_debug_.fit_mode.c_str(),
            planner_debug_.fit_samples,
            planner_debug_.inlier_samples,
            planner_debug_.fit_rms,
            planner_debug_.fit_confidence,
            planner_debug_.history_span,
            planner_debug_.cross_track_error,
            planner_debug_.heading_error * 180.0 / M_PI,
            planner_debug_.first_curvature,
            planner_debug_.clearance_margin,
            planner_debug_.speed_target,
            planner_debug_.line_search_alpha,
            planner_debug_.rollout_cost,
            planner_debug_.max_violation,
            planner_debug_.lambda_speed,
            planner_debug_.lambda_steer,
            planner_debug_.lambda_clearance,
            planner_debug_.mu,
            planner_debug_.speed_clipped ? 1 : 0,
            planner_debug_.steer_clipped ? 1 : 0,
            planner_debug_.stale_hold ? 1 : 0,
            emergency_stop_active_ ? 1 : 0);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber tracked_sub_;
    ros::Subscriber car_sub_;
    ros::Subscriber car_vel_sub_;
    ros::Subscriber reference_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer timer_;

    std::string tracked_topic_;
    std::string car_topic_;
    std::string car_vel_topic_;
    std::string cmd_topic_;
    std::string reference_topic_;

    std::deque<WorldTargetSample> target_world_history_;
    std::vector<PathPoint> reference_path_points_;
    TargetState filtered_target_;
    std::vector<ControlInput> nominal_controls_;
    PlannerDebugInfo planner_debug_;

    double control_rate_ = 20.0;
    double chassis_speed_scale_ = 0.001;
    double wheelbase_ = 2.6;
    double steer_sign_ = -1.0;
    bool carvel_angular_is_steer_ = false;
    double carvel_yaw_rate_sign_ = 1.0;
    double compensation_gain_ = 0.94;
    double yaw_rate_filter_alpha_ = 0.82;
    double max_compensation_yaw_rate_ = 0.90;
    double min_compensation_speed_ = 0.02;
    double max_speed_ = 0.50;
    double min_speed_ = 0.0;
    double max_steer_ = 0.38;
    double max_accel_ = 0.10;
    double min_accel_ = -0.22;
    double max_steer_rate_ = 0.06;
    double emergency_stop_distance_ = 0.50;
    double desired_follow_distance_ = 1.00;
    double stop_buffer_distance_ = 0.50;
    double target_timeout_ = 0.60;
    double target_hold_timeout_ = 1.00;
    double reference_timeout_ = 0.25;
    double reference_hold_timeout_ = 0.80;
    int reference_min_points_ = 5;
    double reference_sample_spacing_ = 0.12;
    double reference_anchor_forward_offset_ = 0.10;
    double reference_anchor_lateral_weight_ = 1.8;
    double reference_quality_span_ = 0.80;
    int reference_quality_points_ = 18;
    double path_curvature_speed_gain_ = 1.6;
    double path_path_hold_speed_scale_ = 0.85;
    double path_confidence_floor_ = 0.35;
    double lateral_gate_ = 1.00;
    double min_forward_target_x_ = 0.12;
    double pose_alpha_ = 0.28;
    double yaw_alpha_ = 0.18;
    double velocity_alpha_ = 0.14;
    double speed_cmd_alpha_ = 0.14;
    double steer_cmd_alpha_ = 0.16;

    int horizon_steps_ = 20;
    double horizon_dt_ = 0.10;
    double reference_history_window_ = 1.50;
    int fit_min_samples_ = 4;
    double fit_max_rms_ = 0.35;
    double fit_blend_gain_ = 2.5;
    double outlier_reject_distance_ = 0.65;
    double outlier_reject_speed_ = 1.20;
    double huber_delta_ = 0.18;
    double fit_ridge_lambda_ = 0.45;
    double low_speed_heading_threshold_ = 0.12;
    double low_speed_span_threshold_ = 0.45;
    double heading_memory_alpha_ = 0.85;
    double curvature_memory_alpha_ = 0.82;
    double min_progress_speed_ = 0.12;
    int preview_speed_steps_ = 4;
    double max_curvature_ = 0.22;
    double max_curvature_rate_ = 0.035;
    double max_heading_step_ = 0.10;
    double gap_speed_gain_ = 0.24;
    double fit_bad_speed_scale_ = 0.55;
    double line_search_decay_ = 0.5;
    double temporal_heading_weight_ = 2.4;
    double temporal_speed_weight_ = 1.2;
    double lock_distance_gate_ = 0.90;
    double lock_yaw_gate_ = 0.90;
    double lock_hold_timeout_ = 1.20;
    int al_outer_iterations_ = 3;
    double al_violation_tolerance_ = 0.02;
    double lambda_speed_ = 0.0;
    double lambda_steer_ = 0.0;
    double lambda_clearance_ = 0.0;
    double mu_penalty_ = 10.0;
    double mu_growth_ = 1.35;
    double mu_penalty_max_ = 120.0;
    double lambda_max_ = 400.0;
    double lambda_decay_ = 0.80;
    double path_curvature_change_speed_gain_ = 3.2;
    double path_heading_speed_gain_ = 1.05;

    double q_x_ = 2.5;
    double q_y_ = 14.0;
    double q_yaw_ = 4.0;
    double q_v_ = 2.0;
    double r_a_ = 0.35;
    double r_delta_ = 0.55;
    double r_da_ = 1.40;
    double r_ddelta_ = 1.80;
    double clearance_penalty_ = 220.0;
    double speed_penalty_ = 160.0;
    double steer_penalty_ = 60.0;
    double terminal_x_weight_ = 4.0;
    double terminal_y_weight_ = 18.0;
    double terminal_yaw_weight_ = 6.0;
    double ilqr_w_curv_ = 14.0;
    double lag_preview_time_ = 0.10;

    double ego_speed_mps_ = 0.0;
    double steering_feedback_ = 0.0;
    double carvel_yaw_rate_ = 0.0;
    int chassis_error_flag_ = 0;
    double nearest_obstacle_distance_ = std::numeric_limits<double>::infinity();
    bool emergency_stop_active_ = false;
    double filtered_speed_cmd_ = 0.0;
    double filtered_steer_cmd_ = 0.0;

    bool ego_pose_initialized_ = false;
    ros::Time last_ego_pose_time_;
    double ego_world_x_ = 0.0;
    double ego_world_y_ = 0.0;
    double ego_world_yaw_ = 0.0;
    double target_world_vx_ = 0.0;
    double target_world_vy_ = 0.0;
    bool target_lock_active_ = false;
    ros::Time target_lock_stamp_;
    double target_lock_world_x_ = 0.0;
    double target_lock_world_y_ = 0.0;
    double target_lock_world_yaw_ = 0.0;
    double target_lock_world_vx_ = 0.0;
    double target_lock_world_vy_ = 0.0;
    double target_lock_world_yaw_rate_ = 0.0;
    double path_heading_memory_ = 0.0;
    double path_curvature_memory_ = 0.0;
    bool has_reference_path_ = false;
    ros::Time reference_path_stamp_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointpillars_follower_controller");
    FollowerController controller;
    ros::spin();
    return 0;
}