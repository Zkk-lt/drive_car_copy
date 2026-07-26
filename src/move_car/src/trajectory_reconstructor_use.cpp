/*
改进版：响应更快
*/
#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_car/car_parameter.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>

namespace {

double clampValue(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double hypot2(double x, double y) {
  return std::sqrt(x * x + y * y);
}

double pointDistance(double x0, double y0, double x1, double y1) {
  return hypot2(x1 - x0, y1 - y0);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

struct EgoState {
  ros::Time stamp;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double delta = 0.0;
  double yaw_rate = 0.0;
  double sigma_yaw = 0.0;
};

using EgoPose2D = EgoState;

struct HistoryPoint2D {
  ros::Time stamp;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double curvature = 0.0;
  bool predicted = false;
  double weight = 1.0;
  double ego_turn_contamination_score = 0.0;
  bool heading_decoupled_by_ego_turn = false;
  bool bbox_heading_used_for_history = false;
  double fixed_frame_lateral_velocity = 0.0;
  double fixed_frame_lateral_velocity_filtered = 0.0;
  double lateral_innovation_gate = 0.0;
  double lateral_innovation_frenet = 0.0;
  double cross_residual_filtered = 0.0;
  double tail_along_weight = 1.0;
  double tail_cross_weight = 1.0;
  bool cross_hold_active = false;
};

struct LocalPathPoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double curvature = 0.0;
};

struct LeadNode {
  ros::Time stamp;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double a = 0.0;
  double kappa = 0.0;
  double dkappa = 0.0;
  bool has_measurement = false;
  double z_rel_x = 0.0;
  double z_rel_y = 0.0;
  double w_along = 0.0;
  double w_cross = 0.0;
  EgoState ego;
  double heading_hint = 0.0;
  double range = 0.0;
  double residual_along = 0.0;
  double residual_cross = 0.0;
  double robust_weight = 1.0;
  double fixed_frame_speed = 0.0;
  double static_hypothesis_residual = 0.0;
  double short_window_prediction_residual = 0.0;
  double tail_weight = 1.0;
  double ego_turn_contamination_score = 0.0;
  bool heading_decoupled_by_ego_turn = false;
  bool bbox_heading_used_for_history = false;
  double fixed_frame_lateral_velocity = 0.0;
  double fixed_frame_lateral_velocity_filtered = 0.0;
  double lateral_innovation_gate = 0.0;
  double lateral_innovation_frenet = 0.0;
  double cross_residual_filtered = 0.0;
  double tail_along_weight = 1.0;
  double tail_cross_weight = 1.0;
  bool cross_hold_active = false;
};

struct RawTrailPoint {
  double s = 0.0;
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double kappa = 0.0;
  ros::Time stamp;
};

struct RefPoint {
  double s = 0.0;
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double kappa = 0.0;
  double v_ref = 0.0;
  double a_ref = 0.0;
};

LocalPathPoint localPathPointFromRef(const RefPoint& ref) {
  LocalPathPoint pt;
  pt.x = ref.x;
  pt.y = ref.y;
  pt.yaw = ref.yaw;
  pt.curvature = ref.kappa;
  return pt;
}

struct LeadObservation {
  bool valid = false;
  ros::Time stamp;
  double range = 0.0;
  double local_x = 0.0;
  double local_y = 0.0;
  double local_heading = 0.0;
  double world_x = 0.0;
  double world_y = 0.0;
  double world_heading = 0.0;
};

struct LeadState2D {
  ros::Time stamp;
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double heading = 0.0;
  bool active = false;
  bool static_mode = false;
};

double polylineLengthLocal(const std::vector<LocalPathPoint>& pts) {
  double length = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    length += pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
  }
  return length;
}

double polylineLengthWorld(const std::deque<HistoryPoint2D>& pts) {
  double length = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    length += pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
  }
  return length;
}

struct TrackerDebugState {
  std::string projector_reason = "init";
  std::string history_reason = "init";
  std::string raw_reason = "init";
  std::string reference_reason = "init";
  int input_pose_count = 0;
  int valid_candidate_count = 0;
  int lock_reject_count = 0;
  size_t history_size = 0;
  double raw_span = 0.0;
  double reference_span = 0.0;
  double candidate_reference_span = 0.0;
  double published_reference_span = 0.0;
  std::string published_reference_mode = "init";
  std::string published_reference_reason = "init";
  double published_span_jump = 0.0;
  double published_endpoint_jump = 0.0;
  double published_start_jump = 0.0;
  double published_heading_jump = 0.0;
  bool held_by_hysteresis = false;
  bool candidate_full_valid = false;
  bool published_full_valid = false;
  double reference_full_max_yaw_jump = 0.0;
  int reference_full_backtrack_count = 0;
  double reference_full_kappa_max = 0.0;
  int reference_cut_prefix_points = 0;
  bool reference_output_growth_limited = false;
  double published_span_target = 0.0;
  std::string reference_reject_full_reason = "none";
  double ff_speed = 0.0;
  double mean_ego_speed = 0.0;
  double static_hyp_res = 0.0;
  double hs_score = 0.0;
  double hd_score = 0.0;
  double accum_s = 0.0;
  double accum_t = 0.0;
  size_t confirmed_count = 0;
  size_t tail_count = 0;
  std::string ego_pose_source = "init";
  bool ego_pose_valid = false;
  double ego_pose_age = -1.0;
  double ego_delta_yaw = 0.0;
  double ego_delta_xy = 0.0;
  double ego_yaw_uncertainty = 0.0;
  double steer_raw = 0.0;
  double steer_scaled = 0.0;
  bool steer_scale_suspect = false;
  double bicycle_yaw_rate_from_steer = 0.0;
  double imu_vs_bicycle_yaw_rate_error = 0.0;
  bool ned_odom_received = false;
  bool tf_pose_received = false;
  bool imu_yaw_rate_used = false;
  bool fixed_frame_append_valid = false;
  double fixed_frame_progress = 0.0;
  double fixed_frame_kink_angle = 0.0;
  double history_yaw_jump = 0.0;
  double history_curvature_peak = 0.0;
  bool reference_from_fixed_frame = false;
  bool reference_backtrack_reject = false;
  double reference_splice_kappa_max = 0.0;
  double reference_splice_yaw_jump = 0.0;
  int reference_join_retry_count = 0;
  int stable_segment_count = 0;
  double selected_segment_span = 0.0;
  double selected_segment_score = 0.0;
  double reference_hold_age = -1.0;
  std::string reference_publish_mode = "init";
  bool history_window_commit = false;
  int provisional_tail_points = 0;
  double provisional_tail_span = 0.0;
  double provisional_tail_age = -1.0;
  int motion_tail_points = 0;
  double motion_tail_span = 0.0;
  double motion_tail_age = -1.0;
  bool static_anchor_suppressed_by_motion = false;
  bool static_anchor_blocked_by_motion_tail = false;
  bool fixed_lag_commit = false;
  double fixed_lag_commit_span = 0.0;
  bool reference_trackable_raw = false;
  bool motion_tail_used_as_reference = false;
  bool motion_tail_promoted_to_reference = false;
  bool motion_tail_direct_publish = false;
  bool full_audit_hard_valid = false;
  bool full_audit_kappa_limited = false;
  bool published_geometry_valid = false;
  int actual_published_points = 0;
  double actual_published_span = 0.0;
  std::string actual_published_mode = "init";
  std::string actual_published_reason = "init";
  std::string candidate_block_reason = "none";
  std::string held_publish_reason = "none";
  std::string provisional_publish_block_reason = "none";
  std::string provisional_tail_block_detail = "none";
  std::string confirmed_append_block_reason = "none";
  std::string selected_segment_source = "none";
  double reference_empty_duration = 0.0;
  double reference_mode_age = 0.0;
  std::string publisher_action = "init";
  double published_hold_age = -1.0;
  double append_stall_age = -1.0;
  double candidate_jump_to_published = 0.0;
  double published_stale_age = 0.0;
  double published_endpoint_speed = 0.0;
  bool reference_stale = false;
  bool motion_tail_extension_used = false;
  bool far_start_recovered_by_tail = false;
  bool stale_hold_demoted = false;
  bool motion_tail_hard_valid = false;
  std::string motion_tail_block_reason = "none";
  std::string outlier_gate_reason = "none";
  int unique_tail_points = 0;
  double unique_tail_span = 0.0;
  double unique_tail_time = -1.0;
  bool hold_forced_release = false;
  bool shrink_limited_publish = false;
  double candidate_vs_published_span_diff = 0.0;
  bool raw_direct_publish = false;
  bool connector_used = false;
  int connector_join_index = -1;
  double connector_span = 0.0;
  double suffix_span = 0.0;
  double stitched_span = 0.0;
  std::string reference_source = "none";
  double reference_estimator_confidence = 0.0;
  double confirmed_suffix_span = 0.0;
  double tail_suffix_span = 0.0;
  double suffix_after_join_span = 0.0;
  double splice_preserved_suffix_span = 0.0;
  bool splice_trim_rejected_connector_only = false;
  double smooth_output_span = 0.0;
  double candidate_full_span_before_audit = 0.0;
  double candidate_full_span_after_audit = 0.0;
  double tail_extension_latency = -1.0;
  bool fixed_lag_tail_used = false;
  std::string join_reject_reason = "none";
  double reference_state_update_alpha = 1.0;
  std::string published_update_mode = "init";
  bool published_suffix_preserved_ok = true;
  bool connector_only_publish_blocked = false;
  double reference_refresh_latency = -1.0;
  double suffix_estimator_tail_age = -1.0;
  double suffix_estimator_output_span = 0.0;
  std::string stale_hold_block_reason = "none";
  double suffix_extension_span = 0.0;
  bool suffix_extension_growth_limited = false;
  bool reconnected_last_good_suffix = false;
  double join_connector_ratio = 0.0;
  double connector_inner_cut_score = 0.0;
  double connector_lateral_shortcut = 0.0;
  bool anti_inner_cut_rejoin = false;
  std::string candidate_source = "none";
  std::string reference_empty_reason_final = "none";
  double suffix_wiggle_score = 0.0;
  int suffix_sign_flip_count = 0;
  double suffix_smooth_rms = 0.0;
  double suffix_curvature_p95 = 0.0;
  std::string reference_freshness_state = "invalid";
  bool append_gate_softened = false;
  double tail_point_weight_min = 1.0;
  double tail_point_weight_max = 1.0;
  bool suffix_s_wave_suppressed = false;
  double suffix_wiggle_before = 0.0;
  double suffix_wiggle_after = 0.0;
  double reference_health_score = 0.0;
  bool reference_publish_forced_by_tail = false;
  bool ego_motion_decoupled_tail_yaw = false;
  double tail_point_weight = 1.0;
  double tail_point_weight_mean = 1.0;
  double ego_turn_contamination_score = 0.0;
  bool heading_decoupled_by_ego_turn = false;
  bool bbox_heading_used_for_history = false;
  double lateral_innovation_gate = 0.0;
  double fixed_frame_lateral_velocity = 0.0;
  double fixed_frame_lateral_velocity_filtered = 0.0;
  std::string suffix_fit_mode = "none";
  double suffix_curvature_rate_p95 = 0.0;
  bool reference_flatness_valid = false;
  double reference_tracking_confidence = 0.0;
  double tail_along_weight_mean = 1.0;
  double tail_cross_weight_mean = 1.0;
  bool cross_hold_active = false;
  std::string cross_hold_reason = "none";
  bool suffix_centerline_fit_active = false;
  double tail_cross_weight_applied = 1.0;
  bool ego_turn_cross_hold_active = false;
  double lateral_innovation_frenet = 0.0;
  double cross_residual_filtered = 0.0;
  double ego_pose_deskew_age = 0.0;
  double join_index_smooth = -1.0;
  double join_index_jump = 0.0;
  double join_state_s = -1.0;
  std::string join_reselect_reason = "none";
  int publisher_retry_join_count = 0;
  bool bootstrap_stitched = false;
  double published_endpoint_age = -1.0;
  double tail_endpoint_age = -1.0;
  std::string reference_latency_source = "none";
  double accepted_source_age = -1.0;
  double geometry_progress_age = -1.0;
  double candidate_source_stamp = -1.0;
  uint64_t reference_update_seq = 0;
  bool curve_refit_active = false;
  double curve_refit_rms = 0.0;
  double reconstruction_compute_ms = 0.0;
  std::string tail_sample_action = "none";
  double tail_pending_distance = 0.0;
  double tail_pending_time = 0.0;
  double accepted_suffix_source_age = -1.0;
  double join_skip_span = 0.0;
  double final_connector_ratio = 0.0;
  double tangent_progress_min = 0.0;
  double fallback_suffix_age = -1.0;
};

void resetReferenceCycleDebug(TrackerDebugState* debug) {
  if (debug == NULL) {
    return;
  }
  debug->candidate_block_reason = "none";
  debug->reference_reject_full_reason = "none";
  debug->join_reject_reason = "none";
  debug->reference_join_retry_count = 0;
  debug->publisher_retry_join_count = 0;
  debug->anti_inner_cut_rejoin = false;
  debug->connector_inner_cut_score = 0.0;
  debug->connector_lateral_shortcut = 0.0;
  debug->splice_trim_rejected_connector_only = false;
  debug->connector_only_publish_blocked = false;
  debug->curve_refit_active = false;
  debug->curve_refit_rms = 0.0;
  debug->suffix_centerline_fit_active = false;
  debug->suffix_s_wave_suppressed = false;
  debug->suffix_fit_mode = "none";
  debug->join_skip_span = 0.0;
  debug->final_connector_ratio = 0.0;
  debug->tangent_progress_min = 0.0;
  debug->fallback_suffix_age = -1.0;
  debug->accepted_suffix_source_age = -1.0;
}

EgoPose2D interpolatePose(const EgoPose2D& a, const EgoPose2D& b, double t) {
  EgoPose2D out;
  out.stamp = (t < 0.5) ? a.stamp : b.stamp;
  out.x = a.x + t * (b.x - a.x);
  out.y = a.y + t * (b.y - a.y);
  out.yaw = wrapAngle(a.yaw + t * wrapAngle(b.yaw - a.yaw));
  out.v = a.v + t * (b.v - a.v);
  out.delta = a.delta + t * (b.delta - a.delta);
  out.yaw_rate = a.yaw_rate + t * (b.yaw_rate - a.yaw_rate);
  out.sigma_yaw = a.sigma_yaw + t * (b.sigma_yaw - a.sigma_yaw);
  return out;
}

void transformLocalToWorld(const EgoPose2D& ego,
                           double local_x,
                           double local_y,
                           double* world_x,
                           double* world_y) {
  const double c = std::cos(ego.yaw);
  const double s = std::sin(ego.yaw);
  *world_x = ego.x + c * local_x - s * local_y;
  *world_y = ego.y + s * local_x + c * local_y;
}

void transformWorldToLocal(const EgoPose2D& ego,
                           double world_x,
                           double world_y,
                           double* local_x,
                           double* local_y) {
  const double dx = world_x - ego.x;
  const double dy = world_y - ego.y;
  const double c = std::cos(ego.yaw);
  const double s = std::sin(ego.yaw);
  *local_x = c * dx + s * dy;
  *local_y = -s * dx + c * dy;
}

void recomputeHeading(std::vector<HistoryPoint2D>& pts) {
  if (pts.size() < 2) {
    return;
  }
  for (size_t i = 0; i < pts.size(); ++i) {
    const size_t i0 = (i == 0) ? i : i - 1;
    const size_t i1 = (i + 1 >= pts.size()) ? i : i + 1;
    if (i0 == i1) {
      continue;
    }
    const double dx = pts[i1].x - pts[i0].x;
    const double dy = pts[i1].y - pts[i0].y;
    if (hypot2(dx, dy) > 1e-4) {
      pts[i].heading = std::atan2(dy, dx);
    }
  }
  pts.front().heading = pts[std::min<size_t>(1, pts.size() - 1)].heading;
  pts.back().heading = pts[pts.size() - 2].heading;
}

void recomputeHeadingStable(std::vector<HistoryPoint2D>& pts,
                            double min_distance,
                            double hold_distance,
                            double max_yaw_step) {
  if (pts.size() < 2) {
    return;
  }
  const double heading_ds = std::max(0.02, min_distance);
  const double hold_ds = std::max(0.0, hold_distance);
  std::vector<double> headings(pts.size(), pts.front().heading);
  for (size_t i = 0; i < pts.size(); ++i) {
    size_t i0 = i;
    size_t i1 = i;
    while (i0 > 0 &&
           pointDistance(pts[i0].x, pts[i0].y, pts[i].x, pts[i].y) < heading_ds) {
      --i0;
    }
    while (i1 + 1 < pts.size() &&
           pointDistance(pts[i1].x, pts[i1].y, pts[i].x, pts[i].y) < heading_ds) {
      ++i1;
    }
    if (i0 == i && i > 0) {
      i0 = i - 1;
    }
    if (i1 == i && i + 1 < pts.size()) {
      i1 = i + 1;
    }
    const double dx = pts[i1].x - pts[i0].x;
    const double dy = pts[i1].y - pts[i0].y;
    if (hypot2(dx, dy) > 1e-4) {
      headings[i] = std::atan2(dy, dx);
    } else if (i > 0) {
      headings[i] = headings[i - 1];
    }
  }

  pts.front().heading = headings[std::min<size_t>(1, headings.size() - 1)];
  for (size_t i = 1; i < pts.size(); ++i) {
    const double step = pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
    double desired = headings[i];
    if (step < hold_ds) {
      desired = pts[i - 1].heading;
    }
    const double limited =
        clampValue(wrapAngle(desired - pts[i - 1].heading),
                   -std::fabs(max_yaw_step),
                   std::fabs(max_yaw_step));
    pts[i].heading = wrapAngle(pts[i - 1].heading + limited);
  }
  if (pts.size() >= 2) {
    pts.front().heading = pts[1].heading;
  }
}

void recomputeLocalHeading(std::vector<LocalPathPoint>& pts) {
  if (pts.size() < 2) {
    return;
  }
  for (size_t i = 0; i < pts.size(); ++i) {
    const size_t i0 = (i == 0) ? i : i - 1;
    const size_t i1 = (i + 1 >= pts.size()) ? i : i + 1;
    if (i0 == i1) {
      continue;
    }
    const double dx = pts[i1].x - pts[i0].x;
    const double dy = pts[i1].y - pts[i0].y;
    if (hypot2(dx, dy) > 1e-4) {
      pts[i].yaw = std::atan2(dy, dx);
    }
  }
  pts.front().yaw = pts[std::min<size_t>(1, pts.size() - 1)].yaw;
  pts.back().yaw = pts[pts.size() - 2].yaw;
}

std::vector<HistoryPoint2D> removeNearDuplicatePoints(const std::vector<HistoryPoint2D>& in,
                                                      double dist_eps) {
  if (in.empty()) {
    return in;
  }

  std::vector<HistoryPoint2D> out;
  out.reserve(in.size());
  out.push_back(in.front());
  for (size_t i = 1; i < in.size(); ++i) {
    if (pointDistance(out.back().x, out.back().y, in[i].x, in[i].y) >= dist_eps) {
      out.push_back(in[i]);
    } else {
      out.back() = in[i];
    }
  }
  return out;
}

std::vector<HistoryPoint2D> resampleWorldByArcLength(const std::vector<HistoryPoint2D>& in,
                                                     double ds) {
  if (in.size() < 2 || ds <= 1e-4) {
    return in;
  }

  std::vector<double> arc(in.size(), 0.0);
  for (size_t i = 1; i < in.size(); ++i) {
    arc[i] = arc[i - 1] + pointDistance(in[i - 1].x, in[i - 1].y, in[i].x, in[i].y);
  }

  const double total = arc.back();
  if (total < 1e-4) {
    return {in.back()};
  }

  std::vector<HistoryPoint2D> out;
  out.reserve(static_cast<size_t>(std::ceil(total / ds)) + 2);
  size_t seg = 0;
  const size_t samples = std::max<size_t>(2, static_cast<size_t>(std::floor(total / ds)) + 1);

  for (size_t k = 0; k < samples; ++k) {
    const double sk = std::min(total, static_cast<double>(k) * ds);
    while (seg + 1 < arc.size() && arc[seg + 1] < sk) {
      ++seg;
    }
    if (seg + 1 >= in.size()) {
      out.push_back(in.back());
      break;
    }

    const double denom = std::max(1e-6, arc[seg + 1] - arc[seg]);
    const double t = clampValue((sk - arc[seg]) / denom, 0.0, 1.0);
    HistoryPoint2D pt;
    pt.stamp = (t < 0.5) ? in[seg].stamp : in[seg + 1].stamp;
    pt.x = in[seg].x + t * (in[seg + 1].x - in[seg].x);
    pt.y = in[seg].y + t * (in[seg + 1].y - in[seg].y);
    pt.heading = wrapAngle(in[seg].heading + t * wrapAngle(in[seg + 1].heading - in[seg].heading));
    pt.speed = in[seg].speed + t * (in[seg + 1].speed - in[seg].speed);
    pt.curvature = in[seg].curvature + t * (in[seg + 1].curvature - in[seg].curvature);
    pt.predicted = in[seg].predicted || in[seg + 1].predicted;
    pt.weight = in[seg].weight + t * (in[seg + 1].weight - in[seg].weight);
    pt.ego_turn_contamination_score =
        in[seg].ego_turn_contamination_score +
        t * (in[seg + 1].ego_turn_contamination_score -
             in[seg].ego_turn_contamination_score);
    pt.heading_decoupled_by_ego_turn =
        in[seg].heading_decoupled_by_ego_turn || in[seg + 1].heading_decoupled_by_ego_turn;
    pt.bbox_heading_used_for_history =
        in[seg].bbox_heading_used_for_history || in[seg + 1].bbox_heading_used_for_history;
    pt.fixed_frame_lateral_velocity =
        in[seg].fixed_frame_lateral_velocity +
        t * (in[seg + 1].fixed_frame_lateral_velocity -
             in[seg].fixed_frame_lateral_velocity);
    pt.lateral_innovation_gate =
        in[seg].lateral_innovation_gate +
        t * (in[seg + 1].lateral_innovation_gate -
             in[seg].lateral_innovation_gate);
    pt.lateral_innovation_frenet =
        in[seg].lateral_innovation_frenet +
        t * (in[seg + 1].lateral_innovation_frenet -
             in[seg].lateral_innovation_frenet);
    pt.tail_along_weight =
        in[seg].tail_along_weight +
        t * (in[seg + 1].tail_along_weight -
             in[seg].tail_along_weight);
    pt.tail_cross_weight =
        in[seg].tail_cross_weight +
        t * (in[seg + 1].tail_cross_weight -
             in[seg].tail_cross_weight);
    pt.cross_hold_active = in[seg].cross_hold_active || in[seg + 1].cross_hold_active;
    out.push_back(pt);
  }

  if (pointDistance(out.back().x, out.back().y, in.back().x, in.back().y) > 1e-3) {
    out.push_back(in.back());
  } else {
    out.back() = in.back();
  }

  recomputeHeading(out);
  return out;
}

std::vector<HistoryPoint2D> smoothWorldPath5(const std::vector<HistoryPoint2D>& in,
                                             int passes) {
  if (in.size() < 5 || passes <= 0) {
    return in;
  }

  std::vector<HistoryPoint2D> cur = in;
  std::vector<HistoryPoint2D> nxt = in;
  for (int pass = 0; pass < passes; ++pass) {
    nxt = cur;
    for (size_t i = 2; i + 2 < cur.size(); ++i) {
      nxt[i].x = (-3.0 * cur[i - 2].x + 12.0 * cur[i - 1].x + 17.0 * cur[i].x +
                  12.0 * cur[i + 1].x - 3.0 * cur[i + 2].x) / 35.0;
      nxt[i].y = (-3.0 * cur[i - 2].y + 12.0 * cur[i - 1].y + 17.0 * cur[i].y +
                  12.0 * cur[i + 1].y - 3.0 * cur[i + 2].y) / 35.0;
    }
    cur.swap(nxt);
  }
  recomputeHeading(cur);
  return cur;
}

std::vector<HistoryPoint2D> smoothWorldSplineLike(const std::vector<HistoryPoint2D>& in,
                                                  int passes,
                                                  double alpha,
                                                  double tail_weight_scale) {
  if (in.size() < 3 || passes <= 0 || alpha <= 1e-4) {
    return in;
  }

  std::vector<HistoryPoint2D> cur = in;
  std::vector<HistoryPoint2D> nxt = in;
  const double tail_scale = clampValue(tail_weight_scale, 0.05, 1.0);

  for (int pass = 0; pass < passes; ++pass) {
    nxt = cur;
    for (size_t i = 1; i + 1 < cur.size(); ++i) {
      const double neighbor_x = 0.5 * (cur[i - 1].x + cur[i + 1].x);
      const double neighbor_y = 0.5 * (cur[i - 1].y + cur[i + 1].y);
      const double keep_weight = cur[i].predicted ? tail_scale : 1.0;
      const double smooth_gain = alpha * (1.0 - keep_weight);
      nxt[i].x = (1.0 - smooth_gain) * cur[i].x + smooth_gain * neighbor_x;
      nxt[i].y = (1.0 - smooth_gain) * cur[i].y + smooth_gain * neighbor_y;
    }
    cur.swap(nxt);
  }

  recomputeHeading(cur);
  return cur;
}

HistoryPoint2D interpolateWorldHermite(const HistoryPoint2D& a,
                                       const HistoryPoint2D& b,
                                       double tangent_scale_a,
                                       double tangent_scale_b,
                                       double t) {
  const double tt = clampValue(t, 0.0, 1.0);
  const double h00 = 2.0 * tt * tt * tt - 3.0 * tt * tt + 1.0;
  const double h10 = tt * tt * tt - 2.0 * tt * tt + tt;
  const double h01 = -2.0 * tt * tt * tt + 3.0 * tt * tt;
  const double h11 = tt * tt * tt - tt * tt;

  const double m0x = tangent_scale_a * std::cos(a.heading);
  const double m0y = tangent_scale_a * std::sin(a.heading);
  const double m1x = tangent_scale_b * std::cos(b.heading);
  const double m1y = tangent_scale_b * std::sin(b.heading);

  HistoryPoint2D out;
  out.stamp = (tt < 0.5) ? a.stamp : b.stamp;
  out.x = h00 * a.x + h10 * m0x + h01 * b.x + h11 * m1x;
  out.y = h00 * a.y + h10 * m0y + h01 * b.y + h11 * m1y;

  const double dh00 = 6.0 * tt * tt - 6.0 * tt;
  const double dh10 = 3.0 * tt * tt - 4.0 * tt + 1.0;
  const double dh01 = -6.0 * tt * tt + 6.0 * tt;
  const double dh11 = 3.0 * tt * tt - 2.0 * tt;
  const double dx = dh00 * a.x + dh10 * m0x + dh01 * b.x + dh11 * m1x;
  const double dy = dh00 * a.y + dh10 * m0y + dh01 * b.y + dh11 * m1y;
  out.heading = std::atan2(dy, dx);
  out.speed = (1.0 - tt) * a.speed + tt * b.speed;

  const double ddh00 = 12.0 * tt - 6.0;
  const double ddh10 = 6.0 * tt - 4.0;
  const double ddh01 = -12.0 * tt + 6.0;
  const double ddh11 = 6.0 * tt - 2.0;
  const double ddx = ddh00 * a.x + ddh10 * m0x + ddh01 * b.x + ddh11 * m1x;
  const double ddy = ddh00 * a.y + ddh10 * m0y + ddh01 * b.y + ddh11 * m1y;
  const double denom = std::pow(std::max(1e-6, dx * dx + dy * dy), 1.5);
  out.curvature = clampValue((dx * ddy - dy * ddx) / denom, -0.8, 0.8);
  return out;
}

std::vector<HistoryPoint2D> fitWorldTrajectoryHermite(const std::vector<HistoryPoint2D>& knots,
                                                      double ds) {
  if (knots.size() < 2 || ds <= 1e-4) {
    return knots;
  }

  std::vector<HistoryPoint2D> fitted;
  fitted.reserve(knots.size() * 4);
  fitted.push_back(knots.front());
  for (size_t i = 0; i + 1 < knots.size(); ++i) {
    const HistoryPoint2D& a = knots[i];
    const HistoryPoint2D& b = knots[i + 1];
    const double seg_len = pointDistance(a.x, a.y, b.x, b.y);
    if (seg_len < 1e-4) {
      continue;
    }
    const double tangent_scale_a =
        (i == 0) ? seg_len : 0.5 * (seg_len + pointDistance(knots[i - 1].x, knots[i - 1].y, a.x, a.y));
    const double tangent_scale_b =
        (i + 2 >= knots.size()) ? seg_len
                                : 0.5 * (seg_len + pointDistance(b.x, b.y, knots[i + 2].x, knots[i + 2].y));
    const int steps = std::max(2, static_cast<int>(std::ceil(seg_len / ds)));
    for (int k = 1; k <= steps; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(steps);
      HistoryPoint2D sample = interpolateWorldHermite(a, b, tangent_scale_a, tangent_scale_b, t);
      if (fitted.empty() ||
          pointDistance(fitted.back().x, fitted.back().y, sample.x, sample.y) > 1e-3) {
        fitted.push_back(sample);
      } else {
        fitted.back() = sample;
      }
    }
  }
  recomputeHeading(fitted);
  return fitted;
}

void offsetWorldPathAlongHeading(std::vector<HistoryPoint2D>* pts, double offset) {
  if (pts == NULL || pts->empty() || std::fabs(offset) <= 1e-6) {
    return;
  }

  recomputeHeading(*pts);
  for (size_t i = 0; i < pts->size(); ++i) {
    (*pts)[i].x -= offset * std::cos((*pts)[i].heading);
    (*pts)[i].y -= offset * std::sin((*pts)[i].heading);
  }
}

std::vector<LocalPathPoint> transformWorldPathToLocal(const std::vector<HistoryPoint2D>& world_pts,
                                                      const EgoPose2D& ego) {
  std::vector<LocalPathPoint> out;
  out.reserve(world_pts.size());
  for (size_t i = 0; i < world_pts.size(); ++i) {
    LocalPathPoint pt;
    transformWorldToLocal(ego, world_pts[i].x, world_pts[i].y, &pt.x, &pt.y);
    pt.yaw = wrapAngle(world_pts[i].heading - ego.yaw);
    out.push_back(pt);
  }
  return out;
}

std::vector<RawTrailPoint> makeRawTrailPoints(const std::vector<HistoryPoint2D>& world_pts) {
  std::vector<RawTrailPoint> out;
  out.reserve(world_pts.size());
  double s = 0.0;
  const ros::Time base_stamp = world_pts.empty() ? ros::Time() : world_pts.front().stamp;
  for (size_t i = 0; i < world_pts.size(); ++i) {
    if (i > 0) {
      s += pointDistance(world_pts[i - 1].x, world_pts[i - 1].y, world_pts[i].x, world_pts[i].y);
    }
    RawTrailPoint pt;
    pt.s = s;
    pt.t = (base_stamp.isValid() && world_pts[i].stamp.isValid()) ? (world_pts[i].stamp - base_stamp).toSec() : 0.0;
    pt.x = world_pts[i].x;
    pt.y = world_pts[i].y;
    pt.yaw = world_pts[i].heading;
    pt.kappa = world_pts[i].curvature;
    pt.stamp = world_pts[i].stamp;
    out.push_back(pt);
  }
  return out;
}

std::vector<RefPoint> makeRefPoints(const std::vector<LocalPathPoint>& local_pts,
                                    double nominal_speed) {
  std::vector<RefPoint> out;
  out.reserve(local_pts.size());
  double s = 0.0;
  double prev_v = nominal_speed;
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i > 0) {
      s += pointDistance(local_pts[i - 1].x, local_pts[i - 1].y, local_pts[i].x, local_pts[i].y);
    }
    RefPoint pt;
    pt.s = s;
    pt.x = local_pts[i].x;
    pt.y = local_pts[i].y;
    pt.yaw = local_pts[i].yaw;
    pt.kappa = local_pts[i].curvature;
    pt.v_ref = clampValue(nominal_speed * std::max(0.4, 1.0 - 1.5 * std::fabs(pt.kappa)), 0.0, nominal_speed);
    pt.a_ref = (i == 0 || s < 1e-3) ? 0.0 : (pt.v_ref - prev_v) / std::max(1e-3, s - out.back().s);
    prev_v = pt.v_ref;
    out.push_back(pt);
  }
  return out;
}

std::vector<RefPoint> makeRefPointsFromRawTrail(const std::vector<RawTrailPoint>& raw_trail,
                                                const EgoPose2D& ego,
                                                double nominal_speed) {
  std::vector<RefPoint> out;
  out.reserve(raw_trail.size());
  double prev_v = nominal_speed;
  for (size_t i = 0; i < raw_trail.size(); ++i) {
    RefPoint pt;
    pt.s = raw_trail[i].s;
    pt.t = raw_trail[i].t;
    transformWorldToLocal(ego, raw_trail[i].x, raw_trail[i].y, &pt.x, &pt.y);
    pt.yaw = wrapAngle(raw_trail[i].yaw - ego.yaw);
    pt.kappa = raw_trail[i].kappa;
    pt.v_ref = clampValue(nominal_speed * std::max(0.4, 1.0 - 1.5 * std::fabs(pt.kappa)), 0.0, nominal_speed);
    if (i == 0) {
      pt.a_ref = 0.0;
    } else {
      const double dt = std::max(1e-3, pt.t - out.back().t);
      pt.a_ref = (pt.v_ref - prev_v) / dt;
    }
    prev_v = pt.v_ref;
    out.push_back(pt);
  }
  return out;
}

void recomputeRefGeometry(std::vector<RefPoint>* pts) {
  if (pts == NULL || pts->empty()) {
    return;
  }

  for (size_t i = 0; i < pts->size(); ++i) {
    if (i == 0) {
      (*pts)[i].s = 0.0;
    } else {
      (*pts)[i].s = (*pts)[i - 1].s +
                    pointDistance((*pts)[i - 1].x, (*pts)[i - 1].y, (*pts)[i].x, (*pts)[i].y);
    }
  }

  if (pts->size() < 2) {
    (*pts)[0].yaw = 0.0;
    (*pts)[0].kappa = 0.0;
    return;
  }

  for (size_t i = 0; i < pts->size(); ++i) {
    const size_t i0 = (i == 0) ? i : i - 1;
    const size_t i1 = (i + 1 >= pts->size()) ? i : i + 1;
    if (i0 == i1) {
      continue;
    }
    const double dx = (*pts)[i1].x - (*pts)[i0].x;
    const double dy = (*pts)[i1].y - (*pts)[i0].y;
    if (hypot2(dx, dy) > 1e-4) {
      (*pts)[i].yaw = std::atan2(dy, dx);
    }
  }
  (*pts).front().yaw = (*pts)[std::min<size_t>(1, pts->size() - 1)].yaw;
  (*pts).back().yaw = (*pts)[pts->size() - 2].yaw;

  for (size_t i = 1; i + 1 < pts->size(); ++i) {
    const double ds = std::max(1e-3, (*pts)[i + 1].s - (*pts)[i - 1].s);
    const double dpsi = wrapAngle((*pts)[i + 1].yaw - (*pts)[i - 1].yaw);
    (*pts)[i].kappa = clampValue(dpsi / ds, -0.8, 0.8);
  }
  if (pts->size() >= 2) {
    (*pts).front().kappa = (*pts)[std::min<size_t>(1, pts->size() - 1)].kappa;
    (*pts).back().kappa = (*pts)[pts->size() - 2].kappa;
  }
}

std::vector<RefPoint> resampleRefByArcLength(const std::vector<RefPoint>& in, double ds) {
  if (in.size() < 2 || ds <= 1e-4) {
    return in;
  }

  std::vector<double> arc(in.size(), 0.0);
  for (size_t i = 1; i < in.size(); ++i) {
    arc[i] = arc[i - 1] + pointDistance(in[i - 1].x, in[i - 1].y, in[i].x, in[i].y);
  }
  const double total = arc.back();
  if (total < 1e-4) {
    return {in.back()};
  }

  std::vector<RefPoint> out;
  out.reserve(static_cast<size_t>(std::ceil(total / ds)) + 2);
  size_t seg = 0;
  const size_t samples = std::max<size_t>(2, static_cast<size_t>(std::floor(total / ds)) + 1);
  for (size_t k = 0; k < samples; ++k) {
    const double sk = std::min(total, static_cast<double>(k) * ds);
    while (seg + 1 < arc.size() && arc[seg + 1] < sk) {
      ++seg;
    }
    if (seg + 1 >= in.size()) {
      out.push_back(in.back());
      break;
    }
    const double denom = std::max(1e-6, arc[seg + 1] - arc[seg]);
    const double t = clampValue((sk - arc[seg]) / denom, 0.0, 1.0);
    RefPoint pt;
    pt.s = sk;
    pt.x = in[seg].x + t * (in[seg + 1].x - in[seg].x);
    pt.y = in[seg].y + t * (in[seg + 1].y - in[seg].y);
    pt.yaw = wrapAngle(in[seg].yaw + t * wrapAngle(in[seg + 1].yaw - in[seg].yaw));
    pt.kappa = (1.0 - t) * in[seg].kappa + t * in[seg + 1].kappa;
    pt.v_ref = (1.0 - t) * in[seg].v_ref + t * in[seg + 1].v_ref;
    pt.a_ref = (1.0 - t) * in[seg].a_ref + t * in[seg + 1].a_ref;
    out.push_back(pt);
  }

  if (pointDistance(out.back().x, out.back().y, in.back().x, in.back().y) > 1e-3) {
    out.push_back(in.back());
  } else {
    out.back() = in.back();
  }
  recomputeRefGeometry(&out);
  return out;
}

std::vector<LocalPathPoint> makeLocalPathFromRefPoints(const std::vector<RefPoint>& ref_pts) {
  std::vector<LocalPathPoint> out;
  out.reserve(ref_pts.size());
  for (size_t i = 0; i < ref_pts.size(); ++i) {
    out.push_back(localPathPointFromRef(ref_pts[i]));
  }
  return out;
}

std::vector<LocalPathPoint> resampleLocalByArcLength(const std::vector<LocalPathPoint>& in,
                                                     double ds) {
  if (in.size() < 2 || ds <= 1e-4) {
    return in;
  }

  std::vector<double> arc(in.size(), 0.0);
  for (size_t i = 1; i < in.size(); ++i) {
    arc[i] = arc[i - 1] + pointDistance(in[i - 1].x, in[i - 1].y, in[i].x, in[i].y);
  }
  const double total = arc.back();
  if (total < 1e-4) {
    return {in.back()};
  }

  std::vector<LocalPathPoint> out;
  out.reserve(static_cast<size_t>(std::ceil(total / ds)) + 2);
  size_t seg = 0;
  const size_t samples = std::max<size_t>(2, static_cast<size_t>(std::floor(total / ds)) + 1);

  for (size_t k = 0; k < samples; ++k) {
    const double sk = std::min(total, static_cast<double>(k) * ds);
    while (seg + 1 < arc.size() && arc[seg + 1] < sk) {
      ++seg;
    }
    if (seg + 1 >= in.size()) {
      out.push_back(in.back());
      break;
    }

    const double denom = std::max(1e-6, arc[seg + 1] - arc[seg]);
    const double t = clampValue((sk - arc[seg]) / denom, 0.0, 1.0);
    LocalPathPoint pt;
    pt.x = in[seg].x + t * (in[seg + 1].x - in[seg].x);
    pt.y = in[seg].y + t * (in[seg + 1].y - in[seg].y);
    pt.yaw = wrapAngle(in[seg].yaw + t * wrapAngle(in[seg + 1].yaw - in[seg].yaw));
    out.push_back(pt);
  }

  if (pointDistance(out.back().x, out.back().y, in.back().x, in.back().y) > 1e-3) {
    out.push_back(in.back());
  } else {
    out.back() = in.back();
  }

  recomputeLocalHeading(out);
  return out;
}

std::vector<LocalPathPoint> smoothLocalPath5(const std::vector<LocalPathPoint>& in,
                                             int passes) {
  if (in.size() < 5 || passes <= 0) {
    return in;
  }

  std::vector<LocalPathPoint> cur = in;
  std::vector<LocalPathPoint> nxt = in;
  for (int pass = 0; pass < passes; ++pass) {
    nxt = cur;
    for (size_t i = 2; i + 2 < cur.size(); ++i) {
      nxt[i].x = (-3.0 * cur[i - 2].x + 12.0 * cur[i - 1].x + 17.0 * cur[i].x +
                  12.0 * cur[i + 1].x - 3.0 * cur[i + 2].x) / 35.0;
      nxt[i].y = (-3.0 * cur[i - 2].y + 12.0 * cur[i - 1].y + 17.0 * cur[i].y +
                  12.0 * cur[i + 1].y - 3.0 * cur[i + 2].y) / 35.0;
    }
    cur.swap(nxt);
  }

  recomputeLocalHeading(cur);
  return cur;
}

nav_msgs::Path makePathMsg(const std::vector<LocalPathPoint>& pts,
                           const std::string& frame_id,
                           const ros::Time& stamp) {
  nav_msgs::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = frame_id;
  path.poses.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = pts[i].x;
    pose.pose.position.y = pts[i].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = quaternionFromYaw(pts[i].yaw);
    path.poses.push_back(pose);
  }
  return path;
}

LeadState2D predictLeadState(const LeadState2D& state, const ros::Time& stamp) {
  LeadState2D out = state;
  if (!state.active) {
    out.stamp = stamp;
    return out;
  }

  const double dt = std::max(0.0, (stamp - state.stamp).toSec());
  out.x += state.vx * dt;
  out.y += state.vy * dt;
  out.stamp = stamp;
  if (hypot2(out.vx, out.vy) > 1e-4) {
    out.heading = std::atan2(out.vy, out.vx);
  }
  return out;
}

class EgoMotionEstimator {
 public:
  struct Config {
    double wheelbase = 0.60;
    double min_motion_speed_for_yaw = 0.15;
    double max_abs_yaw_rate = 0.8;
    double max_integration_dt = 0.02;
    double pose_buffer_duration = 5.0;
    double steer_lpf_alpha = 0.25;
    double yaw_sigma_base = 0.01;
    double yaw_sigma_speed_gain = 0.02;
    double yaw_sigma_steer_gain = 0.08;
    bool use_speed_hint = false;
    double speed_hint_weight = 0.20;
    double speed_hint_timeout = 0.30;
    double speed_hint_agreement_gate = 0.40;
    bool use_external_pose = true;
    double external_pose_source_timeout = 0.25;
    double external_pose_position_alpha = 0.85;
    double external_pose_yaw_alpha = 0.85;
    double external_pose_reset_distance = 0.60;
    double external_pose_reset_yaw = 0.70;
    bool use_imu_yaw_rate = true;
    double imu_yaw_rate_lpf_alpha = 0.35;
    double imu_yaw_rate_timeout = 0.15;
    double imu_yaw_rate_deadband = 0.003;
    double imu_yaw_rate_spike_gate = 0.80;
    double imu_yaw_sigma_base = 0.006;
    double imu_yaw_sigma_rate_gain = 0.015;
    double standstill_gyro_gate = 0.004;
  };

  explicit EgoMotionEstimator(const Config& cfg) : cfg_(cfg) {}

  void updateWheelState(const ros::Time& stamp, double wheel_speed_mps, double steer_angle_rad) {
    integrateTo(stamp);
    wheel_speed_mps_ = wheel_speed_mps;
    if (!steer_initialized_) {
      filtered_steer_angle_rad_ = steer_angle_rad;
      steer_initialized_ = true;
    } else {
      filtered_steer_angle_rad_ =
          (1.0 - cfg_.steer_lpf_alpha) * filtered_steer_angle_rad_ + cfg_.steer_lpf_alpha * steer_angle_rad;
    }
    last_motion_stamp_ = stamp;
    updateFusedSpeed(stamp);
  }

  void updateSpeedHint(const ros::Time& stamp, double speed_hint_mps) {
    integrateTo(stamp);
    speed_hint_mps_ = speed_hint_mps;
    speed_hint_stamp_ = stamp;
    updateFusedSpeed(stamp);
  }

  void updateExternalPose(const ros::Time& stamp,
                          double x,
                          double y,
                          double yaw,
                          double speed_mps,
                          bool speed_valid,
                          const std::string& source = "external",
                          int priority = 1) {
    if (!cfg_.use_external_pose || !stamp.isValid() ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      return;
    }
    if (last_external_pose_stamp_.isValid() &&
        priority < external_pose_priority_ &&
        (stamp - last_external_pose_stamp_).toSec() <= cfg_.external_pose_source_timeout) {
      return;
    }
    if (speed_valid && std::isfinite(speed_mps)) {
      speed_hint_mps_ = speed_mps;
      speed_hint_stamp_ = stamp;
      if (cfg_.use_speed_hint) {
        updateFusedSpeed(stamp);
      } else {
        fused_speed_mps_ = speed_mps;
      }
    }

    if (!initialized_) {
      pose_.stamp = stamp;
      pose_.x = x;
      pose_.y = y;
      pose_.yaw = wrapAngle(yaw);
      pose_.v = fused_speed_mps_;
      pose_.delta = filtered_steer_angle_rad_;
      pose_.yaw_rate = 0.0;
      pose_.sigma_yaw = cfg_.yaw_sigma_base;
      buffer_.push_back(pose_);
      initialized_ = true;
      pose_source_ = source;
      external_pose_priority_ = priority;
      last_external_pose_stamp_ = stamp;
      ++external_pose_update_count_;
      return;
    }

    integrateTo(stamp);
    if (stamp < pose_.stamp - ros::Duration(0.05)) {
      return;
    }

    const double pos_err = pointDistance(pose_.x, pose_.y, x, y);
    const double yaw_err = wrapAngle(yaw - pose_.yaw);
    const bool reset =
        pos_err > cfg_.external_pose_reset_distance ||
        std::fabs(yaw_err) > cfg_.external_pose_reset_yaw;
    const double pos_alpha = reset ? 1.0 : clampValue(cfg_.external_pose_position_alpha, 0.0, 1.0);
    const double yaw_alpha = reset ? 1.0 : clampValue(cfg_.external_pose_yaw_alpha, 0.0, 1.0);

    pose_.stamp = stamp;
    pose_.x = (1.0 - pos_alpha) * pose_.x + pos_alpha * x;
    pose_.y = (1.0 - pos_alpha) * pose_.y + pos_alpha * y;
    pose_.yaw = wrapAngle(pose_.yaw + yaw_alpha * yaw_err);
    pose_.v = fused_speed_mps_;
    pose_.delta = filtered_steer_angle_rad_;
    pose_.yaw_rate = currentYawRate();
    pose_.sigma_yaw = reset ? cfg_.yaw_sigma_base : yawSigmaEstimate();
    buffer_.push_back(pose_);
    pose_source_ = source;
    external_pose_priority_ = priority;
    last_external_pose_stamp_ = stamp;
    ++external_pose_update_count_;
    pruneBuffer(stamp);
  }

  void updateImuYawRate(const ros::Time& stamp, double yaw_rate_radps) {
    if (!cfg_.use_imu_yaw_rate || !stamp.isValid() || !std::isfinite(yaw_rate_radps)) {
      return;
    }
    double rate = clampValue(yaw_rate_radps,
                             -std::fabs(cfg_.imu_yaw_rate_spike_gate),
                             std::fabs(cfg_.imu_yaw_rate_spike_gate));
    if (std::fabs(rate) < cfg_.imu_yaw_rate_deadband) {
      rate = 0.0;
    }
    if (!imu_initialized_) {
      filtered_imu_yaw_rate_ = rate;
      imu_initialized_ = true;
    } else {
      filtered_imu_yaw_rate_ =
          (1.0 - cfg_.imu_yaw_rate_lpf_alpha) * filtered_imu_yaw_rate_ +
          cfg_.imu_yaw_rate_lpf_alpha * rate;
    }
    imu_yaw_rate_ = rate;
    imu_stamp_ = stamp;
  }

  void integrateTo(const ros::Time& target_stamp) {
    if (!target_stamp.isValid()) {
      return;
    }

    if (!initialized_) {
      pose_.stamp = target_stamp;
      pose_.v = fused_speed_mps_;
      pose_.delta = filtered_steer_angle_rad_;
      pose_.yaw_rate = 0.0;
      pose_.sigma_yaw = cfg_.yaw_sigma_base;
      buffer_.push_back(pose_);
      initialized_ = true;
      return;
    }

    if (target_stamp <= pose_.stamp) {
      return;
    }

    double remaining = (target_stamp - pose_.stamp).toSec();
    while (remaining > 1e-6) {
      const double dt = std::min(cfg_.max_integration_dt, remaining);
      const double yaw_rate = currentYawRate();
      const double v = fused_speed_mps_;
      const double dtheta = yaw_rate * dt;
      const double mid_yaw = pose_.yaw + 0.5 * dtheta;
      if (std::fabs(v) >= 1e-4) {
        pose_.x += v * dt * std::cos(mid_yaw);
        pose_.y += v * dt * std::sin(mid_yaw);
      }
      pose_.yaw = wrapAngle(pose_.yaw + dtheta);
      pose_.stamp = pose_.stamp + ros::Duration(dt);
      pose_.v = v;
      pose_.delta = filtered_steer_angle_rad_;
      pose_.yaw_rate = yaw_rate;
      pose_.sigma_yaw = yawSigmaEstimate();
      buffer_.push_back(pose_);
      remaining -= dt;
    }

    pruneBuffer(target_stamp);
  }

  bool queryPoseAt(const ros::Time& stamp, EgoPose2D* pose) const {
    if (buffer_.empty() || pose == NULL) {
      return false;
    }
    if (stamp <= buffer_.front().stamp) {
      *pose = buffer_.front();
      return true;
    }
    if (stamp >= buffer_.back().stamp) {
      *pose = buffer_.back();
      return true;
    }

    for (size_t i = 1; i < buffer_.size(); ++i) {
      if (buffer_[i].stamp < stamp) {
        continue;
      }
      const EgoPose2D& a = buffer_[i - 1];
      const EgoPose2D& b = buffer_[i];
      const double denom = std::max(1e-6, (b.stamp - a.stamp).toSec());
      const double t = clampValue((stamp - a.stamp).toSec() / denom, 0.0, 1.0);
      *pose = interpolatePose(a, b, t);
      return true;
    }

    *pose = buffer_.back();
    return true;
  }

  double currentYawRate() const {
    if (usingFreshImuYawRate()) {
      if (std::fabs(fused_speed_mps_) < cfg_.min_motion_speed_for_yaw &&
          std::fabs(filtered_imu_yaw_rate_) < cfg_.standstill_gyro_gate) {
        return 0.0;
      }
      return clampValue(filtered_imu_yaw_rate_, -cfg_.max_abs_yaw_rate, cfg_.max_abs_yaw_rate);
    }
    return bicycleYawRateEstimate();
  }

  double bicycleYawRateEstimate() const {
    if (std::fabs(fused_speed_mps_) < cfg_.min_motion_speed_for_yaw) {
      return 0.0;
    }
    const double yaw_rate =
        fused_speed_mps_ * std::tan(filtered_steer_angle_rad_) / std::max(0.5, cfg_.wheelbase);
    return clampValue(yaw_rate, -cfg_.max_abs_yaw_rate, cfg_.max_abs_yaw_rate);
  }

  double yawSigmaEstimate() const {
    if (usingFreshImuYawRate()) {
      return cfg_.imu_yaw_sigma_base +
             cfg_.imu_yaw_sigma_rate_gain * std::fabs(filtered_imu_yaw_rate_);
    }
    return cfg_.yaw_sigma_base + cfg_.yaw_sigma_speed_gain * std::fabs(currentYawRate()) +
           cfg_.yaw_sigma_steer_gain * std::fabs(filtered_steer_angle_rad_);
  }

  double fusedSpeed() const { return fused_speed_mps_; }
  double wheelSpeed() const { return wheel_speed_mps_; }
  double speedHint() const { return speed_hint_mps_; }
  double steerAngle() const { return filtered_steer_angle_rad_; }
  bool initialized() const { return initialized_; }
  bool usingFreshImuYawRate() const {
    if (!cfg_.use_imu_yaw_rate || !imu_initialized_ || !imu_stamp_.isValid() || !pose_.stamp.isValid()) {
      return false;
    }
    return (pose_.stamp - imu_stamp_).toSec() <= cfg_.imu_yaw_rate_timeout;
  }
  bool externalPoseFresh(const ros::Time& now) const {
    return last_external_pose_stamp_.isValid() &&
           (now - last_external_pose_stamp_).toSec() <= cfg_.external_pose_source_timeout;
  }
  const std::string& poseSource() const {
    if (externalPoseFresh(pose_.stamp)) {
      return pose_source_;
    }
    static const std::string fallback = "imu_wheel";
    return fallback;
  }
  ros::Time lastExternalPoseStamp() const { return last_external_pose_stamp_; }
  int externalPoseUpdateCount() const { return external_pose_update_count_; }
  double imuYawRate() const { return imu_yaw_rate_; }
  double filteredImuYawRate() const { return filtered_imu_yaw_rate_; }

 private:
  void updateFusedSpeed(const ros::Time& now) {
    if (!cfg_.use_speed_hint) {
      fused_speed_mps_ = wheel_speed_mps_;
      return;
    }

    const bool hint_fresh = speed_hint_stamp_.isValid() &&
                            (now - speed_hint_stamp_).toSec() <= cfg_.speed_hint_timeout;
    if (!hint_fresh) {
      fused_speed_mps_ = wheel_speed_mps_;
      return;
    }

    const double diff = std::fabs(wheel_speed_mps_ - speed_hint_mps_);
    const bool agreement = diff <= cfg_.speed_hint_agreement_gate ||
                           (wheel_speed_mps_ * speed_hint_mps_ >= 0.0 &&
                            std::min(std::fabs(wheel_speed_mps_), std::fabs(speed_hint_mps_)) < 0.15);
    if (!agreement) {
      fused_speed_mps_ = wheel_speed_mps_;
      return;
    }

    fused_speed_mps_ =
        (1.0 - cfg_.speed_hint_weight) * wheel_speed_mps_ + cfg_.speed_hint_weight * speed_hint_mps_;
  }

  void pruneBuffer(const ros::Time& now) {
    while (buffer_.size() > 2 && (now - buffer_.front().stamp).toSec() > cfg_.pose_buffer_duration) {
      buffer_.pop_front();
    }
  }

  Config cfg_;
  bool initialized_ = false;
  EgoPose2D pose_;
  std::deque<EgoPose2D> buffer_;
  ros::Time last_motion_stamp_;
  ros::Time speed_hint_stamp_;
  ros::Time imu_stamp_;
  ros::Time last_external_pose_stamp_;
  double wheel_speed_mps_ = 0.0;
  double speed_hint_mps_ = 0.0;
  double fused_speed_mps_ = 0.0;
  bool steer_initialized_ = false;
  double filtered_steer_angle_rad_ = 0.0;
  bool imu_initialized_ = false;
  double imu_yaw_rate_ = 0.0;
  double filtered_imu_yaw_rate_ = 0.0;
  std::string pose_source_ = "dead_reckoning";
  int external_pose_priority_ = 0;
  int external_pose_update_count_ = 0;
};

class LeadObservationProjector {
 public:
  struct Config {
    double target_min_forward_x = 0.30;
    double target_lateral_gate = 4.0;
    double target_lock_dist_gate = 1.20;
    double target_lock_yaw_gate = 1.10;
  };

  explicit LeadObservationProjector(const Config& cfg) : cfg_(cfg) {}

  bool project(const geometry_msgs::PoseArray& msg,
               const EgoMotionEstimator& ego_motion,
               const LeadState2D* track,
               LeadObservation* selected,
               EgoPose2D* ego_at_obs,
               TrackerDebugState* debug) const {
    if (selected == NULL || ego_at_obs == NULL) {
      return false;
    }
    if (debug != NULL) {
      debug->input_pose_count = static_cast<int>(msg.poses.size());
      debug->valid_candidate_count = 0;
      debug->lock_reject_count = 0;
      debug->projector_reason = "init";
    }

    const ros::Time stamp = msg.header.stamp;
    if (!stamp.isValid()) {
      if (debug != NULL) {
        debug->projector_reason = "unstamped_posearray";
      }
      return false;
    }

    if (!ego_motion.queryPoseAt(stamp, ego_at_obs)) {
      if (debug != NULL) {
        debug->projector_reason = "ego_pose_unavailable";
      }
      return false;
    }

    std::vector<LeadObservation> candidates;
    candidates.reserve(msg.poses.size());
    for (size_t i = 0; i < msg.poses.size(); ++i) {
      LeadObservation obs = buildObservation(msg.poses[i], *ego_at_obs, track, stamp);
      if (obs.valid) {
        candidates.push_back(obs);
      }
    }
    if (debug != NULL) {
      debug->valid_candidate_count = static_cast<int>(candidates.size());
    }

    if (candidates.empty()) {
      if (debug != NULL) {
        debug->projector_reason = msg.poses.empty() ? "no_target" : "all_rejected_by_gate";
      }
      return false;
    }

    double best_cost = std::numeric_limits<double>::infinity();
    int best_idx = -1;

    if (track != NULL && track->active) {
      const LeadState2D pred = predictLeadState(*track, stamp);
      for (size_t i = 0; i < candidates.size(); ++i) {
        const double pos_err = pointDistance(candidates[i].world_x, candidates[i].world_y, pred.x, pred.y);
        const double yaw_err = std::fabs(wrapAngle(candidates[i].world_heading - pred.heading));
        if (pos_err > cfg_.target_lock_dist_gate || yaw_err > cfg_.target_lock_yaw_gate) {
          if (debug != NULL) {
            ++debug->lock_reject_count;
          }
          continue;
        }
        const double cost = pos_err + 0.2 * yaw_err + 0.08 * std::fabs(candidates[i].local_y);
        if (cost < best_cost) {
          best_cost = cost;
          best_idx = static_cast<int>(i);
        }
      }
    }

    if (best_idx < 0) {
      for (size_t i = 0; i < candidates.size(); ++i) {
        const double cost = candidates[i].local_x + 0.85 * std::fabs(candidates[i].local_y);
        if (cost < best_cost) {
          best_cost = cost;
          best_idx = static_cast<int>(i);
        }
      }
    }

    if (best_idx < 0) {
      if (debug != NULL) {
        debug->projector_reason = "selection_failed";
      }
      return false;
    }

    *selected = candidates[static_cast<size_t>(best_idx)];
    if (debug != NULL) {
      debug->projector_reason = (track != NULL && track->active) ? "selected_locked_or_fallback"
                                                                 : "selected_bootstrap";
    }
    return true;
  }

 private:
  LeadObservation buildObservation(const geometry_msgs::Pose& pose,
                                   const EgoPose2D& ego_at_obs,
                                   const LeadState2D* track,
                                   const ros::Time& stamp) const {
    LeadObservation out;
    const double center_x = pose.position.x;
    const double center_y = pose.position.y;
    if (center_x < cfg_.target_min_forward_x || std::fabs(center_y) > cfg_.target_lateral_gate) {
      return out;
    }

    double local_heading = yawFromQuaternion(pose.orientation);
    double world_heading = wrapAngle(ego_at_obs.yaw + local_heading);
    if (track != NULL && track->active) {
      const double err0 = std::fabs(wrapAngle(world_heading - track->heading));
      const double err1 = std::fabs(wrapAngle(world_heading + M_PI - track->heading));
      if (err1 < err0) {
        local_heading = wrapAngle(local_heading + M_PI);
        world_heading = wrapAngle(world_heading + M_PI);
      }
    }

    out.valid = true;
    out.stamp = stamp;
    out.local_x = center_x;
    out.local_y = center_y;
    out.local_heading = local_heading;
    out.range = hypot2(center_x, center_y);
    transformLocalToWorld(ego_at_obs, center_x, center_y, &out.world_x, &out.world_y);
    out.world_heading = world_heading;
    return out;
  }

  Config cfg_;
};

class LeadTrailTracker {
 public:
  struct Config {
    double lost_timeout = 0.60;
    double heading_blend_alpha = 0.20;
    double heading_from_velocity_speed_gate = 0.25;
    double innovation_base_gate = 0.10;
    double innovation_range_gain = 0.015;
    double innovation_turn_gain = 0.80;
    double outlier_reject_scale = 3.0;
    double lead_static_speed_gate = 0.08;
    double lead_moving_speed_gate = 0.12;
    double static_residual_gate = 0.12;
    double moving_residual_gate = 0.30;
    int static_vote_on = 12;
    int static_vote_release = 3;
    int moving_vote_on = 2;
    double anchor_alpha = 0.08;
    double history_max_age = 20.0;
    int history_max_points = 300;
    double smoother_position_gain = 0.35;
    double smoother_cross_gain = 0.18;
    double smoother_temporal_alpha = 0.22;
    double smoother_heading_alpha = 0.20;
    double smoother_curvature_alpha = 0.18;
    double measurement_heading_alpha = 0.12;
    double dynamics_position_alpha = 0.12;
    double dynamics_heading_alpha = 0.16;
    double acceleration_regularization_alpha = 0.16;
    double curvature_prediction_alpha = 0.18;
    double dkappa_regularization_alpha = 0.20;
    double ego_compensated_static_gate = 0.10;
    double ego_compensated_moving_gate = 0.22;
    double static_measurement_anchor_alpha = 0.18;
    double robust_cauchy_scale = 2.5;
    int smoother_iterations = 5;
    double fixed_lag_window = 3.0;
    bool use_bbox_heading_hint = false;
    double max_ego_yaw_sigma_for_confirm = 0.11;
    double fixed_frame_heading_min_distance = 0.12;
    double fixed_frame_heading_hold_distance = 0.08;
    double fixed_frame_max_yaw_jump = 0.70;
    double fixed_frame_kink_gate = 0.85;
    double fixed_frame_curvature_gate = 1.50;
    double fixed_frame_reverse_gate = -0.03;
    double fixed_frame_duplicate_distance = 0.045;
    double motion_tail_sample_min_distance = 0.015;
    double motion_tail_sample_max_interval = 0.15;
    double motion_tail_sample_min_along_progress = 0.006;
    double imu_wheel_append_yaw_rate_gate = 0.06;
    double imu_wheel_append_speed_gate = 0.12;
    double fixed_lag_observation_weight = 8.0;
    double fixed_lag_smooth_weight = 2.5;
    double fixed_lag_curvature_weight = 0.8;
    double measurement_along_sigma = 0.08;
    double measurement_cross_sigma = 0.12;
    double measurement_cross_range_gain = 0.020;
    double measurement_cross_yaw_uncertainty_gain = 1.80;
    double temporal_min_progress = 0.02;
    double static_preserve_trail_span = 0.80;
    double trail_append_min_distance = 0.12;
    double trail_anchor_update_alpha = 0.10;
    int tail_confirm_min_points = 3;
    double tail_confirm_min_distance = 0.035;
    double tail_confirm_min_age = 0.08;
    double confirm_dynamic_score_margin = 0.12;
    double confirm_static_residual_gate = 0.14;
    double confirm_min_progress = 0.08;
    double confirm_accum_distance = 0.04;
    double confirm_accum_timeout = 0.80;
    double ego_static_speed_gate = 0.05;
    double static_release_score_margin = 0.06;
    int raw_tail_keep_points = 6;
    double dual_hypothesis_score_margin = 0.02;
    int tail_buffer_max_points = 12;
    double tail_buffer_max_age = 0.60;
    int static_tail_keep_points = 3;
    bool publish_tail_suffix = false;
    double tail_confirm_min_time = 0.08;
    double lead_tail_max_speed = 1.20;
    double tail_cross_gate = 0.30;
    double tail_reverse_gate = -0.03;
    double static_anchor_res_gate = 0.04;
    double static_anchor_progress_gate = 0.04;
    double static_anchor_speed_gate = 0.035;
    double static_anchor_append_distance_gate = 0.08;
    double heading_recompute_min_distance = 0.06;
    bool provisional_tail_enabled = true;
    int provisional_tail_min_points = 3;
    double provisional_tail_min_span = 0.18;
    double provisional_tail_short_min_span = 0.08;
    double provisional_tail_min_time = 0.08;
    double provisional_tail_short_min_time = 0.10;
    double provisional_tail_max_age = 1.50;
    int provisional_tail_standalone_until_confirmed_points = 8;
    double motion_tail_force_publish_stall = 0.15;
    double motion_static_suppress_speed = 0.035;
    double motion_static_suppress_progress = 0.015;
    double motion_static_suppress_residual = 0.050;
    int suffix_estimator_min_points = 2;
    double suffix_estimator_min_span = 0.06;
    double suffix_estimator_min_time = 0.05;
    double suffix_estimator_bootstrap_min_span = 0.12;
    double suffix_estimator_confirmed_min_span = 0.04;
    double suffix_estimator_tail_min_time = 0.20;
    double suffix_estimator_max_extension_span = 0.35;
    double suffix_smooth_resample_ds = 0.08;
    int suffix_smooth_passes = 2;
    double suffix_smooth_alpha = 0.30;
    double suffix_smooth_huber_delta = 0.12;
    double suffix_smooth_max_adjust = 0.10;
    double suffix_smooth_wiggle_gate = 0.040;
    int suffix_smooth_sign_flip_gate = 2;
    double suffix_smooth_alpha_wiggle = 0.55;
    double suffix_smooth_max_adjust_wiggle = 0.14;
    double suffix_smooth_curve_preserve_kappa = 1.35;
    int suffix_smooth_extra_s_wave_passes = 0;
    int suffix_smooth_five_point_passes = 0;
    double suffix_smooth_s_wave_window = 0.60;
    double suffix_smooth_centerline_alpha = 0.25;
    double suffix_trackable_kappa_limit = 0.77;
    double suffix_curve_rate_refit_gate = 15.0;
    double suffix_curve_refit_window = 0.50;
    double suffix_curve_refit_alpha = 0.30;
    double reference_health_wiggle_gate = 0.060;
    double reference_health_curvature_gate = 3.0;
    double reference_health_wiggle_penalty = 0.35;
    double reference_health_flip_penalty = 0.16;
    double reference_health_curvature_penalty = 0.12;
    double reference_health_s_wave_cap = 0.62;
    bool tail_soft_geometry_enabled = true;
    bool ego_turn_weight_enabled = true;
    double ego_turn_yaw_rate_gate = 0.035;
    double ego_turn_yaw_rate_full = 0.12;
    double ego_turn_steer_gate = 0.08;
    double ego_turn_steer_full = 0.28;
    double ego_turn_progress_gate = 0.08;
    double ego_turn_lateral_velocity_gate = 0.08;
    double ego_turn_lateral_velocity_full = 0.35;
    double ego_turn_min_tail_weight = 0.30;
    double ego_turn_min_along_weight = 0.55;
    double ego_turn_min_cross_weight = 0.08;
    double ego_turn_cross_hold_score = 0.35;
    double ego_turn_cross_hold_progress = 0.10;
    double lateral_innovation_gate_m = 0.060;
    double lateral_innovation_full_m = 0.20;
    double ego_turn_cross_sigma_scale = 2.50;
    double ego_turn_heading_alpha_scale = 0.15;
    double ego_turn_bbox_heading_disable_score = 0.25;
    double ego_turn_cross_hold_residual_scale = 0.10;
    double lateral_velocity_filter_tau = 0.35;
    double lateral_velocity_filtered_cap = 2.0;
    double lateral_low_weight_velocity_gate = 0.25;
    double lateral_outlier_velocity_gate = 0.55;
    double lateral_low_weight_min = 0.35;
  };

  explicit LeadTrailTracker(const Config& cfg) : cfg_(cfg) {}

  struct CenterlineProjection {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double s = 0.0;
    double distance = 0.0;
  };

  void markLostIfExpired(const ros::Time& stamp) {
    if (track_.active && (stamp - track_.stamp).toSec() > cfg_.lost_timeout) {
      clearStateUnlocked();
    }
  }

  void ingestObservation(const LeadObservation& obs,
                         const EgoPose2D& ego_at_obs,
                         double ego_yaw_rate,
                         double ego_yaw_sigma,
                         double steer_angle_rad,
                         TrackerDebugState* debug) {
    if (!obs.valid) {
      if (debug != NULL) {
        debug->history_reason = "invalid_observation";
        debug->history_size = exportHistoryUnlocked().size();
      }
      return;
    }

    const double sigma =
        cfg_.innovation_base_gate + cfg_.innovation_range_gain * obs.range +
        cfg_.innovation_turn_gain * std::fabs(ego_yaw_rate) * obs.range;
    if (track_.active) {
      const LeadState2D pred = predictLeadState(track_, obs.stamp);
      const double residual =
          pointDistance(obs.world_x, obs.world_y, pred.x, pred.y);
      const double metric = residual / std::max(1e-3, sigma);
      last_innovation_norm_ = residual;
      last_innovation_metric_ = metric;
      if (metric > cfg_.outlier_reject_scale && (obs.stamp - track_.stamp).toSec() <= cfg_.lost_timeout) {
        if (debug != NULL) {
          debug->history_reason = "outlier_rejected";
          debug->history_size = exportHistoryUnlocked().size();
          debug->outlier_gate_reason = "innovation_metric";
        }
        last_outlier_gate_reason_ = "innovation_metric";
        return;
      }
      last_outlier_gate_reason_ = "none";
    } else {
      last_innovation_norm_ = 0.0;
      last_innovation_metric_ = 0.0;
      last_outlier_gate_reason_ = "none";
    }

    LeadNode node;
    node.stamp = obs.stamp;
    node.ego = ego_at_obs;
    node.x = obs.world_x;
    node.y = obs.world_y;
    node.yaw = obs.world_heading;
    double lateral_velocity = 0.0;
    double lateral_innovation_score = 0.0;
    double contamination_score = 0.0;
    double tail_weight = 1.0;
    double tail_along_weight = 1.0;
    double tail_cross_weight = 1.0;
    double lateral_innovation_frenet = 0.0;
    bool cross_hold_active = false;
    const bool has_recent_prev =
        !nodes_.empty() &&
        obs.stamp.isValid() &&
        nodes_.back().stamp.isValid() &&
        (obs.stamp - nodes_.back().stamp).toSec() > 1e-4 &&
        (obs.stamp - nodes_.back().stamp).toSec() <= cfg_.lost_timeout;
    if (!nodes_.empty()) {
      const LeadNode& prev = nodes_.back();
      const double dx = node.x - prev.x;
      const double dy = node.y - prev.y;
      if (hypot2(dx, dy) >= cfg_.fixed_frame_heading_min_distance) {
        node.yaw = std::atan2(dy, dx);
      } else {
        node.yaw = prev.yaw;
      }
      if (has_recent_prev && cfg_.ego_turn_weight_enabled) {
        const double dt = std::max(1e-3, (obs.stamp - prev.stamp).toSec());
        const LeadNode pred = predictNodeForwardUnlocked(prev, dt);
        // Use the causal fixed-frame prediction as the Frenet baseline. Feeding
        // the prior published centerline back here can preserve its old bends.
        const double base_x = pred.x;
        const double base_y = pred.y;
        const double base_yaw = pred.yaw;
        const double tx = std::cos(base_yaw);
        const double ty = std::sin(base_yaw);
        const double innov_x = node.x - base_x;
        const double innov_y = node.y - base_y;
        const double cross_innov = -innov_x * ty + innov_y * tx;
        const double along_step = dx * tx + dy * ty;
        lateral_innovation_frenet = cross_innov;
        const double residual_alpha =
            clampValue(dt / (std::max(1e-3, cfg_.lateral_velocity_filter_tau) + dt), 0.0, 1.0);
        if (!lateral_residual_filter_initialized_ ||
            !lateral_residual_filter_stamp_.isValid() ||
            (obs.stamp - lateral_residual_filter_stamp_).toSec() > 1.0) {
          lateral_residual_filtered_ = cross_innov;
          lateral_velocity_filtered_ = 0.0;
          lateral_residual_filter_initialized_ = true;
        } else {
          const double raw_lateral_velocity =
              std::fabs(cross_innov - lateral_residual_filtered_) / dt;
          lateral_velocity_filtered_ =
              (1.0 - residual_alpha) * lateral_velocity_filtered_ +
              residual_alpha * std::min(std::max(0.0, cfg_.lateral_velocity_filtered_cap),
                                        raw_lateral_velocity);
          lateral_residual_filtered_ =
              (1.0 - residual_alpha) * lateral_residual_filtered_ +
              residual_alpha * cross_innov;
        }
        lateral_residual_filter_stamp_ = obs.stamp;
        lateral_velocity = lateral_velocity_filtered_;
        const double yaw_score =
            clampValue((std::fabs(ego_yaw_rate) - cfg_.ego_turn_yaw_rate_gate) /
                           std::max(1e-3, cfg_.ego_turn_yaw_rate_full - cfg_.ego_turn_yaw_rate_gate),
                       0.0,
                       1.0);
        const double steer_score =
            clampValue((std::fabs(steer_angle_rad) - cfg_.ego_turn_steer_gate) /
                           std::max(1e-3, cfg_.ego_turn_steer_full - cfg_.ego_turn_steer_gate),
	                       0.0,
	                       1.0);
	        const double ego_turn_score =
	            clampValue(0.55 * yaw_score + 0.45 * steer_score, 0.0, 1.0);
	        const double progress_support =
	            clampValue(std::max(0.0, along_step) / 0.04, 0.0, 1.0);
	        const double lateral_innovation_abs = std::fabs(cross_innov);
        const double lateral_innovation_mag_score =
            clampValue((lateral_innovation_abs - cfg_.lateral_innovation_gate_m) /
                           std::max(1e-3,
                                    cfg_.lateral_innovation_full_m -
                                        cfg_.lateral_innovation_gate_m),
                       0.0,
                       1.0);
        const double lateral_velocity_score_raw =
            clampValue((lateral_velocity - cfg_.ego_turn_lateral_velocity_gate) /
                           std::max(1e-3,
                                    cfg_.ego_turn_lateral_velocity_full -
                                        cfg_.ego_turn_lateral_velocity_gate),
                       0.0,
                       1.0);
	        const double lateral_velocity_score = lateral_velocity_score_raw;
	        const double lateral_score =
	            std::max(lateral_innovation_mag_score, lateral_velocity_score);
	        contamination_score =
	            ego_turn_score * lateral_score * (1.0 - 0.60 * progress_support);
	        lateral_innovation_score = lateral_score;
	        tail_weight = 1.0;
	        tail_along_weight =
	            clampValue(1.0 - 0.15 * contamination_score, 0.85, 1.0);
	        tail_cross_weight =
	            clampValue(1.0 - 0.85 * contamination_score -
	                           0.25 * lateral_innovation_mag_score,
	                       0.15,
	                       1.0);
	        tail_cross_weight = std::min(tail_cross_weight, tail_along_weight);
	        cross_hold_active = false;
      }
    } else if (track_.active) {
      node.yaw = track_.heading;
    }
    node.tail_weight = tail_weight;
    node.tail_along_weight = tail_along_weight;
    node.tail_cross_weight = tail_cross_weight;
    node.cross_hold_active = cross_hold_active;
    node.ego_turn_contamination_score = contamination_score;
    node.fixed_frame_lateral_velocity = lateral_velocity;
    node.fixed_frame_lateral_velocity_filtered = lateral_velocity;
    node.lateral_innovation_gate = lateral_innovation_score;
    node.lateral_innovation_frenet = lateral_innovation_frenet;
    node.cross_residual_filtered = lateral_residual_filtered_;
    node.heading_decoupled_by_ego_turn =
        contamination_score >= cfg_.ego_turn_bbox_heading_disable_score ||
        lateral_innovation_score > 0.50 ||
        tail_cross_weight < 0.50;
    node.bbox_heading_used_for_history =
        cfg_.use_bbox_heading_hint && !node.heading_decoupled_by_ego_turn;
    node.heading_hint = node.bbox_heading_used_for_history ? obs.world_heading : node.yaw;
    node.has_measurement = true;
    node.z_rel_x = obs.local_x;
    node.z_rel_y = obs.local_y;
    node.range = obs.range;
    node.w_cross =
        cfg_.measurement_cross_sigma + cfg_.measurement_cross_range_gain * obs.range +
        cfg_.measurement_cross_yaw_uncertainty_gain * obs.range * ego_yaw_sigma +
        0.10 * std::fabs(steer_angle_rad);
    if (cfg_.ego_turn_weight_enabled) {
      node.w_cross *=
          1.0 + cfg_.ego_turn_cross_sigma_scale * contamination_score +
          1.25 * lateral_innovation_score;
    }
    node.w_along = cfg_.measurement_along_sigma;

    if (nodes_.empty() || (obs.stamp - nodes_.back().stamp).toSec() > cfg_.lost_timeout) {
      clearStateUnlocked();
      static_anchor_x_ = obs.world_x;
      static_anchor_y_ = obs.world_y;
    }

    updateMeasurementHypothesisUnlocked(&node);

    nodes_.push_back(node);

    pruneWindowUnlocked(obs.stamp);
    smoothWindowUnlocked();
    updateTrackFromWindowUnlocked(debug);
  }

  void pruneHistory(const ros::Time& now) {
    pruneWindowUnlocked(now);
  }

  std::vector<HistoryPoint2D> worldHistoryForPublish(const ros::Time& stamp) const {
    (void)stamp;
    return exportHistoryUnlocked();
  }

  std::vector<HistoryPoint2D> referenceHistoryForPublish(const ros::Time& stamp,
                                                         TrackerDebugState* debug) const {
    (void)stamp;
    return exportReferenceHistoryWithTailUnlocked(debug);
  }

  const LeadState2D& track() const { return track_; }
  bool hasActiveTrack() const { return track_.active; }
  bool movingFlag() const { return moving_flag_; }
  double innovationNorm() const { return last_innovation_norm_; }
  double innovationMetric() const { return last_innovation_metric_; }

 private:
  double estimateWindowProgressUnlocked() const {
    if (nodes_.size() < 2) {
      return 0.0;
    }
    double progress = 0.0;
    for (size_t i = 1; i < nodes_.size(); ++i) {
      const double tx = std::cos(nodes_[i - 1].yaw);
      const double ty = std::sin(nodes_[i - 1].yaw);
      const double dx = nodes_[i].x - nodes_[i - 1].x;
      const double dy = nodes_[i].y - nodes_[i - 1].y;
      progress += std::max(0.0, dx * tx + dy * ty);
    }
    return progress;
  }

  double meanFixedFrameSpeedUnlocked() const {
    if (nodes_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < nodes_.size(); ++i) {
      sum += nodes_[i].fixed_frame_speed;
    }
    return sum / static_cast<double>(nodes_.size());
  }

  double regressionSpeedFromNodesUnlocked(const std::deque<LeadNode>& seq) const {
    if (seq.size() < 2) {
      return 0.0;
    }

    const LeadNode& first = seq.front();
    const LeadNode& last = seq.back();
    const double ref_dx = last.x - first.x;
    const double ref_dy = last.y - first.y;
    double ref_yaw = 0.0;
    if (hypot2(ref_dx, ref_dy) > 1e-3) {
      ref_yaw = std::atan2(ref_dy, ref_dx);
    } else {
      double sx = 0.0;
      double sy = 0.0;
      for (size_t i = 0; i < seq.size(); ++i) {
        sx += std::cos(seq[i].yaw);
        sy += std::sin(seq[i].yaw);
      }
      ref_yaw = std::atan2(sy, sx);
    }

    const double tx = std::cos(ref_yaw);
    const double ty = std::sin(ref_yaw);
    const double t0 = first.stamp.toSec();
    const double x0 = first.x;
    const double y0 = first.y;

    double wsum = 0.0;
    double mean_t = 0.0;
    double mean_s = 0.0;
    for (size_t i = 0; i < seq.size(); ++i) {
      const double ti = seq[i].stamp.toSec() - t0;
      const double si = (seq[i].x - x0) * tx + (seq[i].y - y0) * ty;
      const double wi = std::max(1e-3, seq[i].robust_weight);
      wsum += wi;
      mean_t += wi * ti;
      mean_s += wi * si;
    }
    if (wsum < 1e-6) {
      return 0.0;
    }
    mean_t /= wsum;
    mean_s /= wsum;

    double cov_ts = 0.0;
    double var_t = 0.0;
    for (size_t i = 0; i < seq.size(); ++i) {
      const double ti = seq[i].stamp.toSec() - t0;
      const double si = (seq[i].x - x0) * tx + (seq[i].y - y0) * ty;
      const double wi = std::max(1e-3, seq[i].robust_weight);
      cov_ts += wi * (ti - mean_t) * (si - mean_s);
      var_t += wi * (ti - mean_t) * (ti - mean_t);
    }
    if (var_t < 1e-6) {
      return 0.0;
    }
    return std::fabs(cov_ts / var_t);
  }

  double meanStaticHypothesisResidualUnlocked() const {
    if (nodes_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    double wsum = 0.0;
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const double w = std::max(1e-3, nodes_[i].robust_weight);
      sum += w * nodes_[i].static_hypothesis_residual;
      wsum += w;
    }
    return sum / std::max(1e-3, wsum);
  }

  double meanDynamicHypothesisResidualUnlocked() const {
    if (nodes_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    double wsum = 0.0;
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const double w = std::max(1e-3, nodes_[i].robust_weight);
      sum += w * nodes_[i].short_window_prediction_residual;
      wsum += w;
    }
    return sum / std::max(1e-3, wsum);
  }

  double meanEgoSpeedUnlocked() const {
    if (nodes_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < nodes_.size(); ++i) {
      sum += std::fabs(nodes_[i].ego.v);
    }
    return sum / static_cast<double>(nodes_.size());
  }

  double dynamicTrailProgressUnlocked() const {
    if (tail_buffer_world_.size() < 2) {
      return estimateWindowProgressUnlocked();
    }
    double progress = 0.0;
    for (size_t i = 1; i < tail_buffer_world_.size(); ++i) {
      const double tx = std::cos(tail_buffer_world_[i - 1].heading);
      const double ty = std::sin(tail_buffer_world_[i - 1].heading);
      const double dx = tail_buffer_world_[i].x - tail_buffer_world_[i - 1].x;
      const double dy = tail_buffer_world_[i].y - tail_buffer_world_[i - 1].y;
      progress += std::max(0.0, dx * tx + dy * ty);
    }
    return progress;
  }

  double staticHypothesisScoreUnlocked() const {
    if (nodes_.empty()) {
      return 1.0;
    }
    const double residual_term =
        meanStaticHypothesisResidualUnlocked() / std::max(1e-3, cfg_.ego_compensated_static_gate);
    const double speed_term =
        meanFixedFrameSpeedUnlocked() / std::max(1e-3, cfg_.ego_compensated_moving_gate);
    const double progress_term =
        dynamicTrailProgressUnlocked() /
        std::max(0.05, static_cast<double>(std::max<size_t>(2, nodes_.size() - 1)) * cfg_.temporal_min_progress);
    const double dynamic_advantage =
        meanDynamicHypothesisResidualUnlocked() - meanStaticHypothesisResidualUnlocked();
    const double advantage_term = clampValue(dynamic_advantage / std::max(1e-3, cfg_.ego_compensated_static_gate),
                                             -1.0,
                                             2.0);
    return 0.55 * residual_term + 0.20 * speed_term + 0.15 * progress_term - 0.10 * advantage_term;
  }

  double dynamicHypothesisScoreUnlocked() const {
    if (nodes_.empty()) {
      return 1.0;
    }
    const double residual_term =
        meanDynamicHypothesisResidualUnlocked() / std::max(1e-3, cfg_.ego_compensated_moving_gate);
    const double speed_ratio =
        meanFixedFrameSpeedUnlocked() / std::max(1e-3, cfg_.ego_compensated_static_gate);
    const double progress =
        dynamicTrailProgressUnlocked() /
        std::max(0.05, static_cast<double>(std::max<size_t>(2, nodes_.size() - 1)) * cfg_.temporal_min_progress);
    const double speed_penalty =
        clampValue((cfg_.lead_moving_speed_gate - meanFixedFrameSpeedUnlocked()) /
                       std::max(1e-3, cfg_.lead_moving_speed_gate),
                   0.0,
                   1.5);
    const double progress_penalty =
        clampValue((cfg_.confirm_min_progress - dynamicTrailProgressUnlocked()) /
                       std::max(1e-3, cfg_.confirm_min_progress),
                   0.0,
                   1.5);
    const double advantage_penalty =
        clampValue((meanDynamicHypothesisResidualUnlocked() - meanStaticHypothesisResidualUnlocked()) /
                       std::max(1e-3, cfg_.ego_compensated_static_gate),
                   0.0,
                   2.0);
    (void)speed_ratio;
    (void)progress;
    return 0.60 * residual_term + 0.15 * speed_penalty + 0.15 * progress_penalty + 0.10 * advantage_penalty;
  }

  bool dynamicHypothesisWinsUnlocked() const {
    return dynamicHypothesisScoreUnlocked() + cfg_.dual_hypothesis_score_margin <
           staticHypothesisScoreUnlocked();
  }

  bool confirmDynamicHypothesisWinsUnlocked() const {
    return dynamicHypothesisScoreUnlocked() + cfg_.confirm_dynamic_score_margin <
           staticHypothesisScoreUnlocked();
  }

  bool egoMotionConditionedDynamicUnlocked() const {
    const double hs = staticHypothesisScoreUnlocked();
    const double hd = dynamicHypothesisScoreUnlocked();
    const double ego_speed = meanEgoSpeedUnlocked();
    const double progress = dynamicTrailProgressUnlocked();
    const double ff_speed = meanFixedFrameSpeedUnlocked();
    const bool score_wins = hd + 0.5 * cfg_.confirm_dynamic_score_margin < hs;
    const bool accum_support =
        unconfirmed_progress_accum_ >= 0.5 * cfg_.confirm_accum_distance ||
        (unconfirmed_time_accum_ >= 0.4 * cfg_.confirm_accum_timeout &&
         unconfirmed_progress_accum_ >= 2.0 * cfg_.measurement_cross_sigma);
    const bool progress_support =
        progress >= std::max(0.02, 0.5 * cfg_.confirm_min_progress) ||
        ff_speed >= 0.5 * cfg_.lead_moving_speed_gate;
    const bool ego_support = ego_speed >= 0.6 * cfg_.ego_static_speed_gate;
    return score_wins && (accum_support || progress_support || ego_support);
  }

  double residualAgainstWorldPointUnlocked(const EgoState& ego,
                                           double world_x,
                                           double world_y,
                                           double meas_x,
                                           double meas_y) const {
    double expected_local_x = 0.0;
    double expected_local_y = 0.0;
    transformWorldToLocal(ego, world_x, world_y, &expected_local_x, &expected_local_y);
    return pointDistance(meas_x, meas_y, expected_local_x, expected_local_y);
  }

  double estimateStaticConfidenceUnlocked() const {
    if (nodes_.empty()) {
      return 1.0;
    }
    double residual_cross_sum = 0.0;
    double residual_along_sum = 0.0;
    double weight_sum = 0.0;
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const double w = std::max(1e-3, nodes_[i].robust_weight);
      residual_cross_sum += w * std::fabs(nodes_[i].residual_cross);
      residual_along_sum += w * std::fabs(nodes_[i].residual_along);
      weight_sum += w;
    }
    const double cross_mean = residual_cross_sum / std::max(1e-3, weight_sum);
    const double along_mean = residual_along_sum / std::max(1e-3, weight_sum);
    const double progress = estimateWindowProgressUnlocked();
    const double progress_ratio = progress / std::max(0.3, static_cast<double>(nodes_.size()) * cfg_.temporal_min_progress);
    const double cross_ratio = cross_mean / std::max(1e-3, cfg_.measurement_cross_sigma);
    const double along_ratio = along_mean / std::max(1e-3, cfg_.measurement_along_sigma);
    const double confidence = 1.0 - clampValue(0.55 * progress_ratio + 0.25 * along_ratio + 0.20 * cross_ratio,
                                               0.0,
                                               1.0);
    return clampValue(confidence, 0.0, 1.0);
  }

  void applyStaticAnchorFactorUnlocked() {
    if (nodes_.empty()) {
      return;
    }
    const double static_confidence = estimateStaticConfidenceUnlocked();
    const bool enable_anchor = track_.static_mode || static_votes_ > 0 || static_confidence > 0.65;
    if (!enable_anchor) {
      return;
    }

    const double measurement_static =
        1.0 - clampValue(meanStaticHypothesisResidualUnlocked() /
                             std::max(1e-3, cfg_.ego_compensated_static_gate),
                         0.0,
                         1.0);
    const double anchor_gain =
        cfg_.anchor_alpha * (0.25 + 0.45 * static_confidence + 0.30 * measurement_static);
    const double preserve_span = std::max(0.05, cfg_.static_preserve_trail_span);
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const double age_weight =
          (i + 1 == nodes_.size()) ? 1.0 : clampValue(1.0 - (nodes_.back().stamp - nodes_[i].stamp).toSec() / 2.0,
                                                      0.15,
                                                      1.0);
      const double dx = static_anchor_x_ - nodes_[i].x;
      const double dy = static_anchor_y_ - nodes_[i].y;
      const double dist = hypot2(dx, dy);
      if (dist > preserve_span && i + 1 < nodes_.size()) {
        continue;
      }
      const double gain = anchor_gain * age_weight * nodes_[i].robust_weight;
      nodes_[i].x += gain * dx;
      nodes_[i].y += gain * dy;
    }
  }

  void updateMeasurementHypothesisUnlocked(LeadNode* node) const {
    if (node == NULL) {
      return;
    }
    if (nodes_.empty()) {
      node->fixed_frame_speed = 0.0;
      node->static_hypothesis_residual = 0.0;
      return;
    }

    const LeadNode& prev = nodes_.back();
    std::deque<LeadNode> window = nodes_;
    window.push_back(*node);
    const size_t max_window = 20;
    while (window.size() > max_window) {
      window.pop_front();
    }
    node->fixed_frame_speed = regressionSpeedFromNodesUnlocked(window);
    const double prev_residual =
        residualAgainstWorldPointUnlocked(node->ego, prev.x, prev.y, node->z_rel_x, node->z_rel_y);

    double trail_tail_residual = prev_residual;
    if (!confirmed_trail_world_.empty()) {
      const HistoryPoint2D& tail = confirmed_trail_world_.back();
      trail_tail_residual =
          residualAgainstWorldPointUnlocked(node->ego, tail.x, tail.y, node->z_rel_x, node->z_rel_y);
    }

    double window_pred_residual = prev_residual;
    if (nodes_.size() >= 2) {
      const double dt_pred = std::max(1e-3, (node->stamp - nodes_.back().stamp).toSec());
      const LeadNode predicted = predictNodeForwardUnlocked(nodes_.back(), dt_pred);
      window_pred_residual =
          residualAgainstWorldPointUnlocked(node->ego, predicted.x, predicted.y, node->z_rel_x, node->z_rel_y);
    }

    node->static_hypothesis_residual = std::min(prev_residual, trail_tail_residual);
    node->short_window_prediction_residual = window_pred_residual;
  }

  void pullWindowTowardStaticHypothesisUnlocked() {
    if (nodes_.size() < 2) {
      return;
    }
    for (size_t i = 1; i < nodes_.size(); ++i) {
      if (nodes_[i].static_hypothesis_residual > cfg_.ego_compensated_static_gate) {
        continue;
      }
      const double confidence =
          1.0 - clampValue(nodes_[i].static_hypothesis_residual /
                               std::max(1e-3, cfg_.ego_compensated_static_gate),
                           0.0,
                           1.0);
      const double gain =
          cfg_.static_measurement_anchor_alpha * confidence * nodes_[i].robust_weight;
      nodes_[i].x = (1.0 - gain) * nodes_[i].x + gain * nodes_[i - 1].x;
      nodes_[i].y = (1.0 - gain) * nodes_[i].y + gain * nodes_[i - 1].y;
    }
  }

  LeadNode predictNodeForwardUnlocked(const LeadNode& prev, double dt) const {
    LeadNode pred = prev;
    const double v = prev.v + prev.a * dt;
    const double kappa = prev.kappa + prev.dkappa * dt;
    const double yaw_rate = prev.v * prev.kappa;
    const double mid_yaw = prev.yaw + 0.5 * yaw_rate * dt;
    pred.x = prev.x + prev.v * std::cos(mid_yaw) * dt;
    pred.y = prev.y + prev.v * std::sin(mid_yaw) * dt;
    pred.yaw = wrapAngle(prev.yaw + yaw_rate * dt);
    pred.v = v;
    pred.kappa = clampValue(kappa, -0.8, 0.8);
    return pred;
  }

  LeadNode predictNodeBackwardUnlocked(const LeadNode& next, double dt) const {
    LeadNode pred = next;
    const double kappa = next.kappa - next.dkappa * dt;
    const double v = next.v - next.a * dt;
    const double yaw_rate = v * kappa;
    const double mid_yaw = next.yaw - 0.5 * yaw_rate * dt;
    pred.x = next.x - v * std::cos(mid_yaw) * dt;
    pred.y = next.y - v * std::sin(mid_yaw) * dt;
    pred.yaw = wrapAngle(next.yaw - yaw_rate * dt);
    pred.v = v;
    pred.kappa = clampValue(kappa, -0.8, 0.8);
    return pred;
  }

  void applyDynamicsConsistencyUnlocked() {
    if (nodes_.size() < 3) {
      return;
    }

    std::deque<LeadNode> next_nodes = nodes_;
    for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
      const double dt_prev = std::max(1e-3, (nodes_[i].stamp - nodes_[i - 1].stamp).toSec());
      const double dt_next = std::max(1e-3, (nodes_[i + 1].stamp - nodes_[i].stamp).toSec());
      const LeadNode pred_prev = predictNodeForwardUnlocked(nodes_[i - 1], dt_prev);
      const LeadNode pred_next = predictNodeBackwardUnlocked(nodes_[i + 1], dt_next);
      const double w_prev = std::max(1e-3, nodes_[i - 1].robust_weight);
      const double w_next = std::max(1e-3, nodes_[i + 1].robust_weight);
      const double norm = 1.0 / (w_prev + w_next);

      const double dyn_x = (w_prev * pred_prev.x + w_next * pred_next.x) * norm;
      const double dyn_y = (w_prev * pred_prev.y + w_next * pred_next.y) * norm;
      const double dyn_v = (w_prev * pred_prev.v + w_next * pred_next.v) * norm;
      const double dyn_kappa = clampValue((w_prev * pred_prev.kappa + w_next * pred_next.kappa) * norm, -0.8, 0.8);

      const double yaw_prev_err = wrapAngle(pred_prev.yaw - nodes_[i].yaw);
      const double yaw_next_err = wrapAngle(pred_next.yaw - nodes_[i].yaw);
      const double yaw_dyn = wrapAngle(nodes_[i].yaw + (w_prev * yaw_prev_err + w_next * yaw_next_err) * norm);

      next_nodes[i].x =
          (1.0 - cfg_.dynamics_position_alpha) * nodes_[i].x + cfg_.dynamics_position_alpha * dyn_x;
      next_nodes[i].y =
          (1.0 - cfg_.dynamics_position_alpha) * nodes_[i].y + cfg_.dynamics_position_alpha * dyn_y;
      next_nodes[i].yaw = wrapAngle((1.0 - cfg_.dynamics_heading_alpha) * nodes_[i].yaw +
                                    cfg_.dynamics_heading_alpha * yaw_dyn);
      next_nodes[i].v = (1.0 - cfg_.dynamics_position_alpha) * nodes_[i].v +
                        cfg_.dynamics_position_alpha * dyn_v;
      next_nodes[i].kappa =
          (1.0 - cfg_.curvature_prediction_alpha) * nodes_[i].kappa +
          cfg_.curvature_prediction_alpha * dyn_kappa;
    }
    nodes_.swap(next_nodes);
  }

  void regularizeDynamicStatesUnlocked() {
    if (nodes_.size() < 3) {
      return;
    }

    std::deque<LeadNode> next_nodes = nodes_;
    for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
      const double dt_prev = std::max(1e-3, (nodes_[i].stamp - nodes_[i - 1].stamp).toSec());
      const double dt_next = std::max(1e-3, (nodes_[i + 1].stamp - nodes_[i].stamp).toSec());
      const double a_smooth = 0.5 * (nodes_[i - 1].a + nodes_[i + 1].a);
      const double dkappa_smooth = 0.5 * (nodes_[i - 1].dkappa + nodes_[i + 1].dkappa);
      const double kappa_pred_prev = nodes_[i - 1].kappa + nodes_[i - 1].dkappa * dt_prev;
      const double kappa_pred_next = nodes_[i + 1].kappa - nodes_[i + 1].dkappa * dt_next;
      const double kappa_smooth = 0.5 * (kappa_pred_prev + kappa_pred_next);

      next_nodes[i].a =
          (1.0 - cfg_.acceleration_regularization_alpha) * nodes_[i].a +
          cfg_.acceleration_regularization_alpha * a_smooth;
      next_nodes[i].dkappa =
          (1.0 - cfg_.dkappa_regularization_alpha) * nodes_[i].dkappa +
          cfg_.dkappa_regularization_alpha * dkappa_smooth;
      next_nodes[i].kappa =
          (1.0 - cfg_.curvature_prediction_alpha) * nodes_[i].kappa +
          cfg_.curvature_prediction_alpha * clampValue(kappa_smooth, -0.8, 0.8);
    }
    nodes_.swap(next_nodes);
  }

  void strengthenMeasurementModelUnlocked() {
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const double lateral_ratio =
          std::fabs(nodes_[i].z_rel_y) / std::max(0.5, nodes_[i].range);
      const double heading_gain =
          cfg_.measurement_heading_alpha * nodes_[i].robust_weight *
          clampValue(1.0 - 0.6 * lateral_ratio, 0.2, 1.0);
      const double heading_err = wrapAngle(nodes_[i].heading_hint - nodes_[i].yaw);
      nodes_[i].yaw = wrapAngle(nodes_[i].yaw + heading_gain * heading_err);
    }
  }

  void clearStateUnlocked() {
    nodes_.clear();
    tail_buffer_world_.clear();
    motion_tail_buffer_world_.clear();
    motion_tail_pending_valid_ = false;
    last_tail_sample_action_ = "cleared";
    last_tail_pending_distance_ = 0.0;
    last_tail_pending_time_ = 0.0;
    track_ = LeadState2D();
    moving_flag_ = false;
    static_votes_ = 0;
    moving_votes_ = 0;
    resetUnconfirmedAccumulatorUnlocked();
    has_static_anchor_ = false;
    static_anchor_stamp_ = ros::Time();
    static_anchor_heading_ = 0.0;
    lateral_residual_filter_initialized_ = false;
    lateral_residual_filtered_ = 0.0;
    lateral_velocity_filtered_ = 0.0;
    lateral_residual_filter_stamp_ = ros::Time();
  }

  CenterlineProjection projectToSmoothedCenterlineUnlocked(double x, double y) const {
    CenterlineProjection best;
    if (nodes_.size() < 2) {
      return best;
    }
    double accumulated_s = 0.0;
    double best_d2 = std::numeric_limits<double>::infinity();
    const size_t start = nodes_.size() > 30 ? nodes_.size() - 30 : 0;
    for (size_t i = start; i + 1 < nodes_.size(); ++i) {
      const LeadNode& a = nodes_[i];
      const LeadNode& b = nodes_[i + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double seg_len2 = std::max(1e-9, dx * dx + dy * dy);
      const double t = clampValue(((x - a.x) * dx + (y - a.y) * dy) / seg_len2, 0.0, 1.0);
      const double px = a.x + t * dx;
      const double py = a.y + t * dy;
      const double ex = x - px;
      const double ey = y - py;
      const double d2 = ex * ex + ey * ey;
      if (d2 < best_d2) {
        best_d2 = d2;
        best.valid = true;
        best.x = px;
        best.y = py;
        best.yaw = std::atan2(dy, dx);
        best.s = accumulated_s + t * std::sqrt(seg_len2);
        best.distance = std::sqrt(std::max(0.0, d2));
      }
      accumulated_s += std::sqrt(seg_len2);
    }
    return best;
  }

  void pruneWindowUnlocked(const ros::Time& now) {
    const double window_age =
        std::max(0.30, std::min(cfg_.history_max_age, cfg_.fixed_lag_window));
    while (!nodes_.empty() && (now - nodes_.front().stamp).toSec() > window_age) {
      nodes_.pop_front();
    }
    while (nodes_.size() > static_cast<size_t>(std::max(2, cfg_.history_max_points))) {
      nodes_.pop_front();
    }
  }

  double robustWeight(double norm_metric) const {
    const double c = std::max(1e-3, cfg_.robust_cauchy_scale);
    return 1.0 / (1.0 + (norm_metric * norm_metric) / (c * c));
  }

  void smoothWindowUnlocked() {
    if (nodes_.empty()) {
      return;
    }

    for (size_t i = 0; i < nodes_.size(); ++i) {
        double pred_local_x = 0.0;
        double pred_local_y = 0.0;
        transformWorldToLocal(nodes_[i].ego, nodes_[i].x, nodes_[i].y,
                              &pred_local_x, &pred_local_y);
        const double rx = nodes_[i].z_rel_x - pred_local_x;
        const double ry = nodes_[i].z_rel_y - pred_local_y;
        const double metric =
            std::sqrt((rx * rx) / std::max(1e-6, nodes_[i].w_along * nodes_[i].w_along) +
                      (ry * ry) / std::max(1e-6, nodes_[i].w_cross * nodes_[i].w_cross));
        const double w = robustWeight(metric);
        nodes_[i].residual_along = rx;
        nodes_[i].residual_cross = ry;
        // Robustness and directional confidence are independent: a weak cross
        // observation must not suppress valid longitudinal progress.
        nodes_[i].robust_weight = clampValue(w, 0.02, 1.0);
    }
    applyFixedLagSmootherUnlocked();
    recomputeKnotGeometryUnlocked();
  }

  void recomputeKnotGeometryUnlocked() {
    if (nodes_.empty()) {
      return;
    }
    if (nodes_.size() == 1) {
      nodes_.front().v = 0.0;
      nodes_.front().kappa = 0.0;
      nodes_.front().dkappa = 0.0;
      return;
    }
    const double heading_ds = std::max(0.02, cfg_.heading_recompute_min_distance);
    for (size_t i = 0; i < nodes_.size(); ++i) {
      size_t i0 = i;
      size_t i1 = i;
      while (i0 > 0 &&
             pointDistance(nodes_[i0].x, nodes_[i0].y, nodes_[i].x, nodes_[i].y) < heading_ds) {
        --i0;
      }
      while (i1 + 1 < nodes_.size() &&
             pointDistance(nodes_[i1].x, nodes_[i1].y, nodes_[i].x, nodes_[i].y) < heading_ds) {
        ++i1;
      }
      if (i0 == i && i > 0) {
        i0 = i - 1;
      }
      if (i1 == i && i + 1 < nodes_.size()) {
        i1 = i + 1;
      }
      const double dx = nodes_[i1].x - nodes_[i0].x;
      const double dy = nodes_[i1].y - nodes_[i0].y;
      const double ds = hypot2(dx, dy);
      if (ds >= heading_ds) {
        const double geom_yaw = std::atan2(dy, dx);
        nodes_[i].yaw =
            wrapAngle(nodes_[i].yaw + 0.65 * wrapAngle(geom_yaw - nodes_[i].yaw));
      }
      const double dt =
          std::max(1e-3, (nodes_[i1].stamp - nodes_[i0].stamp).toSec());
      const double v_new = ds / dt;
      nodes_[i].a = (v_new - nodes_[i].v) / dt;
      nodes_[i].v = v_new;
    }
    if (nodes_.size() >= 2) {
      nodes_.front().yaw = nodes_[std::min<size_t>(1, nodes_.size() - 1)].yaw;
      nodes_.back().yaw = nodes_[nodes_.size() - 2].yaw;
    }

    for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
      const double ds = pointDistance(nodes_[i - 1].x, nodes_[i - 1].y,
                                      nodes_[i + 1].x, nodes_[i + 1].y);
      if (ds >= 2.0 * heading_ds) {
        const double dpsi = wrapAngle(nodes_[i + 1].yaw - nodes_[i - 1].yaw);
        nodes_[i].kappa = clampValue(dpsi / std::max(1e-3, ds), -0.8, 0.8);
      } else {
        nodes_[i].kappa *= 0.75;
      }
    }
    if (nodes_.size() >= 2) {
      nodes_.front().kappa = nodes_[std::min<size_t>(1, nodes_.size() - 1)].kappa;
      nodes_.back().kappa = nodes_[nodes_.size() - 2].kappa;
    }
    for (size_t i = 1; i < nodes_.size(); ++i) {
      const double dt = std::max(1e-3, (nodes_[i].stamp - nodes_[i - 1].stamp).toSec());
      nodes_[i - 1].dkappa =
          clampValue((nodes_[i].kappa - nodes_[i - 1].kappa) / dt, -1.5, 1.5);
    }
    if (!nodes_.empty()) {
      nodes_.back().dkappa = nodes_[nodes_.size() >= 2 ? nodes_.size() - 2 : 0].dkappa;
    }
  }

  void smoothCurvatureUnlocked() {
    if (nodes_.size() < 3) {
      return;
    }
    std::deque<LeadNode> next = nodes_;
    for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
      const double mk = 0.5 * (nodes_[i - 1].kappa + nodes_[i + 1].kappa);
      next[i].kappa =
          (1.0 - cfg_.smoother_curvature_alpha) * nodes_[i].kappa +
          cfg_.smoother_curvature_alpha * mk;
    }
    nodes_.swap(next);
  }

  void applyFixedLagSmootherUnlocked() {
    const int n = static_cast<int>(nodes_.size());
    if (n < 3) {
      return;
    }

    // Keep longitudinal progress observable while allowing ego-turn-contaminated
    // lateral measurements to be down-weighted in the historical Frenet frame.
    // A scalar observation weight cannot express this distinction.
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2 * n, 2 * n);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(2 * n);
    const auto xIndex = [](int i) { return 2 * i; };
    const auto yIndex = [](int i) { return 2 * i + 1; };

    for (int i = 0; i < n; ++i) {
      const LeadNode& node = nodes_[static_cast<size_t>(i)];
      const double yaw_uncertainty =
          1.0 + node.range * node.range * node.ego.sigma_yaw * node.ego.sigma_yaw;
      const double base_w =
          cfg_.fixed_lag_observation_weight *
          clampValue(node.robust_weight * node.tail_weight /
                         std::max(0.15, yaw_uncertainty),
                     0.02,
                     1.0);
      const double along_w =
          base_w * clampValue(node.tail_along_weight, 0.05, 1.0);
      const double cross_confidence =
          clampValue(node.tail_cross_weight, 0.02, 1.0);
      const double cross_w = base_w * cross_confidence;

      const double tx = std::cos(node.yaw);
      const double ty = std::sin(node.yaw);
      const double nx = -ty;
      const double ny = tx;
      const double wxx = along_w * tx * tx + cross_w * nx * nx;
      const double wxy = along_w * tx * ty + cross_w * nx * ny;
      const double wyy = along_w * ty * ty + cross_w * ny * ny;
      const int ix = xIndex(i);
      const int iy = yIndex(i);
      h(ix, ix) += wxx;
      h(ix, iy) += wxy;
      h(iy, ix) += wxy;
      h(iy, iy) += wyy;
      b(ix) += wxx * node.x + wxy * node.y;
      b(iy) += wxy * node.x + wyy * node.y;
    }

    const double w1 = std::max(0.0, cfg_.fixed_lag_smooth_weight);
    for (int i = 0; i + 1 < n; ++i) {
      for (int axis = 0; axis < 2; ++axis) {
        const int a = 2 * i + axis;
        const int c = 2 * (i + 1) + axis;
        h(a, a) += w1;
        h(c, c) += w1;
        h(a, c) -= w1;
        h(c, a) -= w1;
      }
    }

    const double w2 = std::max(0.0, cfg_.fixed_lag_curvature_weight);
    for (int i = 1; i + 1 < n; ++i) {
      for (int axis = 0; axis < 2; ++axis) {
        const int a = 2 * (i - 1) + axis;
        const int c = 2 * i + axis;
        const int d = 2 * (i + 1) + axis;
        h(a, a) += w2;
        h(c, c) += 4.0 * w2;
        h(d, d) += w2;
        h(a, c) -= 2.0 * w2;
        h(c, a) -= 2.0 * w2;
        h(c, d) -= 2.0 * w2;
        h(d, c) -= 2.0 * w2;
        h(a, d) += w2;
        h(d, a) += w2;
      }
    }

    for (int i = 0; i < 2 * n; ++i) {
      h(i, i) += 1e-6;
    }

    Eigen::LDLT<Eigen::MatrixXd> ldlt(h);
    if (ldlt.info() != Eigen::Success) {
      return;
    }
    const Eigen::VectorXd solution = ldlt.solve(b);
    if (ldlt.info() != Eigen::Success || !solution.allFinite()) {
      return;
    }
    for (int i = 0; i < n; ++i) {
      nodes_[static_cast<size_t>(i)].x = solution(xIndex(i));
      nodes_[static_cast<size_t>(i)].y = solution(yIndex(i));
    }
  }

  void enforceProgressionUnlocked() {
    if (nodes_.size() < 2) {
      return;
    }
    const double static_confidence = estimateStaticConfidenceUnlocked();
    const double static_residual = meanStaticHypothesisResidualUnlocked();
    const bool static_like =
        staticHypothesisWinsUnlocked() ||
        (static_confidence > 0.45 && static_residual < cfg_.ego_compensated_static_gate);
    if (!moving_flag_ && static_like) {
      return;
    }
    const double adaptive_min_progress =
        cfg_.temporal_min_progress * (1.0 - 0.9 * static_confidence) * (moving_flag_ ? 1.0 : 0.6);
    if (adaptive_min_progress <= 1e-4) {
      return;
    }
    for (size_t i = 1; i < nodes_.size(); ++i) {
      const double tx = std::cos(nodes_[i - 1].yaw);
      const double ty = std::sin(nodes_[i - 1].yaw);
      const double dx = nodes_[i].x - nodes_[i - 1].x;
      const double dy = nodes_[i].y - nodes_[i - 1].y;
      const double ds = dx * tx + dy * ty;
      if (ds < adaptive_min_progress) {
        const double corr = 0.5 * (adaptive_min_progress - ds);
        nodes_[i].x += corr * tx;
        nodes_[i].y += corr * ty;
      }
    }
  }

  HistoryPoint2D nodeToHistoryPointUnlocked(const LeadNode& node) const {
    HistoryPoint2D pt;
    pt.stamp = node.stamp;
    pt.x = node.x;
    pt.y = node.y;
    pt.heading = node.yaw;
    pt.speed = node.v;
    pt.curvature = node.kappa;
    pt.weight = clampValue(node.tail_weight, 0.0, 1.0);
    pt.ego_turn_contamination_score = clampValue(node.ego_turn_contamination_score, 0.0, 1.0);
    pt.heading_decoupled_by_ego_turn = node.heading_decoupled_by_ego_turn;
    pt.bbox_heading_used_for_history = node.bbox_heading_used_for_history;
    pt.fixed_frame_lateral_velocity = std::max(0.0, node.fixed_frame_lateral_velocity);
    pt.fixed_frame_lateral_velocity_filtered =
        std::max(0.0, node.fixed_frame_lateral_velocity_filtered);
    pt.lateral_innovation_gate = clampValue(node.lateral_innovation_gate, 0.0, 1.0);
    pt.lateral_innovation_frenet = node.lateral_innovation_frenet;
    pt.cross_residual_filtered = node.cross_residual_filtered;
    pt.tail_along_weight = clampValue(node.tail_along_weight, 0.0, 1.0);
    pt.tail_cross_weight = clampValue(node.tail_cross_weight, 0.0, 1.0);
    pt.cross_hold_active = node.cross_hold_active;
    return pt;
  }

  bool fastRejectTailPointUnlocked(const HistoryPoint2D& candidate) const {
    if (!tail_buffer_world_.empty()) {
      const HistoryPoint2D& prev = tail_buffer_world_.back();
      if (candidate.stamp.isValid() && prev.stamp.isValid() && candidate.stamp <= prev.stamp) {
        return true;
      }
      const double dt = (candidate.stamp.isValid() && prev.stamp.isValid())
                            ? std::max(1e-3, (candidate.stamp - prev.stamp).toSec())
                            : 1e-3;
      const double dx = candidate.x - prev.x;
      const double dy = candidate.y - prev.y;
      const double step = hypot2(dx, dy);
      if (step / dt > cfg_.lead_tail_max_speed) {
        return true;
      }

      const double tx = std::cos(prev.heading);
      const double ty = std::sin(prev.heading);
      const double along = dx * tx + dy * ty;
      const double cross = -dx * ty + dy * tx;
      if (cross > cfg_.tail_cross_gate || cross < -cfg_.tail_cross_gate) {
        return true;
      }
      if (along < cfg_.tail_reverse_gate) {
        return true;
      }
    }

    if (has_static_anchor_) {
      const double d_anchor =
          pointDistance(candidate.x, candidate.y, static_anchor_x_, static_anchor_y_);
      const bool static_like =
          staticHypothesisWinsUnlocked() &&
          dynamicTrailProgressUnlocked() < cfg_.static_anchor_progress_gate;
      if (static_like && d_anchor > std::max(0.20, 2.0 * cfg_.tail_cross_gate)) {
        return true;
      }
    }

    return false;
  }

  void refreshTailBufferUnlocked() {
    tail_buffer_world_.clear();
    if (nodes_.empty()) {
      return;
    }

    const ros::Time newest = nodes_.back().stamp;
    const size_t max_points =
        static_cast<size_t>(std::max(1, track_.static_mode ? cfg_.static_tail_keep_points
                                                           : cfg_.tail_buffer_max_points));
    for (size_t i = 0; i < nodes_.size(); ++i) {
      if (newest.isValid() && nodes_[i].stamp.isValid() &&
          (newest - nodes_[i].stamp).toSec() > cfg_.tail_buffer_max_age) {
        continue;
      }
      HistoryPoint2D pt = nodeToHistoryPointUnlocked(nodes_[i]);
      if (fastRejectTailPointUnlocked(pt)) {
        last_history_action_ = "tail_fast_rejected";
        continue;
      }
      pt.predicted = true;
      tail_buffer_world_.push_back(pt);
      while (tail_buffer_world_.size() > max_points) {
        tail_buffer_world_.pop_front();
      }
    }
    if (track_.static_mode && !tail_buffer_world_.empty()) {
      tail_buffer_world_.back().x = static_anchor_x_;
      tail_buffer_world_.back().y = static_anchor_y_;
      tail_buffer_world_.back().heading = track_.heading;
      tail_buffer_world_.back().predicted = true;
    }
  }

  void pushMotionTailPointUnlocked(const HistoryPoint2D& input, bool motion_suggests_dynamic) {
    if (!cfg_.provisional_tail_enabled || !motion_suggests_dynamic || !input.stamp.isValid()) {
      last_motion_tail_block_reason_ =
          !cfg_.provisional_tail_enabled ? "tail_disabled" :
          (!motion_suggests_dynamic ? "not_dynamic" : "bad_stamp");
      return;
    }
    last_motion_tail_block_reason_ = "none";
    HistoryPoint2D pt = input;
    pt.predicted = true;
    last_tail_sample_action_ = "append_first";
    last_tail_pending_distance_ = 0.0;
    last_tail_pending_time_ = 0.0;
    if (!motion_tail_buffer_world_.empty()) {
      const HistoryPoint2D& prev = motion_tail_buffer_world_.back();
      if (pt.stamp <= prev.stamp) {
        last_motion_tail_block_reason_ = "nonmonotonic_stamp";
        last_tail_sample_action_ = "reject_nonmonotonic";
        return;
      }
      const double dt = std::max(1e-3, (pt.stamp - prev.stamp).toSec());
      const double step = pointDistance(prev.x, prev.y, pt.x, pt.y);
      if (step / dt > cfg_.lead_tail_max_speed) {
        last_motion_tail_block_reason_ = "tail_speed";
        last_tail_sample_action_ = "reject_speed";
        return;
      }
      if (pt.fixed_frame_lateral_velocity > 2.0 * cfg_.lateral_outlier_velocity_gate &&
          pt.ego_turn_contamination_score > 0.50) {
        last_motion_tail_block_reason_ = "lateral_ego_turn_outlier";
        last_tail_sample_action_ = "reject_lateral_outlier";
        return;
      }
      const double tx = std::cos(prev.heading);
      const double ty = std::sin(prev.heading);
      const double along_progress =
          (pt.x - prev.x) * tx + (pt.y - prev.y) * ty;
      const bool distance_sample =
          step >= std::max(0.005, cfg_.motion_tail_sample_min_distance);
      const bool timed_sample =
          dt >= std::max(0.02, cfg_.motion_tail_sample_max_interval) &&
          along_progress >= cfg_.motion_tail_sample_min_along_progress;
      if (!distance_sample && !timed_sample) {
        motion_tail_pending_ = pt;
        motion_tail_pending_valid_ = true;
        last_tail_pending_distance_ = step;
        last_tail_pending_time_ = dt;
        last_tail_sample_action_ = "pending_near_sample";
        last_motion_tail_block_reason_ = "pending_sample";
        return;
      }
      last_tail_sample_action_ = distance_sample ? "append_distance" : "append_time_progress";
    }
    motion_tail_buffer_world_.push_back(pt);
    motion_tail_pending_valid_ = false;
    last_tail_pending_distance_ = 0.0;
    last_tail_pending_time_ = 0.0;
    const ros::Time newest = motion_tail_buffer_world_.back().stamp;
    while (!motion_tail_buffer_world_.empty() &&
           newest.isValid() &&
           motion_tail_buffer_world_.front().stamp.isValid() &&
           (newest - motion_tail_buffer_world_.front().stamp).toSec() > cfg_.provisional_tail_max_age) {
      motion_tail_buffer_world_.pop_front();
    }
    const size_t max_points = static_cast<size_t>(std::max(3, cfg_.tail_buffer_max_points));
    while (motion_tail_buffer_world_.size() > max_points) {
      motion_tail_buffer_world_.pop_front();
    }
  }

  void resetUnconfirmedAccumulatorUnlocked() {
    unconfirmed_accum_initialized_ = false;
    unconfirmed_progress_accum_ = 0.0;
    unconfirmed_time_accum_ = 0.0;
    unconfirmed_last_x_ = 0.0;
    unconfirmed_last_y_ = 0.0;
    unconfirmed_last_heading_ = 0.0;
    unconfirmed_last_stamp_ = ros::Time();
    unconfirmed_start_stamp_ = ros::Time();
  }

  void resetFixedFrameAppendDebugUnlocked() {
    last_fixed_frame_append_valid_ = false;
    last_fixed_frame_progress_ = 0.0;
    last_fixed_frame_kink_angle_ = 0.0;
    last_history_yaw_jump_ = 0.0;
    last_history_curvature_peak_ = 0.0;
  }

  void stabilizeHistoryCandidateUnlocked(HistoryPoint2D* candidate) const {
    if (candidate == NULL || confirmed_trail_world_.empty()) {
      return;
    }
    const HistoryPoint2D& last = confirmed_trail_world_.back();
    const double dist = pointDistance(last.x, last.y, candidate->x, candidate->y);
    if (dist < cfg_.fixed_frame_heading_hold_distance) {
      candidate->heading = last.heading;
      candidate->curvature = last.curvature;
      return;
    }
    if (dist >= cfg_.fixed_frame_heading_min_distance) {
      candidate->heading = std::atan2(candidate->y - last.y, candidate->x - last.x);
    } else {
      const double step =
          clampValue(wrapAngle(candidate->heading - last.heading),
                     -cfg_.fixed_frame_max_yaw_jump,
                     cfg_.fixed_frame_max_yaw_jump);
      candidate->heading = wrapAngle(last.heading + step);
    }
  }

  bool fixedFrameGeometryValidUnlocked(const HistoryPoint2D& candidate) {
    resetFixedFrameAppendDebugUnlocked();
    last_fixed_frame_append_valid_ = true;
    if (confirmed_trail_world_.empty()) {
      return true;
    }
    const HistoryPoint2D& last = confirmed_trail_world_.back();
    const double dx = candidate.x - last.x;
    const double dy = candidate.y - last.y;
    const double dist = hypot2(dx, dy);
    const double tx = std::cos(last.heading);
    const double ty = std::sin(last.heading);
    last_fixed_frame_progress_ = dx * tx + dy * ty;
    last_history_yaw_jump_ = std::fabs(wrapAngle(candidate.heading - last.heading));

    if (last_fixed_frame_progress_ < cfg_.fixed_frame_reverse_gate) {
      last_fixed_frame_append_valid_ = false;
      last_history_action_ = "history_fixed_frame_reverse_reject";
      return false;
    }
    if (dist < cfg_.fixed_frame_heading_hold_distance &&
        last_history_yaw_jump_ > cfg_.fixed_frame_max_yaw_jump) {
      last_fixed_frame_append_valid_ = false;
      last_history_action_ = "history_small_step_yaw_reject";
      return false;
    }

    if (confirmed_trail_world_.size() >= 2) {
      const HistoryPoint2D& prev = confirmed_trail_world_[confirmed_trail_world_.size() - 2];
      const double v1x = last.x - prev.x;
      const double v1y = last.y - prev.y;
      const double v2x = candidate.x - last.x;
      const double v2y = candidate.y - last.y;
      const double l1 = hypot2(v1x, v1y);
      const double l2 = hypot2(v2x, v2y);
      if (l1 > 1e-3 && l2 > 1e-3) {
        const double a1 = std::atan2(v1y, v1x);
        const double a2 = std::atan2(v2y, v2x);
        last_fixed_frame_kink_angle_ = std::fabs(wrapAngle(a2 - a1));
        last_history_curvature_peak_ =
            last_fixed_frame_kink_angle_ / std::max(1e-3, 0.5 * (l1 + l2));
        if (last_fixed_frame_kink_angle_ > cfg_.fixed_frame_kink_gate &&
            last_history_curvature_peak_ > cfg_.fixed_frame_curvature_gate) {
          last_fixed_frame_append_valid_ = false;
          last_history_action_ = "history_fixed_frame_kink_reject";
          return false;
        }
      }
    }
    return true;
  }

  void updateUnconfirmedAccumulatorUnlocked(const HistoryPoint2D& candidate) {
    if (!unconfirmed_accum_initialized_ || !candidate.stamp.isValid() ||
        (unconfirmed_last_stamp_.isValid() && candidate.stamp <= unconfirmed_last_stamp_)) {
      unconfirmed_accum_initialized_ = true;
      unconfirmed_progress_accum_ = 0.0;
      unconfirmed_time_accum_ = 0.0;
      unconfirmed_last_x_ = candidate.x;
      unconfirmed_last_y_ = candidate.y;
      unconfirmed_last_heading_ = candidate.heading;
      unconfirmed_last_stamp_ = candidate.stamp;
      unconfirmed_start_stamp_ = candidate.stamp;
      return;
    }

    const double dx = candidate.x - unconfirmed_last_x_;
    const double dy = candidate.y - unconfirmed_last_y_;
    const double seg_progress =
        std::max(0.0, dx * std::cos(unconfirmed_last_heading_) + dy * std::sin(unconfirmed_last_heading_));
    unconfirmed_progress_accum_ += seg_progress;
    unconfirmed_time_accum_ = std::max(0.0, (candidate.stamp - unconfirmed_start_stamp_).toSec());
    unconfirmed_last_x_ = candidate.x;
    unconfirmed_last_y_ = candidate.y;
    unconfirmed_last_heading_ = candidate.heading;
    unconfirmed_last_stamp_ = candidate.stamp;
  }

  bool staticHypothesisWinsUnlocked() const {
    return staticHypothesisScoreUnlocked() <=
           dynamicHypothesisScoreUnlocked() + cfg_.dual_hypothesis_score_margin;
  }

  void pruneTrailContainerUnlocked(std::deque<HistoryPoint2D>* trail, const ros::Time& now) {
    if (trail == NULL) {
      return;
    }
    // Confirmed history is the trajectory replay cache.  The fixed-lag node
    // window is bounded separately; removing confirmed points by wall time
    // makes an unconsumed path disappear when the lead vehicle stops.
    (void)now;
  }

  void commitConfirmedTrailPointUnlocked() {
    resetFixedFrameAppendDebugUnlocked();
    if (tail_buffer_world_.size() < static_cast<size_t>(std::max(2, cfg_.tail_confirm_min_points))) {
      if (track_.static_mode) {
        last_history_action_ = "history_anchor_hold";
      }
      return;
    }

    const bool conditioned_dynamic = egoMotionConditionedDynamicUnlocked();
    if ((track_.static_mode && !conditioned_dynamic) ||
        (!confirmDynamicHypothesisWinsUnlocked() && !conditioned_dynamic)) {
      last_history_action_ = "history_anchor_hold";
      return;
    }

    const double ego_speed = meanEgoSpeedUnlocked();
    const double min_confirm_age = std::max(cfg_.tail_confirm_min_time, cfg_.tail_confirm_min_age);
    const double min_confirm_progress = cfg_.confirm_min_progress;

    const ros::Time tail_stamp = tail_buffer_world_.back().stamp;
    ros::Time last_confirmed_stamp;
    if (!confirmed_trail_world_.empty()) {
      last_confirmed_stamp = confirmed_trail_world_.back().stamp;
    }

    size_t candidate_index = tail_buffer_world_.size();
    for (size_t i = tail_buffer_world_.size(); i > 0; --i) {
      const size_t idx = i - 1;
      const HistoryPoint2D& pt = tail_buffer_world_[idx];
      if (last_confirmed_stamp.isValid() && pt.stamp <= last_confirmed_stamp) {
        continue;
      }
      if ((tail_stamp - pt.stamp).toSec() >= min_confirm_age) {
        candidate_index = idx;
        break;
      }
    }
    if (candidate_index >= tail_buffer_world_.size()) {
      last_history_action_ = "history_tail_too_fresh";
      return;
    }

    HistoryPoint2D candidate = tail_buffer_world_[candidate_index];
    stabilizeHistoryCandidateUnlocked(&candidate);
    if (candidate.stamp.isValid() &&
        nodes_.back().ego.sigma_yaw > cfg_.max_ego_yaw_sigma_for_confirm) {
      last_history_action_ = "history_ego_yaw_uncertain_hold";
      return;
    }
    if (std::fabs(nodes_.back().ego.yaw_rate) > cfg_.imu_wheel_append_yaw_rate_gate &&
        meanEgoSpeedUnlocked() < cfg_.imu_wheel_append_speed_gate) {
      last_history_action_ = "history_imu_wheel_yaw_hold";
      last_fixed_frame_append_valid_ = false;
      return;
    }
    if (!fixedFrameGeometryValidUnlocked(candidate)) {
      return;
    }
    updateUnconfirmedAccumulatorUnlocked(candidate);
    const double candidate_static_res =
        residualAgainstWorldPointUnlocked(nodes_.back().ego, candidate.x, candidate.y,
                                          nodes_.back().z_rel_x, nodes_.back().z_rel_y);

    bool history_window_commit = false;
    if (dynamicTrailProgressUnlocked() < min_confirm_progress) {
      const double accum_distance_gate =
          std::min(cfg_.tail_confirm_min_distance, cfg_.confirm_accum_distance);
      const bool window_progress_ready =
          unconfirmed_progress_accum_ >= accum_distance_gate &&
          unconfirmed_time_accum_ >= std::min(0.50, cfg_.confirm_accum_timeout);
      if (!window_progress_ready) {
        last_history_action_ = "history_progress_too_small";
        return;
      }
      last_history_action_ = "history_window_progress_override";
      history_window_commit = true;
    }

    if (tail_buffer_world_.size() >= 2) {
      size_t checked = 0;
      size_t consistent = 0;
      for (size_t i = 1; i < tail_buffer_world_.size(); ++i) {
        const double dx = tail_buffer_world_[i].x - tail_buffer_world_[i - 1].x;
        const double dy = tail_buffer_world_[i].y - tail_buffer_world_[i - 1].y;
        const double along =
            dx * std::cos(tail_buffer_world_[i - 1].heading) +
            dy * std::sin(tail_buffer_world_[i - 1].heading);
        if (along >= cfg_.tail_reverse_gate) {
          ++consistent;
        }
        ++checked;
      }
      if (checked > 0 && consistent + 1 < checked) {
        last_history_action_ = "history_direction_unstable";
        return;
      }
    }

    const double accum_distance_gate = std::min(cfg_.tail_confirm_min_distance, cfg_.confirm_accum_distance);
    const double accum_timeout_gate = cfg_.confirm_accum_timeout;
    const bool accum_ready =
        history_window_commit ||
        unconfirmed_progress_accum_ >= accum_distance_gate ||
        (unconfirmed_time_accum_ >= accum_timeout_gate &&
         unconfirmed_progress_accum_ >= 3.0 * cfg_.measurement_cross_sigma);
    if (!accum_ready) {
      last_history_action_ = "history_progress_accumulating";
      return;
    }

    const bool dynamic_override_static =
        accum_ready &&
        (confirmDynamicHypothesisWinsUnlocked() || conditioned_dynamic) &&
        (history_window_commit ||
         unconfirmed_progress_accum_ >= cfg_.confirm_accum_distance ||
         unconfirmed_time_accum_ >= cfg_.confirm_accum_timeout) &&
        dynamicTrailProgressUnlocked() >= 0.5 * min_confirm_progress;
    if (ego_speed >= cfg_.ego_static_speed_gate &&
        candidate_static_res < cfg_.confirm_static_residual_gate &&
        !dynamic_override_static) {
      last_history_action_ = "history_static_explained";
      return;
    }
    if (ego_speed >= cfg_.ego_static_speed_gate &&
        candidate_static_res < cfg_.confirm_static_residual_gate &&
        dynamic_override_static) {
      last_history_action_ = "history_dynamic_accum_override";
    }

    if (confirmed_trail_world_.empty()) {
      candidate.predicted = false;
      confirmed_trail_world_.push_back(candidate);
      last_confirmed_append_stamp_ = candidate.stamp;
      if (last_history_action_ != "history_dynamic_accum_override") {
        last_history_action_ = "history_bootstrap";
      }
      pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
      resetUnconfirmedAccumulatorUnlocked();
      return;
    }

    const HistoryPoint2D& last_confirmed = confirmed_trail_world_.back();
    if (candidate.stamp <= last_confirmed.stamp) {
      last_history_action_ = "history_lowpass_update";
      return;
    }

    const double dist = pointDistance(last_confirmed.x, last_confirmed.y, candidate.x, candidate.y);
    const double heading_err = std::fabs(wrapAngle(candidate.heading - last_confirmed.heading));
    const double confirm_dt = std::max(1e-3, (candidate.stamp - last_confirmed.stamp).toSec());
    const double confirm_speed = dist / confirm_dt;
    const bool append =
        dist >= cfg_.tail_confirm_min_distance ||
        (dist >= 0.08 && heading_err > 0.18) ||
        (confirm_speed > cfg_.lead_static_speed_gate && dist >= 0.06);

    if (append) {
      candidate.predicted = false;
      confirmed_trail_world_.push_back(candidate);
      last_confirmed_append_stamp_ = candidate.stamp;
      if (history_window_commit) {
        last_history_action_ = "history_window_commit";
      } else if (last_history_action_ != "history_dynamic_accum_override") {
        last_history_action_ = "history_appended";
      }
      resetUnconfirmedAccumulatorUnlocked();
    } else {
      last_history_action_ = "history_lowpass_update";
    }

    pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
  }

  double confirmedAppendStallAgeUnlocked(const ros::Time& now) const {
    if (last_confirmed_append_stamp_.isZero() || !now.isValid()) {
      return confirmed_trail_world_.empty() ? -1.0 : std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, (now - last_confirmed_append_stamp_).toSec());
  }

  std::vector<HistoryPoint2D> exportHistoryUnlocked() const {
    std::vector<HistoryPoint2D> history;
    history.reserve(confirmed_trail_world_.size() + 1);
    for (size_t i = 0; i < confirmed_trail_world_.size(); ++i) {
      HistoryPoint2D pt = confirmed_trail_world_[i];
      pt.predicted = false;
      history.push_back(pt);
    }

    if (has_static_anchor_) {
      HistoryPoint2D anchor;
      anchor.stamp = static_anchor_stamp_.isValid()
                         ? static_anchor_stamp_
                         : (history.empty() ? ros::Time() : history.back().stamp);
      anchor.x = static_anchor_x_;
      anchor.y = static_anchor_y_;
      anchor.heading = static_anchor_heading_;
      anchor.speed = 0.0;
      anchor.curvature = 0.0;
      anchor.predicted = false;

      if (history.empty()) {
        history.push_back(anchor);
      } else {
        const double d = pointDistance(history.back().x, history.back().y, anchor.x, anchor.y);
        if (d > cfg_.static_anchor_append_distance_gate &&
            (!history.back().stamp.isValid() || !anchor.stamp.isValid() ||
             anchor.stamp > history.back().stamp)) {
          history.push_back(anchor);
        } else {
          history.back().stamp = anchor.stamp;
          history.back().x = anchor.x;
          history.back().y = anchor.y;
          history.back().heading = anchor.heading;
          history.back().speed = 0.0;
          history.back().predicted = false;
        }
      }
    }

    std::vector<HistoryPoint2D> out =
        removeNearDuplicatePoints(history, std::max(0.01, cfg_.fixed_frame_duplicate_distance));
    recomputeHeadingStable(out,
                           cfg_.fixed_frame_heading_min_distance,
                           cfg_.fixed_frame_heading_hold_distance,
                           cfg_.fixed_frame_max_yaw_jump);
    return out;
  }

  double worldHistorySpan(const std::vector<HistoryPoint2D>& pts) const {
    double span = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      span += pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
    }
    return span;
  }

  bool tailSuffixGeometryValidUnlocked(const std::vector<HistoryPoint2D>& pts) const {
    if (pts.size() < static_cast<size_t>(std::max(2, cfg_.provisional_tail_min_points))) {
      return false;
    }
    for (size_t i = 1; i < pts.size(); ++i) {
      const HistoryPoint2D& a = pts[i - 1];
      const HistoryPoint2D& b = pts[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double ds = hypot2(dx, dy);
      if (ds < 1e-4) {
        continue;
      }
      const double along = dx * std::cos(a.heading) + dy * std::sin(a.heading);
      const double dyaw = std::fabs(wrapAngle(b.heading - a.heading));
      if (along < cfg_.tail_reverse_gate) {
        return false;
      }
      if (ds < cfg_.fixed_frame_heading_hold_distance && dyaw > cfg_.fixed_frame_kink_gate) {
        return false;
      }
      if (dyaw > cfg_.fixed_frame_max_yaw_jump) {
        return false;
      }
    }
    return true;
  }

  bool tailSuffixSoftGeometryAcceptableUnlocked(const std::vector<HistoryPoint2D>& pts) const {
    if (!cfg_.tail_soft_geometry_enabled ||
        pts.size() < static_cast<size_t>(std::max(2, cfg_.provisional_tail_min_points))) {
      return false;
    }
    int soft_events = 0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const HistoryPoint2D& a = pts[i - 1];
      const HistoryPoint2D& b = pts[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double ds = hypot2(dx, dy);
      if (ds < 1e-4) {
        continue;
      }
      const double along = dx * std::cos(a.heading) + dy * std::sin(a.heading);
      const double dyaw = std::fabs(wrapAngle(b.heading - a.heading));
      const double hard_reverse = std::min(-0.08, 2.0 * cfg_.tail_reverse_gate);
      const double hard_yaw_jump = std::max(0.95, 1.50 * cfg_.fixed_frame_max_yaw_jump);
      const double hard_short_kink = std::max(0.85, 1.50 * cfg_.fixed_frame_kink_gate);
      if (along < hard_reverse || dyaw > hard_yaw_jump ||
          (ds < cfg_.fixed_frame_heading_hold_distance && dyaw > hard_short_kink)) {
        return false;
      }
      if (along < cfg_.tail_reverse_gate ||
          dyaw > cfg_.fixed_frame_max_yaw_jump ||
          (ds < cfg_.fixed_frame_heading_hold_distance && dyaw > cfg_.fixed_frame_kink_gate)) {
        ++soft_events;
      }
    }
    return soft_events <= 2;
  }

  struct SuffixShapeSummary {
    int flips = 0;
    double wiggle = 0.0;
    double rms = 0.0;
    double curvature_p95 = 0.0;
    double curvature_rate_p95 = 0.0;
    double net_yaw_change = 0.0;
  };

  SuffixShapeSummary computeSuffixShapeSummary(
      const std::vector<HistoryPoint2D>& pts,
      const std::vector<HistoryPoint2D>& before) const {
    SuffixShapeSummary out;
    if (pts.size() < 3) {
      return out;
    }
    std::vector<double> abs_kappa;
    std::vector<double> abs_dkappa;
    abs_kappa.reserve(pts.size());
    double last_signed = 0.0;
    double last_kappa = 0.0;
    bool has_last_kappa = false;
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
      const double ds =
          pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i + 1].x, pts[i + 1].y);
      if (ds < 1e-4) {
        continue;
      }
      const double dyaw = wrapAngle(pts[i + 1].heading - pts[i - 1].heading);
      const double kappa = dyaw / std::max(1e-3, ds);
      abs_kappa.push_back(std::fabs(kappa));
      if (has_last_kappa) {
        abs_dkappa.push_back(std::fabs(kappa - last_kappa) / std::max(1e-3, ds));
      }
      last_kappa = kappa;
      has_last_kappa = true;
      if (std::fabs(kappa) > 0.15) {
        if (std::fabs(last_signed) > 0.15 && last_signed * kappa < 0.0) {
          ++out.flips;
        }
        last_signed = kappa;
      }
      const double mx = 0.5 * (pts[i - 1].x + pts[i + 1].x);
      const double my = 0.5 * (pts[i - 1].y + pts[i + 1].y);
      out.wiggle += pointDistance(pts[i].x, pts[i].y, mx, my);
    }
    double rms = 0.0;
    size_t rms_n = 0;
    if (before.size() == pts.size()) {
      for (size_t i = 0; i < pts.size(); ++i) {
        const double e = pointDistance(pts[i].x, pts[i].y, before[i].x, before[i].y);
        rms += e * e;
        ++rms_n;
      }
    }
    if (!abs_kappa.empty()) {
      std::sort(abs_kappa.begin(), abs_kappa.end());
      out.curvature_p95 =
          abs_kappa[std::min(abs_kappa.size() - 1,
                             static_cast<size_t>(0.95 * static_cast<double>(abs_kappa.size() - 1)))];
    }
    if (!abs_dkappa.empty()) {
      std::sort(abs_dkappa.begin(), abs_dkappa.end());
      out.curvature_rate_p95 =
          abs_dkappa[std::min(abs_dkappa.size() - 1,
                              static_cast<size_t>(0.95 * static_cast<double>(abs_dkappa.size() - 1)))];
    }
    // Normalize by arc length so a longer, otherwise identical trajectory is
    // not automatically classified as a stronger S-wave.
    out.wiggle /= std::max(0.10, worldHistorySpan(pts));
    out.net_yaw_change =
        std::fabs(wrapAngle(pts.back().heading - pts.front().heading));
    out.rms = rms_n > 0 ? std::sqrt(rms / static_cast<double>(rms_n)) : 0.0;
    return out;
  }

  void fillSuffixShapeMetrics(const std::vector<HistoryPoint2D>& pts,
                              const std::vector<HistoryPoint2D>& before,
                              TrackerDebugState* debug) const {
    if (debug == NULL || pts.size() < 3) {
      return;
    }
    const SuffixShapeSummary summary = computeSuffixShapeSummary(pts, before);
    debug->suffix_sign_flip_count = summary.flips;
    debug->suffix_wiggle_score = summary.wiggle;
    debug->suffix_wiggle_after = summary.wiggle;
    debug->suffix_smooth_rms = summary.rms;
    debug->suffix_curvature_p95 = summary.curvature_p95;
    debug->suffix_curvature_rate_p95 = summary.curvature_rate_p95;
    const double wiggle_norm =
        clampValue(summary.wiggle / std::max(0.05, cfg_.reference_health_wiggle_gate), 0.0, 1.0);
    const double flip_penalty =
        cfg_.reference_health_flip_penalty *
        std::min(4.0, static_cast<double>(summary.flips));
    const double kappa_excess =
        clampValue((summary.curvature_p95 - cfg_.reference_health_curvature_gate) /
                       std::max(1e-3, cfg_.reference_health_curvature_gate),
                   0.0,
                   1.0);
    double health =
        1.0 - cfg_.reference_health_wiggle_penalty * wiggle_norm -
        flip_penalty - cfg_.reference_health_curvature_penalty * kappa_excess;
    const bool s_wave =
        summary.net_yaw_change < 0.22 &&
        (summary.flips >= cfg_.suffix_smooth_sign_flip_gate ||
         (summary.flips >= 1 && summary.wiggle > cfg_.suffix_smooth_wiggle_gate));
    if (s_wave) {
      health = std::min(health, cfg_.reference_health_s_wave_cap);
    }
    debug->reference_health_score = clampValue(health, 0.0, 1.0);
  }

  void fillTailWeightDebug(const std::vector<HistoryPoint2D>& pts,
                           TrackerDebugState* debug) const {
    if (debug == NULL || pts.empty()) {
      return;
    }
    double min_w = 1.0;
    double max_w = 0.0;
    double sum_w = 0.0;
    double sum_along_w = 0.0;
    double sum_cross_w = 0.0;
    double max_contamination = 0.0;
    double max_lateral_gate = 0.0;
    double max_lateral_velocity = 0.0;
    double max_lateral_velocity_filtered = 0.0;
    double max_lateral_innovation = 0.0;
    double max_cross_residual_filtered = 0.0;
    bool heading_decoupled = false;
    bool bbox_heading_used = false;
    bool cross_hold = false;
    for (size_t i = 0; i < pts.size(); ++i) {
      const HistoryPoint2D& pt = pts[i];
      const double w = clampValue(pt.weight, 0.0, 1.0);
      min_w = std::min(min_w, w);
      max_w = std::max(max_w, w);
      sum_w += w;
      sum_along_w += clampValue(pt.tail_along_weight, 0.0, 1.0);
      sum_cross_w += clampValue(pt.tail_cross_weight, 0.0, 1.0);
      max_contamination =
          std::max(max_contamination, clampValue(pt.ego_turn_contamination_score, 0.0, 1.0));
      max_lateral_gate =
          std::max(max_lateral_gate, clampValue(pt.lateral_innovation_gate, 0.0, 1.0));
      max_lateral_velocity =
          std::max(max_lateral_velocity, std::max(0.0, pt.fixed_frame_lateral_velocity));
      max_lateral_velocity_filtered =
          std::max(max_lateral_velocity_filtered,
                   std::max(0.0, pt.fixed_frame_lateral_velocity_filtered));
      max_lateral_innovation =
          std::max(max_lateral_innovation, std::fabs(pt.lateral_innovation_frenet));
      max_cross_residual_filtered =
          std::max(max_cross_residual_filtered, std::fabs(pt.cross_residual_filtered));
      heading_decoupled = heading_decoupled || pt.heading_decoupled_by_ego_turn;
      bbox_heading_used = bbox_heading_used || pt.bbox_heading_used_for_history;
      cross_hold = cross_hold || pt.cross_hold_active;
    }
    const double mean_w = sum_w / static_cast<double>(pts.size());
    const double mean_along_w = sum_along_w / static_cast<double>(pts.size());
    const double mean_cross_w = sum_cross_w / static_cast<double>(pts.size());
    debug->tail_point_weight_min = min_w;
    debug->tail_point_weight_max = max_w;
    debug->tail_point_weight_mean = mean_w;
    debug->tail_point_weight = mean_w;
    debug->tail_along_weight_mean = mean_along_w;
    debug->tail_cross_weight_mean = mean_cross_w;
    debug->tail_cross_weight_applied = mean_cross_w;
    debug->ego_turn_contamination_score = max_contamination;
    debug->heading_decoupled_by_ego_turn = heading_decoupled;
    debug->bbox_heading_used_for_history = bbox_heading_used;
    debug->lateral_innovation_gate = max_lateral_gate;
    debug->fixed_frame_lateral_velocity = max_lateral_velocity;
    debug->fixed_frame_lateral_velocity_filtered = max_lateral_velocity_filtered;
    debug->lateral_innovation_frenet = max_lateral_innovation;
    debug->cross_residual_filtered = max_cross_residual_filtered;
    debug->cross_hold_active = cross_hold;
    debug->ego_turn_cross_hold_active = cross_hold;
    debug->cross_hold_reason = cross_hold ? "ego_turn_low_progress" : "none";
    debug->ego_motion_decoupled_tail_yaw = heading_decoupled || max_contamination > 0.0;
  }

  std::vector<HistoryPoint2D> fitSuffixCenterlineWindowed(
      const std::vector<HistoryPoint2D>& input,
      double window,
      double alpha,
      double max_adjust) const {
    if (input.size() < 5 || window <= 1e-3 || alpha <= 1e-6) {
      return input;
    }
    std::vector<double> arc(input.size(), 0.0);
    for (size_t i = 1; i < input.size(); ++i) {
      arc[i] = arc[i - 1] +
               pointDistance(input[i - 1].x, input[i - 1].y, input[i].x, input[i].y);
    }
    const double half_window = 0.5 * std::max(0.20, window);
    std::vector<HistoryPoint2D> out = input;
    for (size_t i = 1; i + 1 < input.size(); ++i) {
      size_t begin = i;
      while (begin > 0 && arc[i] - arc[begin - 1] <= half_window) {
        --begin;
      }
      size_t end = i;
      while (end + 1 < input.size() && arc[end + 1] - arc[i] <= half_window) {
        ++end;
      }
      if (end <= begin + 2) {
        continue;
      }
      double mx = 0.0;
      double my = 0.0;
      double wsum = 0.0;
      for (size_t j = begin; j <= end; ++j) {
        const double d = std::fabs(arc[j] - arc[i]);
        const double w = 1.0 / (1.0 + d / std::max(0.05, half_window));
        mx += w * input[j].x;
        my += w * input[j].y;
        wsum += w;
      }
      if (wsum <= 1e-6) {
        continue;
      }
      mx /= wsum;
      my /= wsum;
      double cxx = 0.0;
      double cxy = 0.0;
      double cyy = 0.0;
      for (size_t j = begin; j <= end; ++j) {
        const double d = std::fabs(arc[j] - arc[i]);
        const double w = 1.0 / (1.0 + d / std::max(0.05, half_window));
        const double dx = input[j].x - mx;
        const double dy = input[j].y - my;
        cxx += w * dx * dx;
        cxy += w * dx * dy;
        cyy += w * dy * dy;
      }
      const double theta = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
      const double tx = std::cos(theta);
      const double ty = std::sin(theta);
      const double proj = (input[i].x - mx) * tx + (input[i].y - my) * ty;
      const double px = mx + proj * tx;
      const double py = my + proj * ty;
      double dx = px - input[i].x;
      double dy = py - input[i].y;
      const double dist = hypot2(dx, dy);
      const double step = std::min(std::max(0.0, max_adjust), alpha * dist);
      if (step <= 1e-6 || dist <= 1e-6) {
        continue;
      }
      const double scale = step / dist;
      out[i].x = input[i].x + scale * dx;
      out[i].y = input[i].y + scale * dy;
    }
    recomputeHeadingStable(out,
                           cfg_.fixed_frame_heading_min_distance,
                           cfg_.fixed_frame_heading_hold_distance,
                           cfg_.fixed_frame_max_yaw_jump);
    return out;
  }

  std::vector<HistoryPoint2D> smoothReferenceSuffixUnlocked(
      const std::vector<HistoryPoint2D>& input,
      TrackerDebugState* debug) const {
    if (input.size() < 4 || worldHistorySpan(input) < 0.25) {
      fillTailWeightDebug(input, debug);
      fillSuffixShapeMetrics(input, input, debug);
      return input;
    }
    fillTailWeightDebug(input, debug);
    std::vector<HistoryPoint2D> cur =
        resampleWorldByArcLength(input, std::max(0.04, cfg_.suffix_smooth_resample_ds));
    cur = removeNearDuplicatePoints(cur, std::max(0.01, cfg_.fixed_frame_duplicate_distance));
    recomputeHeadingStable(cur,
                           cfg_.fixed_frame_heading_min_distance,
                           cfg_.fixed_frame_heading_hold_distance,
                           cfg_.fixed_frame_max_yaw_jump);
    if (cur.size() < 4) {
      fillSuffixShapeMetrics(cur, cur, debug);
      return cur;
    }
    const std::vector<HistoryPoint2D> before = cur;
    const SuffixShapeSummary before_summary = computeSuffixShapeSummary(before, before);
    if (debug != NULL) {
      debug->suffix_wiggle_before = before_summary.wiggle;
    }
    if (cfg_.suffix_smooth_passes <= 0 ||
        cfg_.suffix_smooth_alpha <= 0.0 ||
        cfg_.suffix_smooth_max_adjust <= 0.0) {
      if (debug != NULL) {
        debug->suffix_s_wave_suppressed = false;
        debug->suffix_centerline_fit_active = false;
        debug->suffix_fit_mode = "fixed_lag_only";
        debug->curve_refit_active = false;
        debug->curve_refit_rms = 0.0;
      }
      fillSuffixShapeMetrics(cur, before, debug);
      return cur;
    }
    const bool s_wave =
        before_summary.net_yaw_change < 0.22 &&
        (before_summary.flips >= cfg_.suffix_smooth_sign_flip_gate ||
         (before_summary.flips >= 1 &&
          before_summary.wiggle > cfg_.suffix_smooth_wiggle_gate));
    const bool same_sign_curve =
        before_summary.net_yaw_change >= 0.18 &&
        before_summary.flips <= 1;
    const double alpha = same_sign_curve
                             ? std::min(0.20, cfg_.suffix_smooth_alpha)
                             : clampValue(cfg_.suffix_smooth_alpha, 0.0, 0.60);
    const double huber = std::max(1e-3, cfg_.suffix_smooth_huber_delta);
    const double max_adjust = same_sign_curve
                                  ? std::min(0.055, cfg_.suffix_smooth_max_adjust)
                                  : std::max(0.0, cfg_.suffix_smooth_max_adjust);
    std::vector<HistoryPoint2D> next = cur;
    for (size_t i = 1; i + 1 < cur.size(); ++i) {
      const double mx = 0.5 * (cur[i - 1].x + cur[i + 1].x);
      const double my = 0.5 * (cur[i - 1].y + cur[i + 1].y);
      double dx = mx - cur[i].x;
      double dy = my - cur[i].y;
      const double err = hypot2(dx, dy);
      if (err > huber) {
        const double scale = huber / std::max(1e-6, err);
        dx *= scale;
        dy *= scale;
      }
      const double cross_weight =
          clampValue(cur[i].tail_cross_weight, 0.05, 1.0);
      const double low_confidence =
          std::max(1.0 - clampValue(cur[i].weight, 0.0, 1.0),
                   1.0 - cross_weight);
      const double innovation =
          clampValue(cur[i].lateral_innovation_gate, 0.0, 1.0);
      const double local_alpha =
          clampValue(alpha * (1.0 + 0.70 * low_confidence + 0.30 * innovation),
                     0.0,
                     0.70);
      const double step =
          std::min(max_adjust * (1.0 + 0.50 * low_confidence),
                   local_alpha * hypot2(dx, dy));
      if (step > 1e-6) {
        const double scale = step / std::max(1e-6, hypot2(dx, dy));
        next[i].x = cur[i].x + scale * dx;
        next[i].y = cur[i].y + scale * dy;
      }
    }
    cur.swap(next);
    recomputeHeadingStable(cur,
                           cfg_.fixed_frame_heading_min_distance,
                           cfg_.fixed_frame_heading_hold_distance,
                           cfg_.fixed_frame_max_yaw_jump);
    if (debug != NULL) {
      debug->suffix_s_wave_suppressed = s_wave;
      debug->suffix_centerline_fit_active = true;
      debug->suffix_fit_mode = same_sign_curve
                                   ? "huber_single_curve_preserve"
                                   : "huber_single";
      debug->curve_refit_active = false;
      debug->curve_refit_rms = 0.0;
    }
    fillSuffixShapeMetrics(cur, before, debug);
    return cur;
  }

  std::vector<HistoryPoint2D> directMotionTailCandidateUnlocked(
      TrackerDebugState* debug) const {
    std::vector<HistoryPoint2D> candidate;
    if (debug != NULL) {
      debug->motion_tail_hard_valid = false;
      debug->motion_tail_block_reason =
          last_motion_tail_block_reason_.empty() ? "none" : last_motion_tail_block_reason_;
    }
    if (!cfg_.provisional_tail_enabled ||
        motion_tail_buffer_world_.size() <
            static_cast<size_t>(std::max(2, cfg_.provisional_tail_min_points))) {
      if (debug != NULL && debug->provisional_publish_block_reason == "none") {
        debug->provisional_publish_block_reason = "tail_points";
        debug->motion_tail_block_reason = "tail_points";
      }
      return candidate;
    }

    const ros::Time newest = motion_tail_buffer_world_.back().stamp;
    for (size_t i = 0; i < motion_tail_buffer_world_.size(); ++i) {
      const HistoryPoint2D& pt = motion_tail_buffer_world_[i];
      if (newest.isValid() && pt.stamp.isValid() &&
          (newest - pt.stamp).toSec() > cfg_.provisional_tail_max_age) {
        continue;
      }
      candidate.push_back(pt);
    }

    // The motion-tail buffer is already sampled against its last retained
    // point. A second, larger dedup threshold used to collapse low-speed
    // motion back to one effective point.
    recomputeHeadingStable(candidate,
                           cfg_.fixed_frame_heading_min_distance,
                           cfg_.fixed_frame_heading_hold_distance,
                           cfg_.fixed_frame_max_yaw_jump);
    fillTailWeightDebug(candidate, debug);
    const double span = worldHistorySpan(candidate);
    const double age =
        candidate.size() >= 2 && candidate.back().stamp.isValid() && candidate.front().stamp.isValid()
            ? std::max(0.0, (candidate.back().stamp - candidate.front().stamp).toSec())
            : 0.0;
    if (debug != NULL) {
      debug->provisional_tail_points = static_cast<int>(candidate.size());
      debug->provisional_tail_span = span;
      debug->provisional_tail_age = age;
      debug->unique_tail_points = static_cast<int>(candidate.size());
      debug->unique_tail_span = span;
      debug->unique_tail_time = age;
      debug->motion_tail_points = static_cast<int>(motion_tail_buffer_world_.size());
      debug->motion_tail_span = worldHistorySpan(
          std::vector<HistoryPoint2D>(motion_tail_buffer_world_.begin(), motion_tail_buffer_world_.end()));
      debug->motion_tail_age =
          motion_tail_buffer_world_.size() >= 2 &&
                  motion_tail_buffer_world_.back().stamp.isValid() &&
                  motion_tail_buffer_world_.front().stamp.isValid()
              ? std::max(0.0,
                         (motion_tail_buffer_world_.back().stamp -
                          motion_tail_buffer_world_.front().stamp).toSec())
              : -1.0;
    }

    const bool enough_points =
        candidate.size() >= static_cast<size_t>(std::max(2, cfg_.provisional_tail_min_points));
    const bool short_span_ok = span >= cfg_.provisional_tail_short_min_span;
    const bool full_span_ok = span >= cfg_.provisional_tail_min_span;
    const bool short_time_ok = age >= cfg_.provisional_tail_short_min_time;
    const bool full_time_ok = age >= cfg_.provisional_tail_min_time;
    const bool geometry_ok = tailSuffixGeometryValidUnlocked(candidate);
    const bool soft_geometry_ok = geometry_ok || tailSuffixSoftGeometryAcceptableUnlocked(candidate);
    if (!enough_points || !short_span_ok || !short_time_ok || !soft_geometry_ok) {
      if (debug != NULL) {
        if (!enough_points) {
          debug->provisional_publish_block_reason = "tail_points";
        } else if (!short_span_ok) {
          debug->provisional_publish_block_reason = "tail_span";
        } else if (!short_time_ok) {
          debug->provisional_publish_block_reason = "tail_age";
        } else {
          debug->provisional_publish_block_reason = "tail_geometry";
        }
        debug->motion_tail_block_reason = debug->provisional_publish_block_reason;
      }
      return {};
    }

    if (debug != NULL) {
      debug->provisional_publish_block_reason = "none";
      debug->motion_tail_block_reason = geometry_ok ? "none" : "soft_geometry";
      debug->motion_tail_hard_valid = geometry_ok;
      debug->motion_tail_used_as_reference = true;
      debug->motion_tail_promoted_to_reference = true;
      debug->append_gate_softened = !geometry_ok;
      if (!geometry_ok) {
        debug->tail_point_weight_min = std::min(debug->tail_point_weight_min, 0.35);
        debug->tail_point_weight =
            std::min(debug->tail_point_weight, debug->tail_point_weight_min);
      }
      if (!full_span_ok || !full_time_ok) {
        debug->provisional_tail_block_detail = "short_tail_direct";
      }
    }
    return candidate;
  }

  std::vector<HistoryPoint2D> suffixEstimatorTailUnlocked(TrackerDebugState* debug,
                                                          double confirmed_span) const {
    std::vector<HistoryPoint2D> candidate;
    const int active_min_points = std::max(3, cfg_.suffix_estimator_min_points);
    if (!cfg_.provisional_tail_enabled ||
        motion_tail_buffer_world_.size() <
            static_cast<size_t>(active_min_points)) {
      return candidate;
    }

    const ros::Time newest = motion_tail_buffer_world_.back().stamp;
    for (size_t i = 0; i < motion_tail_buffer_world_.size(); ++i) {
      const HistoryPoint2D& pt = motion_tail_buffer_world_[i];
      if (newest.isValid() && pt.stamp.isValid() &&
          (newest - pt.stamp).toSec() > cfg_.provisional_tail_max_age) {
        continue;
      }
      candidate.push_back(pt);
    }

    recomputeHeadingStable(candidate,
                           cfg_.fixed_frame_heading_min_distance,
                           cfg_.fixed_frame_heading_hold_distance,
                           cfg_.fixed_frame_max_yaw_jump);
    fillTailWeightDebug(candidate, debug);
    double span = worldHistorySpan(candidate);
    const double age =
        candidate.size() >= 2 && candidate.back().stamp.isValid() && candidate.front().stamp.isValid()
            ? std::max(0.0, (candidate.back().stamp - candidate.front().stamp).toSec())
            : 0.0;
    if (debug != NULL) {
      debug->suffix_estimator_tail_age = age;
    }
    const bool hard_geometry_ok = tailSuffixGeometryValidUnlocked(candidate);
    const bool soft_geometry_ok = hard_geometry_ok || tailSuffixSoftGeometryAcceptableUnlocked(candidate);
    const double active_min_span =
        confirmed_span >= 0.80
            ? cfg_.suffix_estimator_confirmed_min_span
            : cfg_.suffix_estimator_bootstrap_min_span;
    const double active_min_time =
        std::max(cfg_.suffix_estimator_min_time, cfg_.suffix_estimator_tail_min_time);
    if (candidate.size() < static_cast<size_t>(active_min_points) ||
        span < active_min_span ||
        age < active_min_time ||
        !soft_geometry_ok) {
      return {};
    }
    if (debug != NULL) {
      debug->append_gate_softened = !hard_geometry_ok;
      if (!hard_geometry_ok) {
        debug->tail_point_weight_min = std::min(debug->tail_point_weight_min, 0.35);
        debug->tail_point_weight =
            std::min(debug->tail_point_weight, debug->tail_point_weight_min);
      }
    }
    const double max_extension =
        std::max(0.08,
                 std::min(cfg_.suffix_estimator_max_extension_span,
                          std::max(0.08, age)));
    if (span > max_extension && candidate.size() >= 2) {
      std::vector<HistoryPoint2D> limited;
      limited.push_back(candidate.back());
      double accum = 0.0;
      for (size_t i = candidate.size() - 1; i > 0; --i) {
        const HistoryPoint2D& cur = candidate[i];
        const HistoryPoint2D& prev = candidate[i - 1];
        const double ds = pointDistance(cur.x, cur.y, prev.x, prev.y);
        if (accum + ds > max_extension && limited.size() >= static_cast<size_t>(active_min_points)) {
          break;
        }
        limited.push_back(prev);
        accum += ds;
      }
      std::reverse(limited.begin(), limited.end());
      if (limited.size() >= static_cast<size_t>(active_min_points) &&
          worldHistorySpan(limited) >= active_min_span &&
          (tailSuffixGeometryValidUnlocked(limited) ||
           tailSuffixSoftGeometryAcceptableUnlocked(limited))) {
        candidate.swap(limited);
        span = worldHistorySpan(candidate);
        if (debug != NULL) {
          debug->suffix_extension_growth_limited = true;
        }
      }
    }
    if (debug != NULL) {
      debug->suffix_extension_span = span;
      debug->suffix_estimator_output_span = span;
    }
    return candidate;
  }

  std::vector<HistoryPoint2D> stableTailSuffixUnlocked(TrackerDebugState* debug) const {
    std::vector<HistoryPoint2D> suffix;
    const std::deque<HistoryPoint2D>& source =
        motion_tail_buffer_world_.size() >= static_cast<size_t>(std::max(2, cfg_.provisional_tail_min_points))
            ? motion_tail_buffer_world_
            : tail_buffer_world_;
    if (!cfg_.provisional_tail_enabled ||
        source.size() < static_cast<size_t>(std::max(2, cfg_.provisional_tail_min_points))) {
      return suffix;
    }

    const ros::Time newest = source.back().stamp;
    ros::Time last_confirmed_stamp;
    if (!confirmed_trail_world_.empty()) {
      last_confirmed_stamp = confirmed_trail_world_.back().stamp;
    }

    for (size_t i = 0; i < source.size(); ++i) {
      const HistoryPoint2D& pt = source[i];
      if (last_confirmed_stamp.isValid() && pt.stamp.isValid() &&
          pt.stamp <= last_confirmed_stamp) {
        continue;
      }
      if (newest.isValid() && pt.stamp.isValid() &&
          (newest - pt.stamp).toSec() > cfg_.provisional_tail_max_age) {
        continue;
      }
      suffix.push_back(pt);
    }

    recomputeHeadingStable(suffix,
                           cfg_.fixed_frame_heading_min_distance,
                           cfg_.fixed_frame_heading_hold_distance,
                           cfg_.fixed_frame_max_yaw_jump);
    const double span = worldHistorySpan(suffix);
    const double age = (suffix.size() >= 2 && newest.isValid() && suffix.front().stamp.isValid())
                           ? std::max(0.0, (newest - suffix.front().stamp).toSec())
                           : 0.0;
    if (debug != NULL) {
      debug->provisional_tail_points = static_cast<int>(suffix.size());
      debug->provisional_tail_span = span;
      debug->provisional_tail_age = age;
      debug->motion_tail_points = static_cast<int>(motion_tail_buffer_world_.size());
      debug->motion_tail_span = worldHistorySpan(
          std::vector<HistoryPoint2D>(motion_tail_buffer_world_.begin(), motion_tail_buffer_world_.end()));
      debug->motion_tail_age =
          motion_tail_buffer_world_.size() >= 2 &&
                  motion_tail_buffer_world_.back().stamp.isValid() &&
                  motion_tail_buffer_world_.front().stamp.isValid()
              ? std::max(0.0,
                         (motion_tail_buffer_world_.back().stamp -
                          motion_tail_buffer_world_.front().stamp).toSec())
              : -1.0;
    }

    if (suffix.size() < static_cast<size_t>(std::max(2, cfg_.provisional_tail_min_points)) ||
        span < cfg_.provisional_tail_min_span ||
        age < cfg_.provisional_tail_min_time ||
        !tailSuffixGeometryValidUnlocked(suffix)) {
      if (debug != NULL) {
        if (suffix.size() < static_cast<size_t>(std::max(2, cfg_.provisional_tail_min_points))) {
          debug->provisional_publish_block_reason = "tail_points";
        } else if (span < cfg_.provisional_tail_min_span) {
          debug->provisional_publish_block_reason = "tail_span";
        } else if (age < cfg_.provisional_tail_min_time) {
          debug->provisional_publish_block_reason = "tail_age";
        } else {
          debug->provisional_publish_block_reason = "tail_geometry";
        }
      }
      return {};
    }
    if (debug != NULL) {
      debug->provisional_publish_block_reason = "none";
      debug->motion_tail_used_as_reference = (&source == &motion_tail_buffer_world_);
    }
    return suffix;
  }

  std::vector<HistoryPoint2D> exportReferenceHistoryWithTailUnlocked(
      TrackerDebugState* debug) const {
    std::vector<HistoryPoint2D> history = exportHistoryUnlocked();
    if (debug != NULL) {
      debug->selected_segment_source = history.empty() ? "none" : "confirmed";
      debug->provisional_tail_points = 0;
      debug->provisional_tail_span = 0.0;
      debug->provisional_tail_age = -1.0;
      debug->motion_tail_points = static_cast<int>(motion_tail_buffer_world_.size());
      debug->motion_tail_span = worldHistorySpan(
          std::vector<HistoryPoint2D>(motion_tail_buffer_world_.begin(), motion_tail_buffer_world_.end()));
      debug->motion_tail_age =
          motion_tail_buffer_world_.size() >= 2 &&
                  motion_tail_buffer_world_.back().stamp.isValid() &&
                  motion_tail_buffer_world_.front().stamp.isValid()
              ? std::max(0.0,
                         (motion_tail_buffer_world_.back().stamp -
                          motion_tail_buffer_world_.front().stamp).toSec())
              : -1.0;
    }

    std::vector<HistoryPoint2D> direct_motion_tail;
    if (!history.empty()) {
      direct_motion_tail = suffixEstimatorTailUnlocked(debug, worldHistorySpan(history));
      if (!direct_motion_tail.empty()) {
        if (debug != NULL) {
          debug->provisional_publish_block_reason = "none";
          debug->provisional_tail_block_detail = "suffix_estimator";
          debug->motion_tail_block_reason = "none";
          debug->motion_tail_hard_valid = true;
        }
      }
    } else {
      direct_motion_tail = directMotionTailCandidateUnlocked(debug);
    }
    ros::Time newest_tail_stamp;
    if (!motion_tail_buffer_world_.empty()) {
      newest_tail_stamp = motion_tail_buffer_world_.back().stamp;
    } else if (!history.empty()) {
      newest_tail_stamp = history.back().stamp;
    }
    const double append_stall_age = confirmedAppendStallAgeUnlocked(newest_tail_stamp);
    if (debug != NULL) {
      debug->append_stall_age = append_stall_age;
    }
    const bool confirmed_append_stalled =
        !direct_motion_tail.empty() &&
        (history.empty() ||
         append_stall_age < 0.0 ||
         append_stall_age > cfg_.motion_tail_force_publish_stall ||
         last_history_action_ == "history_progress_accumulating" ||
         last_history_action_ == "history_progress_too_small" ||
         last_history_action_.find("reverse") != std::string::npos ||
         last_history_action_.find("kink") != std::string::npos);
    const bool prefer_direct_motion_tail =
        !direct_motion_tail.empty() && history.empty();
    if (prefer_direct_motion_tail) {
      std::vector<HistoryPoint2D> standalone = direct_motion_tail;
      recomputeHeadingStable(standalone,
                             cfg_.fixed_frame_heading_min_distance,
                             cfg_.fixed_frame_heading_hold_distance,
                             cfg_.fixed_frame_max_yaw_jump);
      if (debug != NULL) {
        debug->selected_segment_source = "motion_tail";
        debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(standalone));
        debug->motion_tail_used_as_reference = true;
        debug->motion_tail_promoted_to_reference = true;
        debug->motion_tail_direct_publish = true;
        debug->fixed_lag_tail_used = true;
        debug->tail_extension_latency = append_stall_age;
        debug->provisional_tail_block_detail = "none";
        debug->provisional_publish_block_reason = "none";
        debug->publisher_action = history.empty() ? "publish_motion_tail" : "force_motion_tail_stall";
        debug->reference_publish_forced_by_tail = append_stall_age > 0.50;
      }
      std::vector<HistoryPoint2D> smoothed = smoothReferenceSuffixUnlocked(standalone, debug);
      if (debug != NULL) {
        debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(smoothed));
      }
      return smoothed;
    }

    if (!direct_motion_tail.empty() && !history.empty()) {
      std::vector<HistoryPoint2D> combined = history;
      const ros::Time last_confirmed_stamp = combined.back().stamp;
      bool appended = false;
      for (size_t i = 0; i < direct_motion_tail.size(); ++i) {
        HistoryPoint2D pt = direct_motion_tail[i];
        const double spatial_gap =
            combined.empty()
                ? std::numeric_limits<double>::infinity()
                : pointDistance(combined.back().x, combined.back().y, pt.x, pt.y);
        if (last_confirmed_stamp.isValid() && pt.stamp.isValid() &&
            pt.stamp <= last_confirmed_stamp &&
            spatial_gap < std::max(0.005, cfg_.motion_tail_sample_min_distance)) {
          continue;
        }
        if (!combined.empty() &&
            spatial_gap < std::max(0.005, cfg_.motion_tail_sample_min_distance)) {
          continue;
        }
        pt.predicted = true;
        combined.push_back(pt);
        appended = true;
      }
      if (appended) {
        recomputeHeadingStable(combined,
                               cfg_.fixed_frame_heading_min_distance,
                               cfg_.fixed_frame_heading_hold_distance,
                               cfg_.fixed_frame_max_yaw_jump);
        if (debug != NULL) {
          debug->selected_segment_source = "confirmed_tail";
          debug->selected_segment_span = std::max(debug->selected_segment_span,
                                                  worldHistorySpan(combined));
          debug->confirmed_suffix_span = worldHistorySpan(history);
          debug->tail_suffix_span = worldHistorySpan(direct_motion_tail);
          debug->suffix_extension_span = debug->tail_suffix_span;
          if (debug->suffix_estimator_output_span <= 1e-6) {
            debug->suffix_estimator_output_span = debug->tail_suffix_span;
          }
          debug->reference_estimator_confidence = confirmed_append_stalled ? 0.65 : 0.80;
          debug->motion_tail_used_as_reference = true;
          debug->motion_tail_promoted_to_reference = true;
          debug->motion_tail_direct_publish = true;
          debug->fixed_lag_tail_used = true;
          debug->tail_extension_latency = append_stall_age;
          debug->provisional_tail_block_detail = "none";
          debug->provisional_publish_block_reason = "none";
          debug->publisher_action = "publish_confirmed_tail";
          debug->reference_publish_forced_by_tail = append_stall_age > 0.50 || confirmed_append_stalled;
        }
        std::vector<HistoryPoint2D> smoothed = smoothReferenceSuffixUnlocked(combined, debug);
        if (debug != NULL) {
          debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(smoothed));
        }
        return smoothed;
      }
      if (confirmed_append_stalled && history.empty()) {
        std::vector<HistoryPoint2D> standalone = direct_motion_tail;
        recomputeHeadingStable(standalone,
                               cfg_.fixed_frame_heading_min_distance,
                               cfg_.fixed_frame_heading_hold_distance,
                               cfg_.fixed_frame_max_yaw_jump);
        if (debug != NULL) {
          debug->selected_segment_source = "motion_tail";
          debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(standalone));
          debug->motion_tail_used_as_reference = true;
          debug->motion_tail_promoted_to_reference = true;
          debug->motion_tail_direct_publish = true;
          debug->fixed_lag_tail_used = true;
          debug->tail_extension_latency = append_stall_age;
          debug->provisional_tail_block_detail = "none";
          debug->provisional_publish_block_reason = "none";
          debug->publisher_action = "publish_tail_stalled_confirmed";
        }
        std::vector<HistoryPoint2D> smoothed = smoothReferenceSuffixUnlocked(standalone, debug);
        if (debug != NULL) {
          debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(smoothed));
        }
        return smoothed;
      }
    }

    std::vector<HistoryPoint2D> tail = stableTailSuffixUnlocked(debug);
    if (tail.empty()) {
      std::vector<HistoryPoint2D> smoothed = smoothReferenceSuffixUnlocked(history, debug);
      if (debug != NULL) {
        debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(smoothed));
      }
      return smoothed;
    }

    const bool prefer_motion_tail_standalone =
        debug != NULL &&
        debug->motion_tail_used_as_reference &&
        confirmed_trail_world_.size() <
            static_cast<size_t>(std::max(1, cfg_.provisional_tail_standalone_until_confirmed_points));
    if (prefer_motion_tail_standalone) {
      std::vector<HistoryPoint2D> standalone = tail;
      recomputeHeadingStable(standalone,
                             cfg_.fixed_frame_heading_min_distance,
                             cfg_.fixed_frame_heading_hold_distance,
                             cfg_.fixed_frame_max_yaw_jump);
      if (debug != NULL) {
        debug->selected_segment_source = "motion_tail";
        debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(standalone));
        debug->motion_tail_promoted_to_reference = true;
        debug->provisional_tail_block_detail = "none";
        debug->provisional_publish_block_reason = "none";
      }
      std::vector<HistoryPoint2D> smoothed = smoothReferenceSuffixUnlocked(standalone, debug);
      if (debug != NULL) {
        debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(smoothed));
      }
      return smoothed;
    }

    for (size_t i = 0; i < tail.size(); ++i) {
      HistoryPoint2D pt = tail[i];
      pt.predicted = true;
      if (!history.empty()) {
        const HistoryPoint2D& last = history.back();
        if (pt.stamp.isValid() && last.stamp.isValid() && pt.stamp <= last.stamp) {
          continue;
        }
        if (pointDistance(last.x, last.y, pt.x, pt.y) <
            std::max(0.005, cfg_.motion_tail_sample_min_distance)) {
          continue;
        }
      }
      history.push_back(pt);
    }

    history = removeNearDuplicatePoints(
        history, std::max(0.005, cfg_.motion_tail_sample_min_distance));
    recomputeHeadingStable(history,
                           cfg_.fixed_frame_heading_min_distance,
                           cfg_.fixed_frame_heading_hold_distance,
                           cfg_.fixed_frame_max_yaw_jump);
    if (debug != NULL) {
      debug->selected_segment_source = "confirmed_tail";
      debug->selected_segment_span = std::max(debug->selected_segment_span, worldHistorySpan(history));
      debug->suffix_extension_span = worldHistorySpan(tail);
      if (debug->suffix_estimator_output_span <= 1e-6) {
        debug->suffix_estimator_output_span = debug->suffix_extension_span;
      }
    }
    return smoothReferenceSuffixUnlocked(history, debug);
  }

  void updateTrackFromWindowUnlocked(TrackerDebugState* debug) {
    if (nodes_.empty()) {
      track_ = LeadState2D();
      moving_flag_ = false;
      if (debug != NULL) {
        debug->history_reason = "window_empty";
        debug->history_size = 0;
      }
      return;
    }

    const LeadNode& last = nodes_.back();
    track_.stamp = last.stamp;
    track_.x = last.x;
    track_.y = last.y;
    track_.heading = last.yaw;
    track_.active = true;
    track_.vx = last.v * std::cos(last.yaw);
    track_.vy = last.v * std::sin(last.yaw);

    double window_disp = 0.0;
    double window_speed = 0.0;
    double window_progress = 0.0;
    const double measurement_speed = meanFixedFrameSpeedUnlocked();
    const double static_measurement_residual = meanStaticHypothesisResidualUnlocked();
    double short_window_prediction_residual = 0.0;
    if (!nodes_.empty()) {
      for (size_t i = 0; i < nodes_.size(); ++i) {
        short_window_prediction_residual += nodes_[i].short_window_prediction_residual;
      }
      short_window_prediction_residual /= static_cast<double>(nodes_.size());
    }
    if (nodes_.size() >= 2) {
      window_disp = pointDistance(nodes_.front().x, nodes_.front().y, nodes_.back().x, nodes_.back().y);
      const double dt =
          std::max(1e-3, (nodes_.back().stamp - nodes_.front().stamp).toSec());
      window_speed = window_disp / dt;
      window_progress = estimateWindowProgressUnlocked() / dt;
    }
    const double current_speed = last.v;
    const double vote_speed =
        std::max(current_speed, std::max(measurement_speed, std::max(window_speed, window_progress)));
    const double static_confidence = estimateStaticConfidenceUnlocked();
    const double mean_ego_speed = meanEgoSpeedUnlocked();

    const double hs_score = staticHypothesisScoreUnlocked();
    const double hd_score = dynamicHypothesisScoreUnlocked();
    const bool conditioned_dynamic = egoMotionConditionedDynamicUnlocked();
    const double fixed_frame_progress = dynamicTrailProgressUnlocked();
    const bool static_anchor_project =
        std::min(static_measurement_residual, short_window_prediction_residual) <
            cfg_.static_anchor_res_gate &&
        fixed_frame_progress < cfg_.static_anchor_progress_gate &&
        vote_speed < cfg_.static_anchor_speed_gate &&
        last_innovation_norm_ < cfg_.static_residual_gate;
    const bool motion_suggests_dynamic =
        conditioned_dynamic ||
        vote_speed > cfg_.motion_static_suppress_speed ||
        fixed_frame_progress > cfg_.motion_static_suppress_progress ||
        std::max(static_measurement_residual, short_window_prediction_residual) >
            cfg_.motion_static_suppress_residual;
    const bool allow_static_anchor_project = static_anchor_project && !motion_suggests_dynamic;
    pushMotionTailPointUnlocked(nodeToHistoryPointUnlocked(last), motion_suggests_dynamic);

    if (vote_speed < cfg_.lead_static_speed_gate &&
        measurement_speed < cfg_.ego_compensated_static_gate &&
        std::min(static_measurement_residual, short_window_prediction_residual) <
            cfg_.ego_compensated_static_gate &&
        hs_score <= hd_score + cfg_.dual_hypothesis_score_margin &&
        static_confidence > 0.55 &&
        last_innovation_norm_ < cfg_.static_residual_gate) {
      static_votes_ = std::min(static_votes_ + 1, 1000);
    } else {
      static_votes_ = std::max(0, static_votes_ - 1);
    }

    if (vote_speed > cfg_.lead_moving_speed_gate &&
        measurement_speed > cfg_.ego_compensated_moving_gate &&
        static_measurement_residual > 0.5 * cfg_.ego_compensated_static_gate &&
        short_window_prediction_residual > 0.5 * cfg_.ego_compensated_static_gate &&
        hd_score + cfg_.dual_hypothesis_score_margin < hs_score &&
        static_confidence < 0.55 &&
        last_innovation_norm_ < cfg_.moving_residual_gate) {
      moving_votes_ = std::min(moving_votes_ + 1, 1000);
    } else if (conditioned_dynamic) {
      moving_votes_ = std::min(moving_votes_ + 1, 1000);
      static_votes_ = std::max(0, static_votes_ - 1);
    } else if (mean_ego_speed < cfg_.ego_static_speed_gate &&
               vote_speed > 0.5 * cfg_.lead_moving_speed_gate &&
               hd_score < hs_score &&
               dynamicTrailProgressUnlocked() > 0.5 * cfg_.confirm_min_progress) {
      moving_votes_ = std::min(moving_votes_ + 1, 1000);
    } else {
      moving_votes_ = std::max(0, moving_votes_ - 1);
    }

    if (!track_.static_mode && (static_votes_ >= cfg_.static_vote_on || allow_static_anchor_project)) {
      track_.static_mode = true;
      static_anchor_x_ = track_.x;
      static_anchor_y_ = track_.y;
      static_anchor_heading_ = track_.heading;
      static_anchor_stamp_ = last.stamp;
      has_static_anchor_ = true;
      moving_flag_ = false;
    }

    if (track_.static_mode && motion_suggests_dynamic) {
      track_.static_mode = false;
      moving_flag_ = true;
    }

    if (track_.static_mode) {
      const bool strong_static = hs_score + cfg_.static_release_score_margin < hd_score;
      if (strong_static || allow_static_anchor_project) {
        static_anchor_x_ = (1.0 - cfg_.anchor_alpha) * static_anchor_x_ + cfg_.anchor_alpha * track_.x;
        static_anchor_y_ = (1.0 - cfg_.anchor_alpha) * static_anchor_y_ + cfg_.anchor_alpha * track_.y;
        static_anchor_heading_ =
            wrapAngle((1.0 - cfg_.anchor_alpha) * static_anchor_heading_ + cfg_.anchor_alpha * track_.heading);
        static_anchor_stamp_ = last.stamp;
        has_static_anchor_ = true;
      }
      track_.x = static_anchor_x_;
      track_.y = static_anchor_y_;
      track_.vx = 0.0;
      track_.vy = 0.0;
      if (!allow_static_anchor_project &&
          ((static_votes_ <= cfg_.static_vote_release && moving_votes_ >= cfg_.moving_vote_on) ||
          conditioned_dynamic ||
          (hd_score + cfg_.static_release_score_margin < hs_score))) {
        track_.static_mode = false;
        moving_flag_ = true;
      }
    } else {
      moving_flag_ = moving_votes_ >= cfg_.moving_vote_on;
    }

    refreshTailBufferUnlocked();
    if (track_.static_mode && allow_static_anchor_project && !conditioned_dynamic) {
      resetUnconfirmedAccumulatorUnlocked();
      last_history_action_ = "static_anchor_project";
    } else {
      commitConfirmedTrailPointUnlocked();
    }

    if (debug != NULL) {
      debug->static_anchor_suppressed_by_motion = static_anchor_project && motion_suggests_dynamic;
      debug->static_anchor_blocked_by_motion_tail =
          debug->static_anchor_suppressed_by_motion && !motion_tail_buffer_world_.empty();
      debug->history_reason =
          (track_.static_mode && !debug->static_anchor_suppressed_by_motion)
              ? "static_anchor_update"
              : last_history_action_;
      debug->history_size = exportHistoryUnlocked().size();
      debug->ff_speed = measurement_speed;
      debug->mean_ego_speed = mean_ego_speed;
      debug->static_hyp_res = std::min(static_measurement_residual, short_window_prediction_residual);
      debug->hs_score = hs_score;
      debug->hd_score = hd_score;
      debug->accum_s = unconfirmed_progress_accum_;
      debug->accum_t = unconfirmed_time_accum_;
      debug->confirmed_count = confirmed_trail_world_.size();
      debug->tail_count = tail_buffer_world_.size();
      debug->motion_tail_points = static_cast<int>(motion_tail_buffer_world_.size());
      debug->motion_tail_span = worldHistorySpan(
          std::vector<HistoryPoint2D>(motion_tail_buffer_world_.begin(), motion_tail_buffer_world_.end()));
      debug->motion_tail_age =
          motion_tail_buffer_world_.size() >= 2 &&
                  motion_tail_buffer_world_.back().stamp.isValid() &&
                  motion_tail_buffer_world_.front().stamp.isValid()
              ? std::max(0.0,
                         (motion_tail_buffer_world_.back().stamp -
                          motion_tail_buffer_world_.front().stamp).toSec())
              : -1.0;
      debug->tail_sample_action = last_tail_sample_action_;
      debug->tail_pending_distance = last_tail_pending_distance_;
      debug->tail_pending_time = last_tail_pending_time_;
      debug->fixed_frame_append_valid = last_fixed_frame_append_valid_;
      debug->fixed_frame_progress = last_fixed_frame_progress_;
      debug->fixed_frame_kink_angle = last_fixed_frame_kink_angle_;
      debug->history_yaw_jump = last_history_yaw_jump_;
      debug->history_curvature_peak = last_history_curvature_peak_;
      debug->history_window_commit = last_history_action_ == "history_window_commit";
      debug->fixed_lag_commit = debug->history_window_commit;
      debug->fixed_lag_commit_span = debug->history_window_commit ? unconfirmed_progress_accum_ : 0.0;
      debug->confirmed_append_block_reason =
          (last_history_action_ == "history_appended" || debug->history_window_commit)
              ? "none"
              : last_history_action_;
      const ros::Time debug_now =
          last.stamp.isValid() ? last.stamp : (ros::Time::now());
      debug->append_stall_age = confirmedAppendStallAgeUnlocked(debug_now);
      debug->outlier_gate_reason =
          last_outlier_gate_reason_.empty() ? "none" : last_outlier_gate_reason_;
      if (debug->motion_tail_block_reason == "none" &&
          !last_motion_tail_block_reason_.empty()) {
        debug->motion_tail_block_reason = last_motion_tail_block_reason_;
      }
    }
  }

  Config cfg_;
  LeadState2D track_;
  std::deque<LeadNode> nodes_;
  std::deque<HistoryPoint2D> confirmed_trail_world_;
  std::deque<HistoryPoint2D> tail_buffer_world_;
  std::deque<HistoryPoint2D> motion_tail_buffer_world_;
  HistoryPoint2D motion_tail_pending_;
  bool motion_tail_pending_valid_ = false;
  std::string last_tail_sample_action_ = "none";
  double last_tail_pending_distance_ = 0.0;
  double last_tail_pending_time_ = 0.0;
  ros::Time last_confirmed_append_stamp_;
  int static_votes_ = 0;
  int moving_votes_ = 0;
  bool moving_flag_ = false;
  std::string last_history_action_ = "init";
  double static_anchor_x_ = 0.0;
  double static_anchor_y_ = 0.0;
  double static_anchor_heading_ = 0.0;
  bool has_static_anchor_ = false;
  ros::Time static_anchor_stamp_;
  std::string last_motion_tail_block_reason_ = "none";
  std::string last_outlier_gate_reason_ = "none";
  double last_innovation_norm_ = 0.0;
  double last_innovation_metric_ = 0.0;
  bool unconfirmed_accum_initialized_ = false;
  double unconfirmed_progress_accum_ = 0.0;
  double unconfirmed_time_accum_ = 0.0;
  double unconfirmed_last_x_ = 0.0;
  double unconfirmed_last_y_ = 0.0;
  double unconfirmed_last_heading_ = 0.0;
  ros::Time unconfirmed_last_stamp_;
  ros::Time unconfirmed_start_stamp_;
  bool last_fixed_frame_append_valid_ = false;
  double last_fixed_frame_progress_ = 0.0;
  double last_fixed_frame_kink_angle_ = 0.0;
  double last_history_yaw_jump_ = 0.0;
  double last_history_curvature_peak_ = 0.0;
  bool lateral_residual_filter_initialized_ = false;
  double lateral_residual_filtered_ = 0.0;
  double lateral_velocity_filtered_ = 0.0;
  ros::Time lateral_residual_filter_stamp_;
};

class PathGenerator {
 public:
  struct Config {
    double raw_resample_ds = 0.08;
    double raw_fit_ds = 0.06;
    int raw_spline_passes = 4;
    double raw_spline_alpha = 0.45;
    double raw_tail_weight_scale = 0.25;
    bool raw_preserve_history_geometry = true;
    double raw_duplicate_min_distance = 0.04;
    double raw_heading_min_distance = 0.12;
    double raw_heading_hold_distance = 0.08;
    double raw_max_heading_step = 0.70;
    double raw_max_yaw_jump = 1.00;
    double raw_max_segment_kappa = 5.00;
    double raw_backtrack_epsilon = 0.02;
    double raw_stable_segment_min_span = 0.60;
    int raw_stable_segment_min_points = 5;
    double lead_center_to_path_ref_offset = 0.0;
    int raw_min_points_for_publish = 2;
    double raw_min_span_for_publish = 0.08;
    double raw_keep_behind_x = 0.50;
    double raw_max_range = 30.0;
    int raw_max_points = 250;
    double reference_join_min_forward_x = 0.12;
    double reference_join_max_lateral = 3.0;
    double reference_join_lookahead = 0.35;
    double reference_join_skip_weight = 0.20;
    double reference_join_curvature_weight = 0.90;
    double reference_join_time_weight = 0.60;
    double reference_join_progress_weight = 0.25;
    double reference_join_s_window = 3.0;
    double reference_join_max_skip_span = 0.20;
    double reference_join_min_suffix_span_short = 0.25;
    double reference_join_min_suffix_span = 0.50;
    double reference_join_min_suffix_span_mature = 0.80;
    double reference_join_suffix_weight = 2.00;
    double reference_join_max_connector_ratio = 0.55;
    double reference_join_max_connector_ratio_bootstrap = 0.65;
    double reference_inner_cut_connector_ratio = 0.65;
    double reference_inner_cut_lateral_gate = 0.25;
    double reference_inner_cut_score_gate = 0.45;
    double reference_join_stability_weight = 0.85;
    double reference_join_stability_window = 0.45;
    double reference_min_suffix_preserve_span = 0.35;
    double reference_min_suffix_preserve_span_short = 0.18;
    int reference_min_points = 2;
    double reference_min_span = 0.25;
    double reference_direct_attach_dist = 0.18;
    double reference_connector_enable_dist = 0.15;
    double reference_connector_resolution = 0.08;
    double reference_connector_tangent_scale = 0.65;
    double reference_connector_curvature_scale = 1.0;
    double reference_splice_blend_length = 0.60;
    double reference_resample_ds = 0.08;
    int reference_sg_passes = 1;
    double suffix_smooth_wiggle_gate = 0.040;
    int suffix_smooth_sign_flip_gate = 2;
    double reference_health_wiggle_gate = 0.060;
    double reference_health_curvature_gate = 3.0;
    double reference_health_wiggle_penalty = 0.35;
    double reference_health_flip_penalty = 0.16;
    double reference_health_curvature_penalty = 0.12;
    double reference_health_s_wave_cap = 0.62;
    double reference_max_length = 20.0;
    bool reference_follow_raw_only = false;
    double reference_validation_front_s = 2.50;
    double reference_max_front_yaw_jump = 0.90;
    double reference_max_front_kappa = 8.0;
    double reference_backtrack_epsilon = 0.02;
    double reference_short_kink_distance = 0.04;
    double reference_short_kink_yaw = 0.50;
    int reference_max_join_retries = 4;
    bool reference_anchor_connector_enabled = false;
    int reference_mature_min_history_points = 8;
    int reference_mature_min_points = 8;
    double reference_mature_min_raw_span = 1.00;
    double reference_mature_min_span = 0.80;
    double reference_last_good_hold_timeout = 0.80;
    bool reference_degraded_raw_fallback_enabled = true;
    double reference_degraded_raw_min_span = 0.18;
    int reference_degraded_raw_min_points = 2;
    double reference_degraded_raw_max_start_distance = 0.85;
    double stable_segment_min_span = 0.18;
    int stable_segment_min_points = 2;
    double stable_segment_start_distance_weight = 1.20;
    bool reference_output_hysteresis_enabled = true;
    double reference_output_min_hold_time = 0.50;
    double reference_output_max_hold_time = 0.50;
    int reference_output_confirm_frames = 3;
    double reference_output_max_span_jump = 0.50;
    double reference_output_max_endpoint_jump = 0.35;
    double reference_output_max_start_jump = 0.25;
    double reference_output_max_heading_jump = 0.35;
    double reference_full_max_yaw_jump = 0.80;
    double reference_full_short_kink_yaw = 0.45;
    double reference_full_short_kink_distance = 0.05;
    double reference_full_max_kappa = 5.0;
    double reference_output_span_growth_per_sec = 1.8;
    double reference_output_span_shrink_per_sec = 1.2;
    double reference_output_min_span_growth = 0.12;
    double reference_state_update_alpha = 0.65;
    double reference_output_min_publish_span = 0.80;
    int reference_output_min_publish_points = 8;
    double reference_stale_hold_time = 0.60;
    double reference_dynamic_shrink_min_span = 0.08;
    int reference_dynamic_shrink_min_points = 3;
  };

  explicit PathGenerator(const Config& cfg) : cfg_(cfg) {}

  std::vector<RawTrailPoint> buildRawTrail(const std::vector<HistoryPoint2D>& world_history,
                                           TrackerDebugState* debug) const {
    if (world_history.empty()) {
      if (debug != NULL) {
        debug->raw_reason = "history_empty";
        debug->raw_span = 0.0;
      }
      return {};
    }

    std::vector<HistoryPoint2D> world_pts =
        removeNearDuplicatePoints(world_history, std::max(0.005, cfg_.raw_duplicate_min_distance));
    if (world_pts.size() >= 2) {
      recomputeHeadingStable(world_pts,
                             cfg_.raw_heading_min_distance,
                             cfg_.raw_heading_hold_distance,
                             cfg_.raw_max_heading_step);
      if (!cfg_.raw_preserve_history_geometry) {
        world_pts =
            smoothWorldSplineLike(world_pts, cfg_.raw_spline_passes, cfg_.raw_spline_alpha, cfg_.raw_tail_weight_scale);
        world_pts = fitWorldTrajectoryHermite(world_pts, cfg_.raw_fit_ds);
      }
      world_pts = resampleWorldByArcLength(world_pts, cfg_.raw_resample_ds);
      recomputeHeadingStable(world_pts,
                             cfg_.raw_heading_min_distance,
                             cfg_.raw_heading_hold_distance,
                             cfg_.raw_max_heading_step);
      world_pts = selectStableWorldSegment(world_pts, debug);
      if (world_pts.size() >= 2) {
        recomputeHeadingStable(world_pts,
                               cfg_.raw_heading_min_distance,
                               cfg_.raw_heading_hold_distance,
                               cfg_.raw_max_heading_step);
      }
    }
    offsetWorldPathAlongHeading(&world_pts, cfg_.lead_center_to_path_ref_offset);
    const std::vector<RawTrailPoint> raw_trail = makeRawTrailPoints(world_pts);
    if (debug != NULL) {
      const std::string prior_raw_reason = debug->raw_reason;
      debug->raw_span = raw_trail.empty() ? 0.0 : raw_trail.back().s;
      debug->raw_reason =
          raw_trail.empty()
              ? (world_history.empty() ? "history_empty" : "raw_no_stable_segment")
              : (prior_raw_reason == "raw_relaxed_stable_segment"
                     ? "raw_relaxed_stable_segment"
                     : "raw_ready");
    }
    return raw_trail;
  }

  std::vector<HistoryPoint2D> selectStableWorldSegment(const std::vector<HistoryPoint2D>& pts,
                                                       TrackerDebugState* debug) const {
    if (static_cast<int>(pts.size()) < std::max(2, cfg_.raw_stable_segment_min_points)) {
      return pts;
    }

    struct Segment {
      size_t begin = 0;
      size_t end = 0;
      double span = 0.0;
      double score = -1e9;
    };

    std::vector<Segment> segments;
    std::vector<Segment> relaxed_segments;
    size_t begin = 0;
    auto finish_segment = [&](size_t end) {
      if (end <= begin || end >= pts.size()) {
        return;
      }
      double span = 0.0;
      double max_yaw_jump = 0.0;
      double max_kappa = 0.0;
      double backtrack_penalty = 0.0;
      for (size_t i = begin + 1; i <= end; ++i) {
        const HistoryPoint2D& a = pts[i - 1];
        const HistoryPoint2D& b = pts[i];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double ds = hypot2(dx, dy);
        if (ds < 1e-5) {
          continue;
        }
        const double yaw = std::atan2(dy, dx);
        const double dyaw = i > begin + 1
                                ? std::fabs(wrapAngle(yaw - std::atan2(pts[i - 1].y - pts[i - 2].y,
                                                                        pts[i - 1].x - pts[i - 2].x)))
                                : 0.0;
        const double progress = dx * std::cos(a.heading) + dy * std::sin(a.heading);
        max_yaw_jump = std::max(max_yaw_jump, dyaw);
        max_kappa = std::max(max_kappa, dyaw / std::max(1e-4, ds));
        if (progress < 0.0) {
          backtrack_penalty += std::fabs(progress);
        }
        span += ds;
      }
      const int count = static_cast<int>(end - begin + 1);
      if (count < 2 || span < cfg_.raw_min_span_for_publish) {
        return;
      }
      Segment s;
      s.begin = begin;
      s.end = end;
      s.span = span;
      s.score = span - 0.20 * max_kappa - 0.25 * max_yaw_jump - 2.0 * backtrack_penalty;
      relaxed_segments.push_back(s);
      if (count >= cfg_.raw_stable_segment_min_points &&
          span >= cfg_.raw_stable_segment_min_span) {
        segments.push_back(s);
      }
    };

    for (size_t i = 1; i < pts.size(); ++i) {
      const HistoryPoint2D& a = pts[i - 1];
      const HistoryPoint2D& b = pts[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double ds = hypot2(dx, dy);
      if (ds < 1e-5) {
        continue;
      }
      const double progress = dx * std::cos(a.heading) + dy * std::sin(a.heading);
      double dyaw = std::fabs(wrapAngle(b.heading - a.heading));
      if (i >= 2) {
        const double prev_yaw = std::atan2(a.y - pts[i - 2].y, a.x - pts[i - 2].x);
        const double seg_yaw = std::atan2(dy, dx);
        dyaw = std::max(dyaw, std::fabs(wrapAngle(seg_yaw - prev_yaw)));
      }
      const double kappa = dyaw / std::max(1e-4, ds);
      const bool cut = progress < -cfg_.raw_backtrack_epsilon ||
                       dyaw > cfg_.raw_max_yaw_jump ||
                       kappa > cfg_.raw_max_segment_kappa;
      if (cut) {
        finish_segment(i - 1);
        begin = i;
      }
    }
    finish_segment(pts.size() - 1);

    if (debug != NULL) {
      debug->stable_segment_count = std::max(debug->stable_segment_count,
                                             static_cast<int>(segments.size()));
    }
    const std::vector<Segment>& candidates = segments.empty() ? relaxed_segments : segments;
    if (candidates.empty()) {
      if (debug != NULL) {
        debug->raw_reason = "raw_no_stable_segment";
      }
      return {};
    }
    const Segment* best = &candidates.front();
    for (size_t i = 1; i < candidates.size(); ++i) {
      if (candidates[i].score > best->score) {
        best = &candidates[i];
      }
    }
    std::vector<HistoryPoint2D> out(pts.begin() + static_cast<long>(best->begin),
                                    pts.begin() + static_cast<long>(best->end + 1));
    if (debug != NULL) {
      debug->selected_segment_span = std::max(debug->selected_segment_span, best->span);
      debug->selected_segment_score = std::max(debug->selected_segment_score, best->score);
      debug->selected_segment_source =
          debug->selected_segment_source == "none"
              ? (segments.empty() ? "raw_relaxed_stable" : "raw_stable")
              : debug->selected_segment_source;
      if (segments.empty()) {
        debug->raw_reason = "raw_relaxed_stable_segment";
      }
    }
    return out;
  }

  std::vector<LocalPathPoint> buildRawPath(const std::vector<HistoryPoint2D>& world_history,
                                           const EgoPose2D& ego_now,
                                           TrackerDebugState* debug) const {
    const std::vector<RawTrailPoint> raw_trail = buildRawTrail(world_history, debug);
    if (raw_trail.empty()) {
      return {};
    }

    std::vector<HistoryPoint2D> fitted_world;
    fitted_world.reserve(raw_trail.size());
    for (size_t i = 0; i < raw_trail.size(); ++i) {
      HistoryPoint2D pt;
      pt.stamp = raw_trail[i].stamp;
      pt.x = raw_trail[i].x;
      pt.y = raw_trail[i].y;
      pt.heading = raw_trail[i].yaw;
      pt.curvature = raw_trail[i].kappa;
      fitted_world.push_back(pt);
    }
    std::vector<LocalPathPoint> local = transformWorldPathToLocal(fitted_world, ego_now);
    std::vector<LocalPathPoint> trimmed;
    trimmed.reserve(local.size());
    for (size_t i = 0; i < local.size(); ++i) {
      if (local[i].x < -cfg_.raw_keep_behind_x) {
        continue;
      }
      if (hypot2(local[i].x, local[i].y) > cfg_.raw_max_range) {
        continue;
      }
      trimmed.push_back(local[i]);
      trimmed.back().curvature = raw_trail[i].kappa;
      if (static_cast<int>(trimmed.size()) >= cfg_.raw_max_points) {
        break;
      }
    }

    if (trimmed.size() >= 2) {
      recomputeLocalHeading(trimmed);
    }
    const double span = polylineLengthLocal(trimmed);
    if (debug != NULL) {
      debug->raw_span = span;
    }
    if (static_cast<int>(trimmed.size()) < cfg_.raw_min_points_for_publish) {
      if (debug != NULL) {
        debug->raw_reason = "raw_insufficient_points";
      }
      return {};
    }
    if (span < cfg_.raw_min_span_for_publish) {
      if (debug != NULL) {
        debug->raw_reason = "raw_span_too_short";
      }
      return {};
    }
    if (debug != NULL) {
      debug->raw_reason = "raw_ready";
    }
    return trimmed;
  }

  std::vector<LocalPathPoint> buildReferencePath(const std::vector<RawTrailPoint>& raw_trail,
                                                 const std::vector<LocalPathPoint>& raw,
                                                 const EgoPose2D& ego_now,
                                                 TrackerDebugState* debug) const {
    current_candidate_source_stamp_ = ros::Time();
    for (size_t i = 0; i < raw_trail.size(); ++i) {
      if (raw_trail[i].stamp.isValid() &&
          (!current_candidate_source_stamp_.isValid() ||
           raw_trail[i].stamp > current_candidate_source_stamp_)) {
        current_candidate_source_stamp_ = raw_trail[i].stamp;
      }
    }
    if (debug != NULL) {
      debug->stable_segment_count = 0;
      debug->selected_segment_span = 0.0;
      debug->selected_segment_score = 0.0;
      debug->reference_hold_age = -1.0;
      debug->reference_publish_mode = "empty";
    }
    if (raw_trail.empty()) {
      if (debug != NULL) {
        debug->reference_reason = "raw_empty";
        debug->reference_span = 0.0;
        debug->reference_publish_mode = "empty_raw";
      }
      return {};
    }

    const double raw_span = polylineLengthLocal(raw);
    if (debug != NULL) {
      debug->raw_direct_publish = false;
      debug->reference_source = "none";
    }

    std::vector<RefPoint> ref_profile = makeRefPointsFromRawTrail(raw_trail, ego_now, 0.5);
    recomputeRefGeometry(&ref_profile);
    if (ref_profile.empty()) {
      if (debug != NULL) {
        debug->reference_reason = "reference_profile_too_short";
        debug->reference_span = 0.0;
        debug->reference_publish_mode = "empty_profile";
      }
      return {};
    }
    const bool provisional_tail_source =
        debug != NULL &&
        (debug->selected_segment_source == "motion_tail" ||
         debug->selected_segment_source == "confirmed_tail" ||
         debug->motion_tail_promoted_to_reference ||
         debug->motion_tail_direct_publish);

    if (raw.empty() && ref_profile.size() == 1) {
      std::vector<LocalPathPoint> held = heldReferenceLocal(ego_now);
      if (!held.empty()) {
        if (debug != NULL) {
          debug->reference_reason = "reference_hold_last_good_anchor_suppressed";
          debug->reference_span = polylineLengthLocal(held);
          debug->reference_hold_age = heldReferenceAge(ego_now);
          debug->reference_publish_mode = "held_last_good";
        }
        return held;
      }
      if (!cfg_.reference_anchor_connector_enabled) {
        if (debug != NULL) {
          debug->reference_reason = "reference_anchor_connector_suppressed";
          debug->reference_span = 0.0;
          debug->reference_publish_mode = "empty_anchor_suppressed";
        }
        return {};
      }
      std::vector<RefPoint> anchor_ref;
      RefPoint origin;
      origin.v_ref = std::min(0.3, std::max(0.10, ref_profile.front().v_ref));
      origin.a_ref = 0.0;
      anchor_ref.push_back(origin);
      const std::vector<RefPoint> connector = buildConnector(ref_profile.front());
      if (connector.empty()) {
        anchor_ref.push_back(ref_profile.front());
      } else {
        for (size_t i = 1; i < connector.size(); ++i) {
          anchor_ref.push_back(connector[i]);
        }
      }
      recomputeRefGeometry(&anchor_ref);
      std::vector<LocalPathPoint> final_local = makeLocalPathFromRefPoints(anchor_ref);
      if (debug != NULL) {
        debug->reference_reason = "reference_anchor_connector";
        debug->reference_span = polylineLengthLocal(final_local);
        debug->reference_publish_mode = "anchor_connector";
      }
      return final_local;
    }

    if (static_cast<int>(raw.size()) < cfg_.reference_min_points && ref_profile.size() < 2) {
      if (debug != NULL) {
        debug->reference_reason = "reference_insufficient_points";
        debug->reference_span = raw_span;
        debug->reference_publish_mode = "empty_insufficient";
      }
      return {};
    }
    if (!raw.empty() && raw_span < cfg_.reference_min_span && ref_profile.size() < 2) {
      if (debug != NULL) {
        debug->reference_reason = "reference_span_too_short";
        debug->reference_span = raw_span;
        debug->reference_publish_mode = "empty_span_short";
      }
      return {};
    }

    const double min_suffix_after_join =
        raw_span >= cfg_.reference_mature_min_raw_span
            ? cfg_.reference_join_min_suffix_span_mature
            : (raw_span >= cfg_.reference_join_min_suffix_span
                   ? cfg_.reference_join_min_suffix_span
                   : cfg_.reference_join_min_suffix_span_short);
    ReferenceValidation validation;
    std::vector<RefPoint> best_combined;
    JoinChoice best_join;
    std::string reject_reason = "reference_join_invalid";
    const JoinChoice candidate_join = chooseJoinIndex(ref_profile, min_suffix_after_join);
    if (candidate_join.valid) {
      const double suffix_after_join =
          remainingSuffixSpan(ref_profile, candidate_join.join_point.s);
      const double available_at_cursor =
          remainingSuffixSpan(ref_profile, candidate_join.projected_s);
      const double effective_min_suffix =
          std::min(min_suffix_after_join, available_at_cursor);
      if (debug != NULL) {
        debug->suffix_after_join_span = suffix_after_join;
        debug->join_skip_span =
            std::max(0.0, candidate_join.join_point.s - candidate_join.projected_s);
        debug->reference_join_retry_count = 0;
        debug->publisher_retry_join_count = 0;
        debug->anti_inner_cut_rejoin = false;
      }
      if (suffix_after_join + 1e-6 < effective_min_suffix) {
        reject_reason = "reference_suffix_span_short";
        if (debug != NULL) {
          debug->join_reject_reason = reject_reason;
        }
      } else {
        std::vector<RefPoint> combined =
            buildReferenceCandidate(ref_profile, candidate_join, &validation);
        if (debug != NULL) {
          debug->reference_splice_kappa_max = validation.kappa_max;
          debug->reference_splice_yaw_jump = validation.max_yaw_jump;
          debug->reference_backtrack_reject = validation.backtrack;
        }
        if (validation.ok) {
          const std::vector<LocalPathPoint> combined_local =
              makeLocalPathFromRefPoints(combined);
          const ReferenceGeometryAudit full_audit =
              auditLocalReferenceFull(combined_local);
          if (debug != NULL) {
            debug->candidate_full_span_before_audit =
                polylineLengthLocal(combined_local);
            debug->tangent_progress_min = full_audit.tangent_progress_min;
            debug->reference_full_max_yaw_jump = full_audit.max_yaw_jump;
            debug->reference_full_backtrack_count = full_audit.backtrack_count;
            debug->reference_full_kappa_max = full_audit.kappa_max;
          }
          bool preserve_ok = true;
          if (!full_audit.ok && full_audit.valid_prefix_points >= 2 &&
              full_audit.valid_prefix_points <= combined_local.size()) {
            std::vector<LocalPathPoint> prefix(
                combined_local.begin(),
                combined_local.begin() + full_audit.valid_prefix_points);
            const double prefix_span = polylineLengthLocal(prefix);
            const double connector_span =
                hypot2(candidate_join.join_point.x, candidate_join.join_point.y);
            const double preserved_suffix =
                std::max(0.0, prefix_span - connector_span);
            const double min_preserved_suffix =
                suffix_after_join >= cfg_.reference_join_min_suffix_span
                    ? cfg_.reference_min_suffix_preserve_span
                    : cfg_.reference_min_suffix_preserve_span_short;
            preserve_ok = preserved_suffix + 1e-6 >= min_preserved_suffix;
            if (debug != NULL) {
              debug->splice_preserved_suffix_span = preserved_suffix;
              debug->candidate_full_span_after_audit = prefix_span;
              debug->splice_trim_rejected_connector_only = !preserve_ok;
            }
          }
          if (preserve_ok) {
            best_combined.swap(combined);
            best_join = candidate_join;
          } else {
            reject_reason = "reference_trim_connector_only";
          }
        } else {
          reject_reason = validation.reason;
        }
      }
    } else if (debug != NULL) {
      debug->join_reject_reason = "join_constraints";
    }

    if (best_combined.empty()) {
      std::vector<LocalPathPoint> held = heldReferenceLocal(ego_now);
      if (!held.empty()) {
        if (debug != NULL) {
          debug->reference_reason = "reference_hold_last_good_" + reject_reason;
          debug->reference_span = polylineLengthLocal(held);
          debug->reference_hold_age = heldReferenceAge(ego_now);
          debug->reference_publish_mode = "held_last_good";
        }
        return held;
      }
      if (debug != NULL) {
        debug->reference_reason = reject_reason;
        debug->reference_span = 0.0;
        debug->reference_publish_mode = "empty_reject";
      }
      return {};
    }

    std::vector<LocalPathPoint> final_local = makeLocalPathFromRefPoints(best_combined);
    const double final_span = polylineLengthLocal(final_local);
	    if (debug != NULL) {
	      debug->connector_used = best_join.valid && hypot2(best_join.join_point.x, best_join.join_point.y) >
	                                  cfg_.reference_direct_attach_dist;
      debug->connector_join_index =
          best_join.valid ? static_cast<int>(best_join.suffix_index) : -1;
      debug->connector_span = best_join.valid ? hypot2(best_join.join_point.x, best_join.join_point.y) : 0.0;
      debug->stitched_span = final_span;
      debug->suffix_span = best_join.valid ? remainingSuffixSpan(ref_profile, best_join.join_point.s) : 0.0;
      debug->suffix_after_join_span = debug->suffix_span;
      debug->join_connector_ratio =
          debug->connector_span / std::max(0.10, debug->connector_span + debug->suffix_span);
      debug->final_connector_ratio = debug->join_connector_ratio;
	      debug->join_index_smooth = best_join.valid ? best_join.join_point.s : -1.0;
	      debug->join_index_jump =
	          (best_join.valid && last_join_s_valid_)
	              ? std::fabs(best_join.join_point.s - last_join_s_)
	              : 0.0;
	      debug->join_state_s = best_join.valid ? best_join.join_point.s : -1.0;
	      debug->join_reselect_reason =
	          !best_join.valid ? "invalid" :
	          (!last_join_s_valid_ ? "init" :
	           (debug->join_index_jump <= std::max(0.05, cfg_.reference_join_stability_window)
	                ? "stable_neighborhood"
	                : "reselected"));
	      debug->publisher_retry_join_count = debug->reference_join_retry_count;
	      if (debug->reference_estimator_confidence <= 1e-6) {
        const bool mature_input =
            static_cast<int>(raw_trail.size()) >= cfg_.reference_mature_min_history_points &&
            raw_span >= cfg_.reference_mature_min_raw_span;
        debug->reference_estimator_confidence =
            provisional_tail_source ? 0.65 : (mature_input ? 1.0 : 0.75);
      }
    }
	    const bool mature =
        static_cast<int>(raw_trail.size()) >= cfg_.reference_mature_min_history_points &&
        static_cast<int>(best_combined.size()) >= cfg_.reference_mature_min_points &&
        raw_span >= cfg_.reference_mature_min_raw_span &&
        final_span >= cfg_.reference_mature_min_span;
	    if (!mature) {
      if (final_local.size() >= static_cast<size_t>(std::max(2, cfg_.reference_degraded_raw_min_points)) &&
          final_span >= cfg_.reference_degraded_raw_min_span) {
        if (final_span >= std::max(0.35, cfg_.reference_degraded_raw_min_span)) {
          rememberLastGoodSuffix(ref_profile, best_join, ego_now);
        }
	        if (debug != NULL) {
          debug->reference_reason =
              provisional_tail_source ? "reference_stitched_tail" : "reference_stitched_provisional";
          debug->reference_span = final_span;
          debug->reference_publish_mode = "provisional_tail";
          debug->reference_trackable_raw = final_span >= std::max(0.60, cfg_.reference_degraded_raw_min_span);
	          debug->reference_source =
	              provisional_tail_source ? "stitched_tail" : "stitched_confirmed";
	          debug->bootstrap_stitched =
	              final_span < std::max(0.80, cfg_.reference_mature_min_span) ||
	              raw_span < cfg_.reference_mature_min_raw_span;
	          debug->selected_segment_source =
              provisional_tail_source ? debug->selected_segment_source : "stitched_confirmed";
          debug->motion_tail_used_as_reference =
              debug->motion_tail_used_as_reference || provisional_tail_source;
          debug->motion_tail_promoted_to_reference =
              debug->motion_tail_promoted_to_reference || provisional_tail_source;
	        }
	        if (best_join.valid) {
	          last_join_s_ = best_join.join_point.s;
	          last_join_index_ = static_cast<int>(best_join.suffix_index);
	          last_join_s_valid_ = true;
	        }
	        return final_local;
	      }
      std::vector<LocalPathPoint> held = heldReferenceLocal(ego_now);
      if (!held.empty()) {
        if (debug != NULL) {
          debug->reference_reason = "reference_hold_last_good_immature";
          debug->reference_span = polylineLengthLocal(held);
          debug->reference_hold_age = heldReferenceAge(ego_now);
          debug->reference_publish_mode = "held_last_good";
        }
        return held;
      }
      if (debug != NULL) {
        debug->reference_reason = "reference_immature";
        debug->reference_span = final_span;
        debug->reference_publish_mode = "empty_immature";
      }
      return {};
    }
	    rememberLastGoodSuffix(ref_profile, best_join, ego_now);
	    if (best_join.valid) {
	      last_join_s_ = best_join.join_point.s;
	      last_join_index_ = static_cast<int>(best_join.suffix_index);
	      last_join_s_valid_ = true;
	    }
    if (debug != NULL) {
      debug->reference_reason = "reference_ready";
      debug->reference_span = final_span;
      debug->reference_publish_mode = "ready";
      debug->reference_source = provisional_tail_source ? "stitched_tail" : "stitched_confirmed";
    }

    return final_local;
  }

  bool stitchedDebugContext(const TrackerDebugState* debug) const {
    if (debug == NULL) {
      return false;
    }
    return debug->reference_source == "stitched_tail" ||
           debug->reference_source == "stitched_confirmed" ||
           debug->reference_reason == "reference_stitched_tail" ||
           debug->reference_reason == "reference_stitched_provisional" ||
           debug->selected_segment_source == "confirmed_tail" ||
           debug->selected_segment_source == "stitched_confirmed";
  }

  double minRequiredPreservedSuffix(const TrackerDebugState* debug) const {
    if (debug == NULL) {
      return cfg_.reference_min_suffix_preserve_span;
    }
    return debug->suffix_span >= cfg_.reference_join_min_suffix_span
               ? cfg_.reference_min_suffix_preserve_span
               : cfg_.reference_min_suffix_preserve_span_short;
  }

  bool stitchedSuffixPreserved(const std::vector<LocalPathPoint>& pts,
                               TrackerDebugState* debug) const {
    if (!stitchedDebugContext(debug)) {
      return true;
    }
    const double span = polylineLengthLocal(pts);
    const double preserved_suffix = std::max(0.0, span - std::max(0.0, debug->connector_span));
    const double min_preserved_suffix = minRequiredPreservedSuffix(debug);
    debug->splice_preserved_suffix_span = preserved_suffix;
    debug->published_suffix_preserved_ok = preserved_suffix + 1e-6 >= min_preserved_suffix;
    if (!debug->published_suffix_preserved_ok) {
      debug->splice_trim_rejected_connector_only = true;
      debug->connector_only_publish_blocked = true;
      debug->reference_reject_full_reason = "reference_trim_connector_only";
      debug->join_reject_reason = "reference_trim_connector_only";
    }
    return debug->published_suffix_preserved_ok;
  }

  std::vector<LocalPathPoint> stabilizeReferenceOutput(
      const std::vector<LocalPathPoint>& candidate,
      const EgoPose2D& ego_now,
      TrackerDebugState* debug) const {
    const ros::Time now = ego_now.stamp.isValid() ? ego_now.stamp : ros::Time::now();
    if (!cfg_.reference_output_hysteresis_enabled) {
      const std::string mode =
          debug != NULL && !debug->reference_publish_mode.empty()
              ? debug->reference_publish_mode
              : std::string("unknown");
      const std::string source =
          debug != NULL && !debug->selected_segment_source.empty()
              ? debug->selected_segment_source
              : std::string("none");
      std::vector<LocalPathPoint> audited = auditAndTrimLocalReference(candidate, debug, true);
      if (debug != NULL) {
        debug->candidate_reference_span = polylineLengthLocal(audited);
      }
      if (audited.empty()) {
        return {};
      }
      acceptPublishedReference(localReferenceToWorld(audited, ego_now), now, mode, source, debug);
      return audited;
    }

    if (debug != NULL) {
      debug->candidate_reference_span = polylineLengthLocal(candidate);
      debug->held_by_hysteresis = false;
      debug->reference_output_growth_limited = false;
      debug->published_suffix_preserved_ok = true;
      debug->connector_only_publish_blocked = false;
      debug->reference_refresh_latency = -1.0;
      debug->stale_hold_block_reason = "none";
      debug->suffix_extension_span = 0.0;
      debug->suffix_extension_growth_limited = false;
      debug->reconnected_last_good_suffix = false;
      if (debug->join_connector_ratio < 0.0 || debug->join_connector_ratio > 1.0) {
        debug->join_connector_ratio = 0.0;
      }
      debug->candidate_source = "none";
      debug->reference_empty_reason_final = "none";
      debug->published_span_target = 0.0;
      debug->publisher_action = "candidate";
      debug->published_hold_age = -1.0;
      debug->candidate_jump_to_published = 0.0;
      debug->published_full_valid = false;
      debug->published_reference_mode =
          last_published_reference_mode_.empty() ? debug->reference_publish_mode
                                                 : last_published_reference_mode_;
      debug->published_reference_reason = debug->reference_reason;
      debug->hold_forced_release = false;
      debug->shrink_limited_publish = false;
      debug->candidate_vs_published_span_diff = 0.0;
    }
    if (candidate.empty()) {
      std::vector<LocalPathPoint> held = publishedReferenceLocal(ego_now, false);
      held = auditAndTrimLocalReference(held, debug, false);
      if (!held.empty()) {
        const ReferenceGeometryAudit held_audit = auditLocalReferenceFull(held);
        if (debug != NULL) {
          debug->reference_reason = "reference_hold_output_empty";
          debug->reference_span = polylineLengthLocal(held);
          debug->published_reference_span = debug->reference_span;
          debug->published_reference_mode =
              last_published_reference_mode_.empty() ? "held_last_good" : last_published_reference_mode_;
          debug->published_reference_reason =
              last_published_reference_reason_.empty() ? "reference_hold_output_empty"
                                                       : last_published_reference_reason_;
          debug->held_by_hysteresis = false;
          debug->reference_hold_age =
              last_published_reference_stamp_.isZero()
                  ? -1.0
                  : std::max(0.0, (now - last_published_reference_stamp_).toSec());
          debug->reference_publish_mode =
              last_published_reference_mode_.empty() ? "held_last_good" : last_published_reference_mode_;
          debug->published_full_valid = held_audit.ok;
          debug->reference_full_max_yaw_jump = held_audit.max_yaw_jump;
          debug->reference_full_backtrack_count = held_audit.backtrack_count;
          debug->reference_full_kappa_max = held_audit.kappa_max;
          debug->published_span_target = debug->reference_span;
          debug->publisher_action = "hold_empty_candidate";
          debug->published_hold_age = debug->reference_hold_age;
          debug->reconnected_last_good_suffix = true;
          if (!last_published_reference_source_.empty()) {
            debug->selected_segment_source = last_published_reference_source_;
          }
        }
        return held;
      }
      if (debug != NULL) {
        debug->reference_empty_reason_final = "empty_candidate_no_last_good";
      }
      return candidate;
    }

    std::vector<LocalPathPoint> audited_candidate =
        auditAndTrimLocalReference(candidate, debug, true);
    if (debug != NULL) {
      debug->candidate_reference_span = polylineLengthLocal(audited_candidate);
    }
    if (audited_candidate.empty()) {
      std::vector<LocalPathPoint> held = publishedReferenceLocal(ego_now, false);
      held = auditAndTrimLocalReference(held, debug, false);
      if (!held.empty()) {
        const ReferenceGeometryAudit held_audit = auditLocalReferenceFull(held);
        if (debug != NULL) {
          debug->reference_reason = "reference_hold_output_full_reject";
          debug->reference_span = polylineLengthLocal(held);
          debug->published_reference_span = debug->reference_span;
          debug->published_reference_mode =
              last_published_reference_mode_.empty() ? "held_last_good" : last_published_reference_mode_;
          debug->published_reference_reason =
              last_published_reference_reason_.empty() ? "reference_hold_output_full_reject"
                                                       : last_published_reference_reason_;
          debug->held_by_hysteresis = true;
          debug->published_full_valid = held_audit.ok;
          debug->reference_full_max_yaw_jump = held_audit.max_yaw_jump;
          debug->reference_full_backtrack_count = held_audit.backtrack_count;
          debug->reference_full_kappa_max = held_audit.kappa_max;
          debug->published_span_target = debug->reference_span;
          debug->publisher_action = "hold_full_reject";
          debug->reconnected_last_good_suffix = true;
          debug->published_hold_age =
              last_published_reference_stamp_.isZero()
                  ? -1.0
                  : std::max(0.0, (now - last_published_reference_stamp_).toSec());
          if (!last_published_reference_source_.empty()) {
            debug->selected_segment_source = last_published_reference_source_;
          }
        }
        return held;
      }
      if (debug != NULL) {
        debug->reference_empty_reason_final = "full_reject_no_last_good";
      }
      return {};
    }

    std::vector<LocalPathPoint> output_candidate = audited_candidate;
    const double candidate_span = polylineLengthLocal(output_candidate);
    const std::string candidate_mode =
        debug != NULL && !debug->reference_publish_mode.empty()
            ? debug->reference_publish_mode
            : std::string("unknown");
    const std::string candidate_source =
        debug != NULL && !debug->selected_segment_source.empty()
            ? debug->selected_segment_source
            : std::string("none");
    if (debug != NULL) {
      debug->candidate_source = candidate_source;
    }
    const bool dynamic_candidate =
        candidate_mode == "provisional_tail" ||
        candidate_mode == "degraded_raw" ||
        (debug != NULL &&
         (debug->reference_reason == "reference_stitched_tail" ||
          debug->reference_reason == "reference_stitched_provisional")) ||
        candidate_source == "motion_tail" ||
        candidate_source == "confirmed_tail" ||
        candidate_source == "stitched_confirmed" ||
        (debug != NULL &&
         (debug->motion_tail_direct_publish ||
          debug->motion_tail_promoted_to_reference ||
          debug->motion_tail_hard_valid));

    if (last_published_reference_world_.empty()) {
      if (debug != NULL) {
        debug->publisher_action = "accept_initial";
      }
      acceptPublishedReference(localReferenceToWorld(output_candidate, ego_now),
                               now,
                               candidate_mode,
                               candidate_source,
                               debug);
      return output_candidate;
    }

    const double published_age =
        last_published_reference_stamp_.isZero()
            ? std::numeric_limits<double>::infinity()
            : std::max(0.0, (now - last_published_reference_stamp_).toSec());
    if (debug != NULL) {
      debug->published_hold_age = std::isfinite(published_age) ? published_age : -1.0;
    }

    if (dynamic_candidate) {
      std::vector<LocalPathPoint> smoothed_candidate = output_candidate;
      std::vector<LocalPathPoint> previous_local = publishedReferenceLocal(ego_now, false);
      if (!previous_local.empty()) {
        const double full_candidate_span = polylineLengthLocal(output_candidate);
        smoothed_candidate =
            smoothReferenceUpdate(output_candidate, previous_local, cfg_.reference_state_update_alpha);
        smoothed_candidate = auditAndTrimLocalReference(smoothed_candidate, debug, true);
        if (smoothed_candidate.empty() && !output_candidate.empty()) {
          smoothed_candidate = output_candidate;
          if (debug != NULL) {
            debug->published_update_mode = "smooth_bypass_candidate";
            debug->reference_reject_full_reason = "smooth_trim_rejected_candidate";
            debug->smooth_output_span = full_candidate_span;
          }
        }
      }
      if (!stitchedSuffixPreserved(smoothed_candidate, debug)) {
        std::vector<LocalPathPoint> held = publishedReferenceLocal(ego_now, false);
        held = auditAndTrimLocalReference(held, debug, false);
        if (!held.empty()) {
          const ReferenceGeometryAudit held_audit = auditLocalReferenceFull(held);
          if (debug != NULL) {
            debug->reference_reason = "reference_hold_output_connector_only_blocked";
            debug->reference_span = polylineLengthLocal(held);
            debug->published_reference_span = debug->reference_span;
            debug->published_reference_mode =
                last_published_reference_mode_.empty() ? "held_last_good" : last_published_reference_mode_;
            debug->published_reference_reason =
                last_published_reference_reason_.empty() ? debug->reference_reason
                                                         : last_published_reference_reason_;
            debug->held_by_hysteresis = true;
            debug->publisher_action = "hold_connector_only_blocked";
            debug->reconnected_last_good_suffix = true;
            debug->published_hold_age = std::isfinite(published_age) ? published_age : -1.0;
            debug->published_full_valid = held_audit.ok;
            debug->reference_full_max_yaw_jump = held_audit.max_yaw_jump;
            debug->reference_full_backtrack_count = held_audit.backtrack_count;
            debug->reference_full_kappa_max = held_audit.kappa_max;
            debug->published_span_target = debug->reference_span;
            debug->published_update_mode = "retry_join";
            debug->stale_hold_block_reason = "connector_only_publish_blocked";
          }
          return held;
        }
        if (debug != NULL) {
          debug->reference_empty_reason_final = "connector_only_no_last_good";
        }
        return {};
      }
      if (!smoothed_candidate.empty()) {
        if (debug != NULL &&
            (debug->published_update_mode == "init" ||
             debug->published_update_mode == "none" ||
             debug->published_update_mode == "smooth_bypass_candidate")) {
          debug->published_update_mode = "smooth_full_stitched";
        }
        std::vector<HistoryPoint2D> candidate_world =
            localReferenceToWorld(smoothed_candidate, ego_now);
        const ReferenceJumpMetrics candidate_jump =
            referenceJumpMetrics(candidate_world, last_published_reference_world_);
        if (debug != NULL) {
          debug->publisher_action = "smooth_update";
          if (debug->published_update_mode == "init" ||
              debug->published_update_mode == "none") {
            debug->published_update_mode = "smooth_full_stitched";
          }
          debug->reference_state_update_alpha = cfg_.reference_state_update_alpha;
          debug->held_by_hysteresis = false;
          debug->reference_output_growth_limited = false;
          debug->published_hold_age = 0.0;
          debug->published_span_target = polylineLengthLocal(smoothed_candidate);
          debug->smooth_output_span = debug->published_span_target;
          debug->candidate_reference_span = debug->published_span_target;
          debug->published_span_jump =
              std::isfinite(candidate_jump.span_jump) ? candidate_jump.span_jump : 0.0;
          debug->published_endpoint_jump =
              std::isfinite(candidate_jump.endpoint_jump) ? candidate_jump.endpoint_jump : 0.0;
          debug->published_start_jump =
              std::isfinite(candidate_jump.start_jump) ? candidate_jump.start_jump : 0.0;
          debug->published_heading_jump =
              std::isfinite(candidate_jump.heading_jump) ? candidate_jump.heading_jump : 0.0;
          debug->candidate_jump_to_published =
              std::max(debug->published_endpoint_jump,
                       std::max(debug->published_start_jump, debug->published_span_jump));
        }
        pending_reference_world_.clear();
        pending_reference_count_ = 0;
        acceptPublishedReference(candidate_world, now, candidate_mode, candidate_source, debug);
        return smoothed_candidate;
      }
    }

    const double previous_span = historySpan(last_published_reference_world_);
    const double dt_since_publish =
        last_published_reference_stamp_.isZero()
            ? (1.0 / 30.0)
            : std::max(1.0 / 60.0, (now - last_published_reference_stamp_).toSec());
    const double span_update_dt = clampValue(dt_since_publish, 1.0 / 60.0, 0.10);
    const double allowed_growth =
        std::max(cfg_.reference_output_min_span_growth,
                 cfg_.reference_output_span_growth_per_sec * span_update_dt);
    const double allowed_shrink =
        std::max(cfg_.reference_output_min_span_growth,
                 cfg_.reference_output_span_shrink_per_sec * span_update_dt);
    double target_span = candidate_span;
    bool growth_limited = false;
    bool shrink_limited = false;
    bool forced_dynamic_release = false;
    if (debug != NULL) {
      debug->candidate_vs_published_span_diff = candidate_span - previous_span;
    }
    if (candidate_span > previous_span + allowed_growth) {
      target_span = previous_span + allowed_growth;
      output_candidate = limitLocalReferenceBySpan(output_candidate, target_span);
      const std::string previous_full_reject_reason =
          debug != NULL ? debug->reference_reject_full_reason : std::string("none");
      output_candidate = auditAndTrimLocalReference(output_candidate, debug, true);
      if (debug != NULL && previous_full_reject_reason != "none" &&
          debug->reference_reject_full_reason == "none") {
        debug->reference_reject_full_reason = previous_full_reject_reason;
      }
      growth_limited = !output_candidate.empty();
      if (debug != NULL) {
        debug->reference_output_growth_limited = growth_limited;
        debug->published_span_target = target_span;
        debug->candidate_reference_span = polylineLengthLocal(output_candidate);
      }
      if (output_candidate.empty()) {
        std::vector<LocalPathPoint> held = publishedReferenceLocal(ego_now, false);
        held = auditAndTrimLocalReference(held, debug, false);
        if (!held.empty()) {
          if (debug != NULL) {
            const ReferenceGeometryAudit held_audit = auditLocalReferenceFull(held);
            debug->reference_reason = "reference_hold_output_growth_limit";
            debug->reference_span = polylineLengthLocal(held);
            debug->published_reference_span = debug->reference_span;
            debug->published_reference_mode =
                last_published_reference_mode_.empty() ? "held_last_good" : last_published_reference_mode_;
            debug->published_reference_reason =
                last_published_reference_reason_.empty() ? "reference_hold_output_growth_limit"
                                                         : last_published_reference_reason_;
            debug->held_by_hysteresis = true;
            debug->publisher_action = "hold_growth_limit_empty";
            debug->reconnected_last_good_suffix = true;
            debug->published_hold_age = std::isfinite(published_age) ? published_age : -1.0;
            debug->published_full_valid = held_audit.ok;
            debug->reference_full_max_yaw_jump = held_audit.max_yaw_jump;
            debug->reference_full_backtrack_count = held_audit.backtrack_count;
            debug->reference_full_kappa_max = held_audit.kappa_max;
            if (!last_published_reference_source_.empty()) {
              debug->selected_segment_source = last_published_reference_source_;
            }
          }
          return held;
        }
        if (debug != NULL) {
          debug->reference_empty_reason_final = "growth_limit_no_last_good";
        }
        return {};
      }
    } else if (previous_span > cfg_.reference_output_min_publish_span &&
               candidate_span + allowed_shrink < previous_span) {
      const std::vector<HistoryPoint2D> shrink_candidate_world =
          localReferenceToWorld(output_candidate, ego_now);
      const ReferenceJumpMetrics shrink_jump =
          referenceJumpMetrics(shrink_candidate_world, last_published_reference_world_);
      const bool shrink_start_ok =
          !std::isfinite(shrink_jump.start_jump) ||
          shrink_jump.start_jump <= std::max(0.45, 2.0 * cfg_.reference_output_max_start_jump);
      const bool shrink_heading_ok =
          !std::isfinite(shrink_jump.heading_jump) ||
          shrink_jump.heading_jump <= std::max(0.55, 2.0 * cfg_.reference_output_max_heading_jump);
      forced_dynamic_release =
          dynamic_candidate &&
          std::isfinite(published_age) &&
          published_age >= std::max(0.05, cfg_.reference_stale_hold_time) &&
          candidate_span >= cfg_.reference_dynamic_shrink_min_span &&
          output_candidate.size() >=
              static_cast<size_t>(std::max(2, cfg_.reference_dynamic_shrink_min_points)) &&
          shrink_start_ok &&
          shrink_heading_ok;
      if (forced_dynamic_release) {
        shrink_limited = true;
        if (debug != NULL) {
          debug->reference_output_growth_limited = true;
          debug->published_span_target = candidate_span;
          debug->candidate_reference_span = candidate_span;
          debug->held_by_hysteresis = false;
          debug->hold_forced_release = true;
          debug->shrink_limited_publish = true;
          debug->published_span_jump =
              std::isfinite(shrink_jump.span_jump) ? shrink_jump.span_jump : 0.0;
          debug->published_endpoint_jump =
              std::isfinite(shrink_jump.endpoint_jump) ? shrink_jump.endpoint_jump : 0.0;
          debug->published_start_jump =
              std::isfinite(shrink_jump.start_jump) ? shrink_jump.start_jump : 0.0;
          debug->published_heading_jump =
              std::isfinite(shrink_jump.heading_jump) ? shrink_jump.heading_jump : 0.0;
          debug->candidate_jump_to_published =
              std::max(debug->published_endpoint_jump,
                       std::max(debug->published_start_jump, debug->published_span_jump));
        }
      } else {
      std::vector<LocalPathPoint> held = publishedReferenceLocal(ego_now, false);
      held = auditAndTrimLocalReference(held, debug, false);
      if (!held.empty()) {
        const ReferenceGeometryAudit held_audit = auditLocalReferenceFull(held);
        if (debug != NULL) {
          debug->reference_reason = "reference_hold_output_shrink_limit";
          debug->reference_span = polylineLengthLocal(held);
          debug->published_reference_span = debug->reference_span;
          debug->published_reference_mode =
              last_published_reference_mode_.empty() ? "held_last_good" : last_published_reference_mode_;
          debug->published_reference_reason =
              last_published_reference_reason_.empty() ? "reference_hold_output_shrink_limit"
                                                       : last_published_reference_reason_;
          debug->held_by_hysteresis = true;
          debug->publisher_action = "hold_shrink_limit";
          debug->reconnected_last_good_suffix = true;
          debug->published_hold_age = std::isfinite(published_age) ? published_age : -1.0;
          debug->published_full_valid = held_audit.ok;
          debug->reference_full_max_yaw_jump = held_audit.max_yaw_jump;
          debug->reference_full_backtrack_count = held_audit.backtrack_count;
          debug->reference_full_kappa_max = held_audit.kappa_max;
          debug->reference_output_growth_limited = true;
          debug->published_span_target = std::max(candidate_span, previous_span - allowed_shrink);
          debug->candidate_reference_span = candidate_span;
          if (!last_published_reference_source_.empty()) {
            debug->selected_segment_source = last_published_reference_source_;
          }
        }
        return held;
      }
      }
    } else if (debug != NULL) {
      debug->published_span_target = target_span;
    }

    std::vector<HistoryPoint2D> candidate_world = localReferenceToWorld(output_candidate, ego_now);
    const ReferenceJumpMetrics candidate_jump =
        referenceJumpMetrics(candidate_world, last_published_reference_world_);
    if (debug != NULL) {
      debug->published_span_jump =
          std::isfinite(candidate_jump.span_jump) ? candidate_jump.span_jump : 0.0;
      debug->published_endpoint_jump =
          std::isfinite(candidate_jump.endpoint_jump) ? candidate_jump.endpoint_jump : 0.0;
      debug->published_start_jump =
          std::isfinite(candidate_jump.start_jump) ? candidate_jump.start_jump : 0.0;
      debug->published_heading_jump =
          std::isfinite(candidate_jump.heading_jump) ? candidate_jump.heading_jump : 0.0;
      debug->candidate_jump_to_published =
          std::max(debug->published_endpoint_jump,
                   std::max(debug->published_start_jump, debug->published_span_jump));
    }
    const bool candidate_similar =
        referencesSimilar(candidate_world, last_published_reference_world_);
    const double max_hold_time =
        std::max(cfg_.reference_output_min_hold_time, cfg_.reference_output_max_hold_time);
    const bool hold_expired_dynamic =
        dynamic_candidate &&
        std::isfinite(published_age) &&
        published_age >= std::max(0.05, max_hold_time);
    const bool continuity_forced_ok =
        hold_expired_dynamic &&
        (!std::isfinite(candidate_jump.start_jump) ||
         candidate_jump.start_jump <=
             std::max(0.45, 2.0 * cfg_.reference_output_max_start_jump)) &&
        (!std::isfinite(candidate_jump.heading_jump) ||
         candidate_jump.heading_jump <=
             std::max(0.55, 2.0 * cfg_.reference_output_max_heading_jump));
    if ((candidate_similar &&
         published_age >= std::max(0.05, cfg_.reference_output_min_hold_time)) ||
        (growth_limited && candidate_similar) ||
        continuity_forced_ok) {
      pending_reference_world_.clear();
      pending_reference_count_ = 0;
      if (debug != NULL) {
        debug->publisher_action =
            forced_dynamic_release
                ? "force_dynamic_shrink_after_hold" :
            (continuity_forced_ok && !candidate_similar
                ? "force_dynamic_candidate_after_hold"
                : ((growth_limited || shrink_limited) ? "accept_growth_limited" : "accept_candidate"));
        debug->hold_forced_release =
            debug->hold_forced_release || forced_dynamic_release || continuity_forced_ok;
        debug->shrink_limited_publish = debug->shrink_limited_publish || shrink_limited;
      }
      acceptPublishedReference(candidate_world, now, candidate_mode, candidate_source, debug);
      return output_candidate;
    }

    if (!pending_reference_world_.empty() &&
        candidate_mode == pending_reference_mode_ &&
        referencesSimilar(candidate_world, pending_reference_world_)) {
      ++pending_reference_count_;
    } else {
      pending_reference_world_ = candidate_world;
      pending_reference_mode_ = candidate_mode;
      pending_reference_source_ = candidate_source;
      pending_reference_count_ = 1;
    }

    const bool dynamic_suffix_preserved =
        dynamic_candidate &&
        debug != NULL &&
        debug->splice_preserved_suffix_span >=
            std::max(cfg_.reference_min_suffix_preserve_span, 0.35);
    const bool dynamic_forced_publish =
        dynamic_suffix_preserved &&
        std::isfinite(published_age) &&
        published_age >= std::max(0.05, cfg_.reference_output_max_hold_time) &&
        (!std::isfinite(candidate_jump.start_jump) ||
         candidate_jump.start_jump <=
             std::max(0.45, 2.0 * cfg_.reference_output_max_start_jump)) &&
        (!std::isfinite(candidate_jump.heading_jump) ||
         candidate_jump.heading_jump <=
             std::max(0.55, 2.0 * cfg_.reference_output_max_heading_jump));
    if (dynamic_forced_publish) {
      if (debug != NULL) {
        debug->publisher_action = "force_dynamic_suffix_after_hold";
        debug->hold_forced_release = true;
        debug->held_by_hysteresis = false;
      }
      acceptPublishedReference(candidate_world, now, candidate_mode, candidate_source, debug);
      pending_reference_world_.clear();
      pending_reference_count_ = 0;
      return output_candidate;
    }

    if (pending_reference_count_ >= std::max(1, cfg_.reference_output_confirm_frames) &&
        published_age >= std::max(0.05, cfg_.reference_output_min_hold_time) &&
        (candidate_similar || (dynamic_candidate && growth_limited) || continuity_forced_ok)) {
      if (debug != NULL) {
        debug->publisher_action =
            forced_dynamic_release ? "force_confirmed_dynamic_shrink" :
            (candidate_similar ? "accept_confirmed_candidate" : "accept_confirmed_dynamic_candidate");
        debug->hold_forced_release =
            debug->hold_forced_release || forced_dynamic_release || continuity_forced_ok;
        debug->shrink_limited_publish = debug->shrink_limited_publish || shrink_limited;
      }
      acceptPublishedReference(candidate_world, now, candidate_mode, candidate_source, debug);
      pending_reference_world_.clear();
      pending_reference_count_ = 0;
      return output_candidate;
    }

    std::vector<LocalPathPoint> held = publishedReferenceLocal(ego_now, false);
    held = auditAndTrimLocalReference(held, debug, false);
    if (!held.empty()) {
      const ReferenceGeometryAudit held_audit = auditLocalReferenceFull(held);
      if (debug != NULL) {
        debug->reference_reason = "reference_hold_output_hysteresis";
        debug->reference_span = polylineLengthLocal(held);
        debug->published_reference_span = debug->reference_span;
        debug->published_reference_mode =
            last_published_reference_mode_.empty() ? "held_last_good" : last_published_reference_mode_;
        debug->published_reference_reason =
            last_published_reference_reason_.empty() ? "reference_hold_output_hysteresis"
                                                     : last_published_reference_reason_;
        debug->held_by_hysteresis = true;
        debug->reference_hold_age = published_age;
        debug->publisher_action = "hold_hysteresis";
        debug->reconnected_last_good_suffix = true;
        debug->published_hold_age = published_age;
        debug->reference_publish_mode =
            last_published_reference_mode_.empty() ? "held_last_good" : last_published_reference_mode_;
        debug->published_full_valid = held_audit.ok;
        debug->reference_full_max_yaw_jump = held_audit.max_yaw_jump;
        debug->reference_full_backtrack_count = held_audit.backtrack_count;
        debug->reference_full_kappa_max = held_audit.kappa_max;
        debug->published_span_target = debug->reference_span;
        if (!last_published_reference_source_.empty()) {
          debug->selected_segment_source = last_published_reference_source_;
        }
      }
      return held;
    }

    if (debug != NULL) {
      debug->reference_empty_reason_final = "hysteresis_no_last_good";
    }
    return {};
  }

  void finalizePublishedReferenceDebug(const std::vector<LocalPathPoint>& published,
                                       TrackerDebugState* debug) const {
    if (debug == NULL) {
      return;
    }
    debug->actual_published_points = static_cast<int>(published.size());
    debug->actual_published_span = polylineLengthLocal(published);
    if (published.empty()) {
      debug->actual_published_mode = "INVALID";
      debug->actual_published_reason =
          debug->reference_reason.empty() ? "empty" : debug->reference_reason;
      if (debug->reference_empty_reason_final == "none") {
        debug->reference_empty_reason_final = debug->actual_published_reason;
      }
      if (debug->candidate_block_reason == "none") {
        debug->candidate_block_reason = debug->actual_published_reason;
      }
      debug->published_reference_span = 0.0;
      debug->published_full_valid = false;
      debug->published_geometry_valid = false;
      debug->full_audit_hard_valid = false;
      debug->published_stale_age = 0.0;
      debug->published_endpoint_speed = 0.0;
      debug->published_endpoint_age = -1.0;
      debug->tail_endpoint_age = -1.0;
      debug->reference_latency_source = "invalid";
      debug->reference_stale = false;
      debug->reference_freshness_state = "invalid";
      debug->motion_tail_extension_used = false;
      debug->far_start_recovered_by_tail = false;
      debug->stale_hold_demoted = false;
      return;
    }

    const ReferenceGeometryAudit audit = auditLocalReferenceFull(published);
    const std::string mode =
        !debug->published_reference_mode.empty() && debug->published_reference_mode != "init"
            ? debug->published_reference_mode
            : (!debug->reference_publish_mode.empty() ? debug->reference_publish_mode : "published");
    const std::string reason =
        !debug->published_reference_reason.empty() && debug->published_reference_reason != "init"
            ? debug->published_reference_reason
            : (!debug->reference_reason.empty() ? debug->reference_reason : "published");

    debug->reference_span = debug->actual_published_span;
    debug->published_reference_span = debug->actual_published_span;
    debug->actual_published_mode = mode;
    debug->actual_published_reason = reason;
    if (debug->held_by_hysteresis ||
        mode == "held_last_good" ||
        reason.find("reference_hold") == 0) {
      debug->held_publish_reason = reason;
    }
    debug->published_reference_mode = mode;
    debug->published_reference_reason = reason;
    debug->published_full_valid = audit.ok;
    debug->published_geometry_valid = audit.ok;
    debug->full_audit_hard_valid = audit.ok;
    debug->full_audit_kappa_limited = audit.kappa_limited;
    debug->reference_full_max_yaw_jump = audit.max_yaw_jump;
    debug->reference_full_backtrack_count = audit.backtrack_count;
    debug->reference_full_kappa_max = audit.kappa_max;
    debug->tangent_progress_min = audit.tangent_progress_min;
    int full_sign_flips = 0;
    double full_wiggle = 0.0;
    double full_span = 0.0;
    double last_signed_kappa = 0.0;
    double last_kappa = 0.0;
    bool has_last_kappa = false;
    std::vector<double> abs_kappa;
    std::vector<double> abs_dkappa;
    for (size_t i = 1; i < published.size(); ++i) {
      full_span += pointDistance(published[i - 1].x,
                                 published[i - 1].y,
                                 published[i].x,
                                 published[i].y);
    }
    for (size_t i = 1; i + 1 < published.size(); ++i) {
      const double ds_prev = pointDistance(published[i - 1].x,
                                           published[i - 1].y,
                                           published[i].x,
                                           published[i].y);
      const double ds_next = pointDistance(published[i].x,
                                           published[i].y,
                                           published[i + 1].x,
                                           published[i + 1].y);
      if (ds_prev < 1e-4 || ds_next < 1e-4) {
        continue;
      }
      const double yaw_prev = std::atan2(published[i].y - published[i - 1].y,
                                         published[i].x - published[i - 1].x);
      const double yaw_next = std::atan2(published[i + 1].y - published[i].y,
                                         published[i + 1].x - published[i].x);
      const double local_ds = 0.5 * (ds_prev + ds_next);
      const double kappa = wrapAngle(yaw_next - yaw_prev) / std::max(1e-3, local_ds);
      abs_kappa.push_back(std::fabs(kappa));
      if (has_last_kappa) {
        abs_dkappa.push_back(std::fabs(kappa - last_kappa) / std::max(1e-3, local_ds));
      }
      last_kappa = kappa;
      has_last_kappa = true;
      if (std::fabs(kappa) > 0.15) {
        if (std::fabs(last_signed_kappa) > 0.15 && last_signed_kappa * kappa < 0.0) {
          ++full_sign_flips;
        }
        last_signed_kappa = kappa;
      }
      const double mx = 0.5 * (published[i - 1].x + published[i + 1].x);
      const double my = 0.5 * (published[i - 1].y + published[i + 1].y);
      full_wiggle += pointDistance(published[i].x, published[i].y, mx, my);
    }
    full_wiggle /= std::max(0.10, full_span);
    debug->suffix_sign_flip_count = full_sign_flips;
    debug->suffix_wiggle_score = full_wiggle;
    debug->suffix_wiggle_after = full_wiggle;
    if (!abs_kappa.empty()) {
      std::sort(abs_kappa.begin(), abs_kappa.end());
      debug->suffix_curvature_p95 =
          abs_kappa[std::min(abs_kappa.size() - 1,
                             static_cast<size_t>(0.95 * static_cast<double>(abs_kappa.size() - 1)))];
    }
    if (!abs_dkappa.empty()) {
      std::sort(abs_dkappa.begin(), abs_dkappa.end());
      debug->suffix_curvature_rate_p95 =
          abs_dkappa[std::min(abs_dkappa.size() - 1,
                              static_cast<size_t>(0.95 * static_cast<double>(abs_dkappa.size() - 1)))];
    }
    const double full_wiggle_norm =
        clampValue(full_wiggle / std::max(0.05, cfg_.reference_health_wiggle_gate),
                   0.0,
                   1.0);
    const double full_flip_penalty =
        cfg_.reference_health_flip_penalty *
        std::min(4.0, static_cast<double>(full_sign_flips));
    const double full_kappa_p95 =
        abs_kappa.empty() ? 0.0 : debug->suffix_curvature_p95;
    const double full_kappa_excess =
        clampValue((full_kappa_p95 - cfg_.reference_health_curvature_gate) /
                       std::max(1e-3, cfg_.reference_health_curvature_gate),
                   0.0,
                   1.0);
    double full_health =
        1.0 - cfg_.reference_health_wiggle_penalty * full_wiggle_norm -
        full_flip_penalty - cfg_.reference_health_curvature_penalty * full_kappa_excess;
    double full_net_yaw_change = 0.0;
    if (published.size() >= 3) {
      const double first_yaw =
          std::atan2(published[1].y - published[0].y,
                     published[1].x - published[0].x);
      const size_t last = published.size() - 1;
      const double last_yaw =
          std::atan2(published[last].y - published[last - 1].y,
                     published[last].x - published[last - 1].x);
      full_net_yaw_change = std::fabs(wrapAngle(last_yaw - first_yaw));
    }
    const bool full_s_wave =
        full_net_yaw_change < 0.22 &&
        (full_sign_flips >= cfg_.suffix_smooth_sign_flip_gate ||
         (full_sign_flips >= 1 && full_wiggle > cfg_.suffix_smooth_wiggle_gate));
    if (full_s_wave) {
      full_health = std::min(full_health, cfg_.reference_health_s_wave_cap);
    }
    const double audited_health = clampValue(full_health, 0.0, 1.0);
    debug->reference_health_score =
        debug->reference_health_score > 1e-6
            ? std::min(debug->reference_health_score, audited_health)
            : audited_health;
    if (debug->published_span_target <= 1e-6) {
      debug->published_span_target = debug->actual_published_span;
    }
    fillPublishedReferenceStaleDebug(debug);
  }

 private:
  void fillPublishedReferenceStaleDebug(TrackerDebugState* debug) const {
    if (debug == NULL) {
      return;
    }
    const bool has_path =
        debug->actual_published_points >= 2 && debug->actual_published_span > 0.0;
    const ros::Time now = ros::Time::now();
    const double publish_age =
        has_path && last_published_reference_stamp_.isValid()
            ? std::max(0.0, (now - last_published_reference_stamp_).toSec())
            : -1.0;
    const double source_age =
        last_published_endpoint_source_stamp_.isValid()
            ? std::max(0.0, (now - last_published_endpoint_source_stamp_).toSec())
            : -1.0;
    if (has_path && publish_age >= 0.0) {
      // Trackability follows the stitched history replay generated for the
      // current ego pose.  The underlying observation age remains visible in
      // tail_endpoint_age and geometry_progress_age.
      debug->published_endpoint_age = publish_age;
      debug->accepted_source_age = publish_age;
      debug->accepted_suffix_source_age = publish_age;
      debug->reference_latency_source =
          source_age > std::max(0.05, cfg_.reference_stale_hold_time)
              ? "history_cache_replay"
              : "accepted_publish";
    }
    if (last_published_endpoint_progress_stamp_.isValid()) {
      debug->geometry_progress_age =
          std::max(0.0, (now - last_published_endpoint_progress_stamp_).toSec());
    }
    if (current_candidate_source_stamp_.isValid()) {
      debug->tail_endpoint_age =
          std::max(0.0, (now - current_candidate_source_stamp_).toSec());
      debug->candidate_source_stamp = current_candidate_source_stamp_.toSec();
    } else if (source_age >= 0.0) {
      debug->tail_endpoint_age = source_age;
    }
    const bool held_output =
        debug->held_by_hysteresis ||
        debug->actual_published_mode == "held_last_good" ||
        debug->actual_published_reason.find("reference_hold") == 0 ||
        debug->publisher_action.find("hold") == 0;
    debug->fallback_suffix_age =
        held_output && last_good_suffix_source_stamp_.isValid()
            ? std::max(0.0, (now - last_good_suffix_source_stamp_).toSec())
            : -1.0;
    const double hold_age =
        debug->published_hold_age >= 0.0
            ? debug->published_hold_age
            : (debug->reference_hold_age >= 0.0 ? debug->reference_hold_age : 0.0);
    const double endpoint_age =
        debug->published_endpoint_age >= 0.0 ? debug->published_endpoint_age : 0.0;
    const bool endpoint_stale =
        has_path && endpoint_age > std::max(0.05, cfg_.reference_stale_hold_time);
    debug->reference_stale =
        endpoint_stale ||
        (has_path && held_output &&
         hold_age > std::max(0.05, cfg_.reference_stale_hold_time));
    debug->published_stale_age =
        debug->reference_stale ? std::max(hold_age, endpoint_age) : 0.0;
    if (debug->reference_stale && debug->stale_hold_block_reason == "none") {
      debug->stale_hold_block_reason =
          debug->actual_published_reason.empty() ? "stale_hold" : debug->actual_published_reason;
    }
    debug->published_endpoint_speed =
        hold_age > 1e-3 ? std::max(0.0, debug->candidate_jump_to_published) / hold_age : 0.0;
    debug->stale_hold_demoted = debug->reference_stale;
    const double hard_stale_age =
        2.0 * std::max(0.05, cfg_.reference_stale_hold_time);
    debug->reference_freshness_state =
        !has_path || endpoint_age > hard_stale_age ||
                (held_output && hold_age > hard_stale_age)
            ? "invalid"
            : (debug->reference_stale ? "stale_reconnected" : "fresh");
    debug->motion_tail_extension_used =
        has_path &&
        (debug->motion_tail_direct_publish ||
         debug->motion_tail_used_as_reference ||
         debug->motion_tail_promoted_to_reference ||
         debug->selected_segment_source == "motion_tail" ||
         debug->selected_segment_source == "confirmed_tail") &&
        !debug->held_by_hysteresis;
    if (debug->published_endpoint_age >= 0.0) {
      debug->reference_refresh_latency = debug->published_endpoint_age;
    } else if (debug->motion_tail_extension_used && debug->tail_extension_latency >= 0.0) {
      debug->reference_refresh_latency = debug->tail_extension_latency;
      debug->reference_latency_source = "tail_extension";
    } else if (debug->published_hold_age >= 0.0) {
      debug->reference_refresh_latency = debug->published_hold_age;
      debug->reference_latency_source = "published_hold";
    } else {
      debug->reference_refresh_latency = debug->append_stall_age;
      debug->reference_latency_source = "append_stall";
    }
    debug->far_start_recovered_by_tail =
        debug->motion_tail_extension_used &&
        (debug->actual_published_reason.find("far_start") != std::string::npos ||
         debug->reference_reason.find("far_start") != std::string::npos);
  }

  struct JoinChoice {
    bool valid = false;
    size_t suffix_index = 0;
    RefPoint join_point;
    double projected_s = 0.0;
  };

  struct ReferenceValidation {
    bool ok = true;
    bool backtrack = false;
    double kappa_max = 0.0;
    double max_yaw_jump = 0.0;
    std::string reason = "reference_ready";
  };

  struct ReferenceGeometryAudit {
    bool ok = true;
    bool backtrack = false;
    bool kappa_limited = false;
    int backtrack_count = 0;
    double max_yaw_jump = 0.0;
    double kappa_max = 0.0;
    double tangent_progress_min = std::numeric_limits<double>::infinity();
    size_t valid_prefix_points = 0;
    std::string reason = "reference_full_valid";
  };

  struct ConnectorInnerCutAudit {
    bool inner_cut = false;
    double score = 0.0;
    double lateral_shortcut = 0.0;
  };

  ConnectorInnerCutAudit auditConnectorInnerCut(const JoinChoice& join,
                                                double suffix_after_join) const {
    ConnectorInnerCutAudit out;
    if (!join.valid || suffix_after_join <= 1e-6) {
      return out;
    }
    const double connector_span = hypot2(join.join_point.x, join.join_point.y);
    const double connector_ratio =
        connector_span / std::max(0.10, connector_span + suffix_after_join);
    if (connector_span <= cfg_.reference_connector_enable_dist) {
      return out;
    }
    const double chord_yaw = std::atan2(join.join_point.y, join.join_point.x);
    const double yaw_mismatch = std::fabs(wrapAngle(join.join_point.yaw - chord_yaw));
    out.lateral_shortcut = connector_span * std::sin(std::min(yaw_mismatch, 1.45));
    const double ratio_excess =
        clampValue((connector_ratio - cfg_.reference_inner_cut_connector_ratio) /
                       std::max(1e-3, 0.85 - cfg_.reference_inner_cut_connector_ratio),
                   0.0,
                   1.0);
    const double lateral_excess =
        clampValue((out.lateral_shortcut - cfg_.reference_inner_cut_lateral_gate) /
                       std::max(1e-3, 0.35),
                   0.0,
                   1.0);
    out.score = ratio_excess * lateral_excess;
    out.inner_cut =
        out.score >= cfg_.reference_inner_cut_score_gate &&
        (connector_ratio > cfg_.reference_inner_cut_connector_ratio ||
         out.lateral_shortcut > cfg_.reference_inner_cut_lateral_gate);
    return out;
  }

  ReferenceGeometryAudit auditLocalReferenceFull(
      const std::vector<LocalPathPoint>& pts) const {
    ReferenceGeometryAudit out;
    out.valid_prefix_points = pts.size();
    if (pts.size() < 2) {
      out.ok = false;
      out.valid_prefix_points = 0;
      out.reason = "reference_full_too_short";
      return out;
    }

    double prev_segment_yaw = pts.front().yaw;
    bool have_prev_segment = false;
    for (size_t i = 1; i < pts.size(); ++i) {
      const LocalPathPoint& a = pts[i - 1];
      const LocalPathPoint& b = pts[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double ds = hypot2(dx, dy);
      if (ds < 1e-5) {
        continue;
      }

      const double seg_yaw = std::atan2(dy, dx);
      const double point_yaw_jump = std::fabs(wrapAngle(b.yaw - a.yaw));
      const double segment_yaw_jump =
          have_prev_segment ? std::fabs(wrapAngle(seg_yaw - prev_segment_yaw)) : point_yaw_jump;
      const double dyaw = std::max(point_yaw_jump, segment_yaw_jump);
      const double kappa = dyaw / std::max(1e-4, ds);
      const double progress_yaw = have_prev_segment ? prev_segment_yaw : a.yaw;
      const double progress = dx * std::cos(progress_yaw) + dy * std::sin(progress_yaw);

      out.max_yaw_jump = std::max(out.max_yaw_jump, dyaw);
      out.kappa_max = std::max(out.kappa_max, kappa);
      out.tangent_progress_min = std::min(out.tangent_progress_min, progress);

      std::string reason;
      if (progress < -cfg_.reference_backtrack_epsilon) {
        reason = "reference_full_backtrack_reject";
        out.backtrack = true;
        ++out.backtrack_count;
      } else if (ds < cfg_.reference_full_short_kink_distance &&
                 dyaw > cfg_.reference_full_short_kink_yaw) {
        reason = "reference_full_short_kink_reject";
      } else if (dyaw > cfg_.reference_full_max_yaw_jump) {
        reason = "reference_full_yaw_jump_reject";
      } else if (kappa > cfg_.reference_full_max_kappa) {
        out.kappa_limited = true;
      }

      if (!reason.empty()) {
        out.ok = false;
        out.valid_prefix_points = std::max<size_t>(1, i);
        out.reason = reason;
        return out;
      }

      prev_segment_yaw = seg_yaw;
      have_prev_segment = true;
    }
    if (!std::isfinite(out.tangent_progress_min)) {
      out.tangent_progress_min = 0.0;
    }
    return out;
  }

  std::vector<LocalPathPoint> limitLocalReferenceBySpan(
      const std::vector<LocalPathPoint>& pts,
      double target_span) const {
    if (pts.size() < 2 || target_span <= 1e-4) {
      return {};
    }
    std::vector<LocalPathPoint> out;
    out.reserve(pts.size());
    out.push_back(pts.front());
    double accum = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const double ds = pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
      if (ds < 1e-5) {
        continue;
      }
      if (accum + ds <= target_span + 1e-6) {
        out.push_back(pts[i]);
        accum += ds;
        continue;
      }

      const double ratio = clampValue((target_span - accum) / ds, 0.0, 1.0);
      if (ratio > 1e-3) {
        LocalPathPoint pt;
        pt.x = pts[i - 1].x + ratio * (pts[i].x - pts[i - 1].x);
        pt.y = pts[i - 1].y + ratio * (pts[i].y - pts[i - 1].y);
        pt.yaw = wrapAngle(pts[i - 1].yaw +
                           ratio * wrapAngle(pts[i].yaw - pts[i - 1].yaw));
        pt.curvature =
            pts[i - 1].curvature + ratio * (pts[i].curvature - pts[i - 1].curvature);
        out.push_back(pt);
      }
      break;
    }
    if (out.size() >= 2) {
      recomputeLocalHeading(out);
    }
    return out;
  }

  LocalPathPoint sampleLocalPathBySpan(const std::vector<LocalPathPoint>& pts,
                                       double target_span) const {
    if (pts.empty()) {
      return LocalPathPoint();
    }
    if (pts.size() == 1 || target_span <= 0.0) {
      return pts.front();
    }
    double accum = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const double ds = pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
      if (ds < 1e-6) {
        continue;
      }
      if (accum + ds >= target_span) {
        const double r = clampValue((target_span - accum) / ds, 0.0, 1.0);
        LocalPathPoint out;
        out.x = pts[i - 1].x + r * (pts[i].x - pts[i - 1].x);
        out.y = pts[i - 1].y + r * (pts[i].y - pts[i - 1].y);
        out.yaw = wrapAngle(pts[i - 1].yaw + r * wrapAngle(pts[i].yaw - pts[i - 1].yaw));
        out.curvature = pts[i - 1].curvature + r * (pts[i].curvature - pts[i - 1].curvature);
        return out;
      }
      accum += ds;
    }
    return pts.back();
  }

  std::vector<LocalPathPoint> smoothReferenceUpdate(
      const std::vector<LocalPathPoint>& candidate,
      const std::vector<LocalPathPoint>& previous,
      double alpha) const {
    if (candidate.size() < 2 || previous.size() < 2 || alpha >= 0.999) {
      return candidate;
    }
    alpha = clampValue(alpha, 0.0, 1.0);
    const double previous_span = polylineLengthLocal(previous);
    std::vector<LocalPathPoint> out = candidate;
    double s = 0.0;
    out.front().x = 0.0;
    out.front().y = 0.0;
    out.front().yaw = 0.0;
    for (size_t i = 1; i < out.size(); ++i) {
      s += pointDistance(candidate[i - 1].x, candidate[i - 1].y,
                         candidate[i].x, candidate[i].y);
      if (s > previous_span + 1e-3) {
        continue;
      }
      const LocalPathPoint prev = sampleLocalPathBySpan(previous, s);
      out[i].x = alpha * candidate[i].x + (1.0 - alpha) * prev.x;
      out[i].y = alpha * candidate[i].y + (1.0 - alpha) * prev.y;
      out[i].yaw = wrapAngle(prev.yaw + alpha * wrapAngle(candidate[i].yaw - prev.yaw));
      out[i].curvature =
          alpha * candidate[i].curvature + (1.0 - alpha) * prev.curvature;
    }
    if (out.size() >= 2) {
      recomputeLocalHeading(out);
    }
    return out;
  }

  std::vector<LocalPathPoint> auditAndTrimLocalReference(
      const std::vector<LocalPathPoint>& in,
      TrackerDebugState* debug,
      bool candidate_path) const {
    if (candidate_path && debug != NULL) {
      debug->candidate_full_valid = false;
      debug->reference_cut_prefix_points = 0;
      debug->reference_reject_full_reason = "none";
      debug->candidate_full_span_before_audit = polylineLengthLocal(in);
      debug->candidate_full_span_after_audit = 0.0;
      debug->splice_trim_rejected_connector_only = false;
      debug->splice_preserved_suffix_span = 0.0;
    }
    if (in.empty()) {
      if (candidate_path && debug != NULL) {
        debug->reference_reject_full_reason = "reference_full_empty";
      }
      return {};
    }

    std::vector<LocalPathPoint> pts = in;
    if (pts.size() >= 2) {
      recomputeLocalHeading(pts);
    }
    ReferenceGeometryAudit audit = auditLocalReferenceFull(pts);
    if (candidate_path && debug != NULL) {
      debug->reference_full_max_yaw_jump = audit.max_yaw_jump;
      debug->reference_full_backtrack_count = audit.backtrack_count;
      debug->reference_full_kappa_max = audit.kappa_max;
      debug->reference_reject_full_reason = audit.ok ? "none" : audit.reason;
    }
    if (audit.ok) {
      if (candidate_path && stitchedDebugContext(debug)) {
        const double span = polylineLengthLocal(pts);
        const double preserved_suffix =
            std::max(0.0, span - std::max(0.0, debug->connector_span));
        const double min_preserved_suffix = minRequiredPreservedSuffix(debug);
        debug->splice_preserved_suffix_span = preserved_suffix;
        if (preserved_suffix + 1e-6 < min_preserved_suffix) {
          debug->splice_trim_rejected_connector_only = true;
          debug->connector_only_publish_blocked = true;
          debug->published_suffix_preserved_ok = false;
          debug->candidate_full_valid = false;
          debug->full_audit_hard_valid = false;
          debug->candidate_full_span_after_audit = span;
          debug->reference_reject_full_reason = "reference_trim_connector_only";
          debug->join_reject_reason = "reference_trim_connector_only";
          return {};
        }
      }
      if (candidate_path && debug != NULL) {
        debug->candidate_full_valid = true;
        debug->full_audit_hard_valid = true;
        debug->full_audit_kappa_limited = audit.kappa_limited;
        debug->candidate_full_span_after_audit = polylineLengthLocal(pts);
        debug->splice_preserved_suffix_span =
            std::max(0.0, debug->candidate_full_span_after_audit - debug->connector_span);
        debug->reference_reject_full_reason =
            audit.kappa_limited ? "reference_full_kappa_limited" : "none";
      }
      return pts;
    }

    if (audit.valid_prefix_points >= 2 &&
        audit.valid_prefix_points <= pts.size()) {
      std::vector<LocalPathPoint> prefix(pts.begin(), pts.begin() + audit.valid_prefix_points);
      if (prefix.size() >= 2) {
        recomputeLocalHeading(prefix);
      }
      const double prefix_span = polylineLengthLocal(prefix);
      const bool provisional_candidate =
          debug != NULL &&
          (debug->reference_publish_mode == "provisional_tail" ||
           debug->selected_segment_source == "motion_tail" ||
           debug->selected_segment_source == "confirmed_tail" ||
           debug->selected_segment_source == "stitched_confirmed" ||
           debug->motion_tail_promoted_to_reference);
      const bool stitched_candidate =
          debug != NULL &&
          (debug->reference_source == "stitched_tail" ||
           debug->reference_source == "stitched_confirmed" ||
           debug->reference_reason == "reference_stitched_tail" ||
           debug->reference_reason == "reference_stitched_provisional" ||
           debug->selected_segment_source == "confirmed_tail" ||
           debug->selected_segment_source == "stitched_confirmed");
      if (stitched_candidate) {
        const double preserved_suffix =
            std::max(0.0, prefix_span - std::max(0.0, debug->connector_span));
        const double min_preserved_suffix =
            debug->suffix_span >= cfg_.reference_join_min_suffix_span
                ? cfg_.reference_min_suffix_preserve_span
                : cfg_.reference_min_suffix_preserve_span_short;
        if (candidate_path && debug != NULL) {
          debug->splice_preserved_suffix_span = preserved_suffix;
        }
        if (preserved_suffix + 1e-6 < min_preserved_suffix) {
          if (candidate_path && debug != NULL) {
            debug->splice_trim_rejected_connector_only = true;
            debug->reference_reject_full_reason = "reference_trim_connector_only";
            debug->candidate_full_valid = false;
            debug->full_audit_hard_valid = false;
            debug->candidate_full_span_after_audit = prefix_span;
          }
          return {};
        }
      }
      const int min_publish_points =
          provisional_candidate ? 3 : cfg_.reference_output_min_publish_points;
      const double min_publish_span =
          provisional_candidate ? 0.18 : cfg_.reference_output_min_publish_span;
      if (static_cast<int>(prefix.size()) >= min_publish_points &&
          prefix_span >= min_publish_span) {
        ReferenceGeometryAudit prefix_audit = auditLocalReferenceFull(prefix);
        if (prefix_audit.ok) {
          if (candidate_path && debug != NULL) {
            debug->candidate_full_valid = true;
            debug->reference_cut_prefix_points =
                static_cast<int>(pts.size() - prefix.size());
            debug->reference_full_max_yaw_jump = prefix_audit.max_yaw_jump;
            debug->reference_full_backtrack_count = prefix_audit.backtrack_count;
            debug->reference_full_kappa_max = prefix_audit.kappa_max;
            debug->reference_reject_full_reason = audit.reason;
            debug->full_audit_hard_valid = true;
            debug->full_audit_kappa_limited = prefix_audit.kappa_limited;
            debug->candidate_full_span_after_audit = prefix_span;
          }
          return prefix;
        }
      }
    }

    if (candidate_path && debug != NULL) {
      debug->candidate_full_valid = false;
      debug->full_audit_hard_valid = false;
      debug->full_audit_kappa_limited = audit.kappa_limited;
      debug->reference_cut_prefix_points = 0;
      debug->reference_reject_full_reason = audit.reason;
      debug->candidate_block_reason = audit.reason;
    }
    return {};
  }

  static void evaluateQuinticBezier(const RefPoint control[6], double u, RefPoint* out) {
    if (out == NULL) {
      return;
    }
    const double one_minus_u = 1.0 - u;
    const double b0 = std::pow(one_minus_u, 5);
    const double b1 = 5.0 * u * std::pow(one_minus_u, 4);
    const double b2 = 10.0 * u * u * std::pow(one_minus_u, 3);
    const double b3 = 10.0 * u * u * u * std::pow(one_minus_u, 2);
    const double b4 = 5.0 * u * u * u * u * one_minus_u;
    const double b5 = std::pow(u, 5);

    out->x = b0 * control[0].x + b1 * control[1].x + b2 * control[2].x +
             b3 * control[3].x + b4 * control[4].x + b5 * control[5].x;
    out->y = b0 * control[0].y + b1 * control[1].y + b2 * control[2].y +
             b3 * control[3].y + b4 * control[4].y + b5 * control[5].y;

    const double db0 = -5.0 * std::pow(one_minus_u, 4);
    const double db1 = 5.0 * std::pow(one_minus_u, 4) - 20.0 * u * std::pow(one_minus_u, 3);
    const double db2 = 20.0 * u * std::pow(one_minus_u, 3) - 30.0 * u * u * std::pow(one_minus_u, 2);
    const double db3 = 30.0 * u * u * std::pow(one_minus_u, 2) - 20.0 * u * u * u * one_minus_u;
    const double db4 = 20.0 * u * u * u * one_minus_u - 5.0 * std::pow(u, 4);
    const double db5 = 5.0 * std::pow(u, 4);

    const double dx = db0 * control[0].x + db1 * control[1].x + db2 * control[2].x +
                      db3 * control[3].x + db4 * control[4].x + db5 * control[5].x;
    const double dy = db0 * control[0].y + db1 * control[1].y + db2 * control[2].y +
                      db3 * control[3].y + db4 * control[4].y + db5 * control[5].y;
    out->yaw = std::atan2(dy, dx);
    out->kappa = 0.0;
    out->v_ref = 0.0;
    out->a_ref = 0.0;
  }

  void applyLocalSpliceBlend(std::vector<RefPoint>* pts, size_t splice_index) const {
    if (pts == NULL || pts->size() < 4 || splice_index == 0 || splice_index >= pts->size()) {
      return;
    }

    recomputeRefGeometry(pts);
    const double blend_length = std::max(0.10, cfg_.reference_splice_blend_length);
    double accum = 0.0;
    size_t end_index = splice_index;
    while (end_index + 1 < pts->size() && accum < blend_length) {
      accum += pointDistance((*pts)[end_index].x, (*pts)[end_index].y,
                             (*pts)[end_index + 1].x, (*pts)[end_index + 1].y);
      ++end_index;
    }
    if (end_index <= splice_index + 1) {
      recomputeRefGeometry(pts);
      return;
    }

    const RefPoint anchor = (*pts)[splice_index];
    for (size_t i = splice_index + 1; i <= end_index; ++i) {
      const double s = (*pts)[i].s - anchor.s;
      const double ratio = clampValue(s / std::max(1e-3, accum), 0.0, 1.0);
      const double w = ratio * ratio * (3.0 - 2.0 * ratio);
      const double tx = std::cos(anchor.yaw);
      const double ty = std::sin(anchor.yaw);
      const double proj = ((*pts)[i].x - anchor.x) * tx + ((*pts)[i].y - anchor.y) * ty;
      const double guided_x = anchor.x + proj * tx;
      const double guided_y = anchor.y + proj * ty;
      (*pts)[i].x = (1.0 - 0.35 * (1.0 - w)) * (*pts)[i].x + 0.35 * (1.0 - w) * guided_x;
      (*pts)[i].y = (1.0 - 0.35 * (1.0 - w)) * (*pts)[i].y + 0.35 * (1.0 - w) * guided_y;
      (*pts)[i].t = std::max((*pts)[i].t, anchor.t + s / std::max(0.10, anchor.v_ref));
    }
    recomputeRefGeometry(pts);
  }

  double remainingSuffixSpan(const std::vector<RefPoint>& ref_profile,
                             double join_s) const {
    if (ref_profile.empty()) {
      return 0.0;
    }
    return std::max(0.0, ref_profile.back().s - join_s);
  }

  JoinChoice retryJoinChoice(const std::vector<RefPoint>& ref_profile,
                             const JoinChoice& initial,
                             int retry,
                             double min_remaining_suffix_span) const {
    if (ref_profile.empty()) {
      return JoinChoice();
    }
    if (retry <= 0) {
      return initial;
    }
    JoinChoice out;
    const size_t step = static_cast<size_t>(retry);
    const size_t idx =
        initial.suffix_index > step ? initial.suffix_index - step : 0;
    if (remainingSuffixSpan(ref_profile, ref_profile[idx].s) + 1e-6 <
        min_remaining_suffix_span) {
      return JoinChoice();
    }
    out.valid = true;
    out.suffix_index = idx;
    out.join_point = ref_profile[idx];
    return out;
  }

  std::vector<RefPoint> buildReferenceCandidate(const std::vector<RefPoint>& ref_profile,
                                                const JoinChoice& join,
                                                ReferenceValidation* validation) const {
    std::vector<RefPoint> combined;
    if (validation != NULL) {
      *validation = ReferenceValidation();
    }
    if (!join.valid || ref_profile.empty()) {
      if (validation != NULL) {
        validation->ok = false;
        validation->reason = "reference_join_invalid";
      }
      return combined;
    }

    const RefPoint& join_ref = join.join_point;
    RefPoint origin;
    origin.v_ref = std::min(0.3, join_ref.v_ref);
    origin.a_ref = 0.0;
    combined.push_back(origin);

    size_t splice_begin_index = 0;
    if (hypot2(join_ref.x, join_ref.y) > cfg_.reference_direct_attach_dist) {
      const std::vector<RefPoint> connector = buildConnector(join_ref);
      for (size_t i = 1; i < connector.size(); ++i) {
        combined.push_back(connector[i]);
      }
      splice_begin_index = combined.empty() ? 0 : combined.size() - 1;
    } else {
      combined.push_back(join_ref);
      splice_begin_index = combined.size() - 1;
    }

    appendMonotonicSuffix(&combined, ref_profile, join.suffix_index, validation);
    if (combined.size() < static_cast<size_t>(std::max(2, cfg_.reference_min_points))) {
      if (validation != NULL) {
        validation->ok = false;
        validation->reason = "reference_suffix_too_short";
      }
      return combined;
    }

    recomputeRefGeometry(&combined);
    applyLocalSpliceBlend(&combined, splice_begin_index);

    if (cfg_.reference_max_length > 0.0 && combined.size() >= 2) {
      std::vector<RefPoint> clipped;
      clipped.reserve(combined.size());
      clipped.push_back(combined.front());
      for (size_t i = 1; i < combined.size(); ++i) {
        clipped.push_back(combined[i]);
        if (combined[i].s >= cfg_.reference_max_length) {
          break;
        }
      }
      combined.swap(clipped);
      recomputeRefGeometry(&combined);
    }

    ReferenceValidation local_validation = validateReferenceFront(combined);
    if (validation != NULL) {
      *validation = local_validation;
    }
    return combined;
  }

  void appendMonotonicSuffix(std::vector<RefPoint>* combined,
                             const std::vector<RefPoint>& ref_profile,
                             size_t start_index,
                             ReferenceValidation* validation) const {
    if (combined == NULL || ref_profile.empty()) {
      return;
    }
    for (size_t i = std::min(start_index, ref_profile.size()); i < ref_profile.size(); ++i) {
      const RefPoint& next = ref_profile[i];
      if (!combined->empty()) {
        const RefPoint& last = combined->back();
        const double dx = next.x - last.x;
        const double dy = next.y - last.y;
        const double ds = hypot2(dx, dy);
        const double progress = dx * std::cos(last.yaw) + dy * std::sin(last.yaw);
        const double dyaw = std::fabs(wrapAngle(next.yaw - last.yaw));
        const double kappa = dyaw / std::max(1e-4, ds);
        if (progress < -cfg_.reference_backtrack_epsilon) {
          if (validation != NULL) {
            validation->ok = false;
            validation->backtrack = true;
            validation->reason = "reference_suffix_backtrack_reject";
          }
          break;
        }
        if (ds < cfg_.reference_short_kink_distance &&
            dyaw > cfg_.reference_short_kink_yaw) {
          if (validation != NULL) {
            validation->ok = false;
            validation->kappa_max = std::max(validation->kappa_max, kappa);
            validation->max_yaw_jump = std::max(validation->max_yaw_jump, dyaw);
            validation->reason = "reference_suffix_kink_reject";
          }
          break;
        }
      }
      if (!combined->empty() &&
          pointDistance(combined->back().x, combined->back().y, next.x, next.y) < 0.02) {
        combined->back() = next;
      } else {
        combined->push_back(next);
      }
    }
  }

  ReferenceValidation validateReferenceFront(const std::vector<RefPoint>& pts) const {
    ReferenceValidation out;
    if (pts.size() < 2) {
      out.ok = false;
      out.reason = "reference_validation_too_short";
      return out;
    }
    double traversed = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const RefPoint& a = pts[i - 1];
      const RefPoint& b = pts[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double ds = hypot2(dx, dy);
      if (ds < 1e-5) {
        continue;
      }
      traversed += ds;
      const double progress = dx * std::cos(a.yaw) + dy * std::sin(a.yaw);
      const double dyaw = std::fabs(wrapAngle(b.yaw - a.yaw));
      const double kappa = dyaw / std::max(1e-4, ds);
      out.max_yaw_jump = std::max(out.max_yaw_jump, dyaw);
      out.kappa_max = std::max(out.kappa_max, kappa);
      if (progress < -cfg_.reference_backtrack_epsilon) {
        out.ok = false;
        out.backtrack = true;
        out.reason = "reference_backtrack_reject";
        return out;
      }
      if (ds < cfg_.reference_short_kink_distance &&
          dyaw > cfg_.reference_short_kink_yaw) {
        out.ok = false;
        out.reason = "reference_splice_short_kink_reject";
        return out;
      }
      if (dyaw > cfg_.reference_max_front_yaw_jump) {
        out.ok = false;
        out.reason = "reference_splice_yaw_jump_reject";
        return out;
      }
      if (kappa > cfg_.reference_max_front_kappa) {
        out.kappa_max = std::max(out.kappa_max, kappa);
      }
      if (traversed >= cfg_.reference_validation_front_s) {
        break;
      }
    }
    return out;
  }

  JoinChoice chooseJoinIndex(const std::vector<RefPoint>& ref_profile,
                             double min_remaining_suffix_span,
                             bool use_replay_cursor = true) const {
    JoinChoice best;
    if (ref_profile.empty()) {
      return best;
    }
    if (ref_profile.size() == 1) {
      if (min_remaining_suffix_span <= 1e-6) {
        best.valid = true;
        best.suffix_index = 0;
        best.join_point = ref_profile.front();
        best.projected_s = ref_profile.front().s;
      }
      return best;
    }

    double s_proj = ref_profile.front().s;
    double best_proj_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < ref_profile.size(); ++i) {
      const RefPoint& a = ref_profile[i];
      const RefPoint& b = ref_profile[i + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double seg_len2 = std::max(1e-6, dx * dx + dy * dy);
      const double t = clampValue((-(a.x * dx + a.y * dy)) / seg_len2, 0.0, 1.0);
      const double px = a.x + t * dx;
      const double py = a.y + t * dy;
      const double proj_dist = hypot2(px, py);
      if (proj_dist < best_proj_dist) {
        best_proj_dist = proj_dist;
        s_proj = (1.0 - t) * a.s + t * b.s;
      }
    }

    const bool bootstrap =
        min_remaining_suffix_span <= cfg_.reference_join_min_suffix_span_short + 1e-6;
    const double max_skip = std::max(0.05, cfg_.reference_join_max_skip_span);
    double s_min = std::max(ref_profile.front().s, s_proj);
    double s_max = std::min(ref_profile.back().s, s_min + max_skip);
    if (bootstrap && (!last_join_s_valid_ || !use_replay_cursor)) {
      // Bootstrap attaches to the first measured history point.  The
      // connector may be long, but no observed trajectory is skipped.
      s_min = ref_profile.front().s;
      s_max = s_min;
      s_proj = s_min;
    } else if (last_join_s_valid_ && use_replay_cursor) {
      // Treat the previous join as a causal replay cursor.  Nearest-point
      // geometry may only advance it inside one small local window.
      const double cursor =
          clampValue(last_join_s_, ref_profile.front().s, ref_profile.back().s);
      s_min = cursor;
      s_max = std::min(ref_profile.back().s, cursor + max_skip);
      s_proj = cursor;
    }
    const double available_at_cursor = remainingSuffixSpan(ref_profile, s_min);
    if (available_at_cursor + 1e-6 <
        std::max(0.0, cfg_.reference_join_min_suffix_span_short)) {
      return best;
    }
    const double effective_min_remaining =
        std::min(min_remaining_suffix_span, available_at_cursor);
    const double connector_ratio_limit =
        bootstrap ? cfg_.reference_join_max_connector_ratio_bootstrap
                  : cfg_.reference_join_max_connector_ratio;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < ref_profile.size(); ++i) {
      const RefPoint& a = ref_profile[i];
      const RefPoint& b = ref_profile[i + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double seg_len2 = std::max(1e-6, dx * dx + dy * dy);
      const double ds = std::max(1e-6, b.s - a.s);
      const double t_window_min = clampValue((s_min - a.s) / ds, 0.0, 1.0);
      const double t_window_max = clampValue((s_max - a.s) / ds, 0.0, 1.0);
      if (t_window_max + 1e-9 < t_window_min || b.s < s_min || a.s > s_max) {
        continue;
      }
      const double t_projection =
          clampValue((-(a.x * dx + a.y * dy)) / seg_len2, 0.0, 1.0);
      const double t = clampValue(t_projection, t_window_min, t_window_max);

      RefPoint cand;
      cand.x = a.x + t * dx;
      cand.y = a.y + t * dy;
      cand.yaw = wrapAngle(a.yaw + t * wrapAngle(b.yaw - a.yaw));
      cand.kappa = (1.0 - t) * a.kappa + t * b.kappa;
      cand.s = (1.0 - t) * a.s + t * b.s;
      cand.t = (1.0 - t) * a.t + t * b.t;
      cand.v_ref = std::max(0.05, (1.0 - t) * a.v_ref + t * b.v_ref);
      cand.a_ref = (1.0 - t) * a.a_ref + t * b.a_ref;

      if (cand.s < s_min - 1e-6 || cand.s > s_max + 1e-6) {
        continue;
      }
      const double remaining_suffix = remainingSuffixSpan(ref_profile, cand.s);
      if (remaining_suffix + 1e-6 < effective_min_remaining) {
        continue;
      }

      if (std::fabs(cand.y) > cfg_.reference_join_max_lateral) {
        continue;
      }

      const double dist = hypot2(cand.x, cand.y);
      const double yaw_err = std::fabs(wrapAngle(cand.yaw));
      const double connector_len = dist;
      const double connector_ratio =
          connector_len / std::max(0.10, connector_len + remaining_suffix);
      const double connector_ratio_excess =
          std::max(0.0, connector_ratio - connector_ratio_limit);
      const double forward_shortfall =
          std::max(0.0, cfg_.reference_join_min_forward_x - cand.x);
      double stable_join_cost = 0.0;
      if (last_join_s_valid_) {
        const double window = std::max(0.05, cfg_.reference_join_stability_window);
        stable_join_cost =
            cfg_.reference_join_stability_weight *
            clampValue(std::fabs(cand.s - last_join_s_) / window, 0.0, 2.0);
      }
      const double cost = connector_len +
                          0.35 * yaw_err +
                          0.45 * std::fabs(cand.kappa) +
                          0.65 * connector_ratio +
                          2.0 * connector_ratio_excess +
                          3.0 * forward_shortfall +
                          cfg_.reference_join_skip_weight *
                              std::max(0.0, cand.s - s_proj) +
                          stable_join_cost;
      if (cost < best_cost) {
        best_cost = cost;
        best.valid = true;
        best.suffix_index = i + 1;
        best.join_point = cand;
        best.projected_s = s_proj;
      }
    }

    if (!best.valid) {
      const double remaining = remainingSuffixSpan(ref_profile, ref_profile.front().s);
      if (remaining + 1e-6 >= effective_min_remaining &&
          ref_profile.front().s >= s_min - 1e-6 &&
          ref_profile.front().s <= s_max + 1e-6) {
        best.valid = true;
        best.suffix_index = 0;
        best.join_point = ref_profile.front();
        best.projected_s = s_proj;
      }
    }
    return best;
  }

  std::vector<LocalPathPoint> heldReferenceLocal(const EgoPose2D& ego_now) const {
    if (last_good_reference_world_.size() < 2 || last_good_reference_stamp_.isZero()) {
      return {};
    }
    if (ego_now.stamp.isValid() &&
        (ego_now.stamp - last_good_reference_stamp_).toSec() > cfg_.reference_last_good_hold_timeout) {
      return {};
    }
    const std::vector<LocalPathPoint> local_suffix =
        transformWorldPathToLocal(last_good_reference_world_, ego_now);
    std::vector<RefPoint> suffix_profile;
    suffix_profile.reserve(local_suffix.size());
    double arc = 0.0;
    for (size_t i = 0; i < local_suffix.size(); ++i) {
      if (hypot2(local_suffix[i].x, local_suffix[i].y) > cfg_.raw_max_range) {
        continue;
      }
      RefPoint point;
      point.x = local_suffix[i].x;
      point.y = local_suffix[i].y;
      point.yaw = local_suffix[i].yaw;
      point.kappa = local_suffix[i].curvature;
      point.v_ref = 0.10;
      if (!suffix_profile.empty()) {
        arc += pointDistance(suffix_profile.back().x,
                             suffix_profile.back().y,
                             point.x,
                             point.y);
      }
      point.s = arc;
      suffix_profile.push_back(point);
    }
    if (suffix_profile.size() < 2 || arc < cfg_.reference_join_min_suffix_span_short) {
      return {};
    }
    recomputeRefGeometry(&suffix_profile);
    const double min_suffix =
        arc >= cfg_.reference_join_min_suffix_span
            ? cfg_.reference_join_min_suffix_span
            : cfg_.reference_join_min_suffix_span_short;
    const JoinChoice join = chooseJoinIndex(suffix_profile, min_suffix, false);
    if (!join.valid) {
      return {};
    }
    ReferenceValidation validation;
    const std::vector<RefPoint> stitched =
        buildReferenceCandidate(suffix_profile, join, &validation);
    if (!validation.ok) {
      return {};
    }
    const std::vector<LocalPathPoint> local = makeLocalPathFromRefPoints(stitched);
    const ReferenceGeometryAudit audit = auditLocalReferenceFull(local);
    if (!audit.ok || local.size() < 2 || polylineLengthLocal(local) < 0.15) {
      return {};
    }
    current_candidate_source_stamp_ = last_good_suffix_source_stamp_;
    return local;
  }

  std::vector<LocalPathPoint> publishedReferenceLocal(const EgoPose2D& ego_now,
                                                      bool enforce_timeout = true) const {
    if (last_published_reference_world_.size() < 2 || last_published_reference_stamp_.isZero()) {
      return {};
    }
    const ros::Time now = ego_now.stamp.isValid() ? ego_now.stamp : ros::Time::now();
    if (enforce_timeout &&
        (now - last_published_reference_stamp_).toSec() > cfg_.reference_last_good_hold_timeout) {
      return {};
    }
    std::vector<LocalPathPoint> local = transformWorldPathToLocal(last_published_reference_world_, ego_now);
    for (size_t i = 0; i < local.size() && i < last_published_reference_world_.size(); ++i) {
      local[i].curvature = last_published_reference_world_[i].curvature;
    }
    std::vector<LocalPathPoint> trimmed;
    trimmed.reserve(local.size());
    for (size_t i = 0; i < local.size(); ++i) {
      if (local[i].x < -cfg_.raw_keep_behind_x) {
        continue;
      }
      if (hypot2(local[i].x, local[i].y) > cfg_.raw_max_range) {
        continue;
      }
      trimmed.push_back(local[i]);
      if (static_cast<int>(trimmed.size()) >= cfg_.raw_max_points) {
        break;
      }
    }
    if (trimmed.size() >= 2) {
      recomputeLocalHeading(trimmed);
    }
    const int min_points = enforce_timeout ? std::max(2, cfg_.reference_min_points) : 2;
    const double min_span =
        enforce_timeout ? std::max(0.18, cfg_.reference_min_span) : 0.15;
    if (static_cast<int>(trimmed.size()) < min_points ||
        polylineLengthLocal(trimmed) < min_span) {
      return {};
    }
    return trimmed;
  }

  double heldReferenceAge(const EgoPose2D& ego_now) const {
    if (last_good_reference_stamp_.isZero()) {
      return -1.0;
    }
    const ros::Time now = ego_now.stamp.isValid() ? ego_now.stamp : ros::Time::now();
    return std::max(0.0, (now - last_good_reference_stamp_).toSec());
  }

  std::vector<LocalPathPoint> degradedRawReference(const std::vector<LocalPathPoint>& raw,
                                                   const EgoPose2D& ego_now,
                                                   const std::string& reason,
                                                   TrackerDebugState* debug) const {
    if (!cfg_.reference_degraded_raw_fallback_enabled ||
        static_cast<int>(raw.size()) < cfg_.reference_degraded_raw_min_points) {
      return {};
    }
    std::vector<LocalPathPoint> stable = selectStableLocalSegment(raw, debug);
    if (stable.empty()) {
      return {};
    }
    const double start_distance = hypot2(stable.front().x, stable.front().y);
    if (start_distance > cfg_.reference_degraded_raw_max_start_distance) {
      std::vector<LocalPathPoint> held = heldReferenceLocal(ego_now);
      if (!held.empty()) {
        if (debug != NULL) {
          debug->reference_reason = reason + "_hold_far_start";
          debug->reference_span = polylineLengthLocal(held);
          debug->reference_hold_age = heldReferenceAge(ego_now);
          debug->reference_publish_mode = "held_last_good";
          debug->reference_trackable_raw = false;
        }
        return held;
      }
      if (debug != NULL) {
        debug->reference_reason = reason + "_far_start";
        debug->reference_span = 0.0;
        debug->reference_publish_mode = "empty_far_start";
        debug->reference_trackable_raw = false;
      }
      return {};
    }
    const double span = polylineLengthLocal(stable);
    if (debug != NULL) {
      debug->reference_reason = reason;
      debug->reference_span = span;
      debug->reference_publish_mode = "degraded_raw";
      debug->reference_trackable_raw =
          span >= std::max(cfg_.reference_degraded_raw_min_span, 1.20) &&
          static_cast<int>(stable.size()) >= cfg_.reference_degraded_raw_min_points;
      debug->reference_splice_kappa_max = std::max(debug->reference_splice_kappa_max, 0.0);
    }
    if (span >= std::max(0.60, cfg_.reference_degraded_raw_min_span)) {
      rememberLastGoodLocalReference(stable, ego_now);
    }
    return stable;
  }

  std::vector<LocalPathPoint> provisionalTailReference(const std::vector<LocalPathPoint>& raw,
                                                       const EgoPose2D& ego_now,
                                                       TrackerDebugState* debug) const {
    if (static_cast<int>(raw.size()) < cfg_.reference_degraded_raw_min_points) {
      return {};
    }
    std::vector<LocalPathPoint> stable = selectStableLocalSegment(raw, debug);
    if (stable.empty()) {
      return {};
    }
    const double span = polylineLengthLocal(stable);
    if (span < cfg_.reference_dynamic_shrink_min_span) {
      return {};
    }
    const bool short_tail = span < 0.35;
    if (debug != NULL) {
      debug->reference_reason = "reference_provisional_tail";
      debug->reference_span = span;
      debug->reference_publish_mode = "provisional_tail";
      debug->reference_trackable_raw = !short_tail;
      debug->motion_tail_used_as_reference = true;
      debug->motion_tail_promoted_to_reference = true;
      debug->motion_tail_direct_publish = true;
      debug->provisional_tail_block_detail =
          short_tail ? "short_tail_direct_no_connector" : "tail_direct_no_connector";
      debug->reference_splice_kappa_max = 0.0;
      debug->reference_splice_yaw_jump = 0.0;
    }
    if (!short_tail) {
      rememberLastGoodLocalReference(stable, ego_now);
    }
    return stable;
  }

  std::vector<LocalPathPoint> selectStableLocalSegment(const std::vector<LocalPathPoint>& pts,
                                                       TrackerDebugState* debug) const {
    if (static_cast<int>(pts.size()) < cfg_.stable_segment_min_points) {
      return {};
    }
    size_t valid_end = 0;
    bool cut = false;
    double max_yaw_jump = 0.0;
    double max_kappa = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const LocalPathPoint& a = pts[i - 1];
      const LocalPathPoint& b = pts[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double ds = hypot2(dx, dy);
      const double progress = dx * std::cos(a.yaw) + dy * std::sin(a.yaw);
      const double dyaw = std::fabs(wrapAngle(b.yaw - a.yaw));
      const double kappa = dyaw / std::max(1e-4, ds);
      max_yaw_jump = std::max(max_yaw_jump, dyaw);
      max_kappa = std::max(max_kappa, kappa);
      if (progress < -cfg_.reference_backtrack_epsilon ||
          (ds < cfg_.reference_short_kink_distance &&
           dyaw > cfg_.reference_short_kink_yaw) ||
          dyaw > cfg_.reference_max_front_yaw_jump) {
        cut = true;
        break;
      }
      valid_end = i;
    }

    if (valid_end == 0) {
      if (debug != NULL) {
        debug->stable_segment_count = 0;
        debug->selected_segment_span = 0.0;
        debug->selected_segment_score = -1.0;
      }
      return {};
    }

    std::vector<LocalPathPoint> out(pts.begin(), pts.begin() + static_cast<long>(valid_end + 1));
    recomputeLocalHeading(out);
    const double span = polylineLengthLocal(out);
    if (static_cast<int>(out.size()) < cfg_.stable_segment_min_points ||
        span < cfg_.reference_dynamic_shrink_min_span) {
      if (debug != NULL) {
        debug->stable_segment_count = 0;
        debug->selected_segment_span = span;
        debug->selected_segment_score = span;
      }
      return {};
    }
    if (!localPathFrontValid(out)) {
      return {};
    }
    if (debug != NULL) {
      debug->stable_segment_count = 1;
      debug->selected_segment_span = span;
      debug->selected_segment_score = span - 0.15 * max_yaw_jump - 0.10 * max_kappa;
      if (cut && debug->candidate_block_reason == "none") {
        debug->candidate_block_reason = "stable_prefix_cut";
      }
    }
    return out;
  }

  bool localPathFrontValid(const std::vector<LocalPathPoint>& pts) const {
    if (pts.size() < 2) {
      return false;
    }
    double traversed = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const LocalPathPoint& a = pts[i - 1];
      const LocalPathPoint& b = pts[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double ds = hypot2(dx, dy);
      if (ds < 1e-5) {
        continue;
      }
      traversed += ds;
      const double progress = dx * std::cos(a.yaw) + dy * std::sin(a.yaw);
      const double dyaw = std::fabs(wrapAngle(b.yaw - a.yaw));
      if (progress < -cfg_.reference_backtrack_epsilon ||
          (ds < cfg_.reference_short_kink_distance && dyaw > cfg_.reference_short_kink_yaw) ||
          dyaw > cfg_.reference_max_front_yaw_jump) {
        return false;
      }
      if (traversed >= cfg_.reference_validation_front_s) {
        break;
      }
    }
    return true;
  }

  void rememberLastGoodSuffix(const std::vector<RefPoint>& ref,
                              const JoinChoice& join,
                              const EgoPose2D& ego_now) const {
    if (!join.valid || ref.size() < 2) {
      return;
    }
    std::vector<RefPoint> suffix;
    suffix.reserve(ref.size() - std::min(join.suffix_index, ref.size() - 1) + 1);
    suffix.push_back(join.join_point);
    for (size_t i = join.suffix_index; i < ref.size(); ++i) {
      if (pointDistance(suffix.back().x, suffix.back().y, ref[i].x, ref[i].y) < 1e-4) {
        continue;
      }
      suffix.push_back(ref[i]);
    }
    if (suffix.size() < 2) {
      return;
    }
    last_good_reference_world_.clear();
    last_good_reference_world_.reserve(suffix.size());
    for (size_t i = 0; i < suffix.size(); ++i) {
      HistoryPoint2D pt;
      pt.stamp = current_candidate_source_stamp_.isValid()
                     ? current_candidate_source_stamp_
                     : ego_now.stamp;
      transformLocalToWorld(ego_now, suffix[i].x, suffix[i].y, &pt.x, &pt.y);
      pt.heading = wrapAngle(suffix[i].yaw + ego_now.yaw);
      pt.curvature = suffix[i].kappa;
      pt.speed = suffix[i].v_ref;
      pt.predicted = false;
      last_good_reference_world_.push_back(pt);
    }
    last_good_reference_stamp_ = ego_now.stamp.isValid() ? ego_now.stamp : ros::Time::now();
    last_good_suffix_source_stamp_ = current_candidate_source_stamp_.isValid()
                                         ? current_candidate_source_stamp_
                                         : last_good_reference_stamp_;
  }

  void rememberLastGoodLocalReference(const std::vector<LocalPathPoint>& local,
                                      const EgoPose2D& ego_now) const {
    if (local.size() < 2) {
      return;
    }
    last_good_reference_world_.clear();
    last_good_reference_world_.reserve(local.size());
    for (size_t i = 0; i < local.size(); ++i) {
      HistoryPoint2D pt;
      pt.stamp = ego_now.stamp;
      transformLocalToWorld(ego_now, local[i].x, local[i].y, &pt.x, &pt.y);
      pt.heading = wrapAngle(local[i].yaw + ego_now.yaw);
      pt.curvature = local[i].curvature;
      pt.speed = 0.0;
      pt.predicted = false;
      last_good_reference_world_.push_back(pt);
    }
    last_good_reference_stamp_ = ego_now.stamp.isValid() ? ego_now.stamp : ros::Time::now();
  }

  std::vector<HistoryPoint2D> localReferenceToWorld(const std::vector<LocalPathPoint>& local,
                                                    const EgoPose2D& ego_now) const {
    std::vector<HistoryPoint2D> world;
    world.reserve(local.size());
    const ros::Time stamp = ego_now.stamp.isValid() ? ego_now.stamp : ros::Time::now();
    for (size_t i = 0; i < local.size(); ++i) {
      HistoryPoint2D pt;
      pt.stamp = stamp;
      transformLocalToWorld(ego_now, local[i].x, local[i].y, &pt.x, &pt.y);
      pt.heading = wrapAngle(local[i].yaw + ego_now.yaw);
      pt.curvature = local[i].curvature;
      pt.speed = 0.0;
      pt.predicted = false;
      world.push_back(pt);
    }
    return world;
  }

  double historySpan(const std::vector<HistoryPoint2D>& pts) const {
    double span = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      span += pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
    }
    return span;
  }

  struct ReferenceJumpMetrics {
    double span_jump = std::numeric_limits<double>::infinity();
    double start_jump = std::numeric_limits<double>::infinity();
    double endpoint_jump = std::numeric_limits<double>::infinity();
    double heading_jump = std::numeric_limits<double>::infinity();
  };

  ReferenceJumpMetrics referenceJumpMetrics(const std::vector<HistoryPoint2D>& a,
                                            const std::vector<HistoryPoint2D>& b) const {
    ReferenceJumpMetrics m;
    if (a.size() < 2 || b.size() < 2) {
      return m;
    }
    m.span_jump = std::fabs(historySpan(a) - historySpan(b));
    m.start_jump = pointDistance(a.front().x, a.front().y, b.front().x, b.front().y);
    m.endpoint_jump = pointDistance(a.back().x, a.back().y, b.back().x, b.back().y);
    const double ayaw = std::atan2(a[1].y - a[0].y, a[1].x - a[0].x);
    const double byaw = std::atan2(b[1].y - b[0].y, b[1].x - b[0].x);
    m.heading_jump = std::fabs(wrapAngle(ayaw - byaw));
    return m;
  }

  bool referencesSimilar(const std::vector<HistoryPoint2D>& a,
                         const std::vector<HistoryPoint2D>& b) const {
    if (a.size() < 2 || b.size() < 2) {
      return false;
    }
    const ReferenceJumpMetrics m = referenceJumpMetrics(a, b);
    return m.span_jump <= cfg_.reference_output_max_span_jump &&
           m.start_jump <= cfg_.reference_output_max_start_jump &&
           m.endpoint_jump <= cfg_.reference_output_max_endpoint_jump &&
           m.heading_jump <= cfg_.reference_output_max_heading_jump;
  }

  void acceptPublishedReference(const std::vector<HistoryPoint2D>& world,
                                const ros::Time& stamp,
                                const std::string& mode,
                                const std::string& source,
                                TrackerDebugState* debug) const {
    if (world.size() < 2) {
      return;
    }
    const ReferenceJumpMetrics jump =
        last_published_reference_world_.empty()
            ? ReferenceJumpMetrics()
            : referenceJumpMetrics(world, last_published_reference_world_);
    const ros::Time publish_stamp = stamp.isValid() ? stamp : ros::Time::now();
    const HistoryPoint2D& endpoint = world.back();
    const ros::Time endpoint_source_stamp = current_candidate_source_stamp_;
    const bool endpoint_source_valid = endpoint_source_stamp.isValid();
    const double endpoint_move =
        last_published_endpoint_valid_
            ? pointDistance(last_published_endpoint_x_,
                            last_published_endpoint_y_,
                            endpoint.x,
                            endpoint.y)
            : std::numeric_limits<double>::infinity();
    const bool endpoint_source_advanced =
        endpoint_source_valid &&
        (last_published_endpoint_source_stamp_.isZero() ||
         endpoint_source_stamp > last_published_endpoint_source_stamp_ + ros::Duration(1e-3));
    // Source freshness and geometric progress are different signals.  A
    // fixed-lag fit can receive a new tail sample without moving its smoothed
    // endpoint by 2 cm, so do not let that make a live reference look stale.
    if (endpoint_source_advanced) {
      last_published_endpoint_source_stamp_ = endpoint_source_stamp;
    }
    const bool endpoint_progressed =
        !last_published_endpoint_valid_ ||
        (endpoint_move > 0.02 && endpoint_source_advanced) ||
        (endpoint_move > 0.02 && !endpoint_source_valid);
    if (endpoint_progressed) {
      last_published_endpoint_x_ = endpoint.x;
      last_published_endpoint_y_ = endpoint.y;
      last_published_endpoint_valid_ = true;
      last_published_endpoint_progress_stamp_ =
          endpoint_source_valid ? endpoint_source_stamp : publish_stamp;
      if (!endpoint_source_valid) {
        last_published_endpoint_source_stamp_ = publish_stamp;
      }
    }
    last_published_reference_world_ = world;
    last_published_reference_stamp_ = publish_stamp;
    last_published_reference_mode_ = mode;
    last_published_reference_source_ = source;
    last_published_reference_reason_ =
        debug != NULL && !debug->reference_reason.empty() ? debug->reference_reason : "published";
    ++reference_update_seq_;
    if (debug != NULL) {
      const double span = historySpan(last_published_reference_world_);
      debug->reference_span = span;
      debug->published_reference_span = span;
      debug->published_reference_mode = mode;
      debug->published_reference_reason = last_published_reference_reason_;
      debug->held_by_hysteresis = false;
      debug->published_span_jump =
          std::isfinite(jump.span_jump) ? jump.span_jump : 0.0;
      debug->published_endpoint_jump =
          std::isfinite(jump.endpoint_jump) ? jump.endpoint_jump : 0.0;
      debug->published_start_jump =
          std::isfinite(jump.start_jump) ? jump.start_jump : 0.0;
      debug->published_heading_jump =
          std::isfinite(jump.heading_jump) ? jump.heading_jump : 0.0;
      debug->published_full_valid = debug->candidate_full_valid || debug->published_full_valid;
      debug->reference_update_seq = reference_update_seq_;
      debug->candidate_source_stamp =
          endpoint_source_valid ? endpoint_source_stamp.toSec() : -1.0;
      if (debug->published_span_target <= 1e-6) {
        debug->published_span_target = span;
      }
      if (last_published_endpoint_source_stamp_.isValid()) {
        debug->published_endpoint_age = 0.0;
        debug->tail_endpoint_age =
            endpoint_source_valid
                ? std::max(0.0, (publish_stamp - endpoint_source_stamp).toSec())
                : -1.0;
        debug->reference_latency_source =
            debug->tail_endpoint_age > std::max(0.05, cfg_.reference_stale_hold_time)
                ? "history_cache_replay"
                : "accepted_publish";
        debug->accepted_source_age = 0.0;
        debug->accepted_suffix_source_age = 0.0;
        debug->geometry_progress_age =
            last_published_endpoint_progress_stamp_.isValid()
                ? std::max(0.0,
                           (publish_stamp - last_published_endpoint_progress_stamp_).toSec())
                : -1.0;
      }
    }
  }

  std::vector<RefPoint> buildConnector(const RefPoint& goal) const {
    std::vector<RefPoint> out;
    const double dist = hypot2(goal.x, goal.y);
    if (dist < cfg_.reference_connector_enable_dist) {
      return out;
    }

    const double x_goal = std::max(0.05, goal.x);
    const double y_goal = goal.y;
    const double kappa1 =
        clampValue(goal.kappa, -0.6, 0.6) * cfg_.reference_connector_curvature_scale;
    double terminal_slope = std::tan(clampValue(goal.yaw, -0.85, 0.85));
    if (std::fabs(y_goal) > 1e-3) {
      const double sign_y = y_goal > 0.0 ? 1.0 : -1.0;
      terminal_slope = sign_y * clampValue(sign_y * terminal_slope,
                                           0.0,
                                           1.8 * std::fabs(y_goal) / x_goal);
    } else {
      terminal_slope = 0.0;
    }
    const double terminal_dy_du = terminal_slope * x_goal;
    const double a = terminal_dy_du - 2.0 * y_goal;
    const double b = 3.0 * y_goal - terminal_dy_du;
    const int steps =
        std::max(3, static_cast<int>(std::ceil(dist / std::max(0.03, cfg_.reference_connector_resolution))));
    out.reserve(static_cast<size_t>(steps) + 1);

    for (int i = 0; i < steps; ++i) {
      RefPoint pt;
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      const double tt = t * t;
      const double ttt = tt * t;
      const double dy_du = 3.0 * a * tt + 2.0 * b * t;
      const double d2y_du2 = 6.0 * a * t + 2.0 * b;
      pt.x = x_goal * t;
      pt.y = a * ttt + b * tt;
      pt.yaw = std::atan2(dy_du, x_goal);
      const double denom =
          std::pow(std::max(1e-6, x_goal * x_goal + dy_du * dy_du), 1.5);
      pt.kappa = clampValue((x_goal * d2y_du2) / denom, -0.8, 0.8);
      pt.kappa = (1.0 - t) * pt.kappa + t * kappa1;
      pt.t = t * goal.t;
      pt.v_ref = std::max(0.10, goal.v_ref * (0.45 + 0.55 * t));
      pt.a_ref = 0.0;
      out.push_back(pt);
    }

    out.push_back(goal);
    recomputeRefGeometry(&out);
    return out;
  }

  Config cfg_;
  mutable std::vector<HistoryPoint2D> last_good_reference_world_;
  mutable ros::Time last_good_reference_stamp_;
  mutable ros::Time last_good_suffix_source_stamp_;
  mutable std::vector<HistoryPoint2D> last_published_reference_world_;
  mutable ros::Time last_published_reference_stamp_;
  mutable std::string last_published_reference_mode_;
  mutable std::string last_published_reference_source_;
  mutable std::string last_published_reference_reason_;
  mutable bool last_published_endpoint_valid_ = false;
  mutable double last_published_endpoint_x_ = 0.0;
  mutable double last_published_endpoint_y_ = 0.0;
  mutable ros::Time last_published_endpoint_source_stamp_;
  mutable ros::Time last_published_endpoint_progress_stamp_;
  mutable ros::Time current_candidate_source_stamp_;
  mutable uint64_t reference_update_seq_ = 0;
  mutable std::vector<HistoryPoint2D> pending_reference_world_;
  mutable std::string pending_reference_mode_;
  mutable std::string pending_reference_source_;
  mutable int pending_reference_count_ = 0;
  mutable bool last_join_s_valid_ = false;
  mutable double last_join_s_ = 0.0;
  mutable int last_join_index_ = -1;
};

class FixedFrameTrajectoryReconstructor {
 public:
  FixedFrameTrajectoryReconstructor()
      : nh_(),
        pnh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        ego_motion_(loadEgoMotionConfig()),
        projector_(loadProjectorConfig()),
        tracker_(loadTrackerConfig()),
        path_generator_(loadPathConfig()) {
    base_frame_ = pnh_.param<std::string>("base_frame", "base_link");
    odom_frame_ = pnh_.param<std::string>("odom_frame", "reconstruction_odom");
    tracked_topic_ = pnh_.param<std::string>("tracked_topic", "/pointpillars/tracked_objects");
    car_topic_ = pnh_.param<std::string>("car_topic", "/car_message");
    carvel_topic_ = pnh_.param<std::string>("carvel_topic", "/carvel");
    imu_topic_ = pnh_.param<std::string>("imu_topic", "/imu");
    odom_topic_ = pnh_.param<std::string>("odom_topic", "/NED_odometry");
    raw_topic_ = pnh_.param<std::string>("raw_topic", "/planner/raw_path");
    reference_topic_ = pnh_.param<std::string>("reference_topic", "/planner/reference_path");
    reference_status_topic_ =
        pnh_.param<std::string>("reference_status_topic", "/planner/reference_status");
    final_traj_log_file_ =
      pnh_.param<std::string>("final_traj_log_file", "/home/ubuntu/front_car_final_trajectory.txt");
    publish_rate_hz_ = pnh_.param("publish_rate_hz", 30.0);
    enable_debug_csv_logs_ = pnh_.param("enable_debug_csv_logs", true);
    debug_log_dir_ = pnh_.param<std::string>("debug_log_dir", "/home/ubuntu/drive_car_copy/data");
    runtime_debug_file_ = pnh_.param<std::string>("runtime_debug_file", "runtime_debug.csv");
    history_relative_file_ =
        pnh_.param<std::string>("history_relative_file", "history_relative_trajectory.txt");
    history_status_file_ =
        pnh_.param<std::string>("history_status_file", "history_status.csv");
    lead_filter_state_file_ =
        pnh_.param<std::string>("lead_filter_state_file", "lead_filter_state.csv");
    observation_filter_file_ =
        pnh_.param<std::string>("observation_filter_file", "observation_filter.csv");
    reconstructed_relative_file_ =
        pnh_.param<std::string>("reconstructed_relative_file", "reconstructed_relative_trajectory.txt");

    wheel_speed_scale_ = pnh_.param("wheel_speed_scale", 0.001);
    speed_hint_scale_ = pnh_.param("speed_hint_scale", 1.0);
    steer_angle_scale_ = pnh_.param("steer_angle_scale", 1.0);
    steer_angle_bias_ = pnh_.param("steer_angle_bias", 0.0);
    steer_sign_ = pnh_.param("steer_sign", 1.0);
    steer_deadband_rad_ = pnh_.param("steer_deadband_rad", 0.002);
    lost_timeout_ = pnh_.param("lost_timeout", 0.60);
    min_motion_speed_for_yaw_ = pnh_.param("min_motion_speed_for_yaw", 0.15);
    allow_unstamped_tracked_fallback_ = pnh_.param("allow_unstamped_tracked_fallback", true);
    use_odom_pose_ = pnh_.param("use_odom_pose", true);
    enable_tf_pose_fallback_ = pnh_.param("enable_tf_pose_fallback", true);
    tf_world_frame_ = pnh_.param<std::string>("tf_world_frame", odom_frame_);
    tf_base_frame_ = pnh_.param<std::string>("tf_base_frame", base_frame_);
    tf_allow_latest_fallback_ = pnh_.param("tf_allow_latest_fallback", true);
    tf_pose_timeout_ = pnh_.param("tf_pose_timeout", 0.20);
    odom_position_scale_ = pnh_.param("odom_position_scale", 1.0);
    odom_yaw_sign_ = pnh_.param("odom_yaw_sign", 1.0);
    odom_yaw_bias_ = pnh_.param("odom_yaw_bias", 0.0);
    odom_speed_scale_ = pnh_.param("odom_speed_scale", 1.0);

    tracked_sub_ = nh_.subscribe(tracked_topic_, 5, &FixedFrameTrajectoryReconstructor::trackedCallback, this);
    car_sub_ = nh_.subscribe(car_topic_, 20, &FixedFrameTrajectoryReconstructor::carCallback, this);
    carvel_sub_ = nh_.subscribe(carvel_topic_, 20, &FixedFrameTrajectoryReconstructor::carvelCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 50, &FixedFrameTrajectoryReconstructor::imuCallback, this);
    if (use_odom_pose_) {
      odom_sub_ = nh_.subscribe(odom_topic_, 20, &FixedFrameTrajectoryReconstructor::odomCallback, this);
    }
    raw_pub_ = nh_.advertise<nav_msgs::Path>(raw_topic_, 1);
    reference_pub_ = nh_.advertise<nav_msgs::Path>(reference_topic_, 1);
    reference_status_pub_ = nh_.advertise<std_msgs::String>(reference_status_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
                             &FixedFrameTrajectoryReconstructor::timerCallback,
                             this);

    bool need_log_header = false;
    {
      std::ifstream existing_log(final_traj_log_file_.c_str(), std::ios::in);
      need_log_header = !existing_log.good() || existing_log.peek() == std::ifstream::traits_type::eof();
    }

    final_traj_log_stream_.open(final_traj_log_file_.c_str(), std::ios::out | std::ios::app);
    if (!final_traj_log_stream_.is_open()) {
      ROS_WARN("trajectory_reconstructor failed to open fixed-frame trajectory log file: %s",
               final_traj_log_file_.c_str());
    } else if (need_log_header) {
      final_traj_log_stream_ << "timestamp,x,y,yaw\n";
      final_traj_log_stream_.flush();
    }
    initializeRuntimeDebugLog();
    initializeLegacyDebugLogs();

    ROS_INFO("trajectory_reconstructor started | tracked=%s car=%s carvel=%s imu=%s odom_pose=%s tf=%s raw=%s ref=%s base=%s odom=%s",
             tracked_topic_.c_str(),
             car_topic_.c_str(),
             carvel_topic_.c_str(),
             imu_topic_.c_str(),
             use_odom_pose_ ? odom_topic_.c_str() : "disabled",
             enable_tf_pose_fallback_ ? tf_world_frame_.c_str() : "disabled",
             raw_topic_.c_str(),
             reference_topic_.c_str(),
             base_frame_.c_str(),
             odom_frame_.c_str());
    ROS_INFO("trajectory_reconstructor fixed-frame trajectory logging | file=%s enabled=%d",
             final_traj_log_file_.c_str(),
             final_traj_log_stream_.is_open() ? 1 : 0);
  }

 private:
  void initializeRuntimeDebugLog() {
    if (!enable_debug_csv_logs_) {
      return;
    }
    mkdir(debug_log_dir_.c_str(), 0775);
    runtime_debug_stream_.open((debug_log_dir_ + "/" + runtime_debug_file_).c_str(),
                               std::ios::out | std::ios::trunc);
    if (!runtime_debug_stream_) {
      ROS_WARN("trajectory_reconstructor failed to open runtime debug csv: %s/%s",
               debug_log_dir_.c_str(),
               runtime_debug_file_.c_str());
      return;
    }
    runtime_debug_stream_
        << "stamp,lock_active,lead_moving,ego_standstill,raw_points,ref_points,world_points,"
        << "lead_x,lead_y,ego_speed,ego_yaw_rate,wheel_speed,speed_hint,steer,"
        << "steer_raw,steer_scaled,steer_scale_suspect,bicycle_yaw_rate_from_steer,"
        << "imu_vs_bicycle_yaw_rate_error,"
        << "ego_pose_source,ego_pose_valid,ego_pose_age,ego_delta_yaw,ego_delta_xy,ego_yaw_uncertainty,"
        << "ned_odom_received,tf_pose_received,imu_yaw_rate_used,"
        << "fixed_frame_append_valid,fixed_frame_progress,fixed_frame_kink_angle,"
        << "history_yaw_jump,history_curvature_peak,reference_from_fixed_frame,"
        << "reference_backtrack_reject,reference_splice_kappa_max,reference_splice_yaw_jump,reference_join_retry_count,"
        << "ff_speed,mean_ego_speed,static_hyp_res,hs_score,hd_score,accum_s,accum_t,"
        << "confirmed_count,tail_count,obs_version,input_pose_count,valid_candidate_count,"
        << "lock_reject_count,history_size,raw_span,reference_span,"
        << "candidate_reference_span,published_reference_span,published_reference_mode,"
        << "published_reference_reason,published_span_jump,published_endpoint_jump,"
        << "published_start_jump,published_heading_jump,held_by_hysteresis,"
        << "candidate_full_valid,published_full_valid,reference_full_max_yaw_jump,"
        << "reference_full_backtrack_count,reference_full_kappa_max,reference_cut_prefix_points,"
        << "reference_output_growth_limited,published_span_target,reference_reject_full_reason,"
        << "actual_published_points,actual_published_span,actual_published_mode,actual_published_reason,"
        << "published_geometry_valid,full_audit_hard_valid,full_audit_kappa_limited,"
        << "candidate_block_reason,held_publish_reason,motion_tail_direct_publish,"
        << "stable_segment_count,selected_segment_span,selected_segment_score,reference_hold_age,"
        << "history_window_commit,provisional_tail_points,provisional_tail_span,provisional_tail_age,"
        << "motion_tail_points,motion_tail_span,motion_tail_age,"
        << "static_anchor_suppressed_by_motion,static_anchor_blocked_by_motion_tail,"
        << "fixed_lag_commit,fixed_lag_commit_span,reference_trackable_raw,"
        << "motion_tail_used_as_reference,motion_tail_promoted_to_reference,"
        << "provisional_publish_block_reason,provisional_tail_block_detail,"
        << "reference_empty_duration,reference_mode_age,confirmed_append_block_reason,"
        << "publisher_action,published_hold_age,append_stall_age,candidate_jump_to_published,"
        << "published_stale_age,published_endpoint_speed,reference_stale,"
        << "motion_tail_extension_used,far_start_recovered_by_tail,stale_hold_demoted,"
        << "motion_tail_hard_valid,motion_tail_block_reason,unique_tail_points,unique_tail_span,"
        << "unique_tail_time,hold_forced_release,shrink_limited_publish,"
        << "candidate_vs_published_span_diff,raw_direct_publish,connector_used,"
        << "connector_join_index,connector_span,suffix_span,stitched_span,reference_source,"
        << "reference_estimator_confidence,confirmed_suffix_span,tail_suffix_span,"
        << "suffix_after_join_span,join_reject_reason,reference_state_update_alpha,"
        << "published_update_mode,"
        << "splice_preserved_suffix_span,splice_trim_rejected_connector_only,smooth_output_span,"
        << "candidate_full_span_before_audit,candidate_full_span_after_audit,"
        << "tail_extension_latency,fixed_lag_tail_used,"
        << "published_suffix_preserved_ok,connector_only_publish_blocked,reference_refresh_latency,"
        << "published_endpoint_age,tail_endpoint_age,reference_latency_source,"
        << "suffix_estimator_tail_age,suffix_estimator_output_span,stale_hold_block_reason,"
        << "suffix_extension_span,suffix_extension_growth_limited,reconnected_last_good_suffix,"
        << "join_connector_ratio,connector_inner_cut_score,connector_lateral_shortcut,anti_inner_cut_rejoin,"
        << "candidate_source,reference_empty_reason_final,"
        << "suffix_wiggle_score,suffix_sign_flip_count,suffix_smooth_rms,suffix_curvature_p95,"
        << "reference_freshness_state,append_gate_softened,tail_point_weight_min,tail_point_weight_max,"
        << "tail_point_weight_mean,tail_along_weight_mean,tail_cross_weight_mean,"
        << "cross_hold_active,cross_hold_reason,ego_turn_contamination_score,heading_decoupled_by_ego_turn,"
        << "suffix_centerline_fit_active,tail_cross_weight_applied,ego_turn_cross_hold_active,"
        << "bbox_heading_used_for_history,lateral_innovation_gate,lateral_innovation_frenet,"
        << "cross_residual_filtered,fixed_frame_lateral_velocity,fixed_frame_lateral_velocity_filtered,"
        << "ego_pose_deskew_age,join_index_smooth,join_index_jump,join_state_s,join_reselect_reason,"
        << "publisher_retry_join_count,bootstrap_stitched,"
        << "suffix_s_wave_suppressed,suffix_wiggle_before,suffix_wiggle_after,"
        << "reference_health_score,reference_publish_forced_by_tail,"
        << "ego_motion_decoupled_tail_yaw,tail_point_weight,suffix_fit_mode,"
        << "suffix_curvature_rate_p95,reference_flatness_valid,reference_tracking_confidence,"
        << "accepted_source_age,geometry_progress_age,candidate_source_stamp,reference_update_seq,"
        << "curve_refit_active,curve_refit_rms,reconstruction_compute_ms,"
        << "tail_sample_action,tail_pending_distance,tail_pending_time,"
        << "accepted_suffix_source_age,join_skip_span,final_connector_ratio,"
        << "tangent_progress_min,fallback_suffix_age,"
        << "outlier_gate_reason,"
        << "selected_segment_source,projector_reason,history_reason,raw_reason,reference_reason,"
        << "reference_publish_mode\n";
  }

  std::string debugFilePath(const std::string& file) const {
    if (file.empty()) {
      return file;
    }
    if (file[0] == '/') {
      return file;
    }
    return debug_log_dir_ + "/" + file;
  }

  void initializeLegacyDebugLogs() {
    if (!enable_debug_csv_logs_) {
      return;
    }
    mkdir(debug_log_dir_.c_str(), 0775);
    history_relative_stream_.open(debugFilePath(history_relative_file_).c_str(),
                                  std::ios::out | std::ios::trunc);
    history_status_stream_.open(debugFilePath(history_status_file_).c_str(),
                                std::ios::out | std::ios::trunc);
    lead_filter_stream_.open(debugFilePath(lead_filter_state_file_).c_str(),
                             std::ios::out | std::ios::trunc);
    observation_filter_stream_.open(debugFilePath(observation_filter_file_).c_str(),
                                    std::ios::out | std::ios::trunc);
    reconstructed_relative_stream_.open(debugFilePath(reconstructed_relative_file_).c_str(),
                                        std::ios::out | std::ios::trunc);

    if (history_status_stream_) {
      history_status_stream_
          << "stamp,lock_active,lead_moving,ego_standstill,raw_points,ref_points,world_points,"
          << "history_size,confirmed_count,tail_count,raw_span,reference_span,"
          << "candidate_reference_span,published_reference_span,published_reference_mode,"
          << "published_reference_reason,published_span_jump,published_endpoint_jump,"
          << "published_start_jump,published_heading_jump,held_by_hysteresis,"
          << "candidate_full_valid,published_full_valid,reference_full_max_yaw_jump,"
          << "reference_full_backtrack_count,reference_full_kappa_max,reference_cut_prefix_points,"
          << "reference_output_growth_limited,published_span_target,reference_reject_full_reason,"
          << "actual_published_points,actual_published_span,actual_published_mode,actual_published_reason,"
          << "published_geometry_valid,full_audit_hard_valid,full_audit_kappa_limited,"
          << "candidate_block_reason,held_publish_reason,motion_tail_direct_publish,"
          << "fixed_frame_append_valid,fixed_frame_progress,fixed_frame_kink_angle,"
          << "history_yaw_jump,history_curvature_peak,reference_from_fixed_frame,"
          << "reference_backtrack_reject,reference_splice_kappa_max,reference_splice_yaw_jump,reference_join_retry_count,"
          << "stable_segment_count,selected_segment_span,selected_segment_score,reference_hold_age,"
          << "history_window_commit,provisional_tail_points,provisional_tail_span,provisional_tail_age,"
          << "motion_tail_points,motion_tail_span,motion_tail_age,"
          << "static_anchor_suppressed_by_motion,static_anchor_blocked_by_motion_tail,"
          << "fixed_lag_commit,fixed_lag_commit_span,reference_trackable_raw,"
          << "motion_tail_used_as_reference,motion_tail_promoted_to_reference,"
          << "provisional_publish_block_reason,provisional_tail_block_detail,"
          << "reference_empty_duration,reference_mode_age,confirmed_append_block_reason,"
          << "publisher_action,published_hold_age,append_stall_age,candidate_jump_to_published,"
          << "published_stale_age,published_endpoint_speed,reference_stale,"
          << "motion_tail_extension_used,far_start_recovered_by_tail,stale_hold_demoted,"
          << "motion_tail_hard_valid,motion_tail_block_reason,unique_tail_points,unique_tail_span,"
          << "unique_tail_time,hold_forced_release,shrink_limited_publish,"
          << "candidate_vs_published_span_diff,raw_direct_publish,connector_used,"
          << "connector_join_index,connector_span,suffix_span,stitched_span,reference_source,"
          << "reference_estimator_confidence,confirmed_suffix_span,tail_suffix_span,"
          << "suffix_after_join_span,join_reject_reason,reference_state_update_alpha,"
          << "published_update_mode,"
          << "splice_preserved_suffix_span,splice_trim_rejected_connector_only,smooth_output_span,"
          << "candidate_full_span_before_audit,candidate_full_span_after_audit,"
          << "tail_extension_latency,fixed_lag_tail_used,"
          << "published_suffix_preserved_ok,connector_only_publish_blocked,reference_refresh_latency,"
          << "published_endpoint_age,tail_endpoint_age,reference_latency_source,"
          << "suffix_estimator_tail_age,suffix_estimator_output_span,stale_hold_block_reason,"
          << "suffix_extension_span,suffix_extension_growth_limited,reconnected_last_good_suffix,"
          << "join_connector_ratio,connector_inner_cut_score,connector_lateral_shortcut,anti_inner_cut_rejoin,"
          << "candidate_source,reference_empty_reason_final,"
          << "suffix_wiggle_score,suffix_sign_flip_count,suffix_smooth_rms,suffix_curvature_p95,"
          << "reference_freshness_state,append_gate_softened,tail_point_weight_min,tail_point_weight_max,"
          << "tail_point_weight_mean,tail_along_weight_mean,tail_cross_weight_mean,"
          << "cross_hold_active,cross_hold_reason,ego_turn_contamination_score,heading_decoupled_by_ego_turn,"
          << "suffix_centerline_fit_active,tail_cross_weight_applied,ego_turn_cross_hold_active,"
          << "bbox_heading_used_for_history,lateral_innovation_gate,lateral_innovation_frenet,"
          << "cross_residual_filtered,fixed_frame_lateral_velocity,fixed_frame_lateral_velocity_filtered,"
          << "ego_pose_deskew_age,join_index_smooth,join_index_jump,join_state_s,join_reselect_reason,"
          << "publisher_retry_join_count,bootstrap_stitched,"
          << "suffix_s_wave_suppressed,suffix_wiggle_before,suffix_wiggle_after,"
          << "reference_health_score,reference_publish_forced_by_tail,"
          << "ego_motion_decoupled_tail_yaw,tail_point_weight,suffix_fit_mode,"
          << "suffix_curvature_rate_p95,reference_flatness_valid,reference_tracking_confidence,"
          << "accepted_source_age,geometry_progress_age,candidate_source_stamp,reference_update_seq,"
          << "curve_refit_active,curve_refit_rms,reconstruction_compute_ms,"
          << "tail_sample_action,tail_pending_distance,tail_pending_time,"
          << "accepted_suffix_source_age,join_skip_span,final_connector_ratio,"
          << "tangent_progress_min,fallback_suffix_age,"
          << "outlier_gate_reason,"
          << "selected_segment_source,projector_reason,history_reason,raw_reason,reference_reason,"
          << "reference_publish_mode\n";
    }
    if (lead_filter_stream_) {
      lead_filter_stream_
          << "stamp,lock_active,lead_moving,lead_x,lead_y,ff_speed,mean_ego_speed,"
          << "innovation_version,accum_s,accum_t,ego_pose_source,ego_pose_valid,"
          << "ego_pose_age,ego_yaw_uncertainty\n";
    }
    if (observation_filter_stream_) {
      observation_filter_stream_
          << "stamp,obs_version,input_pose_count,valid_candidate_count,lock_reject_count,"
          << "projector_reason,ego_pose_valid,ego_pose_source,lead_x,lead_y,"
          << "fixed_frame_append_valid,fixed_frame_progress\n";
    }
    if (history_relative_stream_) {
      history_relative_stream_ << "# frame_stamp,index,x,y,yaw,curvature,source\n";
    }
    if (reconstructed_relative_stream_) {
      reconstructed_relative_stream_ << "# frame_stamp,index,x,y,yaw,curvature,source\n";
    }
    ROS_INFO("trajectory_reconstructor legacy debug logs reset in %s", debug_log_dir_.c_str());
  }

  void fillEgoDebug(const ros::Time& now, const EgoPose2D* ego) {
    debug_state_.ego_pose_valid = ego != NULL && ego_motion_.initialized();
    debug_state_.ego_pose_source = ego_motion_.poseSource();
    const ros::Time ext_stamp = ego_motion_.lastExternalPoseStamp();
    debug_state_.ego_pose_age =
        !ext_stamp.isZero() ? std::max(0.0, (now - ext_stamp).toSec()) : -1.0;
    debug_state_.ego_yaw_uncertainty = ego != NULL ? ego->sigma_yaw : 0.0;
    debug_state_.steer_raw = last_steer_raw_;
    debug_state_.steer_scaled = last_steer_scaled_;
    debug_state_.steer_scale_suspect = steer_scale_suspect_;
    debug_state_.bicycle_yaw_rate_from_steer = ego_motion_.bicycleYawRateEstimate();
    debug_state_.imu_vs_bicycle_yaw_rate_error =
        ego_motion_.usingFreshImuYawRate()
            ? std::fabs(ego_motion_.filteredImuYawRate() -
                        debug_state_.bicycle_yaw_rate_from_steer)
            : -1.0;
    if (ego_motion_.usingFreshImuYawRate() &&
        std::fabs(ego_motion_.filteredImuYawRate()) > 0.04 &&
        ego_motion_.wheelSpeed() > 0.08 &&
        std::fabs(debug_state_.bicycle_yaw_rate_from_steer) <
            0.15 * std::fabs(ego_motion_.filteredImuYawRate())) {
      debug_state_.steer_scale_suspect = true;
      steer_scale_suspect_ = true;
    }
    if (ego_motion_.usingFreshImuYawRate() &&
        std::fabs(ego_motion_.filteredImuYawRate()) > 0.03 &&
        std::fabs(debug_state_.bicycle_yaw_rate_from_steer) > 0.03 &&
        ego_motion_.filteredImuYawRate() * debug_state_.bicycle_yaw_rate_from_steer < 0.0) {
      debug_state_.steer_scale_suspect = true;
      steer_scale_suspect_ = true;
    }
    debug_state_.ned_odom_received = ned_odom_received_;
    debug_state_.tf_pose_received = tf_pose_received_;
    debug_state_.imu_yaw_rate_used = ego_motion_.usingFreshImuYawRate();
    if (ego != NULL && last_debug_ego_pose_valid_) {
      debug_state_.ego_delta_yaw = wrapAngle(ego->yaw - last_debug_ego_pose_.yaw);
      debug_state_.ego_delta_xy = pointDistance(last_debug_ego_pose_.x,
                                                last_debug_ego_pose_.y,
                                                ego->x,
                                                ego->y);
    } else {
      debug_state_.ego_delta_yaw = 0.0;
      debug_state_.ego_delta_xy = 0.0;
    }
    if (ego != NULL) {
      last_debug_ego_pose_ = *ego;
      last_debug_ego_pose_valid_ = true;
    }
  }

  void updateReferenceTimingDebug(const ros::Time& now,
                                  size_t ref_points,
                                  TrackerDebugState* debug) {
    if (debug == NULL) {
      return;
    }
    const std::string mode =
        !debug->actual_published_mode.empty() && debug->actual_published_mode != "init"
            ? debug->actual_published_mode
            : (debug->reference_publish_mode.empty()
                   ? std::string("unknown")
                   : debug->reference_publish_mode);
    if (last_reference_publish_mode_ != mode || reference_mode_since_.isZero()) {
      last_reference_publish_mode_ = mode;
      reference_mode_since_ = now;
      debug->reference_mode_age = 0.0;
    } else {
      debug->reference_mode_age = std::max(0.0, (now - reference_mode_since_).toSec());
    }

    if (ref_points == 0) {
      if (reference_empty_since_.isZero()) {
        reference_empty_since_ = now;
      }
      debug->reference_empty_duration =
          std::max(0.0, (now - reference_empty_since_).toSec());
    } else {
      reference_empty_since_ = ros::Time();
      debug->reference_empty_duration = 0.0;
    }

    if (debug->provisional_tail_block_detail == "none" &&
        debug->provisional_publish_block_reason != "none") {
      debug->provisional_tail_block_detail = debug->provisional_publish_block_reason;
    }
  }

  void publishReferenceStatus(const ros::Time& now,
                              const TrackerDebugState& debug,
                              size_t ref_points) {
    if (!reference_status_pub_) {
      return;
    }
    std_msgs::String msg;
    std::ostringstream ss;
    const std::string effective_publish_mode =
        !debug.actual_published_mode.empty() && debug.actual_published_mode != "init"
            ? debug.actual_published_mode
            : (debug.held_by_hysteresis && !debug.published_reference_mode.empty()
                   ? debug.published_reference_mode
                   : debug.reference_publish_mode);
    const std::string effective_reason =
        !debug.actual_published_reason.empty() && debug.actual_published_reason != "init"
            ? debug.actual_published_reason
            : (debug.held_by_hysteresis && !debug.published_reference_reason.empty()
                   ? debug.published_reference_reason
                   : debug.reference_reason);
    const bool has_published_path = ref_points >= 2 && debug.actual_published_span > 0.0;
    const bool good =
        has_published_path &&
        debug.ego_pose_valid &&
        (effective_reason == "reference_ready" ||
         effective_publish_mode == "ready") &&
        ref_points >= 8 &&
        debug.actual_published_span >= 0.80;
    const bool degraded_raw =
        has_published_path &&
        debug.ego_pose_valid &&
        (effective_reason.find("reference_degraded_raw") == 0 ||
         effective_publish_mode == "degraded_raw") &&
        ref_points >= 6 &&
        debug.actual_published_span >= 0.80;
    const bool provisional_tail =
        has_published_path &&
        debug.ego_pose_valid &&
        (effective_publish_mode == "provisional_tail" ||
         effective_reason == "reference_provisional_tail" ||
         effective_reason == "reference_stitched_tail" ||
         effective_reason == "reference_stitched_provisional" ||
         debug.selected_segment_source == "confirmed_tail" ||
         debug.selected_segment_source == "motion_tail" ||
         debug.selected_segment_source == "stitched_confirmed" ||
         debug.motion_tail_promoted_to_reference) &&
        ref_points >= 4 &&
        debug.actual_published_span >= 0.35;
    const bool stitched_reference =
        debug.reference_source == "stitched_confirmed" ||
        debug.reference_source == "stitched_tail";
    const bool provisional_tail_short =
        has_published_path &&
        debug.ego_pose_valid &&
        (effective_publish_mode == "provisional_tail" ||
         effective_reason == "reference_provisional_tail" ||
         effective_reason == "reference_stitched_tail" ||
         effective_reason == "reference_stitched_provisional" ||
         debug.selected_segment_source == "confirmed_tail" ||
         debug.selected_segment_source == "motion_tail" ||
         debug.selected_segment_source == "stitched_confirmed" ||
         debug.motion_tail_promoted_to_reference) &&
        ref_points >= 2 &&
        debug.actual_published_span < 0.35;
    const bool provisional_tail_curve_valid =
        provisional_tail &&
        debug.reference_splice_yaw_jump < 0.70 &&
        debug.reference_splice_kappa_max < 5.0;
    const std::string mode =
        (!has_published_path || debug.reference_freshness_state == "invalid") ? "INVALID" :
        (debug.reference_stale ? "DEGRADED_HELD_STALE" :
         (good ? "TRACK_GOOD" :
          (provisional_tail ? (stitched_reference ? "TRACK_PROVISIONAL_STITCHED"
                                                  : "TRACK_PROVISIONAL_TAIL") :
           (provisional_tail_short ? "TRACK_PROVISIONAL_TAIL_SHORT" :
           (degraded_raw ? "TRACK_DEGRADED_RAW" :
            (debug.ego_pose_valid ? "DEGRADED_HELD" : "EGO_POSE_UNCERTAIN"))))));
    double quality = 0.0;
    if (has_published_path) {
      if (debug.reference_stale) {
        quality = 0.35;
      } else if (good) {
        quality = 1.0;
      } else if (provisional_tail && stitched_reference) {
        quality = 0.72;
      } else if (provisional_tail) {
        quality = 0.62;
      } else if (provisional_tail_short) {
        quality = 0.45;
      } else if (degraded_raw) {
        quality = 0.72;
      } else {
        quality = debug.ego_pose_valid ? 0.55 : 0.25;
      }
    }
    const bool reference_internal_yaw_jump_ok =
        debug.reference_full_max_yaw_jump <= 0.80 &&
        debug.reference_full_backtrack_count == 0;
    const bool reference_control_trackable_hint =
        has_published_path &&
        !debug.reference_stale &&
        debug.published_geometry_valid &&
        reference_internal_yaw_jump_ok &&
        quality >= 0.55 &&
        debug.actual_published_span >= 0.60;
    const bool reference_control_resync_allowed =
        reference_control_trackable_hint &&
        debug.reference_freshness_state == "fresh" &&
        debug.reference_health_score >= 0.70 &&
        debug.actual_published_span >= 0.80;
    ss << "mode=" << mode
       << " reason=" << (debug.ego_pose_valid ? effective_reason : "ego_pose_uncertain")
       << " quality=" << quality
       << " age=" << debug.reference_mode_age
       << " ego_pose_age=" << debug.ego_pose_age
       << " ref_points=" << ref_points
       << " span=" << debug.actual_published_span
       << " provisional_tail_short=" << (provisional_tail_short ? 1 : 0)
       << " provisional_tail_curve_valid=" << (provisional_tail_curve_valid ? 1 : 0)
       << " publish_mode=" << debug.reference_publish_mode
       << " held_by_hysteresis=" << (debug.held_by_hysteresis ? 1 : 0)
       << " published_span_jump=" << debug.published_span_jump
       << " reference_geometry_valid=" << (debug.published_geometry_valid ? 1 : 0)
       << " reference_full_max_yaw_jump=" << debug.reference_full_max_yaw_jump
       << " reference_full_backtrack_count=" << debug.reference_full_backtrack_count
       << " reference_full_kappa_max=" << debug.reference_full_kappa_max
       << " reference_output_growth_limited=" << (debug.reference_output_growth_limited ? 1 : 0)
       << " reference_full_kappa_limited=" << (debug.full_audit_kappa_limited ? 1 : 0)
       << " published_span_target=" << debug.published_span_target
       << " published_stale_age=" << debug.published_stale_age
       << " reference_stale=" << (debug.reference_stale ? 1 : 0)
       << " published_endpoint_speed=" << debug.published_endpoint_speed
       << " published_mode=" << debug.published_reference_mode
       << " published_reason=" << debug.published_reference_reason
       << " actual_published_points=" << debug.actual_published_points
       << " actual_published_span=" << debug.actual_published_span
       << " actual_published_mode=" << debug.actual_published_mode
       << " actual_published_reason=" << debug.actual_published_reason
       << " publisher_action=" << debug.publisher_action
       << " published_hold_age=" << debug.published_hold_age
       << " append_stall_age=" << debug.append_stall_age
       << " candidate_jump_to_published=" << debug.candidate_jump_to_published
       << " motion_tail_hard_valid=" << (debug.motion_tail_hard_valid ? 1 : 0)
       << " motion_tail_block_reason=" << debug.motion_tail_block_reason
       << " unique_tail_points=" << debug.unique_tail_points
       << " unique_tail_span=" << debug.unique_tail_span
       << " unique_tail_time=" << debug.unique_tail_time
       << " hold_forced_release=" << (debug.hold_forced_release ? 1 : 0)
       << " shrink_limited_publish=" << (debug.shrink_limited_publish ? 1 : 0)
       << " candidate_vs_published_span_diff=" << debug.candidate_vs_published_span_diff
       << " outlier_gate_reason=" << debug.outlier_gate_reason
       << " selected_source=" << debug.selected_segment_source
       << " reference_source=" << debug.reference_source
       << " reference_estimator_confidence=" << debug.reference_estimator_confidence
       << " confirmed_suffix_span=" << debug.confirmed_suffix_span
       << " tail_suffix_span=" << debug.tail_suffix_span
       << " suffix_after_join_span=" << debug.suffix_after_join_span
       << " join_reject_reason=" << debug.join_reject_reason
       << " reference_state_update_alpha=" << debug.reference_state_update_alpha
       << " published_update_mode=" << debug.published_update_mode
       << " splice_preserved_suffix_span=" << debug.splice_preserved_suffix_span
       << " splice_trim_rejected_connector_only=" << (debug.splice_trim_rejected_connector_only ? 1 : 0)
       << " smooth_output_span=" << debug.smooth_output_span
       << " candidate_full_span_before_audit=" << debug.candidate_full_span_before_audit
       << " candidate_full_span_after_audit=" << debug.candidate_full_span_after_audit
       << " tail_extension_latency=" << debug.tail_extension_latency
       << " fixed_lag_tail_used=" << (debug.fixed_lag_tail_used ? 1 : 0)
       << " published_suffix_preserved_ok=" << (debug.published_suffix_preserved_ok ? 1 : 0)
       << " connector_only_publish_blocked=" << (debug.connector_only_publish_blocked ? 1 : 0)
       << " reference_refresh_latency=" << debug.reference_refresh_latency
       << " published_endpoint_age=" << debug.published_endpoint_age
       << " tail_endpoint_age=" << debug.tail_endpoint_age
       << " reference_latency_source=" << debug.reference_latency_source
       << " accepted_source_age=" << debug.accepted_source_age
       << " geometry_progress_age=" << debug.geometry_progress_age
       << " candidate_source_stamp=" << debug.candidate_source_stamp
       << " reference_update_seq=" << debug.reference_update_seq
       << " curve_refit_active=" << (debug.curve_refit_active ? 1 : 0)
       << " curve_refit_rms=" << debug.curve_refit_rms
       << " reconstruction_compute_ms=" << debug.reconstruction_compute_ms
       << " tail_sample_action=" << debug.tail_sample_action
       << " tail_pending_distance=" << debug.tail_pending_distance
       << " tail_pending_time=" << debug.tail_pending_time
       << " accepted_suffix_source_age=" << debug.accepted_suffix_source_age
       << " join_skip_span=" << debug.join_skip_span
       << " final_connector_ratio=" << debug.final_connector_ratio
       << " tangent_progress_min=" << debug.tangent_progress_min
       << " fallback_suffix_age=" << debug.fallback_suffix_age
       << " suffix_estimator_tail_age=" << debug.suffix_estimator_tail_age
       << " suffix_estimator_output_span=" << debug.suffix_estimator_output_span
       << " stale_hold_block_reason=" << debug.stale_hold_block_reason
       << " suffix_extension_span=" << debug.suffix_extension_span
       << " suffix_extension_growth_limited=" << (debug.suffix_extension_growth_limited ? 1 : 0)
       << " reconnected_last_good_suffix=" << (debug.reconnected_last_good_suffix ? 1 : 0)
       << " join_connector_ratio=" << debug.join_connector_ratio
       << " candidate_source=" << debug.candidate_source
       << " reference_empty_reason_final=" << debug.reference_empty_reason_final
       << " suffix_wiggle_score=" << debug.suffix_wiggle_score
       << " suffix_sign_flip_count=" << debug.suffix_sign_flip_count
       << " suffix_smooth_rms=" << debug.suffix_smooth_rms
       << " suffix_curvature_p95=" << debug.suffix_curvature_p95
       << " reference_freshness_state=" << debug.reference_freshness_state
       << " append_gate_softened=" << (debug.append_gate_softened ? 1 : 0)
	       << " tail_point_weight_min=" << debug.tail_point_weight_min
	       << " tail_point_weight_max=" << debug.tail_point_weight_max
	       << " tail_point_weight_mean=" << debug.tail_point_weight_mean
	       << " tail_along_weight_mean=" << debug.tail_along_weight_mean
	       << " tail_cross_weight_mean=" << debug.tail_cross_weight_mean
	       << " cross_hold_active=" << (debug.cross_hold_active ? 1 : 0)
	       << " cross_hold_reason=" << debug.cross_hold_reason
	       << " ego_turn_contamination_score=" << debug.ego_turn_contamination_score
	       << " heading_decoupled_by_ego_turn=" << (debug.heading_decoupled_by_ego_turn ? 1 : 0)
	       << " bbox_heading_used_for_history=" << (debug.bbox_heading_used_for_history ? 1 : 0)
	       << " lateral_innovation_gate=" << debug.lateral_innovation_gate
	       << " lateral_innovation_frenet=" << debug.lateral_innovation_frenet
	       << " cross_residual_filtered=" << debug.cross_residual_filtered
	       << " fixed_frame_lateral_velocity=" << debug.fixed_frame_lateral_velocity
	       << " fixed_frame_lateral_velocity_filtered=" << debug.fixed_frame_lateral_velocity_filtered
	       << " ego_pose_deskew_age=" << debug.ego_pose_deskew_age
	       << " join_index_smooth=" << debug.join_index_smooth
	       << " join_index_jump=" << debug.join_index_jump
	       << " join_state_s=" << debug.join_state_s
	       << " join_reselect_reason=" << debug.join_reselect_reason
	       << " publisher_retry_join_count=" << debug.publisher_retry_join_count
	       << " bootstrap_stitched=" << (debug.bootstrap_stitched ? 1 : 0)
	       << " suffix_s_wave_suppressed=" << (debug.suffix_s_wave_suppressed ? 1 : 0)
       << " suffix_wiggle_before=" << debug.suffix_wiggle_before
       << " suffix_wiggle_after=" << debug.suffix_wiggle_after
       << " suffix_curvature_rate_p95=" << debug.suffix_curvature_rate_p95
       << " reference_health_score=" << debug.reference_health_score
       << " ego_motion_decoupled_tail_yaw=" << (debug.ego_motion_decoupled_tail_yaw ? 1 : 0)
       << " tail_point_weight=" << debug.tail_point_weight
       << " steer_raw=" << debug.steer_raw
       << " steer_scaled=" << debug.steer_scaled
       << " steer_scale_suspect=" << (debug.steer_scale_suspect ? 1 : 0)
       << " bicycle_yaw_rate_from_steer=" << debug.bicycle_yaw_rate_from_steer
       << " imu_vs_bicycle_yaw_rate_error=" << debug.imu_vs_bicycle_yaw_rate_error
       << " suffix_fit_mode=" << debug.suffix_fit_mode
       << " reference_flatness_valid=" << (debug.reference_flatness_valid ? 1 : 0)
       << " reference_tracking_confidence=" << debug.reference_tracking_confidence
       << " reference_control_resync_allowed=" << (reference_control_resync_allowed ? 1 : 0)
       << " reference_internal_yaw_jump_ok=" << (reference_internal_yaw_jump_ok ? 1 : 0)
       << " reference_control_trackable_hint=" << (reference_control_trackable_hint ? 1 : 0)
       << " reference_publish_forced_by_tail=" << (debug.reference_publish_forced_by_tail ? 1 : 0)
       << " connector_used=" << (debug.connector_used ? 1 : 0)
       << " connector_join_index=" << debug.connector_join_index
       << " connector_span=" << debug.connector_span
       << " suffix_span=" << debug.suffix_span
       << " stitched_span=" << debug.stitched_span
       << " raw_direct_publish=" << (debug.raw_direct_publish ? 1 : 0)
       << " reference_connector_to_target=0"
       << " stamp=" << now.toSec();
    msg.data = ss.str();
    reference_status_pub_.publish(msg);
  }

  void writeRuntimeDebug(const ros::Time& now,
                         const TrackerDebugState& debug,
                         bool lock_active,
                         bool lead_moving,
                         bool ego_standstill,
                         size_t raw_points,
                         size_t ref_points,
                         size_t world_points,
                         double lead_x,
                         double lead_y,
                         double speed,
                         double yaw_rate,
                         double wheel_speed,
                         double speed_hint,
                         double steer,
                         uint64_t obs_ver) {
    if (!enable_debug_csv_logs_ || !runtime_debug_stream_) {
      return;
    }
    runtime_debug_stream_ << std::fixed << std::setprecision(6)
                          << now.toSec() << ","
                          << (lock_active ? 1 : 0) << ","
                          << (lead_moving ? 1 : 0) << ","
                          << (ego_standstill ? 1 : 0) << ","
                          << raw_points << ","
                          << ref_points << ","
                          << world_points << ","
                          << lead_x << ","
                          << lead_y << ","
                          << speed << ","
                          << yaw_rate << ","
                          << wheel_speed << ","
                          << speed_hint << ","
                          << steer << ","
                          << debug.steer_raw << ","
                          << debug.steer_scaled << ","
                          << (debug.steer_scale_suspect ? 1 : 0) << ","
                          << debug.bicycle_yaw_rate_from_steer << ","
                          << debug.imu_vs_bicycle_yaw_rate_error << ","
                          << debug.ego_pose_source << ","
                          << (debug.ego_pose_valid ? 1 : 0) << ","
                          << debug.ego_pose_age << ","
                          << debug.ego_delta_yaw << ","
                          << debug.ego_delta_xy << ","
                          << debug.ego_yaw_uncertainty << ","
                          << (debug.ned_odom_received ? 1 : 0) << ","
                          << (debug.tf_pose_received ? 1 : 0) << ","
                          << (debug.imu_yaw_rate_used ? 1 : 0) << ","
                          << (debug.fixed_frame_append_valid ? 1 : 0) << ","
                          << debug.fixed_frame_progress << ","
                          << debug.fixed_frame_kink_angle << ","
                          << debug.history_yaw_jump << ","
                          << debug.history_curvature_peak << ","
                          << (debug.reference_from_fixed_frame ? 1 : 0) << ","
                          << (debug.reference_backtrack_reject ? 1 : 0) << ","
                          << debug.reference_splice_kappa_max << ","
                          << debug.reference_splice_yaw_jump << ","
                          << debug.reference_join_retry_count << ","
                          << debug.ff_speed << ","
                          << debug.mean_ego_speed << ","
                          << debug.static_hyp_res << ","
                          << debug.hs_score << ","
                          << debug.hd_score << ","
                          << debug.accum_s << ","
                          << debug.accum_t << ","
                          << debug.confirmed_count << ","
                          << debug.tail_count << ","
                          << static_cast<unsigned long long>(obs_ver) << ","
                          << debug.input_pose_count << ","
                          << debug.valid_candidate_count << ","
                          << debug.lock_reject_count << ","
                          << debug.history_size << ","
                          << debug.raw_span << ","
                          << debug.reference_span << ","
                          << debug.candidate_reference_span << ","
                          << debug.published_reference_span << ","
                          << debug.published_reference_mode << ","
                          << debug.published_reference_reason << ","
                          << debug.published_span_jump << ","
                          << debug.published_endpoint_jump << ","
                          << debug.published_start_jump << ","
                          << debug.published_heading_jump << ","
                          << (debug.held_by_hysteresis ? 1 : 0) << ","
                          << (debug.candidate_full_valid ? 1 : 0) << ","
                          << (debug.published_full_valid ? 1 : 0) << ","
                          << debug.reference_full_max_yaw_jump << ","
                          << debug.reference_full_backtrack_count << ","
                          << debug.reference_full_kappa_max << ","
                          << debug.reference_cut_prefix_points << ","
                          << (debug.reference_output_growth_limited ? 1 : 0) << ","
                          << debug.published_span_target << ","
                          << debug.reference_reject_full_reason << ","
                          << debug.actual_published_points << ","
                          << debug.actual_published_span << ","
                          << debug.actual_published_mode << ","
                          << debug.actual_published_reason << ","
                          << (debug.published_geometry_valid ? 1 : 0) << ","
                          << (debug.full_audit_hard_valid ? 1 : 0) << ","
                          << (debug.full_audit_kappa_limited ? 1 : 0) << ","
                          << debug.candidate_block_reason << ","
                          << debug.held_publish_reason << ","
                          << (debug.motion_tail_direct_publish ? 1 : 0) << ","
                          << debug.stable_segment_count << ","
                          << debug.selected_segment_span << ","
                          << debug.selected_segment_score << ","
                          << debug.reference_hold_age << ","
                          << (debug.history_window_commit ? 1 : 0) << ","
                          << debug.provisional_tail_points << ","
                          << debug.provisional_tail_span << ","
                          << debug.provisional_tail_age << ","
                          << debug.motion_tail_points << ","
                          << debug.motion_tail_span << ","
                          << debug.motion_tail_age << ","
                          << (debug.static_anchor_suppressed_by_motion ? 1 : 0) << ","
                          << (debug.static_anchor_blocked_by_motion_tail ? 1 : 0) << ","
                          << (debug.fixed_lag_commit ? 1 : 0) << ","
                          << debug.fixed_lag_commit_span << ","
                          << (debug.reference_trackable_raw ? 1 : 0) << ","
                          << (debug.motion_tail_used_as_reference ? 1 : 0) << ","
                          << (debug.motion_tail_promoted_to_reference ? 1 : 0) << ","
                          << debug.provisional_publish_block_reason << ","
                          << debug.provisional_tail_block_detail << ","
                          << debug.reference_empty_duration << ","
                          << debug.reference_mode_age << ","
                          << debug.confirmed_append_block_reason << ","
                          << debug.publisher_action << ","
                          << debug.published_hold_age << ","
                          << debug.append_stall_age << ","
                          << debug.candidate_jump_to_published << ","
                          << debug.published_stale_age << ","
                          << debug.published_endpoint_speed << ","
                          << (debug.reference_stale ? 1 : 0) << ","
                          << (debug.motion_tail_extension_used ? 1 : 0) << ","
                          << (debug.far_start_recovered_by_tail ? 1 : 0) << ","
                          << (debug.stale_hold_demoted ? 1 : 0) << ","
                          << (debug.motion_tail_hard_valid ? 1 : 0) << ","
                          << debug.motion_tail_block_reason << ","
                          << debug.unique_tail_points << ","
                          << debug.unique_tail_span << ","
                          << debug.unique_tail_time << ","
                          << (debug.hold_forced_release ? 1 : 0) << ","
                          << (debug.shrink_limited_publish ? 1 : 0) << ","
                          << debug.candidate_vs_published_span_diff << ","
                          << (debug.raw_direct_publish ? 1 : 0) << ","
                          << (debug.connector_used ? 1 : 0) << ","
                          << debug.connector_join_index << ","
                          << debug.connector_span << ","
                          << debug.suffix_span << ","
                          << debug.stitched_span << ","
                          << debug.reference_source << ","
                          << debug.reference_estimator_confidence << ","
                          << debug.confirmed_suffix_span << ","
                          << debug.tail_suffix_span << ","
                          << debug.suffix_after_join_span << ","
                          << debug.join_reject_reason << ","
                          << debug.reference_state_update_alpha << ","
                          << debug.published_update_mode << ","
                          << debug.splice_preserved_suffix_span << ","
                          << (debug.splice_trim_rejected_connector_only ? 1 : 0) << ","
                          << debug.smooth_output_span << ","
                          << debug.candidate_full_span_before_audit << ","
                          << debug.candidate_full_span_after_audit << ","
                          << debug.tail_extension_latency << ","
                          << (debug.fixed_lag_tail_used ? 1 : 0) << ","
                          << (debug.published_suffix_preserved_ok ? 1 : 0) << ","
                          << (debug.connector_only_publish_blocked ? 1 : 0) << ","
                          << debug.reference_refresh_latency << ","
                          << debug.published_endpoint_age << ","
                          << debug.tail_endpoint_age << ","
                          << debug.reference_latency_source << ","
                          << debug.suffix_estimator_tail_age << ","
                          << debug.suffix_estimator_output_span << ","
                          << debug.stale_hold_block_reason << ","
                          << debug.suffix_extension_span << ","
                          << (debug.suffix_extension_growth_limited ? 1 : 0) << ","
                          << (debug.reconnected_last_good_suffix ? 1 : 0) << ","
                          << debug.join_connector_ratio << ","
                          << debug.connector_inner_cut_score << ","
                          << debug.connector_lateral_shortcut << ","
                          << (debug.anti_inner_cut_rejoin ? 1 : 0) << ","
                          << debug.candidate_source << ","
                          << debug.reference_empty_reason_final << ","
                          << debug.suffix_wiggle_score << ","
                          << debug.suffix_sign_flip_count << ","
                          << debug.suffix_smooth_rms << ","
                          << debug.suffix_curvature_p95 << ","
                          << debug.reference_freshness_state << ","
                          << (debug.append_gate_softened ? 1 : 0) << ","
	                          << debug.tail_point_weight_min << ","
	                          << debug.tail_point_weight_max << ","
	                          << debug.tail_point_weight_mean << ","
	                          << debug.tail_along_weight_mean << ","
	                          << debug.tail_cross_weight_mean << ","
	                          << (debug.cross_hold_active ? 1 : 0) << ","
	                          << debug.cross_hold_reason << ","
	                          << debug.ego_turn_contamination_score << ","
	                          << (debug.heading_decoupled_by_ego_turn ? 1 : 0) << ","
	                          << (debug.suffix_centerline_fit_active ? 1 : 0) << ","
	                          << debug.tail_cross_weight_applied << ","
	                          << (debug.ego_turn_cross_hold_active ? 1 : 0) << ","
	                          << (debug.bbox_heading_used_for_history ? 1 : 0) << ","
	                          << debug.lateral_innovation_gate << ","
	                          << debug.lateral_innovation_frenet << ","
	                          << debug.cross_residual_filtered << ","
	                          << debug.fixed_frame_lateral_velocity << ","
	                          << debug.fixed_frame_lateral_velocity_filtered << ","
	                          << debug.ego_pose_deskew_age << ","
	                          << debug.join_index_smooth << ","
	                          << debug.join_index_jump << ","
	                          << debug.join_state_s << ","
	                          << debug.join_reselect_reason << ","
	                          << debug.publisher_retry_join_count << ","
	                          << (debug.bootstrap_stitched ? 1 : 0) << ","
	                          << (debug.suffix_s_wave_suppressed ? 1 : 0) << ","
                          << debug.suffix_wiggle_before << ","
                          << debug.suffix_wiggle_after << ","
                          << debug.reference_health_score << ","
                          << (debug.reference_publish_forced_by_tail ? 1 : 0) << ","
                          << (debug.ego_motion_decoupled_tail_yaw ? 1 : 0) << ","
                          << debug.tail_point_weight << ","
                          << debug.suffix_fit_mode << ","
                          << debug.suffix_curvature_rate_p95 << ","
                          << (debug.reference_flatness_valid ? 1 : 0) << ","
                          << debug.reference_tracking_confidence << ","
                          << debug.accepted_source_age << ","
                          << debug.geometry_progress_age << ","
                          << debug.candidate_source_stamp << ","
                          << static_cast<unsigned long long>(debug.reference_update_seq) << ","
                          << (debug.curve_refit_active ? 1 : 0) << ","
                          << debug.curve_refit_rms << ","
                          << debug.reconstruction_compute_ms << ","
                          << debug.tail_sample_action << ","
                          << debug.tail_pending_distance << ","
                          << debug.tail_pending_time << ","
                          << debug.accepted_suffix_source_age << ","
                          << debug.join_skip_span << ","
                          << debug.final_connector_ratio << ","
                          << debug.tangent_progress_min << ","
                          << debug.fallback_suffix_age << ","
                          << debug.outlier_gate_reason << ","
                          << debug.selected_segment_source << ","
                          << debug.projector_reason << ","
                          << debug.history_reason << ","
                          << debug.raw_reason << ","
                          << debug.reference_reason << ","
                          << debug.reference_publish_mode << "\n";
    runtime_debug_stream_.flush();
  }

  void writeLocalPathFrame(std::ofstream& stream,
                           const ros::Time& now,
                           const std::vector<LocalPathPoint>& points,
                           const std::string& source) {
    if (!enable_debug_csv_logs_ || !stream) {
      return;
    }
    stream << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < points.size(); ++i) {
      stream << now.toSec() << ","
             << i << ","
             << points[i].x << ","
             << points[i].y << ","
             << points[i].yaw << ","
             << points[i].curvature << ","
             << source << "\n";
    }
    stream.flush();
  }

  void writeLegacyDebugLogs(const ros::Time& now,
                            const TrackerDebugState& debug,
                            bool lock_active,
                            bool lead_moving,
                            bool ego_standstill,
                            const std::vector<LocalPathPoint>& raw_pts,
                            const std::vector<LocalPathPoint>& ref_pts,
                            size_t world_points,
                            double lead_x,
                            double lead_y,
                            uint64_t obs_ver) {
    if (!enable_debug_csv_logs_) {
      return;
    }
    if (history_status_stream_) {
      history_status_stream_ << std::fixed << std::setprecision(6)
                             << now.toSec() << ","
                             << (lock_active ? 1 : 0) << ","
                             << (lead_moving ? 1 : 0) << ","
                             << (ego_standstill ? 1 : 0) << ","
                             << raw_pts.size() << ","
                             << ref_pts.size() << ","
                             << world_points << ","
                             << debug.history_size << ","
                             << debug.confirmed_count << ","
                             << debug.tail_count << ","
                             << debug.raw_span << ","
                             << debug.reference_span << ","
                             << debug.candidate_reference_span << ","
                             << debug.published_reference_span << ","
                             << debug.published_reference_mode << ","
                             << debug.published_reference_reason << ","
                             << debug.published_span_jump << ","
                             << debug.published_endpoint_jump << ","
                             << debug.published_start_jump << ","
                             << debug.published_heading_jump << ","
                             << (debug.held_by_hysteresis ? 1 : 0) << ","
                             << (debug.candidate_full_valid ? 1 : 0) << ","
                             << (debug.published_full_valid ? 1 : 0) << ","
                             << debug.reference_full_max_yaw_jump << ","
                             << debug.reference_full_backtrack_count << ","
                             << debug.reference_full_kappa_max << ","
                             << debug.reference_cut_prefix_points << ","
                             << (debug.reference_output_growth_limited ? 1 : 0) << ","
                             << debug.published_span_target << ","
                             << debug.reference_reject_full_reason << ","
                             << debug.actual_published_points << ","
                             << debug.actual_published_span << ","
                             << debug.actual_published_mode << ","
                             << debug.actual_published_reason << ","
                             << (debug.published_geometry_valid ? 1 : 0) << ","
                             << (debug.full_audit_hard_valid ? 1 : 0) << ","
                             << (debug.full_audit_kappa_limited ? 1 : 0) << ","
                             << debug.candidate_block_reason << ","
                             << debug.held_publish_reason << ","
                             << (debug.motion_tail_direct_publish ? 1 : 0) << ","
                             << (debug.fixed_frame_append_valid ? 1 : 0) << ","
                             << debug.fixed_frame_progress << ","
                             << debug.fixed_frame_kink_angle << ","
                             << debug.history_yaw_jump << ","
                             << debug.history_curvature_peak << ","
                             << (debug.reference_from_fixed_frame ? 1 : 0) << ","
                             << (debug.reference_backtrack_reject ? 1 : 0) << ","
                             << debug.reference_splice_kappa_max << ","
                             << debug.reference_splice_yaw_jump << ","
                             << debug.reference_join_retry_count << ","
                             << debug.stable_segment_count << ","
                             << debug.selected_segment_span << ","
                             << debug.selected_segment_score << ","
                             << debug.reference_hold_age << ","
                             << (debug.history_window_commit ? 1 : 0) << ","
                             << debug.provisional_tail_points << ","
                             << debug.provisional_tail_span << ","
                             << debug.provisional_tail_age << ","
                             << debug.motion_tail_points << ","
                             << debug.motion_tail_span << ","
                             << debug.motion_tail_age << ","
                             << (debug.static_anchor_suppressed_by_motion ? 1 : 0) << ","
                             << (debug.static_anchor_blocked_by_motion_tail ? 1 : 0) << ","
                             << (debug.fixed_lag_commit ? 1 : 0) << ","
                             << debug.fixed_lag_commit_span << ","
                             << (debug.reference_trackable_raw ? 1 : 0) << ","
                             << (debug.motion_tail_used_as_reference ? 1 : 0) << ","
                             << (debug.motion_tail_promoted_to_reference ? 1 : 0) << ","
                             << debug.provisional_publish_block_reason << ","
                             << debug.provisional_tail_block_detail << ","
                             << debug.reference_empty_duration << ","
                             << debug.reference_mode_age << ","
                             << debug.confirmed_append_block_reason << ","
                             << debug.publisher_action << ","
                             << debug.published_hold_age << ","
                             << debug.append_stall_age << ","
                             << debug.candidate_jump_to_published << ","
                             << debug.published_stale_age << ","
                             << debug.published_endpoint_speed << ","
                             << (debug.reference_stale ? 1 : 0) << ","
                             << (debug.motion_tail_extension_used ? 1 : 0) << ","
                             << (debug.far_start_recovered_by_tail ? 1 : 0) << ","
                             << (debug.stale_hold_demoted ? 1 : 0) << ","
                             << (debug.motion_tail_hard_valid ? 1 : 0) << ","
                             << debug.motion_tail_block_reason << ","
                             << debug.unique_tail_points << ","
                             << debug.unique_tail_span << ","
                             << debug.unique_tail_time << ","
                             << (debug.hold_forced_release ? 1 : 0) << ","
                             << (debug.shrink_limited_publish ? 1 : 0) << ","
                             << debug.candidate_vs_published_span_diff << ","
                             << (debug.raw_direct_publish ? 1 : 0) << ","
                             << (debug.connector_used ? 1 : 0) << ","
                             << debug.connector_join_index << ","
                             << debug.connector_span << ","
                             << debug.suffix_span << ","
                             << debug.stitched_span << ","
                             << debug.reference_source << ","
                             << debug.reference_estimator_confidence << ","
                             << debug.confirmed_suffix_span << ","
                             << debug.tail_suffix_span << ","
                             << debug.suffix_after_join_span << ","
                             << debug.join_reject_reason << ","
                             << debug.reference_state_update_alpha << ","
                             << debug.published_update_mode << ","
                             << debug.splice_preserved_suffix_span << ","
                             << (debug.splice_trim_rejected_connector_only ? 1 : 0) << ","
                             << debug.smooth_output_span << ","
                             << debug.candidate_full_span_before_audit << ","
                             << debug.candidate_full_span_after_audit << ","
                             << debug.tail_extension_latency << ","
                             << (debug.fixed_lag_tail_used ? 1 : 0) << ","
                             << (debug.published_suffix_preserved_ok ? 1 : 0) << ","
                             << (debug.connector_only_publish_blocked ? 1 : 0) << ","
                             << debug.reference_refresh_latency << ","
                             << debug.published_endpoint_age << ","
                             << debug.tail_endpoint_age << ","
                             << debug.reference_latency_source << ","
                             << debug.suffix_estimator_tail_age << ","
                             << debug.suffix_estimator_output_span << ","
                             << debug.stale_hold_block_reason << ","
                             << debug.suffix_extension_span << ","
                             << (debug.suffix_extension_growth_limited ? 1 : 0) << ","
                             << (debug.reconnected_last_good_suffix ? 1 : 0) << ","
                             << debug.join_connector_ratio << ","
                             << debug.connector_inner_cut_score << ","
                             << debug.connector_lateral_shortcut << ","
                             << (debug.anti_inner_cut_rejoin ? 1 : 0) << ","
                             << debug.candidate_source << ","
                             << debug.reference_empty_reason_final << ","
                             << debug.suffix_wiggle_score << ","
                             << debug.suffix_sign_flip_count << ","
                             << debug.suffix_smooth_rms << ","
                             << debug.suffix_curvature_p95 << ","
                             << debug.reference_freshness_state << ","
                             << (debug.append_gate_softened ? 1 : 0) << ","
                             << debug.tail_point_weight_min << ","
                             << debug.tail_point_weight_max << ","
                             << debug.tail_point_weight_mean << ","
                             << debug.tail_along_weight_mean << ","
                             << debug.tail_cross_weight_mean << ","
                             << (debug.cross_hold_active ? 1 : 0) << ","
                             << debug.cross_hold_reason << ","
                             << debug.ego_turn_contamination_score << ","
                             << (debug.heading_decoupled_by_ego_turn ? 1 : 0) << ","
                             << (debug.suffix_centerline_fit_active ? 1 : 0) << ","
                             << debug.tail_cross_weight_applied << ","
                             << (debug.ego_turn_cross_hold_active ? 1 : 0) << ","
                             << (debug.bbox_heading_used_for_history ? 1 : 0) << ","
                             << debug.lateral_innovation_gate << ","
                             << debug.lateral_innovation_frenet << ","
                             << debug.cross_residual_filtered << ","
                             << debug.fixed_frame_lateral_velocity << ","
                             << debug.fixed_frame_lateral_velocity_filtered << ","
                             << debug.ego_pose_deskew_age << ","
                             << debug.join_index_smooth << ","
                             << debug.join_index_jump << ","
                             << debug.join_state_s << ","
                             << debug.join_reselect_reason << ","
                             << debug.publisher_retry_join_count << ","
                             << (debug.bootstrap_stitched ? 1 : 0) << ","
                             << (debug.suffix_s_wave_suppressed ? 1 : 0) << ","
                             << debug.suffix_wiggle_before << ","
                             << debug.suffix_wiggle_after << ","
                             << debug.reference_health_score << ","
                             << (debug.reference_publish_forced_by_tail ? 1 : 0) << ","
                             << (debug.ego_motion_decoupled_tail_yaw ? 1 : 0) << ","
                             << debug.tail_point_weight << ","
                             << debug.suffix_fit_mode << ","
                             << debug.suffix_curvature_rate_p95 << ","
                             << (debug.reference_flatness_valid ? 1 : 0) << ","
                             << debug.reference_tracking_confidence << ","
                             << debug.accepted_source_age << ","
                             << debug.geometry_progress_age << ","
                             << debug.candidate_source_stamp << ","
                             << static_cast<unsigned long long>(debug.reference_update_seq) << ","
                             << (debug.curve_refit_active ? 1 : 0) << ","
                             << debug.curve_refit_rms << ","
                             << debug.reconstruction_compute_ms << ","
                             << debug.tail_sample_action << ","
                             << debug.tail_pending_distance << ","
                             << debug.tail_pending_time << ","
                             << debug.accepted_suffix_source_age << ","
                             << debug.join_skip_span << ","
                             << debug.final_connector_ratio << ","
                             << debug.tangent_progress_min << ","
                             << debug.fallback_suffix_age << ","
                             << debug.outlier_gate_reason << ","
                             << debug.selected_segment_source << ","
                             << debug.projector_reason << ","
                             << debug.history_reason << ","
                             << debug.raw_reason << ","
                             << debug.reference_reason << ","
                             << debug.reference_publish_mode << "\n";
      history_status_stream_.flush();
    }
    if (lead_filter_stream_) {
      lead_filter_stream_ << std::fixed << std::setprecision(6)
                          << now.toSec() << ","
                          << (lock_active ? 1 : 0) << ","
                          << (lead_moving ? 1 : 0) << ","
                          << lead_x << ","
                          << lead_y << ","
                          << debug.ff_speed << ","
                          << debug.mean_ego_speed << ","
                          << static_cast<unsigned long long>(obs_ver) << ","
                          << debug.accum_s << ","
                          << debug.accum_t << ","
                          << debug.ego_pose_source << ","
                          << (debug.ego_pose_valid ? 1 : 0) << ","
                          << debug.ego_pose_age << ","
                          << debug.ego_yaw_uncertainty << "\n";
      lead_filter_stream_.flush();
    }
    if (observation_filter_stream_) {
      observation_filter_stream_ << std::fixed << std::setprecision(6)
                                 << now.toSec() << ","
                                 << static_cast<unsigned long long>(obs_ver) << ","
                                 << debug.input_pose_count << ","
                                 << debug.valid_candidate_count << ","
                                 << debug.lock_reject_count << ","
                                 << debug.projector_reason << ","
                                 << (debug.ego_pose_valid ? 1 : 0) << ","
                                 << debug.ego_pose_source << ","
                                 << lead_x << ","
                                 << lead_y << ","
                                 << (debug.fixed_frame_append_valid ? 1 : 0) << ","
                                 << debug.fixed_frame_progress << "\n";
      observation_filter_stream_.flush();
    }
    writeLocalPathFrame(history_relative_stream_, now, raw_pts, "raw_relative");
    writeLocalPathFrame(reconstructed_relative_stream_, now, ref_pts, "reference_relative");
  }

  bool updateTfPose(const ros::Time& stamp) {
    if (!enable_tf_pose_fallback_ || !stamp.isValid()) {
      return false;
    }
    geometry_msgs::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(tf_world_frame_, tf_base_frame_, stamp, ros::Duration(0.0));
    } catch (const std::exception&) {
      if (!tf_allow_latest_fallback_) {
        return false;
      }
      try {
        tf_msg = tf_buffer_.lookupTransform(tf_world_frame_, tf_base_frame_, ros::Time(0), ros::Duration(0.0));
      } catch (const std::exception&) {
        return false;
      }
    }
    const ros::Time tf_stamp = tf_msg.header.stamp.isValid() ? tf_msg.header.stamp : stamp;
    if ((stamp - tf_stamp).toSec() > tf_pose_timeout_) {
      return false;
    }
    const double x = tf_msg.transform.translation.x;
    const double y = tf_msg.transform.translation.y;
    const double yaw = yawFromQuaternion(tf_msg.transform.rotation);
    ego_motion_.updateExternalPose(tf_stamp, x, y, yaw, ego_motion_.fusedSpeed(), true, "tf", 2);
    tf_pose_received_ = true;
    return true;
  }

  EgoMotionEstimator::Config loadEgoMotionConfig() const {
    EgoMotionEstimator::Config cfg;
    cfg.wheelbase = pnh_.param("wheelbase", 0.60);
    cfg.min_motion_speed_for_yaw = pnh_.param("min_motion_speed_for_yaw", 0.15);
    cfg.max_abs_yaw_rate = pnh_.param("max_abs_yaw_rate", 0.80);
    cfg.max_integration_dt = pnh_.param("max_integration_dt", 0.02);
    cfg.pose_buffer_duration = pnh_.param("pose_buffer_duration", 5.0);
    cfg.steer_lpf_alpha = clampValue(pnh_.param("steer_lpf_alpha", 0.25), 0.0, 1.0);
    cfg.yaw_sigma_base = pnh_.param("yaw_sigma_base", 0.01);
    cfg.yaw_sigma_speed_gain = pnh_.param("yaw_sigma_speed_gain", 0.02);
    cfg.yaw_sigma_steer_gain = pnh_.param("yaw_sigma_steer_gain", 0.08);
    cfg.use_speed_hint = pnh_.param("use_speed_hint", false);
    cfg.speed_hint_weight = clampValue(pnh_.param("speed_hint_weight", 0.20), 0.0, 1.0);
    cfg.speed_hint_timeout = pnh_.param("speed_hint_timeout", 0.0);
    cfg.speed_hint_agreement_gate = pnh_.param("speed_hint_agreement_gate", 0.40);
    cfg.use_external_pose = pnh_.param("use_odom_pose", true);
    cfg.external_pose_position_alpha =
        clampValue(pnh_.param("odom_pose_position_alpha", 0.85), 0.0, 1.0);
    cfg.external_pose_yaw_alpha =
        clampValue(pnh_.param("odom_pose_yaw_alpha", 0.85), 0.0, 1.0);
    cfg.external_pose_source_timeout = pnh_.param("external_pose_source_timeout", 0.25);
    cfg.external_pose_reset_distance = pnh_.param("odom_pose_reset_distance", 0.60);
    cfg.external_pose_reset_yaw = pnh_.param("odom_pose_reset_yaw", 0.70);
    cfg.use_imu_yaw_rate = pnh_.param("use_imu_yaw_rate", true);
    cfg.imu_yaw_rate_lpf_alpha =
        clampValue(pnh_.param("imu_yaw_rate_lpf_alpha", 0.35), 0.0, 1.0);
    cfg.imu_yaw_rate_timeout = pnh_.param("imu_yaw_rate_timeout", 0.15);
    cfg.imu_yaw_rate_deadband = pnh_.param("imu_yaw_rate_deadband", 0.003);
    cfg.imu_yaw_rate_spike_gate = pnh_.param("imu_yaw_rate_spike_gate", 0.80);
    cfg.imu_yaw_sigma_base = pnh_.param("imu_yaw_sigma_base", 0.006);
    cfg.imu_yaw_sigma_rate_gain = pnh_.param("imu_yaw_sigma_rate_gain", 0.015);
    cfg.standstill_gyro_gate = pnh_.param("standstill_gyro_gate", 0.004);
    return cfg;
  }

  LeadObservationProjector::Config loadProjectorConfig() const {
    LeadObservationProjector::Config cfg;
    cfg.target_min_forward_x = pnh_.param("target_min_forward_x", 0.30);
    cfg.target_lateral_gate = pnh_.param("target_lateral_gate", 4.0);
    cfg.target_lock_dist_gate = pnh_.param("target_lock_dist_gate", 1.20);
    cfg.target_lock_yaw_gate = pnh_.param("target_lock_yaw_gate", 1.10);
    return cfg;
  }

  LeadTrailTracker::Config loadTrackerConfig() const {
    LeadTrailTracker::Config cfg;
    cfg.lost_timeout = pnh_.param("lost_timeout", 0.60);
    cfg.heading_blend_alpha = clampValue(pnh_.param("heading_blend_alpha", 0.20), 0.0, 1.0);
    cfg.heading_from_velocity_speed_gate = pnh_.param("heading_from_velocity_speed_gate", 0.25);
    cfg.innovation_base_gate = pnh_.param("innovation_base_gate", 0.10);
    cfg.innovation_range_gain = pnh_.param("innovation_range_gain", 0.015);
    cfg.innovation_turn_gain = pnh_.param("innovation_turn_gain", 0.80);
    cfg.outlier_reject_scale = pnh_.param("outlier_reject_scale", 3.0);
    cfg.lead_static_speed_gate = pnh_.param("lead_static_speed_gate", 0.08);
    cfg.lead_moving_speed_gate = pnh_.param("lead_moving_speed_gate", 0.12);
    cfg.static_residual_gate = pnh_.param("static_residual_gate", 0.12);
    cfg.moving_residual_gate = pnh_.param("moving_residual_gate", 0.30);
    cfg.static_vote_on = pnh_.param("static_vote_on", 12);
    cfg.static_vote_release = pnh_.param("static_vote_release", 3);
    cfg.moving_vote_on = pnh_.param("moving_vote_on", 2);
    cfg.anchor_alpha = clampValue(pnh_.param("anchor_alpha", 0.08), 0.0, 1.0);
    cfg.static_preserve_trail_span = pnh_.param("static_preserve_trail_span", 0.80);
    cfg.history_max_age = pnh_.param("history_max_age", 20.0);
    cfg.history_max_points = pnh_.param("history_max_points", 300);
    cfg.smoother_position_gain = clampValue(pnh_.param("smoother_position_gain", 0.35), 0.0, 1.0);
    cfg.smoother_cross_gain = clampValue(pnh_.param("smoother_cross_gain", 0.18), 0.0, 1.0);
    cfg.smoother_temporal_alpha = clampValue(pnh_.param("smoother_temporal_alpha", 0.22), 0.0, 1.0);
    cfg.smoother_heading_alpha = clampValue(pnh_.param("smoother_heading_alpha", 0.20), 0.0, 1.0);
    cfg.smoother_curvature_alpha =
        clampValue(pnh_.param("smoother_curvature_alpha", 0.18), 0.0, 1.0);
    cfg.measurement_heading_alpha =
        clampValue(pnh_.param("measurement_heading_alpha", 0.12), 0.0, 1.0);
    cfg.dynamics_position_alpha =
        clampValue(pnh_.param("dynamics_position_alpha", 0.12), 0.0, 1.0);
    cfg.dynamics_heading_alpha =
        clampValue(pnh_.param("dynamics_heading_alpha", 0.16), 0.0, 1.0);
    cfg.acceleration_regularization_alpha =
        clampValue(pnh_.param("acceleration_regularization_alpha", 0.16), 0.0, 1.0);
    cfg.curvature_prediction_alpha =
        clampValue(pnh_.param("curvature_prediction_alpha", 0.18), 0.0, 1.0);
    cfg.dkappa_regularization_alpha =
        clampValue(pnh_.param("dkappa_regularization_alpha", 0.20), 0.0, 1.0);
    cfg.robust_cauchy_scale = pnh_.param("robust_cauchy_scale", 2.5);
    cfg.smoother_iterations = pnh_.param("smoother_iterations", 5);
    cfg.fixed_lag_window = pnh_.param("fixed_lag_window", 3.0);
    cfg.use_bbox_heading_hint = pnh_.param("use_bbox_heading_hint", false);
    cfg.max_ego_yaw_sigma_for_confirm = pnh_.param("max_ego_yaw_sigma_for_confirm", 0.11);
    cfg.fixed_frame_heading_min_distance =
        pnh_.param("fixed_frame_heading_min_distance", 0.12);
    cfg.fixed_frame_heading_hold_distance =
        pnh_.param("fixed_frame_heading_hold_distance", 0.08);
    cfg.fixed_frame_max_yaw_jump = pnh_.param("fixed_frame_max_yaw_jump", 0.70);
    cfg.fixed_frame_kink_gate = pnh_.param("fixed_frame_kink_gate", 0.85);
    cfg.fixed_frame_curvature_gate = pnh_.param("fixed_frame_curvature_gate", 1.50);
    cfg.fixed_frame_reverse_gate = pnh_.param("fixed_frame_reverse_gate", -0.03);
    cfg.fixed_frame_duplicate_distance = pnh_.param("fixed_frame_duplicate_distance", 0.045);
    cfg.motion_tail_sample_min_distance =
        std::max(0.005, pnh_.param("motion_tail_sample_min_distance", 0.015));
    cfg.motion_tail_sample_max_interval =
        std::max(0.02, pnh_.param("motion_tail_sample_max_interval", 0.15));
    cfg.motion_tail_sample_min_along_progress =
        std::max(0.0, pnh_.param("motion_tail_sample_min_along_progress", 0.006));
    cfg.imu_wheel_append_yaw_rate_gate = pnh_.param("imu_wheel_append_yaw_rate_gate", 0.06);
    cfg.imu_wheel_append_speed_gate = pnh_.param("imu_wheel_append_speed_gate", 0.12);
    cfg.fixed_lag_observation_weight = pnh_.param("fixed_lag_observation_weight", 8.0);
    cfg.fixed_lag_smooth_weight = pnh_.param("fixed_lag_smooth_weight", 2.5);
    cfg.fixed_lag_curvature_weight = pnh_.param("fixed_lag_curvature_weight", 0.8);
    cfg.measurement_along_sigma = pnh_.param("measurement_along_sigma", 0.08);
    cfg.measurement_cross_sigma = pnh_.param("measurement_cross_sigma", 0.12);
    cfg.measurement_cross_range_gain = pnh_.param("measurement_cross_range_gain", 0.020);
    cfg.measurement_cross_yaw_uncertainty_gain =
        pnh_.param("measurement_cross_yaw_uncertainty_gain", 1.80);
    cfg.temporal_min_progress = pnh_.param("temporal_min_progress", 0.02);
    cfg.trail_append_min_distance = pnh_.param("trail_append_min_distance", 0.12);
    cfg.trail_anchor_update_alpha =
        clampValue(pnh_.param("trail_anchor_update_alpha", 0.10), 0.0, 1.0);
    cfg.tail_confirm_min_points = pnh_.param("tail_confirm_min_points", 3);
    cfg.tail_confirm_min_distance = pnh_.param("tail_confirm_min_distance", 0.035);
    cfg.tail_confirm_min_age = pnh_.param("tail_confirm_min_age", 0.08);
    cfg.confirm_dynamic_score_margin = pnh_.param("confirm_dynamic_score_margin", 0.12);
    cfg.confirm_static_residual_gate = pnh_.param("confirm_static_residual_gate", 0.14);
    cfg.confirm_min_progress = pnh_.param("confirm_min_progress", 0.08);
    cfg.confirm_accum_distance = pnh_.param("confirm_accum_distance", 0.04);
    cfg.confirm_accum_timeout = pnh_.param("confirm_accum_timeout", 0.80);
    cfg.ego_static_speed_gate = pnh_.param("ego_static_speed_gate", 0.05);
    cfg.static_release_score_margin = pnh_.param("static_release_score_margin", 0.06);
    cfg.raw_tail_keep_points = pnh_.param("raw_tail_keep_points", 6);
    cfg.dual_hypothesis_score_margin = pnh_.param("dual_hypothesis_score_margin", 0.02);
    cfg.tail_buffer_max_points = pnh_.param("tail_buffer_max_points", 12);
    cfg.tail_buffer_max_age = pnh_.param("tail_buffer_max_age", 0.60);
    cfg.static_tail_keep_points = pnh_.param("static_tail_keep_points", 3);
    cfg.publish_tail_suffix = pnh_.param("publish_tail_suffix", false);
    cfg.tail_confirm_min_time = pnh_.param("tail_confirm_min_time", 0.08);
    cfg.lead_tail_max_speed = pnh_.param("lead_tail_max_speed", 1.20);
    cfg.tail_cross_gate = pnh_.param("tail_cross_gate", 0.30);
    cfg.tail_reverse_gate = pnh_.param("tail_reverse_gate", -0.03);
    cfg.static_anchor_res_gate = pnh_.param("static_anchor_res_gate", 0.04);
    cfg.static_anchor_progress_gate = pnh_.param("static_anchor_progress_gate", 0.04);
    cfg.static_anchor_speed_gate = pnh_.param("static_anchor_speed_gate", 0.035);
    cfg.static_anchor_append_distance_gate =
        pnh_.param("static_anchor_append_distance_gate", 0.08);
    cfg.heading_recompute_min_distance = pnh_.param("heading_recompute_min_distance", 0.06);
    cfg.provisional_tail_enabled = pnh_.param("provisional_tail_enabled", true);
    cfg.provisional_tail_min_points = pnh_.param("provisional_tail_min_points", 3);
    cfg.provisional_tail_min_span = pnh_.param("provisional_tail_min_span", 0.18);
    cfg.provisional_tail_short_min_span =
        pnh_.param("provisional_tail_short_min_span", 0.08);
    cfg.provisional_tail_min_time = pnh_.param("provisional_tail_min_time", 0.08);
    cfg.provisional_tail_short_min_time =
        pnh_.param("provisional_tail_short_min_time", 0.10);
    cfg.provisional_tail_max_age = pnh_.param("provisional_tail_max_age", 1.50);
    cfg.provisional_tail_standalone_until_confirmed_points =
        pnh_.param("provisional_tail_standalone_until_confirmed_points", 8);
    cfg.motion_tail_force_publish_stall =
        pnh_.param("motion_tail_force_publish_stall", 0.15);
    cfg.motion_static_suppress_speed = pnh_.param("motion_static_suppress_speed", 0.035);
    cfg.motion_static_suppress_progress = pnh_.param("motion_static_suppress_progress", 0.015);
    cfg.motion_static_suppress_residual = pnh_.param("motion_static_suppress_residual", 0.050);
    cfg.suffix_estimator_min_points = pnh_.param("suffix_estimator_min_points", 3);
    cfg.suffix_estimator_min_span = pnh_.param("suffix_estimator_min_span", 0.10);
    cfg.suffix_estimator_min_time = pnh_.param("suffix_estimator_min_time", 0.08);
    cfg.suffix_estimator_bootstrap_min_span =
        std::max(0.025, pnh_.param("suffix_estimator_bootstrap_min_span", 0.12));
    cfg.suffix_estimator_confirmed_min_span =
        std::max(0.010, pnh_.param("suffix_estimator_confirmed_min_span", 0.04));
    cfg.suffix_estimator_tail_min_time =
        std::max(0.05, pnh_.param("suffix_estimator_tail_min_time", 0.20));
    cfg.suffix_estimator_max_extension_span =
        std::max(0.08, pnh_.param("suffix_estimator_max_extension_span", 0.35));
    cfg.suffix_smooth_resample_ds =
        std::max(0.04, pnh_.param("suffix_smooth_resample_ds", 0.08));
    cfg.suffix_smooth_passes =
        std::max(0, pnh_.param("suffix_smooth_passes", 2));
    cfg.suffix_smooth_alpha =
        clampValue(pnh_.param("suffix_smooth_alpha", 0.30), 0.0, 0.85);
    cfg.suffix_smooth_huber_delta =
        std::max(0.01, pnh_.param("suffix_smooth_huber_delta", 0.12));
    cfg.suffix_smooth_max_adjust =
        std::max(0.0, pnh_.param("suffix_smooth_max_adjust", 0.10));
    cfg.suffix_smooth_wiggle_gate =
        std::max(0.0, pnh_.param("suffix_smooth_wiggle_gate", 0.040));
    cfg.suffix_smooth_sign_flip_gate =
        std::max(1, pnh_.param("suffix_smooth_sign_flip_gate", 2));
    cfg.suffix_smooth_alpha_wiggle =
        clampValue(pnh_.param("suffix_smooth_alpha_wiggle", 0.55), 0.0, 0.85);
    cfg.suffix_smooth_max_adjust_wiggle =
        std::max(0.0, pnh_.param("suffix_smooth_max_adjust_wiggle", 0.14));
    cfg.suffix_smooth_curve_preserve_kappa =
        std::max(0.0, pnh_.param("suffix_smooth_curve_preserve_kappa", 1.35));
    cfg.suffix_smooth_extra_s_wave_passes =
        std::max(0, pnh_.param("suffix_smooth_extra_s_wave_passes", 0));
    cfg.suffix_smooth_five_point_passes =
        std::max(0, pnh_.param("suffix_smooth_five_point_passes", 0));
    cfg.suffix_smooth_s_wave_window =
        std::max(0.20, pnh_.param("suffix_smooth_s_wave_window", 0.60));
    cfg.suffix_smooth_centerline_alpha =
        clampValue(pnh_.param("suffix_smooth_centerline_alpha", 0.25), 0.0, 1.0);
    cfg.suffix_trackable_kappa_limit =
        std::max(0.10, pnh_.param("suffix_trackable_kappa_limit", 0.77));
    cfg.suffix_curve_rate_refit_gate =
        std::max(1.0, pnh_.param("suffix_curve_rate_refit_gate", 15.0));
    cfg.suffix_curve_refit_window =
        std::max(0.20, pnh_.param("suffix_curve_refit_window", 0.50));
    cfg.suffix_curve_refit_alpha =
        clampValue(pnh_.param("suffix_curve_refit_alpha", 0.30), 0.0, 0.80);
    cfg.reference_health_wiggle_gate =
        std::max(0.01, pnh_.param("reference_health_wiggle_gate", 0.060));
    cfg.reference_health_curvature_gate =
        std::max(0.1, pnh_.param("reference_health_curvature_gate", 3.0));
    cfg.reference_health_wiggle_penalty =
        clampValue(pnh_.param("reference_health_wiggle_penalty", 0.35), 0.0, 1.0);
    cfg.reference_health_flip_penalty =
        clampValue(pnh_.param("reference_health_flip_penalty", 0.16), 0.0, 1.0);
    cfg.reference_health_curvature_penalty =
        clampValue(pnh_.param("reference_health_curvature_penalty", 0.12), 0.0, 1.0);
    cfg.reference_health_s_wave_cap =
        clampValue(pnh_.param("reference_health_s_wave_cap", 0.62), 0.0, 1.0);
    cfg.tail_soft_geometry_enabled =
        pnh_.param("tail_soft_geometry_enabled", true);
    cfg.ego_turn_weight_enabled = pnh_.param("ego_turn_weight_enabled", true);
    cfg.ego_turn_yaw_rate_gate =
        std::max(0.0, pnh_.param("ego_turn_yaw_rate_gate", 0.035));
    cfg.ego_turn_yaw_rate_full =
        std::max(cfg.ego_turn_yaw_rate_gate + 1e-3,
                 pnh_.param("ego_turn_yaw_rate_full", 0.12));
    cfg.ego_turn_steer_gate =
        std::max(0.0, pnh_.param("ego_turn_steer_gate", 0.08));
    cfg.ego_turn_steer_full =
        std::max(cfg.ego_turn_steer_gate + 1e-3,
                 pnh_.param("ego_turn_steer_full", 0.28));
    cfg.ego_turn_progress_gate =
        std::max(1e-3, pnh_.param("ego_turn_progress_gate", 0.08));
    cfg.ego_turn_lateral_velocity_gate =
        std::max(0.0, pnh_.param("ego_turn_lateral_velocity_gate", 0.08));
    cfg.ego_turn_lateral_velocity_full =
        std::max(cfg.ego_turn_lateral_velocity_gate + 1e-3,
                 pnh_.param("ego_turn_lateral_velocity_full", 0.35));
    cfg.ego_turn_min_tail_weight =
        clampValue(pnh_.param("ego_turn_min_tail_weight", 0.30), 0.05, 1.0);
    cfg.ego_turn_min_along_weight =
        clampValue(pnh_.param("ego_turn_min_along_weight", 0.55), 0.05, 1.0);
    cfg.ego_turn_min_cross_weight =
        clampValue(pnh_.param("ego_turn_min_cross_weight", 0.08), 0.02, 1.0);
    cfg.ego_turn_cross_hold_score =
        clampValue(pnh_.param("ego_turn_cross_hold_score", 0.55), 0.0, 1.0);
    cfg.ego_turn_cross_hold_progress =
        std::max(0.0, pnh_.param("ego_turn_cross_hold_progress", 0.05));
    cfg.lateral_innovation_gate_m =
        std::max(0.0, pnh_.param("lateral_innovation_gate_m", 0.08));
    cfg.lateral_innovation_full_m =
        std::max(cfg.lateral_innovation_gate_m + 1e-3,
                 pnh_.param("lateral_innovation_full_m", 0.28));
    cfg.ego_turn_cross_sigma_scale =
        std::max(0.0, pnh_.param("ego_turn_cross_sigma_scale", 2.50));
    cfg.ego_turn_heading_alpha_scale =
        clampValue(pnh_.param("ego_turn_heading_alpha_scale", 0.15), 0.0, 1.0);
    cfg.ego_turn_bbox_heading_disable_score =
        clampValue(pnh_.param("ego_turn_bbox_heading_disable_score", 0.25), 0.0, 1.0);
    cfg.ego_turn_cross_hold_residual_scale =
        clampValue(pnh_.param("ego_turn_cross_hold_residual_scale", 0.10), 0.0, 1.0);
    cfg.lateral_velocity_filter_tau =
        std::max(0.02, pnh_.param("lateral_velocity_filter_tau", 0.35));
    cfg.lateral_velocity_filtered_cap =
        std::max(0.20, pnh_.param("lateral_velocity_filtered_cap", 2.0));
    cfg.lateral_low_weight_velocity_gate =
        std::max(0.0, pnh_.param("lateral_low_weight_velocity_gate", 0.25));
    cfg.lateral_outlier_velocity_gate =
        std::max(cfg.lateral_low_weight_velocity_gate + 1e-3,
                 pnh_.param("lateral_outlier_velocity_gate", 0.55));
    cfg.lateral_low_weight_min =
        clampValue(pnh_.param("lateral_low_weight_min", 0.35), 0.05, 1.0);
    return cfg;
  }

  PathGenerator::Config loadPathConfig() const {
    PathGenerator::Config cfg;
    cfg.raw_resample_ds = pnh_.param("raw_resample_ds", 0.08);
    cfg.raw_fit_ds = pnh_.param("raw_fit_ds", 0.06);
    cfg.raw_spline_passes = pnh_.param("raw_spline_passes", 4);
    cfg.raw_spline_alpha = clampValue(pnh_.param("raw_spline_alpha", 0.45), 0.0, 1.0);
    cfg.raw_tail_weight_scale = clampValue(pnh_.param("raw_tail_weight_scale", 0.25), 0.05, 1.0);
    cfg.raw_preserve_history_geometry = pnh_.param("raw_preserve_history_geometry", true);
    cfg.raw_duplicate_min_distance = pnh_.param("raw_duplicate_min_distance", 0.04);
    cfg.raw_heading_min_distance = pnh_.param("raw_heading_min_distance", 0.12);
    cfg.raw_heading_hold_distance = pnh_.param("raw_heading_hold_distance", 0.08);
    cfg.raw_max_heading_step = pnh_.param("raw_max_heading_step", 0.70);
    cfg.raw_max_yaw_jump = pnh_.param("raw_max_yaw_jump", 1.00);
    cfg.raw_max_segment_kappa = pnh_.param("raw_max_segment_kappa", 5.00);
    cfg.raw_backtrack_epsilon = pnh_.param("raw_backtrack_epsilon", 0.02);
    cfg.raw_stable_segment_min_span = pnh_.param("raw_stable_segment_min_span", 0.60);
    cfg.raw_stable_segment_min_points = pnh_.param("raw_stable_segment_min_points", 5);
    cfg.lead_center_to_path_ref_offset = pnh_.param("lead_center_to_path_ref_offset", 0.0);
    cfg.raw_min_points_for_publish = pnh_.param("raw_min_points_for_publish", 2);
    cfg.raw_min_span_for_publish = pnh_.param("raw_min_span_for_publish", 0.08);
    cfg.raw_keep_behind_x = pnh_.param("raw_keep_behind_x", 0.50);
    cfg.raw_max_range = pnh_.param("raw_max_range", 30.0);
    cfg.raw_max_points = pnh_.param("raw_max_points", 250);
    cfg.reference_join_min_forward_x = pnh_.param("reference_join_min_forward_x", 0.12);
    cfg.reference_join_max_lateral = pnh_.param("reference_join_max_lateral", 3.0);
    cfg.reference_join_lookahead = pnh_.param("reference_join_lookahead", 0.35);
    cfg.reference_join_skip_weight = pnh_.param("reference_join_skip_weight", 0.20);
    cfg.reference_join_curvature_weight = pnh_.param("reference_join_curvature_weight", 0.90);
    cfg.reference_join_time_weight = pnh_.param("reference_join_time_weight", 0.60);
    cfg.reference_join_progress_weight = pnh_.param("reference_join_progress_weight", 0.25);
    cfg.reference_join_s_window = pnh_.param("reference_join_s_window", 3.0);
    cfg.reference_join_max_skip_span =
        std::max(0.05, pnh_.param("reference_join_max_skip_span", 0.20));
    cfg.reference_join_min_suffix_span_short =
        pnh_.param("reference_join_min_suffix_span_short", 0.25);
    cfg.reference_join_min_suffix_span =
        pnh_.param("reference_join_min_suffix_span", 0.50);
    cfg.reference_join_min_suffix_span_mature =
        pnh_.param("reference_join_min_suffix_span_mature", 0.80);
    cfg.reference_join_suffix_weight =
        pnh_.param("reference_join_suffix_weight", 2.00);
    cfg.reference_join_max_connector_ratio =
        clampValue(pnh_.param("reference_join_max_connector_ratio", 0.55), 0.10, 0.95);
    cfg.reference_join_max_connector_ratio_bootstrap =
        clampValue(pnh_.param("reference_join_max_connector_ratio_bootstrap", 0.65),
                   cfg.reference_join_max_connector_ratio,
                   0.95);
    cfg.reference_inner_cut_connector_ratio =
        clampValue(pnh_.param("reference_inner_cut_connector_ratio", 0.65), 0.20, 0.90);
    cfg.reference_inner_cut_lateral_gate =
        std::max(0.02, pnh_.param("reference_inner_cut_lateral_gate", 0.25));
    cfg.reference_inner_cut_score_gate =
        clampValue(pnh_.param("reference_inner_cut_score_gate", 0.45), 0.0, 1.0);
    cfg.reference_join_stability_weight =
        std::max(0.0, pnh_.param("reference_join_stability_weight", 0.85));
    cfg.reference_join_stability_window =
        std::max(0.05, pnh_.param("reference_join_stability_window", 0.45));
    cfg.reference_min_suffix_preserve_span =
        pnh_.param("reference_min_suffix_preserve_span", 0.35);
    cfg.reference_min_suffix_preserve_span_short =
        pnh_.param("reference_min_suffix_preserve_span_short", 0.18);
    cfg.reference_min_points = pnh_.param("reference_min_points", 2);
    cfg.reference_min_span = pnh_.param("reference_min_span", 0.25);
    cfg.reference_direct_attach_dist = pnh_.param("reference_direct_attach_dist", 0.18);
    cfg.reference_connector_enable_dist = pnh_.param("reference_connector_enable_dist", 0.15);
    cfg.reference_connector_resolution = pnh_.param("reference_connector_resolution", 0.08);
    cfg.reference_connector_tangent_scale = pnh_.param("reference_connector_tangent_scale", 0.65);
    cfg.reference_connector_curvature_scale = pnh_.param("reference_connector_curvature_scale", 1.0);
    cfg.reference_splice_blend_length = pnh_.param("reference_splice_blend_length", 0.60);
    cfg.reference_resample_ds = pnh_.param("reference_resample_ds", 0.08);
    cfg.reference_sg_passes = pnh_.param("reference_sg_passes", 1);
    cfg.suffix_smooth_wiggle_gate =
        std::max(0.0, pnh_.param("suffix_smooth_wiggle_gate", 0.040));
    cfg.suffix_smooth_sign_flip_gate =
        std::max(1, pnh_.param("suffix_smooth_sign_flip_gate", 2));
    cfg.reference_health_wiggle_gate =
        std::max(0.01, pnh_.param("reference_health_wiggle_gate", 0.060));
    cfg.reference_health_curvature_gate =
        std::max(0.10, pnh_.param("reference_health_curvature_gate", 3.0));
    cfg.reference_health_wiggle_penalty =
        clampValue(pnh_.param("reference_health_wiggle_penalty", 0.35), 0.0, 1.0);
    cfg.reference_health_flip_penalty =
        clampValue(pnh_.param("reference_health_flip_penalty", 0.16), 0.0, 1.0);
    cfg.reference_health_curvature_penalty =
        clampValue(pnh_.param("reference_health_curvature_penalty", 0.12), 0.0, 1.0);
    cfg.reference_health_s_wave_cap =
        clampValue(pnh_.param("reference_health_s_wave_cap", 0.62), 0.0, 1.0);
    cfg.reference_max_length = pnh_.param("reference_max_length", 20.0);
    cfg.reference_follow_raw_only = pnh_.param("reference_follow_raw_only", false);
    cfg.reference_validation_front_s = pnh_.param("reference_validation_front_s", 2.50);
    cfg.reference_max_front_yaw_jump = pnh_.param("reference_max_front_yaw_jump", 0.90);
    cfg.reference_max_front_kappa = pnh_.param("reference_max_front_kappa", 8.0);
    cfg.reference_backtrack_epsilon = pnh_.param("reference_backtrack_epsilon", 0.02);
    cfg.reference_short_kink_distance = pnh_.param("reference_short_kink_distance", 0.04);
    cfg.reference_short_kink_yaw = pnh_.param("reference_short_kink_yaw", 0.50);
    cfg.reference_max_join_retries = pnh_.param("reference_max_join_retries", 4);
    cfg.reference_anchor_connector_enabled =
        pnh_.param("reference_anchor_connector_enabled", false);
    cfg.reference_mature_min_history_points =
        pnh_.param("reference_mature_min_history_points", 8);
    cfg.reference_mature_min_points = pnh_.param("reference_mature_min_points", 8);
    cfg.reference_mature_min_raw_span = pnh_.param("reference_mature_min_raw_span", 0.80);
    cfg.reference_mature_min_span = pnh_.param("reference_mature_min_span", 0.80);
    cfg.reference_last_good_hold_timeout = pnh_.param("reference_last_good_hold_timeout", 0.80);
    cfg.reference_degraded_raw_fallback_enabled =
        pnh_.param("reference_degraded_raw_fallback_enabled", true);
    cfg.reference_degraded_raw_min_span = pnh_.param("reference_degraded_raw_min_span", 0.18);
    cfg.reference_degraded_raw_min_points = pnh_.param("reference_degraded_raw_min_points", 2);
    cfg.reference_degraded_raw_max_start_distance =
        pnh_.param("reference_degraded_raw_max_start_distance", 0.85);
    cfg.stable_segment_min_span = pnh_.param("stable_segment_min_span", 0.18);
    cfg.stable_segment_min_points = pnh_.param("stable_segment_min_points", 2);
    cfg.stable_segment_start_distance_weight =
        pnh_.param("stable_segment_start_distance_weight", 1.20);
    cfg.reference_output_hysteresis_enabled =
        pnh_.param("reference_output_hysteresis_enabled", true);
    cfg.reference_output_min_hold_time = pnh_.param("reference_output_min_hold_time", 0.25);
    cfg.reference_output_max_hold_time = pnh_.param("reference_output_max_hold_time", 0.50);
    cfg.reference_output_confirm_frames =
        std::max(1, pnh_.param("reference_output_confirm_frames", 3));
    cfg.reference_output_max_span_jump = pnh_.param("reference_output_max_span_jump", 0.50);
    cfg.reference_output_max_endpoint_jump =
        pnh_.param("reference_output_max_endpoint_jump", 0.35);
    cfg.reference_output_max_start_jump =
        pnh_.param("reference_output_max_start_jump", 0.25);
    cfg.reference_output_max_heading_jump =
        pnh_.param("reference_output_max_heading_jump", 0.35);
    cfg.reference_full_max_yaw_jump = pnh_.param("reference_full_max_yaw_jump", 0.80);
    cfg.reference_full_short_kink_yaw =
        pnh_.param("reference_full_short_kink_yaw", 0.45);
    cfg.reference_full_short_kink_distance =
        pnh_.param("reference_full_short_kink_distance", 0.05);
    cfg.reference_full_max_kappa = pnh_.param("reference_full_max_kappa", 5.0);
    cfg.reference_output_span_growth_per_sec =
        pnh_.param("reference_output_span_growth_per_sec", 1.8);
    cfg.reference_output_span_shrink_per_sec =
        pnh_.param("reference_output_span_shrink_per_sec", 1.2);
    cfg.reference_output_min_span_growth =
        pnh_.param("reference_output_min_span_growth", 0.12);
    cfg.reference_state_update_alpha =
        clampValue(pnh_.param("reference_state_update_alpha", 0.65), 0.0, 1.0);
    cfg.reference_output_min_publish_span =
        pnh_.param("reference_output_min_publish_span", 0.80);
    cfg.reference_output_min_publish_points =
        std::max(2, pnh_.param("reference_output_min_publish_points", 8));
    cfg.reference_stale_hold_time =
        std::max(0.05, pnh_.param("reference_stale_hold_time", 0.60));
    cfg.reference_dynamic_shrink_min_span =
        pnh_.param("reference_dynamic_shrink_min_span", 0.08);
    cfg.reference_dynamic_shrink_min_points =
        std::max(2, pnh_.param("reference_dynamic_shrink_min_points", 3));
    return cfg;
  }

  ros::Time resolveStampedPoseArrayTime(const geometry_msgs::PoseArray& msg) const {
    if (msg.header.stamp.isValid()) {
      return msg.header.stamp;
    }

    if (allow_unstamped_tracked_fallback_) {
      ROS_WARN_THROTTLE(1.0,
                        "tracked_topic PoseArray has no valid header.stamp; falling back to "
                        "receipt time. Fixed-frame reconstruction will be less accurate.");
      return ros::Time::now();
    }

    ROS_WARN_THROTTLE(1.0,
                      "tracked_topic PoseArray has no valid header.stamp; dropping frame because "
                      "fixed-frame reconstruction requires measurement-time ego pose.");
    return ros::Time();
  }

  void carCallback(const move_car::car_parameter::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // car_parameter has no header, so arrival time is the only available fallback.
    const ros::Time stamp = ros::Time::now();
    const double wheel_speed_mps = static_cast<double>(msg->back_wheel_speed) * wheel_speed_scale_;
    const double steer_raw = static_cast<double>(msg->turn_angle);
    double steer_angle_rad =
        steer_sign_ * (steer_raw * steer_angle_scale_ - steer_angle_bias_);
    if (std::fabs(steer_angle_rad) < steer_deadband_rad_) {
      steer_angle_rad = 0.0;
    }
    last_steer_raw_ = steer_raw;
    last_steer_scaled_ = steer_angle_rad;
    steer_scale_suspect_ =
        std::fabs(steer_raw) > 0.05 &&
        std::fabs(steer_raw) < 1.2 &&
        std::fabs(steer_angle_scale_) < 0.10 &&
        std::fabs(steer_angle_rad) < 0.03;
    ego_motion_.updateWheelState(stamp, wheel_speed_mps, steer_angle_rad);
  }

  void carvelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // geometry_msgs/Twist has no header, so arrival time is the only available fallback.
    const ros::Time stamp = ros::Time::now();
    const double speed_hint_mps = static_cast<double>(msg->linear.x) * speed_hint_scale_;
    ego_motion_.updateSpeedHint(stamp, speed_hint_mps);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time stamp = msg->header.stamp.isValid() ? msg->header.stamp : ros::Time::now();
    const double x = odom_position_scale_ * msg->pose.pose.position.x;
    const double y = odom_position_scale_ * msg->pose.pose.position.y;
    const double yaw = wrapAngle(odom_yaw_sign_ * yawFromQuaternion(msg->pose.pose.orientation) + odom_yaw_bias_);
    const double speed = odom_speed_scale_ *
                         hypot2(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    ego_motion_.updateExternalPose(stamp, x, y, yaw, speed, std::isfinite(speed), "ned_odom", 3);
    ned_odom_received_ = true;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time stamp = msg->header.stamp.isValid() ? msg->header.stamp : ros::Time::now();
    ego_motion_.updateImuYawRate(stamp, msg->angular_velocity.z);
  }

  void trackedCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time obs_stamp = resolveStampedPoseArrayTime(*msg);
    if (!obs_stamp.isValid()) {
      debug_state_.input_pose_count = static_cast<int>(msg->poses.size());
      debug_state_.projector_reason = "obs_stamp_invalid";
      return;
    }

    updateTfPose(obs_stamp);
    ego_motion_.integrateTo(obs_stamp);
    tracker_.markLostIfExpired(obs_stamp);
    tracker_.pruneHistory(obs_stamp);

    LeadObservation obs;
    EgoPose2D ego_at_obs;
    const LeadState2D* track_ptr = tracker_.hasActiveTrack() ? &tracker_.track() : NULL;
    if (!projector_.project(*msg, ego_motion_, track_ptr, &obs, &ego_at_obs, &debug_state_)) {
      ++obs_version_;
      debug_state_.history_size = tracker_.worldHistoryForPublish(obs_stamp).size();
      fillEgoDebug(obs_stamp, NULL);
      return;
    }
    fillEgoDebug(obs_stamp, &ego_at_obs);

    tracker_.ingestObservation(obs,
                               ego_at_obs,
                               ego_motion_.currentYawRate(),
                               ego_motion_.yawSigmaEstimate(),
                               ego_motion_.steerAngle(),
                               &debug_state_);
    tracker_.pruneHistory(obs_stamp);
    debug_state_.history_size = tracker_.worldHistoryForPublish(obs_stamp).size();
    ++obs_version_;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::WallTime compute_start = ros::WallTime::now();
    const ros::Time now = ros::Time::now();

    std::vector<LocalPathPoint> raw_pts;
    std::vector<LocalPathPoint> ref_pts;
    std::vector<HistoryPoint2D> world_history;
    std::vector<HistoryPoint2D> reference_history;
    bool lock_active = false;
    bool lead_moving = false;
    bool ego_standstill = false;
    double lead_x = 0.0;
    double lead_y = 0.0;
    double speed = 0.0;
    double yaw_rate = 0.0;
    double wheel_speed = 0.0;
    double speed_hint = 0.0;
    double steer = 0.0;
    double innovation = 0.0;
    double metric = 0.0;
    uint64_t obs_ver = 0;
    TrackerDebugState debug_snapshot;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      updateTfPose(now);
      ego_motion_.integrateTo(now);
      tracker_.markLostIfExpired(now);
      tracker_.pruneHistory(now);
      resetReferenceCycleDebug(&debug_state_);

      EgoPose2D ego_now;
      if (ego_motion_.queryPoseAt(now, &ego_now)) {
        world_history = tracker_.worldHistoryForPublish(now);
        reference_history = tracker_.referenceHistoryForPublish(now, &debug_state_);
        if (reference_history.empty()) {
          reference_history = world_history;
        }
        debug_state_.history_size = reference_history.size();
        const std::vector<RawTrailPoint> raw_trail =
            path_generator_.buildRawTrail(reference_history, &debug_state_);
        raw_pts = path_generator_.buildRawPath(reference_history, ego_now, &debug_state_);
        ref_pts = path_generator_.buildReferencePath(raw_trail, raw_pts, ego_now, &debug_state_);
        if ((debug_state_.reference_publish_mode == "ready" ||
             debug_state_.reference_publish_mode == "degraded_raw") &&
            (debug_state_.selected_segment_source == "confirmed_tail" ||
             debug_state_.selected_segment_source == "motion_tail" ||
             debug_state_.motion_tail_promoted_to_reference) &&
            !ref_pts.empty()) {
          debug_state_.reference_publish_mode = "provisional_tail";
          if (debug_state_.reference_reason != "reference_stitched_tail" &&
              debug_state_.reference_reason != "reference_stitched_provisional") {
            debug_state_.reference_reason = "reference_stitched_tail";
          }
          if (debug_state_.reference_source == "none") {
            debug_state_.reference_source = "stitched_tail";
          }
        }
        ref_pts = path_generator_.stabilizeReferenceOutput(ref_pts, ego_now, &debug_state_);
        path_generator_.finalizePublishedReferenceDebug(ref_pts, &debug_state_);
        updateReferenceTimingDebug(now, ref_pts.size(), &debug_state_);
        debug_state_.reference_flatness_valid =
            !ref_pts.empty() &&
            debug_state_.reference_full_max_yaw_jump <= 0.80 &&
            debug_state_.reference_full_backtrack_count == 0 &&
            debug_state_.reference_health_score >= 0.45;
        debug_state_.reference_tracking_confidence =
            clampValue(debug_state_.reference_health_score *
                           std::max(0.35, debug_state_.reference_estimator_confidence),
                       0.0,
                       1.0);
        debug_state_.reference_from_fixed_frame = !reference_history.empty() && !ref_pts.empty();
        fillEgoDebug(now, &ego_now);

        if (tracker_.hasActiveTrack()) {
          const LeadState2D pred = predictLeadState(tracker_.track(), now);
          transformWorldToLocal(ego_now, pred.x, pred.y, &lead_x, &lead_y);
        }
      } else {
        fillEgoDebug(now, NULL);
        debug_state_.reference_reason = "ego_pose_unavailable";
        updateReferenceTimingDebug(now, ref_pts.size(), &debug_state_);
      }

      lock_active = tracker_.hasActiveTrack();
      lead_moving = tracker_.movingFlag();
      speed = ego_motion_.fusedSpeed();
      yaw_rate = ego_motion_.currentYawRate();
      wheel_speed = ego_motion_.wheelSpeed();
      speed_hint = ego_motion_.speedHint();
      steer = ego_motion_.steerAngle();
      innovation = tracker_.innovationNorm();
      metric = tracker_.innovationMetric();
      obs_ver = obs_version_;
      ego_standstill = std::fabs(speed) < min_motion_speed_for_yaw_ * 0.9;
      debug_state_.reconstruction_compute_ms =
          (ros::WallTime::now() - compute_start).toSec() * 1000.0;
      debug_snapshot = debug_state_;
    }

    raw_pub_.publish(makePathMsg(raw_pts, base_frame_, now));
    reference_pub_.publish(makePathMsg(ref_pts, base_frame_, now));
    publishReferenceStatus(now, debug_snapshot, ref_pts.size());

    appendFixedFrameTrajectoryToFile(world_history);
    writeRuntimeDebug(now,
                      debug_snapshot,
                      lock_active,
                      lead_moving,
                      ego_standstill,
                      raw_pts.size(),
                      ref_pts.size(),
                      world_history.size(),
                      lead_x,
                      lead_y,
                      speed,
                      yaw_rate,
                      wheel_speed,
                      speed_hint,
                      steer,
                      obs_ver);
    writeLegacyDebugLogs(now,
                         debug_snapshot,
                         lock_active,
                         lead_moving,
                         ego_standstill,
                         raw_pts,
                         ref_pts,
                         world_history.size(),
                         lead_x,
                         lead_y,
                         obs_ver);

    ROS_INFO_THROTTLE(
        1.0,
        "traj_ff | lock=%d moving=%d standstill=%d raw=%zu ref=%zu x=%.2f y=%.2f v=%.3f yaw=%.3f wheel=%.3f hint=%.3f steer=%.3f innov=%.3f metric=%.3f ff_speed=%.3f ego_v=%.3f static_hyp_res=%.3f hs=%.3f hd=%.3f acc_s=%.3f acc_t=%.3f conf=%zu tail=%zu obs_ver=%llu poses=%d cand=%d lockrej=%d hist=%zu raw_span=%.2f ref_span=%.2f proj=%s histr=%s rawr=%s refr=%s",
        lock_active ? 1 : 0,
        lead_moving ? 1 : 0,
        ego_standstill ? 1 : 0,
        raw_pts.size(),
        ref_pts.size(),
        lead_x,
        lead_y,
        speed,
        yaw_rate,
        wheel_speed,
        speed_hint,
        steer,
        innovation,
        metric,
        debug_snapshot.ff_speed,
        debug_snapshot.mean_ego_speed,
        debug_snapshot.static_hyp_res,
        debug_snapshot.hs_score,
        debug_snapshot.hd_score,
        debug_snapshot.accum_s,
        debug_snapshot.accum_t,
        debug_snapshot.confirmed_count,
        debug_snapshot.tail_count,
        static_cast<unsigned long long>(obs_ver),
        debug_snapshot.input_pose_count,
        debug_snapshot.valid_candidate_count,
        debug_snapshot.lock_reject_count,
        debug_snapshot.history_size,
        debug_snapshot.raw_span,
        debug_snapshot.reference_span,
        debug_snapshot.projector_reason.c_str(),
        debug_snapshot.history_reason.c_str(),
        debug_snapshot.raw_reason.c_str(),
        debug_snapshot.reference_reason.c_str());
  }

  void appendFixedFrameTrajectoryToFile(const std::vector<HistoryPoint2D>& world_history) {
    if (!final_traj_log_stream_.is_open() || world_history.empty()) {
      return;
    }

    for (size_t i = 0; i < world_history.size(); ++i) {
      if (world_history[i].stamp.isValid() && last_logged_world_stamp_.isValid() &&
          world_history[i].stamp <= last_logged_world_stamp_) {
        continue;
      }
      final_traj_log_stream_ << world_history[i].stamp.toSec() << ","
                             << world_history[i].x << ","
                             << world_history[i].y << ","
                             << world_history[i].heading << "\n";
      if (world_history[i].stamp.isValid()) {
        last_logged_world_stamp_ = world_history[i].stamp;
      }
    }
    final_traj_log_stream_.flush();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber tracked_sub_;
  ros::Subscriber car_sub_;
  ros::Subscriber carvel_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher raw_pub_;
  ros::Publisher reference_pub_;
  ros::Publisher reference_status_pub_;
  ros::Timer timer_;
  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string base_frame_;
  std::string odom_frame_;
  std::string tracked_topic_;
  std::string car_topic_;
  std::string carvel_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string raw_topic_;
  std::string reference_topic_;
  std::string reference_status_topic_;
  std::string final_traj_log_file_;
  std::string debug_log_dir_ = "/home/ubuntu/drive_car_copy/data";
  std::string runtime_debug_file_ = "runtime_debug.csv";
  std::string history_relative_file_ = "history_relative_trajectory.txt";
  std::string history_status_file_ = "history_status.csv";
  std::string lead_filter_state_file_ = "lead_filter_state.csv";
  std::string observation_filter_file_ = "observation_filter.csv";
  std::string reconstructed_relative_file_ = "reconstructed_relative_trajectory.txt";

  double publish_rate_hz_ = 30.0;
  double wheel_speed_scale_ = 0.001;
  double speed_hint_scale_ = 1.0;
  double steer_angle_scale_ = 1.0;
  double steer_angle_bias_ = 0.0;
  double steer_sign_ = 1.0;
  double steer_deadband_rad_ = 0.002;
  double last_steer_raw_ = 0.0;
  double last_steer_scaled_ = 0.0;
  bool steer_scale_suspect_ = false;
  double lost_timeout_ = 0.60;
  double min_motion_speed_for_yaw_ = 0.15;
  bool allow_unstamped_tracked_fallback_ = true;
  bool use_odom_pose_ = true;
  bool enable_tf_pose_fallback_ = true;
  bool tf_allow_latest_fallback_ = true;
  double tf_pose_timeout_ = 0.20;
  std::string tf_world_frame_;
  std::string tf_base_frame_;
  double odom_position_scale_ = 1.0;
  double odom_yaw_sign_ = 1.0;
  double odom_yaw_bias_ = 0.0;
  double odom_speed_scale_ = 1.0;
  bool enable_debug_csv_logs_ = true;
  std::ofstream final_traj_log_stream_;
  std::ofstream runtime_debug_stream_;
  std::ofstream history_relative_stream_;
  std::ofstream history_status_stream_;
  std::ofstream lead_filter_stream_;
  std::ofstream observation_filter_stream_;
  std::ofstream reconstructed_relative_stream_;
  ros::Time last_logged_world_stamp_;
  bool ned_odom_received_ = false;
  bool tf_pose_received_ = false;
  EgoPose2D last_debug_ego_pose_;
  bool last_debug_ego_pose_valid_ = false;
  std::string last_reference_publish_mode_;
  ros::Time reference_mode_since_;
  ros::Time reference_empty_since_;

  EgoMotionEstimator ego_motion_;
  LeadObservationProjector projector_;
  LeadTrailTracker tracker_;
  PathGenerator path_generator_;
  TrackerDebugState debug_state_;
  uint64_t obs_version_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_reconstructor");
  FixedFrameTrajectoryReconstructor node;
  ros::spin();
  return 0;
}
