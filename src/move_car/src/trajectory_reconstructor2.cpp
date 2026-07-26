/*
改进版：响应更快
*/
#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <move_car/car_parameter.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

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

double alignAngleToReference(double angle, double reference) {
  double aligned = wrapAngle(angle);
  const double direct_error = std::fabs(wrapAngle(aligned - reference));
  const double flipped_error = std::fabs(wrapAngle(aligned + M_PI - reference));
  if (flipped_error < direct_error) {
    aligned = wrapAngle(aligned + M_PI);
  }
  return aligned;
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
  bool speed_source_valid = false;
};

using EgoPose2D = EgoState;

// "World" in this node is a short-term anchor frame reconstructed from ego
// motion, not a GPS/global map frame.
struct HistoryPoint2D {
  ros::Time stamp;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double curvature = 0.0;
  bool predicted = false;
  int motion_class = 0;
  bool dynamic_residual_win = false;
  bool ego_translation_observable = false;
  bool relative_shortening_only = false;
  bool endpoint_quality_ok = true;
  int segment_id = 0;
  int anchor_epoch_id = 0;
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
  double measurement_confidence = 1.0;
  double fixed_frame_speed = 0.0;
  double static_hypothesis_residual = 0.0;
  double short_window_prediction_residual = 0.0;
  double ego_motion_explained_residual = 0.0;
  double r_static = 0.0;
  double r_ego = 0.0;
  double r_dynamic = 0.0;
  double r_heading_static = 0.0;
  double imu_yaw_activity = 0.0;
  double reverse_progress = 0.0;
  int motion_class = 0;
  bool ego_yaw_mixing = false;
  bool static_explained = false;
  bool dynamic_candidate = false;
  bool dynamic_residual_win = false;
  bool ego_translation_observable = false;
  bool relative_shortening_only = false;
  bool endpoint_quality_ok = true;
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
  double world_vx = 0.0;
  double world_vy = 0.0;
  double speed = 0.0;
  double position_sigma = 0.20;
  double heading_sigma = 0.40;
  double speed_sigma = 0.50;
  double confidence = 0.50;
  double robust_weight = 1.0;
  double filter_residual = 0.0;
  bool filtered = false;
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

struct LeadFilterSnapshot {
  bool valid = false;
  ros::Time stamp;
  double local_x = 0.0;
  double local_y = 0.0;
  double relative_heading = 0.0;
  double speed = 0.0;
  double confidence = 0.0;
  double position_sigma = 0.0;
  double heading_sigma = 0.0;
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
  double clip_start_s = 0.0;
  double clip_end_s = 0.0;
  std::string clip_reason = "init";
  std::string clip_start_source = "init";
  std::string clip_end_source = "init";
  bool clip_hold = false;
  double endpoint_proj_dist = 0.0;
  std::string endpoint_quality = "init";
  bool curve_mode = false;
  int endpoint_recapture_frames = 0;
  double endpoint_hold_age = 0.0;
  double curve_arc_progress = 0.0;
  double curve_heading_consistency = 0.0;
  std::string recapture_reason = "init";
  double clip_start_proj_dist = 0.0;
  double clip_endpoint_s_raw = 0.0;
  double clip_endpoint_s_filtered = 0.0;
  bool clip_reorder_used = false;
  std::string clip_hold_reason = "init";
  double clip_stale_age = 0.0;
  double reference_join_s = 0.0;
  std::string reference_join_mode = "init";
  bool reference_join_step_limited = false;
  int reference_connector_points = 0;
  int confirmed_prune_count = 0;
  size_t raw_world_points = 0;
  size_t raw_local_points_before_clip = 0;
  double ff_speed = 0.0;
  double mean_ego_speed = 0.0;
  double lead_filter_confidence = 0.0;
  double lead_filter_pos_sigma = 0.0;
  double lead_filter_heading_sigma = 0.0;
  double lead_filter_speed = 0.0;
  double lead_filter_residual = 0.0;
  bool filter_velocity_damped = false;
  double static_hyp_res = 0.0;
  double r_static = 0.0;
  double r_ego = 0.0;
  double r_dynamic = 0.0;
  double r_heading_static = 0.0;
  double imu_yaw_activity = 0.0;
  double reverse_progress = 0.0;
  std::string motion_class = "init";
  bool ego_translation_observable = false;
  bool relative_shortening_only = false;
  bool confirmed_append_allowed = false;
  std::string confirmed_reject_reason = "init";
  std::string candidate_motion_class = "init";
  bool candidate_dynamic_residual_win = false;
  bool candidate_ego_translation_observable = false;
  std::string tail_admit_reason = "init";
  int confirmed_segment_id = 0;
  bool history_segment_break = false;
  bool speed_source_valid = false;
  double odom_pose_delta_speed = 0.0;
  bool reference_degraded_mode = false;
  std::string endpoint_recapture_mode = "init";
  std::string ego_speed_source = "none";
  std::string ego_speed_source_type = "none";
  double ego_speed_source_age = -1.0;
  bool ned_odom_received = false;
  bool ned_odom_zero_motion = false;
  bool lead_only_bootstrap = false;
  double lead_fusion_res_static = 0.0;
  double lead_fusion_res_dynamic = 0.0;
  double fusion_static_likelihood = 0.0;
  double fusion_dynamic_likelihood = 0.0;
  double fusion_ego_likelihood = 0.0;
  double fusion_motion_score = 0.0;
  double fusion_confirm_accum_s = 0.0;
  double fusion_confirm_accum_t = 0.0;
  double fusion_pos_cov = 0.0;
  double fusion_vel_cov = 0.0;
  double low_speed_motion_score = 0.0;
  bool speed_clamped = false;
  bool raw_clip_min_span_applied = false;
  bool raw_clip_collapse_prevented = false;
  bool dynamic_residual_win_now = false;
  bool ego_explained_win_now = false;
  bool yaw_mix_freeze = false;
  bool signed_wheel_speed_used = false;
  std::string confirmed_freeze_reason = "init";
  double static_prob = 0.5;
  double dynamic_prob = 0.5;
  double fusion_cov_trace = 0.0;
  double wheel_speed_raw = 0.0;
  double carvel_speed_raw = 0.0;
  double system_speed_raw = 0.0;
  double ned_odom_speed_raw = 0.0;
  int anchor_epoch_id = 0;
  double anchor_rebase_dx = 0.0;
  double anchor_rebase_dy = 0.0;
  double anchor_rebase_dyaw = 0.0;
  std::string segment_switch_reason = "none";
  double raw_front_jump = 0.0;
  int raw_heading_jump_count = 0;
  bool reference_continuity_limited = false;
  bool reference_control_valid = true;
  std::string raw_clip_policy = "init";
  double ego_progress_s = 0.0;
  double history_tail_s = 0.0;
  double lead_endpoint_s = 0.0;
  bool clip_end_limited_by_endpoint = false;
  bool raw_preserved_by_history_tail = false;
  double control_feasible_start_s = 0.0;
  int history_geometry_reject_count = 0;
  double history_kink_angle = 0.0;
  bool history_reverse_append_reject = false;
  bool confirmed_append_geometry_valid = true;
  double ego_steer_sign_for_reconstruction = -1.0;
  std::string reconstruction_state = "TRACK_GOOD";
  bool reference_held = false;
  double reference_hold_age = 0.0;
  int raw_segment_count = 0;
  double raw_selected_segment_score = 0.0;
  std::string raw_selected_segment_reason = "init";
  double history_project_progress = 0.0;
  bool history_loop_reject = false;
  int history_tail_only_count = 0;
  double hs_score = 0.0;
  double hd_score = 0.0;
  double ego_motion_explained_res = 0.0;
  bool ego_yaw_mixing = false;
  bool static_mode = false;
  double accum_s = 0.0;
  double accum_t = 0.0;
  size_t confirmed_count = 0;
  size_t tail_count = 0;
};

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
  out.speed_source_valid = a.speed_source_valid || b.speed_source_valid;
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

void recomputeLocalHeading(std::vector<LocalPathPoint>& pts) {
  if (pts.size() < 2) {
    return;
  }

  double prev_yaw = pts.front().yaw;
  {
    const double dx = pts[1].x - pts[0].x;
    const double dy = pts[1].y - pts[0].y;
    if (hypot2(dx, dy) > 1e-4) {
      prev_yaw = std::atan2(dy, dx);
    }
    pts.front().yaw = prev_yaw;
  }

  for (size_t i = 1; i < pts.size(); ++i) {
    const double dx = pts[i].x - pts[i - 1].x;
    const double dy = pts[i].y - pts[i - 1].y;
    if (hypot2(dx, dy) <= 1e-4) {
      pts[i].yaw = prev_yaw;
      continue;
    }

    const double candidate = std::atan2(dy, dx);
    prev_yaw = alignAngleToReference(candidate, prev_yaw);
    pts[i].yaw = prev_yaw;
  }

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
    pt.speed = (1.0 - t) * in[seg].speed + t * in[seg + 1].speed;
    pt.curvature = (1.0 - t) * in[seg].curvature + t * in[seg + 1].curvature;
    pt.predicted = in[seg].predicted || in[seg + 1].predicted;
    pt.motion_class = (t < 0.5) ? in[seg].motion_class : in[seg + 1].motion_class;
    pt.dynamic_residual_win = in[seg].dynamic_residual_win || in[seg + 1].dynamic_residual_win;
    pt.ego_translation_observable =
        in[seg].ego_translation_observable || in[seg + 1].ego_translation_observable;
    pt.relative_shortening_only =
        in[seg].relative_shortening_only || in[seg + 1].relative_shortening_only;
    pt.endpoint_quality_ok = in[seg].endpoint_quality_ok && in[seg + 1].endpoint_quality_ok;
    pt.segment_id = (t < 0.5) ? in[seg].segment_id : in[seg + 1].segment_id;
    pt.anchor_epoch_id = (t < 0.5) ? in[seg].anchor_epoch_id : in[seg + 1].anchor_epoch_id;
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
  out.predicted = a.predicted || b.predicted;
  out.motion_class = (tt < 0.5) ? a.motion_class : b.motion_class;
  out.dynamic_residual_win = a.dynamic_residual_win || b.dynamic_residual_win;
  out.ego_translation_observable = a.ego_translation_observable || b.ego_translation_observable;
  out.relative_shortening_only = a.relative_shortening_only || b.relative_shortening_only;
  out.endpoint_quality_ok = a.endpoint_quality_ok && b.endpoint_quality_ok;
  out.segment_id = (tt < 0.5) ? a.segment_id : b.segment_id;
  out.anchor_epoch_id = (tt < 0.5) ? a.anchor_epoch_id : b.anchor_epoch_id;
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
    pt.curvature = world_pts[i].curvature;
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
                           const ros::Time& stamp,
                           bool held_reference = false) {
  nav_msgs::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = frame_id;
  path.header.seq = held_reference ? 1u : 0u;
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

nav_msgs::Path makeWorldPathMsg(const std::vector<HistoryPoint2D>& pts,
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
    pose.pose.orientation = quaternionFromYaw(pts[i].heading);
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
    double wheelbase = 2.60;
    double min_motion_speed_for_yaw = 0.15;
    double max_abs_yaw_rate = 0.8;
    double max_integration_dt = 0.02;
    double pose_buffer_duration = 5.0;
    double steer_lpf_alpha = 0.25;
    double yaw_sigma_base = 0.01;
    double yaw_sigma_speed_gain = 0.02;
    double yaw_sigma_steer_gain = 0.08;
    bool use_speed_hint = false;
    bool use_system_speed = true;
    bool use_ned_odometry_speed = true;
    bool use_ned_odometry_pose_delta = true;
    double speed_hint_weight = 0.20;
    double speed_hint_timeout = 0.30;
    double speed_source_timeout = 0.30;
    double ego_translation_observable_speed_gate = 0.04;
    double odom_pose_delta_speed_gate = 0.04;
    double odom_pose_delta_max_speed = 3.0;
    double speed_hint_agreement_gate = 0.40;
    bool use_imu_yaw_rate = true;
    double imu_yaw_rate_lpf_alpha = 0.35;
    double imu_yaw_rate_timeout = 0.10;
    double imu_yaw_rate_bias = 0.0;
    double imu_yaw_rate_agreement_gate = 0.35;
    double imu_yaw_sigma_base = 0.005;
    double imu_yaw_sigma_rate_gain = 0.01;
    double imu_yaw_rate_deadband = 0.004;
    double imu_yaw_spike_gate = 0.45;
    double imu_yaw_activity_gate = 0.025;
    double imu_activity_hold_time = 0.25;
    double standstill_speed_gate = 0.04;
    double standstill_gyro_gate = 0.015;
    double low_speed_yaw_gate = 0.12;
    double imu_yaw_mixing_speed_gate = 0.20;
    bool zero_yaw_rate_when_standstill = true;
    double static_anchor_ego_xy_gain = 0.45;
    double static_anchor_ego_xy_gate = 1.50;
    double static_anchor_ego_xy_step = 0.35;
    double static_anchor_ego_xy_min_track_age = 0.60;
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

  void updateSystemSpeed(const ros::Time& stamp,
                         double speed_mps,
                         const std::string& source_type = "geometry_msgs/Twist") {
    integrateTo(stamp);
    system_speed_mps_ = speed_mps;
    system_speed_stamp_ = stamp;
    system_speed_source_type_ = source_type;
    updateFusedSpeed(stamp);
  }

  void updateNedOdometrySpeed(const ros::Time& stamp, double speed_mps) {
    integrateTo(stamp);
    ned_odom_received_ = true;
    ned_odom_twist_zero_ = std::fabs(speed_mps) < cfg_.ego_translation_observable_speed_gate;
    ned_odom_speed_mps_ = speed_mps;
    ned_odom_speed_stamp_ = stamp;
    updateFusedSpeed(stamp);
  }

  void updateNedOdometryPoseDelta(const ros::Time& stamp,
                                  double x,
                                  double y,
                                  double yaw) {
    if (!cfg_.use_ned_odometry_pose_delta || !stamp.isValid()) {
      return;
    }
    ned_odom_received_ = true;
    if (!last_ned_pose_stamp_.isValid()) {
      last_ned_pose_stamp_ = stamp;
      last_ned_pose_x_ = x;
      last_ned_pose_y_ = y;
      last_ned_pose_yaw_ = yaw;
      ned_odom_pose_zero_ = true;
      return;
    }
    const double dt = (stamp - last_ned_pose_stamp_).toSec();
    if (dt <= 1e-3 || dt > std::max(0.05, cfg_.speed_source_timeout * 2.0)) {
      last_ned_pose_stamp_ = stamp;
      last_ned_pose_x_ = x;
      last_ned_pose_y_ = y;
      last_ned_pose_yaw_ = yaw;
      return;
    }

    const double dx = x - last_ned_pose_x_;
    const double dy = y - last_ned_pose_y_;
    const double distance = hypot2(dx, dy);
    double speed = distance / dt;
    const double forward =
        dx * std::cos(last_ned_pose_yaw_) + dy * std::sin(last_ned_pose_yaw_);
    if (forward < -1e-4) {
      speed = -speed;
    }
    if (std::fabs(speed) >= cfg_.odom_pose_delta_speed_gate &&
        std::fabs(speed) <= cfg_.odom_pose_delta_max_speed) {
      ned_odom_pose_delta_speed_mps_ = speed;
      ned_odom_pose_delta_stamp_ = stamp;
      ned_odom_pose_zero_ = false;
    } else {
      ned_odom_pose_delta_speed_mps_ = 0.0;
      ned_odom_pose_delta_stamp_ = ros::Time();
      ned_odom_pose_zero_ = true;
    }

    last_ned_pose_stamp_ = stamp;
    last_ned_pose_x_ = x;
    last_ned_pose_y_ = y;
    last_ned_pose_yaw_ = yaw;
    updateFusedSpeed(stamp);
  }

  void updateImuYawRate(const ros::Time& stamp, double yaw_rate_radps) {
    if (!stamp.isValid()) {
      return;
    }

    integrateTo(stamp);

    double corrected_yaw_rate = yaw_rate_radps - cfg_.imu_yaw_rate_bias;
    corrected_yaw_rate =
        clampValue(corrected_yaw_rate, -std::fabs(cfg_.imu_yaw_spike_gate), std::fabs(cfg_.imu_yaw_spike_gate));
    if (std::fabs(corrected_yaw_rate) < cfg_.imu_yaw_rate_deadband) {
      corrected_yaw_rate = 0.0;
    }
    if (!imu_initialized_) {
      filtered_imu_yaw_rate_ = corrected_yaw_rate;
      imu_initialized_ = true;
    } else {
      filtered_imu_yaw_rate_ =
          (1.0 - cfg_.imu_yaw_rate_lpf_alpha) * filtered_imu_yaw_rate_ +
          cfg_.imu_yaw_rate_lpf_alpha * corrected_yaw_rate;
    }

    imu_yaw_rate_ = corrected_yaw_rate;
    imu_stamp_ = stamp;
    const double activity = std::max(std::fabs(imu_yaw_rate_), std::fabs(filtered_imu_yaw_rate_));
    if (activity >= cfg_.imu_yaw_activity_gate) {
      imu_activity_until_ = stamp + ros::Duration(std::max(0.0, cfg_.imu_activity_hold_time));
    }
  }

  bool correctPoseFromStaticObservation(const ros::Time& stamp,
                                        double anchor_x,
                                        double anchor_y,
                                        double local_x,
                                        double local_y,
                                        double anchor_age) {
    if (!stamp.isValid() || anchor_age < cfg_.static_anchor_ego_xy_min_track_age) {
      return false;
    }
    if (pose_.stamp.isValid() && (pose_.stamp - stamp).toSec() > 0.05) {
      return false;
    }

    integrateTo(stamp);

    const double c = std::cos(pose_.yaw);
    const double s = std::sin(pose_.yaw);
    const double implied_x = anchor_x - (c * local_x - s * local_y);
    const double implied_y = anchor_y - (s * local_x + c * local_y);
    double dx = implied_x - pose_.x;
    double dy = implied_y - pose_.y;
    const double err = hypot2(dx, dy);
    if (err > cfg_.static_anchor_ego_xy_gate) {
      return false;
    }

    const double max_step = std::max(1e-3, cfg_.static_anchor_ego_xy_step);
    if (err > max_step) {
      const double scale = max_step / err;
      dx *= scale;
      dy *= scale;
    }

    const double gain = clampValue(cfg_.static_anchor_ego_xy_gain, 0.0, 1.0);
    pose_.x += gain * dx;
    pose_.y += gain * dy;
    pose_.stamp = stamp;
    pose_.v = fused_speed_mps_;
    pose_.delta = filtered_steer_angle_rad_;
    pose_.yaw_rate = currentYawRate();
    pose_.sigma_yaw = yawSigmaEstimate();
    pose_.speed_source_valid = speedSourceValid();
    pushOrReplacePose(stamp);
    return true;
  }

  void integrateTo(const ros::Time& target_stamp) {
    if (!target_stamp.isValid()) {
      return;
    }

    if (!initialized_) {
      updateFusedSpeed(target_stamp);
      pose_.stamp = target_stamp;
      pose_.v = fused_speed_mps_;
      pose_.delta = filtered_steer_angle_rad_;
      pose_.yaw_rate = 0.0;
      pose_.sigma_yaw = cfg_.yaw_sigma_base;
      pose_.speed_source_valid = speedSourceValid();
      buffer_.push_back(pose_);
      initialized_ = true;
      return;
    }

    if (target_stamp <= pose_.stamp) {
      return;
    }

    updateFusedSpeed(target_stamp);
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
      pose_.speed_source_valid = speedSourceValid();
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

  double steeringYawRate() const {
    if (std::fabs(fused_speed_mps_) < cfg_.min_motion_speed_for_yaw) {
      return 0.0;
    }
    const double yaw_rate =
        fused_speed_mps_ * std::tan(filtered_steer_angle_rad_) / std::max(0.5, cfg_.wheelbase);
    return clampValue(yaw_rate, -cfg_.max_abs_yaw_rate, cfg_.max_abs_yaw_rate);
  }

  double currentYawRate() const {
    const double steer_rate = steeringYawRate();

    if (!cfg_.use_imu_yaw_rate || !imu_initialized_ || !imu_stamp_.isValid()) {
      return steer_rate;
    }

    const double age = pose_.stamp.isValid() ? (pose_.stamp - imu_stamp_).toSec() : 0.0;
    if (age > cfg_.imu_yaw_rate_timeout) {
      return steer_rate;
    }

    const double imu_rate =
        clampValue(filtered_imu_yaw_rate_, -cfg_.max_abs_yaw_rate, cfg_.max_abs_yaw_rate);
    const bool standstill = std::fabs(fused_speed_mps_) < cfg_.standstill_speed_gate;
    if (cfg_.zero_yaw_rate_when_standstill && standstill) {
      return 0.0;
    }

    if (std::fabs(fused_speed_mps_) < cfg_.low_speed_yaw_gate) {
      if (std::fabs(imu_rate) < cfg_.standstill_gyro_gate) {
        return 0.0;
      }
      return 0.75 * imu_rate + 0.25 * steer_rate;
    }

    const bool agreement = std::fabs(imu_rate - steer_rate) < cfg_.imu_yaw_rate_agreement_gate;

    if (!agreement) {
      return 0.75 * imu_rate + 0.25 * steer_rate;
    }

    if (std::fabs(fused_speed_mps_) < cfg_.imu_yaw_mixing_speed_gate) {
      return 0.80 * imu_rate + 0.20 * steer_rate;
    }

    return 0.85 * imu_rate + 0.15 * steer_rate;
  }

  double yawSigmaEstimate() const {
    if (usingFreshImuYawRate()) {
      const double yaw_rate = currentYawRate();
      const double steer_rate = steeringYawRate();
      const double agreement_error = std::fabs(yaw_rate - steer_rate);
      return cfg_.imu_yaw_sigma_base +
             cfg_.imu_yaw_sigma_rate_gain * std::fabs(yaw_rate) +
             0.01 * agreement_error;
    }
    return cfg_.yaw_sigma_base + cfg_.yaw_sigma_speed_gain * std::fabs(steeringYawRate()) +
           cfg_.yaw_sigma_steer_gain * std::fabs(filtered_steer_angle_rad_);
  }

  double fusedSpeed() const { return fused_speed_mps_; }
  double wheelSpeed() const { return wheel_speed_mps_; }
  double speedHint() const { return speed_hint_mps_; }
  double systemSpeed() const { return system_speed_mps_; }
  double nedOdometrySpeed() const { return ned_odom_speed_mps_; }
  double nedOdometryPoseDeltaSpeed() const { return ned_odom_pose_delta_speed_mps_; }
  std::string speedSource() const { return speed_source_; }
  std::string speedSourceType() const { return speed_source_type_; }
  bool nedOdometryReceived() const { return ned_odom_received_; }
  bool nedOdometryZeroMotion() const {
    return ned_odom_received_ && ned_odom_twist_zero_ && ned_odom_pose_zero_;
  }
  double speedSourceAge(const ros::Time& now) const {
    if (speed_source_ == "none" || !selected_speed_stamp_.isValid() || !now.isValid()) {
      return -1.0;
    }
    return std::max(0.0, (now - selected_speed_stamp_).toSec());
  }
  bool egoTranslationObservable() const {
    return speed_source_ != "none" &&
           std::fabs(fused_speed_mps_) >= cfg_.ego_translation_observable_speed_gate;
  }
  bool speedSourceValid() const { return speed_source_ != "none"; }
  double steerAngle() const { return filtered_steer_angle_rad_; }
  double imuYawRate() const { return imu_yaw_rate_; }
  double filteredImuYawRate() const { return filtered_imu_yaw_rate_; }
  double imuYawActivity() const {
    const double activity = std::max(std::fabs(imu_yaw_rate_), std::fabs(filtered_imu_yaw_rate_));
    if (imu_activity_until_.isValid() && pose_.stamp.isValid() && pose_.stamp <= imu_activity_until_) {
      return std::max(activity, cfg_.imu_yaw_activity_gate);
    }
    return activity;
  }
  bool usingFreshImuYawRate() const {
    if (!cfg_.use_imu_yaw_rate || !imu_initialized_ || !imu_stamp_.isValid() || !pose_.stamp.isValid()) {
      return false;
    }
    return (pose_.stamp - imu_stamp_).toSec() <= cfg_.imu_yaw_rate_timeout;
  }
  bool initialized() const { return initialized_; }

 private:
  bool freshSpeedSample(const ros::Time& now, const ros::Time& stamp) const {
    if (!now.isValid() || !stamp.isValid()) {
      return false;
    }
    const double age = (now - stamp).toSec();
    return age >= -0.02 && age <= cfg_.speed_source_timeout;
  }

  bool usableSpeedValue(double speed) const {
    return std::fabs(speed) >= cfg_.ego_translation_observable_speed_gate;
  }

  void updateFusedSpeed(const ros::Time& now) {
    const bool wheel_fresh = freshSpeedSample(now, last_motion_stamp_);
    const bool system_fresh = cfg_.use_system_speed && freshSpeedSample(now, system_speed_stamp_);
    const bool ned_fresh = cfg_.use_ned_odometry_speed && freshSpeedSample(now, ned_odom_speed_stamp_);
    const bool ned_pose_delta_fresh =
        cfg_.use_ned_odometry_pose_delta && freshSpeedSample(now, ned_odom_pose_delta_stamp_);
    const bool hint_fresh = cfg_.use_speed_hint && speed_hint_stamp_.isValid() &&
                            (now - speed_hint_stamp_).toSec() <=
                                std::max(cfg_.speed_hint_timeout, cfg_.speed_source_timeout);

    if (wheel_fresh) {
      fused_speed_mps_ = wheel_speed_mps_;
      speed_source_ = "wheel";
      speed_source_type_ = "move_car/car_parameter";
      selected_speed_stamp_ = last_motion_stamp_;
      return;
    }
    if (system_fresh) {
      fused_speed_mps_ = system_speed_mps_;
      speed_source_ = "system_speed";
      speed_source_type_ = system_speed_source_type_;
      selected_speed_stamp_ = system_speed_stamp_;
      return;
    }
    if (ned_fresh) {
      fused_speed_mps_ = ned_odom_speed_mps_;
      speed_source_ = "ned_odometry";
      speed_source_type_ = "nav_msgs/Odometry.twist";
      selected_speed_stamp_ = ned_odom_speed_stamp_;
      return;
    }
    if (ned_pose_delta_fresh) {
      fused_speed_mps_ = ned_odom_pose_delta_speed_mps_;
      speed_source_ = "ned_pose_delta";
      speed_source_type_ = "nav_msgs/Odometry.pose_delta";
      selected_speed_stamp_ = ned_odom_pose_delta_stamp_;
      return;
    }
    if (hint_fresh) {
      fused_speed_mps_ = speed_hint_mps_;
      speed_source_ = "carvel";
      speed_source_type_ = "geometry_msgs/Twist";
      selected_speed_stamp_ = speed_hint_stamp_;
      return;
    }

    fused_speed_mps_ = 0.0;
    speed_source_ = "none";
    speed_source_type_ = "none";
    selected_speed_stamp_ = ros::Time();
  }

  void pruneBuffer(const ros::Time& now) {
    while (buffer_.size() > 2 && (now - buffer_.front().stamp).toSec() > cfg_.pose_buffer_duration) {
      buffer_.pop_front();
    }
  }

  void pushOrReplacePose(const ros::Time& stamp) {
    if (!buffer_.empty() && std::fabs((buffer_.back().stamp - stamp).toSec()) < 1e-6) {
      buffer_.back() = pose_;
    } else {
      buffer_.push_back(pose_);
    }
    pruneBuffer(stamp);
  }

  Config cfg_;
  bool initialized_ = false;
  EgoPose2D pose_;
  std::deque<EgoPose2D> buffer_;
  ros::Time last_motion_stamp_;
  ros::Time speed_hint_stamp_;
  ros::Time system_speed_stamp_;
  ros::Time ned_odom_speed_stamp_;
  ros::Time ned_odom_pose_delta_stamp_;
  ros::Time selected_speed_stamp_;
  ros::Time imu_stamp_;
  ros::Time imu_activity_until_;
  ros::Time last_ned_pose_stamp_;
  double wheel_speed_mps_ = 0.0;
  double speed_hint_mps_ = 0.0;
  double system_speed_mps_ = 0.0;
  double ned_odom_speed_mps_ = 0.0;
  double ned_odom_pose_delta_speed_mps_ = 0.0;
  double fused_speed_mps_ = 0.0;
  std::string speed_source_ = "none";
  std::string speed_source_type_ = "none";
  std::string system_speed_source_type_ = "geometry_msgs/Twist";
  bool ned_odom_received_ = false;
  bool ned_odom_twist_zero_ = true;
  bool ned_odom_pose_zero_ = true;
  double last_ned_pose_x_ = 0.0;
  double last_ned_pose_y_ = 0.0;
  double last_ned_pose_yaw_ = 0.0;
  bool steer_initialized_ = false;
  double filtered_steer_angle_rad_ = 0.0;
  double imu_yaw_rate_ = 0.0;
  double filtered_imu_yaw_rate_ = 0.0;
  bool imu_initialized_ = false;
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

class LeadObservationFilter {
 public:
  struct Config {
    bool enabled = true;
    double measurement_position_sigma = 0.18;
    double measurement_position_range_gain = 0.015;
    double measurement_heading_sigma = 0.45;
    double process_position_sigma = 0.10;
    double process_velocity_sigma = 0.70;
    double process_heading_sigma = 0.35;
    double max_prediction_dt = 0.25;
    double reset_timeout = 0.80;
    double robust_cauchy_scale = 2.5;
    double min_robust_weight = 0.08;
    double window_duration = 0.80;
    int window_min_points = 4;
    double window_velocity_blend = 0.65;
    double heading_from_velocity_speed_gate = 0.12;
    double heading_update_speed_gate = 0.18;
    double heading_flip_gate = 1.20;
    double static_velocity_damping = 0.35;
    double outlier_velocity_damping = 0.60;
    double confidence_lpf_alpha = 0.35;
    double confidence_position_sigma_ref = 0.40;
    double confidence_heading_sigma_ref = 0.75;
  };

  explicit LeadObservationFilter(const Config& cfg) : cfg_(cfg) {}

  LeadObservation update(const LeadObservation& measurement,
                         const EgoPose2D& ego_at_obs,
                         TrackerDebugState* debug) {
    if (!cfg_.enabled || !measurement.valid) {
      return measurement;
    }

    if (!initialized_ || !stamp_.isValid() ||
        (measurement.stamp.isValid() && (measurement.stamp - stamp_).toSec() > cfg_.reset_timeout)) {
      initialize(measurement);
      LeadObservation out = buildOutput(measurement, ego_at_obs, 1.0, 0.0);
      updateDebug(out, debug);
      return out;
    }

    predictTo(measurement.stamp);

    const double meas_pos_sigma =
        std::max(0.03, cfg_.measurement_position_sigma +
                           cfg_.measurement_position_range_gain * measurement.range);
    const double meas_heading_sigma = std::max(0.05, cfg_.measurement_heading_sigma);
    const double rx = measurement.world_x - x_;
    const double ry = measurement.world_y - y_;
    double measurement_heading = measurement.world_heading;
    if (std::fabs(wrapAngle(measurement_heading + M_PI - heading_)) <
        std::fabs(wrapAngle(measurement_heading - heading_))) {
      measurement_heading = wrapAngle(measurement_heading + M_PI);
    }
    const double rh = wrapAngle(measurement_heading - heading_);
    const double pos_norm =
        std::sqrt((rx * rx + ry * ry) /
                  std::max(1e-6, 0.5 * (cov_x_ + cov_y_) + meas_pos_sigma * meas_pos_sigma));
    const double heading_norm =
        std::fabs(rh) / std::max(1e-6, std::sqrt(cov_heading_) + meas_heading_sigma);
    const double residual_metric = std::sqrt(pos_norm * pos_norm + 0.25 * heading_norm * heading_norm);
    const double robust_weight =
        std::max(cfg_.min_robust_weight,
                 1.0 / (1.0 + (residual_metric * residual_metric) /
                                  std::max(1e-6, cfg_.robust_cauchy_scale * cfg_.robust_cauchy_scale)));

    const double pos_r_eff = (meas_pos_sigma * meas_pos_sigma) / robust_weight;
    const double heading_r_eff = (meas_heading_sigma * meas_heading_sigma) / robust_weight;
    const double kx = cov_x_ / std::max(1e-6, cov_x_ + pos_r_eff);
    const double ky = cov_y_ / std::max(1e-6, cov_y_ + pos_r_eff);
    double kh = cov_heading_ / std::max(1e-6, cov_heading_ + heading_r_eff);
    const double predicted_speed = hypot2(vx_, vy_);
    if (predicted_speed < cfg_.heading_update_speed_gate &&
        std::fabs(rh) > cfg_.heading_flip_gate) {
      kh = 0.0;
    }

    x_ += kx * rx;
    y_ += ky * ry;
    heading_ = wrapAngle(heading_ + kh * rh);
    cov_x_ = std::max(1e-5, (1.0 - kx) * cov_x_);
    cov_y_ = std::max(1e-5, (1.0 - ky) * cov_y_);
    cov_heading_ = std::max(1e-5, (1.0 - kh) * cov_heading_);
    stamp_ = measurement.stamp;

    pushSample(measurement.stamp, x_, y_, heading_, robust_weight);
    applyWindowRegression();
    bool velocity_damped = false;
    if (debug != NULL) {
      const bool static_like =
          debug->ego_yaw_mixing ||
          debug->motion_class == "YAW_MIXING" ||
          debug->motion_class == "STATIC_OR_EGO_EXPLAINED" ||
          debug->motion_class == "RELATIVE_SHORTENING_ONLY" ||
          debug->history_reason == "static_mode_freeze_confirmed_history";
      const bool bad_update =
          debug->history_reason == "outlier_rejected" ||
          debug->history_reason == "tail_fast_rejected";
      double damping = 1.0;
      if (static_like) {
        damping = std::min(damping, cfg_.static_velocity_damping);
      }
      if (bad_update) {
        damping = std::min(damping, cfg_.outlier_velocity_damping);
      }
      if (damping < 0.999) {
        vx_ *= damping;
        vy_ *= damping;
        velocity_damped = true;
      }
    }
    const double confidence = updateConfidence(robust_weight);

    LeadObservation out = buildOutput(measurement, ego_at_obs, robust_weight, residual_metric);
    out.confidence = confidence;
    if (debug != NULL) {
      debug->filter_velocity_damped = velocity_damped;
    }
    updateDebug(out, debug);
    return out;
  }

  bool snapshot(const EgoPose2D& ego_now, LeadFilterSnapshot* out) const {
    if (out == NULL || !initialized_ || !stamp_.isValid()) {
      return false;
    }
    if (ego_now.stamp.isValid() && (ego_now.stamp - stamp_).toSec() > cfg_.reset_timeout) {
      return false;
    }

    out->valid = true;
    out->stamp = stamp_;
    transformWorldToLocal(ego_now, x_, y_, &out->local_x, &out->local_y);
    out->relative_heading = wrapAngle(heading_ - ego_now.yaw);
    out->speed = hypot2(vx_, vy_);
    out->confidence = confidence_;
    out->position_sigma = positionSigma();
    out->heading_sigma = std::sqrt(std::max(1e-6, cov_heading_));
    return true;
  }

 private:
  struct Sample {
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double weight = 1.0;
  };

  void initialize(const LeadObservation& measurement) {
    initialized_ = true;
    stamp_ = measurement.stamp;
    x_ = measurement.world_x;
    y_ = measurement.world_y;
    heading_ = measurement.world_heading;
    vx_ = 0.0;
    vy_ = 0.0;
    cov_x_ = std::pow(std::max(0.03, cfg_.measurement_position_sigma), 2.0);
    cov_y_ = cov_x_;
    cov_vx_ = std::pow(std::max(0.05, cfg_.process_velocity_sigma), 2.0);
    cov_vy_ = cov_vx_;
    cov_heading_ = std::pow(std::max(0.05, cfg_.measurement_heading_sigma), 2.0);
    confidence_ = 0.55;
    samples_.clear();
    pushSample(measurement.stamp, x_, y_, heading_, 1.0);
  }

  void predictTo(const ros::Time& stamp) {
    if (!stamp.isValid() || !stamp_.isValid() || stamp <= stamp_) {
      return;
    }
    const double dt = clampValue((stamp - stamp_).toSec(), 0.0, cfg_.max_prediction_dt);
    x_ += vx_ * dt;
    y_ += vy_ * dt;
    cov_x_ += cov_vx_ * dt * dt + cfg_.process_position_sigma * cfg_.process_position_sigma * dt;
    cov_y_ += cov_vy_ * dt * dt + cfg_.process_position_sigma * cfg_.process_position_sigma * dt;
    cov_vx_ += cfg_.process_velocity_sigma * cfg_.process_velocity_sigma * dt;
    cov_vy_ += cfg_.process_velocity_sigma * cfg_.process_velocity_sigma * dt;
    cov_heading_ += cfg_.process_heading_sigma * cfg_.process_heading_sigma * dt;
    stamp_ = stamp;
  }

  void pushSample(const ros::Time& stamp, double x, double y, double heading, double weight) {
    Sample sample;
    sample.stamp = stamp;
    sample.x = x;
    sample.y = y;
    sample.heading = heading;
    sample.weight = clampValue(weight, cfg_.min_robust_weight, 1.0);
    samples_.push_back(sample);
    while (samples_.size() > 2 && stamp.isValid() && samples_.front().stamp.isValid() &&
           (stamp - samples_.front().stamp).toSec() > cfg_.window_duration) {
      samples_.pop_front();
    }
  }

  void applyWindowRegression() {
    if (samples_.size() < static_cast<size_t>(std::max(2, cfg_.window_min_points))) {
      return;
    }

    const ros::Time t0 = samples_.front().stamp;
    double wsum = 0.0;
    double mean_t = 0.0;
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (size_t i = 0; i < samples_.size(); ++i) {
      const double t = (samples_[i].stamp - t0).toSec();
      const double w = std::max(1e-3, samples_[i].weight);
      wsum += w;
      mean_t += w * t;
      mean_x += w * samples_[i].x;
      mean_y += w * samples_[i].y;
    }
    if (wsum < 1e-6) {
      return;
    }
    mean_t /= wsum;
    mean_x /= wsum;
    mean_y /= wsum;

    double var_t = 0.0;
    double cov_tx = 0.0;
    double cov_ty = 0.0;
    double residual_sum = 0.0;
    for (size_t i = 0; i < samples_.size(); ++i) {
      const double t = (samples_[i].stamp - t0).toSec();
      const double w = std::max(1e-3, samples_[i].weight);
      var_t += w * (t - mean_t) * (t - mean_t);
      cov_tx += w * (t - mean_t) * (samples_[i].x - mean_x);
      cov_ty += w * (t - mean_t) * (samples_[i].y - mean_y);
    }
    if (var_t < 1e-5) {
      return;
    }

    const double reg_vx = cov_tx / var_t;
    const double reg_vy = cov_ty / var_t;
    const double blend = clampValue(cfg_.window_velocity_blend, 0.0, 1.0);
    vx_ = (1.0 - blend) * vx_ + blend * reg_vx;
    vy_ = (1.0 - blend) * vy_ + blend * reg_vy;

    for (size_t i = 0; i < samples_.size(); ++i) {
      const double t = (samples_[i].stamp - t0).toSec();
      const double px = mean_x + reg_vx * (t - mean_t);
      const double py = mean_y + reg_vy * (t - mean_t);
      residual_sum += samples_[i].weight * pointDistance(px, py, samples_[i].x, samples_[i].y);
    }
    const double reg_residual = residual_sum / std::max(1e-3, wsum);
    cov_vx_ = std::max(1e-4, std::min(cov_vx_, reg_residual * reg_residual + 0.04));
    cov_vy_ = std::max(1e-4, std::min(cov_vy_, reg_residual * reg_residual + 0.04));

    const double speed = hypot2(vx_, vy_);
    if (speed > cfg_.heading_from_velocity_speed_gate) {
      const double reg_heading = std::atan2(vy_, vx_);
      heading_ = wrapAngle(heading_ + blend * wrapAngle(reg_heading - heading_));
      cov_heading_ = std::max(1e-5, std::min(cov_heading_, reg_residual * reg_residual + 0.03));
    }
  }

  double updateConfidence(double robust_weight) {
    const double pos_term =
        1.0 / (1.0 + positionSigma() / std::max(1e-3, cfg_.confidence_position_sigma_ref));
    const double heading_term =
        1.0 / (1.0 + std::sqrt(std::max(1e-6, cov_heading_)) /
                         std::max(1e-3, cfg_.confidence_heading_sigma_ref));
    const double inst_conf = clampValue(robust_weight * (0.65 * pos_term + 0.35 * heading_term), 0.0, 1.0);
    confidence_ =
        (1.0 - cfg_.confidence_lpf_alpha) * confidence_ + cfg_.confidence_lpf_alpha * inst_conf;
    return confidence_;
  }

  double positionSigma() const {
    return std::sqrt(std::max(1e-6, 0.5 * (cov_x_ + cov_y_)));
  }

  LeadObservation buildOutput(const LeadObservation& measurement,
                              const EgoPose2D& ego_at_obs,
                              double robust_weight,
                              double residual_metric) const {
    LeadObservation out = measurement;
    out.world_x = x_;
    out.world_y = y_;
    out.world_heading = heading_;
    out.world_vx = vx_;
    out.world_vy = vy_;
    out.speed = hypot2(vx_, vy_);
    out.position_sigma = positionSigma();
    out.heading_sigma = std::sqrt(std::max(1e-6, cov_heading_));
    out.speed_sigma = std::sqrt(std::max(1e-6, 0.5 * (cov_vx_ + cov_vy_)));
    out.confidence = confidence_;
    out.robust_weight = robust_weight;
    out.filter_residual = residual_metric;
    out.filtered = true;
    transformWorldToLocal(ego_at_obs, out.world_x, out.world_y, &out.local_x, &out.local_y);
    out.local_heading = wrapAngle(out.world_heading - ego_at_obs.yaw);
    out.range = hypot2(out.local_x, out.local_y);
    return out;
  }

  void updateDebug(const LeadObservation& out, TrackerDebugState* debug) const {
    if (debug == NULL) {
      return;
    }
    debug->lead_filter_confidence = out.confidence;
    debug->lead_filter_pos_sigma = out.position_sigma;
    debug->lead_filter_heading_sigma = out.heading_sigma;
    debug->lead_filter_speed = out.speed;
    debug->lead_filter_residual = out.filter_residual;
  }

  Config cfg_;
  bool initialized_ = false;
  ros::Time stamp_;
  double x_ = 0.0;
  double y_ = 0.0;
  double vx_ = 0.0;
  double vy_ = 0.0;
  double heading_ = 0.0;
  double cov_x_ = 0.04;
  double cov_y_ = 0.04;
  double cov_vx_ = 0.25;
  double cov_vy_ = 0.25;
  double cov_heading_ = 0.25;
  double confidence_ = 0.0;
  std::deque<Sample> samples_;
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
    double confirmed_history_max_age = 60.0;
    int confirmed_history_min_points_keep = 30;
    int history_max_points = 300;
    bool export_static_anchor_to_history = false;
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
    double measurement_along_sigma = 0.08;
    double measurement_cross_sigma = 0.12;
    double measurement_cross_range_gain = 0.020;
    double measurement_cross_yaw_uncertainty_gain = 1.80;
    double yaw_rate_cross_weight_gain = 0.20;
    double temporal_min_progress = 0.02;
    double static_preserve_trail_span = 0.80;
    double trail_append_min_distance = 0.12;
    double trail_anchor_update_alpha = 0.10;
    int tail_confirm_min_points = 3;
    double tail_confirm_min_distance = 0.06;
    double tail_confirm_min_age = 0.18;
    double confirm_dynamic_score_margin = 0.12;
    double confirm_static_residual_gate = 0.14;
    double confirm_min_progress = 0.08;
    double confirm_accum_distance = 0.05;
    double confirm_accum_timeout = 0.80;
    double ego_static_speed_gate = 0.05;
    double static_release_score_margin = 0.06;
    int raw_tail_keep_points = 6;
    double dual_hypothesis_score_margin = 0.02;
    int tail_buffer_max_points = 12;
    double tail_buffer_max_age = 0.60;
    int static_tail_keep_points = 3;
    bool publish_tail_suffix = false;
    double tail_confirm_min_time = 0.25;
    double lead_tail_max_speed = 1.20;
    double tail_cross_gate = 0.30;
    double tail_reverse_gate = -0.03;
    double static_anchor_res_gate = 0.04;
    double static_anchor_progress_gate = 0.04;
    double static_anchor_speed_gate = 0.06;
    double static_anchor_append_distance_gate = 0.08;
    double ego_yaw_mixing_yaw_rate_gate = 0.04;
    double ego_yaw_mixing_lateral_gate = 0.04;
    double ego_yaw_mixing_residual_gate = 0.10;
    double ego_yaw_mixing_anchor_gate = 0.10;
    double ego_yaw_mixing_dynamic_ratio = 0.75;
    double ego_yaw_mixing_heading_gate = 0.20;
    double ego_yaw_mixing_weight_scale = 0.25;
    double imu_yaw_activity_gate = 0.025;
    double dynamic_residual_margin = 0.08;
    double reverse_progress_gate = 0.04;
    int bootstrap_consecutive_dynamic = 3;
    double history_kink_min_dist = 0.30;
    double history_kink_heading_gate = 0.35;
    double confirmed_history_kink_reject_angle = 0.55;
    double confirmed_history_reverse_gate = 0.035;
    double confirmed_history_short_reverse_dist = 0.18;
    double confirmed_history_curvature_impulse_gate = 7.0;
    double confirmed_history_stop_speed_gate = 0.03;
    double confirmed_history_stop_residual_gate = 0.45;
    double confirmed_history_stop_lead_speed_gate = 0.55;
    bool confirmed_history_projection_gate_enabled = true;
    double confirmed_history_projection_backtrack_gate = 0.08;
    double confirmed_history_projection_near_dist = 0.16;
    double confirmed_history_projection_lateral_gate = 0.22;
    int reconstruction_reseed_bad_frames = 18;
    double history_max_confirm_speed = 3.0;
    int static_anchor_ego_correction_min_static_votes = 10;
    int static_anchor_ego_correction_max_moving_votes = 0;
    double static_dynamic_probe_min_progress_ratio = 0.35;
    double static_dynamic_probe_speed_ratio = 0.45;
    double static_dynamic_probe_residual_margin = 0.03;
    int dynamic_bootstrap_min_tail_points = 4;
    double dynamic_bootstrap_min_span = 0.20;
    double dynamic_bootstrap_min_ff_speed = 0.08;
    double dynamic_bootstrap_score_margin = 0.08;
    int endpoint_recapture_min_frames = 4;
    double raw_clip_projection_far_hold_max_time = 0.80;
    double endpoint_recapture_confidence_gate = 0.75;
    double dynamic_curve_cross_gate = 1.20;
    double dynamic_curve_min_arc = 0.20;
    double dynamic_curve_heading_gate = 0.45;
    bool yaw_mixing_freeze_requires_endpoint_near_path = true;
    double confirmed_curve_interp_ds = 0.10;
    double ego_translation_observable_speed_gate = 0.04;
    bool confirmed_require_dynamic_motion_class = true;
    bool relative_shortening_freeze_confirmed = true;
    bool confirm_require_candidate_metadata = true;
    bool disable_filter_speed_dynamic_when_ego_unobservable = true;
    double history_segment_break_gap = 0.45;
    bool lead_only_bootstrap_enabled = true;
    int lead_only_bootstrap_min_points = 4;
    double lead_only_bootstrap_min_span = 0.20;
    double lead_only_bootstrap_min_progress = 0.12;
    double lead_only_bootstrap_min_ff_speed = 0.08;
    double lead_only_bootstrap_max_imu_activity = 0.015;
    double lead_only_bootstrap_static_residual_gate = 0.025;
    bool use_fusion_tracker = true;
    double fusion_meas_sigma_base = 0.10;
    double fusion_meas_sigma_range_gain = 0.015;
    double fusion_process_pos = 0.03;
    double fusion_process_vel = 0.08;
    double fusion_prob_alpha = 0.25;
    double fusion_static_residual_gate = 0.08;
    double fusion_dynamic_residual_margin = 0.04;
    bool fusion_hard_residual_gate_enabled = false;
    double fusion_dynamic_prob_bootstrap = 0.55;
    double fusion_dynamic_prob_append = 0.58;
    double fusion_static_prob_freeze = 0.72;
    double fusion_append_min_distance = 0.04;
    double fusion_bootstrap_min_span = 0.18;
    int fusion_bootstrap_min_points = 3;
    double fusion_cov_trace_gate = 2.50;
    double fusion_pos_cov_gate = 0.35;
    double fusion_append_max_interval = 0.60;
    double fusion_yaw_freeze_activity = 0.035;
    double fusion_relative_shorten_gate = 0.04;
    int fusion_static_hold_frames = 5;
    int fusion_ego_explained_hold_frames = 5;
    int fusion_yaw_mix_hold_frames = 3;
    double fusion_max_confirm_speed = 0.80;
    double fusion_segment_break_gap = 0.80;
    double fusion_reverse_progress_gate = 0.05;
    double fusion_unobservable_cross_gate = 0.45;
    double ego_explained_freeze_gate = 0.03;
    bool yaw_mix_confirm_freeze = true;
    double fusion_prob_decay_on_static = 0.45;
    double lead_confirm_max_speed = 0.80;
    double lead_speed_nominal_min = 0.10;
    double lead_speed_nominal_max = 0.50;
    double lead_speed_abs_max = 0.80;
    bool fusion_static_observation_hold_enabled = true;
    double fusion_static_observation_res_gate = 0.08;
    double fusion_static_observation_speed_gate = 0.06;
    double fusion_static_observation_step_gate = 0.03;
  };

  explicit LeadTrailTracker(const Config& cfg) : cfg_(cfg) {}

  enum MotionClass {
    MOTION_STATIC_OR_EGO_EXPLAINED = 0,
    MOTION_YAW_MIXING = 1,
    MOTION_DYNAMIC_CANDIDATE = 2,
    MOTION_OUTLIER = 3,
    MOTION_CURVED_DYNAMIC_CANDIDATE = 4,
    MOTION_RELATIVE_SHORTENING_ONLY = 5,
    MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP = 6
  };

  void markLostIfExpired(const ros::Time& stamp) {
    if (track_.active && (stamp - track_.stamp).toSec() > cfg_.lost_timeout) {
      clearStateUnlocked();
    }
  }

  void ingestObservation(const LeadObservation& obs,
                         const EgoPose2D& ego_at_obs,
                         double ego_yaw_rate,
                         double imu_yaw_activity,
                         double ego_yaw_sigma,
                         double steer_angle_rad,
                         TrackerDebugState* debug) {
    (void)steer_angle_rad;
    if (!obs.valid) {
      if (debug != NULL) {
        debug->history_reason = "invalid_observation";
        debug->history_size = exportHistoryUnlocked().size();
      }
      return;
    }

    const double confidence_scale = 1.0 / std::max(0.20, obs.confidence);
    const double sigma =
        cfg_.innovation_base_gate + cfg_.innovation_range_gain * obs.range +
        cfg_.innovation_turn_gain * std::fabs(ego_yaw_rate) * obs.range +
        obs.position_sigma * confidence_scale;
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
        }
        return;
      }
    } else {
      last_innovation_norm_ = 0.0;
      last_innovation_metric_ = 0.0;
    }

    LeadNode node;
    node.stamp = obs.stamp;
    node.ego = ego_at_obs;
    node.x = obs.world_x;
    node.y = obs.world_y;
    node.yaw = obs.world_heading;
    node.v = obs.speed;
    node.robust_weight = obs.robust_weight;
    node.measurement_confidence = clampValue(obs.confidence * obs.robust_weight, 0.05, 1.0);
    node.fixed_frame_speed = obs.speed;
    node.imu_yaw_activity = imu_yaw_activity;
    node.heading_hint = obs.world_heading;
    node.has_measurement = true;
    node.z_rel_x = obs.local_x;
    node.z_rel_y = obs.local_y;
    node.range = obs.range;
    node.w_cross =
        cfg_.measurement_cross_sigma + cfg_.measurement_cross_range_gain * obs.range +
        cfg_.measurement_cross_yaw_uncertainty_gain * obs.range * ego_yaw_sigma +
        cfg_.yaw_rate_cross_weight_gain * obs.range * std::fabs(ego_yaw_rate) +
        obs.position_sigma * confidence_scale;
    node.w_along = cfg_.measurement_along_sigma + 0.50 * obs.position_sigma * confidence_scale;

    if (nodes_.empty() || (obs.stamp - nodes_.back().stamp).toSec() > cfg_.lost_timeout) {
      clearStateUnlocked();
      static_anchor_x_ = obs.world_x;
      static_anchor_y_ = obs.world_y;
    }

    updateMeasurementHypothesisUnlocked(&node);
    if (!nodes_.empty()) {
      const LeadNode& prev = nodes_.back();
      const double dx = node.x - prev.x;
      const double dy = node.y - prev.y;
      node.reverse_progress = dx * std::cos(prev.yaw) + dy * std::sin(prev.yaw);
    }
    node.motion_class = classifyMotionUnlocked(node, nodes_.empty() ? NULL : &nodes_.back());
    node.dynamic_residual_win =
        node.r_dynamic + cfg_.dynamic_residual_margin < std::min(node.r_static, node.r_ego);
    node.ego_translation_observable = egoTranslationObservableUnlocked();
    node.relative_shortening_only = node.motion_class == MOTION_RELATIVE_SHORTENING_ONLY;
    node.endpoint_quality_ok =
        node.measurement_confidence >= std::max(0.20, cfg_.endpoint_recapture_confidence_gate);
    node.static_explained = node.motion_class == MOTION_STATIC_OR_EGO_EXPLAINED ||
                            node.motion_class == MOTION_RELATIVE_SHORTENING_ONLY;
    node.dynamic_candidate = node.motion_class == MOTION_DYNAMIC_CANDIDATE ||
                             node.motion_class == MOTION_CURVED_DYNAMIC_CANDIDATE ||
                             node.motion_class == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP;

    last_ego_yaw_mixing_ = false;
    if (!nodes_.empty()) {
      const LeadNode& prev = nodes_.back();
      double mix_score = 0.0;
      if (node.motion_class != MOTION_CURVED_DYNAMIC_CANDIDATE &&
          (node.motion_class == MOTION_YAW_MIXING ||
           egoYawMixingLikelyUnlocked(node, prev, &mix_score))) {
        node.x = prev.x;
        node.y = prev.y;
        node.yaw = prev.yaw;
        node.heading_hint = prev.yaw;
        node.v = 0.0;
        node.a = 0.0;
        node.kappa = prev.kappa;
        node.dkappa = 0.0;
        node.fixed_frame_speed = 0.0;
        node.robust_weight *= cfg_.ego_yaw_mixing_weight_scale;
        node.measurement_confidence *= cfg_.ego_yaw_mixing_weight_scale;
        node.ego_yaw_mixing = true;
        node.motion_class = MOTION_YAW_MIXING;
        node.dynamic_residual_win = false;
        node.relative_shortening_only = false;
        node.endpoint_quality_ok = false;
        node.static_explained = true;
        node.dynamic_candidate = false;
        last_ego_yaw_mixing_ = true;
        last_history_action_ = "ego_yaw_mixing_freeze_tail";
        (void)mix_score;
      }
    }

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

  std::vector<HistoryPoint2D> worldHistoryForLog(const ros::Time& stamp) const {
    std::vector<HistoryPoint2D> history;
    history.insert(history.end(), confirmed_trail_world_.begin(), confirmed_trail_world_.end());

    if (history.empty() && track_.static_mode && has_static_anchor_) {
      HistoryPoint2D anchor;
      anchor.stamp = static_anchor_stamp_.isValid() ? static_anchor_stamp_ : stamp;
      anchor.x = static_anchor_x_;
      anchor.y = static_anchor_y_;
      anchor.heading = static_anchor_heading_;
      anchor.speed = 0.0;
      anchor.curvature = 0.0;
      anchor.predicted = false;
      history.push_back(anchor);
    }

    return removeNearDuplicatePoints(history, 0.01);
  }

  const LeadState2D& track() const { return track_; }
  bool hasActiveTrack() const { return track_.active; }
  double staticModeAge(const ros::Time& stamp) const {
    if (!track_.active || !track_.static_mode || !static_mode_start_stamp_.isValid() || !stamp.isValid()) {
      return 0.0;
    }
    return std::max(0.0, (stamp - static_mode_start_stamp_).toSec());
  }
  bool canUseStaticAnchorForEgoCorrection(const ros::Time& stamp) const {
    if (!track_.active || !track_.static_mode || !has_static_anchor_) {
      return false;
    }
    if (staticModeAge(stamp) <= 0.0) {
      return false;
    }
    if (static_votes_ < cfg_.static_anchor_ego_correction_min_static_votes ||
        moving_votes_ > cfg_.static_anchor_ego_correction_max_moving_votes) {
      return false;
    }
    if (staticModeDynamicProbeUnlocked()) {
      return false;
    }
    const double fixed_speed = meanFixedFrameSpeedUnlocked();
    const double progress = std::max(dynamicTrailProgressUnlocked(), estimateWindowProgressUnlocked());
    return fixed_speed < cfg_.static_anchor_speed_gate &&
           progress < std::max(cfg_.static_anchor_progress_gate, 0.5 * cfg_.confirm_min_progress);
  }
  bool movingFlag() const { return moving_flag_; }
  double innovationNorm() const { return last_innovation_norm_; }
  double innovationMetric() const { return last_innovation_metric_; }

 private:
  struct LeadFusionState {
    bool active = false;
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double heading = 0.0;
    double cov_x = 0.20;
    double cov_y = 0.20;
    double cov_vx = 0.40;
    double cov_vy = 0.40;
    double static_anchor_x = 0.0;
    double static_anchor_y = 0.0;
    bool static_anchor_valid = false;
    double static_prob = 0.70;
    double dynamic_prob = 0.30;
    ros::Time last_append_stamp;
    double last_range = 0.0;
    double last_local_x = 0.0;
    bool last_measurement_valid = false;
  };

  static double logistic(double x) {
    if (x > 30.0) {
      return 1.0;
    }
    if (x < -30.0) {
      return 0.0;
    }
    return 1.0 / (1.0 + std::exp(-x));
  }

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

  double meanEgoMotionExplainedResidualUnlocked() const {
    if (nodes_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    double wsum = 0.0;
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const double w = std::max(1e-3, nodes_[i].robust_weight);
      sum += w * nodes_[i].ego_motion_explained_residual;
      wsum += w;
    }
    return sum / std::max(1e-3, wsum);
  }

  double meanAbsEgoYawRateUnlocked() const {
    if (nodes_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < nodes_.size(); ++i) {
      sum += std::fabs(nodes_[i].ego.yaw_rate);
    }
    return sum / static_cast<double>(nodes_.size());
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
    const double ego_explain_ratio =
        meanEgoMotionExplainedResidualUnlocked() / std::max(1e-3, cfg_.ego_compensated_static_gate);
    const double yaw_activity = clampValue(meanAbsEgoYawRateUnlocked() / 0.15, 0.0, 1.0);
    const double ego_motion_explained_bonus =
        yaw_activity * (1.0 - clampValue(ego_explain_ratio, 0.0, 1.0));
    return 0.55 * residual_term + 0.20 * speed_term + 0.15 * progress_term -
           0.10 * advantage_term - 0.08 * ego_motion_explained_bonus;
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
    const double ego_explain_ratio =
        meanEgoMotionExplainedResidualUnlocked() / std::max(1e-3, cfg_.ego_compensated_static_gate);
    const double yaw_activity = clampValue(meanAbsEgoYawRateUnlocked() / 0.15, 0.0, 1.0);
    const double ego_motion_explained_penalty =
        yaw_activity * (1.0 - clampValue(ego_explain_ratio, 0.0, 1.0));
    (void)speed_ratio;
    (void)progress;
    return 0.60 * residual_term + 0.15 * speed_penalty + 0.15 * progress_penalty +
           0.10 * advantage_penalty + 0.10 * ego_motion_explained_penalty;
  }

  const char* motionClassName(MotionClass motion_class) const {
    switch (motion_class) {
      case MOTION_YAW_MIXING:
        return "YAW_MIXING";
      case MOTION_DYNAMIC_CANDIDATE:
        return "DYNAMIC_CANDIDATE";
      case MOTION_CURVED_DYNAMIC_CANDIDATE:
        return "CURVED_DYNAMIC_CANDIDATE";
      case MOTION_OUTLIER:
        return "OUTLIER";
      case MOTION_RELATIVE_SHORTENING_ONLY:
        return "RELATIVE_SHORTENING_ONLY";
      case MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP:
        return "LEAD_ONLY_DYNAMIC_BOOTSTRAP";
      case MOTION_STATIC_OR_EGO_EXPLAINED:
      default:
        return "STATIC_OR_EGO_EXPLAINED";
    }
  }

  bool egoTranslationObservableUnlocked() const {
    if (nodes_.empty()) {
      return false;
    }
    const double mean_speed = meanEgoSpeedUnlocked();
    const double latest_speed = std::fabs(nodes_.back().ego.v);
    return std::max(mean_speed, latest_speed) >= cfg_.ego_translation_observable_speed_gate;
  }

  bool egoTranslationSourceKnownUnlocked() const {
    if (nodes_.empty()) {
      return false;
    }
    return nodes_.back().ego.speed_source_valid;
  }

  bool relativeShorteningOnlyUnlocked(const LeadNode& node, const LeadNode* prev) const {
    if (!cfg_.relative_shortening_freeze_confirmed || prev == NULL ||
        egoTranslationSourceKnownUnlocked()) {
      return false;
    }
    const double static_or_ego_res = std::min(node.r_static, node.r_ego);
    const bool static_or_ego_explains =
        static_or_ego_res <= cfg_.ego_yaw_mixing_residual_gate ||
        (node.r_static <= cfg_.ego_yaw_mixing_anchor_gate &&
         node.r_ego <= 1.5 * cfg_.ego_yaw_mixing_residual_gate);
    const bool dynamic_residual_wins =
        node.r_dynamic + cfg_.dynamic_residual_margin < static_or_ego_res;
    const double range_delta = node.range - prev->range;
    const double x_delta = node.z_rel_x - prev->z_rel_x;
    const bool relative_distance_shrinks =
        range_delta < -std::max(0.02, 0.5 * cfg_.tail_confirm_min_distance) ||
        x_delta < -std::max(0.02, 0.5 * cfg_.tail_confirm_min_distance) ||
        node.reverse_progress < -0.5 * cfg_.reverse_progress_gate;
    return relative_distance_shrinks && static_or_ego_explains && !dynamic_residual_wins;
  }

  double meanImuYawActivityUnlocked() const {
    if (nodes_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    double weight_sum = 0.0;
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const double w = std::max(1e-3, nodes_[i].robust_weight);
      sum += w * nodes_[i].imu_yaw_activity;
      weight_sum += w;
    }
    return sum / std::max(1e-3, weight_sum);
  }

  int consecutiveDynamicNodesUnlocked() const {
    int count = 0;
    for (size_t i = nodes_.size(); i > 0; --i) {
      const LeadNode& node = nodes_[i - 1];
      if ((node.motion_class == MOTION_DYNAMIC_CANDIDATE ||
           node.motion_class == MOTION_CURVED_DYNAMIC_CANDIDATE ||
           node.motion_class == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP) &&
          !node.ego_yaw_mixing) {
        ++count;
      } else if (node.motion_class == MOTION_OUTLIER) {
        continue;
      } else {
        break;
      }
    }
    return count;
  }

  double recentCurveArcProgressUnlocked(const LeadNode* pending = NULL) const {
    std::vector<const LeadNode*> seq;
    const size_t keep = std::min<size_t>(nodes_.size(), 6);
    seq.reserve(keep + (pending != NULL ? 1 : 0));
    for (size_t i = nodes_.size() - keep; i < nodes_.size(); ++i) {
      if (!nodes_[i].ego_yaw_mixing) {
        seq.push_back(&nodes_[i]);
      }
    }
    if (pending != NULL && !pending->ego_yaw_mixing) {
      seq.push_back(pending);
    }
    if (seq.size() < 2) {
      return 0.0;
    }
    double arc = 0.0;
    for (size_t i = 1; i < seq.size(); ++i) {
      arc += pointDistance(seq[i - 1]->x, seq[i - 1]->y, seq[i]->x, seq[i]->y);
    }
    return arc;
  }

  double recentCurveHeadingConsistencyUnlocked(const LeadNode* pending = NULL) const {
    std::vector<const LeadNode*> seq;
    const size_t keep = std::min<size_t>(nodes_.size(), 6);
    seq.reserve(keep + (pending != NULL ? 1 : 0));
    for (size_t i = nodes_.size() - keep; i < nodes_.size(); ++i) {
      if (!nodes_[i].ego_yaw_mixing) {
        seq.push_back(&nodes_[i]);
      }
    }
    if (pending != NULL && !pending->ego_yaw_mixing) {
      seq.push_back(pending);
    }
    if (seq.size() < 3) {
      return 0.0;
    }
    int checked = 0;
    int consistent = 0;
    double prev_turn = 0.0;
    for (size_t i = 2; i < seq.size(); ++i) {
      const double h0 = std::atan2(seq[i - 1]->y - seq[i - 2]->y,
                                   seq[i - 1]->x - seq[i - 2]->x);
      const double h1 = std::atan2(seq[i]->y - seq[i - 1]->y,
                                   seq[i]->x - seq[i - 1]->x);
      const double turn = wrapAngle(h1 - h0);
      if (std::fabs(turn) < 0.02) {
        ++consistent;
      } else if (prev_turn == 0.0 || turn * prev_turn >= -1e-4) {
        ++consistent;
        prev_turn = turn;
      }
      ++checked;
    }
    return checked > 0 ? static_cast<double>(consistent) / static_cast<double>(checked) : 0.0;
  }

  bool curvedDynamicCandidateUnlocked(const LeadNode& node, const LeadNode* prev) const {
    if (prev == NULL || node.ego_yaw_mixing ||
        node.measurement_confidence < cfg_.endpoint_recapture_confidence_gate) {
      return false;
    }

    const double dx = node.x - prev->x;
    const double dy = node.y - prev->y;
    const double step = hypot2(dx, dy);
    if (step < std::max(0.02, 0.5 * cfg_.tail_confirm_min_distance)) {
      return false;
    }
    const double along = dx * std::cos(prev->yaw) + dy * std::sin(prev->yaw);
    const double cross = -dx * std::sin(prev->yaw) + dy * std::cos(prev->yaw);
    if (along < -std::max(cfg_.reverse_progress_gate, 0.02)) {
      return false;
    }
    if (std::fabs(cross) > cfg_.dynamic_curve_cross_gate) {
      return false;
    }

    const double step_heading = std::atan2(dy, dx);
    const double step_heading_err = std::fabs(wrapAngle(step_heading - prev->yaw));
    const double obs_heading_err = std::fabs(wrapAngle(node.yaw - prev->yaw));
    const double arc = recentCurveArcProgressUnlocked(&node);
    const double consistency = recentCurveHeadingConsistencyUnlocked(&node);
    const bool heading_supported =
        step_heading_err <= cfg_.dynamic_curve_heading_gate ||
        obs_heading_err <= 1.5 * cfg_.dynamic_curve_heading_gate ||
        consistency >= 0.50;
    const bool arc_supported =
        arc >= cfg_.dynamic_curve_min_arc ||
        (nodes_.size() >= 3 && step >= std::max(0.04, 0.5 * cfg_.dynamic_curve_min_arc));
    const double static_or_ego_res = std::min(node.r_static, node.r_ego);
    const bool dynamic_residual_wins =
        node.r_dynamic + cfg_.dynamic_residual_margin < static_or_ego_res;
    const bool dynamic_not_bad =
        node.r_dynamic <= static_or_ego_res + std::max(cfg_.dynamic_residual_margin,
                                                       cfg_.ego_compensated_moving_gate);
    const bool ego_yaw_active =
        node.imu_yaw_activity >= cfg_.imu_yaw_activity_gate ||
        std::fabs(node.ego.yaw_rate) >= cfg_.ego_yaw_mixing_yaw_rate_gate;
    if (!egoTranslationObservableUnlocked() && !dynamic_residual_wins) {
      return false;
    }
    return heading_supported && arc_supported && dynamic_not_bad;
  }

  bool leadOnlyBootstrapCandidateUnlocked(const LeadNode& node, const LeadNode* prev) const {
    if (!cfg_.lead_only_bootstrap_enabled ||
        !confirmed_trail_world_.empty() ||
        prev == NULL ||
        egoTranslationObservableUnlocked() ||
        node.ego_yaw_mixing ||
        node.measurement_confidence < std::max(0.55, cfg_.endpoint_recapture_confidence_gate - 0.15)) {
      return false;
    }

    const double imu_activity =
        std::max(node.imu_yaw_activity, std::fabs(node.ego.yaw_rate));
    if (imu_activity > cfg_.lead_only_bootstrap_max_imu_activity) {
      return false;
    }

    const double dx = node.x - prev->x;
    const double dy = node.y - prev->y;
    const double along = dx * std::cos(prev->yaw) + dy * std::sin(prev->yaw);
    const double cross = -dx * std::sin(prev->yaw) + dy * std::cos(prev->yaw);
    const double step = hypot2(dx, dy);
    const double range_delta = node.range - prev->range;
    const double x_delta = node.z_rel_x - prev->z_rel_x;

    const bool forward_progress =
        along >= std::max(0.02, 0.5 * cfg_.tail_confirm_min_distance) ||
        range_delta >= std::max(0.025, 0.5 * cfg_.tail_confirm_min_distance) ||
        x_delta >= std::max(0.025, 0.5 * cfg_.tail_confirm_min_distance);
    if (!forward_progress || step < 0.5 * cfg_.tail_confirm_min_distance) {
      return false;
    }
    if (std::fabs(cross) > std::max(0.45, 2.0 * cfg_.tail_cross_gate)) {
      return false;
    }

    const double static_or_ego_res = std::min(node.r_static, node.r_ego);
    const bool dynamic_residual_wins =
        node.r_dynamic + cfg_.dynamic_residual_margin < static_or_ego_res;
    const bool static_not_tight =
        static_or_ego_res >= cfg_.lead_only_bootstrap_static_residual_gate ||
        node.static_hypothesis_residual >= cfg_.lead_only_bootstrap_static_residual_gate;
    const bool motion_magnitude_support =
        meanFixedFrameSpeedUnlocked() >= cfg_.lead_only_bootstrap_min_ff_speed ||
        estimateWindowProgressUnlocked() >= 0.5 * cfg_.lead_only_bootstrap_min_progress;
    const bool strong_step_support =
        step >= std::max(0.04, cfg_.tail_confirm_min_distance) &&
        (range_delta >= 0.5 * cfg_.tail_confirm_min_distance ||
         x_delta >= 0.5 * cfg_.tail_confirm_min_distance ||
         along >= cfg_.tail_confirm_min_distance);

    return dynamic_residual_wins ||
           (static_not_tight && motion_magnitude_support) ||
           strong_step_support;
  }

  MotionClass classifyMotionUnlocked(const LeadNode& node, const LeadNode* prev) const {
    if (prev == NULL) {
      return MOTION_STATIC_OR_EGO_EXPLAINED;
    }

    const double static_or_ego_res = std::min(node.r_static, node.r_ego);
    const bool imu_active =
        node.imu_yaw_activity >= cfg_.imu_yaw_activity_gate ||
        std::fabs(node.ego.yaw_rate) >= cfg_.ego_yaw_mixing_yaw_rate_gate;
    const bool imu_quiet = !imu_active;
    const bool dynamic_residual_wins =
        node.r_dynamic + cfg_.dynamic_residual_margin < static_or_ego_res;
    const bool dynamic_score_wins =
        dynamicHypothesisScoreUnlocked() + cfg_.dynamic_bootstrap_score_margin <
        staticHypothesisScoreUnlocked();

    const double dx = node.x - prev->x;
    const double dy = node.y - prev->y;
    const double along = dx * std::cos(prev->yaw) + dy * std::sin(prev->yaw);
    const double cross = -dx * std::sin(prev->yaw) + dy * std::cos(prev->yaw);
    const double step = hypot2(dx, dy);
    const bool positive_anchor_progress =
        along >= std::max(0.5 * cfg_.tail_confirm_min_distance, 0.02) &&
        std::fabs(cross) <= std::max(2.0 * cfg_.tail_cross_gate, 0.40);
    const bool static_or_ego_explains =
        static_or_ego_res <= cfg_.ego_yaw_mixing_residual_gate ||
        (node.r_static <= cfg_.ego_yaw_mixing_anchor_gate &&
         node.r_ego <= 1.5 * cfg_.ego_yaw_mixing_residual_gate);
    const bool speed_support =
        (!cfg_.disable_filter_speed_dynamic_when_ego_unobservable ||
         egoTranslationObservableUnlocked()) &&
        node.fixed_frame_speed >= cfg_.dynamic_bootstrap_min_ff_speed &&
        node.measurement_confidence > 0.70 &&
        !static_or_ego_explains;
    const bool motion_support = positive_anchor_progress || speed_support;
    const bool curve_support = curvedDynamicCandidateUnlocked(node, prev);

    if (relativeShorteningOnlyUnlocked(node, prev)) {
      return MOTION_RELATIVE_SHORTENING_ONLY;
    }
    if (curve_support) {
      return MOTION_CURVED_DYNAMIC_CANDIDATE;
    }
    if (leadOnlyBootstrapCandidateUnlocked(node, prev)) {
      return MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP;
    }
    if (imu_active && static_or_ego_explains && !dynamic_residual_wins) {
      return MOTION_YAW_MIXING;
    }
    if (dynamic_residual_wins && motion_support) {
      return MOTION_DYNAMIC_CANDIDATE;
    }
    if (imu_quiet && positive_anchor_progress && dynamic_score_wins &&
        (egoTranslationObservableUnlocked() || dynamic_residual_wins)) {
      return MOTION_DYNAMIC_CANDIDATE;
    }
    if (imu_quiet && !static_or_ego_explains && positive_anchor_progress &&
        step >= cfg_.dynamic_bootstrap_min_span &&
        (egoTranslationObservableUnlocked() || dynamic_residual_wins)) {
      return MOTION_DYNAMIC_CANDIDATE;
    }
    return MOTION_STATIC_OR_EGO_EXPLAINED;
  }

  bool dynamicHypothesisWinsUnlocked() const {
    return dynamicHypothesisScoreUnlocked() + cfg_.dual_hypothesis_score_margin <
           staticHypothesisScoreUnlocked();
  }

  bool confirmDynamicHypothesisWinsUnlocked() const {
    const double hs = staticHypothesisScoreUnlocked();
    const double hd = dynamicHypothesisScoreUnlocked();
    const bool score_ok = hd + cfg_.confirm_dynamic_score_margin < hs;
    const bool progress_ok =
        std::max(dynamicTrailProgressUnlocked(), estimateWindowProgressUnlocked()) >= cfg_.confirm_min_progress;
    return score_ok && progress_ok;
  }

  bool egoMotionConditionedDynamicUnlocked() const {
    const double hs = staticHypothesisScoreUnlocked();
    const double hd = dynamicHypothesisScoreUnlocked();
    const double progress = std::max(dynamicTrailProgressUnlocked(), estimateWindowProgressUnlocked());
    const double ff_speed = meanFixedFrameSpeedUnlocked();
    const bool score_wins = hd + 0.5 * cfg_.confirm_dynamic_score_margin < hs;
    const bool accum_support =
        unconfirmed_progress_accum_ >= 0.5 * cfg_.confirm_accum_distance ||
        (unconfirmed_time_accum_ >= 0.4 * cfg_.confirm_accum_timeout &&
         unconfirmed_progress_accum_ >= 2.0 * cfg_.measurement_cross_sigma);
    const bool progress_support =
        progress >= std::max(0.02, 0.5 * cfg_.confirm_min_progress) ||
        ff_speed >= 0.5 * cfg_.lead_moving_speed_gate;
    return score_wins && (accum_support || progress_support);
  }

  bool dynamicCandidateEvidenceUnlocked() const {
    if (nodes_.size() < 3 || last_ego_yaw_mixing_) {
      return false;
    }

    const double progress = std::max(dynamicTrailProgressUnlocked(), estimateWindowProgressUnlocked());
    const double ff_speed = meanFixedFrameSpeedUnlocked();
    const double r_static = meanStaticHypothesisResidualUnlocked();
    const double r_ego = meanEgoMotionExplainedResidualUnlocked();
    const double r_dynamic = meanDynamicHypothesisResidualUnlocked();
    const double static_or_ego_res = std::min(r_static, r_ego);
    const double imu_activity = meanImuYawActivityUnlocked();
    double window_span = 0.0;
    if (nodes_.size() >= 2) {
      window_span = pointDistance(nodes_.front().x, nodes_.front().y,
                                  nodes_.back().x, nodes_.back().y);
    }
    const int dynamic_count = consecutiveDynamicNodesUnlocked();
    const bool latest_curve =
        !nodes_.empty() &&
        nodes_.back().motion_class == MOTION_CURVED_DYNAMIC_CANDIDATE &&
        recentCurveArcProgressUnlocked() >= cfg_.dynamic_curve_min_arc &&
        recentCurveHeadingConsistencyUnlocked() >= 0.45;
    const bool latest_lead_only =
        !nodes_.empty() &&
        confirmed_trail_world_.empty() &&
        nodes_.back().motion_class == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP;

    const bool residual_support =
        r_dynamic + cfg_.dynamic_residual_margin < static_or_ego_res;
    const bool motion_support =
        progress >= std::max(0.5 * cfg_.confirm_min_progress, cfg_.dynamic_bootstrap_min_span) ||
        window_span >= cfg_.dynamic_bootstrap_min_span;
    const bool speed_support =
        (!cfg_.disable_filter_speed_dynamic_when_ego_unobservable ||
         egoTranslationObservableUnlocked()) &&
        ff_speed >= cfg_.dynamic_bootstrap_min_ff_speed &&
        residual_support;
    const bool consecutive_support =
        dynamic_count >= std::max(1, cfg_.bootstrap_consecutive_dynamic);
    const bool imu_quiet = imu_activity < cfg_.imu_yaw_activity_gate;
    const bool lead_only_bootstrap_support =
        latest_lead_only &&
        imu_activity <= cfg_.lead_only_bootstrap_max_imu_activity &&
        dynamic_count >= std::max(1, cfg_.lead_only_bootstrap_min_points) &&
        (progress >= cfg_.lead_only_bootstrap_min_progress ||
         window_span >= cfg_.lead_only_bootstrap_min_span) &&
        (ff_speed >= cfg_.lead_only_bootstrap_min_ff_speed ||
         progress >= cfg_.lead_only_bootstrap_min_progress);

    const bool observable_or_residual_strong =
        egoTranslationObservableUnlocked() || residual_support;

    return (latest_curve && residual_support) ||
           lead_only_bootstrap_support ||
           (consecutive_support &&
           ((residual_support && (motion_support || speed_support)) ||
            (imu_quiet && motion_support && observable_or_residual_strong)));
  }

  bool staticModeDynamicProbeUnlocked() const {
    if (!track_.static_mode || nodes_.size() < 3 || last_ego_yaw_mixing_) {
      return false;
    }

    const double progress = std::max(dynamicTrailProgressUnlocked(), estimateWindowProgressUnlocked());
    const double ff_speed = meanFixedFrameSpeedUnlocked();
    const double hs = staticHypothesisScoreUnlocked();
    const double hd = dynamicHypothesisScoreUnlocked();
    const double r_static = meanStaticHypothesisResidualUnlocked();
    const double r_ego = meanEgoMotionExplainedResidualUnlocked();
    const double r_dynamic = meanDynamicHypothesisResidualUnlocked();

    const bool progress_support =
        progress >= std::max(0.02, cfg_.static_dynamic_probe_min_progress_ratio * cfg_.confirm_min_progress) ||
        ff_speed >= std::max(0.03, cfg_.static_dynamic_probe_speed_ratio * cfg_.lead_moving_speed_gate);
    const bool dynamic_score_support =
        hd + cfg_.static_dynamic_probe_residual_margin < hs ||
        r_dynamic + cfg_.static_dynamic_probe_residual_margin < std::min(r_static, r_ego);
    const bool static_anchor_not_stable =
        r_static > cfg_.static_anchor_res_gate ||
        progress > cfg_.static_anchor_progress_gate ||
        ff_speed > cfg_.static_anchor_speed_gate;

    return progress_support && (dynamic_score_support || static_anchor_not_stable);
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
    const bool enable_anchor =
        track_.static_mode || static_votes_ >= cfg_.static_vote_on;
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
      node->short_window_prediction_residual = 0.0;
      node->ego_motion_explained_residual = 0.0;
      node->r_static = 0.0;
      node->r_ego = 0.0;
      node->r_dynamic = 0.0;
      node->r_heading_static = 0.0;
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
    node->ego_motion_explained_residual = prev_residual;

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

    double anchor_residual = std::min(prev_residual, trail_tail_residual);
    double anchor_heading = prev.yaw;
    if (cfg_.export_static_anchor_to_history && has_static_anchor_) {
      anchor_residual = pointDistance(node->x, node->y, static_anchor_x_, static_anchor_y_);
      anchor_heading = static_anchor_heading_;
    }

    node->r_static = anchor_residual;
    node->r_ego = std::min(prev_residual, trail_tail_residual);
    node->r_dynamic = window_pred_residual;
    node->r_heading_static = std::fabs(wrapAngle(node->yaw - anchor_heading));
    node->static_hypothesis_residual = std::min(node->r_static, node->r_ego);
    node->short_window_prediction_residual = window_pred_residual;
  }

  bool egoYawMixingLikelyUnlocked(const LeadNode& node,
                                  const LeadNode& prev,
                                  double* mix_score) const {
    const double r_ego = node.r_ego;
    const double r_dyn = node.r_dynamic;
    const double r_static = node.r_static;
    const double heading_static = node.r_heading_static;
    const double yaw_activity =
        std::max(node.imu_yaw_activity,
                 std::fabs(node.ego.yaw_rate) * std::max(0.5, node.range));
    const bool ego_active =
        node.imu_yaw_activity > cfg_.imu_yaw_activity_gate ||
        std::fabs(node.ego.yaw_rate) > cfg_.ego_yaw_mixing_yaw_rate_gate ||
        yaw_activity > cfg_.ego_yaw_mixing_lateral_gate;
    const bool static_explains =
        r_ego < cfg_.ego_yaw_mixing_residual_gate &&
        r_static < cfg_.ego_yaw_mixing_anchor_gate;
    const bool dynamic_not_stronger =
        r_dyn > cfg_.ego_yaw_mixing_dynamic_ratio * std::max(1e-3, r_ego);
    const bool heading_not_changed_in_anchor =
        heading_static < cfg_.ego_yaw_mixing_heading_gate ||
        std::fabs(wrapAngle(node.yaw - prev.yaw)) < cfg_.ego_yaw_mixing_heading_gate;
    const bool mixed = ego_active && static_explains && dynamic_not_stronger && heading_not_changed_in_anchor;

    if (mix_score != NULL) {
      *mix_score =
          0.40 * clampValue(r_ego / std::max(1e-3, cfg_.ego_yaw_mixing_residual_gate), 0.0, 2.0) +
          0.30 * clampValue(r_static / std::max(1e-3, cfg_.ego_yaw_mixing_anchor_gate), 0.0, 2.0) +
          0.30 * clampValue(heading_static / std::max(1e-3, cfg_.ego_yaw_mixing_heading_gate), 0.0, 2.0);
    }
    return mixed;
  }

  void pullWindowTowardStaticHypothesisUnlocked() {
    if (nodes_.size() < 2) {
      return;
    }
    if (!track_.static_mode && static_votes_ < cfg_.static_vote_on) {
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
      if (nodes_[i].ego_yaw_mixing) {
        continue;
      }
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
    track_ = LeadState2D();
    moving_flag_ = false;
    static_votes_ = 0;
    moving_votes_ = 0;
    resetUnconfirmedAccumulatorUnlocked();
    has_static_anchor_ = false;
    static_anchor_stamp_ = ros::Time();
    static_mode_start_stamp_ = ros::Time();
    static_anchor_heading_ = 0.0;
    last_ego_yaw_mixing_ = false;
    fusion_ = LeadFusionState();
    fusion_recent_points_.clear();
    fusion_confirm_accum_s_ = 0.0;
    fusion_confirm_accum_t_ = 0.0;
    fusion_static_hold_count_ = 0;
    fusion_ego_explained_hold_count_ = 0;
    fusion_yaw_hold_count_ = 0;
    consecutive_geometry_bad_frames_ = 0;
    history_tail_only_count_ = 0;
    reconstruction_state_ = "TRACK_GOOD";
  }

  void pruneWindowUnlocked(const ros::Time& now) {
    while (!nodes_.empty() && (now - nodes_.front().stamp).toSec() > cfg_.history_max_age) {
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

    for (int iter = 0; iter < std::max(1, cfg_.smoother_iterations); ++iter) {
      for (size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].ego_yaw_mixing) {
          continue;
        }
        double pred_local_x = 0.0;
        double pred_local_y = 0.0;
        transformWorldToLocal(nodes_[i].ego, nodes_[i].x, nodes_[i].y,
                              &pred_local_x, &pred_local_y);
        const double rx = nodes_[i].z_rel_x - pred_local_x;
        const double ry = nodes_[i].z_rel_y - pred_local_y;
        const double metric =
            std::sqrt((rx * rx) / std::max(1e-6, nodes_[i].w_along * nodes_[i].w_along) +
                      (ry * ry) / std::max(1e-6, nodes_[i].w_cross * nodes_[i].w_cross));
        const double w =
            clampValue(nodes_[i].measurement_confidence, 0.05, 1.0) * robustWeight(metric);
        nodes_[i].residual_along = rx;
        nodes_[i].residual_cross = ry;
        nodes_[i].robust_weight = w;
        const double corr_local_x = cfg_.smoother_position_gain * w * rx;
        const double corr_local_y = cfg_.smoother_cross_gain * w * ry;
        const double c = std::cos(nodes_[i].ego.yaw);
        const double s = std::sin(nodes_[i].ego.yaw);
        nodes_[i].x += c * corr_local_x - s * corr_local_y;
        nodes_[i].y += s * corr_local_x + c * corr_local_y;
        const double dh = wrapAngle(nodes_[i].heading_hint - nodes_[i].yaw);
        nodes_[i].yaw = wrapAngle(nodes_[i].yaw + cfg_.smoother_heading_alpha * w * dh);
      }

      strengthenMeasurementModelUnlocked();
      pullWindowTowardStaticHypothesisUnlocked();

      if (nodes_.size() >= 3) {
        std::deque<LeadNode> next = nodes_;
        for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
          const double mx = 0.5 * (nodes_[i - 1].x + nodes_[i + 1].x);
          const double my = 0.5 * (nodes_[i - 1].y + nodes_[i + 1].y);
          next[i].x =
              (1.0 - cfg_.smoother_temporal_alpha) * nodes_[i].x + cfg_.smoother_temporal_alpha * mx;
          next[i].y =
              (1.0 - cfg_.smoother_temporal_alpha) * nodes_[i].y + cfg_.smoother_temporal_alpha * my;
        }
        nodes_.swap(next);
      }

      recomputeKnotGeometryUnlocked();
      regularizeDynamicStatesUnlocked();
      applyDynamicsConsistencyUnlocked();
      applyStaticAnchorFactorUnlocked();
      enforceProgressionUnlocked();
      smoothCurvatureUnlocked();
      recomputeKnotGeometryUnlocked();
      regularizeDynamicStatesUnlocked();
    }
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
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const size_t i0 = (i == 0) ? i : i - 1;
      const size_t i1 = (i + 1 >= nodes_.size()) ? i : i + 1;
      const double dx = nodes_[i1].x - nodes_[i0].x;
      const double dy = nodes_[i1].y - nodes_[i0].y;
      if (hypot2(dx, dy) > 1e-4) {
        nodes_[i].yaw = std::atan2(dy, dx);
      }
      const double dt =
          std::max(1e-3, (nodes_[i1].stamp - nodes_[i0].stamp).toSec());
      const double v_new = hypot2(dx, dy) / dt;
      nodes_[i].a = (v_new - nodes_[i].v) / dt;
      nodes_[i].v = v_new;
    }
    nodes_.front().yaw = nodes_[std::min<size_t>(1, nodes_.size() - 1)].yaw;
    nodes_.back().yaw = nodes_[nodes_.size() - 2].yaw;

    for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
      const double ds = std::max(1e-3,
                                 pointDistance(nodes_[i - 1].x, nodes_[i - 1].y,
                                               nodes_[i + 1].x, nodes_[i + 1].y));
      const double dpsi = wrapAngle(nodes_[i + 1].yaw - nodes_[i - 1].yaw);
      nodes_[i].kappa = clampValue(dpsi / ds, -0.8, 0.8);
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
    pt.motion_class = node.motion_class;
    pt.dynamic_residual_win = node.dynamic_residual_win;
    pt.ego_translation_observable = node.ego_translation_observable;
    pt.relative_shortening_only = node.relative_shortening_only;
    pt.endpoint_quality_ok = node.endpoint_quality_ok;
    pt.segment_id = confirmed_segment_id_;
    pt.anchor_epoch_id = current_anchor_epoch_id_;
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
      const bool bootstrap_dynamic =
          confirmed_trail_world_.empty() && dynamicCandidateEvidenceUnlocked();
      const bool curve_dynamic =
          !nodes_.empty() &&
          nodes_.back().motion_class == MOTION_CURVED_DYNAMIC_CANDIDATE;
      const double max_speed =
          (bootstrap_dynamic || curve_dynamic)
              ? std::max(cfg_.lead_tail_max_speed, 2.5 * cfg_.history_max_confirm_speed)
              : cfg_.lead_tail_max_speed;
      if (step / dt > max_speed) {
        return true;
      }

      const double tx = std::cos(prev.heading);
      const double ty = std::sin(prev.heading);
      const double along = dx * tx + dy * ty;
      const double cross = -dx * ty + dy * tx;
      const double cross_gate =
          curve_dynamic ? std::max(cfg_.dynamic_curve_cross_gate, 2.0 * cfg_.tail_cross_gate)
                        : (bootstrap_dynamic ? std::max(cfg_.tail_cross_gate, 2.0 * cfg_.tail_cross_gate)
                                             : cfg_.tail_cross_gate);
      const double reverse_gate =
          (bootstrap_dynamic || curve_dynamic) ? std::min(cfg_.tail_reverse_gate, -0.15)
                                               : cfg_.tail_reverse_gate;
      if (cross > cross_gate || cross < -cross_gate) {
        return true;
      }
      if (along < reverse_gate) {
        return true;
      }
    }

    if (cfg_.export_static_anchor_to_history && has_static_anchor_) {
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

  void refreshTailBufferUnlocked(bool allow_static_tail) {
    tail_buffer_world_.clear();
    last_tail_admit_reason_ = "tail_empty";
    if (nodes_.empty()) {
      return;
    }
    if (last_ego_yaw_mixing_) {
      last_history_action_ = "tail_cleared_by_ego_yaw_mixing";
      last_tail_admit_reason_ = "ego_yaw_mixing";
      return;
    }
    if (track_.static_mode && !allow_static_tail) {
      last_history_action_ = "tail_cleared_by_static_lock";
      last_tail_admit_reason_ = "static_lock";
      return;
    }

    const ros::Time newest = nodes_.back().stamp;
    const size_t max_points =
        static_cast<size_t>(std::max(1, (track_.static_mode && !allow_static_tail)
                                            ? cfg_.static_tail_keep_points
                                            : cfg_.tail_buffer_max_points));
    for (size_t i = 0; i < nodes_.size(); ++i) {
      if (newest.isValid() && nodes_[i].stamp.isValid() &&
          (newest - nodes_[i].stamp).toSec() > cfg_.tail_buffer_max_age) {
        continue;
      }
      if (cfg_.confirmed_require_dynamic_motion_class) {
        const MotionClass mc = static_cast<MotionClass>(nodes_[i].motion_class);
        const bool lead_only_bootstrap =
            mc == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP &&
            confirmed_trail_world_.empty();
        if (mc != MOTION_DYNAMIC_CANDIDATE &&
            mc != MOTION_CURVED_DYNAMIC_CANDIDATE &&
            !lead_only_bootstrap) {
          last_tail_admit_reason_ = "motion_class_not_dynamic";
          continue;
        }
        if (mc == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP &&
            !confirmed_trail_world_.empty()) {
          last_tail_admit_reason_ = "ego_unobservable_append_blocked";
          continue;
        }
        if (nodes_[i].relative_shortening_only || nodes_[i].ego_yaw_mixing) {
          last_tail_admit_reason_ = nodes_[i].relative_shortening_only
                                        ? "relative_shortening_blocked"
                                        : "ego_yaw_mixing_node";
          continue;
        }
        if (!nodes_[i].endpoint_quality_ok) {
          last_tail_admit_reason_ = "endpoint_quality_low";
          continue;
        }
        if (!lead_only_bootstrap &&
            !nodes_[i].ego_translation_observable && !nodes_[i].dynamic_residual_win) {
          last_tail_admit_reason_ = "ego_unobservable_without_dynamic_residual";
          continue;
        }
      }
      HistoryPoint2D pt = nodeToHistoryPointUnlocked(nodes_[i]);
      if (fastRejectTailPointUnlocked(pt)) {
        last_history_action_ = "tail_fast_rejected";
        last_tail_admit_reason_ = "tail_fast_rejected";
        continue;
      }
      pt.predicted = true;
      tail_buffer_world_.push_back(pt);
      last_tail_admit_reason_ =
          static_cast<MotionClass>(nodes_[i].motion_class) == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP
              ? "lead_only_bootstrap_admitted"
              : "tail_admitted";
      while (tail_buffer_world_.size() > max_points) {
        tail_buffer_world_.pop_front();
      }
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

  int pruneTrailContainerUnlocked(std::deque<HistoryPoint2D>* trail, const ros::Time& now) {
    if (trail == NULL) {
      return 0;
    }
    int pruned = 0;
    const size_t min_keep =
        static_cast<size_t>(std::max(2, cfg_.confirmed_history_min_points_keep));
    while (trail->size() > min_keep &&
           !trail->empty() &&
           (now - trail->front().stamp).toSec() > cfg_.confirmed_history_max_age) {
      trail->pop_front();
      ++pruned;
    }
    while (trail->size() > static_cast<size_t>(std::max(2, cfg_.history_max_points))) {
      trail->pop_front();
      ++pruned;
    }
    return pruned;
  }

  double tailBufferSpanUnlocked() const {
    if (tail_buffer_world_.size() < 2) {
      return 0.0;
    }
    double span = 0.0;
    for (size_t i = 1; i < tail_buffer_world_.size(); ++i) {
      span += pointDistance(tail_buffer_world_[i - 1].x,
                            tail_buffer_world_[i - 1].y,
                            tail_buffer_world_[i].x,
                            tail_buffer_world_[i].y);
    }
    return span;
  }

  void regularizeConfirmedTailHeadingUnlocked() {
    if (confirmed_trail_world_.size() < 2) {
      return;
    }
    const size_t n = confirmed_trail_world_.size();
    const size_t begin = n > 8 ? n - 8 : 0;
    const double alpha = clampValue(std::max(0.25, cfg_.smoother_heading_alpha), 0.0, 0.65);
    for (size_t i = begin; i + 1 < n; ++i) {
      const HistoryPoint2D& cur = confirmed_trail_world_[i];
      const HistoryPoint2D& nxt = confirmed_trail_world_[i + 1];
      const double d = pointDistance(cur.x, cur.y, nxt.x, nxt.y);
      if (d < 1e-4) {
        continue;
      }
      double geom = std::atan2(nxt.y - cur.y, nxt.x - cur.x);
      if (i > 0) {
        geom = alignAngleToReference(geom, confirmed_trail_world_[i - 1].heading);
      } else {
        geom = alignAngleToReference(geom, confirmed_trail_world_[i].heading);
      }
      confirmed_trail_world_[i].heading =
          wrapAngle(confirmed_trail_world_[i].heading +
                    alpha * wrapAngle(geom - confirmed_trail_world_[i].heading));
    }
    confirmed_trail_world_.back().heading =
        alignAngleToReference(confirmed_trail_world_[n - 2].heading,
                              confirmed_trail_world_.back().heading);
  }

  bool bootstrapConfirmedTrailFromTailUnlocked(const ros::Time& tail_stamp,
                                               double min_confirm_age) {
    if (!confirmed_trail_world_.empty() ||
        tail_buffer_world_.size() <
            static_cast<size_t>(std::max(2, cfg_.dynamic_bootstrap_min_tail_points))) {
      return false;
    }
    if (!dynamicCandidateEvidenceUnlocked()) {
      return false;
    }
    if (tailBufferSpanUnlocked() < cfg_.dynamic_bootstrap_min_span) {
      return false;
    }

    std::deque<HistoryPoint2D> bootstrap;
    double bootstrap_span = 0.0;
    for (size_t i = 0; i < tail_buffer_world_.size(); ++i) {
      const HistoryPoint2D& src = tail_buffer_world_[i];
      const bool aged = !tail_stamp.isValid() || !src.stamp.isValid() ||
                        (tail_stamp - src.stamp).toSec() >= min_confirm_age;
      if (!aged && bootstrap.size() >= 2) {
        break;
      }
      if (!bootstrap.empty()) {
        const HistoryPoint2D& prev = bootstrap.back();
        if (src.stamp.isValid() && prev.stamp.isValid() && src.stamp <= prev.stamp) {
          continue;
        }
        const double d = pointDistance(prev.x, prev.y, src.x, src.y);
        if (d < 0.5 * cfg_.tail_confirm_min_distance) {
          continue;
        }
        bootstrap_span += d;
      }
      HistoryPoint2D pt = src;
      pt.predicted = false;
      bootstrap.push_back(pt);
      if (bootstrap.size() >= static_cast<size_t>(std::max(2, cfg_.dynamic_bootstrap_min_tail_points)) &&
          bootstrap_span >= cfg_.dynamic_bootstrap_min_span) {
        break;
      }
    }

    if (bootstrap.size() < 2 || bootstrap_span < cfg_.dynamic_bootstrap_min_span) {
      return false;
    }

    int positive_segments = 0;
    int checked_segments = 0;
    for (size_t i = 1; i < bootstrap.size(); ++i) {
      const HistoryPoint2D& prev = bootstrap[i - 1];
      const HistoryPoint2D& cur = bootstrap[i];
      const double dx = cur.x - prev.x;
      const double dy = cur.y - prev.y;
      const double along = dx * std::cos(prev.heading) + dy * std::sin(prev.heading);
      const double cross = -dx * std::sin(prev.heading) + dy * std::cos(prev.heading);
      if (along >= -cfg_.reverse_progress_gate &&
          std::fabs(cross) <= std::max(0.40, 2.0 * cfg_.tail_cross_gate)) {
        ++positive_segments;
      }
      ++checked_segments;
    }
    if (checked_segments > 0 && positive_segments < checked_segments) {
      last_history_action_ = "history_bootstrap_reverse_rejected";
      return false;
    }

    for (size_t i = 0; i < bootstrap.size(); ++i) {
      HistoryPoint2D pt = bootstrap[i];
      pt.segment_id = confirmed_segment_id_;
      pt.anchor_epoch_id = current_anchor_epoch_id_;
      confirmed_trail_world_.push_back(pt);
    }
    regularizeConfirmedTailHeadingUnlocked();
    last_confirmed_prune_count_ +=
        pruneTrailContainerUnlocked(&confirmed_trail_world_, confirmed_trail_world_.back().stamp);
    resetUnconfirmedAccumulatorUnlocked();
    const MotionClass bootstrap_motion =
        static_cast<MotionClass>(bootstrap.back().motion_class);
    last_history_action_ =
        bootstrap_motion == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP
            ? "history_bootstrap_lead_only_tail"
            : "history_bootstrap_dynamic_tail";
    last_confirmed_append_allowed_ = true;
    last_confirmed_reject_reason_ =
        bootstrap_motion == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP
            ? "confirmed_bootstrap_lead_only_tail"
            : "confirmed_bootstrap_dynamic_tail";
    return true;
  }

  void appendConfirmedPointWithCurveInterpolationUnlocked(const HistoryPoint2D& candidate) {
    if (confirmed_trail_world_.empty()) {
      confirmed_trail_world_.push_back(candidate);
      return;
    }
    const HistoryPoint2D last = confirmed_trail_world_.back();
    const double dist = pointDistance(last.x, last.y, candidate.x, candidate.y);
    const double ds = std::max(0.02, cfg_.confirmed_curve_interp_ds);
    if (dist > 1.5 * ds) {
      const int segments = std::min(12, std::max(1, static_cast<int>(std::floor(dist / ds))));
      for (int i = 1; i < segments; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(segments);
        HistoryPoint2D mid;
        mid.stamp = candidate.stamp;
        mid.x = (1.0 - u) * last.x + u * candidate.x;
        mid.y = (1.0 - u) * last.y + u * candidate.y;
        mid.heading = wrapAngle(last.heading +
                                u * wrapAngle(candidate.heading - last.heading));
        mid.speed = (1.0 - u) * last.speed + u * candidate.speed;
        mid.curvature = (1.0 - u) * last.curvature + u * candidate.curvature;
        mid.predicted = false;
        mid.motion_class = candidate.motion_class;
        mid.dynamic_residual_win = candidate.dynamic_residual_win;
        mid.ego_translation_observable = candidate.ego_translation_observable;
        mid.relative_shortening_only = candidate.relative_shortening_only;
        mid.endpoint_quality_ok = candidate.endpoint_quality_ok;
        mid.segment_id = candidate.segment_id;
        mid.anchor_epoch_id = candidate.anchor_epoch_id;
        confirmed_trail_world_.push_back(mid);
      }
    }
    confirmed_trail_world_.push_back(candidate);
  }

  HistoryPoint2D fusionStateToHistoryPointUnlocked(const LeadNode& node) const {
    HistoryPoint2D pt;
    pt.stamp = fusion_.stamp.isValid() ? fusion_.stamp : node.stamp;
    pt.x = fusion_.x;
    pt.y = fusion_.y;
    pt.heading = fusion_.heading;
    pt.speed = hypot2(fusion_.vx, fusion_.vy);
    pt.curvature = node.kappa;
    pt.predicted = false;
    pt.motion_class = node.motion_class;
    pt.dynamic_residual_win = node.dynamic_residual_win;
    pt.ego_translation_observable = egoTranslationObservableUnlocked();
    pt.relative_shortening_only = node.relative_shortening_only;
    pt.endpoint_quality_ok = node.endpoint_quality_ok;
    pt.segment_id = confirmed_segment_id_;
    pt.anchor_epoch_id = current_anchor_epoch_id_;
    return pt;
  }

  double fusionRecentSpanUnlocked() const {
    if (fusion_recent_points_.size() < 2) {
      return 0.0;
    }
    double span = 0.0;
    for (size_t i = 1; i < fusion_recent_points_.size(); ++i) {
      span += pointDistance(fusion_recent_points_[i - 1].x,
                            fusion_recent_points_[i - 1].y,
                            fusion_recent_points_[i].x,
                            fusion_recent_points_[i].y);
    }
    return span;
  }

  void pushFusionRecentUnlocked(const HistoryPoint2D& pt) {
    if (!fusion_recent_points_.empty() &&
        pt.stamp.isValid() &&
        fusion_recent_points_.back().stamp.isValid() &&
        pt.stamp <= fusion_recent_points_.back().stamp) {
      fusion_recent_points_.back() = pt;
    } else if (fusion_recent_points_.empty() ||
               pointDistance(fusion_recent_points_.back().x,
                             fusion_recent_points_.back().y,
                             pt.x,
                             pt.y) >= 0.5 * cfg_.fusion_append_min_distance) {
      fusion_recent_points_.push_back(pt);
    }
    while (fusion_recent_points_.size() > 20) {
      fusion_recent_points_.pop_front();
    }
  }

  bool bootstrapFusionConfirmedUnlocked(const LeadNode& node) {
    if (!confirmed_trail_world_.empty()) {
      return false;
    }
    const double span = fusionRecentSpanUnlocked();
    if (fusion_recent_points_.size() <
            static_cast<size_t>(std::max(2, cfg_.fusion_bootstrap_min_points)) ||
        span < cfg_.fusion_bootstrap_min_span) {
      last_confirmed_reject_reason_ = "fusion_bootstrap_accumulating";
      return false;
    }

    confirmed_trail_world_.clear();
    for (size_t i = 0; i < fusion_recent_points_.size(); ++i) {
      HistoryPoint2D pt = fusion_recent_points_[i];
      pt.predicted = false;
      pt.segment_id = confirmed_segment_id_;
      pt.anchor_epoch_id = current_anchor_epoch_id_;
      confirmed_trail_world_.push_back(pt);
    }
    regularizeConfirmedTailHeadingUnlocked();
    last_confirmed_prune_count_ +=
        pruneTrailContainerUnlocked(&confirmed_trail_world_, node.stamp);
    fusion_confirm_accum_s_ = 0.0;
    fusion_confirm_accum_t_ = 0.0;
    last_confirmed_append_allowed_ = true;
    last_confirmed_reject_reason_ = "fusion_bootstrap_confirmed";
    last_history_action_ = "fusion_bootstrap_confirmed";
    return true;
  }

  struct HistoryGeometryGate {
    bool valid = true;
    bool reverse = false;
    bool loop = false;
    double kink_angle = 0.0;
    double project_progress = 0.0;
    std::string reason = "geometry_valid";
  };

  double confirmedPolylineLengthUnlocked() const {
    double s = 0.0;
    for (size_t i = 1; i < confirmed_trail_world_.size(); ++i) {
      s += pointDistance(confirmed_trail_world_[i - 1].x,
                         confirmed_trail_world_[i - 1].y,
                         confirmed_trail_world_[i].x,
                         confirmed_trail_world_[i].y);
    }
    return s;
  }

  bool projectOntoConfirmedHistoryUnlocked(const HistoryPoint2D& p,
                                           double* s_out,
                                           double* dist_out) const {
    if (confirmed_trail_world_.size() < 2) {
      return false;
    }
    double best_dist = std::numeric_limits<double>::infinity();
    double best_s = 0.0;
    double accum = 0.0;
    for (size_t i = 1; i < confirmed_trail_world_.size(); ++i) {
      const HistoryPoint2D& a = confirmed_trail_world_[i - 1];
      const HistoryPoint2D& b = confirmed_trail_world_[i];
      const double vx = b.x - a.x;
      const double vy = b.y - a.y;
      const double len2 = vx * vx + vy * vy;
      const double len = std::sqrt(std::max(0.0, len2));
      if (len2 > 1e-8) {
        const double t =
            clampValue(((p.x - a.x) * vx + (p.y - a.y) * vy) / len2, 0.0, 1.0);
        const double px = a.x + t * vx;
        const double py = a.y + t * vy;
        const double d = pointDistance(p.x, p.y, px, py);
        if (d < best_dist) {
          best_dist = d;
          best_s = accum + t * len;
        }
      }
      accum += len;
    }
    if (!std::isfinite(best_dist)) {
      return false;
    }
    if (s_out != NULL) {
      *s_out = best_s;
    }
    if (dist_out != NULL) {
      *dist_out = best_dist;
    }
    return true;
  }

  HistoryGeometryGate confirmedHistoryGeometryGateUnlocked(const HistoryPoint2D& candidate,
                                                           const LeadNode& node) const {
    HistoryGeometryGate out;
    if (confirmed_trail_world_.size() < 2) {
      return out;
    }

    const HistoryPoint2D& prev =
        confirmed_trail_world_[confirmed_trail_world_.size() - 2];
    const HistoryPoint2D& last = confirmed_trail_world_.back();
    const double d0 = pointDistance(prev.x, prev.y, last.x, last.y);
    const double d1 = pointDistance(last.x, last.y, candidate.x, candidate.y);
    if (d1 < 1e-4) {
      return out;
    }

    if (cfg_.confirmed_history_projection_gate_enabled && confirmed_trail_world_.size() >= 4) {
      double cand_s = 0.0;
      double cand_dist = 0.0;
      const double last_s = confirmedPolylineLengthUnlocked();
      if (projectOntoConfirmedHistoryUnlocked(candidate, &cand_s, &cand_dist)) {
        out.project_progress = cand_s - last_s;
        const bool projected_backtrack =
            out.project_progress < -cfg_.confirmed_history_projection_backtrack_gate;
        const bool local_loop =
            cand_dist < cfg_.confirmed_history_projection_near_dist &&
            out.project_progress < cfg_.confirmed_history_projection_backtrack_gate &&
            d1 > cfg_.fusion_append_min_distance;
        const bool lateral_revisit =
            std::fabs(out.project_progress) < cfg_.confirmed_history_short_reverse_dist &&
            cand_dist > cfg_.confirmed_history_projection_lateral_gate &&
            d1 > cfg_.fusion_append_min_distance;
        if (projected_backtrack || local_loop || lateral_revisit) {
          out.valid = false;
          out.loop = true;
          out.reason = projected_backtrack ? "history_projection_backtrack_reject"
                                           : (local_loop ? "history_loop_reject"
                                                         : "history_lateral_revisit_reject");
          return out;
        }
      }
    }

    double along = d1;
    if (d0 > 1e-4) {
      const double ux = (last.x - prev.x) / d0;
      const double uy = (last.y - prev.y) / d0;
      const double dx = candidate.x - last.x;
      const double dy = candidate.y - last.y;
      along = dx * ux + dy * uy;
      if (along < -cfg_.confirmed_history_reverse_gate) {
        out.valid = false;
        out.reverse = true;
        out.reason = "history_reverse_append_reject";
        return out;
      }
      const double yaw0 = std::atan2(last.y - prev.y, last.x - prev.x);
      const double yaw1 = std::atan2(candidate.y - last.y, candidate.x - last.x);
      out.kink_angle = std::fabs(wrapAngle(yaw1 - yaw0));
      const double curvature_impulse = out.kink_angle / std::max(0.03, std::min(d0, d1));
      const bool kink_reject =
          d0 >= 0.05 &&
          d1 >= 0.05 &&
          out.kink_angle > cfg_.confirmed_history_kink_reject_angle;
      const bool short_reverse =
          d1 <= cfg_.confirmed_history_short_reverse_dist &&
          out.kink_angle > cfg_.confirmed_history_kink_reject_angle;
      const bool curvature_reject =
          curvature_impulse > cfg_.confirmed_history_curvature_impulse_gate &&
          out.kink_angle > cfg_.history_kink_heading_gate;
      if (kink_reject || short_reverse || curvature_reject) {
        out.valid = false;
        out.reason = kink_reject ? "history_kink_reject"
                                 : (short_reverse ? "history_short_reverse_reject"
                                                  : "history_curvature_impulse_reject");
        return out;
      }
    }

    const bool ego_stopped = std::fabs(node.ego.v) < cfg_.confirmed_history_stop_speed_gate;
    const bool noisy_when_stopped =
        ego_stopped &&
        (node.fixed_frame_speed > cfg_.confirmed_history_stop_lead_speed_gate ||
         node.r_dynamic > cfg_.confirmed_history_stop_residual_gate ||
         node.r_heading_static > cfg_.confirmed_history_stop_residual_gate ||
         node.measurement_confidence < 0.55);
    if (noisy_when_stopped && d1 > cfg_.fusion_append_min_distance) {
      out.valid = false;
      out.reason = "history_stop_noise_reject";
      return out;
    }
    return out;
  }

  bool updateFusionHistoryUnlocked(const LeadNode& node, TrackerDebugState* debug) {
    last_confirmed_append_allowed_ = false;
    last_history_segment_break_ = false;
    const bool ego_observable = egoTranslationObservableUnlocked();
    const bool ego_source_known = egoTranslationSourceKnownUnlocked();
    const double yaw_activity =
        std::max(node.imu_yaw_activity, std::fabs(node.ego.yaw_rate));

    if (!fusion_.active) {
      fusion_.active = true;
      fusion_.stamp = node.stamp;
      fusion_.x = node.x;
      fusion_.y = node.y;
      fusion_.vx = 0.0;
      fusion_.vy = 0.0;
      fusion_.heading = node.yaw;
      fusion_.cov_x = 0.18;
      fusion_.cov_y = 0.18;
      fusion_.cov_vx = 0.35;
      fusion_.cov_vy = 0.35;
      fusion_.static_anchor_x = node.x;
      fusion_.static_anchor_y = node.y;
      fusion_.static_anchor_valid = true;
      fusion_.static_prob = 0.75;
      fusion_.dynamic_prob = 0.25;
      fusion_.last_range = node.range;
      fusion_.last_local_x = node.z_rel_x;
      fusion_.last_measurement_valid = true;
      fusion_recent_points_.clear();
      last_history_action_ = "fusion_initialized";
      last_confirmed_reject_reason_ = "fusion_initialized";
      if (debug != NULL) {
        debug->lead_fusion_res_static = 0.0;
        debug->lead_fusion_res_dynamic = 0.0;
        debug->fusion_static_likelihood = 1.0;
        debug->fusion_dynamic_likelihood = 1.0;
        debug->fusion_ego_likelihood = 1.0;
        debug->fusion_motion_score = 0.0;
        debug->fusion_confirm_accum_s = 0.0;
        debug->fusion_confirm_accum_t = 0.0;
        debug->static_prob = fusion_.static_prob;
        debug->dynamic_prob = fusion_.dynamic_prob;
        debug->fusion_pos_cov = fusion_.cov_x + fusion_.cov_y;
        debug->fusion_vel_cov = fusion_.cov_vx + fusion_.cov_vy;
        debug->low_speed_motion_score = 0.0;
        debug->speed_clamped = false;
        debug->fusion_cov_trace =
            debug->fusion_pos_cov + debug->fusion_vel_cov;
      }
      return false;
    }

    const double dt =
        (node.stamp.isValid() && fusion_.stamp.isValid())
            ? clampValue((node.stamp - fusion_.stamp).toSec(), 1e-3, 0.30)
            : 0.05;
    const double pred_x = fusion_.x + fusion_.vx * dt;
    const double pred_y = fusion_.y + fusion_.vy * dt;
    fusion_.cov_x += dt * (fusion_.cov_vx + cfg_.fusion_process_pos);
    fusion_.cov_y += dt * (fusion_.cov_vy + cfg_.fusion_process_pos);
    fusion_.cov_vx += dt * cfg_.fusion_process_vel;
    fusion_.cov_vy += dt * cfg_.fusion_process_vel;

    const double res_dynamic = pointDistance(node.x, node.y, pred_x, pred_y);
    const double res_static =
        fusion_.static_anchor_valid
            ? pointDistance(node.x, node.y, fusion_.static_anchor_x, fusion_.static_anchor_y)
            : 0.0;
    const double prev_range = fusion_.last_measurement_valid ? fusion_.last_range : node.range;
    const double prev_local_x = fusion_.last_measurement_valid ? fusion_.last_local_x : node.z_rel_x;
    const double static_ego_res =
        std::min(std::max(0.0, node.r_static), std::max(0.0, node.r_ego));
    const bool dynamic_residual_win_now =
        node.r_dynamic + cfg_.dynamic_residual_margin < static_ego_res;
    const bool ego_explained_win_now =
        static_ego_res + cfg_.ego_explained_freeze_gate < node.r_dynamic;
    const bool relative_shortening =
        !ego_source_known &&
        ((node.range - prev_range) < -cfg_.fusion_relative_shorten_gate ||
         (node.z_rel_x - prev_local_x) < -cfg_.fusion_relative_shorten_gate);

    const double motion_step = pointDistance(node.x, node.y, fusion_.x, fusion_.y);
    const double scale =
        std::max(0.03,
                 cfg_.fusion_meas_sigma_base +
                     cfg_.fusion_meas_sigma_range_gain * std::max(0.0, node.range));
    const double inv_var = 1.0 / std::max(1e-6, scale * scale);
    const double static_likelihood =
        std::exp(-0.5 * res_static * res_static * inv_var);
    const double dynamic_likelihood =
        std::exp(-0.5 * res_dynamic * res_dynamic * inv_var);
    const double ego_likelihood =
        std::exp(-0.5 * static_ego_res * static_ego_res * inv_var);
    const double ego_static_likelihood = std::max(static_likelihood, ego_likelihood);
    const double likelihood_ratio =
        std::log((dynamic_likelihood + 1e-4) / (ego_static_likelihood + 1e-4));
    const double normalized_motion =
        clampValue(motion_step / std::max(0.03, cfg_.fusion_append_min_distance), 0.0, 2.5);
    double evidence = likelihood_ratio + normalized_motion * 0.45;
    if (res_static < cfg_.fusion_static_residual_gate && res_dynamic < 1.5 * cfg_.fusion_static_residual_gate) {
      evidence -= 0.45;
    }
    if (!ego_source_known && yaw_activity >= cfg_.fusion_yaw_freeze_activity) {
      evidence -= 2.0;
    }
    if (relative_shortening) {
      evidence -= 2.5;
    }
    if (node.measurement_confidence < 0.35) {
      evidence -= 0.75;
    }

    const double dynamic_meas_prob = clampValue(logistic(evidence), 0.02, 0.98);
    const double alpha = clampValue(cfg_.fusion_prob_alpha, 0.02, 0.80);
    fusion_.dynamic_prob =
        clampValue((1.0 - alpha) * fusion_.dynamic_prob + alpha * dynamic_meas_prob, 0.0, 1.0);
    fusion_.static_prob = 1.0 - fusion_.dynamic_prob;

    if (ego_explained_win_now && !dynamic_residual_win_now) {
      ++fusion_ego_explained_hold_count_;
    } else {
      fusion_ego_explained_hold_count_ = std::max(0, fusion_ego_explained_hold_count_ - 1);
    }
    if (ego_static_likelihood > dynamic_likelihood * 1.35 &&
        motion_step < 1.5 * cfg_.fusion_append_min_distance) {
      ++fusion_static_hold_count_;
    } else {
      fusion_static_hold_count_ = std::max(0, fusion_static_hold_count_ - 1);
    }
    if (cfg_.yaw_mix_confirm_freeze &&
        (node.ego_yaw_mixing ||
         (yaw_activity >= cfg_.fusion_yaw_freeze_activity && ego_explained_win_now))) {
      ++fusion_yaw_hold_count_;
    } else {
      fusion_yaw_hold_count_ = std::max(0, fusion_yaw_hold_count_ - 1);
    }

    const bool ego_explained_freeze =
        fusion_ego_explained_hold_count_ >= std::max(1, cfg_.fusion_ego_explained_hold_frames);
    const bool static_freeze =
        fusion_static_hold_count_ >= std::max(1, cfg_.fusion_static_hold_frames);
    const bool yaw_mix_freeze =
        fusion_yaw_hold_count_ >= std::max(1, cfg_.fusion_yaw_mix_hold_frames);
    const bool static_observation_hold =
        cfg_.fusion_static_observation_hold_enabled &&
        !confirmed_trail_world_.empty() &&
        res_static <= cfg_.fusion_static_observation_res_gate &&
        motion_step <= cfg_.fusion_static_observation_step_gate &&
        node.fixed_frame_speed <= cfg_.fusion_static_observation_speed_gate &&
        ego_static_likelihood >= 0.8 * dynamic_likelihood;
    if ((ego_explained_freeze || static_freeze || yaw_mix_freeze || relative_shortening ||
         static_observation_hold) &&
        cfg_.fusion_prob_decay_on_static > 1e-4) {
      fusion_.dynamic_prob =
          clampValue(fusion_.dynamic_prob * (1.0 - cfg_.fusion_prob_decay_on_static), 0.0, 1.0);
      fusion_.static_prob = 1.0 - fusion_.dynamic_prob;
    }

    const double meas_var = scale * scale;
    const double kx = clampValue(fusion_.cov_x / std::max(1e-6, fusion_.cov_x + meas_var), 0.08, 0.85);
    const double ky = clampValue(fusion_.cov_y / std::max(1e-6, fusion_.cov_y + meas_var), 0.08, 0.85);
    const double upd_x = pred_x + kx * (node.x - pred_x);
    const double upd_y = pred_y + ky * (node.y - pred_y);
    const double vx_meas = (upd_x - fusion_.x) / dt;
    const double vy_meas = (upd_y - fusion_.y) / dt;
    const double vel_alpha = fusion_.dynamic_prob > 0.50 ? 0.35 : 0.12;
    fusion_.vx = (1.0 - vel_alpha) * fusion_.vx + vel_alpha * vx_meas;
    fusion_.vy = (1.0 - vel_alpha) * fusion_.vy + vel_alpha * vy_meas;
    bool speed_clamped = false;
    const double speed_before_clamp = hypot2(fusion_.vx, fusion_.vy);
    const double abs_speed_cap = std::max(0.05, cfg_.lead_speed_abs_max);
    if (speed_before_clamp > abs_speed_cap) {
      const double scale_speed = abs_speed_cap / std::max(1e-6, speed_before_clamp);
      fusion_.vx *= scale_speed;
      fusion_.vy *= scale_speed;
      speed_clamped = true;
    }
    if (fusion_.static_prob > cfg_.fusion_static_prob_freeze) {
      fusion_.vx *= 0.55;
      fusion_.vy *= 0.55;
    }
    fusion_.x = upd_x;
    fusion_.y = upd_y;
    if (hypot2(fusion_.vx, fusion_.vy) > 0.03) {
      fusion_.heading = std::atan2(fusion_.vy, fusion_.vx);
    } else {
      fusion_.heading = wrapAngle(0.7 * fusion_.heading + 0.3 * node.yaw);
    }
    fusion_.cov_x *= (1.0 - kx);
    fusion_.cov_y *= (1.0 - ky);
    fusion_.cov_vx = clampValue(fusion_.cov_vx, 0.02, 1.0);
    fusion_.cov_vy = clampValue(fusion_.cov_vy, 0.02, 1.0);
    fusion_.stamp = node.stamp;

    if (!fusion_.static_anchor_valid || fusion_.static_prob > 0.65) {
      const double anchor_alpha = fusion_.static_anchor_valid ? 0.08 : 1.0;
      fusion_.static_anchor_x =
          (1.0 - anchor_alpha) * fusion_.static_anchor_x + anchor_alpha * node.x;
      fusion_.static_anchor_y =
          (1.0 - anchor_alpha) * fusion_.static_anchor_y + anchor_alpha * node.y;
      fusion_.static_anchor_valid = true;
    }

    HistoryPoint2D candidate = fusionStateToHistoryPointUnlocked(node);
    const double fusion_pos_cov = fusion_.cov_x + fusion_.cov_y;
    const double fusion_vel_cov = fusion_.cov_vx + fusion_.cov_vy;
    const bool pos_cov_ok = fusion_pos_cov <= cfg_.fusion_pos_cov_gate;
    const bool yaw_freeze =
        (!ego_source_known && yaw_activity >= cfg_.fusion_yaw_freeze_activity) || yaw_mix_freeze;
    const double low_speed_motion_score =
        clampValue(normalized_motion, 0.0, 2.5) *
        clampValue((dynamic_likelihood + 1e-4) / (ego_static_likelihood + 1e-4),
                   0.0,
                   3.0);
    const double fusion_motion_score =
        0.55 * normalized_motion +
        0.45 * clampValue(likelihood_ratio + 1.0, 0.0, 2.0);
    const bool ego_unobservable_forward_append =
        !ego_source_known &&
        confirmed_trail_world_.empty() == false &&
        yaw_activity < cfg_.fusion_yaw_freeze_activity &&
        !relative_shortening &&
        (node.range - prev_range > 0.5 * cfg_.fusion_append_min_distance ||
         node.z_rel_x - prev_local_x > 0.5 * cfg_.fusion_append_min_distance);
    const bool hard_residual_ok =
        !cfg_.fusion_hard_residual_gate_enabled ||
        (dynamic_residual_win_now && !ego_explained_win_now);
    const bool accumulated_dynamic_evidence =
        dynamic_likelihood >= 0.70 &&
        fusion_motion_score >= 0.45 &&
        (fusion_confirm_accum_s_ >= cfg_.fusion_append_min_distance ||
         fusion_confirm_accum_t_ >= cfg_.fusion_append_max_interval);
    const bool dynamic_probability_ready =
        fusion_.dynamic_prob >= (confirmed_trail_world_.empty()
                                     ? cfg_.fusion_dynamic_prob_bootstrap
                                     : cfg_.fusion_dynamic_prob_append) ||
        accumulated_dynamic_evidence;
    const bool soft_motion_ready =
        fusion_motion_score >= 0.70 ||
        dynamic_likelihood >= 0.35 ||
        fusion_confirm_accum_s_ >= cfg_.fusion_append_min_distance ||
        fusion_confirm_accum_t_ >= cfg_.fusion_append_max_interval;
    const bool dynamic_ready_base =
        dynamic_probability_ready &&
        pos_cov_ok && hard_residual_ok &&
        !ego_explained_freeze && !static_freeze && !static_observation_hold &&
        !relative_shortening && !yaw_freeze && soft_motion_ready;
    const bool dynamic_ready =
        dynamic_ready_base &&
        (ego_source_known || confirmed_trail_world_.empty());
    const bool lead_only_bootstrap =
        cfg_.lead_only_bootstrap_enabled &&
        confirmed_trail_world_.empty() &&
        !ego_source_known &&
        yaw_activity <= cfg_.lead_only_bootstrap_max_imu_activity &&
        hard_residual_ok &&
        !ego_explained_freeze &&
        (res_static >= cfg_.lead_only_bootstrap_static_residual_gate ||
         fusion_confirm_accum_s_ >= cfg_.fusion_bootstrap_min_span) &&
        (node.range - prev_range > 0.5 * cfg_.fusion_append_min_distance ||
         node.z_rel_x - prev_local_x > 0.5 * cfg_.fusion_append_min_distance ||
         motion_step > cfg_.fusion_append_min_distance ||
         fusion_confirm_accum_s_ >= cfg_.fusion_bootstrap_min_span);

    const bool recent_admissible =
        node.endpoint_quality_ok &&
        node.measurement_confidence >= 0.20 &&
        !relative_shortening && !static_observation_hold &&
        !yaw_freeze;
    if (recent_admissible) {
      pushFusionRecentUnlocked(candidate);
      fusion_confirm_accum_s_ += motion_step;
      fusion_confirm_accum_t_ += dt;
    } else if (!confirmed_trail_world_.empty() && fusion_.static_prob > cfg_.fusion_static_prob_freeze) {
      fusion_recent_points_.clear();
      fusion_confirm_accum_s_ = 0.0;
      fusion_confirm_accum_t_ = 0.0;
    }

    bool appended = false;
    HistoryGeometryGate geometry_gate;
    if (confirmed_trail_world_.empty()) {
      if ((dynamic_ready || lead_only_bootstrap) && bootstrapFusionConfirmedUnlocked(node)) {
        appended = true;
      } else {
        last_history_action_ =
            relative_shortening ? "fusion_relative_shortening_hold"
                                : (yaw_freeze ? "fusion_yaw_hold" : "fusion_bootstrap_wait");
        last_confirmed_reject_reason_ =
            relative_shortening ? "fusion_relative_shortening"
                                : (yaw_freeze ? "fusion_yaw_activity" : "fusion_bootstrap_wait");
      }
    } else {
      const HistoryPoint2D& last = confirmed_trail_world_.back();
      const double dist = pointDistance(last.x, last.y, candidate.x, candidate.y);
      const double dt_confirm =
          (candidate.stamp.isValid() && last.stamp.isValid())
              ? std::max(1e-3, (candidate.stamp - last.stamp).toSec())
              : 1e-3;
      const double append_interval =
          (fusion_.last_append_stamp.isValid() && candidate.stamp.isValid())
              ? std::max(0.0, (candidate.stamp - fusion_.last_append_stamp).toSec())
              : std::numeric_limits<double>::infinity();
      const double effective_confirm_dt =
          std::max(dt_confirm,
                   std::max(std::isfinite(append_interval) ? append_interval : 0.0,
                            fusion_confirm_accum_t_));
      const double speed = dist / std::max(1e-3, effective_confirm_dt);
      const bool low_speed_interval_append =
          (append_interval >= cfg_.fusion_append_max_interval ||
           fusion_confirm_accum_t_ >= cfg_.fusion_append_max_interval) &&
          (dist >= 0.5 * cfg_.fusion_append_min_distance ||
           fusion_confirm_accum_s_ >= cfg_.fusion_append_min_distance);
      double along_confirmed = 0.0;
      double cross_confirmed = 0.0;
      bool confirmed_direction_valid = false;
      if (confirmed_trail_world_.size() >= 2) {
        const HistoryPoint2D& prev =
            confirmed_trail_world_[confirmed_trail_world_.size() - 2];
        const double dir_x = last.x - prev.x;
        const double dir_y = last.y - prev.y;
        const double dir_norm = hypot2(dir_x, dir_y);
        if (dir_norm > 1e-3) {
          const double ux = dir_x / dir_norm;
          const double uy = dir_y / dir_norm;
          const double dx = candidate.x - last.x;
          const double dy = candidate.y - last.y;
          along_confirmed = dx * ux + dy * uy;
          cross_confirmed = -dx * uy + dy * ux;
          confirmed_direction_valid = true;
        }
      }
      const bool reverse_confirmed =
          !ego_source_known &&
          confirmed_direction_valid &&
          along_confirmed < -cfg_.fusion_reverse_progress_gate;
      const bool unobservable_cross_jump =
          !ego_source_known &&
          confirmed_direction_valid &&
          along_confirmed < 0.5 * cfg_.fusion_append_min_distance &&
          std::fabs(cross_confirmed) > cfg_.fusion_unobservable_cross_gate;
      geometry_gate = confirmedHistoryGeometryGateUnlocked(candidate, node);
      if (!dynamic_ready) {
        last_history_action_ =
            static_observation_hold ? "fusion_static_observation_hold"
                                    : (relative_shortening ? "fusion_relative_shortening_hold"
                                                           : (yaw_freeze ? "fusion_yaw_hold" : "fusion_static_hold"));
        if (relative_shortening) {
          last_confirmed_reject_reason_ = "fusion_relative_shortening";
        } else if (static_observation_hold) {
          last_confirmed_reject_reason_ = "fusion_static_observation_hold";
        } else if (yaw_mix_freeze) {
          last_confirmed_reject_reason_ = node.ego_yaw_mixing ? "fusion_yaw_mix_freeze"
                                                              : "fusion_yaw_ego_explained_freeze";
        } else if (yaw_freeze) {
          last_confirmed_reject_reason_ = "fusion_yaw_activity";
        } else if (ego_explained_freeze) {
          last_confirmed_reject_reason_ = "fusion_ego_explained_sustained";
        } else if (static_freeze) {
          last_confirmed_reject_reason_ = "fusion_static_sustained";
        } else if (!hard_residual_ok) {
          last_confirmed_reject_reason_ = "fusion_hard_residual_gate";
        } else if (!ego_source_known && !confirmed_trail_world_.empty() &&
                   !ego_unobservable_forward_append) {
          last_confirmed_reject_reason_ = "fusion_ego_unobservable_no_forward_progress";
        } else if (fusion_.dynamic_prob <
                   (confirmed_trail_world_.empty() ? cfg_.fusion_dynamic_prob_bootstrap
                                                   : cfg_.fusion_dynamic_prob_append)) {
          last_confirmed_reject_reason_ =
              accumulated_dynamic_evidence ? "fusion_accum_dynamic_wait"
                                           : "fusion_dynamic_prob_low";
        } else if (!pos_cov_ok) {
          last_confirmed_reject_reason_ = "fusion_pos_cov_high";
        } else if (!soft_motion_ready) {
          last_confirmed_reject_reason_ = "fusion_motion_score_low";
        } else {
          last_confirmed_reject_reason_ = "fusion_dynamic_gate";
        }
      } else if (candidate.stamp <= last.stamp) {
        last_history_action_ = "fusion_timestamp_hold";
        last_confirmed_reject_reason_ = "fusion_timestamp_not_newer";
      } else if (speed > std::min(cfg_.fusion_max_confirm_speed, cfg_.lead_confirm_max_speed)) {
        last_history_action_ = "fusion_speed_rejected";
        last_confirmed_reject_reason_ = "fusion_speed_rejected";
      } else if (reverse_confirmed) {
        last_history_action_ = "fusion_reverse_progress_hold";
        last_confirmed_reject_reason_ = "fusion_reverse_progress";
      } else if (unobservable_cross_jump) {
        last_history_action_ = "fusion_unobservable_cross_hold";
        last_confirmed_reject_reason_ = "fusion_unobservable_cross_jump";
      } else if (!geometry_gate.valid) {
        ++history_geometry_reject_count_;
        ++consecutive_geometry_bad_frames_;
        ++history_tail_only_count_;
        reconstruction_state_ =
            consecutive_geometry_bad_frames_ >= cfg_.reconstruction_reseed_bad_frames
                ? "RECOVERY_RESEED"
                : "GEOMETRY_HOLD";
        if (reconstruction_state_ == "RECOVERY_RESEED") {
          fusion_recent_points_.clear();
          resetUnconfirmedAccumulatorUnlocked();
        }
        last_history_action_ = geometry_gate.reason;
        last_confirmed_reject_reason_ = geometry_gate.reason;
      } else if (dist < cfg_.fusion_append_min_distance && !low_speed_interval_append) {
        reconstruction_state_ =
            consecutive_geometry_bad_frames_ > 0 ? "GEOMETRY_HOLD" : "TRACK_GOOD";
        ++history_tail_only_count_;
        last_history_action_ = "fusion_append_distance_small";
        last_confirmed_reject_reason_ = "fusion_append_distance_small";
      } else {
        if (dist > cfg_.fusion_segment_break_gap) {
          ++confirmed_segment_id_;
          ++current_anchor_epoch_id_;
          candidate.segment_id = confirmed_segment_id_;
          candidate.anchor_epoch_id = current_anchor_epoch_id_;
          last_history_segment_break_ = true;
        }
        appendConfirmedPointWithCurveInterpolationUnlocked(candidate);
        regularizeConfirmedTailHeadingUnlocked();
        last_confirmed_prune_count_ +=
            pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
        fusion_confirm_accum_s_ = 0.0;
        fusion_confirm_accum_t_ = 0.0;
        last_confirmed_append_allowed_ = true;
        last_confirmed_reject_reason_ =
            last_history_segment_break_ ? "fusion_segment_break_appended" : "fusion_appended";
        last_history_action_ =
            last_history_segment_break_ ? "fusion_segment_break_appended" : "fusion_appended";
        appended = true;
        consecutive_geometry_bad_frames_ = 0;
        reconstruction_state_ = "TRACK_GOOD";
      }
    }

    fusion_.last_range = node.range;
    fusion_.last_local_x = node.z_rel_x;
    fusion_.last_measurement_valid = true;
    if (appended) {
      fusion_.last_append_stamp = node.stamp;
    }

    if (debug != NULL) {
      debug->lead_fusion_res_static = res_static;
      debug->lead_fusion_res_dynamic = res_dynamic;
      debug->fusion_static_likelihood = static_likelihood;
      debug->fusion_dynamic_likelihood = dynamic_likelihood;
      debug->fusion_ego_likelihood = ego_likelihood;
      debug->fusion_motion_score = fusion_motion_score;
      debug->fusion_confirm_accum_s = fusion_confirm_accum_s_;
      debug->fusion_confirm_accum_t = fusion_confirm_accum_t_;
      debug->fusion_pos_cov = fusion_pos_cov;
      debug->fusion_vel_cov = fusion_vel_cov;
      debug->low_speed_motion_score = low_speed_motion_score;
      debug->speed_clamped = speed_clamped;
      debug->static_prob = fusion_.static_prob;
      debug->dynamic_prob = fusion_.dynamic_prob;
      debug->fusion_cov_trace =
          fusion_.cov_x + fusion_.cov_y + fusion_.cov_vx + fusion_.cov_vy;
      debug->lead_only_bootstrap = lead_only_bootstrap;
      debug->dynamic_residual_win_now = dynamic_residual_win_now;
      debug->ego_explained_win_now = ego_explained_win_now;
      debug->yaw_mix_freeze = yaw_mix_freeze;
      debug->signed_wheel_speed_used = ego_source_known;
      debug->confirmed_freeze_reason = last_confirmed_reject_reason_;
      debug->history_geometry_reject_count = history_geometry_reject_count_;
      debug->history_kink_angle =
          confirmed_trail_world_.size() >= 2 ? geometry_gate.kink_angle : 0.0;
      debug->history_reverse_append_reject =
          confirmed_trail_world_.size() >= 2 ? geometry_gate.reverse : false;
      debug->confirmed_append_geometry_valid =
          confirmed_trail_world_.size() >= 2 ? geometry_gate.valid : true;
      debug->history_project_progress = geometry_gate.project_progress;
      debug->history_loop_reject = geometry_gate.loop;
      debug->history_tail_only_count = history_tail_only_count_;
      debug->reconstruction_state = reconstruction_state_;
    }
    return appended;
  }

  void commitConfirmedTrailPointUnlocked() {
    last_confirmed_append_allowed_ = false;
    last_confirmed_reject_reason_ = "init";
    if (tail_buffer_world_.size() < static_cast<size_t>(std::max(2, cfg_.tail_confirm_min_points))) {
      if (track_.static_mode) {
        last_history_action_ = "history_anchor_hold";
      }
      last_confirmed_reject_reason_ = "tail_too_small";
      return;
    }

    if (last_ego_yaw_mixing_) {
      resetUnconfirmedAccumulatorUnlocked();
      last_history_action_ = "ego_mixing_hold_history";
      last_confirmed_reject_reason_ = "ego_yaw_mixing";
      return;
    }

    const MotionClass latest_motion =
        nodes_.empty() ? MOTION_STATIC_OR_EGO_EXPLAINED
                       : static_cast<MotionClass>(nodes_.back().motion_class);
    const bool latest_lead_only_bootstrap =
        confirmed_trail_world_.empty() &&
        latest_motion == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP;
    const bool latest_dynamic_motion =
        latest_motion == MOTION_DYNAMIC_CANDIDATE ||
        latest_motion == MOTION_CURVED_DYNAMIC_CANDIDATE ||
        latest_lead_only_bootstrap;
    if (cfg_.confirmed_require_dynamic_motion_class && !latest_dynamic_motion) {
      resetUnconfirmedAccumulatorUnlocked();
      last_history_action_ = motionClassName(latest_motion);
      last_confirmed_reject_reason_ = "motion_class_not_dynamic";
      return;
    }

    const bool conditioned_dynamic = egoMotionConditionedDynamicUnlocked();
    const bool dynamic_probe = staticModeDynamicProbeUnlocked();
    const bool dynamic_evidence = dynamicCandidateEvidenceUnlocked();
    if (!dynamic_evidence) {
      resetUnconfirmedAccumulatorUnlocked();
      last_history_action_ = nodes_.empty() ? "history_anchor_hold" : motionClassName(
          static_cast<MotionClass>(nodes_.back().motion_class));
      last_confirmed_reject_reason_ = "dynamic_evidence_false";
      return;
    }
    if ((track_.static_mode && !conditioned_dynamic && !dynamic_probe && !dynamic_evidence) ||
        (!confirmDynamicHypothesisWinsUnlocked() && !conditioned_dynamic &&
         !dynamic_probe && !dynamic_evidence)) {
      last_history_action_ = "history_anchor_hold";
      last_confirmed_reject_reason_ = "dynamic_confirm_gate";
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
      last_confirmed_reject_reason_ = "tail_too_fresh";
      return;
    }

    HistoryPoint2D candidate = tail_buffer_world_[candidate_index];
    const MotionClass candidate_motion =
        static_cast<MotionClass>(candidate.motion_class);
    const bool candidate_dynamic_motion =
        candidate_motion == MOTION_DYNAMIC_CANDIDATE ||
        candidate_motion == MOTION_CURVED_DYNAMIC_CANDIDATE ||
        (confirmed_trail_world_.empty() &&
         candidate_motion == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP);
    const bool curve_dynamic =
        candidate_motion == MOTION_CURVED_DYNAMIC_CANDIDATE;
    const bool candidate_lead_only_bootstrap =
        confirmed_trail_world_.empty() &&
        candidate_motion == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP;
    if (cfg_.confirm_require_candidate_metadata &&
        (!candidate_dynamic_motion ||
         candidate.relative_shortening_only ||
         !candidate.endpoint_quality_ok ||
         (!candidate_lead_only_bootstrap &&
          !candidate.ego_translation_observable && !candidate.dynamic_residual_win))) {
      resetUnconfirmedAccumulatorUnlocked();
      last_history_action_ = "candidate_metadata_rejected";
      if (!candidate_dynamic_motion) {
        last_confirmed_reject_reason_ = "candidate_motion_class_not_dynamic";
      } else if (candidate.relative_shortening_only) {
        last_confirmed_reject_reason_ = "candidate_relative_shortening";
      } else if (!candidate.endpoint_quality_ok) {
        last_confirmed_reject_reason_ = "candidate_endpoint_quality";
      } else if (candidate_lead_only_bootstrap) {
        last_confirmed_reject_reason_ = "candidate_lead_only_bootstrap_rejected";
      } else {
        last_confirmed_reject_reason_ = "candidate_ego_unobservable_no_residual_win";
      }
      return;
    }
    updateUnconfirmedAccumulatorUnlocked(candidate);
    const double candidate_static_res =
        residualAgainstWorldPointUnlocked(nodes_.back().ego, candidate.x, candidate.y,
                                          nodes_.back().z_rel_x, nodes_.back().z_rel_y);

    if (dynamicTrailProgressUnlocked() < min_confirm_progress && !dynamic_evidence) {
      last_history_action_ = "history_progress_too_small";
      last_confirmed_reject_reason_ = "progress_too_small";
      return;
    }

    if (meanEgoMotionExplainedResidualUnlocked() <= cfg_.confirm_static_residual_gate &&
        !conditioned_dynamic && !dynamic_probe && !dynamic_evidence) {
      last_history_action_ = "history_ego_motion_explained";
      last_confirmed_reject_reason_ = "ego_motion_explained";
      return;
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
      if (checked > 0 && consistent + 1 < checked &&
          !curve_dynamic &&
          (!confirmed_trail_world_.empty() || !dynamic_evidence)) {
        last_history_action_ = "history_direction_unstable";
        last_confirmed_reject_reason_ = "direction_unstable";
        return;
      }
    }

    const double accum_distance_gate = std::min(cfg_.tail_confirm_min_distance, cfg_.confirm_accum_distance);
    const double accum_timeout_gate = cfg_.confirm_accum_timeout;
    const bool accum_ready =
        unconfirmed_progress_accum_ >= accum_distance_gate ||
        (unconfirmed_time_accum_ >= accum_timeout_gate &&
         unconfirmed_progress_accum_ >= 3.0 * cfg_.measurement_cross_sigma);
    const bool bootstrap_ready =
        confirmed_trail_world_.empty() &&
        dynamic_evidence &&
        tail_buffer_world_.size() >=
            static_cast<size_t>(std::max(2, cfg_.dynamic_bootstrap_min_tail_points)) &&
        tailBufferSpanUnlocked() >= cfg_.dynamic_bootstrap_min_span;
    if (!accum_ready && !bootstrap_ready) {
      last_history_action_ = "history_progress_accumulating";
      last_confirmed_reject_reason_ = "progress_accumulating";
      return;
    }

    const bool dynamic_override_static =
        (accum_ready || bootstrap_ready) &&
        (confirmDynamicHypothesisWinsUnlocked() || conditioned_dynamic || dynamic_probe || dynamic_evidence) &&
        (unconfirmed_progress_accum_ >= cfg_.confirm_accum_distance ||
         unconfirmed_time_accum_ >= cfg_.confirm_accum_timeout ||
         bootstrap_ready) &&
        (dynamicTrailProgressUnlocked() >= 0.5 * min_confirm_progress || bootstrap_ready);
    if (ego_speed >= cfg_.ego_static_speed_gate &&
        candidate_static_res < cfg_.confirm_static_residual_gate &&
        !dynamic_override_static) {
      last_history_action_ = "history_static_explained";
      last_confirmed_reject_reason_ = "static_explained";
      return;
    }
    if (ego_speed >= cfg_.ego_static_speed_gate &&
        candidate_static_res < cfg_.confirm_static_residual_gate &&
        dynamic_override_static) {
      last_history_action_ = "history_dynamic_accum_override";
    }

    if (confirmed_trail_world_.empty()) {
      if (bootstrapConfirmedTrailFromTailUnlocked(tail_stamp, min_confirm_age)) {
        return;
      }
      candidate.predicted = false;
      confirmed_trail_world_.push_back(candidate);
      regularizeConfirmedTailHeadingUnlocked();
      if (last_history_action_ != "history_dynamic_accum_override") {
        last_history_action_ = "history_bootstrap";
      }
      last_confirmed_prune_count_ +=
          pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
      resetUnconfirmedAccumulatorUnlocked();
      last_confirmed_append_allowed_ = true;
      last_confirmed_reject_reason_ = "confirmed_bootstrap";
      return;
    }

    const HistoryPoint2D& last_confirmed = confirmed_trail_world_.back();
    if (candidate.stamp <= last_confirmed.stamp) {
      last_history_action_ = "history_lowpass_update";
      last_confirmed_reject_reason_ = "candidate_not_newer";
      return;
    }

    const double dist = pointDistance(last_confirmed.x, last_confirmed.y, candidate.x, candidate.y);
    const double heading_err = std::fabs(wrapAngle(candidate.heading - last_confirmed.heading));
    const double dx_confirmed = candidate.x - last_confirmed.x;
    const double dy_confirmed = candidate.y - last_confirmed.y;
    const double along_confirmed =
        dx_confirmed * std::cos(last_confirmed.heading) +
        dy_confirmed * std::sin(last_confirmed.heading);
    const double cross_confirmed =
        -dx_confirmed * std::sin(last_confirmed.heading) +
        dy_confirmed * std::cos(last_confirmed.heading);
    if (!curve_dynamic &&
        along_confirmed < -cfg_.reverse_progress_gate &&
        dist > std::max(0.5 * cfg_.tail_confirm_min_distance, 0.02)) {
      resetUnconfirmedAccumulatorUnlocked();
      last_history_action_ = "reject_history_reverse_progress";
      last_confirmed_reject_reason_ = "reverse_progress";
      return;
    }
    if (!curve_dynamic &&
        along_confirmed < 0.0 && std::fabs(cross_confirmed) > cfg_.tail_cross_gate) {
      resetUnconfirmedAccumulatorUnlocked();
      last_history_action_ = "reject_history_reverse_cross";
      last_confirmed_reject_reason_ = "reverse_cross";
      return;
    }
    const double confirm_dt = std::max(1e-3, (candidate.stamp - last_confirmed.stamp).toSec());
    const double confirm_speed = dist / confirm_dt;
    const bool segment_break =
        cfg_.history_segment_break_gap > 1e-3 &&
        dist > cfg_.history_segment_break_gap &&
        !curve_dynamic;
    if (!curve_dynamic &&
        dist < cfg_.history_kink_min_dist && heading_err > cfg_.history_kink_heading_gate) {
      last_history_action_ = "reject_history_kink_small_dist_large_heading";
      last_confirmed_reject_reason_ = "small_dist_large_heading";
      return;
    }
    if (confirm_speed > cfg_.history_max_confirm_speed) {
      last_history_action_ = "reject_history_unrealistic_speed";
      last_confirmed_reject_reason_ = "unrealistic_speed";
      return;
    }
    if (segment_break) {
      ++confirmed_segment_id_;
      ++current_anchor_epoch_id_;
      candidate.segment_id = confirmed_segment_id_;
      candidate.anchor_epoch_id = current_anchor_epoch_id_;
      candidate.predicted = false;
      confirmed_trail_world_.push_back(candidate);
      resetUnconfirmedAccumulatorUnlocked();
      last_history_action_ = "history_segment_break_appended";
      last_confirmed_append_allowed_ = true;
      last_confirmed_reject_reason_ = "confirmed_segment_break";
      last_history_segment_break_ = true;
      last_confirmed_prune_count_ +=
          pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
      return;
    }
    const bool append =
        dist >= cfg_.tail_confirm_min_distance ||
        (curve_dynamic && dist >= 0.5 * cfg_.tail_confirm_min_distance) ||
        (dist >= 0.08 && heading_err > 0.18) ||
        (confirm_speed > cfg_.lead_static_speed_gate && dist >= 0.06);

    if (append) {
      candidate.predicted = false;
      appendConfirmedPointWithCurveInterpolationUnlocked(candidate);
      regularizeConfirmedTailHeadingUnlocked();
      if (last_history_action_ != "history_dynamic_accum_override") {
        last_history_action_ = curve_dynamic ? "history_appended_curve" : "history_appended";
      }
      resetUnconfirmedAccumulatorUnlocked();
      last_confirmed_append_allowed_ = true;
      last_confirmed_reject_reason_ = curve_dynamic ? "confirmed_appended_curve" : "confirmed_appended";
    } else {
      last_history_action_ = "history_lowpass_update";
      last_confirmed_reject_reason_ = "append_distance_small";
    }

    last_confirmed_prune_count_ +=
        pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
  }

  std::vector<HistoryPoint2D> exportHistoryUnlocked() const {
    std::vector<HistoryPoint2D> history;
    history.reserve(confirmed_trail_world_.size() + 1);
    for (size_t i = 0; i < confirmed_trail_world_.size(); ++i) {
      HistoryPoint2D pt = confirmed_trail_world_[i];
      pt.predicted = false;
      history.push_back(pt);
    }

    if (cfg_.export_static_anchor_to_history && has_static_anchor_) {
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
        const double heading = history.back().heading;
        const double dx = anchor.x - history.back().x;
        const double dy = anchor.y - history.back().y;
        const double along = dx * std::cos(heading) + dy * std::sin(heading);
        const double cross = -dx * std::sin(heading) + dy * std::cos(heading);
        if (d > cfg_.static_anchor_append_distance_gate &&
            (!history.back().stamp.isValid() || !anchor.stamp.isValid() ||
             anchor.stamp > history.back().stamp) &&
            along >= -std::max(0.02, cfg_.tail_confirm_min_distance) &&
            std::fabs(cross) <= std::max(0.30, cfg_.tail_cross_gate)) {
          history.push_back(anchor);
        }
      }
    }

    return removeNearDuplicatePoints(history, 0.01);
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
    const bool classified_dynamic =
        last.motion_class == MOTION_DYNAMIC_CANDIDATE ||
        last.motion_class == MOTION_CURVED_DYNAMIC_CANDIDATE ||
        (confirmed_trail_world_.empty() &&
         last.motion_class == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP);
    const bool classified_static_or_mix =
        last.motion_class == MOTION_STATIC_OR_EGO_EXPLAINED ||
        last.motion_class == MOTION_YAW_MIXING ||
        last.motion_class == MOTION_RELATIVE_SHORTENING_ONLY;
    const double fixed_frame_progress = dynamicTrailProgressUnlocked();
    const bool static_anchor_project =
        std::min(static_measurement_residual, short_window_prediction_residual) <
            cfg_.static_anchor_res_gate &&
        fixed_frame_progress < cfg_.static_anchor_progress_gate &&
        vote_speed < cfg_.static_anchor_speed_gate &&
        last_innovation_norm_ < cfg_.static_residual_gate;

    if (classified_static_or_mix ||
        (vote_speed < cfg_.lead_static_speed_gate &&
         measurement_speed < cfg_.ego_compensated_static_gate &&
         std::min(static_measurement_residual, short_window_prediction_residual) <
             cfg_.ego_compensated_static_gate &&
         hs_score <= hd_score + cfg_.dual_hypothesis_score_margin &&
         static_confidence > 0.55 &&
         last_innovation_norm_ < cfg_.static_residual_gate)) {
      static_votes_ = std::min(static_votes_ + 1, 1000);
    } else {
      static_votes_ = std::max(0, static_votes_ - 1);
    }

    if (classified_dynamic &&
        vote_speed > cfg_.lead_moving_speed_gate &&
        measurement_speed > cfg_.ego_compensated_moving_gate &&
        static_measurement_residual > 0.5 * cfg_.ego_compensated_static_gate &&
        short_window_prediction_residual > 0.5 * cfg_.ego_compensated_static_gate &&
        hd_score + cfg_.dual_hypothesis_score_margin < hs_score &&
        static_confidence < 0.55 &&
        last_innovation_norm_ < cfg_.moving_residual_gate) {
      moving_votes_ = std::min(moving_votes_ + 1, 1000);
    } else if (classified_dynamic && conditioned_dynamic) {
      moving_votes_ = std::min(moving_votes_ + 1, 1000);
      static_votes_ = std::max(0, static_votes_ - 1);
    } else if (mean_ego_speed < cfg_.ego_static_speed_gate &&
               classified_dynamic &&
               vote_speed > 0.5 * cfg_.lead_moving_speed_gate &&
               hd_score < hs_score &&
               dynamicTrailProgressUnlocked() > 0.5 * cfg_.confirm_min_progress) {
      moving_votes_ = std::min(moving_votes_ + 1, 1000);
    } else {
      moving_votes_ = std::max(0, moving_votes_ - 1);
    }

    if (!track_.static_mode && static_votes_ >= cfg_.static_vote_on) {
      track_.static_mode = true;
      static_anchor_x_ = track_.x;
      static_anchor_y_ = track_.y;
      static_anchor_heading_ = track_.heading;
      static_anchor_stamp_ = last.stamp;
      static_mode_start_stamp_ = last.stamp;
      has_static_anchor_ = true;
      moving_flag_ = false;
    }

    if (track_.static_mode) {
      const bool strong_static = hs_score + cfg_.static_release_score_margin < hd_score;
      if (strong_static || static_anchor_project) {
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
      if (!static_anchor_project &&
          ((static_votes_ <= cfg_.static_vote_release && moving_votes_ >= cfg_.moving_vote_on) ||
          conditioned_dynamic ||
          classified_dynamic ||
          staticModeDynamicProbeUnlocked() ||
          (hd_score + cfg_.static_release_score_margin < hs_score))) {
        track_.static_mode = false;
        static_mode_start_stamp_ = ros::Time();
        moving_flag_ = true;
      }
    } else {
      moving_flag_ = moving_votes_ >= cfg_.moving_vote_on;
    }

    const bool dynamic_probe = staticModeDynamicProbeUnlocked();
    const bool curve_dynamic =
        last.motion_class == MOTION_CURVED_DYNAMIC_CANDIDATE &&
        recentCurveArcProgressUnlocked() >= 0.5 * cfg_.dynamic_curve_min_arc;
    const bool lead_only_bootstrap =
        confirmed_trail_world_.empty() &&
        last.motion_class == MOTION_LEAD_ONLY_DYNAMIC_BOOTSTRAP;
    (void)dynamic_probe;
    (void)conditioned_dynamic;
    (void)lead_only_bootstrap;
    tail_buffer_world_.clear();
    resetUnconfirmedAccumulatorUnlocked();
    if (cfg_.use_fusion_tracker) {
      updateFusionHistoryUnlocked(last, debug);
      moving_flag_ = fusion_.dynamic_prob > 0.55;
      track_.static_mode = fusion_.static_prob > cfg_.fusion_static_prob_freeze;
    } else {
      const bool allow_static_tail = conditioned_dynamic || dynamic_probe || curve_dynamic || lead_only_bootstrap;
      refreshTailBufferUnlocked(allow_static_tail);
      commitConfirmedTrailPointUnlocked();
    }

    if (debug != NULL) {
      debug->history_reason = last_history_action_;
      debug->history_size = exportHistoryUnlocked().size();
      debug->ff_speed = measurement_speed;
      debug->mean_ego_speed = mean_ego_speed;
      debug->static_hyp_res = std::min(static_measurement_residual, short_window_prediction_residual);
      debug->r_static = last.r_static;
      debug->r_ego = last.r_ego;
      debug->r_dynamic = last.r_dynamic;
      debug->r_heading_static = last.r_heading_static;
      debug->imu_yaw_activity = last.imu_yaw_activity;
      debug->reverse_progress = last.reverse_progress;
      debug->motion_class = motionClassName(static_cast<MotionClass>(last.motion_class));
      debug->ego_translation_observable = egoTranslationObservableUnlocked();
      debug->relative_shortening_only =
          last.motion_class == MOTION_RELATIVE_SHORTENING_ONLY;
      debug->confirmed_append_allowed = last_confirmed_append_allowed_;
      debug->confirmed_reject_reason = last_confirmed_reject_reason_;
      debug->candidate_motion_class =
          tail_buffer_world_.empty()
              ? "none"
              : motionClassName(static_cast<MotionClass>(tail_buffer_world_.back().motion_class));
      debug->candidate_dynamic_residual_win =
          !tail_buffer_world_.empty() && tail_buffer_world_.back().dynamic_residual_win;
      debug->candidate_ego_translation_observable =
          !tail_buffer_world_.empty() && tail_buffer_world_.back().ego_translation_observable;
      debug->tail_admit_reason = last_tail_admit_reason_;
      debug->confirmed_segment_id = confirmed_segment_id_;
      debug->history_segment_break = last_history_segment_break_;
      debug->hs_score = hs_score;
      debug->hd_score = hd_score;
      debug->ego_motion_explained_res = meanEgoMotionExplainedResidualUnlocked();
      debug->ego_yaw_mixing = last_ego_yaw_mixing_;
      debug->static_mode = track_.static_mode;
      debug->accum_s = unconfirmed_progress_accum_;
      debug->accum_t = unconfirmed_time_accum_;
      debug->confirmed_count = confirmed_trail_world_.size();
      debug->tail_count = tail_buffer_world_.size();
      debug->curve_mode = curve_dynamic;
      debug->lead_only_bootstrap = debug->lead_only_bootstrap || lead_only_bootstrap;
      debug->curve_arc_progress = recentCurveArcProgressUnlocked();
      debug->curve_heading_consistency = recentCurveHeadingConsistencyUnlocked();
      debug->confirmed_prune_count = last_confirmed_prune_count_;
      if (cfg_.use_fusion_tracker) {
        debug->candidate_motion_class = "fusion_state";
        debug->candidate_dynamic_residual_win =
            debug->dynamic_prob >= cfg_.fusion_dynamic_prob_append;
        debug->candidate_ego_translation_observable = egoTranslationObservableUnlocked();
        debug->tail_admit_reason = "fusion_tracker";
        debug->tail_count = fusion_recent_points_.size();
        debug->motion_class =
            debug->dynamic_prob >= cfg_.fusion_dynamic_prob_append
                ? "FUSION_DYNAMIC"
                : (debug->static_prob >= cfg_.fusion_static_prob_freeze
                       ? "FUSION_STATIC"
                       : "FUSION_UNCERTAIN");
      }
      last_confirmed_prune_count_ = 0;
      last_history_segment_break_ = false;
    }
  }

  Config cfg_;
  LeadState2D track_;
  std::deque<LeadNode> nodes_;
  std::deque<HistoryPoint2D> confirmed_trail_world_;
  std::deque<HistoryPoint2D> tail_buffer_world_;
  int last_confirmed_prune_count_ = 0;
  bool last_confirmed_append_allowed_ = false;
  std::string last_confirmed_reject_reason_ = "init";
  std::string last_tail_admit_reason_ = "init";
  int confirmed_segment_id_ = 0;
  int current_anchor_epoch_id_ = 0;
  bool last_history_segment_break_ = false;
  LeadFusionState fusion_;
  std::deque<HistoryPoint2D> fusion_recent_points_;
  double fusion_confirm_accum_s_ = 0.0;
  double fusion_confirm_accum_t_ = 0.0;
  int fusion_static_hold_count_ = 0;
  int fusion_ego_explained_hold_count_ = 0;
  int fusion_yaw_hold_count_ = 0;
  int history_geometry_reject_count_ = 0;
  int consecutive_geometry_bad_frames_ = 0;
  int history_tail_only_count_ = 0;
  std::string reconstruction_state_ = "TRACK_GOOD";
  int static_votes_ = 0;
  int moving_votes_ = 0;
  bool moving_flag_ = false;
  std::string last_history_action_ = "init";
  double static_anchor_x_ = 0.0;
  double static_anchor_y_ = 0.0;
  double static_anchor_heading_ = 0.0;
  bool has_static_anchor_ = false;
  ros::Time static_anchor_stamp_;
  ros::Time static_mode_start_stamp_;
  double last_innovation_norm_ = 0.0;
  double last_innovation_metric_ = 0.0;
  bool last_ego_yaw_mixing_ = false;
  bool unconfirmed_accum_initialized_ = false;
  double unconfirmed_progress_accum_ = 0.0;
  double unconfirmed_time_accum_ = 0.0;
  double unconfirmed_last_x_ = 0.0;
  double unconfirmed_last_y_ = 0.0;
  double unconfirmed_last_heading_ = 0.0;
  ros::Time unconfirmed_last_stamp_;
  ros::Time unconfirmed_start_stamp_;
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
    double lead_center_to_path_ref_offset = 0.0;
    int raw_min_points_for_publish = 2;
    double raw_min_span_for_publish = 0.08;
    double raw_keep_behind_x = 0.50;
    bool raw_clip_to_ego_progress = true;
    std::string raw_spatial_window_mode = "ego_progress_suffix";
    double raw_clip_duplicate_distance = 0.02;
    bool raw_clip_use_current_lead_endpoint = true;
    bool raw_clip_end_follow_lead_endpoint = false;
    bool raw_clip_keep_history_tail_when_endpoint_behind = true;
    bool raw_clip_endpoint_hysteresis = true;
    double raw_clip_max_s_jump = 0.40;
    double raw_clip_max_s_step = 0.08;
    double raw_clip_max_s_rate = 1.20;
    double raw_clip_start_max_s_step = 0.08;
    double raw_clip_start_max_s_rate = 1.20;
    double raw_clip_endpoint_hold_time = 0.50;
    double raw_clip_endpoint_projection_gate = 0.75;
    double raw_clip_min_endpoint_confidence = 0.55;
    int endpoint_recapture_min_frames = 4;
    double raw_clip_projection_far_hold_max_time = 0.80;
    double endpoint_recapture_confidence_gate = 0.75;
    double endpoint_recapture_projection_gate = 1.00;
    double endpoint_recapture_max_far_time = 0.30;
    double raw_clip_start_search_window = 1.20;
    double raw_clip_endpoint_search_window = 1.50;
    bool raw_clip_allow_endpoint_reorder = true;
    double raw_clip_valid_endpoint_hold_max_time = 0.15;
    double raw_clip_missing_endpoint_hold_time = 0.50;
    double raw_clip_stale_max_time = 0.15;
    double raw_clip_min_visible_span = 0.45;
    double raw_max_range = 30.0;
    int raw_max_points = 250;
    double reference_join_min_forward_x = 0.60;
    double reference_join_max_lateral = 3.0;
    double reference_join_lookahead = 0.35;
    double reference_join_skip_weight = 0.20;
    double reference_join_curvature_weight = 0.90;
    double reference_join_time_weight = 0.60;
    double reference_join_progress_weight = 0.25;
    double reference_join_s_window = 2.0;
    double reference_join_hysteresis_weight = 2.0;
    double reference_join_max_s_backtrack = 0.10;
    std::string reference_join_mode = "front_anchored";
    double reference_join_front_lookahead = 0.25;
    double reference_join_max_step = 0.08;
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
    double reference_max_length = 20.0;
    bool reference_follow_raw_only = false;
    bool reference_publish_short_endpoint = true;
    int reference_short_endpoint_min_confirmed = 2;
    bool reference_short_endpoint_when_clip_stale = true;
    bool reference_short_endpoint_only_when_confirmed_empty = true;
    double control_feasible_max_lateral = 1.20;
    double control_feasible_max_heading = 0.80;
    bool anchor_rebase_enabled = true;
    double anchor_rebase_xy_gate = 0.05;
    double anchor_rebase_yaw_gate = 0.03;
    double raw_output_segment_gap = 0.45;
    double raw_output_heading_flip_gate = 0.80;
    double raw_output_min_stable_span = 0.80;
    double raw_front_jump_gate = 0.35;
    double path_heading_recompute_min_ds = 0.05;
    bool reference_temporal_consistency = true;
    double reference_max_heading_step = 0.15;
  };

  explicit PathGenerator(const Config& cfg) : cfg_(cfg) {}

  double worldSpan(const std::vector<HistoryPoint2D>& pts, size_t begin, size_t end) const {
    if (end <= begin + 1 || end > pts.size()) {
      return 0.0;
    }
    double span = 0.0;
    for (size_t i = begin + 1; i < end; ++i) {
      span += pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
    }
    return span;
  }

  double localSpan(const std::vector<LocalPathPoint>& pts, size_t begin, size_t end) const {
    if (end <= begin + 1 || end > pts.size()) {
      return 0.0;
    }
    double span = 0.0;
    for (size_t i = begin + 1; i < end; ++i) {
      span += pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
    }
    return span;
  }

  std::vector<HistoryPoint2D> selectContinuousWorldSuffix(const std::vector<HistoryPoint2D>& world_history,
                                                          TrackerDebugState* debug) const {
    std::vector<HistoryPoint2D> source = world_history;
    if (source.empty()) {
      return source;
    }

    struct Segment {
      size_t begin = 0;
      size_t end = 0;
      std::string reason = "same_segment";
      double span = 0.0;
      double score = 0.0;
    };
    std::vector<Segment> segments;
    size_t segment_begin = 0;
    std::string reason = "same_segment";
    int heading_jump_count = 0;
    for (size_t i = 1; i < source.size(); ++i) {
      const HistoryPoint2D& prev = source[i - 1];
      const HistoryPoint2D& cur = source[i];
      const double d = pointDistance(prev.x, prev.y, cur.x, cur.y);
      bool broken = false;
      std::string break_reason;
      if (cfg_.anchor_rebase_enabled && cur.anchor_epoch_id != prev.anchor_epoch_id) {
        broken = true;
        break_reason = "anchor_epoch_switch";
      } else if (cur.segment_id != prev.segment_id) {
        broken = true;
        break_reason = "segment_switch";
      } else if (d > cfg_.raw_output_segment_gap) {
        broken = true;
        break_reason = "segment_gap";
      } else if (i >= 2) {
        const HistoryPoint2D& pprev = source[i - 2];
        const double d0 = pointDistance(pprev.x, pprev.y, prev.x, prev.y);
        if (d0 >= cfg_.path_heading_recompute_min_ds && d >= cfg_.path_heading_recompute_min_ds) {
          const double yaw0 = std::atan2(prev.y - pprev.y, prev.x - pprev.x);
          const double yaw1 = std::atan2(cur.y - prev.y, cur.x - prev.x);
          const double dyaw = std::fabs(wrapAngle(yaw1 - yaw0));
          if (dyaw > cfg_.raw_output_heading_flip_gate) {
            ++heading_jump_count;
            broken = true;
            break_reason = "curvature_impulse";
          }
        }
      }

      if (broken) {
        Segment seg;
        seg.begin = segment_begin;
        seg.end = i;
        seg.reason = reason;
        seg.span = worldSpan(source, seg.begin, seg.end);
        segments.push_back(seg);
        segment_begin = i;
        reason = break_reason;
      }
    }
    Segment last_seg;
    last_seg.begin = segment_begin;
    last_seg.end = source.size();
    last_seg.reason = reason;
    last_seg.span = worldSpan(source, last_seg.begin, last_seg.end);
    segments.push_back(last_seg);

    const double min_stable_span =
        std::max(cfg_.raw_output_min_stable_span, cfg_.raw_min_span_for_publish);
    size_t selected = segments.empty() ? 0 : segments.size() - 1;
    double best_score = -std::numeric_limits<double>::infinity();
    for (size_t idx = 0; idx < segments.size(); ++idx) {
      Segment& seg = segments[idx];
      const bool stable_span = seg.span >= min_stable_span;
      const double recency = segments.empty() ? 0.0 : static_cast<double>(seg.end) /
                                                    static_cast<double>(std::max<size_t>(1, source.size()));
      const double latest_bonus = (idx + 1 == segments.size()) ? 0.15 : 0.0;
      const double break_penalty = (seg.reason == "same_segment") ? 0.0 : 0.10;
      seg.score = stable_span ? (seg.span + 0.20 * recency + latest_bonus - break_penalty)
                              : (-10.0 + seg.span);
      if (seg.score > best_score) {
        best_score = seg.score;
        selected = idx;
      }
    }

    if (!segments.empty()) {
      const Segment& seg = segments[selected];
      source = std::vector<HistoryPoint2D>(source.begin() + seg.begin, source.begin() + seg.end);
      if (selected + 1 < segments.size()) {
        reason = "score_previous_stable_segment";
        if (debug != NULL) {
          debug->raw_clip_collapse_prevented = true;
          debug->raw_preserved_by_history_tail = true;
        }
      } else {
        reason = seg.reason;
      }
    }
    if (debug != NULL) {
      debug->segment_switch_reason = reason;
      debug->raw_heading_jump_count =
          (!segments.empty() && segments[selected].span >= min_stable_span) ? 0 : heading_jump_count;
      debug->raw_segment_count = static_cast<int>(segments.size());
      debug->raw_selected_segment_score = !segments.empty() ? segments[selected].score : 0.0;
      debug->raw_selected_segment_reason = reason;
      if (!source.empty()) {
        debug->anchor_epoch_id = source.back().anchor_epoch_id;
      }
    }
    return source;
  }

  std::vector<LocalPathPoint> selectContinuousLocalSuffix(const std::vector<LocalPathPoint>& local,
                                                          TrackerDebugState* debug) const {
    if (local.size() < 2) {
      return local;
    }
    struct Segment {
      size_t begin = 0;
      size_t end = 0;
      std::string reason = "same_segment";
      double span = 0.0;
      double score = 0.0;
    };
    std::vector<Segment> segments;
    size_t segment_begin = 0;
    int heading_jump_count = 0;
    std::string reason = debug != NULL ? debug->segment_switch_reason : "same_segment";
    for (size_t i = 1; i < local.size(); ++i) {
      const double d = pointDistance(local[i - 1].x, local[i - 1].y, local[i].x, local[i].y);
      bool broken = false;
      std::string break_reason;
      if (d > cfg_.raw_output_segment_gap) {
        broken = true;
        break_reason = "local_gap";
      } else if (i >= 2) {
        const double d0 =
            pointDistance(local[i - 2].x, local[i - 2].y, local[i - 1].x, local[i - 1].y);
        if (d0 >= cfg_.path_heading_recompute_min_ds && d >= cfg_.path_heading_recompute_min_ds) {
          const double yaw0 =
              std::atan2(local[i - 1].y - local[i - 2].y, local[i - 1].x - local[i - 2].x);
          const double yaw1 =
              std::atan2(local[i].y - local[i - 1].y, local[i].x - local[i - 1].x);
          const double dyaw = std::fabs(wrapAngle(yaw1 - yaw0));
          if (dyaw > cfg_.raw_output_heading_flip_gate) {
            broken = true;
            break_reason = "local_curvature_impulse";
            ++heading_jump_count;
          }
        }
      }
      if (broken) {
        Segment seg;
        seg.begin = segment_begin;
        seg.end = i;
        seg.reason = reason;
        seg.span = localSpan(local, seg.begin, seg.end);
        segments.push_back(seg);
        segment_begin = i;
        reason = break_reason;
      }
    }
    Segment last_seg;
    last_seg.begin = segment_begin;
    last_seg.end = local.size();
    last_seg.reason = reason;
    last_seg.span = localSpan(local, last_seg.begin, last_seg.end);
    segments.push_back(last_seg);

    const double min_stable_span =
        std::max(cfg_.raw_output_min_stable_span, cfg_.raw_min_span_for_publish);
    size_t selected = segments.empty() ? 0 : segments.size() - 1;
    double best_score = -std::numeric_limits<double>::infinity();
    for (size_t idx = 0; idx < segments.size(); ++idx) {
      Segment& seg = segments[idx];
      const bool stable_span = seg.span >= min_stable_span;
      const double recency = segments.empty() ? 0.0 : static_cast<double>(seg.end) /
                                                    static_cast<double>(std::max<size_t>(1, local.size()));
      const double latest_bonus = (idx + 1 == segments.size()) ? 0.15 : 0.0;
      const double break_penalty = (seg.reason == "same_segment") ? 0.0 : 0.10;
      seg.score = stable_span ? (seg.span + 0.20 * recency + latest_bonus - break_penalty)
                              : (-10.0 + seg.span);
      if (seg.score > best_score) {
        best_score = seg.score;
        selected = idx;
      }
    }

    const Segment& selected_seg = segments[selected];
    std::vector<LocalPathPoint> out(local.begin() + selected_seg.begin,
                                    local.begin() + selected_seg.end);
    if (selected + 1 < segments.size()) {
      reason = "score_previous_stable_local_segment";
      if (debug != NULL) {
        debug->raw_clip_collapse_prevented = true;
        debug->raw_preserved_by_history_tail = true;
      }
    } else {
      reason = selected_seg.reason;
    }
    if (out.size() >= 2) {
      recomputeLocalHeading(out);
    }
    if (debug != NULL) {
      debug->segment_switch_reason = reason;
      debug->raw_heading_jump_count +=
          (selected_seg.span >= min_stable_span) ? 0 : heading_jump_count;
      debug->raw_segment_count = std::max(debug->raw_segment_count, static_cast<int>(segments.size()));
      debug->raw_selected_segment_score = selected_seg.score;
      debug->raw_selected_segment_reason = reason;
      if (selected_seg.begin > 0) {
        debug->raw_clip_collapse_prevented = true;
      }
    }
    return out;
  }

  std::vector<RawTrailPoint> buildRawTrail(const std::vector<HistoryPoint2D>& world_history,
                                           TrackerDebugState* debug) const {
    if (world_history.empty()) {
      if (debug != NULL) {
        debug->raw_reason = "history_empty";
        debug->raw_span = 0.0;
      }
      return {};
    }

    std::vector<HistoryPoint2D> world_source = selectContinuousWorldSuffix(world_history, debug);
    std::vector<HistoryPoint2D> world_pts = removeNearDuplicatePoints(world_source, 0.01);
    if (world_pts.size() >= 2) {
      recomputeHeading(world_pts);
      if (!cfg_.raw_preserve_history_geometry) {
        world_pts =
            smoothWorldSplineLike(world_pts, cfg_.raw_spline_passes, cfg_.raw_spline_alpha, cfg_.raw_tail_weight_scale);
        world_pts = fitWorldTrajectoryHermite(world_pts, cfg_.raw_fit_ds);
      }
      world_pts = resampleWorldByArcLength(world_pts, cfg_.raw_resample_ds);
      recomputeHeading(world_pts);
    }
    offsetWorldPathAlongHeading(&world_pts, cfg_.lead_center_to_path_ref_offset);
    const std::vector<RawTrailPoint> raw_trail = makeRawTrailPoints(world_pts);
    if (debug != NULL) {
      debug->raw_span = raw_trail.empty() ? 0.0 : raw_trail.back().s;
      debug->raw_reason = raw_trail.empty() ? "history_empty" : "raw_ready";
    }
    return raw_trail;
  }

  std::vector<LocalPathPoint> buildRawPath(const std::vector<HistoryPoint2D>& world_history,
                                           const EgoPose2D& ego_now,
                                           bool lead_endpoint_valid,
                                           double lead_endpoint_x,
                                           double lead_endpoint_y,
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
    if (local.size() >= 2) {
      recomputeLocalHeading(local);
    }
    local = selectContinuousLocalSuffix(local, debug);
    if (debug != NULL) {
      debug->raw_world_points = world_history.size();
      debug->raw_local_points_before_clip = local.size();
    }
    local = clipLocalPathToEgoProgress(local,
                                       lead_endpoint_valid,
                                       lead_endpoint_x,
                                       lead_endpoint_y,
                                       ego_now.stamp,
                                       debug);
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
    if (!trimmed.empty()) {
      double front_jump = 0.0;
      if (last_raw_front_valid_) {
        front_jump = pointDistance(last_raw_front_x_, last_raw_front_y_,
                                   trimmed.front().x, trimmed.front().y);
      }
      last_raw_front_x_ = trimmed.front().x;
      last_raw_front_y_ = trimmed.front().y;
      last_raw_front_valid_ = true;
      if (debug != NULL) {
        debug->raw_front_jump = front_jump;
        if (front_jump > cfg_.raw_front_jump_gate) {
          debug->reference_control_valid = false;
          if (debug->segment_switch_reason == "same_segment" ||
              debug->segment_switch_reason == "none") {
            debug->segment_switch_reason = "raw_front_jump";
          }
        }
      }
    } else {
      last_raw_front_valid_ = false;
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
                                                 bool lead_endpoint_valid,
                                                 double lead_endpoint_x,
                                                 double lead_endpoint_y,
                                                 TrackerDebugState* debug) {
    std::vector<LocalPathPoint> raw_for_ref = raw;
    if (raw_trail.empty()) {
      last_join_valid_ = false;
      std::vector<LocalPathPoint> short_ref =
          buildShortEndpointReference(raw_trail,
                                      ego_now,
                                      lead_endpoint_valid,
                                      lead_endpoint_x,
                                      lead_endpoint_y);
      if (!short_ref.empty()) {
        if (debug != NULL) {
          debug->reference_reason = "reference_short_live_endpoint";
          debug->reference_span = polylineLengthLocal(short_ref);
          debug->reference_join_mode = "short_live_endpoint";
          debug->reference_join_step_limited = false;
          debug->reference_connector_points = static_cast<int>(short_ref.size());
          debug->reference_degraded_mode = true;
        }
        return short_ref;
      }
      if (debug != NULL) {
        debug->reference_reason = "raw_empty";
        debug->reference_span = 0.0;
      }
      return {};
    }

    if (raw_for_ref.empty() && cfg_.reference_short_endpoint_only_when_confirmed_empty) {
      std::vector<HistoryPoint2D> history_tail;
      history_tail.reserve(raw_trail.size());
      for (size_t i = 0; i < raw_trail.size(); ++i) {
        HistoryPoint2D pt;
        pt.stamp = raw_trail[i].stamp;
        pt.x = raw_trail[i].x;
        pt.y = raw_trail[i].y;
        pt.heading = raw_trail[i].yaw;
        pt.curvature = raw_trail[i].kappa;
        history_tail.push_back(pt);
      }
      raw_for_ref = transformWorldPathToLocal(history_tail, ego_now);
      if (raw_for_ref.size() >= 2) {
        recomputeLocalHeading(raw_for_ref);
        raw_for_ref = selectContinuousLocalSuffix(raw_for_ref, debug);
      }
      std::vector<LocalPathPoint> trimmed_ref;
      trimmed_ref.reserve(raw_for_ref.size());
      for (size_t i = 0; i < raw_for_ref.size(); ++i) {
        if (raw_for_ref[i].x < -cfg_.raw_keep_behind_x) {
          continue;
        }
        if (hypot2(raw_for_ref[i].x, raw_for_ref[i].y) > cfg_.raw_max_range) {
          continue;
        }
        trimmed_ref.push_back(raw_for_ref[i]);
        if (static_cast<int>(trimmed_ref.size()) >= cfg_.raw_max_points) {
          break;
        }
      }
      raw_for_ref.swap(trimmed_ref);
      if (raw_for_ref.size() >= 2) {
        recomputeLocalHeading(raw_for_ref);
        if (debug != NULL) {
          debug->reference_degraded_mode = false;
          debug->raw_preserved_by_history_tail = true;
        }
      }
    }

    if (raw_for_ref.empty()) {
      last_join_valid_ = false;
      const bool endpoint_quality_blocks_live =
          debug != NULL &&
          (debug->endpoint_quality == "history_bad_no_expand" ||
           debug->endpoint_quality == "low_conf_no_expand");
      const bool live_projection_far =
          debug != NULL &&
          (debug->endpoint_quality == "projection_far" ||
           debug->endpoint_quality == "stale_endpoint_short_reference");
      std::vector<LocalPathPoint> short_ref =
          buildShortEndpointReference(raw_trail,
                                      ego_now,
                                      lead_endpoint_valid && !endpoint_quality_blocks_live,
                                      lead_endpoint_x,
                                      lead_endpoint_y);
      if (!short_ref.empty()) {
        if (debug != NULL) {
          debug->reference_reason = live_projection_far || raw_trail.empty()
                                        ? "reference_short_live_endpoint"
                                        : "reference_short_confirmed_endpoint";
          debug->reference_span = polylineLengthLocal(short_ref);
          debug->reference_join_mode =
              endpoint_quality_blocks_live ? "short_history_endpoint" : "short_lead_endpoint";
          debug->reference_join_step_limited = false;
          debug->reference_connector_points = static_cast<int>(short_ref.size());
          debug->reference_degraded_mode = live_projection_far || endpoint_quality_blocks_live;
        }
        return short_ref;
      }
      if (debug != NULL) {
        debug->reference_reason = "reference_clipped_empty_no_endpoint";
        debug->reference_span = 0.0;
      }
      return {};
    }

    const double raw_span = polylineLengthLocal(raw_for_ref);
    if (cfg_.reference_follow_raw_only && !raw_for_ref.empty() &&
        pointDistance(0.0, 0.0, raw_for_ref.front().x, raw_for_ref.front().y) <= cfg_.reference_direct_attach_dist) {
      if (debug != NULL) {
        debug->reference_reason = "reference_direct_raw_near_origin";
        debug->reference_span = raw_span;
      }
      return raw_for_ref;
    }

    std::vector<RefPoint> ref_profile = makeRefPoints(raw_for_ref, 0.5);
    recomputeRefGeometry(&ref_profile);
    if (ref_profile.empty()) {
      if (debug != NULL) {
        debug->reference_reason = "reference_profile_too_short";
        debug->reference_span = 0.0;
      }
      return {};
    }

    if (raw_for_ref.empty() && ref_profile.size() == 1) {
      if (debug != NULL) {
        debug->reference_reason = "reference_waiting_confirmed_history";
        debug->reference_span = 0.0;
      }
      return {};
    }

    if (static_cast<int>(raw_for_ref.size()) < cfg_.reference_min_points && ref_profile.size() < 2) {
      if (debug != NULL) {
        debug->reference_reason = "reference_insufficient_points";
        debug->reference_span = raw_span;
      }
      return {};
    }
    if (!raw_for_ref.empty() && raw_span < cfg_.reference_min_span && ref_profile.size() < 2) {
      if (debug != NULL) {
        debug->reference_reason = "reference_span_too_short";
        debug->reference_span = raw_span;
      }
      return {};
    }

    const JoinChoice join = chooseJoinIndex(ref_profile);
    const RefPoint& join_ref = join.join_point;

    std::vector<RefPoint> combined;
    RefPoint origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.yaw = 0.0;
    origin.kappa = 0.0;
    origin.s = 0.0;
    origin.t = 0.0;
    origin.v_ref = std::min(0.3, join_ref.v_ref);
    origin.a_ref = 0.0;
    combined.push_back(origin);

    size_t splice_begin_index = 0;
    int connector_points = 0;
    if (hypot2(join_ref.x, join_ref.y) > cfg_.reference_direct_attach_dist) {
      const std::vector<RefPoint> connector = buildConnector(join_ref);
      if (connector.empty()) {
        combined.push_back(join_ref);
        connector_points = 2;
      } else {
        for (size_t i = 1; i < connector.size(); ++i) {
          combined.push_back(connector[i]);
        }
        connector_points = static_cast<int>(connector.size());
      }
      splice_begin_index = combined.empty() ? 0 : combined.size() - 1;
    } else {
      combined.push_back(join_ref);
      connector_points = 2;
      splice_begin_index = combined.size() - 1;
    }
    for (size_t i = std::min(join.suffix_index, ref_profile.size()); i < ref_profile.size(); ++i) {
      if (!combined.empty() &&
          pointDistance(combined.back().x, combined.back().y, ref_profile[i].x, ref_profile[i].y) < 0.02) {
        combined.back() = ref_profile[i];
      } else {
        combined.push_back(ref_profile[i]);
      }
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

    std::vector<LocalPathPoint> final_local = makeLocalPathFromRefPoints(combined);
    int reference_heading_jumps = 0;
    for (size_t i = 1; i < final_local.size(); ++i) {
      const double d = pointDistance(final_local[i - 1].x, final_local[i - 1].y,
                                     final_local[i].x, final_local[i].y);
      const double dyaw = std::fabs(wrapAngle(final_local[i].yaw - final_local[i - 1].yaw));
      if (d < cfg_.path_heading_recompute_min_ds && dyaw > cfg_.raw_output_heading_flip_gate) {
        ++reference_heading_jumps;
      }
    }
    last_join_s_ = join_ref.s;
    last_join_valid_ = true;
    if (debug != NULL) {
      debug->reference_reason = "reference_ready";
      debug->reference_span = polylineLengthLocal(final_local);
      debug->reference_degraded_mode = false;
      debug->reference_join_s = join_ref.s;
      debug->reference_join_mode = last_join_mode_debug_;
      debug->reference_join_step_limited = last_join_step_limited_flag_;
      debug->reference_connector_points = connector_points;
      debug->reference_continuity_limited =
          cfg_.reference_temporal_consistency && last_join_step_limited_flag_;
      debug->reference_control_valid =
          debug->reference_control_valid && reference_heading_jumps == 0;
      debug->raw_heading_jump_count += reference_heading_jumps;
    }

    return final_local;
  }

 private:
  struct JoinChoice {
    bool valid = false;
    size_t suffix_index = 0;
    RefPoint join_point;
  };

  struct PathProjection {
    bool valid = false;
    size_t seg = 0;
    double t = 0.0;
    double s = 0.0;
    double dist2 = 0.0;
    LocalPathPoint point;
  };

  PathProjection projectPointToLocalPath(const std::vector<LocalPathPoint>& local,
                                         double qx,
                                         double qy) const {
    PathProjection out;
    if (local.size() < 2) {
      return out;
    }

    double accum_s = 0.0;
    out.dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < local.size(); ++i) {
      const double ax = local[i].x;
      const double ay = local[i].y;
      const double bx = local[i + 1].x;
      const double by = local[i + 1].y;
      const double dx = bx - ax;
      const double dy = by - ay;
      const double len = hypot2(dx, dy);
      const double len2 = std::max(1e-6, dx * dx + dy * dy);
      const double t = clampValue(((qx - ax) * dx + (qy - ay) * dy) / len2, 0.0, 1.0);
      const double px = ax + t * dx;
      const double py = ay + t * dy;
      const double d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy);
      if (d2 < out.dist2) {
        out.valid = true;
        out.seg = i;
        out.t = t;
        out.s = accum_s + t * len;
        out.dist2 = d2;
        out.point.x = px;
        out.point.y = py;
        out.point.yaw = wrapAngle(local[i].yaw + t * wrapAngle(local[i + 1].yaw - local[i].yaw));
        out.point.curvature = (1.0 - t) * local[i].curvature + t * local[i + 1].curvature;
      }
      accum_s += len;
    }
    return out;
  }

  PathProjection projectPointToLocalPathWindowed(const std::vector<LocalPathPoint>& local,
                                                 double qx,
                                                 double qy,
                                                 bool use_window,
                                                 double center_s,
                                                 double window_radius) const {
    if (!use_window || window_radius <= 1e-4 || local.size() < 2) {
      return projectPointToLocalPath(local, qx, qy);
    }

    const double s_min = std::max(0.0, center_s - window_radius);
    const double s_max = std::min(polylineLengthLocal(local), center_s + window_radius);
    PathProjection out;
    double accum_s = 0.0;
    out.dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < local.size(); ++i) {
      const double ax = local[i].x;
      const double ay = local[i].y;
      const double bx = local[i + 1].x;
      const double by = local[i + 1].y;
      const double dx = bx - ax;
      const double dy = by - ay;
      const double len = hypot2(dx, dy);
      const double seg_s0 = accum_s;
      const double seg_s1 = accum_s + len;
      accum_s += len;
      if (seg_s1 + 1e-6 < s_min || seg_s0 - 1e-6 > s_max) {
        continue;
      }

      const double len2 = std::max(1e-6, dx * dx + dy * dy);
      double t = clampValue(((qx - ax) * dx + (qy - ay) * dy) / len2, 0.0, 1.0);
      const double cand_s = seg_s0 + t * len;
      if (cand_s < s_min && len > 1e-6) {
        t = clampValue((s_min - seg_s0) / len, 0.0, 1.0);
      } else if (cand_s > s_max && len > 1e-6) {
        t = clampValue((s_max - seg_s0) / len, 0.0, 1.0);
      }
      const double px = ax + t * dx;
      const double py = ay + t * dy;
      const double d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy);
      if (d2 < out.dist2) {
        out.valid = true;
        out.seg = i;
        out.t = t;
        out.s = seg_s0 + t * len;
        out.dist2 = d2;
        out.point.x = px;
        out.point.y = py;
        out.point.yaw = wrapAngle(local[i].yaw + t * wrapAngle(local[i + 1].yaw - local[i].yaw));
        out.point.curvature = (1.0 - t) * local[i].curvature + t * local[i + 1].curvature;
      }
    }
    if (!out.valid) {
      return projectPointToLocalPath(local, qx, qy);
    }
    return out;
  }

  PathProjection projectSOnLocalPath(const std::vector<LocalPathPoint>& local, double query_s) const {
    PathProjection out;
    if (local.size() < 2) {
      return out;
    }
    const double total = polylineLengthLocal(local);
    const double target_s = clampValue(query_s, 0.0, total);
    double accum_s = 0.0;
    for (size_t i = 0; i + 1 < local.size(); ++i) {
      const double len = pointDistance(local[i].x, local[i].y, local[i + 1].x, local[i + 1].y);
      if (accum_s + len + 1e-6 >= target_s || i + 2 == local.size()) {
        const double t = len > 1e-6 ? clampValue((target_s - accum_s) / len, 0.0, 1.0) : 0.0;
        out.valid = true;
        out.seg = i;
        out.t = t;
        out.s = accum_s + t * len;
        out.dist2 = 0.0;
        out.point.x = local[i].x + t * (local[i + 1].x - local[i].x);
        out.point.y = local[i].y + t * (local[i + 1].y - local[i].y);
        out.point.yaw = wrapAngle(local[i].yaw + t * wrapAngle(local[i + 1].yaw - local[i].yaw));
        out.point.curvature = (1.0 - t) * local[i].curvature + t * local[i + 1].curvature;
        return out;
      }
      accum_s += len;
    }
    return out;
  }

  std::vector<LocalPathPoint> sliceLocalPathByProjection(const std::vector<LocalPathPoint>& local,
                                                         const PathProjection& start,
                                                         const PathProjection& end) const {
    std::vector<LocalPathPoint> clipped;
    if (!start.valid || !end.valid || local.size() < 2) {
      return clipped;
    }

    clipped.reserve(local.size());
    clipped.push_back(start.point);
    const double dup = std::max(1e-4, cfg_.raw_clip_duplicate_distance);
    for (size_t i = start.seg + 1; i <= std::min(end.seg, local.size() - 1); ++i) {
      if (pointDistance(clipped.back().x, clipped.back().y, local[i].x, local[i].y) < dup) {
        clipped.back() = local[i];
      } else {
        clipped.push_back(local[i]);
      }
    }
    if (pointDistance(clipped.back().x, clipped.back().y, end.point.x, end.point.y) < dup) {
      clipped.back() = end.point;
    } else {
      clipped.push_back(end.point);
    }
    if (clipped.size() >= 2) {
      recomputeLocalHeading(clipped);
    }
    return clipped;
  }

  std::vector<LocalPathPoint> clipLocalPathToEgoProgress(const std::vector<LocalPathPoint>& local,
                                                         bool lead_endpoint_valid,
                                                         double lead_endpoint_x,
                                                         double lead_endpoint_y,
                                                         const ros::Time& stamp,
                                                         TrackerDebugState* debug) const {
    if (!cfg_.raw_clip_to_ego_progress || local.size() < 2) {
      if (debug != NULL) {
        debug->clip_reason = cfg_.raw_clip_to_ego_progress ? "clip_path_too_short" : "clip_disabled";
        debug->clip_start_s = 0.0;
        debug->clip_end_s = polylineLengthLocal(local);
        debug->clip_start_source = "path_begin";
        debug->clip_end_source = cfg_.raw_clip_to_ego_progress ? "path_too_short" : "disabled";
        debug->clip_hold = false;
        debug->endpoint_proj_dist = 0.0;
        debug->endpoint_quality = cfg_.raw_clip_to_ego_progress ? "path_too_short" : "disabled";
      }
      return local;
    }

    const double total_s = polylineLengthLocal(local);
    const double min_gap = std::max(0.02, cfg_.raw_clip_duplicate_distance);
    PathProjection start =
        projectPointToLocalPathWindowed(local,
                                        0.0,
                                        0.0,
                                        last_clip_valid_,
                                        last_clip_start_s_,
                                        cfg_.raw_clip_start_search_window);
    if (!start.valid) {
      if (debug != NULL) {
        debug->clip_reason = "clip_start_projection_failed";
        debug->clip_start_s = 0.0;
        debug->clip_end_s = total_s;
        debug->clip_start_source = "invalid";
        debug->clip_end_source = "invalid";
        debug->clip_hold = false;
        debug->endpoint_proj_dist = 0.0;
        debug->endpoint_quality = "start_projection_failed";
      }
      return {};
    }
    const double start_proj_dist = std::sqrt(std::max(0.0, start.dist2));
    bool start_slew_limited = false;
    if (last_clip_valid_) {
      const double dt =
          (stamp.isValid() && last_clip_stamp_.isValid())
              ? std::max(0.0, (stamp - last_clip_stamp_).toSec())
              : 0.0;
      const double rate_step =
          dt > 1e-4 ? cfg_.raw_clip_start_max_s_rate * dt : cfg_.raw_clip_start_max_s_step;
      const double max_start_step =
          std::max(0.0, std::min(cfg_.raw_clip_start_max_s_step, rate_step));
      if (max_start_step > 1e-3 && std::fabs(start.s - last_clip_start_s_) > max_start_step) {
        const double limited_s =
            last_clip_start_s_ + (start.s > last_clip_start_s_ ? max_start_step : -max_start_step);
        PathProjection limited_start = projectSOnLocalPath(local, clampValue(limited_s, 0.0, total_s));
        if (limited_start.valid) {
          start = limited_start;
          start_slew_limited = true;
        }
      }
    }

    PathProjection desired_end;
    bool use_endpoint = cfg_.raw_clip_use_current_lead_endpoint &&
                        cfg_.raw_clip_end_follow_lead_endpoint &&
                        lead_endpoint_valid;
    if (debug != NULL) {
      debug->raw_clip_policy = cfg_.raw_spatial_window_mode;
      debug->ego_progress_s = start.s;
      debug->history_tail_s = total_s;
      debug->control_feasible_start_s = start.s;
    }
    if (use_endpoint) {
      const double endpoint_center_s =
          last_endpoint_s_valid_ ? last_endpoint_s_ :
          (last_clip_valid_ ? last_clip_end_s_ : total_s);
      desired_end =
          projectPointToLocalPathWindowed(local,
                                          lead_endpoint_x,
                                          lead_endpoint_y,
                                          last_endpoint_s_valid_ || last_clip_valid_,
                                          endpoint_center_s,
                                          cfg_.raw_clip_endpoint_search_window);
    } else {
      desired_end = projectSOnLocalPath(local, total_s);
    }
    const double endpoint_s_raw = desired_end.valid ? desired_end.s : 0.0;
    double endpoint_proj_dist =
        desired_end.valid ? std::sqrt(std::max(0.0, desired_end.dist2))
                          : std::numeric_limits<double>::infinity();
    const bool endpoint_far =
        desired_end.valid && endpoint_proj_dist > cfg_.raw_clip_endpoint_projection_gate;
    const bool low_endpoint_confidence =
        debug != NULL &&
        debug->lead_filter_confidence > 1e-6 &&
        debug->lead_filter_confidence < cfg_.raw_clip_min_endpoint_confidence;
    const double endpoint_confidence =
        debug != NULL ? debug->lead_filter_confidence : 1.0;
    double endpoint_hold_age = 0.0;
    bool endpoint_recapture = false;
    bool stale_endpoint_short_reference = false;
    std::string recapture_reason = "none";
    std::string endpoint_recapture_mode = "none";
    if (endpoint_far && lead_endpoint_valid && stamp.isValid() &&
        endpoint_confidence >= cfg_.endpoint_recapture_confidence_gate) {
      const bool endpoint_stable =
          last_endpoint_valid_ &&
          pointDistance(last_endpoint_x_, last_endpoint_y_, lead_endpoint_x, lead_endpoint_y) < 0.25;
      if (!endpoint_far_start_stamp_.isValid() || !endpoint_stable) {
        endpoint_far_start_stamp_ = stamp;
        endpoint_recapture_frames_ = 1;
      } else {
        endpoint_recapture_frames_ = std::min(endpoint_recapture_frames_ + 1, 1000000);
      }
      endpoint_hold_age = (stamp - endpoint_far_start_stamp_).toSec();
      stale_endpoint_short_reference =
          endpoint_proj_dist > cfg_.endpoint_recapture_projection_gate ||
          endpoint_hold_age > cfg_.endpoint_recapture_max_far_time;
      endpoint_recapture =
          endpoint_recapture_frames_ >= std::max(1, cfg_.endpoint_recapture_min_frames) &&
          endpoint_hold_age >= cfg_.raw_clip_projection_far_hold_max_time &&
          !stale_endpoint_short_reference;
      if (stale_endpoint_short_reference) {
        recapture_reason = "stable_projection_far_short_reference";
        endpoint_recapture_mode = "short_reference";
      } else {
        recapture_reason = endpoint_recapture ? "stable_projection_far_recapture"
                                              : "stable_projection_far_wait";
        endpoint_recapture_mode = endpoint_recapture ? "normal_clip_recapture" : "wait";
      }
    } else {
      if (!endpoint_far) {
        endpoint_far_start_stamp_ = ros::Time();
        endpoint_recapture_frames_ = 0;
      }
      recapture_reason = endpoint_far ? "projection_far_low_conf_or_missing" : "not_projection_far";
      endpoint_recapture_mode = endpoint_far ? "reject_far" : "near_path";
    }
    if (lead_endpoint_valid) {
      last_endpoint_x_ = lead_endpoint_x;
      last_endpoint_y_ = lead_endpoint_y;
      last_endpoint_valid_ = true;
    } else {
      last_endpoint_valid_ = false;
    }
    const bool bad_history_for_expansion =
        debug != NULL &&
        (debug->history_reason == "outlier_rejected" ||
         debug->history_reason == "tail_fast_rejected");
    std::string endpoint_quality = "ok";
    if (!lead_endpoint_valid) {
      endpoint_quality = "missing";
    } else if (!desired_end.valid) {
      endpoint_quality = "projection_failed";
    } else if (stale_endpoint_short_reference) {
      endpoint_quality = "stale_endpoint_short_reference";
    } else if (endpoint_far && endpoint_recapture) {
      endpoint_quality = "endpoint_recapture";
    } else if (endpoint_far) {
      endpoint_quality = "projection_far";
    } else if (bad_history_for_expansion) {
      endpoint_quality = "history_bad_no_expand";
    } else if (low_endpoint_confidence) {
      endpoint_quality = "low_conf_no_expand";
    }
    if (endpoint_far && !endpoint_recapture) {
      desired_end.valid = false;
    }

    std::string end_source = "lead_endpoint";
    std::string hold_reason = "none";
    bool clip_reorder_used = false;
    bool hold = false;
    bool min_span_applied = false;
    bool collapse_prevented = false;
    if (desired_end.valid &&
        cfg_.raw_clip_allow_endpoint_reorder &&
        (endpoint_quality == "ok" || endpoint_quality == "endpoint_recapture") &&
        desired_end.s < start.s + min_gap) {
      const double reordered_start_s = clampValue(desired_end.s - min_gap, 0.0, total_s);
      PathProjection reordered_start = projectSOnLocalPath(local, reordered_start_s);
      if (reordered_start.valid && desired_end.s >= reordered_start.s + 0.5 * min_gap) {
        start = reordered_start;
        clip_reorder_used = true;
      }
    }

    if (stale_endpoint_short_reference) {
      desired_end = projectSOnLocalPath(local, total_s);
      if (desired_end.valid) {
        end_source = "history_tail_stale_endpoint";
        hold_reason = "stale_endpoint_use_history_tail";
        use_endpoint = false;
        if (debug != NULL) {
          debug->raw_preserved_by_history_tail = true;
        }
        if (debug != NULL) {
          debug->reference_degraded_mode = true;
        }
      } else {
        if (debug != NULL) {
          debug->clip_start_s = start.s;
          debug->clip_end_s = start.s;
          debug->clip_reason = "clip_stale_endpoint_short_reference";
          debug->clip_start_source = "ego_origin";
          debug->clip_end_source = "stale_endpoint_short_reference";
          debug->clip_hold = false;
          debug->endpoint_proj_dist =
              std::isfinite(endpoint_proj_dist) ? endpoint_proj_dist : -1.0;
          debug->endpoint_quality = endpoint_quality;
          debug->endpoint_recapture_frames = endpoint_recapture_frames_;
          debug->endpoint_hold_age = endpoint_hold_age;
          debug->recapture_reason = recapture_reason;
          debug->clip_start_proj_dist = start_proj_dist;
          debug->clip_endpoint_s_raw = endpoint_s_raw;
          debug->clip_endpoint_s_filtered = 0.0;
          debug->clip_reorder_used = clip_reorder_used;
          debug->clip_hold_reason = "stale_endpoint_short_reference";
          debug->clip_stale_age = endpoint_hold_age;
          debug->endpoint_recapture_mode = endpoint_recapture_mode;
        }
        last_clip_valid_ = false;
        last_endpoint_s_valid_ = false;
        return {};
      }
    }

    if (!desired_end.valid || desired_end.s < start.s + min_gap) {
      use_endpoint = false;
      const bool valid_endpoint_behind =
          desired_end.valid &&
          (endpoint_quality == "ok" || endpoint_quality == "endpoint_recapture");
      if (valid_endpoint_behind) {
        if (cfg_.raw_clip_keep_history_tail_when_endpoint_behind) {
          desired_end = projectSOnLocalPath(local, total_s);
          end_source = "history_tail_endpoint_behind";
          hold_reason = "endpoint_behind_preserve_history_tail";
          if (debug != NULL) {
            debug->raw_preserved_by_history_tail = true;
          }
        } else {
          if (debug != NULL) {
            debug->clip_start_s = start.s;
            debug->clip_end_s = desired_end.s;
            debug->clip_reason = "clip_endpoint_valid_but_behind";
            debug->clip_start_source =
                clip_reorder_used ? "reordered_ego_origin"
                                  : (start_slew_limited ? "ego_origin_slew_limited" : "ego_origin");
            debug->clip_end_source = "valid_endpoint_behind";
            debug->clip_hold = false;
            debug->endpoint_proj_dist =
                std::isfinite(endpoint_proj_dist) ? endpoint_proj_dist : -1.0;
            debug->endpoint_quality = endpoint_quality;
            debug->endpoint_recapture_frames = endpoint_recapture_frames_;
            debug->endpoint_hold_age = endpoint_hold_age;
            debug->recapture_reason = recapture_reason;
            debug->clip_start_proj_dist = start_proj_dist;
            debug->clip_endpoint_s_raw = endpoint_s_raw;
            debug->clip_endpoint_s_filtered = desired_end.s;
            debug->clip_reorder_used = clip_reorder_used;
            debug->clip_hold_reason = "valid_endpoint_behind_short_reference";
            debug->clip_stale_age = endpoint_hold_age;
            debug->endpoint_recapture_mode = endpoint_recapture_mode;
          }
          last_clip_valid_ = false;
          last_endpoint_s_ = desired_end.s;
          last_endpoint_s_valid_ = true;
          return {};
        }
      }

      const double stale_hold_limit =
          std::min(cfg_.raw_clip_stale_max_time,
                   lead_endpoint_valid ? cfg_.raw_clip_valid_endpoint_hold_max_time
                                       : cfg_.raw_clip_missing_endpoint_hold_time);
      const bool can_hold =
          cfg_.raw_clip_endpoint_hysteresis &&
          last_clip_valid_ &&
          stamp.isValid() &&
          last_real_clip_stamp_.isValid() &&
          (stamp - last_real_clip_stamp_).toSec() <= stale_hold_limit;
      if (can_hold) {
        desired_end = projectSOnLocalPath(local, clampValue(last_clip_end_s_, start.s + min_gap, total_s));
        end_source = "held_endpoint";
        hold_reason = lead_endpoint_valid ? "invalid_endpoint_short_hold" : "missing_endpoint_short_hold";
        hold = true;
      } else if (!lead_endpoint_valid) {
        desired_end = projectSOnLocalPath(local, total_s);
        end_source = "history_tail_no_endpoint";
        hold_reason = "missing_endpoint_history_tail";
        if (debug != NULL) {
          debug->reference_degraded_mode = true;
        }
      } else if (cfg_.raw_clip_endpoint_hysteresis) {
        if (debug != NULL) {
          debug->clip_start_s = start.s;
          debug->clip_end_s = start.s;
          debug->clip_reason = lead_endpoint_valid ? "clip_endpoint_behind_or_invalid" : "clip_no_endpoint";
          debug->clip_start_source = start_slew_limited ? "ego_origin_slew_limited" : "ego_origin";
          debug->clip_end_source = lead_endpoint_valid ? "endpoint_invalid" : "endpoint_missing";
          debug->clip_hold = false;
          debug->endpoint_proj_dist =
              std::isfinite(endpoint_proj_dist) ? endpoint_proj_dist : -1.0;
          debug->endpoint_quality = endpoint_quality;
          debug->endpoint_recapture_frames = endpoint_recapture_frames_;
          debug->endpoint_hold_age = endpoint_hold_age;
          debug->recapture_reason = recapture_reason;
          debug->clip_start_proj_dist = start_proj_dist;
          debug->clip_endpoint_s_raw = endpoint_s_raw;
          debug->clip_endpoint_s_filtered = desired_end.valid ? desired_end.s : 0.0;
          debug->clip_reorder_used = clip_reorder_used;
          debug->clip_hold_reason = lead_endpoint_valid ? "endpoint_invalid_no_hold" : "endpoint_missing_no_hold";
          debug->clip_stale_age = endpoint_hold_age;
          debug->endpoint_recapture_mode = endpoint_recapture_mode;
        }
        last_clip_valid_ = false;
        return {};
      } else {
        desired_end = projectSOnLocalPath(local, total_s);
        end_source = "history_tail";
        hold_reason = "clip_disabled_history_tail";
      }
    } else if (cfg_.raw_clip_endpoint_hysteresis && last_clip_valid_) {
      if ((bad_history_for_expansion || low_endpoint_confidence) &&
          desired_end.s > last_clip_end_s_) {
        desired_end = projectSOnLocalPath(local, clampValue(last_clip_end_s_, start.s + min_gap, total_s));
        end_source = bad_history_for_expansion ? "quality_hold_history_bad" : "quality_hold_low_conf";
        hold_reason = bad_history_for_expansion ? "history_bad_no_expand" : "low_conf_no_expand";
        hold = true;
      }

      const double dt =
          (stamp.isValid() && last_real_clip_stamp_.isValid())
              ? std::max(0.0, (stamp - last_real_clip_stamp_).toSec())
              : 0.0;
      const double rate_step =
          dt > 1e-4 ? cfg_.raw_clip_max_s_rate * dt : cfg_.raw_clip_max_s_step;
      const double max_jump =
          std::max(0.0,
                   std::min(std::max(0.0, cfg_.raw_clip_max_s_jump),
                            std::min(std::max(0.0, cfg_.raw_clip_max_s_step),
                                     std::max(0.0, rate_step))));
      if (desired_end.valid && max_jump > 1e-3 &&
          std::fabs(desired_end.s - last_clip_end_s_) > max_jump) {
        const double limited_s =
            last_clip_end_s_ + (desired_end.s > last_clip_end_s_ ? max_jump : -max_jump);
        desired_end = projectSOnLocalPath(local, clampValue(limited_s, start.s + min_gap, total_s));
        end_source = "endpoint_slew_limited";
        hold_reason = "endpoint_slew_limited";
        hold = true;
      }
    }

    if (desired_end.valid) {
      const double min_visible_span = std::max(0.0, cfg_.raw_clip_min_visible_span);
      const double current_span = desired_end.s - start.s;
      if (min_visible_span > min_gap &&
          total_s >= min_visible_span &&
          current_span < min_visible_span &&
          (endpoint_quality == "ok" || endpoint_quality == "endpoint_recapture" ||
           endpoint_quality == "missing")) {
        const double protected_start_s =
            clampValue(desired_end.s - min_visible_span, 0.0, total_s);
        PathProjection protected_start = projectSOnLocalPath(local, protected_start_s);
        if (protected_start.valid && protected_start.s + min_gap < desired_end.s) {
          start = protected_start;
          min_span_applied = true;
          collapse_prevented = current_span < cfg_.raw_min_span_for_publish;
          if (hold_reason == "none") {
            hold_reason = "min_visible_span";
          }
        }
      }
    }

    std::vector<LocalPathPoint> clipped = sliceLocalPathByProjection(local, start, desired_end);
    if (debug != NULL) {
      debug->clip_start_s = start.valid ? start.s : 0.0;
      debug->clip_end_s = desired_end.valid ? desired_end.s : 0.0;
      debug->clip_reason =
          use_endpoint ? "clip_ego_to_lead_endpoint"
                       : (end_source == "history_tail_endpoint_behind"
                              ? "clip_ego_to_history_tail_endpoint_behind"
                              : (hold ? "clip_endpoint_hold" : "clip_ego_to_history_tail"));
      debug->clip_start_source = start_slew_limited ? "ego_origin_slew_limited" : "ego_origin";
      debug->clip_end_source = end_source;
      debug->clip_hold = hold;
      debug->endpoint_proj_dist =
          std::isfinite(endpoint_proj_dist) ? endpoint_proj_dist : -1.0;
      debug->endpoint_quality = endpoint_quality;
      debug->endpoint_recapture_frames = endpoint_recapture_frames_;
      debug->endpoint_hold_age = endpoint_hold_age;
      debug->recapture_reason = recapture_reason;
      debug->clip_start_proj_dist = start_proj_dist;
      debug->clip_endpoint_s_raw = endpoint_s_raw;
      debug->clip_endpoint_s_filtered = desired_end.valid ? desired_end.s : 0.0;
      debug->lead_endpoint_s = endpoint_s_raw;
      debug->clip_end_limited_by_endpoint = use_endpoint;
      debug->clip_reorder_used = clip_reorder_used;
      debug->clip_hold_reason = (hold || min_span_applied) ? hold_reason : "none";
      debug->raw_clip_min_span_applied = min_span_applied;
      debug->raw_clip_collapse_prevented = collapse_prevented;
      debug->clip_stale_age = hold ? ((stamp.isValid() && last_real_clip_stamp_.isValid())
                                        ? std::max(0.0, (stamp - last_real_clip_stamp_).toSec())
                                        : 0.0)
                                   : 0.0;
      debug->endpoint_recapture_mode = endpoint_recapture_mode;
    }
    if (clipped.empty()) {
      last_clip_valid_ = false;
      return {};
    }
    last_clip_valid_ = true;
    last_clip_start_s_ = start.s;
    last_clip_end_s_ = desired_end.s;
    last_clip_stamp_ = stamp;
    if (!hold && use_endpoint && endpoint_quality == "ok") {
      last_real_clip_stamp_ = stamp;
    }
    last_endpoint_s_ = desired_end.s;
    last_endpoint_s_valid_ = use_endpoint || endpoint_quality == "ok" ||
                             endpoint_quality == "endpoint_recapture";
    return clipped;
  }

  std::vector<LocalPathPoint> buildShortEndpointReference(const std::vector<RawTrailPoint>& raw_trail,
                                                          const EgoPose2D& ego_now,
                                                          bool lead_endpoint_valid,
                                                          double lead_endpoint_x,
                                                          double lead_endpoint_y) const {
    if (!cfg_.reference_publish_short_endpoint) {
      return {};
    }
    if (!lead_endpoint_valid &&
        static_cast<int>(raw_trail.size()) < std::max(1, cfg_.reference_short_endpoint_min_confirmed)) {
      return {};
    }

    LocalPathPoint endpoint;
    if (lead_endpoint_valid) {
      endpoint.x = lead_endpoint_x;
      endpoint.y = lead_endpoint_y;
    } else {
      const RawTrailPoint& tail = raw_trail.back();
      HistoryPoint2D tail_world;
      tail_world.x = tail.x;
      tail_world.y = tail.y;
      tail_world.heading = tail.yaw;
      std::vector<HistoryPoint2D> one;
      one.push_back(tail_world);
      std::vector<LocalPathPoint> local_tail = transformWorldPathToLocal(one, ego_now);
      if (local_tail.empty()) {
        return {};
      }
      endpoint = local_tail.front();
    }
    if (hypot2(endpoint.x, endpoint.y) < 0.05 || endpoint.x < -cfg_.raw_keep_behind_x) {
      return {};
    }
    endpoint.yaw = std::atan2(endpoint.y, endpoint.x);
    endpoint.curvature = 0.0;

    std::vector<LocalPathPoint> out;
    LocalPathPoint origin;
    origin.yaw = endpoint.yaw;
    out.push_back(origin);
    if (hypot2(endpoint.x, endpoint.y) > cfg_.reference_connector_resolution) {
      const double dist = hypot2(endpoint.x, endpoint.y);
      const int steps = std::max(1, static_cast<int>(std::ceil(dist / std::max(0.03, cfg_.reference_connector_resolution))));
      for (int i = 1; i <= steps; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(steps);
        LocalPathPoint pt;
        pt.x = u * endpoint.x;
        pt.y = u * endpoint.y;
        pt.yaw = endpoint.yaw;
        pt.curvature = 0.0;
        out.push_back(pt);
      }
    } else {
      out.push_back(endpoint);
    }
    recomputeLocalHeading(out);
    return out;
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

  JoinChoice chooseJoinIndex(const std::vector<RefPoint>& ref_profile) {
    JoinChoice best;
    if (ref_profile.empty()) {
      return best;
    }
    if (ref_profile.size() == 1) {
      best.valid = true;
      best.suffix_index = 0;
      best.join_point = ref_profile.front();
      last_join_mode_debug_ = cfg_.reference_join_mode;
      last_join_step_limited_flag_ = false;
      return best;
    }

    last_join_mode_debug_ = cfg_.reference_join_mode;
    last_join_step_limited_flag_ = false;
    if (cfg_.reference_join_mode == "front_anchored") {
      const double s0 = ref_profile.front().s;
      const double s1 = ref_profile.back().s;
      double target_s = clampValue(s0 + cfg_.reference_join_front_lookahead, s0, s1);
      if (last_join_valid_ && cfg_.reference_join_max_step > 1e-4) {
        const double last_s = clampValue(last_join_s_, s0, s1);
        const double delta = target_s - last_s;
        if (std::fabs(delta) > cfg_.reference_join_max_step) {
          target_s = last_s + (delta > 0.0 ? cfg_.reference_join_max_step : -cfg_.reference_join_max_step);
          last_join_step_limited_flag_ = true;
        }
      }

      size_t seg = 0;
      for (size_t i = 0; i + 1 < ref_profile.size(); ++i) {
        if (ref_profile[i + 1].s + 1e-6 >= target_s) {
          seg = i;
          break;
        }
        seg = i;
      }
      const RefPoint& a = ref_profile[seg];
      const RefPoint& b = ref_profile[std::min(seg + 1, ref_profile.size() - 1)];
      const double denom = std::max(1e-6, b.s - a.s);
      const double t = clampValue((target_s - a.s) / denom, 0.0, 1.0);
      RefPoint cand;
      cand.x = a.x + t * (b.x - a.x);
      cand.y = a.y + t * (b.y - a.y);
      cand.yaw = wrapAngle(a.yaw + t * wrapAngle(b.yaw - a.yaw));
      cand.kappa = (1.0 - t) * a.kappa + t * b.kappa;
      cand.s = (1.0 - t) * a.s + t * b.s;
      cand.t = (1.0 - t) * a.t + t * b.t;
      cand.v_ref = std::max(0.05, (1.0 - t) * a.v_ref + t * b.v_ref);
      cand.a_ref = (1.0 - t) * a.a_ref + t * b.a_ref;
      best.valid = true;
      best.suffix_index = std::min(seg + 1, ref_profile.size() - 1);
      best.join_point = cand;
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

    const double s_min = s_proj;
    const double s_max = s_proj + std::max(0.5, cfg_.reference_join_s_window);
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < ref_profile.size(); ++i) {
      const RefPoint& a = ref_profile[i];
      const RefPoint& b = ref_profile[i + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double seg_len2 = std::max(1e-6, dx * dx + dy * dy);
      const double t = clampValue((-(a.x * dx + a.y * dy)) / seg_len2, 0.0, 1.0);

      RefPoint cand;
      cand.x = a.x + t * dx;
      cand.y = a.y + t * dy;
      cand.yaw = wrapAngle(a.yaw + t * wrapAngle(b.yaw - a.yaw));
      cand.kappa = (1.0 - t) * a.kappa + t * b.kappa;
      cand.s = (1.0 - t) * a.s + t * b.s;
      cand.t = (1.0 - t) * a.t + t * b.t;
      cand.v_ref = std::max(0.05, (1.0 - t) * a.v_ref + t * b.v_ref);
      cand.a_ref = (1.0 - t) * a.a_ref + t * b.a_ref;

      if (cand.s < s_min || cand.s > s_max) {
        continue;
      }

      if (cand.x < cfg_.reference_join_min_forward_x) {
        continue;
      }
      if (std::fabs(cand.y) > cfg_.reference_join_max_lateral) {
        continue;
      }

      const double dist = hypot2(cand.x, cand.y);
      const double yaw_err = std::fabs(wrapAngle(cand.yaw));
      const double eta = cand.t;
      const double tangent_align = std::fabs(std::sin(cand.yaw));
      const double connector_len = dist;
      double cost = connector_len +
                    0.50 * std::fabs(cand.y) +
                    0.35 * yaw_err +
                    0.25 * tangent_align +
                    cfg_.reference_join_curvature_weight * std::fabs(cand.kappa) +
                    cfg_.reference_join_skip_weight * std::max(0.0, cand.s - s_proj) +
                    cfg_.reference_join_time_weight * std::min(eta, 6.0) +
                    cfg_.reference_join_progress_weight *
                        std::max(0.0, cfg_.reference_join_min_forward_x - cand.x);
      if (last_join_valid_) {
        if (cand.s < last_join_s_ - cfg_.reference_join_max_s_backtrack) {
          continue;
        }
        cost += cfg_.reference_join_hysteresis_weight * std::fabs(cand.s - last_join_s_);
      }
      if (cost < best_cost) {
        best_cost = cost;
        best.valid = true;
        best.suffix_index = i + 1;
        best.join_point = cand;
      }
    }

    if (!best.valid) {
      best.valid = true;
      best.suffix_index = 0;
      best.join_point = ref_profile.front();
    }

    double lookahead = 0.0;
    while (best.suffix_index < ref_profile.size() &&
           best.suffix_index + 1 < ref_profile.size() &&
           lookahead < cfg_.reference_join_lookahead) {
      lookahead += pointDistance(ref_profile[best.suffix_index].x,
                                 ref_profile[best.suffix_index].y,
                                 ref_profile[best.suffix_index + 1].x,
                                 ref_profile[best.suffix_index + 1].y);
      ++best.suffix_index;
      best.join_point = ref_profile[best.suffix_index];
    }
    return best;
  }

  std::vector<RefPoint> buildConnector(const RefPoint& goal) const {
    std::vector<RefPoint> out;
    const double dist = hypot2(goal.x, goal.y);
    if (dist < cfg_.reference_connector_enable_dist) {
      return out;
    }

    const double tangent0 = std::min(cfg_.reference_connector_tangent_scale * dist, 2.5);
    const double tangent1 = std::min(cfg_.reference_connector_tangent_scale * dist, 2.5);
    const double kappa0 = 0.0;
    const double kappa1 = clampValue(goal.kappa, -0.6, 0.6) * cfg_.reference_connector_curvature_scale;
    const double t0x = 1.0;
    const double t0y = 0.0;
    const double n0x = 0.0;
    const double n0y = 1.0;
    const double t1x = std::cos(goal.yaw);
    const double t1y = std::sin(goal.yaw);
    const double n1x = -std::sin(goal.yaw);
    const double n1y = std::cos(goal.yaw);

    RefPoint control[6];
    control[0].x = 0.0;
    control[0].y = 0.0;
    control[0].yaw = 0.0;
    control[0].kappa = kappa0;
    control[0].t = 0.0;
    control[0].v_ref = std::max(0.10, goal.v_ref * 0.6);

    control[5] = goal;
    control[1].x = control[0].x + tangent0 * t0x / 5.0;
    control[1].y = control[0].y + tangent0 * t0y / 5.0;
    control[2].x = 2.0 * control[1].x - control[0].x + (tangent0 * tangent0 * kappa0 / 20.0) * n0x;
    control[2].y = 2.0 * control[1].y - control[0].y + (tangent0 * tangent0 * kappa0 / 20.0) * n0y;

    control[4].x = control[5].x - tangent1 * t1x / 5.0;
    control[4].y = control[5].y - tangent1 * t1y / 5.0;
    control[3].x = 2.0 * control[4].x - control[5].x + (tangent1 * tangent1 * kappa1 / 20.0) * n1x;
    control[3].y = 2.0 * control[4].y - control[5].y + (tangent1 * tangent1 * kappa1 / 20.0) * n1y;

    const int steps =
        std::max(3, static_cast<int>(std::ceil(dist / std::max(0.03, cfg_.reference_connector_resolution))));
    out.reserve(static_cast<size_t>(steps) + 1);

    for (int i = 0; i < steps; ++i) {
      RefPoint pt;
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      evaluateQuinticBezier(control, t, &pt);
      pt.kappa = (1.0 - t) * kappa0 + t * kappa1;
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
  double last_join_s_ = 0.0;
  bool last_join_valid_ = false;
  mutable std::string last_join_mode_debug_ = "init";
  mutable bool last_join_step_limited_flag_ = false;
  mutable double last_clip_start_s_ = 0.0;
  mutable double last_clip_end_s_ = 0.0;
  mutable ros::Time last_clip_stamp_;
  mutable ros::Time last_real_clip_stamp_;
  mutable bool last_clip_valid_ = false;
  mutable double last_endpoint_s_ = 0.0;
  mutable bool last_endpoint_s_valid_ = false;
  mutable int endpoint_recapture_frames_ = 0;
  mutable ros::Time endpoint_far_start_stamp_;
  mutable double last_endpoint_x_ = 0.0;
  mutable double last_endpoint_y_ = 0.0;
  mutable bool last_endpoint_valid_ = false;
  mutable double last_raw_front_x_ = 0.0;
  mutable double last_raw_front_y_ = 0.0;
  mutable bool last_raw_front_valid_ = false;
};

class FixedFrameTrajectoryReconstructor {
 public:
  FixedFrameTrajectoryReconstructor()
      : nh_(),
        pnh_("~"),
        ego_motion_(loadEgoMotionConfig()),
        projector_(loadProjectorConfig()),
        lead_filter_(loadLeadFilterConfig()),
        tracker_(loadTrackerConfig()),
        path_generator_(loadPathConfig()) {
    base_frame_ = pnh_.param<std::string>("base_frame", "base_link");
    odom_frame_ = pnh_.param<std::string>("odom_frame", "reconstruction_odom");
    tracked_topic_ = pnh_.param<std::string>("tracked_topic", "/pointpillars/tracked_objects");
    car_topic_ = pnh_.param<std::string>("car_topic", "/car_message");
    carvel_topic_ = pnh_.param<std::string>("carvel_topic", "/carvel");
    system_speed_topic_ = pnh_.param<std::string>("system_speed_topic", "/system_speed");
    ned_odom_topic_ = pnh_.param<std::string>("ned_odom_topic", "/NED_odometry");
    imu_topic_ = pnh_.param<std::string>("imu_topic", "/imu");
    raw_topic_ = pnh_.param<std::string>("raw_topic", "/planner/raw_path");
    reference_topic_ = pnh_.param<std::string>("reference_topic", "/planner/reference_path");
    world_confirmed_topic_ =
        pnh_.param<std::string>("world_confirmed_topic", "/reconstruction/world_confirmed_path");
    lead_filter_state_topic_ =
        pnh_.param<std::string>("lead_filter_state_topic", "/planner/lead_filter_state");
    lead_filter_confidence_topic_ =
        pnh_.param<std::string>("lead_filter_confidence_topic", "/planner/lead_filter_confidence");
    reconstructed_traj_log_file_ = pnh_.param<std::string>(
      "reconstructed_traj_log_file",
      "/home/ubuntu/drive_car_copy/data/reconstructed_relative_trajectory.txt");
    history_traj_log_file_ = pnh_.param<std::string>(
      "history_traj_log_file",
      "/home/ubuntu/drive_car_copy/data/history_relative_trajectory.txt");
    debug_log_dir_ = pnh_.param<std::string>(
      "debug_log_dir",
      pnh_.param<std::string>("debug_dir", "/home/ubuntu/drive_car_copy/src/move_car/log"));
    publish_rate_hz_ = pnh_.param("publish_rate_hz", 30.0);

    wheel_speed_scale_ = pnh_.param("wheel_speed_scale", 0.001);
    speed_hint_scale_ = pnh_.param("speed_hint_scale", 0.001);
    system_speed_scale_ = pnh_.param("system_speed_scale", 1.0);
    system_speed_msg_type_ = pnh_.param<std::string>("system_speed_msg_type", "twist");
    ned_odom_speed_scale_ = pnh_.param("ned_odom_speed_scale", 1.0);
    steer_angle_scale_ = pnh_.param("steer_angle_scale", 1.0);
    steer_angle_bias_ = pnh_.param("steer_angle_bias", 0.0);
    steer_sign_ = pnh_.param("steer_sign", -1.0);
    steer_deadband_rad_ = pnh_.param("steer_deadband_rad", 0.002);
    imu_yaw_rate_scale_ = pnh_.param("imu_yaw_rate_scale", 1.0);
    imu_yaw_rate_bias_ = pnh_.param("imu_yaw_rate_bias", 0.0);
    imu_yaw_rate_sign_ = pnh_.param("imu_yaw_rate_sign", 1.0);
    lost_timeout_ = pnh_.param("lost_timeout", 0.60);
    min_motion_speed_for_yaw_ = pnh_.param("min_motion_speed_for_yaw", 0.15);
    allow_unstamped_tracked_fallback_ = pnh_.param("allow_unstamped_tracked_fallback", false);
    enable_debug_csv_logs_ = pnh_.param("enable_debug_csv_logs", true);
    reference_publish_quality_gate_enabled_ =
        pnh_.param("reference_publish_quality_gate_enabled", true);
    reference_publish_require_control_valid_ =
        pnh_.param("reference_publish_require_control_valid", true);
    reference_publish_hold_on_short_endpoint_ =
        pnh_.param("reference_publish_hold_on_short_endpoint", true);
    reference_publish_max_heading_jump_count_ =
        pnh_.param("reference_publish_max_heading_jump_count", 0);
    reference_publish_min_points_ =
        pnh_.param("reference_publish_min_points", 4);
    reference_publish_min_span_ =
        pnh_.param("reference_publish_min_span", 0.45);
    reference_publish_hold_timeout_ =
        pnh_.param("reference_publish_hold_timeout", 0.45);

    tracked_sub_ = nh_.subscribe(tracked_topic_, 5, &FixedFrameTrajectoryReconstructor::trackedCallback, this);
    car_sub_ = nh_.subscribe(car_topic_, 20, &FixedFrameTrajectoryReconstructor::carCallback, this);
    carvel_sub_ = nh_.subscribe(carvel_topic_, 20, &FixedFrameTrajectoryReconstructor::carvelCallback, this);
    subscribeSystemSpeed();
    ned_odom_sub_ =
        nh_.subscribe(ned_odom_topic_, 20, &FixedFrameTrajectoryReconstructor::nedOdometryCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 100, &FixedFrameTrajectoryReconstructor::imuCallback, this);
    raw_pub_ = nh_.advertise<nav_msgs::Path>(raw_topic_, 1);
    reference_pub_ = nh_.advertise<nav_msgs::Path>(reference_topic_, 1);
    world_confirmed_pub_ = nh_.advertise<nav_msgs::Path>(world_confirmed_topic_, 1);
    lead_filter_state_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(lead_filter_state_topic_, 1);
    lead_filter_confidence_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(lead_filter_confidence_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
                             &FixedFrameTrajectoryReconstructor::timerCallback,
                             this);

    openTrajectoryLogStream(reconstructed_traj_log_file_, "FRAME,timestamp,x,y,yaw", &reconstructed_traj_log_stream_);
    openTrajectoryLogStream(history_traj_log_file_, "timestamp,x,y,yaw", &history_traj_log_stream_);
    openDebugLogStreams();

    ROS_INFO("trajectory_reconstructor started | tracked=%s car=%s carvel=%s system_speed=%s ned_odom=%s imu=%s raw=%s ref=%s world=%s filter=%s conf=%s base=%s odom=%s",
             tracked_topic_.c_str(),
             car_topic_.c_str(),
             carvel_topic_.c_str(),
             system_speed_topic_.c_str(),
             ned_odom_topic_.c_str(),
             imu_topic_.c_str(),
             raw_topic_.c_str(),
             reference_topic_.c_str(),
             world_confirmed_topic_.c_str(),
             lead_filter_state_topic_.c_str(),
             lead_filter_confidence_topic_.c_str(),
             base_frame_.c_str(),
             odom_frame_.c_str());
    ROS_INFO("trajectory_reconstructor trajectory logging | reconstructed=%s enabled=%d history=%s enabled=%d",
         reconstructed_traj_log_file_.c_str(),
         reconstructed_traj_log_stream_.is_open() ? 1 : 0,
         history_traj_log_file_.c_str(),
         history_traj_log_stream_.is_open() ? 1 : 0);
    ROS_INFO("trajectory_reconstructor ego input scales | wheel_speed_scale=%.6f speed_hint_scale=%.6f steer_angle_scale=%.6f steer_sign=%.2f system_speed_type=%s",
             wheel_speed_scale_,
             speed_hint_scale_,
             steer_angle_scale_,
             steer_sign_,
             system_speed_msg_type_.c_str());
    ROS_INFO("trajectory_reconstructor safety gates | reconstruct_steer_sign=%.2f reference_hold=%.2fs history_kink_gate=%.3f raw_heading_gate=%.3f projection_backtrack=%.3f",
             steer_sign_,
             reference_publish_hold_timeout_,
             pnh_.param("confirmed_history_kink_reject_angle", 0.55),
             pnh_.param("raw_output_heading_flip_gate", 0.80),
             pnh_.param("confirmed_history_projection_backtrack_gate", 0.08));
    ROS_INFO("trajectory_reconstructor debug csv logging | dir=%s enabled=%d runtime=%d obs=%d history=%d filter=%d",
             debug_log_dir_.c_str(),
             enable_debug_csv_logs_ ? 1 : 0,
             runtime_debug_log_stream_.is_open() ? 1 : 0,
             observation_filter_log_stream_.is_open() ? 1 : 0,
             history_status_log_stream_.is_open() ? 1 : 0,
             lead_filter_state_log_stream_.is_open() ? 1 : 0);
  }

 private:
  EgoMotionEstimator::Config loadEgoMotionConfig() const {
    EgoMotionEstimator::Config cfg;
    cfg.wheelbase = pnh_.param("wheelbase", 2.60);
    cfg.min_motion_speed_for_yaw = pnh_.param("min_motion_speed_for_yaw", 0.15);
    cfg.max_abs_yaw_rate = pnh_.param("max_abs_yaw_rate", 0.80);
    cfg.max_integration_dt = pnh_.param("max_integration_dt", 0.02);
    cfg.pose_buffer_duration = pnh_.param("pose_buffer_duration", 5.0);
    cfg.steer_lpf_alpha = clampValue(pnh_.param("steer_lpf_alpha", 0.25), 0.0, 1.0);
    cfg.yaw_sigma_base = pnh_.param("yaw_sigma_base", 0.01);
    cfg.yaw_sigma_speed_gain = pnh_.param("yaw_sigma_speed_gain", 0.02);
    cfg.yaw_sigma_steer_gain = pnh_.param("yaw_sigma_steer_gain", 0.08);
    cfg.use_speed_hint = pnh_.param("use_speed_hint", false);
    cfg.use_system_speed = pnh_.param("use_system_speed", true);
    cfg.use_ned_odometry_speed = pnh_.param("use_ned_odometry_speed", true);
    cfg.use_ned_odometry_pose_delta = pnh_.param("use_ned_odometry_pose_delta", true);
    cfg.speed_hint_weight = clampValue(pnh_.param("speed_hint_weight", 0.20), 0.0, 1.0);
    cfg.speed_hint_timeout = pnh_.param("speed_hint_timeout", 0.0);
    cfg.speed_source_timeout = pnh_.param("speed_source_timeout", 0.30);
    cfg.ego_translation_observable_speed_gate =
        pnh_.param("ego_translation_observable_speed_gate", 0.04);
    cfg.odom_pose_delta_speed_gate =
        pnh_.param("odom_pose_delta_speed_gate", 0.04);
    cfg.odom_pose_delta_max_speed =
        pnh_.param("odom_pose_delta_max_speed", 3.0);
    cfg.speed_hint_agreement_gate = pnh_.param("speed_hint_agreement_gate", 0.40);
    cfg.use_imu_yaw_rate = pnh_.param("use_imu_yaw_rate", true);
    cfg.imu_yaw_rate_lpf_alpha =
        clampValue(pnh_.param("imu_yaw_rate_lpf_alpha", 0.35), 0.0, 1.0);
    cfg.imu_yaw_rate_timeout = pnh_.param("imu_yaw_rate_timeout", 0.10);
    cfg.imu_yaw_rate_bias = pnh_.param("imu_yaw_rate_bias", 0.0);
    cfg.imu_yaw_rate_agreement_gate = pnh_.param("imu_yaw_rate_agreement_gate", 0.35);
    cfg.imu_yaw_sigma_base = pnh_.param("imu_yaw_sigma_base", 0.005);
    cfg.imu_yaw_sigma_rate_gain = pnh_.param("imu_yaw_sigma_rate_gain", 0.01);
    cfg.imu_yaw_rate_deadband = pnh_.param("imu_yaw_rate_deadband", 0.004);
    cfg.imu_yaw_spike_gate = pnh_.param("imu_yaw_spike_gate", 0.45);
    cfg.imu_yaw_activity_gate = pnh_.param("imu_yaw_activity_gate", 0.025);
    cfg.imu_activity_hold_time = pnh_.param("imu_activity_hold_time", 0.25);
    cfg.standstill_speed_gate = pnh_.param("standstill_speed_gate", 0.04);
    cfg.standstill_gyro_gate = pnh_.param("standstill_gyro_gate", 0.015);
    cfg.low_speed_yaw_gate = pnh_.param("low_speed_yaw_gate", 0.12);
    cfg.imu_yaw_mixing_speed_gate = pnh_.param("imu_yaw_mixing_speed_gate", 0.20);
    cfg.zero_yaw_rate_when_standstill = pnh_.param("zero_yaw_rate_when_standstill", true);
    cfg.static_anchor_ego_xy_gain =
        clampValue(pnh_.param("static_anchor_ego_xy_gain", 0.45), 0.0, 1.0);
    cfg.static_anchor_ego_xy_gate = pnh_.param("static_anchor_ego_xy_gate", 1.50);
    cfg.static_anchor_ego_xy_step = pnh_.param("static_anchor_ego_xy_step", 0.35);
    cfg.static_anchor_ego_xy_min_track_age =
        pnh_.param("static_anchor_ego_xy_min_track_age", 0.60);
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

  LeadObservationFilter::Config loadLeadFilterConfig() const {
    LeadObservationFilter::Config cfg;
    cfg.enabled = pnh_.param("lead_filter_enabled", true);
    cfg.measurement_position_sigma = pnh_.param("lead_filter_measurement_position_sigma", 0.18);
    cfg.measurement_position_range_gain = pnh_.param("lead_filter_measurement_position_range_gain", 0.015);
    cfg.measurement_heading_sigma = pnh_.param("lead_filter_measurement_heading_sigma", 0.45);
    cfg.process_position_sigma = pnh_.param("lead_filter_process_position_sigma", 0.10);
    cfg.process_velocity_sigma = pnh_.param("lead_filter_process_velocity_sigma", 0.70);
    cfg.process_heading_sigma = pnh_.param("lead_filter_process_heading_sigma", 0.35);
    cfg.max_prediction_dt = pnh_.param("lead_filter_max_prediction_dt", 0.25);
    cfg.reset_timeout = pnh_.param("lead_filter_reset_timeout", 0.80);
    cfg.robust_cauchy_scale = pnh_.param("lead_filter_robust_cauchy_scale", 2.5);
    cfg.min_robust_weight = clampValue(pnh_.param("lead_filter_min_robust_weight", 0.08), 0.01, 1.0);
    cfg.window_duration = pnh_.param("lead_filter_window_duration", 0.80);
    cfg.window_min_points = pnh_.param("lead_filter_window_min_points", 4);
    cfg.window_velocity_blend =
        clampValue(pnh_.param("lead_filter_window_velocity_blend", 0.65), 0.0, 1.0);
    cfg.heading_from_velocity_speed_gate =
        pnh_.param("lead_filter_heading_from_velocity_speed_gate", 0.12);
    cfg.heading_update_speed_gate =
        pnh_.param("lead_filter_heading_update_speed_gate", 0.18);
    cfg.heading_flip_gate =
        pnh_.param("lead_filter_heading_flip_gate", 1.20);
    cfg.static_velocity_damping =
        clampValue(pnh_.param("lead_filter_static_velocity_damping", 0.35), 0.0, 1.0);
    cfg.outlier_velocity_damping =
        clampValue(pnh_.param("lead_filter_outlier_velocity_damping", 0.60), 0.0, 1.0);
    cfg.confidence_lpf_alpha =
        clampValue(pnh_.param("lead_filter_confidence_lpf_alpha", 0.35), 0.0, 1.0);
    cfg.confidence_position_sigma_ref = pnh_.param("lead_filter_confidence_position_sigma_ref", 0.40);
    cfg.confidence_heading_sigma_ref = pnh_.param("lead_filter_confidence_heading_sigma_ref", 0.75);
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
    cfg.confirmed_history_max_age = pnh_.param(
        "confirmed_history_max_age", std::max(60.0, cfg.history_max_age));
    cfg.confirmed_history_min_points_keep =
        pnh_.param("confirmed_history_min_points_keep", 30);
    cfg.history_max_points = pnh_.param("history_max_points", 300);
    cfg.export_static_anchor_to_history =
        pnh_.param("export_static_anchor_to_history", false);
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
    cfg.measurement_along_sigma = pnh_.param("measurement_along_sigma", 0.08);
    cfg.measurement_cross_sigma = pnh_.param("measurement_cross_sigma", 0.12);
    cfg.measurement_cross_range_gain = pnh_.param("measurement_cross_range_gain", 0.020);
    cfg.measurement_cross_yaw_uncertainty_gain =
        pnh_.param("measurement_cross_yaw_uncertainty_gain", 1.80);
    cfg.yaw_rate_cross_weight_gain = pnh_.param("yaw_rate_cross_weight_gain", 0.20);
    cfg.temporal_min_progress = pnh_.param("temporal_min_progress", 0.02);
    cfg.trail_append_min_distance = pnh_.param("trail_append_min_distance", 0.12);
    cfg.trail_anchor_update_alpha =
        clampValue(pnh_.param("trail_anchor_update_alpha", 0.10), 0.0, 1.0);
    cfg.tail_confirm_min_points = pnh_.param("tail_confirm_min_points", 3);
    cfg.tail_confirm_min_distance = pnh_.param("tail_confirm_min_distance", 0.06);
    cfg.tail_confirm_min_age = pnh_.param("tail_confirm_min_age", 0.18);
    cfg.confirm_dynamic_score_margin = pnh_.param("confirm_dynamic_score_margin", 0.12);
    cfg.confirm_static_residual_gate = pnh_.param("confirm_static_residual_gate", 0.14);
    cfg.confirm_min_progress = pnh_.param("confirm_min_progress", 0.08);
    cfg.confirm_accum_distance = pnh_.param("confirm_accum_distance", 0.05);
    cfg.confirm_accum_timeout = pnh_.param("confirm_accum_timeout", 0.80);
    cfg.ego_static_speed_gate = pnh_.param("ego_static_speed_gate", 0.05);
    cfg.static_release_score_margin = pnh_.param("static_release_score_margin", 0.06);
    cfg.raw_tail_keep_points = pnh_.param("raw_tail_keep_points", 6);
    cfg.dual_hypothesis_score_margin = pnh_.param("dual_hypothesis_score_margin", 0.02);
    cfg.tail_buffer_max_points = pnh_.param("tail_buffer_max_points", 12);
    cfg.tail_buffer_max_age = pnh_.param("tail_buffer_max_age", 0.60);
    cfg.static_tail_keep_points = pnh_.param("static_tail_keep_points", 3);
    cfg.publish_tail_suffix = pnh_.param("publish_tail_suffix", false);
    cfg.tail_confirm_min_time = pnh_.param("tail_confirm_min_time", 0.25);
    cfg.lead_tail_max_speed = pnh_.param("lead_tail_max_speed", 1.20);
    cfg.tail_cross_gate = pnh_.param("tail_cross_gate", 0.30);
    cfg.tail_reverse_gate = pnh_.param("tail_reverse_gate", -0.03);
    cfg.static_anchor_res_gate = pnh_.param("static_anchor_res_gate", 0.04);
    cfg.static_anchor_progress_gate = pnh_.param("static_anchor_progress_gate", 0.04);
    cfg.static_anchor_speed_gate = pnh_.param("static_anchor_speed_gate", 0.06);
    cfg.static_anchor_append_distance_gate =
        pnh_.param("static_anchor_append_distance_gate", 0.08);
    cfg.ego_yaw_mixing_yaw_rate_gate = pnh_.param("ego_yaw_mixing_yaw_rate_gate", 0.04);
    cfg.ego_yaw_mixing_lateral_gate = pnh_.param("ego_yaw_mixing_lateral_gate", 0.04);
    cfg.ego_yaw_mixing_residual_gate = pnh_.param("ego_yaw_mixing_residual_gate", 0.10);
    cfg.ego_yaw_mixing_anchor_gate = pnh_.param("ego_yaw_mixing_anchor_gate", 0.10);
    cfg.ego_yaw_mixing_dynamic_ratio = pnh_.param("ego_yaw_mixing_dynamic_ratio", 0.75);
    cfg.ego_yaw_mixing_heading_gate = pnh_.param("ego_yaw_mixing_heading_gate", 0.20);
    cfg.ego_yaw_mixing_weight_scale =
        clampValue(pnh_.param("ego_yaw_mixing_weight_scale", 0.25), 0.01, 1.0);
    cfg.imu_yaw_activity_gate = pnh_.param("imu_yaw_activity_gate", 0.025);
    cfg.dynamic_residual_margin = pnh_.param("dynamic_residual_margin", 0.08);
    cfg.reverse_progress_gate = pnh_.param("reverse_progress_gate", 0.04);
    cfg.bootstrap_consecutive_dynamic = pnh_.param("bootstrap_consecutive_dynamic", 3);
    cfg.history_kink_min_dist = pnh_.param("history_kink_min_dist", 0.30);
    cfg.history_kink_heading_gate = pnh_.param("history_kink_heading_gate", 0.35);
    cfg.confirmed_history_kink_reject_angle =
        pnh_.param("confirmed_history_kink_reject_angle", 0.55);
    cfg.confirmed_history_reverse_gate =
        pnh_.param("confirmed_history_reverse_gate", 0.035);
    cfg.confirmed_history_short_reverse_dist =
        pnh_.param("confirmed_history_short_reverse_dist", 0.18);
    cfg.confirmed_history_curvature_impulse_gate =
        pnh_.param("confirmed_history_curvature_impulse_gate", 7.0);
    cfg.confirmed_history_stop_speed_gate =
        pnh_.param("confirmed_history_stop_speed_gate", 0.03);
    cfg.confirmed_history_stop_residual_gate =
        pnh_.param("confirmed_history_stop_residual_gate", 0.45);
    cfg.confirmed_history_stop_lead_speed_gate =
        pnh_.param("confirmed_history_stop_lead_speed_gate", 0.55);
    cfg.confirmed_history_projection_gate_enabled =
        pnh_.param("confirmed_history_projection_gate_enabled", true);
    cfg.confirmed_history_projection_backtrack_gate =
        pnh_.param("confirmed_history_projection_backtrack_gate", 0.08);
    cfg.confirmed_history_projection_near_dist =
        pnh_.param("confirmed_history_projection_near_dist", 0.16);
    cfg.confirmed_history_projection_lateral_gate =
        pnh_.param("confirmed_history_projection_lateral_gate", 0.22);
    cfg.reconstruction_reseed_bad_frames =
        pnh_.param("reconstruction_reseed_bad_frames", 18);
    cfg.history_max_confirm_speed = pnh_.param("history_max_confirm_speed", 3.0);
    cfg.static_anchor_ego_correction_min_static_votes =
        pnh_.param("static_anchor_ego_correction_min_static_votes", 10);
    cfg.static_anchor_ego_correction_max_moving_votes =
        pnh_.param("static_anchor_ego_correction_max_moving_votes", 0);
    cfg.static_dynamic_probe_min_progress_ratio =
        pnh_.param("static_dynamic_probe_min_progress_ratio", 0.35);
    cfg.static_dynamic_probe_speed_ratio =
        pnh_.param("static_dynamic_probe_speed_ratio", 0.45);
    cfg.static_dynamic_probe_residual_margin =
        pnh_.param("static_dynamic_probe_residual_margin", 0.03);
    cfg.dynamic_bootstrap_min_tail_points =
        pnh_.param("dynamic_bootstrap_min_tail_points", 4);
    cfg.dynamic_bootstrap_min_span =
        pnh_.param("dynamic_bootstrap_min_span", 0.20);
    cfg.dynamic_bootstrap_min_ff_speed =
        pnh_.param("dynamic_bootstrap_min_ff_speed", 0.08);
    cfg.dynamic_bootstrap_score_margin =
        pnh_.param("dynamic_bootstrap_score_margin", 0.08);
    cfg.endpoint_recapture_min_frames =
        pnh_.param("endpoint_recapture_min_frames", 4);
    cfg.raw_clip_projection_far_hold_max_time =
        pnh_.param("raw_clip_projection_far_hold_max_time", 0.80);
    cfg.endpoint_recapture_confidence_gate =
        pnh_.param("endpoint_recapture_confidence_gate", 0.75);
    cfg.dynamic_curve_cross_gate =
        pnh_.param("dynamic_curve_cross_gate", 1.20);
    cfg.dynamic_curve_min_arc =
        pnh_.param("dynamic_curve_min_arc", 0.20);
    cfg.dynamic_curve_heading_gate =
        pnh_.param("dynamic_curve_heading_gate", 0.45);
    cfg.yaw_mixing_freeze_requires_endpoint_near_path =
        pnh_.param("yaw_mixing_freeze_requires_endpoint_near_path", true);
    cfg.confirmed_curve_interp_ds =
        pnh_.param("confirmed_curve_interp_ds", 0.10);
    cfg.ego_translation_observable_speed_gate =
        pnh_.param("ego_translation_observable_speed_gate", 0.04);
    cfg.confirmed_require_dynamic_motion_class =
        pnh_.param("confirmed_require_dynamic_motion_class", true);
    cfg.relative_shortening_freeze_confirmed =
        pnh_.param("relative_shortening_freeze_confirmed", true);
    cfg.confirm_require_candidate_metadata =
        pnh_.param("confirm_require_candidate_metadata", true);
    cfg.disable_filter_speed_dynamic_when_ego_unobservable =
        pnh_.param("disable_filter_speed_dynamic_when_ego_unobservable", true);
    cfg.history_segment_break_gap =
        pnh_.param("history_segment_break_gap", 0.45);
    cfg.lead_only_bootstrap_enabled =
        pnh_.param("lead_only_bootstrap_enabled", true);
    cfg.lead_only_bootstrap_min_points =
        pnh_.param("lead_only_bootstrap_min_points", 4);
    cfg.lead_only_bootstrap_min_span =
        pnh_.param("lead_only_bootstrap_min_span", 0.20);
    cfg.lead_only_bootstrap_min_progress =
        pnh_.param("lead_only_bootstrap_min_progress", 0.12);
    cfg.lead_only_bootstrap_min_ff_speed =
        pnh_.param("lead_only_bootstrap_min_ff_speed", 0.08);
    cfg.lead_only_bootstrap_max_imu_activity =
        pnh_.param("lead_only_bootstrap_max_imu_activity", 0.015);
    cfg.lead_only_bootstrap_static_residual_gate =
        pnh_.param("lead_only_bootstrap_static_residual_gate", 0.025);
    cfg.use_fusion_tracker =
        pnh_.param("use_fusion_tracker", true);
    cfg.fusion_meas_sigma_base =
        pnh_.param("fusion_meas_sigma_base", 0.10);
    cfg.fusion_meas_sigma_range_gain =
        pnh_.param("fusion_meas_sigma_range_gain", 0.015);
    cfg.fusion_process_pos =
        pnh_.param("fusion_process_pos", 0.03);
    cfg.fusion_process_vel =
        pnh_.param("fusion_process_vel", 0.08);
    cfg.fusion_prob_alpha =
        pnh_.param("fusion_prob_alpha", 0.25);
    cfg.fusion_static_residual_gate =
        pnh_.param("fusion_static_residual_gate", 0.08);
    cfg.fusion_dynamic_residual_margin =
        pnh_.param("fusion_dynamic_residual_margin", 0.04);
    cfg.fusion_hard_residual_gate_enabled =
        pnh_.param("fusion_hard_residual_gate_enabled", false);
    cfg.fusion_dynamic_prob_bootstrap =
        pnh_.param("fusion_dynamic_prob_bootstrap", 0.55);
    cfg.fusion_dynamic_prob_append =
        pnh_.param("fusion_dynamic_prob_append", 0.58);
    cfg.fusion_static_prob_freeze =
        pnh_.param("fusion_static_prob_freeze", 0.72);
    cfg.fusion_append_min_distance =
        pnh_.param("fusion_append_min_distance", 0.04);
    cfg.fusion_bootstrap_min_span =
        pnh_.param("fusion_bootstrap_min_span", 0.18);
    cfg.fusion_bootstrap_min_points =
        pnh_.param("fusion_bootstrap_min_points", 3);
    cfg.fusion_cov_trace_gate =
        pnh_.param("fusion_cov_trace_gate", 2.50);
    cfg.fusion_pos_cov_gate =
        pnh_.param("fusion_pos_cov_gate", 0.35);
    cfg.fusion_append_max_interval =
        pnh_.param("fusion_append_max_interval", 0.60);
    cfg.fusion_yaw_freeze_activity =
        pnh_.param("fusion_yaw_freeze_activity", 0.035);
    cfg.fusion_relative_shorten_gate =
        pnh_.param("fusion_relative_shorten_gate", 0.04);
    cfg.fusion_static_hold_frames =
        pnh_.param("fusion_static_hold_frames", 5);
    cfg.fusion_ego_explained_hold_frames =
        pnh_.param("fusion_ego_explained_hold_frames", 5);
    cfg.fusion_yaw_mix_hold_frames =
        pnh_.param("fusion_yaw_mix_hold_frames", 3);
    cfg.fusion_max_confirm_speed =
        pnh_.param("fusion_max_confirm_speed", 0.80);
    cfg.fusion_segment_break_gap =
        pnh_.param("fusion_segment_break_gap", 0.80);
    cfg.fusion_reverse_progress_gate =
        pnh_.param("fusion_reverse_progress_gate", 0.05);
    cfg.fusion_unobservable_cross_gate =
        pnh_.param("fusion_unobservable_cross_gate", 0.45);
    cfg.ego_explained_freeze_gate =
        pnh_.param("ego_explained_freeze_gate", 0.03);
    cfg.yaw_mix_confirm_freeze =
        pnh_.param("yaw_mix_confirm_freeze", true);
    cfg.fusion_prob_decay_on_static =
        clampValue(pnh_.param("fusion_prob_decay_on_static", 0.45), 0.0, 1.0);
    cfg.lead_confirm_max_speed =
        pnh_.param("lead_confirm_max_speed", 0.80);
    cfg.lead_speed_nominal_min =
        pnh_.param("lead_speed_nominal_min", 0.10);
    cfg.lead_speed_nominal_max =
        pnh_.param("lead_speed_nominal_max", 0.50);
    cfg.lead_speed_abs_max =
        pnh_.param("lead_speed_abs_max", 0.80);
    cfg.fusion_static_observation_hold_enabled =
        pnh_.param("fusion_static_observation_hold_enabled", true);
    cfg.fusion_static_observation_res_gate =
        pnh_.param("fusion_static_observation_res_gate", 0.08);
    cfg.fusion_static_observation_speed_gate =
        pnh_.param("fusion_static_observation_speed_gate", 0.06);
    cfg.fusion_static_observation_step_gate =
        pnh_.param("fusion_static_observation_step_gate", 0.03);
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
    cfg.lead_center_to_path_ref_offset = pnh_.param("lead_center_to_path_ref_offset", 0.0);
    cfg.raw_min_points_for_publish = pnh_.param("raw_min_points_for_publish", 2);
    cfg.raw_min_span_for_publish = pnh_.param("raw_min_span_for_publish", 0.08);
    cfg.raw_keep_behind_x = pnh_.param("raw_keep_behind_x", 0.50);
    cfg.raw_clip_to_ego_progress = pnh_.param("raw_clip_to_ego_progress", true);
    cfg.raw_spatial_window_mode =
        pnh_.param<std::string>("raw_spatial_window_mode", "ego_progress_suffix");
    cfg.raw_clip_duplicate_distance = pnh_.param("raw_clip_duplicate_distance", 0.02);
    cfg.raw_clip_use_current_lead_endpoint =
        pnh_.param("raw_clip_use_current_lead_endpoint", true);
    cfg.raw_clip_end_follow_lead_endpoint =
        pnh_.param("raw_clip_end_follow_lead_endpoint", false);
    cfg.raw_clip_keep_history_tail_when_endpoint_behind =
        pnh_.param("raw_clip_keep_history_tail_when_endpoint_behind", true);
    cfg.raw_clip_endpoint_hysteresis =
        pnh_.param("raw_clip_endpoint_hysteresis", true);
    cfg.raw_clip_max_s_jump = pnh_.param("raw_clip_max_s_jump", 0.40);
    cfg.raw_clip_max_s_step = pnh_.param("raw_clip_max_s_step", 0.08);
    cfg.raw_clip_max_s_rate = pnh_.param("raw_clip_max_s_rate", 1.20);
    cfg.raw_clip_start_max_s_step =
        pnh_.param("raw_clip_start_max_s_step", 0.08);
    cfg.raw_clip_start_max_s_rate =
        pnh_.param("raw_clip_start_max_s_rate", 1.20);
    cfg.raw_clip_endpoint_hold_time =
        pnh_.param("raw_clip_endpoint_hold_time", 0.50);
    cfg.raw_clip_endpoint_projection_gate =
        pnh_.param("raw_clip_endpoint_projection_gate", 0.75);
    cfg.raw_clip_min_endpoint_confidence =
        pnh_.param("raw_clip_min_endpoint_confidence", 0.55);
    cfg.endpoint_recapture_min_frames =
        pnh_.param("endpoint_recapture_min_frames", 4);
    cfg.raw_clip_projection_far_hold_max_time =
        pnh_.param("raw_clip_projection_far_hold_max_time", 0.80);
    cfg.endpoint_recapture_confidence_gate =
        pnh_.param("endpoint_recapture_confidence_gate", 0.75);
    cfg.endpoint_recapture_projection_gate =
        pnh_.param("endpoint_recapture_projection_gate", 1.00);
    cfg.endpoint_recapture_max_far_time =
        pnh_.param("endpoint_recapture_max_far_time", 0.30);
    cfg.raw_clip_start_search_window =
        pnh_.param("raw_clip_start_search_window", 1.20);
    cfg.raw_clip_endpoint_search_window =
        pnh_.param("raw_clip_endpoint_search_window", 1.50);
    cfg.raw_clip_allow_endpoint_reorder =
        pnh_.param("raw_clip_allow_endpoint_reorder", true);
    cfg.raw_clip_valid_endpoint_hold_max_time =
        pnh_.param("raw_clip_valid_endpoint_hold_max_time", 0.15);
    cfg.raw_clip_missing_endpoint_hold_time =
        pnh_.param("raw_clip_missing_endpoint_hold_time", 0.50);
    cfg.raw_clip_stale_max_time =
        pnh_.param("raw_clip_stale_max_time", 0.15);
    cfg.raw_clip_min_visible_span =
        pnh_.param("raw_clip_min_visible_span", 0.45);
    cfg.raw_max_range = pnh_.param("raw_max_range", 30.0);
    cfg.raw_max_points = pnh_.param("raw_max_points", 250);
    cfg.reference_join_min_forward_x = pnh_.param("reference_join_min_forward_x", 0.60);
    cfg.reference_join_max_lateral = pnh_.param("reference_join_max_lateral", 3.0);
    cfg.reference_join_lookahead = pnh_.param("reference_join_lookahead", 0.35);
    cfg.reference_join_skip_weight = pnh_.param("reference_join_skip_weight", 0.20);
    cfg.reference_join_curvature_weight = pnh_.param("reference_join_curvature_weight", 0.90);
    cfg.reference_join_time_weight = pnh_.param("reference_join_time_weight", 0.60);
    cfg.reference_join_progress_weight = pnh_.param("reference_join_progress_weight", 0.25);
    cfg.reference_join_s_window = pnh_.param("reference_join_s_window", 2.0);
    cfg.reference_join_hysteresis_weight = pnh_.param("reference_join_hysteresis_weight", 2.0);
    cfg.reference_join_max_s_backtrack = pnh_.param("reference_join_max_s_backtrack", 0.10);
    cfg.reference_join_mode =
        pnh_.param<std::string>("reference_join_mode", "front_anchored");
    cfg.reference_join_front_lookahead =
        pnh_.param("reference_join_front_lookahead", 0.25);
    cfg.reference_join_max_step =
        pnh_.param("reference_join_max_step", 0.08);
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
    cfg.reference_max_length = pnh_.param("reference_max_length", 20.0);
    cfg.reference_follow_raw_only = pnh_.param("reference_follow_raw_only", false);
    cfg.reference_publish_short_endpoint =
        pnh_.param("reference_publish_short_endpoint", true);
    cfg.reference_short_endpoint_min_confirmed =
        pnh_.param("reference_short_endpoint_min_confirmed", 2);
    cfg.reference_short_endpoint_when_clip_stale =
        pnh_.param("reference_short_endpoint_when_clip_stale", true);
    cfg.reference_short_endpoint_only_when_confirmed_empty =
        pnh_.param("reference_short_endpoint_only_when_confirmed_empty", true);
    cfg.control_feasible_max_lateral =
        pnh_.param("control_feasible_max_lateral", 1.20);
    cfg.control_feasible_max_heading =
        pnh_.param("control_feasible_max_heading", 0.80);
    cfg.anchor_rebase_enabled = pnh_.param("anchor_rebase_enabled", true);
    cfg.anchor_rebase_xy_gate = pnh_.param("anchor_rebase_xy_gate", 0.05);
    cfg.anchor_rebase_yaw_gate = pnh_.param("anchor_rebase_yaw_gate", 0.03);
    cfg.raw_output_segment_gap = pnh_.param("raw_output_segment_gap", 0.45);
    cfg.raw_output_heading_flip_gate =
        pnh_.param("raw_output_heading_flip_gate", 0.80);
    cfg.raw_output_min_stable_span =
        pnh_.param("raw_output_min_stable_span", 0.80);
    cfg.raw_front_jump_gate = pnh_.param("raw_front_jump_gate", 0.35);
    cfg.path_heading_recompute_min_ds =
        pnh_.param("path_heading_recompute_min_ds", 0.05);
    cfg.reference_temporal_consistency =
        pnh_.param("reference_temporal_consistency", true);
    cfg.reference_max_heading_step =
        pnh_.param("reference_max_heading_step", 0.15);
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
    double steer_angle_rad =
        steer_sign_ * (static_cast<double>(msg->turn_angle) * steer_angle_scale_ - steer_angle_bias_);
    if (std::fabs(steer_angle_rad) < steer_deadband_rad_) {
      steer_angle_rad = 0.0;
    }
    ego_motion_.updateWheelState(stamp, wheel_speed_mps, steer_angle_rad);
  }

  void carvelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // geometry_msgs/Twist has no header, so arrival time is the only available fallback.
    const ros::Time stamp = ros::Time::now();
    const double speed_hint_mps = static_cast<double>(msg->linear.x) * speed_hint_scale_;
    ego_motion_.updateSpeedHint(stamp, speed_hint_mps);
  }

  void subscribeSystemSpeed() {
    if (system_speed_topic_.empty() || system_speed_msg_type_ == "disabled") {
      ROS_INFO("trajectory_reconstructor system_speed disabled | topic=%s type=%s",
               system_speed_topic_.c_str(),
               system_speed_msg_type_.c_str());
      return;
    }

    if (system_speed_msg_type_ == "twist") {
      system_speed_sub_ =
          nh_.subscribe(system_speed_topic_, 20, &FixedFrameTrajectoryReconstructor::systemSpeedCallback, this);
    } else if (system_speed_msg_type_ == "twist_stamped") {
      system_speed_sub_ =
          nh_.subscribe(system_speed_topic_, 20, &FixedFrameTrajectoryReconstructor::systemSpeedStampedCallback, this);
    } else if (system_speed_msg_type_ == "float32") {
      system_speed_sub_ =
          nh_.subscribe(system_speed_topic_, 20, &FixedFrameTrajectoryReconstructor::systemSpeedFloat32Callback, this);
    } else if (system_speed_msg_type_ == "float64") {
      system_speed_sub_ =
          nh_.subscribe(system_speed_topic_, 20, &FixedFrameTrajectoryReconstructor::systemSpeedFloat64Callback, this);
    } else {
      ROS_WARN("trajectory_reconstructor unknown system_speed_msg_type=%s; falling back to geometry_msgs/Twist",
               system_speed_msg_type_.c_str());
      system_speed_msg_type_ = "twist";
      system_speed_sub_ =
          nh_.subscribe(system_speed_topic_, 20, &FixedFrameTrajectoryReconstructor::systemSpeedCallback, this);
    }

    ROS_INFO("trajectory_reconstructor system_speed subscription | topic=%s type=%s",
             system_speed_topic_.c_str(),
             system_speed_msg_type_.c_str());
  }

  void systemSpeedCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time stamp = ros::Time::now();
    const double speed_mps = static_cast<double>(msg->linear.x) * system_speed_scale_;
    ego_motion_.updateSystemSpeed(stamp, speed_mps, "geometry_msgs/Twist");
  }

  void systemSpeedStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ros::Time stamp = msg->header.stamp;
    if (!stamp.isValid()) {
      stamp = ros::Time::now();
    }
    const double speed_mps = static_cast<double>(msg->twist.linear.x) * system_speed_scale_;
    ego_motion_.updateSystemSpeed(stamp, speed_mps, "geometry_msgs/TwistStamped");
  }

  void systemSpeedFloat32Callback(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time stamp = ros::Time::now();
    const double speed_mps = static_cast<double>(msg->data) * system_speed_scale_;
    ego_motion_.updateSystemSpeed(stamp, speed_mps, "std_msgs/Float32");
  }

  void systemSpeedFloat64Callback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time stamp = ros::Time::now();
    const double speed_mps = static_cast<double>(msg->data) * system_speed_scale_;
    ego_motion_.updateSystemSpeed(stamp, speed_mps, "std_msgs/Float64");
  }

  void nedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ros::Time stamp = msg->header.stamp;
    if (!stamp.isValid()) {
      stamp = ros::Time::now();
    }
    const double vx = static_cast<double>(msg->twist.twist.linear.x);
    const double vy = static_cast<double>(msg->twist.twist.linear.y);
    const double planar_speed = hypot2(vx, vy);
    const double signed_speed = (vx < -1e-4 ? -planar_speed : planar_speed) * ned_odom_speed_scale_;
    ego_motion_.updateNedOdometrySpeed(stamp, signed_speed);
    ego_motion_.updateNedOdometryPoseDelta(stamp,
                                           msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           yawFromQuaternion(msg->pose.pose.orientation));
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ros::Time stamp = msg->header.stamp;
    if (!stamp.isValid()) {
      ROS_WARN_THROTTLE(1.0,
                        "imu_topic message has no valid header.stamp; falling back to receipt time. "
                        "Ego yaw compensation will be less accurate.");
      stamp = ros::Time::now();
    }

    const double yaw_rate =
        imu_yaw_rate_sign_ *
        (static_cast<double>(msg->angular_velocity.z) * imu_yaw_rate_scale_ - imu_yaw_rate_bias_);
    ego_motion_.updateImuYawRate(stamp, yaw_rate);
  }

  void trackedCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time obs_stamp = resolveStampedPoseArrayTime(*msg);
    if (!obs_stamp.isValid()) {
      debug_state_.input_pose_count = static_cast<int>(msg->poses.size());
      debug_state_.projector_reason = "obs_stamp_invalid";
      return;
    }

    ego_motion_.integrateTo(obs_stamp);
    tracker_.markLostIfExpired(obs_stamp);
    tracker_.pruneHistory(obs_stamp);

    LeadObservation obs;
    EgoPose2D ego_at_obs;
    const LeadState2D* track_ptr = tracker_.hasActiveTrack() ? &tracker_.track() : NULL;
    if (!projector_.project(*msg, ego_motion_, track_ptr, &obs, &ego_at_obs, &debug_state_)) {
      ++obs_version_;
      debug_state_.history_size = tracker_.worldHistoryForPublish(obs_stamp).size();
      return;
    }

    if (tracker_.canUseStaticAnchorForEgoCorrection(obs_stamp)) {
      const LeadState2D static_anchor = tracker_.track();
      if (ego_motion_.correctPoseFromStaticObservation(obs_stamp,
                                                       static_anchor.x,
                                                       static_anchor.y,
                                                       obs.local_x,
                                                       obs.local_y,
                                                       tracker_.staticModeAge(obs_stamp))) {
        ego_motion_.queryPoseAt(obs_stamp, &ego_at_obs);
        transformLocalToWorld(ego_at_obs, obs.local_x, obs.local_y, &obs.world_x, &obs.world_y);
        obs.world_heading = wrapAngle(ego_at_obs.yaw + obs.local_heading);
      }
    }

    const LeadObservation filtered_obs = lead_filter_.update(obs, ego_at_obs, &debug_state_);

    tracker_.ingestObservation(filtered_obs,
                               ego_at_obs,
                               ego_motion_.currentYawRate(),
                               ego_motion_.imuYawActivity(),
                               ego_motion_.yawSigmaEstimate(),
                               ego_motion_.steerAngle(),
                               &debug_state_);
    tracker_.pruneHistory(obs_stamp);
    debug_state_.history_size = tracker_.worldHistoryForPublish(obs_stamp).size();
    appendObservationFilterLog(obs, filtered_obs, ego_at_obs, debug_state_);
    ++obs_version_;
  }

  bool referencePublishGood(const std::vector<LocalPathPoint>& ref_pts,
                            const TrackerDebugState& debug,
                            std::string* reason) const {
    if (!reference_publish_quality_gate_enabled_) {
      if (reason) *reason = "gate_disabled";
      return true;
    }
    if (static_cast<int>(ref_pts.size()) < reference_publish_min_points_) {
      if (reason) *reason = "publish_short_points";
      return false;
    }
    const double span = polylineLengthLocal(ref_pts);
    if (span < reference_publish_min_span_) {
      if (reason) *reason = "publish_short_span";
      return false;
    }
    if (reference_publish_require_control_valid_ && !debug.reference_control_valid) {
      if (reason) *reason = "publish_control_invalid";
      return false;
    }
    if (debug.raw_heading_jump_count > reference_publish_max_heading_jump_count_) {
      if (reason) *reason = "publish_heading_jump";
      return false;
    }
    if (reference_publish_hold_on_short_endpoint_ &&
        (debug.reference_reason == "reference_short_live_endpoint" ||
         debug.reference_reason == "reference_short_confirmed_endpoint")) {
      if (reason) *reason = debug.reference_reason;
      return false;
    }
    if (debug.yaw_mix_freeze) {
      if (reason) *reason = "publish_yaw_mix_freeze";
      return false;
    }
    if (reason) *reason = "publish_good";
    return true;
  }

  std::vector<LocalPathPoint> selectReferenceForPublish(const std::vector<LocalPathPoint>& ref_pts,
                                                        const ros::Time& now,
                                                        const EgoPose2D& ego_now,
                                                        bool ego_now_valid,
                                                        TrackerDebugState* debug) {
    if (debug == NULL || !reference_publish_quality_gate_enabled_) {
      if (!ref_pts.empty()) {
        last_good_reference_pts_ = ref_pts;
        last_good_reference_stamp_ = now;
        has_last_good_reference_ = true;
        last_good_reference_ego_pose_ = ego_now;
        last_good_reference_ego_valid_ = ego_now_valid;
      }
      return ref_pts;
    }

    std::string reject_reason;
    if (referencePublishGood(ref_pts, *debug, &reject_reason)) {
      last_good_reference_pts_ = ref_pts;
      last_good_reference_stamp_ = now;
      has_last_good_reference_ = true;
      last_good_reference_ego_pose_ = ego_now;
      last_good_reference_ego_valid_ = ego_now_valid;
      debug->reference_reason = "publish_good";
      debug->reference_degraded_mode = false;
      debug->reference_control_valid = true;
      debug->reference_held = false;
      debug->reference_hold_age = 0.0;
      debug->reconstruction_state = "TRACK_GOOD";
      debug->reference_span = polylineLengthLocal(ref_pts);
      return ref_pts;
    }

    const bool can_hold = has_last_good_reference_ &&
                          (now - last_good_reference_stamp_).toSec() <= reference_publish_hold_timeout_;
    debug->reference_degraded_mode = true;
    debug->reference_control_valid = can_hold;
    debug->reference_held = can_hold;
    debug->reference_hold_age = has_last_good_reference_ ? (now - last_good_reference_stamp_).toSec() : -1.0;
    debug->reference_reason = can_hold ? ("publish_hold_last_good_" + reject_reason)
                                       : ("publish_empty_" + reject_reason);
    if (can_hold) {
      std::vector<LocalPathPoint> held = last_goodReferenceInCurrentEgo(ego_now, ego_now_valid);
      debug->reconstruction_state = "GEOMETRY_HOLD";
      debug->reference_span = polylineLengthLocal(held);
      return held;
    }
    debug->reconstruction_state = "GEOMETRY_HOLD";
    debug->reference_span = 0.0;
    return std::vector<LocalPathPoint>();
  }

  std::vector<LocalPathPoint> last_goodReferenceInCurrentEgo(const EgoPose2D& ego_now,
                                                             bool ego_now_valid) const {
    if (!ego_now_valid || !last_good_reference_ego_valid_) {
      return last_good_reference_pts_;
    }
    std::vector<LocalPathPoint> out = last_good_reference_pts_;
    for (size_t i = 0; i < out.size(); ++i) {
      double wx = 0.0;
      double wy = 0.0;
      transformLocalToWorld(last_good_reference_ego_pose_, last_good_reference_pts_[i].x, last_good_reference_pts_[i].y,
                            &wx, &wy);
      transformWorldToLocal(ego_now, wx, wy, &out[i].x, &out[i].y);
      out[i].yaw = wrapAngle(last_good_reference_ego_pose_.yaw + last_good_reference_pts_[i].yaw - ego_now.yaw);
    }
    return out;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();

    std::vector<LocalPathPoint> raw_pts;
    std::vector<LocalPathPoint> ref_pts;
    std::vector<RawTrailPoint> raw_trail;
    std::vector<HistoryPoint2D> world_history;
    std::vector<HistoryPoint2D> world_history_log;
    bool lock_active = false;
    bool lead_moving = false;
    bool ego_standstill = false;
    double lead_x = 0.0;
    double lead_y = 0.0;
    bool lead_endpoint_valid = false;
    double speed = 0.0;
    double yaw_rate = 0.0;
    double wheel_speed = 0.0;
    double speed_hint = 0.0;
    double system_speed = 0.0;
    double ned_odom_speed = 0.0;
    double ned_odom_pose_delta_speed = 0.0;
    std::string ego_speed_source = "none";
    std::string ego_speed_source_type = "none";
    double ego_speed_source_age = -1.0;
    double steer = 0.0;
    double imu_yaw_rate = 0.0;
    double imu_yaw_rate_filtered = 0.0;
    bool imu_fresh = false;
    double innovation = 0.0;
    double metric = 0.0;
    uint64_t obs_ver = 0;
    TrackerDebugState debug_snapshot;
    LeadFilterSnapshot filter_snapshot;
    EgoPose2D ego_now_for_publish;
    bool ego_now_valid_for_publish = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ego_motion_.integrateTo(now);
      tracker_.markLostIfExpired(now);
      tracker_.pruneHistory(now);

      EgoPose2D ego_now;
      if (ego_motion_.queryPoseAt(now, &ego_now)) {
        if (!ego_origin_initialized_) {
          ego_origin_pose_ = ego_now;
          ego_origin_initialized_ = true;
        }
        ego_now_for_publish = ego_now;
        ego_now_valid_for_publish = true;

        world_history = tracker_.worldHistoryForPublish(now);
        world_history_log = tracker_.worldHistoryForLog(now);
        debug_state_.history_size = world_history.size();
        if (tracker_.hasActiveTrack()) {
          const LeadState2D pred = predictLeadState(tracker_.track(), now);
          transformWorldToLocal(ego_now, pred.x, pred.y, &lead_x, &lead_y);
          lead_endpoint_valid = true;
        }
        raw_trail = path_generator_.buildRawTrail(world_history, &debug_state_);
        raw_pts = path_generator_.buildRawPath(world_history,
                                               ego_now,
                                               lead_endpoint_valid,
                                               lead_x,
                                               lead_y,
                                               &debug_state_);
        ref_pts = path_generator_.buildReferencePath(raw_trail,
                                                     raw_pts,
                                                     ego_now,
                                                     lead_endpoint_valid,
                                                     lead_x,
                                                     lead_y,
                                                     &debug_state_);
        lead_filter_.snapshot(ego_now, &filter_snapshot);
      }

      lock_active = tracker_.hasActiveTrack();
      lead_moving = tracker_.movingFlag();
      speed = ego_motion_.fusedSpeed();
      yaw_rate = ego_motion_.currentYawRate();
      wheel_speed = ego_motion_.wheelSpeed();
      speed_hint = ego_motion_.speedHint();
      system_speed = ego_motion_.systemSpeed();
      ned_odom_speed = ego_motion_.nedOdometrySpeed();
      ned_odom_pose_delta_speed = ego_motion_.nedOdometryPoseDeltaSpeed();
      ego_speed_source = ego_motion_.speedSource();
      ego_speed_source_type = ego_motion_.speedSourceType();
      ego_speed_source_age = ego_motion_.speedSourceAge(now);
      steer = ego_motion_.steerAngle();
      imu_yaw_rate = ego_motion_.imuYawRate();
      imu_yaw_rate_filtered = ego_motion_.filteredImuYawRate();
      imu_fresh = ego_motion_.usingFreshImuYawRate();
      innovation = tracker_.innovationNorm();
      metric = tracker_.innovationMetric();
      obs_ver = obs_version_;
      ego_standstill = std::fabs(speed) < min_motion_speed_for_yaw_ * 0.9;
      debug_snapshot = debug_state_;
      debug_snapshot.ego_speed_source = ego_speed_source;
      debug_snapshot.ego_speed_source_type = ego_speed_source_type;
      debug_snapshot.ego_speed_source_age = ego_speed_source_age;
      debug_snapshot.ned_odom_received = ego_motion_.nedOdometryReceived();
      debug_snapshot.ned_odom_zero_motion = ego_motion_.nedOdometryZeroMotion();
      debug_snapshot.wheel_speed_raw = wheel_speed;
      debug_snapshot.carvel_speed_raw = speed_hint;
      debug_snapshot.system_speed_raw = system_speed;
      debug_snapshot.ned_odom_speed_raw = ned_odom_speed;
      debug_snapshot.odom_pose_delta_speed = ned_odom_pose_delta_speed;
      debug_snapshot.speed_source_valid = ego_speed_source != "none" && ego_speed_source_age >= 0.0;
      debug_snapshot.ego_translation_observable = ego_motion_.egoTranslationObservable();
      debug_snapshot.ego_steer_sign_for_reconstruction = steer_sign_;
      if (ego_speed_source == "none") {
        ROS_WARN_THROTTLE(
            1.0,
            "trajectory_reconstructor ego translation source is none(type=%s ned_rx=%d ned_zero=%d); "
            "rear-car translation cannot be fully decoupled. Lead-only bootstrap may start history, "
            "but confirmed append stays conservative until wheel/system_speed/NED_odometry becomes valid.",
            ego_speed_source_type.c_str(),
            debug_snapshot.ned_odom_received ? 1 : 0,
            debug_snapshot.ned_odom_zero_motion ? 1 : 0);
      }
    }

    ref_pts = selectReferenceForPublish(ref_pts,
                                        now,
                                        ego_now_for_publish,
                                        ego_now_valid_for_publish,
                                        &debug_snapshot);
    raw_pub_.publish(makePathMsg(raw_pts, base_frame_, now));
    reference_pub_.publish(makePathMsg(ref_pts, base_frame_, now, debug_snapshot.reference_held));
    world_confirmed_pub_.publish(makeWorldPathMsg(world_history_log, odom_frame_, now));
    publishLeadFilterOutputs(filter_snapshot, now);

    appendReconstructedTrajectoryToFile(now, raw_pts);
    appendHistoryTrajectoryToFile(world_history_log);
    appendRuntimeDebugLog(now,
                          lock_active,
                          lead_moving,
                          ego_standstill,
                          raw_pts.size(),
                          ref_pts.size(),
                          world_history_log.size(),
                          lead_x,
                          lead_y,
                          speed,
                          yaw_rate,
                          wheel_speed,
                          speed_hint,
                          system_speed,
                          ned_odom_speed,
                          ego_speed_source,
                          ego_speed_source_age,
                          steer,
                          imu_fresh,
                          imu_yaw_rate,
                          imu_yaw_rate_filtered,
                          innovation,
                          metric,
                          obs_ver,
                          debug_snapshot);
    appendHistoryStatusLog(now,
                           lock_active,
                           lead_moving,
                           raw_pts.size(),
                           ref_pts.size(),
                           world_history_log.size(),
                           debug_snapshot);
    appendLeadFilterStateLog(now, filter_snapshot);

    ROS_INFO_THROTTLE(
        1.0,
        "traj_ff | lock=%d moving=%d standstill=%d raw=%zu ref=%zu world=%zu x=%.2f y=%.2f v=%.3f src=%s age=%.2f yaw=%.3f wheel=%.3f hint=%.3f sys=%.3f ned=%.3f nedp=%.3f steer=%.3f imu=%d imu_z=%.3f imu_f=%.3f imu_act=%.3f motion=%s rev=%.3f filt_conf=%.2f filt_sig=%.3f filt_hsig=%.3f filt_v=%.3f filt_res=%.3f fdamp=%d innov=%.3f metric=%.3f ff_speed=%.3f ego_v=%.3f ego_obs=%d r_static=%.3f r_ego=%.3f r_dyn=%.3f r_head=%.3f mix=%d static=%d hs=%.3f hd=%.3f acc_s=%.3f acc_t=%.3f conf=%zu tail=%zu obs_ver=%llu poses=%d cand=%d lockrej=%d hist=%zu raw_span=%.2f ref_span=%.2f clip=[%.2f,%.2f] clipr=%s clips=%s clipe=%s hold=%d ed=%.2f eq=%s curve=%d rec=%d hold_age=%.2f arc=%.2f hcons=%.2f recr=%s ss=%.2f es_raw=%.2f es=%.2f reorder=%d holdr=%s join=%.2f jm=%s jl=%d conn=%d prune=%d proj=%s histr=%s rawr=%s refr=%s",
        lock_active ? 1 : 0,
        lead_moving ? 1 : 0,
        ego_standstill ? 1 : 0,
        raw_pts.size(),
        ref_pts.size(),
        world_history_log.size(),
        lead_x,
        lead_y,
        speed,
        ego_speed_source.c_str(),
        ego_speed_source_age,
        yaw_rate,
        wheel_speed,
        speed_hint,
        system_speed,
        ned_odom_speed,
        ned_odom_pose_delta_speed,
        steer,
        imu_fresh ? 1 : 0,
        imu_yaw_rate,
        imu_yaw_rate_filtered,
        debug_snapshot.imu_yaw_activity,
        debug_snapshot.motion_class.c_str(),
        debug_snapshot.reverse_progress,
        debug_snapshot.lead_filter_confidence,
        debug_snapshot.lead_filter_pos_sigma,
        debug_snapshot.lead_filter_heading_sigma,
        debug_snapshot.lead_filter_speed,
        debug_snapshot.lead_filter_residual,
        debug_snapshot.filter_velocity_damped ? 1 : 0,
        innovation,
        metric,
        debug_snapshot.ff_speed,
        debug_snapshot.mean_ego_speed,
        debug_snapshot.ego_translation_observable ? 1 : 0,
        debug_snapshot.r_static,
        debug_snapshot.r_ego,
        debug_snapshot.r_dynamic,
        debug_snapshot.r_heading_static,
        debug_snapshot.ego_yaw_mixing ? 1 : 0,
        debug_snapshot.static_mode ? 1 : 0,
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
        debug_snapshot.clip_start_s,
        debug_snapshot.clip_end_s,
        debug_snapshot.clip_reason.c_str(),
        debug_snapshot.clip_start_source.c_str(),
        debug_snapshot.clip_end_source.c_str(),
        debug_snapshot.clip_hold ? 1 : 0,
        debug_snapshot.endpoint_proj_dist,
        debug_snapshot.endpoint_quality.c_str(),
        debug_snapshot.curve_mode ? 1 : 0,
        debug_snapshot.endpoint_recapture_frames,
        debug_snapshot.endpoint_hold_age,
        debug_snapshot.curve_arc_progress,
        debug_snapshot.curve_heading_consistency,
        debug_snapshot.recapture_reason.c_str(),
        debug_snapshot.clip_start_proj_dist,
        debug_snapshot.clip_endpoint_s_raw,
        debug_snapshot.clip_endpoint_s_filtered,
        debug_snapshot.clip_reorder_used ? 1 : 0,
        debug_snapshot.clip_hold_reason.c_str(),
        debug_snapshot.reference_join_s,
        debug_snapshot.reference_join_mode.c_str(),
        debug_snapshot.reference_join_step_limited ? 1 : 0,
        debug_snapshot.reference_connector_points,
        debug_snapshot.confirmed_prune_count,
        debug_snapshot.projector_reason.c_str(),
        debug_snapshot.history_reason.c_str(),
        debug_snapshot.raw_reason.c_str(),
        debug_snapshot.reference_reason.c_str());
  }

  void publishLeadFilterOutputs(const LeadFilterSnapshot& snapshot, const ros::Time& stamp) {
    if (!snapshot.valid) {
      return;
    }

    geometry_msgs::TwistStamped state_msg;
    state_msg.header.stamp = stamp;
    state_msg.header.frame_id = base_frame_;
    state_msg.twist.linear.x = snapshot.local_x;
    state_msg.twist.linear.y = snapshot.local_y;
    state_msg.twist.linear.z = snapshot.speed;
    state_msg.twist.angular.x = snapshot.confidence;
    state_msg.twist.angular.y = snapshot.position_sigma;
    state_msg.twist.angular.z = snapshot.relative_heading;
    lead_filter_state_pub_.publish(state_msg);

    geometry_msgs::Vector3Stamped confidence_msg;
    confidence_msg.header = state_msg.header;
    confidence_msg.vector.x = snapshot.confidence;
    confidence_msg.vector.y = snapshot.position_sigma;
    confidence_msg.vector.z = snapshot.heading_sigma;
    lead_filter_confidence_pub_.publish(confidence_msg);
  }

  std::string debugLogPath(const std::string& file_name) const {
    if (debug_log_dir_.empty()) {
      return file_name;
    }
    if (debug_log_dir_[debug_log_dir_.size() - 1] == '/') {
      return debug_log_dir_ + file_name;
    }
    return debug_log_dir_ + "/" + file_name;
  }

  void ensureDebugLogDirectory() const {
    if (debug_log_dir_.empty()) {
      return;
    }
    if (mkdir(debug_log_dir_.c_str(), 0755) != 0 && errno != EEXIST) {
      ROS_WARN("trajectory_reconstructor failed to create debug log dir: %s errno=%d",
               debug_log_dir_.c_str(),
               errno);
    }
  }

  void openDebugLogStreams() {
    if (!enable_debug_csv_logs_) {
      return;
    }

    ensureDebugLogDirectory();
    openTrajectoryLogStream(
        debugLogPath("runtime_debug.csv"),
        "stamp,lock_active,lead_moving,ego_standstill,raw_points,ref_points,world_points,"
        "lead_x,lead_y,ego_speed,ego_yaw_rate,wheel_speed,speed_hint,steer,imu_fresh,"
        "imu_yaw_rate,imu_yaw_rate_filtered,lead_filter_confidence,lead_filter_pos_sigma,"
        "lead_filter_heading_sigma,lead_filter_speed,lead_filter_residual,innovation,"
        "innovation_metric,ff_speed,mean_ego_speed,static_hyp_res,ego_motion_explained_res,"
        "r_static,r_ego,r_dynamic,r_heading_static,imu_yaw_activity,reverse_progress,"
        "motion_class,ego_yaw_mixing,static_mode,ego_translation_observable,"
        "relative_shortening_only,confirmed_append_allowed,confirmed_reject_reason,"
        "candidate_motion_class,candidate_dynamic_residual_win,"
        "candidate_ego_translation_observable,tail_admit_reason,confirmed_segment_id,"
        "history_segment_break,reference_degraded_mode,reconstruction_state,reference_held,"
        "reference_hold_age,raw_segment_count,raw_selected_segment_score,"
        "raw_selected_segment_reason,history_project_progress,history_loop_reject,history_tail_only_count,"
        "hs_score,hd_score,accum_s,accum_t,confirmed_count,tail_count,obs_version,"
        "input_pose_count,valid_candidate_count,lock_reject_count,history_size,raw_span,"
        "reference_span,clip_start_s,clip_end_s,clip_start_source,clip_end_source,clip_hold,"
        "endpoint_proj_dist,endpoint_quality,curve_mode,endpoint_recapture_frames,"
        "endpoint_hold_age,curve_arc_progress,curve_heading_consistency,recapture_reason,"
        "clip_start_proj_dist,clip_endpoint_s_raw,clip_endpoint_s_filtered,"
        "clip_reorder_used,clip_hold_reason,endpoint_recapture_mode,clip_stale_age,"
        "reference_join_s,reference_join_mode,"
        "reference_join_step_limited,reference_connector_points,confirmed_prune_count,"
        "filter_velocity_damped,"
        "raw_world_points,raw_local_points_before_clip,"
        "ego_speed_source,ego_speed_source_type,ego_speed_source_age,speed_source_valid,"
        "ned_odom_received,ned_odom_zero_motion,lead_only_bootstrap,"
        "lead_fusion_res_static,lead_fusion_res_dynamic,static_prob,dynamic_prob,fusion_cov_trace,"
        "fusion_static_likelihood,fusion_dynamic_likelihood,fusion_ego_likelihood,"
        "fusion_motion_score,fusion_confirm_accum_s,fusion_confirm_accum_t,"
        "fusion_pos_cov,fusion_vel_cov,low_speed_motion_score,speed_clamped,"
        "raw_clip_min_span_applied,raw_clip_collapse_prevented,"
        "dynamic_residual_win_now,ego_explained_win_now,yaw_mix_freeze,"
        "signed_wheel_speed_used,confirmed_freeze_reason,"
        "anchor_epoch_id,anchor_rebase_dx,anchor_rebase_dy,anchor_rebase_dyaw,"
        "segment_switch_reason,raw_front_jump,raw_heading_jump_count,"
        "reference_continuity_limited,reference_control_valid,"
        "raw_clip_policy,ego_progress_s,history_tail_s,lead_endpoint_s,"
        "clip_end_limited_by_endpoint,raw_preserved_by_history_tail,control_feasible_start_s,"
        "history_geometry_reject_count,history_kink_angle,history_reverse_append_reject,"
        "confirmed_append_geometry_valid,ego_steer_sign_for_reconstruction,"
        "wheel_speed_raw,carvel_speed_raw,system_speed_raw,ned_odom_speed_raw,odom_pose_delta_speed,"
        "projector_reason,history_reason,raw_reason,reference_reason,clip_reason",
        &runtime_debug_log_stream_);
    openTrajectoryLogStream(
        debugLogPath("observation_filter.csv"),
        "stamp,raw_local_x,raw_local_y,raw_local_heading,raw_world_x,raw_world_y,"
        "raw_world_heading,raw_range,filtered_local_x,filtered_local_y,filtered_local_heading,"
        "filtered_world_x,filtered_world_y,filtered_world_heading,filtered_vx,filtered_vy,"
        "filtered_speed,confidence,position_sigma,heading_sigma,speed_sigma,robust_weight,"
        "filter_residual,ego_x,ego_y,ego_yaw,ego_yaw_rate,ego_sigma_yaw,input_pose_count,"
        "valid_candidate_count,lock_reject_count,projector_reason,history_reason",
        &observation_filter_log_stream_);
    openTrajectoryLogStream(
        debugLogPath("history_status.csv"),
        "stamp,lock_active,lead_moving,raw_points,ref_points,world_points,history_size,"
        "confirmed_count,tail_count,raw_span,reference_span,ff_speed,mean_ego_speed,"
        "lead_filter_speed,lead_filter_confidence,lead_filter_pos_sigma,static_hyp_res,"
        "ego_motion_explained_res,r_static,r_ego,r_dynamic,r_heading_static,imu_yaw_activity,"
        "reverse_progress,motion_class,ego_yaw_mixing,"
        "static_mode,ego_translation_observable,relative_shortening_only,"
        "confirmed_append_allowed,confirmed_reject_reason,"
        "candidate_motion_class,candidate_dynamic_residual_win,"
        "candidate_ego_translation_observable,tail_admit_reason,confirmed_segment_id,"
        "history_segment_break,reference_degraded_mode,reconstruction_state,reference_held,"
        "reference_hold_age,raw_segment_count,raw_selected_segment_score,"
        "raw_selected_segment_reason,history_project_progress,history_loop_reject,history_tail_only_count,"
        "hs_score,hd_score,accum_s,accum_t,clip_start_s,clip_end_s,"
        "clip_start_source,clip_end_source,clip_hold,endpoint_proj_dist,endpoint_quality,"
        "curve_mode,endpoint_recapture_frames,endpoint_hold_age,curve_arc_progress,"
        "curve_heading_consistency,recapture_reason,"
        "clip_start_proj_dist,clip_endpoint_s_raw,clip_endpoint_s_filtered,"
        "clip_reorder_used,clip_hold_reason,endpoint_recapture_mode,clip_stale_age,"
        "reference_join_s,reference_join_mode,reference_join_step_limited,"
        "reference_connector_points,confirmed_prune_count,filter_velocity_damped,"
        "raw_world_points,raw_local_points_before_clip,"
        "ego_speed_source,ego_speed_source_type,ego_speed_source_age,speed_source_valid,"
        "ned_odom_received,ned_odom_zero_motion,lead_only_bootstrap,"
        "lead_fusion_res_static,lead_fusion_res_dynamic,static_prob,dynamic_prob,fusion_cov_trace,"
        "fusion_static_likelihood,fusion_dynamic_likelihood,fusion_ego_likelihood,"
        "fusion_motion_score,fusion_confirm_accum_s,fusion_confirm_accum_t,"
        "fusion_pos_cov,fusion_vel_cov,low_speed_motion_score,speed_clamped,"
        "raw_clip_min_span_applied,raw_clip_collapse_prevented,"
        "dynamic_residual_win_now,ego_explained_win_now,yaw_mix_freeze,"
        "signed_wheel_speed_used,confirmed_freeze_reason,"
        "anchor_epoch_id,anchor_rebase_dx,anchor_rebase_dy,anchor_rebase_dyaw,"
        "segment_switch_reason,raw_front_jump,raw_heading_jump_count,"
        "reference_continuity_limited,reference_control_valid,"
        "raw_clip_policy,ego_progress_s,history_tail_s,lead_endpoint_s,"
        "clip_end_limited_by_endpoint,raw_preserved_by_history_tail,control_feasible_start_s,"
        "history_geometry_reject_count,history_kink_angle,history_reverse_append_reject,"
        "confirmed_append_geometry_valid,ego_steer_sign_for_reconstruction,"
        "wheel_speed_raw,carvel_speed_raw,system_speed_raw,ned_odom_speed_raw,odom_pose_delta_speed,projector_reason,"
        "history_reason,raw_reason,reference_reason,clip_reason",
        &history_status_log_stream_);
    openTrajectoryLogStream(
        debugLogPath("lead_filter_state.csv"),
        "stamp,valid,snapshot_stamp,local_x,local_y,relative_heading,speed,confidence,"
        "position_sigma,heading_sigma",
        &lead_filter_state_log_stream_);
  }

  void appendRuntimeDebugLog(const ros::Time& stamp,
                             bool lock_active,
                             bool lead_moving,
                             bool ego_standstill,
                             size_t raw_points,
                             size_t ref_points,
                             size_t world_points,
                             double lead_x,
                             double lead_y,
                             double ego_speed,
                             double ego_yaw_rate,
                             double wheel_speed,
                             double speed_hint,
                             double system_speed,
                             double ned_odom_speed,
                             const std::string& ego_speed_source,
                             double ego_speed_source_age,
                             double steer,
                             bool imu_fresh,
                             double imu_yaw_rate,
                             double imu_yaw_rate_filtered,
                             double innovation,
                             double innovation_metric,
                             uint64_t obs_version,
                             const TrackerDebugState& debug) {
    if (!runtime_debug_log_stream_.is_open()) {
      return;
    }

    runtime_debug_log_stream_ << std::fixed << std::setprecision(6) << stamp.toSec() << ","
                              << (lock_active ? 1 : 0) << ","
                              << (lead_moving ? 1 : 0) << ","
                              << (ego_standstill ? 1 : 0) << ","
                              << raw_points << ","
                              << ref_points << ","
                              << world_points << ","
                              << lead_x << ","
                              << lead_y << ","
                              << ego_speed << ","
                              << ego_yaw_rate << ","
                              << wheel_speed << ","
                              << speed_hint << ","
                              << steer << ","
                              << (imu_fresh ? 1 : 0) << ","
                              << imu_yaw_rate << ","
                              << imu_yaw_rate_filtered << ","
                              << debug.lead_filter_confidence << ","
                              << debug.lead_filter_pos_sigma << ","
                              << debug.lead_filter_heading_sigma << ","
                              << debug.lead_filter_speed << ","
                              << debug.lead_filter_residual << ","
                              << innovation << ","
                              << innovation_metric << ","
                              << debug.ff_speed << ","
                              << debug.mean_ego_speed << ","
                              << debug.static_hyp_res << ","
                              << debug.ego_motion_explained_res << ","
                              << debug.r_static << ","
                              << debug.r_ego << ","
                              << debug.r_dynamic << ","
                              << debug.r_heading_static << ","
                              << debug.imu_yaw_activity << ","
                              << debug.reverse_progress << ","
                              << debug.motion_class << ","
                              << (debug.ego_yaw_mixing ? 1 : 0) << ","
                              << (debug.static_mode ? 1 : 0) << ","
                              << (debug.ego_translation_observable ? 1 : 0) << ","
                              << (debug.relative_shortening_only ? 1 : 0) << ","
                              << (debug.confirmed_append_allowed ? 1 : 0) << ","
                              << debug.confirmed_reject_reason << ","
                              << debug.candidate_motion_class << ","
                              << (debug.candidate_dynamic_residual_win ? 1 : 0) << ","
                              << (debug.candidate_ego_translation_observable ? 1 : 0) << ","
                              << debug.tail_admit_reason << ","
                              << debug.confirmed_segment_id << ","
                              << (debug.history_segment_break ? 1 : 0) << ","
                              << (debug.reference_degraded_mode ? 1 : 0) << ","
                              << debug.reconstruction_state << ","
                              << (debug.reference_held ? 1 : 0) << ","
                              << debug.reference_hold_age << ","
                              << debug.raw_segment_count << ","
                              << debug.raw_selected_segment_score << ","
                              << debug.raw_selected_segment_reason << ","
                              << debug.history_project_progress << ","
                              << (debug.history_loop_reject ? 1 : 0) << ","
                              << debug.history_tail_only_count << ","
                              << debug.hs_score << ","
                              << debug.hd_score << ","
                              << debug.accum_s << ","
                              << debug.accum_t << ","
                              << debug.confirmed_count << ","
                              << debug.tail_count << ","
                              << static_cast<unsigned long long>(obs_version) << ","
                              << debug.input_pose_count << ","
                              << debug.valid_candidate_count << ","
                              << debug.lock_reject_count << ","
                              << debug.history_size << ","
                              << debug.raw_span << ","
                              << debug.reference_span << ","
                              << debug.clip_start_s << ","
                              << debug.clip_end_s << ","
                              << debug.clip_start_source << ","
                              << debug.clip_end_source << ","
                              << (debug.clip_hold ? 1 : 0) << ","
                              << debug.endpoint_proj_dist << ","
                              << debug.endpoint_quality << ","
                              << (debug.curve_mode ? 1 : 0) << ","
                              << debug.endpoint_recapture_frames << ","
                              << debug.endpoint_hold_age << ","
                              << debug.curve_arc_progress << ","
                              << debug.curve_heading_consistency << ","
                              << debug.recapture_reason << ","
                              << debug.clip_start_proj_dist << ","
                              << debug.clip_endpoint_s_raw << ","
                              << debug.clip_endpoint_s_filtered << ","
                              << (debug.clip_reorder_used ? 1 : 0) << ","
                              << debug.clip_hold_reason << ","
                              << debug.endpoint_recapture_mode << ","
                              << debug.clip_stale_age << ","
                              << debug.reference_join_s << ","
                              << debug.reference_join_mode << ","
                              << (debug.reference_join_step_limited ? 1 : 0) << ","
                              << debug.reference_connector_points << ","
                              << debug.confirmed_prune_count << ","
                              << (debug.filter_velocity_damped ? 1 : 0) << ","
                              << debug.raw_world_points << ","
                              << debug.raw_local_points_before_clip << ","
                              << ego_speed_source << ","
                              << debug.ego_speed_source_type << ","
                              << ego_speed_source_age << ","
                              << (debug.speed_source_valid ? 1 : 0) << ","
                              << (debug.ned_odom_received ? 1 : 0) << ","
                              << (debug.ned_odom_zero_motion ? 1 : 0) << ","
                              << (debug.lead_only_bootstrap ? 1 : 0) << ","
                              << debug.lead_fusion_res_static << ","
                              << debug.lead_fusion_res_dynamic << ","
                              << debug.static_prob << ","
                              << debug.dynamic_prob << ","
                              << debug.fusion_cov_trace << ","
                              << debug.fusion_static_likelihood << ","
                              << debug.fusion_dynamic_likelihood << ","
                              << debug.fusion_ego_likelihood << ","
                              << debug.fusion_motion_score << ","
                              << debug.fusion_confirm_accum_s << ","
                              << debug.fusion_confirm_accum_t << ","
                              << debug.fusion_pos_cov << ","
                              << debug.fusion_vel_cov << ","
                              << debug.low_speed_motion_score << ","
                              << (debug.speed_clamped ? 1 : 0) << ","
                              << (debug.raw_clip_min_span_applied ? 1 : 0) << ","
                              << (debug.raw_clip_collapse_prevented ? 1 : 0) << ","
                              << (debug.dynamic_residual_win_now ? 1 : 0) << ","
                              << (debug.ego_explained_win_now ? 1 : 0) << ","
                              << (debug.yaw_mix_freeze ? 1 : 0) << ","
                              << (debug.signed_wheel_speed_used ? 1 : 0) << ","
                              << debug.confirmed_freeze_reason << ","
                              << debug.anchor_epoch_id << ","
                              << debug.anchor_rebase_dx << ","
                              << debug.anchor_rebase_dy << ","
                              << debug.anchor_rebase_dyaw << ","
                              << debug.segment_switch_reason << ","
                              << debug.raw_front_jump << ","
                              << debug.raw_heading_jump_count << ","
                              << (debug.reference_continuity_limited ? 1 : 0) << ","
                              << (debug.reference_control_valid ? 1 : 0) << ","
                              << debug.raw_clip_policy << ","
                              << debug.ego_progress_s << ","
                              << debug.history_tail_s << ","
                              << debug.lead_endpoint_s << ","
                              << (debug.clip_end_limited_by_endpoint ? 1 : 0) << ","
                              << (debug.raw_preserved_by_history_tail ? 1 : 0) << ","
                              << debug.control_feasible_start_s << ","
                              << debug.history_geometry_reject_count << ","
                              << debug.history_kink_angle << ","
                              << (debug.history_reverse_append_reject ? 1 : 0) << ","
                              << (debug.confirmed_append_geometry_valid ? 1 : 0) << ","
                              << debug.ego_steer_sign_for_reconstruction << ","
                              << wheel_speed << ","
                              << speed_hint << ","
                              << system_speed << ","
                              << ned_odom_speed << ","
                              << debug.odom_pose_delta_speed << ","
                              << debug.projector_reason << ","
                              << debug.history_reason << ","
                              << debug.raw_reason << ","
                              << debug.reference_reason << ","
                              << debug.clip_reason << "\n";
    runtime_debug_log_stream_.flush();
  }

  void appendObservationFilterLog(const LeadObservation& raw,
                                  const LeadObservation& filtered,
                                  const EgoPose2D& ego_at_obs,
                                  const TrackerDebugState& debug) {
    if (!observation_filter_log_stream_.is_open()) {
      return;
    }

    observation_filter_log_stream_ << std::fixed << std::setprecision(6) << filtered.stamp.toSec() << ","
                                   << raw.local_x << ","
                                   << raw.local_y << ","
                                   << raw.local_heading << ","
                                   << raw.world_x << ","
                                   << raw.world_y << ","
                                   << raw.world_heading << ","
                                   << raw.range << ","
                                   << filtered.local_x << ","
                                   << filtered.local_y << ","
                                   << filtered.local_heading << ","
                                   << filtered.world_x << ","
                                   << filtered.world_y << ","
                                   << filtered.world_heading << ","
                                   << filtered.world_vx << ","
                                   << filtered.world_vy << ","
                                   << filtered.speed << ","
                                   << filtered.confidence << ","
                                   << filtered.position_sigma << ","
                                   << filtered.heading_sigma << ","
                                   << filtered.speed_sigma << ","
                                   << filtered.robust_weight << ","
                                   << filtered.filter_residual << ","
                                   << ego_at_obs.x << ","
                                   << ego_at_obs.y << ","
                                   << ego_at_obs.yaw << ","
                                   << ego_at_obs.yaw_rate << ","
                                   << ego_at_obs.sigma_yaw << ","
                                   << debug.input_pose_count << ","
                                   << debug.valid_candidate_count << ","
                                   << debug.lock_reject_count << ","
                                   << debug.projector_reason << ","
                                   << debug.history_reason << "\n";
    observation_filter_log_stream_.flush();
  }

  void appendHistoryStatusLog(const ros::Time& stamp,
                              bool lock_active,
                              bool lead_moving,
                              size_t raw_points,
                              size_t ref_points,
                              size_t world_points,
                              const TrackerDebugState& debug) {
    if (!history_status_log_stream_.is_open()) {
      return;
    }

    history_status_log_stream_ << std::fixed << std::setprecision(6) << stamp.toSec() << ","
                               << (lock_active ? 1 : 0) << ","
                               << (lead_moving ? 1 : 0) << ","
                               << raw_points << ","
                               << ref_points << ","
                               << world_points << ","
                               << debug.history_size << ","
                               << debug.confirmed_count << ","
                               << debug.tail_count << ","
                               << debug.raw_span << ","
                               << debug.reference_span << ","
                               << debug.ff_speed << ","
                               << debug.mean_ego_speed << ","
                               << debug.lead_filter_speed << ","
                               << debug.lead_filter_confidence << ","
                               << debug.lead_filter_pos_sigma << ","
                               << debug.static_hyp_res << ","
                               << debug.ego_motion_explained_res << ","
                               << debug.r_static << ","
                               << debug.r_ego << ","
                               << debug.r_dynamic << ","
                               << debug.r_heading_static << ","
                               << debug.imu_yaw_activity << ","
                               << debug.reverse_progress << ","
                               << debug.motion_class << ","
                               << (debug.ego_yaw_mixing ? 1 : 0) << ","
                               << (debug.static_mode ? 1 : 0) << ","
                               << (debug.ego_translation_observable ? 1 : 0) << ","
                               << (debug.relative_shortening_only ? 1 : 0) << ","
                               << (debug.confirmed_append_allowed ? 1 : 0) << ","
                               << debug.confirmed_reject_reason << ","
                               << debug.candidate_motion_class << ","
                               << (debug.candidate_dynamic_residual_win ? 1 : 0) << ","
                               << (debug.candidate_ego_translation_observable ? 1 : 0) << ","
                               << debug.tail_admit_reason << ","
                               << debug.confirmed_segment_id << ","
                               << (debug.history_segment_break ? 1 : 0) << ","
                               << (debug.reference_degraded_mode ? 1 : 0) << ","
                               << debug.reconstruction_state << ","
                               << (debug.reference_held ? 1 : 0) << ","
                               << debug.reference_hold_age << ","
                               << debug.raw_segment_count << ","
                               << debug.raw_selected_segment_score << ","
                               << debug.raw_selected_segment_reason << ","
                               << debug.history_project_progress << ","
                               << (debug.history_loop_reject ? 1 : 0) << ","
                               << debug.history_tail_only_count << ","
                               << debug.hs_score << ","
                               << debug.hd_score << ","
                               << debug.accum_s << ","
                               << debug.accum_t << ","
                               << debug.clip_start_s << ","
                               << debug.clip_end_s << ","
                               << debug.clip_start_source << ","
                               << debug.clip_end_source << ","
                               << (debug.clip_hold ? 1 : 0) << ","
                               << debug.endpoint_proj_dist << ","
                               << debug.endpoint_quality << ","
                               << (debug.curve_mode ? 1 : 0) << ","
                               << debug.endpoint_recapture_frames << ","
                               << debug.endpoint_hold_age << ","
                               << debug.curve_arc_progress << ","
                               << debug.curve_heading_consistency << ","
                               << debug.recapture_reason << ","
                               << debug.clip_start_proj_dist << ","
                               << debug.clip_endpoint_s_raw << ","
                               << debug.clip_endpoint_s_filtered << ","
                               << (debug.clip_reorder_used ? 1 : 0) << ","
                               << debug.clip_hold_reason << ","
                               << debug.endpoint_recapture_mode << ","
                               << debug.clip_stale_age << ","
                               << debug.reference_join_s << ","
                               << debug.reference_join_mode << ","
                               << (debug.reference_join_step_limited ? 1 : 0) << ","
                               << debug.reference_connector_points << ","
                               << debug.confirmed_prune_count << ","
                               << (debug.filter_velocity_damped ? 1 : 0) << ","
                               << debug.raw_world_points << ","
                               << debug.raw_local_points_before_clip << ","
                               << debug.ego_speed_source << ","
                               << debug.ego_speed_source_type << ","
                               << debug.ego_speed_source_age << ","
                               << (debug.speed_source_valid ? 1 : 0) << ","
                               << (debug.ned_odom_received ? 1 : 0) << ","
                               << (debug.ned_odom_zero_motion ? 1 : 0) << ","
                               << (debug.lead_only_bootstrap ? 1 : 0) << ","
                               << debug.lead_fusion_res_static << ","
                               << debug.lead_fusion_res_dynamic << ","
                               << debug.static_prob << ","
                               << debug.dynamic_prob << ","
                               << debug.fusion_cov_trace << ","
                               << debug.fusion_static_likelihood << ","
                               << debug.fusion_dynamic_likelihood << ","
                               << debug.fusion_ego_likelihood << ","
                               << debug.fusion_motion_score << ","
                               << debug.fusion_confirm_accum_s << ","
                               << debug.fusion_confirm_accum_t << ","
                               << debug.fusion_pos_cov << ","
                               << debug.fusion_vel_cov << ","
                               << debug.low_speed_motion_score << ","
                               << (debug.speed_clamped ? 1 : 0) << ","
                               << (debug.raw_clip_min_span_applied ? 1 : 0) << ","
                               << (debug.raw_clip_collapse_prevented ? 1 : 0) << ","
                               << (debug.dynamic_residual_win_now ? 1 : 0) << ","
                               << (debug.ego_explained_win_now ? 1 : 0) << ","
                               << (debug.yaw_mix_freeze ? 1 : 0) << ","
                               << (debug.signed_wheel_speed_used ? 1 : 0) << ","
                               << debug.confirmed_freeze_reason << ","
                               << debug.anchor_epoch_id << ","
                               << debug.anchor_rebase_dx << ","
                               << debug.anchor_rebase_dy << ","
                               << debug.anchor_rebase_dyaw << ","
                               << debug.segment_switch_reason << ","
                               << debug.raw_front_jump << ","
                               << debug.raw_heading_jump_count << ","
                               << (debug.reference_continuity_limited ? 1 : 0) << ","
                               << (debug.reference_control_valid ? 1 : 0) << ","
                               << debug.raw_clip_policy << ","
                               << debug.ego_progress_s << ","
                               << debug.history_tail_s << ","
                               << debug.lead_endpoint_s << ","
                               << (debug.clip_end_limited_by_endpoint ? 1 : 0) << ","
                               << (debug.raw_preserved_by_history_tail ? 1 : 0) << ","
                               << debug.control_feasible_start_s << ","
                               << debug.history_geometry_reject_count << ","
                               << debug.history_kink_angle << ","
                               << (debug.history_reverse_append_reject ? 1 : 0) << ","
                               << (debug.confirmed_append_geometry_valid ? 1 : 0) << ","
                               << debug.ego_steer_sign_for_reconstruction << ","
                               << debug.wheel_speed_raw << ","
                               << debug.carvel_speed_raw << ","
                               << debug.system_speed_raw << ","
                               << debug.ned_odom_speed_raw << ","
                               << debug.odom_pose_delta_speed << ","
                               << debug.projector_reason << ","
                               << debug.history_reason << ","
                               << debug.raw_reason << ","
                               << debug.reference_reason << ","
                               << debug.clip_reason << "\n";
    history_status_log_stream_.flush();
  }

  void appendLeadFilterStateLog(const ros::Time& stamp,
                                const LeadFilterSnapshot& snapshot) {
    if (!lead_filter_state_log_stream_.is_open()) {
      return;
    }

    lead_filter_state_log_stream_ << std::fixed << std::setprecision(6) << stamp.toSec() << ","
                                  << (snapshot.valid ? 1 : 0) << ","
                                  << (snapshot.stamp.isValid() ? snapshot.stamp.toSec() : 0.0) << ","
                                  << snapshot.local_x << ","
                                  << snapshot.local_y << ","
                                  << snapshot.relative_heading << ","
                                  << snapshot.speed << ","
                                  << snapshot.confidence << ","
                                  << snapshot.position_sigma << ","
                                  << snapshot.heading_sigma << "\n";
    lead_filter_state_log_stream_.flush();
  }

  void appendReconstructedTrajectoryToFile(const ros::Time& frame_stamp,
                                           const std::vector<LocalPathPoint>& raw_pts) {
    if (!reconstructed_traj_log_stream_.is_open() || raw_pts.empty()) {
      return;
    }

    std::vector<LocalPathPoint> compact_pts = compactLocalPath(raw_pts, 0.02);
    if (compact_pts.empty()) {
      return;
    }

    if (has_last_exported_reconstructed_ && sameLocalPath(compact_pts, last_exported_reconstructed_pts_)) {
      return;
    }

    reconstructed_traj_log_stream_ << std::fixed << std::setprecision(6)
                                   << "FRAME," << frame_stamp.toSec() << "," << compact_pts.size() << "\n";
    for (size_t i = 0; i < compact_pts.size(); ++i) {
      reconstructed_traj_log_stream_ << std::fixed << std::setprecision(6) << frame_stamp.toSec() << ","
                                     << compact_pts[i].x << ","
                                     << compact_pts[i].y << ","
                                     << compact_pts[i].yaw << "\n";
    }
    reconstructed_traj_log_stream_ << "END\n";
    reconstructed_traj_log_stream_.flush();
    last_exported_reconstructed_pts_ = compact_pts;
    has_last_exported_reconstructed_ = true;
  }

  void appendHistoryTrajectoryToFile(const std::vector<HistoryPoint2D>& world_history) {
    if (!history_traj_log_stream_.is_open() || world_history.empty() || !ego_origin_initialized_) {
      return;
    }

    for (size_t i = 0; i < world_history.size(); ++i) {
      if (world_history[i].stamp.isValid() && last_history_logged_stamp_.isValid() &&
          world_history[i].stamp <= last_history_logged_stamp_) {
        continue;
      }
      double rel_x = 0.0;
      double rel_y = 0.0;
      transformWorldToLocal(ego_origin_pose_, world_history[i].x, world_history[i].y, &rel_x, &rel_y);
      const double rel_yaw = wrapAngle(world_history[i].heading - ego_origin_pose_.yaw);
      history_traj_log_stream_ << std::fixed << std::setprecision(6) << world_history[i].stamp.toSec() << ","
                               << rel_x << ","
                               << rel_y << ","
                               << rel_yaw << "\n";
      if (world_history[i].stamp.isValid()) {
        last_history_logged_stamp_ = world_history[i].stamp;
      }
    }
    history_traj_log_stream_.flush();
  }

  void openTrajectoryLogStream(const std::string& file_path,
                               const std::string& header_line,
                               std::ofstream* stream) {
    if (stream == NULL) {
      return;
    }

    stream->open(file_path.c_str(), std::ios::out | std::ios::trunc);
    if (!stream->is_open()) {
      ROS_WARN("trajectory_reconstructor failed to open trajectory log file: %s", file_path.c_str());
    } else {
      *stream << header_line << "\n";
      stream->flush();
    }
  }

  bool sameLocalPath(const std::vector<LocalPathPoint>& a, const std::vector<LocalPathPoint>& b) const {
    if (a.size() != b.size()) {
      return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
      if (std::fabs(a[i].x - b[i].x) > 1e-4 ||
          std::fabs(a[i].y - b[i].y) > 1e-4 ||
          std::fabs(wrapAngle(a[i].yaw - b[i].yaw)) > 1e-4) {
        return false;
      }
    }
    return true;
  }

  std::vector<LocalPathPoint> compactLocalPath(const std::vector<LocalPathPoint>& in,
                                               double dist_eps) const {
    if (in.empty()) {
      return {};
    }

    std::vector<LocalPathPoint> out;
    out.reserve(in.size());
    out.push_back(in.front());
    for (size_t i = 1; i < in.size(); ++i) {
      const double dx = in[i].x - out.back().x;
      const double dy = in[i].y - out.back().y;
      if (std::sqrt(dx * dx + dy * dy) >= dist_eps) {
        out.push_back(in[i]);
      }
    }
    return out;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber tracked_sub_;
  ros::Subscriber car_sub_;
  ros::Subscriber carvel_sub_;
  ros::Subscriber system_speed_sub_;
  ros::Subscriber ned_odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher raw_pub_;
  ros::Publisher reference_pub_;
  ros::Publisher world_confirmed_pub_;
  ros::Publisher lead_filter_state_pub_;
  ros::Publisher lead_filter_confidence_pub_;
  ros::Timer timer_;
  std::mutex mutex_;

  std::string base_frame_;
  std::string odom_frame_;
  std::string tracked_topic_;
  std::string car_topic_;
  std::string carvel_topic_;
  std::string system_speed_topic_;
  std::string ned_odom_topic_;
  std::string imu_topic_;
  std::string raw_topic_;
  std::string reference_topic_;
  std::string system_speed_msg_type_;
  std::string world_confirmed_topic_;
  std::string lead_filter_state_topic_;
  std::string lead_filter_confidence_topic_;
  std::string reconstructed_traj_log_file_;
  std::string history_traj_log_file_;
  std::string debug_log_dir_;

  double publish_rate_hz_ = 30.0;
  double wheel_speed_scale_ = 0.001;
  double speed_hint_scale_ = 0.001;
  double system_speed_scale_ = 1.0;
  double ned_odom_speed_scale_ = 1.0;
  double steer_angle_scale_ = 1.0;
  double steer_angle_bias_ = 0.0;
  double steer_sign_ = -1.0;
  double steer_deadband_rad_ = 0.002;
  double imu_yaw_rate_scale_ = 1.0;
  double imu_yaw_rate_bias_ = 0.0;
  double imu_yaw_rate_sign_ = 1.0;
  double lost_timeout_ = 0.60;
  double min_motion_speed_for_yaw_ = 0.15;
  bool allow_unstamped_tracked_fallback_ = false;
  bool enable_debug_csv_logs_ = true;
  bool reference_publish_quality_gate_enabled_ = true;
  bool reference_publish_require_control_valid_ = true;
  bool reference_publish_hold_on_short_endpoint_ = true;
  int reference_publish_max_heading_jump_count_ = 0;
  int reference_publish_min_points_ = 4;
  double reference_publish_min_span_ = 0.45;
  double reference_publish_hold_timeout_ = 0.45;
  bool ego_origin_initialized_ = false;
  EgoPose2D ego_origin_pose_;
  std::ofstream reconstructed_traj_log_stream_;
  std::ofstream history_traj_log_stream_;
  std::ofstream runtime_debug_log_stream_;
  std::ofstream observation_filter_log_stream_;
  std::ofstream history_status_log_stream_;
  std::ofstream lead_filter_state_log_stream_;
  std::vector<LocalPathPoint> last_exported_reconstructed_pts_;
  std::vector<LocalPathPoint> last_good_reference_pts_;
  bool has_last_exported_reconstructed_ = false;
  bool has_last_good_reference_ = false;
  ros::Time last_history_logged_stamp_;
  ros::Time last_good_reference_stamp_;
  EgoPose2D last_good_reference_ego_pose_;
  bool last_good_reference_ego_valid_ = false;

  EgoMotionEstimator ego_motion_;
  LeadObservationProjector projector_;
  LeadObservationFilter lead_filter_;
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
