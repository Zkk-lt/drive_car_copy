// #include <ros/ros.h>

// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/Twist.h>
// #include <move_car/car_parameter.h>
// #include <nav_msgs/Path.h>

// #include <algorithm>
// #include <cmath>
// #include <cstdint>
// #include <deque>
// #include <limits>
// #include <mutex>
// #include <string>
// #include <vector>

// namespace {

// double clampValue(double value, double lower, double upper) {
//   return std::max(lower, std::min(value, upper));
// }

// double wrapAngle(double angle) {
//   while (angle > M_PI) angle -= 2.0 * M_PI;
//   while (angle < -M_PI) angle += 2.0 * M_PI;
//   return angle;
// }

// double hypot2(double x, double y) {
//   return std::sqrt(x * x + y * y);
// }

// double pointDistance(double x0, double y0, double x1, double y1) {
//   return hypot2(x1 - x0, y1 - y0);
// }

// double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
//   const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
//   const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
//   return std::atan2(siny_cosp, cosy_cosp);
// }

// geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
//   geometry_msgs::Quaternion q;
//   q.x = 0.0;
//   q.y = 0.0;
//   q.z = std::sin(0.5 * yaw);
//   q.w = std::cos(0.5 * yaw);
//   return q;
// }

// struct EgoState {
//   ros::Time stamp;
//   double x = 0.0;
//   double y = 0.0;
//   double yaw = 0.0;
//   double v = 0.0;
//   double delta = 0.0;
//   double yaw_rate = 0.0;
//   double sigma_yaw = 0.0;
// };

// using EgoPose2D = EgoState;

// struct HistoryPoint2D {
//   ros::Time stamp;
//   double x = 0.0;
//   double y = 0.0;
//   double heading = 0.0;
//   double speed = 0.0;
//   double curvature = 0.0;
//   bool predicted = false;
// };

// struct LocalPathPoint {
//   double x = 0.0;
//   double y = 0.0;
//   double yaw = 0.0;
//   double curvature = 0.0;
// };

// struct LeadNode {
//   ros::Time stamp;
//   double x = 0.0;
//   double y = 0.0;
//   double yaw = 0.0;
//   double v = 0.0;
//   double a = 0.0;
//   double kappa = 0.0;
//   double dkappa = 0.0;
//   bool has_measurement = false;
//   double z_rel_x = 0.0;
//   double z_rel_y = 0.0;
//   double w_along = 0.0;
//   double w_cross = 0.0;
//   EgoState ego;
//   double heading_hint = 0.0;
//   double range = 0.0;
//   double residual_along = 0.0;
//   double residual_cross = 0.0;
//   double robust_weight = 1.0;
//   double fixed_frame_speed = 0.0;
//   double static_hypothesis_residual = 0.0;
//   double short_window_prediction_residual = 0.0;
// };

// struct RawTrailPoint {
//   double s = 0.0;
//   double t = 0.0;
//   double x = 0.0;
//   double y = 0.0;
//   double yaw = 0.0;
//   double kappa = 0.0;
//   ros::Time stamp;
// };

// struct RefPoint {
//   double s = 0.0;
//   double t = 0.0;
//   double x = 0.0;
//   double y = 0.0;
//   double yaw = 0.0;
//   double kappa = 0.0;
//   double v_ref = 0.0;
//   double a_ref = 0.0;
// };

// LocalPathPoint localPathPointFromRef(const RefPoint& ref) {
//   LocalPathPoint pt;
//   pt.x = ref.x;
//   pt.y = ref.y;
//   pt.yaw = ref.yaw;
//   pt.curvature = ref.kappa;
//   return pt;
// }

// struct LeadObservation {
//   bool valid = false;
//   ros::Time stamp;
//   double range = 0.0;
//   double local_x = 0.0;
//   double local_y = 0.0;
//   double local_heading = 0.0;
//   double world_x = 0.0;
//   double world_y = 0.0;
//   double world_heading = 0.0;
// };

// struct LeadState2D {
//   ros::Time stamp;
//   double x = 0.0;
//   double y = 0.0;
//   double vx = 0.0;
//   double vy = 0.0;
//   double heading = 0.0;
//   bool active = false;
//   bool static_mode = false;
// };

// double polylineLengthLocal(const std::vector<LocalPathPoint>& pts) {
//   double length = 0.0;
//   for (size_t i = 1; i < pts.size(); ++i) {
//     length += pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
//   }
//   return length;
// }

// double polylineLengthWorld(const std::deque<HistoryPoint2D>& pts) {
//   double length = 0.0;
//   for (size_t i = 1; i < pts.size(); ++i) {
//     length += pointDistance(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y);
//   }
//   return length;
// }

// struct TrackerDebugState {
//   std::string projector_reason = "init";
//   std::string history_reason = "init";
//   std::string raw_reason = "init";
//   std::string reference_reason = "init";
//   int input_pose_count = 0;
//   int valid_candidate_count = 0;
//   int lock_reject_count = 0;
//   size_t history_size = 0;
//   double raw_span = 0.0;
//   double reference_span = 0.0;
//   double ff_speed = 0.0;
//   double mean_ego_speed = 0.0;
//   double static_hyp_res = 0.0;
//   double hs_score = 0.0;
//   double hd_score = 0.0;
//   double accum_s = 0.0;
//   double accum_t = 0.0;
//   size_t confirmed_count = 0;
//   size_t tail_count = 0;
// };

// EgoPose2D interpolatePose(const EgoPose2D& a, const EgoPose2D& b, double t) {
//   EgoPose2D out;
//   out.stamp = (t < 0.5) ? a.stamp : b.stamp;
//   out.x = a.x + t * (b.x - a.x);
//   out.y = a.y + t * (b.y - a.y);
//   out.yaw = wrapAngle(a.yaw + t * wrapAngle(b.yaw - a.yaw));
//   out.v = a.v + t * (b.v - a.v);
//   out.delta = a.delta + t * (b.delta - a.delta);
//   out.yaw_rate = a.yaw_rate + t * (b.yaw_rate - a.yaw_rate);
//   out.sigma_yaw = a.sigma_yaw + t * (b.sigma_yaw - a.sigma_yaw);
//   return out;
// }

// void transformLocalToWorld(const EgoPose2D& ego,
//                            double local_x,
//                            double local_y,
//                            double* world_x,
//                            double* world_y) {
//   const double c = std::cos(ego.yaw);
//   const double s = std::sin(ego.yaw);
//   *world_x = ego.x + c * local_x - s * local_y;
//   *world_y = ego.y + s * local_x + c * local_y;
// }

// void transformWorldToLocal(const EgoPose2D& ego,
//                            double world_x,
//                            double world_y,
//                            double* local_x,
//                            double* local_y) {
//   const double dx = world_x - ego.x;
//   const double dy = world_y - ego.y;
//   const double c = std::cos(ego.yaw);
//   const double s = std::sin(ego.yaw);
//   *local_x = c * dx + s * dy;
//   *local_y = -s * dx + c * dy;
// }

// void recomputeHeading(std::vector<HistoryPoint2D>& pts) {
//   if (pts.size() < 2) {
//     return;
//   }
//   for (size_t i = 0; i < pts.size(); ++i) {
//     const size_t i0 = (i == 0) ? i : i - 1;
//     const size_t i1 = (i + 1 >= pts.size()) ? i : i + 1;
//     if (i0 == i1) {
//       continue;
//     }
//     const double dx = pts[i1].x - pts[i0].x;
//     const double dy = pts[i1].y - pts[i0].y;
//     if (hypot2(dx, dy) > 1e-4) {
//       pts[i].heading = std::atan2(dy, dx);
//     }
//   }
//   pts.front().heading = pts[std::min<size_t>(1, pts.size() - 1)].heading;
//   pts.back().heading = pts[pts.size() - 2].heading;
// }

// void recomputeLocalHeading(std::vector<LocalPathPoint>& pts) {
//   if (pts.size() < 2) {
//     return;
//   }
//   for (size_t i = 0; i < pts.size(); ++i) {
//     const size_t i0 = (i == 0) ? i : i - 1;
//     const size_t i1 = (i + 1 >= pts.size()) ? i : i + 1;
//     if (i0 == i1) {
//       continue;
//     }
//     const double dx = pts[i1].x - pts[i0].x;
//     const double dy = pts[i1].y - pts[i0].y;
//     if (hypot2(dx, dy) > 1e-4) {
//       pts[i].yaw = std::atan2(dy, dx);
//     }
//   }
//   pts.front().yaw = pts[std::min<size_t>(1, pts.size() - 1)].yaw;
//   pts.back().yaw = pts[pts.size() - 2].yaw;
// }

// std::vector<HistoryPoint2D> removeNearDuplicatePoints(const std::vector<HistoryPoint2D>& in,
//                                                       double dist_eps) {
//   if (in.empty()) {
//     return in;
//   }

//   std::vector<HistoryPoint2D> out;
//   out.reserve(in.size());
//   out.push_back(in.front());
//   for (size_t i = 1; i < in.size(); ++i) {
//     if (pointDistance(out.back().x, out.back().y, in[i].x, in[i].y) >= dist_eps) {
//       out.push_back(in[i]);
//     } else {
//       out.back() = in[i];
//     }
//   }
//   return out;
// }

// std::vector<HistoryPoint2D> resampleWorldByArcLength(const std::vector<HistoryPoint2D>& in,
//                                                      double ds) {
//   if (in.size() < 2 || ds <= 1e-4) {
//     return in;
//   }

//   std::vector<double> arc(in.size(), 0.0);
//   for (size_t i = 1; i < in.size(); ++i) {
//     arc[i] = arc[i - 1] + pointDistance(in[i - 1].x, in[i - 1].y, in[i].x, in[i].y);
//   }

//   const double total = arc.back();
//   if (total < 1e-4) {
//     return {in.back()};
//   }

//   std::vector<HistoryPoint2D> out;
//   out.reserve(static_cast<size_t>(std::ceil(total / ds)) + 2);
//   size_t seg = 0;
//   const size_t samples = std::max<size_t>(2, static_cast<size_t>(std::floor(total / ds)) + 1);

//   for (size_t k = 0; k < samples; ++k) {
//     const double sk = std::min(total, static_cast<double>(k) * ds);
//     while (seg + 1 < arc.size() && arc[seg + 1] < sk) {
//       ++seg;
//     }
//     if (seg + 1 >= in.size()) {
//       out.push_back(in.back());
//       break;
//     }

//     const double denom = std::max(1e-6, arc[seg + 1] - arc[seg]);
//     const double t = clampValue((sk - arc[seg]) / denom, 0.0, 1.0);
//     HistoryPoint2D pt;
//     pt.stamp = (t < 0.5) ? in[seg].stamp : in[seg + 1].stamp;
//     pt.x = in[seg].x + t * (in[seg + 1].x - in[seg].x);
//     pt.y = in[seg].y + t * (in[seg + 1].y - in[seg].y);
//     pt.heading = wrapAngle(in[seg].heading + t * wrapAngle(in[seg + 1].heading - in[seg].heading));
//     pt.predicted = in[seg].predicted || in[seg + 1].predicted;
//     out.push_back(pt);
//   }

//   if (pointDistance(out.back().x, out.back().y, in.back().x, in.back().y) > 1e-3) {
//     out.push_back(in.back());
//   } else {
//     out.back() = in.back();
//   }

//   recomputeHeading(out);
//   return out;
// }

// std::vector<HistoryPoint2D> smoothWorldPath5(const std::vector<HistoryPoint2D>& in,
//                                              int passes) {
//   if (in.size() < 5 || passes <= 0) {
//     return in;
//   }

//   std::vector<HistoryPoint2D> cur = in;
//   std::vector<HistoryPoint2D> nxt = in;
//   for (int pass = 0; pass < passes; ++pass) {
//     nxt = cur;
//     for (size_t i = 2; i + 2 < cur.size(); ++i) {
//       nxt[i].x = (-3.0 * cur[i - 2].x + 12.0 * cur[i - 1].x + 17.0 * cur[i].x +
//                   12.0 * cur[i + 1].x - 3.0 * cur[i + 2].x) / 35.0;
//       nxt[i].y = (-3.0 * cur[i - 2].y + 12.0 * cur[i - 1].y + 17.0 * cur[i].y +
//                   12.0 * cur[i + 1].y - 3.0 * cur[i + 2].y) / 35.0;
//     }
//     cur.swap(nxt);
//   }
//   recomputeHeading(cur);
//   return cur;
// }

// std::vector<HistoryPoint2D> smoothWorldSplineLike(const std::vector<HistoryPoint2D>& in,
//                                                   int passes,
//                                                   double alpha,
//                                                   double tail_weight_scale) {
//   if (in.size() < 3 || passes <= 0 || alpha <= 1e-4) {
//     return in;
//   }

//   std::vector<HistoryPoint2D> cur = in;
//   std::vector<HistoryPoint2D> nxt = in;
//   const double tail_scale = clampValue(tail_weight_scale, 0.05, 1.0);

//   for (int pass = 0; pass < passes; ++pass) {
//     nxt = cur;
//     for (size_t i = 1; i + 1 < cur.size(); ++i) {
//       const double neighbor_x = 0.5 * (cur[i - 1].x + cur[i + 1].x);
//       const double neighbor_y = 0.5 * (cur[i - 1].y + cur[i + 1].y);
//       const double keep_weight = cur[i].predicted ? tail_scale : 1.0;
//       const double smooth_gain = alpha * (1.0 - keep_weight);
//       nxt[i].x = (1.0 - smooth_gain) * cur[i].x + smooth_gain * neighbor_x;
//       nxt[i].y = (1.0 - smooth_gain) * cur[i].y + smooth_gain * neighbor_y;
//     }
//     cur.swap(nxt);
//   }

//   recomputeHeading(cur);
//   return cur;
// }

// HistoryPoint2D interpolateWorldHermite(const HistoryPoint2D& a,
//                                        const HistoryPoint2D& b,
//                                        double tangent_scale_a,
//                                        double tangent_scale_b,
//                                        double t) {
//   const double tt = clampValue(t, 0.0, 1.0);
//   const double h00 = 2.0 * tt * tt * tt - 3.0 * tt * tt + 1.0;
//   const double h10 = tt * tt * tt - 2.0 * tt * tt + tt;
//   const double h01 = -2.0 * tt * tt * tt + 3.0 * tt * tt;
//   const double h11 = tt * tt * tt - tt * tt;

//   const double m0x = tangent_scale_a * std::cos(a.heading);
//   const double m0y = tangent_scale_a * std::sin(a.heading);
//   const double m1x = tangent_scale_b * std::cos(b.heading);
//   const double m1y = tangent_scale_b * std::sin(b.heading);

//   HistoryPoint2D out;
//   out.stamp = (tt < 0.5) ? a.stamp : b.stamp;
//   out.x = h00 * a.x + h10 * m0x + h01 * b.x + h11 * m1x;
//   out.y = h00 * a.y + h10 * m0y + h01 * b.y + h11 * m1y;

//   const double dh00 = 6.0 * tt * tt - 6.0 * tt;
//   const double dh10 = 3.0 * tt * tt - 4.0 * tt + 1.0;
//   const double dh01 = -6.0 * tt * tt + 6.0 * tt;
//   const double dh11 = 3.0 * tt * tt - 2.0 * tt;
//   const double dx = dh00 * a.x + dh10 * m0x + dh01 * b.x + dh11 * m1x;
//   const double dy = dh00 * a.y + dh10 * m0y + dh01 * b.y + dh11 * m1y;
//   out.heading = std::atan2(dy, dx);
//   out.speed = (1.0 - tt) * a.speed + tt * b.speed;

//   const double ddh00 = 12.0 * tt - 6.0;
//   const double ddh10 = 6.0 * tt - 4.0;
//   const double ddh01 = -12.0 * tt + 6.0;
//   const double ddh11 = 6.0 * tt - 2.0;
//   const double ddx = ddh00 * a.x + ddh10 * m0x + ddh01 * b.x + ddh11 * m1x;
//   const double ddy = ddh00 * a.y + ddh10 * m0y + ddh01 * b.y + ddh11 * m1y;
//   const double denom = std::pow(std::max(1e-6, dx * dx + dy * dy), 1.5);
//   out.curvature = clampValue((dx * ddy - dy * ddx) / denom, -0.8, 0.8);
//   return out;
// }

// std::vector<HistoryPoint2D> fitWorldTrajectoryHermite(const std::vector<HistoryPoint2D>& knots,
//                                                       double ds) {
//   if (knots.size() < 2 || ds <= 1e-4) {
//     return knots;
//   }

//   std::vector<HistoryPoint2D> fitted;
//   fitted.reserve(knots.size() * 4);
//   fitted.push_back(knots.front());
//   for (size_t i = 0; i + 1 < knots.size(); ++i) {
//     const HistoryPoint2D& a = knots[i];
//     const HistoryPoint2D& b = knots[i + 1];
//     const double seg_len = pointDistance(a.x, a.y, b.x, b.y);
//     if (seg_len < 1e-4) {
//       continue;
//     }
//     const double tangent_scale_a =
//         (i == 0) ? seg_len : 0.5 * (seg_len + pointDistance(knots[i - 1].x, knots[i - 1].y, a.x, a.y));
//     const double tangent_scale_b =
//         (i + 2 >= knots.size()) ? seg_len
//                                 : 0.5 * (seg_len + pointDistance(b.x, b.y, knots[i + 2].x, knots[i + 2].y));
//     const int steps = std::max(2, static_cast<int>(std::ceil(seg_len / ds)));
//     for (int k = 1; k <= steps; ++k) {
//       const double t = static_cast<double>(k) / static_cast<double>(steps);
//       HistoryPoint2D sample = interpolateWorldHermite(a, b, tangent_scale_a, tangent_scale_b, t);
//       if (fitted.empty() ||
//           pointDistance(fitted.back().x, fitted.back().y, sample.x, sample.y) > 1e-3) {
//         fitted.push_back(sample);
//       } else {
//         fitted.back() = sample;
//       }
//     }
//   }
//   recomputeHeading(fitted);
//   return fitted;
// }

// void offsetWorldPathAlongHeading(std::vector<HistoryPoint2D>* pts, double offset) {
//   if (pts == NULL || pts->empty() || std::fabs(offset) <= 1e-6) {
//     return;
//   }

//   recomputeHeading(*pts);
//   for (size_t i = 0; i < pts->size(); ++i) {
//     (*pts)[i].x -= offset * std::cos((*pts)[i].heading);
//     (*pts)[i].y -= offset * std::sin((*pts)[i].heading);
//   }
// }

// std::vector<LocalPathPoint> transformWorldPathToLocal(const std::vector<HistoryPoint2D>& world_pts,
//                                                       const EgoPose2D& ego) {
//   std::vector<LocalPathPoint> out;
//   out.reserve(world_pts.size());
//   for (size_t i = 0; i < world_pts.size(); ++i) {
//     LocalPathPoint pt;
//     transformWorldToLocal(ego, world_pts[i].x, world_pts[i].y, &pt.x, &pt.y);
//     pt.yaw = wrapAngle(world_pts[i].heading - ego.yaw);
//     out.push_back(pt);
//   }
//   return out;
// }

// std::vector<RawTrailPoint> makeRawTrailPoints(const std::vector<HistoryPoint2D>& world_pts) {
//   std::vector<RawTrailPoint> out;
//   out.reserve(world_pts.size());
//   double s = 0.0;
//   const ros::Time base_stamp = world_pts.empty() ? ros::Time() : world_pts.front().stamp;
//   for (size_t i = 0; i < world_pts.size(); ++i) {
//     if (i > 0) {
//       s += pointDistance(world_pts[i - 1].x, world_pts[i - 1].y, world_pts[i].x, world_pts[i].y);
//     }
//     RawTrailPoint pt;
//     pt.s = s;
//     pt.t = (base_stamp.isValid() && world_pts[i].stamp.isValid()) ? (world_pts[i].stamp - base_stamp).toSec() : 0.0;
//     pt.x = world_pts[i].x;
//     pt.y = world_pts[i].y;
//     pt.yaw = world_pts[i].heading;
//     pt.kappa = world_pts[i].curvature;
//     pt.stamp = world_pts[i].stamp;
//     out.push_back(pt);
//   }
//   return out;
// }

// std::vector<RefPoint> makeRefPoints(const std::vector<LocalPathPoint>& local_pts,
//                                     double nominal_speed) {
//   std::vector<RefPoint> out;
//   out.reserve(local_pts.size());
//   double s = 0.0;
//   double prev_v = nominal_speed;
//   for (size_t i = 0; i < local_pts.size(); ++i) {
//     if (i > 0) {
//       s += pointDistance(local_pts[i - 1].x, local_pts[i - 1].y, local_pts[i].x, local_pts[i].y);
//     }
//     RefPoint pt;
//     pt.s = s;
//     pt.x = local_pts[i].x;
//     pt.y = local_pts[i].y;
//     pt.yaw = local_pts[i].yaw;
//     pt.kappa = local_pts[i].curvature;
//     pt.v_ref = clampValue(nominal_speed * std::max(0.4, 1.0 - 1.5 * std::fabs(pt.kappa)), 0.0, nominal_speed);
//     pt.a_ref = (i == 0 || s < 1e-3) ? 0.0 : (pt.v_ref - prev_v) / std::max(1e-3, s - out.back().s);
//     prev_v = pt.v_ref;
//     out.push_back(pt);
//   }
//   return out;
// }

// std::vector<RefPoint> makeRefPointsFromRawTrail(const std::vector<RawTrailPoint>& raw_trail,
//                                                 const EgoPose2D& ego,
//                                                 double nominal_speed) {
//   std::vector<RefPoint> out;
//   out.reserve(raw_trail.size());
//   double prev_v = nominal_speed;
//   for (size_t i = 0; i < raw_trail.size(); ++i) {
//     RefPoint pt;
//     pt.s = raw_trail[i].s;
//     pt.t = raw_trail[i].t;
//     transformWorldToLocal(ego, raw_trail[i].x, raw_trail[i].y, &pt.x, &pt.y);
//     pt.yaw = wrapAngle(raw_trail[i].yaw - ego.yaw);
//     pt.kappa = raw_trail[i].kappa;
//     pt.v_ref = clampValue(nominal_speed * std::max(0.4, 1.0 - 1.5 * std::fabs(pt.kappa)), 0.0, nominal_speed);
//     if (i == 0) {
//       pt.a_ref = 0.0;
//     } else {
//       const double dt = std::max(1e-3, pt.t - out.back().t);
//       pt.a_ref = (pt.v_ref - prev_v) / dt;
//     }
//     prev_v = pt.v_ref;
//     out.push_back(pt);
//   }
//   return out;
// }

// void recomputeRefGeometry(std::vector<RefPoint>* pts) {
//   if (pts == NULL || pts->empty()) {
//     return;
//   }

//   for (size_t i = 0; i < pts->size(); ++i) {
//     if (i == 0) {
//       (*pts)[i].s = 0.0;
//     } else {
//       (*pts)[i].s = (*pts)[i - 1].s +
//                     pointDistance((*pts)[i - 1].x, (*pts)[i - 1].y, (*pts)[i].x, (*pts)[i].y);
//     }
//   }

//   if (pts->size() < 2) {
//     (*pts)[0].yaw = 0.0;
//     (*pts)[0].kappa = 0.0;
//     return;
//   }

//   for (size_t i = 0; i < pts->size(); ++i) {
//     const size_t i0 = (i == 0) ? i : i - 1;
//     const size_t i1 = (i + 1 >= pts->size()) ? i : i + 1;
//     if (i0 == i1) {
//       continue;
//     }
//     const double dx = (*pts)[i1].x - (*pts)[i0].x;
//     const double dy = (*pts)[i1].y - (*pts)[i0].y;
//     if (hypot2(dx, dy) > 1e-4) {
//       (*pts)[i].yaw = std::atan2(dy, dx);
//     }
//   }
//   (*pts).front().yaw = (*pts)[std::min<size_t>(1, pts->size() - 1)].yaw;
//   (*pts).back().yaw = (*pts)[pts->size() - 2].yaw;

//   for (size_t i = 1; i + 1 < pts->size(); ++i) {
//     const double ds = std::max(1e-3, (*pts)[i + 1].s - (*pts)[i - 1].s);
//     const double dpsi = wrapAngle((*pts)[i + 1].yaw - (*pts)[i - 1].yaw);
//     (*pts)[i].kappa = clampValue(dpsi / ds, -0.8, 0.8);
//   }
//   if (pts->size() >= 2) {
//     (*pts).front().kappa = (*pts)[std::min<size_t>(1, pts->size() - 1)].kappa;
//     (*pts).back().kappa = (*pts)[pts->size() - 2].kappa;
//   }
// }

// std::vector<RefPoint> resampleRefByArcLength(const std::vector<RefPoint>& in, double ds) {
//   if (in.size() < 2 || ds <= 1e-4) {
//     return in;
//   }

//   std::vector<double> arc(in.size(), 0.0);
//   for (size_t i = 1; i < in.size(); ++i) {
//     arc[i] = arc[i - 1] + pointDistance(in[i - 1].x, in[i - 1].y, in[i].x, in[i].y);
//   }
//   const double total = arc.back();
//   if (total < 1e-4) {
//     return {in.back()};
//   }

//   std::vector<RefPoint> out;
//   out.reserve(static_cast<size_t>(std::ceil(total / ds)) + 2);
//   size_t seg = 0;
//   const size_t samples = std::max<size_t>(2, static_cast<size_t>(std::floor(total / ds)) + 1);
//   for (size_t k = 0; k < samples; ++k) {
//     const double sk = std::min(total, static_cast<double>(k) * ds);
//     while (seg + 1 < arc.size() && arc[seg + 1] < sk) {
//       ++seg;
//     }
//     if (seg + 1 >= in.size()) {
//       out.push_back(in.back());
//       break;
//     }
//     const double denom = std::max(1e-6, arc[seg + 1] - arc[seg]);
//     const double t = clampValue((sk - arc[seg]) / denom, 0.0, 1.0);
//     RefPoint pt;
//     pt.s = sk;
//     pt.x = in[seg].x + t * (in[seg + 1].x - in[seg].x);
//     pt.y = in[seg].y + t * (in[seg + 1].y - in[seg].y);
//     pt.yaw = wrapAngle(in[seg].yaw + t * wrapAngle(in[seg + 1].yaw - in[seg].yaw));
//     pt.kappa = (1.0 - t) * in[seg].kappa + t * in[seg + 1].kappa;
//     pt.v_ref = (1.0 - t) * in[seg].v_ref + t * in[seg + 1].v_ref;
//     pt.a_ref = (1.0 - t) * in[seg].a_ref + t * in[seg + 1].a_ref;
//     out.push_back(pt);
//   }

//   if (pointDistance(out.back().x, out.back().y, in.back().x, in.back().y) > 1e-3) {
//     out.push_back(in.back());
//   } else {
//     out.back() = in.back();
//   }
//   recomputeRefGeometry(&out);
//   return out;
// }

// std::vector<LocalPathPoint> makeLocalPathFromRefPoints(const std::vector<RefPoint>& ref_pts) {
//   std::vector<LocalPathPoint> out;
//   out.reserve(ref_pts.size());
//   for (size_t i = 0; i < ref_pts.size(); ++i) {
//     out.push_back(localPathPointFromRef(ref_pts[i]));
//   }
//   return out;
// }

// std::vector<LocalPathPoint> resampleLocalByArcLength(const std::vector<LocalPathPoint>& in,
//                                                      double ds) {
//   if (in.size() < 2 || ds <= 1e-4) {
//     return in;
//   }

//   std::vector<double> arc(in.size(), 0.0);
//   for (size_t i = 1; i < in.size(); ++i) {
//     arc[i] = arc[i - 1] + pointDistance(in[i - 1].x, in[i - 1].y, in[i].x, in[i].y);
//   }
//   const double total = arc.back();
//   if (total < 1e-4) {
//     return {in.back()};
//   }

//   std::vector<LocalPathPoint> out;
//   out.reserve(static_cast<size_t>(std::ceil(total / ds)) + 2);
//   size_t seg = 0;
//   const size_t samples = std::max<size_t>(2, static_cast<size_t>(std::floor(total / ds)) + 1);

//   for (size_t k = 0; k < samples; ++k) {
//     const double sk = std::min(total, static_cast<double>(k) * ds);
//     while (seg + 1 < arc.size() && arc[seg + 1] < sk) {
//       ++seg;
//     }
//     if (seg + 1 >= in.size()) {
//       out.push_back(in.back());
//       break;
//     }

//     const double denom = std::max(1e-6, arc[seg + 1] - arc[seg]);
//     const double t = clampValue((sk - arc[seg]) / denom, 0.0, 1.0);
//     LocalPathPoint pt;
//     pt.x = in[seg].x + t * (in[seg + 1].x - in[seg].x);
//     pt.y = in[seg].y + t * (in[seg + 1].y - in[seg].y);
//     pt.yaw = wrapAngle(in[seg].yaw + t * wrapAngle(in[seg + 1].yaw - in[seg].yaw));
//     out.push_back(pt);
//   }

//   if (pointDistance(out.back().x, out.back().y, in.back().x, in.back().y) > 1e-3) {
//     out.push_back(in.back());
//   } else {
//     out.back() = in.back();
//   }

//   recomputeLocalHeading(out);
//   return out;
// }

// std::vector<LocalPathPoint> smoothLocalPath5(const std::vector<LocalPathPoint>& in,
//                                              int passes) {
//   if (in.size() < 5 || passes <= 0) {
//     return in;
//   }

//   std::vector<LocalPathPoint> cur = in;
//   std::vector<LocalPathPoint> nxt = in;
//   for (int pass = 0; pass < passes; ++pass) {
//     nxt = cur;
//     for (size_t i = 2; i + 2 < cur.size(); ++i) {
//       nxt[i].x = (-3.0 * cur[i - 2].x + 12.0 * cur[i - 1].x + 17.0 * cur[i].x +
//                   12.0 * cur[i + 1].x - 3.0 * cur[i + 2].x) / 35.0;
//       nxt[i].y = (-3.0 * cur[i - 2].y + 12.0 * cur[i - 1].y + 17.0 * cur[i].y +
//                   12.0 * cur[i + 1].y - 3.0 * cur[i + 2].y) / 35.0;
//     }
//     cur.swap(nxt);
//   }

//   recomputeLocalHeading(cur);
//   return cur;
// }

// nav_msgs::Path makePathMsg(const std::vector<LocalPathPoint>& pts,
//                            const std::string& frame_id,
//                            const ros::Time& stamp) {
//   nav_msgs::Path path;
//   path.header.stamp = stamp;
//   path.header.frame_id = frame_id;
//   path.poses.reserve(pts.size());
//   for (size_t i = 0; i < pts.size(); ++i) {
//     geometry_msgs::PoseStamped pose;
//     pose.header = path.header;
//     pose.pose.position.x = pts[i].x;
//     pose.pose.position.y = pts[i].y;
//     pose.pose.position.z = 0.0;
//     pose.pose.orientation = quaternionFromYaw(pts[i].yaw);
//     path.poses.push_back(pose);
//   }
//   return path;
// }

// LeadState2D predictLeadState(const LeadState2D& state, const ros::Time& stamp) {
//   LeadState2D out = state;
//   if (!state.active) {
//     out.stamp = stamp;
//     return out;
//   }

//   const double dt = std::max(0.0, (stamp - state.stamp).toSec());
//   out.x += state.vx * dt;
//   out.y += state.vy * dt;
//   out.stamp = stamp;
//   if (hypot2(out.vx, out.vy) > 1e-4) {
//     out.heading = std::atan2(out.vy, out.vx);
//   }
//   return out;
// }

// class EgoMotionEstimator {
//  public:
//   struct Config {
//     double wheelbase = 2.60;
//     double min_motion_speed_for_yaw = 0.15;
//     double max_abs_yaw_rate = 0.8;
//     double max_integration_dt = 0.02;
//     double pose_buffer_duration = 5.0;
//     double steer_lpf_alpha = 0.25;
//     double yaw_sigma_base = 0.01;
//     double yaw_sigma_speed_gain = 0.02;
//     double yaw_sigma_steer_gain = 0.08;
//     bool use_speed_hint = false;
//     double speed_hint_weight = 0.20;
//     double speed_hint_timeout = 0.30;
//     double speed_hint_agreement_gate = 0.40;
//   };

//   explicit EgoMotionEstimator(const Config& cfg) : cfg_(cfg) {}

//   void updateWheelState(const ros::Time& stamp, double wheel_speed_mps, double steer_angle_rad) {
//     integrateTo(stamp);
//     wheel_speed_mps_ = wheel_speed_mps;
//     if (!steer_initialized_) {
//       filtered_steer_angle_rad_ = steer_angle_rad;
//       steer_initialized_ = true;
//     } else {
//       filtered_steer_angle_rad_ =
//           (1.0 - cfg_.steer_lpf_alpha) * filtered_steer_angle_rad_ + cfg_.steer_lpf_alpha * steer_angle_rad;
//     }
//     last_motion_stamp_ = stamp;
//     updateFusedSpeed(stamp);
//   }

//   void updateSpeedHint(const ros::Time& stamp, double speed_hint_mps) {
//     integrateTo(stamp);
//     speed_hint_mps_ = speed_hint_mps;
//     speed_hint_stamp_ = stamp;
//     updateFusedSpeed(stamp);
//   }

//   void integrateTo(const ros::Time& target_stamp) {
//     if (!target_stamp.isValid()) {
//       return;
//     }

//     if (!initialized_) {
//       pose_.stamp = target_stamp;
//       pose_.v = fused_speed_mps_;
//       pose_.delta = filtered_steer_angle_rad_;
//       pose_.yaw_rate = 0.0;
//       pose_.sigma_yaw = cfg_.yaw_sigma_base;
//       buffer_.push_back(pose_);
//       initialized_ = true;
//       return;
//     }

//     if (target_stamp <= pose_.stamp) {
//       return;
//     }

//     double remaining = (target_stamp - pose_.stamp).toSec();
//     while (remaining > 1e-6) {
//       const double dt = std::min(cfg_.max_integration_dt, remaining);
//       const double yaw_rate = currentYawRate();
//       const double v = fused_speed_mps_;
//       const double dtheta = yaw_rate * dt;
//       const double mid_yaw = pose_.yaw + 0.5 * dtheta;
//       if (std::fabs(v) >= 1e-4) {
//         pose_.x += v * dt * std::cos(mid_yaw);
//         pose_.y += v * dt * std::sin(mid_yaw);
//       }
//       pose_.yaw = wrapAngle(pose_.yaw + dtheta);
//       pose_.stamp = pose_.stamp + ros::Duration(dt);
//       pose_.v = v;
//       pose_.delta = filtered_steer_angle_rad_;
//       pose_.yaw_rate = yaw_rate;
//       pose_.sigma_yaw = yawSigmaEstimate();
//       buffer_.push_back(pose_);
//       remaining -= dt;
//     }

//     pruneBuffer(target_stamp);
//   }

//   bool queryPoseAt(const ros::Time& stamp, EgoPose2D* pose) const {
//     if (buffer_.empty() || pose == NULL) {
//       return false;
//     }
//     if (stamp <= buffer_.front().stamp) {
//       *pose = buffer_.front();
//       return true;
//     }
//     if (stamp >= buffer_.back().stamp) {
//       *pose = buffer_.back();
//       return true;
//     }

//     for (size_t i = 1; i < buffer_.size(); ++i) {
//       if (buffer_[i].stamp < stamp) {
//         continue;
//       }
//       const EgoPose2D& a = buffer_[i - 1];
//       const EgoPose2D& b = buffer_[i];
//       const double denom = std::max(1e-6, (b.stamp - a.stamp).toSec());
//       const double t = clampValue((stamp - a.stamp).toSec() / denom, 0.0, 1.0);
//       *pose = interpolatePose(a, b, t);
//       return true;
//     }

//     *pose = buffer_.back();
//     return true;
//   }

//   double currentYawRate() const {
//     if (std::fabs(fused_speed_mps_) < cfg_.min_motion_speed_for_yaw) {
//       return 0.0;
//     }
//     const double yaw_rate =
//         fused_speed_mps_ * std::tan(filtered_steer_angle_rad_) / std::max(0.5, cfg_.wheelbase);
//     return clampValue(yaw_rate, -cfg_.max_abs_yaw_rate, cfg_.max_abs_yaw_rate);
//   }

//   double yawSigmaEstimate() const {
//     return cfg_.yaw_sigma_base + cfg_.yaw_sigma_speed_gain * std::fabs(currentYawRate()) +
//            cfg_.yaw_sigma_steer_gain * std::fabs(filtered_steer_angle_rad_);
//   }

//   double fusedSpeed() const { return fused_speed_mps_; }
//   double wheelSpeed() const { return wheel_speed_mps_; }
//   double speedHint() const { return speed_hint_mps_; }
//   double steerAngle() const { return filtered_steer_angle_rad_; }
//   bool initialized() const { return initialized_; }

//  private:
//   void updateFusedSpeed(const ros::Time& now) {
//     if (!cfg_.use_speed_hint) {
//       fused_speed_mps_ = wheel_speed_mps_;
//       return;
//     }

//     const bool hint_fresh = speed_hint_stamp_.isValid() &&
//                             (now - speed_hint_stamp_).toSec() <= cfg_.speed_hint_timeout;
//     if (!hint_fresh) {
//       fused_speed_mps_ = wheel_speed_mps_;
//       return;
//     }

//     const double diff = std::fabs(wheel_speed_mps_ - speed_hint_mps_);
//     const bool agreement = diff <= cfg_.speed_hint_agreement_gate ||
//                            (wheel_speed_mps_ * speed_hint_mps_ >= 0.0 &&
//                             std::min(std::fabs(wheel_speed_mps_), std::fabs(speed_hint_mps_)) < 0.15);
//     if (!agreement) {
//       fused_speed_mps_ = wheel_speed_mps_;
//       return;
//     }

//     fused_speed_mps_ =
//         (1.0 - cfg_.speed_hint_weight) * wheel_speed_mps_ + cfg_.speed_hint_weight * speed_hint_mps_;
//   }

//   void pruneBuffer(const ros::Time& now) {
//     while (buffer_.size() > 2 && (now - buffer_.front().stamp).toSec() > cfg_.pose_buffer_duration) {
//       buffer_.pop_front();
//     }
//   }

//   Config cfg_;
//   bool initialized_ = false;
//   EgoPose2D pose_;
//   std::deque<EgoPose2D> buffer_;
//   ros::Time last_motion_stamp_;
//   ros::Time speed_hint_stamp_;
//   double wheel_speed_mps_ = 0.0;
//   double speed_hint_mps_ = 0.0;
//   double fused_speed_mps_ = 0.0;
//   bool steer_initialized_ = false;
//   double filtered_steer_angle_rad_ = 0.0;
// };

// class LeadObservationProjector {
//  public:
//   struct Config {
//     double target_min_forward_x = 0.30;
//     double target_lateral_gate = 4.0;
//     double target_lock_dist_gate = 1.20;
//     double target_lock_yaw_gate = 1.10;
//   };

//   explicit LeadObservationProjector(const Config& cfg) : cfg_(cfg) {}

//   bool project(const geometry_msgs::PoseArray& msg,
//                const EgoMotionEstimator& ego_motion,
//                const LeadState2D* track,
//                LeadObservation* selected,
//                EgoPose2D* ego_at_obs,
//                TrackerDebugState* debug) const {
//     if (selected == NULL || ego_at_obs == NULL) {
//       return false;
//     }
//     if (debug != NULL) {
//       debug->input_pose_count = static_cast<int>(msg.poses.size());
//       debug->valid_candidate_count = 0;
//       debug->lock_reject_count = 0;
//       debug->projector_reason = "init";
//     }

//     const ros::Time stamp = msg.header.stamp;
//     if (!stamp.isValid()) {
//       if (debug != NULL) {
//         debug->projector_reason = "unstamped_posearray";
//       }
//       return false;
//     }

//     if (!ego_motion.queryPoseAt(stamp, ego_at_obs)) {
//       if (debug != NULL) {
//         debug->projector_reason = "ego_pose_unavailable";
//       }
//       return false;
//     }

//     std::vector<LeadObservation> candidates;
//     candidates.reserve(msg.poses.size());
//     for (size_t i = 0; i < msg.poses.size(); ++i) {
//       LeadObservation obs = buildObservation(msg.poses[i], *ego_at_obs, track, stamp);
//       if (obs.valid) {
//         candidates.push_back(obs);
//       }
//     }
//     if (debug != NULL) {
//       debug->valid_candidate_count = static_cast<int>(candidates.size());
//     }

//     if (candidates.empty()) {
//       if (debug != NULL) {
//         debug->projector_reason = msg.poses.empty() ? "no_target" : "all_rejected_by_gate";
//       }
//       return false;
//     }

//     double best_cost = std::numeric_limits<double>::infinity();
//     int best_idx = -1;

//     if (track != NULL && track->active) {
//       const LeadState2D pred = predictLeadState(*track, stamp);
//       for (size_t i = 0; i < candidates.size(); ++i) {
//         const double pos_err = pointDistance(candidates[i].world_x, candidates[i].world_y, pred.x, pred.y);
//         const double yaw_err = std::fabs(wrapAngle(candidates[i].world_heading - pred.heading));
//         if (pos_err > cfg_.target_lock_dist_gate || yaw_err > cfg_.target_lock_yaw_gate) {
//           if (debug != NULL) {
//             ++debug->lock_reject_count;
//           }
//           continue;
//         }
//         const double cost = pos_err + 0.2 * yaw_err + 0.08 * std::fabs(candidates[i].local_y);
//         if (cost < best_cost) {
//           best_cost = cost;
//           best_idx = static_cast<int>(i);
//         }
//       }
//     }

//     if (best_idx < 0) {
//       for (size_t i = 0; i < candidates.size(); ++i) {
//         const double cost = candidates[i].local_x + 0.85 * std::fabs(candidates[i].local_y);
//         if (cost < best_cost) {
//           best_cost = cost;
//           best_idx = static_cast<int>(i);
//         }
//       }
//     }

//     if (best_idx < 0) {
//       if (debug != NULL) {
//         debug->projector_reason = "selection_failed";
//       }
//       return false;
//     }

//     *selected = candidates[static_cast<size_t>(best_idx)];
//     if (debug != NULL) {
//       debug->projector_reason = (track != NULL && track->active) ? "selected_locked_or_fallback"
//                                                                  : "selected_bootstrap";
//     }
//     return true;
//   }

//  private:
//   LeadObservation buildObservation(const geometry_msgs::Pose& pose,
//                                    const EgoPose2D& ego_at_obs,
//                                    const LeadState2D* track,
//                                    const ros::Time& stamp) const {
//     LeadObservation out;
//     const double center_x = pose.position.x;
//     const double center_y = pose.position.y;
//     if (center_x < cfg_.target_min_forward_x || std::fabs(center_y) > cfg_.target_lateral_gate) {
//       return out;
//     }

//     double local_heading = yawFromQuaternion(pose.orientation);
//     double world_heading = wrapAngle(ego_at_obs.yaw + local_heading);
//     if (track != NULL && track->active) {
//       const double err0 = std::fabs(wrapAngle(world_heading - track->heading));
//       const double err1 = std::fabs(wrapAngle(world_heading + M_PI - track->heading));
//       if (err1 < err0) {
//         local_heading = wrapAngle(local_heading + M_PI);
//         world_heading = wrapAngle(world_heading + M_PI);
//       }
//     }

//     out.valid = true;
//     out.stamp = stamp;
//     out.local_x = center_x;
//     out.local_y = center_y;
//     out.local_heading = local_heading;
//     out.range = hypot2(center_x, center_y);
//     transformLocalToWorld(ego_at_obs, center_x, center_y, &out.world_x, &out.world_y);
//     out.world_heading = world_heading;
//     return out;
//   }

//   Config cfg_;
// };

// class LeadTrailTracker {
//  public:
//   struct Config {
//     double lost_timeout = 0.60;
//     double heading_blend_alpha = 0.20;
//     double heading_from_velocity_speed_gate = 0.25;
//     double innovation_base_gate = 0.10;
//     double innovation_range_gain = 0.015;
//     double innovation_turn_gain = 0.80;
//     double outlier_reject_scale = 3.0;
//     double lead_static_speed_gate = 0.08;
//     double lead_moving_speed_gate = 0.12;
//     double static_residual_gate = 0.12;
//     double moving_residual_gate = 0.30;
//     int static_vote_on = 12;
//     int static_vote_release = 3;
//     int moving_vote_on = 2;
//     double anchor_alpha = 0.08;
//     double history_max_age = 20.0;
//     int history_max_points = 300;
//     double smoother_position_gain = 0.35;
//     double smoother_cross_gain = 0.18;
//     double smoother_temporal_alpha = 0.22;
//     double smoother_heading_alpha = 0.20;
//     double smoother_curvature_alpha = 0.18;
//     double measurement_heading_alpha = 0.12;
//     double dynamics_position_alpha = 0.12;
//     double dynamics_heading_alpha = 0.16;
//     double acceleration_regularization_alpha = 0.16;
//     double curvature_prediction_alpha = 0.18;
//     double dkappa_regularization_alpha = 0.20;
//     double ego_compensated_static_gate = 0.10;
//     double ego_compensated_moving_gate = 0.22;
//     double static_measurement_anchor_alpha = 0.18;
//     double robust_cauchy_scale = 2.5;
//     int smoother_iterations = 5;
//     double measurement_along_sigma = 0.08;
//     double measurement_cross_sigma = 0.12;
//     double measurement_cross_range_gain = 0.020;
//     double measurement_cross_yaw_uncertainty_gain = 1.80;
//     double temporal_min_progress = 0.02;
//     double static_preserve_trail_span = 0.80;
//     double trail_append_min_distance = 0.12;
//     double trail_anchor_update_alpha = 0.10;
//     int tail_confirm_min_points = 3;
//     double tail_confirm_min_distance = 0.06;
//     double tail_confirm_min_age = 0.18;
//     double confirm_dynamic_score_margin = 0.12;
//     double confirm_static_residual_gate = 0.14;
//     double confirm_min_progress = 0.08;
//     double confirm_accum_distance = 0.05;
//     double confirm_accum_timeout = 0.80;
//     double ego_static_speed_gate = 0.05;
//     double static_release_score_margin = 0.06;
//     int raw_tail_keep_points = 6;
//     double dual_hypothesis_score_margin = 0.02;
//   };

//   explicit LeadTrailTracker(const Config& cfg) : cfg_(cfg) {}

//   void markLostIfExpired(const ros::Time& stamp) {
//     if (track_.active && (stamp - track_.stamp).toSec() > cfg_.lost_timeout) {
//       clearStateUnlocked();
//     }
//   }

//   void ingestObservation(const LeadObservation& obs,
//                          const EgoPose2D& ego_at_obs,
//                          double ego_yaw_rate,
//                          double ego_yaw_sigma,
//                          double steer_angle_rad,
//                          TrackerDebugState* debug) {
//     if (!obs.valid) {
//       if (debug != NULL) {
//         debug->history_reason = "invalid_observation";
//         debug->history_size = exportHistoryUnlocked().size();
//       }
//       return;
//     }

//     const double sigma =
//         cfg_.innovation_base_gate + cfg_.innovation_range_gain * obs.range +
//         cfg_.innovation_turn_gain * std::fabs(ego_yaw_rate) * obs.range;
//     if (track_.active) {
//       const LeadState2D pred = predictLeadState(track_, obs.stamp);
//       const double residual =
//           pointDistance(obs.world_x, obs.world_y, pred.x, pred.y);
//       const double metric = residual / std::max(1e-3, sigma);
//       last_innovation_norm_ = residual;
//       last_innovation_metric_ = metric;
//       if (metric > cfg_.outlier_reject_scale && (obs.stamp - track_.stamp).toSec() <= cfg_.lost_timeout) {
//         if (debug != NULL) {
//           debug->history_reason = "outlier_rejected";
//           debug->history_size = exportHistoryUnlocked().size();
//         }
//         return;
//       }
//     } else {
//       last_innovation_norm_ = 0.0;
//       last_innovation_metric_ = 0.0;
//     }

//     LeadNode node;
//     node.stamp = obs.stamp;
//     node.ego = ego_at_obs;
//     node.x = obs.world_x;
//     node.y = obs.world_y;
//     node.yaw = obs.world_heading;
//     node.heading_hint = obs.world_heading;
//     node.has_measurement = true;
//     node.z_rel_x = obs.local_x;
//     node.z_rel_y = obs.local_y;
//     node.range = obs.range;
//     node.w_cross =
//         cfg_.measurement_cross_sigma + cfg_.measurement_cross_range_gain * obs.range +
//         cfg_.measurement_cross_yaw_uncertainty_gain * obs.range * ego_yaw_sigma +
//         0.10 * std::fabs(steer_angle_rad);
//     node.w_along = cfg_.measurement_along_sigma;

//     if (nodes_.empty() || (obs.stamp - nodes_.back().stamp).toSec() > cfg_.lost_timeout) {
//       clearStateUnlocked();
//       static_anchor_x_ = obs.world_x;
//       static_anchor_y_ = obs.world_y;
//     }

//     updateMeasurementHypothesisUnlocked(&node);

//     nodes_.push_back(node);

//     pruneWindowUnlocked(obs.stamp);
//     smoothWindowUnlocked();
//     updateTrackFromWindowUnlocked(debug);
//   }

//   void pruneHistory(const ros::Time& now) {
//     pruneWindowUnlocked(now);
//   }

//   std::vector<HistoryPoint2D> worldHistoryForPublish(const ros::Time& stamp) const {
//     (void)stamp;
//     return exportHistoryUnlocked();
//   }

//   const LeadState2D& track() const { return track_; }
//   bool hasActiveTrack() const { return track_.active; }
//   bool movingFlag() const { return moving_flag_; }
//   double innovationNorm() const { return last_innovation_norm_; }
//   double innovationMetric() const { return last_innovation_metric_; }

//  private:
//   double estimateWindowProgressUnlocked() const {
//     if (nodes_.size() < 2) {
//       return 0.0;
//     }
//     double progress = 0.0;
//     for (size_t i = 1; i < nodes_.size(); ++i) {
//       const double tx = std::cos(nodes_[i - 1].yaw);
//       const double ty = std::sin(nodes_[i - 1].yaw);
//       const double dx = nodes_[i].x - nodes_[i - 1].x;
//       const double dy = nodes_[i].y - nodes_[i - 1].y;
//       progress += std::max(0.0, dx * tx + dy * ty);
//     }
//     return progress;
//   }

//   double meanFixedFrameSpeedUnlocked() const {
//     if (nodes_.empty()) {
//       return 0.0;
//     }
//     double sum = 0.0;
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       sum += nodes_[i].fixed_frame_speed;
//     }
//     return sum / static_cast<double>(nodes_.size());
//   }

//   double regressionSpeedFromNodesUnlocked(const std::deque<LeadNode>& seq) const {
//     if (seq.size() < 2) {
//       return 0.0;
//     }

//     const LeadNode& first = seq.front();
//     const LeadNode& last = seq.back();
//     const double ref_dx = last.x - first.x;
//     const double ref_dy = last.y - first.y;
//     double ref_yaw = 0.0;
//     if (hypot2(ref_dx, ref_dy) > 1e-3) {
//       ref_yaw = std::atan2(ref_dy, ref_dx);
//     } else {
//       double sx = 0.0;
//       double sy = 0.0;
//       for (size_t i = 0; i < seq.size(); ++i) {
//         sx += std::cos(seq[i].yaw);
//         sy += std::sin(seq[i].yaw);
//       }
//       ref_yaw = std::atan2(sy, sx);
//     }

//     const double tx = std::cos(ref_yaw);
//     const double ty = std::sin(ref_yaw);
//     const double t0 = first.stamp.toSec();
//     const double x0 = first.x;
//     const double y0 = first.y;

//     double wsum = 0.0;
//     double mean_t = 0.0;
//     double mean_s = 0.0;
//     for (size_t i = 0; i < seq.size(); ++i) {
//       const double ti = seq[i].stamp.toSec() - t0;
//       const double si = (seq[i].x - x0) * tx + (seq[i].y - y0) * ty;
//       const double wi = std::max(1e-3, seq[i].robust_weight);
//       wsum += wi;
//       mean_t += wi * ti;
//       mean_s += wi * si;
//     }
//     if (wsum < 1e-6) {
//       return 0.0;
//     }
//     mean_t /= wsum;
//     mean_s /= wsum;

//     double cov_ts = 0.0;
//     double var_t = 0.0;
//     for (size_t i = 0; i < seq.size(); ++i) {
//       const double ti = seq[i].stamp.toSec() - t0;
//       const double si = (seq[i].x - x0) * tx + (seq[i].y - y0) * ty;
//       const double wi = std::max(1e-3, seq[i].robust_weight);
//       cov_ts += wi * (ti - mean_t) * (si - mean_s);
//       var_t += wi * (ti - mean_t) * (ti - mean_t);
//     }
//     if (var_t < 1e-6) {
//       return 0.0;
//     }
//     return std::fabs(cov_ts / var_t);
//   }

//   double meanStaticHypothesisResidualUnlocked() const {
//     if (nodes_.empty()) {
//       return 0.0;
//     }
//     double sum = 0.0;
//     double wsum = 0.0;
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       const double w = std::max(1e-3, nodes_[i].robust_weight);
//       sum += w * nodes_[i].static_hypothesis_residual;
//       wsum += w;
//     }
//     return sum / std::max(1e-3, wsum);
//   }

//   double meanDynamicHypothesisResidualUnlocked() const {
//     if (nodes_.empty()) {
//       return 0.0;
//     }
//     double sum = 0.0;
//     double wsum = 0.0;
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       const double w = std::max(1e-3, nodes_[i].robust_weight);
//       sum += w * nodes_[i].short_window_prediction_residual;
//       wsum += w;
//     }
//     return sum / std::max(1e-3, wsum);
//   }

//   double meanEgoSpeedUnlocked() const {
//     if (nodes_.empty()) {
//       return 0.0;
//     }
//     double sum = 0.0;
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       sum += std::fabs(nodes_[i].ego.v);
//     }
//     return sum / static_cast<double>(nodes_.size());
//   }

//   double dynamicTrailProgressUnlocked() const {
//     if (tail_buffer_world_.size() < 2) {
//       return estimateWindowProgressUnlocked();
//     }
//     double progress = 0.0;
//     for (size_t i = 1; i < tail_buffer_world_.size(); ++i) {
//       const double tx = std::cos(tail_buffer_world_[i - 1].heading);
//       const double ty = std::sin(tail_buffer_world_[i - 1].heading);
//       const double dx = tail_buffer_world_[i].x - tail_buffer_world_[i - 1].x;
//       const double dy = tail_buffer_world_[i].y - tail_buffer_world_[i - 1].y;
//       progress += std::max(0.0, dx * tx + dy * ty);
//     }
//     return progress;
//   }

//   double staticHypothesisScoreUnlocked() const {
//     if (nodes_.empty()) {
//       return 1.0;
//     }
//     const double residual_term =
//         meanStaticHypothesisResidualUnlocked() / std::max(1e-3, cfg_.ego_compensated_static_gate);
//     const double speed_term =
//         meanFixedFrameSpeedUnlocked() / std::max(1e-3, cfg_.ego_compensated_moving_gate);
//     const double progress_term =
//         dynamicTrailProgressUnlocked() /
//         std::max(0.05, static_cast<double>(std::max<size_t>(2, nodes_.size() - 1)) * cfg_.temporal_min_progress);
//     const double dynamic_advantage =
//         meanDynamicHypothesisResidualUnlocked() - meanStaticHypothesisResidualUnlocked();
//     const double advantage_term = clampValue(dynamic_advantage / std::max(1e-3, cfg_.ego_compensated_static_gate),
//                                              -1.0,
//                                              2.0);
//     return 0.55 * residual_term + 0.20 * speed_term + 0.15 * progress_term - 0.10 * advantage_term;
//   }

//   double dynamicHypothesisScoreUnlocked() const {
//     if (nodes_.empty()) {
//       return 1.0;
//     }
//     const double residual_term =
//         meanDynamicHypothesisResidualUnlocked() / std::max(1e-3, cfg_.ego_compensated_moving_gate);
//     const double speed_ratio =
//         meanFixedFrameSpeedUnlocked() / std::max(1e-3, cfg_.ego_compensated_static_gate);
//     const double progress =
//         dynamicTrailProgressUnlocked() /
//         std::max(0.05, static_cast<double>(std::max<size_t>(2, nodes_.size() - 1)) * cfg_.temporal_min_progress);
//     const double speed_penalty =
//         clampValue((cfg_.lead_moving_speed_gate - meanFixedFrameSpeedUnlocked()) /
//                        std::max(1e-3, cfg_.lead_moving_speed_gate),
//                    0.0,
//                    1.5);
//     const double progress_penalty =
//         clampValue((cfg_.confirm_min_progress - dynamicTrailProgressUnlocked()) /
//                        std::max(1e-3, cfg_.confirm_min_progress),
//                    0.0,
//                    1.5);
//     const double advantage_penalty =
//         clampValue((meanDynamicHypothesisResidualUnlocked() - meanStaticHypothesisResidualUnlocked()) /
//                        std::max(1e-3, cfg_.ego_compensated_static_gate),
//                    0.0,
//                    2.0);
//     (void)speed_ratio;
//     (void)progress;
//     return 0.60 * residual_term + 0.15 * speed_penalty + 0.15 * progress_penalty + 0.10 * advantage_penalty;
//   }

//   bool dynamicHypothesisWinsUnlocked() const {
//     return dynamicHypothesisScoreUnlocked() + cfg_.dual_hypothesis_score_margin <
//            staticHypothesisScoreUnlocked();
//   }

//   bool confirmDynamicHypothesisWinsUnlocked() const {
//     return dynamicHypothesisScoreUnlocked() + cfg_.confirm_dynamic_score_margin <
//            staticHypothesisScoreUnlocked();
//   }

//   double residualAgainstWorldPointUnlocked(const EgoState& ego,
//                                            double world_x,
//                                            double world_y,
//                                            double meas_x,
//                                            double meas_y) const {
//     double expected_local_x = 0.0;
//     double expected_local_y = 0.0;
//     transformWorldToLocal(ego, world_x, world_y, &expected_local_x, &expected_local_y);
//     return pointDistance(meas_x, meas_y, expected_local_x, expected_local_y);
//   }

//   double estimateStaticConfidenceUnlocked() const {
//     if (nodes_.empty()) {
//       return 1.0;
//     }
//     double residual_cross_sum = 0.0;
//     double residual_along_sum = 0.0;
//     double weight_sum = 0.0;
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       const double w = std::max(1e-3, nodes_[i].robust_weight);
//       residual_cross_sum += w * std::fabs(nodes_[i].residual_cross);
//       residual_along_sum += w * std::fabs(nodes_[i].residual_along);
//       weight_sum += w;
//     }
//     const double cross_mean = residual_cross_sum / std::max(1e-3, weight_sum);
//     const double along_mean = residual_along_sum / std::max(1e-3, weight_sum);
//     const double progress = estimateWindowProgressUnlocked();
//     const double progress_ratio = progress / std::max(0.3, static_cast<double>(nodes_.size()) * cfg_.temporal_min_progress);
//     const double cross_ratio = cross_mean / std::max(1e-3, cfg_.measurement_cross_sigma);
//     const double along_ratio = along_mean / std::max(1e-3, cfg_.measurement_along_sigma);
//     const double confidence = 1.0 - clampValue(0.55 * progress_ratio + 0.25 * along_ratio + 0.20 * cross_ratio,
//                                                0.0,
//                                                1.0);
//     return clampValue(confidence, 0.0, 1.0);
//   }

//   void applyStaticAnchorFactorUnlocked() {
//     if (nodes_.empty()) {
//       return;
//     }
//     const double static_confidence = estimateStaticConfidenceUnlocked();
//     const bool enable_anchor = track_.static_mode || static_votes_ > 0 || static_confidence > 0.65;
//     if (!enable_anchor) {
//       return;
//     }

//     const double measurement_static =
//         1.0 - clampValue(meanStaticHypothesisResidualUnlocked() /
//                              std::max(1e-3, cfg_.ego_compensated_static_gate),
//                          0.0,
//                          1.0);
//     const double anchor_gain =
//         cfg_.anchor_alpha * (0.25 + 0.45 * static_confidence + 0.30 * measurement_static);
//     const double preserve_span = std::max(0.05, cfg_.static_preserve_trail_span);
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       const double age_weight =
//           (i + 1 == nodes_.size()) ? 1.0 : clampValue(1.0 - (nodes_.back().stamp - nodes_[i].stamp).toSec() / 2.0,
//                                                       0.15,
//                                                       1.0);
//       const double dx = static_anchor_x_ - nodes_[i].x;
//       const double dy = static_anchor_y_ - nodes_[i].y;
//       const double dist = hypot2(dx, dy);
//       if (dist > preserve_span && i + 1 < nodes_.size()) {
//         continue;
//       }
//       const double gain = anchor_gain * age_weight * nodes_[i].robust_weight;
//       nodes_[i].x += gain * dx;
//       nodes_[i].y += gain * dy;
//     }
//   }

//   void updateMeasurementHypothesisUnlocked(LeadNode* node) const {
//     if (node == NULL) {
//       return;
//     }
//     if (nodes_.empty()) {
//       node->fixed_frame_speed = 0.0;
//       node->static_hypothesis_residual = 0.0;
//       return;
//     }

//     const LeadNode& prev = nodes_.back();
//     std::deque<LeadNode> window = nodes_;
//     window.push_back(*node);
//     const size_t max_window = 20;
//     while (window.size() > max_window) {
//       window.pop_front();
//     }
//     node->fixed_frame_speed = regressionSpeedFromNodesUnlocked(window);
//     const double prev_residual =
//         residualAgainstWorldPointUnlocked(node->ego, prev.x, prev.y, node->z_rel_x, node->z_rel_y);

//     double trail_tail_residual = prev_residual;
//     if (!confirmed_trail_world_.empty()) {
//       const HistoryPoint2D& tail = confirmed_trail_world_.back();
//       trail_tail_residual =
//           residualAgainstWorldPointUnlocked(node->ego, tail.x, tail.y, node->z_rel_x, node->z_rel_y);
//     }

//     double window_pred_residual = prev_residual;
//     if (nodes_.size() >= 2) {
//       const double dt_pred = std::max(1e-3, (node->stamp - nodes_.back().stamp).toSec());
//       const LeadNode predicted = predictNodeForwardUnlocked(nodes_.back(), dt_pred);
//       window_pred_residual =
//           residualAgainstWorldPointUnlocked(node->ego, predicted.x, predicted.y, node->z_rel_x, node->z_rel_y);
//     }

//     node->static_hypothesis_residual = std::min(prev_residual, trail_tail_residual);
//     node->short_window_prediction_residual = window_pred_residual;
//   }

//   void pullWindowTowardStaticHypothesisUnlocked() {
//     if (nodes_.size() < 2) {
//       return;
//     }
//     for (size_t i = 1; i < nodes_.size(); ++i) {
//       if (nodes_[i].static_hypothesis_residual > cfg_.ego_compensated_static_gate) {
//         continue;
//       }
//       const double confidence =
//           1.0 - clampValue(nodes_[i].static_hypothesis_residual /
//                                std::max(1e-3, cfg_.ego_compensated_static_gate),
//                            0.0,
//                            1.0);
//       const double gain =
//           cfg_.static_measurement_anchor_alpha * confidence * nodes_[i].robust_weight;
//       nodes_[i].x = (1.0 - gain) * nodes_[i].x + gain * nodes_[i - 1].x;
//       nodes_[i].y = (1.0 - gain) * nodes_[i].y + gain * nodes_[i - 1].y;
//     }
//   }

//   LeadNode predictNodeForwardUnlocked(const LeadNode& prev, double dt) const {
//     LeadNode pred = prev;
//     const double v = prev.v + prev.a * dt;
//     const double kappa = prev.kappa + prev.dkappa * dt;
//     const double yaw_rate = prev.v * prev.kappa;
//     const double mid_yaw = prev.yaw + 0.5 * yaw_rate * dt;
//     pred.x = prev.x + prev.v * std::cos(mid_yaw) * dt;
//     pred.y = prev.y + prev.v * std::sin(mid_yaw) * dt;
//     pred.yaw = wrapAngle(prev.yaw + yaw_rate * dt);
//     pred.v = v;
//     pred.kappa = clampValue(kappa, -0.8, 0.8);
//     return pred;
//   }

//   LeadNode predictNodeBackwardUnlocked(const LeadNode& next, double dt) const {
//     LeadNode pred = next;
//     const double kappa = next.kappa - next.dkappa * dt;
//     const double v = next.v - next.a * dt;
//     const double yaw_rate = v * kappa;
//     const double mid_yaw = next.yaw - 0.5 * yaw_rate * dt;
//     pred.x = next.x - v * std::cos(mid_yaw) * dt;
//     pred.y = next.y - v * std::sin(mid_yaw) * dt;
//     pred.yaw = wrapAngle(next.yaw - yaw_rate * dt);
//     pred.v = v;
//     pred.kappa = clampValue(kappa, -0.8, 0.8);
//     return pred;
//   }

//   void applyDynamicsConsistencyUnlocked() {
//     if (nodes_.size() < 3) {
//       return;
//     }

//     std::deque<LeadNode> next_nodes = nodes_;
//     for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
//       const double dt_prev = std::max(1e-3, (nodes_[i].stamp - nodes_[i - 1].stamp).toSec());
//       const double dt_next = std::max(1e-3, (nodes_[i + 1].stamp - nodes_[i].stamp).toSec());
//       const LeadNode pred_prev = predictNodeForwardUnlocked(nodes_[i - 1], dt_prev);
//       const LeadNode pred_next = predictNodeBackwardUnlocked(nodes_[i + 1], dt_next);
//       const double w_prev = std::max(1e-3, nodes_[i - 1].robust_weight);
//       const double w_next = std::max(1e-3, nodes_[i + 1].robust_weight);
//       const double norm = 1.0 / (w_prev + w_next);

//       const double dyn_x = (w_prev * pred_prev.x + w_next * pred_next.x) * norm;
//       const double dyn_y = (w_prev * pred_prev.y + w_next * pred_next.y) * norm;
//       const double dyn_v = (w_prev * pred_prev.v + w_next * pred_next.v) * norm;
//       const double dyn_kappa = clampValue((w_prev * pred_prev.kappa + w_next * pred_next.kappa) * norm, -0.8, 0.8);

//       const double yaw_prev_err = wrapAngle(pred_prev.yaw - nodes_[i].yaw);
//       const double yaw_next_err = wrapAngle(pred_next.yaw - nodes_[i].yaw);
//       const double yaw_dyn = wrapAngle(nodes_[i].yaw + (w_prev * yaw_prev_err + w_next * yaw_next_err) * norm);

//       next_nodes[i].x =
//           (1.0 - cfg_.dynamics_position_alpha) * nodes_[i].x + cfg_.dynamics_position_alpha * dyn_x;
//       next_nodes[i].y =
//           (1.0 - cfg_.dynamics_position_alpha) * nodes_[i].y + cfg_.dynamics_position_alpha * dyn_y;
//       next_nodes[i].yaw = wrapAngle((1.0 - cfg_.dynamics_heading_alpha) * nodes_[i].yaw +
//                                     cfg_.dynamics_heading_alpha * yaw_dyn);
//       next_nodes[i].v = (1.0 - cfg_.dynamics_position_alpha) * nodes_[i].v +
//                         cfg_.dynamics_position_alpha * dyn_v;
//       next_nodes[i].kappa =
//           (1.0 - cfg_.curvature_prediction_alpha) * nodes_[i].kappa +
//           cfg_.curvature_prediction_alpha * dyn_kappa;
//     }
//     nodes_.swap(next_nodes);
//   }

//   void regularizeDynamicStatesUnlocked() {
//     if (nodes_.size() < 3) {
//       return;
//     }

//     std::deque<LeadNode> next_nodes = nodes_;
//     for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
//       const double dt_prev = std::max(1e-3, (nodes_[i].stamp - nodes_[i - 1].stamp).toSec());
//       const double dt_next = std::max(1e-3, (nodes_[i + 1].stamp - nodes_[i].stamp).toSec());
//       const double a_smooth = 0.5 * (nodes_[i - 1].a + nodes_[i + 1].a);
//       const double dkappa_smooth = 0.5 * (nodes_[i - 1].dkappa + nodes_[i + 1].dkappa);
//       const double kappa_pred_prev = nodes_[i - 1].kappa + nodes_[i - 1].dkappa * dt_prev;
//       const double kappa_pred_next = nodes_[i + 1].kappa - nodes_[i + 1].dkappa * dt_next;
//       const double kappa_smooth = 0.5 * (kappa_pred_prev + kappa_pred_next);

//       next_nodes[i].a =
//           (1.0 - cfg_.acceleration_regularization_alpha) * nodes_[i].a +
//           cfg_.acceleration_regularization_alpha * a_smooth;
//       next_nodes[i].dkappa =
//           (1.0 - cfg_.dkappa_regularization_alpha) * nodes_[i].dkappa +
//           cfg_.dkappa_regularization_alpha * dkappa_smooth;
//       next_nodes[i].kappa =
//           (1.0 - cfg_.curvature_prediction_alpha) * nodes_[i].kappa +
//           cfg_.curvature_prediction_alpha * clampValue(kappa_smooth, -0.8, 0.8);
//     }
//     nodes_.swap(next_nodes);
//   }

//   void strengthenMeasurementModelUnlocked() {
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       const double lateral_ratio =
//           std::fabs(nodes_[i].z_rel_y) / std::max(0.5, nodes_[i].range);
//       const double heading_gain =
//           cfg_.measurement_heading_alpha * nodes_[i].robust_weight *
//           clampValue(1.0 - 0.6 * lateral_ratio, 0.2, 1.0);
//       const double heading_err = wrapAngle(nodes_[i].heading_hint - nodes_[i].yaw);
//       nodes_[i].yaw = wrapAngle(nodes_[i].yaw + heading_gain * heading_err);
//     }
//   }

//   void clearStateUnlocked() {
//     nodes_.clear();
//     tail_buffer_world_.clear();
//     track_ = LeadState2D();
//     moving_flag_ = false;
//     static_votes_ = 0;
//     moving_votes_ = 0;
//     resetUnconfirmedAccumulatorUnlocked();
//   }

//   void pruneWindowUnlocked(const ros::Time& now) {
//     while (!nodes_.empty() && (now - nodes_.front().stamp).toSec() > cfg_.history_max_age) {
//       nodes_.pop_front();
//     }
//     while (nodes_.size() > static_cast<size_t>(std::max(2, cfg_.history_max_points))) {
//       nodes_.pop_front();
//     }
//   }

//   double robustWeight(double norm_metric) const {
//     const double c = std::max(1e-3, cfg_.robust_cauchy_scale);
//     return 1.0 / (1.0 + (norm_metric * norm_metric) / (c * c));
//   }

//   void smoothWindowUnlocked() {
//     if (nodes_.empty()) {
//       return;
//     }

//     for (int iter = 0; iter < std::max(1, cfg_.smoother_iterations); ++iter) {
//       for (size_t i = 0; i < nodes_.size(); ++i) {
//         double pred_local_x = 0.0;
//         double pred_local_y = 0.0;
//         transformWorldToLocal(nodes_[i].ego, nodes_[i].x, nodes_[i].y,
//                               &pred_local_x, &pred_local_y);
//         const double rx = nodes_[i].z_rel_x - pred_local_x;
//         const double ry = nodes_[i].z_rel_y - pred_local_y;
//         const double metric =
//             std::sqrt((rx * rx) / std::max(1e-6, nodes_[i].w_along * nodes_[i].w_along) +
//                       (ry * ry) / std::max(1e-6, nodes_[i].w_cross * nodes_[i].w_cross));
//         const double w = robustWeight(metric);
//         nodes_[i].residual_along = rx;
//         nodes_[i].residual_cross = ry;
//         nodes_[i].robust_weight = w;
//         const double corr_local_x = cfg_.smoother_position_gain * w * rx;
//         const double corr_local_y = cfg_.smoother_cross_gain * w * ry;
//         const double c = std::cos(nodes_[i].ego.yaw);
//         const double s = std::sin(nodes_[i].ego.yaw);
//         nodes_[i].x += c * corr_local_x - s * corr_local_y;
//         nodes_[i].y += s * corr_local_x + c * corr_local_y;
//         const double dh = wrapAngle(nodes_[i].heading_hint - nodes_[i].yaw);
//         nodes_[i].yaw = wrapAngle(nodes_[i].yaw + cfg_.smoother_heading_alpha * w * dh);
//       }

//       strengthenMeasurementModelUnlocked();
//       pullWindowTowardStaticHypothesisUnlocked();

//       if (nodes_.size() >= 3) {
//         std::deque<LeadNode> next = nodes_;
//         for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
//           const double mx = 0.5 * (nodes_[i - 1].x + nodes_[i + 1].x);
//           const double my = 0.5 * (nodes_[i - 1].y + nodes_[i + 1].y);
//           next[i].x =
//               (1.0 - cfg_.smoother_temporal_alpha) * nodes_[i].x + cfg_.smoother_temporal_alpha * mx;
//           next[i].y =
//               (1.0 - cfg_.smoother_temporal_alpha) * nodes_[i].y + cfg_.smoother_temporal_alpha * my;
//         }
//         nodes_.swap(next);
//       }

//       recomputeKnotGeometryUnlocked();
//       regularizeDynamicStatesUnlocked();
//       applyDynamicsConsistencyUnlocked();
//       applyStaticAnchorFactorUnlocked();
//       enforceProgressionUnlocked();
//       smoothCurvatureUnlocked();
//       recomputeKnotGeometryUnlocked();
//       regularizeDynamicStatesUnlocked();
//     }
//   }

//   void recomputeKnotGeometryUnlocked() {
//     if (nodes_.empty()) {
//       return;
//     }
//     if (nodes_.size() == 1) {
//       nodes_.front().v = 0.0;
//       nodes_.front().kappa = 0.0;
//       nodes_.front().dkappa = 0.0;
//       return;
//     }
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       const size_t i0 = (i == 0) ? i : i - 1;
//       const size_t i1 = (i + 1 >= nodes_.size()) ? i : i + 1;
//       const double dx = nodes_[i1].x - nodes_[i0].x;
//       const double dy = nodes_[i1].y - nodes_[i0].y;
//       if (hypot2(dx, dy) > 1e-4) {
//         nodes_[i].yaw = std::atan2(dy, dx);
//       }
//       const double dt =
//           std::max(1e-3, (nodes_[i1].stamp - nodes_[i0].stamp).toSec());
//       const double v_new = hypot2(dx, dy) / dt;
//       nodes_[i].a = (v_new - nodes_[i].v) / dt;
//       nodes_[i].v = v_new;
//     }
//     nodes_.front().yaw = nodes_[std::min<size_t>(1, nodes_.size() - 1)].yaw;
//     nodes_.back().yaw = nodes_[nodes_.size() - 2].yaw;

//     for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
//       const double ds = std::max(1e-3,
//                                  pointDistance(nodes_[i - 1].x, nodes_[i - 1].y,
//                                                nodes_[i + 1].x, nodes_[i + 1].y));
//       const double dpsi = wrapAngle(nodes_[i + 1].yaw - nodes_[i - 1].yaw);
//       nodes_[i].kappa = clampValue(dpsi / ds, -0.8, 0.8);
//     }
//     if (nodes_.size() >= 2) {
//       nodes_.front().kappa = nodes_[std::min<size_t>(1, nodes_.size() - 1)].kappa;
//       nodes_.back().kappa = nodes_[nodes_.size() - 2].kappa;
//     }
//     for (size_t i = 1; i < nodes_.size(); ++i) {
//       const double dt = std::max(1e-3, (nodes_[i].stamp - nodes_[i - 1].stamp).toSec());
//       nodes_[i - 1].dkappa =
//           clampValue((nodes_[i].kappa - nodes_[i - 1].kappa) / dt, -1.5, 1.5);
//     }
//     if (!nodes_.empty()) {
//       nodes_.back().dkappa = nodes_[nodes_.size() >= 2 ? nodes_.size() - 2 : 0].dkappa;
//     }
//   }

//   void smoothCurvatureUnlocked() {
//     if (nodes_.size() < 3) {
//       return;
//     }
//     std::deque<LeadNode> next = nodes_;
//     for (size_t i = 1; i + 1 < nodes_.size(); ++i) {
//       const double mk = 0.5 * (nodes_[i - 1].kappa + nodes_[i + 1].kappa);
//       next[i].kappa =
//           (1.0 - cfg_.smoother_curvature_alpha) * nodes_[i].kappa +
//           cfg_.smoother_curvature_alpha * mk;
//     }
//     nodes_.swap(next);
//   }

//   void enforceProgressionUnlocked() {
//     if (nodes_.size() < 2) {
//       return;
//     }
//     const double static_confidence = estimateStaticConfidenceUnlocked();
//     const double adaptive_min_progress =
//         cfg_.temporal_min_progress * (1.0 - 0.9 * static_confidence) * (moving_flag_ ? 1.0 : 0.6);
//     for (size_t i = 1; i < nodes_.size(); ++i) {
//       const double tx = std::cos(nodes_[i - 1].yaw);
//       const double ty = std::sin(nodes_[i - 1].yaw);
//       const double dx = nodes_[i].x - nodes_[i - 1].x;
//       const double dy = nodes_[i].y - nodes_[i - 1].y;
//       const double ds = dx * tx + dy * ty;
//       if (ds < adaptive_min_progress) {
//         const double corr = 0.5 * (adaptive_min_progress - ds);
//         nodes_[i].x += corr * tx;
//         nodes_[i].y += corr * ty;
//       }
//     }
//   }

//   HistoryPoint2D nodeToHistoryPointUnlocked(const LeadNode& node) const {
//     HistoryPoint2D pt;
//     pt.stamp = node.stamp;
//     pt.x = node.x;
//     pt.y = node.y;
//     pt.heading = node.yaw;
//     pt.speed = node.v;
//     pt.curvature = node.kappa;
//     return pt;
//   }

//   void refreshTailBufferUnlocked() {
//     tail_buffer_world_.clear();
//     tail_buffer_world_.resize(nodes_.size());
//     for (size_t i = 0; i < nodes_.size(); ++i) {
//       tail_buffer_world_[i] = nodeToHistoryPointUnlocked(nodes_[i]);
//     }
//     if (track_.static_mode && !tail_buffer_world_.empty()) {
//       tail_buffer_world_.back().x = static_anchor_x_;
//       tail_buffer_world_.back().y = static_anchor_y_;
//       tail_buffer_world_.back().heading = track_.heading;
//     }
//   }

//   void resetUnconfirmedAccumulatorUnlocked() {
//     unconfirmed_accum_initialized_ = false;
//     unconfirmed_progress_accum_ = 0.0;
//     unconfirmed_time_accum_ = 0.0;
//     unconfirmed_last_x_ = 0.0;
//     unconfirmed_last_y_ = 0.0;
//     unconfirmed_last_heading_ = 0.0;
//     unconfirmed_last_stamp_ = ros::Time();
//     unconfirmed_start_stamp_ = ros::Time();
//   }

//   void updateUnconfirmedAccumulatorUnlocked(const HistoryPoint2D& candidate) {
//     if (!unconfirmed_accum_initialized_ || !candidate.stamp.isValid() ||
//         (unconfirmed_last_stamp_.isValid() && candidate.stamp <= unconfirmed_last_stamp_)) {
//       unconfirmed_accum_initialized_ = true;
//       unconfirmed_progress_accum_ = 0.0;
//       unconfirmed_time_accum_ = 0.0;
//       unconfirmed_last_x_ = candidate.x;
//       unconfirmed_last_y_ = candidate.y;
//       unconfirmed_last_heading_ = candidate.heading;
//       unconfirmed_last_stamp_ = candidate.stamp;
//       unconfirmed_start_stamp_ = candidate.stamp;
//       return;
//     }

//     const double dx = candidate.x - unconfirmed_last_x_;
//     const double dy = candidate.y - unconfirmed_last_y_;
//     const double seg_progress =
//         std::max(0.0, dx * std::cos(unconfirmed_last_heading_) + dy * std::sin(unconfirmed_last_heading_));
//     unconfirmed_progress_accum_ += seg_progress;
//     unconfirmed_time_accum_ = std::max(0.0, (candidate.stamp - unconfirmed_start_stamp_).toSec());
//     unconfirmed_last_x_ = candidate.x;
//     unconfirmed_last_y_ = candidate.y;
//     unconfirmed_last_heading_ = candidate.heading;
//     unconfirmed_last_stamp_ = candidate.stamp;
//   }

//   bool staticHypothesisWinsUnlocked() const {
//     return staticHypothesisScoreUnlocked() <=
//            dynamicHypothesisScoreUnlocked() + cfg_.dual_hypothesis_score_margin;
//   }

//   void pruneTrailContainerUnlocked(std::deque<HistoryPoint2D>* trail, const ros::Time& now) {
//     if (trail == NULL) {
//       return;
//     }
//     while (!trail->empty() && (now - trail->front().stamp).toSec() > cfg_.history_max_age) {
//       trail->pop_front();
//     }
//     while (trail->size() > static_cast<size_t>(std::max(2, cfg_.history_max_points))) {
//       trail->pop_front();
//     }
//   }

//   void commitConfirmedTrailPointUnlocked() {
//     if (tail_buffer_world_.size() < static_cast<size_t>(std::max(2, cfg_.tail_confirm_min_points))) {
//       if (track_.static_mode) {
//         last_history_action_ = "history_anchor_hold";
//       }
//       return;
//     }

//     if (track_.static_mode || !confirmDynamicHypothesisWinsUnlocked()) {
//       last_history_action_ = "history_anchor_hold";
//       return;
//     }

//     const size_t candidate_index =
//         tail_buffer_world_.size() - static_cast<size_t>(std::max(2, cfg_.tail_confirm_min_points - 1));
//     HistoryPoint2D candidate = tail_buffer_world_[candidate_index];
//     updateUnconfirmedAccumulatorUnlocked(candidate);
//     const double ego_speed = meanEgoSpeedUnlocked();
//     const double min_confirm_age =
//         (ego_speed < cfg_.ego_static_speed_gate) ? 0.08 : cfg_.tail_confirm_min_age;
//     const double min_confirm_progress =
//         (ego_speed < cfg_.ego_static_speed_gate) ? 0.03 : cfg_.confirm_min_progress;
//     const double candidate_static_res =
//         residualAgainstWorldPointUnlocked(nodes_.back().ego, candidate.x, candidate.y,
//                                           nodes_.back().z_rel_x, nodes_.back().z_rel_y);

//     if ((tail_buffer_world_.back().stamp - candidate.stamp).toSec() < min_confirm_age) {
//       last_history_action_ = "history_tail_too_fresh";
//       return;
//     }

//     if (dynamicTrailProgressUnlocked() < min_confirm_progress) {
//       last_history_action_ = "history_progress_too_small";
//       return;
//     }

//     const double accum_distance_gate = std::min(cfg_.tail_confirm_min_distance, cfg_.confirm_accum_distance);
//     const double accum_timeout_gate = cfg_.confirm_accum_timeout;
//     const bool accum_ready =
//         unconfirmed_progress_accum_ >= accum_distance_gate ||
//         (unconfirmed_time_accum_ >= accum_timeout_gate &&
//          unconfirmed_progress_accum_ >= 3.0 * cfg_.measurement_cross_sigma);
//     if (!accum_ready) {
//       last_history_action_ = "history_progress_accumulating";
//       return;
//     }

//     const bool dynamic_override_static =
//         accum_ready &&
//         confirmDynamicHypothesisWinsUnlocked() &&
//         (unconfirmed_progress_accum_ >= cfg_.confirm_accum_distance ||
//          unconfirmed_time_accum_ >= cfg_.confirm_accum_timeout) &&
//         dynamicTrailProgressUnlocked() >= 0.7 * min_confirm_progress;
//     if (ego_speed >= cfg_.ego_static_speed_gate &&
//         candidate_static_res < cfg_.confirm_static_residual_gate &&
//         !dynamic_override_static) {
//       last_history_action_ = "history_static_explained";
//       return;
//     }
//     if (ego_speed >= cfg_.ego_static_speed_gate &&
//         candidate_static_res < cfg_.confirm_static_residual_gate &&
//         dynamic_override_static) {
//       last_history_action_ = "history_dynamic_accum_override";
//     }

//     if (confirmed_trail_world_.empty()) {
//       candidate.predicted = false;
//       confirmed_trail_world_.push_back(candidate);
//       if (last_history_action_ != "history_dynamic_accum_override") {
//         last_history_action_ = "history_bootstrap";
//       }
//       pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
//       resetUnconfirmedAccumulatorUnlocked();
//       return;
//     }

//     const HistoryPoint2D& last_confirmed = confirmed_trail_world_.back();
//     if (candidate.stamp <= last_confirmed.stamp) {
//       last_history_action_ = "history_lowpass_update";
//       return;
//     }

//     const double dist = pointDistance(last_confirmed.x, last_confirmed.y, candidate.x, candidate.y);
//     const double heading_err = std::fabs(wrapAngle(candidate.heading - last_confirmed.heading));
//     const double confirm_dt = std::max(1e-3, (candidate.stamp - last_confirmed.stamp).toSec());
//     const double confirm_speed = dist / confirm_dt;
//     const bool append =
//         dist >= cfg_.tail_confirm_min_distance ||
//         (dist >= 0.08 && heading_err > 0.18) ||
//         (confirm_speed > cfg_.lead_static_speed_gate && dist >= 0.06);

//     if (append) {
//       candidate.predicted = false;
//       confirmed_trail_world_.push_back(candidate);
//       if (last_history_action_ != "history_dynamic_accum_override") {
//         last_history_action_ = "history_appended";
//       }
//       resetUnconfirmedAccumulatorUnlocked();
//     } else {
//       last_history_action_ = "history_lowpass_update";
//     }

//     pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
//   }

//   std::vector<HistoryPoint2D> exportHistoryUnlocked() const {
//     std::vector<HistoryPoint2D> history;
//     history.reserve(confirmed_trail_world_.size() + tail_buffer_world_.size());
//     for (size_t i = 0; i < confirmed_trail_world_.size(); ++i) {
//       HistoryPoint2D pt = confirmed_trail_world_[i];
//       pt.predicted = false;
//       history.push_back(pt);
//     }

//     if (tail_buffer_world_.empty()) {
//       return removeNearDuplicatePoints(history, 0.01);
//     }

//     if (!dynamicHypothesisWinsUnlocked() || confirmed_trail_world_.empty()) {
//       return removeNearDuplicatePoints(history, 0.01);
//     }

//     const size_t keep_tail =
//         std::min(tail_buffer_world_.size(), static_cast<size_t>(std::max(0, cfg_.raw_tail_keep_points)));
//     const size_t start_idx = tail_buffer_world_.size() - keep_tail;
//     const HistoryPoint2D& confirmed_tail = confirmed_trail_world_.back();
//     for (size_t i = start_idx; i < tail_buffer_world_.size(); ++i) {
//       if (tail_buffer_world_[i].predicted) {
//         continue;
//       }
//       const double dist_from_confirmed =
//           pointDistance(confirmed_tail.x, confirmed_tail.y, tail_buffer_world_[i].x, tail_buffer_world_[i].y);
//       if (dist_from_confirmed > std::max(0.25, 3.0 * cfg_.tail_confirm_min_distance)) {
//         continue;
//       }
//       if (!history.empty() &&
//           pointDistance(history.back().x, history.back().y, tail_buffer_world_[i].x, tail_buffer_world_[i].y) <
//               0.02) {
//         HistoryPoint2D pt = tail_buffer_world_[i];
//         pt.predicted = true;
//         history.back() = pt;
//       } else {
//         HistoryPoint2D pt = tail_buffer_world_[i];
//         pt.predicted = true;
//         history.push_back(pt);
//       }
//     }

//     return removeNearDuplicatePoints(history, 0.01);
//   }

//   void updateTrackFromWindowUnlocked(TrackerDebugState* debug) {
//     if (nodes_.empty()) {
//       track_ = LeadState2D();
//       moving_flag_ = false;
//       if (debug != NULL) {
//         debug->history_reason = "window_empty";
//         debug->history_size = 0;
//       }
//       return;
//     }

//     const LeadNode& last = nodes_.back();
//     track_.stamp = last.stamp;
//     track_.x = last.x;
//     track_.y = last.y;
//     track_.heading = last.yaw;
//     track_.active = true;
//     track_.vx = last.v * std::cos(last.yaw);
//     track_.vy = last.v * std::sin(last.yaw);

//     double window_disp = 0.0;
//     double window_speed = 0.0;
//     double window_progress = 0.0;
//     const double measurement_speed = meanFixedFrameSpeedUnlocked();
//     const double static_measurement_residual = meanStaticHypothesisResidualUnlocked();
//     double short_window_prediction_residual = 0.0;
//     if (!nodes_.empty()) {
//       for (size_t i = 0; i < nodes_.size(); ++i) {
//         short_window_prediction_residual += nodes_[i].short_window_prediction_residual;
//       }
//       short_window_prediction_residual /= static_cast<double>(nodes_.size());
//     }
//     if (nodes_.size() >= 2) {
//       window_disp = pointDistance(nodes_.front().x, nodes_.front().y, nodes_.back().x, nodes_.back().y);
//       const double dt =
//           std::max(1e-3, (nodes_.back().stamp - nodes_.front().stamp).toSec());
//       window_speed = window_disp / dt;
//       window_progress = estimateWindowProgressUnlocked() / dt;
//     }
//     const double current_speed = last.v;
//     const double vote_speed =
//         std::max(current_speed, std::max(measurement_speed, std::max(window_speed, window_progress)));
//     const double static_confidence = estimateStaticConfidenceUnlocked();
//     const double mean_ego_speed = meanEgoSpeedUnlocked();

//     const double hs_score = staticHypothesisScoreUnlocked();
//     const double hd_score = dynamicHypothesisScoreUnlocked();

//     if (vote_speed < cfg_.lead_static_speed_gate &&
//         measurement_speed < cfg_.ego_compensated_static_gate &&
//         std::min(static_measurement_residual, short_window_prediction_residual) <
//             cfg_.ego_compensated_static_gate &&
//         hs_score <= hd_score + cfg_.dual_hypothesis_score_margin &&
//         static_confidence > 0.55 &&
//         last_innovation_norm_ < cfg_.static_residual_gate) {
//       static_votes_ = std::min(static_votes_ + 1, 1000);
//     } else {
//       static_votes_ = std::max(0, static_votes_ - 1);
//     }

//     if (vote_speed > cfg_.lead_moving_speed_gate &&
//         measurement_speed > cfg_.ego_compensated_moving_gate &&
//         static_measurement_residual > 0.5 * cfg_.ego_compensated_static_gate &&
//         short_window_prediction_residual > 0.5 * cfg_.ego_compensated_static_gate &&
//         hd_score + cfg_.dual_hypothesis_score_margin < hs_score &&
//         static_confidence < 0.55 &&
//         last_innovation_norm_ < cfg_.moving_residual_gate) {
//       moving_votes_ = std::min(moving_votes_ + 1, 1000);
//     } else if (mean_ego_speed < cfg_.ego_static_speed_gate &&
//                vote_speed > 0.5 * cfg_.lead_moving_speed_gate &&
//                hd_score < hs_score &&
//                dynamicTrailProgressUnlocked() > 0.5 * cfg_.confirm_min_progress) {
//       moving_votes_ = std::min(moving_votes_ + 1, 1000);
//     } else {
//       moving_votes_ = std::max(0, moving_votes_ - 1);
//     }

//     if (!track_.static_mode && static_votes_ >= cfg_.static_vote_on) {
//       track_.static_mode = true;
//       static_anchor_x_ = track_.x;
//       static_anchor_y_ = track_.y;
//       moving_flag_ = false;
//     }

//     if (track_.static_mode) {
//       const bool strong_static = hs_score + cfg_.static_release_score_margin < hd_score;
//       if (strong_static) {
//         static_anchor_x_ = (1.0 - cfg_.anchor_alpha) * static_anchor_x_ + cfg_.anchor_alpha * track_.x;
//         static_anchor_y_ = (1.0 - cfg_.anchor_alpha) * static_anchor_y_ + cfg_.anchor_alpha * track_.y;
//       }
//       track_.x = static_anchor_x_;
//       track_.y = static_anchor_y_;
//       track_.vx = 0.0;
//       track_.vy = 0.0;
//       if ((static_votes_ <= cfg_.static_vote_release && moving_votes_ >= cfg_.moving_vote_on) ||
//           (hd_score + cfg_.static_release_score_margin < hs_score)) {
//         track_.static_mode = false;
//         moving_flag_ = true;
//       }
//     } else {
//       moving_flag_ = moving_votes_ >= cfg_.moving_vote_on;
//     }

//     refreshTailBufferUnlocked();
//     commitConfirmedTrailPointUnlocked();

//     if (debug != NULL) {
//       debug->history_reason = track_.static_mode ? "static_anchor_update" : last_history_action_;
//       debug->history_size = exportHistoryUnlocked().size();
//       debug->ff_speed = measurement_speed;
//       debug->mean_ego_speed = mean_ego_speed;
//       debug->static_hyp_res = std::min(static_measurement_residual, short_window_prediction_residual);
//       debug->hs_score = hs_score;
//       debug->hd_score = hd_score;
//       debug->accum_s = unconfirmed_progress_accum_;
//       debug->accum_t = unconfirmed_time_accum_;
//       debug->confirmed_count = confirmed_trail_world_.size();
//       debug->tail_count = tail_buffer_world_.size();
//     }
//   }

//   Config cfg_;
//   LeadState2D track_;
//   std::deque<LeadNode> nodes_;
//   std::deque<HistoryPoint2D> confirmed_trail_world_;
//   std::deque<HistoryPoint2D> tail_buffer_world_;
//   int static_votes_ = 0;
//   int moving_votes_ = 0;
//   bool moving_flag_ = false;
//   std::string last_history_action_ = "init";
//   double static_anchor_x_ = 0.0;
//   double static_anchor_y_ = 0.0;
//   double last_innovation_norm_ = 0.0;
//   double last_innovation_metric_ = 0.0;
//   bool unconfirmed_accum_initialized_ = false;
//   double unconfirmed_progress_accum_ = 0.0;
//   double unconfirmed_time_accum_ = 0.0;
//   double unconfirmed_last_x_ = 0.0;
//   double unconfirmed_last_y_ = 0.0;
//   double unconfirmed_last_heading_ = 0.0;
//   ros::Time unconfirmed_last_stamp_;
//   ros::Time unconfirmed_start_stamp_;
// };

// class PathGenerator {
//  public:
//   struct Config {
//     double raw_resample_ds = 0.08;
//     double raw_fit_ds = 0.06;
//     int raw_spline_passes = 4;
//     double raw_spline_alpha = 0.45;
//     double raw_tail_weight_scale = 0.25;
//     double lead_center_to_path_ref_offset = 0.0;
//     int raw_min_points_for_publish = 2;
//     double raw_min_span_for_publish = 0.08;
//     double raw_keep_behind_x = 0.50;
//     double raw_max_range = 30.0;
//     int raw_max_points = 250;
//     double reference_join_min_forward_x = 0.60;
//     double reference_join_max_lateral = 3.0;
//     double reference_join_lookahead = 0.35;
//     double reference_join_skip_weight = 0.20;
//     double reference_join_curvature_weight = 0.90;
//     double reference_join_time_weight = 0.60;
//     double reference_join_progress_weight = 0.25;
//     double reference_join_s_window = 2.0;
//     int reference_min_points = 2;
//     double reference_min_span = 0.25;
//     double reference_direct_attach_dist = 0.18;
//     double reference_connector_enable_dist = 0.15;
//     double reference_connector_resolution = 0.08;
//     double reference_connector_tangent_scale = 0.65;
//     double reference_connector_curvature_scale = 1.0;
//     double reference_splice_blend_length = 0.60;
//     double reference_resample_ds = 0.08;
//     int reference_sg_passes = 1;
//     double reference_max_length = 20.0;
//   };

//   explicit PathGenerator(const Config& cfg) : cfg_(cfg) {}

//   std::vector<RawTrailPoint> buildRawTrail(const std::vector<HistoryPoint2D>& world_history,
//                                            TrackerDebugState* debug) const {
//     if (world_history.empty()) {
//       if (debug != NULL) {
//         debug->raw_reason = "history_empty";
//         debug->raw_span = 0.0;
//       }
//       return {};
//     }

//     std::vector<HistoryPoint2D> world_pts = removeNearDuplicatePoints(world_history, 0.01);
//     if (world_pts.size() >= 2) {
//       recomputeHeading(world_pts);
//       world_pts =
//           smoothWorldSplineLike(world_pts, cfg_.raw_spline_passes, cfg_.raw_spline_alpha, cfg_.raw_tail_weight_scale);
//       world_pts = fitWorldTrajectoryHermite(world_pts, cfg_.raw_fit_ds);
//       world_pts = resampleWorldByArcLength(world_pts, cfg_.raw_resample_ds);
//       recomputeHeading(world_pts);
//     }
//     offsetWorldPathAlongHeading(&world_pts, cfg_.lead_center_to_path_ref_offset);
//     const std::vector<RawTrailPoint> raw_trail = makeRawTrailPoints(world_pts);
//     if (debug != NULL) {
//       debug->raw_span = raw_trail.empty() ? 0.0 : raw_trail.back().s;
//       debug->raw_reason = raw_trail.empty() ? "history_empty" : "raw_ready";
//     }
//     return raw_trail;
//   }

//   std::vector<LocalPathPoint> buildRawPath(const std::vector<HistoryPoint2D>& world_history,
//                                            const EgoPose2D& ego_now,
//                                            TrackerDebugState* debug) const {
//     const std::vector<RawTrailPoint> raw_trail = buildRawTrail(world_history, debug);
//     if (raw_trail.empty()) {
//       return {};
//     }

//     std::vector<HistoryPoint2D> fitted_world;
//     fitted_world.reserve(raw_trail.size());
//     for (size_t i = 0; i < raw_trail.size(); ++i) {
//       HistoryPoint2D pt;
//       pt.stamp = raw_trail[i].stamp;
//       pt.x = raw_trail[i].x;
//       pt.y = raw_trail[i].y;
//       pt.heading = raw_trail[i].yaw;
//       pt.curvature = raw_trail[i].kappa;
//       fitted_world.push_back(pt);
//     }
//     std::vector<LocalPathPoint> local = transformWorldPathToLocal(fitted_world, ego_now);
//     std::vector<LocalPathPoint> trimmed;
//     trimmed.reserve(local.size());
//     for (size_t i = 0; i < local.size(); ++i) {
//       if (local[i].x < -cfg_.raw_keep_behind_x) {
//         continue;
//       }
//       if (hypot2(local[i].x, local[i].y) > cfg_.raw_max_range) {
//         continue;
//       }
//       trimmed.push_back(local[i]);
//       trimmed.back().curvature = raw_trail[i].kappa;
//       if (static_cast<int>(trimmed.size()) >= cfg_.raw_max_points) {
//         break;
//       }
//     }

//     if (trimmed.size() >= 2) {
//       recomputeLocalHeading(trimmed);
//     }
//     const double span = polylineLengthLocal(trimmed);
//     if (debug != NULL) {
//       debug->raw_span = span;
//     }
//     if (static_cast<int>(trimmed.size()) < cfg_.raw_min_points_for_publish) {
//       if (debug != NULL) {
//         debug->raw_reason = "raw_insufficient_points";
//       }
//       return {};
//     }
//     if (span < cfg_.raw_min_span_for_publish) {
//       if (debug != NULL) {
//         debug->raw_reason = "raw_span_too_short";
//       }
//       return {};
//     }
//     if (debug != NULL) {
//       debug->raw_reason = "raw_ready";
//     }
//     return trimmed;
//   }

//   std::vector<LocalPathPoint> buildReferencePath(const std::vector<RawTrailPoint>& raw_trail,
//                                                  const std::vector<LocalPathPoint>& raw,
//                                                  const EgoPose2D& ego_now,
//                                                  TrackerDebugState* debug) const {
//     if (raw.empty() || raw_trail.empty()) {
//       if (debug != NULL) {
//         debug->reference_reason = "raw_empty";
//         debug->reference_span = 0.0;
//       }
//       return {};
//     }
//     if (static_cast<int>(raw.size()) < cfg_.reference_min_points) {
//       if (debug != NULL) {
//         debug->reference_reason = "reference_insufficient_points";
//         debug->reference_span = polylineLengthLocal(raw);
//       }
//       return {};
//     }
//     const double raw_span = polylineLengthLocal(raw);
//     if (raw_span < cfg_.reference_min_span) {
//       if (debug != NULL) {
//         debug->reference_reason = "reference_span_too_short";
//         debug->reference_span = raw_span;
//       }
//       return {};
//     }

//     std::vector<RefPoint> ref_profile = makeRefPointsFromRawTrail(raw_trail, ego_now, 0.5);
//     recomputeRefGeometry(&ref_profile);
//     if (ref_profile.size() < static_cast<size_t>(cfg_.reference_min_points)) {
//       if (debug != NULL) {
//         debug->reference_reason = "reference_profile_too_short";
//         debug->reference_span = raw_span;
//       }
//       return {};
//     }

//     const JoinChoice join = chooseJoinIndex(ref_profile);
//     const RefPoint& join_ref = join.join_point;

//     std::vector<RefPoint> combined;
//     RefPoint origin;
//     origin.v_ref = std::min(0.3, join_ref.v_ref);
//     origin.a_ref = 0.0;
//     combined.push_back(origin);

//     size_t splice_begin_index = 0;
//     if (hypot2(join_ref.x, join_ref.y) > cfg_.reference_direct_attach_dist) {
//       const std::vector<RefPoint> connector = buildConnector(join_ref);
//       for (size_t i = 1; i < connector.size(); ++i) {
//         combined.push_back(connector[i]);
//       }
//       splice_begin_index = combined.empty() ? 0 : combined.size() - 1;
//     } else {
//       combined.push_back(join_ref);
//       splice_begin_index = combined.size() - 1;
//     }
//     for (size_t i = std::min(join.suffix_index, ref_profile.size()); i < ref_profile.size(); ++i) {
//       if (!combined.empty() &&
//           pointDistance(combined.back().x, combined.back().y, ref_profile[i].x, ref_profile[i].y) < 0.02) {
//         combined.back() = ref_profile[i];
//       } else {
//         combined.push_back(ref_profile[i]);
//       }
//     }

//     recomputeRefGeometry(&combined);
//     applyLocalSpliceBlend(&combined, splice_begin_index);

//     if (cfg_.reference_max_length > 0.0 && combined.size() >= 2) {
//       std::vector<RefPoint> clipped;
//       clipped.reserve(combined.size());
//       clipped.push_back(combined.front());
//       for (size_t i = 1; i < combined.size(); ++i) {
//         clipped.push_back(combined[i]);
//         if (combined[i].s >= cfg_.reference_max_length) {
//           break;
//         }
//       }
//       combined.swap(clipped);
//       recomputeRefGeometry(&combined);
//     }

//     std::vector<LocalPathPoint> final_local = makeLocalPathFromRefPoints(combined);
//     if (debug != NULL) {
//       debug->reference_reason = "reference_ready";
//       debug->reference_span = polylineLengthLocal(final_local);
//     }

//     return final_local;
//   }

//  private:
//   struct JoinChoice {
//     bool valid = false;
//     size_t suffix_index = 0;
//     RefPoint join_point;
//   };

//   static void evaluateQuinticBezier(const RefPoint control[6], double u, RefPoint* out) {
//     if (out == NULL) {
//       return;
//     }
//     const double one_minus_u = 1.0 - u;
//     const double b0 = std::pow(one_minus_u, 5);
//     const double b1 = 5.0 * u * std::pow(one_minus_u, 4);
//     const double b2 = 10.0 * u * u * std::pow(one_minus_u, 3);
//     const double b3 = 10.0 * u * u * u * std::pow(one_minus_u, 2);
//     const double b4 = 5.0 * u * u * u * u * one_minus_u;
//     const double b5 = std::pow(u, 5);

//     out->x = b0 * control[0].x + b1 * control[1].x + b2 * control[2].x +
//              b3 * control[3].x + b4 * control[4].x + b5 * control[5].x;
//     out->y = b0 * control[0].y + b1 * control[1].y + b2 * control[2].y +
//              b3 * control[3].y + b4 * control[4].y + b5 * control[5].y;

//     const double db0 = -5.0 * std::pow(one_minus_u, 4);
//     const double db1 = 5.0 * std::pow(one_minus_u, 4) - 20.0 * u * std::pow(one_minus_u, 3);
//     const double db2 = 20.0 * u * std::pow(one_minus_u, 3) - 30.0 * u * u * std::pow(one_minus_u, 2);
//     const double db3 = 30.0 * u * u * std::pow(one_minus_u, 2) - 20.0 * u * u * u * one_minus_u;
//     const double db4 = 20.0 * u * u * u * one_minus_u - 5.0 * std::pow(u, 4);
//     const double db5 = 5.0 * std::pow(u, 4);

//     const double dx = db0 * control[0].x + db1 * control[1].x + db2 * control[2].x +
//                       db3 * control[3].x + db4 * control[4].x + db5 * control[5].x;
//     const double dy = db0 * control[0].y + db1 * control[1].y + db2 * control[2].y +
//                       db3 * control[3].y + db4 * control[4].y + db5 * control[5].y;
//     out->yaw = std::atan2(dy, dx);
//     out->kappa = 0.0;
//     out->v_ref = 0.0;
//     out->a_ref = 0.0;
//   }

//   void applyLocalSpliceBlend(std::vector<RefPoint>* pts, size_t splice_index) const {
//     if (pts == NULL || pts->size() < 4 || splice_index == 0 || splice_index >= pts->size()) {
//       return;
//     }

//     recomputeRefGeometry(pts);
//     const double blend_length = std::max(0.10, cfg_.reference_splice_blend_length);
//     double accum = 0.0;
//     size_t end_index = splice_index;
//     while (end_index + 1 < pts->size() && accum < blend_length) {
//       accum += pointDistance((*pts)[end_index].x, (*pts)[end_index].y,
//                              (*pts)[end_index + 1].x, (*pts)[end_index + 1].y);
//       ++end_index;
//     }
//     if (end_index <= splice_index + 1) {
//       recomputeRefGeometry(pts);
//       return;
//     }

//     const RefPoint anchor = (*pts)[splice_index];
//     for (size_t i = splice_index + 1; i <= end_index; ++i) {
//       const double s = (*pts)[i].s - anchor.s;
//       const double ratio = clampValue(s / std::max(1e-3, accum), 0.0, 1.0);
//       const double w = ratio * ratio * (3.0 - 2.0 * ratio);
//       const double tx = std::cos(anchor.yaw);
//       const double ty = std::sin(anchor.yaw);
//       const double proj = ((*pts)[i].x - anchor.x) * tx + ((*pts)[i].y - anchor.y) * ty;
//       const double guided_x = anchor.x + proj * tx;
//       const double guided_y = anchor.y + proj * ty;
//       (*pts)[i].x = (1.0 - 0.35 * (1.0 - w)) * (*pts)[i].x + 0.35 * (1.0 - w) * guided_x;
//       (*pts)[i].y = (1.0 - 0.35 * (1.0 - w)) * (*pts)[i].y + 0.35 * (1.0 - w) * guided_y;
//       (*pts)[i].t = std::max((*pts)[i].t, anchor.t + s / std::max(0.10, anchor.v_ref));
//     }
//     recomputeRefGeometry(pts);
//   }

//   JoinChoice chooseJoinIndex(const std::vector<RefPoint>& ref_profile) const {
//     JoinChoice best;
//     if (ref_profile.empty()) {
//       return best;
//     }
//     if (ref_profile.size() == 1) {
//       best.valid = true;
//       best.suffix_index = 0;
//       best.join_point = ref_profile.front();
//       return best;
//     }

//     double s_proj = ref_profile.front().s;
//     double best_proj_dist = std::numeric_limits<double>::infinity();
//     for (size_t i = 0; i + 1 < ref_profile.size(); ++i) {
//       const RefPoint& a = ref_profile[i];
//       const RefPoint& b = ref_profile[i + 1];
//       const double dx = b.x - a.x;
//       const double dy = b.y - a.y;
//       const double seg_len2 = std::max(1e-6, dx * dx + dy * dy);
//       const double t = clampValue((-(a.x * dx + a.y * dy)) / seg_len2, 0.0, 1.0);
//       const double px = a.x + t * dx;
//       const double py = a.y + t * dy;
//       const double proj_dist = hypot2(px, py);
//       if (proj_dist < best_proj_dist) {
//         best_proj_dist = proj_dist;
//         s_proj = (1.0 - t) * a.s + t * b.s;
//       }
//     }

//     const double s_min = s_proj;
//     const double s_max = s_proj + std::max(0.5, cfg_.reference_join_s_window);
//     double best_cost = std::numeric_limits<double>::infinity();
//     for (size_t i = 0; i + 1 < ref_profile.size(); ++i) {
//       const RefPoint& a = ref_profile[i];
//       const RefPoint& b = ref_profile[i + 1];
//       const double dx = b.x - a.x;
//       const double dy = b.y - a.y;
//       const double seg_len2 = std::max(1e-6, dx * dx + dy * dy);
//       const double t = clampValue((-(a.x * dx + a.y * dy)) / seg_len2, 0.0, 1.0);

//       RefPoint cand;
//       cand.x = a.x + t * dx;
//       cand.y = a.y + t * dy;
//       cand.yaw = wrapAngle(a.yaw + t * wrapAngle(b.yaw - a.yaw));
//       cand.kappa = (1.0 - t) * a.kappa + t * b.kappa;
//       cand.s = (1.0 - t) * a.s + t * b.s;
//       cand.t = (1.0 - t) * a.t + t * b.t;
//       cand.v_ref = std::max(0.05, (1.0 - t) * a.v_ref + t * b.v_ref);
//       cand.a_ref = (1.0 - t) * a.a_ref + t * b.a_ref;

//       if (cand.s < s_min || cand.s > s_max) {
//         continue;
//       }

//       if (cand.x < cfg_.reference_join_min_forward_x) {
//         continue;
//       }
//       if (std::fabs(cand.y) > cfg_.reference_join_max_lateral) {
//         continue;
//       }

//       const double dist = hypot2(cand.x, cand.y);
//       const double yaw_err = std::fabs(wrapAngle(cand.yaw));
//       const double eta = cand.t;
//       const double tangent_align = std::fabs(std::sin(cand.yaw));
//       const double connector_len = dist;
//       const double cost = connector_len +
//                           0.50 * std::fabs(cand.y) +
//                           0.35 * yaw_err +
//                           0.25 * tangent_align +
//                           cfg_.reference_join_curvature_weight * std::fabs(cand.kappa) +
//                           cfg_.reference_join_skip_weight * std::max(0.0, cand.s - s_proj) +
//                           cfg_.reference_join_time_weight * std::min(eta, 6.0) +
//                           cfg_.reference_join_progress_weight *
//                               std::max(0.0, cfg_.reference_join_min_forward_x - cand.x);
//       if (cost < best_cost) {
//         best_cost = cost;
//         best.valid = true;
//         best.suffix_index = i + 1;
//         best.join_point = cand;
//       }
//     }

//     if (!best.valid) {
//       best.valid = true;
//       best.suffix_index = 0;
//       best.join_point = ref_profile.front();
//     }

//     double lookahead = 0.0;
//     while (best.suffix_index < ref_profile.size() &&
//            best.suffix_index + 1 < ref_profile.size() &&
//            lookahead < cfg_.reference_join_lookahead) {
//       lookahead += pointDistance(ref_profile[best.suffix_index].x,
//                                  ref_profile[best.suffix_index].y,
//                                  ref_profile[best.suffix_index + 1].x,
//                                  ref_profile[best.suffix_index + 1].y);
//       ++best.suffix_index;
//       best.join_point = ref_profile[best.suffix_index];
//     }
//     return best;
//   }

//   std::vector<RefPoint> buildConnector(const RefPoint& goal) const {
//     std::vector<RefPoint> out;
//     const double dist = hypot2(goal.x, goal.y);
//     if (dist < cfg_.reference_connector_enable_dist) {
//       return out;
//     }

//     const double tangent0 = std::min(cfg_.reference_connector_tangent_scale * dist, 2.5);
//     const double tangent1 = std::min(cfg_.reference_connector_tangent_scale * dist, 2.5);
//     const double kappa0 = 0.0;
//     const double kappa1 = clampValue(goal.kappa, -0.6, 0.6) * cfg_.reference_connector_curvature_scale;
//     const double t0x = 1.0;
//     const double t0y = 0.0;
//     const double n0x = 0.0;
//     const double n0y = 1.0;
//     const double t1x = std::cos(goal.yaw);
//     const double t1y = std::sin(goal.yaw);
//     const double n1x = -std::sin(goal.yaw);
//     const double n1y = std::cos(goal.yaw);

//     RefPoint control[6];
//     control[0].x = 0.0;
//     control[0].y = 0.0;
//     control[0].yaw = 0.0;
//     control[0].kappa = kappa0;
//     control[0].t = 0.0;
//     control[0].v_ref = std::max(0.10, goal.v_ref * 0.6);

//     control[5] = goal;
//     control[1].x = control[0].x + tangent0 * t0x / 5.0;
//     control[1].y = control[0].y + tangent0 * t0y / 5.0;
//     control[2].x = 2.0 * control[1].x - control[0].x + (tangent0 * tangent0 * kappa0 / 20.0) * n0x;
//     control[2].y = 2.0 * control[1].y - control[0].y + (tangent0 * tangent0 * kappa0 / 20.0) * n0y;

//     control[4].x = control[5].x - tangent1 * t1x / 5.0;
//     control[4].y = control[5].y - tangent1 * t1y / 5.0;
//     control[3].x = 2.0 * control[4].x - control[5].x + (tangent1 * tangent1 * kappa1 / 20.0) * n1x;
//     control[3].y = 2.0 * control[4].y - control[5].y + (tangent1 * tangent1 * kappa1 / 20.0) * n1y;

//     const int steps =
//         std::max(3, static_cast<int>(std::ceil(dist / std::max(0.03, cfg_.reference_connector_resolution))));
//     out.reserve(static_cast<size_t>(steps) + 1);

//     for (int i = 0; i < steps; ++i) {
//       RefPoint pt;
//       const double t = static_cast<double>(i) / static_cast<double>(steps);
//       evaluateQuinticBezier(control, t, &pt);
//       pt.kappa = (1.0 - t) * kappa0 + t * kappa1;
//       pt.t = t * goal.t;
//       pt.v_ref = std::max(0.10, goal.v_ref * (0.45 + 0.55 * t));
//       pt.a_ref = 0.0;
//       out.push_back(pt);
//     }

//     out.push_back(goal);
//     recomputeRefGeometry(&out);
//     return out;
//   }

//   Config cfg_;
// };

// class FixedFrameTrajectoryReconstructor {
//  public:
//   FixedFrameTrajectoryReconstructor()
//       : nh_(),
//         pnh_("~"),
//         ego_motion_(loadEgoMotionConfig()),
//         projector_(loadProjectorConfig()),
//         tracker_(loadTrackerConfig()),
//         path_generator_(loadPathConfig()) {
//     base_frame_ = pnh_.param<std::string>("base_frame", "base_link");
//     odom_frame_ = pnh_.param<std::string>("odom_frame", "reconstruction_odom");
//     tracked_topic_ = pnh_.param<std::string>("tracked_topic", "/pointpillars/tracked_objects");
//     car_topic_ = pnh_.param<std::string>("car_topic", "/car_message");
//     carvel_topic_ = pnh_.param<std::string>("carvel_topic", "/carvel");
//     raw_topic_ = pnh_.param<std::string>("raw_topic", "/planner/raw_path");
//     reference_topic_ = pnh_.param<std::string>("reference_topic", "/planner/reference_path");
//     publish_rate_hz_ = pnh_.param("publish_rate_hz", 30.0);

//     wheel_speed_scale_ = pnh_.param("wheel_speed_scale", 0.001);
//     speed_hint_scale_ = pnh_.param("speed_hint_scale", 1.0);
//     steer_angle_scale_ = pnh_.param("steer_angle_scale", M_PI / 180.0);
//     steer_angle_bias_ = pnh_.param("steer_angle_bias", 0.0);
//     steer_sign_ = pnh_.param("steer_sign", -1.0);
//     steer_deadband_rad_ = pnh_.param("steer_deadband_rad", 0.002);
//     lost_timeout_ = pnh_.param("lost_timeout", 0.60);
//     min_motion_speed_for_yaw_ = pnh_.param("min_motion_speed_for_yaw", 0.15);
//     allow_unstamped_tracked_fallback_ = pnh_.param("allow_unstamped_tracked_fallback", true);

//     tracked_sub_ = nh_.subscribe(tracked_topic_, 5, &FixedFrameTrajectoryReconstructor::trackedCallback, this);
//     car_sub_ = nh_.subscribe(car_topic_, 20, &FixedFrameTrajectoryReconstructor::carCallback, this);
//     carvel_sub_ = nh_.subscribe(carvel_topic_, 20, &FixedFrameTrajectoryReconstructor::carvelCallback, this);
//     raw_pub_ = nh_.advertise<nav_msgs::Path>(raw_topic_, 1);
//     reference_pub_ = nh_.advertise<nav_msgs::Path>(reference_topic_, 1);
//     timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
//                              &FixedFrameTrajectoryReconstructor::timerCallback,
//                              this);

//     ROS_INFO("trajectory_reconstructor started | tracked=%s car=%s carvel=%s raw=%s ref=%s base=%s odom=%s",
//              tracked_topic_.c_str(),
//              car_topic_.c_str(),
//              carvel_topic_.c_str(),
//              raw_topic_.c_str(),
//              reference_topic_.c_str(),
//              base_frame_.c_str(),
//              odom_frame_.c_str());
//   }

//  private:
//   EgoMotionEstimator::Config loadEgoMotionConfig() const {
//     EgoMotionEstimator::Config cfg;
//     cfg.wheelbase = pnh_.param("wheelbase", 2.60);
//     cfg.min_motion_speed_for_yaw = pnh_.param("min_motion_speed_for_yaw", 0.15);
//     cfg.max_abs_yaw_rate = pnh_.param("max_abs_yaw_rate", 0.80);
//     cfg.max_integration_dt = pnh_.param("max_integration_dt", 0.02);
//     cfg.pose_buffer_duration = pnh_.param("pose_buffer_duration", 5.0);
//     cfg.steer_lpf_alpha = clampValue(pnh_.param("steer_lpf_alpha", 0.25), 0.0, 1.0);
//     cfg.yaw_sigma_base = pnh_.param("yaw_sigma_base", 0.01);
//     cfg.yaw_sigma_speed_gain = pnh_.param("yaw_sigma_speed_gain", 0.02);
//     cfg.yaw_sigma_steer_gain = pnh_.param("yaw_sigma_steer_gain", 0.08);
//     cfg.use_speed_hint = pnh_.param("use_speed_hint", false);
//     cfg.speed_hint_weight = clampValue(pnh_.param("speed_hint_weight", 0.20), 0.0, 1.0);
//     cfg.speed_hint_timeout = pnh_.param("speed_hint_timeout", 0.30);
//     cfg.speed_hint_agreement_gate = pnh_.param("speed_hint_agreement_gate", 0.40);
//     return cfg;
//   }

//   LeadObservationProjector::Config loadProjectorConfig() const {
//     LeadObservationProjector::Config cfg;
//     cfg.target_min_forward_x = pnh_.param("target_min_forward_x", 0.30);
//     cfg.target_lateral_gate = pnh_.param("target_lateral_gate", 4.0);
//     cfg.target_lock_dist_gate = pnh_.param("target_lock_dist_gate", 1.20);
//     cfg.target_lock_yaw_gate = pnh_.param("target_lock_yaw_gate", 1.10);
//     return cfg;
//   }

//   LeadTrailTracker::Config loadTrackerConfig() const {
//     LeadTrailTracker::Config cfg;
//     cfg.lost_timeout = pnh_.param("lost_timeout", 0.60);
//     cfg.heading_blend_alpha = clampValue(pnh_.param("heading_blend_alpha", 0.20), 0.0, 1.0);
//     cfg.heading_from_velocity_speed_gate = pnh_.param("heading_from_velocity_speed_gate", 0.25);
//     cfg.innovation_base_gate = pnh_.param("innovation_base_gate", 0.10);
//     cfg.innovation_range_gain = pnh_.param("innovation_range_gain", 0.015);
//     cfg.innovation_turn_gain = pnh_.param("innovation_turn_gain", 0.80);
//     cfg.outlier_reject_scale = pnh_.param("outlier_reject_scale", 3.0);
//     cfg.lead_static_speed_gate = pnh_.param("lead_static_speed_gate", 0.08);
//     cfg.lead_moving_speed_gate = pnh_.param("lead_moving_speed_gate", 0.12);
//     cfg.static_residual_gate = pnh_.param("static_residual_gate", 0.12);
//     cfg.moving_residual_gate = pnh_.param("moving_residual_gate", 0.30);
//     cfg.static_vote_on = pnh_.param("static_vote_on", 12);
//     cfg.static_vote_release = pnh_.param("static_vote_release", 3);
//     cfg.moving_vote_on = pnh_.param("moving_vote_on", 2);
//     cfg.anchor_alpha = clampValue(pnh_.param("anchor_alpha", 0.08), 0.0, 1.0);
//     cfg.static_preserve_trail_span = pnh_.param("static_preserve_trail_span", 0.80);
//     cfg.history_max_age = pnh_.param("history_max_age", 20.0);
//     cfg.history_max_points = pnh_.param("history_max_points", 300);
//     cfg.smoother_position_gain = clampValue(pnh_.param("smoother_position_gain", 0.35), 0.0, 1.0);
//     cfg.smoother_cross_gain = clampValue(pnh_.param("smoother_cross_gain", 0.18), 0.0, 1.0);
//     cfg.smoother_temporal_alpha = clampValue(pnh_.param("smoother_temporal_alpha", 0.22), 0.0, 1.0);
//     cfg.smoother_heading_alpha = clampValue(pnh_.param("smoother_heading_alpha", 0.20), 0.0, 1.0);
//     cfg.smoother_curvature_alpha =
//         clampValue(pnh_.param("smoother_curvature_alpha", 0.18), 0.0, 1.0);
//     cfg.measurement_heading_alpha =
//         clampValue(pnh_.param("measurement_heading_alpha", 0.12), 0.0, 1.0);
//     cfg.dynamics_position_alpha =
//         clampValue(pnh_.param("dynamics_position_alpha", 0.12), 0.0, 1.0);
//     cfg.dynamics_heading_alpha =
//         clampValue(pnh_.param("dynamics_heading_alpha", 0.16), 0.0, 1.0);
//     cfg.acceleration_regularization_alpha =
//         clampValue(pnh_.param("acceleration_regularization_alpha", 0.16), 0.0, 1.0);
//     cfg.curvature_prediction_alpha =
//         clampValue(pnh_.param("curvature_prediction_alpha", 0.18), 0.0, 1.0);
//     cfg.dkappa_regularization_alpha =
//         clampValue(pnh_.param("dkappa_regularization_alpha", 0.20), 0.0, 1.0);
//     cfg.robust_cauchy_scale = pnh_.param("robust_cauchy_scale", 2.5);
//     cfg.smoother_iterations = pnh_.param("smoother_iterations", 5);
//     cfg.measurement_along_sigma = pnh_.param("measurement_along_sigma", 0.08);
//     cfg.measurement_cross_sigma = pnh_.param("measurement_cross_sigma", 0.12);
//     cfg.measurement_cross_range_gain = pnh_.param("measurement_cross_range_gain", 0.020);
//     cfg.measurement_cross_yaw_uncertainty_gain =
//         pnh_.param("measurement_cross_yaw_uncertainty_gain", 1.80);
//     cfg.temporal_min_progress = pnh_.param("temporal_min_progress", 0.02);
//     cfg.trail_append_min_distance = pnh_.param("trail_append_min_distance", 0.12);
//     cfg.trail_anchor_update_alpha =
//         clampValue(pnh_.param("trail_anchor_update_alpha", 0.10), 0.0, 1.0);
//     cfg.tail_confirm_min_points = pnh_.param("tail_confirm_min_points", 3);
//     cfg.tail_confirm_min_distance = pnh_.param("tail_confirm_min_distance", 0.06);
//     cfg.tail_confirm_min_age = pnh_.param("tail_confirm_min_age", 0.18);
//     cfg.confirm_dynamic_score_margin = pnh_.param("confirm_dynamic_score_margin", 0.12);
//     cfg.confirm_static_residual_gate = pnh_.param("confirm_static_residual_gate", 0.14);
//     cfg.confirm_min_progress = pnh_.param("confirm_min_progress", 0.08);
//     cfg.confirm_accum_distance = pnh_.param("confirm_accum_distance", 0.05);
//     cfg.confirm_accum_timeout = pnh_.param("confirm_accum_timeout", 0.80);
//     cfg.ego_static_speed_gate = pnh_.param("ego_static_speed_gate", 0.05);
//     cfg.static_release_score_margin = pnh_.param("static_release_score_margin", 0.06);
//     cfg.raw_tail_keep_points = pnh_.param("raw_tail_keep_points", 6);
//     cfg.dual_hypothesis_score_margin = pnh_.param("dual_hypothesis_score_margin", 0.02);
//     return cfg;
//   }

//   PathGenerator::Config loadPathConfig() const {
//     PathGenerator::Config cfg;
//     cfg.raw_resample_ds = pnh_.param("raw_resample_ds", 0.08);
//     cfg.raw_fit_ds = pnh_.param("raw_fit_ds", 0.06);
//     cfg.raw_spline_passes = pnh_.param("raw_spline_passes", 4);
//     cfg.raw_spline_alpha = clampValue(pnh_.param("raw_spline_alpha", 0.45), 0.0, 1.0);
//     cfg.raw_tail_weight_scale = clampValue(pnh_.param("raw_tail_weight_scale", 0.25), 0.05, 1.0);
//     cfg.lead_center_to_path_ref_offset = pnh_.param("lead_center_to_path_ref_offset", 0.0);
//     cfg.raw_min_points_for_publish = pnh_.param("raw_min_points_for_publish", 2);
//     cfg.raw_min_span_for_publish = pnh_.param("raw_min_span_for_publish", 0.08);
//     cfg.raw_keep_behind_x = pnh_.param("raw_keep_behind_x", 0.50);
//     cfg.raw_max_range = pnh_.param("raw_max_range", 30.0);
//     cfg.raw_max_points = pnh_.param("raw_max_points", 250);
//     cfg.reference_join_min_forward_x = pnh_.param("reference_join_min_forward_x", 0.60);
//     cfg.reference_join_max_lateral = pnh_.param("reference_join_max_lateral", 3.0);
//     cfg.reference_join_lookahead = pnh_.param("reference_join_lookahead", 0.35);
//     cfg.reference_join_skip_weight = pnh_.param("reference_join_skip_weight", 0.20);
//     cfg.reference_join_curvature_weight = pnh_.param("reference_join_curvature_weight", 0.90);
//     cfg.reference_join_time_weight = pnh_.param("reference_join_time_weight", 0.60);
//     cfg.reference_join_progress_weight = pnh_.param("reference_join_progress_weight", 0.25);
//     cfg.reference_join_s_window = pnh_.param("reference_join_s_window", 2.0);
//     cfg.reference_min_points = pnh_.param("reference_min_points", 2);
//     cfg.reference_min_span = pnh_.param("reference_min_span", 0.25);
//     cfg.reference_direct_attach_dist = pnh_.param("reference_direct_attach_dist", 0.18);
//     cfg.reference_connector_enable_dist = pnh_.param("reference_connector_enable_dist", 0.15);
//     cfg.reference_connector_resolution = pnh_.param("reference_connector_resolution", 0.08);
//     cfg.reference_connector_tangent_scale = pnh_.param("reference_connector_tangent_scale", 0.65);
//     cfg.reference_connector_curvature_scale = pnh_.param("reference_connector_curvature_scale", 1.0);
//     cfg.reference_splice_blend_length = pnh_.param("reference_splice_blend_length", 0.60);
//     cfg.reference_resample_ds = pnh_.param("reference_resample_ds", 0.08);
//     cfg.reference_sg_passes = pnh_.param("reference_sg_passes", 1);
//     cfg.reference_max_length = pnh_.param("reference_max_length", 20.0);
//     return cfg;
//   }

//   ros::Time resolveStampedPoseArrayTime(const geometry_msgs::PoseArray& msg) const {
//     if (msg.header.stamp.isValid()) {
//       return msg.header.stamp;
//     }

//     if (allow_unstamped_tracked_fallback_) {
//       ROS_WARN_THROTTLE(1.0,
//                         "tracked_topic PoseArray has no valid header.stamp; falling back to "
//                         "receipt time. Fixed-frame reconstruction will be less accurate.");
//       return ros::Time::now();
//     }

//     ROS_WARN_THROTTLE(1.0,
//                       "tracked_topic PoseArray has no valid header.stamp; dropping frame because "
//                       "fixed-frame reconstruction requires measurement-time ego pose.");
//     return ros::Time();
//   }

//   void carCallback(const move_car::car_parameter::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(mutex_);
//     // car_parameter has no header, so arrival time is the only available fallback.
//     const ros::Time stamp = ros::Time::now();
//     const double wheel_speed_mps = static_cast<double>(msg->back_wheel_speed) * wheel_speed_scale_;
//     double steer_angle_rad =
//         steer_sign_ * (static_cast<double>(msg->turn_angle) * steer_angle_scale_ - steer_angle_bias_);
//     if (std::fabs(steer_angle_rad) < steer_deadband_rad_) {
//       steer_angle_rad = 0.0;
//     }
//     ego_motion_.updateWheelState(stamp, wheel_speed_mps, steer_angle_rad);
//   }

//   void carvelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(mutex_);
//     // geometry_msgs/Twist has no header, so arrival time is the only available fallback.
//     const ros::Time stamp = ros::Time::now();
//     const double speed_hint_mps = static_cast<double>(msg->linear.x) * speed_hint_scale_;
//     ego_motion_.updateSpeedHint(stamp, speed_hint_mps);
//   }

//   void trackedCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(mutex_);
//     const ros::Time obs_stamp = resolveStampedPoseArrayTime(*msg);
//     if (!obs_stamp.isValid()) {
//       debug_state_.input_pose_count = static_cast<int>(msg->poses.size());
//       debug_state_.projector_reason = "obs_stamp_invalid";
//       return;
//     }

//     ego_motion_.integrateTo(obs_stamp);
//     tracker_.markLostIfExpired(obs_stamp);
//     tracker_.pruneHistory(obs_stamp);

//     LeadObservation obs;
//     EgoPose2D ego_at_obs;
//     const LeadState2D* track_ptr = tracker_.hasActiveTrack() ? &tracker_.track() : NULL;
//     if (!projector_.project(*msg, ego_motion_, track_ptr, &obs, &ego_at_obs, &debug_state_)) {
//       ++obs_version_;
//       debug_state_.history_size = tracker_.worldHistoryForPublish(obs_stamp).size();
//       return;
//     }

//     tracker_.ingestObservation(obs,
//                                ego_at_obs,
//                                ego_motion_.currentYawRate(),
//                                ego_motion_.yawSigmaEstimate(),
//                                ego_motion_.steerAngle(),
//                                &debug_state_);
//     tracker_.pruneHistory(obs_stamp);
//     debug_state_.history_size = tracker_.worldHistoryForPublish(obs_stamp).size();
//     ++obs_version_;
//   }

//   void timerCallback(const ros::TimerEvent&) {
//     const ros::Time now = ros::Time::now();

//     std::vector<LocalPathPoint> raw_pts;
//     std::vector<LocalPathPoint> ref_pts;
//     bool lock_active = false;
//     bool lead_moving = false;
//     bool ego_standstill = false;
//     double lead_x = 0.0;
//     double lead_y = 0.0;
//     double speed = 0.0;
//     double yaw_rate = 0.0;
//     double wheel_speed = 0.0;
//     double speed_hint = 0.0;
//     double steer = 0.0;
//     double innovation = 0.0;
//     double metric = 0.0;
//     uint64_t obs_ver = 0;
//     TrackerDebugState debug_snapshot;

//     {
//       std::lock_guard<std::mutex> lock(mutex_);
//       ego_motion_.integrateTo(now);
//       tracker_.markLostIfExpired(now);
//       tracker_.pruneHistory(now);

//       EgoPose2D ego_now;
//       if (ego_motion_.queryPoseAt(now, &ego_now)) {
//         const std::vector<HistoryPoint2D> world_history = tracker_.worldHistoryForPublish(now);
//         debug_state_.history_size = world_history.size();
//         const std::vector<RawTrailPoint> raw_trail =
//             path_generator_.buildRawTrail(world_history, &debug_state_);
//         raw_pts = path_generator_.buildRawPath(world_history, ego_now, &debug_state_);
//         ref_pts = path_generator_.buildReferencePath(raw_trail, raw_pts, ego_now, &debug_state_);

//         if (tracker_.hasActiveTrack()) {
//           const LeadState2D pred = predictLeadState(tracker_.track(), now);
//           transformWorldToLocal(ego_now, pred.x, pred.y, &lead_x, &lead_y);
//         }
//       }

//       lock_active = tracker_.hasActiveTrack();
//       lead_moving = tracker_.movingFlag();
//       speed = ego_motion_.fusedSpeed();
//       yaw_rate = ego_motion_.currentYawRate();
//       wheel_speed = ego_motion_.wheelSpeed();
//       speed_hint = ego_motion_.speedHint();
//       steer = ego_motion_.steerAngle();
//       innovation = tracker_.innovationNorm();
//       metric = tracker_.innovationMetric();
//       obs_ver = obs_version_;
//       ego_standstill = std::fabs(speed) < min_motion_speed_for_yaw_ * 0.9;
//       debug_snapshot = debug_state_;
//     }

//     raw_pub_.publish(makePathMsg(raw_pts, base_frame_, now));
//     reference_pub_.publish(makePathMsg(ref_pts, base_frame_, now));

//     ROS_INFO_THROTTLE(
//         1.0,
//         "traj_ff | lock=%d moving=%d standstill=%d raw=%zu ref=%zu x=%.2f y=%.2f v=%.3f yaw=%.3f wheel=%.3f hint=%.3f steer=%.3f innov=%.3f metric=%.3f ff_speed=%.3f ego_v=%.3f static_hyp_res=%.3f hs=%.3f hd=%.3f acc_s=%.3f acc_t=%.3f conf=%zu tail=%zu obs_ver=%llu poses=%d cand=%d lockrej=%d hist=%zu raw_span=%.2f ref_span=%.2f proj=%s histr=%s rawr=%s refr=%s",
//         lock_active ? 1 : 0,
//         lead_moving ? 1 : 0,
//         ego_standstill ? 1 : 0,
//         raw_pts.size(),
//         ref_pts.size(),
//         lead_x,
//         lead_y,
//         speed,
//         yaw_rate,
//         wheel_speed,
//         speed_hint,
//         steer,
//         innovation,
//         metric,
//         debug_snapshot.ff_speed,
//         debug_snapshot.mean_ego_speed,
//         debug_snapshot.static_hyp_res,
//         debug_snapshot.hs_score,
//         debug_snapshot.hd_score,
//         debug_snapshot.accum_s,
//         debug_snapshot.accum_t,
//         debug_snapshot.confirmed_count,
//         debug_snapshot.tail_count,
//         static_cast<unsigned long long>(obs_ver),
//         debug_snapshot.input_pose_count,
//         debug_snapshot.valid_candidate_count,
//         debug_snapshot.lock_reject_count,
//         debug_snapshot.history_size,
//         debug_snapshot.raw_span,
//         debug_snapshot.reference_span,
//         debug_snapshot.projector_reason.c_str(),
//         debug_snapshot.history_reason.c_str(),
//         debug_snapshot.raw_reason.c_str(),
//         debug_snapshot.reference_reason.c_str());
//   }

//   ros::NodeHandle nh_;
//   ros::NodeHandle pnh_;
//   ros::Subscriber tracked_sub_;
//   ros::Subscriber car_sub_;
//   ros::Subscriber carvel_sub_;
//   ros::Publisher raw_pub_;
//   ros::Publisher reference_pub_;
//   ros::Timer timer_;
//   std::mutex mutex_;

//   std::string base_frame_;
//   std::string odom_frame_;
//   std::string tracked_topic_;
//   std::string car_topic_;
//   std::string carvel_topic_;
//   std::string raw_topic_;
//   std::string reference_topic_;

//   double publish_rate_hz_ = 30.0;
//   double wheel_speed_scale_ = 0.001;
//   double speed_hint_scale_ = 1.0;
//   double steer_angle_scale_ = M_PI / 180.0;
//   double steer_angle_bias_ = 0.0;
//   double steer_sign_ = -1.0;
//   double steer_deadband_rad_ = 0.002;
//   double lost_timeout_ = 0.60;
//   double min_motion_speed_for_yaw_ = 0.15;
//   bool allow_unstamped_tracked_fallback_ = true;

//   EgoMotionEstimator ego_motion_;
//   LeadObservationProjector projector_;
//   LeadTrailTracker tracker_;
//   PathGenerator path_generator_;
//   TrackerDebugState debug_state_;
//   uint64_t obs_version_ = 0;
// };

// }  // namespace

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "trajectory_reconstructor");
//   FixedFrameTrajectoryReconstructor node;
//   ros::spin();
//   return 0;
// }









#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <move_car/car_parameter.h>
#include <nav_msgs/Path.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

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
  double ff_speed = 0.0;
  double mean_ego_speed = 0.0;
  double static_hyp_res = 0.0;
  double hs_score = 0.0;
  double hd_score = 0.0;
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
    pt.predicted = in[seg].predicted || in[seg + 1].predicted;
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
    double speed_hint_weight = 0.20;
    double speed_hint_timeout = 0.30;
    double speed_hint_agreement_gate = 0.40;
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
    if (std::fabs(fused_speed_mps_) < cfg_.min_motion_speed_for_yaw) {
      return 0.0;
    }
    const double yaw_rate =
        fused_speed_mps_ * std::tan(filtered_steer_angle_rad_) / std::max(0.5, cfg_.wheelbase);
    return clampValue(yaw_rate, -cfg_.max_abs_yaw_rate, cfg_.max_abs_yaw_rate);
  }

  double yawSigmaEstimate() const {
    return cfg_.yaw_sigma_base + cfg_.yaw_sigma_speed_gain * std::fabs(currentYawRate()) +
           cfg_.yaw_sigma_steer_gain * std::fabs(filtered_steer_angle_rad_);
  }

  double fusedSpeed() const { return fused_speed_mps_; }
  double wheelSpeed() const { return wheel_speed_mps_; }
  double speedHint() const { return speed_hint_mps_; }
  double steerAngle() const { return filtered_steer_angle_rad_; }
  bool initialized() const { return initialized_; }

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
  double wheel_speed_mps_ = 0.0;
  double speed_hint_mps_ = 0.0;
  double fused_speed_mps_ = 0.0;
  bool steer_initialized_ = false;
  double filtered_steer_angle_rad_ = 0.0;
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
    double measurement_along_sigma = 0.08;
    double measurement_cross_sigma = 0.12;
    double measurement_cross_range_gain = 0.020;
    double measurement_cross_yaw_uncertainty_gain = 1.80;
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
  };

  explicit LeadTrailTracker(const Config& cfg) : cfg_(cfg) {}

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
    node.heading_hint = obs.world_heading;
    node.has_measurement = true;
    node.z_rel_x = obs.local_x;
    node.z_rel_y = obs.local_y;
    node.range = obs.range;
    node.w_cross =
        cfg_.measurement_cross_sigma + cfg_.measurement_cross_range_gain * obs.range +
        cfg_.measurement_cross_yaw_uncertainty_gain * obs.range * ego_yaw_sigma +
        0.10 * std::fabs(steer_angle_rad);
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
    track_ = LeadState2D();
    moving_flag_ = false;
    static_votes_ = 0;
    moving_votes_ = 0;
    resetUnconfirmedAccumulatorUnlocked();
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
    const double adaptive_min_progress =
        cfg_.temporal_min_progress * (1.0 - 0.9 * static_confidence) * (moving_flag_ ? 1.0 : 0.6);
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
    return pt;
  }

  void refreshTailBufferUnlocked() {
    tail_buffer_world_.clear();
    tail_buffer_world_.resize(nodes_.size());
    for (size_t i = 0; i < nodes_.size(); ++i) {
      tail_buffer_world_[i] = nodeToHistoryPointUnlocked(nodes_[i]);
    }
    if (track_.static_mode && !tail_buffer_world_.empty()) {
      tail_buffer_world_.back().x = static_anchor_x_;
      tail_buffer_world_.back().y = static_anchor_y_;
      tail_buffer_world_.back().heading = track_.heading;
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

  void pruneTrailContainerUnlocked(std::deque<HistoryPoint2D>* trail, const ros::Time& now) {
    if (trail == NULL) {
      return;
    }
    while (!trail->empty() && (now - trail->front().stamp).toSec() > cfg_.history_max_age) {
      trail->pop_front();
    }
    while (trail->size() > static_cast<size_t>(std::max(2, cfg_.history_max_points))) {
      trail->pop_front();
    }
  }

  void commitConfirmedTrailPointUnlocked() {
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
    const double min_confirm_age =
        (ego_speed < cfg_.ego_static_speed_gate) ? 0.05 : cfg_.tail_confirm_min_age;
    const double min_confirm_progress =
        (ego_speed < cfg_.ego_static_speed_gate) ? 0.02 : cfg_.confirm_min_progress;

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
    updateUnconfirmedAccumulatorUnlocked(candidate);
    const double candidate_static_res =
        residualAgainstWorldPointUnlocked(nodes_.back().ego, candidate.x, candidate.y,
                                          nodes_.back().z_rel_x, nodes_.back().z_rel_y);

    if (dynamicTrailProgressUnlocked() < min_confirm_progress) {
      last_history_action_ = "history_progress_too_small";
      return;
    }

    const double accum_distance_gate = std::min(cfg_.tail_confirm_min_distance, cfg_.confirm_accum_distance);
    const double accum_timeout_gate = cfg_.confirm_accum_timeout;
    const bool accum_ready =
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
        (unconfirmed_progress_accum_ >= cfg_.confirm_accum_distance ||
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
      if (last_history_action_ != "history_dynamic_accum_override") {
        last_history_action_ = "history_appended";
      }
      resetUnconfirmedAccumulatorUnlocked();
    } else {
      last_history_action_ = "history_lowpass_update";
    }

    pruneTrailContainerUnlocked(&confirmed_trail_world_, candidate.stamp);
  }

  std::vector<HistoryPoint2D> exportHistoryUnlocked() const {
    std::vector<HistoryPoint2D> history;
    history.reserve(confirmed_trail_world_.size() + tail_buffer_world_.size());
    for (size_t i = 0; i < confirmed_trail_world_.size(); ++i) {
      HistoryPoint2D pt = confirmed_trail_world_[i];
      pt.predicted = false;
      history.push_back(pt);
    }

    if (tail_buffer_world_.empty()) {
      return removeNearDuplicatePoints(history, 0.01);
    }

    if (confirmed_trail_world_.empty()) {
      return removeNearDuplicatePoints(history, 0.01);
    }

    const bool allow_tail_suffix = dynamicHypothesisWinsUnlocked() || egoMotionConditionedDynamicUnlocked();
    if (!allow_tail_suffix) {
      return removeNearDuplicatePoints(history, 0.01);
    }

    const size_t keep_tail =
        std::min(tail_buffer_world_.size(), static_cast<size_t>(std::max(0, cfg_.raw_tail_keep_points)));
    const size_t start_idx = tail_buffer_world_.size() - keep_tail;
    const HistoryPoint2D& confirmed_tail = confirmed_trail_world_.back();
    for (size_t i = start_idx; i < tail_buffer_world_.size(); ++i) {
      if (tail_buffer_world_[i].predicted) {
        continue;
      }
      const double dist_from_confirmed =
          pointDistance(confirmed_tail.x, confirmed_tail.y, tail_buffer_world_[i].x, tail_buffer_world_[i].y);
      if (dist_from_confirmed > std::max(0.25, 3.0 * cfg_.tail_confirm_min_distance)) {
        continue;
      }
      if (!history.empty() &&
          pointDistance(history.back().x, history.back().y, tail_buffer_world_[i].x, tail_buffer_world_[i].y) <
              0.02) {
        HistoryPoint2D pt = tail_buffer_world_[i];
        pt.predicted = true;
        history.back() = pt;
      } else {
        HistoryPoint2D pt = tail_buffer_world_[i];
        pt.predicted = true;
        history.push_back(pt);
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

    if (!track_.static_mode && static_votes_ >= cfg_.static_vote_on) {
      track_.static_mode = true;
      static_anchor_x_ = track_.x;
      static_anchor_y_ = track_.y;
      moving_flag_ = false;
    }

    if (track_.static_mode) {
      const bool strong_static = hs_score + cfg_.static_release_score_margin < hd_score;
      if (strong_static) {
        static_anchor_x_ = (1.0 - cfg_.anchor_alpha) * static_anchor_x_ + cfg_.anchor_alpha * track_.x;
        static_anchor_y_ = (1.0 - cfg_.anchor_alpha) * static_anchor_y_ + cfg_.anchor_alpha * track_.y;
      }
      track_.x = static_anchor_x_;
      track_.y = static_anchor_y_;
      track_.vx = 0.0;
      track_.vy = 0.0;
      if ((static_votes_ <= cfg_.static_vote_release && moving_votes_ >= cfg_.moving_vote_on) ||
          conditioned_dynamic ||
          (hd_score + cfg_.static_release_score_margin < hs_score)) {
        track_.static_mode = false;
        moving_flag_ = true;
      }
    } else {
      moving_flag_ = moving_votes_ >= cfg_.moving_vote_on;
    }

    refreshTailBufferUnlocked();
    commitConfirmedTrailPointUnlocked();

    if (debug != NULL) {
      debug->history_reason = track_.static_mode ? "static_anchor_update" : last_history_action_;
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
    }
  }

  Config cfg_;
  LeadState2D track_;
  std::deque<LeadNode> nodes_;
  std::deque<HistoryPoint2D> confirmed_trail_world_;
  std::deque<HistoryPoint2D> tail_buffer_world_;
  int static_votes_ = 0;
  int moving_votes_ = 0;
  bool moving_flag_ = false;
  std::string last_history_action_ = "init";
  double static_anchor_x_ = 0.0;
  double static_anchor_y_ = 0.0;
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
};

class PathGenerator {
 public:
  struct Config {
    double raw_resample_ds = 0.08;
    double raw_fit_ds = 0.06;
    int raw_spline_passes = 4;
    double raw_spline_alpha = 0.45;
    double raw_tail_weight_scale = 0.25;
    double lead_center_to_path_ref_offset = 0.0;
    int raw_min_points_for_publish = 2;
    double raw_min_span_for_publish = 0.08;
    double raw_keep_behind_x = 0.50;
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

    std::vector<HistoryPoint2D> world_pts = removeNearDuplicatePoints(world_history, 0.01);
    if (world_pts.size() >= 2) {
      recomputeHeading(world_pts);
      world_pts =
          smoothWorldSplineLike(world_pts, cfg_.raw_spline_passes, cfg_.raw_spline_alpha, cfg_.raw_tail_weight_scale);
      world_pts = fitWorldTrajectoryHermite(world_pts, cfg_.raw_fit_ds);
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
    if (raw.empty() || raw_trail.empty()) {
      if (debug != NULL) {
        debug->reference_reason = "raw_empty";
        debug->reference_span = 0.0;
      }
      return {};
    }
    if (static_cast<int>(raw.size()) < cfg_.reference_min_points) {
      if (debug != NULL) {
        debug->reference_reason = "reference_insufficient_points";
        debug->reference_span = polylineLengthLocal(raw);
      }
      return {};
    }
    const double raw_span = polylineLengthLocal(raw);
    if (raw_span < cfg_.reference_min_span) {
      if (debug != NULL) {
        debug->reference_reason = "reference_span_too_short";
        debug->reference_span = raw_span;
      }
      return {};
    }

    std::vector<RefPoint> ref_profile = makeRefPointsFromRawTrail(raw_trail, ego_now, 0.5);
    recomputeRefGeometry(&ref_profile);
    if (ref_profile.size() < static_cast<size_t>(cfg_.reference_min_points)) {
      if (debug != NULL) {
        debug->reference_reason = "reference_profile_too_short";
        debug->reference_span = raw_span;
      }
      return {};
    }

    const JoinChoice join = chooseJoinIndex(ref_profile);
    const RefPoint& join_ref = join.join_point;

    std::vector<RefPoint> combined;
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
    if (debug != NULL) {
      debug->reference_reason = "reference_ready";
      debug->reference_span = polylineLengthLocal(final_local);
    }

    return final_local;
  }

 private:
  struct JoinChoice {
    bool valid = false;
    size_t suffix_index = 0;
    RefPoint join_point;
  };

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

  JoinChoice chooseJoinIndex(const std::vector<RefPoint>& ref_profile) const {
    JoinChoice best;
    if (ref_profile.empty()) {
      return best;
    }
    if (ref_profile.size() == 1) {
      best.valid = true;
      best.suffix_index = 0;
      best.join_point = ref_profile.front();
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
      const double cost = connector_len +
                          0.50 * std::fabs(cand.y) +
                          0.35 * yaw_err +
                          0.25 * tangent_align +
                          cfg_.reference_join_curvature_weight * std::fabs(cand.kappa) +
                          cfg_.reference_join_skip_weight * std::max(0.0, cand.s - s_proj) +
                          cfg_.reference_join_time_weight * std::min(eta, 6.0) +
                          cfg_.reference_join_progress_weight *
                              std::max(0.0, cfg_.reference_join_min_forward_x - cand.x);
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
};

class FixedFrameTrajectoryReconstructor {
 public:
  FixedFrameTrajectoryReconstructor()
      : nh_(),
        pnh_("~"),
        ego_motion_(loadEgoMotionConfig()),
        projector_(loadProjectorConfig()),
        tracker_(loadTrackerConfig()),
        path_generator_(loadPathConfig()) {
    base_frame_ = pnh_.param<std::string>("base_frame", "base_link");
    odom_frame_ = pnh_.param<std::string>("odom_frame", "reconstruction_odom");
    tracked_topic_ = pnh_.param<std::string>("tracked_topic", "/pointpillars/tracked_objects");
    car_topic_ = pnh_.param<std::string>("car_topic", "/car_message");
    carvel_topic_ = pnh_.param<std::string>("carvel_topic", "/carvel");
    raw_topic_ = pnh_.param<std::string>("raw_topic", "/planner/raw_path");
    reference_topic_ = pnh_.param<std::string>("reference_topic", "/planner/reference_path");
    final_traj_log_file_ =
      pnh_.param<std::string>("final_traj_log_file", "/home/ubuntu/front_car_final_trajectory.txt");
    publish_rate_hz_ = pnh_.param("publish_rate_hz", 30.0);

    wheel_speed_scale_ = pnh_.param("wheel_speed_scale", 0.001);
    speed_hint_scale_ = pnh_.param("speed_hint_scale", 1.0);
    steer_angle_scale_ = pnh_.param("steer_angle_scale", M_PI / 180.0);
    steer_angle_bias_ = pnh_.param("steer_angle_bias", 0.0);
    steer_sign_ = pnh_.param("steer_sign", -1.0);
    steer_deadband_rad_ = pnh_.param("steer_deadband_rad", 0.002);
    lost_timeout_ = pnh_.param("lost_timeout", 0.60);
    min_motion_speed_for_yaw_ = pnh_.param("min_motion_speed_for_yaw", 0.15);
    allow_unstamped_tracked_fallback_ = pnh_.param("allow_unstamped_tracked_fallback", true);

    tracked_sub_ = nh_.subscribe(tracked_topic_, 5, &FixedFrameTrajectoryReconstructor::trackedCallback, this);
    car_sub_ = nh_.subscribe(car_topic_, 20, &FixedFrameTrajectoryReconstructor::carCallback, this);
    carvel_sub_ = nh_.subscribe(carvel_topic_, 20, &FixedFrameTrajectoryReconstructor::carvelCallback, this);
    raw_pub_ = nh_.advertise<nav_msgs::Path>(raw_topic_, 1);
    reference_pub_ = nh_.advertise<nav_msgs::Path>(reference_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
                             &FixedFrameTrajectoryReconstructor::timerCallback,
                             this);

    final_traj_log_stream_.open(final_traj_log_file_.c_str(), std::ios::out | std::ios::app);
    if (!final_traj_log_stream_.is_open()) {
      ROS_WARN("trajectory_reconstructor failed to open final trajectory log file: %s",
               final_traj_log_file_.c_str());
    }

    ROS_INFO("trajectory_reconstructor started | tracked=%s car=%s carvel=%s raw=%s ref=%s base=%s odom=%s",
             tracked_topic_.c_str(),
             car_topic_.c_str(),
             carvel_topic_.c_str(),
             raw_topic_.c_str(),
             reference_topic_.c_str(),
             base_frame_.c_str(),
             odom_frame_.c_str());
          ROS_INFO("trajectory_reconstructor final trajectory logging | file=%s enabled=%d",
             final_traj_log_file_.c_str(),
             final_traj_log_stream_.is_open() ? 1 : 0);
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
    cfg.speed_hint_weight = clampValue(pnh_.param("speed_hint_weight", 0.20), 0.0, 1.0);
    cfg.speed_hint_timeout = pnh_.param("speed_hint_timeout", 0.0);
    cfg.speed_hint_agreement_gate = pnh_.param("speed_hint_agreement_gate", 0.40);
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
    return cfg;
  }

  PathGenerator::Config loadPathConfig() const {
    PathGenerator::Config cfg;
    cfg.raw_resample_ds = pnh_.param("raw_resample_ds", 0.08);
    cfg.raw_fit_ds = pnh_.param("raw_fit_ds", 0.06);
    cfg.raw_spline_passes = pnh_.param("raw_spline_passes", 4);
    cfg.raw_spline_alpha = clampValue(pnh_.param("raw_spline_alpha", 0.45), 0.0, 1.0);
    cfg.raw_tail_weight_scale = clampValue(pnh_.param("raw_tail_weight_scale", 0.25), 0.05, 1.0);
    cfg.lead_center_to_path_ref_offset = pnh_.param("lead_center_to_path_ref_offset", 0.0);
    cfg.raw_min_points_for_publish = pnh_.param("raw_min_points_for_publish", 2);
    cfg.raw_min_span_for_publish = pnh_.param("raw_min_span_for_publish", 0.08);
    cfg.raw_keep_behind_x = pnh_.param("raw_keep_behind_x", 0.50);
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
    const ros::Time now = ros::Time::now();

    std::vector<LocalPathPoint> raw_pts;
    std::vector<LocalPathPoint> ref_pts;
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
      ego_motion_.integrateTo(now);
      tracker_.markLostIfExpired(now);
      tracker_.pruneHistory(now);

      EgoPose2D ego_now;
      if (ego_motion_.queryPoseAt(now, &ego_now)) {
        const std::vector<HistoryPoint2D> world_history = tracker_.worldHistoryForPublish(now);
        debug_state_.history_size = world_history.size();
        const std::vector<RawTrailPoint> raw_trail =
            path_generator_.buildRawTrail(world_history, &debug_state_);
        raw_pts = path_generator_.buildRawPath(world_history, ego_now, &debug_state_);
        ref_pts = path_generator_.buildReferencePath(raw_trail, raw_pts, ego_now, &debug_state_);

        if (tracker_.hasActiveTrack()) {
          const LeadState2D pred = predictLeadState(tracker_.track(), now);
          transformWorldToLocal(ego_now, pred.x, pred.y, &lead_x, &lead_y);
        }
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
      debug_snapshot = debug_state_;
    }

    raw_pub_.publish(makePathMsg(raw_pts, base_frame_, now));
    reference_pub_.publish(makePathMsg(ref_pts, base_frame_, now));

    appendReferenceTrajectoryToFile(now, ref_pts);

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

  void appendReferenceTrajectoryToFile(const ros::Time& stamp,
                                       const std::vector<LocalPathPoint>& ref_pts) {
    if (!final_traj_log_stream_.is_open() || ref_pts.empty()) {
      return;
    }

    final_traj_log_stream_ << "FRAME," << stamp.toSec() << "," << ref_pts.size() << "\n";
    for (size_t i = 0; i < ref_pts.size(); ++i) {
      final_traj_log_stream_ << i << ","
                             << ref_pts[i].x << ","
                             << ref_pts[i].y << ","
                             << ref_pts[i].yaw << "\n";
    }
    final_traj_log_stream_ << "END\n";
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber tracked_sub_;
  ros::Subscriber car_sub_;
  ros::Subscriber carvel_sub_;
  ros::Publisher raw_pub_;
  ros::Publisher reference_pub_;
  ros::Timer timer_;
  std::mutex mutex_;

  std::string base_frame_;
  std::string odom_frame_;
  std::string tracked_topic_;
  std::string car_topic_;
  std::string carvel_topic_;
  std::string raw_topic_;
  std::string reference_topic_;
  std::string final_traj_log_file_;

  double publish_rate_hz_ = 30.0;
  double wheel_speed_scale_ = 0.001;
  double speed_hint_scale_ = 1.0;
  double steer_angle_scale_ = M_PI / 180.0;
  double steer_angle_bias_ = 0.0;
  double steer_sign_ = -1.0;
  double steer_deadband_rad_ = 0.002;
  double lost_timeout_ = 0.60;
  double min_motion_speed_for_yaw_ = 0.15;
  bool allow_unstamped_tracked_fallback_ = true;
  std::ofstream final_traj_log_stream_;

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




