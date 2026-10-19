#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

// This namespace owns diagnostic state only; it never produces vehicle commands.
namespace history_tracking {

inline double unavailable() { return std::numeric_limits<double>::quiet_NaN(); }
inline double angle(double x) { return std::atan2(std::sin(x), std::cos(x)); }

struct Pose {
    double stamp = 0.0, x = 0.0, y = 0.0, yaw = 0.0, speed = 0.0, yaw_rate = 0.0;
};

struct Point {
    uint64_t id = 0, segment = 0;
    double stamp = 0.0, x = 0.0, y = 0.0, s = 0.0;
    std::string boundary_reason;
};

struct Projection {
    bool valid = false;
    std::string reason = "near_history_missing";
    uint64_t segment = 0, begin_id = 0, end_id = 0;
    double s = unavailable(), x = unavailable(), y = unavailable();
    double tangent = unavailable(), lateral = unavailable(), heading = unavailable();
    double distance = unavailable();
    double source_stamp = unavailable();
};

inline Pose advancePose(Pose pose, double dt) {
    const double dyaw = pose.yaw_rate * dt;
    const double half = 0.5 * dyaw;
    const double sinc = std::fabs(half) < 1e-8 ? 1.0 : std::sin(half) / half;
    pose.x += pose.speed * dt * sinc * std::cos(pose.yaw + half);
    pose.y += pose.speed * dt * sinc * std::sin(pose.yaw + half);
    pose.yaw = angle(pose.yaw + dyaw);
    pose.stamp += dt;
    return pose;
}

inline Point localPoint(const Point& world, const Pose& pose) {
    Point local = world;
    const double dx = world.x - pose.x, dy = world.y - pose.y;
    local.x = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
    local.y = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
    return local;
}

class PoseBuffer {
public:
    void clear() { poses_.clear(); }
    void push(const Pose& pose) {
        if (!poses_.empty() && pose.stamp <= poses_.back().stamp) return;
        poses_.push_back(pose);
        while (poses_.size() > 2 && pose.stamp - poses_.front().stamp > 5.0) poses_.pop_front();
    }
    bool query(double stamp, Pose& pose, std::string& method) const {
        method = "pose_unavailable";
        if (poses_.empty() || stamp < poses_.front().stamp) return false;
        if (stamp >= poses_.back().stamp) {
            const double dt = stamp - poses_.back().stamp;
            if (dt > 0.10) { method = "pose_stale"; return false; }
            pose = advancePose(poses_.back(), dt);
            method = dt > 1e-6 ? "constant_twist_extrapolation" : "exact";
            return true;
        }
        for (size_t i = 1; i < poses_.size(); ++i) {
            if (poses_[i].stamp < stamp) continue;
            const Pose& a = poses_[i - 1];
            const Pose& b = poses_[i];
            if (stamp == a.stamp || stamp == b.stamp) {
                pose = stamp == a.stamp ? a : b;
                method = "exact";
                return true;
            }
            if (b.stamp - a.stamp > 0.10) { method = "pose_gap"; return false; }
            const double u = (stamp - a.stamp) / (b.stamp - a.stamp);
            pose = a;
            pose.stamp = stamp;
            pose.x += u * (b.x - a.x);
            pose.y += u * (b.y - a.y);
            pose.yaw = angle(a.yaw + u * angle(b.yaw - a.yaw));
            pose.speed += u * (b.speed - a.speed);
            pose.yaw_rate += u * (b.yaw_rate - a.yaw_rate);
            method = "interpolation";
            return true;
        }
        return false;
    }
private:
    std::deque<Pose> poses_;
};

inline Projection project(const std::vector<Point>& points, const Pose& pose,
                          double offset = 0.0, double min_s = -1e9, double max_s = 1e9) {
    Projection out;
    const double x = pose.x + offset * std::cos(pose.yaw);
    const double y = pose.y + offset * std::sin(pose.yaw);
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < points.size(); ++i) {
        const Point& a = points[i - 1];
        const Point& b = points[i];
        if (a.segment != b.segment || b.s < min_s || a.s > max_s) continue;
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        if (len2 < 1e-10) continue;
        const double raw_u = ((x - a.x) * dx + (y - a.y) * dy) / len2;
        const double u = std::max(0.0, std::min(1.0, raw_u));
        const double s = a.s + u * (b.s - a.s);
        if (s < min_s || s > max_s) continue;
        const double px = a.x + u * dx, py = a.y + u * dy;
        const double distance = std::hypot(x - px, y - py);
        if (distance >= best) continue;
        best = distance;
        const bool before = raw_u < -1e-6 && (i == 1 || points[i - 2].segment != a.segment);
        const bool after = raw_u > 1.0 + 1e-6 && (i + 1 == points.size() || points[i + 1].segment != a.segment);
        out.valid = !before && !after;
        out.reason = before ? "before_history_start" : (after ? "after_history_end" : "ok");
        out.segment = a.segment;
        out.begin_id = a.id;
        out.end_id = b.id;
        // Endpoint distance is diagnostic even when no supported projection exists.
        out.distance = distance;
        if (!out.valid) {
            out.s = out.x = out.y = out.tangent = out.lateral = out.heading = unavailable();
            out.source_stamp = unavailable();
            continue;
        }
        out.s = s;
        out.x = px;
        out.y = py;
        out.tangent = std::atan2(dy, dx);
        out.lateral = -std::sin(out.tangent) * (x - px) + std::cos(out.tangent) * (y - py);
        out.heading = angle(pose.yaw - out.tangent);
        out.source_stamp = a.stamp + u * (b.stamp - a.stamp);
    }
    return out;
}

struct SameHistoryComparison {
    bool valid = false;
    double displacement = unavailable(), lateral_change = unavailable(), heading_change = unavailable();
};

inline SameHistoryComparison compareAtProjection(const std::vector<Point>& candidate,
                                                const Projection& historical) {
    SameHistoryComparison out;
    if (!historical.valid) return out;
    for (size_t i = 1; i < candidate.size(); ++i) {
        const Point& a = candidate[i-1];
        const Point& b = candidate[i];
        if (a.segment != b.segment || b.stamp <= a.stamp ||
            historical.source_stamp < a.stamp || historical.source_stamp > b.stamp) continue;
        const double u = (historical.source_stamp-a.stamp)/(b.stamp-a.stamp);
        const double dx = b.x-a.x, dy = b.y-a.y;
        if (std::hypot(dx, dy) < 1e-6) return out;
        const double ex = a.x+u*dx-historical.x, ey = a.y+u*dy-historical.y;
        out.displacement = std::hypot(ex, ey);
        out.lateral_change = -std::sin(historical.tangent)*ex + std::cos(historical.tangent)*ey;
        out.heading_change = angle(std::atan2(dy, dx)-historical.tangent);
        out.valid = true;
        break;
    }
    return out;
}

class Ledger {
public:
    size_t capacity = 2048;
    double max_step = 0.25;
    double max_yaw_jump = 1.0;
    double max_kappa = 5.0;
    double overlap_tolerance = 0.10;
    unsigned overlap_skipped = 0, trimmed = 0;
    uint64_t last_trimmed_id = 0;
    double overlap_drift = 0.0;
    std::string status = "empty";

    void append(const std::vector<Point>& candidate, bool accepted) {
        overlap_skipped = trimmed = 0;
        last_trimmed_id = 0;
        overlap_drift = 0.0;
        status = accepted ? "no_new_points" : "candidate_not_accepted";
        if (!accepted || candidate.size() < 2) return;
        for (size_t i = 0; i < candidate.size(); ++i) {
            const Point& p = candidate[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.stamp) || p.stamp <= 0.0) {
                status = "invalid_candidate";
                return;
            }
            if (i && p.stamp <= candidate[i - 1].stamp) {
                status = "nonmonotonic_candidate";
                return;
            }
        }
        bool continuous = points_.empty();
        if (!points_.empty()) {
            const Point& last = points_.back();
            for (size_t i = 1; i < candidate.size(); ++i) {
                const Point& a = candidate[i - 1];
                const Point& b = candidate[i];
                if (a.stamp <= last.stamp && b.stamp >= last.stamp) {
                    const double u = (last.stamp - a.stamp) / (b.stamp - a.stamp);
                    overlap_drift = std::hypot(a.x + u * (b.x - a.x) - last.x,
                                               a.y + u * (b.y - a.y) - last.y);
                    continuous = overlap_drift <= overlap_tolerance;
                    break;
                }
            }
        }
        bool first_new = true;
        for (Point p : candidate) {
            if (p.stamp <= watermark_) { ++overlap_skipped; continue; }
            if (points_.size() >= capacity) { status = "capacity_unpassed_history_retained"; return; }
            std::string boundary = first_new && !continuous
                ? (overlap_drift > overlap_tolerance ? "overlap_geometry_changed" : "no_time_overlap") : "";
            if (!points_.empty()) {
                const Point& last = points_.back();
                const double step = std::hypot(p.x - last.x, p.y - last.y);
                if (step < 0.005) { watermark_ = p.stamp; continue; }
                if (step > max_step) boundary = "position_gap";
                if (points_.size() >= 2 && points_[points_.size() - 2].segment == last.segment) {
                    const Point& prev = points_[points_.size() - 2];
                    const double turn = std::fabs(angle(std::atan2(p.y - last.y, p.x - last.x) -
                                                        std::atan2(last.y - prev.y, last.x - prev.x)));
                    if (turn > max_yaw_jump) boundary = "heading_jump";
                    else if (turn / step > max_kappa) boundary = "curvature_jump";
                }
            }
            const bool split = !boundary.empty();
            if (points_.empty()) boundary = "bootstrap";
            if (points_.empty() || split) ++segment_counter_;
            p.id = ++point_counter_;
            p.segment = segment_counter_;
            p.boundary_reason = boundary;
            p.s = points_.empty() || split ? 0.0 : points_.back().s + std::hypot(p.x - points_.back().x, p.y - points_.back().y);
            points_.push_back(p);
            watermark_ = p.stamp;
            if (!active_segment_) active_segment_ = p.segment;
            if (split || status != "new_disconnected_segment") status = split ? "new_disconnected_segment" : "appended";
            first_new = false;
        }
    }

    Projection updateProgress(const Pose& pose) {
        const double travel = have_pose_ ? std::hypot(pose.x - previous_pose_.x, pose.y - previous_pose_.y) : 0.0;
        previous_pose_ = pose;
        have_pose_ = true;
        std::vector<Point> active = activePoints();
        // An isolated point has no measurable tangent. It must not permanently
        // hide the first subsequent segment with actual two-point support.
        while (!acquired_ && active.size() < 2) {
            auto next = std::find_if(points_.begin(), points_.end(), [&](const Point& q) { return q.segment > active_segment_; });
            if (next == points_.end()) break;
            active_segment_ = next->segment;
            active = activePoints();
            status = "unsupported_singleton_skipped";
        }
        Projection p = project(active, pose, 0.0, acquired_ ? progress_ - 0.10 : -1e9,
                               acquired_ ? progress_ + std::min(0.50, travel + 0.25) : 1e9);
        if (p.valid && p.distance <= 0.50) {
            progress_ = acquired_ ? std::max(progress_, p.s) : p.s;
            acquired_ = true;
        }
        // A discontinuity is never crossed by a synthetic line. Reacquire only
        // after actually reaching the old end and the start of the next segment.
        if (acquired_ && !active.empty() && progress_ >= active.back().s - 0.10) {
            auto next = std::find_if(points_.begin(), points_.end(), [&](const Point& q) { return q.segment > active_segment_; });
            if (next != points_.end() && std::hypot(pose.x - next->x, pose.y - next->y) <= 0.25) {
                active_segment_ = next->segment;
                acquired_ = false;
                progress_ = 0.0;
                p = project(activePoints(), pose);
            }
        }
        while (points_.size() > 2 &&
               (points_[1].segment < active_segment_ ||
                (acquired_ && points_[1].segment == active_segment_ && points_[1].s < progress_ - 1.0))) {
            last_trimmed_id = points_.front().id;
            points_.pop_front();
            ++trimmed;
        }
        return p;
    }

    std::vector<Point> activePoints() const {
        std::vector<Point> out;
        for (const Point& p : points_) if (p.segment == active_segment_) out.push_back(p);
        return out;
    }
    const std::deque<Point>& points() const { return points_; }
    uint64_t activeSegment() const { return active_segment_; }
    bool acquired() const { return acquired_; }
    double progress() const { return acquired_ ? progress_ : unavailable(); }

private:
    std::deque<Point> points_;
    uint64_t segment_counter_ = 0, point_counter_ = 0, active_segment_ = 0;
    double watermark_ = 0.0, progress_ = 0.0;
    bool acquired_ = false, have_pose_ = false;
    Pose previous_pose_;
};

}  // namespace history_tracking
