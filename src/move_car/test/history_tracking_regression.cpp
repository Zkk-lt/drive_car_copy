#include "history_tracking_diagnostics.h"
#include <iostream>
#include <stdexcept>

using namespace history_tracking;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool close(double a, double b, double tolerance = 1e-9) { return std::fabs(a-b) < tolerance; }

std::vector<Point> arc(double sign) {
    std::vector<Point> points;
    for (int i = 0; i <= 40; ++i) {
        const double theta = i * M_PI / 80.0;
        Point p;
        p.stamp = 10.0 + i * 0.1;
        p.x = 2.0 * std::sin(theta);
        p.y = sign * 2.0 * (1.0 - std::cos(theta));
        points.push_back(p);
    }
    return points;
}

int main() {
    for (double sign : {-1.0, 1.0}) {
        Ledger ledger;
        auto input = arc(sign);
        ledger.append(input, true);
        const auto original = ledger.activePoints();
        require(original.size() == input.size(), "valid 90 degree bend was split");
        Pose pose;
        pose.x = (original[15].x + original[16].x) * 0.5;
        pose.y = (original[15].y + original[16].y) * 0.5;
        pose.yaw = std::atan2(original[16].y-original[15].y, original[16].x-original[15].x);
        const auto baseline = project(original, pose);
        require(baseline.valid && close(baseline.lateral, 0) && close(baseline.heading, 0), "history projection wrong");
        auto unchanged = compareAtProjection(original, baseline);
        require(unchanged.valid && close(unchanged.displacement, 0) && close(unchanged.heading_change, 0),
                "same-history comparison changed the lookup location");
        auto shifted = original;
        for (Point& p : shifted) p.y += .12;
        const auto changed = compareAtProjection(shifted, baseline);
        require(changed.valid && close(changed.displacement, .12) && close(changed.heading_change, 0),
                "same-source-time geometry displacement not detected");
        require(close(changed.lateral_change, .12*std::cos(baseline.tangent)),
                "same-source-time displacement lost its lateral sign");
        require(!compareAtProjection({}, baseline).valid, "absent candidate overlap reported as zero drift");
        for (double rotation : {-0.10, 0.10}) {
            Pose turned = pose;
            turned.yaw += rotation;
            const auto world_error = project(original, turned);
            std::vector<Point> local;
            for (const Point& p : original) {
                const Point q = localPoint(p, turned);
                local.push_back(q);
                const double x = turned.x + std::cos(turned.yaw)*q.x - std::sin(turned.yaw)*q.y;
                const double y = turned.y + std::sin(turned.yaw)*q.x + std::cos(turned.yaw)*q.y;
                require(close(x, p.x) && close(y, p.y) && q.id == p.id, "ego rotation moved immutable history");
            }
            const auto local_error = project(local, Pose());
            require(world_error.valid && local_error.valid, "rotation lost supported projection");
            require(close(world_error.heading, rotation) && close(local_error.heading, rotation), "heading missing or yaw subtracted twice");
            require(close(world_error.lateral, local_error.lateral), "lateral error not frame invariant");
            require(-world_error.heading * rotation < 0, "heading correction sign wrong");
        }

        ledger.updateProgress(pose);
        const uint64_t segment = ledger.activeSegment();
        std::vector<Point> tail(input.end()-5, input.end());
        for (int i = 1; i <= 8; ++i) {
            Point p = input.back();
            p.stamp += i * 0.1;
            p.y += sign * i * 0.08;
            tail.push_back(p);
        }
        ledger.append(tail, true);
        require(ledger.activeSegment() == segment, "latest tail replaced unpassed bend");
        for (const Point& p : ledger.points()) {
            if (p.id <= original.back().id) {
                require(close(p.x, original[p.id-1].x) && close(p.y, original[p.id-1].y), "accepted history refitted");
            }
        }
        const size_t count = ledger.points().size();
        ledger.append({}, false);
        require(ledger.points().size() == count, "target loss cleared history");
        ledger.append(tail, true);
        require(ledger.points().size() == count && ledger.overlap_skipped == tail.size(), "overlap duplicated history");

        std::vector<Point> disconnected;
        for (int i = 0; i < 5; ++i) {
            Point p;
            p.stamp = 30+i*.1; p.x = 5+i*.08; p.y = 5;
            disconnected.push_back(p);
        }
        ledger.append(disconnected, true);
        require(ledger.activeSegment() == segment && ledger.points().back().segment != segment, "gap joined or active bend skipped");
        require(ledger.activePoints().back().x < 3, "new disconnected tail leaked into active projection");
        std::cout << (sign > 0 ? "left" : "right") << " rotation, immutable history and continuity PASS\n";
    }

    Ledger bounded;
    bounded.capacity = 5;
    bounded.append(arc(1), true);
    require(bounded.points().size() == 5 && bounded.points().front().id == 1, "capacity evicted unpassed history");
    require(bounded.status == "capacity_unpassed_history_retained", "capacity diagnostic missing");
    Ledger singleton;
    std::vector<Point> initial(3);
    for (int i=0; i<3; ++i) { initial[i].stamp=10+i*.1; initial[i].x=1+i*.08; }
    initial[0].x=0;
    singleton.append(initial, true);
    Pose at_supported_segment;
    at_supported_segment.x=1.10;
    const auto supported=singleton.updateProgress(at_supported_segment);
    require(supported.valid && supported.segment == 2, "isolated first point permanently hid supported segment");
    Ledger invalid;
    auto broken = arc(1);
    broken[5].stamp = broken[4].stamp;
    invalid.append(broken, true);
    require(invalid.points().empty(), "nonmonotonic candidate partially accepted");
    Ledger drifted;
    auto straight = arc(1);
    straight.resize(15);
    drifted.append(straight, true);
    const auto frozen = drifted.activePoints();
    auto shifted_tail = arc(1);
    for (Point& p : shifted_tail) p.y += .15;
    drifted.append(shifted_tail, true);
    require(drifted.points().back().segment != frozen.front().segment,
            "changed overlapping geometry silently attached to old history");
    for (size_t i = 0; i < frozen.size(); ++i)
        require(close(drifted.points()[i].x, frozen[i].x) && close(drifted.points()[i].y, frozen[i].y),
                "overlapping candidate moved an accepted point");
    Ledger advancing;
    auto line = arc(1);
    for (size_t i = 0; i < line.size(); ++i) { line[i].x = i*.08; line[i].y = 0; }
    advancing.append(line, true);
    Pose follower;
    follower.x = .01;
    advancing.updateProgress(follower);
    for (int i = 1; i <= 25; ++i) {
        advancing.append({}, false);
        follower.x = i*.08;
        advancing.updateProgress(follower);
    }
    require(advancing.points().front().x > .8 && advancing.points().front().x < 1.1,
            "passed prefix not trimmed with one meter of support retained");
    require(advancing.points().back().x == line.back().x, "progress trimmed unpassed history");
    const auto missing = project({}, Pose());
    require(!missing.valid && std::isnan(missing.heading), "missing history masquerades as zero error");
    Pose before;
    before.x = -1;
    const auto before_error = project(bounded.activePoints(), before);
    require(!before_error.valid && std::isnan(before_error.lateral), "unsupported endpoint tangent extrapolated");

    PoseBuffer buffer;
    Pose a;
    a.stamp = 100; a.speed = .2; a.yaw_rate = .3;
    buffer.push(a);
    const Pose b = advancePose(a, .05);
    buffer.push(b);
    Pose query;
    std::string method;
    require(buffer.query(100.05, query, method) && close(query.yaw, .015), "exact pose query wrong");
    require(buffer.query(100.025, query, method) && close(query.yaw, .0075), "interpolated pose query wrong");
    require(buffer.query(100.10, query, method) && close(query.yaw, .03), "short extrapolation wrong");
    require(!buffer.query(99.9, query, method) && !buffer.query(100.3, query, method), "pose outside buffer silently clamped");
    Pose future = b;
    future.stamp = 101;
    buffer.push(future);
    require(buffer.query(b.stamp, query, method) && method == "exact", "exact pose rejected because next sample has a gap");
    require(!buffer.query(100.5, query, method) && method == "pose_gap", "missing pose interval interpolated");
    buffer.clear();
    require(!buffer.query(100.0, query, method), "epoch reset retained old pose");

    Pose old_pose;
    old_pose.stamp = 50; old_pose.x = 2; old_pose.y = 3; old_pose.yaw = .4;
    old_pose.speed = .3; old_pose.yaw_rate = .2;
    const Pose now_pose = advancePose(old_pose, .08);
    Point world;
    world.x = 4; world.y = 4;
    const Point old_local = localPoint(world, old_pose);
    Point reconstructed;
    reconstructed.x = old_pose.x + std::cos(old_pose.yaw)*old_local.x - std::sin(old_pose.yaw)*old_local.y;
    reconstructed.y = old_pose.y + std::sin(old_pose.yaw)*old_local.x + std::cos(old_pose.yaw)*old_local.y;
    const auto rebased = localPoint(reconstructed, now_pose);
    const auto direct = localPoint(world, now_pose);
    require(close(rebased.x, direct.x) && close(rebased.y, direct.y), "old local path rebase applied yaw twice");
    std::cout << "capacity, invalid coverage, timestamp bounds and local-path rebase PASS\n";
}
