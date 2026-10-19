// Compile with -fno-access-control; exercise the production implementation.
#define main trajectory_reconstructor_node_main
#include "../src/trajectory_reconstructor.cpp"
#undef main
#include <iostream>
#include <stdexcept>

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

int main() {
  ros::Time::init();
  for (int shape = 0; shape < 3; ++shape) {
    LeadTrailTracker::Config cfg;
    cfg.provisional_tail_enabled = true;
    cfg.provisional_tail_min_points = 3;
    cfg.provisional_tail_max_age = 1.5;
    cfg.suffix_smooth_passes = 1;
    cfg.suffix_smooth_alpha = 0.16;
    cfg.suffix_smooth_max_adjust = 0.025;
    LeadTrailTracker tracker(cfg);
    std::vector<HistoryPoint2D> expected_history;
    for (int i = 0; i <= 24; ++i) {
      HistoryPoint2D p;
      p.stamp.fromSec(100.0 + i * 0.1);
      const double s = i * (shape == 0 ? 0.05 : M_PI / 48.0);
      p.x = shape == 0 ? s : std::sin(s);
      p.y = shape == 0 ? 0.0 : (shape == 1 ? 1.0 : -1.0) * (1.0 - std::cos(s));
      p.heading = shape == 0 ? 0.0 : (shape == 1 ? s : -s);
      if (i <= 20) tracker.confirmed_trail_world_.push_back(p);
      if (i >= 10) tracker.motion_tail_buffer_world_.push_back(p);
      expected_history.push_back(p);
    }
    TrackerDebugState debug;
    const auto result = tracker.exportReferenceHistoryWithTailUnlocked(&debug);
    require(!result.empty(), "history disappeared");
    for (size_t i = 1; i < result.size(); ++i)
      require(result[i].stamp >= result[i - 1].stamp, "overlap appended older observations after committed history");
    require(tracker.worldHistorySpan(result) < 1.65, "overlap counted as extra arc length");
    require(result.back().stamp.toSec() > 102.3, "fresh tail lost");
    require(debug.tail_overlap_skipped == 11, "overlap diagnostic incorrect");
    const auto expected = tracker.smoothReferenceSuffixUnlocked(expected_history, nullptr);
    require(expected.size() == result.size(), "chronological history size changed");
    for (size_t i = 0; i < result.size(); ++i)
      require(pointDistance(result[i].x, result[i].y, expected[i].x, expected[i].y) < 1e-9,
              "merge changed existing centerline geometry");
    PathGenerator::Config pcfg;
    pcfg.raw_preserve_history_geometry = true;
    PathGenerator generator(pcfg);
    const auto raw = generator.buildRawTrail(result, &debug);
    require(raw.size() > 10, "bend was replaced by a short tail");
    require(pointDistance(raw.front().x, raw.front().y, result.front().x, result.front().y) < 0.1,
            "stable selection discarded the bend entrance");
    std::cout << "shape=" << shape << " span=" << tracker.worldHistorySpan(result)
              << " raw_points=" << raw.size() << " PASS\n";
    // No new observations must leave the existing history available.
    while (tracker.motion_tail_buffer_world_.back().stamp > tracker.confirmed_trail_world_.back().stamp)
      tracker.motion_tail_buffer_world_.pop_back();
    const auto held = tracker.exportReferenceHistoryWithTailUnlocked(&debug);
    require(!held.empty(), "overlap-only update cleared history");
    require(held.back().stamp <= tracker.confirmed_trail_world_.back().stamp,
            "overlap-only update invented a newer endpoint");
  }
}
