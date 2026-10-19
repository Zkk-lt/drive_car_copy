#pragma once

#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

// Diagnostics only: bounded snapshots and asynchronous disk writes.
class CollisionEventRecorder {
 public:
  ~CollisionEventRecorder() {
    if (!worker_.joinable()) return;
    finishCapture();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_one();
    worker_.join();
  }

  void start(const std::string& directory) {
    directory_ = directory;
    worker_ = std::thread(&CollisionEventRecorder::run, this);
  }

  void cloud(const sensor_msgs::PointCloud2& msg, double stamp,
             const std::string& source, const Eigen::Affine3d& transform,
             bool tf_ok, double tf_age) {
    if (!worker_.joinable()) return;
    double& last = source == "cluster_cloud" ? last_cluster_ : last_filtered_;
    if (stamp >= last && stamp - last < 0.2) return;
    last = stamp;
    const size_t size = ros::serialization::serializationLength(msg);
    if (size > 2 * 1024 * 1024) { ++dropped_frames_; return; }
    auto frame = std::make_shared<Frame>();
    frame->stamp = stamp;
    frame->source = source;
    frame->tf_ok = tf_ok && transform.matrix().allFinite();
    frame->tf_age = tf_age;
    frame->transform = transform.matrix();
    frame->bytes.resize(size);
    ros::serialization::OStream stream(frame->bytes.data(), size);
    ros::serialization::serialize(stream, msg);
    if (!ring_.empty() && stamp < ring_.back()->stamp) {
      ring_.clear();
      ring_bytes_ = 0;
    }
    ring_.push_back(frame);
    ring_bytes_ += size;
    while (!ring_.empty() &&
           (stamp - ring_.front()->stamp > 2.0 || ring_bytes_ > kBufferBytes)) {
      ring_bytes_ -= ring_.front()->bytes.size();
      ring_.pop_front();
    }
    if (capturing_ && stamp <= capture_until_) {
      if (capture_bytes_ + size <= kEventBytes) {
        capture_.push_back(frame);
        capture_bytes_ += size;
      } else {
        ++dropped_frames_;
      }
    }
  }

  void tick(double stamp, bool active, const std::string& row) {
    if (!worker_.joinable()) return;
    if (capturing_ && (stamp >= capture_until_ || stamp < last_tick_)) finishCapture();
    if (active && !was_active_ && !capturing_) {
      capture_.assign(ring_.begin(), ring_.end());
      capture_bytes_ = ring_bytes_;
      capturing_ = true;
      capture_until_ = stamp + 2.0;
    }
    if (active != was_active_ || stamp - last_log_ >= 0.1 || stamp < last_log_) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (rows_.size() < 256) {
        rows_.push_back(row + "," + std::to_string(dropped_frames_) + "," +
                        std::to_string(dropped_events_) + "," + std::to_string(dropped_rows_));
      } else {
        ++dropped_rows_;
      }
      last_log_ = stamp;
      ready_.notify_one();
    }
    was_active_ = active;
    last_tick_ = stamp;
  }

 private:
  struct Frame {
    double stamp = 0.0;
    std::string source;
    bool tf_ok = false;
    double tf_age = -1.0;
    Eigen::Matrix4d transform;
    std::vector<uint8_t> bytes;
  };
  using Frames = std::vector<std::shared_ptr<Frame>>;
  static constexpr size_t kBufferBytes = 8 * 1024 * 1024;
  static constexpr size_t kEventBytes = 16 * 1024 * 1024;

  void finishCapture() {
    if (!capturing_) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (jobs_.empty()) jobs_.push_back(std::move(capture_));
      else { ++dropped_events_; capture_.clear(); }
    }
    capturing_ = false;
    capture_bytes_ = 0;
    ready_.notify_one();
  }

  void writeEvent(const Frames& frames, size_t slot) {
    const std::string base = directory_ + "/collision_cloud_event_" + std::to_string(slot);
    std::ofstream data(base + ".bin", std::ios::binary | std::ios::trunc);
    std::ofstream index(base + ".csv", std::ios::trunc);
    if (!data || !index) {
      ROS_WARN("Cannot write collision event capture in %s", directory_.c_str());
      return;
    }
    index << "stamp,source,offset,length,tf_ok,tf_age";
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) index << ",base_from_cloud_" << r << c;
    index << "\n" << std::fixed << std::setprecision(9);
    for (const auto& frame : frames) {
      index << frame->stamp << ',' << frame->source << ',' << data.tellp() << ','
            << frame->bytes.size() << ',' << frame->tf_ok << ',' << frame->tf_age;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) index << ',' << frame->transform(r, c);
      index << '\n';
      data.write(reinterpret_cast<const char*>(frame->bytes.data()), frame->bytes.size());
    }
    if (!data || !index) ROS_WARN("Collision event capture write failed");
  }

  void run() {
    size_t event_slot = 0, csv_slot = 0, row_count = 0;
    std::ofstream log;
    while (true) {
      Frames frames;
      std::deque<std::string> rows;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [&] { return stopping_ || !jobs_.empty() || !rows_.empty(); });
        if (stopping_ && jobs_.empty() && rows_.empty()) break;
        if (!jobs_.empty()) { frames = std::move(jobs_.front()); jobs_.pop_front(); }
        rows.swap(rows_);
      }
      for (const auto& row : rows) {
        if (!log.is_open() || row_count >= 20000) {
          log.close();
          log.clear();
          log.open(directory_ + "/collision_events_" + std::to_string(csv_slot++ % 4) + ".csv",
                   std::ios::trunc);
          log << "stamp,active,source,release_reason,cloud_validity,cloud_age,cloud_source_stamp,"
                 "input_points,kept_points,self_rejected,raw_nearest,effective_nearest,"
                 "nearest_x,nearest_y,tf_ok,tf_age,dropped_frames,dropped_events,dropped_rows\n";
          row_count = 0;
          if (!log) ROS_WARN("Cannot open collision event log in %s", directory_.c_str());
        }
        log << row << '\n';
        ++row_count;
      }
      if (!rows.empty()) log.flush();
      if (!frames.empty()) writeEvent(frames, event_slot++ % 8);
    }
  }

  std::string directory_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stopping_ = false;
  std::deque<Frames> jobs_;
  std::deque<std::string> rows_;
  std::deque<std::shared_ptr<Frame>> ring_;
  Frames capture_;
  size_t ring_bytes_ = 0, capture_bytes_ = 0;
  size_t dropped_frames_ = 0, dropped_events_ = 0, dropped_rows_ = 0;
  bool capturing_ = false, was_active_ = false;
  double capture_until_ = 0.0, last_log_ = -1.0, last_tick_ = 0.0;
  double last_cluster_ = -1.0, last_filtered_ = -1.0;
};
