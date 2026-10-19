#define main control_pub_node_main
#include "../src/control_pub.cpp"
#undef main
#include <iostream>
#include <cstdio>

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

sensor_msgs::PointCloud2 cloud(double stamp, double x) {
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp.fromSec(stamp);
    msg.header.frame_id = "base_link";
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> px(msg, "x"), py(msg, "y"), pz(msg, "z");
    *px = x; *py = 0.0; *pz = 0.0;
    return msg;
}

int main() {
    ros::Time::init();
    CollisionSupervisor supervisor;
    const Eigen::Affine3d identity = Eigen::Affine3d::Identity();
    auto send = [&](sensor_msgs::PointCloud2 msg, double receipt, bool tf_ok) {
        supervisor.updatePointCloud(msg, ros::Time(receipt), "cluster_cloud", identity,
                                    tf_ok, "base_link", tf_ok ? 0.0 : -1.0);
        return supervisor.state(ros::Time(receipt));
    };
    auto state = send(cloud(10.0, 0.30), 10.0, true);
    require(state.active, "real close obstacle must stop");
    state = supervisor.state(ros::Time(12.0));
    require(state.active && state.cloud_validity == "timeout", "timeout cleared obstacle");
    state = send(cloud(12.1, 1.0), 12.1, false);
    require(state.active && state.cloud_validity == "tf_failed", "TF failure cleared obstacle");
    auto bad = cloud(12.2, 1.0);
    bad.data.clear();
    state = send(bad, 12.2, true);
    require(state.active && state.cloud_validity == "parse_failed", "malformed cloud cleared obstacle");
    state = send(cloud(10.0, 1.0), 12.3, true);
    require(state.active && state.cloud_validity == "timeout", "old header accepted as fresh");
    state = send(cloud(12.4, std::numeric_limits<double>::quiet_NaN()), 12.4, true);
    require(state.active && state.cloud_validity == "parse_failed", "NaN cloud cleared obstacle");
    state = send(cloud(12.5, 1.0), 12.5, true);
    require(!state.active && state.release_reason == "clear_distance", "clear observation did not release");
    state = send(cloud(12.6, 0.30), 12.6, true);
    require(state.active, "new obstacle failed to stop");
    auto empty = cloud(12.7, 1.0);
    sensor_msgs::PointCloud2Modifier(empty).resize(0);
    state = send(empty, 12.7, true);
    require(!state.active && state.cloud_validity == "valid_empty", "valid empty cloud did not release");
    LeadTargetState missing;
    supervisor.updateLeadTarget(missing, ros::Time(13.0));
    require(!supervisor.state(ros::Time(13.0)).active, "target loss fabricated collision");

    char folder[] = "/tmp/collision-recorder-XXXXXX";
    require(mkdtemp(folder) != nullptr, "temporary directory failed");
    {
        CollisionEventRecorder recorder;
        recorder.start(folder);
        auto oversized = cloud(19.0, 0.3);
        oversized.data.resize(2 * 1024 * 1024 + 1);
        recorder.cloud(oversized, 19.0, "cluster_cloud", identity, true, 0.0);
        recorder.cloud(cloud(20.0, 0.3), 20.0, "cluster_cloud", identity, true, 0.0);
        recorder.tick(20.1, true, "20.1,1,cluster_cloud,obstacle_stop,valid_points,0.1,20,1,1,0,0.3,0.3,0.3,0,1,0");
        recorder.cloud(cloud(20.3, 1.0), 20.3, "cluster_cloud", identity, true, 0.0);
        recorder.tick(22.2, false, "22.2,0,none,valid_empty,valid_empty,0,22.2,0,0,0,-1,-1,0,0,1,0");
    }
    std::ifstream binary(std::string(folder) + "/collision_cloud_event_0.bin", std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(binary)), std::istreambuf_iterator<char>());
    require(!bytes.empty(), "event cloud missing");
    ros::serialization::IStream stream(bytes.data(), bytes.size());
    sensor_msgs::PointCloud2 restored;
    ros::serialization::deserialize(stream, restored);
    require(restored.header.stamp == ros::Time(20.0), "pre-event cloud not recoverable");
    require(restored.data == cloud(20.0, 0.3).data, "cloud payload changed");
    std::ifstream event_log(std::string(folder) + "/collision_events_0.csv");
    std::string row;
    std::getline(event_log, row);
    require(std::count(row.begin(), row.end(), ',') == 18, "event header misaligned");
    std::getline(event_log, row);
    require(std::count(row.begin(), row.end(), ',') == 18, "event row misaligned");
    require(row.substr(row.size() - 6) == ",1,0,0", "oversized frame drop not recorded");
    for (const char* file : {"collision_cloud_event_0.bin", "collision_cloud_event_0.csv", "collision_events_0.csv"})
        std::remove((std::string(folder) + "/" + file).c_str());
    rmdir(folder);
    std::cout << "collision validity, latch release and asynchronous event capture PASS\n";
}
