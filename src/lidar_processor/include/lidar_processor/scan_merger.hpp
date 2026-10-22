#pragma once
#include <sensor_msgs/msg/laser_scan.hpp>
#include <optional>
#include <string>

namespace lidar_processing {
struct Pose2D { double x = 0, y = 0, yaw = 0; };
struct MergerConfig {
  std::string target_frame = "base_link";
  int bins = 720;
  double range_max = 8.0, max_age = 0.2;
  double front_fov_deg = 200.0, rear_fov_deg = 200.0;
  double fov_near_m = 0.30, body_radius_m = 0.25;
};
// Pure processing module: the caller owns subscriptions, TF and publishing.
class ScanMerger {
public:
  bool configure(const MergerConfig &config);
  static bool is_valid_input(const sensor_msgs::msg::LaserScan &scan);
  std::optional<sensor_msgs::msg::LaserScan> merge(
    const sensor_msgs::msg::LaserScan &front, const Pose2D &front_pose,
    const sensor_msgs::msg::LaserScan *rear = nullptr, const Pose2D *rear_pose = nullptr) const;
private:
  void add_scan(const sensor_msgs::msg::LaserScan &scan, const Pose2D &pose,
                double fov_deg, sensor_msgs::msg::LaserScan &out) const;
  MergerConfig config_;
};
}
