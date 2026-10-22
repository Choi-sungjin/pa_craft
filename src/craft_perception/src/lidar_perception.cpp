#include "craft_perception/lidar_perception.hpp"

namespace craft_perception {
bool LidarPerception::configure(const lidar_processing::MergerConfig &config,
  double max_range, int stride) {
  return merger_.configure(config) && obstacle_.configure(max_range, stride);
}
std::optional<sensor_msgs::msg::LaserScan> LidarPerception::merge(
  const sensor_msgs::msg::LaserScan &front, const lidar_processing::Pose2D &front_pose,
  const sensor_msgs::msg::LaserScan *rear, const lidar_processing::Pose2D *rear_pose) const {
  return merger_.merge(front, front_pose, rear, rear_pose);
}
std::optional<geometry_msgs::msg::PoseArray> LidarPerception::detect_obstacles(
  const sensor_msgs::msg::LaserScan &scan) const {
  if (!obstacle_.is_valid_input(scan)) return std::nullopt;
  return obstacle_.compute_result(scan);
}
}
