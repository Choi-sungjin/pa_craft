#pragma once
#include "craft_perception/lidar_obstacle_detector.hpp"
#include <lidar_processor/scan_merger.hpp>

namespace craft_perception {
// Like D435Perception/C310Perception: algorithms only; master owns ROS I/O and timers.
class LidarPerception {
public:
  bool configure(const lidar_processing::MergerConfig &config, double max_range, int stride);
  std::optional<sensor_msgs::msg::LaserScan> merge(
    const sensor_msgs::msg::LaserScan &front, const lidar_processing::Pose2D &front_pose,
    const sensor_msgs::msg::LaserScan *rear, const lidar_processing::Pose2D *rear_pose) const;
  std::optional<geometry_msgs::msg::PoseArray> detect_obstacles(
    const sensor_msgs::msg::LaserScan &scan) const;
private:
  lidar_processing::ScanMerger merger_;
  LidarObstacleDetector obstacle_;
};
}
