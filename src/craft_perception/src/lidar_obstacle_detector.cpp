#include "craft_perception/lidar_obstacle_detector.hpp"
#include <cmath>
#include <stdexcept>

namespace craft_perception {
bool LidarObstacleDetector::configure(double maximum_range, int stride) {
  if (!std::isfinite(maximum_range) || maximum_range <= 0 || stride < 1) return false;
  maximum_range_ = maximum_range; stride_ = stride;
  return true;
}

bool LidarObstacleDetector::is_valid_input(const sensor_msgs::msg::LaserScan &scan) const {
  return !scan.header.frame_id.empty() && std::isfinite(scan.angle_min) &&
    std::isfinite(scan.angle_increment) && std::isfinite(scan.range_min) &&
    std::isfinite(scan.range_max) && scan.range_min >= 0 && scan.range_max >= scan.range_min;
}

geometry_msgs::msg::PoseArray LidarObstacleDetector::compute_result(const sensor_msgs::msg::LaserScan &scan) const {
  geometry_msgs::msg::PoseArray output;
  output.header = scan.header;
  for (size_t i = 0; i < scan.ranges.size(); i += stride_) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max ||
        range > maximum_range_) continue;
    const double angle = scan.angle_min + i * scan.angle_increment;
    geometry_msgs::msg::Pose point;
    point.position.x = range * std::cos(angle);
    point.position.y = range * std::sin(angle);
    point.orientation.w = 1;
    output.poses.push_back(point);
  }
  return output;
}
}
