#pragma once
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace craft_perception {
class ObstacleDetector {
public:
  bool configure(double maximum_range, int stride);
  bool is_valid_input(const sensor_msgs::msg::LaserScan &scan) const;
  geometry_msgs::msg::PoseArray compute_result(const sensor_msgs::msg::LaserScan &scan) const;
private:
  double maximum_range_ = 0;
  int stride_ = 0;
};
}
