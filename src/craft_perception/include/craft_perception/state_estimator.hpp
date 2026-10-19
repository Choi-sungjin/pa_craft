#pragma once
#include <nav_msgs/msg/odometry.hpp>

namespace craft_perception {
// Relay one configured odometry source. Wheel/IMU/lidar fusion remains unimplemented.
class StateEstimator {
public:
  bool is_valid_input(const nav_msgs::msg::Odometry &input) const;
  nav_msgs::msg::Odometry compute_result(const nav_msgs::msg::Odometry &input) const;
};
}
