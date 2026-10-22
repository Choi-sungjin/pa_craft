#pragma once
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

namespace craft_perception {
// Re-express one odometry source in the map frame. Wheel/IMU/lidar fusion remains unimplemented.
class StateEstimator {
public:
  bool is_valid_input(const nav_msgs::msg::Odometry &input) const;
  nav_msgs::msg::Odometry compute_result(const nav_msgs::msg::Odometry &input) const;
  // Pose becomes map_from_odom * input pose; twist stays in child_frame_id.
  nav_msgs::msg::Odometry compute_result(const nav_msgs::msg::Odometry &input,
    const geometry_msgs::msg::Transform &map_from_odom, const std::string &map_frame) const;
};
}
