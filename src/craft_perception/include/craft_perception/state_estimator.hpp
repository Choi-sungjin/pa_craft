#pragma once
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

namespace craft_perception {
// Re-express ekf_local odometry (odom frame) in the map frame via slam_toolbox's map->odom; relay if already in map.
class StateEstimator {
public:
  bool is_valid_input(const nav_msgs::msg::Odometry &input) const;
  nav_msgs::msg::Odometry compute_result(const nav_msgs::msg::Odometry &input) const;
  // Pose becomes map_from_odom * input pose; twist stays in child_frame_id.
  nav_msgs::msg::Odometry compute_result(const nav_msgs::msg::Odometry &input,
    const geometry_msgs::msg::Transform &map_from_odom, const std::string &map_frame) const;
};
}
