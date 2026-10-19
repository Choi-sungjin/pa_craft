#pragma once

#include <array>
#include <optional>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace craft_control
{

class LateralController
{
public:
  // All five values are validated positive by ControlMaster.
  void configure(
    double wheel_radius, double wheel_separation, double lookahead, double goal_tolerance,
    double path_spacing);
  // Nonempty frame_id, finite position, nonzero quaternion.
  bool is_valid_odom(const nav_msgs::msg::Odometry & odom) const;
  // Nonempty path in one nonempty frame with finite positions.
  bool is_valid_target(const nav_msgs::msg::Path & path) const;
  // Store odometry/path after validation succeeds; no validation here.
  void set_odom(const nav_msgs::msg::Odometry & odom);
  void clear_odom();
  // Stores the path resampled every path_spacing along a cubic spline through its points.
  void set_target(const nav_msgs::msg::Path & path);
  void clear_target();
  // Resampled path being tracked (empty when there is no target).
  const nav_msgs::msg::Path & target_path() const;
  bool can_track(double target_v) const;
  // Path endpoint within goal_tolerance of the current odometry (same frame required).
  bool is_target_reached() const;
  // Pure Pursuit: omega = v * curvature. Only the rotational wheel-speed contribution.
  std::array<double, 2> compute_command(double linear_speed) const;  // {left, right} [rad/s]

private:
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path path_;
  std::optional<double> get_curvature(double target_v) const;
  double wheel_radius_ = 0.0;
  double wheel_separation_ = 0.0;
  double lookahead_ = 0.0;
  double goal_tolerance_ = 0.0;
  double path_spacing_ = 0.0;
  bool have_odom_ = false;
};

}  // namespace craft_control
