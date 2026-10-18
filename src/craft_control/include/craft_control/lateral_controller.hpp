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
  bool set_params(double wheel_radius, double wheel_separation, double lookahead, double goal_tolerance);
  bool set_odom(const nav_msgs::msg::Odometry & odom);
  bool set_path(const nav_msgs::msg::Path & path);
  bool can_track(double target_v) const;
  // Pure Pursuit: omega = v * curvature. Only the rotational wheel-speed contribution.
  std::array<double, 2> get_wheel_speed(double linear_speed) const;  // {left, right} [rad/s]

private:
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path path_;
  std::optional<double> get_curvature(double target_v) const;
  double wheel_radius_ = 0.0;
  double wheel_separation_ = 0.0;
  double lookahead_ = 0.0;
  double goal_tolerance_ = 0.0;
  bool configured_ = false;
  bool have_odom_ = false;
};

}  // namespace craft_control
