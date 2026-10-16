#pragma once

#include <array>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace craft_control
{

class LateralController
{
public:
  void set_odom(const nav_msgs::msg::Odometry & odom);
  void set_path(const nav_msgs::msg::Path & path);
  std::array<double, 2> get_wheel_speed() const;  // {left, right}

private:
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path path_;
};

}  // namespace craft_control
