#include "craft_control/lateral_controller.hpp"

namespace craft_control
{

void LateralController::set_odom(const nav_msgs::msg::Odometry & odom)
{
  odom_ = odom;
}

void LateralController::set_path(const nav_msgs::msg::Path & path)
{
  path_ = path;
}

std::array<double, 2> LateralController::get_wheel_speed() const
{
  // TODO: path tracking (pure pursuit / stanley ...)
  return {0.0, 0.0};
}

}  // namespace craft_control
