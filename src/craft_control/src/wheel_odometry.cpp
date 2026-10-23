#include "craft_control/wheel_odometry.hpp"

#include <cmath>

namespace craft_control
{

void WheelOdometry::configure(double wheel_radius, double wheel_separation)
{
  radius_ = wheel_radius;
  separation_ = wheel_separation;
}

void WheelOdometry::update(double wheel_left, double wheel_right, double dt)
{
  if (!(dt > 0.0) || !std::isfinite(dt) || !std::isfinite(wheel_left) || !std::isfinite(wheel_right)) {
    return;
  }
  v_ = radius_ * (wheel_left + wheel_right) / 2.0;
  w_ = radius_ * (wheel_right - wheel_left) / separation_;
  const double mid_yaw = yaw_ + w_ * dt / 2.0;
  x_ += v_ * std::cos(mid_yaw) * dt;
  y_ += v_ * std::sin(mid_yaw) * dt;
  yaw_ = std::remainder(yaw_ + w_ * dt, 2.0 * M_PI);
}

}  // namespace craft_control
