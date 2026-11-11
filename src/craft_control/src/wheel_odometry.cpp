#include "craft_control/wheel_odometry.hpp"

#include <cmath>
#include <utility>

namespace craft_control
{

void WheelOdometry::configure(
  double wheel_radius, double wheel_separation, double in_place_ratio, std::vector<double> yaw_rate_coeffs)
{
  radius_ = wheel_radius;
  separation_ = wheel_separation;
  in_place_ratio_ = in_place_ratio;
  yaw_rate_coeffs_ = std::move(yaw_rate_coeffs);
}

void WheelOdometry::update(double wheel_left, double wheel_right, double dt)
{
  if (!(dt > 0.0) || !std::isfinite(dt) || !std::isfinite(wheel_left) || !std::isfinite(wheel_right)) {
    return;
  }
  const bool in_place = wheel_left * wheel_right < 0.0 &&
    std::fabs(wheel_left + wheel_right) < in_place_ratio_ * std::fabs(wheel_right - wheel_left);
  v_ = in_place ? 0.0 : radius_ * (wheel_left + wheel_right) / 2.0;
  const double delta = wheel_right - wheel_left;
  if (yaw_rate_coeffs_.empty()) {
    w_ = radius_ * delta / separation_;
  } else {
    double rate = 0.0, power = 1.0;
    for (const double c : yaw_rate_coeffs_) {
      power *= std::fabs(delta);
      rate += c * power;
    }
    w_ = std::copysign(rate, delta);
  }
  const double mid_yaw = yaw_ + w_ * dt / 2.0;
  x_ += v_ * std::cos(mid_yaw) * dt;
  y_ += v_ * std::sin(mid_yaw) * dt;
  yaw_ = std::remainder(yaw_ + w_ * dt, 2.0 * M_PI);
}

}  // namespace craft_control
