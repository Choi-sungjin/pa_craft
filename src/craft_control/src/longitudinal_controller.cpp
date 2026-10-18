#include "craft_control/longitudinal_controller.hpp"

#include <algorithm>

namespace craft_control
{

void LongitudinalController::set_params(double wheel_radius, double max_v)
{
  wheel_radius_ = wheel_radius;
  max_v_ = max_v;
}

void LongitudinalController::set_target_v(double v)
{
  target_v_ = v;
}

void LongitudinalController::stop()
{
  target_v_ = 0.0;
}

std::array<double, 2> LongitudinalController::get_wheel_speed() const
{
  if (wheel_radius_ <= 0.0) {
    return {0.0, 0.0};
  }
  const double limited_v = std::clamp(target_v_, -max_v_, max_v_);
  const double w = limited_v / wheel_radius_;
  return {w, w};
}

}  // namespace craft_control
