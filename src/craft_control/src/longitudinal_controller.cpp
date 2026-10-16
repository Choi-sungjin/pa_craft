#include "craft_control/longitudinal_controller.hpp"

#include <algorithm>

namespace craft_control
{

void LongitudinalController::set_params(double wheel_radius, double max_v, double max_acc_v)
{
  wheel_radius_ = wheel_radius;
  max_v_ = max_v;
  max_acc_v_ = max_acc_v;
}

void LongitudinalController::set_target_v(double v)
{
  target_v_ = v;
}

std::array<double, 2> LongitudinalController::get_wheel_speed(double dt)
{
  const double v_goal = std::clamp(target_v_, -max_v_, max_v_);
  v_ref_ += std::clamp(v_goal - v_ref_, -max_acc_v_ * dt, max_acc_v_ * dt);
  const double w = v_ref_ / wheel_radius_;
  return {w, w};
}

}  // namespace craft_control
