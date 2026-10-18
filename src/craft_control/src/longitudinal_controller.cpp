#include "craft_control/longitudinal_controller.hpp"

#include <algorithm>
#include <cmath>

namespace craft_control
{

void LongitudinalController::configure(double wheel_radius, double max_v)
{
  wheel_radius_ = wheel_radius;
  max_v_ = max_v;
}

bool LongitudinalController::is_valid_target(double v) const
{
  return std::isfinite(v);
}

void LongitudinalController::set_target(double v)
{
  target_v_ = v;
}

void LongitudinalController::clear_target()
{
  target_v_ = 0.0;
}

std::array<double, 2> LongitudinalController::compute_command() const
{
  const double limited_v = std::clamp(target_v_, -max_v_, max_v_);
  const double w = limited_v / wheel_radius_;
  return {w, w};
}

}  // namespace craft_control
