#include "craft_control/longitudinal_controller.hpp"

namespace craft_control
{

void LongitudinalController::set_target_v(double v)
{
  target_v_ = v;
}

std::array<double, 2> LongitudinalController::get_wheel_speed() const
{
  // TODO: speed control
  return {target_v_, target_v_};
}

}  // namespace craft_control
