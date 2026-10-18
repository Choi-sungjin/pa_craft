// Longitudinal: target chassis velocity -> left/right wheel speed.
// DYNAMIXEL velocity PID, feedforward, and acceleration profile belong to OpenCR firmware.
#pragma once

#include <array>

namespace craft_control
{

class LongitudinalController
{
public:
  // wheel_radius [m], max_v [m/s]; ControlMaster validates both as positive.
  void configure(double wheel_radius, double max_v);
  bool is_valid_target(double v) const;  // finite [m/s]
  void set_target(double v);  // [m/s], after is_valid_target() succeeds
  void clear_target();
  std::array<double, 2> compute_command() const;  // {left, right} [rad/s]

private:
  // Set by ControlMaster from control.yaml, before computing wheel commands.
  double wheel_radius_ = 0.0;
  double max_v_ = 0.0;
  double target_v_ = 0.0;
};

}  // namespace craft_control
