// Longitudinal: target chassis velocity -> left/right wheel speed.
// DYNAMIXEL velocity PID, feedforward, and acceleration profile belong to OpenCR firmware.
#pragma once

#include <array>

namespace craft_control
{

class LongitudinalController
{
public:
  // wheel_radius [m], max_v [m/s]
  void set_params(double wheel_radius, double max_v);
  void set_target_v(double v);  // [m/s]
  void stop();
  std::array<double, 2> get_wheel_speed() const;  // {left, right} [rad/s]

private:
  // Set by ControlMaster from control.yaml, before computing wheel commands.
  double wheel_radius_ = 0.0;
  double max_v_ = 0.0;
  double target_v_ = 0.0;
};

}  // namespace craft_control
