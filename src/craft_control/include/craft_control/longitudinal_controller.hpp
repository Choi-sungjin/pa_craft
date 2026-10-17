// Longitudinal: target v -> left/right wheel speed (Lv2 base_kinematics, no calibration gains)
//   speed limit + accel limit (no jerky start/stop), wheel speed itself is tracked by OpenCR motor PID
#pragma once

#include <array>

namespace craft_control
{

class LongitudinalController
{
public:
  // wheel_radius [m], max_v [m/s], max_acc_v [m/s^2]
  void set_params(double wheel_radius, double max_v, double max_acc_v);
  void set_target_v(double v);  // [m/s]
  // dt: real elapsed time [s]
  std::array<double, 2> get_wheel_speed(double dt);  // {left, right} [rad/s]

private:
  double wheel_radius_ = 0.033;  // TurtleBot3 Waffle Pi
  double max_v_ = 0.26;
  double max_acc_v_ = 0.5;
  double target_v_ = 0.0;
  double v_ref_ = 0.0;  // accel-limited target
};

}  // namespace craft_control
