#include "craft_control/slip_monitor.hpp"

#include <cmath>

namespace craft_control
{

bool SlipMonitor::configure(double threshold, double hold_s, double offset_gain)
{
  if (!std::isfinite(threshold) || threshold <= 0.0 || !std::isfinite(hold_s) || hold_s < 0.0 ||
    !std::isfinite(offset_gain) || offset_gain <= 0.0 || offset_gain > 1.0)
  {
    return false;
  }
  threshold_ = threshold;
  hold_s_ = hold_s;
  offset_gain_ = offset_gain;
  return true;
}

bool SlipMonitor::update(double t, double wheel_rate, std::optional<double> gyro_z, bool wheels_still)
{
  if (!gyro_z || !std::isfinite(*gyro_z) || !std::isfinite(wheel_rate)) {
    slipping_ = false;
    return slipping_;
  }
  // Still wheels but a turning chassis (pushed, sliding) is not offset: only small changes are tracked.
  if (wheels_still && (!offset_ || std::fabs(*gyro_z - *offset_) < threshold_)) {
    offset_ = offset_ ? *offset_ + offset_gain_ * (*gyro_z - *offset_) : *gyro_z;
  }
  if (offset_ && std::fabs(wheel_rate - (*gyro_z - *offset_)) > threshold_) {
    slip_at_ = t;
  }
  slipping_ = slip_at_ && t - *slip_at_ <= hold_s_;
  return slipping_;
}

}  // namespace craft_control
