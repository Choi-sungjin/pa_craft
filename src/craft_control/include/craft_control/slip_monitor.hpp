#pragma once

#include <optional>

namespace craft_control
{

// Wheel slip from two yaw rates: wheel odometry's and the wheel board gyro's (raw gz, upright board).
// The gyro offset is tracked while both wheels stand still, so the raw chip offset (up to ~0.09 rad/s)
// does not count as slip. Slipping while the rates differ by more than threshold, and hold_s after.
// Inactive (never slipping) until the offset has been seen once and while the gyro is missing.
class SlipMonitor
{
public:
  bool configure(double threshold, double hold_s, double offset_gain);
  // t [s] monotonic; wheel_rate and gyro_z [rad/s].
  bool update(double t, double wheel_rate, std::optional<double> gyro_z, bool wheels_still);
  bool slipping() const {return slipping_;}
  std::optional<double> offset() const {return offset_;}

private:
  double threshold_ = 0.0, hold_s_ = 0.0, offset_gain_ = 0.0;
  std::optional<double> offset_, slip_at_;
  bool slipping_ = false;
};

}  // namespace craft_control
