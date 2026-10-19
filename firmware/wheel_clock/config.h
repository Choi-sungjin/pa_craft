#pragma once

#include <CraftMotorFirmware.h>

// IDs confirmed: wheel_r=1, clock=2, wheel_l=3.
// Confirm motor models and calibration before enabling torque.
constexpr bool kHardwareConfirmed = false;
constexpr uint32_t kDxlBaud = 1000000;
constexpr double kPositionTickRad = craft::kTwoPi / 4096.0;
constexpr double kVelocityTickRadS = 0.229 * craft::kTwoPi / 60.0;

// Protocol 2.0 X-series example calibration; IDs are per-board, not global.
// CSV channel order stays wheel_l, wheel_r, clock, independent of bus ID order.
// id, velocity mode, units/tick, zero tick, direction, min, max, accel, speed, tolerance
const craft::AxisConfig kAxes[3] = {
  {3, true, kVelocityTickRadS, 0, 1, -7.8, 7.8, 10, 0, 0.05}, // wheel_l
  {1, true, kVelocityTickRadS, 0, 1, -7.8, 7.8, 10, 0, 0.05}, // wheel_r
  {2, false, kPositionTickRad, 0, 1, -256 * craft::kTwoPi + kPositionTickRad, // clock
   256 * craft::kTwoPi - kPositionTickRad, 10, 30, 0.03490658503988659},
};
