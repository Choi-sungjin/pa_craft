#pragma once

#include <CraftMotorFirmware.h>

// IDs confirmed: wheel_l=1, wheel_r=2, clock=3.
// Models confirmed: wheels XM430-W210, clock XM430-W350.
// Confirm calibration before enabling torque.
constexpr bool kHardwareConfirmed = true;
constexpr uint32_t kDxlBaud = 1000000;
constexpr double kPositionTickRad = craft::kTwoPi / 4096.0;
constexpr double kVelocityTickRadS = 0.229 * craft::kTwoPi / 60.0;

// Protocol 2.0 XM430 calibration; IDs are per-board, not global.
// CSV channel order stays wheel_l, wheel_r, clock, independent of bus ID order.
// id, velocity mode, units/tick, zero tick, direction, min, max, accel, speed, tolerance
const craft::AxisConfig kAxes[3] = {
  // Wheel profile acceleration 200 x 214.577 rev/min^2 = 74.9 rad/s^2 at the wheel = 2.47 m/s^2: only a
  // ceiling while test/slip ramps on the host (was 10 = 0.124 m/s^2, which would clip every steeper slope).
  // control_master sends steps, so put a driving value back here after the slip test (2026-10-27).
  {1, true, kVelocityTickRadS, 0, 1, -7.8, 7.8, 200, 0, 0.05}, // wheel_l
  {2, true, kVelocityTickRadS, 0, -1, -7.8, 7.8, 200, 0, 0.05}, // wheel_r: raw + is backward; host + is forward
  {3, false, kPositionTickRad, 0, 1, -256 * craft::kTwoPi + kPositionTickRad, // clock
   256 * craft::kTwoPi - kPositionTickRad, 10, 30, 0.03490658503988659},
};

// OpenCR on-board IMU (2018 boards: MPU9250, ±2000 dps as set by the OpenCR IMU library).
// Same factor as the official TurtleBot3 firmware.
constexpr double kGyroRadSPerLsb = 0.0010642;  // (2000/32768) deg/s * pi/180
