#pragma once

#include <CraftMotorFirmware.h>

// IDs (4WD, 2026-10-30): wheel_l=1, wheel_r=2, wheel_rear_l=3, wheel_rear_r=4, clock=5 (was 3).
// Models confirmed: wheels XM430-W210, clock XM430-W350.
// Confirm calibration before enabling torque.
constexpr bool kHardwareConfirmed = true;
constexpr uint32_t kDxlBaud = 1000000;
constexpr double kCurrentAPerTick = 0.00269;  // XM430 PRESENT_CURRENT unit: the D line peak currents
constexpr double kPositionTickRad = craft::kTwoPi / 4096.0;
constexpr double kVelocityTickRadS = 0.229 * craft::kTwoPi / 60.0;

// Protocol 2.0 XM430 calibration; IDs are per-board, not global.
// CSV channel order stays wheel_l, wheel_r, clock, independent of bus ID order.
// id, velocity mode, units/tick, zero tick, direction, min, max, accel, speed, tolerance
const craft::AxisConfig kAxes[3] = {
  // Wheel profile acceleration at its maximum (32767, 2026-11-08; was 10 = 0.124 m/s^2): steps are followed at once.
  // planning's speed profile (a_lon_max) and control's max_accel shape driving instead. In-place turns still start
  // and stop as steps (200 made them slip, 2026-10-29): watch for that.
  {1, true, kVelocityTickRadS, 0, 1, -7.8, 7.8, 32767, 0, 0.05}, // wheel_l
  {2, true, kVelocityTickRadS, 0, -1, -7.8, 7.8, 32767, 0, 0.05}, // wheel_r: raw + is backward; host + is forward
  {5, false, kPositionTickRad, 0, 1, -256 * craft::kTwoPi + kPositionTickRad, // clock
   256 * craft::kTwoPi - kPositionTickRad, 10, 30, 0.03490658503988659},
};

// Rear wheels copy their side's channel (not CSV channels; wheel_l/wheel_r feedback is the side mean).
// Directions assume they are mounted like the middle pair; confirm by lifting the robot.
// false = front-wheel drive only (2WD comparison): rear motors get torque off and roll freely,
// wheel_l/wheel_r feedback is the front motor alone. resetwheel after changing it.
constexpr bool kRearWheelsDrive = true;
// id, leader channel, direction
const craft::FollowerConfig kFollowers[2] = {
  {3, 0, 1},   // wheel_rear_l follows wheel_l
  {4, 1, -1},  // wheel_rear_r follows wheel_r
};
