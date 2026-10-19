#pragma once

#include <CraftMotorFirmware.h>

// Confirmed IDs: LM=1, shoulder=2, elbow=3, wrist_pitch=4, tool_roll=5, gripper=6.
// Example Protocol 2.0 X-series calibration. Confirm units, physical limits
// and zero ticks before changing this flag. Host mapping flags are independent.
constexpr bool kHardwareConfirmed = false;
constexpr uint32_t kDxlBaud = 1000000;
constexpr double kPositionTickRad = craft::kTwoPi / 4096.0;
// E.g. XM430: 2.69 mA/tick. Requires a current-sensing gripper model.
constexpr double kGripperCurrentAPerTick = 0.00269;

// Fixed order: LM, shoulder, elbow, wrist_pitch, tool_roll, gripper.
// id, velocity mode, units/tick, zero tick, direction, min, max, accel, speed, tolerance
// Limits below match current host placeholders, not measured hardware limits.
const craft::AxisConfig kAxes[6] = {
  {1, false, kPositionTickRad, 0, 1, 0.0, 0.3, 10, 30, 0.03490658503988659},
  {2, false, kPositionTickRad, 0, 1, -3.14, 3.14, 10, 30, 0.03490658503988659},
  {3, false, kPositionTickRad, 0, 1, -1.57, 1.57, 10, 30, 0.03490658503988659},
  {4, false, kPositionTickRad, 0, 1, -1.57, 1.57, 10, 30, 0.03490658503988659},
  {5, false, kPositionTickRad, 0, 1, -1.57, 1.57, 10, 30, 0.03490658503988659},
  {6, false, kPositionTickRad, 0, 1, -3.14, 3.14, 10, 30, 0.03490658503988659},
};
