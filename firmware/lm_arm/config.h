#pragma once

#include <CraftMotorFirmware.h>

// Confirmed IDs: LM=1, shoulder=2, elbow=3, wrist_pitch=4, tool_roll=5, gripper=6.
// All six motors: XM430-W350, Protocol 2.0. Confirm units, physical limits
// and zero ticks before changing this flag. Host mapping flags are independent.
constexpr bool kHardwareConfirmed = true;
constexpr uint32_t kDxlBaud = 1000000;
constexpr double kPositionTickRad = craft::kTwoPi / 4096.0;
// XM430-W350 current feedback: 2.69 mA/tick.
constexpr double kGripperCurrentAPerTick = 0.00269;

// Fixed order: LM, shoulder, elbow, wrist_pitch, tool_roll, gripper.
// Hardware Reverse OFF; firmware preserves raw motor signs (direction +1).
// URDF direction corrections are applied once by the host YAML.
// id, velocity mode, units/tick, zero tick, direction, min, max, accel, speed, tolerance
// Tolerance: LM and arm 0.5 degree, gripper 2 degrees; keep equal to control.yaml
// lm_/arm_/gripper_angle_tolerance (the board reports moving until each axis is inside its own).
// Measured degrees: shoulder 44.6–185, elbow 35.2–320.6, wrist_pitch -16.7–250.5, tool_roll -234.1–247.8 (2026-11-02;
// was 39–187, 38–323, 70.1–340, -180–180). tool_roll keeps -175–175: one turn, so the boot unwrap stays unique.
// Gripper: -53.1–234 degrees (both closed), open at 94 degrees; limits use -48.1–229.
// LM: -399.4–509.2 degrees (re-measured 2026-11-02; was -391.8–527.5, before that -141.4–808.1).
// Limits below keep a 5-degree margin inside the measured values above (approximate sync),
// rounded inward to 4 decimals to match the host's '%.4f' commands exactly (control.yaml).
const craft::AxisConfig kAxes[6] = {
  // LM spans ~2.6 turns, but the motor forgets its turn count at power-up / mode setup (0-360 deg).
  // Procedure: power up with the carriage pushed against the LM-motor end (509.2 deg reads as 149.2),
  // so zero_tick = -360 deg (-4096 ticks) restores the calibrated frame. Started elsewhere -> LM
  // reads > 509.2 deg; check with 'checkarm' before starting control_master.
  // Profile velocity 180 x 0.229 rpm = 41.2 rpm ~ 28 mm/s carriage (was 30: 4.7 mm/s; no-load max
  // ~46 rpm); acceleration 30 x 214.577 rev/min^2 reaches it in ~0.4 s.
  {1, false, kPositionTickRad, -4096, 1, -6.8835, 8.7999, 30, 180, 0.008726646259971648},  // 0.5 deg = 0.06 mm carriage
  {2, false, kPositionTickRad, 0, 1, 0.8657, 3.1415, 10, 30, 0.008726646259971648},
  {3, false, kPositionTickRad, 0, 1, 0.7017, 5.5082, 10, 30, 0.008726646259971648},
  {4, false, kPositionTickRad, 0, 1, -0.2042, 4.2847, 10, 30, 0.008726646259971648},
  {5, false, kPositionTickRad, 0, 1, -3.0543, 3.0543, 10, 30, 0.008726646259971648},
  {6, false, kPositionTickRad, 0, 1, -0.8395, 3.9968, 10, 30, 0.03490658503988659},
};

// Position I gain (XM430 default 0, P 800): the integral keeps pushing until a sagging arm joint
// reaches its goal, so the 0.5-degree tolerance (~6 ticks) is met under load. Not on the LM, nor on the gripper,
// which stalls against the puck (an integral would wind up to the current limit).
// Shoulder and wrist_pitch held the 0.5-degree residual longest (sis1 bag 2026-11-03: wrist 12.9 s, shoulder 9.0 s,
// 1 tick at a time), so they go up in small steps, one run + bag each, stopping at overshoot (bag_plot.py):
// 100 -> 150 (2026-11-03) -> 200 -> 250 ...
// 150 run (bag sis_1103_1029): wrist residual 14.3 -> 7.9 s (overshoot 0.32 -> 0.67 deg), shoulder 14.0 -> 18.6 s.
// Next: wrist 200; shoulder back to 130 (150 did not shorten its residual).
const uint16_t kPositionIGains[6] = {0, 130, 100, 200, 100, 0};
