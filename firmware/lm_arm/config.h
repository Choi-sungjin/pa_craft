#pragma once

#include <CraftMotorFirmware.h>

// Confirmed IDs: LM=1, shoulder=2, elbow=3, wrist_pitch=4, tool_roll=5, gripper=6.
// All six motors: XM430-W350, Protocol 2.0. Confirm units, physical limits
// and zero ticks before changing this flag. Host mapping flags are independent.
constexpr bool kHardwareConfirmed = true;
constexpr uint32_t kDxlBaud = 1000000;
constexpr double kPositionTickRad = craft::kTwoPi / 4096.0;
// XM430-W350 current feedback: 2.69 mA/tick (all six motors; the name predates reading the other five).
constexpr double kGripperCurrentAPerTick = 0.00269;

// Fixed order: LM, shoulder, elbow, wrist_pitch, tool_roll, gripper.
// Hardware Reverse OFF; firmware preserves raw motor signs (direction +1).
// URDF direction corrections are applied once by the host YAML.
// id, velocity mode, units/tick, zero tick, direction, min, max, accel, speed, tolerance
// Tolerance: LM and arm 0.5 degree, gripper 2 degrees; keep equal to control.yaml
// lm_/arm_/gripper_angle_tolerance (the board reports moving until each axis is inside its own).
// Measured degrees: shoulder 44.6–185, elbow 36.3–309.2 (new bracket, 2026-11-06; was 35.2–320.6), wrist_pitch
// -110.7–152.1 (2026-11-06 after the reassembly; was -16.7–250.5), tool_roll -234.1–247.8 (2026-11-02;
// was 39–187, 38–323, 70.1–340, -180–180). tool_roll keeps -175–175: one turn, so the boot unwrap stays unique.
// Gripper: -53.1–234 degrees (both closed), open at 94 degrees; limits use -48.1–229.
// LM: 136.2–1052.0 degrees (2026-11-07, timing belt re-fitted; was
// 579.6–1511.3, guide moved against the sag, 2026-11-05; was -399.4–509.2, -391.8–527.5, -141.4–808.1).
// Limits below keep a 5-degree margin inside the measured values above (approximate sync),
// rounded inward to 4 decimals to match the host's '%.4f' commands exactly (control.yaml).
const craft::AxisConfig kAxes[6] = {
  // LM spans ~2.6 turns, but the motor forgets its turn count at power-up / mode setup (0-360 deg).
  // Procedure: power up with the carriage within +-180 deg (+-19.7 mm) of the nominal pose (kBootCenters,
  // carriage at the nominal pose); the turn nearest it is taken (2026-11-05; was: pushed against the LM-motor end,
  // zero_tick -4 turns). Started elsewhere -> LM reads whole turns off; check with 'checkarm'.
  // Profile velocity 180 x 0.229 rpm = 41.2 rpm ~ 28 mm/s carriage (90 % of the 46 rpm no-load speed: stays); the
  // arm joints 50 x 0.229 rpm = 68.7 deg/s (2026-11-07, was 30 = 41.2 deg/s; 0.3x and 0.6x before that). Bags
  // 11-06/07: the shoulder peaks at 1.1 A (half the 2.3 A stall) whatever the speed (15-57 deg/s, gravity, not
  // acceleration); the measured peak runs ~1.34x the cap, ~90 deg/s at 50, under the ~125-140 deg/s a half-loaded
  // XM430-W350 reaches at 11-12 V. Do not go past 60. With the time-based profile these are the speed caps. The
  // gripper keeps 30. Acceleration 30 x 214.577 rev/min^2 (LM), 10 (arm).
  {1, false, kPositionTickRad, -16384, 1, 2.4645, 18.2735, 30, 180, 0.008726646259971648},  // 0.5 deg = 0.06 mm carriage
  {2, false, kPositionTickRad, 0, 1, 0.8657, 3.1415, 10, 50, 0.008726646259971648},
  {3, false, kPositionTickRad, 0, 1, 0.7209, 5.3092, 10, 50, 0.008726646259971648},
  {4, false, kPositionTickRad, 0, 1, -1.8448, 2.5848, 10, 50, 0.008726646259971648},
  {5, false, kPositionTickRad, 0, 1, -3.0543, 3.0543, 10, 50, 0.008726646259971648},
  {6, false, kPositionTickRad, 0, 1, -0.8395, 3.9968, 10, 30, 0.03490658503988659},
};

// Power-up turn per axis (MotorBoard::setBootCenters): LM at the nominal pose, raw 548.3 deg = 9.5696 rad, carriage
// lined up with clock 12 (demo/puck_pick/execute.py NOMINAL_RAW_DEG; 2026-11-06 LM and clock re-aligned; was 829.7, 941.1, before
// the 2026-11-06 reassembly 1030.9). The others are single-turn: NAN.
const double kBootCenters[6] = {9.5696, NAN, NAN, NAN, NAN, NAN};

// Every axis moves with a time-based profile (MotorBoard::setTimeBased, 2026-11-06): one TIME per task
// (control_master sends TaskCommand.duration_s; 0 for a GRIPPER task), the axes of a move start together. The
// profile velocities above stay the speed caps (1x). The gripper uses 70 % of the move's time (kTimeScale), so in
// a MANIPULATOR task that also opens it (sis2: to the stored puck) it is set before the arm arrives.
const bool kTimeBased[6] = {true, true, true, true, true, true};
const double kTimeScale[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 0.7};

// Axes whose arrival tolerance the host widens for one move with TOL,<0.01 deg> (MotorBoard::setLooseTolerance,
// 2026-11-07): via poses need no precision, so they take 3 deg and the next move starts without waiting out the
// slow last-degree creep (bag sis_1107_0324: 3-4 s per via). Grasp, place and the nominal pose keep 0.5 deg
// (the host sends TOL,0). Not the gripper.
const bool kLooseTolerance[6] = {true, true, true, true, true, false};

// Position I gain (XM430 default 0, P 800): the integral keeps pushing until a sagging arm joint
// reaches its goal, so the 0.5-degree tolerance (~6 ticks) is met under load. Not on the LM, nor on the gripper,
// which stalls against the puck (an integral would wind up to the current limit).
// Shoulder and wrist_pitch held the 0.5-degree residual longest (sis1 bag 2026-11-03: wrist 12.9 s, shoulder 9.0 s,
// 1 tick at a time), so they go up in small steps, one run + bag each, stopping at overshoot (bag_plot.py):
// 100 -> 150 (2026-11-03) -> 200 -> 250 ...
// 150 run (bag sis_1103_1029): wrist residual 14.3 -> 7.9 s (overshoot 0.32 -> 0.67 deg), shoulder 14.0 -> 18.6 s.
// Next: wrist 200; shoulder back to 130 (150 did not shorten its residual).
// These are the power-up values; from 2026-11-07 the host may rewrite an axis's I gain per move (IGAIN,<axis>,<gain>,
// TaskCommand.position_i_gains: demo/puck_pick/execute.py I_GAINS_BASE must equal these, I_GAINS_AGAINST_GRAVITY is
// the lower one for moves that lift the arm).
const uint16_t kPositionIGains[6] = {0, 130, 100, 200, 100, 0};
