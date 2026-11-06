#pragma once
// Dice demo (2026-11-06): take the stored pucks out of the clock one by one and stand them on the box as a dice
// face. Headless parts only (kinematics, the step list, the clearance check); dice_demo_main.cpp runs them.
// Ported from demo/puck_pick (execute.py, plan.py, reach.py, dice.py).

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace craft_planning::dice {

// MANIPULATOR target: slide [m], shoulder, elbow, wrist_pitch, tool_roll [rad], optionally gripper motor [rad].
using Pose = std::vector<double>;

// control_master parameters the demo needs (control.yaml).
struct ControlParams {
  double lm_motor_offset = 0, lm_motor_direction = 1, lm_meters_per_rad = 0, lm_min_angle = 0, lm_max_angle = 0;
  std::array<double, 4> arm_offsets{}, arm_directions{}, arm_min{}, arm_max{};
  double gripper_open_angle = 0;
};

// Demo geometry and poses (config/dice_demo.yaml).
struct DiceParams {
  // Raw motor degrees LM, shoulder, elbow, wrist_pitch, tool_roll, gripper.
  std::vector<double> nominal_raw_deg, via_raw_deg;
  std::vector<double> clock_grasp_raw_deg;      // empty: not taught (dry run uses the old loading pose)
  std::vector<double> old_load_urdf_deg;        // shoulder, elbow, wrist_pitch, tool_roll (pre-2026-11-06)
  double old_unload_depth = 0.03;               // old grasp: this far straight under old_load_urdf_deg
  double release_gripper_deg = 147.9;
  double lm_backlash = 0.002;
  double unload_above = 0.06;     // swing in this high over the clock grasp, then straight down
  double lift = 0.03;             // straight up out of the slot, tool angle kept
  double carry_above = 0.075;     // puck bottom over the box top while over the box
  double release_above = 0.005;   // puck bottom over the box top when the jaws open
  double down_step = 0.01;
  double put_roll = -1.5707963267948966;
  // Box and pips: x from the LiDAR centre (+: robot left, away from the LM motor), ahead of the LiDAR face.
  double box_x = -0.0125, box_gap = 0.01, box_w = 0.205, box_d = 0.205, box_h = 0.07;
  double pip = 0.0475;
  // Puck and gripper.
  double puck_r = 0.015, puck_h = 0.06, grasp_depth = 0.028;
  double finger_beyond_tcp = 0.027, finger_thick = 0.03, finger_half_width = 0.0175, release_open = 0.004;
  double clear = 0.005;
  // Robot model.
  double lidar_center_x = 0.128, lidar_front_y = 0.034, floor_z = 0.0;
  double hw_z_correction = -0.010;  // the real TCP sits 10 mm above the model
  double limit_margin_deg = 3.0, max_tilt_deg = 30.0;
};

struct Step {
  enum Kind { Manipulator, Gripper, Clock } kind = Manipulator;
  std::string name;
  std::vector<Pose> poses;   // Manipulator: one task per pose, in order
  bool close = false;        // Gripper: close; else open to open_angle
  double open_angle = 0;     // Gripper [motor rad]; 0: full open (gripper_open_angle)
  int slot = 0;              // Clock
  bool staged = true;        // Manipulator: elbow first, shoulder last (stage())
};

struct Clearance {
  std::string what;          // "box top" or "placed puck N"
  double gap = 1.0;          // [m], worst
  std::string step;          // where the worst gap happens
};

ControlParams load_control(const std::string &control_yaml);
DiceParams load_dice(const std::string &dice_yaml);

// The 3x3 pip grid of a face (1..6), picture rows top = far; put-down order: robot-left column first, far to near.
std::vector<std::array<double, 2>> dice_spots(int face, const DiceParams &p);

class Kinematics {
public:
  Kinematics(const std::string &urdf, const ControlParams &c, const DiceParams &p);
  ~Kinematics();
  Kinematics(Kinematics &&) noexcept;
  Kinematics &operator=(Kinematics &&) noexcept;

  // 4x4 row-major TCP pose in base_link of q (slide, shoulder, elbow, wrist_pitch, tool_roll).
  std::array<double, 16> fk(const Pose &q) const;
  std::array<double, 2> limits(int joint) const;     // URDF, 0 = slide .. 4 = tool_roll
  bool inside(const Pose &q, bool margin = true) const;  // URDF limits (revolute: limit_margin_deg inside)
  std::string motor_bad(const Pose &q) const;        // joints outside the control.yaml motor limits, "" if none
  Pose raw_to_target(const std::vector<double> &raw_deg) const;  // 6 raw motor degrees -> 6-value target
  // Analytic IK, tool tip tilt_deg forward of straight down; both elbow branches (may be outside limits).
  std::vector<Pose> ik_top_down(const std::array<double, 3> &tcp, double tilt_deg, double roll) const;
  // Smallest whole-degree tilt <= max_tilt_deg with a solution inside the limits.
  bool ik_tilted(const std::array<double, 3> &tcp, double roll, Pose &out) const;
  // Arm (shoulder, elbow, wrist_pitch, tool_roll) moved so the TCP goes dz straight down, tool angle kept.
  std::vector<double> below(const std::vector<double> &arm, double dz, double slide) const;
  // Arm with wrist_pitch and tool_roll kept, shoulder/elbow solved for the TCP dz straight up.
  std::vector<double> above(const std::vector<double> &arm, double dz, double slide) const;
  // Model TCP over spot (x, ahead) for a vertical puck standing on the box top (hw_z_correction applied).
  std::array<double, 3> grasp_at(double x, double ahead) const;
  // Pose whose held puck (GRASP_DEPTH deep) has its bottom centre h over the box top at spot (x, ahead).
  bool bottom_at(double x, double ahead, double h, double roll, Pose &out) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Raw-degree poses as 6-value targets.
Pose nominal_pose(const Kinematics &k, const DiceParams &p);
Pose via_pose(const Kinematics &k, const DiceParams &p);

// The whole run: from start (nominal) out to the via pose, the pucks, back to start. slots[i] holds puck i.
// taught: false when the clock grasp falls back to the old loading pose. Throws runtime_error with the reason
// when a pose has no solution.
std::vector<Step> build_run(const Kinematics &k, const DiceParams &p, int face, const std::vector<int> &slots,
  bool &taught);

// Elbow first, shoulder last: each staged Manipulator pose is preceded by the pose with the shoulder held.
std::vector<Step> stage(const std::vector<Step> &steps, Pose prev);

// Held puck and jaws (closed while held, full open after a release on the box) along every move, joint-linear:
// worst gap to the box top and to each placed puck.
std::vector<Clearance> check(const Kinematics &k, const DiceParams &p, int face, const std::vector<Step> &staged);

}  // namespace craft_planning::dice
