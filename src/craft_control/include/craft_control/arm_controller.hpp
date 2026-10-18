#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace craft_control
{

constexpr std::size_t kArmMotors = 4;  // shoulder, elbow, wrist_pitch, tool_roll only
constexpr std::size_t kArmIkJoints = kArmMotors;
using ArmJoints = std::array<double, kArmMotors>;  // rotational motor feedback [rad]
using ArmIkJointNames = std::array<std::string, kArmIkJoints>;
using ArmIkJointValues = std::array<double, kArmIkJoints>;

class ArmController
{
public:
  // from config/control.yaml; tolerance [rad] is validated positive by ControlMaster.
  bool configure(
    const ArmIkJointValues & min_angle, const ArmIkJointValues & max_angle,
    const ArmIkJointNames & ik_joint_names,
    const ArmIkJointValues & motor_directions, const ArmIkJointValues & motor_offsets,
    double tolerance);
  bool is_valid_target(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions) const;
  // Store positions after is_valid_target() succeeds; no validation or conversion here.
  void set_target(const ArmIkJointValues & joint_positions);
  void clear_target();
  // Four rotational motor positions in; four rotational motor commands out. LM/gripper are separate.
  ArmIkJointValues compute_command(const ArmJoints & current_pose) const;
  // All four motor feedback angles within tolerance of the stored target.
  bool is_target_reached(const ArmJoints & current_pose) const;

private:
  ArmIkJointValues min_angle_{};
  ArmIkJointValues max_angle_{};
  double tolerance_ = 0.0;
  ArmIkJointNames ik_joint_names_{};
  ArmIkJointValues motor_directions_{};
  ArmIkJointValues motor_offsets_{};
  ArmIkJointValues target_joint_positions_{};
  bool has_target_ = false;
};

}  // namespace craft_control
