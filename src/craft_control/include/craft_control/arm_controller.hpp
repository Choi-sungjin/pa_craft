#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace craft_control
{

constexpr std::size_t kArmMotors = 6;  // total OpenCR actuator channels, including gripper
constexpr std::size_t kArmIkJoints = 5;  // slide + four revolute joints in the IK chain
using ArmJoints = std::array<double, kArmMotors>;  // OpenCR motor-angle channel order
using ArmIkJointNames = std::array<std::string, kArmIkJoints>;
using ArmIkJointValues = std::array<double, kArmIkJoints>;
static_assert(kArmMotors == kArmIkJoints + 1, "OpenCR frame is five IK joints followed by gripper");

class ArmController
{
public:
  // from config/control.yaml
  bool set_angle_limits(const ArmIkJointValues & min_angle, const ArmIkJointValues & max_angle);
  bool set_actuator_mapping(
    const ArmIkJointNames & ik_joint_names,
    const ArmIkJointValues & motor_directions, const ArmIkJointValues & motor_offsets,
    const std::string & lm_linear_joint_name, double lm_pulley_radius, double lm_gear_ratio);
  bool is_valid_target(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions) const;
  // Store positions after is_valid_target() succeeds; no validation or conversion here.
  void set_target_arm_pose(const ArmIkJointValues & joint_positions);
  void clear_target();
  // current_pose: six OpenCR positions; returns the five arm commands
  ArmIkJointValues compute_arm_command(const ArmJoints & current_pose) const;
  // arm_state: arm OpenCR state (0 idle, 1 using, 2 fault)
  int get_arm_status(int arm_state) const;

private:
  ArmIkJointValues min_angle_{};
  ArmIkJointValues max_angle_{};
  ArmIkJointNames ik_joint_names_{};
  ArmIkJointValues motor_directions_{};
  ArmIkJointValues motor_offsets_{};
  bool actuator_mapping_ready_ = false;
  bool lm_conversion_ready_ = false;
  std::string lm_linear_joint_name_;
  double lm_pulley_radius_ = 0.0;
  double lm_gear_ratio_ = 0.0;
  ArmIkJointValues target_joint_positions_{};
  bool has_target_ = false;
};

}  // namespace craft_control
