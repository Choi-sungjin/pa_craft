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
using ArmIkMotorChannels = std::array<std::size_t, kArmIkJoints>;

class ArmPlanner
{
public:
  // from config/control.yaml
  void set_angle_limits(const ArmJoints & min_angle, const ArmJoints & max_angle);
  bool set_actuator_mapping(
    const ArmIkJointNames & ik_joint_names, const ArmIkMotorChannels & motor_channels,
    const ArmIkJointValues & motor_directions, const ArmIkJointValues & motor_offsets,
    const std::string & lm_linear_joint_name, double lm_pulley_radius, double lm_gear_ratio);
  bool set_target_arm_pose(
    const std::vector<std::string> & joint_names, const std::vector<double> & joint_positions);
  // current_pose: arm OpenCR feedback
  ArmJoints compute_arm_command(const ArmJoints & current_pose) const;
  // arm_state: arm OpenCR state (0 idle, 1 using, 2 fault)
  int get_arm_status(int arm_state) const;

private:
  bool is_valid_target(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions) const;

  ArmJoints min_angle_{};
  ArmJoints max_angle_{};
  ArmIkJointNames ik_joint_names_{};
  ArmIkMotorChannels motor_channels_{};
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
