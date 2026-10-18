#include "craft_control/arm_controller.hpp"

#include <algorithm>
#include <cmath>

namespace craft_control
{
bool ArmController::set_angle_limits(
  const ArmIkJointValues & min_angle, const ArmIkJointValues & max_angle)
{
  for (std::size_t joint = 0; joint < kArmIkJoints; ++joint) {
    if (!std::isfinite(min_angle[joint]) || !std::isfinite(max_angle[joint]) ||
      min_angle[joint] >= max_angle[joint])
    {
      return false;
    }
  }
  min_angle_ = min_angle;
  max_angle_ = max_angle;
  return true;
}

bool ArmController::set_actuator_mapping(
  const ArmIkJointNames & ik_joint_names,
  const ArmIkJointValues & motor_directions, const ArmIkJointValues & motor_offsets,
  const std::string & lm_linear_joint_name, double lm_pulley_radius, double lm_gear_ratio)
{
  // arm ik joints value check
  for (std::size_t joint = 0; joint < kArmIkJoints; ++joint) {
    if (ik_joint_names[joint].empty() ||
      !std::isfinite(motor_directions[joint]) || std::abs(motor_directions[joint]) != 1.0 ||
      !std::isfinite(motor_offsets[joint]))
    {
      return false;
    }
    for (std::size_t previous = 0; previous < joint; ++previous) {
      if (ik_joint_names[joint] == ik_joint_names[previous]) {
        return false;
      }
    }
  }

  // lm joint value check
  const auto lm_joint = std::find(ik_joint_names.begin(), ik_joint_names.end(), lm_linear_joint_name);
  const bool lm_values_are_placeholder = lm_pulley_radius == 0.0 && lm_gear_ratio == 0.0;
  const bool lm_values_are_valid = std::isfinite(lm_pulley_radius) && lm_pulley_radius > 0.0 &&
    std::isfinite(lm_gear_ratio) && lm_gear_ratio > 0.0;
  if (lm_joint == ik_joint_names.end() || (!lm_values_are_placeholder && !lm_values_are_valid))
  {
    return false;
  }

  ik_joint_names_ = ik_joint_names;
  motor_directions_ = motor_directions;
  motor_offsets_ = motor_offsets;
  lm_linear_joint_name_ = lm_linear_joint_name;
  lm_pulley_radius_ = lm_pulley_radius;
  lm_gear_ratio_ = lm_gear_ratio;
  lm_conversion_ready_ = lm_values_are_valid;
  actuator_mapping_ready_ = true;
  return true;
}

void ArmController::set_target_arm_pose(const ArmIkJointValues & joint_positions)
{
  target_joint_positions_ = joint_positions;
  has_target_ = true;
}

void ArmController::clear_target()
{
  has_target_ = false;
}

bool ArmController::is_valid_target(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions) const
{
  // Require a complete mapping and LM calibration, then validate this incoming IK result.
  if (!actuator_mapping_ready_ || !lm_conversion_ready_ ||
    joint_names.size() != kArmIkJoints || joint_positions.size() != kArmIkJoints ||
    !std::equal(joint_names.begin(), joint_names.end(), ik_joint_names_.begin()) ||
    !std::all_of(joint_positions.begin(), joint_positions.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    return false;
  }

  return true;
}

ArmIkJointValues ArmController::compute_arm_command(const ArmJoints & current_pose) const
{
  ArmIkJointValues out{};
  for (std::size_t joint = 0; joint < kArmIkJoints; ++joint) {
    out[joint] = std::clamp(current_pose[joint], min_angle_[joint], max_angle_[joint]);
  }
  if (!has_target_) {
    return out;
  }

  for (std::size_t joint = 0; joint < kArmIkJoints; ++joint) {
    double requested = motor_offsets_[joint] + motor_directions_[joint] * target_joint_positions_[joint];
    if (ik_joint_names_[joint] == lm_linear_joint_name_) {
      requested = motor_offsets_[joint] + motor_directions_[joint] *
        lm_gear_ratio_ * target_joint_positions_[joint] / lm_pulley_radius_;
    }
    out[joint] = std::clamp(requested, min_angle_[joint], max_angle_[joint]);
  }
  return out;
}

int ArmController::get_arm_status(int arm_state) const
{
  return arm_state;
}

}  // namespace craft_control
