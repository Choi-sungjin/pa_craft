#include "craft_control/arm_controller.hpp"

#include <algorithm>
#include <cmath>

namespace craft_control
{
// set angle min/max limit
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

// set actuator mappin
bool ArmController::set_actuator_mapping(
  const ArmIkJointNames & ik_joint_names,
  const ArmIkJointValues & motor_directions, const ArmIkJointValues & motor_offsets)
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

  ik_joint_names_ = ik_joint_names;
  motor_directions_ = motor_directions;
  motor_offsets_ = motor_offsets;
  actuator_mapping_ready_ = true;
  return true;
}

// set target arm motor pose; wo gripper / lm guide
void ArmController::set_target_arm_pose(const ArmIkJointValues & joint_positions)
{
  target_joint_positions_ = joint_positions;
  has_target_ = true;
}

// clear target arm motor pose
void ArmController::clear_target()
{
  has_target_ = false;
}

// validate target arm motor pose
bool ArmController::is_valid_target(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions) const
{
  // Require the rotational motor mapping, then validate this incoming joint target.
  if (!actuator_mapping_ready_ ||
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
    const double requested = motor_offsets_[joint] + motor_directions_[joint] * target_joint_positions_[joint];
    out[joint] = std::clamp(requested, min_angle_[joint], max_angle_[joint]);
  }
  return out;
}

int ArmController::get_arm_status(int arm_state) const
{
  return arm_state;
}

}  // namespace craft_control
