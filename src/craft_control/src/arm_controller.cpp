#include "craft_control/arm_controller.hpp"

#include <algorithm>
#include <cmath>

namespace craft_control
{
// 모터별 min/max 범위, IK 관절 이름, 설치 방향/영점, 도착 허용 오차를 검증·저장한다.
bool ArmController::configure(
  const ArmIkJointValues & min_angle, const ArmIkJointValues & max_angle,
  const ArmIkJointNames & ik_joint_names,
  const ArmIkJointValues & motor_directions, const ArmIkJointValues & motor_offsets,
  double tolerance)
{
  for (std::size_t joint = 0; joint < kArmIkJoints; ++joint) {
    if (!std::isfinite(min_angle[joint]) || !std::isfinite(max_angle[joint]) ||
      min_angle[joint] >= max_angle[joint] || ik_joint_names[joint].empty() ||
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

  min_angle_ = min_angle;
  max_angle_ = max_angle;
  ik_joint_names_ = ik_joint_names;
  motor_directions_ = motor_directions;
  motor_offsets_ = motor_offsets;
  tolerance_ = tolerance;
  return true;
}

// set target arm motor pose; wo gripper / lm guide
void ArmController::set_target(const ArmIkJointValues & joint_positions)
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
  // Names must match the configured IK order; positions must be finite.
  if (joint_names.size() != kArmIkJoints || joint_positions.size() != kArmIkJoints ||
    !std::equal(joint_names.begin(), joint_names.end(), ik_joint_names_.begin()) ||
    !std::all_of(joint_positions.begin(), joint_positions.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    return false;
  }

  return true;
}

// 회전 관절 4개의 모델 기준 목표각을 실제 모터 기준 목표각으로 변환한다. 단위는 모두 rad.
// motor_angle = motor_offset + motor_direction * joint_angle (영점 및 설치 방향 보정).
// 변환한 목표각은 모터별 min/max 범위로 제한한다.
// 목표가 없으면 현재 모터 피드백 각도를 같은 범위로 제한해 반환하여 위치를 유지한다.
// IK는 planning에서, LM 변환은 LmController에서, PID 및 이동 프로파일은 모터 내부에서 담당한다.
ArmIkJointValues ArmController::compute_command(const ArmJoints & current_pose) const
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

std::optional<ArmIkJointValues> ArmController::get_joint_positions(
  const ArmJoints & current_pose) const
{
  ArmIkJointValues positions{};
  for (std::size_t joint = 0; joint < kArmIkJoints; ++joint) {
    positions[joint] = (current_pose[joint] - motor_offsets_[joint]) / motor_directions_[joint];
    if (!std::isfinite(positions[joint])) {
      return std::nullopt;
    }
  }
  return positions;
}

// 목표가 있고, 회전 관절 4개의 모터 피드백이 모두 목표 모터각에서 허용 오차 이내면 완료.
bool ArmController::is_target_reached(const ArmJoints & current_pose) const
{
  if (!has_target_) {
    return false;
  }
  const auto command = compute_command(current_pose);
  for (std::size_t joint = 0; joint < kArmIkJoints; ++joint) {
    if (std::abs(current_pose[joint] - command[joint]) > tolerance_) {
      return false;
    }
  }
  return true;
}

}  // namespace craft_control
