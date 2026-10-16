#include "craft_control/arm_planner.hpp"

#include <algorithm>

namespace craft_control
{

void ArmPlanner::set_limits(
  const ArmJoints & min_angle, const ArmJoints & max_angle, const ArmJoints & max_vel)
{
  min_angle_ = min_angle;
  max_angle_ = max_angle;
  max_vel_ = max_vel;
}

void ArmPlanner::set_target(const ArmJoints & target)
{
  target_ = target;
  has_target_ = true;
}

ArmJoints ArmPlanner::get_arm_pose(const ArmJoints & arm_joints, double dt) const
{
  if (!has_target_) {
    return arm_joints;  // hold current pose
  }
  ArmJoints out;
  for (std::size_t i = 0; i < kArmMotors; ++i) {
    const double goal = std::clamp(target_[i], min_angle_[i], max_angle_[i]);
    const double step = max_vel_[i] * dt;
    // ponytail: step limited around feedback (stateless); lagging motor = slower, never faster
    out[i] = std::clamp(goal, arm_joints[i] - step, arm_joints[i] + step);
  }
  return out;
}

int ArmPlanner::get_arm_status(int arm_state) const
{
  return arm_state;
}

}  // namespace craft_control
