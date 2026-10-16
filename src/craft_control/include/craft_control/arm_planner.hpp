#pragma once

#include <array>
#include <cstddef>

namespace craft_control
{

constexpr std::size_t kArmMotors = 6;  // arm_j0..arm_j4 + LM guide
using ArmJoints = std::array<double, kArmMotors>;  // index = arm OpenCR motor order, 5 = LM guide

class ArmPlanner
{
public:
  // from config/control.yaml
  void set_limits(const ArmJoints & min_angle, const ArmJoints & max_angle, const ArmJoints & max_vel);
  void set_target(const ArmJoints & target);
  // arm_joints: arm OpenCR feedback, dt: control period [s]
  ArmJoints get_arm_pose(const ArmJoints & arm_joints, double dt) const;
  // arm_state: arm OpenCR state (0 idle, 1 using, 2 fault)
  int get_arm_status(int arm_state) const;

private:
  ArmJoints min_angle_{};
  ArmJoints max_angle_{};
  ArmJoints max_vel_{};
  ArmJoints target_{};
  bool has_target_ = false;
};

}  // namespace craft_control
