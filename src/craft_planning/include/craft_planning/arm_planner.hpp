#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace craft_planning {

struct ArmPlannerConfig {
  std::string urdf;
  std::string base_frame = "base_link";
  std::string tip_frame = "tool_tcp";
  double position_tolerance = 0.0001;
  double orientation_tolerance = 0.001;
  int max_iterations = 300;
  int max_restarts = 30;
};

struct ArmPlan {
  bool ok = false;
  sensor_msgs::msg::JointState joints;
  std::string error;
};

// Headless URDF IK. Configure before use; const calculations own their scratch data
// and can run concurrently. Goals use base_frame, translations use m, rotations rad.
class ArmPlanner {
public:
  ArmPlanner();
  ~ArmPlanner();
  ArmPlanner(ArmPlanner &&) noexcept;
  ArmPlanner &operator=(ArmPlanner &&) noexcept;
  ArmPlanner(const ArmPlanner &) = delete;
  ArmPlanner &operator=(const ArmPlanner &) = delete;

  void configure(const ArmPlannerConfig &config);
  ArmPlan plan(const geometry_msgs::msg::PoseStamped &goal,
    const sensor_msgs::msg::JointState &current) const;
  geometry_msgs::msg::PoseStamped forward(const sensor_msgs::msg::JointState &joints) const;
  const std::vector<std::string> &joint_names() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace craft_planning
