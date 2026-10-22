#pragma once

#include "craft_planning/arm_planner.hpp"
#include "craft_planning/path_planner.hpp"
#include "craft_planning/state_machine.hpp"

#include <map>
#include <string>
#include <vector>

namespace rclcpp {class Node;}

namespace craft_planning {

struct PlanningStep {
  std::string id;
  StepKind kind = StepKind::Wait;
  geometry_msgs::msg::PoseStamped goal;
  // For an observed source, goal.position is an offset expressed in goal.frame_id.
  std::string source = "fixed";
  int class_id = -1;
  bool close = false;
  int slot = 1;
  double duration_s = 0.0;
};

struct PlanningConfig {
  PathPlannerConfig path;
  ArmPlannerConfig arm;
  StateMachineConfig fsm;
  double loop_hz = 20.0;
  double odom_timeout_s = 0.5;
  double joint_timeout_s = 0.5;
  double observation_timeout_s = 0.5;
  double obstacle_timeout_s = 0.5;
  double cruise_speed = 0.12;
  double goal_tolerance = 0.03;
  double heading_tolerance = 0.5;
  double observation_min_confidence = 0.5;
  bool require_lidar = true;
  bool goal_heading_required = false;
  std::string joint_state_topic = "/control/arm_joints";
  std::map<std::string, std::vector<PlanningStep>> missions;
};

// Invalid files, unknown fields and invalid values throw invalid_argument.
std::map<std::string, std::vector<PlanningStep>> parse_missions(
  const std::string &filename, const std::string &map_frame, const std::string &base_frame);

// All application parameters are required in the supplied ROS parameter YAML.
// Relative map, URDF and mission paths resolve against the installed package share.
PlanningConfig load_planning_config(rclcpp::Node &node, const std::string &share_dir);

}  // namespace craft_planning
