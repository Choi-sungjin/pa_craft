#pragma once

#include "craft_planning/arm_planner.hpp"
#include "craft_planning/grasp_region.hpp"
#include "craft_planning/path_planner.hpp"
#include "craft_planning/state_machine.hpp"

#include <array>
#include <map>
#include <string>
#include <vector>

namespace rclcpp {class Node;}

namespace craft_planning {

struct PlanningStep {
  std::string id;
  uint64_t task_id = 0;
  StepKind kind = StepKind::Wait;
  geometry_msgs::msg::PoseStamped goal;
  // For an observed source, goal.position is an offset expressed in goal.frame_id.
  std::string source = "fixed";
  int class_id = -1;
  bool close = false;
  int slot = 1;
  double duration_s = 0.0;
  std::string box;  // Navigate to this box_approach box (/planning/approach_box) instead of goal
  bool box_near_entry = false;  // explicit held near-box entry, without stage motion
};

// Driving up to a box (/planning/approach_box <name>): PathPlanner::plan_box with the box from the list, the goal
// gap from the LiDAR front face to the face, at speed.
struct BoxApproachConfig {
  bool camera_enabled = false;
  // LiDAR only builds the dock line; the dock tracks it and the upper cam is only the stop trigger at
  // camera_final_gap (no LiDAR gap stop/handoff, no camera speed control; user, 2026-11-09).
  // false: LiDAR dock to handoff_gap, stop, camera distance-controlled dock.
  bool camera_only_longitudinal = false;
  double camera_slow_gap = 0.100;   // camera_only_longitudinal: camera_speed from this gap (LiDAR line) to the end
  double camera_fast_speed = 0.10;  // [m/s] camera_only_longitudinal: the dock before camera_slow_gap
  double camera_final_gap = 0.010, camera_gap_tolerance = 0.003, handoff_gap = 0.100;
  double source_gap_diff = 0.020, camera_timeout = 0.300, lidar_observation_timeout = 0.200, camera_speed = 0.005;
  double camera_uncertainty_max = 0.005, camera_far_uncertainty_max = 0.010,
    camera_near_gap = 0.030, lidar_uncertainty_max = 0.010;
  double observation_initial_timeout = 3.0;
  double camera_calibrated_gap_min = 0.010, camera_calibrated_gap_max = 0.080;
  double camera_operational_gap_max = 0.200;
  double front_lidar_x = 0.155, front_lidar_fov_deg = 180.0;
  int camera_confirm_count = 3;
  double speed = 0.20;          // [m/s]
  double gap = 0.08;            // [m] LiDAR front face -> box face at the goal
  double front_face_x = 0.165;  // [m] axle midpoint to LiDAR front face, measured 2026-11-09
  double side = 0.205;          // [m]
  double max_offset = 0.5;       // [m] a perceived box farther than this from the expected position is another box
  // At the face goal: stand still settle_s, perceive again, check the alignment; out of tolerance, back out
  // reverse_distance along the approach at reverse_speed and approach the same face again, at most max_corrections.
  double settle_s = 0.5;             // [s]
  double lateral_tolerance = 0.005;  // [m]
  double yaw_tolerance = 0.0175;     // [rad]
  int max_corrections = 3;
  double reverse_distance = 0.30;    // [m]
  double reverse_min_distance = 0.10;  // [m] blocked behind: the longest clear distance down to this
  double stage_lead = 0.20;            // [m] the stage point is this far behind the approach entry
  double stage_goal_tolerance = 0.03;  // [m] the stage drive's arrival distance (the dock starts near the normal)
  double dock_speed = 0.05;            // [m/s] the straight dock from the stage point (or a back-out) to the goal
  double dock_lookahead = 0.020;       // [m] target distance from the front reference during dock tracking
  double dock_goal_tolerance = 0.01;   // [m] the dock's arrival distance (yaw_tolerance its fine alignment)
  double reverse_speed = 0.10;       // [m/s]
  std::map<std::string, std::array<double, 3>> boxes;  // name -> x, y [m, map], yaw of a face normal [rad]
};

struct PlanningConfig {
  PathPlannerConfig path;
  ArmPlannerConfig arm;
  GraspRegionConfig grasp;
  BoxApproachConfig box;
  StateMachineConfig fsm;
  double loop_hz = 20.0;
  double replan_period_s = 0.2;
  double block_wait_s = 2.0;    // blocked path: pause this long before replanning
  double min_segment_m = 0.15;  // STOP_TURN corners closer than this to the start are not segment ends
  double odom_timeout_s = 0.5;
  double localization_observation_timeout_s = 0.3;
  bool require_lidar_localization = false;
  double joint_timeout_s = 0.5;
  double observation_timeout_s = 0.5;
  double obstacle_timeout_s = 0.5;
  double box_detection_timeout_s = 1.0;  // /perception/box/pose older than this is not used
  double cruise_speed = 0.12;
  double goal_tolerance = 0.10;
  double observation_min_confidence = 0.5;
  bool require_lidar = true;
  bool require_arm_joints = true;  // false: Ready without arm joint feedback (drive-only, arm board off)
  bool goal_heading_required = false;
  std::string joint_state_topic = "/control/arm_joints";
  std::map<std::string, std::vector<PlanningStep>> missions;
  std::map<int, std::string> mission_by_tag;
};

// Invalid files, unknown fields and invalid values throw invalid_argument.
std::map<std::string, std::vector<PlanningStep>> parse_missions(
  const std::string &filename, const std::string &map_frame, const std::string &base_frame);

// All application parameters are required in the supplied ROS parameter YAML.
// Relative map, URDF and mission paths resolve against the installed package share.
PlanningConfig load_planning_config(rclcpp::Node &node, const std::string &share_dir);

}  // namespace craft_planning
