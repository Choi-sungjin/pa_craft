#pragma once

#include "craft_planning/arm_planner.hpp"
#include "craft_planning/grasp_region.hpp"
#include "craft_planning/path_planner.hpp"
#include "craft_planning/state_machine.hpp"

#include <craft_interfaces/msg/task_command.hpp>
#include <optional>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace rclcpp {class Node;}

namespace craft_planning {

// Map x/y and dock heading. Positive lateral_offset is LEFT in the dock's robot frame.
// Always derive from the measured box, so repeated path updates never accumulate the offset.
std::array<double, 3> box_dock_goal(const BoxTarget &box, double face_normal, double lateral_offset);

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
  double dock_lateral = 0;  // [m] puck re-dock: added to box_approach.lateral_offset (+y left in dock heading)
  // Navigate (/planning/target_base_route): points the route passes before goal, in order (PathPlanner::plan via).
  std::vector<geometry_msgs::msg::Point> via;
  // /planning/leave_box: back straight out (the robot keeps its heading) box_approach.reverse_distance, down to
  // reverse_min_distance when the end is not free. Docked 20 mm from a face the robot stands in the box's padding,
  // where no route can start ("Start is in an obstacle"); this undoes the dock line first. leave_from: box name, log only.
  bool leave = false;
  std::string leave_from;
  double leave_distance = 0.0;  // explicit distance: never shorten a regrasp back-out
  uint64_t pick_request_id = 0;
  std::optional<craft_interfaces::msg::TaskCommand> taught_command;
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
  bool dock_wheel_odometry = true;  // the dock tracks the map pose at its start + wheel odometry (no localization jumps)
  double lidar_window_s = 1.0;     // [s] LiDAR scans standing at the stage point whose median makes the dock line
  bool yaw_from_map = false;       // perceived box: map yaw (quarter turns), LiDAR normal distance, face position only when its ends were seen
  // Boxes that always use the map yaw, whatever yaw_from_map says (L1-L3 stand square: user, 2026-11-12).
  std::vector<std::string> square_boxes;
  double dock_straight = 0.25;     // [m] camera_only_longitudinal: the dock path is on the face normal this far before the goal
  double dock_extend = 0.15;       // [m] camera_only_longitudinal: the dock path goes on this far past the goal (into the box)
  double dock_overrun = 0.005;     // [m] camera_only_longitudinal: stop this far past the goal by odometry if the camera has not
  double camera_final_gap = 0.010, camera_gap_tolerance = 0.003, handoff_gap = 0.100;
  double source_gap_diff = 0.020, camera_timeout = 0.300, lidar_observation_timeout = 0.200, camera_speed = 0.005;
  double camera_uncertainty_max = 0.005, camera_far_uncertainty_max = 0.010,
    camera_near_gap = 0.030, lidar_uncertainty_max = 0.010;
  double observation_initial_timeout = 3.0;
  double camera_calibrated_gap_min = 0.010, camera_calibrated_gap_max = 0.080;
  double camera_operational_gap_max = 0.200;
  double front_lidar_x = 0.203, front_lidar_fov_deg = 180.0;
  int camera_confirm_count = 3;
  // After the dock is accepted: turn in place by the remaining heading error to the box face (upper cam, mean of the
  // confirm frames) when it is above dock_final_yaw_tolerance_deg and not above dock_final_yaw_max_deg (larger =
  // a bad measurement: left as it is). The turn runs at control's slow goal turn (wheel_goal_rotate_*).
  double dock_final_yaw_tolerance_deg = 0.5, dock_final_yaw_max_deg = 10.0;
  double speed = 0.20;          // [m/s]
  double gap = 0.08;            // [m] LiDAR front face -> box face at the goal
  double front_face_x = 0.213;  // [m] base_link to LiDAR front face (0.165 axle midpoint + 0.048, base_link moved back 2026-11-15)
  double side = 0.205;          // [m]
  double max_offset = 0.5;       // [m] a perceived box farther than this from the expected position is another box
  // At the face goal: stand still settle_s, perceive again, check the alignment; out of tolerance, back out
  // reverse_distance along the approach at reverse_speed and approach the same face again, at most max_corrections.
  double settle_s = 0.5;             // [s]
  double lateral_tolerance = 0.005;  // [m]
  double lateral_offset = 0.0015;   // [m] robot +y (left), user 2026-11-16; all box faces
  double yaw_tolerance = 0.0175;     // [rad]
  int max_corrections = 3;
  double reverse_distance = 0.30;    // [m]
  double reverse_min_distance = 0.10;  // [m] blocked behind: the longest clear distance down to this
  double stage_lead = 0.20;            // [m] the stage point is this far behind the approach entry
  double stage_goal_tolerance = 0.03;  // [m] the stage drive's arrival distance (the dock starts near the normal)
  double dock_speed = 0.05;            // [m/s] the straight dock from the stage point (or a back-out) to the goal
  double stage_final_m = 0.30, stage_final_speed = 0.05, stage_final_lookahead = 0.08;  // the last of the stage drive
  double stage_capture_m = 0.15, stage_capture_s = 2.0;
  double stage_still_s = 2.0;  // turned in place, within the heading tolerance and still this long: go on  // stuck off the stage point: turn in place instead
  double ignore_radius = 0.25;  // [m] LiDAR obstacle points this close to the box being approached are dropped
  double reorient_deg = 10.0;   // [deg] a box measured turned more from the face approached: plan again
  // The robot at the stage point further than this off the measured box's dock line (sideways [m], heading [deg]):
  // to the measured box's stage point, square to it, and the LiDAR measures again there (once per box step).
  double restage_lateral_m = 0.04, restage_yaw_deg = 2.0;
  // One stage (user, 2026-11-14): the LiDAR looks for the box all the way to the stage point; seen in splice_samples
  // scans in a row, the stage drive ends on an L (arc of dock_corner_radius) onto the measured box's face normal at its
  // stage point, square to the face; there the robot stops and measures standing, the dock as before.
  bool one_shot = false;
  int splice_samples = 3;
  double dock_corner_radius = 0.30;  // [m]
  double dock_update_period_s = 0.2;  // [s] how often the dock line is made again (LiDAR track, camera_lateral)
  // The dock line from the LiDAR until this far [m] from the face goal (odometry), then held and the upper cam
  // stops it (user 2026-11-15). 0 = the upper cam from the start of the dock, the path held all the way.
  double lidar_track_until_m = 0.28;
  // The dock in the upper cam's range: the dock line from the upper cam (median of camera_track_samples frames
  // within camera_track_max_yaw_deg of square, at most camera_track_max_jump from the box the path goes by).
  bool camera_lateral = false;
  int camera_track_samples = 3;
  double camera_track_max_yaw_deg = 10.0, camera_track_max_jump = 0.03;
  double final_yaw_tolerance = 0.0698;  // [rad] (4 deg) the upper cam check at the end of a dock
  double dock_reference_x = 0.0;  // [m] base_link x of the point the dock tracks (0 = base_link, front_face_x = LiDAR front)
  double dock_lookahead = 0.020;       // [m] target distance from the front reference during dock tracking
  double dock_goal_tolerance = 0.01;   // [m] the dock's arrival distance (yaw_tolerance its fine alignment)
  double reverse_speed = 0.10;       // [m/s]
  std::map<std::string, std::array<double, 3>> boxes;  // name -> x, y [m, map], yaw of a face normal [rad]
  std::map<std::string, std::vector<double>> box_faces;  // name -> outward normals [rad, map] it can be approached from
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
  double dock_block_wait_s = 10.0;  // blocked dock / back-out line (no replan there): fail after this long
  double block_stop_distance = 0.15;  // [m] a block further ahead than this on the path: keep driving up to it
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
