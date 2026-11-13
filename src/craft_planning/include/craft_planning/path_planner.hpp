#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cstddef>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace craft_planning
{
// Lengths are metres; origin/pose headings are radians. Maps and obstacles use map_frame.
struct PathPlannerConfig
{
  std::string map_file;
  std::string map_frame = "map";
  // r_robot: padding while driving (half diagonal). r_approach: the padding of the whole map for the box dock, the
  // straight second path after the turn at the stage point (plan_box's face/dock line checks, dock = true). 0 = r_robot.
  double res = 0.05, block_size = 0.05, r_robot = 0.21, margin = 0.03, r_approach = 0.0;
  double lam = 0.6, w_prox = 1.0, prox_band = 0.15, arc_sample = 0.02;
  // A*+arc corner fillets: PASS needs R >= r_min (else STOP_TURN); R <= r_hi_global; bisection to r_tol.
  double r_min = 0.08, r_hi_global = 0.40, r_tol = 0.01;
  // Speed limits for PathPlan::speed_limit and the approach-length choice (A*+arc Waffle Pi values).
  double v_ref = 0.26, w_max = 1.82, a_lat_max = 0.50, a_lon_max = 0.50;
  // global_local speed profile: both wheels within max_wheel [wheel rad/s] with the turn added (wheel_radius [m],
  // wheel_separation = control's effective skid steer separation [m]); a_lon_max up from and down to v_min [m/s],
  // which it holds over the last stop_distance [m] (the arrival tolerance) and never goes below.
  double wheel_radius = 0.033, wheel_separation = 0.333, max_wheel = 7.8, v_min = 0.03, stop_distance = 0.10;
  // Straight, turn-free final stretch before a goal whose heading is required (ALIGN).
  double approach_len = 0.30;
  int occupancy_threshold = 65;
  bool goal_heading_required = false;  // A*+arc ALIGN approach along the goal heading
  std::vector<double> stl_slice_heights{0.1, 0.2, 0.3};
  std::size_t max_cells = 2000000, max_expansions = 500000;
  // "arc": A*+arc, the whole path at once (STOP_TURN segments). "global_local": 8-connected A* + line-of-sight
  // pruning, smoothed (clearance + curvature, every point within local_max_dev of the polyline) and splined once,
  // is the global path (planned again only when blocked); the robot drives a local path, the global window
  // [ego - local_behind, ego + local_ahead] cut from it unchanged with its speed profile, renewed after
  // local_update of progress.
  std::string mode = "arc";
  double local_behind = 1.0, local_ahead = 4.0, local_update = 2.0, local_max_dev = 0.20;
};

// A*+arc waypoint in map_frame. STOP_TURN: stop and turn in place to theta; PASS: fillet of radius R;
// ALIGN: goal reached straight along theta.
struct PlanWaypoint
{
  double x = 0, y = 0, theta = 0, radius = 0, v_max = 0;
  std::string mode, reason;
};

struct PathPlan
{
  bool ok = false;
  // True when changing obstacle input may make the same request feasible later.
  bool retryable = false;
  nav_msgs::msg::Path path;  // lines and fillets, 2 cm samples; orientation = travel heading
  std::vector<PlanWaypoint> waypoints;
  // Pose indices of intermediate STOP_TURN corners: the robot must stop there and turn in place.
  std::vector<std::size_t> stops;
  std::vector<double> speed_limit;  // per pose [m/s]
  std::size_t expanded = 0;         // A* states expanded
  std::string error;
  // global_local mode: the global path (0.02 m samples, start at the odometry, end at the goal); path is then
  // the local path cut from it.
  nav_msgs::msg::Path global;
  // plan_box: the goal of the chosen face (heading into the box) and that face's outward normal [rad, map].
  geometry_msgs::msg::PoseStamped goal;
  double face_normal = 0;
  std::string faces;  // plan_box: every face tried, its path length or why not (for the log)
};

// A box to drive up to (map frame): centre, yaw of one face normal [rad], side [m], and the goal standoff = base_link
// to the face it approaches [m] (LiDAR front face offset + the gap left).
struct BoxTarget
{
  double x = 0, y = 0, yaw = 0, side = 0.205, standoff = 0.29;
};

// A box being approached and the chosen face goal (PathPlan::goal of plan_box). Along the approach (entry approach_len
// behind the goal -> goal, padding wide) the box's own padding does not block: the goal sits inside it (the upper cam
// gap and the final stop are closer than the padding). The box itself, walls and obstacles still block.
struct BoxApproach
{
  BoxTarget box;
  geometry_msgs::msg::Pose goal;
};

// Arc length [m] along path of the point on it nearest to p (0 for an empty path).
// Optional arc interval disambiguates repeat visits and the return to a closed route's start.
double path_progress(const nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & p,
  double min_s = 0.0, double max_s = std::numeric_limits<double>::infinity());
double path_length(const nav_msgs::msg::Path & path);

class PathPlanner
{
public:
  // Invalid maps/configuration throw invalid_argument; the prior snapshot remains intact.
  // Completed configuration is immutable, so const queries may run concurrently.
  void configure(const PathPlannerConfig & config);
  // heading_required: end along the goal heading over approach_len (as goal_heading_required, for this call). In
  // global_local mode any heading works; arc mode rounds it to the nearest quarter turn.
  // box: the goal approaches this box (its padding does not block the approach, see BoxApproach).
  PathPlan plan(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & goal,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray(),
    bool heading_required = false, const BoxTarget * box = nullptr,
    // global_local: points the route passes on the way to the goal, in order (2026-11-12, run_tour.py: one path round
    // the arena instead of a goal per waypoint). Corners there are rounded like any other (local_max_dev).
    const std::vector<geometry_msgs::msg::Point> * via = nullptr) const;
  // Drive up to a box: one goal per face (standoff from the face along its normal, heading into the box). A face is
  // approachable only when its goal, the approach entry approach_len behind it and the line between are clear of
  // the inflated map + obstacles, the box's own padding aside (checked before A*); each approachable face is planned
  // with its heading required and the shortest global path wins (PathPlan::goal, face_normal). Any box yaw.
  // face_normal: only the face whose outward normal is nearest this [rad, map] (a correction keeps its face).
  // stage_lead > 0: the path ends at the stage point instead, approach_len + stage_lead behind the goal on the face
  // normal, an ordinary path (no heading): the robot stops there, turns square and docks straight (planning). The
  // stage point and the dock line stage -> goal must be clear as well.
  PathPlan plan_box(
    const nav_msgs::msg::Odometry & odom, const BoxTarget & box,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray(),
    const double * face_normal = nullptr, double stage_lead = 0.0,
    const std::vector<double> * allowed_normals = nullptr) const;
  // An ordinary path toward a point that is itself blocked (a box): to the free cell nearest to it, the one nearest
  // the robot among equals. No heading, no approach (plan_box takes over close by).
  PathPlan plan_toward(
    const nav_msgs::msg::Odometry & odom, double x, double y,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray()) const;
  // global_local mode: the local path for the robot at odom, the window of global around it (samples of global
  // itself) with its speed profile. Returns path + speed_limit; ok = false when the window is blocked (retryable).
  PathPlan local_path(
    const nav_msgs::msg::Path & global, const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray(),
    const BoxApproach * approach = nullptr, std::optional<double> progress = std::nullopt) const;
  // The grid every plan checks against, for display: 100 = occupied (map or obstacle point), 99 = within the padding
  // (r_robot + margin) of one, 0 = free. Obstacles as given (the box being approached already left out by planning).
  // approach: the box's padding open on the approach. dock: the map padded with r_approach (the dock after the stage
  // point), the cells only r_robot pads as 50.
  nav_msgs::msg::OccupancyGrid padding(
    const geometry_msgs::msg::PoseArray & map_obstacles, const BoxApproach * approach = nullptr, bool dock = false) const;
  // Every sample (and between samples) outside the inflated map plus obstacles, as A*+arc checks it.
  bool collision_free(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray(),
    const BoxApproach * approach = nullptr, bool dock = false) const;
  // Arc length [m] from the path's first pose to its first blocked sample (as collision_free checks it); infinity
  // when the whole path is clear, 0 for an invalid path.
  double clear_length(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray(),
    const BoxApproach * approach = nullptr, bool dock = false) const;

  struct Data;

private:
  PathPlan plan_on(
    const std::shared_ptr<const Data> & data, const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & goal, const geometry_msgs::msg::PoseArray & map_obstacles,
    bool heading_required = false, const BoxTarget * box = nullptr,
    const std::vector<geometry_msgs::msg::Point> * via = nullptr) const;
  double clear_length_on(
    const std::shared_ptr<const Data> & data, const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseArray & map_obstacles, const BoxApproach * approach) const;
  bool collision_free_on(
    const std::shared_ptr<const Data> & data, const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseArray & map_obstacles, const BoxApproach * approach) const;
  std::shared_ptr<const Data> data_;
};
}  // namespace craft_planning
