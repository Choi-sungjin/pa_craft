#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace craft_planning
{
// Lengths are metres; origin/pose headings are radians. Maps and obstacles use map_frame.
struct PathPlannerConfig
{
  std::string map_file;
  std::string map_frame = "map";
  double res = 0.05, block_size = 0.05, r_robot = 0.21, margin = 0.03;
  double lam = 0.6, w_prox = 1.0, prox_band = 0.15, arc_sample = 0.02;
  // A*+arc corner fillets: PASS needs R >= r_min (else STOP_TURN); R <= r_hi_global; bisection to r_tol.
  double r_min = 0.08, r_hi_global = 0.40, r_tol = 0.01;
  // Speed limits for PathPlan::speed_limit and the approach-length choice (A*+arc Waffle Pi values).
  double v_ref = 0.26, w_max = 1.82, a_lat_max = 0.50, a_lon_max = 0.50;
  // Straight, turn-free final stretch before a goal whose heading is required (ALIGN).
  double approach_len = 0.30;
  int occupancy_threshold = 65;
  bool goal_heading_required = false;  // A*+arc ALIGN approach along the goal heading
  std::vector<double> stl_slice_heights{0.1, 0.2, 0.3};
  std::size_t max_cells = 2000000, max_expansions = 500000;
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
};

class PathPlanner
{
public:
  // Invalid maps/configuration throw invalid_argument; the prior snapshot remains intact.
  // Completed configuration is immutable, so const queries may run concurrently.
  void configure(const PathPlannerConfig & config);
  PathPlan plan(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & goal,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray()) const;
  // Every sample (and between samples) outside the inflated map plus obstacles, as A*+arc checks it.
  bool collision_free(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray()) const;

private:
  struct Data;
  std::shared_ptr<const Data> data_;
};
}  // namespace craft_planning
