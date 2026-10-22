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
  int occupancy_threshold = 65;
  std::vector<double> stl_slice_heights{0.1, 0.2, 0.3};
  std::size_t max_cells = 2000000, max_expansions = 500000;
};

struct PathPlan
{
  bool ok = false;
  nav_msgs::msg::Path path;
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
  bool collision_free(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseArray & map_obstacles = geometry_msgs::msg::PoseArray()) const;

private:
  struct Data;
  std::shared_ptr<const Data> data_;
};
}  // namespace craft_planning
