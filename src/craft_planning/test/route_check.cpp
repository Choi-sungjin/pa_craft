// Offline look at a route through via points on a real map, as planning_master plans it for run_tour.py
// (/planning/target_base_route). Built with the tests (colcon build --packages-select craft_planning), not installed:
//   ~/craft/build/craft_planning/route_check ~/maps/arena5_legend_opt_arena1cm.yaml out.csv X Y  X1 Y1 [X2 Y2 ...]
// X Y: the robot; then the via points in order, the last pair is the goal [m, map]. Writes the global path
// (x,y per line) and prints its length, the smallest x and the sharpest turn radius.
// The path_planner values below are planning.yaml's (2026-11-12): change both.
#include "craft_planning/path_planner.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

int main(int argc, char ** argv)
{
  if (argc < 7 || argc % 2 == 0) {std::fprintf(stderr, "route_check map.yaml out.csv X Y X1 Y1 [X2 Y2 ...]\n"); return 2;}
  craft_planning::PathPlannerConfig c;
  c.map_file = argv[1]; c.mode = "global_local";
  c.res = c.block_size = 0.05; c.r_robot = 0.23; c.r_approach = 0.16; c.margin = 0.0; c.lam = 0.60;
  c.a_lon_max = 0.25; c.max_expansions = 500000;
  craft_planning::PathPlanner planner;
  planner.configure(c);
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map"; odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.position.x = std::atof(argv[3]); odom.pose.pose.position.y = std::atof(argv[4]);
  std::vector<geometry_msgs::msg::Point> via;
  for (int i = 5; i + 1 < argc; i += 2) {
    geometry_msgs::msg::Point p; p.x = std::atof(argv[i]); p.y = std::atof(argv[i + 1]); via.push_back(p);
  }
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map"; goal.pose.orientation.w = 1.0; goal.pose.position = via.back(); via.pop_back();
  const auto plan = planner.plan(odom, goal, {}, false, nullptr, &via);
  if (!plan.ok) {std::fprintf(stderr, "no route: %s\n", plan.error.c_str()); return 1;}
  std::ofstream out(argv[2]);
  double length = 0, min_x = 1e9, min_radius = 1e9;
  const auto & p = plan.global.poses;
  for (std::size_t i = 0; i < p.size(); ++i) {
    const auto & q = p[i].pose.position;
    out << q.x << ',' << q.y << '\n';
    min_x = std::min(min_x, q.x);
    if (i == 0) {continue;}
    const auto & a = p[i - 1].pose.position;
    length += std::hypot(q.x - a.x, q.y - a.y);
    if (i + 10 < p.size() && i >= 10) {  // turn radius over 20 samples (0.4 m)
      const auto & b = p[i - 10].pose.position, & e = p[i + 10].pose.position;
      const double turn = std::remainder(std::atan2(e.y - q.y, e.x - q.x) - std::atan2(q.y - b.y, q.x - b.x), 2 * M_PI);
      const double chord = std::hypot(e.x - q.x, e.y - q.y);
      if (std::abs(turn) > 1e-3) {min_radius = std::min(min_radius, chord / std::abs(turn));}
    }
  }
  std::printf("%zu poses, %.2f m, smallest x %.2f m, sharpest turn radius about %.2f m, clear of the padding: %s\n",
              p.size(), length, min_x, min_radius, planner.collision_free(plan.global) ? "yes" : "NO");
  return 0;
}
