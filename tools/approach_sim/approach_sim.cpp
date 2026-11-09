// Closed-loop drive simulation with the robot's own planner and controllers (no ROS graph, ideal wheels).
//   approach_sim <cases.yaml> <out.json>
// cases.yaml: cases: [{name, map (ROS trinary yaml), start: [x, y, yaw], goal: [x, y, yaw], heading: bool,
//                      obstacles: [[x, y], ...]}]   (map frame, metres, radians)
// Planner: craft_planning::PathPlanner, planning.yaml path_planner values (global_local), goal_heading_required =
// the case's heading. Control: craft_control Lateral/Longitudinal controllers with control.yaml values, stepped
// every 20 ms like control_master::control_loop (max_accel ramp, max_wheel scaling). Planning side like
// planning_master: local path renewed after local_update of progress, the rest planned again when a local path
// that ends short of the global end is reached. Plant: wheels follow commands exactly; yaw rate from the
// controller's effective skid-steer separation.
#include "craft_control/longitudinal_controller.hpp"
#include "craft_planning/path_planner.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <limits>
#include <optional>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#ifdef SIM_PEEK
#define private public   // diagnostics only: read the controller's node / target
#endif
#include "craft_control/lateral_controller.hpp"
#ifdef SIM_PEEK
#undef private
#endif

namespace
{
constexpr double kDt = 0.02;               // control period [s]
constexpr double kCruise = 0.26;           // planning cruise_speed [m/s]
constexpr double kMaxAccel = 0.25;         // control max_accel [m/s^2]
constexpr double kMaxWheel = 7.8;          // control max_wheel [rad/s]
constexpr double kWheelRadius = 0.033;
constexpr double kTimeout = 150.0;         // [s]

craft_planning::PathPlannerConfig planner_config(const std::string & map, bool heading)
{
  craft_planning::PathPlannerConfig c;   // planning.yaml (2026-11-08)
  c.map_file = map; c.map_frame = "map"; c.occupancy_threshold = 65; c.stl_slice_heights = {0.10, 0.20, 0.30};
  c.goal_heading_required = heading;
  c.mode = "global_local"; c.local_behind = 1.0; c.local_ahead = 4.0; c.local_update = 2.0; c.local_max_dev = 0.20;
  c.block_size = 0.05; c.res = 0.05; c.r_robot = 0.243; c.margin = 0.05; c.lam = 0.60; c.w_prox = 1.0;
  c.prox_band = 0.15; c.arc_sample = 0.02; c.max_cells = 2000000; c.max_expansions = 500000;
  c.r_min = 0.08; c.r_hi_global = 0.40; c.r_tol = 0.01; c.v_ref = 0.26; c.w_max = 1.82; c.a_lat_max = 0.50;
  c.a_lon_max = 0.25; c.wheel_radius = 0.033; c.wheel_separation = 0.333; c.max_wheel = 7.8; c.v_min = 0.03;
  c.approach_len = 0.30; c.stop_distance = 0.10;   // = goal_tolerance
  return c;
}

nav_msgs::msg::Odometry odom_of(double x, double y, double th)
{
  nav_msgs::msg::Odometry o; o.header.frame_id = "map"; o.child_frame_id = "base_link";
  o.pose.pose.position.x = x; o.pose.pose.position.y = y;
  o.pose.pose.orientation.z = std::sin(th / 2); o.pose.pose.orientation.w = std::cos(th / 2);
  return o;
}

std::string path_json(const nav_msgs::msg::Path & p, double step = 0.04)
{
  std::ostringstream s; s << std::fixed << std::setprecision(3) << "[";
  double last_x = 1e9, last_y = 1e9; bool first = true;
  for (std::size_t i = 0; i < p.poses.size(); ++i) {
    const auto & q = p.poses[i].pose.position;
    if (i + 1 < p.poses.size() && std::hypot(q.x - last_x, q.y - last_y) < step) continue;
    s << (first ? "" : ",") << "[" << q.x << "," << q.y << "]"; first = false; last_x = q.x; last_y = q.y;
  }
  return s.str() + "]";
}

std::string esc(const std::string & t)
{
  std::string o; for (char ch : t) {if (ch == '"' || ch == '\\') o += '\\'; o += ch;} return o;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {std::cerr << "usage: approach_sim <cases.yaml> <out.json>\n"; return 2;}
  const auto doc = YAML::LoadFile(argv[1]);
  std::ofstream out(argv[2]);
  out << "[";
  std::map<std::string, craft_planning::PathPlanner> planners;
  bool first_case = true;
  for (const auto & cs : doc["cases"]) {
    const auto name = cs["name"].as<std::string>(), map = cs["map"].as<std::string>();
    const auto st = cs["start"].as<std::vector<double>>(), gl = cs["goal"].as<std::vector<double>>();
    const bool heading = cs["heading"].as<bool>();
    const std::string key = map + (heading ? "#h" : "#n");
    if (!planners.count(key)) {planners[key].configure(planner_config(map, heading));}
    const auto & planner = planners.at(key);
    geometry_msgs::msg::PoseArray obstacles; obstacles.header.frame_id = "map";
    if (cs["obstacles"]) {
      for (const auto & o : cs["obstacles"]) {
        geometry_msgs::msg::Pose p; p.position.x = o[0].as<double>(); p.position.y = o[1].as<double>();
        p.orientation.w = 1; obstacles.poses.push_back(p);
      }
    }
    geometry_msgs::msg::PoseStamped goal; goal.header.frame_id = "map";
    goal.pose.position.x = gl[0]; goal.pose.position.y = gl[1];
    goal.pose.orientation.z = std::sin(gl[2] / 2); goal.pose.orientation.w = std::cos(gl[2] / 2);

    double x = st[0], y = st[1], th = st[2];
    std::ostringstream rec; rec << std::fixed << std::setprecision(3);
    std::vector<std::string> globals, locals, events;
    auto plan = planner.plan(odom_of(x, y, th), goal, obstacles);
    std::string result = "planner_error", error = plan.error;
    double t = 0; int n_rec = 0;
    if (plan.ok) {
      craft_control::LateralController lat;
      lat.configure(kWheelRadius, 0.289, 0.16, 1.153, {0.12, 0.26}, {0.18, 0.25}, 0.10, 0.02, 0.5, 0.6, 1.2);
      craft_control::LongitudinalController lon; lon.configure(kWheelRadius, 0.26);
      nav_msgs::msg::Path global = plan.global;
      auto start_segment = [&](const craft_planning::PathPlan & p) {
        lat.set_target(p.path, heading, p.speed_limit);
        locals.push_back(path_json(p.path));
      };
      start_segment(plan); globals.push_back(path_json(global));
      geometry_msgs::msg::Point here; here.x = x; here.y = y;
      double local_origin = craft_planning::path_progress(global, here);
      auto ends_short = [&](const nav_msgs::msg::Path & local) {
        const auto & a = local.poses.back().pose.position, & b = global.poses.back().pose.position;
        return std::hypot(a.x - b.x, a.y - b.y) > 1e-6;
      };
      bool more = ends_short(plan.path);
      double drive_v = 0, v_meas = 0;
      result = "timeout";
      for (int step = 0; t < kTimeout; ++step, t += kDt) {
        const auto odom = odom_of(x, y, th);
        lat.set_odom(odom); lat.set_speed(v_meas);
        if (lat.is_target_reached()) {
          if (!more) {result = "reached"; break;}
          plan = planner.plan(odom, goal, obstacles);   // local path driven to its end: plan the rest
          if (!plan.ok) {result = "replan_error"; error = plan.error; break;}
          global = plan.global; globals.push_back(path_json(global)); start_segment(plan);
          here.x = x; here.y = y; local_origin = craft_planning::path_progress(global, here); more = ends_short(plan.path);
          std::ostringstream e; e << std::fixed << std::setprecision(2) << "{\"t\":" << t << ",\"kind\":\"replan\"}";
          events.push_back(e.str());
          lat.set_odom(odom);
        }
        if (lat.can_track(kCruise)) {
          lon.set_target(std::min({kCruise, lat.speed_limit(), drive_v + kMaxAccel * kDt}));
        } else {
          lon.clear_target();
        }
        if (std::getenv("SIM_DEBUG") && t > 9.1 && t < 9.6 && name == std::getenv("SIM_DEBUG")) {
          const auto & tp = lat.target_path(); const auto & e = tp.poses.back().pose;
          std::cerr << std::fixed << std::setprecision(3) << "t " << t << " pos " << x << "," << y << " th " << th
                    << " end " << e.position.x << "," << e.position.y << " endyaw " << 2 * std::atan2(e.orientation.z, e.orientation.w)
                    << " n " << tp.poses.size() << " la " << lat.lookahead() << " track " << lat.can_track(kCruise)
                    << " reached " << lat.is_target_reached()
#ifdef SIM_PEEK
                    << " node " << lat.node_ << " tgt " << lat.target_index() << " tgtxy " << tp.poses[lat.target_index()].pose.position.x << "," << tp.poses[lat.target_index()].pose.position.y
                    << " nodexy " << tp.poses[lat.node_].pose.position.x << "," << tp.poses[lat.node_].pose.position.y << " herr " << lat.heading_error(kCruise).value_or(99) << " atgoal " << lat.at_goal() << " rot " << lat.rotating_
                    << " arcleft " << lat.arc_.back() - lat.arc_[lat.node_]
#endif
                    << "\n";
        }
        const auto v = lon.compute_command();
        const double linear = kWheelRadius * (v[0] + v[1]) / 2.0;
        const auto w = lat.compute_command(linear, kCruise);
        double wl = v[0] + w[0], wr = v[1] + w[1];
        const double k = std::max({1.0, std::abs(wl) / kMaxWheel, std::abs(wr) / kMaxWheel});
        wl /= k; wr /= k; drive_v = kWheelRadius * (wl + wr) / 2.0;
        const double vel = kWheelRadius * (wl + wr) / 2.0, om = kWheelRadius * (wr - wl) / lat.effective_separation();
        x += vel * std::cos(th + om * kDt / 2) * kDt; y += vel * std::sin(th + om * kDt / 2) * kDt; th += om * kDt;
        v_meas = vel;
        if (step % 5 == 0) {rec << (n_rec++ ? "," : "") << "[" << t << "," << x << "," << y << "," << th << "," << vel << "," << om << "]";}
        // planning_master at 20 Hz: renew the local path after local_update of progress
        if (step % 3 == 0 && more) {
          here.x = x; here.y = y;
          if (craft_planning::path_progress(global, here) - local_origin >= 2.0) {
            const auto lp = planner.local_path(global, odom_of(x, y, th), obstacles);
            std::ostringstream e; e << std::fixed << std::setprecision(2) << "{\"t\":" << t << ",\"kind\":\"" << (lp.ok ? "local" : "local_fail") << "\"}";
            events.push_back(e.str());
            if (lp.ok) {start_segment(lp); local_origin = craft_planning::path_progress(global, here); more = ends_short(lp.path);}
          }
        }
      }
      rec << (n_rec++ ? "," : "") << "[" << t << "," << x << "," << y << "," << th << ",0,0]";
    }
    auto join = [](const std::vector<std::string> & v) {std::string s; for (std::size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + v[i]; return s;};
    out << (first_case ? "" : ",\n") << std::fixed << std::setprecision(4)
        << "{\"name\":\"" << esc(name) << "\",\"result\":\"" << result << "\",\"error\":\"" << esc(error) << "\",\"t\":" << t
        << ",\"final\":[" << x << "," << y << "," << th << "],\"globals\":[" << join(globals) << "],\"locals\":[" << join(locals)
        << "],\"events\":[" << join(events) << "],\"traj\":[" << rec.str() << "]}";
    first_case = false;
    std::cerr << name << ": " << result << (error.empty() ? "" : " (" + error + ")") << " t=" << t << "\n";
  }
  out << "]\n";
  return 0;
}
