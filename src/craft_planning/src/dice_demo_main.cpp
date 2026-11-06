// dice_demo: the dice run's steps and clearance check (no robot yet; the ROS runner comes next).
// Usage: ros2 run craft_planning dice_demo [--face N] [--slots 1,2,...] [--config F] [--urdf F] [--control F]
#include "craft_planning/dice_demo.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace craft_planning::dice;

namespace {

std::string arg(int argc, char **argv, const std::string &key, const std::string &fallback) {
  for (int i = 1; i + 1 < argc; ++i) if (key == argv[i]) return argv[i + 1];
  return fallback;
}

std::string describe(const Step &s) {
  std::ostringstream o;
  o.setf(std::ios::fixed);
  if (s.kind == Step::Clock) return "slot " + std::to_string(s.slot);
  if (s.kind == Step::Gripper) {
    if (s.close) return "close";
    if (s.open_angle <= 0) return "open (full)";
    o.precision(1);
    o << "open to " << s.open_angle * 180 / M_PI << " deg";
    return o.str();
  }
  o.precision(4);
  o << "[";
  const auto &q = s.poses.back();
  for (std::size_t i = 0; i < q.size(); ++i) o << (i ? ", " : "") << q[i];
  o << "]";
  if (s.poses.size() > 1) o << " " << s.poses.size() << " moves";
  return o.str();
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const std::string share = ament_index_cpp::get_package_share_path("craft_planning").string();
    const std::string config = arg(argc, argv, "--config", share + "/config/dice_demo.yaml");
    const std::string urdf = arg(argc, argv, "--urdf", share + "/urdf/v75_robot.urdf");
    const std::string control = arg(argc, argv, "--control",
      ament_index_cpp::get_package_share_path("craft_control").string() + "/config/control.yaml");
    const int face = std::stoi(arg(argc, argv, "--face", "6"));
    const auto p = load_dice(config);
    const Kinematics k(urdf, load_control(control), p);
    std::vector<int> slots;
    std::stringstream ss(arg(argc, argv, "--slots", ""));
    for (std::string v; std::getline(ss, v, ',');) slots.push_back(std::stoi(v));
    if (slots.empty()) for (std::size_t i = 1; i <= dice_spots(face, p).size(); ++i) slots.push_back(static_cast<int>(i));

    bool taught = false;
    const auto steps = stage(build_run(k, p, face, slots, taught), nominal_pose(k, p));
    if (!taught) std::cout << "!! clock_grasp_raw_deg not taught: the clock part uses the old loading pose\n";
    for (const auto &s : steps) std::printf("  %-60s %s\n", s.name.c_str(), describe(s).c_str());
    std::cout << "\nclearance of the held puck and the jaws (worst over the joint-linear moves):\n";
    bool hit = false;
    for (const auto &c : check(k, p, face, steps)) {
      if (c.step.empty()) continue;
      const double need = c.what == "box top" ? p.release_above - 1e-4 : p.clear;
      const bool bad = c.gap < need;
      hit = hit || bad;
      std::printf("  %-16s %+6.1f mm%s  (%s)\n", c.what.c_str(), c.gap * 1000, bad ? "  <-- HIT" : "", c.step.c_str());
    }
    return hit ? 2 : 0;
  } catch (const std::exception &e) {
    std::cerr << "dice_demo: " << e.what() << "\n";
    return 1;
  }
}
