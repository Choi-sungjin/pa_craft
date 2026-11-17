#include "teach.hpp"

#include "plan.hpp"

#include <algorithm>
#include <cmath>

namespace puck::teach {

YAML::Node control_params() {
  return YAML::LoadFile(control_yaml().string())["control_master"]["ros__parameters"];
}

std::vector<double> raw_to_target(const std::vector<double> &raw_deg) {
  const auto c = control_params();
  if (raw_deg.size() < 2) throw std::invalid_argument("raw motor values require LM and gripper");
  std::vector<double> r;
  for (double v : raw_deg) r.push_back(v * (M_PI / 180.0));
  const auto off = c["arm_joint_offsets"].as<std::vector<double>>();
  const auto dir = c["arm_joint_directions"].as<std::vector<double>>();
  std::vector<double> out{(r.front() - c["lm_motor_offset"].as<double>()) * c["lm_motor_direction"].as<double>() *
                          c["lm_meters_per_rad"].as<double>()};
  const size_t n_arm = std::min({r.size() - 2, off.size(), dir.size()});
  for (size_t i = 0; i < n_arm; ++i) out.push_back((r[i + 1] - off[i]) / dir[i]);
  out.push_back(r.back());
  return out;
}

std::vector<double> target_to_raw(const std::vector<double> &target) {
  if (target.size() != 6) throw std::invalid_argument("motor conversion requires six target values");
  const auto c = control_params();
  const auto off = c["arm_joint_offsets"].as<std::vector<double>>();
  const auto dir = c["arm_joint_directions"].as<std::vector<double>>();
  std::vector<double> raw{c["lm_motor_offset"].as<double>() +
    target[0] / (c["lm_motor_direction"].as<double>() * c["lm_meters_per_rad"].as<double>())};
  for (size_t i = 0; i < 4; ++i) raw.push_back(off.at(i) + dir.at(i) * target[i + 1]);
  raw.push_back(target[5]);
  for (double &v : raw) v *= 180.0 / M_PI;
  return raw;
}

std::vector<std::pair<double, double>> motor_limits() {
  const auto c = control_params();
  std::vector<std::pair<double, double>> out{{c["lm_min_angle"].as<double>(), c["lm_max_angle"].as<double>()}};
  const auto lo = c["arm_min_angle"].as<std::vector<double>>(), hi = c["arm_max_angle"].as<std::vector<double>>();
  for (size_t i = 0; i < std::min(lo.size(), hi.size()); ++i) out.push_back({lo[i], hi[i]});
  out.push_back({c["gripper_min_angle"].as<double>(), c["gripper_max_angle"].as<double>()});
  return out;
}

}  // namespace puck::teach
