#include "plan.hpp"

#include <urdf_parser/urdf_parser.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace puck {

namespace {
bool source_override() {
  return std::getenv("PUCK_PICK_DIR") || std::getenv("CRAFT_REPO");
}
fs::path share(const std::string &package) {
  return ament_index_cpp::get_package_share_directory(package);
}
// Installed defaults are immutable; user calibration and generated plans live separately.
// Never replace an existing user file, including when upgrading the installed package.
fs::path writable_defaults(const std::string &package, const std::string &directory) {
  const auto dest = state_dir() / directory;
  fs::create_directories(dest);
  for (const auto &entry : fs::directory_iterator(share(package) / directory)) {
    if (entry.is_regular_file())
      fs::copy_file(entry.path(), dest / entry.path().filename(), fs::copy_options::skip_existing);
  }
  return dest;
}
}
fs::path state_dir() {
  if (const char *d = std::getenv("CRAFT_STATE_DIR")) return fs::absolute(d);
  if (const char *d = std::getenv("XDG_DATA_HOME")) return fs::path(d) / "craft";
  return home() / ".local/share/craft";
}
fs::path here() {
  if (const char *d = std::getenv("PUCK_PICK_DIR")) return fs::path(d);
  if (const char *d = std::getenv("CRAFT_REPO")) return fs::path(d) / "demo/puck_pick";
  static const auto path = writable_defaults("craft_planning", "puck_pick");
  return path;
}
fs::path ws() { return source_override() ? here().parent_path().parent_path() : state_dir(); }
fs::path urdf_path() {
  return source_override() ? ws() / "src/craft_planning/v75_ws/src/v75_robot_description/urdf/v75_robot.urdf"
                           : share("craft_planning") / "urdf/v75_robot.urdf";
}
fs::path control_yaml() {
  if (const char *d = std::getenv("CRAFT_CONTROL_CONFIG")) return fs::path(d);
  return source_override() ? ws() / "src/craft_control/config/control.yaml"
                           : share("craft_control") / "config/control.yaml";
}
fs::path current_arm_urdf_path() {
  const auto directory = source_override() ? ws() / "src/craft_description" : share("craft_description");
  return directory / "urdf/v307_robotis_arm.urdf";
}
fs::path camera_data_dir() {
  if (const char *d = std::getenv("CRAFT_CAMERA_DATA_DIR")) return fs::path(d);
  if (const char *d = std::getenv("CRAFT_REPO")) return fs::path(d) / "tools/realsense_check";
  if (source_override()) return home() / "craft/tools/realsense_check";
  static const auto path = writable_defaults("craft_sensors", "puck_camera");
  return path;
}
fs::path camera_executable(const std::string &name) {
  if (source_override()) return camera_data_dir() / "build" / name;
  return fs::path(ament_index_cpp::get_package_prefix("craft_sensors")) / "lib/craft_sensors" / name;
}
fs::path home() {
  const char *h = std::getenv("HOME");
  return fs::path(h ? h : "/");
}

double py_mod(double a, double b) {
  double r = std::fmod(a, b);
  if (r != 0 && ((r < 0) != (b < 0))) r += b;
  return r;
}

const std::vector<std::string> Chain::NAMES = {"slide_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint",
                                               "tool_roll_joint"};

namespace {
Eigen::Matrix4d rpy_xyz(const urdf::Pose &origin) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = origin.position.x;
  T(1, 3) = origin.position.y;
  T(2, 3) = origin.position.z;
  double r, pt, y;
  origin.rotation.getRPY(r, pt, y);
  Eigen::Matrix3d Rx, Ry, Rz;
  Rx << 1, 0, 0, 0, std::cos(r), -std::sin(r), 0, std::sin(r), std::cos(r);
  Ry << std::cos(pt), 0, std::sin(pt), 0, 1, 0, -std::sin(pt), 0, std::cos(pt);
  Rz << std::cos(y), -std::sin(y), 0, std::sin(y), std::cos(y), 0, 0, 0, 1;
  T.block<3, 3>(0, 0) = Rz * Ry * Rx;
  return T;
}

Eigen::Matrix4d axis_motion(const Eigen::Vector3d &axis, const std::string &kind, double q) {
  const Eigen::Vector3d a = axis / axis.norm();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (kind == "prismatic") {
    T.block<3, 1>(0, 3) = a * q;
  } else if (kind == "revolute" || kind == "continuous") {
    Eigen::Matrix3d K;
    K << 0, -a[2], a[1], a[2], 0, -a[0], -a[1], a[0], 0;
    T.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() + std::sin(q) * K + (1 - std::cos(q)) * K * K;
  }
  return T;
}
}  // namespace

Chain::Chain(const fs::path &urdf, const std::string &tip) {
  const auto model = urdf::parseURDFFile(urdf.string());
  if (!model) throw std::runtime_error("cannot read URDF " + urdf.string());
  std::map<std::string, std::string> parent_of_child, joint_of_child;
  for (const auto &[name, parsed] : model->joints_) {
    Joint j;
    j.name = name;
    switch (parsed->type) {
      case urdf::Joint::FIXED: j.type = "fixed"; break;
      case urdf::Joint::REVOLUTE: j.type = "revolute"; break;
      case urdf::Joint::CONTINUOUS: j.type = "continuous"; break;
      case urdf::Joint::PRISMATIC: j.type = "prismatic"; break;
      default: j.type = "other"; break;
    }
    const auto &origin = parsed->parent_to_joint_origin_transform;
    j.origin = rpy_xyz(origin);
    j.xyz = {origin.position.x, origin.position.y, origin.position.z};
    j.axis = {parsed->axis.x, parsed->axis.y, parsed->axis.z};
    if (parsed->limits) {
      j.has_limit = true;
      j.lower = parsed->limits->lower;
      j.upper = parsed->limits->upper;
    }
    parent_of_child[parsed->child_link_name] = parsed->parent_link_name;
    joint_of_child[parsed->child_link_name] = j.name;
    j_[j.name] = j;
  }
  std::vector<Joint> rev;
  for (std::string link = tip; joint_of_child.count(link); link = parent_of_child[link]) rev.push_back(j_[joint_of_child[link]]);
  if (rev.empty()) throw std::runtime_error("missing TCP chain for " + tip);
  chain_.assign(rev.rbegin(), rev.rend());
}

std::pair<double, double> Chain::limits(const std::string &name) const {
  const auto &j = j_.at(name);
  if (!j.has_limit) throw std::runtime_error("joint " + name + " has no limits");
  return {j.lower, j.upper};
}

Eigen::Matrix4d Chain::fk(const JointMap &q) const {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (const auto &j : chain_) {
    T = T * j.origin;
    if (j.type != "fixed") {
      auto it = q.find(j.name);
      T = T * axis_motion(j.axis, j.type, it == q.end() ? 0.0 : it->second);
    }
  }
  return T;
}

Eigen::Vector3d Chain::origin(const std::string &name) const { return j_.at(name).xyz; }
Eigen::Vector3d Chain::axis(const std::string &name) const { return j_.at(name).axis; }

const Chain &chain() {
  static const Chain c(urdf_path());
  return c;
}

std::vector<JointMap> ik_top_down(const Chain &ch, const Eigen::Vector3d &tcp, double tilt_deg) {
  JointMap q0;
  for (const auto &n : Chain::NAMES) q0[n] = 0.0;
  const Eigen::Matrix4d base = ch.fk(q0);
  const double slide = (tcp[0] - base(0, 3)) / ch.axis("slide_joint")[0];
  const Eigen::Vector3d S = ch.origin("slide_joint") + ch.origin("shoulder_joint");
  const Eigen::Vector2d L1 = ch.origin("elbow_joint").tail<2>(), L2 = ch.origin("wrist_pitch_joint").tail<2>();
  const Eigen::Vector2d L3 = ch.origin("tool_roll_joint").tail<2>() + ch.origin("tool_tcp_joint").tail<2>();
  const double a1 = L1.norm(), a2 = L2.norm(), l3 = L3.norm();
  const double al1 = std::atan2(L1[1], L1[0]), al2 = std::atan2(L2[1], L2[0]), al3 = std::atan2(L3[1], L3[0]);
  const double phi = -M_PI / 2 - tilt_deg * (M_PI / 180.0);
  const Eigen::Vector2d W = tcp.tail<2>() - l3 * Eigen::Vector2d(std::cos(phi), std::sin(phi));
  const Eigen::Vector2d d = W - S.tail<2>();
  const double r = d.norm();
  const double c2 = (r * r - a1 * a1 - a2 * a2) / (2 * a1 * a2);
  if (std::abs(c2) > 1) return {};
  auto fit = [&](const std::string &name, double a) {
    auto [lo, hi] = ch.limits(name);
    double best = 0, best_v = 0;
    bool have = false;
    for (int k = -2; k <= 2; ++k) {
      const double c = a + k * 2 * M_PI;
      if (lo <= c && c <= hi) return c;
      const double v = std::max(lo - c, c - hi);
      if (!have || v < best_v) {best = c; best_v = v; have = true;}
    }
    return best;
  };
  std::vector<JointMap> sols;
  for (int s : {+1, -1}) {
    const double b = s * std::acos(c2);
    const double t1 = std::atan2(d[1], d[0]) - std::atan2(a2 * std::sin(b), a1 + a2 * std::cos(b));
    const double t2 = t1 + b;
    const double q1 = t1 - al1, q2 = t2 - al2 - q1, q3 = phi - al3 - q1 - q2;
    sols.push_back({{"slide_joint", slide}, {"shoulder_joint", fit("shoulder_joint", q1)},
                    {"elbow_joint", fit("elbow_joint", q2)}, {"wrist_pitch_joint", fit("wrist_pitch_joint", q3)},
                    {"tool_roll_joint", ASSUME_TOOL_ROLL}});
  }
  return sols;
}

}  // namespace puck
