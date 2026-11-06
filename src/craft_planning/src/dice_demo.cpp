// Pinocchio must precede headers that use Boost (including ROS headers).
#include "v75_arm_ik/arm_ik_core.hpp"
#include "craft_planning/dice_demo.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>

namespace craft_planning::dice {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad = kPi / 180.0;
const std::vector<std::string> kJoints = {
  "slide_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"};

template<std::size_t N>
std::array<double, N> array_of(const YAML::Node &n, const char *key) {
  const auto v = n[key].as<std::vector<double>>();
  if (v.size() < N) throw std::invalid_argument(std::string(key) + ": needs " + std::to_string(N) + " values");
  std::array<double, N> out{};
  std::copy_n(v.begin(), N, out.begin());
  return out;
}

std::string fmt(double v, int digits = 1) {
  std::ostringstream s;
  s.setf(std::ios::fixed);
  s.precision(digits);
  s << v;
  return s.str();
}


}  // namespace

ControlParams load_control(const std::string &control_yaml) {
  const YAML::Node n = YAML::LoadFile(control_yaml)["control_master"]["ros__parameters"];
  if (!n) throw std::invalid_argument(control_yaml + ": no control_master.ros__parameters");
  ControlParams c;
  c.lm_motor_offset = n["lm_motor_offset"].as<double>();
  c.lm_motor_direction = n["lm_motor_direction"].as<double>();
  c.lm_meters_per_rad = n["lm_meters_per_rad"].as<double>();
  c.lm_min_angle = n["lm_min_angle"].as<double>();
  c.lm_max_angle = n["lm_max_angle"].as<double>();
  c.arm_offsets = array_of<4>(n, "arm_joint_offsets");
  c.arm_directions = array_of<4>(n, "arm_joint_directions");
  c.arm_min = array_of<4>(n, "arm_min_angle");
  c.arm_max = array_of<4>(n, "arm_max_angle");
  c.gripper_open_angle = n["gripper_open_angle"].as<double>();
  return c;
}

DiceParams load_dice(const std::string &dice_yaml) {
  const YAML::Node n = YAML::LoadFile(dice_yaml)["dice_demo"];
  if (!n) throw std::invalid_argument(dice_yaml + ": no dice_demo section");
  DiceParams p;
  std::map<std::string, double *> scalars = {
    {"old_unload_depth", &p.old_unload_depth}, {"release_gripper_deg", &p.release_gripper_deg},
    {"lm_backlash", &p.lm_backlash}, {"unload_above", &p.unload_above}, {"lift", &p.lift},
    {"carry_above", &p.carry_above}, {"release_above", &p.release_above}, {"down_step", &p.down_step},
    {"put_roll_deg", nullptr}, {"box_x", &p.box_x}, {"box_gap", &p.box_gap}, {"box_w", &p.box_w},
    {"box_d", &p.box_d}, {"box_h", &p.box_h}, {"pip", &p.pip}, {"puck_r", &p.puck_r}, {"puck_h", &p.puck_h},
    {"grasp_depth", &p.grasp_depth}, {"finger_beyond_tcp", &p.finger_beyond_tcp},
    {"finger_thick", &p.finger_thick}, {"finger_half_width", &p.finger_half_width},
    {"release_open", &p.release_open}, {"clear", &p.clear}, {"lidar_center_x", &p.lidar_center_x},
    {"lidar_front_y", &p.lidar_front_y}, {"floor_z", &p.floor_z}, {"hw_z_correction", &p.hw_z_correction},
    {"limit_margin_deg", &p.limit_margin_deg}, {"max_tilt_deg", &p.max_tilt_deg}};
  for (const auto &kv : n) {
    const auto key = kv.first.as<std::string>();
    if (key == "nominal_raw_deg") p.nominal_raw_deg = kv.second.as<std::vector<double>>();
    else if (key == "via_raw_deg") p.via_raw_deg = kv.second.as<std::vector<double>>();
    else if (key == "clock_grasp_raw_deg") {
      if (!kv.second.IsNull()) p.clock_grasp_raw_deg = kv.second.as<std::vector<double>>();
    } else if (key == "old_load_urdf_deg") p.old_load_urdf_deg = kv.second.as<std::vector<double>>();
    else if (key == "put_roll_deg") p.put_roll = kv.second.as<double>() * kRad;
    else if (scalars.count(key)) *scalars[key] = kv.second.as<double>();
    else throw std::invalid_argument(dice_yaml + ": unknown key " + key);
  }
  if (p.nominal_raw_deg.size() != 6 || p.via_raw_deg.size() != 6) {
    throw std::invalid_argument(dice_yaml + ": nominal_raw_deg and via_raw_deg need 6 values");
  }
  if (!p.clock_grasp_raw_deg.empty() && p.clock_grasp_raw_deg.size() != 6) {
    throw std::invalid_argument(dice_yaml + ": clock_grasp_raw_deg needs 6 values (or null)");
  }
  if (p.clock_grasp_raw_deg.empty() && p.old_load_urdf_deg.size() != 4) {
    throw std::invalid_argument(dice_yaml + ": old_load_urdf_deg needs 4 values");
  }
  return p;
}

std::vector<std::array<double, 2>> dice_spots(int face, const DiceParams &p) {
  static const std::map<int, std::vector<std::array<int, 2>>> kDice = {  // (row, col), row -1 = far
    {1, {{0, 0}}}, {2, {{-1, -1}, {1, 1}}}, {3, {{-1, -1}, {0, 0}, {1, 1}}},
    {4, {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}}}, {5, {{-1, -1}, {-1, 1}, {0, 0}, {1, -1}, {1, 1}}},
    {6, {{-1, -1}, {0, -1}, {1, -1}, {-1, 1}, {0, 1}, {1, 1}}}};
  const auto found = kDice.find(face);
  if (found == kDice.end()) throw std::invalid_argument("dice face must be 1..6");
  const double cy = p.box_gap + p.box_d / 2;
  std::vector<std::array<double, 2>> out;
  for (const auto &rc : found->second) out.push_back({p.box_x - rc[1] * p.pip, cy - rc[0] * p.pip});
  std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
    const double ax = std::round(a[0] * 1e4), bx = std::round(b[0] * 1e4);
    return ax != bx ? ax > bx : a[1] > b[1];
  });
  return out;
}

// ---------------------------------------------------------------- kinematics

struct Kinematics::Impl {
  v75::Arm arm;
  mutable v75::pin::Data data{v75::pin::Model()};
  std::array<int, 5> index{};       // canonical joint -> q index
  ControlParams c;
  DiceParams p;
  // Planar chain (y, z) in the shoulder plane, URDF joint origins.
  Eigen::Vector2d S, L1, L2, L3;
  double base_x = 0, slide_dx = 0;  // TCP x at slide 0 and per metre of slide

  Eigen::VectorXd q_of(const Pose &q) const {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(arm.model.nq);
    for (int i = 0; i < 5; ++i) v[index[i]] = i < static_cast<int>(q.size()) ? q[i] : 0.0;
    return v;
  }
  v75::pin::SE3 tcp(const Pose &q) const {
    v75::pin::forwardKinematics(arm.model, data, q_of(q));
    v75::pin::updateFramePlacement(arm.model, data, arm.tip);
    v75::pin::updateFramePlacement(arm.model, data, arm.base);
    return data.oMf[arm.base].inverse() * data.oMf[arm.tip];
  }
  Eigen::Vector2d origin_yz(const std::string &joint) const {
    const auto &t = arm.model.jointPlacements[arm.model.getJointId(joint)].translation();
    return {t[1], t[2]};
  }
};

Kinematics::Kinematics(const std::string &urdf, const ControlParams &c, const DiceParams &p)
: impl_(std::make_unique<Impl>()) {
  auto &m = *impl_;
  m.arm = v75::buildArm(urdf, "tool_tcp", "base_link");
  m.data = v75::pin::Data(m.arm.model);
  m.c = c;
  m.p = p;
  for (int i = 0; i < 5; ++i) {
    const auto it = std::find(m.arm.chain_joints.begin(), m.arm.chain_joints.end(), kJoints[i]);
    if (it == m.arm.chain_joints.end()) throw std::runtime_error("URDF chain has no " + kJoints[i]);
    m.index[i] = m.arm.model.idx_qs[m.arm.model.getJointId(kJoints[i])];
  }
  m.S = m.origin_yz("slide_joint") + m.origin_yz("shoulder_joint");
  m.L1 = m.origin_yz("elbow_joint");
  m.L2 = m.origin_yz("wrist_pitch_joint");
  const auto &tip = m.arm.model.frames[m.arm.tip].placement.translation();
  m.L3 = m.origin_yz("tool_roll_joint") + Eigen::Vector2d(tip[1], tip[2]);
  m.base_x = m.tcp({0, 0, 0, 0, 0}).translation()[0];
  m.slide_dx = m.tcp({1, 0, 0, 0, 0}).translation()[0] - m.base_x;
}

Kinematics::~Kinematics() = default;
Kinematics::Kinematics(Kinematics &&) noexcept = default;
Kinematics &Kinematics::operator=(Kinematics &&) noexcept = default;

std::array<double, 16> Kinematics::fk(const Pose &q) const {
  const auto T = impl_->tcp(q).toHomogeneousMatrix();
  std::array<double, 16> out{};
  for (int r = 0; r < 4; ++r) for (int col = 0; col < 4; ++col) out[r * 4 + col] = T(r, col);
  return out;
}

std::array<double, 2> Kinematics::limits(int joint) const {
  const int i = impl_->index[joint];
  return {impl_->arm.model.lowerPositionLimit[i], impl_->arm.model.upperPositionLimit[i]};
}

bool Kinematics::inside(const Pose &q, bool margin) const {
  const double mg = margin ? impl_->p.limit_margin_deg * kRad : 0.0;
  for (int i = 0; i < 5; ++i) {
    const auto lim = limits(i);
    const double m = i == 0 ? 0.0 : mg;
    if (!(lim[0] + m <= q[i] && q[i] <= lim[1] - m)) return false;
  }
  return true;
}

std::string Kinematics::motor_bad(const Pose &q) const {
  const auto &c = impl_->c;
  std::string bad;
  for (int i = 0; i < 4; ++i) {
    const double motor = c.arm_offsets[i] + c.arm_directions[i] * q[i + 1];
    if (!(c.arm_min[i] <= motor && motor <= c.arm_max[i])) bad += (bad.empty() ? "" : ", ") + kJoints[i + 1];
  }
  const double lm = c.lm_motor_offset + c.lm_motor_direction * q[0] / c.lm_meters_per_rad;
  if (!(c.lm_min_angle <= lm && lm <= c.lm_max_angle)) bad += std::string(bad.empty() ? "" : ", ") + "LM";
  return bad;
}

Pose Kinematics::raw_to_target(const std::vector<double> &raw_deg) const {
  const auto &c = impl_->c;
  if (raw_deg.size() != 6) throw std::invalid_argument("raw pose needs 6 motor angles");
  Pose t(6);
  t[0] = (raw_deg[0] * kRad - c.lm_motor_offset) * c.lm_motor_direction * c.lm_meters_per_rad;
  for (int i = 0; i < 4; ++i) t[i + 1] = (raw_deg[i + 1] * kRad - c.arm_offsets[i]) / c.arm_directions[i];
  t[5] = raw_deg[5] * kRad;
  return t;
}

std::vector<Pose> Kinematics::ik_top_down(const std::array<double, 3> &tcp, double tilt_deg, double roll) const {
  const auto &m = *impl_;
  const double slide = (tcp[0] - m.base_x) / m.slide_dx;
  const double a1 = m.L1.norm(), a2 = m.L2.norm(), l3 = m.L3.norm();
  const double al1 = std::atan2(m.L1[1], m.L1[0]), al2 = std::atan2(m.L2[1], m.L2[0]),
    al3 = std::atan2(m.L3[1], m.L3[0]);
  const double phi = -kPi / 2 - tilt_deg * kRad;  // tool direction in the YZ plane
  const Eigen::Vector2d W = Eigen::Vector2d(tcp[1], tcp[2]) - l3 * Eigen::Vector2d(std::cos(phi), std::sin(phi));
  const Eigen::Vector2d d = W - m.S;
  const double r = d.norm();
  const double c2 = (r * r - a1 * a1 - a2 * a2) / (2 * a1 * a2);
  if (std::abs(c2) > 1) return {};
  auto fit = [this](int joint, double a) {  // the 2*pi-equivalent angle inside the URDF limits, if any
    const auto lim = limits(joint);
    double best = a, worst = 1e9;
    for (int k = -2; k <= 2; ++k) {
      const double v = a + k * 2 * kPi;
      if (lim[0] <= v && v <= lim[1]) return v;
      const double out = std::max(lim[0] - v, v - lim[1]);
      if (out < worst) {worst = out; best = v;}
    }
    return best;
  };
  std::vector<Pose> sols;
  for (int s : {+1, -1}) {
    const double b = s * std::acos(c2);
    const double t1 = std::atan2(d[1], d[0]) - std::atan2(a2 * std::sin(b), a1 + a2 * std::cos(b));
    const double t2 = t1 + b;
    const double q1 = t1 - al1, q2 = t2 - al2 - q1, q3 = phi - al3 - q1 - q2;
    sols.push_back({slide, fit(1, q1), fit(2, q2), fit(3, q3), roll});
  }
  return sols;
}

bool Kinematics::ik_tilted(const std::array<double, 3> &tcp, double roll, Pose &out) const {
  for (int tilt = 0; tilt <= static_cast<int>(impl_->p.max_tilt_deg); ++tilt) {
    for (const auto &q : ik_top_down(tcp, tilt, roll)) {
      if (inside(q)) {out = q; return true;}
    }
  }
  return false;
}

std::vector<double> Kinematics::below(const std::vector<double> &arm, double dz, double slide) const {
  auto pose = [&](const Eigen::Vector3d &q) {
    const auto T = impl_->tcp({slide, q[0], q[1], q[2], arm[3]}).toHomogeneousMatrix();
    return Eigen::Vector3d(T(1, 3), T(2, 3), std::atan2(T(2, 2), T(1, 2)));
  };
  Eigen::Vector3d q(arm[0], arm[1], arm[2]);
  const Eigen::Vector3d goal = pose(q) - Eigen::Vector3d(0, dz, 0);
  bool ok = false;
  for (int it = 0; it < 50; ++it) {
    Eigen::Vector3d e = goal - pose(q);
    e[2] = std::atan2(std::sin(e[2]), std::cos(e[2]));
    if (e.norm() < 1e-7) {ok = true; break;}
    Eigen::Matrix3d J;
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d h = Eigen::Vector3d::Zero();
      h[j] = 1e-6;
      J.col(j) = (pose(q + h) - pose(q)) / 1e-6;
    }
    q += J.colPivHouseholderQr().solve(e);
  }
  if (!ok) throw std::runtime_error("no IK " + fmt(dz * 100) + " cm straight down, tool angle kept");
  for (int j = 0; j < 3; ++j) {
    const auto lim = limits(j + 1);
    if (!(lim[0] <= q[j] && q[j] <= lim[1])) {
      throw std::runtime_error(kJoints[j + 1] + " " + fmt(q[j] / kRad) + " deg outside the URDF limits " +
        fmt(lim[0] / kRad) + ".." + fmt(lim[1] / kRad) + " at " + fmt(dz * 100) + " cm straight down");
    }
  }
  return {q[0], q[1], q[2], arm[3]};
}

std::vector<double> Kinematics::above(const std::vector<double> &arm, double dz, double slide) const {
  auto tcp = [&](const Eigen::Vector2d &q) {
    const auto t = impl_->tcp({slide, q[0], q[1], arm[2], arm[3]}).translation();
    return Eigen::Vector2d(t[1], t[2]);
  };
  Eigen::Vector2d q(arm[0], arm[1]);
  const Eigen::Vector2d goal = tcp(q) + Eigen::Vector2d(0, dz);
  bool ok = false;
  for (int it = 0; it < 50; ++it) {
    const Eigen::Vector2d e = goal - tcp(q);
    if (e.norm() < 1e-7) {ok = true; break;}
    Eigen::Matrix2d J;
    for (int j = 0; j < 2; ++j) {
      Eigen::Vector2d h = Eigen::Vector2d::Zero();
      h[j] = 1e-6;
      J.col(j) = (tcp(q + h) - tcp(q)) / 1e-6;
    }
    q += J.colPivHouseholderQr().solve(e);
  }
  if (!ok) throw std::runtime_error("no IK " + fmt(dz * 100) + " cm above the clock grasp");
  for (int j = 0; j < 2; ++j) {
    const auto lim = limits(j + 1);
    if (!(lim[0] <= q[j] && q[j] <= lim[1])) {
      throw std::runtime_error(kJoints[j + 1] + " outside the URDF limits " + fmt(dz * 100) + " cm above the grasp");
    }
  }
  return {q[0], q[1], arm[2], arm[3]};
}

std::array<double, 3> Kinematics::grasp_at(double x, double ahead) const {
  const auto &p = impl_->p;
  return {p.lidar_center_x + x, p.lidar_front_y - ahead,
    p.floor_z + p.box_h + p.puck_h - p.grasp_depth + p.finger_beyond_tcp + p.hw_z_correction};
}

bool Kinematics::bottom_at(double x, double ahead, double h, double roll, Pose &out) const {
  const auto &p = impl_->p;
  const double L = p.finger_beyond_tcp + p.puck_h - p.grasp_depth;  // TCP -> puck bottom along the tool
  const auto g = grasp_at(x, ahead);
  const Eigen::Vector3d bottom(g[0], g[1], g[2] - L + h);
  // The smallest forward lean (0.1 deg steps) whose TCP, L back up the leaned tool from the bottom, has a solution
  // inside the limits. Tool direction in the YZ plane: (-sin t, -cos t).
  for (int k = 0; k <= static_cast<int>(p.max_tilt_deg * 10); ++k) {
    const double t = k * 0.1 * kRad;
    const Eigen::Vector3d tcp = bottom - L * Eigen::Vector3d(0, -std::sin(t), -std::cos(t));
    for (const auto &q : ik_top_down({tcp[0], tcp[1], tcp[2]}, k * 0.1, roll)) {
      if (!inside(q)) continue;
      const auto T = impl_->tcp(q).toHomogeneousMatrix();
      const Eigen::Vector3d got = T.block<3, 1>(0, 3) + L * T.block<3, 1>(0, 1);
      if ((got - bottom).norm() < 1e-4) {out = q; return true;}
    }
  }
  return false;
}

Pose nominal_pose(const Kinematics &k, const DiceParams &p) {return k.raw_to_target(p.nominal_raw_deg);}
Pose via_pose(const Kinematics &k, const DiceParams &p) {return k.raw_to_target(p.via_raw_deg);}

// ---------------------------------------------------------------- the run

namespace {

Step move(const std::string &name, std::vector<Pose> poses, bool staged = true) {
  Step s;
  s.kind = Step::Manipulator;
  s.name = name;
  s.poses = std::move(poses);
  s.staged = staged;
  return s;
}

Step gripper(const std::string &name, bool close, double open_angle = 0) {
  Step s;
  s.kind = Step::Gripper;
  s.name = name;
  s.close = close;
  s.open_angle = open_angle;
  return s;
}

Step clock(const std::string &name, int slot) {
  Step s;
  s.kind = Step::Clock;
  s.name = name;
  s.slot = slot;
  return s;
}

Pose join(double slide, const std::vector<double> &arm) {
  Pose q{slide};
  q.insert(q.end(), arm.begin(), arm.end());
  return q;
}

void require_ok(const Kinematics &k, const Pose &q, const std::string &what) {
  if (!k.inside(q, false)) throw std::runtime_error(what + ": outside the URDF limits");
  const auto bad = k.motor_bad(q);
  if (!bad.empty()) throw std::runtime_error(what + ": motor limit " + bad);
}

}  // namespace

std::vector<Step> build_run(const Kinematics &k, const DiceParams &p, int face, const std::vector<int> &slots,
  bool &taught)
{
  const auto spots = dice_spots(face, p);
  if (slots.size() != spots.size()) {
    throw std::invalid_argument("face " + std::to_string(face) + " needs " + std::to_string(spots.size()) + " slots");
  }
  const Pose start = nominal_pose(k, p), via = via_pose(k, p);
  double slide;
  std::vector<double> grasp;
  taught = !p.clock_grasp_raw_deg.empty();
  if (taught) {
    const auto t = k.raw_to_target(p.clock_grasp_raw_deg);
    slide = t[0];
    grasp = {t[1], t[2], t[3], t[4]};
  } else {
    slide = start[0];
    std::vector<double> load;
    for (double v : p.old_load_urdf_deg) load.push_back(v * kRad);
    grasp = k.below(load, p.old_unload_depth, slide);
  }
  const auto high = k.above(grasp, p.unload_above, slide);
  const auto lift = k.below(grasp, -p.lift, slide);
  const double release = p.release_gripper_deg * kRad;
  const std::string rel = fmt(p.release_gripper_deg);
  std::vector<double> downs;
  for (double h = p.carry_above - p.down_step; h > p.release_above + 1e-9; h -= p.down_step) downs.push_back(h);
  downs.push_back(p.release_above);

  std::vector<Step> s;
  // Out to the via pose: shoulder, then elbow with LM, tool_roll, gripper, then wrist_pitch (one move each).
  Pose shoulder = start, elbow = via;
  shoulder[1] = via[1];
  elbow[3] = start[3];
  s.push_back(move("via pose: shoulder first, the rest stays", {shoulder}, false));
  s.push_back(move("via pose: elbow, LM, tool_roll, gripper next (wrist_pitch stays)", {elbow}, false));
  s.push_back(move("via pose (off the nominal pose): wrist_pitch last", {via}, false));
  s.push_back(move("LM to the clock line, arm stays", {join(slide, {via[1], via[2], via[3], via[4]})}));
  Pose prev_carry;
  for (std::size_t i = 0; i < spots.size(); ++i) {
    const double x = spots[i][0], ahead = spots[i][1];
    const std::string tag = "puck " + std::to_string(i + 1) + "/" + std::to_string(spots.size());
    const std::string at = fmt((x - p.box_x) * 1000) + ", " + fmt(ahead * 1000) + " mm";
    Pose carry;
    if (!k.bottom_at(x, ahead, p.carry_above, p.put_roll, carry)) {
      throw std::runtime_error(tag + ": no pose " + fmt(p.carry_above * 1000, 0) + " mm over spot " + at);
    }
    carry[0] += p.lm_backlash;
    require_ok(k, carry, tag + " carry pose");
    std::vector<Pose> down;
    for (double h : downs) {
      Pose q;
      if (!k.bottom_at(x, ahead, h, p.put_roll, q)) {
        throw std::runtime_error(tag + ": no pose " + fmt(h * 1000, 0) + " mm over spot " + at);
      }
      q[0] += p.lm_backlash;
      require_ok(k, q, tag + " put-down line");
      down.push_back(q);
    }
    Pose clock_carry = carry;
    clock_carry[0] = slide;
    if (i > 0) {
      Pose back = prev_carry;
      back[0] = slide;
      s.push_back(move(tag + ": LM to the clock line, arm stays", {back}));
    }
    s.push_back(clock(tag + ": clock to slot " + std::to_string(slots[i]) + " (loading guide)", slots[i]));
    s.push_back(gripper(tag + ": open gripper part way to " + rel + " deg (into the slot)", false, release));
    s.push_back(move(tag + ": swing back over the slot, " + fmt(p.unload_above * 100, 0) +
      " cm above the grasp, LM stays", {join(slide, high)}));
    s.push_back(move("straight down to the grasp (" + tag + "), LM stays", {join(slide, grasp)}, false));
    s.push_back(gripper(tag + ": close gripper (grasp the puck in the clock)", true));
    s.push_back(move("straight up " + fmt(p.lift * 100, 0) + " cm out of the slot (" + tag +
      "), tool angle kept, LM stays", {join(slide, lift)}, false));
    s.push_back(move(tag + ": swing to the front, " + fmt(p.carry_above * 1000, 0) +
      " mm over the box, jaws left-right, LM stays", {clock_carry}));
    s.push_back(move(tag + ": LM to the spot line (" + fmt((x - p.box_x) * 1000) + " mm from the box centre)",
      {carry}));
    s.push_back(move("put down (" + tag + ") at " + at + ", to " + fmt(p.release_above * 1000, 0) +
      " mm over the box", down, false));
    s.push_back(gripper(tag + ": open gripper full (release on the box)", false, 0.0));
    std::vector<Pose> up(down.rbegin() + 1, down.rend());
    up.push_back(carry);
    s.push_back(move("rise above the puck (" + tag + ")", up, false));
    prev_carry = carry;
  }
  // Back: LM, then wrist_pitch to the via pose, the via pose, wrist_pitch to the start, the start (staged).
  Pose lm_back = prev_carry;
  lm_back[0] = start[0];
  Pose wrist = lm_back;
  wrist[3] = via[3];
  Pose via_wrist = via;
  via_wrist[3] = start[3];
  s.push_back(move("LM back to the start position", {lm_back}));
  s.push_back(move("wrist_pitch to the via pose first, the rest stays", {wrist}));
  s.push_back(move("via pose before the nominal pose", {via}));
  s.push_back(move("wrist_pitch to the nominal pose first, the rest stays", {via_wrist}));
  s.push_back(move("arm back to the start pose", {start}));
  return s;
}

std::vector<Step> stage(const std::vector<Step> &steps, Pose prev) {
  std::vector<Step> out;
  for (const auto &st : steps) {
    if (st.kind != Step::Manipulator) {out.push_back(st); continue;}
    Step n = st;
    n.poses.clear();
    for (const auto &q : st.poses) {
      if (st.staged && !prev.empty()) {
        Pose mid = q;
        mid[1] = prev[1];
        if (mid != q && mid != prev) n.poses.push_back(mid);
      }
      n.poses.push_back(q);
      prev = q;
    }
    out.push_back(n);
  }
  return out;
}

std::vector<Clearance> check(const Kinematics &k, const DiceParams &p, int face, const std::vector<Step> &staged) {
  const auto spots = dice_spots(face, p);
  const double top_z = p.floor_z + p.box_h, box_ahead = p.box_gap + p.box_d / 2;
  std::vector<Clearance> worst(1 + spots.size());
  worst[0].what = "box top";
  for (std::size_t i = 0; i < spots.size(); ++i) worst[i + 1].what = "placed puck " + std::to_string(i + 1);
  std::size_t placed = 0;
  // Jaws: held = closed on a puck; wide = full open after a release on the box (until the next part open / close).
  bool held = false, wide = false;
  Pose prev;
  for (const auto &st : staged) {
    if (st.kind == Step::Gripper) {
      if (st.close) {held = true; wide = false;}
      else {
        if (held) ++placed;
        held = false;
        wide = st.open_angle <= 0;
      }
      continue;
    }
    if (st.kind != Step::Manipulator) continue;
    for (const auto &q : st.poses) {
      if (!prev.empty() && (held || (wide && placed > 0))) {
        for (int n = 0; n <= 20; ++n) {
          const double f = n / 20.0;
          Pose v(5);
          for (int j = 0; j < 5; ++j) v[j] = prev[j] + (q[j] - prev[j]) * f;
          const auto T = k.fk(v);
          const Eigen::Vector3d d(T[1], T[5], T[9]), side(T[2], T[6], T[10]);
          const Eigen::Vector3d across = d.cross(side);
          const Eigen::Vector3d tcp(T[3], T[7], T[11] - p.hw_z_correction);
          const Eigen::Vector3d tip = tcp + p.finger_beyond_tcp * d;
          std::vector<std::pair<Eigen::Vector3d, double>> pts;
          if (held) for (int u = 0; u <= 6; ++u) pts.push_back({tip + (p.puck_h * u / 6 - p.grasp_depth) * d, p.puck_r});
          // Jaw corners at the fingertip and grasp_depth up: inner and outer face, both edges across.
          const double inner = p.puck_r + (held ? 0.0 : p.release_open);
          for (double up : {0.0, p.grasp_depth}) {
            for (int sg : {-1, 1}) {
              for (double out : {inner, inner + p.finger_thick}) {
                for (int e : {-1, 1}) {
                  pts.push_back({tip - up * d + sg * out * side + e * p.finger_half_width * across, 0.0});
                }
              }
            }
          }
          for (const auto &[c, r] : pts) {
            const double x_off = c[0] - p.lidar_center_x, ahead = p.lidar_front_y - c[1];
            if (std::abs(x_off - p.box_x) < p.box_w / 2 + r && std::abs(ahead - box_ahead) < p.box_d / 2 + r &&
              c[2] - top_z < worst[0].gap)
            {
              worst[0].gap = c[2] - top_z;
              worst[0].step = st.name;
            }
            for (std::size_t i = 0; i < placed; ++i) {
              if (c[2] >= top_z + p.puck_h + p.clear) continue;
              const double gap = std::hypot(x_off - spots[i][0], ahead - spots[i][1]) - p.puck_r - r;
              if (gap < worst[i + 1].gap) {worst[i + 1].gap = gap; worst[i + 1].step = st.name;}
            }
          }
        }
      }
      prev = q;
    }
  }
  return worst;
}

}  // namespace craft_planning::dice
