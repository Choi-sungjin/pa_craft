// Pinocchio must precede headers that use Boost (including ROS headers).
#include "v75_arm_ik/arm_ik_core.hpp"
#include "craft_planning/arm_planner.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>

namespace craft_planning {
namespace {

const std::vector<std::string> kControlJointOrder = {
  "slide_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"};

bool positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

struct ArmPlanner::Impl {
  v75::Arm arm;
  v75::IkParams params;
  std::string base_frame;

  v75::VectorXd ordered_joints(const sensor_msgs::msg::JointState &input) const {
    const auto &names = arm.chain_joints;
    if (input.name.size() != names.size() || input.position.size() != names.size()) {
      throw std::invalid_argument("arm seed requires exactly five named joint positions");
    }
    v75::VectorXd q(names.size());
    std::vector<bool> seen(names.size(), false);
    for (std::size_t i = 0; i < input.name.size(); ++i) {
      const auto found = std::find(names.begin(), names.end(), input.name[i]);
      if (found == names.end()) {
        throw std::invalid_argument("unknown arm joint: " + input.name[i]);
      }
      const auto index = static_cast<std::size_t>(found - names.begin());
      if (seen[index]) throw std::invalid_argument("duplicate arm joint: " + input.name[i]);
      seen[index] = true;
      const double value = input.position[i];
      const double lower = arm.model.lowerPositionLimit[index];
      const double upper = arm.model.upperPositionLimit[index];
      if (!std::isfinite(value))
      {
        throw std::invalid_argument(input.name[i] + " position is non-finite");
      }
      // Measured rotary feedback may overshoot by up to 1 degree. Clamp only the
      // FK/IK seed; control.yaml limits and commanded targets remain unchanged. Slide is in m.
      const double tolerance = index == 0 ? 0.0 : 3.14159265358979323846 / 180.0;
      if (value < lower - tolerance || value > upper + tolerance)
      {
        std::ostringstream reason;
        reason << input.name[i] << ' ' << std::fixed << std::setprecision(4) << value
               << " beyond limit " << (value < lower ? lower : upper);
        throw std::invalid_argument(reason.str());
      }
      q[index] = std::clamp(value, lower, upper);
    }
    return q;
  }

  // Gravity moment about the elbow of everything past it, as a proxy for the elbow holding torque:
  // the URDF has no inertias, so the wrist pitch, tool roll and TCP origins count as equal masses.
  // Base +z is up. [m, per unit weight]
  double elbow_load(v75::pin::Data &data, const v75::VectorXd &q) const {
    const auto &m = arm.model;
    v75::pin::forwardKinematics(m, data, q);
    v75::pin::updateFramePlacement(m, data, arm.base);
    v75::pin::updateFramePlacement(m, data, arm.tip);
    const auto &elbow = data.oMi[m.getJointId("elbow_joint")];
    const v75::Vector3d down = -data.oMf[arm.base].rotation().col(2);
    v75::Vector3d moment = v75::Vector3d::Zero();
    for (const v75::Vector3d &point : {data.oMi[m.getJointId("wrist_pitch_joint")].translation(),
        data.oMi[m.getJointId("tool_roll_joint")].translation(), data.oMf[arm.tip].translation()})
    {
      moment += (point - elbow.translation()).cross(down);
    }
    return std::abs(moment.dot(elbow.rotation().col(0)));
  }
};

ArmPlanner::ArmPlanner() = default;
ArmPlanner::~ArmPlanner() = default;
ArmPlanner::ArmPlanner(ArmPlanner &&) noexcept = default;
ArmPlanner &ArmPlanner::operator=(ArmPlanner &&) noexcept = default;

void ArmPlanner::configure(const ArmPlannerConfig &config) {
  if (config.urdf.empty() || config.base_frame.empty() || config.tip_frame.empty() ||
    !positive(config.position_tolerance) || !positive(config.orientation_tolerance) ||
    config.max_iterations < 1 || config.max_restarts < 0)
  {
    throw std::invalid_argument("arm planner requires URDF/base/tip, positive tolerances and valid iteration limits");
  }
  auto next = std::make_unique<Impl>();
  try {
    next->arm = v75::buildArm(config.urdf, config.tip_frame, config.base_frame);
  } catch (const std::exception &error) {
    throw std::invalid_argument("cannot configure arm model: " + std::string(error.what()));
  }
  if (next->arm.chain_joints != kControlJointOrder) {
    throw std::invalid_argument("URDF IK chain must match slide, shoulder, elbow, wrist pitch and tool roll control order");
  }
  try {
    const auto path = ament_index_cpp::get_package_share_path("craft_control") / "config/control.yaml";
    const YAML::Node p = YAML::LoadFile(path.string())["control_master"]["ros__parameters"];
    if (!p || !p.IsMap()) throw std::invalid_argument("missing control_master.ros__parameters");
    const auto array = [&p](const char *key) {
      const auto values = p[key].as<std::vector<double>>(std::vector<double>{});
      if (values.size() != 4 || !std::all_of(values.begin(), values.end(),
          [](double value) {return std::isfinite(value);}))
      {
        throw std::invalid_argument(std::string(key) + " requires four finite values");
      }
      return values;
    };
    const auto scalar = [&p](const char *key) {
      const double value = p[key].as<double>(std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(value)) throw std::invalid_argument(std::string(key) + " must be finite");
      return value;
    };
    const auto lows = array("arm_min_angle"), highs = array("arm_max_angle");
    const auto offsets = array("arm_joint_offsets"), directions = array("arm_joint_directions");
    auto &model = next->arm.model;
    // Invert the standalone motor_bad() mappings: use the calibrated motor stops,
    // including their existing safety margins, for both feedback checks and IK.
    for (std::size_t i = 0; i < 4; ++i) {
      if (lows[i] >= highs[i] || std::abs(directions[i]) != 1.0) {
        throw std::invalid_argument(kControlJointOrder[i + 1] +
          ": arm_min_angle must be < arm_max_angle and arm_joint_directions must be +/-1");
      }
      const double a = (lows[i] - offsets[i]) / directions[i];
      const double b = (highs[i] - offsets[i]) / directions[i];
      model.lowerPositionLimit[i + 1] = std::min(a, b);
      model.upperPositionLimit[i + 1] = std::max(a, b);
    }
    const double low = scalar("lm_min_angle"), high = scalar("lm_max_angle");
    const double offset = scalar("lm_motor_offset"), direction = scalar("lm_motor_direction");
    const double scale = scalar("lm_meters_per_rad");
    if (low >= high || std::abs(direction) != 1.0 || scale <= 0.0) {
      throw std::invalid_argument("slide_joint: lm_min_angle must be < lm_max_angle, "
        "lm_motor_direction must be +/-1 and lm_meters_per_rad must be > 0");
    }
    const double a = (low - offset) * scale / direction;
    const double b = (high - offset) * scale / direction;
    model.lowerPositionLimit[0] = std::min(a, b);
    model.upperPositionLimit[0] = std::max(a, b);
  } catch (const std::exception &error) {
    throw std::invalid_argument("cannot configure arm limits from craft_control/config/control.yaml: " +
      std::string(error.what()));
  }
  if (!next->arm.model.lowerPositionLimit.allFinite() ||
    !next->arm.model.upperPositionLimit.allFinite() ||
    (next->arm.model.lowerPositionLimit.array() >= next->arm.model.upperPositionLimit.array()).any())
  {
    throw std::invalid_argument("control.yaml converted arm joint limits must be finite and strictly ordered");
  }
  next->base_frame = config.base_frame;
  next->params.pos_tol = config.position_tolerance;
  next->params.rot_tol = config.orientation_tolerance;
  next->params.max_iters = config.max_iterations;
  next->params.max_restarts = config.max_restarts;
  impl_ = std::move(next);
}

ArmPlan ArmPlanner::plan(const geometry_msgs::msg::PoseStamped &goal,
  const sensor_msgs::msg::JointState &current) const
{
  ArmPlan output;
  if (!impl_) {
    output.error = "arm planner is not configured";
    return output;
  }
  try {
    if (goal.header.frame_id != impl_->base_frame) {
      throw std::invalid_argument("TCP goal frame must be " + impl_->base_frame);
    }
    const auto &p = goal.pose.position;
    const auto &o = goal.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.z) || !std::isfinite(o.w))
    {
      throw std::invalid_argument("TCP goal position and quaternion must be finite");
    }
    const double norm = std::hypot(std::hypot(o.x, o.y), std::hypot(o.z, o.w));
    if (!std::isfinite(norm) || norm < 1e-9) {
      throw std::invalid_argument("TCP goal quaternion must have nonzero finite norm");
    }
    const Eigen::Quaterniond quaternion(o.w / norm, o.x / norm, o.y / norm, o.z / norm);
    const v75::pin::SE3 target(quaternion.toRotationMatrix(), v75::Vector3d(p.x, p.y, p.z));
    const auto seed = impl_->ordered_joints(current);
    // Collect every distinct solution (seed first, then the fixed-seed random restarts) instead of
    // stopping at the seed's: of several branches, take the one with the least elbow load, and of
    // equal loads the one nearest the seed.
    const auto &m = impl_->arm.model;
    v75::pin::Data data(m);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<v75::VectorXd> solutions;
    for (int attempt = 0; attempt <= impl_->params.max_restarts; ++attempt) {
      v75::VectorXd q(m.nq);
      if (attempt == 0) {
        q = seed;
      } else {
        for (int i = 0; i < m.nq; ++i) {
          q[i] = m.lowerPositionLimit[i] + unit(rng) * (m.upperPositionLimit[i] - m.lowerPositionLimit[i]);
        }
      }
      double pos_err = 0, rot_err = 0;
      if (!v75::solveOnce(impl_->arm, data, target, q, impl_->params, pos_err, rot_err) ||
        !q.allFinite() || !std::isfinite(pos_err) || !std::isfinite(rot_err))
      {
        continue;
      }
      if (std::none_of(solutions.begin(), solutions.end(),
          [&q](const v75::VectorXd &s) {return (s - q).norm() < 1e-3;}))
      {
        solutions.push_back(q);
      }
    }
    if (solutions.empty()) {
      output.error = "TCP goal is unreachable within control.yaml joint limits and IK tolerances";
      return output;
    }
    constexpr double kSameLoad = 1e-3;  // [m]
    const v75::VectorXd *best = nullptr;
    double best_load = 0;
    for (const auto &q : solutions) {
      const double load = impl_->elbow_load(data, q);
      if (!best || load < best_load - kSameLoad ||
        (load < best_load + kSameLoad && (q - seed).norm() < (*best - seed).norm()))
      {
        best = &q;
        best_load = load;
      }
    }
    output.joints.header = goal.header;
    output.joints.name = impl_->arm.chain_joints;
    output.joints.position.assign(best->data(), best->data() + best->size());
    output.ok = true;
  } catch (const std::exception &error) {
    output.error = error.what();
  }
  return output;
}

geometry_msgs::msg::PoseStamped ArmPlanner::forward(const sensor_msgs::msg::JointState &joints) const {
  if (!impl_) throw std::logic_error("arm planner is not configured");
  const auto q = impl_->ordered_joints(joints);
  v75::pin::Data data(impl_->arm.model);
  const auto tcp = v75::forward(impl_->arm, data, q);
  const Eigen::Quaterniond orientation(tcp.rotation());
  geometry_msgs::msg::PoseStamped output;
  output.header = joints.header;
  output.header.frame_id = impl_->base_frame;
  output.pose.position.x = tcp.translation().x();
  output.pose.position.y = tcp.translation().y();
  output.pose.position.z = tcp.translation().z();
  output.pose.orientation.x = orientation.x();
  output.pose.orientation.y = orientation.y();
  output.pose.orientation.z = orientation.z();
  output.pose.orientation.w = orientation.w();
  return output;
}

const std::vector<std::string> &ArmPlanner::joint_names() const {
  if (!impl_) throw std::logic_error("arm planner is not configured");
  return impl_->arm.chain_joints;
}

}  // namespace craft_planning
