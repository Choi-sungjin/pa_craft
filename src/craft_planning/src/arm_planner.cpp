// Pinocchio must precede headers that use Boost (including ROS headers).
#include "v75_arm_ik/arm_ik_core.hpp"
#include "craft_planning/arm_planner.hpp"

#include <algorithm>
#include <cmath>
#include <random>
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
      if (!std::isfinite(value) || value < arm.model.lowerPositionLimit[index] ||
        value > arm.model.upperPositionLimit[index])
      {
        throw std::invalid_argument("arm joint position is non-finite or outside URDF limits: " + input.name[i]);
      }
      q[index] = value;
    }
    return q;
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
  if (!next->arm.model.lowerPositionLimit.allFinite() ||
    !next->arm.model.upperPositionLimit.allFinite() ||
    (next->arm.model.lowerPositionLimit.array() > next->arm.model.upperPositionLimit.array()).any())
  {
    throw std::invalid_argument("URDF arm joint limits must be finite and ordered");
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
    std::mt19937 rng(42);
    const auto result = v75::solveIk(impl_->arm, target, seed, impl_->params, rng);
    if (!result.ok || result.q.size() != static_cast<Eigen::Index>(kControlJointOrder.size()) ||
      !result.q.allFinite() || !std::isfinite(result.pos_err) || !std::isfinite(result.rot_err))
    {
      output.error = "TCP goal is unreachable within URDF joint limits and IK tolerances";
      return output;
    }
    output.joints.header = goal.header;
    output.joints.name = impl_->arm.chain_joints;
    output.joints.position.assign(result.q.data(), result.q.data() + result.q.size());
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
