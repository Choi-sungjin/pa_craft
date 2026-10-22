#include "craft_planning/planning_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>

namespace craft_planning {
namespace {

void require(bool valid, const std::string &message) {
  if (!valid) throw std::invalid_argument(message);
}

void fields(const YAML::Node &node, const std::set<std::string> &allowed,
  const std::string &context)
{
  require(node.IsMap(), context + " must be a mapping");
  std::set<std::string> seen;
  for (const auto &entry : node) {
    require(entry.first.IsScalar(), context + " field names must be strings");
    const auto key = entry.first.as<std::string>();
    require(allowed.count(key) != 0, "unknown " + context + " field: " + key);
    require(seen.insert(key).second, "duplicate " + context + " field: " + key);
  }
}

std::string text_value(const YAML::Node &node, const std::string &context) {
  require(node.IsScalar(), context + " must be a nonempty string");
  double number;
  bool boolean;
  require(node.Tag() != "tag:yaml.org,2002:int" && node.Tag() != "tag:yaml.org,2002:float" &&
    node.Tag() != "tag:yaml.org,2002:bool" &&
    (node.Tag() != "?" || (!YAML::convert<double>::decode(node, number) &&
    !YAML::convert<bool>::decode(node, boolean))), context + " must be a string");
  const auto value = node.as<std::string>();
  require(!value.empty(), context + " must be a nonempty string");
  return value;
}

template<typename T>
T scalar(const YAML::Node &node, const std::string &context) {
  require(node.IsScalar() && node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str",
    context + " must be a typed scalar");
  return node.as<T>();
}

double finite_value(const YAML::Node &node, const std::string &context) {
  const double value = scalar<double>(node, context);
  require(std::isfinite(value), context + " must be finite");
  return value;
}

PlanningStep parse_step(const YAML::Node &node, const std::string &map_frame,
  const std::string &base_frame)
{
  require(node.IsMap(), "mission step must be a mapping");
  PlanningStep step;
  step.id = text_value(node["id"], "step id");
  const auto type = text_value(node["type"], "step type");
  if (type == "navigate" || type == "tcp") {
    fields(node, {"id", "type", "pose", "frame", "source", "class_id"}, "pose step");
    step.kind = type == "navigate" ? StepKind::Navigate : StepKind::Tcp;
    const auto &expected_frame = type == "navigate" ? map_frame : base_frame;
    step.goal.header.frame_id = node["frame"] ? text_value(node["frame"], "frame") : expected_frame;
    require(step.goal.header.frame_id == expected_frame, type + " frame must be " + expected_frame);
    if (node["source"]) step.source = text_value(node["source"], "source");
    const std::set<std::string> sources = {
      "fixed", "d435_knob", "d435_puck", "object_pose", "object_detection"};
    require(sources.count(step.source) != 0, "unknown observation source: " + step.source);
    if (node["class_id"]) step.class_id = scalar<int>(node["class_id"], "class_id");
    require(step.class_id >= -1, "class_id must be -1 or nonnegative");
    const auto pose = node["pose"];
    require(pose.IsSequence() && pose.size() == 7,
      "pose requires [x,y,z,qx,qy,qz,qw], including an explicit quaternion for observations");
    double values[7];
    for (std::size_t i = 0; i < 7; ++i) values[i] = finite_value(pose[i], "pose component");
    const double norm = std::hypot(std::hypot(values[3], values[4]), std::hypot(values[5], values[6]));
    require(std::isfinite(norm) && norm >= 1e-9, "pose quaternion must have nonzero finite norm");
    step.goal.pose.position.x = values[0];
    step.goal.pose.position.y = values[1];
    step.goal.pose.position.z = values[2];
    step.goal.pose.orientation.x = values[3] / norm;
    step.goal.pose.orientation.y = values[4] / norm;
    step.goal.pose.orientation.z = values[5] / norm;
    step.goal.pose.orientation.w = values[6] / norm;
  } else if (type == "gripper") {
    fields(node, {"id", "type", "close"}, "gripper step");
    step.kind = StepKind::Gripper;
    step.close = scalar<bool>(node["close"], "close");
  } else if (type == "clock") {
    fields(node, {"id", "type", "slot"}, "clock step");
    step.kind = StepKind::Clock;
    step.slot = scalar<int>(node["slot"], "slot");
    require(step.slot >= 1 && step.slot <= 12, "clock slot must be 1..12");
  } else if (type == "wait") {
    fields(node, {"id", "type", "duration_s"}, "wait step");
    step.kind = StepKind::Wait;
    step.duration_s = finite_value(node["duration_s"], "duration_s");
    require(step.duration_s >= 0.0, "wait duration_s must be nonnegative");
  } else {
    throw std::invalid_argument("unknown mission step type: " + type);
  }
  return step;
}

class Parameters {
public:
  explicit Parameters(rclcpp::Node &node) : node_(node) {}

  template<typename T> T param(const std::string &name) {
    names_.insert(name);
    try {
      return node_.declare_parameter<T>(name);
    } catch (const std::exception &error) {
      throw std::invalid_argument("provide " + name + " with the correct type in planning.yaml: " + error.what());
    }
  }

  double positive_param(const std::string &name) {
    const double value = param<double>(name);
    require(std::isfinite(value) && value > 0.0, name + " must be finite and positive");
    return value;
  }

  double nonnegative(const std::string &name) {
    const double value = param<double>(name);
    require(std::isfinite(value) && value >= 0.0, name + " must be finite and nonnegative");
    return value;
  }

  std::string nonempty(const std::string &name) {
    const auto value = param<std::string>(name);
    require(!value.empty(), name + " must not be empty");
    return value;
  }

  int integer(const std::string &name, int minimum, int maximum = std::numeric_limits<int>::max()) {
    const auto value = param<int64_t>(name);
    require(value >= minimum && value <= maximum, name + " is outside the permitted integer range");
    return static_cast<int>(value);
  }

  void reject_unknown() const {
    for (const auto &entry : node_.get_node_parameters_interface()->get_parameter_overrides()) {
      require(names_.count(entry.first) || entry.first == "use_sim_time" ||
        entry.first == "start_type_description_service", "unknown planning parameter: " + entry.first);
    }
  }

private:
  rclcpp::Node &node_;
  std::set<std::string> names_;
};

std::string file_path(const std::string &value, const std::string &share, const std::string &name) {
  std::filesystem::path path(value);
  if (!path.is_absolute()) path = std::filesystem::path(share) / path;
  path = path.lexically_normal();
  std::error_code error;
  require(std::filesystem::is_regular_file(path, error),
    name + " is not an existing regular file: " + path.string());
  return path.string();
}

}  // namespace

std::map<std::string, std::vector<PlanningStep>> parse_missions(
  const std::string &filename, const std::string &map_frame, const std::string &base_frame)
{
  require(!map_frame.empty() && !base_frame.empty(), "mission goal frames must not be empty");
  try {
    const auto root = YAML::LoadFile(filename);
    fields(root, {"missions"}, "mission file");
    const auto missions = root["missions"];
    require(missions.IsMap(), "missions must be a mapping");
    std::map<std::string, std::vector<PlanningStep>> output;
    for (const auto &entry : missions) {
      const auto name = text_value(entry.first, "mission name");
      require(output.count(name) == 0, "duplicate mission name: " + name);
      require(entry.second.IsSequence(), "mission " + name + " must be a step sequence");
      std::set<std::string> ids;
      std::vector<PlanningStep> steps;
      for (const auto &node : entry.second) {
        auto step = parse_step(node, map_frame, base_frame);
        require(ids.insert(step.id).second, "duplicate step id in mission " + name + ": " + step.id);
        steps.push_back(std::move(step));
      }
      output.emplace(name, std::move(steps));
    }
    return output;
  } catch (const std::exception &error) {
    throw std::invalid_argument("invalid mission file " + filename + ": " + error.what());
  }
}

PlanningConfig load_planning_config(rclcpp::Node &node, const std::string &share_dir) {
  require(!share_dir.empty(), "planning package share directory must not be empty");
  PlanningConfig config;
  Parameters p(node);
  config.path.map_file = file_path(p.nonempty("map_file"), share_dir, "map_file");
  config.path.map_frame = p.nonempty("map_frame");
  config.path.stl_slice_heights = p.param<std::vector<double>>("stl_slice_heights");
  require(!config.path.stl_slice_heights.empty(), "stl_slice_heights must not be empty");
  for (const double height : config.path.stl_slice_heights) {
    require(std::isfinite(height), "stl_slice_heights must be finite");
  }
  config.path.occupancy_threshold = p.integer("occupancy_threshold", 0, 100);
  config.goal_heading_required = p.param<bool>("goal_heading_required");
  config.path.res = p.positive_param("path_planner.res");
  config.path.block_size = p.positive_param("path_planner.block_size");
  config.path.r_robot = p.positive_param("path_planner.r_robot");
  config.path.margin = p.nonnegative("path_planner.margin");
  config.path.lam = p.nonnegative("path_planner.lam");
  config.path.w_prox = p.nonnegative("path_planner.w_prox");
  config.path.prox_band = p.positive_param("path_planner.prox_band");
  config.path.arc_sample = p.positive_param("path_planner.arc_sample");
  config.path.max_cells = static_cast<std::size_t>(p.integer("path_planner.max_cells", 1));
  config.path.max_expansions = static_cast<std::size_t>(p.integer("path_planner.max_expansions", 1));
  config.arm.urdf = file_path(p.nonempty("ik.urdf"), share_dir, "ik.urdf");
  config.arm.base_frame = p.nonempty("ik.base_frame");
  config.arm.tip_frame = p.nonempty("ik.tip_frame");
  config.arm.position_tolerance = p.positive_param("ik.position_tolerance");
  config.arm.orientation_tolerance = p.positive_param("ik.orientation_tolerance");
  config.arm.max_iterations = p.integer("ik.max_iterations", 1);
  config.arm.max_restarts = p.integer("ik.max_restarts", 0);
  config.fsm.ready_timeout_s = p.positive_param("fsm.ready_timeout_s");
  config.fsm.compute_timeout_s = p.positive_param("fsm.compute_timeout_s");
  config.fsm.execution_timeout_s = p.positive_param("fsm.execution_timeout_s");
  config.fsm.feedback_timeout_s = p.positive_param("fsm.feedback_timeout_s");
  config.fsm.completion_debounce_s = p.nonnegative("fsm.completion_debounce_s");
  config.fsm.max_compute_retries = p.integer("fsm.max_compute_retries", 0);
  config.loop_hz = p.positive_param("loop_hz");
  config.odom_timeout_s = p.positive_param("odom_timeout_s");
  config.joint_timeout_s = p.positive_param("joint_timeout_s");
  config.observation_timeout_s = p.positive_param("observation_timeout_s");
  config.obstacle_timeout_s = p.positive_param("obstacle_timeout_s");
  config.cruise_speed = p.positive_param("cruise_speed");
  config.goal_tolerance = p.positive_param("goal_tolerance");
  config.heading_tolerance = p.positive_param("heading_tolerance");
  require(config.heading_tolerance <= std::acos(-1.0), "heading_tolerance must not exceed pi radians");
  config.observation_min_confidence = p.nonnegative("observation_min_confidence");
  require(config.observation_min_confidence <= 1.0, "observation_min_confidence must be in [0,1]");
  config.require_lidar = p.param<bool>("require_lidar");
  config.joint_state_topic = p.nonempty("joint_state_topic");
  const auto mission_file = file_path(p.nonempty("mission_file"), share_dir, "mission_file");
  p.reject_unknown();
  config.missions = parse_missions(mission_file, config.path.map_frame, config.arm.base_frame);
  return config;
}

}  // namespace craft_planning
