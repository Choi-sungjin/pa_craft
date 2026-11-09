#include "craft_planning/planning_config.hpp"
#include "craft_perception/tag_registry.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

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
  config.path.goal_heading_required = config.goal_heading_required;
  for (auto [name, box] : {std::pair{"grasp_region.lying", &config.grasp.lying},
                           std::pair{"grasp_region.standing", &config.grasp.standing}}) {
    const auto values = p.param<std::vector<double>>(name);
    require(values.size() == 6, std::string(name) + " needs [x_min, x_max, y_min, y_max, z_min, z_max]");
    std::copy(values.begin(), values.end(), box->begin());
  }
  config.grasp.margin = p.nonnegative("grasp_region.margin");
  config.grasp.axis_tolerance = p.nonnegative("grasp_region.axis_tolerance_deg") * M_PI / 180.0;
  config.grasp.confirm_count = static_cast<int>(p.integer("grasp_region.confirm_count", 1));
  config.grasp.front_face_x = p.param<double>("grasp_region.front_face_x");
  config.grasp.front_face_width = p.nonnegative("grasp_region.front_face_width");
  config.grasp.good_plane = p.nonnegative("grasp_region.good_plane");
  config.grasp.min_box_gap = p.nonnegative("grasp_region.min_box_gap");
  config.grasp.w_box = p.nonnegative("grasp_region.w_box");
  config.grasp.w_grasp = p.nonnegative("grasp_region.w_grasp");
  validate(config.grasp);
  config.path.r_min = p.nonnegative("path_planner.r_min");
  config.path.r_hi_global = p.positive_param("path_planner.r_hi_global");
  config.path.r_tol = p.positive_param("path_planner.r_tol");
  config.path.v_ref = p.positive_param("path_planner.v_ref");
  config.path.w_max = p.positive_param("path_planner.w_max");
  config.path.a_lat_max = p.positive_param("path_planner.a_lat_max");
  config.path.a_lon_max = p.positive_param("path_planner.a_lon_max");
  config.path.approach_len = p.positive_param("path_planner.approach_len");
  config.path.mode = p.nonempty("path_planner.mode");
  require(config.path.mode == "arc" || config.path.mode == "global_local", "path_planner.mode must be arc or global_local");
  config.path.local_behind = p.nonnegative("path_planner.local_behind");
  config.path.local_ahead = p.positive_param("path_planner.local_ahead");
  config.path.local_update = p.positive_param("path_planner.local_update");
  config.path.local_max_dev = p.nonnegative("path_planner.local_max_dev");
  config.path.wheel_radius = p.positive_param("path_planner.wheel_radius");
  config.path.wheel_separation = p.positive_param("path_planner.wheel_separation");
  config.path.max_wheel = p.positive_param("path_planner.max_wheel");
  config.path.v_min = p.positive_param("path_planner.v_min");
  require(config.path.local_update < config.path.local_ahead, "path_planner.local_update must be below local_ahead");
  require(config.path.r_min <= config.path.r_hi_global, "path_planner.r_min must not exceed r_hi_global");
  config.arm.urdf = file_path(p.nonempty("ik.urdf"), share_dir, "ik.urdf");
  config.arm.base_frame = p.nonempty("ik.base_frame");
  config.arm.tip_frame = p.nonempty("ik.tip_frame");
  config.arm.position_tolerance = p.positive_param("ik.position_tolerance");
  config.arm.orientation_tolerance = p.positive_param("ik.orientation_tolerance");
  config.arm.max_iterations = p.integer("ik.max_iterations", 1);
  config.arm.max_restarts = p.integer("ik.max_restarts", 0);
  config.fsm.initialization_timeout_s = p.positive_param("fsm.initialization_timeout_s");
  config.fsm.ready_timeout_s = p.positive_param("fsm.ready_timeout_s");
  config.fsm.compute_timeout_s = p.positive_param("fsm.compute_timeout_s");
  config.fsm.execution_timeout_s = p.positive_param("fsm.execution_timeout_s");
  config.fsm.feedback_timeout_s = p.positive_param("fsm.feedback_timeout_s");
  config.fsm.stop_timeout_s = p.positive_param("fsm.stop_timeout_s");
  config.fsm.max_compute_retries = p.integer("fsm.max_compute_retries", 0);
  config.loop_hz = p.positive_param("loop_hz");
  config.replan_period_s = p.positive_param("replan_period_s");
  config.block_wait_s = p.nonnegative("block_wait_s");
  config.min_segment_m = p.nonnegative("min_segment_m");
  config.odom_timeout_s = p.positive_param("odom_timeout_s");
  config.localization_observation_timeout_s = p.positive_param("localization_observation_timeout_s");
  config.require_lidar_localization = p.param<bool>("require_lidar_localization");
  config.joint_timeout_s = p.positive_param("joint_timeout_s");
  config.observation_timeout_s = p.positive_param("observation_timeout_s");
  config.obstacle_timeout_s = p.positive_param("obstacle_timeout_s");
  config.cruise_speed = p.positive_param("cruise_speed");
  config.goal_tolerance = p.positive_param("goal_tolerance");
  config.path.stop_distance = config.goal_tolerance;  // the speed profile creeps at v_min where control stops
  config.observation_min_confidence = p.nonnegative("observation_min_confidence");
  require(config.observation_min_confidence <= 1.0, "observation_min_confidence must be in [0,1]");
  config.require_lidar = p.param<bool>("require_lidar");
  config.require_arm_joints = p.param<bool>("require_arm_joints");
  config.joint_state_topic = p.nonempty("joint_state_topic");
  const auto mission_file = file_path(p.nonempty("mission_file"), share_dir, "mission_file");
  config.missions = parse_missions(mission_file, config.path.map_frame, config.arm.base_frame);
  config.box.camera_enabled = p.param<bool>("box_approach.camera_enabled");
  config.box.camera_only_longitudinal = p.param<bool>("box_approach.camera_only_longitudinal");
  config.box.camera_slow_gap = p.positive_param("box_approach.camera_slow_gap");
  config.box.camera_fast_speed = p.positive_param("box_approach.camera_fast_speed");
  require(!config.box.camera_only_longitudinal || config.box.camera_enabled,
          "box_approach.camera_only_longitudinal needs camera_enabled");
  config.box.camera_final_gap = p.positive_param("box_approach.camera_final_gap");
  config.box.camera_gap_tolerance = p.positive_param("box_approach.camera_gap_tolerance");
  config.box.handoff_gap = p.positive_param("box_approach.handoff_gap");
  config.box.source_gap_diff = p.positive_param("box_approach.source_gap_diff");
  config.box.camera_timeout = p.positive_param("box_approach.camera_timeout");
  config.box.lidar_observation_timeout = p.positive_param("box_approach.lidar_observation_timeout");
  config.box.camera_speed = p.positive_param("box_approach.camera_speed");
  config.box.camera_uncertainty_max = p.positive_param("box_approach.camera_uncertainty_max");
  config.box.camera_far_uncertainty_max = p.positive_param("box_approach.camera_far_uncertainty_max");
  config.box.camera_near_gap = p.positive_param("box_approach.camera_near_gap");
  config.box.lidar_uncertainty_max = p.positive_param("box_approach.lidar_uncertainty_max");
  config.box.observation_initial_timeout = p.positive_param("box_approach.observation_initial_timeout");
  config.box.camera_confirm_count = p.integer("box_approach.camera_confirm_count", 1);
  config.box.camera_calibrated_gap_min = p.positive_param("box_approach.camera_calibrated_gap_min");
  config.box.camera_calibrated_gap_max = p.positive_param("box_approach.camera_calibrated_gap_max");
  config.box.camera_operational_gap_max = p.positive_param("box_approach.camera_operational_gap_max");
  config.box.front_lidar_x = p.nonnegative("box_approach.front_lidar_x");
  config.box.front_lidar_fov_deg = p.positive_param("box_approach.front_lidar_fov_deg");
  require(config.box.front_lidar_fov_deg <= 180.0, "front LiDAR FOV must not exceed 180 degrees");
  require(config.box.camera_calibrated_gap_min >= 0.010 && config.box.camera_calibrated_gap_max <= 0.080,
          "box camera gap limits must stay within the calibrated 0.010 to 0.080 m range");
  require(config.box.camera_final_gap < config.box.handoff_gap &&
          config.box.handoff_gap <= config.box.camera_operational_gap_max &&
          config.box.camera_operational_gap_max <= 0.200,
          "box camera operational gap must include handoff and not exceed 0.200 m");
  require(config.box.camera_final_gap + config.box.camera_gap_tolerance < config.box.handoff_gap,
          "box camera final gap must be below handoff gap");
  config.box.speed = p.positive_param("box_approach.speed");
  config.box.gap = p.nonnegative("box_approach.gap");
  config.box.front_face_x = p.positive_param("box_approach.front_face_x");
  config.box.side = p.positive_param("box_approach.side");
  config.box.max_offset = p.positive_param("box_approach.max_offset");
  config.box.settle_s = p.nonnegative("box_approach.settle_s");
  config.box.lateral_tolerance = p.positive_param("box_approach.lateral_tolerance");
  config.box.yaw_tolerance = p.positive_param("box_approach.yaw_tolerance_deg") * M_PI / 180.0;
  config.box.max_corrections = static_cast<int>(p.integer("box_approach.max_corrections", 0));
  config.box.reverse_distance = p.positive_param("box_approach.reverse_distance");
  config.box.reverse_min_distance = p.positive_param("box_approach.reverse_min_distance");
  config.box.stage_lead = p.nonnegative("box_approach.stage_lead");
  config.box.stage_goal_tolerance = p.positive_param("box_approach.stage_goal_tolerance");
  config.box.dock_speed = p.positive_param("box_approach.dock_speed");
  config.box.dock_lookahead = p.positive_param("box_approach.dock_lookahead");
  config.box.dock_goal_tolerance = p.positive_param("box_approach.dock_goal_tolerance");
  require(config.box.reverse_min_distance <= config.box.reverse_distance,
          "box_approach.reverse_min_distance must not exceed reverse_distance");
  config.box.reverse_speed = p.positive_param("box_approach.reverse_speed");
  config.box_detection_timeout_s = p.positive_param("box_detection_timeout_s");
  for (const auto &name : p.param<std::vector<std::string>>("box_approach.names")) {
    const auto v = p.param<std::vector<double>>("box_approach." + name);
    require(!name.empty() && v.size() == 3 && std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]),
            "box_approach." + name + " must be [x, y, yaw_deg]");
    config.box.boxes[name] = {v[0], v[1], v[2] * M_PI / 180.0};
  }
  auto tag_file = std::filesystem::path(p.nonempty("tag_registry_file"));
  if (!tag_file.is_absolute()) {
    tag_file = ament_index_cpp::get_package_share_path("craft_perception") / tag_file;
  }
  for (const auto &[tag_id, target] : craft_perception::load_tag_registry(tag_file.string())) {
    if (target.role != craft_perception::TagRole::Mission) continue;
    require(config.missions.count(target.mission) != 0,
      "mission tag references unknown mission: " + target.mission);
    config.mission_by_tag.emplace(tag_id, target.mission);
  }
  p.reject_unknown();
  return config;
}

}  // namespace craft_planning
