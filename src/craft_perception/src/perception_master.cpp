#include "craft_perception/perception_master.hpp"
#include "craft_perception/tag_registry.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <utility>
#include <filesystem>
#include <cmath>
#include <cstdarg>
#include <cstring>
#include <limits>

namespace craft_perception {
namespace {
const char *const kTasks[] = {"d435_yolo", "apriltag", "object_detector",
  "obstacle_detector", "puck_axis", "traffic_light", "puck_survey", "uppercam_puck", "lidar_obstacle",
  "state_estimator"};

geometry_msgs::msg::Point point(const cv::Vec3d &v) {
  geometry_msgs::msg::Point output; 
  output.x = v[0]; 
  output.y = v[1]; 
  output.z = v[2]; 
  return output;
}
geometry_msgs::msg::Vector3 vector(const cv::Vec3d &v) {
  geometry_msgs::msg::Vector3 output; 
  output.x = v[0]; 
  output.y = v[1]; 
  output.z = v[2]; 
  return output;
}
// Orientation is identity: PoseArray carries positions only.
std::unique_ptr<geometry_msgs::msg::PoseArray> poses(
  const std_msgs::msg::Header &header, const std::vector<ObjectPoint> &points) {
  auto output = std::make_unique<geometry_msgs::msg::PoseArray>(); 
  output->header = header;
  for (const auto &p : points) {
    geometry_msgs::msg::Pose pose; 
    pose.position = point(p.position); 
    pose.orientation.w = 1;
    output->poses.push_back(pose);
  }
  return output;
}

msg::Detection message(const Detection &box, const std::vector<std::string> &labels) {
  msg::Detection output;
  output.class_id = box.class_id;
  output.label = labels.at(box.class_id);
  output.confidence = box.score;
  output.bbox = {box.box.x, box.box.y, box.box.width, box.box.height};
  return output;
}

std::unique_ptr<msg::DetectionArray> detections(const std_msgs::msg::Header &header,
  const std::vector<Detection> &boxes, const std::vector<std::string> &labels) {
  auto output = std::make_unique<msg::DetectionArray>();
  output->header = header;
  for (const auto &box : boxes) output->detections.push_back(message(box, labels));
  return output;
}

// The survey records pucks by AprilTag id: only boxes with one usable id, one box per id (best kept, as
// associate_pucks does without every_box).
std::vector<TaggedPuck> identified(const std::vector<TaggedPuck> &pucks) {
  std::vector<TaggedPuck> out;
  for (const auto &p : pucks) if (p.id >= 0) out.push_back(p);
  std::sort(out.begin(), out.end(), [](const TaggedPuck &a, const TaggedPuck &b) {
    if (a.id != b.id) return a.id < b.id;
    if (a.shape_from_tags != b.shape_from_tags) return a.shape_from_tags;
    return a.score > b.score;
  });
  out.erase(std::unique(out.begin(), out.end(), [](const TaggedPuck &a, const TaggedPuck &b) { return a.id == b.id; }),
    out.end());
  return out;
}

std::unique_ptr<msg::PuckObservationArray> survey_message(const std_msgs::msg::Header &header,
  const std::vector<TaggedPuck> &pucks) {
  auto output = std::make_unique<msg::PuckObservationArray>();
  output->header = header;
  for (const auto &p : pucks) {
    msg::PuckObservation o;
    o.detection.class_id = p.id;
    o.detection.label = std::string(p.color == PuckColor::kRed ? "red_" : "green_") +
      (p.shape == PuckShape::kCylinder ? "cylinder" : "prism") + (p.shape_from_tags ? "_tags" : "");
    o.detection.confidence = p.score;
    o.detection.bbox = {p.box.x, p.box.y, p.box.width, p.box.height};
    o.puck_id = p.id;
    o.tag_valid = true;
    o.tags = p.tags;
    if (p.axis) {
      o.axis_valid = true;
      o.center = point(p.axis->center);
      o.axis = vector(p.axis->axis);
      o.shelf_normal = vector(p.axis->normal);
      o.standing = p.axis->standing;
      o.height_m = p.axis->height_m;
      o.tags = p.axis->tags;
      o.tag_fit_error_deg = p.axis->fit_error_deg;
    }
    output->observations.push_back(o);
  }
  return output;
}

std::unique_ptr<msg::PuckObservationArray> puck_message(const std_msgs::msg::Header &header,
  const std::vector<PuckObservation> &pucks) {
  auto output = std::make_unique<msg::PuckObservationArray>();
  output->header = header;
  for (const auto &puck : pucks) {
    msg::PuckObservation observation;
    observation.detection.class_id = puck.detection.class_id;
    observation.detection.label = "puck_tag_" + std::to_string(puck.detection.class_id);
    observation.detection.confidence = puck.detection.score;
    observation.detection.bbox = {puck.detection.box.x, puck.detection.box.y,
      puck.detection.box.width, puck.detection.box.height};
    observation.puck_id = puck.detection.class_id;
    observation.tag_valid = true;
    if (puck.axis) {
      observation.axis_valid = true;
      observation.center = point(puck.axis->center);
      observation.axis = vector(puck.axis->axis);
      observation.shelf_normal = vector(puck.axis->normal);
      observation.standing = puck.axis->standing;
      observation.height_m = puck.axis->height_m;
      observation.plane_rms_m = puck.axis->plane_rms_m;
      observation.points = puck.axis->points;
      observation.tags = puck.axis->tags;
      observation.tag_fit_error_deg = puck.axis->fit_error_deg;
    }
    output->observations.push_back(observation);
  }
  return output;
}
}  // namespace

PerceptionMaster::PerceptionMaster() : Node("perception_master") {
  configure();
  connect();
  RCLCPP_INFO(get_logger(), "StateEstimator: %s -> /perception/state/odom in %s (fusion: robot_localization)",
    odometry_topic_.c_str(), map_frame_.c_str());
  RCLCPP_INFO(get_logger(), "PerceptionMaster ready; D435 classes=%zu, "
    "executor_threads=%d", d435_labels_.size(), executor_threads_);
}

template <typename T> T PerceptionMaster::param(const std::string &name) {
  // No C++ defaults: perception.yaml is the only source of configuration values.
  try { return declare_parameter<T>(name); }
  catch (const std::exception &e) {
    throw std::invalid_argument("provide " + name + " with the right type in perception.yaml "
      "(--params-file): " + e.what());
  }
}

double PerceptionMaster::positive_param(const std::string &name) {
  const double value = param<double>(name);
  if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name + " must be finite and positive");
  return value;
}

DetectorConfig PerceptionMaster::yolo_param(const std::string &prefix, std::vector<std::string> &labels) {
  const auto share_path = [](const std::string &path) {
    return std::filesystem::path(path).is_absolute() ? path :
      ament_index_cpp::get_package_share_directory("craft_perception") + "/" + path;
  };
  DetectorConfig config;
  config.backend = param<std::string>(prefix + ".backend");
  if (config.backend != "ncnn" && config.backend != "tensorrt") {
    throw std::invalid_argument(prefix + ".backend must be ncnn or tensorrt");
  }
  // perception.yaml says tensorrt for the Jetson GPU (2026-11-12, every camera model); a build without TensorRT
  // (laptop, CI) runs the same model on ncnn instead of refusing to start.
  if (config.backend == "tensorrt" && !tensorrt_available()) {
    RCLCPP_WARN(get_logger(), "%s.backend tensorrt: built without TensorRT, using ncnn", prefix.c_str());
    config.backend = "ncnn";
  }
  config.model_dir = share_path(param<std::string>(prefix + ".model_dir"));
  config.engine_path = share_path(param<std::string>(prefix + ".engine"));
  // Blob names differ per export: ncnn in0/out0, Ultralytics ONNX -> TensorRT images/output0.
  const std::string blobs = config.backend == "tensorrt" ? ".tensorrt_blobs" : ".ncnn_blobs";
  // Two names = detect model; a third (prototype masks) makes it a seg model.
  const auto names = param<std::vector<std::string>>(prefix + blobs);
  if (names.size() != 2 && names.size() != 3) {
    throw std::invalid_argument(prefix + blobs + " must be [input, output] or [input, output, protos]");
  }
  config.input_blob = names[0];
  config.output_blob = names[1];
  if (names.size() == 3) config.proto_blob = names[2];
  config.input_size = param<int64_t>(prefix + ".input_size");
  labels = param<std::vector<std::string>>(prefix + ".class_names");
  config.classes = labels.size();
  config.threads = param<int64_t>(prefix + ".ncnn_threads");
  config.confidence = param<double>(prefix + ".confidence");
  config.iou = param<double>(prefix + ".nms_iou");
  config.max_detections = param<int64_t>(prefix + ".max_detections");
  return config;
}

void PerceptionMaster::configure() {
  executor_threads_ = param<int64_t>("executor_threads");
  if (executor_threads_ < 2) throw std::invalid_argument("executor_threads must be >=2");
  max_age_ = positive_param("max_input_age_s");
  for (const std::string name : kTasks) {
    const bool on = param<bool>("schedule." + name + ".enabled");
    const double hz = positive_param("schedule." + name + ".hz");
    const double seconds = 1.0 / hz;
    const double maximum = std::chrono::duration<double>(std::chrono::nanoseconds::max()).count();
    if (!std::isfinite(seconds) || seconds < 1e-9 || seconds >= maximum) {
      throw std::invalid_argument("schedule." + name + ".hz must yield a timer period >=1ns");
    }
    schedules_.emplace(name, ModuleSchedule{on, hz});
  }
  for (const char *name : {"object_detector", "obstacle_detector"}) {
    if (enabled(name) && !enabled("d435_yolo")) {
      RCLCPP_WARN(get_logger(), "%s is enabled but d435_yolo is disabled; it will never get boxes", name);
    }
  }
  cameras_[kD435].frame_id = param<std::string>("d435.frame");
  cameras_[kD435].shm = param<std::string>("d435.shm");

  D435Config d435;
  d435.use_yolo = enabled("d435_yolo");
  d435.use_apriltag = enabled("apriltag");
  d435.use_puck = enabled("puck_axis");
  d435.yolo = yolo_param("d435.yolo", d435_labels_);
  d435.apriltag_family = param<std::string>("d435.apriltag_family");
  const auto registry_file = param<std::string>("tag_registry_file");
  d435.tag_registry = load_tag_registry(std::filesystem::path(registry_file).is_absolute() ?
    registry_file : ament_index_cpp::get_package_share_directory("craft_perception") + "/" + registry_file);
  d435.puck_tag_box_scale = positive_param("d435.puck_axis.tag_box_scale");
  d435.object_inner_fraction = param<double>("d435.object_detector.inner_fraction");
  for (const auto id : param<std::vector<int64_t>>("d435.obstacle_detector.class_ids")) {
    d435.obstacle_class_ids.push_back(id);
  }
  d435.obstacle_max_range_m = param<double>("d435.obstacle_detector.max_range_m");
  d435.obstacle_near_quantile = param<double>("d435.obstacle_detector.near_quantile");
  d435.puck.grow = param<double>("d435.puck_axis.grow");
  d435.puck.plane_threshold_m = param<double>("d435.puck_axis.plane_threshold_m");
  d435.puck.min_height_m = param<double>("d435.puck_axis.min_height_m");
  d435.puck.max_height_m = param<double>("d435.puck_axis.max_height_m");
  d435.puck.standing_height_m = param<double>("d435.puck_axis.standing_height_m");
  d435.puck.min_depth_m = param<double>("d435.puck_axis.min_depth_m");
  d435.puck.max_depth_m = param<double>("d435.puck_axis.max_depth_m");
  d435.puck.min_points = param<int64_t>("d435.puck_axis.min_points");
  d435.puck.ransac_iterations = param<int64_t>("d435.puck_axis.ransac_iterations");
  d435.puck_method = param<std::string>("d435.puck_axis.method");
  {
    auto &t = d435.puck_tag;
    const std::string tag = "d435.puck_axis.tag.";
    t.tag_size_m = param<double>(tag + "tag_size_m");
    t.length_m = param<double>(tag + "length_m");
    t.width_m = param<double>(tag + "width_m");
    const auto up = param<std::vector<double>>(tag + "up_optical");
    if (up.size() != 3) throw std::invalid_argument(tag + "up_optical needs 3 values");
    t.up = {up[0], up[1], up[2]};
    t.standing_max_tilt_deg = param<double>(tag + "standing_max_tilt_deg");
    t.max_fit_error_deg = param<double>(tag + "max_fit_error_deg");
    t.max_center_spread_m = param<double>(tag + "max_center_spread_m");
    t.center_spread_per_m = param<double>(tag + "center_spread_per_m");
    t.center_weight_deg_per_mm = param<double>(tag + "center_weight_deg_per_mm");
    t.box_iou_weight_deg = param<double>(tag + "box_iou_weight_deg");
    t.min_box_iou = param<double>(tag + "min_box_iou");
    t.box_border_px = param<double>(tag + "box_border_px");
    t.ambiguity_margin_deg = param<double>(tag + "ambiguity_margin_deg");
    t.max_reprojection_px = param<double>(tag + "max_reprojection_px");
    t.max_tags = param<int64_t>(tag + "max_tags");
    t.shelf_prior_weight = param<double>(tag + "shelf_prior_weight");
  }

  // <prefix>.{red,green}_{cylinder,prism}_class_id, each an index of that survey's yolo.class_names.
  const auto puck_class_map = [this](const std::string &prefix, const std::vector<std::string> &labels) {
    const auto class_id = [&](const char *name) {
      const int id = param<int64_t>(prefix + "." + name);
      if (id < 0 || id >= static_cast<int>(labels.size())) {
        throw std::invalid_argument(prefix + "." + name + " must index its yolo.class_names");
      }
      return id;
    };
    return PuckClassMap{class_id("red_cylinder_class_id"), class_id("red_prism_class_id"),
                        class_id("green_cylinder_class_id"), class_id("green_prism_class_id")};
  };
  // <prefix>.tag_roi_grow / tag_roi_upscale / shape_rule.*: where tags are searched and how they set the shape.
  const auto tag_search = [this](const std::string &prefix) {
    PuckTagSearch s;
    s.grow = param<double>(prefix + ".tag_roi_grow");
    s.upscale = static_cast<int>(param<int64_t>(prefix + ".tag_roi_upscale"));
    s.rule.enabled = param<bool>(prefix + ".shape_rule.enabled");
    s.rule.red_only = param<bool>(prefix + ".shape_rule.red_only");
    s.rule.face_on_side_ratio = param<double>(prefix + ".shape_rule.face_on_side_ratio");
    s.rule.face_on_skew_deg = param<double>(prefix + ".shape_rule.face_on_skew_deg");
    s.rule.face_on_max_angle_deg = param<double>(prefix + ".shape_rule.face_on_max_angle_deg");
    s.rule.side_face_aspect = param<double>(prefix + ".shape_rule.side_face_aspect");
    if (!(s.grow >= 1.0 && s.grow <= 4.0) || s.upscale < 1 || s.upscale > 4 ||
        !(s.rule.face_on_side_ratio > 0 && s.rule.face_on_side_ratio <= 1) || !(s.rule.face_on_skew_deg >= 0) ||
        !(s.rule.face_on_max_angle_deg >= 0 && s.rule.face_on_max_angle_deg < 90) || !(s.rule.side_face_aspect >= 1)) {
      throw std::invalid_argument(prefix + ": tag_roi_grow 1..4, tag_roi_upscale 1..4, shape_rule values out of range");
    }
    return s;
  };
  // Puck survey on the D435: its own YOLO (puck model), loaded only when the task is enabled.
  if (enabled("puck_survey")) {
    const auto yolo = yolo_param("d435.puck_survey.yolo", puck_labels_);
    puck_classes_ = puck_class_map("d435.puck_survey", puck_labels_);
    puck_min_score_ = param<double>("d435.puck_survey.min_score");
    puck_tag_search_ = tag_search("d435.puck_survey");
    if (!puck_yolo_.configure(yolo)) {
      throw std::invalid_argument("cannot load the " + yolo.backend + " model for d435.puck_survey.yolo (see stderr)");
    }
    if (!puck_tags_.configure(d435.apriltag_family)) {
      throw std::invalid_argument("invalid d435.apriltag_family for puck_survey");
    }
  }
  if (enabled("traffic_light") || enabled("uppercam_puck")) uppercam_frame_ = param<std::string>("uppercam.frame");
  // Traffic light on the upper cam: its own YOLO, loaded only when the task is enabled.
  if (enabled("traffic_light")) {
    if (!barcode_decoder_available()) {
      mission_event("ERROR", "BARCODE_DECODER_MISSING", "built without zxing-cpp (libzxing-dev): the barcode will not be read");
    }
    barcode_locator_.work_width = param<int64_t>("uppercam.barcode.work_width");
    barcode_locator_.min_gradient = static_cast<float>(param<double>("uppercam.barcode.min_gradient"));
    barcode_locator_.max_regions = param<int64_t>("uppercam.barcode.max_regions");
    barcode_decode_ = param<bool>("uppercam.barcode.decode");
    barcode_full_frame_every_ = param<int64_t>("uppercam.barcode.full_frame_every");
    barcode_match_.enabled = param<bool>("uppercam.barcode.match.enabled");
    barcode_match_.codes = param<std::vector<std::string>>("uppercam.barcode.match.codes");
    barcode_match_.regions = param<int64_t>("uppercam.barcode.match.regions");
    barcode_match_.min_score = param<double>("uppercam.barcode.match.min_score");
    barcode_match_.min_margin = param<double>("uppercam.barcode.match.min_margin");
    barcode_match_.votes = param<int64_t>("uppercam.barcode.match.votes");
    barcode_match_.min_share = param<double>("uppercam.barcode.match.min_share");
    if (!barcode_decode_ && !barcode_match_.enabled) {
      throw std::invalid_argument("uppercam.barcode: decode and match.enabled are both off, the barcode is never read");
    }
    if (barcode_match_.enabled && (barcode_match_.codes.size() < 2 || barcode_match_.regions < 1 ||
        barcode_match_.votes < 1 || barcode_match_.min_share <= 0.5 || barcode_match_.min_share > 1.0)) {
      throw std::invalid_argument("invalid uppercam.barcode.match configuration in perception.yaml");
    }
    if (!barcode_locator_.valid() || barcode_full_frame_every_ < 0) {
      throw std::invalid_argument("invalid uppercam.barcode configuration in perception.yaml");
    }
    const auto yolo = yolo_param("uppercam.yolo", uppercam_labels_);
    if (!uppercam_yolo_.configure(yolo)) {
      throw std::invalid_argument("cannot load the " + yolo.backend + " model for uppercam.yolo (see stderr)");
    }
    TrafficLightConfig light;
    light.red_class_id = param<int64_t>("uppercam.traffic_light.red_class_id");
    light.yellow_class_id = param<int64_t>("uppercam.traffic_light.yellow_class_id");
    light.green_class_id = param<int64_t>("uppercam.traffic_light.green_class_id");
    light.min_score = param<double>("uppercam.traffic_light.min_score");
    light.yellow_min_score = param<double>("uppercam.traffic_light.yellow_min_score");
    light.min_on_s = param<double>("uppercam.traffic_light.min_on_s");
    light.max_on_s = param<double>("uppercam.traffic_light.max_on_s");
    light.max_gap_s = param<double>("uppercam.traffic_light.max_gap_s");
    light.min_vote_ratio = param<double>("uppercam.traffic_light.min_vote_ratio");
    light.expected_blinks = param<int64_t>("uppercam.traffic_light.expected_blinks");
    light.end_off_s = param<double>("uppercam.traffic_light.end_off_s");
    // yolo: color = YOLO class; pixels: YOLO finds the tower, color measured in its pixels (both kept, pick one).
    const auto source = param<std::string>("uppercam.traffic_light.color_source");
    if (source != "yolo" && source != "pixels") {
      throw std::invalid_argument("uppercam.traffic_light.color_source must be yolo or pixels");
    }
    light.color_source = source == "pixels" ? ColorSource::kPixels : ColorSource::kYolo;
    light.off_class_id = param<int64_t>("uppercam.traffic_light.off_class_id");
    light.locate_min_score = param<double>("uppercam.traffic_light.locate_min_score");
    const std::string px = "uppercam.traffic_light.pixels.";
    light.pixels.bright_v = param<int64_t>(px + "bright_v");
    light.pixels.on_fraction = param<double>(px + "on_fraction");
    light.pixels.ring_fraction = param<double>(px + "ring_fraction");
    light.pixels.ring_min_saturation = param<int64_t>(px + "ring_min_saturation");
    light.pixels.ring_min_pixels = param<int64_t>(px + "ring_min_pixels");
    light.pixels.green_hue_min = param<double>(px + "green_hue_min");
    light.pixels.green_hue_max = param<double>(px + "green_hue_max");
    light.pixels.red_hue_below = param<double>(px + "red_hue_below");
    light.pixels.red_hue_from = param<double>(px + "red_hue_from");
    light.pixels.yellow_hue_max = param<double>(px + "yellow_hue_max");
    light.pixels.red_cut = param<double>(px + "red_cut");
    light.pixels.yellow_cut = param<double>(px + "yellow_cut");
    light.pixels.upright_ratio = param<double>(px + "upright_ratio");
    light.pixels.red_by_position = param<bool>(px + "red_by_position");
    light.pixels.cap_fraction = param<double>(px + "cap_fraction");
    light.pixels.use_ring_hue = param<bool>(px + "use_ring_hue");
    mission_event("INFO", "SIGNAL_COLOR_SOURCE", "color_source=%s", source.c_str());
    for (const int id : {light.red_class_id, light.yellow_class_id, light.green_class_id, light.off_class_id}) {
      if (id >= static_cast<int>(uppercam_labels_.size())) {
        throw std::invalid_argument("uppercam.traffic_light class ids must index uppercam.yolo.class_names");
      }
    }
    if (!traffic_light_.configure(light)) {
      throw std::invalid_argument("invalid uppercam.traffic_light configuration in perception.yaml");
    }
  }
  // Puck survey on the upper cam after the light: both models are loaded here, so the switch at
  // traffic_light complete needs no restart or reload.
  if (enabled("uppercam_puck")) {
    const auto yolo = yolo_param("uppercam.puck_survey.yolo", uppercam_puck_labels_);
    uppercam_puck_classes_ = puck_class_map("uppercam.puck_survey", uppercam_puck_labels_);
    uppercam_puck_min_score_ = param<double>("uppercam.puck_survey.min_score");
    uppercam_puck_tag_search_ = tag_search("uppercam.puck_survey");
    if (!uppercam_puck_yolo_.configure(yolo)) {
      throw std::invalid_argument("cannot load the " + yolo.backend + " model for uppercam.puck_survey.yolo (see stderr)");
    }
    if (!uppercam_puck_tags_.configure(d435.apriltag_family)) {
      throw std::invalid_argument("invalid d435.apriltag_family for uppercam_puck");
    }
  }

  // Puck-survey axis (step 4): same PuckTagAxis settings as d435.puck_axis.tag (up_optical is the upper cam's;
  // the gripper's up depends on the arm pose, TODO).
  if (enabled("puck_survey") && !d435_survey_axis_.configure(d435.puck_tag)) {
    throw std::invalid_argument("invalid d435.puck_axis.tag configuration (puck survey axis)");
  }
  if (enabled("uppercam_puck")) {
    if (!uppercam_survey_axis_.configure(d435.puck_tag)) {
      throw std::invalid_argument("invalid d435.puck_axis.tag configuration (upper-cam puck axis)");
    }
    const auto k = param<std::vector<double>>("uppercam.intrinsics_1920x1080");
    if (k.size() != 4 || !(k[0] > 0 && k[1] > 0)) throw std::invalid_argument("uppercam.intrinsics_1920x1080: [fx, fy, cx, cy]");
    std::copy(k.begin(), k.end(), uppercam_intrinsics_1080_.begin());
  }
  cv::setNumThreads(1); // Executor handles module parallelism; avoid nested CPU pools.
  if (!d435_.configure(d435)) {
    throw std::invalid_argument("invalid d435 configuration in perception.yaml, or cannot load the "
      + d435.yolo.backend + " model for d435.yolo (see stderr)");
  }
  lidar_input_mode_ = param<std::string>("lidar.input_mode");
  if (lidar_input_mode_ != "raw" && lidar_input_mode_ != "merged") {
    throw std::invalid_argument("lidar.input_mode must be raw or merged");
  }
  lidar_config_.target_frame = param<std::string>("lidar.target_frame");
  lidar_config_.bins = param<int64_t>("lidar.bins");
  lidar_config_.range_max = param<double>("lidar.range_max");
  lidar_config_.max_age = param<double>("lidar.max_age");
  lidar_motion_compensation_ = param<bool>("lidar.motion_compensation");
  lidar_config_.first_beam_latest = param<bool>("lidar.first_beam_latest");
  lidar_config_.pose_time_offset = param<double>("lidar.pose_time_offset");
  lidar_config_.front_fov_deg = param<double>("lidar.front_fov_deg");
  lidar_config_.rear_fov_deg = param<double>("lidar.rear_fov_deg");
  lidar_config_.fov_near_m = param<double>("lidar.fov_near_m");
  lidar_config_.body_radius_m = param<double>("lidar.body_radius_m");
  lidar_config_.body_center_x = param<double>("lidar.body_center_x");
  auto &roi = lidar_config_.floor_roi;
  roi.enabled = param<bool>("lidar.floor_roi.enabled");
  roi.step_deg = param<double>("lidar.floor_roi.step_deg");
  for (auto [name, side] : {std::pair{"front_left", &roi.front_left}, std::pair{"front_right", &roi.front_right},
                            std::pair{"rear_left", &roi.rear_left}, std::pair{"rear_right", &roi.rear_right}}) {
    side->clear_deg = param<double>(std::string("lidar.floor_roi.") + name + "_clear_deg");
    side->max_range_m = param<std::vector<double>>(std::string("lidar.floor_roi.") + name + "_max_range_m");
  }
  if (!lidar_.configure(lidar_config_, positive_param("lidar_obstacle.max_range_m"),
                        param<int64_t>("lidar_obstacle.stride"))) {
    throw std::invalid_argument("invalid lidar / lidar_obstacle configuration");
  }
  obstacle_filter_enabled_ = param<bool>("lidar_obstacle.filter.enabled");
  brake_decel_ = positive_param("lidar_obstacle.brake_decel");
  brake_window_s_ = positive_param("lidar_obstacle.brake_window_s");
  brake_before_s_ = positive_param("lidar_obstacle.brake_before_s");
  brake_after_s_ = positive_param("lidar_obstacle.brake_after_s");
  LidarObstacleFilterConfig filter;
  filter.wall_margin_m = param<double>("lidar_obstacle.filter.wall_margin_m");
  filter.cell_m = param<double>("lidar_obstacle.filter.cell_m");
  filter.persist_frames = static_cast<int>(param<int64_t>("lidar_obstacle.filter.persist_frames"));
  filter.neighbor_cells = static_cast<int>(param<int64_t>("lidar_obstacle.filter.neighbor_cells"));
  filter.min_cluster_points = static_cast<int>(param<int64_t>("lidar_obstacle.filter.min_cluster_points"));
  filter.memory_s = param<double>("lidar_obstacle.filter.memory_s");
  obstacle_odom_frame_ = param<std::string>("lidar_obstacle.odom_frame");
  if (!obstacle_filter_.configure(filter)) throw std::invalid_argument("invalid lidar_obstacle.filter configuration");
  odometry_topic_ = param<std::string>("odometry_source_topic");
  map_frame_ = param<std::string>("map_frame");
  map_correction_speed_ = param<double>("map_correction.speed");
  map_correction_yaw_rate_ = param<double>("map_correction.yaw_rate_deg") * M_PI / 180.0;
  map_correction_snap_m_ = param<double>("map_correction.snap_m");
  map_correction_snap_rad_ = param<double>("map_correction.snap_deg") * M_PI / 180.0;
  map_correction_still_v_ = param<double>("map_correction.still_speed");
  map_correction_still_w_ = param<double>("map_correction.still_yaw_rate_deg") * M_PI / 180.0;

  imu_enabled_ = param<bool>("imu.enabled");
  imu_frame_ = param<std::string>("imu.output_frame");
  wheel_still_linear_ = positive_param("imu.wheel_still_linear");
  wheel_still_angular_ = positive_param("imu.wheel_still_angular");
  const auto primary = param<std::string>("imu.primary");
  if (primary != "wheel_clock" && primary != "lm_arm") {
    throw std::invalid_argument("imu.primary must be wheel_clock or lm_arm");
  }
  const ImuProcessorConfig imu{primary == "wheel_clock" ? 0u : 1u,
    static_cast<int>(param<int64_t>("imu.calibration_samples")), param<double>("imu.bias_tracking_gain"),
    positive_param("imu.still_rate"), positive_param("imu.gyro_variance"), param<double>("imu.max_disagreement"),
    param<double>("imu.max_disagreement_ratio"), positive_param("imu.max_age_s"),
    positive_param("imu.max_yaw_rate"), positive_param("imu.max_yaw_accel"), param<double>("imu.jump_noise"),
    static_cast<int>(param<int64_t>("imu.disagreement_samples")),
    static_cast<int>(param<int64_t>("imu.frozen_samples"))};
  if (!imu_.configure(imu)) throw std::invalid_argument("invalid imu configuration");

  localization_enabled_ = param<bool>("localization.enabled");
  const LocalizationMonitorConfig monitor{positive_param("localization.rejection_sigma"),
    positive_param("localization.dead_reckoning_m"), param<double>("localization.imu_learning.stable_s"),
    positive_param("localization.imu_learning.bias_time_constant_s"), param<double>("localization.imu_learning.bias_limit"),
    param<double>("localization.imu_learning.scale_gain"), param<double>("localization.imu_learning.scale_limit"),
    positive_param("localization.imu_learning.min_turn"), param<double>("localization.imu_learning.settle_s"),
    positive_param("localization.imu_learning.straight_rate"), positive_param("localization.imu_learning.turn_rate"),
    positive_param("localization.imu_learning.min_bias_segment_s"),
    static_cast<int>(param<int64_t>("localization.misfit_checks"))};
  if (!localization_.configure(monitor)) throw std::invalid_argument("invalid localization configuration");
  const ScanRelocalizerConfig relocalizer{positive_param("localization.relocalize.window_m"),
    positive_param("localization.relocalize.window_deg") * M_PI / 180.0, positive_param("localization.relocalize.step_m"),
    positive_param("localization.relocalize.step_deg") * M_PI / 180.0, positive_param("localization.relocalize.sigma_m"),
    positive_param("localization.relocalize.min_score"),
    static_cast<int>(param<int64_t>("localization.relocalize.max_points")),
    static_cast<int>(param<int64_t>("localization.relocalize.occupied_threshold"))};
  if (!relocalizer_.configure(relocalizer)) throw std::invalid_argument("invalid localization.relocalize configuration");
  lidar_odometry_enabled_ = param<bool>("lidar_odometry.enabled");
  if (lidar_odometry_enabled_) {
    const ScanOdometryConfig scan_odometry{{positive_param("lidar_odometry.window_m"),
      positive_param("lidar_odometry.window_deg") * M_PI / 180.0, positive_param("lidar_odometry.step_m"),
      positive_param("lidar_odometry.step_deg") * M_PI / 180.0, positive_param("lidar_odometry.sigma_m"),
      positive_param("lidar_odometry.min_score"), static_cast<int>(param<int64_t>("lidar_odometry.max_points")), 50},
      positive_param("lidar_odometry.grid_resolution_m"), positive_param("lidar_odometry.grid_half_extent_m"),
      positive_param("lidar_odometry.keyframe_m"), positive_param("lidar_odometry.keyframe_deg") * M_PI / 180.0};
    if (!scan_odometry_.configure(scan_odometry)) throw std::invalid_argument("invalid lidar_odometry configuration");
    const auto twist = param<std::vector<double>>("lidar_odometry.twist_variance");
    if (twist.size() != 3 || !(twist[0] > 0 && twist[1] > 0 && twist[2] > 0)) {
      throw std::invalid_argument("lidar_odometry.twist_variance needs 3 positive values: vx, vy, vyaw");
    }
    std::copy(twist.begin(), twist.end(), lidar_twist_variance_.begin());
    lidar_odom_frame_ = param<std::string>("lidar_odometry.odom_frame");
    lidar_base_frame_ = param<std::string>("lidar_odometry.base_frame");
    if (!imu_enabled_) RCLCPP_WARN(get_logger(), "lidar_odometry without imu: yaw predicted from the last scans only");
  }
  const auto variance = param<std::vector<double>>("localization.relocalize.pose_variance");
  if (variance.size() != 3) throw std::invalid_argument("localization.relocalize.pose_variance needs x, y, yaw");
  std::copy(variance.begin(), variance.end(), relocalized_variance_.begin());
}

void PerceptionMaster::connect() {
  for (int camera = 0; camera < kCameraCount; ++camera) {
    cameras_[camera].reader = std::make_unique<SharedFrameReader>(cameras_[camera].shm, camera == kD435);
  }
  apriltag_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/perception/vision/d435/apriltag", 10);
  detection_pub_ = create_publisher<msg::DetectionArray>("/perception/vision/d435/detections", 10);
  puck_pub_ = create_publisher<msg::PuckObservationArray>("/perception/vision/d435/puck_observations", 10);
  // Latched: the mission may subscribe after the light has finished blinking.
  traffic_light_pub_ = create_publisher<msg::TrafficLight>("/perception/vision/uppercam/traffic_light",
    rclcpp::QoS(1).transient_local());
  mission_event_pub_ = create_publisher<std_msgs::msg::String>("/perception/mission/events", 50);
  // Confirmed Code 128 text (P2-01..04; anything else = unregistered -> mission error). Latched.
  barcode_pub_ = create_publisher<std_msgs::msg::String>("/perception/vision/uppercam/barcode",
    rclcpp::QoS(1).transient_local());
  // Upper-cam colour size for the bridge (2026-11-14): the light is counted at the normal size (640x360, full rate);
  // only after SIGNAL_COMPLETE, while the barcode is not confirmed, 1080p (the barcode is unreadable below it). Latched.
  hires_pub_ = create_publisher<std_msgs::msg::Bool>("/perception/uppercam/hires", rclcpp::QoS(1).transient_local());
  publish_hires(false);
  traffic_light_reset_sub_ = create_subscription<std_msgs::msg::Empty>("/perception/traffic_light/reset", 10,
    [this](std_msgs::msg::Empty::ConstSharedPtr) {
      std::lock_guard<std::mutex> lock(traffic_light_mutex_);
      traffic_light_.reset();
      barcode_.reset();
      barcode_pub_->publish(std_msgs::msg::String());  // empty = not read yet
      publish_hires(false);
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = uppercam_frame_;
      publish_traffic_light(header);
      mission_event("INFO", "SIGNAL_RESET", "signal counter and barcode reset");
    });
  object_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/perception/vision/d435/object_detection", 10);
  obstacle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/perception/vision/d435/obstacle", 10);
  lidar_obstacle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/perception/lidar/obstacle", 10);
  brake_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/control/wheel/odom", rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr input) {
      const int64_t t = rclcpp::Time(input->header.stamp).nanoseconds();
      const double v = std::fabs(input->twist.twist.linear.x);
      merge_v_ = input->twist.twist.linear.x; merge_w_wheel_ = input->twist.twist.angular.z;  // for the scan merge
      // Turning faster than localization_scan.max_yaw_rate: remembered for publish_localization_scan().
      if (localization_max_yaw_rate_ > 0.0 && std::fabs(input->twist.twist.angular.z) > localization_max_yaw_rate_) turning_at_ns_ = t;
      if (!brake_speeds_.empty() && t <= brake_speeds_.back().first) brake_speeds_.clear();
      brake_speeds_.emplace_back(t, v);
      const int64_t window = static_cast<int64_t>(brake_window_s_ * 1e9);
      while (brake_speeds_.size() > 1 && brake_speeds_.front().first < t - window) brake_speeds_.pop_front();
      const auto &[t0, v0] = brake_speeds_.front();
      if (t - t0 >= window / 2 && (v0 - v) / ((t - t0) * 1e-9) > brake_decel_) {
        if (t - brake_at_ns_.load() > static_cast<int64_t>((brake_before_s_ + brake_after_s_) * 1e9)) brake_at_ns_ = t;
      }
    });
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/perception/state/odom", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/perception/diagnostics", 10);

  for (const Camera camera : {kD435}) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    image_subs_[camera] = create_subscription<std_msgs::msg::UInt64MultiArray>(
      "/vision/d435/frame_meta", rclcpp::QoS(1),
      [this, camera](std_msgs::msg::UInt64MultiArray::ConstSharedPtr input) { receive_frame(camera, input->data); },
      options);
  }
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("/vision/d435/camera_info", 10,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
      std::lock_guard<std::mutex> lock(info_mutex_); info_ = std::move(info);
    });
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr input) {
      std::lock_guard<std::mutex> lock(odom_mutex_); odom_ = std::move(input); ++odom_id_;
    });
  // Raw mode calls LidarPerception directly, just like the camera modules.
  // Merged mode supports a merger already running on a separate sensor computer.
  if (enabled("lidar_obstacle") && obstacle_filter_enabled_) {
    obstacle_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/perception/lidar/map",
      rclcpp::QoS(1).transient_local().reliable(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
        std::lock_guard<std::mutex> lock(obstacle_filter_mutex_); obstacle_filter_.set_map(*map);
      });
    planning_command_sub_ = create_subscription<std_msgs::msg::String>("/planning/command", 10,
      [this](const std_msgs::msg::String &command) {
        if (command.data != "START" && command.data != "RESET") return;
        // Finish any in-flight publication before clearing; old scan callbacks may
        // still arrive afterwards, so the acquisition-time barrier remains active.
        std::lock_guard<std::mutex> cycle_lock(obstacle_cycle_mutex_);
        const auto stamp = now();
        obstacle_reset_stamp_ns_ = stamp.nanoseconds();
        {
          std::lock_guard<std::mutex> lock(obstacle_filter_mutex_);
          obstacle_filter_.reset();  // Persistence and memory only; preserve the map.
        }
        {
          std::lock_guard<std::mutex> lock(scan_mutex_);
          scan_.reset(); rear_scan_.reset();
        }
        geometry_msgs::msg::PoseArray empty;
        empty.header.frame_id = map_frame_;
        empty.header.stamp = stamp;
        lidar_obstacle_pub_->publish(empty);
        RCLCPP_INFO(get_logger(), "%s: obstacle history and memory cleared before planning", command.data.c_str());
      });
    task_command_sub_ = create_subscription<craft_interfaces::msg::TaskCommand>("/planning/task_command", 10,
      [this](const craft_interfaces::msg::TaskCommand &c) {
        using craft_interfaces::msg::TaskCommand;
        if (c.kind != TaskCommand::NAVIGATE || c.operation != TaskCommand::EXECUTE || c.task_id == navigate_task_id_) return;
        navigate_task_id_ = c.task_id;
        std::lock_guard<std::mutex> lock(obstacle_filter_mutex_); obstacle_filter_.clear_memory();
        RCLCPP_INFO(get_logger(), "new drive (task %lu): obstacle memory cleared", static_cast<unsigned long>(c.task_id));
      });
  }
  if (lidar_input_mode_ == "raw" && enabled("lidar_obstacle")) {
    merged_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/lidar/merged", 10);
    rear_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/lidar/rear/scan",
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr input) {
        std::lock_guard<std::mutex> lock(scan_mutex_); rear_scan_ = std::move(input);
      });
  }
  localization_box_radius_ = param<double>("localization_scan.box_radius");
  localization_max_yaw_rate_ = param<double>("localization_scan.max_yaw_rate_deg") * M_PI / 180.0;
  localization_turn_hold_s_ = param<double>("localization_scan.turn_hold_s");
  localization_max_tilt_rate_ = param<double>("localization_scan.max_tilt_rate_deg") * M_PI / 180.0;
  if (!(localization_max_tilt_rate_ >= 0.0)) throw std::invalid_argument("localization_scan.max_tilt_rate_deg must be >= 0");
  localization_max_slope_ = param<double>("localization_scan.max_slope_deg") * M_PI / 180.0;
  if (!(localization_max_slope_ >= 0.0)) throw std::invalid_argument("localization_scan.max_slope_deg must be >= 0");
  if (localization_max_slope_ > 0.0)
    RCLCPP_INFO(get_logger(), "localization scan: none to SLAM on slopes over %.1f deg (mapping)", localization_max_slope_ * 180.0 / M_PI);
  if (!(localization_max_yaw_rate_ >= 0.0) || !(localization_turn_hold_s_ >= 0.0)) {
    throw std::invalid_argument("localization_scan.max_yaw_rate_deg and turn_hold_s must be >= 0");
  }
  if (enabled("lidar_obstacle")) {
    localization_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/lidar/localization", 10);
    localization_boxes_sub_ = create_subscription<geometry_msgs::msg::PoseArray>("/planning/boxes",
      rclcpp::QoS(1).transient_local(), [this](geometry_msgs::msg::PoseArray::ConstSharedPtr input) {
        if (input->header.frame_id != map_frame_) return;
        std::lock_guard<std::mutex> lock(localization_boxes_mutex_); localization_boxes_ = std::move(input);
      });
  }
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_input_mode_ == "raw" ? "/lidar/front/scan" : "/lidar/merged", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr input) {
      std::lock_guard<std::mutex> lock(scan_mutex_); scan_ = std::move(input); ++scan_id_;
    });

  if (imu_enabled_) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/perception/imu/data", rclcpp::SensorDataQoS());
    const char *const topics[] = {"/control/imu/wheel_clock", "/control/imu/lm_arm"};
    for (std::size_t board = 0; board < ImuProcessor::kBoards; ++board) {
      imu_subs_[board] = create_subscription<sensor_msgs::msg::Imu>(topics[board], rclcpp::SensorDataQoS(),
        [this, board](sensor_msgs::msg::Imu::ConstSharedPtr input) { receive_imu(board, *input); }, options);
    }
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/control/wheel/odom",
      rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr input) {
        wheel_odom_ = std::move(input);
      }, options);
  }

  if (localization_enabled_) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    localization_state_pub_ = create_publisher<std_msgs::msg::String>("/perception/localization/state",
      rclcpp::QoS(1).transient_local());
    relocalized_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/perception/localization/relocalized", 10);
    slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/perception/lidar/pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr input) { receive_slam_pose(*input); },
      options);
    global_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/global", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr input) { receive_global_odom(*input); }, options);
    local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/local", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr input) {
        const double t = rclcpp::Time(input->header.stamp).seconds();
        if (local_odom_at_ && t > *local_odom_at_) {
          localization_.moved(std::fabs(input->twist.twist.linear.x) * (t - *local_odom_at_));
          if (localization_.state() == LocalizationState::kRelocalize) relocalize();
        }
        local_odom_at_ = t;
        local_odom_ = std::move(input);
      }, options);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/perception/lidar/map",
      rclcpp::QoS(1).transient_local().reliable(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
        const auto &q = map->info.origin.orientation;
        if (std::fabs(q.z) > 1e-6 || std::fabs(q.x) > 1e-6 || std::fabs(q.y) > 1e-6) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "rotated SLAM map: relocalization off");
          return;
        }
        relocalizer_.set_map(map->info.width, map->info.height, map->info.resolution,
          map->info.origin.position.x, map->info.origin.position.y, map->data);
      }, options);
    relocalization_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/lidar/merged",
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr input) {
        relocalization_scan_ = std::move(input);
      }, options);
    // 1 Hz: does the scan still fit SLAM's map at the EKF pose? Then publish the state.
    localization_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      const auto scan = relocalization_scan_;
      if (scan && !global_poses_.empty() && relocalizer_.has_map()) {
        const auto score = relocalizer_.score(scan_points(*scan), global_poses_.back().pose);
        if (score) {
          localization_.scan_fits(*score >= relocalizer_.min_score());
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "scan-to-map score at the EKF pose: %.2f", *score);
        }
      }
      publish_localization_state();
    }, options.callback_group);
  }

  add_task("d435_yolo", [this](Task &task) { return d435_yolo(task); });
  add_task("apriltag", [this](Task &task) { return d435_apriltag(task); });
  add_task("object_detector", [this](Task &task) { return d435_object_detector(task); });
  add_task("obstacle_detector", [this](Task &task) { return d435_obstacle_detector(task); });
  add_task("puck_axis", [this](Task &task) { return d435_puck_axis(task); });
  if (enabled("traffic_light")) {
    uppercam_detection_pub_ = create_publisher<msg::DetectionArray>("/perception/vision/uppercam/detections", 10);
  }
  if (enabled("uppercam_puck")) {
    uppercam_survey_obs_pub_ = create_publisher<msg::PuckObservationArray>("/perception/vision/uppercam/puck_survey_observations", 10);
    uppercam_puck_detection_pub_ =
      create_publisher<msg::DetectionArray>("/perception/vision/uppercam/puck_detections", 10);
  }
  if (enabled("traffic_light") || enabled("uppercam_puck")) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Bridge publishes best effort (sensor data); its rate is --preview-hz (run_uppercam_bridge.sh: 15).
    uppercam_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      param<std::string>("uppercam.image_topic"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr input) {
        std::lock_guard<std::mutex> lock(uppercam_mutex_);
        uppercam_image_ = std::move(input);
        ++uppercam_received_;
      }, options);
  }
  add_task("traffic_light", [this](Task &task) { return uppercam_traffic_light(task); });
  if (enabled("puck_survey")) {
    puck_detection_pub_ = create_publisher<msg::DetectionArray>("/perception/vision/d435/puck_detections", 10);
    d435_survey_obs_pub_ = create_publisher<msg::PuckObservationArray>("/perception/vision/d435/puck_survey_observations", 10);
  }
  if (enabled("puck_survey") || enabled("uppercam_puck")) {
    // Latched: the mission may subscribe after the pucks were seen.
    puck_survey_pub_ = create_publisher<msg::PuckSurvey>("/perception/vision/d435/puck_survey",
      rclcpp::QoS(1).transient_local());
    puck_survey_reset_sub_ = create_subscription<std_msgs::msg::Empty>("/perception/puck_survey/reset", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(puck_survey_mutex_);
        puck_survey_.reset();
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = cameras_[kD435].frame_id;
        publish_puck_survey(header);
        mission_event("INFO", "PUCK_SURVEY_RESET", "puck survey reset");
      });
  }
  add_task("puck_survey", [this](Task &task) { return d435_puck_survey(task); });
  add_task("uppercam_puck", [this](Task &task) { return uppercam_puck_survey(task); });
  if (lidar_odometry_enabled_) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lidar_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/perception/lidar/odom", 10);
    lidar_odometry_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/lidar/merged",
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr input) { lidar_odometry(*input); },
      options);
  }
  add_task("lidar_obstacle", [this](Task &task) { return lidar_obstacle(task); });
  add_task("state_estimator", [this](Task &task) { return state_estimator(task); });
  diagnostics_at_ = Steady::now();
  diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { diagnostics(); });
}

bool PerceptionMaster::is_fresh_input(const builtin_interfaces::msg::Time &stamp) const {
  const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
  return age >= -0.01 && age <= max_age_;
}

void PerceptionMaster::add_task(const std::string &name, std::function<bool(Task &)> compute) {
  const auto schedule = schedules_.at(name);
  if (!schedule.enabled) return;
  auto task = std::make_shared<Task>(); 
  task->name = name; 
  task->hz = schedule.hz;
  task->group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::weak_ptr<Task> weak = task;
  task->timer = create_wall_timer(std::chrono::duration<double>(1 / schedule.hz), [this, weak, compute] {
    auto current = weak.lock(); 
    if (!current) return;
    const auto start = Steady::now();
    bool processed = false, error = false;
    try { processed = compute(*current); }
    catch (const std::exception &e) {
      error = true;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s: %s", current->name.c_str(), e.what());
    }
    const double ms = std::chrono::duration<double, std::milli>(Steady::now() - start).count();
    std::lock_guard<std::mutex> lock(current->mutex);
    current->runtime_ms = ms; current->maximum_ms = std::max(current->maximum_ms, ms);
    current->processed += processed; current->errors += error; current->skipped += !processed && !error;
    current->overruns += ms > 1000 / current->hz;
  }, task->group);
  tasks_.push_back(task);
}

void PerceptionMaster::receive_frame(Camera camera, const std::vector<uint64_t> &meta) {
  // Subscription only stores small metadata. No image read, allocation or conversion here.
  if (!SharedFrameReader::is_valid_metadata(meta, camera == kD435)) return;
  const auto stamp = rclcpp::Time(static_cast<int64_t>(meta[4]), get_clock()->get_clock_type());
  if (!is_fresh_input(stamp)) return;
  auto &input = cameras_[camera];
  std::lock_guard<std::mutex> lock(input.mutex);
  if (meta == input.metadata) return;
  input.metadata = meta;
  ++input.generation;
}

std::shared_ptr<const Frame> PerceptionMaster::latest_frame(Camera camera) {
  auto &input = cameras_[camera];
  std::lock_guard<std::mutex> lock(input.mutex);
  if (input.metadata.empty()) return {};
  if (input.cached && input.cached->id == input.generation) return input.cached;
  const auto stamp = rclcpp::Time(static_cast<int64_t>(input.metadata[4]), get_clock()->get_clock_type());
  if (!is_fresh_input(stamp)) return {};
  auto data = input.reader->snapshot(input.metadata);
  if (!data) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "frame snapshot skipped (busy writer, overwritten slot or producer unavailable)");
    return {};
  }
  auto frame = std::make_shared<Frame>(); 
  frame->data = std::move(data);
  frame->header.frame_id = input.frame_id; 
  frame->header.stamp = stamp;
  frame->id = input.generation;
  if (camera == kD435) {
    std::lock_guard<std::mutex> info_lock(info_mutex_);
    if (info_ && info_->width == static_cast<uint32_t>(frame->data->bgr.cols) &&
        info_->height == static_cast<uint32_t>(frame->data->bgr.rows) &&
        info_->header.frame_id == frame->header.frame_id) frame->info = info_;
  }
  input.cached = frame;
  return frame;
}

bool PerceptionMaster::consume(const std::shared_ptr<const Frame> &frame, Task &task) {
  if (!frame || frame->id == task.seen || !is_fresh_input(frame->header.stamp)) return false;
  task.seen = frame->id; return true;
}

std::shared_ptr<const DetectedFrame> PerceptionMaster::new_detection(Task &task) {
  std::shared_ptr<const DetectedFrame> result;
  { std::lock_guard<std::mutex> lock(detected_mutex_); result = d435_detected_; }
  if (!result || !consume(result->frame, task)) return {};
  return result;
}

std::optional<Intrinsics> PerceptionMaster::intrinsics(const Frame &frame) {
  if (!frame.info) return {};
  const auto &k = frame.info->k;
  return Intrinsics{k[0], k[4], k[2], k[5]};
}

// Every camera task: newest unseen input -> compute -> drop if it went stale meanwhile -> publish.
// compute returns nullopt for invalid input (nothing is published).
template <typename Compute, typename Publish>
bool PerceptionMaster::on_frame(Camera camera, Task &task, Compute compute, Publish publish) {
  const auto frame = latest_frame(camera);
  if (!consume(frame, task)) return false;
  auto result = compute(frame);
  if (!result || !is_fresh_input(frame->header.stamp)) return false;
  publish(frame, *result);
  return true;
}

// Consumers of d435_yolo: the boxes and the frame they were detected on, plus intrinsics.
template <typename Compute, typename Publish>
bool PerceptionMaster::on_detection(Task &task, Compute compute, Publish publish) {
  const auto detected = new_detection(task);
  if (!detected) return false;
  const auto in = intrinsics(*detected->frame);
  if (!in) return false;
  auto result = compute(*detected->frame->data, detected->boxes, *in);
  if (!result || !is_fresh_input(detected->frame->header.stamp)) return false;
  publish(detected->frame, *result);
  return true;
}

// D435: one YOLO inference per frame; the boxes stay attached to the frame they came from.
bool PerceptionMaster::d435_yolo(Task &task) {
  return on_frame(kD435, task,
    [this](const FramePtr &frame) { return d435_.detect(*frame->data); },
    [this](const FramePtr &frame, std::vector<Detection> &boxes) {
      detection_pub_->publish(detections(frame->header, boxes, d435_labels_));
      auto result = std::make_shared<DetectedFrame>();
      result->frame = frame;
      result->boxes = std::move(boxes);
      std::lock_guard<std::mutex> lock(detected_mutex_);
      d435_detected_ = std::move(result);
    });
}

bool PerceptionMaster::d435_apriltag(Task &task) {
  return on_frame(kD435, task,
    [this](const FramePtr &frame) { return d435_.read_apriltag(*frame->data); },
    [this](const FramePtr &, std::vector<AprilTagDetection> &tags) {
      auto output = std::make_unique<std_msgs::msg::Int32MultiArray>();
      for (const auto &tag : tags) output->data.push_back(tag.id);
      apriltag_pub_->publish(std::move(output));
    });
}

bool PerceptionMaster::d435_object_detector(Task &task) {
  return on_detection(task,
    [this](const SharedFrame &data, const std::vector<Detection> &boxes, const Intrinsics &in) {
      return d435_.detect_objects(data, boxes, in);
    },
    [this](const FramePtr &frame, std::vector<ObjectPoint> &objects) {
      object_pub_->publish(poses(frame->header, objects));
    });
}

bool PerceptionMaster::d435_obstacle_detector(Task &task) {
  return on_detection(task,
    [this](const SharedFrame &data, const std::vector<Detection> &boxes, const Intrinsics &in) {
      return d435_.detect_obstacles(data, boxes, in);
    },
    [this](const FramePtr &frame, std::vector<ObjectPoint> &obstacles) {
      obstacle_pub_->publish(poses(frame->header, obstacles));
    });
}

// Puck identity and ROI come from configured AprilTag IDs; color is not used. Method tag also uses the
// survey's shape per tag id (a lone cylinder end tag is otherwise ambiguous with a cuboid side tag).
bool PerceptionMaster::d435_puck_axis(Task &task) {
  return on_frame(kD435, task,
    [this](const FramePtr &frame) {
      const auto in = intrinsics(*frame);
      if (!in) return std::optional<std::vector<PuckObservation>>{};
      std::unordered_map<int, PuckShape> shapes;  // survey so far, by tag id: tells cylinders (end tags only)
      {
        std::lock_guard<std::mutex> lock(puck_survey_mutex_);
        for (int id = 0; id < 12; ++id) {
          const auto &sighting = puck_survey_.sighting(id);
          if (sighting) shapes[id] = sighting->shape;
        }
      }
      return d435_.puck_axis(*frame->data, *in, shapes);
    },
    [this](const FramePtr &frame, std::vector<PuckObservation> &pucks) {
      puck_pub_->publish(puck_message(frame->header, pucks));
    });
}

// Puck survey: puck YOLO + AprilTags on the same D435 frame; a puck is recorded when a tag (id 0..11)
// lies in its box. Publishes the survey whenever a puck is new or its color/shape reading changed.
bool PerceptionMaster::d435_puck_survey(Task &task) {
  struct Result { std::vector<Detection> boxes; std::vector<TaggedPuck> pucks; };
  return on_frame(kD435, task,
    [this](const FramePtr &frame) -> std::optional<Result> {
      const auto &bgr = frame->data->bgr;
      if (!puck_yolo_.is_valid_input(bgr) || !puck_tags_.is_valid_input(bgr)) return {};
      Result result;
      result.boxes = puck_yolo_.compute_result(bgr);
      const auto &s = puck_tag_search_;
      const auto in = intrinsics(*frame);  // aligned depth: tag 3D position and face normal
      std::vector<PuckTag> tags;
      for (const auto &tag : puck_tags_.compute_result(bgr, puck_boxes(result.boxes, puck_classes_, puck_min_score_),
             s.grow, s.upscale)) {
        PuckTag t{tag.id, tag.center, tag.corners, std::nullopt, std::nullopt};
        if (in) {
          if (const auto d = tag_depth(frame->data->depth, tag.corners, *in)) { t.position = d->center; t.normal = d->normal; }
        }
        tags.push_back(t);
      }
      result.pucks = associate_pucks(result.boxes, tags, puck_classes_, puck_min_score_, s.rule, true);  // every box
      puck_axes(result.pucks, d435_survey_axis_, in, bgr.size());
      return result;
    },
    [this](const FramePtr &frame, Result &result) {
      puck_detection_pub_->publish(detections(frame->header, result.boxes, puck_labels_));
      d435_survey_obs_pub_->publish(survey_message(frame->header, result.pucks));
      record_pucks(frame->header, identified(result.pucks), "d435");
    });
}

bool PerceptionMaster::uppercam_puck_survey(Task &task) {
  if (enabled("traffic_light")) {  // signal model first; pucks only after the 12 blinks
    std::lock_guard<std::mutex> lock(traffic_light_mutex_);
    if (!traffic_light_.complete()) return false;
  }
  sensor_msgs::msg::CompressedImage::ConstSharedPtr image;
  {
    std::lock_guard<std::mutex> lock(uppercam_mutex_);
    if (!uppercam_image_ || uppercam_received_ == task.seen) return false;
    image = uppercam_image_;
    task.seen = uppercam_received_;
  }
  if (!is_fresh_input(image->header.stamp)) return false;
  const cv::Mat bgr = cv::imdecode(cv::Mat(1, static_cast<int>(image->data.size()), CV_8U,
    const_cast<uint8_t *>(image->data.data())), cv::IMREAD_COLOR);
  if (!uppercam_puck_yolo_.is_valid_input(bgr) || !uppercam_puck_tags_.is_valid_input(bgr)) return false;
  const auto boxes = uppercam_puck_yolo_.compute_result(bgr);
  const auto &s = uppercam_puck_tag_search_;
  std::vector<PuckTag> tags;  // JPEG only, no depth: face-on from the tag corners
  for (const auto &tag : uppercam_puck_tags_.compute_result(bgr,
         puck_boxes(boxes, uppercam_puck_classes_, uppercam_puck_min_score_), s.grow, s.upscale)) {
    tags.push_back({tag.id, tag.center, tag.corners, std::nullopt, std::nullopt});
  }
  auto pucks = associate_pucks(boxes, tags, uppercam_puck_classes_, uppercam_puck_min_score_, s.rule, true);  // every box
  // Upper cam: no camera_info; the 1920x1080 intrinsics scaled to this frame (16:9 modes share the FOV).
  const double sx = bgr.cols / 1920.0, sy = bgr.rows / 1080.0;
  const auto &k = uppercam_intrinsics_1080_;
  puck_axes(pucks, uppercam_survey_axis_, Intrinsics{k[0] * sx, k[1] * sy, (k[2] + 0.5) * sx - 0.5, (k[3] + 0.5) * sy - 0.5},
    bgr.size());
  uppercam_puck_detection_pub_->publish(detections(image->header, boxes, uppercam_puck_labels_));
  uppercam_survey_obs_pub_->publish(survey_message(image->header, pucks));
  record_pucks(image->header, identified(pucks), "uppercam");
  return true;
}

void PerceptionMaster::puck_axes(std::vector<TaggedPuck> &pucks, const PuckTagAxis &module,
  const std::optional<Intrinsics> &in, cv::Size image) const {
  if (!in) return;
  for (auto &p : pucks) {
    std::vector<PuckTagAxis::Corners> corners(p.tag_corners.begin(), p.tag_corners.end());
    if (module.is_valid_input(corners, *in)) p.axis = module.compute_result(corners, *in, p.shape, p.box, image);
  }
}

void PerceptionMaster::record_pucks(const std_msgs::msg::Header &header, const std::vector<TaggedPuck> &pucks,
  const char *camera) {
  std::lock_guard<std::mutex> lock(puck_survey_mutex_);
  bool changed = false;
  for (const auto &puck : pucks) {
    const auto before = puck_survey_.sighting(puck.id);
    puck_survey_.add(puck.id, puck.color, puck.shape, puck.score, puck.shape_from_tags);
    const auto &after = puck_survey_.sighting(puck.id);
    if (!before || before->color != after->color || before->shape != after->shape ||
        before->shape_confirmed != after->shape_confirmed) {
      changed = true;
      const cv::Vec3d p = puck.position.value_or(cv::Vec3d(0, 0, 0));
      mission_event("INFO", "PUCK_SEEN", "id=%d color=%s shape=%s(%s) tags=%d score=%.2f xyz=%s%.3f,%.3f,%.3f axis=%s missing=%zu cam=%s",
        puck.id, after->color == PuckColor::kRed ? "red" : "green",
        after->shape == PuckShape::kCylinder ? "cylinder" : "cuboid", after->shape_confirmed ? "tags" : "yolo",
        puck.tags, after->score, puck.position ? "" : "none:", p[0], p[1], p[2],
        !puck.axis ? "none" : puck.axis->standing ? "standing" : "lying",
        puck_survey_.missing_ids().size(), camera);
    }
  }
  if (changed) publish_puck_survey(header);
}

void PerceptionMaster::mission_event(const char *level, const char *code, const char *format, ...) {
  char detail[256];
  va_list args;
  va_start(args, format);
  std::vsnprintf(detail, sizeof(detail), format, args);
  va_end(args);
  if (std::strcmp(level, "ERROR") == 0) RCLCPP_ERROR(get_logger(), "[MISSION] %s %s %s", level, code, detail);
  else if (std::strcmp(level, "WARN") == 0) RCLCPP_WARN(get_logger(), "[MISSION] %s %s %s", level, code, detail);
  else RCLCPP_INFO(get_logger(), "[MISSION] %s %s %s", level, code, detail);
  if (!mission_event_pub_) return;  // configure() runs before the publishers exist
  std_msgs::msg::String output;
  output.data = std::string(level) + "|" + code + "|" + detail;
  mission_event_pub_->publish(output);
}

// Caller holds puck_survey_mutex_.
void PerceptionMaster::publish_puck_survey(const std_msgs::msg::Header &header) {
  auto output = std::make_unique<msg::PuckSurvey>();
  output->header = header;
  for (int id = 0; id < 12; ++id) {
    const auto &sighting = puck_survey_.sighting(id);
    if (!sighting) continue;
    output->ids.push_back(id);
    output->colors.push_back(sighting->color == PuckColor::kRed ? msg::PuckSurvey::RED : msg::PuckSurvey::GREEN);
    output->shapes.push_back(sighting->shape == PuckShape::kCylinder ? msg::PuckSurvey::CYLINDER : msg::PuckSurvey::CUBOID);
    output->scores.push_back(sighting->score);
    output->shape_confirmed.push_back(sighting->shape_confirmed);
  }
  output->complete = puck_survey_.complete();
  puck_survey_pub_->publish(std::move(output));
}

// Traffic light: newest upper-cam JPEG -> YOLO -> blink counter (camera stamps as its clock).
// Once 12 blinks are in, frames are no longer decoded or inferred until a reset.
bool PerceptionMaster::uppercam_traffic_light(Task &task) {
  sensor_msgs::msg::CompressedImage::ConstSharedPtr image;
  {
    std::lock_guard<std::mutex> lock(uppercam_mutex_);
    if (!uppercam_image_ || uppercam_received_ == task.seen) return false;
    image = uppercam_image_;
    task.seen = uppercam_received_;
  }
  bool need_signal, need_barcode;
  {
    std::lock_guard<std::mutex> lock(traffic_light_mutex_);
    need_signal = !traffic_light_.complete();
    need_barcode = !need_signal && !barcode_.confirmed();  // after the light: frames are 1080p only then
  }
  if (!need_signal && !need_barcode) return false;
  if (!is_fresh_input(image->header.stamp)) return false;
  const cv::Mat bgr = cv::imdecode(cv::Mat(1, static_cast<int>(image->data.size()), CV_8U,
    const_cast<uint8_t *>(image->data.data())), cv::IMREAD_COLOR);
  if (!uppercam_yolo_.is_valid_input(bgr)) return false;
  if (need_barcode) {
    // Whole frame every full_frame_every frames, plus the candidate regions (upscaled when small) every frame.
    // uppercam.barcode.decode false (user 2026-11-14: the label is always one of the mission codes): no decoder, the
    // pattern vote alone.
    std::vector<std::string> texts;
    const auto regions = find_barcode_regions(bgr, barcode_locator_);
    if (barcode_decode_) {
      if (barcode_full_frame_every_ > 0 && barcode_frames_++ % barcode_full_frame_every_ == 0) texts = decode_code128(bgr);
      for (auto &t : decode_code128_regions(bgr, regions)) {
        if (std::find(texts.begin(), texts.end(), t) == texts.end()) texts.push_back(std::move(t));
      }
    }
    // Nothing decoded: pattern-match the four mission codes on the best regions (too small / bent to decode).
    std::optional<KnownCodeMatch> match;
    if (texts.empty() && barcode_match_.enabled) {
      const std::vector<cv::Rect> top(regions.begin(),
        regions.begin() + std::min<std::size_t>(barcode_match_.regions, regions.size()));
      match = match_known_code128(bgr, top, barcode_match_.codes);
      if (match && (match->score < barcode_match_.min_score || match->margin < barcode_match_.min_margin)) match.reset();
    }
    std::lock_guard<std::mutex> lock(traffic_light_mutex_);
    const bool confirmed = barcode_.update(texts) ||
      (match && barcode_.vote(match->text, barcode_match_.votes, barcode_match_.min_share));
    if (confirmed) {
      publish_hires(false);
      std_msgs::msg::String output;
      output.data = *barcode_.text();
      if (barcode_.confirmed_by_vote()) {
        const auto &v = barcode_.votes();
        mission_event("INFO", "BARCODE_PATTERN_VOTE", "code=%s votes=%d (not decoded: pattern match of %zu codes)",
          output.data.c_str(), v.at(output.data), barcode_match_.codes.size());
      }
      barcode_pub_->publish(output);
      const auto order = barcode_.order();
      if (order) {
        mission_event("INFO", "BARCODE_OK", "code=%s order=%s", output.data.c_str(),
          *order == SortOrder::kAscending ? "ascending" : "descending");
      } else {
        mission_event("ERROR", "BARCODE_UNREGISTERED", "code=%s (expected P2-01..P2-04)", output.data.c_str());
      }
    }
  }
  if (!need_signal) return true;
  const auto boxes = uppercam_yolo_.compute_result(bgr);
  const auto &header = image->header;
  uppercam_detection_pub_->publish(detections(header, boxes, uppercam_labels_));
  std::lock_guard<std::mutex> lock(traffic_light_mutex_);
  if (traffic_light_.complete()) return true;  // a reset raced the inference: fine either way
  const int discarded = traffic_light_.discarded_cycles();
  const auto blink = traffic_light_.update(rclcpp::Time(header.stamp).seconds(), boxes, bgr);
  if (traffic_light_.discarded_cycles() != discarded) {  // wrong count: start over on the next cycle
    mission_event("WARN", "SIGNAL_CYCLE_DISCARDED", "%s; waiting for the next cycle (discarded=%d)",
      traffic_light_.last_discard().c_str(), traffic_light_.discarded_cycles());
    publish_traffic_light(header);
    return true;
  }
  const auto counts = traffic_light_.counts();
  if (blink) {
    mission_event("INFO", "SIGNAL_BLINK", "(%d/%d) color=%s total R=%d Y=%d G=%d on=%.2fs votes=%d/%d",
      counts.total(), traffic_light_.expected_blinks(), to_string(blink->color),
      counts.red(), counts.yellow(), counts.green(), blink->end_s - blink->start_s, blink->votes, blink->frames);
  }
  if (traffic_light_.complete()) {  // the light stayed dark for end_off_s after a valid 12
    mission_event("INFO", "SIGNAL_COMPLETE", "(%d/%d) total R=%d Y=%d G=%d",
      counts.total(), traffic_light_.expected_blinks(), counts.red(), counts.yellow(), counts.green());
    if (!barcode_.confirmed()) publish_hires(true);  // now the barcode: 1080p until it is confirmed
  }
  if (blink || traffic_light_.complete()) publish_traffic_light(header);
  return true;
}

// Caller holds traffic_light_mutex_.
void PerceptionMaster::publish_traffic_light(const std_msgs::msg::Header &header) {
  auto output = std::make_unique<msg::TrafficLight>();
  output->header = header;
  const auto counts = traffic_light_.counts();
  output->red = counts.red();
  output->yellow = counts.yellow();
  output->green = counts.green();
  for (const auto &b : traffic_light_.blinks()) output->sequence.push_back(static_cast<uint8_t>(b.color));
  output->complete = traffic_light_.complete();
  output->valid = traffic_light_.valid();
  output->discarded_cycles = traffic_light_.discarded_cycles();
  traffic_light_pub_->publish(std::move(output));
}

std::optional<lidar_processing::Pose2D> PerceptionMaster::lidar_pose(const std::string &frame) {
  try {
    const auto tf = tf_buffer_->lookupTransform(lidar_config_.target_frame, frame, tf2::TimePointZero);
    const auto &q = tf.transform.rotation;  // upside-down mounts (roll 180 deg) come back flipped
    return lidar_processing::pose_from_transform(tf.transform.translation.x, tf.transform.translation.y,
      q.x, q.y, q.z, q.w);
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LiDAR TF %s: %s", frame.c_str(), e.what());
    return std::nullopt;
  }
}

bool PerceptionMaster::lidar_obstacle(Task &task) {
  std::lock_guard<std::mutex> cycle_lock(obstacle_cycle_mutex_);
  sensor_msgs::msg::LaserScan::ConstSharedPtr input, rear;
  uint64_t id;
  { std::lock_guard<std::mutex> lock(scan_mutex_); input = scan_; rear = rear_scan_; id = scan_id_; }
  if (!input || task.seen == id || !is_fresh_input(input->header.stamp) ||
      rclcpp::Time(input->header.stamp).nanoseconds() <= obstacle_reset_stamp_ns_) return false;
  if (rear && rclcpp::Time(rear->header.stamp).nanoseconds() <= obstacle_reset_stamp_ns_) rear.reset();
  std::optional<sensor_msgs::msg::LaserScan> merged;
  if (lidar_input_mode_ == "raw") {
    if (!lidar_processing::ScanMerger::is_valid_input(*input)) { task.seen = id; return false; }
    const auto front_pose = lidar_pose(input->header.frame_id);
    if (!front_pose) return false;  // Retry this scan when static TF arrives.
    std::optional<lidar_processing::Pose2D> rear_pose;
    if (rear && is_fresh_input(rear->header.stamp)) {
      const double age = std::fabs((rclcpp::Time(input->header.stamp) - rclcpp::Time(rear->header.stamp)).seconds());
      if (age <= lidar_config_.max_age && lidar_processing::ScanMerger::is_valid_input(*rear)) {
        rear_pose = lidar_pose(rear->header.frame_id);
      }
    }
    if (!rear_pose) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No usable rear LiDAR scan/TF; processing front only");
    // Latest speeds, taken as constant over the two sweeps (0.1 s each, up to lidar.max_age apart).
    const lidar_processing::Motion motion{merge_v_.load(), imu_enabled_ ? merge_w_imu_.load() : merge_w_wheel_.load()};
    merged = lidar_.merge(*input, *front_pose, rear_pose ? rear.get() : nullptr,
                          rear_pose ? &*rear_pose : nullptr, lidar_motion_compensation_ ? &motion : nullptr);
    if (!merged) { task.seen = id; return false; }
    merged_scan_pub_->publish(*merged);  // Preserve SLAM/RViz input without a merger node.
  }
  task.seen = id;
  const auto &scan = merged ? *merged : *input;
  publish_localization_scan(scan);
  const int64_t stamp = rclcpp::Time(scan.header.stamp).nanoseconds(), brake = brake_at_ns_.load();
  const int64_t before = static_cast<int64_t>(brake_before_s_ * 1e9), after = static_cast<int64_t>(brake_after_s_ * 1e9);
  if (brake > 0 && brake != brake_handled_ns_) {
    // A new hard stop: back to before its scans (brake_before_s before it), filter state and published obstacles.
    brake_handled_ns_ = brake;
    for (auto it = obstacle_snapshots_.rbegin(); it != obstacle_snapshots_.rend(); ++it) {
      if (it->stamp > brake - before) continue;
      {std::lock_guard<std::mutex> lock(obstacle_filter_mutex_); obstacle_filter_.restore(it->filter);}
      last_lidar_obstacles_ = it->published;
      break;
    }
    RCLCPP_INFO(get_logger(), "hard stop: LiDAR obstacles from %.1f s before to %.1f s after it left out",
                brake_before_s_, brake_after_s_);
  }
  if (brake > 0 && stamp > brake - before && stamp <= brake + after) {
    if (last_lidar_obstacles_) {
      auto again = *last_lidar_obstacles_; again.header.stamp = scan.header.stamp;
      lidar_obstacle_pub_->publish(again);
    }
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(obstacle_filter_mutex_);
    obstacle_snapshots_.push_back({stamp, obstacle_filter_.state(), last_lidar_obstacles_});
  }
  while (!obstacle_snapshots_.empty() && obstacle_snapshots_.front().stamp < stamp - before - static_cast<int64_t>(1e9))
    obstacle_snapshots_.pop_front();
  auto obstacles = lidar_.detect_obstacles(scan);
  if (!obstacles) return false;
  if (obstacle_filter_enabled_) filter_obstacles(*obstacles, scan);
  lidar_obstacle_pub_->publish(*obstacles);
  last_lidar_obstacles_ = *obstacles;
  return true;
}

// /lidar/localization: the merged scan without the beams that end on an arena box (within localization_scan.box_radius
// of a centre on /planning/boxes: the planning.yaml places, the target box where the LiDAR measured it), for
// slam_toolbox. The boxes are erased from the saved pose graph but stood in every scan (12-17 % of its points), and
// the localization jumped 65 mm on stopping 0.29 m in front of one (2026-11-11). No boxes or no map pose yet: the
// scan as it is.
void PerceptionMaster::publish_localization_scan(const sensor_msgs::msg::LaserScan &scan) {
  if (!localization_scan_pub_) return;
  // No scan to slam_toolbox while the robot turns fast and turn_hold_s after: the merged scan is the latest front and
  // rear sweeps (0.1 s each, up to lidar.max_age apart) laid over each other as if the robot stood still, so at the
  // 0.6 rad/s of a turn on the spot it is bent 3.4 deg within a sweep and up to 6.9 deg between the two. Tour bags
  // 2026-11-11/12: a pose jump of slam_toolbox (> 2.5 deg or 6 cm at once) 8.4 times a minute within 1 s of turning
  // over 30 deg/s, 0.8 times a minute when not turning. The pose runs on wheel + IMU meanwhile (0.5 % yaw scale).
  if (localization_max_yaw_rate_ > 0.0 && rclcpp::Time(scan.header.stamp).nanoseconds() <=
      turning_at_ns_.load() + static_cast<int64_t>(localization_turn_hold_s_ * 1e9)) return;
  // Nor while the chassis rocks (bumpy floor): a tilted LiDAR plane hits the floor or the wall tops, and those ranges
  // went into the map (doubled walls) and pulled the pose. Wheel + IMU odometry carries the pose meanwhile.
  if (localization_max_tilt_rate_ > 0.0 && rclcpp::Time(scan.header.stamp).nanoseconds() <=
      tilting_at_ns_.load() + static_cast<int64_t>(localization_turn_hold_s_ * 1e9)) {
    const auto n = ++tilt_dropped_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "localization scan: %lu scans dropped so far while the "
                         "chassis rocked (roll / pitch rate over %.0f deg/s)", static_cast<unsigned long>(n),
                         localization_max_tilt_rate_ * 180.0 / M_PI);
    return;
  }
  if (localization_max_slope_ > 0.0 && rclcpp::Time(scan.header.stamp).nanoseconds() <=
      sloped_at_ns_.load() + static_cast<int64_t>(localization_turn_hold_s_ * 1e9)) {
    const auto n = ++slope_dropped_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "localization scan: %lu scans dropped so far on slopes "
                         "(over %.1f deg from level)", static_cast<unsigned long>(n), localization_max_slope_ * 180.0 / M_PI);
    return;
  }
  geometry_msgs::msg::PoseArray::ConstSharedPtr boxes;
  { std::lock_guard<std::mutex> lock(localization_boxes_mutex_); boxes = localization_boxes_; }
  if (!boxes || boxes->poses.empty()) {localization_scan_pub_->publish(scan); return;}
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, tf2::TimePointZero, scan.header.frame_id,
                                     tf2_ros::fromRclcpp(rclcpp::Time(scan.header.stamp)), obstacle_odom_frame_);
  } catch (const std::exception &) {
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, scan.header.frame_id, tf2::TimePointZero);
    } catch (const std::exception &) {localization_scan_pub_->publish(scan); return;}
  }
  const auto &q = tf.transform.rotation;
  const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  const double r2 = localization_box_radius_ * localization_box_radius_;
  auto out = scan;
  for (std::size_t i = 0; i < out.ranges.size(); ++i) {
    const float r = out.ranges[i];
    if (!std::isfinite(r)) continue;
    const double a = scan.angle_min + i * scan.angle_increment + yaw;
    const double x = tf.transform.translation.x + r * std::cos(a), y = tf.transform.translation.y + r * std::sin(a);
    for (const auto &b : boxes->poses) {
      const double dx = x - b.position.x, dy = y - b.position.y;
      if (dx * dx + dy * dy <= r2) {out.ranges[i] = std::numeric_limits<float>::quiet_NaN(); break;}
    }
  }
  localization_scan_pub_->publish(out);
}

// Keep only obstacle points worth avoiding (LidarObstacleFilter, judged in the map frame), plus the remembered ones
// out of view (lidar_obstacle.filter.memory_s; cleared by the scan's rays), published in the map frame at the scan
// stamp: the robot pose of that stamp = odom->scan frame at the stamp (EKF, never behind the scan) with the latest
// map->odom (slam_toolbox's correction, which changes slowly and lags the newest scan by its processing time).
// Planning then uses the points as they are, with no TF of its own at the scan stamp: it used to drop every message
// whose map->odom had not reached the stamp yet, and a 0.5 s run faulted a drive as stale input (2026-11-08).
// Without map TF the raw points go out in the scan frame so planning still sees everything.
void PerceptionMaster::filter_obstacles(geometry_msgs::msg::PoseArray &obstacles, const sensor_msgs::msg::LaserScan &scan) {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, tf2::TimePointZero, obstacles.header.frame_id,
                                     tf2_ros::fromRclcpp(rclcpp::Time(obstacles.header.stamp)), obstacle_odom_frame_);
  } catch (const std::exception &) {
    try {  // odom->scan frame not at the stamp yet (startup): the latest pose
      tf = tf_buffer_->lookupTransform(map_frame_, obstacles.header.frame_id, tf2::TimePointZero);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "obstacle filter off (no %s TF: %s), raw points",
                           map_frame_.c_str(), e.what());
      return;
    }
  }
  const auto &q = tf.transform.rotation;
  const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  const double c = std::cos(yaw), s = std::sin(yaw), tx = tf.transform.translation.x, ty = tf.transform.translation.y;
  std::vector<std::array<double, 2>> points;
  points.reserve(obstacles.poses.size());
  for (const auto &p : obstacles.poses) {
    points.push_back({tx + c * p.position.x - s * p.position.y, ty + s * p.position.x + c * p.position.y});
  }
  // Every return of the scan (not only those within lidar_obstacle.max_range_m) for the memory's free-space rays.
  // The scan is in obstacles' frame (base_link, merged): its rays start at that frame's origin.
  std::vector<std::array<double, 2>> ray_ends;
  ray_ends.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i], a = scan.angle_min + i * scan.angle_increment;
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double x = r * std::cos(a), y = r * std::sin(a);
    ray_ends.push_back({tx + c * x - s * y, ty + s * x + c * y});
  }
  std::vector<bool> keep;
  std::vector<std::array<double, 2>> remembered;
  {
    std::lock_guard<std::mutex> lock(obstacle_filter_mutex_);
    if (!obstacle_filter_.has_map()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "obstacle filter: no map yet, walls not removed");
    }
    keep = obstacle_filter_.filter(points);
    remembered = obstacle_filter_.remember(rclcpp::Time(obstacles.header.stamp).seconds(), {tx, ty}, ray_ends,
                                           points, keep);
  }
  std::vector<geometry_msgs::msg::Pose> kept;
  const auto add = [&kept](const std::array<double, 2> &m) {
      geometry_msgs::msg::Pose p; p.position.x = m[0]; p.position.y = m[1]; p.orientation.w = 1; kept.push_back(p);};
  for (size_t i = 0; i < keep.size(); ++i) {if (keep[i]) add(points[i]);}
  for (const auto &m : remembered) add(m);
  obstacles.header.frame_id = map_frame_;
  obstacles.poses.swap(kept);
}

bool PerceptionMaster::state_estimator(Task &task) {
  nav_msgs::msg::Odometry::ConstSharedPtr input;
  uint64_t id;
  { std::lock_guard<std::mutex> lock(odom_mutex_); input = odom_; id = odom_id_; }
  if (!input || task.seen == id || !is_fresh_input(input->header.stamp)) return false;
  task.seen = id;
  if (!state_estimator_.is_valid_input(*input)) return false;
  if (input->header.frame_id == map_frame_) {
    odom_pub_->publish(state_estimator_.compute_result(*input)); return true;
  }
  try {  // slam_toolbox owns map->odom; latest is fine because it changes slowly.
    auto tf = tf_buffer_->lookupTransform(map_frame_, input->header.frame_id, tf2::TimePointZero);
    if (map_correction_speed_ > 0) {
      // The robot's map pose moves toward the one slam_toolbox's map->odom gives at most map_correction.speed /
      // yaw_rate (planar); further off than snap (start, relocalization): taken at once.
      const auto yaw_of = [](const geometry_msgs::msg::Quaternion &q) {
        return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));};
      const double px = input->pose.pose.position.x, py = input->pose.pose.position.y, pyaw = yaw_of(input->pose.pose.orientation);
      const double stamp = rclcpp::Time(input->header.stamp).seconds();
      const double tyaw = yaw_of(tf.transform.rotation), tc = std::cos(tyaw), ts = std::sin(tyaw);
      const double gx = tf.transform.translation.x + tc * px - ts * py, gy = tf.transform.translation.y + ts * px + tc * py;
      const double gyaw = tyaw + pyaw;  // the pose slam_toolbox's correction gives
      double x = gx, y = gy, yaw = gyaw;
      if (map_correction_) {
        const auto &m = *map_correction_;
        const double mc = std::cos(m.yaw), ms = std::sin(m.yaw);
        const double sx = m.x + mc * px - ms * py, sy = m.y + ms * px + mc * py, syaw = m.yaw + pyaw;  // the pose in use
        const double ex = gx - sx, ey = gy - sy, e = std::hypot(ex, ey), eyaw = std::remainder(gyaw - syaw, 2 * M_PI);
        const double dt = std::clamp(stamp - m.stamp, 0.0, 0.5);
        // Standing still nothing steers by the pose: the correction is taken at once, so the pose in use and the
        // TF's (the box observation, the obstacles) agree when the box is measured at the stage point (2026-11-11:
        // a 65 mm correction on stopping there was still 26 mm along / 15 mm sideways behind when the dock was made).
        const auto &v = input->twist.twist;
        const bool still = std::abs(v.linear.x) < map_correction_still_v_ && std::abs(v.angular.z) < map_correction_still_w_;
        if (!still && e <= map_correction_snap_m_ && std::abs(eyaw) <= map_correction_snap_rad_) {
          const double step = std::min(e, map_correction_speed_ * dt), k = e > 0 ? step / e : 0.0;
          x = sx + ex * k; y = sy + ey * k;
          yaw = syaw + std::clamp(eyaw, -map_correction_yaw_rate_ * dt, map_correction_yaw_rate_ * dt);
        }
      }
      MapCorrection m; m.yaw = std::remainder(yaw - pyaw, 2 * M_PI); m.stamp = stamp;
      m.x = x - (std::cos(m.yaw) * px - std::sin(m.yaw) * py); m.y = y - (std::sin(m.yaw) * px + std::cos(m.yaw) * py);
      map_correction_ = m;
      tf.transform.translation.x = m.x; tf.transform.translation.y = m.y;
      tf.transform.rotation.x = 0; tf.transform.rotation.y = 0;
      tf.transform.rotation.z = std::sin(m.yaw / 2); tf.transform.rotation.w = std::cos(m.yaw / 2);
    }
    odom_pub_->publish(state_estimator_.compute_result(*input, tf.transform, map_frame_));
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odometry TF %s->%s: %s",
      map_frame_.c_str(), input->header.frame_id.c_str(), e.what());
    return false;
  }
}

void PerceptionMaster::receive_imu(std::size_t board, const sensor_msgs::msg::Imu &input) {
  static const char *const kNames[] = {"wheel_clock", "lm_arm"};
  constexpr double kGravity = 9.80665;  // control sends m/s^2; BoardImu works in g
  const double t = rclcpp::Time(input.header.stamp).seconds();
  // The wheels say whether the chassis can turn at all (calibration, zero-velocity update) and
  // break a tie when the two boards disagree. Stale wheel odometry: assume moving.
  const bool wheels_fresh = wheel_odom_ && is_fresh_input(wheel_odom_->header.stamp);
  const double wheel_yaw_rate = wheels_fresh ? wheel_odom_->twist.twist.angular.z : 0.0;
  const bool wheels_still = wheels_fresh &&
    std::fabs(wheel_odom_->twist.twist.linear.x) <= wheel_still_linear_ &&
    std::fabs(wheel_yaw_rate) <= wheel_still_angular_;
  const auto &w = input.angular_velocity;
  const auto &a = input.linear_acceleration;
  // The chassis rocking on the floor (wheel board only: the arm board moves with the arm): roll / pitch rate past
  // the gyro offset, remembered for publish_localization_scan().
  if (board == 0 && localization_max_tilt_rate_ > 0.0) {
    const auto &bias = imu_.board(board).bias();
    if (std::fabs(w.x - bias[0]) > localization_max_tilt_rate_ || std::fabs(w.y - bias[1]) > localization_max_tilt_rate_)
      tilting_at_ns_ = rclcpp::Time(input.header.stamp).nanoseconds();
  }
  // Standing tilted on a slope (mapping only): the gravity direction against level.
  if (board == 0 && localization_max_slope_ > 0.0) {
    const double n = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (n > 5.0) {
      const std::array<double, 3> g{a.x / n, a.y / n, a.z / n};
      if (wheels_still) { for (int k = 0; k < 3; ++k) slope_level_sum_[k] += g[k]; ++slope_level_n_; }
      if (wheels_fresh) {  // speeding up or braking tips the measured gravity like a slope: not judged then
        const double tv = rclcpp::Time(wheel_odom_->header.stamp).seconds(), v = wheel_odom_->twist.twist.linear.x;
        if (slope_v_t_ > 0.0 && tv > slope_v_t_ + 1e-3) slope_accel_ += 0.2 * ((v - slope_v_) / (tv - slope_v_t_) - slope_accel_);
        if (tv > slope_v_t_ + 1e-3) { slope_v_ = v; slope_v_t_ = tv; }
      }
      const bool first = slope_gravity_[0] == 0.0 && slope_gravity_[1] == 0.0 && slope_gravity_[2] == 0.0;
      for (int k = 0; k < 3; ++k) slope_gravity_[k] = first ? g[k] : slope_gravity_[k] + 0.04 * (g[k] - slope_gravity_[k]);
      if (slope_level_n_ >= 100 && std::fabs(slope_accel_) < 0.05) {
        double dot = 0, ng = 0, nl = 0;
        for (int k = 0; k < 3; ++k) {
          dot += slope_gravity_[k] * slope_level_sum_[k]; ng += slope_gravity_[k] * slope_gravity_[k];
          nl += slope_level_sum_[k] * slope_level_sum_[k];
        }
        if (std::acos(std::clamp(dot / std::sqrt(ng * nl), -1.0, 1.0)) > localization_max_slope_)
          sloped_at_ns_ = rclcpp::Time(input.header.stamp).nanoseconds();
      }
    }
  }
  if (imu_.update(board, t, {w.x, w.y, w.z}, {a.x / kGravity, a.y / kGravity, a.z / kGravity}, wheels_still)) {
    const auto &bias = imu_.board(board).bias();
    RCLCPP_INFO(get_logger(), "%s IMU calibrated: gyro offset (%.5f, %.5f, %.5f) rad/s", kNames[board],
      bias[0], bias[1], bias[2]);
  }
  // One output per primary sample; the backup drives it only while the primary is unavailable.
  if (board != imu_.primary() && imu_.available(imu_.primary(), t)) return;
  {
    std::lock_guard<std::mutex> lock(correction_mutex_);
    for (std::size_t i = 0; i < ImuProcessor::kBoards; ++i) {
      imu_.set_correction(i, correction_bias_[i], correction_scale_[i]);
    }
  }
  const auto rate = imu_.select(t, wheels_still);
  if (rate) imu_board_ = rate->board;
  if (!rate) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "OpenCR IMU calibrating, keep the robot still (wheels stopped): /perception/imu/data off");
    return;
  }
  if (imu_.dropped(0) + imu_.dropped(1) > 0) {  // check against the real turning profile
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "IMU samples dropped by the spike filter: "
      "wheel_clock %zu, lm_arm %zu", imu_.dropped(0), imu_.dropped(1));
  }
  if (rate->board != imu_.primary()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "primary IMU %s %s: using backup %s",
      kNames[imu_.primary()], imu_.frozen(imu_.primary()) ? "frozen (raw gyro repeating)" : "unavailable",
      kNames[rate->board]);
  }
  if (rate->disagree) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "OpenCR IMUs disagree: wheel_clock %.3f, lm_arm %.3f rad/s (wheels %.3f); still using %s - check the boards",
      imu_.accepted(0).value_or(NAN), imu_.accepted(1).value_or(NAN), wheel_yaw_rate, kNames[rate->board]);
  }
  sensor_msgs::msg::Imu output;
  output.header.stamp = input.header.stamp;
  output.header.frame_id = imu_frame_;  // already projected on the vertical: no IMU TF needed
  // Yaw rate only: orientation and acceleration are not provided (covariance[0] = -1).
  output.orientation_covariance[0] = -1.0;
  output.linear_acceleration_covariance[0] = -1.0;
  output.angular_velocity.z = rate->value;
  merge_w_imu_ = rate->value;  // for the scan merge
  output.angular_velocity_covariance[0] = output.angular_velocity_covariance[4] = 1e6;  // unmeasured x/y
  output.angular_velocity_covariance[8] = rate->variance;
  imu_pub_->publish(output);
  if (lidar_odometry_enabled_) {
    std::lock_guard<std::mutex> lock(imu_yaw_mutex_);
    if (!imu_yaw_.empty() && t > imu_yaw_.back().first) imu_yaw_sum_ += rate->value * (t - imu_yaw_.back().first);
    if (imu_yaw_.empty() || t > imu_yaw_.back().first) imu_yaw_.emplace_back(t, imu_yaw_sum_);
    while (imu_yaw_.size() > 2 && imu_yaw_.back().first - imu_yaw_.front().first > 2.0) imu_yaw_.pop_front();
  }
}

namespace {
double yaw_of(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

std::vector<std::pair<double, double>> PerceptionMaster::scan_points(const sensor_msgs::msg::LaserScan &scan) {
  std::vector<std::pair<double, double>> points;  // /lidar/merged is in base_link
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double a = scan.angle_min + i * scan.angle_increment;
    points.emplace_back(r * std::cos(a), r * std::sin(a));
  }
  return points;
}

void PerceptionMaster::lidar_odometry(const sensor_msgs::msg::LaserScan &scan) {
  const double t = rclcpp::Time(scan.header.stamp).seconds();
  if (lidar_scan_at_ && t <= *lidar_scan_at_) return;  // same scan again
  const double dt = lidar_scan_at_ ? t - *lidar_scan_at_ : 0.0;
  if (dt > 0.5) {  // gap: the last speed says nothing about this scan
    scan_odometry_.reset();
    lidar_velocity_ = {0.0, 0.0, 0.0};
  }
  // Prediction: the IMU turn between the two scans (yaw is not what slips), the last LiDAR speed for x/y.
  std::optional<double> imu_turn;
  if (lidar_scan_at_) {
    std::lock_guard<std::mutex> lock(imu_yaw_mutex_);
    auto yaw_at = [this](double at) -> std::optional<double> {
      if (imu_yaw_.size() < 2 || at < imu_yaw_.front().first || at > imu_yaw_.back().first + 0.05) return std::nullopt;
      auto b = std::lower_bound(imu_yaw_.begin(), imu_yaw_.end(), at,
                                [](const auto &s, double v) { return s.first < v; });
      if (b == imu_yaw_.end()) return imu_yaw_.back().second;
      if (b == imu_yaw_.begin()) return b->second;
      const auto a = std::prev(b);
      return a->second + (b->second - a->second) * (at - a->first) / (b->first - a->first);
    };
    const auto y0 = yaw_at(*lidar_scan_at_), y1 = yaw_at(t);
    if (y0 && y1) imu_turn = *y1 - *y0;
  }
  lidar_scan_at_ = t;
  const auto step = scan_odometry_.update(scan_points(scan),
    {lidar_velocity_.x * dt, lidar_velocity_.y * dt, imu_turn.value_or(lidar_velocity_.yaw * dt)});
  if (!step || dt <= 0.0) {
    if (dt > 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "LiDAR odometry: scan does not fit the keyframe, restarting from it (no /perception/lidar/odom)");
    }
    return;
  }
  lidar_velocity_ = {step->delta.x / dt, step->delta.y / dt, step->delta.yaw / dt};
  lidar_pose_ = compose(lidar_pose_, step->delta);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "LiDAR odometry score %.2f (min %.2f)", step->score,
    get_parameter("lidar_odometry.min_score").as_double());
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = scan.header.stamp;
  odom.header.frame_id = lidar_odom_frame_;
  odom.child_frame_id = lidar_base_frame_;
  odom.pose.pose.position.x = lidar_pose_.x;  // own dead reckoning, for plots; the EKF takes the twist
  odom.pose.pose.position.y = lidar_pose_.y;
  odom.pose.pose.orientation.z = std::sin(lidar_pose_.yaw / 2.0);
  odom.pose.pose.orientation.w = std::cos(lidar_pose_.yaw / 2.0);
  odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 1e3;
  odom.twist.twist.linear.x = lidar_velocity_.x;
  odom.twist.twist.linear.y = lidar_velocity_.y;
  odom.twist.twist.angular.z = lidar_velocity_.yaw;
  odom.twist.covariance[0] = lidar_twist_variance_[0];
  odom.twist.covariance[7] = lidar_twist_variance_[1];
  odom.twist.covariance[35] = lidar_twist_variance_[2];
  lidar_odom_pub_->publish(odom);
}

namespace {
Covariance3 planar(const std::array<double, 36> &c) {  // x, y, yaw rows/cols of a 6x6 pose covariance
  const int k[] = {0, 1, 5};
  Covariance3 out;
  for (int r = 0; r < 3; ++r) {
    for (int col = 0; col < 3; ++col) out[r * 3 + col] = c[k[r] * 6 + k[col]];
  }
  return out;
}
}  // namespace

void PerceptionMaster::receive_global_odom(const nav_msgs::msg::Odometry &input) {
  const double t = rclcpp::Time(input.header.stamp).seconds();
  const auto &p = input.pose.pose;
  global_poses_.push_back({t, {p.position.x, p.position.y, yaw_of(p.orientation)}, planar(input.pose.covariance)});
  while (global_poses_.size() > 2 && t - global_poses_.front().t > 2.0) global_poses_.pop_front();
  if (!local_odom_) return;
  // map->odom yaw = global yaw - local yaw: what SLAM has added on top of the IMU.
  const double local_yaw = yaw_of(local_odom_->pose.pose.orientation);
  localization_.yaw(t, global_poses_.back().pose.yaw - local_yaw, local_yaw, local_odom_->twist.twist.angular.z,
                    imu_board_);
  std::lock_guard<std::mutex> lock(correction_mutex_);
  for (std::size_t i = 0; i < 2; ++i) {
    if (correction_bias_[i] != localization_.bias(i) || correction_scale_[i] != localization_.scale(i)) {
      correction_bias_[i] = localization_.bias(i);
      correction_scale_[i] = localization_.scale(i);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "IMU correction from SLAM yaw: wheel_clock bias %+.5f rad/s scale %.4f, lm_arm bias %+.5f scale %.4f",
        localization_.bias(0), localization_.scale(0), localization_.bias(1), localization_.scale(1));
    }
  }
}

void PerceptionMaster::receive_slam_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &input) {
  if (global_poses_.empty()) return;
  const double t = rclcpp::Time(input.header.stamp).seconds();
  // ekf_global's pose at the SLAM stamp: interpolate its history (the SLAM pose arrives late).
  auto after = std::find_if(global_poses_.begin(), global_poses_.end(), [t](const StampedPose &p) { return p.t >= t; });
  StampedPose ekf = after == global_poses_.end() ? global_poses_.back() : *after;
  if (after != global_poses_.begin() && after != global_poses_.end()) {
    const auto &a = *std::prev(after);
    const double k = (t - a.t) / (after->t - a.t);
    ekf.pose = {a.pose.x + k * (after->pose.x - a.pose.x), a.pose.y + k * (after->pose.y - a.pose.y),
                a.pose.yaw + k * std::remainder(after->pose.yaw - a.pose.yaw, 2.0 * M_PI)};
  }
  const auto &p = input.pose.pose;
  localization_.slam(t, {p.position.x, p.position.y, yaw_of(p.orientation)}, planar(input.pose.covariance),
                     ekf.pose, ekf.cov);
  publish_localization_state();
}

void PerceptionMaster::relocalize() {
  const auto scan = relocalization_scan_;
  if (!scan || global_poses_.empty() || !relocalizer_.has_map()) {
    RCLCPP_WARN(get_logger(), "relocalization impossible (no scan, pose or map yet)");
    localization_.relocalized(false);
    publish_localization_state();
    return;
  }
  const auto points = scan_points(*scan);
  const auto guess = global_poses_.back();
  const auto started = Steady::now();
  const auto result = relocalizer_.match(points, guess.pose);
  const double ms = std::chrono::duration<double, std::milli>(Steady::now() - started).count();
  localization_.relocalized(result.has_value());
  if (!result) {
    RCLCPP_ERROR(get_logger(), "relocalization failed after %.2f m of dead reckoning (%.1f ms): localization lost",
      localization_.dead_reckoning(), ms);
    publish_localization_state();
    return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped output;
  output.header.stamp = rclcpp::Time(static_cast<int64_t>(guess.t * 1e9));
  output.header.frame_id = map_frame_;
  output.pose.pose.position.x = result->pose.x;
  output.pose.pose.position.y = result->pose.y;
  output.pose.pose.orientation.z = std::sin(result->pose.yaw / 2.0);
  output.pose.pose.orientation.w = std::cos(result->pose.yaw / 2.0);
  output.pose.covariance[0] = relocalized_variance_[0];
  output.pose.covariance[7] = relocalized_variance_[1];
  output.pose.covariance[35] = relocalized_variance_[2];
  for (const int k : {14, 21, 28}) output.pose.covariance[k] = 1e6;  // z, roll, pitch unused
  relocalized_pub_->publish(output);
  RCLCPP_WARN(get_logger(), "relocalized after dead reckoning: moved (%+.3f, %+.3f) m, %+.1f deg, score %.2f (%.1f ms)",
    result->pose.x - guess.pose.x, result->pose.y - guess.pose.y,
    std::remainder(result->pose.yaw - guess.pose.yaw, 2.0 * M_PI) * 180.0 / M_PI, result->score, ms);
  publish_localization_state();
}

void PerceptionMaster::publish_localization_state() {
  const std::string state = to_string(localization_.state());
  if (state != localization_published_) {
    RCLCPP_INFO(get_logger(), "localization: %s -> %s", localization_published_.empty() ? "start" :
      localization_published_.c_str(), state.c_str());
    localization_published_ = state;
  }
  std_msgs::msg::String output;
  output.data = state;
  localization_state_pub_->publish(output);
}

void PerceptionMaster::diagnostics() {
  const auto time = Steady::now();
  const double elapsed = std::chrono::duration<double>(time - diagnostics_at_).count();
  diagnostics_at_ = time;
  diagnostic_msgs::msg::DiagnosticArray output; output.header.stamp = now();
  for (const auto &task : tasks_) {
    std::lock_guard<std::mutex> lock(task->mutex);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "perception/" + task->name; status.hardware_id = "cpu";
    const uint64_t delta = task->processed - task->previous_processed;
    status.level = task->errors ? 2 : (task->overruns || delta == 0 ? 1 : 0);
    status.message = task->errors ? "processing errors; see log" :
      (delta == 0 ? "no new usable input/result" : "running");
    const auto value = [&status](const std::string &key, double number) {
      diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = std::to_string(number);
      status.values.push_back(item);
    };
    value("configured_hz", task->hz); value("processed_hz", delta / elapsed);
    value("last_callback_ms", task->runtime_ms); value("maximum_callback_ms", task->maximum_ms);
    value("processed", task->processed); value("skipped_no_new_input", task->skipped);
    value("errors", task->errors); value("overruns", task->overruns);
    task->previous_processed = task->processed; output.status.push_back(status);
  }
  for (const Camera camera : {kD435}) {
    std::lock_guard<std::mutex> lock(cameras_[camera].mutex);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "perception/shared_memory/realsense";
    status.hardware_id = cameras_[camera].shm; status.message = "on-demand shared snapshots";
    const auto value = [&status](const std::string &key, uint64_t number) {
      diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = std::to_string(number);
      status.values.push_back(item);
    };
    const auto &reader = *cameras_[camera].reader;
    value("input_frames", cameras_[camera].generation);
    value("snapshots", reader.snapshots()); value("bytes_copied", reader.bytes_copied());
    output.status.push_back(status);
  }
  diagnostics_pub_->publish(output);
}

}  // namespace craft_perception

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto master = std::make_shared<craft_perception::PerceptionMaster>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), master->executor_threads());
    executor.add_node(master); executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_master"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}

namespace craft_perception {
void PerceptionMaster::publish_hires(bool on) {
  std_msgs::msg::Bool m;
  m.data = on;
  hires_pub_->publish(m);
}
}  // namespace craft_perception
