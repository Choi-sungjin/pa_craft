#include "craft_perception/perception_master.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <cmath>

namespace craft_perception {

PerceptionMaster::PerceptionMaster() : Node("perception_master") {
  const int threads = declare_parameter("executor_threads", 6);
  if (threads < 2) throw std::invalid_argument("executor_threads must be >=2");
  load_schedules();
  max_age_ = load_positive_param("max_input_age_s", 0.5);
  camera_frames_[0] = declare_parameter("realsense_frame", "camera_color_optical_frame");
  camera_frames_[1] = declare_parameter("c310_frame", "c310_optical_frame");
  shm_names_[0] = declare_parameter("realsense_shm", "/craft_vision");
  shm_names_[1] = declare_parameter("c310_shm", "/craft_c310");
  for (int camera = 0; camera < 2; ++camera) {
    cameras_[camera].reader = std::make_unique<SharedFrameReader>(shm_names_[camera], camera == 0);
  }
  labels_ = declare_parameter<std::vector<std::string>>("class_names",
    {"knob", "sign_red", "sign_yellow", "sign_green", "sign_off"});
  knob_id_ = declare_parameter("knob_class_id", 0);
  if (labels_.empty() || knob_id_ < 0 || knob_id_ >= static_cast<int>(labels_.size())) {
    throw std::invalid_argument("class_names and knob_class_id do not match");
  }
  DetectorConfig config;
  config.model_dir = declare_parameter("model_dir", "models/knob_sign");
  if (!std::filesystem::path(config.model_dir).is_absolute()) {
    config.model_dir = ament_index_cpp::get_package_share_directory("craft_perception") + "/" + config.model_dir;
  }
  config.input_blob = declare_parameter("input_blob", "in0");
  config.output_blob = declare_parameter("output_blob", "out0");
  config.input_size = declare_parameter("input_size", 640);
  config.classes = labels_.size();
  config.threads = declare_parameter("ncnn_threads_per_detector", 1);
  config.confidence = declare_parameter("confidence", 0.5);
  config.iou = declare_parameter("nms_iou", 0.45);
  config.max_detections = declare_parameter("max_detections", 100);
  cv::setNumThreads(1); // Executor handles module parallelism; avoid nested CPU pools.
  if (!aruco_.configure(declare_parameter("aruco_dictionary", 0))) {
    throw std::invalid_argument("invalid ArUco dictionary configuration");
  }
  const double panel_grow = declare_parameter("panel_grow", 2.0);
  const double panel_tolerance = load_positive_param("panel_tolerance_m", 0.03);
  const int panel_min_points = declare_parameter("panel_min_points", 50);
  if (!depth_.configure(panel_grow, panel_tolerance, panel_min_points)) {
    throw std::invalid_argument("invalid knob depth configuration");
  }
  const double obstacle_range = load_positive_param("obstacle_max_range_m", 3.0);
  const int obstacle_stride = declare_parameter("obstacle_stride", 1);
  if (!obstacle_.configure(obstacle_range, obstacle_stride)) {
    throw std::invalid_argument("invalid obstacle detector configuration");
  }
  const double near_m = declare_parameter("heatmap_near_m", 0.2);
  const double far_m = load_positive_param("heatmap_far_m", 0.3);
  if (!heatmap_.configure(near_m, far_m)) {
    throw std::invalid_argument("invalid depth heatmap configuration");
  }

  aruco_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/vision/aruco", 10);
  detection_pubs_[0] = create_publisher<msg::DetectionArray>("/vision/detections", 10);
  detection_pubs_[1] = create_publisher<msg::DetectionArray>("/vision/c310_detections", 10);
  knob_pubs_[0] = create_publisher<msg::KnobObservationArray>("/vision/knob_observations", 10);
  knob_pubs_[1] = create_publisher<msg::KnobObservationArray>("/vision/c310_knob_observations", 10);
  heatmap_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/depth_heatmap", rclcpp::SensorDataQoS());
  object_points_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/vision/object_detection", 10);
  obstacle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/lidar/obstacle", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/state/odom", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/perception/diagnostics", 10);

  for (int camera = 0; camera < 2; ++camera) {
    auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    input_groups_.push_back(group);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    image_subs_[camera] = create_subscription<std_msgs::msg::UInt64MultiArray>(
      camera == 0 ? "/vision/image_offset" : "/vision/c310_image_offset", rclcpp::QoS(1),
      [this, camera](std_msgs::msg::UInt64MultiArray::ConstSharedPtr input) { receive_frame(camera, input->data); }, options);
  }
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("/vision/camera_info", 10,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
      std::lock_guard<std::mutex> lock(data_mutex_); info_ = std::move(info);
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    declare_parameter("odometry_source_topic", "/wheel/odom"), rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr input) {
      std::lock_guard<std::mutex> lock(data_mutex_); odom_ = std::move(input); ++odom_id_;
    });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/lidar/merged", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr input) {
      std::lock_guard<std::mutex> lock(data_mutex_); scan_ = std::move(input); ++scan_id_;
    });

  for (int camera = 0; camera < 2; ++camera) {
    const auto name = camera == 0 ? "object_detector" : "c310_detector";
    if (schedules_.at(name).enabled) {
      if (!detectors_[camera].configure(config)) {
        throw std::runtime_error("invalid NCNN configuration or cannot load model: " + config.model_dir);
      }
      add_task(name,
        [this, camera](Task &task) { return detect(camera, task); });
    }
  }
  add_task("aruco", [this](Task &task) {
    auto frame = latest_frame(0);
    if (!consume(frame, task)) return false;
    if (!aruco_.is_valid_input(frame->data->bgr)) return false;
    const auto ids = aruco_.compute_result(frame->data->bgr);
    if (!is_fresh_input(frame->header.stamp)) return false;
    std_msgs::msg::Int32MultiArray output;
    output.data.assign(ids.begin(), ids.end()); aruco_pub_->publish(output); return true;
  });
  add_task("knob_depth", [this](Task &task) { return geometry(0, task); });
  add_task("knob_lever", [this](Task &task) { return geometry(1, task); });
  add_task("depth_heatmap", [this](Task &task) {
    auto frame = latest_frame(0);
    if (!consume(frame, task)) return false;
    if (!heatmap_.is_valid_input(frame->data->depth)) return false;
    auto image = heatmap_.compute_result(frame->data->depth);
    sensor_msgs::msg::Image output;
    output.header = frame->header; output.height = image.rows; output.width = image.cols;
    output.encoding = "bgr8"; output.step = image.cols * 3;
    output.data.assign(image.data, image.data + image.total() * image.elemSize());
    heatmap_pub_->publish(output); return true;
  });
  add_task("state_estimator", [this](Task &task) {
    nav_msgs::msg::Odometry::ConstSharedPtr input;
    uint64_t id;
    { std::lock_guard<std::mutex> lock(data_mutex_); input = odom_; id = odom_id_; }
    if (!input || task.seen == id || !is_fresh_input(input->header.stamp)) return false;
    task.seen = id;
    if (!state_estimator_.is_valid_input(*input)) return false;
    odom_pub_->publish(state_estimator_.compute_result(*input)); return true;
  });
  add_task("obstacle_detector", [this](Task &task) {
    sensor_msgs::msg::LaserScan::ConstSharedPtr input;
    uint64_t id;
    { std::lock_guard<std::mutex> lock(data_mutex_); input = scan_; id = scan_id_; }
    if (!input || task.seen == id || !is_fresh_input(input->header.stamp)) return false;
    task.seen = id;
    if (!obstacle_.is_valid_input(*input)) return false;
    obstacle_pub_->publish(obstacle_.compute_result(*input)); return true;
  });
  diagnostics_at_ = Steady::now();
  diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { diagnostics(); });
  RCLCPP_WARN(get_logger(), "StateEstimator relays one odometry source; wheel/IMU/lidar fusion is not implemented");
  RCLCPP_INFO(get_logger(), "PerceptionMaster ready; model=%s, classes=%zu, executor_threads=%d",
    config.model_dir.c_str(), labels_.size(), threads);
}

double PerceptionMaster::load_positive_param(const std::string &name, double fallback) {
  const double value = declare_parameter(name, fallback);
  if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name + " must be finite and positive");
  return value;
}

bool PerceptionMaster::is_fresh_input(const builtin_interfaces::msg::Time &stamp) const {
  const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
  return age >= -0.01 && age <= max_age_;
}

void PerceptionMaster::load_schedules() {
  for (const std::string name : {"object_detector", "c310_detector", "aruco", "knob_depth",
       "knob_lever", "depth_heatmap", "state_estimator", "obstacle_detector"}) {
    const bool enabled = declare_parameter(name + ".enabled", true);
    // No duplicated C++ Hz defaults: perception.yaml is the schedule source.
    declare_parameter(name + ".hz", rclcpp::ParameterType::PARAMETER_DOUBLE);
    double hz;
    try { hz = get_parameter(name + ".hz").as_double(); }
    catch (const std::exception &) {
      throw std::invalid_argument("provide " + name + ".hz in perception.yaml (--params-file)");
    }
    const double seconds = 1.0 / hz;
    const double maximum = std::chrono::duration<double>(std::chrono::nanoseconds::max()).count();
    if (!std::isfinite(hz) || hz <= 0 || !std::isfinite(seconds) || seconds < 1e-9 || seconds >= maximum) {
      throw std::invalid_argument(name + ".hz must yield a finite, positive timer period >=1ns");
    }
    schedules_.emplace(name, ModuleSchedule{enabled, hz});
  }
}

void PerceptionMaster::add_task(const std::string &name, std::function<bool(Task &)> compute) {
  const auto schedule = schedules_.at(name);
  if (!schedule.enabled) return;
  auto task = std::make_shared<Task>(); task->name = name; task->hz = schedule.hz;
  task->group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::weak_ptr<Task> weak = task;
  task->timer = create_wall_timer(std::chrono::duration<double>(1 / schedule.hz), [this, weak, compute] {
    auto current = weak.lock(); if (!current) return;
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

void PerceptionMaster::receive_frame(int camera, const std::vector<uint64_t> &meta) {
  // Subscription only stores small metadata. No image read, allocation or conversion here.
  if (!SharedFrameReader::is_valid_metadata(meta, camera == 0)) return;
  const auto stamp = rclcpp::Time(static_cast<int64_t>(meta[4]), get_clock()->get_clock_type());
  if (!is_fresh_input(stamp)) return;
  auto &input = cameras_[camera];
  std::lock_guard<std::mutex> lock(input.mutex);
  if (meta == input.metadata) return;
  input.metadata = meta;
  ++input.generation;
}

std::shared_ptr<const Frame> PerceptionMaster::latest_frame(int camera) {
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
  auto frame = std::make_shared<Frame>(); frame->data = std::move(data);
  frame->header.frame_id = camera_frames_[camera]; frame->header.stamp = stamp;
  frame->id = input.generation;
  if (camera == 0) {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
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

msg::Detection PerceptionMaster::message(const Detection &box) const {
  msg::Detection output; output.class_id = box.class_id;
  output.label = labels_.at(box.class_id); output.confidence = box.score;
  output.bbox = {box.box.x, box.box.y, box.box.width, box.box.height}; return output;
}

bool PerceptionMaster::detect(int camera, Task &task) {
  auto frame = latest_frame(camera);
  if (!consume(frame, task)) return false;
  auto result = std::make_shared<DetectedFrame>(); result->frame = frame;
  if (!detectors_[camera].is_valid_input(frame->data->bgr)) return false;
  result->boxes = detectors_[camera].compute_result(frame->data->bgr);
  if (!is_fresh_input(frame->header.stamp)) return false;
  msg::DetectionArray output; output.header = frame->header;
  for (const auto &box : result->boxes) output.detections.push_back(message(box));
  detection_pubs_[camera]->publish(output);
  { std::lock_guard<std::mutex> lock(data_mutex_); detected_[camera] = std::move(result); }
  return true;
}

bool PerceptionMaster::geometry(int camera, Task &task) {
  std::shared_ptr<const DetectedFrame> result;
  { std::lock_guard<std::mutex> lock(data_mutex_); result = detected_[camera]; }
  if (!result || !consume(result->frame, task)) return false;
  const auto &frame = *result->frame;
  if (camera == 0 && !frame.info) return false;
  msg::KnobObservationArray output; output.header = frame.header;
  geometry_msgs::msg::PoseArray points; points.header = frame.header;
  for (const auto &box : result->boxes) {
    std::optional<DepthResult> depth;
    if (camera == 0) {
      const auto &k = frame.info->k;
      const Intrinsics intrinsics{k[0], k[4], k[2], k[5]};
      if (depth_.is_valid_input(frame.data->depth, box.box, intrinsics)) {
        depth = depth_.compute_result(frame.data->depth, box.box, intrinsics);
      }
      if (depth) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = depth->position[0]; pose.position.y = depth->position[1];
        pose.position.z = depth->position[2]; pose.orientation.w = 1; points.poses.push_back(pose);
      }
    }
    if (box.class_id != knob_id_) continue;
    msg::KnobObservation observation; observation.detection = message(box);
    if (depth) {
      observation.depth_valid = true;
      observation.position.x = depth->position[0]; observation.position.y = depth->position[1];
      observation.position.z = depth->position[2]; observation.normal_valid = depth->normal_valid;
      observation.panel_normal.x = depth->normal[0]; observation.panel_normal.y = depth->normal[1];
      observation.panel_normal.z = depth->normal[2]; observation.plane_rms_m = depth->plane_rms_m;
    }
    const auto angle = lever_.is_valid_input(frame.data->bgr, box.box) ?
      lever_.compute_result(frame.data->bgr, box.box) : std::nullopt;
    observation.lever_valid = angle.has_value(); observation.lever_angle = angle.value_or(0);
    output.observations.push_back(observation);
  }
  if (!is_fresh_input(frame.header.stamp)) return false;
  knob_pubs_[camera]->publish(output);
  if (camera == 0) object_points_pub_->publish(points);
  return true;
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
  for (int camera = 0; camera < 2; ++camera) {
    std::lock_guard<std::mutex> lock(cameras_[camera].mutex);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = camera == 0 ? "perception/shared_memory/realsense" : "perception/shared_memory/c310";
    status.hardware_id = shm_names_[camera]; status.message = "on-demand shared snapshots";
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
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
      master->get_parameter("executor_threads").as_int());
    executor.add_node(master); executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_master"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
