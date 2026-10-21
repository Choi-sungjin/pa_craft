#include "craft_perception/perception_master.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <cmath>

namespace craft_perception {
namespace {
const char *const kTasks[] = {"d435_yolo", "apriltag", "knob_depth", "object_detector",
  "obstacle_detector", "puck_axis", "knob_lever", "lidar_obstacle", "state_estimator"};

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

std::unique_ptr<msg::KnobObservationArray> knob_depth_message(const std_msgs::msg::Header &header,
  const std::vector<KnobDepthObservation> &knobs, const std::vector<std::string> &labels) {
  auto output = std::make_unique<msg::KnobObservationArray>();
  output->header = header;
  for (const auto &knob : knobs) {
    msg::KnobObservation observation;
    observation.detection = message(knob.detection, labels);
    if (knob.depth) {
      observation.depth_valid = true;
      observation.position = point(knob.depth->position);
      observation.normal_valid = knob.depth->normal_valid;
      observation.panel_normal = vector(knob.depth->normal);
      observation.plane_rms_m = knob.depth->plane_rms_m;
    }
    output->observations.push_back(observation);
  }
  return output;
}

std::unique_ptr<msg::KnobObservationArray> knob_lever_message(const std_msgs::msg::Header &header,
  const std::vector<LeverObservation> &levers, const std::vector<std::string> &labels) {
  auto output = std::make_unique<msg::KnobObservationArray>();
  output->header = header;
  for (const auto &lever : levers) {
    msg::KnobObservation observation;
    observation.detection = message(lever.detection, labels);
    observation.lever_valid = lever.angle.has_value();
    observation.lever_angle = lever.angle.value_or(0);
    output->observations.push_back(observation);
  }
  return output;
}

std::unique_ptr<msg::PuckObservationArray> puck_message(const std_msgs::msg::Header &header,
  const std::vector<PuckObservation> &pucks, const std::vector<std::string> &labels) {
  auto output = std::make_unique<msg::PuckObservationArray>();
  output->header = header;
  for (const auto &puck : pucks) {
    msg::PuckObservation observation;
    observation.detection = message(puck.detection, labels);
    if (puck.axis) {
      observation.axis_valid = true;
      observation.center = point(puck.axis->center);
      observation.axis = vector(puck.axis->axis);
      observation.shelf_normal = vector(puck.axis->normal);
      observation.standing = puck.axis->standing;
      observation.height_m = puck.axis->height_m;
      observation.plane_rms_m = puck.axis->plane_rms_m;
      observation.points = puck.axis->points;
    }
    output->observations.push_back(observation);
  }
  return output;
}
}  // namespace

PerceptionMaster::PerceptionMaster() : Node("perception_master") {
  configure();
  connect();
  RCLCPP_WARN(get_logger(), "StateEstimator relays one odometry source; wheel/IMU/lidar fusion is not implemented");
  RCLCPP_INFO(get_logger(), "PerceptionMaster ready; D435 classes=%zu, C310 classes=%zu, puck classes=%zu, "
    "executor_threads=%d", d435_labels_.size(), c310_labels_.size(), puck_labels_.size(), executor_threads_);
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
  config.model_dir = share_path(param<std::string>(prefix + ".model_dir"));
  config.engine_path = share_path(param<std::string>(prefix + ".engine"));
  // Blob names differ per export: ncnn in0/out0, Ultralytics ONNX -> TensorRT images/output0.
  const std::string blobs = config.backend == "tensorrt" ? ".tensorrt_blobs" : ".ncnn_blobs";
  const auto names = param<std::vector<std::string>>(prefix + blobs);
  if (names.size() != 2) throw std::invalid_argument(prefix + blobs + " must be [input, output]");
  config.input_blob = names[0];
  config.output_blob = names[1];
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
  for (const char *name : {"knob_depth", "object_detector", "obstacle_detector"}) {
    if (enabled(name) && !enabled("d435_yolo")) {
      RCLCPP_WARN(get_logger(), "%s is enabled but d435_yolo is disabled; it will never get boxes", name);
    }
  }
  cameras_[kD435].frame_id = param<std::string>("d435.frame");
  cameras_[kD435].shm = param<std::string>("d435.shm");
  cameras_[kC310].frame_id = param<std::string>("c310.frame");
  cameras_[kC310].shm = param<std::string>("c310.shm");

  D435Config d435;
  d435.use_yolo = enabled("d435_yolo");
  d435.use_apriltag = enabled("apriltag");
  d435.use_puck = enabled("puck_axis");
  d435.yolo = yolo_param("d435.yolo", d435_labels_);
  d435.knob_class_id = param<int64_t>("d435.yolo.knob_class_id");
  d435.apriltag_family = param<std::string>("d435.apriltag_family");
  d435.knob_grow = param<double>("d435.knob_depth.grow");
  d435.knob_tolerance_m = param<double>("d435.knob_depth.tolerance_m");
  d435.knob_min_points = param<int64_t>("d435.knob_depth.min_points");
  d435.object_inner_fraction = param<double>("d435.object_detector.inner_fraction");
  for (const auto id : param<std::vector<int64_t>>("d435.obstacle_detector.class_ids")) {
    d435.obstacle_class_ids.push_back(id);
  }
  d435.obstacle_max_range_m = param<double>("d435.obstacle_detector.max_range_m");
  d435.obstacle_near_quantile = param<double>("d435.obstacle_detector.near_quantile");
  d435.puck_yolo = yolo_param("d435.puck_yolo", puck_labels_);
  d435.puck.grow = param<double>("d435.puck_axis.grow");
  d435.puck.plane_threshold_m = param<double>("d435.puck_axis.plane_threshold_m");
  d435.puck.min_height_m = param<double>("d435.puck_axis.min_height_m");
  d435.puck.max_height_m = param<double>("d435.puck_axis.max_height_m");
  d435.puck.standing_height_m = param<double>("d435.puck_axis.standing_height_m");
  d435.puck.min_depth_m = param<double>("d435.puck_axis.min_depth_m");
  d435.puck.max_depth_m = param<double>("d435.puck_axis.max_depth_m");
  d435.puck.min_points = param<int64_t>("d435.puck_axis.min_points");
  d435.puck.ransac_iterations = param<int64_t>("d435.puck_axis.ransac_iterations");

  C310Config c310;
  c310.use_knob_lever = enabled("knob_lever");
  c310.yolo = yolo_param("c310.yolo", c310_labels_);
  c310.knob_class_id = param<int64_t>("c310.yolo.knob_class_id");

  cv::setNumThreads(1); // Executor handles module parallelism; avoid nested CPU pools.
  if (!d435_.configure(d435)) {
    throw std::invalid_argument("invalid d435 configuration in perception.yaml, or cannot load the "
      + d435.yolo.backend + " model for d435.yolo / d435.puck_yolo (see stderr)");
  }
  if (!c310_.configure(c310)) {
    throw std::invalid_argument("invalid c310 configuration in perception.yaml, or cannot load the "
      + c310.yolo.backend + " model for c310.yolo (see stderr)");
  }
  if (!lidar_obstacle_.configure(positive_param("lidar_obstacle.max_range_m"),
                                 param<int64_t>("lidar_obstacle.stride"))) {
    throw std::invalid_argument("invalid lidar_obstacle configuration");
  }
  odometry_topic_ = param<std::string>("odometry_source_topic");
}

void PerceptionMaster::connect() {
  for (int camera = 0; camera < 2; ++camera) {
    cameras_[camera].reader = std::make_unique<SharedFrameReader>(cameras_[camera].shm, camera == kD435);
  }
  apriltag_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/perception/vision/d435/apriltag", 10);
  detection_pubs_[kD435] = create_publisher<msg::DetectionArray>("/perception/vision/d435/detections", 10);
  detection_pubs_[kC310] = create_publisher<msg::DetectionArray>("/perception/vision/c310/detections", 10);
  knob_pubs_[kD435] = create_publisher<msg::KnobObservationArray>("/perception/vision/d435/knob_observations", 10);
  knob_pubs_[kC310] = create_publisher<msg::KnobObservationArray>("/perception/vision/c310/knob_observations", 10);
  puck_pub_ = create_publisher<msg::PuckObservationArray>("/perception/vision/d435/puck_observations", 10);
  object_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/perception/vision/d435/object_detection", 10);
  obstacle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/perception/vision/d435/obstacle", 10);
  lidar_obstacle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/perception/lidar/obstacle", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/perception/state/odom", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/perception/diagnostics", 10);

  for (const Camera camera : {kD435, kC310}) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    image_subs_[camera] = create_subscription<std_msgs::msg::UInt64MultiArray>(
      camera == kD435 ? "/vision/d435/frame_meta" : "/vision/c310/frame_meta", rclcpp::QoS(1),
      [this, camera](std_msgs::msg::UInt64MultiArray::ConstSharedPtr input) { receive_frame(camera, input->data); },
      options);
  }
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("/vision/d435/camera_info", 10,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
      std::lock_guard<std::mutex> lock(info_mutex_); info_ = std::move(info);
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr input) {
      std::lock_guard<std::mutex> lock(odom_mutex_); odom_ = std::move(input); ++odom_id_;
    });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/lidar/merged", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr input) {
      std::lock_guard<std::mutex> lock(scan_mutex_); scan_ = std::move(input); ++scan_id_;
    });

  add_task("d435_yolo", [this](Task &task) { return d435_yolo(task); });
  add_task("apriltag", [this](Task &task) { return d435_apriltag(task); });
  add_task("knob_depth", [this](Task &task) { return d435_knob_depth(task); });
  add_task("object_detector", [this](Task &task) { return d435_object_detector(task); });
  add_task("obstacle_detector", [this](Task &task) { return d435_obstacle_detector(task); });
  add_task("puck_axis", [this](Task &task) { return d435_puck_axis(task); });
  add_task("knob_lever", [this](Task &task) { return c310_knob_lever(task); });
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
      detection_pubs_[kD435]->publish(detections(frame->header, boxes, d435_labels_));
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
    [this](const FramePtr &, std::vector<int> &ids) {
      auto output = std::make_unique<std_msgs::msg::Int32MultiArray>();
      output->data.assign(ids.begin(), ids.end());
      apriltag_pub_->publish(std::move(output));
    });
}

bool PerceptionMaster::d435_knob_depth(Task &task) {
  return on_detection(task,
    [this](const SharedFrame &data, const std::vector<Detection> &boxes, const Intrinsics &in) {
      return d435_.knob_depth(data, boxes, in);
    },
    [this](const FramePtr &frame, std::vector<KnobDepthObservation> &knobs) {
      knob_pubs_[kD435]->publish(knob_depth_message(frame->header, knobs, d435_labels_));
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

// Puck runs its own model on the D435 frame, independent of d435_yolo.
bool PerceptionMaster::d435_puck_axis(Task &task) {
  return on_frame(kD435, task,
    [this](const FramePtr &frame) {
      const auto in = intrinsics(*frame);
      return in ? d435_.puck_axis(*frame->data, *in) : std::nullopt;
    },
    [this](const FramePtr &frame, std::vector<PuckObservation> &pucks) {
      puck_pub_->publish(puck_message(frame->header, pucks, puck_labels_));
    });
}

// C310: its own YOLO + lever per frame.
bool PerceptionMaster::c310_knob_lever(Task &task) {
  return on_frame(kC310, task,
    [this](const FramePtr &frame) { return c310_.knob_lever(*frame->data); },
    [this](const FramePtr &frame, KnobLeverResult &result) {
      detection_pubs_[kC310]->publish(detections(frame->header, result.detections, c310_labels_));
      knob_pubs_[kC310]->publish(knob_lever_message(frame->header, result.levers, c310_labels_));
    });
}

bool PerceptionMaster::lidar_obstacle(Task &task) {
  sensor_msgs::msg::LaserScan::ConstSharedPtr input;
  uint64_t id;
  { std::lock_guard<std::mutex> lock(scan_mutex_); input = scan_; id = scan_id_; }
  if (!input || task.seen == id || !is_fresh_input(input->header.stamp)) return false;
  task.seen = id;
  if (!lidar_obstacle_.is_valid_input(*input)) return false;
  lidar_obstacle_pub_->publish(lidar_obstacle_.compute_result(*input)); return true;
}

bool PerceptionMaster::state_estimator(Task &task) {
  nav_msgs::msg::Odometry::ConstSharedPtr input;
  uint64_t id;
  { std::lock_guard<std::mutex> lock(odom_mutex_); input = odom_; id = odom_id_; }
  if (!input || task.seen == id || !is_fresh_input(input->header.stamp)) return false;
  task.seen = id;
  if (!state_estimator_.is_valid_input(*input)) return false;
  odom_pub_->publish(state_estimator_.compute_result(*input)); return true;
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
  for (const Camera camera : {kD435, kC310}) {
    std::lock_guard<std::mutex> lock(cameras_[camera].mutex);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = camera == kD435 ? "perception/shared_memory/realsense" : "perception/shared_memory/c310";
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
