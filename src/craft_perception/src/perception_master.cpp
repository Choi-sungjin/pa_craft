#include "craft_perception/perception_master.hpp"
#include "craft_perception/tag_registry.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <filesystem>
#include <cmath>
#include <limits>

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
  RCLCPP_INFO(get_logger(), "PerceptionMaster ready; D435 classes=%zu, C310 classes=%zu, "
    "executor_threads=%d", d435_labels_.size(), c310_labels_.size(), executor_threads_);
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
  const auto registry_file = param<std::string>("tag_registry_file");
  d435.tag_registry = load_tag_registry(std::filesystem::path(registry_file).is_absolute() ?
    registry_file : ament_index_cpp::get_package_share_directory("craft_perception") + "/" + registry_file);
  d435.puck_tag_box_scale = positive_param("d435.puck_axis.tag_box_scale");
  d435.knob_grow = param<double>("d435.knob_depth.grow");
  d435.knob_tolerance_m = param<double>("d435.knob_depth.tolerance_m");
  d435.knob_min_points = param<int64_t>("d435.knob_depth.min_points");
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

  C310Config c310;
  c310.use_knob_lever = enabled("knob_lever");
  c310.yolo = yolo_param("c310.yolo", c310_labels_);
  c310.knob_class_id = param<int64_t>("c310.yolo.knob_class_id");
  c310.lever_source = param<std::string>("c310.knob_lever.lever_source");
  c310.lever_class_id = param<int64_t>("c310.knob_lever.lever_class_id");
  if (c310.lever_source == "yolo_seg" && c310.yolo.proto_blob.empty()) {
    throw std::invalid_argument("c310.knob_lever.lever_source yolo_seg needs a seg model: give "
      "c310.yolo blobs as [input, output, protos]");
  }

  cv::setNumThreads(1); // Executor handles module parallelism; avoid nested CPU pools.
  if (!d435_.configure(d435)) {
    throw std::invalid_argument("invalid d435 configuration in perception.yaml, or cannot load the "
      + d435.yolo.backend + " model for d435.yolo (see stderr)");
  }
  if (!c310_.configure(c310)) {
    throw std::invalid_argument("invalid c310 configuration in perception.yaml, or cannot load the "
      + c310.yolo.backend + " model for c310.yolo (see stderr)");
  }
  lidar_input_mode_ = param<std::string>("lidar.input_mode");
  if (lidar_input_mode_ != "raw" && lidar_input_mode_ != "merged") {
    throw std::invalid_argument("lidar.input_mode must be raw or merged");
  }
  lidar_config_.target_frame = param<std::string>("lidar.target_frame");
  lidar_config_.bins = param<int64_t>("lidar.bins");
  lidar_config_.range_max = param<double>("lidar.range_max");
  lidar_config_.max_age = param<double>("lidar.max_age");
  lidar_config_.front_fov_deg = param<double>("lidar.front_fov_deg");
  lidar_config_.rear_fov_deg = param<double>("lidar.rear_fov_deg");
  lidar_config_.fov_near_m = param<double>("lidar.fov_near_m");
  lidar_config_.body_radius_m = param<double>("lidar.body_radius_m");
  if (!lidar_.configure(lidar_config_, positive_param("lidar_obstacle.max_range_m"),
                        param<int64_t>("lidar_obstacle.stride"))) {
    throw std::invalid_argument("invalid lidar / lidar_obstacle configuration");
  }
  odometry_topic_ = param<std::string>("odometry_source_topic");
  map_frame_ = param<std::string>("map_frame");

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
  const auto variance = param<std::vector<double>>("localization.relocalize.pose_variance");
  if (variance.size() != 3) throw std::invalid_argument("localization.relocalize.pose_variance needs x, y, yaw");
  std::copy(variance.begin(), variance.end(), relocalized_variance_.begin());
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
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr input) {
      std::lock_guard<std::mutex> lock(odom_mutex_); odom_ = std::move(input); ++odom_id_;
    });
  // Raw mode calls LidarPerception directly, just like the camera modules.
  // Merged mode supports a merger already running on a separate sensor computer.
  if (lidar_input_mode_ == "raw" && enabled("lidar_obstacle")) {
    merged_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/lidar/merged", 10);
    rear_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/lidar/rear/scan",
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr input) {
        std::lock_guard<std::mutex> lock(scan_mutex_); rear_scan_ = std::move(input);
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
    [this](const FramePtr &, std::vector<AprilTagDetection> &tags) {
      auto output = std::make_unique<std_msgs::msg::Int32MultiArray>();
      for (const auto &tag : tags) output->data.push_back(tag.id);
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

// Puck identity and ROI come from configured AprilTag IDs; color is not used.
bool PerceptionMaster::d435_puck_axis(Task &task) {
  return on_frame(kD435, task,
    [this](const FramePtr &frame) {
      const auto in = intrinsics(*frame);
      return in ? d435_.puck_axis(*frame->data, *in) : std::nullopt;
    },
    [this](const FramePtr &frame, std::vector<PuckObservation> &pucks) {
      puck_pub_->publish(puck_message(frame->header, pucks));
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

std::optional<lidar_processing::Pose2D> PerceptionMaster::lidar_pose(const std::string &frame) {
  try {
    const auto tf = tf_buffer_->lookupTransform(lidar_config_.target_frame, frame, tf2::TimePointZero);
    const auto &q = tf.transform.rotation;
    return lidar_processing::Pose2D{tf.transform.translation.x, tf.transform.translation.y,
      std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LiDAR TF %s: %s", frame.c_str(), e.what());
    return std::nullopt;
  }
}

bool PerceptionMaster::lidar_obstacle(Task &task) {
  sensor_msgs::msg::LaserScan::ConstSharedPtr input, rear;
  uint64_t id;
  { std::lock_guard<std::mutex> lock(scan_mutex_); input = scan_; rear = rear_scan_; id = scan_id_; }
  if (!input || task.seen == id || !is_fresh_input(input->header.stamp)) return false;
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
    merged = lidar_.merge(*input, *front_pose, rear_pose ? rear.get() : nullptr,
                          rear_pose ? &*rear_pose : nullptr);
    if (!merged) { task.seen = id; return false; }
    merged_scan_pub_->publish(*merged);  // Preserve SLAM/RViz input without a merger node.
  }
  task.seen = id;
  const auto obstacles = lidar_.detect_obstacles(merged ? *merged : *input);
  if (!obstacles) return false;
  lidar_obstacle_pub_->publish(*obstacles);
  return true;
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
    const auto tf = tf_buffer_->lookupTransform(map_frame_, input->header.frame_id, tf2::TimePointZero);
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
  output.angular_velocity_covariance[0] = output.angular_velocity_covariance[4] = 1e6;  // unmeasured x/y
  output.angular_velocity_covariance[8] = rate->variance;
  imu_pub_->publish(output);
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
