#pragma once
#include "craft_perception/c310_perception.hpp"
#include "craft_perception/d435_perception.hpp"
#include "craft_perception/imu_processor.hpp"
#include "craft_perception/lidar_perception.hpp"
#include "craft_perception/localization_monitor.hpp"
#include "craft_perception/scan_relocalizer.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/state_estimator.hpp"
#include "craft_perception/msg/detection_array.hpp"
#include "craft_perception/msg/knob_observation_array.hpp"
#include "craft_perception/msg/puck_observation_array.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <memory>
#include <optional>

namespace craft_perception {
using Steady = std::chrono::steady_clock;
enum Camera { kD435 = 0, kC310 = 1 };
// Frames and detections move between tasks as shared_ptr<const ...>: one shm snapshot
// per frame, never copied again inside the master.
struct Frame {
  std::shared_ptr<const SharedFrame> data;
  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  uint64_t id;
};
using FramePtr = std::shared_ptr<const Frame>;
struct DetectedFrame {
  std::shared_ptr<const Frame> frame;
  std::vector<Detection> boxes;
};
struct CameraInput {
  std::mutex mutex;
  std::string frame_id, shm;
  std::unique_ptr<SharedFrameReader> reader;
  std::vector<uint64_t> metadata;
  uint64_t generation = 0;
  std::shared_ptr<const Frame> cached;
};
struct ModuleSchedule { bool enabled; double hz; };
struct Task {
  std::string name;
  double hz;
  uint64_t seen = 0;
  std::mutex mutex;
  uint64_t processed = 0, skipped = 0, errors = 0, overruns = 0, previous_processed = 0;
  double runtime_ms = 0, maximum_ms = 0;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::TimerBase::SharedPtr timer;
};

// Owns ROS I/O, parameters and scheduling, like control_master. D435Perception,
// C310Perception and LidarPerception own algorithms called through member functions.
class PerceptionMaster : public rclcpp::Node {
public:
  PerceptionMaster();
  int executor_threads() const { return executor_threads_; }
private:
  // Reads perception.yaml (--params-file) and configures the sensor modules.
  void configure();
  // Shared-memory readers, publishers, subscriptions and one timer per enabled task.
  void connect();
  template <typename T> T param(const std::string &name);
  double positive_param(const std::string &name);
  DetectorConfig yolo_param(const std::string &prefix, std::vector<std::string> &labels);
  bool enabled(const std::string &task) const { return schedules_.at(task).enabled; }
  bool is_fresh_input(const builtin_interfaces::msg::Time &stamp) const;
  void add_task(const std::string &name, std::function<bool(Task &)> compute);
  void receive_frame(Camera camera, const std::vector<uint64_t> &meta);
  std::shared_ptr<const Frame> latest_frame(Camera camera);
  bool consume(const std::shared_ptr<const Frame> &frame, Task &task);
  std::shared_ptr<const DetectedFrame> new_detection(Task &task);
  static std::optional<Intrinsics> intrinsics(const Frame &frame);
  template <typename Compute, typename Publish>
  bool on_frame(Camera camera, Task &task, Compute compute, Publish publish);
  template <typename Compute, typename Publish>
  bool on_detection(Task &task, Compute compute, Publish publish);
  bool d435_yolo(Task &task);
  bool d435_apriltag(Task &task);
  bool d435_knob_depth(Task &task);
  bool d435_object_detector(Task &task);
  bool d435_obstacle_detector(Task &task);
  bool d435_puck_axis(Task &task);
  bool c310_knob_lever(Task &task);
  bool lidar_obstacle(Task &task);
  std::optional<lidar_processing::Pose2D> lidar_pose(const std::string &frame);
  bool state_estimator(Task &task);
  // One raw OpenCR IMU sample (0 = wheel_clock, 1 = lm_arm) -> /perception/imu/data.
  void receive_imu(std::size_t board, const sensor_msgs::msg::Imu &input);
  // Localization monitor (localization:=ekf): SLAM pose check, dead reckoning, wide relocalization,
  // IMU corrections. Runs in its own callback group.
  void receive_slam_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &input);
  void receive_global_odom(const nav_msgs::msg::Odometry &input);
  void relocalize();
  static std::vector<std::pair<double, double>> scan_points(const sensor_msgs::msg::LaserScan &scan);
  void publish_localization_state();
  void diagnostics();

  D435Perception d435_;
  C310Perception c310_;
  LidarPerception lidar_;
  lidar_processing::MergerConfig lidar_config_;
  std::string lidar_input_mode_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  // LiDAR mounts and map->odom
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  StateEstimator state_estimator_;
  ImuProcessor imu_;
  bool imu_enabled_ = false;
  std::string imu_frame_;
  double wheel_still_linear_ = 0, wheel_still_angular_ = 0;
  bool localization_enabled_ = false;
  LocalizationMonitor localization_;
  ScanRelocalizer relocalizer_;
  std::array<double, 3> relocalized_variance_{};  // x, y [m^2], yaw [rad^2] of a relocalized pose
  // Only touched from the localization callback group.
  struct StampedPose { double t; Pose2 pose; Covariance3 cov; };
  std::deque<StampedPose> global_poses_;  // ekf_global, last ~2 s, for the pose at a SLAM stamp
  nav_msgs::msg::Odometry::ConstSharedPtr local_odom_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr relocalization_scan_;
  std::optional<double> local_odom_at_;
  std::string localization_published_;
  // Handed from the localization group to the IMU group.
  std::mutex correction_mutex_;
  std::array<double, 2> correction_bias_{0.0, 0.0}, correction_scale_{1.0, 1.0};
  std::atomic<std::size_t> imu_board_{0};
  std::map<std::string, ModuleSchedule> schedules_;
  std::vector<std::string> d435_labels_, c310_labels_;
  int executor_threads_ = 0;
  double max_age_ = 0;
  std::string odometry_topic_;
  std::string map_frame_;

  // One mutex per input, held only to swap pointers; computation runs unlocked.
  // Lock order: cameras_[i].mutex before info_mutex_ (latest_frame); no other nesting.
  CameraInput cameras_[2];
  std::mutex detected_mutex_;
  std::shared_ptr<const DetectedFrame> d435_detected_;
  std::mutex info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  uint64_t odom_id_ = 0;
  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr rear_scan_;
  uint64_t scan_id_ = 0;
  // Both IMU subscriptions and the wheel odometry share one mutually exclusive callback group,
  // so the processor and wheel_odom_ are only touched from there: no lock needed.
  nav_msgs::msg::Odometry::ConstSharedPtr wheel_odom_;

  std::vector<std::shared_ptr<Task>> tasks_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr image_subs_[2];
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subs_[ImuProcessor::kBoards];
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_odom_sub_, local_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr relocalization_scan_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr localization_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr relocalized_pub_;
  rclcpp::TimerBase::SharedPtr localization_timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub_;
  rclcpp::Publisher<msg::DetectionArray>::SharedPtr detection_pubs_[2];
  rclcpp::Publisher<msg::KnobObservationArray>::SharedPtr knob_pubs_[2];
  rclcpp::Publisher<msg::PuckObservationArray>::SharedPtr puck_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr apriltag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr object_pub_, obstacle_pub_, lidar_obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  Steady::time_point diagnostics_at_;
};
}
