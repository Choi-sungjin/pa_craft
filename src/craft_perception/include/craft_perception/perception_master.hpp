#pragma once
#include "craft_perception/c310_perception.hpp"
#include "craft_perception/d435_perception.hpp"
#include "craft_perception/lidar_perception.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/state_estimator.hpp"
#include "craft_perception/msg/detection_array.hpp"
#include "craft_perception/msg/knob_observation_array.hpp"
#include "craft_perception/msg/puck_observation_array.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <chrono>
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
  void diagnostics();

  D435Perception d435_;
  C310Perception c310_;
  LidarPerception lidar_;
  lidar_processing::MergerConfig lidar_config_;
  std::string lidar_input_mode_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  // LiDAR mounts and map->odom
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  StateEstimator state_estimator_;
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

  std::vector<std::shared_ptr<Task>> tasks_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr image_subs_[2];
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_sub_;
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
