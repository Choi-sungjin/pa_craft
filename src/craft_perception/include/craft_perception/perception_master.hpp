#pragma once
#include "craft_perception/aruco_reader.hpp"
#include "craft_perception/object_detector.hpp"
#include "craft_perception/knob_depth.hpp"
#include "craft_perception/knob_lever.hpp"
#include "craft_perception/depth_heatmap.hpp"
#include "craft_perception/state_estimator.hpp"
#include "craft_perception/obstacle_detector.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/msg/detection_array.hpp"
#include "craft_perception/msg/knob_observation_array.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <memory>

namespace craft_perception {
using Steady = std::chrono::steady_clock;
struct Frame {
  std::shared_ptr<const SharedFrame> data;
  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  uint64_t id;
};
struct DetectedFrame {
  std::shared_ptr<const Frame> frame;
  std::vector<Detection> boxes;
};
struct CameraInput {
  std::mutex mutex;
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

// Only this class owns ROS I/O, parameters and scheduling. Modules own algorithms.
class PerceptionMaster : public rclcpp::Node {
public:
  PerceptionMaster();
private:
  double load_positive_param(const std::string &name, double fallback);
  bool is_fresh_input(const builtin_interfaces::msg::Time &stamp) const;
  void load_schedules();
  void add_task(const std::string &name, std::function<bool(Task &)> compute);
  void receive_frame(int camera, const std::vector<uint64_t> &meta);
  std::shared_ptr<const Frame> latest_frame(int camera);
  bool consume(const std::shared_ptr<const Frame> &frame, Task &task);
  msg::Detection message(const Detection &box) const;
  bool detect(int camera, Task &task);
  bool geometry(int camera, Task &task);
  void diagnostics();
  std::mutex data_mutex_;
  std::string camera_frames_[2];
  std::string shm_names_[2];
  std::vector<std::string> labels_;
  int knob_id_ = 0;
  double max_age_;
  uint64_t odom_id_ = 0, scan_id_ = 0;
  CameraInput cameras_[2];
  std::map<std::string, ModuleSchedule> schedules_;
  std::shared_ptr<const DetectedFrame> detected_[2];
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  ObjectDetector detectors_[2];
  ArucoReader aruco_;
  KnobDepth depth_;
  KnobLever lever_;
  DepthHeatmap heatmap_;
  StateEstimator state_estimator_;
  ObstacleDetector obstacle_;
  std::vector<std::shared_ptr<Task>> tasks_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> input_groups_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr image_subs_[2];
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<msg::DetectionArray>::SharedPtr detection_pubs_[2];
  rclcpp::Publisher<msg::KnobObservationArray>::SharedPtr knob_pubs_[2];
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr aruco_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr heatmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr object_points_pub_, obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  Steady::time_point diagnostics_at_;
};
}
