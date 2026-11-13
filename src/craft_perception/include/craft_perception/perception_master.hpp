#pragma once
#include "craft_perception/d435_perception.hpp"
#include "craft_perception/imu_processor.hpp"
#include "craft_perception/lidar_obstacle_filter.hpp"
#include "craft_perception/lidar_perception.hpp"
#include "craft_perception/localization_monitor.hpp"
#include "craft_perception/scan_odometry.hpp"
#include "craft_perception/scan_relocalizer.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/state_estimator.hpp"
#include "craft_perception/traffic_light.hpp"
#include "craft_perception/barcode_reader.hpp"
#include "craft_perception/msg/detection_array.hpp"
#include "craft_perception/msg/puck_observation_array.hpp"
#include "craft_perception/msg/traffic_light.hpp"
#include "craft_perception/msg/puck_survey.hpp"
#include <craft_interfaces/msg/task_command.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
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
enum Camera { kD435 = 0, kCameraCount };  // gripper D435 on shm; the upper cam has its own bridge
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

// Owns ROS I/O, parameters and scheduling, like control_master. D435Perception
// and LidarPerception own algorithms called through member functions.
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
  bool d435_object_detector(Task &task);
  bool d435_obstacle_detector(Task &task);
  bool d435_puck_axis(Task &task);
  // Puck survey: D435 frame -> puck YOLO (color + shape) + AprilTag (id) -> PuckSurvey.
  bool d435_puck_survey(Task &task);
  // Upper cam after the traffic light: same JPEG frames, puck YOLO + AprilTag -> the same PuckSurvey.
  // Runs only once the light is complete (or the traffic_light task is off): the signal model is
  // never needed again then, and the two never run on one frame.
  bool uppercam_puck_survey(Task &task);
  // Adds one frame's tagged pucks to puck_survey_ and publishes it when a reading changed.
  void record_pucks(const std_msgs::msg::Header &header, const std::vector<TaggedPuck> &pucks, const char *camera);
  void publish_puck_survey(const std_msgs::msg::Header &header);
  // Mission events for the dashboard: one line "[MISSION] <LEVEL> <CODE> <detail>" on the log and
  // "<LEVEL>|<CODE>|<detail>" on /perception/mission/events. LEVEL: INFO | WARN | ERROR.
  // CODE is a fixed word (SIGNAL_CYCLE_DISCARDED, BARCODE_UNREGISTERED, ...); detail is free text.
  void mission_event(const char *level, const char *code, const char *format, ...)
    __attribute__((format(printf, 4, 5)));
  // Traffic light: upper-cam JPEG (uppercam bridge) -> own YOLO -> blink counter.
  bool uppercam_traffic_light(Task &task);
  void publish_traffic_light(const std_msgs::msg::Header &header);
  bool lidar_obstacle(Task &task);
  void filter_obstacles(geometry_msgs::msg::PoseArray &obstacles, const sensor_msgs::msg::LaserScan &scan);
  std::optional<lidar_processing::Pose2D> lidar_pose(const std::string &frame);
  bool state_estimator(Task &task);
  // One raw OpenCR IMU sample (0 = wheel_clock, 1 = lm_arm) -> /perception/imu/data.
  void receive_imu(std::size_t board, const sensor_msgs::msg::Imu &input);
  // Localization monitor (localization:=ekf): SLAM pose check, dead reckoning, wide relocalization,
  // IMU corrections. Runs in its own callback group.
  void receive_slam_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &input);
  void receive_global_odom(const nav_msgs::msg::Odometry &input);
  void relocalize();
  // LiDAR odometry (lidar_odometry.enabled): /lidar/merged matched scan to keyframe -> /perception/lidar/odom.
  void lidar_odometry(const sensor_msgs::msg::LaserScan &scan);
  static std::vector<std::pair<double, double>> scan_points(const sensor_msgs::msg::LaserScan &scan);
  void publish_localization_state();
  void diagnostics();

  D435Perception d435_;
  LidarPerception lidar_;
  // /perception/lidar/obstacle filter (map walls, persistence, clusters); its map arrives on another thread.
  bool obstacle_filter_enabled_ = false;
  // A hard stop pitches the chassis and the LiDAR plane hits the floor (user, 2026-11-10): the scans from
  // brake_before_s before to brake_after_s after a deceleration over brake_decel (wheel odometry over brake_window_s)
  // are kept out of the obstacles (the filter goes back to before them) and the obstacles of then go out again.
  double brake_decel_ = 0.6, brake_window_s_ = 0.1, brake_before_s_ = 0.3, brake_after_s_ = 0.3;
  std::atomic<int64_t> brake_at_ns_{0};   // when the last hard stop was detected (wheel odometry stamp)
  // /lidar/localization is not published while the wheels turn faster than this [rad/s] and turn_hold_s after
  // (publish_localization_scan); 0 = always published.
  double localization_max_yaw_rate_ = 0.0, localization_turn_hold_s_ = 0.0;
  std::atomic<int64_t> turning_at_ns_{0};  // last wheel odometry stamp with |yaw rate| over it
  // Nor while the chassis rocks (bumpy floor, 2026-11-13): wheel board IMU roll or pitch rate over this [rad/s], and
  // turn_hold_s after; 0 = off.
  double localization_max_tilt_rate_ = 0.0;
  std::atomic<int64_t> tilting_at_ns_{0};  // last wheel board IMU stamp with a roll / pitch rate over it
  std::atomic<uint64_t> tilt_dropped_{0};  // scans not sent to SLAM for the tilt, for the log
  // Mapping only (craft.launch.py mapping_tilt_deg; 0 = off): nor while the chassis stands tilted on a slope over this
  // [rad] from level (wheel board gravity direction, ~0.5 s mean, held while the wheels speed up or brake), and
  // turn_hold_s after. Level = the mean gravity direction of all standstill samples so far.
  double localization_max_slope_ = 0.0;
  std::array<double, 3> slope_gravity_{}, slope_level_sum_{};
  int slope_level_n_ = 0;
  double slope_v_ = 0.0, slope_v_t_ = 0.0, slope_accel_ = 0.0;  // wheel forward speed, its stamp, acceleration mean
  std::atomic<int64_t> sloped_at_ns_{0};
  std::atomic<uint64_t> slope_dropped_{0};
  // lidar.motion_compensation: the merge moves every beam to the robot pose of one moment (ScanMerger, Motion) with
  // the latest forward speed (wheel odometry) and yaw rate (IMU; the wheels' without it).
  bool lidar_motion_compensation_ = false;
  std::atomic<double> merge_v_{0.0}, merge_w_imu_{0.0}, merge_w_wheel_{0.0};
  int64_t brake_handled_ns_ = 0;          // the hard stop already rolled back (lidar obstacle task only)
  // Per processed scan (lidar obstacle task only, ~1 s): its stamp, the filter state before it and the obstacles
  // published before it, so a hard stop's scans from brake_before_s_ back can be undone.
  struct ObstacleSnapshot { int64_t stamp; LidarObstacleFilter::State filter; std::optional<geometry_msgs::msg::PoseArray> published; };
  std::deque<ObstacleSnapshot> obstacle_snapshots_;
  std::deque<std::pair<int64_t, double>> brake_speeds_;  // (stamp ns, |vx|) of /control/wheel/odom, brake_window_s_ long
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr brake_odom_sub_;
  std::optional<geometry_msgs::msg::PoseArray> last_lidar_obstacles_;
  LidarObstacleFilter obstacle_filter_;
  std::mutex obstacle_filter_mutex_;
  std::mutex obstacle_cycle_mutex_;  // Serialize reset with filtered publications.
  int64_t obstacle_reset_stamp_ns_ = 0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_command_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_sub_;
  // A new Navigate task (planning EXECUTE, new task_id) clears the obstacle memory; renewals (UPDATE_PATH) keep it.
  rclcpp::Subscription<craft_interfaces::msg::TaskCommand>::SharedPtr task_command_sub_;
  uint64_t navigate_task_id_ = 0;
  std::string obstacle_odom_frame_;  // fixed frame for obstacles at the scan stamp (lidar_obstacle.odom_frame)
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
  // LiDAR odometry; the IMU group hands it the integrated yaw (t, yaw) of the last ~2 s for its prediction.
  bool lidar_odometry_enabled_ = false;
  ScanOdometry scan_odometry_;
  std::array<double, 3> lidar_twist_variance_{};  // vx, vy [(m/s)^2], vyaw [(rad/s)^2]
  std::string lidar_odom_frame_, lidar_base_frame_;
  std::optional<double> lidar_scan_at_;
  Pose2 lidar_pose_{0.0, 0.0, 0.0}, lidar_velocity_{0.0, 0.0, 0.0};
  std::mutex imu_yaw_mutex_;
  std::deque<std::pair<double, double>> imu_yaw_;
  double imu_yaw_sum_ = 0.0;
  std::map<std::string, ModuleSchedule> schedules_;
  std::vector<std::string> d435_labels_;
  int executor_threads_ = 0;
  double max_age_ = 0;
  std::string odometry_topic_;
  std::string map_frame_;
  // map->odom as used for /perception/state/odom: follows slam_toolbox's at a bounded rate (map_correction), so its
  // corrections (4-20 mm sideways about twice a second, 2026-11-10) do not jump the pose the path tracking steers by.
  struct MapCorrection { double x = 0, y = 0, yaw = 0, stamp = 0; };
  std::optional<MapCorrection> map_correction_;
  double map_correction_speed_ = 0, map_correction_yaw_rate_ = 0, map_correction_snap_m_ = 0, map_correction_snap_rad_ = 0;
  double map_correction_still_v_ = 0, map_correction_still_w_ = 0;

  // One mutex per input, held only to swap pointers; computation runs unlocked.
  // Lock order: cameras_[i].mutex before info_mutex_ (latest_frame); no other nesting.
  CameraInput cameras_[kCameraCount];
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
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr image_subs_[kCameraCount];
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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_odometry_scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr localization_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr relocalized_pub_;
  rclcpp::TimerBase::SharedPtr localization_timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub_;
  // /lidar/localization: the scan slam_toolbox localizes on, without the beams on the arena boxes (/planning/boxes).
  void publish_localization_scan(const sensor_msgs::msg::LaserScan &scan);
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr localization_scan_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr localization_boxes_sub_;
  geometry_msgs::msg::PoseArray::ConstSharedPtr localization_boxes_;
  std::mutex localization_boxes_mutex_;
  double localization_box_radius_ = 0;
  rclcpp::Publisher<msg::DetectionArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<msg::PuckObservationArray>::SharedPtr puck_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr apriltag_pub_;
  // Puck survey: its own YOLO (puck model) and tag reader on the D435 shared-memory frames.
  YoloDetector puck_yolo_;
  AprilTagReader puck_tags_;
  PuckClassMap puck_classes_;
  float puck_min_score_ = 0.4f;
  // Tags are searched only inside the puck boxes (grown, optionally upscaled), then judge the shape.
  struct PuckTagSearch { double grow = 1.4; int upscale = 1; PuckShapeRule rule; };
  PuckTagSearch puck_tag_search_, uppercam_puck_tag_search_;
  std::vector<std::string> puck_labels_;
  std::mutex puck_survey_mutex_;  // task and reset subscription
  PuckSurvey puck_survey_;
  rclcpp::Publisher<msg::DetectionArray>::SharedPtr puck_detection_pub_;
  // Step 4 of the puck pipeline (YOLO -> tags in the box -> shape rule -> axis): PuckTagAxis per YOLO puck box,
  // with the box's tags and the box itself (IoU term). One instance per camera; results on
  // /perception/vision/<camera>/puck_survey_observations.
  PuckTagAxis d435_survey_axis_, uppercam_survey_axis_;
  std::array<double, 4> uppercam_intrinsics_1080_{};  // fx, fy, cx, cy of the upper cam at 1920x1080
  rclcpp::Publisher<msg::PuckObservationArray>::SharedPtr d435_survey_obs_pub_, uppercam_survey_obs_pub_;
  // PuckTagAxis on each tagged puck (its tags + YOLO box); nullopt intrinsics: no axis.
  void puck_axes(std::vector<TaggedPuck> &pucks, const PuckTagAxis &module, const std::optional<Intrinsics> &in,
    cv::Size image) const;
  rclcpp::Publisher<msg::PuckSurvey>::SharedPtr puck_survey_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr puck_survey_reset_sub_;
  // Traffic light: the upper cam is not on shared memory; its bridge publishes JPEG frames.
  std::mutex uppercam_mutex_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr uppercam_image_;
  uint64_t uppercam_received_ = 0;  // frame id for Task::seen
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr uppercam_image_sub_;
  YoloDetector uppercam_yolo_;
  std::vector<std::string> uppercam_labels_;
  std::string uppercam_frame_;
  rclcpp::Publisher<msg::DetectionArray>::SharedPtr uppercam_detection_pub_;
  // Upper-cam puck survey (uppercam_puck task): its own puck model and tag reader.
  YoloDetector uppercam_puck_yolo_;
  AprilTagReader uppercam_puck_tags_;
  PuckClassMap uppercam_puck_classes_;
  float uppercam_puck_min_score_ = 0.4f;
  std::vector<std::string> uppercam_puck_labels_;
  rclcpp::Publisher<msg::DetectionArray>::SharedPtr uppercam_puck_detection_pub_;
  // Task and reset subscription share the counter and the barcode reader (same mutex).
  std::mutex traffic_light_mutex_;
  BarcodeReader barcode_;  // Code 128 next to the light, read on the same upper-cam frames
  BarcodeLocatorConfig barcode_locator_;  // uppercam.barcode: decode only where bars are (place not fixed)
  int barcode_full_frame_every_ = 15;     // also the whole frame every N frames while nothing was read
  int barcode_frames_ = 0;
  bool barcode_decode_ = false;  // uppercam.barcode.decode: ZXing as well as the pattern vote
  struct BarcodeMatch {  // uppercam.barcode.match: vote over the mission codes when nothing decodes
    bool enabled = false;
    std::vector<std::string> codes;
    int regions = 3, votes = 15;
    double min_score = 0.6, min_margin = 0.03, min_share = 0.7;
  } barcode_match_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr barcode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hires_pub_;  // /perception/uppercam/hires for the bridge
  void publish_hires(bool on);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;
  TrafficLightCounter traffic_light_;
  rclcpp::Publisher<msg::TrafficLight>::SharedPtr traffic_light_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr traffic_light_reset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr object_pub_, obstacle_pub_, lidar_obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  Steady::time_point diagnostics_at_;
};
}
