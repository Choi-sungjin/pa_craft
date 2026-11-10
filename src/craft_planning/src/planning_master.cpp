#include "craft_planning/planning_master.hpp"
#include "craft_planning/planning_config.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <craft_perception/msg/knob_observation_array.hpp>
#include <craft_perception/msg/puck_observation_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <future>
#include <limits>
#include <map>
#include <optional>
#include <utility>
#include <sstream>
#include <set>
#include <craft_interfaces/msg/box_observation.hpp>
#include <craft_interfaces/msg/box_observation_request.hpp>
#include <craft_interfaces/msg/task_command.hpp>
#include <craft_interfaces/msg/task_result.hpp>
#include <craft_interfaces/msg/control_status.hpp>

namespace craft_planning
{
namespace
{
using TaskCommand = craft_interfaces::msg::TaskCommand;
using TaskResult = craft_interfaces::msg::TaskResult;
using ControlStatus = craft_interfaces::msg::ControlStatus;
using BoxObservation = craft_interfaces::msg::BoxObservation;
using BoxObservationRequest = craft_interfaces::msg::BoxObservationRequest;
using Steady = std::chrono::steady_clock;
using Pose = geometry_msgs::msg::PoseStamped;
using Points = geometry_msgs::msg::PoseArray;
using Path = nav_msgs::msg::Path;
using Odom = nav_msgs::msg::Odometry;
using Joints = sensor_msgs::msg::JointState;
double seconds()
{
    return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
}
bool valid_pose(const Pose &p)
{
    const auto &v = p.pose.position;
    const auto &q = p.pose.orientation;
    const double n = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    return !p.header.frame_id.empty() && std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
           std::isfinite(n) && n > 1e-9;
}
template <class T> struct Sample
{
    T value{};
    double at = -std::numeric_limits<double>::infinity();
    bool valid = false;
    void observe(const T &input, double now)
    {
        value = input;
        at = now;
        valid = true;
    }
    bool fresh(double now, double timeout) const
    {
        return valid && now >= at && now - at <= timeout;
    }
};
struct Observation
{
    Pose pose;
    int class_id = -1;
    double confidence = 1.0;
    double received_at = seconds();
};
struct Computation
{
    std::size_t generation;
    bool ok = false;
    Pose box_goal;  // box approach: the chosen face goal (empty frame otherwise)
    double face_normal = 0;  // box approach: outward normal of the chosen face [rad, map]
    std::string faces;       // box approach: each face tried, its path length or why not
    std::string through_obstacles;  // no route past the obstacles (this error): the map-only route instead
    bool retryable = false;
    Path path;
    std::vector<double> speed_limit;  // per path pose [m/s]
    std::vector<std::size_t> stops;  // STOP_TURN corners: the path is sent one segment at a time
    Path global;                     // global_local mode: the global path the local path is cut from
    Joints joints;
    std::string error;
    Odom start;
    double plan_ms = 0;
    std::size_t expanded = 0;
};
} // namespace

struct PlanningMaster::Impl
{
    explicit Impl(PlanningMaster &owner) : node(owner)
    {
    }
    PlanningMaster &node;
    PlanningConfig config;
    StateMachine fsm;
    PathPlanner path_planner;
    ArmPlanner arm_planner;
    std::shared_ptr<tf2_ros::Buffer> tf;
    std::shared_ptr<tf2_ros::TransformListener> listener;
    Sample<Odom> odom, lidar_odom;
    // The dock by wheel odometry (box_approach.dock_wheel_odometry): the map pose at the dock's start plus the wheel
    // odometry's motion since, in place of the localization's pose while CameraDock / FinalStop / CameraVerify.
    Sample<Odom> wheel_odom;
    struct DockAnchor {double mx, my, myaw, wx, wy, wyaw;};
    std::optional<DockAnchor> dock_anchor;
    // [stamp s, x, y] of /perception/state/odom over the last odom_trail_s: how far the robot moved since a camera
    // frame was taken (travel_since).
    std::deque<std::array<double, 3>> odom_trail;
    static constexpr double odom_trail_s = 2.0;
    Sample<sensor_msgs::msg::Imu> imu;
    int64_t lidar_stamp = 0, imu_stamp = 0;
    Sample<Joints> joints;
    Sample<Points> lidar, vision_obstacles;
    Sample<ControlStatus> control;
    Sample<TaskResult> task_result;
    std::map<std::string, std::vector<Observation>> observations;
    std::optional<PlanningStep> step;
    std::deque<PlanningStep> remaining;
    std::string last_mission, readiness_reason;
    std::string localization_state = "ok";  // perception's localization monitor (localization:=ekf)
    std::deque<std::string> request_keys;
    std::future<Computation> worker;
    std::optional<Computation> result;
    Path active_path;
    std::vector<double> active_speed;  // speed profile of active_path, sent with every command for it
    // Box approach being driven: its face goal, so renewals and the blocked-path check ignore the box's own padding.
    std::optional<BoxApproach> approach;
    // The box as perceived (/perception/box/pose) standing still at the stage point (or after backing out); the dock
    // and the check go by it. Empty: still driving to the stage point of the known box (planning.yaml).
    std::optional<BoxTarget> box_seen;
    Sample<Pose> box_detection;
    // Box step: toward the box (no approach yet) -> Approach (driving the approach) -> Settle (standing at the face
    // goal, then checking the alignment) -> Reverse (backing out to approach the same face again) -> Approach ...
    // Stage (path to the stage point of the known box) -> Perceive (stand still, LiDAR box pose) -> Dock (turn
    // square, straight to the face goal of the perceived box) -> Settle (stand still, check) -> done, or Reverse (back
    // out) -> Perceive -> Dock ...
    enum class BoxPhase {Stage, Perceive, Dock, Settle, Reverse,
      CameraStageStop, LidarPerceive, LidarRelease, LidarDock, HandoffStop, CameraPerceive, CameraRelease, CameraDock, FinalStop, CameraVerify};
    BoxPhase box_phase = BoxPhase::Stage;
    Sample<BoxObservation> box_observation;
    // LiDAR observations taken standing still at the stage point (LidarPerceive, after settle_s): their median
    // makes the dock line (lidar_window_s).
    std::vector<BoxObservation> lidar_samples;
    BoxObservationRequest box_request;
    rclcpp::Publisher<BoxObservationRequest>::SharedPtr box_request_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr box_phase_pub;
    double box_request_at = 0, camera_stop_at = 0, handoff_lidar_gap = 0;
    double handoff_ack_at = 0;
    int64_t handoff_ack_stamp = 0;
    bool box_had_observation = false;
    bool handoff_lidar_captured = false;
    // camera_only_longitudinal: false while the dock follows the LiDAR line before the first valid upper-cam gap.
    bool camera_engaged = true;
    int camera_confirmed = 0;
    int64_t camera_last_confirm_stamp = 0, box_last_observation_stamp = 0, camera_stop_stamp = 0;
    double box_perceive_at = 0;
    double box_settle_at = 0;
    int box_corrections = 0;
    bool box_reoriented = false;  // this box step already planned again for a turned box
    bool stage_turned = false;    // the stage drive was replaced by a turn in place (stuck circling off its point)
    bool stage_final = false;     // within stage_final_m of the stage point: short target
    std::optional<double> stage_still_since;  // turned in place within the heading tolerance and standing still since
    std::optional<double> stage_near_since;  // within stage_capture_m of the stage point: since the last cm closer
    double stage_near_best = 0;              // the closest it got [m]
    bool box_reported = false;  // the END line went out
    std::optional<double> box_face;  // outward normal of the face being approached [rad, map]
    enum class BoxMove {None, Dock, Reverse};
    BoxMove box_pending = BoxMove::None;            // a dock or back-out waiting for a clear line
    std::optional<double> box_blocked_at;           // since when that line was blocked
    double active_goal_tolerance = 0, active_heading_tolerance = 0;  // of the active drive, kept by pause/resume
    double active_tracking_reference_x = 0, active_lookahead_distance = 0;
    bool active_camera_distance_control = false;
    double active_camera_gap = 0;
    builtin_interfaces::msg::Time active_camera_stamp;
    int64_t active_camera_stamp_ns = 0;
    // global_local mode: the global path of the current Navigate step and the global arc length of the robot when
    // the local path being driven was made (renewed after path_planner.local_update of progress).
    Path global_path;
    double local_origin = 0;
    Pose resolved_goal;
    uint64_t next_id = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
    uint64_t revision = 0;
    double command_at = 0, last_report = -1, compute_at = 0, last_replan = 0, stop_at = 0;
    double retry_compute_at = 0;
    bool dispatched = false, fault_reported = false, reset_requested = false;
    bool waiting_for_path = false, route_paused = false;
    // A Navigate path is sent up to its first STOP_TURN corner. Reaching that end plans the next segment
    // from there (new task ID); replanning while driving happens only when the remaining path is blocked.
    bool more_segments = false, next_segment = false;
    // Blocked path: pause first, replan only if still blocked after block_wait_s (a passer-by clears).
    std::optional<double> blocked_since;
    bool route_failed = false;  // the last replan found no route: keep retrying
    geometry_msgs::msg::Point segment_start;
    int no_progress = 0;  // segments in a row that "arrived" where they started
    double plan_ms = 0, plan_ms_max = 0;
    std::size_t plan_count = 0;
    // Graspable check of the nearest observed puck (grasp_region.hpp), refreshed per puck observation.
    GraspCheck grasp_check;
    GraspConfirm grasp_confirm;
    bool grasp_confirmed = false;
    std::string grasp_puck;  // nearest puck in base_link, for measuring the regions
    rclcpp::Publisher<TaskCommand>::SharedPtr command_pub;
    rclcpp::Publisher<Odom>::SharedPtr odom_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::Publisher<Pose>::SharedPtr box_goal_pub;  // /planning/approach_goal: the chosen box face goal
    rclcpp::Publisher<Pose>::SharedPtr box_search_pub;  // /planning/box_search: where the box step expects its box
    rclcpp::Publisher<Points>::SharedPtr ignored_pub;  // /planning/ignored_obstacles: obstacle points dropped for the box
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr padding_pub;  // /planning/padding: what plans are checked against
    double padding_pub_at = -1e9;
    mutable double ignored_pub_at = -1e9;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr box_status_pub;  // /planning/box_status: one line per step
    std::string box_blocked_by;  // why the last dock/back-out line was not clear
    std::string last_compute_error;
    std::string last_obs_reject_reason;
    mutable std::string last_alignment_detail;
    // /planning/box_alignment after each settle: [lateral m, yaw rad, forward m, aligned 0/1, correction]
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr box_alignment_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
    rclcpp::TimerBase::SharedPtr timer;

    template <class T, class Callback> void subscribe(const std::string &topic, Callback callback)
    {
        subscriptions.push_back(node.create_subscription<T>(topic, 10, callback));
    }
    void hold(bool value)
    {
        std_msgs::msg::Bool msg;
        msg.data = value;
        hold_pub->publish(msg);
    }
    void stop() {hold(true);}
    bool active() const {return fsm.active();}
    void request_stop(bool end_run) {
      if (!active()) return;
      if (fsm.phase() != RunPhase::Stopping) stop_at = seconds();
      fsm.request_stop(end_run, seconds());
      remaining.clear(); result.reset(); waiting_for_path = false; route_paused = false;
      more_segments = next_segment = false; global_path.poses.clear(); blocked_since.reset(); route_failed = false; no_progress = 0;
      TaskCommand c; c.header.stamp = node.now(); c.operation = TaskCommand::STOP;
      c.task_id = dispatched && step ? step->task_id : 0;
      command_pub->publish(c); stop();
    }
    bool control_fresh(double now) const {
      return control.fresh(now, config.fsm.feedback_timeout_s) &&
        is_fresh_input(control.value.header, config.fsm.feedback_timeout_s);
    }
    bool localization_fresh(double now) const {
      if (!config.require_lidar_localization) return true;
      const auto timeout = config.localization_observation_timeout_s;
      return lidar_odom.fresh(now, timeout) && is_fresh_input(lidar_odom.value.header, timeout) &&
        imu.fresh(now, timeout) && is_fresh_input(imu.value.header, timeout);
    }
    template<class T> void receive_localization(const T &msg, Sample<T> &sample, int64_t &last_stamp, bool values_valid) {
      const auto &time = msg.header.stamp;
      const int64_t stamp = static_cast<int64_t>(time.sec) * 1000000000LL + time.nanosec;
      if (!values_valid || time.sec < 0 || time.nanosec >= 1000000000U || stamp <= last_stamp ||
          !is_fresh_input(msg.header, config.localization_observation_timeout_s)) {
        sample.valid = false;
        if (config.require_lidar_localization && active()) {
          const auto now = seconds();
          fsm.fail("invalid lidar odom or IMU observation", now);
          remaining.clear(); result.reset(); waiting_for_path = false; route_paused = false;
          TaskCommand c; c.header.stamp = node.now(); c.operation = TaskCommand::STOP;
          c.task_id = dispatched && step ? step->task_id : 0;
          command_pub->publish(c); stop();
        }
        return;
      }
      last_stamp = stamp;
      sample.observe(msg, seconds());
    }
    bool system_ready(double now) const {
      return localization_fresh(now) && control_fresh(now) && control.value.ready && !control.value.fault &&
        odom.fresh(now, config.odom_timeout_s) && is_fresh_input(odom.value.header, config.odom_timeout_s) &&
        (!config.require_arm_joints ||
         (joints.fresh(now, config.joint_timeout_s) && is_fresh_input(joints.value.header, config.joint_timeout_s))) &&
        (!config.require_lidar || (lidar.fresh(now, config.obstacle_timeout_s) &&
         is_fresh_input(lidar.value.header, config.obstacle_timeout_s)));
    }
    bool is_fresh_input(const std_msgs::msg::Header &header, double max_age) const
    {
        const double age = (node.now() - rclcpp::Time(header.stamp, node.get_clock()->get_clock_type())).seconds();
        return age >= -0.01 && age <= max_age;
    }
    Pose transform(const Pose &pose, const std::string &target) const
    {
        if (!valid_pose(pose))
        {
            throw std::invalid_argument("invalid observation pose");
        }
        if (pose.header.frame_id == target)
        {
            return pose;
        }
        // No blocking TF wait in the scheduler. Transform the acquisition timestamp.
        // https://github.com/ros2/geometry2/blob/rolling/tf2_ros/include/tf2_ros/buffer_interface.hpp
        return tf->transform(pose, target, tf2::durationFromSec(0.0));
    }
    void receive_points(const Points &msg, Sample<Points> &sample)
    {
        // A rejected message keeps the previous sample: its obstacle_timeout_s still applies. Invalidating it here
        // let one scan stamped a few ms ahead of TF fault a running Navigate (corner demo, 2026-11-01).
        if (!is_fresh_input(msg.header, config.obstacle_timeout_s) || msg.header.frame_id.empty())
        {
            return;
        }
        try
        {
            Points points;
            points.header = msg.header;
            points.header.frame_id = config.path.map_frame;
            for (const auto &value : msg.poses)
            {
                Pose pose;
                pose.header = msg.header;
                pose.pose = value;
                // Obstacle orientation is unused; only the finite positions are interpreted.
                pose.pose.orientation = geometry_msgs::msg::Quaternion{};
                pose.pose.orientation.w = 1.0;
                points.poses.push_back(transform(pose, config.path.map_frame).pose);
            }
            sample.observe(points, seconds());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000, "obstacle TF/input: %s", e.what());
        }
    }
    Points obstacles(double now) const
    {
        Points points;
        points.header.frame_id = config.path.map_frame;
        for (const auto *input : {&lidar, &vision_obstacles})
        {
            if (input->fresh(now, config.obstacle_timeout_s))
            {
                points.poses.insert(points.poses.end(), input->value.poses.begin(), input->value.poses.end());
            }
        }
        // The box being approached is an obstacle like any other until the last approach path (the dock, after the
        // stage point) is made; from then on (docking()) every point within ignore_radius of it as the LiDAR measured
        // it goes (user, 2026-11-10; its turned or smeared points fell outside the footprint the approach frees and
        // blocked the dock). Not around the planning.yaml place: a box standing off it would be partly freed.
        Points ignored; ignored.header = points.header; ignored.header.stamp = node.now();
        if (step && !step->box.empty() && box_seen && docking()) {
          const double r2 = config.box.ignore_radius * config.box.ignore_radius, cx = box_seen->x, cy = box_seen->y;
          auto &v = points.poses;
          const auto cut = std::stable_partition(v.begin(), v.end(), [&](const geometry_msgs::msg::Pose &q) {
            const double dx = q.position.x - cx, dy = q.position.y - cy; return dx * dx + dy * dy > r2;});
          ignored.poses.assign(cut, v.end()); v.erase(cut, v.end());
        }
        // /planning/ignored_obstacles: the points dropped for the box (RViz, green), at most 5 Hz.
        if (ignored_pub && now - ignored_pub_at >= 0.2) {ignored_pub_at = now; ignored_pub->publish(ignored);}
        return points;
    }
    // A box step drives at box_approach.speed to the stage point (no heading), docks at dock_speed with the face
    // heading required and backs out at -reverse_speed.
    bool box_in(BoxPhase phase) const {return step && !step->box.empty() && box_phase == phase;}
    // One line per box step for the operator (run_corner prints /planning/box_status), also in the log.
    void say(const std::string &text) {
      RCLCPP_INFO(node.get_logger(), "%s", text.c_str());
      std_msgs::msg::String m; m.data = text; box_status_pub->publish(m);
    }
    static std::string xy(double x, double y) {
      char t[48]; std::snprintf(t, sizeof(t), "(%.3f, %.3f)", x, y); return t;
    }
    // What blocks a line: the nearest obstacle point within the padding, else the map (a wall's padding).
    std::string blocker(const Path &path, double now) const {
      const double pad = config.path.r_robot + config.path.margin;
      double best = pad; std::string what = "the map padding (a wall within " + std::to_string(pad).substr(0, 5) + " m)";
      for (const auto &o : obstacles(now).poses) {
        for (std::size_t i = 1; i < path.poses.size(); ++i) {
          const auto &a = path.poses[i - 1].pose.position, &b = path.poses[i].pose.position;
          const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
          const double t = len2 > 0 ? std::clamp(((o.position.x - a.x) * dx + (o.position.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
          const double d = std::hypot(o.position.x - a.x - t * dx, o.position.y - a.y - t * dy);
          if (d < best) {best = d; what = "an obstacle at " + xy(o.position.x, o.position.y) + ", " +
                         std::to_string(static_cast<int>(std::lround(d * 100))) + " cm from the line";}
        }
      }
      return what;
    }
    // The normal of the face of b nearest the one being approached (box_face).
    double face_normal(const BoxTarget &b) const {
      double normal = b.yaw;
      for (int k = 1; k < 4; ++k) {
        const double n = b.yaw + k * M_PI / 2;
        if (std::abs(std::remainder(n - *box_face, 2 * M_PI)) < std::abs(std::remainder(normal - *box_face, 2 * M_PI))) normal = n;
      }
      return normal;
    }
    // The box step's last line (done or failed): the robot against the face goal and the box face, by the box as
    // last perceived (else the known one).
    void box_end(const std::string &how) {
      if (box_reported || !step || step->box.empty() || !odom.valid) return;
      box_reported = true;
      const auto b = box_seen ? *box_seen : box_target(step->box);
      const auto &o = odom.value.pose.pose.position;
      char t[480];
      if (box_face) {
        const double n = face_normal(b), reach = b.side / 2 + b.standoff;
        const double gx = b.x + reach * std::cos(n), gy = b.y + reach * std::sin(n);
        const double face = (o.x - b.x) * std::cos(n) + (o.y - b.y) * std::sin(n) - b.side / 2;  // centre -> face
        // The box centre in base_link (x ahead, y left) and the yaw to the face, by odometry; the upper cam's last
        // gap when it has one (2026-11-10: the distance after the dock, for the operator).
        const auto &q = odom.value.pose.pose.orientation;
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const double cx = std::cos(yaw) * (b.x - o.x) + std::sin(yaw) * (b.y - o.y);
        const double cy = -std::sin(yaw) * (b.x - o.x) + std::cos(yaw) * (b.y - o.y);
        const double yaw_err = std::remainder(yaw - (n + M_PI), 2 * M_PI);
        char cam[80] = "";
        const auto &c = box_observation.value;
        if (box_observation.valid && c.source == "uppercam" && c.valid)
          std::snprintf(cam, sizeof(cam), " | upper cam gap %.1f mm", c.gap_m * 1000);
        std::snprintf(t, sizeof(t), "[box %s] END %s | %.1f mm from the face goal %s | box face %.1f mm from the "
                      "robot centre, %.1f mm from the front (target gap %.0f mm)%s | box centre in base_link "
                      "x %.1f y %+.1f mm, yaw %+.2f deg", step->box.c_str(), how.c_str(),
                      std::hypot(o.x - gx, o.y - gy) * 1000, xy(gx, gy).c_str(), face * 1000,
                      (face - config.box.front_face_x) * 1000,
                      (config.box.camera_enabled ? config.box.camera_final_gap : config.box.gap) * 1000, cam,
                      cx * 1000, cy * 1000, yaw_err * 180 / M_PI);
      } else {
        std::snprintf(t, sizeof(t), "[box %s] END %s | box centre %s %.3f m from the robot", step->box.c_str(),
                      how.c_str(), xy(b.x, b.y).c_str(), std::hypot(o.x - b.x, o.y - b.y));
      }
      say(t);
    }
    double drive_speed() const {
      if (box_in(BoxPhase::Reverse)) return -config.box.reverse_speed;
      if (box_in(BoxPhase::CameraDock))
        return config.box.camera_only_longitudinal ? config.box.camera_fast_speed : config.box.camera_speed;
      if (box_in(BoxPhase::Dock) || box_in(BoxPhase::LidarDock)) return config.box.dock_speed;
      return step && !step->box.empty() ? config.box.speed : config.cruise_speed;
    }
    bool heading_required() const {
      if (step && !step->box.empty()) return box_phase == BoxPhase::Dock ||
        (config.box.camera_enabled && box_phase == BoxPhase::Stage && !more_segments);
      return config.goal_heading_required;
    }
    // The perceived box, if fresh and near the expected position (else another box or none).
    std::optional<BoxTarget> detected_box(double now) const {
      if (!step || step->box.empty() || !box_detection.fresh(now, config.box_detection_timeout_s) ||
          !is_fresh_input(box_detection.value.header, config.box_detection_timeout_s)) return std::nullopt;
      const auto &q = box_detection.value.pose;
      auto t = box_target(step->box);
      if (std::hypot(q.position.x - t.x, q.position.y - t.y) > config.box.max_offset) return std::nullopt;
      t.x = q.position.x; t.y = q.position.y;
      t.yaw = std::atan2(2 * (q.orientation.w * q.orientation.z + q.orientation.x * q.orientation.y),
                         1 - 2 * (q.orientation.y * q.orientation.y + q.orientation.z * q.orientation.z));
      return t;
    }
    BoxTarget box_target(const std::string &name) const {
      const auto &b = config.box.boxes.at(name);
      return {b[0], b[1], b[2], config.box.side, config.box.front_face_x + config.box.gap};
    }
    bool valid_request(const PlanningStep &r) const {
      if (r.kind == StepKind::Navigate && !r.box.empty()) return config.box.boxes.count(r.box) > 0;
      if (r.kind == StepKind::Navigate || r.kind == StepKind::Tcp)
        return valid_pose(r.goal) && r.goal.header.frame_id ==
          (r.kind == StepKind::Navigate ? config.path.map_frame : config.arm.base_frame);
      if (r.kind == StepKind::Clock) return r.slot >= 1 && r.slot <= 12;
      if (r.kind == StepKind::Wait) return std::isfinite(r.duration_s) && r.duration_s >= 0;
      return true;
    }
    void accept(PlanningStep request, std::deque<PlanningStep> rest = {}, std::string key = {}) {
      if (!active() || fsm.phase() == RunPhase::Stopping || remaining.size() + rest.size() >= 64 ||
          !valid_request(request) || !std::all_of(rest.begin(), rest.end(),
            [this](const auto &r) {return valid_request(r);})) {
        RCLCPP_WARN(node.get_logger(), "request rejected: not running, queue full or invalid target"); return;
      }
      if (!key.empty() && std::find(request_keys.begin(), request_keys.end(), key) != request_keys.end()) return;
      if (!key.empty()) {
        request_keys.push_back(key); if (request_keys.size() > 1024) request_keys.pop_front();
      }
      request.task_id = ++next_id; remaining.push_back(std::move(request));
      for (auto &r : rest) {r.task_id = ++next_id; remaining.push_back(std::move(r));}
    }
    void lifecycle(const std::string &name) {
      if (name == "START") {fsm.begin_run(seconds()); return;}
      if (name == "STOP" || name == "CANCEL") {request_stop(name == "STOP"); return;}
      if (name == "RESET") {reset_requested = fsm.state() == SystemState::Fault; return;}
      RCLCPP_WARN(node.get_logger(), "unknown lifecycle command '%s'", name.c_str());
    }
    void mission(const std::string &name) {
      if (!active() || name == last_mission) return;
      const auto found = config.missions.find(name);
      if (found == config.missions.end() || found->second.empty()) {
        RCLCPP_WARN(node.get_logger(), "mission '%s' has no configured steps", name.c_str()); return;
      }
      const auto before = remaining.size();
      accept(found->second.front(), {found->second.begin()+1, found->second.end()});
      if (remaining.size() > before) last_mission = name;
    }
    bool resolve_goal(double now)
    {
        if (step->source == "fixed")
        {
            resolved_goal = step->goal;
            return true;
        }
        const auto found = observations.find(step->source);
        if (found == observations.end())
        {
            readiness_reason = "waiting for target observation";
            return false;
        }
        const Observation *best = nullptr;
        for (const auto &candidate : found->second)
        {
            if (!valid_pose(candidate.pose) || (step->class_id >= 0 && candidate.class_id != step->class_id) ||
                !std::isfinite(candidate.confidence) || candidate.confidence < config.observation_min_confidence ||
                now - candidate.received_at > config.observation_timeout_s ||
                !is_fresh_input(candidate.pose.header, config.observation_timeout_s))
            {
                continue;
            }
            if (!best || candidate.confidence > best->confidence)
            {
                best = &candidate;
            }
        }
        if (!best)
        {
            readiness_reason = "no fresh matching target observation";
            return false;
        }
        try
        {
            resolved_goal = transform(best->pose, step->goal.header.frame_id);
            resolved_goal.pose.position.x += step->goal.pose.position.x;
            resolved_goal.pose.position.y += step->goal.pose.position.y;
            resolved_goal.pose.position.z += step->goal.pose.position.z;
            resolved_goal.pose.orientation = step->goal.pose.orientation;
            return true;
        }
        catch (const std::exception &e)
        {
            readiness_reason = e.what();
            return false;
        }
    }
    bool sensors_fresh(double now) const
    {
        if (step->kind == StepKind::Navigate)
        {
            return odom.fresh(now, config.odom_timeout_s) && is_fresh_input(odom.value.header, config.odom_timeout_s) &&
                   (!config.require_lidar || (lidar.fresh(now, config.obstacle_timeout_s) &&
                                              is_fresh_input(lidar.value.header, config.obstacle_timeout_s)));
        }
        if (step->kind == StepKind::Tcp)
        {
            return joints.fresh(now, config.joint_timeout_s) &&
                   is_fresh_input(joints.value.header, config.joint_timeout_s);
        }
        return true;
    }
    bool ready(double now)
    {
        readiness_reason.clear();
        const bool near = step && step->box_near_entry;
        if (!control_fresh(now) || !control.value.ready ||
            (near ? (!control.value.held || !control.value.stopped) : control.value.held))
        {
            readiness_reason = "waiting for control release/feedback";
            return false;
        }
        if (!sensors_fresh(now))
        {
            readiness_reason = "waiting for fresh odom/lidar/joint state";
            return false;
        }
        if (worker.valid())
        {
            readiness_reason = "waiting for previous calculation to finish";
            return false;
        }
        if (step->kind == StepKind::Tcp || step->kind == StepKind::Navigate)
        {
            return resolve_goal(now);
        }
        return true;
    }
    void collect()
    {
        if (!worker.valid() || worker.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            return;
        }
        Computation value = worker.get();
        if (value.plan_ms > 0)
        {
            plan_ms = value.plan_ms; plan_ms_max = std::max(plan_ms_max, value.plan_ms); ++plan_count;
            if (value.ok)
            {
                RCLCPP_INFO(node.get_logger(), "path plan %.1f ms (max %.1f ms over %zu), %zu A* states, %zu poses, %zu stops",
                            value.plan_ms, plan_ms_max, plan_count, value.expanded, value.path.poses.size(), value.stops.size());
                if (!value.box_goal.header.frame_id.empty())
                {
                    const auto &q = value.box_goal.pose.orientation;
                    const double heading = std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z);
                    const auto &end = (value.global.poses.empty() ? value.path : value.global).poses.back().pose.position;
                    const char *faces[] = {"east", "north", "west", "south"};
                    const int face = ((static_cast<int>(std::lround((heading + M_PI) / (M_PI / 2))) % 4) + 4) % 4;
                    say("[box " + (step ? step->box : std::string("?")) + "] 1/5 shortest of the approachable faces: " +
                        faces[face] + " face, driving to its stage point " + xy(end.x, end.y) + " (face goal " +
                        xy(value.box_goal.pose.position.x, value.box_goal.pose.position.y) + ")");
                    RCLCPP_INFO(node.get_logger(), "[box %s] 1/5 faces: %s", step ? step->box.c_str() : "?", value.faces.c_str());
                    box_goal_pub->publish(value.box_goal);
                }
            }
            else
            {
                last_compute_error = value.error;
                RCLCPP_ERROR(node.get_logger(), "[PLANNING COMPUTE FAIL] step=%s box=%s retryable=%d plan_ms=%.1f obstacles=%zu odom=(%.3f,%.3f): %s",
                             step ? step->id.c_str() : "none", step ? step->box.c_str() : "", value.retryable ? 1 : 0,
                             value.plan_ms, obstacles(seconds()).poses.size(),
                             odom.value.pose.pose.position.x, odom.value.pose.pose.position.y, value.error.c_str());
            }
        }
        if (active() && value.generation == fsm.generation())
        {
            result = std::move(value);
        }
    }
    void compute(double now)
    {
        result.reset();
        compute_at = now;
        if (step->box_near_entry) {
          Computation out; out.generation = fsm.generation(); out.ok = false;
          if (config.box.camera_enabled && sensors_fresh(now) && control_fresh(now) &&
              control.value.held && control.value.stopped) {
            const auto b = box_target(step->box); const auto &p = odom.value.pose.pose.position;
            double best = -std::numeric_limits<double>::infinity(), normal = b.yaw;
            for (int k = 0; k < 4; ++k) {
              const double a = b.yaw + k*M_PI/2;
              const double d = std::cos(a)*(p.x-b.x) + std::sin(a)*(p.y-b.y);
              if (d > best) {best = d; normal = a;}
            }
            const auto &q = odom.value.pose.pose.orientation;
            const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (std::isfinite(norm) && norm >= 1e-12) {
              const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y)/norm, 1-2*(q.y*q.y + q.z*q.z)/norm);
              if (std::isfinite(yaw) && box_in_front_lidar_fov(yaw)) {
                box_seen.reset(); box_face = normal; box_reported = false; out.ok = true;
              }
            }
          }
          if (!out.ok) out.error = "near entry requires held stopped control and box in front lidar FOV";
          result = std::move(out); return;
        }
        if (step->kind != StepKind::Navigate && step->kind != StepKind::Tcp) {
            Computation out; out.generation = fsm.generation(); out.ok = valid_request(*step);
            result = out; return;
        }
        const auto generation = fsm.generation();
        const auto goal = resolved_goal;
        const auto kind = step->kind;
        const auto box = step->box;
        if (!box.empty()) {
          // A new box step forgets the previous one's box: the stage drive goes by the known (planning.yaml) box.
          if (fsm.phase() == RunPhase::Compute) {
            box_seen.reset(); box_phase = BoxPhase::Stage; camera_engaged = true; box_corrections = 0; box_face.reset(); box_reported = false; box_reoriented = false; stage_turned = false; stage_near_since.reset(); stage_final = false; stage_still_since.reset();
            box_pending = BoxMove::None; box_blocked_at.reset();
          }
          const auto expected = box_target(box);
          Pose search; search.header.frame_id = config.path.map_frame; search.header.stamp = node.now();
          search.pose.position.x = expected.x; search.pose.position.y = expected.y;
          search.pose.orientation.z = std::sin(expected.yaw / 2); search.pose.orientation.w = std::cos(expected.yaw / 2);
          box_search_pub->publish(search);
        }
        const auto seen = box_seen;
        const auto face = box_face;
        const auto current_odom = odom.value;
        const auto current_joints = joints.value;
        const auto points = obstacles(now);
        worker = std::async(std::launch::async, [this, generation, goal, kind, box, seen, face, current_odom, current_joints, points] {
            Computation output;
            output.generation = generation;
            output.start = current_odom;
            try
            {
                if (kind == StepKind::Navigate)
                {
                    const auto began = std::chrono::steady_clock::now();
                    // A box: the stage point of every approachable face of the known box (or the perceived one, face
                    // kept, on a replan), the shortest path wins.
                    const auto target = box.empty() ? BoxTarget{} : seen.value_or(box_target(box));
                    const double face_normal = face.value_or(0.0);
                    const auto route = [&](const geometry_msgs::msg::PoseArray &seen_obstacles) {
                      return box.empty() ? path_planner.plan(current_odom, goal, seen_obstacles) :
                        path_planner.plan_box(current_odom, target, seen_obstacles, face ? &face_normal : nullptr,
                                              config.box.stage_lead, &config.box.box_faces.at(box));};
                    auto plan = route(points);
                    // No route past the LiDAR obstacles: the map-only route, driven up to the obstacle (the blocked
                    // path check stops block_stop_distance before it and keeps replanning; user, 2026-11-10).
                    if (!plan.ok && !points.poses.empty()) {
                      auto open = route(geometry_msgs::msg::PoseArray());
                      if (open.ok) {output.through_obstacles = plan.error; plan = std::move(open);}
                    }
                    output.box_goal = plan.goal;
                    output.face_normal = plan.face_normal;
                    output.faces = plan.faces;
                    output.plan_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
                    output.ok = plan.ok;
                    output.retryable = plan.retryable;
                    output.path = plan.path;
                    output.speed_limit = plan.speed_limit;
                    output.stops = plan.stops;
                    output.global = plan.global;
                    output.expanded = plan.expanded;
                    output.error = plan.error;
                }
                else
                {
                    const auto plan = arm_planner.plan(goal, current_joints);
                    output.ok = plan.ok;
                    output.joints = plan.joints;
                    output.error = plan.error;
                }
            }
            catch (const std::exception &e)
            {
                output.error = e.what();
            }
            return output;
        });
    }
    // Past the drive to the stage point (turned there, the dock path): the map padded with r_approach.
    bool docking() const {return approach.has_value() && box_phase != BoxPhase::Stage;}
    void command(double now, bool update = false) {
      if (step->box_near_entry && !update) {
        result.reset(); command_at = now; dispatched = false;
        camera_stop(BoxPhase::CameraStageStop, now);
        say("[box " + step->box + "] near entry: held, checking LiDAR before camera approach");
        return;
      }
      TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id;
      c.revision = ++revision; c.operation = update ? TaskCommand::UPDATE_PATH : TaskCommand::EXECUTE;
      c.kind = static_cast<uint8_t>(step->kind);
      switch (step->kind) {
        case StepKind::Navigate: {
          // A box approach plan (face goal) starts the Approach phase before its speed and heading are chosen.
          if (!result->box_goal.header.frame_id.empty()) {
            approach = BoxApproach{box_seen.value_or(box_target(step->box)), result->box_goal.pose};
            box_face = result->face_normal; box_phase = BoxPhase::Stage; camera_engaged = true;
          } else if (step->box.empty() || !update) {approach.reset();}
          c.path = result->path; c.path.header.stamp = node.now(); c.speed = drive_speed();
          {
            // A STOP_TURN within min_segment_m of the start is skipped (control turns in place first anyway): a
            // segment shorter than the arrival tolerance would "arrive" at once, forever (2026-11-01 bags).
            std::optional<std::size_t> stop;
            double along = 0;
            std::size_t next = 0;
            for (std::size_t i = 1; i < c.path.poses.size() && next < result->stops.size(); ++i) {
              const auto &a = c.path.poses[i - 1].pose.position, &b = c.path.poses[i].pose.position;
              along += std::hypot(b.x - a.x, b.y - a.y);
              while (next < result->stops.size() && result->stops[next] < i) ++next;
              if (next < result->stops.size() && result->stops[next] == i && along >= config.min_segment_m) {stop = i; break;}
            }
            more_segments = stop.has_value();
            c.speed_limit = result->speed_limit;
            if (stop) {c.path.poses.resize(*stop + 1); c.speed_limit.resize(*stop + 1);}
            segment_start = odom.value.pose.pose.position;
            global_path = result->global;
            if (!global_path.poses.empty()) {
              // A local path that ends short of the global end is a segment: if it is driven to its end without a
              // renewal (local planning kept failing), that end plans the rest like a STOP_TURN segment end.
              local_origin = path_progress(global_path, odom.value.pose.pose.position);
              const auto &a = c.path.poses.back().pose.position, &b = global_path.poses.back().pose.position;
              more_segments = std::hypot(a.x - b.x, a.y - b.y) > 1e-6;
            }
          }
          const bool camera_final_stage = config.box.camera_enabled && box_in(BoxPhase::Stage) && !more_segments;
          if (camera_final_stage) {
            if (!box_face || c.path.poses.empty()) throw std::runtime_error("box stage face/path missing");
            const double heading = *box_face + M_PI;
            auto &q = c.path.poses.back().pose.orientation;
            q.x = q.y = 0.0; q.z = std::sin(heading / 2.0); q.w = std::cos(heading / 2.0);
            c.tolerance_rad = config.box.yaw_tolerance;
            // The last stage_final_m into the stage point slowly (stage_final_speed), and once there a short target
            // (stage_final_lookahead from base_link, see camera_box_tick) instead of the speed table's 0.18 m: a point
            // to stop at, not a line to follow (2026-11-10: 4-14 cm off and circling).
            double left = 0;
            for (std::size_t i = c.path.poses.size(); i-- > 0;) {
              if (i + 1 < c.path.poses.size()) {
                const auto &a = c.path.poses[i].pose.position, &b = c.path.poses[i + 1].pose.position;
                left += std::hypot(b.x - a.x, b.y - a.y);
              }
              if (left > config.box.stage_final_m) break;
              if (i < c.speed_limit.size()) c.speed_limit[i] = std::min(c.speed_limit[i], config.box.stage_final_speed);
            }
            if (stage_final) {c.tracking_reference_x = 0.0; c.lookahead_distance = config.box.stage_final_lookahead;}
          }
          c.require_goal_heading = heading_required();
          // The stage drive stops close to the face normal (the dock starts there).
          c.goal_tolerance = box_in(BoxPhase::Stage) ? config.box.stage_goal_tolerance : 0.0;
          active_goal_tolerance = c.goal_tolerance; active_heading_tolerance = c.tolerance_rad;
          active_tracking_reference_x = c.tracking_reference_x; active_lookahead_distance = c.lookahead_distance;
          active_camera_distance_control = false; active_camera_stamp_ns = 0;
          active_path = c.path; active_speed = c.speed_limit; route_paused = false; route_failed = false; blocked_since.reset();
          if (!result->through_obstacles.empty()) {
            say("no route past the LiDAR obstacles (" + result->through_obstacles + "): driving the map route up to the obstacle");
          }
          // Blocked right ahead (a map-only route, or a replan standing at the obstacle): sent standing still.
          if (!remaining_path_clear(now)) {c.speed = 0.0; route_paused = true;}
          break;
        }
        case StepKind::Tcp: c.joints = result->joints; c.joints.header.stamp = node.now(); break;
        case StepKind::Clock: c.clock_slot = step->slot; break;
        case StepKind::Gripper: c.close = step->close; break;
        case StepKind::Wait: c.duration_s = step->duration_s; break;
      }
      command_pub->publish(c);
      if (!update) {command_at = now; dispatched = true; task_result.valid = false;}
      last_replan = now; result.reset();
    }
    // Correction parking: back out along the dock line (the face normal; the robot keeps facing the box) from the
    // robot's foot on it, as a new task: reverse_distance, or the longest clear one down to reverse_min_distance
    // (5 cm steps). The line, not the robot's own heading: the dock lane can be tight, and a robot a couple of cm
    // off the line already sits in a wall's padding (L1 west face, 2026-11-08). False when none is clear.
    bool send_reverse(double now) {
      const auto &b = *box_seen;
      const double normal = face_normal(b), heading = normal + M_PI, ux = std::cos(heading), uy = std::sin(heading);
      const double reach = b.side / 2 + b.standoff, gx = b.x + reach * std::cos(normal), gy = b.y + reach * std::sin(normal);
      const auto &r = odom.value.pose.pose.position;
      const double along = (gx - r.x) * ux + (gy - r.y) * uy;  // the robot's foot on the line, behind the goal
      geometry_msgs::msg::Pose o;
      o.position.x = gx - along * ux; o.position.y = gy - along * uy;
      o.orientation.z = std::sin(heading / 2); o.orientation.w = std::cos(heading / 2);
      const double yaw = heading;
      const auto points = obstacles(now);
      Path path;
      std::vector<double> speeds;
      bool clear = false;
      for (double d = config.box.reverse_distance; !clear && d >= config.box.reverse_min_distance - 1e-9; d -= 0.05) {
        const int n = std::max(1, static_cast<int>(std::ceil(d / 0.02)));
        path = Path(); path.header.frame_id = config.path.map_frame; path.header.stamp = node.now();
        speeds.clear();
        for (int i = 0; i <= n; ++i) {
          Pose p; p.header = path.header; p.pose.orientation = o.orientation;
          p.pose.position.x = o.position.x - std::cos(yaw) * d * i / n;
          p.pose.position.y = o.position.y - std::sin(yaw) * d * i / n;
          path.poses.push_back(p);
          const double left = d * (n - i) / n;  // slow down at a_lon_max to v_min at the end
          speeds.push_back(std::clamp(std::sqrt(2 * config.path.a_lon_max * left), config.path.v_min, config.box.reverse_speed));
        }
        clear = path_planner.collision_free(path, points, approach ? &*approach : nullptr, docking());
      }
      if (!clear) {box_blocked_by = blocker(path, now); return false;}
      box_phase = BoxPhase::Reverse;
      {
        const auto &a = path.poses.front().pose.position, &e = path.poses.back().pose.position;
        char t[140];
        std::snprintf(t, sizeof(t), "[box %s] 5/5 correction %d: back out %.2f m at %.2f m/s", step->box.c_str(),
                      box_corrections, std::hypot(e.x - a.x, e.y - a.y), config.box.reverse_speed);
        say(t);
      }
      send_box_move(now, path, speeds, 0, 0);
      return true;
    }
    void request_box_observation(const std::string &source, double now) {
      box_request = BoxObservationRequest{};
      lidar_samples.clear();
      box_request.header.frame_id = config.path.map_frame; box_request.header.stamp = node.now();
      box_request.request_id = ++next_id; box_request.source = source; box_request.box_id = step->box;
      const auto b = box_seen.value_or(box_target(step->box));
      box_request.expected_box.position.x = b.x; box_request.expected_box.position.y = b.y;
      box_request.expected_box.orientation.z = std::sin(b.yaw / 2);
      box_request.expected_box.orientation.w = std::cos(b.yaw / 2);
      box_request.front_face_x = config.box.front_face_x; box_request.box_side = config.box.side;
      box_request_at = now; box_last_observation_stamp = 0; box_observation.valid = false; box_had_observation = false;
      box_request_pub->publish(box_request);
    }
    void camera_stop(BoxPhase next, double now) {
      if (next == BoxPhase::HandoffStop) {
        handoff_lidar_captured = box_request.source == "lidar" && box_observation_fresh(now);
        if (handoff_lidar_captured) handoff_lidar_gap = box_observation.value.gap_m;
      }
      box_phase = next; camera_stop_at = now;
      handoff_ack_at = 0; handoff_ack_stamp = 0;
      TaskCommand c; c.header.stamp = node.now(); c.operation = TaskCommand::STOP;
      camera_stop_stamp = static_cast<int64_t>(c.header.stamp.sec) * 1000000000LL + c.header.stamp.nanosec;
      c.task_id = dispatched && step ? step->task_id : 0;
      command_pub->publish(c); hold(true);
    }
    bool camera_stopped(double now) const {
      const auto &c = control.value.header.stamp; const auto &r = task_result.value.header.stamp;
      const int64_t control_stamp = static_cast<int64_t>(c.sec) * 1000000000LL + c.nanosec;
      const int64_t result_stamp = static_cast<int64_t>(r.sec) * 1000000000LL + r.nanosec;
      if (step && step->box_near_entry && !dispatched) {
        // No task exists yet: ControlStatus reports fresh measured drive/arm stop,
        // whereas ControlMaster does not publish a TaskResult for task ID zero.
        return control_fresh(now) && control.value.ready && !control.value.fault &&
          control.value.held && control.value.stopped && control.at >= camera_stop_at &&
          control_stamp >= camera_stop_stamp;
      }
      return control_fresh(now) && control.value.held && control.value.stopped && control.at >= camera_stop_at &&
        control_stamp >= camera_stop_stamp && task_result.fresh(now, config.fsm.feedback_timeout_s) &&
        task_result.at >= camera_stop_at && result_stamp >= camera_stop_stamp &&
        step && task_result.value.task_id == step->task_id && task_result.value.status == TaskResult::STOPPED;
    }
    bool box_observation_fresh(double now) const {
      const double timeout = box_request.source == "lidar" ? config.box.lidar_observation_timeout :
        config.box.camera_timeout;
      return box_observation.fresh(now, timeout) && is_fresh_input(box_observation.value.header, timeout);
    }
    // Median of LiDAR observations: gap, yaw (about the first) and the box position; the position from only the
    // scans that saw both face ends when at least half did (face_centre_observed then), else from all.
    static BoxObservation median_observation(const std::vector<BoxObservation> &all) {
      const auto median = [](std::vector<double> v) {
        std::sort(v.begin(), v.end()); const std::size_t k = v.size() / 2;
        return v.size() % 2 ? v[k] : (v[k - 1] + v[k]) / 2;
      };
      const auto yaw_of = [](const BoxObservation &o) {
        const auto &q = o.box_pose.orientation;
        return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      };
      std::vector<const BoxObservation *> seen;
      for (const auto &o : all) if (o.face_centre_observed) seen.push_back(&o);
      const bool centre = 2 * seen.size() >= all.size();
      if (!centre) {seen.clear(); for (const auto &o : all) seen.push_back(&o);}
      std::vector<double> x, y, gap, yaw;
      const double yaw0 = yaw_of(all.front());
      for (const auto *o : seen) {x.push_back(o->box_pose.position.x); y.push_back(o->box_pose.position.y);}
      for (const auto &o : all) {gap.push_back(o.gap_m); yaw.push_back(yaw0 + std::remainder(yaw_of(o) - yaw0, 2 * M_PI));}
      BoxObservation m = all.back();
      m.box_pose.position.x = median(x); m.box_pose.position.y = median(y); m.gap_m = median(gap);
      const double w = median(yaw);
      m.box_pose.orientation.x = m.box_pose.orientation.y = 0;
      m.box_pose.orientation.z = std::sin(w / 2); m.box_pose.orientation.w = std::cos(w / 2);
      m.face_centre_observed = centre;
      return m;
    }
    BoxTarget observed_box(const BoxObservation &o, double gap) const {
      auto b = box_target(step->box);
      const auto &q = o.box_pose.orientation;
      const double seen_yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      if (!config.box.yaw_from_map) {
        b.x = o.box_pose.position.x; b.y = o.box_pose.position.y; b.yaw = seen_yaw;
      } else {
        // The boxes sit square to the map (planning.yaml): its yaw, to the nearest quarter turn of the one seen
        // (2026-11-10: the LiDAR yaw of one box still varied 175.6-186.4 deg over four docks, the dock line with
        // it). From the LiDAR only the distance along the approached face's normal; along the face too only when
        // both face ends were seen (face_centre_observed), else the map's (every dock so far: not seen).
        const double known = b.yaw;
        b.yaw = known + std::round(std::remainder(seen_yaw - known, 2 * M_PI) / (M_PI / 2)) * (M_PI / 2);
        const double n = face_normal(b), nx = std::cos(n), ny = std::sin(n);
        const double dx = o.box_pose.position.x - b.x, dy = o.box_pose.position.y - b.y;
        const double along_normal = dx * nx + dy * ny, along_face = -dx * ny + dy * nx;
        const double face = o.face_centre_observed ? along_face : 0.0;
        b.x += along_normal * nx - face * ny; b.y += along_normal * ny + face * nx;
      }
      b.standoff = config.box.front_face_x + gap;
      return b;
    }
    bool box_in_front_lidar_fov(double heading) const {
      const auto b = box_target(step->box);
      const auto &p = odom.value.pose.pose.position;
      const double c = std::cos(heading), s = std::sin(heading);
      const double lx = p.x + config.box.front_lidar_x * c;
      const double ly = p.y + config.box.front_lidar_x * s;
      const double cb = std::cos(b.yaw), sb = std::sin(b.yaw), half = b.side / 2;
      const double half_fov = config.box.front_lidar_fov_deg * M_PI / 360.0;
      for (int ix : {-1, 1}) for (int iy : {-1, 1}) {
        const double dx = b.x + half * (ix * cb - iy * sb) - lx;
        const double dy = b.y + half * (ix * sb + iy * cb) - ly;
        const double forward = c * dx + s * dy, lateral = -s * dx + c * dy;
        if (!std::isfinite(forward) || !std::isfinite(lateral) || forward <= 0 ||
            std::abs(std::atan2(lateral, forward)) > half_fov) return false;
      }
      return true;
    }
    // Odometry travel along heading [rad, map] from stamp t [s] to the newest odometry; the pose at t is interpolated
    // between the odometry samples around it. Empty when t is older than odom_trail (or there is none).
    std::optional<double> travel_since(double t, double heading) const {
      if (odom_trail.empty() || !std::isfinite(t) || t < odom_trail.front()[0]) return std::nullopt;
      const auto &last = odom_trail.back();
      double x = last[1], y = last[2];
      for (std::size_t i = 1; i < odom_trail.size(); ++i) {
        const auto &a = odom_trail[i - 1], &b = odom_trail[i];
        if (t <= b[0]) {
          const double k = (t - a[0]) / (b[0] - a[0]);
          x = a[1] + k * (b[1] - a[1]); y = a[2] + k * (b[2] - a[2]);
          break;
        }
      }
      return (last[1] - x) * std::cos(heading) + (last[2] - y) * std::sin(heading);
    }
    bool camera_path_alignment(double now, bool final = false) const {
      if (!box_seen || !box_face || !odom.fresh(now, config.odom_timeout_s) ||
          !is_fresh_input(odom.value.header, config.odom_timeout_s)) {
        last_alignment_detail = "box_seen/box_face missing or odom stale";
        return false;
      }
      const auto &p = odom.value.pose.pose.position;
      const auto &q = odom.value.pose.pose.orientation;
      const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (!std::isfinite(norm) || norm < 1e-12) {
        last_alignment_detail = "invalid odom quaternion";
        return false;
      }
      const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y)/norm, 1-2*(q.y*q.y + q.z*q.z)/norm);
      const double normal = face_normal(*box_seen), heading = normal + M_PI;
      const double dx = p.x - box_seen->x, dy = p.y - box_seen->y;
      const double lateral = -std::sin(heading)*dx + std::cos(heading)*dy;
      const double fdx = dx + config.box.front_face_x * std::cos(yaw);
      const double fdy = dy + config.box.front_face_x * std::sin(yaw);
      const double front_lateral = -std::sin(heading)*fdx + std::cos(heading)*fdy;
      const double yaw_err = std::remainder(yaw - heading, 2*M_PI);
      char buf[240];
      std::snprintf(buf, sizeof(buf), "base_lat=%+.1fmm front_lat=%+.1fmm (tol %.1fmm) yaw_err=%+.2fdeg (tol %.2fdeg) final=%d",
                    lateral*1000.0, front_lateral*1000.0, config.box.lateral_tolerance*1000.0,
                    yaw_err*180.0/M_PI, (final ? config.box.final_yaw_tolerance : config.box.yaw_tolerance)*180.0/M_PI,
                    final ? 1 : 0);
      last_alignment_detail = buf;
      const double eff_lat = std::min(std::abs(lateral), std::abs(front_lateral));
      return std::isfinite(lateral) && std::isfinite(front_lateral) && std::isfinite(yaw) &&
        eff_lat <= config.box.lateral_tolerance &&
        (!final || std::abs(yaw_err) <= config.box.final_yaw_tolerance);
    }
    void camera_box_tick(double now, FsmInput &input) {
      input.completed = false;
      auto fail = [this, &input](const std::string &reason) {
        input.healthy = false;
        input.error = reason;
        RCLCPP_ERROR(node.get_logger(),
                     "[BOX FAULT DETAIL] reason='%s' phase=%d box=%s blocked_by='%s' align='%s' obs_reject='%s' obs_gap=%.4f odom=(%.3f,%.3f)",
                     reason.c_str(), static_cast<int>(box_phase), step ? step->box.c_str() : "",
                     box_blocked_by.c_str(), last_alignment_detail.c_str(), last_obs_reject_reason.c_str(),
                     box_observation.value.gap_m, odom.value.pose.pose.position.x, odom.value.pose.pose.position.y);
      };
      const auto phase = box_phase;
      const bool stopping = phase == BoxPhase::CameraStageStop || phase == BoxPhase::HandoffStop || phase == BoxPhase::FinalStop;
      const bool stationary = stopping || phase == BoxPhase::LidarPerceive || phase == BoxPhase::CameraPerceive || phase == BoxPhase::CameraVerify;
      if (stationary) hold(true);
      if (phase == BoxPhase::Stage) {
        // Planning again for a turned box: control keeps repeating the stopped stage task's STOPPED until the new
        // stage drive goes out (next_segment), which is not that drive's result.
        if (next_segment) return;
        // Stuck circling just off the stage point (2026-11-10 noboxes_1505/1559: 4-14 cm off its 3 cm arrival,
        // control kept turning at full rate toward the path's target beside it, 90 s): within stage_capture_m for
        // stage_capture_s without arriving, turn in place to the stage heading instead; the dock takes the offset.
        // Only this stage drive's own result counts (a replaced or stopped one keeps being repeated by control).
        const bool own = task_result.valid && task_result.value.task_id == step->task_id;
        // Within stage_final_m of the stage point: the same path with the short target (and on every renewal).
        if (!stage_final && !stage_turned && !more_segments && own && task_result.value.status == TaskResult::RUNNING &&
            !active_path.poses.empty() && odom.fresh(now, config.odom_timeout_s)) {
          const auto &e = active_path.poses.back().pose.position, &o = odom.value.pose.pose.position;
          if (std::hypot(e.x - o.x, e.y - o.y) <= config.box.stage_final_m) {
            stage_final = true;
            active_tracking_reference_x = 0.0; active_lookahead_distance = config.box.stage_final_lookahead;
            TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id; c.revision = ++revision;
            c.operation = TaskCommand::UPDATE_PATH; c.kind = TaskCommand::NAVIGATE;
            c.path = active_path; c.path.header.stamp = node.now(); c.speed = route_paused ? 0.0 : drive_speed();
            c.speed_limit = active_speed; c.require_goal_heading = heading_required();
            c.goal_tolerance = active_goal_tolerance; c.tolerance_rad = active_heading_tolerance;
            c.tracking_reference_x = active_tracking_reference_x; c.lookahead_distance = active_lookahead_distance;
            camera_fields(c); command_pub->publish(c); last_replan = now;
          }
        }
        if (!stage_turned && box_face && !more_segments && own &&
            task_result.value.status == TaskResult::RUNNING && !active_path.poses.empty() && odom.fresh(now, config.odom_timeout_s)) {
          const auto &e = active_path.poses.back().pose.position, &o = odom.value.pose.pose.position;
          const double d = std::hypot(e.x - o.x, e.y - o.y);
          // Stuck = no centimetre closer for stage_capture_s (a slow but steady arrival keeps closing in).
          // Arrived (within the arrival tolerance), or stuck (no centimetre closer for stage_capture_s): turn in place
          // there with the arrival latched (user 2026-11-10: the heading turn at the stage point slid 10 cm off it,
          // left the 3 cm arrival and the robot circled again).
          const bool arrived = d <= config.box.stage_goal_tolerance;
          if (!arrived && d > config.box.stage_capture_m) {stage_near_since.reset();}
          else if (!arrived && (!stage_near_since || d < stage_near_best - 0.01)) {stage_near_since = now; stage_near_best = d;}
          else if (arrived || now - *stage_near_since >= config.box.stage_capture_s) {
            if (!stage_near_since) stage_near_since = now;
            const double heading = *box_face + M_PI;
            Pose a; a.header.frame_id = config.path.map_frame; a.header.stamp = node.now();
            a.pose.position = o; a.pose.orientation.z = std::sin(heading / 2); a.pose.orientation.w = std::cos(heading / 2);
            Pose b = a; b.pose.position.x += 0.001 * std::cos(heading); b.pose.position.y += 0.001 * std::sin(heading);
            Path turn; turn.header = a.header; turn.poses = {a, b};
            char text[160];
            std::snprintf(text, sizeof(text), arrived ? "[box %s] 1/5 at the stage point (%.0f mm, %.1f s): turning in place to %.0f deg" :
                          "[box %s] 1/5 %.0f mm off the stage point, no closer for %.1f s: turning in place to %.0f deg",
                          step->box.c_str(), std::hypot(e.x - o.x, e.y - o.y) * 1000, now - *stage_near_since,
                          std::remainder(heading, 2 * M_PI) * 180 / M_PI);
            say(text);
            // The running stage task's path becomes this point (control takes no new task while one runs): it is
            // at its goal at once and turns to the heading, then SUCCEEDED as the stage drive. No local renewal
            // may put the old route back.
            stage_turned = true;
            active_path = turn; active_speed = {config.box.speed, config.box.speed};
            global_path.poses.clear(); more_segments = false;
            TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id; c.revision = ++revision;
            c.operation = TaskCommand::UPDATE_PATH; c.kind = TaskCommand::NAVIGATE;
            c.path = active_path; c.speed = drive_speed(); c.speed_limit = active_speed; c.require_goal_heading = true;
            // stage_capture_m as the arrival distance: sliding a little while turning does not undo the arrival.
            active_goal_tolerance = config.box.stage_capture_m; active_tracking_reference_x = 0; active_lookahead_distance = 0;
            c.goal_tolerance = active_goal_tolerance; c.tolerance_rad = active_heading_tolerance;
            c.tracking_reference_x = active_tracking_reference_x; c.lookahead_distance = active_lookahead_distance;
            camera_fields(c); command_pub->publish(c); route_paused = false; last_replan = now;
            return;
          }
        }
        // After the turn in place: within yaw_tolerance and standing still for stage_still_s, go on without
        // control's SUCCEEDED (2026-11-10: the turn ended 0.87 deg off, within 1 deg, and the task stayed RUNNING).
        if (stage_turned && own && task_result.value.status == TaskResult::RUNNING && box_face &&
            odom.fresh(now, config.odom_timeout_s) && wheel_odom.fresh(now, config.odom_timeout_s)) {
          const auto &q = odom.value.pose.pose.orientation;
          const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
          const double err = std::abs(std::remainder(yaw - (*box_face + M_PI), 2 * M_PI));
          const auto &tw = wheel_odom.value.twist.twist;
          if (err <= config.box.yaw_tolerance && std::abs(tw.linear.x) < 0.005 && std::abs(tw.angular.z) < 0.01) {
            if (!stage_still_since) stage_still_since = now;
            else if (now - *stage_still_since >= config.box.stage_still_s) {
              char text[160];
              std::snprintf(text, sizeof(text), "[box %s] 1/5 turned to the stage heading (%.2f deg off) and still for %.1f s; "
                            "control has not reported it done: going on", step->box.c_str(), err * 180 / M_PI,
                            now - *stage_still_since);
              say(text);
              if (!box_in_front_lidar_fov(yaw)) {fail("box corners outside front lidar field of view"); return;}
              camera_stop(BoxPhase::CameraStageStop, now); return;
            }
          } else {stage_still_since.reset();}
        }
        if (own && task_result.value.status == TaskResult::SUCCEEDED && more_segments) {
          input.completed = true; return;  // existing segmentation logic continues the stage path
        }
        if (own && task_result.value.status == TaskResult::SUCCEEDED) {
          if (!box_face || !odom.fresh(now, config.odom_timeout_s) ||
              !is_fresh_input(odom.value.header, config.odom_timeout_s)) {
            fail("box stage heading unavailable"); return;
          }
          const auto &q = odom.value.pose.pose.orientation;
          const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
          const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y) / norm,
                                        1 - 2 * (q.y * q.y + q.z * q.z) / norm);
          const double error = std::abs(std::remainder(yaw - (*box_face + M_PI), 2 * M_PI));
          if (!std::isfinite(error) || error > config.box.yaw_tolerance) {
            fail("box stage heading out of tolerance"); return;
          }
          if (!box_in_front_lidar_fov(yaw)) {fail("box corners outside front lidar field of view"); return;}
          camera_stop(BoxPhase::CameraStageStop, now);
        }
        else if (own && (task_result.value.status == TaskResult::FAILED || task_result.value.status == TaskResult::REJECTED || task_result.value.status == TaskResult::STOPPED)) fail("stage task failed");
        return;
      }
      if (stopping) {
        if (phase != BoxPhase::CameraStageStop && phase != BoxPhase::HandoffStop && box_had_observation && !box_observation_fresh(now) &&
            !(config.box.camera_only_longitudinal && phase == BoxPhase::FinalStop)) {  // trigger mode: CameraVerify re-requests
          fail("box observation stale while stopping"); return;
        }
        if (phase == BoxPhase::HandoffStop &&
            now - camera_stop_at > config.fsm.stop_timeout_s + config.box.settle_s) {
          fail("camera handoff stop/settle timeout"); return;
        }
        if (!camera_stopped(now)) {
          if (now - camera_stop_at > config.fsm.stop_timeout_s) fail("box stop acknowledgement timeout");
          return;
        }
        if (phase == BoxPhase::CameraStageStop) {
          box_phase = BoxPhase::LidarPerceive; request_box_observation("lidar", now);
        } else if (phase == BoxPhase::HandoffStop) {
          if (handoff_ack_at == 0) {
            handoff_ack_at = now;
            handoff_ack_stamp = node.now().nanoseconds();
          }
          if (now - handoff_ack_at < config.box.settle_s) return;
          if (!config.box.camera_only_longitudinal && !handoff_lidar_captured) {fail("valid lidar missing at handoff stop"); return;}
          // LiDAR can lose geometry at its near-range limit. Keep the last reliable
          // threshold measurement; prefer a real post-settle sample when available.
          if (box_observation_fresh(now)) {
            const auto &s = box_observation.value.header.stamp;
            const int64_t observation_stamp = static_cast<int64_t>(s.sec) * 1000000000LL + s.nanosec;
            if (observation_stamp >= handoff_ack_stamp + static_cast<int64_t>(std::llround(config.box.settle_s * 1000000000.0)))
              handoff_lidar_gap = box_observation.value.gap_m;
          }
          if (!config.box.camera_only_longitudinal && step->box_near_entry &&
              (handoff_lidar_gap <= config.box.camera_final_gap + config.box.camera_gap_tolerance ||
               handoff_lidar_gap > config.box.handoff_gap + config.box.lidar_uncertainty_max + 1e-12)) {
            fail("near entry lidar gap outside allowed range"); return;
          }
          if (!box_seen) {fail("lidar approach line missing at camera handoff"); return;}
          box_seen->standoff = config.box.front_face_x + config.box.camera_final_gap;
          box_phase = BoxPhase::CameraPerceive; request_box_observation("uppercam", now);
        } else {
          box_phase = BoxPhase::CameraVerify; camera_confirmed = 0; camera_last_confirm_stamp = 0;
          request_box_observation("uppercam", now);
        }
        return;
      }
      if (phase == BoxPhase::CameraPerceive && !camera_stopped(now)) {
        fail("camera initial held/stopped acknowledgement lost"); return;
      }
      if (config.box.camera_only_longitudinal && phase == BoxPhase::CameraDock) {
        // Plain LiDAR-line tracking at camera_speed; the upper cam is only the stop trigger at camera_final_gap
        // (user, 2026-11-09): no camera speed control, invalid frames (box out of its range) are ignored.
        if (box_observation_fresh(now)) {
          const auto &o = box_observation.value;
          const double taken = o.header.stamp.sec + o.header.stamp.nanosec * 1e-9;
          const double age = std::max(0.0, node.now().seconds() - taken);
          // Travel since the frame was taken, measured by the odometry (2026-11-10; was camera_speed x age, wrong
          // whenever the dock was not at exactly camera_speed). No odometry that old: the old estimate.
          const auto moved = travel_since(taken, face_normal(*box_seen) + M_PI);
          const double travel = moved ? *moved : config.box.camera_speed * age;
          const double gap_now = o.gap_m - travel;
          if (gap_now <= config.box.camera_final_gap + config.box.camera_gap_tolerance) {
            char text[200];
            std::snprintf(text, sizeof(text), "[box %s] 4/5 upper cam %.1f mm (frame %.0f ms old, moved %.1f mm %s, "
                          "now ~%.1f mm): stop", step->box.c_str(), o.gap_m * 1000, age * 1000, travel * 1000,
                          moved ? "by odom" : "assumed", gap_now * 1000);
            say(text); camera_stop(BoxPhase::FinalStop, now); return;
          }
        }
        if (approach && odom.fresh(now, config.odom_timeout_s)) {
          // The dock path runs on into the box: dock_overrun past the face goal by odometry stops it anyway.
          const double heading = face_normal(*box_seen) + M_PI;
          const auto &p = odom.value.pose.pose.position, &g = approach->goal.position;
          const double past = (p.x - g.x) * std::cos(heading) + (p.y - g.y) * std::sin(heading);
          if (past >= config.box.dock_overrun) {
            char text[160];
            std::snprintf(text, sizeof(text), "[box %s] 4/5 %.1f mm past the face goal by odometry before the upper cam "
                          "trigger: stop, the upper cam checks", step->box.c_str(), past * 1000);
            say(text); camera_stop(BoxPhase::FinalStop, now); return;
          }
        }
        if (task_result.valid && task_result.value.status == TaskResult::SUCCEEDED) {
          say("[box " + step->box + "] 4/5 LiDAR line end before the upper cam trigger: stop, the upper cam checks");
          camera_stop(BoxPhase::FinalStop, now); return;
        }
        if (task_result.valid && task_result.value.status != TaskResult::RUNNING) fail("dock ended before the upper cam stop trigger");
        return;
      }
      if (!box_observation_fresh(now)) {
        if (box_had_observation || phase == BoxPhase::LidarDock || phase == BoxPhase::CameraDock ||
            phase == BoxPhase::LidarRelease || phase == BoxPhase::CameraRelease ||
            now - box_request_at > config.box.observation_initial_timeout)
          fail("box observation invalid or stale");
        return;
      }
      const auto &o = box_observation.value;
      if ((phase == BoxPhase::CameraPerceive || phase == BoxPhase::CameraRelease || phase == BoxPhase::CameraDock) &&
          !camera_path_alignment(now)) {
        fail("lidar path alignment outside tolerance"); return;
      }
      if (phase == BoxPhase::LidarPerceive) {
        if (now - box_request_at < config.box.settle_s) return;
        if (step->box_near_entry) {
          const auto &requested = box_request.header.stamp, &observed = o.header.stamp;
          const int64_t request_ns = static_cast<int64_t>(requested.sec)*1000000000LL + requested.nanosec;
          const int64_t observed_ns = static_cast<int64_t>(observed.sec)*1000000000LL + observed.nanosec;
          if (observed_ns < request_ns + static_cast<int64_t>(std::llround(config.box.settle_s*1000000000.0))) return;
          if (!config.box.camera_only_longitudinal &&
              (o.gap_m <= config.box.camera_final_gap + config.box.camera_gap_tolerance ||
               o.gap_m > config.box.handoff_gap + config.box.lidar_uncertainty_max + 1e-12)) {
            fail("near entry lidar gap outside allowed range"); return;
          }
          box_seen = observed_box(o, config.box.camera_final_gap);
          camera_stop(BoxPhase::HandoffStop, now); return;
        }
        if (config.box.camera_only_longitudinal) {
          // The LiDAR box pose only makes the dock line, to camera_final_gap; its gap decides nothing (user, 2026-11-09).
          // The median of the scans over lidar_window_s (2026-11-10: one scan's box centre varies ~4 mm, and the
          // face ends are found in about three scans of four).
          if (now - box_request_at < config.box.settle_s + config.box.lidar_window_s ||
              lidar_samples.size() < 3) return;
          const auto m = median_observation(lidar_samples);
          {
            const auto &q = m.box_pose.orientation;
            char text[220];
            std::snprintf(text, sizeof(text), "[box %s] 2/5 LiDAR, median of %zu scans: box %s, face normal %.1f deg, "
                          "centre along the face %s", step->box.c_str(), lidar_samples.size(),
                          xy(m.box_pose.position.x, m.box_pose.position.y).c_str(),
                          std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z) * 180 / M_PI,
                          m.face_centre_observed ? "measured (both face ends seen)" : "from the expected box (ends not seen)");
            say(text);
          }
          const auto seen = observed_box(m, config.box.camera_final_gap);
          const double turned = std::abs(std::remainder(face_normal(seen) - *box_face, 2 * M_PI));
          if (turned > config.box.reorient_deg * M_PI / 180 && !box_reoriented) {
            // The box stands turned from where this stage point faces (2026-11-10: a box turned 45 deg): plan again
            // from the box as measured, its allowed faces, the shortest; once per box step.
            char text[200];
            std::snprintf(text, sizeof(text), "[box %s] 2/5 the box is turned %.1f deg from the face approached: "
                          "planning again to the stage point of its measured faces", step->box.c_str(), turned * 180 / M_PI);
            say(text);
            box_reoriented = true; box_seen = seen; box_face.reset(); box_phase = BoxPhase::Stage; hold(false); stage_turned = false; stage_near_since.reset(); stage_final = false; stage_still_since.reset();
            box_seen->standoff = box_target(step->box).standoff;  // the stage point as far out as the first one
            task_result.valid = false;  // the stopped stage task's STOPPED is not this new stage drive's result
            next_segment = true; result.reset(); last_replan = now - config.replan_period_s;
            return;
          }
          box_seen = seen; box_phase = BoxPhase::LidarRelease; camera_stop_at = now;
          return;
        }
        if (o.gap_m <= config.box.handoff_gap) {fail("lidar handoff gap already reached"); return;}
        box_seen = observed_box(o, config.box.handoff_gap); box_phase = BoxPhase::LidarRelease; camera_stop_at = now;
      } else if (phase == BoxPhase::LidarRelease) {
        if (control_fresh(now) && !control.value.held && control.at >= camera_stop_at) {
          if (config.box.camera_only_longitudinal) {
            box_phase = BoxPhase::CameraDock; camera_engaged = false; request_box_observation("uppercam", now);
            if (!send_dock(now)) fail("dock path blocked");
            return;
          }
          box_phase = BoxPhase::LidarDock;
          if (!send_dock(now)) fail("lidar dock path blocked");
        } else if (now - camera_stop_at > config.fsm.ready_timeout_s) fail("lidar dock release timeout");
      } else if (phase == BoxPhase::LidarDock) {
        const bool reached_within_lidar_uncertainty = task_result.valid &&
          task_result.value.status == TaskResult::SUCCEEDED &&
          o.gap_m <= config.box.handoff_gap + config.box.lidar_uncertainty_max + 1e-12;
        if (o.gap_m <= config.box.handoff_gap || reached_within_lidar_uncertainty)
          camera_stop(BoxPhase::HandoffStop, now);
        else if (task_result.valid && task_result.value.status != TaskResult::RUNNING) fail("lidar dock ended before handoff");
      } else if (phase == BoxPhase::CameraPerceive) {
        { char text[200]; std::snprintf(text, sizeof(text), "camera handoff held: lidar %.1f mm, camera %.1f mm, difference %.1f mm",
             handoff_lidar_gap*1000, o.gap_m*1000, std::abs(o.gap_m-handoff_lidar_gap)*1000); say(text); }
        if (o.gap_m <= config.box.camera_final_gap + config.box.camera_gap_tolerance) {fail("camera gap too close at handoff"); return;}
        box_phase = BoxPhase::CameraRelease; camera_stop_at = now;
      } else if (phase == BoxPhase::CameraRelease) {
        if (control_fresh(now) && !control.value.held && control.at >= camera_stop_at) {
          box_phase = BoxPhase::CameraDock;
          // Trigger mode (the near entry gets here too): the upper cam only stops the dock, as after LidarRelease;
          // engaged, the dock went out at camera_fast_speed with camera distance control, which control rejects.
          if (config.box.camera_only_longitudinal) camera_engaged = false;
          if (!send_dock(now)) fail("camera dock path blocked");
        } else if (now - camera_stop_at > config.fsm.ready_timeout_s) fail("camera dock release timeout");
      } else if (phase == BoxPhase::CameraDock) {
        if (o.gap_m <= config.box.camera_final_gap + config.box.camera_gap_tolerance)
          camera_stop(BoxPhase::FinalStop, now);
        else if (task_result.valid && task_result.value.status != TaskResult::RUNNING) fail("camera dock ended before target gap");
        else send_camera_gap_update(now, o);
      } else if (phase == BoxPhase::CameraVerify) {
        if (now - box_request_at < config.box.settle_s) return;
        const auto stamp = static_cast<int64_t>(o.header.stamp.sec) * 1000000000LL + o.header.stamp.nanosec;
        const auto &requested = box_request.header.stamp;
        const int64_t settle_stamp = static_cast<int64_t>(requested.sec) * 1000000000LL + requested.nanosec +
          static_cast<int64_t>(std::llround(config.box.settle_s * 1000000000.0));
        if (stamp < settle_stamp || stamp == camera_last_confirm_stamp) return;
        camera_last_confirm_stamp = stamp;
        // The yaw too, within final_yaw_tolerance_deg (user 2026-11-10; it was logged only, 1.8 deg passed).
        if (std::abs(o.gap_m - config.box.camera_final_gap) > config.box.camera_gap_tolerance ||
            !camera_path_alignment(now, true)) {
          fail("camera final gap or alignment out of tolerance (" + last_alignment_detail + ")"); return;
        }
        if (++camera_confirmed >= config.box.camera_confirm_count) {
          request_box_observation("off", now); input.completed = true;
        }
      }
    }
    // The dock: the face normal itself, from the point on it nearest the robot to the face goal of the box as
    // perceived, at dock_speed; control turns toward its lookahead point first (yaw_tolerance), pulls the robot onto
    // the line (the target stays lookahead ahead past the goal) and arrives within dock_goal_tolerance along it.
    // Trigger mode (camera_only_longitudinal): a curve onto the normal, through the goal into the box (the upper
    // cam, or dock_overrun by odometry, stops it). A new task. False when the line is blocked.
    bool send_dock(double now) {
      const auto &b = *box_seen;
      const double normal = face_normal(b);
      const double heading = normal + M_PI, ux = std::cos(heading), uy = std::sin(heading), reach = b.side / 2 + b.standoff;
      const double gx = b.x + reach * std::cos(normal), gy = b.y + reach * std::sin(normal);
      const auto &o = odom.value.pose.pose.position;
      const double along = std::max((gx - o.x) * ux + (gy - o.y) * uy, 0.02);  // robot -> goal along the normal
      Pose goal; goal.header.frame_id = config.path.map_frame; goal.header.stamp = node.now();
      goal.pose.position.x = gx; goal.pose.position.y = gy;
      goal.pose.orientation.z = std::sin(heading / 2); goal.pose.orientation.w = std::cos(heading / 2);
      Path path; path.header = goal.header;
      const auto add = [&path, &goal](double x, double y, double yaw) {
        Pose p = goal; p.pose.position.x = x; p.pose.position.y = y;
        p.pose.orientation.z = std::sin(yaw / 2); p.pose.orientation.w = std::cos(yaw / 2);
        path.poses.push_back(p);
      };
      const bool trigger = config.box.camera_only_longitudinal && box_in(BoxPhase::CameraDock);
      std::size_t goal_index = 0;  // the sample at the face goal
      dock_anchor.reset();
      if (trigger && config.box.dock_wheel_odometry && wheel_odom.fresh(now, config.odom_timeout_s)) {
        const auto &m = odom.value.pose.pose, &w = wheel_odom.value.pose.pose;
        dock_anchor = DockAnchor{m.position.x, m.position.y, 2 * std::atan2(m.orientation.z, m.orientation.w),
                                 w.position.x, w.position.y, 2 * std::atan2(w.orientation.z, w.orientation.w)};
      }
      if (trigger) {
        // From the robot's pose a cubic Hermite curve onto the normal dock_straight before the goal (tangents along
        // the robot's heading and the normal, chord long), the normal through the goal, dock_extend past it. Closer
        // than dock_straight already: the normal from the robot's projection on it.
        const auto &q = odom.value.pose.pose.orientation;
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const double straight = std::min(config.box.dock_straight, along);
        const double mx = gx - straight * ux, my = gy - straight * uy;
        const double chord = std::hypot(mx - o.x, my - o.y);
        if (along > config.box.dock_straight + 0.02 && chord > 0.02) {
          const int n = std::max(2, static_cast<int>(std::ceil(chord / 0.01)));
          const double t0x = chord * std::cos(yaw), t0y = chord * std::sin(yaw), t1x = chord * ux, t1y = chord * uy;
          for (int i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) / n, t2 = t * t, t3 = t2 * t;
            const double h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t, h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
            const double d00 = 6 * t2 - 6 * t, d10 = 3 * t2 - 4 * t + 1, d01 = -6 * t2 + 6 * t, d11 = 3 * t2 - 2 * t;
            add(h00 * o.x + h10 * t0x + h01 * mx + h11 * t1x, h00 * o.y + h10 * t0y + h01 * my + h11 * t1y,
                std::atan2(d00 * o.y + d10 * t0y + d01 * my + d11 * t1y, d00 * o.x + d10 * t0x + d01 * mx + d11 * t1x));
          }
        }
        // Along the normal from -straight (or the robot's projection) through the goal (s = 0) to dock_extend.
        const double from = path.poses.empty() ? -std::max(along, 0.02) : -straight;
        const int before = std::max(1, static_cast<int>(std::ceil(-from / 0.01)));
        const int after = std::max(1, static_cast<int>(std::ceil(config.box.dock_extend / 0.01)));
        for (int i = 0; i < before; ++i) add(gx + from * (1.0 - static_cast<double>(i) / before) * ux,
                                             gy + from * (1.0 - static_cast<double>(i) / before) * uy, heading);
        goal_index = path.poses.size();
        for (int i = 0; i <= after; ++i) {
          const double s = config.box.dock_extend * i / after;
          add(gx + s * ux, gy + s * uy, heading);
        }
      } else {
        std::vector<std::pair<double, double>> corners{{gx - along * ux, gy - along * uy}, {gx, gy}};
        for (std::size_t k = 1; k < corners.size(); ++k) {
          const auto [ax, ay] = corners[k - 1]; const auto [bx, by] = corners[k];
          const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(bx - ax, by - ay) / 0.02)));
          for (int i = (k == 1 ? 0 : 1); i <= n; ++i) add(ax + (bx - ax) * i / n, ay + (by - ay) * i / n, heading);
        }
        goal_index = path.poses.size() - 1;
      }
      // Arc length from each sample to the face goal (negative past it): the speed limits.
      std::vector<double> arc(path.poses.size(), 0.0), speeds(path.poses.size());
      for (std::size_t i = 1; i < path.poses.size(); ++i) {
        const auto &a = path.poses[i - 1].pose.position, &c = path.poses[i].pose.position;
        arc[i] = arc[i - 1] + std::hypot(c.x - a.x, c.y - a.y);
      }
      for (std::size_t i = 0; i < path.poses.size(); ++i) {
        const double left = std::max(arc[goal_index] - arc[i], 0.0);
        const double limit = box_in(BoxPhase::CameraDock) ? config.box.camera_speed : config.box.dock_speed;
        speeds[i] = std::clamp(std::sqrt(2 * config.path.a_lon_max * left), std::min(config.path.v_min, limit), limit);
        if (trigger) {
          // Fast until camera_slow_gap before the face (the goal is at camera_final_gap), down to camera_speed at
          // a_lon_max, camera_speed from there on and past the goal.
          const double slow_zone = config.box.camera_slow_gap - config.box.camera_final_gap;
          const double v = config.box.camera_speed;
          speeds[i] = left <= slow_zone ? v : std::min(config.box.camera_fast_speed,
            std::sqrt(v * v + 2 * config.path.a_lon_max * (left - slow_zone)));
        }
      }
      // The collision check stops at the goal: the extension runs into the box on purpose.
      Path checked = path; checked.poses.resize(goal_index + 1);
      const BoxApproach dock{b, goal.pose};
      if (!path_planner.collision_free(checked, obstacles(now), &dock, true)) {
        box_blocked_by = blocker(path, now);
        RCLCPP_ERROR(node.get_logger(), "[DOCK PATH BLOCKED] box=%s phase=%d goal=(%.3f,%.3f) box_seen=(%.3f,%.3f) blocked_by=%s",
                     step ? step->box.c_str() : "", static_cast<int>(box_phase), gx, gy, b.x, b.y, box_blocked_by.c_str());
        return false;
      }
      approach = dock;
      if (!config.box.camera_enabled) box_phase = BoxPhase::Dock;
      {
        const auto &a = path.poses.front().pose.position;
        char t[240];
        if (config.box.camera_only_longitudinal && box_in(BoxPhase::CameraDock))
          std::snprintf(t, sizeof(t), "[box %s] 3/5 dock: path %.2f m onto the face normal to %s (+%.2f m into the box), "
                        "%.2f m/s, %.2f m/s from %.0f mm; the upper cam stops it", step->box.c_str(),
                        std::hypot(gx - a.x, gy - a.y), xy(gx, gy).c_str(), config.box.dock_extend,
                        config.box.camera_fast_speed, config.box.camera_speed, config.box.camera_slow_gap * 1000);
        else
        std::snprintf(t, sizeof(t), "[box %s] 3/5 dock: turn square, then straight %.2f m at %.2f m/s to %s",
                      step->box.c_str(), std::hypot(gx - a.x, gy - a.y), config.box.dock_speed, xy(gx, gy).c_str());
        say(t);
      }
      send_box_move(now, path, speeds, config.box.camera_enabled ? 0.001 : config.box.dock_goal_tolerance,
                    box_in(BoxPhase::CameraDock) ? 0.0 : config.box.yaw_tolerance, true);
      box_goal_pub->publish(goal);
      return true;
    }
    void send_box_move(double now, const Path &path, const std::vector<double> &speeds, double goal_tolerance,
                       double heading_tolerance, bool dock_tracking = false) {
      TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id = ++next_id; c.revision = ++revision;
      c.operation = TaskCommand::EXECUTE; c.kind = TaskCommand::NAVIGATE;
      c.path = path; c.speed = drive_speed(); c.speed_limit = speeds; c.require_goal_heading = heading_required();
      c.goal_tolerance = goal_tolerance; c.tolerance_rad = heading_tolerance;
      c.tracking_reference_x = dock_tracking ? config.box.dock_reference_x : 0.0;
      c.lookahead_distance = dock_tracking ? config.box.dock_lookahead : 0.0;
      c.camera_distance_control = dock_tracking && box_in(BoxPhase::CameraDock) && camera_engaged;
      if (c.camera_distance_control) {
        c.camera_gap_m = box_observation.value.gap_m;
        c.camera_target_gap_m = config.box.camera_final_gap;
        c.camera_observation_stamp = box_observation.value.header.stamp;
      }
      command_pub->publish(c);
      command_at = now; dispatched = true; task_result.valid = false; last_replan = now; result.reset();
      active_path = path; active_speed = speeds; active_goal_tolerance = goal_tolerance;
      active_tracking_reference_x = c.tracking_reference_x; active_lookahead_distance = c.lookahead_distance;
      active_camera_distance_control = c.camera_distance_control;
      active_camera_gap = c.camera_gap_m; active_camera_stamp = c.camera_observation_stamp;
      active_camera_stamp_ns = static_cast<int64_t>(active_camera_stamp.sec) * 1000000000LL + active_camera_stamp.nanosec;
      active_heading_tolerance = heading_tolerance; global_path.poses.clear(); more_segments = false;
      route_paused = false; route_failed = false; blocked_since.reset(); segment_start = odom.value.pose.pose.position;
    }
    void camera_fields(TaskCommand &c) const {
      c.camera_distance_control = active_camera_distance_control;
      if (active_camera_distance_control) {
        c.camera_gap_m = active_camera_gap; c.camera_target_gap_m = config.box.camera_final_gap;
        c.camera_observation_stamp = active_camera_stamp;
      }
    }
    void send_camera_gap_update(double now, const BoxObservation &o) {
      const int64_t stamp = static_cast<int64_t>(o.header.stamp.sec) * 1000000000LL + o.header.stamp.nanosec;
      if (!active_camera_distance_control || stamp <= active_camera_stamp_ns) return;
      active_camera_gap = o.gap_m; active_camera_stamp = o.header.stamp; active_camera_stamp_ns = stamp;
      TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id; c.revision = ++revision;
      c.operation = TaskCommand::UPDATE_PATH; c.kind = TaskCommand::NAVIGATE;
      c.path = active_path; c.path.header.stamp = node.now(); c.speed = route_paused ? 0.0 : drive_speed();
      c.speed_limit = active_speed; c.require_goal_heading = heading_required();
      c.goal_tolerance = active_goal_tolerance; c.tolerance_rad = active_heading_tolerance;
      c.tracking_reference_x = active_tracking_reference_x; c.lookahead_distance = active_lookahead_distance;
      camera_fields(c); command_pub->publish(c);
    }
    // After settling at the face goal: the robot against the goal of that face of the box as perceived now.
    // Returns the error text when the step must fail, nullopt otherwise (done: aligned; or a correction started).
    std::optional<std::string> check_alignment(double now, bool &done) {
      done = false;
      const auto b = detected_box(now);
      if (!b || box_detection.at < box_settle_at + config.box.settle_s) {
        if (now - box_settle_at > config.box.settle_s + config.box_detection_timeout_s)
          return "box " + step->box + " not perceived after settling";
        return std::nullopt;  // wait for a detection taken standing still
      }
      const double normal = face_normal(*b);
      const double reach = b->side / 2 + b->standoff, ux = -std::cos(normal), uy = -std::sin(normal);
      const auto &o = odom.value.pose.pose;
      const double dx = o.position.x - (b->x - reach * ux), dy = o.position.y - (b->y - reach * uy);
      const double yaw = std::atan2(2 * (o.orientation.w * o.orientation.z + o.orientation.x * o.orientation.y),
                                    1 - 2 * (o.orientation.y * o.orientation.y + o.orientation.z * o.orientation.z));
      const double forward = ux * dx + uy * dy, lateral = ux * dy - uy * dx;
      const double yaw_error = std::remainder(yaw - (normal + M_PI), 2 * M_PI);
      const bool aligned = std::abs(lateral) <= config.box.lateral_tolerance && std::abs(yaw_error) <= config.box.yaw_tolerance;
      std_msgs::msg::Float64MultiArray report;
      report.data = {lateral, yaw_error, forward, aligned ? 1.0 : 0.0, static_cast<double>(box_corrections)};
      box_alignment_pub->publish(report);
      {
        char t[200];
        std::snprintf(t, sizeof(t), "[box %s] 4/5 check: lateral %+.1f mm, yaw %+.2f deg, forward %+.1f mm (tolerance "
                      "%.0f mm, %.1f deg): %s", step->box.c_str(), lateral * 1000, yaw_error * 180 / M_PI, forward * 1000,
                      config.box.lateral_tolerance * 1000, config.box.yaw_tolerance * 180 / M_PI,
                      aligned ? "ALIGNED, done" : (box_corrections < config.box.max_corrections ? "correcting" : "out of tolerance"));
        say(t);
      }
      if (aligned) {done = true; return std::nullopt;}
      if (box_corrections >= config.box.max_corrections)
        return "box " + step->box + " not aligned after " + std::to_string(box_corrections) + " corrections";
      ++box_corrections;
      box_seen = *b;  // the approach again goes by the box as perceived standing still
      box_pending = BoxMove::Reverse;  // backs out once a line behind is clear (the loop retries)
      return std::nullopt;
    }
    // global_local: renew the local path after local_update of progress along the global path.
    bool local_due() const
    {
      if (global_path.poses.empty() || route_paused || !more_segments || !odom.valid) return false;
      return path_progress(global_path, odom.value.pose.pose.position) - local_origin >= config.path.local_update;
    }
    // false = no new local path: the caller then runs the current one's collision check (remaining_path_clear).
    bool refresh_local(double now)
    {
      const auto began = std::chrono::steady_clock::now();
      const auto plan = path_planner.local_path(global_path, odom.value, obstacles(now), approach ? &*approach : nullptr);
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
      if (!plan.ok) {
        // Keep driving the current local path; its collision check pauses and replans the global path if needed.
        readiness_reason = "local path: " + plan.error;
        RCLCPP_WARN(node.get_logger(), "local path failed (%.1f ms): %s", ms, plan.error.c_str());
        return false;
      }
      RCLCPP_INFO(node.get_logger(), "local path %.1f ms, %zu poses", ms, plan.path.poses.size());
      Computation out; out.generation = fsm.generation(); out.ok = true; out.path = plan.path; out.speed_limit = plan.speed_limit;
      out.global = plan.global;
      out.start = odom.value; out.plan_ms = ms;
      result = std::move(out);
      command(now, true);
      return true;
    }
    bool remaining_path_clear(double now) const
    {
      if (active_path.poses.empty()) return true;
      const auto &p = odom.value.pose.pose.position;
      std::size_t nearest = 0; double best = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < active_path.poses.size(); ++i) {
        const auto &q = active_path.poses[i].pose.position;
        const double d = std::hypot(q.x - p.x, q.y - p.y);
        if (d < best) {best = d; nearest = i;}
      }
      Path rest = active_path;
      rest.poses.erase(rest.poses.begin(), rest.poses.begin() + static_cast<std::ptrdiff_t>(nearest));
      if (approach && rest.poses.size() > 1) {
        // A trigger mode dock path runs on into the box past its face goal: only up to the goal is checked.
        const auto &g = approach->goal;
        const double heading = 2 * std::atan2(g.orientation.z, g.orientation.w);
        while (rest.poses.size() > 1) {
          const auto &q = rest.poses.back().pose.position;
          if ((q.x - g.position.x) * std::cos(heading) + (q.y - g.position.y) * std::sin(heading) <= 1e-6) break;
          rest.poses.pop_back();
        }
      }
      // Blocked further ahead than block_stop_distance: not yet, the robot drives up to the obstacle first.
      return path_planner.clear_length(rest, obstacles(now), approach ? &*approach : nullptr, docking()) > config.block_stop_distance;
    }
    // Nearest puck (base_link) of one observation message -> graspable check and its debounce.
    void check_pucks(const craft_perception::msg::PuckObservationArray &msg)
    {
        std::optional<PuckInBase> nearest;
        std::string error = msg.observations.empty() ? "no puck observed" : "";
        for (const auto &value : msg.observations)
        {
            if (!value.axis_valid) {if (error.empty()) error = "puck axis not valid"; continue;}
            try
            {
                // Centre and centre + axis through TF: the axis is a direction, so take the difference.
                Pose a, b;
                a.header = b.header = msg.header;
                a.pose.position = value.center;
                b.pose.position.x = value.center.x + value.axis.x;
                b.pose.position.y = value.center.y + value.axis.y;
                b.pose.position.z = value.center.z + value.axis.z;
                a.pose.orientation.w = b.pose.orientation.w = 1.0;
                const auto ca = transform(a, config.arm.base_frame).pose.position;
                const auto cb = transform(b, config.arm.base_frame).pose.position;
                PuckInBase puck;
                puck.center = ca;
                puck.axis.x = cb.x - ca.x; puck.axis.y = cb.y - ca.y; puck.axis.z = cb.z - ca.z;
                puck.standing = value.standing;
                if (!nearest || std::hypot(ca.x, ca.y) < std::hypot(nearest->center.x, nearest->center.y)) nearest = puck;
            }
            catch (const std::exception &e) {error = std::string("puck TF: ") + e.what();}
        }
        grasp_check = nearest ? check_graspable(config.grasp, *nearest) : GraspCheck{false, error};
        grasp_puck.clear();
        if (nearest)
        {
            char text[128];
            std::snprintf(text, sizeof(text), "%s x %.3f y %.3f z %.3f, axis yaw %.1f deg",
                          nearest->standing ? "standing" : "lying", nearest->center.x, nearest->center.y,
                          nearest->center.z, std::atan2(nearest->axis.y, nearest->axis.x) * 180.0 / M_PI);
            grasp_puck = text;
        }
        const bool confirmed = grasp_confirm.update(grasp_check, config.grasp.confirm_count);
        if (confirmed != grasp_confirmed)
        {
            RCLCPP_INFO(node.get_logger(), "puck %s: %s", confirmed ? "graspable (confirmed)" : "not graspable",
                        grasp_check.reason.c_str());
        }
        grasp_confirmed = confirmed;
    }
    void resume_navigation(double now) {
      if (!route_paused || !step || step->kind != StepKind::Navigate || active_path.poses.empty()) return;
      TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id;
      c.revision = ++revision; c.operation = TaskCommand::UPDATE_PATH; c.kind = TaskCommand::NAVIGATE;
      c.path = active_path; c.path.header.stamp = node.now(); c.speed = drive_speed(); c.speed_limit = active_speed;
      c.require_goal_heading = heading_required();
      c.goal_tolerance = active_goal_tolerance; c.tolerance_rad = active_heading_tolerance;
      c.tracking_reference_x = active_tracking_reference_x; c.lookahead_distance = active_lookahead_distance;
      camera_fields(c);
      command_pub->publish(c); route_paused = false; last_replan = now;
    }
    void pause_navigation(double now) {
      if (route_paused || !step || step->kind != StepKind::Navigate || active_path.poses.empty()) return;
      TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id;
      c.revision = ++revision; c.operation = TaskCommand::UPDATE_PATH; c.kind = TaskCommand::NAVIGATE;
      c.path = active_path; c.path.header.stamp = node.now(); c.speed = 0.0; c.speed_limit = active_speed;
      c.require_goal_heading = heading_required();
      c.goal_tolerance = active_goal_tolerance; c.tolerance_rad = active_heading_tolerance;
      c.tracking_reference_x = active_tracking_reference_x; c.lookahead_distance = active_lookahead_distance;
      camera_fields(c);
      command_pub->publish(c); route_paused = true; last_replan = now;
    }
    void report(double now) {
      std_msgs::msg::String state; state.data = fsm.label(); state_pub->publish(state);
      std_msgs::msg::String box_state;
      if (!config.box.camera_enabled || !step || step->box.empty() || fsm.state() == SystemState::Fault) box_state.data = "off";
      else switch (box_phase) {
        case BoxPhase::Stage: box_state.data = "stage"; break;
        case BoxPhase::CameraStageStop: box_state.data = "stage_stop"; break;
        case BoxPhase::LidarPerceive: box_state.data = "lidar_perceive"; break;
        case BoxPhase::LidarRelease: box_state.data = "lidar_release"; break;
        case BoxPhase::LidarDock: box_state.data = "lidar_dock"; break;
        case BoxPhase::HandoffStop: box_state.data = "handoff_stop"; break;
        case BoxPhase::CameraPerceive: box_state.data = "camera_perceive"; break;
        case BoxPhase::CameraRelease: box_state.data = "camera_release"; break;
        case BoxPhase::CameraDock: box_state.data = "camera_dock"; break;
        case BoxPhase::FinalStop: box_state.data = "final_stop"; break;
        case BoxPhase::CameraVerify: box_state.data = "camera_verify"; break;
        default: box_state.data = "off"; break;
      }
      box_phase_pub->publish(box_state);
      if (fsm.state() == SystemState::Fault && !fault_reported) {
        RCLCPP_ERROR(node.get_logger(),
                     "[PLANNING FAULT] state=%s reason='%s' waiting_for='%s' last_compute_error='%s' blocked_by='%s' align='%s' obs_reject='%s'",
                     fsm.label().c_str(), fsm.reason().c_str(), readiness_reason.c_str(),
                     last_compute_error.c_str(), box_blocked_by.c_str(),
                     last_alignment_detail.c_str(), last_obs_reject_reason.c_str());
        fault_reported = true;
      }
      if (now - last_report < 1) return;
      last_report = now;
      diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = node.now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "planning/state_machine"; status.hardware_id = "planning_master";
      status.level = fsm.state() == SystemState::Fault ? 2 : 0;
      status.message = fsm.label() + ": " + fsm.reason();
      for (const auto &item : std::map<std::string, std::string>{
          {"task_id", step ? std::to_string(step->task_id) : ""},
          {"step", step ? step->id : ""}, {"queued", std::to_string(remaining.size())},
          {"path_revision", std::to_string(revision)}, {"waiting_for", readiness_reason},
          {"plan_ms", std::to_string(plan_ms)}, {"plan_ms_max", std::to_string(plan_ms_max)},
          {"more_segments", more_segments ? "true" : "false"},
          {"puck_graspable", grasp_check.graspable ? "true" : "false"}, {"puck_grasp_reason", grasp_check.reason},
          {"puck_grasp_confirmed", grasp_confirmed ? "true" : "false"}, {"puck_base_link", grasp_puck}}) {
        diagnostic_msgs::msg::KeyValue value; value.key = item.first; value.value = item.second;
        status.values.push_back(value);
      }
      array.status.push_back(status); diagnostics_pub->publish(array);
    }

};

PlanningMaster::PlanningMaster(const rclcpp::NodeOptions &options)
    : Node("planning_master", options), impl_(std::make_unique<Impl>(*this))
{
    configure();
    connect();
}

PlanningMaster::~PlanningMaster()
{
    // Worker inputs are snapshots; keep the configured algorithms alive until it exits.
    if (impl_->worker.valid())
    {
        impl_->worker.wait();
    }
}

void PlanningMaster::configure()
{
    auto &p = *impl_;
    p.config = load_planning_config(*this, ament_index_cpp::get_package_share_path("craft_planning").string());
    p.path_planner.configure(p.config.path);
    p.arm_planner.configure(p.config.arm);
    p.fsm.configure(p.config.fsm);
    p.fsm.initialize(seconds());
}

void PlanningMaster::connect()
{
    auto &p = *impl_;
    p.tf = std::make_shared<tf2_ros::Buffer>(get_clock());
    p.listener = std::make_shared<tf2_ros::TransformListener>(*p.tf);
    p.command_pub = create_publisher<TaskCommand>("/planning/task_command", 10);
    p.odom_pub = create_publisher<Odom>("/planning/odom", 10);
    p.hold_pub = create_publisher<std_msgs::msg::Bool>("/planning/hold", 10);
    p.state_pub = create_publisher<std_msgs::msg::String>("/planning/state", rclcpp::QoS(1).transient_local());
    p.box_phase_pub = create_publisher<std_msgs::msg::String>("/planning/box_phase", rclcpp::QoS(1).transient_local());
    p.box_request_pub = create_publisher<BoxObservationRequest>("/planning/box_observation_request", rclcpp::QoS(1).transient_local());
    p.box_goal_pub = create_publisher<Pose>("/planning/approach_goal", rclcpp::QoS(1).transient_local());
    p.box_search_pub = create_publisher<Pose>("/planning/box_search", rclcpp::QoS(1).transient_local());
    p.ignored_pub = create_publisher<Points>("/planning/ignored_obstacles", 10);
    p.padding_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("/planning/padding", 1);
    p.box_status_pub = create_publisher<std_msgs::msg::String>("/planning/box_status", 10);
    p.box_alignment_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/planning/box_alignment", 10);
    p.diagnostics_pub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/planning/diagnostics", 10);

    p.subscribe<std_msgs::msg::String>("/planning/command",
      [&p](const std_msgs::msg::String &msg) {p.lifecycle(msg.data);});
    p.subscribe<ControlStatus>("/control/status", [&p](const ControlStatus &msg) {
      if (p.is_fresh_input(msg.header, p.config.fsm.feedback_timeout_s)) p.control.observe(msg, seconds());
    });
    p.subscribe<TaskResult>("/control/task_result", [&p](const TaskResult &msg) {
      if (p.step && msg.task_id == p.step->task_id && p.dispatched &&
          p.is_fresh_input(msg.header, p.config.fsm.feedback_timeout_s)) p.task_result.observe(msg, seconds());
    });
    p.subscribe<std_msgs::msg::String>("/planning/mission",
                                       [&p](const std_msgs::msg::String &msg) { p.mission(msg.data); });
    p.subscribe<std_msgs::msg::Int32MultiArray>("/perception/vision/d435/apriltag",
      [&p](const std_msgs::msg::Int32MultiArray &msg) {
        for (const int id : msg.data) {
          const auto found = p.config.mission_by_tag.find(id);
          if (found != p.config.mission_by_tag.end()) {p.mission(found->second); break;}
        }
      });
    p.subscribe<Pose>("/planning/target_base_pose", [&p](const Pose &msg) {
        PlanningStep step;
        step.id = "base_goal";
        step.kind = StepKind::Navigate;
        step.goal = msg;
        std::ostringstream key;
        key.precision(17);
        key << step.id << ':' << msg.header.frame_id << ':' << msg.header.stamp.sec << ':' << msg.header.stamp.nanosec
            << ':' << msg.pose.position.x << ':' << msg.pose.position.y << ':' << msg.pose.position.z
            << ':' << msg.pose.orientation.x << ':' << msg.pose.orientation.y << ':' << msg.pose.orientation.z
            << ':' << msg.pose.orientation.w;
        p.accept(step, {}, key.str());
    });
    // The box as perceived (map frame; orientation = yaw of a face normal). Box detection publishes it; one taken
    // standing still at the stage point starts the dock.
    p.subscribe<BoxObservation>("/perception/box/observation", [&p](const BoxObservation &msg) {
      if (!p.config.box.camera_enabled || msg.source != "lidar" || msg.request_id != p.box_request.request_id ||
          msg.source != p.box_request.source || msg.header.frame_id != p.config.path.map_frame) return;
      const auto stamp = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
      const auto requested = static_cast<int64_t>(p.box_request.header.stamp.sec) * 1000000000LL +
        p.box_request.header.stamp.nanosec;
      Pose pose; pose.header = msg.header; pose.pose = msg.box_pose;
      const bool valid = msg.valid && !msg.extrapolated && valid_pose(pose) &&
        std::isfinite(msg.gap_m) && msg.gap_m >= 0.0 &&
        std::isfinite(msg.lateral_m) && std::isfinite(msg.yaw_rad) &&
        p.step && !p.step->box.empty() &&  // only near the box's planning.yaml place (not the last perceived one)
        std::hypot(msg.box_pose.position.x - p.box_target(p.step->box).x,
                   msg.box_pose.position.y - p.box_target(p.step->box).y) <= p.config.box.max_offset &&
        std::isfinite(msg.gap_uncertainty_m) && msg.gap_uncertainty_m >= 0 &&
        msg.gap_uncertainty_m <= p.config.box.lidar_uncertainty_max + 1e-12 &&
        msg.header.stamp.sec >= 0 && msg.header.stamp.nanosec < 1000000000U &&
        stamp >= requested && stamp > p.box_last_observation_stamp &&
        p.is_fresh_input(msg.header, p.config.box.lidar_observation_timeout);
      // Reject only this sample. Keep the last valid observation and its original
      // acquisition stamp; the normal freshness deadline stops prolonged loss.
      if (!valid) {
        char rbuf[200];
        std::snprintf(rbuf, sizeof(rbuf), "lidar valid=%d extrap=%d gap=%.3f unc=%.3f reason=%s",
                      msg.valid ? 1 : 0, msg.extrapolated ? 1 : 0, msg.gap_m, msg.gap_uncertainty_m, msg.reason.c_str());
        p.last_obs_reject_reason = rbuf;
        RCLCPP_WARN_THROTTLE(p.node.get_logger(), *p.node.get_clock(), 1000, "[OBS REJECT] %s", rbuf);
        return;
      }
      p.box_last_observation_stamp = stamp;
      p.box_had_observation = true;
      p.box_observation.observe(msg, seconds());
      if (p.box_phase == PlanningMaster::Impl::BoxPhase::LidarPerceive &&
          stamp >= requested + static_cast<int64_t>(std::llround(p.config.box.settle_s * 1e9)))
        p.lidar_samples.push_back(msg);
    });
    // The continuous camera provides only a longitudinal gap in base_link.
    // Its acquisition stamp, never a transformed/request-response stamp, gates use.
    p.subscribe<BoxObservation>("/perception/uppercam/observation", [&p](const BoxObservation &msg) {
      using Phase = decltype(p.box_phase);
      if (!p.config.box.camera_enabled || !p.active() || p.box_request.source != "uppercam" ||
          !(p.box_phase == Phase::CameraPerceive || p.box_phase == Phase::CameraRelease ||
            p.box_phase == Phase::CameraDock || p.box_phase == Phase::FinalStop ||
            p.box_phase == Phase::CameraVerify) ||
          msg.source != "uppercam" || msg.request_id != 0 || msg.header.frame_id != "base_link") return;
      const int64_t stamp = static_cast<int64_t>(msg.header.stamp.sec)*1000000000LL + msg.header.stamp.nanosec;
      const int64_t requested = static_cast<int64_t>(p.box_request.header.stamp.sec)*1000000000LL +
        p.box_request.header.stamp.nanosec;
      // Pre-request and reordered frames are not measurements for this acquisition.
      if (stamp <= requested || stamp <= p.box_last_observation_stamp) return;
      const double now = seconds();
      const bool initial = p.box_phase == Phase::CameraPerceive && !p.box_had_observation;
      const bool before_deadline = now >= p.box_request_at &&
        now - p.box_request_at < p.config.box.observation_initial_timeout;
      const bool valid = (!initial || before_deadline) && msg.valid &&
        std::isfinite(msg.gap_m) && msg.gap_m >= 0 &&
        msg.gap_m <= p.config.box.camera_operational_gap_max + 1e-12 &&
        std::isfinite(msg.gap_uncertainty_m) && msg.gap_uncertainty_m >= 0 &&
        msg.gap_uncertainty_m <= (msg.gap_m > p.config.box.camera_near_gap ?
          p.config.box.camera_far_uncertainty_max : p.config.box.camera_uncertainty_max) + 1e-12 &&
        msg.header.stamp.sec >= 0 && msg.header.stamp.nanosec < 1000000000U &&
        p.is_fresh_input(msg.header, p.config.box.camera_timeout);
      if (!valid) {
        if (initial && before_deadline && p.camera_stopped(now)) return;
        if (p.config.box.camera_only_longitudinal) {  // trigger mode: an invalid frame is never a fault
          // Still following the LiDAR line toward the camera's range: not a measurement yet, not a fault.
          char rbuf[200];
          std::snprintf(rbuf, sizeof(rbuf), "uppercam (seeking) valid=%d gap=%.3f unc=%.3f reason=%s", msg.valid ? 1 : 0,
                        msg.gap_m, msg.gap_uncertainty_m, msg.reason.c_str());
          p.last_obs_reject_reason = rbuf;
          return;
        }
        p.box_observation.valid = false;
        p.fsm.fail("invalid continuous camera observation", now);
        TaskCommand c; c.header.stamp = p.node.now(); c.operation = TaskCommand::STOP;
        c.task_id = p.step ? p.step->task_id : 0; p.command_pub->publish(c); p.stop();
        return;
      }
      p.box_last_observation_stamp = stamp;
      p.box_had_observation = true;
      p.box_observation.observe(msg, now);
    });
    p.subscribe<Pose>("/perception/box/pose", [&p](const Pose &msg) {
        if (msg.header.frame_id == p.config.path.map_frame && valid_pose(msg)) p.box_detection.observe(msg, seconds());
    });
    // Drive up to a box_approach box by name (planning.yaml box_approach.names); every message is a new request.
    p.subscribe<std_msgs::msg::String>("/planning/approach_box", [&p](const std_msgs::msg::String &msg) {
        PlanningStep step;
        step.id = "box_approach";
        step.kind = StepKind::Navigate;
        step.box = msg.data;
        step.goal.header.frame_id = p.config.path.map_frame;  // resolved by plan_box
        step.goal.pose.orientation.w = 1.0;
        if (!p.config.box.boxes.count(msg.data)) {
            RCLCPP_WARN(p.node.get_logger(), "approach_box: unknown box '%s'", msg.data.c_str()); return;
        }
        const auto &b = p.config.box.boxes.at(msg.data);
        p.accept(step);
        p.say("[box " + msg.data + "] 1/5 known box " + p.xy(b[0], b[1]) + ": planning to the stage points of its "
              "approachable faces");
    });
    p.subscribe<std_msgs::msg::String>("/planning/approach_box_near", [&p](const std_msgs::msg::String &msg) {
        if (!p.config.box.camera_enabled || !p.config.box.boxes.count(msg.data)) {
          RCLCPP_WARN(p.node.get_logger(), "near box entry unavailable or unknown box '%s'", msg.data.c_str()); return;
        }
        PlanningStep step; step.id = "box_near_entry"; step.kind = StepKind::Navigate;
        step.box = msg.data; step.box_near_entry = true;
        step.goal.header.frame_id = p.config.path.map_frame; step.goal.pose.orientation.w = 1.0;
        p.accept(step);
    });
    p.subscribe<Pose>("/planning/target_tcp_pose", [&p](const Pose &msg) {
        PlanningStep step;
        step.id = "tcp_goal";
        step.kind = StepKind::Tcp;
        step.goal = msg;
        std::ostringstream key;
        key.precision(17);
        key << step.id << ':' << msg.header.frame_id << ':' << msg.header.stamp.sec << ':' << msg.header.stamp.nanosec
            << ':' << msg.pose.position.x << ':' << msg.pose.position.y << ':' << msg.pose.position.z
            << ':' << msg.pose.orientation.x << ':' << msg.pose.orientation.y << ':' << msg.pose.orientation.z
            << ':' << msg.pose.orientation.w;
        p.accept(step, {}, key.str());
    });
    p.subscribe<std_msgs::msg::String>("/perception/localization/state",
      [&p](const std_msgs::msg::String &msg) {p.localization_state = msg.data;});
    p.subscribe<Odom>("/perception/lidar/odom", [&p](const Odom &msg) {
        Pose pose; pose.header = msg.header; pose.pose = msg.pose.pose;
        const auto &v = msg.twist.twist.linear; const auto &w = msg.twist.twist.angular;
        const bool valid = valid_pose(pose) && std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
          std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z) &&
          std::all_of(msg.pose.covariance.begin(), msg.pose.covariance.end(), [](double x) {return std::isfinite(x);}) &&
          std::all_of(msg.twist.covariance.begin(), msg.twist.covariance.end(), [](double x) {return std::isfinite(x);}) &&
          msg.twist.covariance[0] >= 0.0;
        p.receive_localization(msg, p.lidar_odom, p.lidar_stamp, valid);
    });
    p.subscriptions.push_back(create_subscription<sensor_msgs::msg::Imu>("/perception/imu/data",
      rclcpp::SensorDataQoS(), [&p](const sensor_msgs::msg::Imu &msg) {
        const auto &a = msg.linear_acceleration; const auto &w = msg.angular_velocity;
        const bool valid = !msg.header.frame_id.empty() &&
          std::all_of(msg.angular_velocity_covariance.begin(), msg.angular_velocity_covariance.end(),
            [](double x) {return std::isfinite(x);}) && msg.angular_velocity_covariance[0] >= 0.0 &&
          msg.angular_velocity_covariance[8] >= 0.0 && std::isfinite(a.x) && std::isfinite(a.y) &&
          std::isfinite(a.z) && std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z);
        p.receive_localization(msg, p.imu, p.imu_stamp, valid);
    }));
    // control publishes it best effort (SensorDataQoS): the reliable default of subscribe() got nothing (2026-11-10).
    p.subscriptions.push_back(p.node.create_subscription<Odom>("/control/wheel/odom", rclcpp::SensorDataQoS(),
        [&p](const Odom &msg) {p.wheel_odom.observe(msg, seconds());}));
    p.subscribe<Odom>("/perception/state/odom", [&p](const Odom &msg) {
        Pose pose;
        pose.header = msg.header;
        pose.pose = msg.pose.pose;
        p.odom.valid = false;
        if (valid_pose(pose) && msg.header.frame_id == p.config.path.map_frame &&
            p.is_fresh_input(msg.header, p.config.odom_timeout_s))
        {
            Odom adjusted = msg;
            const bool docking = p.step && !p.step->box.empty() &&
              (p.box_phase == PlanningMaster::Impl::BoxPhase::CameraDock || p.box_phase == PlanningMaster::Impl::BoxPhase::FinalStop ||
               p.box_phase == PlanningMaster::Impl::BoxPhase::CameraVerify);
            if (!docking) p.dock_anchor.reset();
            if (docking && p.dock_anchor && p.wheel_odom.fresh(seconds(), p.config.odom_timeout_s)) {
              // 2026-11-10 (L1cam_1110_1116): on the last 0.3 m the localization moved the robot 30 mm sideways
              // against its wheels (+17 vs -13 mm); control steered after it and stopped 5.4 deg off the face.
              const auto &a = *p.dock_anchor;
              const auto &w = p.wheel_odom.value.pose.pose;
              const double wyaw = 2 * std::atan2(w.orientation.z, w.orientation.w);
              const double dx = w.position.x - a.wx, dy = w.position.y - a.wy;
              const double lx = std::cos(a.wyaw) * dx + std::sin(a.wyaw) * dy, ly = -std::sin(a.wyaw) * dx + std::cos(a.wyaw) * dy;
              const double yaw = a.myaw + std::remainder(wyaw - a.wyaw, 2 * M_PI);
              adjusted.pose.pose.position.x = a.mx + std::cos(a.myaw) * lx - std::sin(a.myaw) * ly;
              adjusted.pose.pose.position.y = a.my + std::sin(a.myaw) * lx + std::cos(a.myaw) * ly;
              adjusted.pose.pose.orientation.x = adjusted.pose.pose.orientation.y = 0;
              adjusted.pose.pose.orientation.z = std::sin(yaw / 2); adjusted.pose.pose.orientation.w = std::cos(yaw / 2);
            }
            p.odom.observe(adjusted, seconds());
            const double t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
            if (!p.odom_trail.empty() && t <= p.odom_trail.back()[0]) p.odom_trail.clear();  // time went back
            p.odom_trail.push_back({t, adjusted.pose.pose.position.x, adjusted.pose.pose.position.y});
            while (p.odom_trail.front()[0] < t - p.odom_trail_s) p.odom_trail.pop_front();
        }
    });
    p.subscribe<Joints>(p.config.joint_state_topic, [&p](const Joints &msg) {
        p.joints.valid = false;
        if (!p.is_fresh_input(msg.header, p.config.joint_timeout_s))
        {
            return;
        }
        try
        {
            p.arm_planner.forward(msg);
            p.joints.observe(msg, seconds());
        }
        catch (const std::exception &)
        {
        }
    });
    p.subscribe<Points>("/perception/lidar/obstacle", [&p](const Points &msg) { p.receive_points(msg, p.lidar); });
    p.subscribe<Points>("/perception/vision/d435/obstacle",
                        [&p](const Points &msg) { p.receive_points(msg, p.vision_obstacles); });
    p.subscribe<Pose>("/perception/vision/object_pose",
                      [&p](const Pose &msg) { p.observations["object_pose"] = {{msg, -1, 1.0}}; });
    p.subscribe<Points>("/perception/vision/d435/object_detection", [&p](const Points &msg) {
        auto &values = p.observations["object_detection"];
        values.clear();
        for (const auto &value : msg.poses)
        {
            Pose pose;
            pose.header = msg.header;
            pose.pose = value;
            values.push_back({pose, -1, 1.0});
        }
    });
    p.subscribe<craft_perception::msg::KnobObservationArray>(
        "/perception/vision/d435/knob_observations", [&p](const craft_perception::msg::KnobObservationArray &msg) {
            auto &values = p.observations["d435_knob"];
            values.clear();
            for (const auto &value : msg.observations)
            {
                if (!value.depth_valid)
                {
                    continue;
                }
                Pose pose;
                pose.header = msg.header;
                pose.pose.position = value.position;
                pose.pose.orientation.w = 1.0;
                values.push_back({pose, value.detection.class_id, value.detection.confidence});
            }
        });
    p.subscribe<craft_perception::msg::PuckObservationArray>(
        "/perception/vision/d435/puck_observations", [&p](const craft_perception::msg::PuckObservationArray &msg) {
            p.check_pucks(msg);
            auto &values = p.observations["d435_puck"];
            values.clear();
            for (const auto &value : msg.observations)
            {
                if (!value.axis_valid)
                {
                    continue;
                }
                Pose pose;
                pose.header = msg.header;
                pose.pose.position = value.center;
                pose.pose.orientation.w = 1.0;
                values.push_back({pose, value.detection.class_id, value.detection.confidence});
            }
        });
    p.timer = create_wall_timer(std::chrono::duration<double>(1.0 / p.config.loop_hz), [this] { planning_loop(); });
    RCLCPP_INFO(get_logger(), "planning ready: C++ FSM, static-grid path planner and headless URDF IK");
}

void PlanningMaster::planning_loop() {
  auto &p = *impl_;
  const double now = seconds();
  try {
    p.collect();
    // /planning/padding once a second (RViz): the map and the obstacles as planning uses them, padded.
    if (p.padding_pub->get_subscription_count() > 0 && now - p.padding_pub_at >= 1.0) {
      p.padding_pub_at = now;
      auto grid = p.path_planner.padding(p.obstacles(now), p.approach ? &*p.approach : nullptr, p.docking());
      grid.header.stamp = p.node.now(); p.padding_pub->publish(grid);
    }
    if (p.localization_fresh(now) && p.odom.fresh(now, p.config.odom_timeout_s) &&
        p.is_fresh_input(p.odom.value.header, p.config.odom_timeout_s)) p.odom_pub->publish(p.odom.value);
    if (p.reset_requested && p.control_fresh(now) && p.control.value.held && p.control.value.stopped &&
        !p.worker.valid()) {
      p.path_planner.configure(p.config.path); p.arm_planner.configure(p.config.arm);
      if (p.fsm.reset(true, now)) {
        p.step.reset(); p.remaining.clear(); p.result.reset(); p.dispatched = false;
        p.task_result.valid = false; p.fault_reported = false; p.last_mission.clear();
        p.active_path = Path{}; p.route_paused = false; p.waiting_for_path = false;
        p.more_segments = p.next_segment = false; p.global_path.poses.clear(); p.blocked_since.reset(); p.route_failed = false; p.no_progress = 0;
      }
      p.reset_requested = false;
    }
    if (p.fsm.phase() == RunPhase::AwaitRequest && !p.remaining.empty()) {
      p.step = std::move(p.remaining.front()); p.remaining.pop_front();
      p.step->task_id = ++p.next_id;  // above any segment ID handed out for the previous step
      p.result.reset(); p.task_result.valid = false; p.dispatched = false; p.revision = 0;
      p.waiting_for_path = false; p.route_paused = false; p.active_path = Path{};
      p.more_segments = p.next_segment = false; p.global_path.poses.clear(); p.blocked_since.reset(); p.route_failed = false; p.no_progress = 0;
      p.fsm.start(p.step->kind, now);
    }
    const auto phase = p.fsm.phase();
    const bool camera_stationary_hold = p.config.box.camera_enabled && p.step && !p.step->box.empty() &&
      ((p.step->box_near_entry && !p.dispatched && p.box_phase != decltype(p.box_phase)::CameraRelease) || (p.box_phase != decltype(p.box_phase)::Stage && p.box_phase != decltype(p.box_phase)::LidarDock &&
      p.box_phase != decltype(p.box_phase)::CameraDock && p.box_phase != decltype(p.box_phase)::LidarRelease &&
      p.box_phase != decltype(p.box_phase)::CameraRelease));
    const bool needs_release = !camera_stationary_hold &&
      (phase == RunPhase::PrepareTask || phase == RunPhase::Compute || phase == RunPhase::Execute);
    p.hold(!needs_release);
    FsmInput input;
    input.ready = p.fsm.state() == SystemState::Idle ? p.system_ready(now) :
      phase == RunPhase::PrepareTask && p.ready(now);
    input.stopped = p.control_fresh(now) && p.control.value.held && p.control.value.stopped &&
      (phase != RunPhase::Stopping || p.control.at >= p.stop_at);
    if (phase == RunPhase::Stopping && p.dispatched) {
      input.stopped = input.stopped && p.task_result.fresh(now, p.config.fsm.feedback_timeout_s) &&
        p.task_result.value.status == TaskResult::STOPPED;
    }
    input.healthy = p.control_fresh(now) && !p.control.value.fault && p.control.value.ready;
    input.feedback_fresh = p.control_fresh(now);
    input.error = p.readiness_reason;
    if (!input.healthy) input.error = "control unavailable: " + p.control.value.reason;
    if (input.healthy && p.fsm.state() != SystemState::Idle && !p.localization_fresh(now)) {
      input.healthy = false; input.error = "lidar odom or IMU observation invalid or stale";
    }
    if (input.healthy && p.localization_state == "lost") {
      // Dead reckoning past its budget and the wide relocalization failed: no trustworthy pose.
      input.healthy = false;
      input.error = "localization lost (dead reckoning and relocalization failed)";
    }
    if (phase == RunPhase::Execute) {
      // healthy already implies fresh control; name the input that went stale instead of a leftover reason.
      if (input.healthy && !p.sensors_fresh(now)) {
        input.feedback_fresh = false; input.error = "stale odom/lidar input";
      } else if (input.healthy && p.dispatched && now - p.command_at > p.config.fsm.feedback_timeout_s &&
                 !p.task_result.fresh(now, p.config.fsm.feedback_timeout_s)) {
        input.feedback_fresh = false; input.error = "stale control task result";
      }
      const bool camera_box = p.config.box.camera_enabled && p.step && !p.step->box.empty();
      if (p.task_result.valid && !camera_box) {
        const auto status = p.task_result.value.status;
        input.completed = status == TaskResult::SUCCEEDED;
        if (status == TaskResult::FAILED || status == TaskResult::REJECTED || status == TaskResult::STOPPED ||
            status == TaskResult::STOPPING) {
          input.healthy = false; input.error = "control task: " + p.task_result.value.reason;
        }
      }
      if (input.healthy && input.feedback_fresh && camera_box) p.camera_box_tick(now, input);
      // Box step (correction parking): a drive that ends is not the step's end until the alignment check passes.
      if (!camera_box && input.healthy && input.completed && p.step->kind == StepKind::Navigate && !p.step->box.empty()) {
        using BoxPhase = decltype(p.box_phase);
        input.completed = false;
        if (p.box_pending != decltype(p.box_pending)::None) {
          // A dock or back-out goes as soon as its line is clear; blocked longer than block_wait_s fails.
          const bool dock = p.box_pending == decltype(p.box_pending)::Dock;
          if (dock ? p.send_dock(now) : p.send_reverse(now)) {p.box_pending = decltype(p.box_pending)::None; p.box_blocked_at.reset();}
          else if (!p.box_blocked_at) {
            p.box_blocked_at = now;
            p.readiness_reason = dock ? "box: dock line blocked; waiting" : "box: nothing clear behind to back out on; waiting";
            p.say("[box " + p.step->box + "] " + (dock ? "dock line" : "back-out line") + " blocked by " + p.box_blocked_by +
                  ": waiting up to " + std::to_string(p.config.block_wait_s).substr(0, 4) + " s");
          } else if (now - *p.box_blocked_at >= p.config.block_wait_s) {
            input.healthy = false;
            input.error = "box " + p.step->box + (dock ? ": the dock line stays blocked by " : ": no clear line behind (" +
              std::to_string(p.config.box.reverse_min_distance).substr(0, 4) + " m) to back out on, blocked by ") + p.box_blocked_by;
          }
        } else if (p.box_phase == BoxPhase::Stage || p.box_phase == BoxPhase::Reverse) {
          // At the stage point, or backed out: stand still and perceive the box with the LiDAR.
          p.box_phase = BoxPhase::Perceive; p.box_perceive_at = now; p.readiness_reason = "box: perceiving";
          p.say("[box " + p.step->box + "] 2/5 at the stage point: standing still " +
                std::to_string(p.config.box.settle_s).substr(0, 4) + " s, then the LiDAR box pose");
        } else if (p.box_phase == BoxPhase::Perceive && now - p.box_perceive_at >= p.config.box.settle_s) {
          // A detection taken standing still: the dock goes by it.
          const auto b = p.detected_box(now);
          if (b && p.box_detection.at >= p.box_perceive_at + p.config.box.settle_s) {
            const auto e = p.box_target(p.step->box);
            char t[220];
            std::snprintf(t, sizeof(t), "[box %s] 2/5 perceived at (%.3f, %.3f), yaw %.1f deg (known (%.3f, %.3f), %.0f mm off)",
                          p.step->box.c_str(), b->x, b->y, b->yaw * 180 / M_PI, e.x, e.y,
                          std::hypot(b->x - e.x, b->y - e.y) * 1000);
            p.say(t);
            p.box_seen = *b; p.box_pending = decltype(p.box_pending)::Dock;
          } else if (now - p.box_perceive_at > p.config.box.settle_s + p.config.box_detection_timeout_s) {
            input.healthy = false;
            input.error = "box " + p.step->box + " not perceived at the stage point (/perception/box/pose, within " +
              std::to_string(p.config.box.max_offset).substr(0, 4) + " m of the known box)";
          }
        } else if (p.box_phase == BoxPhase::Dock) {  // at the face goal: stand still, then check
          p.box_phase = BoxPhase::Settle; p.box_settle_at = now; p.readiness_reason = "box: settling before the check";
          p.say("[box " + p.step->box + "] 4/5 at the face goal: standing still " +
                std::to_string(p.config.box.settle_s).substr(0, 4) + " s, then the check");
        } else if (p.box_phase == BoxPhase::Settle && now - p.box_settle_at >= p.config.box.settle_s) {
          bool done = false;
          if (const auto error = p.check_alignment(now, done)) {
            input.healthy = false; input.error = *error;
          }
          input.completed = done;
        }
      }
      // A segment ended at a STOP_TURN corner: not the goal yet. Plan the rest from here as a new task.
      if (input.healthy && input.feedback_fresh && input.completed && p.more_segments &&
          p.step->kind == StepKind::Navigate) {
        input.completed = false;
        if (!p.next_segment) {
          const auto &here = p.odom.value.pose.pose.position;
          p.no_progress = std::hypot(here.x - p.segment_start.x, here.y - p.segment_start.y) < 0.02 ? p.no_progress + 1 : 0;
          p.next_segment = true; p.result.reset(); p.last_replan = now - p.config.replan_period_s;
        }
        if (p.no_progress >= 3) {
          input.healthy = false; input.error = "no progress: 3 segments in a row ended where they started";
        }
      }
      // Replanning stays in Run.Execute; within a segment it preserves the task ID (UPDATE_PATH).
      const bool camera_stationary = camera_box && p.box_phase != decltype(p.box_phase)::Stage &&
        p.box_phase != decltype(p.box_phase)::LidarDock && p.box_phase != decltype(p.box_phase)::CameraDock;
      if (input.healthy && input.feedback_fresh && !input.completed && !camera_stationary && p.step->kind == StepKind::Navigate) {
        if (p.result) {
          if (!p.result->ok && p.result->retryable) {
            p.readiness_reason = "route unavailable (" + p.result->error + "); pausing navigation and retrying";
            if (!p.next_segment) p.pause_navigation(now);
            p.route_failed = true; p.result.reset(); p.last_replan = now;
          } else if (!p.result->ok) {input.healthy = false; input.error = "replan failed: " + p.result->error;}
          else if (p.next_segment) {
            p.next_segment = false; p.step->task_id = ++p.next_id; p.readiness_reason.clear(); p.command(now);
          } else p.command(now, true);
        } else if (p.worker.valid() && now - p.compute_at >= p.config.fsm.compute_timeout_s) {
          input.healthy = false; input.error = "replan timeout";
        } else if (!p.worker.valid() && now - p.last_replan >= p.config.replan_period_s) {
          // No periodic replanning while a segment is clear: A*+arc's start heading is rounded to 90 degrees,
          // so a replan in the middle of a fillet could flip the plan under the robot.
          p.last_replan = now;
          if (p.next_segment || p.route_failed) p.compute(now);
          else if (p.local_due() && p.refresh_local(now)) {}  // renewed; a failed renewal falls through to the check
          else if (p.remaining_path_clear(now)) {
            if (p.blocked_since) {p.blocked_since.reset(); p.readiness_reason.clear(); p.resume_navigation(now);}
          } else if (!p.blocked_since) {
            p.blocked_since = now; p.readiness_reason = "path blocked; waiting before replanning";
            p.pause_navigation(now);
          } else if (now - *p.blocked_since >= p.config.block_wait_s) {
            using BoxPhase = decltype(p.box_phase);
            if (p.box_in(BoxPhase::Dock) || p.box_in(BoxPhase::Reverse) ||
                p.box_in(BoxPhase::LidarDock) || p.box_in(BoxPhase::CameraDock)) {
              // No global replan in the dock or a back-out (it would lead off the face normal): fail instead.
              input.healthy = false;
              input.error = "box " + p.step->box + ": the dock or back-out path stays blocked (obstacle or padding)";
            } else {p.readiness_reason = "path still blocked; replanning"; p.compute(now);}
          }
        }
      }
      if (!input.healthy || !input.feedback_fresh) p.box_end("FAILED: " + input.error);
      else if (input.completed) p.box_end("ALIGNED");
    }
    if (phase == RunPhase::Compute && p.result) {
      if (p.step->kind == StepKind::Navigate && !p.result->ok && p.result->retryable) {
        p.readiness_reason = "route unavailable (" + p.result->error + "); waiting to retry";
        p.result.reset(); p.waiting_for_path = true; p.retry_compute_at = now + p.config.replan_period_s;
      } else {
        input.compute_done = true; input.compute_ok = p.result->ok; input.error = p.result->error;
      }
    }
    if (phase == RunPhase::Compute && p.waiting_for_path && !p.worker.valid() && now >= p.retry_compute_at) {
      p.waiting_for_path = false;
      if (p.fsm.retry_compute(now)) p.compute(now);
    }
    const bool was_active = p.active();
    const auto output = p.fsm.tick(input, now);
    if (was_active && p.fsm.state() == SystemState::Fault) {
      TaskCommand c; c.header.stamp = p.node.now(); c.operation = TaskCommand::STOP;
      c.task_id = p.dispatched && p.step ? p.step->task_id : 0;
      p.command_pub->publish(c);
    }
    if (output.start_compute) p.compute(now);
    if (output.send_command) p.command(now);
    if (output.stop) p.stop();
    if (p.fsm.state() == SystemState::Fault) {
      if (p.config.box.camera_enabled && p.step && !p.step->box.empty() &&
          !p.box_request.source.empty() && p.box_request.source != "off") p.request_box_observation("off", now);
      p.remaining.clear(); p.result.reset(); p.waiting_for_path = false; p.route_paused = false;
    }
    if (p.fsm.phase() == RunPhase::AwaitRequest ||
        (p.fsm.state() == SystemState::Ready && phase == RunPhase::Stopping)) {
      p.step.reset(); p.result.reset(); p.dispatched = false;
    }
    p.report(now);
  } catch (const std::exception &e) {
    p.fsm.fail(e.what(), now); p.remaining.clear(); p.result.reset(); p.stop(); p.report(now);
  }
}

} // namespace craft_planning

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        rclcpp::spin(std::make_shared<craft_planning::PlanningMaster>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("planning_master"), "%s", e.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
