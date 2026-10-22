#include "craft_planning/planning_master.hpp"
#include "craft_planning/planning_config.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <craft_perception/msg/knob_observation_array.hpp>
#include <craft_perception/msg/puck_observation_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
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
    bool retryable = false;
    Path path;
    Joints joints;
    std::string error;
    Odom start;
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
    Sample<Odom> odom;
    Sample<Joints> joints;
    Sample<Points> lidar, vision_obstacles;
    Sample<ControlStatus> control;
    Sample<TaskResult> task_result;
    std::map<std::string, std::vector<Observation>> observations;
    std::optional<PlanningStep> step;
    std::deque<PlanningStep> remaining;
    std::string last_mission, readiness_reason;
    std::deque<std::string> request_keys;
    std::future<Computation> worker;
    std::optional<Computation> result;
    Path active_path;
    Pose resolved_goal;
    uint64_t next_id = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
    uint64_t revision = 0;
    double command_at = 0, last_report = -1, compute_at = 0, last_replan = 0, stop_at = 0;
    double retry_compute_at = 0;
    bool dispatched = false, fault_reported = false, reset_requested = false;
    bool waiting_for_path = false, route_paused = false;
    rclcpp::Publisher<TaskCommand>::SharedPtr command_pub;
    rclcpp::Publisher<Odom>::SharedPtr odom_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
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
      TaskCommand c; c.header.stamp = node.now(); c.operation = TaskCommand::STOP;
      c.task_id = dispatched && step ? step->task_id : 0;
      command_pub->publish(c); stop();
    }
    bool control_fresh(double now) const {
      return control.fresh(now, config.fsm.feedback_timeout_s) &&
        is_fresh_input(control.value.header, config.fsm.feedback_timeout_s);
    }
    bool system_ready(double now) const {
      return control_fresh(now) && control.value.ready && !control.value.fault &&
        odom.fresh(now, config.odom_timeout_s) && is_fresh_input(odom.value.header, config.odom_timeout_s) &&
        joints.fresh(now, config.joint_timeout_s) && is_fresh_input(joints.value.header, config.joint_timeout_s) &&
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
        sample.valid = false;
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
        return points;
    }
    bool valid_request(const PlanningStep &r) const {
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
        if (!control_fresh(now) || !control.value.ready || control.value.held)
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
        if (active() && value.generation == fsm.generation())
        {
            result = std::move(value);
        }
    }
    void compute(double now)
    {
        result.reset();
        compute_at = now;
        if (step->kind != StepKind::Navigate && step->kind != StepKind::Tcp) {
            Computation out; out.generation = fsm.generation(); out.ok = valid_request(*step);
            result = out; return;
        }
        const auto generation = fsm.generation();
        const auto goal = resolved_goal;
        const auto kind = step->kind;
        const auto current_odom = odom.value;
        const auto current_joints = joints.value;
        const auto points = obstacles(now);
        worker = std::async(std::launch::async, [this, generation, goal, kind, current_odom, current_joints, points] {
            Computation output;
            output.generation = generation;
            output.start = current_odom;
            try
            {
                if (kind == StepKind::Navigate)
                {
                    const auto plan = path_planner.plan(current_odom, goal, points);
                    output.ok = plan.ok;
                    output.retryable = plan.retryable;
                    output.path = plan.path;
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
    void command(double now, bool update = false) {
      TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id;
      c.revision = ++revision; c.operation = update ? TaskCommand::UPDATE_PATH : TaskCommand::EXECUTE;
      c.kind = static_cast<uint8_t>(step->kind);
      switch (step->kind) {
        case StepKind::Navigate:
          c.path = result->path; c.path.header.stamp = node.now(); c.speed = config.cruise_speed;
          c.require_goal_heading = config.goal_heading_required;
          active_path = c.path; route_paused = false;
          break;
        case StepKind::Tcp: c.joints = result->joints; c.joints.header.stamp = node.now(); break;
        case StepKind::Clock: c.clock_slot = step->slot; break;
        case StepKind::Gripper: c.close = step->close; break;
        case StepKind::Wait: c.duration_s = step->duration_s; break;
      }
      command_pub->publish(c);
      if (!update) {command_at = now; dispatched = true; task_result.valid = false;}
      last_replan = now; result.reset();
    }
    void pause_navigation(double now) {
      if (route_paused || !step || step->kind != StepKind::Navigate || active_path.poses.empty()) return;
      TaskCommand c; c.header.stamp = node.now(); c.task_id = step->task_id;
      c.revision = ++revision; c.operation = TaskCommand::UPDATE_PATH; c.kind = TaskCommand::NAVIGATE;
      c.path = active_path; c.path.header.stamp = node.now(); c.speed = 0.0;
      c.require_goal_heading = config.goal_heading_required;
      command_pub->publish(c); route_paused = true; last_replan = now;
    }
    void report(double now) {
      std_msgs::msg::String state; state.data = fsm.label(); state_pub->publish(state);
      if (fsm.state() == SystemState::Fault && !fault_reported) {
        RCLCPP_ERROR(node.get_logger(), "%s", fsm.reason().c_str()); fault_reported = true;
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
          {"path_revision", std::to_string(revision)}, {"waiting_for", readiness_reason}}) {
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
    p.subscribe<Odom>("/perception/state/odom", [&p](const Odom &msg) {
        Pose pose;
        pose.header = msg.header;
        pose.pose = msg.pose.pose;
        p.odom.valid = false;
        if (valid_pose(pose) && msg.header.frame_id == p.config.path.map_frame &&
            p.is_fresh_input(msg.header, p.config.odom_timeout_s))
        {
            p.odom.observe(msg, seconds());
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
    if (p.odom.fresh(now, p.config.odom_timeout_s) &&
        p.is_fresh_input(p.odom.value.header, p.config.odom_timeout_s)) p.odom_pub->publish(p.odom.value);
    if (p.reset_requested && p.control_fresh(now) && p.control.value.held && p.control.value.stopped &&
        !p.worker.valid()) {
      p.path_planner.configure(p.config.path); p.arm_planner.configure(p.config.arm);
      if (p.fsm.reset(true, now)) {
        p.step.reset(); p.remaining.clear(); p.result.reset(); p.dispatched = false;
        p.task_result.valid = false; p.fault_reported = false; p.last_mission.clear();
        p.active_path = Path{}; p.route_paused = false; p.waiting_for_path = false;
      }
      p.reset_requested = false;
    }
    if (p.fsm.phase() == RunPhase::AwaitRequest && !p.remaining.empty()) {
      p.step = std::move(p.remaining.front()); p.remaining.pop_front();
      p.result.reset(); p.task_result.valid = false; p.dispatched = false; p.revision = 0;
      p.waiting_for_path = false; p.route_paused = false; p.active_path = Path{};
      p.fsm.start(p.step->kind, now);
    }
    const auto phase = p.fsm.phase();
    const bool needs_release = phase == RunPhase::PrepareTask || phase == RunPhase::Compute || phase == RunPhase::Execute;
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
    if (phase == RunPhase::Execute) {
      input.feedback_fresh = input.feedback_fresh && p.sensors_fresh(now) &&
        (now - p.command_at <= p.config.fsm.feedback_timeout_s ||
         p.task_result.fresh(now, p.config.fsm.feedback_timeout_s));
      if (p.task_result.valid) {
        const auto status = p.task_result.value.status;
        input.completed = status == TaskResult::SUCCEEDED;
        if (status == TaskResult::FAILED || status == TaskResult::REJECTED || status == TaskResult::STOPPED ||
            status == TaskResult::STOPPING) {
          input.healthy = false; input.error = "control task: " + p.task_result.value.reason;
        }
      }
      // Replanning stays in Run.Execute and preserves the task ID.
      if (input.healthy && input.feedback_fresh && !input.completed && p.step->kind == StepKind::Navigate) {
        if (p.result) {
          if (!p.result->ok && p.result->retryable) {
            p.readiness_reason = "route unavailable; pausing navigation and retrying";
            p.pause_navigation(now); p.result.reset(); p.last_replan = now;
          } else if (!p.result->ok) {input.healthy = false; input.error = "replan failed: " + p.result->error;}
          else p.command(now, true);
        } else if (p.worker.valid() && now - p.compute_at >= p.config.fsm.compute_timeout_s) {
          input.healthy = false; input.error = "replan timeout";
        } else if (!p.worker.valid() && now - p.last_replan >= p.config.replan_period_s) p.compute(now);
      }
    }
    if (phase == RunPhase::Compute && p.result) {
      if (p.step->kind == StepKind::Navigate && !p.result->ok && p.result->retryable) {
        p.readiness_reason = "route unavailable; waiting to retry";
        p.result.reset(); p.waiting_for_path = true; p.retry_compute_at = now + p.config.replan_period_s;
      } else {
        input.compute_done = true; input.compute_ok = p.result->ok; input.error = p.result->error;
      }
    }
    if (phase == RunPhase::Compute && p.waiting_for_path && !p.worker.valid() && now >= p.retry_compute_at) {
      p.waiting_for_path = false;
      if (p.fsm.retry_compute(now)) p.compute(now);
    }
    const auto output = p.fsm.tick(input, now);
    if (output.start_compute) p.compute(now);
    if (output.send_command) p.command(now);
    if (output.stop) p.stop();
    if (p.fsm.state() == SystemState::Fault) {
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
