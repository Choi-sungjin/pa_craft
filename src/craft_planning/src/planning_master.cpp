#include "craft_planning/planning_master.hpp"
#include "craft_planning/planning_config.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <craft_perception/msg/knob_observation_array.hpp>
#include <craft_perception/msg/puck_observation_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
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

namespace craft_planning
{
namespace
{
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
double yaw(const geometry_msgs::msg::Quaternion &q)
{
    const double n = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    const double x = q.x / n, y = q.y / n, z = q.z / n, w = q.w / n;
    return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}
double angle_error(double a, double b)
{
    return std::abs(std::remainder(a - b, 2 * std::acos(-1.0)));
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
    Path path;
    Joints joints;
    std::string error;
    Odom start;
};
const char *state_name(PlanningState state)
{
    switch (state)
    {
    case PlanningState::Idle:
        return "IDLE";
    case PlanningState::Waiting:
        return "WAITING";
    case PlanningState::Computing:
        return "COMPUTING";
    case PlanningState::Executing:
        return "EXECUTING";
    case PlanningState::Succeeded:
        return "SUCCEEDED";
    case PlanningState::Fault:
        return "FAULT";
    case PlanningState::Cancelled:
        return "CANCELLED";
    }
    return "FAULT";
}
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
    Sample<Path> tracking;
    Sample<bool> held;
    Sample<int> arm_state, wheel_state, clock_state, clock_slot;
    CompletionGate arm_done, lm_done, wheel_done, clock_done, gripper_done;
    std::map<std::string, std::vector<Observation>> observations;
    std::optional<PlanningStep> step;
    std::deque<PlanningStep> remaining;
    std::string last_mission, readiness_reason;
    std::future<Computation> worker;
    std::optional<Computation> result;
    Path active_path;
    Pose resolved_goal;
    double command_at = 0, last_report = -1;
    bool tracking_checked = false, tracking_safe = false;
    std::size_t obstacles_revision = 0, checked_revision = 0;
    bool fault_reported = false;
    enum class Release
    {
        Hold,
        Release,
        Ready
    };
    Release release = Release::Hold;
    double handshake_at = 0;
    int64_t last_path_stamp_ns = 0;

    rclcpp::Publisher<Path>::SharedPtr path_pub;
    rclcpp::Publisher<Odom>::SharedPtr odom_pub;
    rclcpp::Publisher<Joints>::SharedPtr arm_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub, hold_pub;
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
    void speed(double value)
    {
        std_msgs::msg::Float64 msg;
        msg.data = value;
        speed_pub->publish(msg);
    }
    void stop()
    {
        hold(true);
        speed(0.0);
        Path empty;
        empty.header.frame_id = config.path.map_frame;
        empty.header.stamp = node.now();
        path_pub->publish(empty);
        active_path = Path{};
        tracking.valid = false;
        tracking_checked = tracking_safe = false;
    }
    bool active() const
    {
        return fsm.state() == PlanningState::Waiting || fsm.state() == PlanningState::Computing ||
               fsm.state() == PlanningState::Executing;
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
        ++obstacles_revision;
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
    void accept(PlanningStep request, std::deque<PlanningStep> rest = {})
    {
        if (fsm.state() == PlanningState::Fault)
        {
            RCLCPP_WARN(node.get_logger(), "fault latched; send RESET before a new goal");
            return;
        }
        if (request.kind == StepKind::Navigate)
        {
            if (!valid_pose(request.goal) || request.goal.header.frame_id != config.path.map_frame)
            {
                fsm.fail("invalid goal pose/frame", seconds());
                stop();
                return;
            }
        }
        else if (request.kind == StepKind::Tcp)
        {
            if (!valid_pose(request.goal) || request.goal.header.frame_id != config.arm.base_frame)
            {
                fsm.fail("invalid goal pose/frame", seconds());
                stop();
                return;
            }
        }
        if (active())
        {
            fsm.cancel(seconds());
        }
        stop();
        step = std::move(request);
        remaining = std::move(rest);
        result.reset();
        fault_reported = false;
        release = Release::Hold;
        handshake_at = seconds();
        fsm.start(step->kind, seconds());
    }
    void mission(const std::string &name)
    {
        if (name == "STOP" || name == "CANCEL" || name == "RESET")
        {
            if (name == "RESET" || active())
            {
                fsm.cancel(seconds());
            }
            step.reset();
            remaining.clear();
            result.reset();
            stop();
            return;
        }
        if (name == "INIT")
        {
            last_mission.clear();
            return;
        }
        if (name == last_mission)
        {
            return;
        }
        const auto found = config.missions.find(name);
        if (found == config.missions.end() || found->second.empty())
        {
            RCLCPP_WARN(node.get_logger(), "mission '%s' has no configured steps", name.c_str());
            return;
        }
        if (fsm.state() == PlanningState::Fault)
        {
            return;
        }
        last_mission = name;
        std::deque<PlanningStep> rest(found->second.begin() + 1, found->second.end());
        accept(found->second.front(), std::move(rest));
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
    bool board_fresh(double now) const
    {
        const double timeout = config.fsm.feedback_timeout_s;
        if (!held.fresh(now, timeout) || held.value)
        {
            return false;
        }
        if (step->kind == StepKind::Navigate)
        {
            return wheel_state.fresh(now, timeout);
        }
        if (step->kind == StepKind::Clock)
        {
            return clock_state.fresh(now, timeout);
        }
        if (step->kind == StepKind::Tcp || step->kind == StepKind::Gripper)
        {
            return arm_state.fresh(now, timeout);
        }
        return true;
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
        if (!board_fresh(now))
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
                    output.path = plan.path;
                    output.error = plan.error;
                    if (output.ok && output.path.poses.size() > 1)
                    {
                        const auto &a = output.path.poses[0].pose.position;
                        const auto &b = output.path.poses[1].pose.position;
                        if (angle_error(std::atan2(b.y - a.y, b.x - a.x), yaw(current_odom.pose.pose.orientation)) >
                            config.heading_tolerance)
                        {
                            output.ok = false;
                            output.error = "initial base rotation required; controller has no in-place-turn input";
                        }
                    }
                    if (output.ok && config.goal_heading_required && output.path.poses.size() > 1)
                    {
                        const auto &a = output.path.poses[output.path.poses.size() - 2].pose.position;
                        const auto &b = output.path.poses.back().pose.position;
                        if (angle_error(std::atan2(b.y - a.y, b.x - a.x), yaw(goal.pose.orientation)) >
                            config.heading_tolerance)
                        {
                            output.ok = false;
                            output.error = "final base heading requires unsupported alignment";
                        }
                    }
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
    void command(double now)
    {
        command_at = now;
        switch (step->kind)
        {
        case StepKind::Navigate:
            active_path = result->path;
            // This stamp identifies a path revision, including while /clock is paused.
            last_path_stamp_ns = std::max(node.now().nanoseconds(), last_path_stamp_ns + 1);
            active_path.header.stamp = rclcpp::Time(last_path_stamp_ns, node.get_clock()->get_clock_type());
            for (auto &pose : active_path.poses)
            {
                pose.header = active_path.header;
            }
            tracking.valid = false;
            tracking_checked = tracking_safe = false;
            wheel_done.arm();
            speed(0.0);
            path_pub->publish(active_path);
            break;
        case StepKind::Tcp: {
            arm_done.arm();
            lm_done.arm();
            auto target = result->joints;
            target.header.stamp = node.now();
            arm_pub->publish(target);
            break;
        }
        case StepKind::Gripper: {
            gripper_done.arm();
            std_msgs::msg::Bool msg;
            msg.data = step->close;
            gripper_pub->publish(msg);
            break;
        }
        case StepKind::Clock: {
            clock_done.arm();
            std_msgs::msg::Int32 msg;
            msg.data = step->slot;
            clock_pub->publish(msg);
            break;
        }
        case StepKind::Wait:
            break;
        }
    }
    bool complete(double now) const
    {
        const double timeout = config.fsm.feedback_timeout_s;
        switch (step->kind)
        {
        case StepKind::Navigate: {
            if (!wheel_done.complete(now, timeout) || !odom.fresh(now, config.odom_timeout_s))
            {
                return false;
            }
            const auto &p = odom.value.pose.pose.position;
            const auto &g = resolved_goal.pose.position;
            return std::hypot(p.x - g.x, p.y - g.y) <= config.goal_tolerance &&
                   (!config.goal_heading_required ||
                    angle_error(yaw(odom.value.pose.pose.orientation), yaw(resolved_goal.pose.orientation)) <=
                        config.heading_tolerance);
        }
        case StepKind::Tcp:
            return arm_done.complete(now, timeout) && lm_done.complete(now, timeout);
        case StepKind::Gripper:
            return gripper_done.complete(now, timeout);
        case StepKind::Clock:
            return clock_done.complete(now, timeout) && clock_slot.fresh(now, timeout) &&
                   clock_slot.value == step->slot && clock_state.value == 0;
        case StepKind::Wait:
            return now - command_at >= step->duration_s;
        }
        return false;
    }
    void report(double now)
    {
        const auto current = fsm.state();
        std_msgs::msg::String state;
        state.data = state_name(current);
        state_pub->publish(state);
        if (current == PlanningState::Fault && !fault_reported)
        {
            RCLCPP_ERROR(node.get_logger(), "planning fault: %s (%s)", fsm.reason().c_str(), readiness_reason.c_str());
            fault_reported = true;
        }
        if (now - last_report < 1.0)
        {
            return;
        }
        last_report = now;
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = node.now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "planning/state_machine";
        status.hardware_id = "planning_master";
        status.level = current == PlanningState::Fault ? 2 : current == PlanningState::Waiting ? 1 : 0;
        status.message = std::string(state_name(current)) + ": " + fsm.reason();
        for (const auto &item : std::map<std::string, std::string>{{"step", step ? step->id : ""},
                                                                   {"waiting_for", readiness_reason},
                                                                   {"generation", std::to_string(fsm.generation())}})
        {
            diagnostic_msgs::msg::KeyValue value;
            value.key = item.first;
            value.value = item.second;
            status.values.push_back(value);
        }
        array.status.push_back(status);
        diagnostics_pub->publish(array);
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
}

void PlanningMaster::connect()
{
    auto &p = *impl_;
    p.tf = std::make_shared<tf2_ros::Buffer>(get_clock());
    p.listener = std::make_shared<tf2_ros::TransformListener>(*p.tf);
    p.path_pub = create_publisher<Path>("/planning/target_path", 10);
    p.odom_pub = create_publisher<Odom>("/planning/odom", 10);
    p.arm_pub = create_publisher<Joints>("/planning/target_arm_joints", 10);
    p.speed_pub = create_publisher<std_msgs::msg::Float64>("/planning/target_v", 10);
    p.clock_pub = create_publisher<std_msgs::msg::Int32>("/planning/target_clock_slot", 10);
    p.gripper_pub = create_publisher<std_msgs::msg::Bool>("/planning/gripper_command", 10);
    p.hold_pub = create_publisher<std_msgs::msg::Bool>("/planning/hold", 10);
    p.state_pub = create_publisher<std_msgs::msg::String>("/planning/state", rclcpp::QoS(1).transient_local());
    p.diagnostics_pub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/planning/diagnostics", 10);

    p.subscribe<std_msgs::msg::String>("/planning/mission",
                                       [&p](const std_msgs::msg::String &msg) { p.mission(msg.data); });
    p.subscribe<Pose>("/planning/target_base_pose", [&p](const Pose &msg) {
        PlanningStep step;
        step.id = "base_goal";
        step.kind = StepKind::Navigate;
        step.goal = msg;
        p.accept(step);
    });
    p.subscribe<Pose>("/planning/target_tcp_pose", [&p](const Pose &msg) {
        PlanningStep step;
        step.id = "tcp_goal";
        step.kind = StepKind::Tcp;
        step.goal = msg;
        p.accept(step);
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
    auto state = [&p](const std::string &topic, Sample<int> &sample) {
        p.subscribe<std_msgs::msg::Int32>(topic, [&sample](const std_msgs::msg::Int32 &msg) {
            sample.valid = false;
            if (msg.data >= 0 && msg.data <= 2)
            {
                sample.observe(msg.data, seconds());
            }
        });
    };
    state("/control/arm_lm_state", p.arm_state);
    state("/control/wheel_state", p.wheel_state);
    state("/control/clock_state", p.clock_state);
    p.subscribe<std_msgs::msg::Int32>("/control/clock_slot", [&p](const std_msgs::msg::Int32 &msg) {
        p.clock_slot.valid = false;
        if (msg.data >= 0 && msg.data <= 12)
        {
            p.clock_slot.observe(msg.data, seconds());
        }
    });
    p.subscribe<std_msgs::msg::Bool>("/control/planning_held",
                                     [&p](const std_msgs::msg::Bool &msg) { p.held.observe(msg.data, seconds()); });
    auto completed = [&p](const std::string &topic, CompletionGate &gate) {
        p.subscribe<std_msgs::msg::Bool>(
            topic, [&gate](const std_msgs::msg::Bool &msg) { gate.observe(msg.data, seconds()); });
    };
    completed("/control/arm_target_reached", p.arm_done);
    completed("/control/lm_target_reached", p.lm_done);
    completed("/control/wheel_target_reached", p.wheel_done);
    completed("/control/clock_target_reached", p.clock_done);
    completed("/control/gripper_target_reached", p.gripper_done);
    p.subscribe<Path>("/control/tracking_path", [&p](const Path &msg) {
        if (!p.active_path.poses.empty() && msg.header.stamp == p.active_path.header.stamp &&
            msg.header.frame_id == p.config.path.map_frame && !msg.poses.empty())
        {
            const auto same = [](const auto &a, const auto &b) { return std::hypot(a.x - b.x, a.y - b.y) <= 1e-6; };
            if (same(msg.poses.front().pose.position, p.active_path.poses.front().pose.position) &&
                same(msg.poses.back().pose.position, p.active_path.poses.back().pose.position))
            {
                p.tracking.observe(msg, seconds());
                p.tracking_checked = false;
            }
        }
    });
    p.timer = create_wall_timer(std::chrono::duration<double>(1.0 / p.config.loop_hz), [this] { planning_loop(); });
    RCLCPP_INFO(get_logger(), "planning ready: C++ FSM, static-grid path planner and headless URDF IK");
}

void PlanningMaster::planning_loop()
{
    auto &p = *impl_;
    const double now = seconds();
    try
    {
        p.collect();
        if (p.fsm.state() == PlanningState::Succeeded && !p.remaining.empty())
        {
            const auto next = p.remaining.front();
            p.remaining.pop_front();
            p.step = next;
            p.result.reset();
            p.fsm.start(next.kind, now);
            p.release = Impl::Release::Hold;
            p.handshake_at = now;
        }
        if (p.odom.fresh(now, p.config.odom_timeout_s) &&
            p.is_fresh_input(p.odom.value.header, p.config.odom_timeout_s))
        {
            p.odom_pub->publish(p.odom.value);
        }
        if (!p.active() || !p.step)
        {
            p.stop();
            p.report(now);
            return;
        }
        // A true acknowledgement is a barrier: clear the previous goal before release.
        if (p.release == Impl::Release::Hold && p.held.fresh(now, p.config.fsm.feedback_timeout_s) &&
            p.held.at >= p.handshake_at && p.held.value)
        {
            p.release = Impl::Release::Release;
            p.handshake_at = now;
        }
        p.hold(p.release == Impl::Release::Hold);
        if (p.release == Impl::Release::Release && p.held.fresh(now, p.config.fsm.feedback_timeout_s) &&
            p.held.at >= p.handshake_at && !p.held.value)
        {
            p.release = Impl::Release::Ready;
        }
        if (p.release != Impl::Release::Ready)
        {
            p.readiness_reason = "waiting for control hold/release acknowledgement";
        }
        FsmInput input;
        input.ready = p.fsm.state() == PlanningState::Waiting && p.release == Impl::Release::Ready && p.ready(now);
        input.feedback_fresh = p.release == Impl::Release::Ready && p.board_fresh(now) && p.sensors_fresh(now);
        for (const auto *board : {&p.arm_state, &p.wheel_state, &p.clock_state})
        {
            if (board->valid && board->value == 2)
            {
                input.healthy = false;
                input.error = "control board fault";
            }
        }
        if (p.result)
        {
            input.compute_done = true;
            input.compute_ok = p.result->ok;
            input.error = p.result->error;
            if (input.compute_ok && p.step->kind == StepKind::Navigate)
            {
                const auto &a = p.result->start.pose.pose;
                const auto &b = p.odom.value.pose.pose;
                if (std::hypot(a.position.x - b.position.x, a.position.y - b.position.y) > p.config.goal_tolerance ||
                    angle_error(yaw(a.orientation), yaw(b.orientation)) > p.config.heading_tolerance)
                {
                    input.compute_ok = false;
                    input.error = "base moved while planning; request a new goal";
                }
            }
        }
        if (p.fsm.state() == PlanningState::Executing)
        {
            input.completed = p.complete(now);
            if (p.step->kind == StepKind::Navigate)
            {
                Path footprint;
                footprint.header = p.odom.value.header;
                Pose current;
                current.header = footprint.header;
                current.pose = p.odom.value.pose.pose;
                footprint.poses.push_back(current);
                if (p.odom.valid && !p.path_planner.collision_free(footprint, p.obstacles(now)))
                {
                    input.healthy = false;
                    input.error = "current robot footprint intersects obstacle map";
                }
                if (p.tracking.valid && (!p.tracking_checked || p.checked_revision != p.obstacles_revision))
                {
                    p.tracking_safe = p.path_planner.collision_free(p.tracking.value, p.obstacles(now));
                    p.tracking_checked = true;
                    p.checked_revision = p.obstacles_revision;
                    if (!p.tracking_safe)
                    {
                        input.healthy = false;
                        input.error = "controller tracking path intersects obstacle map";
                    }
                }
                if (!p.tracking.valid && now - p.command_at > p.config.fsm.feedback_timeout_s)
                {
                    input.feedback_fresh = false;
                    input.error = "no matching controller tracking path";
                }
            }
        }
        const auto output = p.fsm.tick(input, now);
        if (output.start_compute)
        {
            p.compute(now);
        }
        if (output.send_command)
        {
            p.command(now);
        }
        if (output.stop)
        {
            p.stop();
        }
        else if (p.fsm.state() == PlanningState::Executing && p.step->kind == StepKind::Navigate)
        {
            p.path_pub->publish(p.active_path);
            p.speed(p.tracking_safe && !input.completed ? p.config.cruise_speed : 0.0);
        }
        else
        {
            p.speed(0.0);
        }
        p.report(now);
    }
    catch (const std::exception &e)
    {
        p.fsm.fail(e.what(), now);
        p.remaining.clear();
        p.stop();
        p.report(now);
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
