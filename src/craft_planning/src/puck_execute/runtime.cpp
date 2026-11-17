#include "runtime.hpp"

#include <ctime>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
namespace puck::execute {

Runner::Runner(bool planned) : Node("puck_pick_execute"), planned_(planned), task_id_(std::time(nullptr)) {
  puck_pubs_["uppercam"] = create_publisher<craft_perception::msg::PuckObservationArray>("/perception/pucks/picker_upper", 10);
  puck_pubs_["d435"] = create_publisher<craft_perception::msg::PuckObservationArray>("/perception/pucks/picker_grip", 10);
  for (const auto &[role, prefix] : std::vector<std::pair<std::string, std::string>>{
           {"uppercam", "/perception/uppercam"}, {"d435", "/vision/d435"}}) {
    camera_ack_[role] = false;
    camera_pause_[role] = create_publisher<std_msgs::msg::Bool>(prefix + "/pick_pause", 10);
    subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(prefix + "/pick_paused", 10,
      [this, role](const std_msgs::msg::Bool &m) { std::lock_guard<std::mutex> lock(mutex_); camera_ack_[role] = m.data; }));
  }
  timers_.push_back(create_wall_timer(200ms, [this] {
    std::optional<std::string> role;
    { std::lock_guard<std::mutex> lock(mutex_); role = camera_leased_; }
    if (role) { std_msgs::msg::Bool m; m.data = true; camera_pause_.at(*role)->publish(m); }
  }));
  lifecycle_ = create_publisher<std_msgs::msg::String>("/planning/command", 10);
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  subscriptions_.push_back(create_subscription<std_msgs::msg::String>("/planning/state", state_qos,
    [this](const std_msgs::msg::String &m) { std::lock_guard<std::mutex> lock(mutex_); planning_state_ = m.data; }));
  cmd_ = create_publisher<TaskCommand>(planned_ ? "/planning/pick_step" : "/planning/task_command", 10);
  if (!planned_) {
    hold_ = create_publisher<std_msgs::msg::Bool>("/planning/hold", 10);
    timers_.push_back(create_wall_timer(100ms, [this] { std_msgs::msg::Bool m; m.data = hold_value_; hold_->publish(m); }));
  }
  subscriptions_.push_back(create_subscription<sensor_msgs::msg::JointState>("/control/arm_joints", 10,
    [this](const sensor_msgs::msg::JointState &m) { std::lock_guard<std::mutex> lock(mutex_); joints_ = m; }));
  subscriptions_.push_back(create_subscription<TaskResult>(planned_ ? "/planning/pick_step_result" : "/control/task_result", 10,
    [this](const TaskResult &m) { std::lock_guard<std::mutex> lock(mutex_); result_ = m; }));
  subscriptions_.push_back(create_subscription<craft_interfaces::msg::ControlStatus>("/control/status", 10,
    [this](const craft_interfaces::msg::ControlStatus &m) { std::lock_guard<std::mutex> lock(mutex_); status_ = m; }));
  subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>("/control/gripper_grasped", 10,
    [this](const std_msgs::msg::Bool &m) { std::lock_guard<std::mutex> lock(mutex_); grasped_ = m.data; }));
}

uint64_t Runner::send(TaskCommand command) {
  if (planned_) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (true) {
      if (is_interrupted()) throw std::runtime_error("interrupted");
      auto state = planning_state();
      if (state && *state == "Run.AwaitRequest") break;
      if ((state && (*state == "Fault" || *state == "Ready")) || std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("planning cannot accept a pick step: " + state.value_or("None"));
      std::this_thread::sleep_for(20ms);
    }
  }
  command.header.stamp = now();
  command.task_id = ++task_id_;
  command.revision = 1;
  command.operation = TaskCommand::EXECUTE;
  { std::lock_guard<std::mutex> lock(mutex_); result_.reset(); }
  cmd_->publish(command);
  return command.task_id;
}

std::pair<std::string, std::string> Runner::wait(uint64_t id, double timeout_s) {
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < until) {
    if (is_interrupted()) { stop(); return {"STOPPED", "interrupted"}; }
    std::optional<TaskResult> result;
    { std::lock_guard<std::mutex> lock(mutex_); result = result_; }
    if (result && result->task_id == id && result->status != TaskResult::RUNNING && result->status != TaskResult::STOPPING) {
      static const std::map<unsigned, std::string> names{{0,"RUNNING"},{1,"SUCCEEDED"},{2,"FAILED"},{3,"STOPPING"},{4,"STOPPED"},{5,"REJECTED"}};
      auto it = names.find(result->status);
      return {it == names.end() ? std::to_string(result->status) : it->second, result->reason};
    }
    std::this_thread::sleep_for(50ms);
  }
  return {"TIMEOUT", "no terminal result"};
}

// shift_m (re-dock only): sideways from this box's last dock [m], + left; planning reads it from tracking_reference_x.
bool Runner::drive(const std::string &box, bool reverse, double shift_m) {
  TaskCommand c; c.kind = TaskCommand::NAVIGATE; c.header.frame_id = box; c.speed = reverse ? -0.28 : 0.0;
  if (!reverse) c.tracking_reference_x = shift_m;
  auto [state, reason] = wait(send(c), 180.0);
  std::cout << "   -> " << state << ": " << reason << '\n';
  return state == "SUCCEEDED";
}

void Runner::stop() {
  if (planned_) { std_msgs::msg::String m; m.data = "STOP"; lifecycle_->publish(m); }
  else { TaskCommand c; c.operation = TaskCommand::STOP; c.task_id = 0; cmd_->publish(c); hold_value_ = true; }
}
void Runner::set_hold(bool value) { hold_value_ = value; }
void Runner::lease_camera(const std::string &role) {
  auto pub = camera_pause_.at(role);
  if (!planned_) return;
  // A fresh puck_execute may not have discovered the camera driver yet (2026-11-16 14:25: driver up 85 s, failed at
  // once): wait up to 5 s for its pick_pause subscription before calling it missing.
  const auto found_by = std::chrono::steady_clock::now() + 5s;
  while (!pub->get_subscription_count() && std::chrono::steady_clock::now() < found_by && !is_interrupted())
    std::this_thread::sleep_for(50ms);
  if (!pub->get_subscription_count()) throw std::runtime_error(role + ": project camera driver with pick_pause support is not running");
  { std::lock_guard<std::mutex> lock(mutex_); camera_ack_.at(role) = false; camera_leased_ = role; }
  std_msgs::msg::Bool m; m.data = true; pub->publish(m);
  const auto until = std::chrono::steady_clock::now() + 5s;
  while (true) {
    if (is_interrupted()) { release_camera(role); throw std::runtime_error("interrupted"); }
    { std::lock_guard<std::mutex> lock(mutex_); if (camera_ack_.at(role)) return; }
    if (std::chrono::steady_clock::now() >= until) {
      release_camera(role);
      throw std::runtime_error(role + ": camera release acknowledgement timed out");
    }
    std::this_thread::sleep_for(20ms);
  }
}
void Runner::release_camera(const std::string &role) {
  if (!planned_) return;
  { std::lock_guard<std::mutex> lock(mutex_); camera_leased_.reset(); }
  std_msgs::msg::Bool m; m.data = false; camera_pause_.at(role)->publish(m);
}
void Runner::publish_observation(const std::string &role, craft_perception::msg::PuckObservationArray message) { puck_pubs_.at(role)->publish(message); }
size_t Runner::observation_subscribers(const std::string &role) const { return puck_pubs_.at(role)->get_subscription_count(); }
std::optional<std::string> Runner::planning_state() const { std::lock_guard<std::mutex> lock(mutex_); return planning_state_; }
std::optional<craft_interfaces::msg::ControlStatus> Runner::status() const { std::lock_guard<std::mutex> lock(mutex_); return status_; }
std::optional<sensor_msgs::msg::JointState> Runner::joints() const { std::lock_guard<std::mutex> lock(mutex_); return joints_; }
std::optional<bool> Runner::grasped() const { std::lock_guard<std::mutex> lock(mutex_); return grasped_; }
}  // namespace puck::execute
