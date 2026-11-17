#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <craft_interfaces/msg/control_status.hpp>
#include <craft_interfaces/msg/task_command.hpp>
#include <craft_interfaces/msg/task_result.hpp>
#include <craft_perception/msg/puck_observation_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace puck::execute {

// Main signal handler exposes cancellation to transport waits.
bool is_interrupted();

// ROS transport for execute.py's Runner. Spin the node concurrently with calls
// to send/wait/lease_camera, as the Python program spins on a separate thread.
class Runner final : public rclcpp::Node {
 public:
  using TaskCommand = craft_interfaces::msg::TaskCommand;
  using TaskResult = craft_interfaces::msg::TaskResult;
  explicit Runner(bool planned);

  uint64_t send(TaskCommand command);
  std::pair<std::string, std::string> wait(uint64_t id, double timeout_s = 35.0);
  bool drive(const std::string &box, bool reverse, double shift_m = 0);
  void stop();
  void set_hold(bool value);
  void lease_camera(const std::string &role);
  void release_camera(const std::string &role);
  void publish_observation(const std::string &role,
                           craft_perception::msg::PuckObservationArray message);
  size_t observation_subscribers(const std::string &role) const;
  std::optional<std::string> planning_state() const;
  std::optional<craft_interfaces::msg::ControlStatus> status() const;
  std::optional<sensor_msgs::msg::JointState> joints() const;
  std::optional<bool> grasped() const;
  bool planned() const { return planned_; }

 private:
  bool planned_;
  mutable std::mutex mutex_;
  std::optional<std::string> planning_state_;
  std::optional<TaskResult> result_;
  std::optional<craft_interfaces::msg::ControlStatus> status_;
  std::optional<sensor_msgs::msg::JointState> joints_;
  std::optional<bool> grasped_;
  std::map<std::string, bool> camera_ack_;
  std::optional<std::string> camera_leased_;
  std::atomic<bool> hold_value_{false};
  uint64_t task_id_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_;
  rclcpp::Publisher<TaskCommand>::SharedPtr cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> camera_pause_;
  std::map<std::string, rclcpp::Publisher<craft_perception::msg::PuckObservationArray>::SharedPtr> puck_pubs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

}  // namespace puck::execute
