#pragma once
#include <craft_interfaces/msg/task_command.hpp>
#include <craft_interfaces/msg/task_result.hpp>
#include <array>
#include <cstdint>
namespace craft_control {
// One row of the controller matrix for the current task. ROS belongs to ControlMaster.
class TaskTracker {
public:
  using Result = craft_interfaces::msg::TaskResult;
  using Command = craft_interfaces::msg::TaskCommand;
  enum Column {Arm, Lm, Wheel, Clock, Gripper};
  bool running() const {return result.task_id && result.status == Result::RUNNING;}
  bool stopping() const {return result.task_id && result.status == Result::STOPPING;}
  void begin(const Command &c, const std::array<bool, 5> &participants) {
    result = Result{}; result.task_id = c.task_id; result.revision = c.revision;
    kind = c.kind; result.status = Result::RUNNING;
    for (size_t i = 0; i < participants.size(); ++i)
      result.controllers[i] = participants[i] ? Result::ACTIVE : Result::UNUSED;
  }
  void observe(const std::array<bool, 5> &done, bool wait_done = true) {
    if (!running()) return;
    bool all = wait_done;
    for (size_t i = 0; i < done.size(); ++i) {
      if (result.controllers[i] == Result::UNUSED) continue;
      result.controllers[i] = done[i] ? Result::DONE : Result::ACTIVE;
      all = all && done[i];
    }
    if (all) result.status = Result::SUCCEEDED;
  }
  void stop() {
    if (!result.task_id || result.status == Result::FAILED) return;
    result.status = Result::STOPPING;
    for (auto &s : result.controllers) if (s != Result::UNUSED) s = Result::HALTING;
  }
  void stopped() {
    if (!stopping()) return;
    result.status = Result::STOPPED;
    for (auto &s : result.controllers) if (s != Result::UNUSED) s = Result::HALTED;
  }
  void fail(const std::string &reason) {
    if (!running() && !stopping()) return;
    result.status = Result::FAILED; result.reason = reason;
    for (auto &s : result.controllers) if (s != Result::UNUSED) s = Result::ERROR;
  }
  Result result;
  uint8_t kind = 0;
};
}
