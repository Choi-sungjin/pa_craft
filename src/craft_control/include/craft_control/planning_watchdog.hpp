#pragma once

#include <chrono>
#include <cmath>
#include <optional>

namespace craft_control
{

// Independent of ROS time, so a paused /clock cannot keep an old target active.
class PlanningWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  bool configure(double timeout_s, bool require_heartbeat)
  {
    if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
      return false;
    }
    timeout_s_ = timeout_s;
    require_heartbeat_ = require_heartbeat;
    return true;
  }

  void observe(bool hold, Clock::time_point now)
  {
    requested_hold_ = hold;
    received_at_ = now;
  }

  bool held(Clock::time_point now) const
  {
    if (!received_at_) {
      return require_heartbeat_;
    }
    const double age = std::chrono::duration<double>(now - *received_at_).count();
    return requested_hold_ || age < 0.0 || age > timeout_s_;
  }

private:
  double timeout_s_ = 0.5;
  bool require_heartbeat_ = true;
  bool requested_hold_ = true;
  std::optional<Clock::time_point> received_at_;
};

class SerialFeedbackWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  bool configure(double timeout_s)
  {
    if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
      return false;
    }
    timeout_s_ = timeout_s;
    return true;
  }

  void observe(Clock::time_point now) {received_at_ = now;}

  bool fresh(Clock::time_point now) const
  {
    if (!received_at_) {
      return false;
    }
    const double age = std::chrono::duration<double>(now - *received_at_).count();
    return age >= 0.0 && age <= timeout_s_;
  }

private:
  double timeout_s_ = 0.5;
  std::optional<Clock::time_point> received_at_;
};

}  // namespace craft_control
