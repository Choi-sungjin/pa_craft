#include "craft_control/gripper_controller.hpp"

#include <algorithm>
#include <cmath>

namespace craft_control
{

bool GripperController::configure(
  double min_angle, double max_angle, double open_angle, double close_angle,
  double grasp_current_threshold_a, double grasp_current_debounce)
{
  configured_ = false;
  has_target_ = false;
  if (!std::isfinite(min_angle) || !std::isfinite(max_angle) ||
    !std::isfinite(open_angle) || !std::isfinite(close_angle) ||
    min_angle >= max_angle || open_angle < min_angle || open_angle > max_angle ||
    close_angle < min_angle || close_angle > max_angle ||
    !std::isfinite(grasp_current_threshold_a) || grasp_current_threshold_a < 0.0 ||
    !std::isfinite(grasp_current_debounce) || grasp_current_debounce < 0.0)
  {
    return false;
  }
  const bool detection_disabled = grasp_current_threshold_a == 0.0 && grasp_current_debounce == 0.0;
  const bool detection_enabled = grasp_current_threshold_a > 0.0 && grasp_current_debounce > 0.0;
  if (!detection_disabled && !detection_enabled) {
    return false;
  }

  min_angle_ = min_angle;
  max_angle_ = max_angle;
  open_angle_ = open_angle;
  close_angle_ = close_angle;
  grasp_current_threshold_a_ = grasp_current_threshold_a;
  grasp_current_debounce_ = grasp_current_debounce;
  grasp_detection_configured_ = detection_enabled;
  configured_ = true;
  return true;
}

bool GripperController::set_target(bool close)
{
  if (!configured_) {
    return false;
  }
  if (!has_target_ || closing_ != close) {
    overcurrent_time_ = 0.0;
    grasped_ = false;
  }
  target_angle_ = close ? close_angle_ : open_angle_;
  closing_ = close;
  has_target_ = true;
  return true;
}

double GripperController::compute_gripper_command(double current_angle) const
{
  if (!configured_) {
    return current_angle;
  }
  const double requested = has_target_ ? target_angle_ : current_angle;
  return std::clamp(requested, min_angle_, max_angle_);
}

bool GripperController::is_target_reached(double current_angle, double tolerance) const
{
  return configured_ && has_target_ && std::isfinite(current_angle) &&
         std::isfinite(tolerance) && tolerance >= 0.0 &&
         std::abs(current_angle - target_angle_) <= tolerance;
}

void GripperController::update_current_feedback(double motor_current_a, double dt)
{
  if (!configured_ || !has_target_ || !closing_ || !grasp_detection_configured_ ||
    !std::isfinite(motor_current_a) || !std::isfinite(dt) || dt <= 0.0)
  {
    return;
  }
  if (std::abs(motor_current_a) >= grasp_current_threshold_a_) {
    overcurrent_time_ += dt;
    if (overcurrent_time_ >= grasp_current_debounce_) {
      grasped_ = true;
    }
  } else {
    overcurrent_time_ = 0.0;
    grasped_ = false;
  }
}

bool GripperController::is_grasped() const
{
  return grasp_detection_configured_ && grasped_;
}

bool GripperController::grasp_detection_configured() const
{
  return grasp_detection_configured_;
}

}  // namespace craft_control
