#include "craft_control/gripper_controller.hpp"

#include <algorithm>
#include <cmath>

namespace craft_control
{

bool GripperController::configure(
  double min_angle, double max_angle, double open_angle, double close_angle,
  double grasp_current_threshold_a, double grasp_current_debounce, double tolerance,
  double grasp_squeeze_angle)
{
  if (!std::isfinite(min_angle) || !std::isfinite(max_angle) ||
    !std::isfinite(open_angle) || !std::isfinite(close_angle) ||
    min_angle >= max_angle || open_angle < min_angle || open_angle > max_angle ||
    close_angle < min_angle || close_angle > max_angle ||
    !std::isfinite(grasp_current_threshold_a) || grasp_current_threshold_a < 0.0 ||
    !std::isfinite(grasp_current_debounce) || grasp_current_debounce < 0.0 ||
    !std::isfinite(grasp_squeeze_angle) || grasp_squeeze_angle < 0.0)
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
  grasp_squeeze_angle_ = grasp_squeeze_angle;
  grasp_detection_configured_ = detection_enabled;
  tolerance_ = tolerance;
  return true;
}

// set gripper open/close target; reset grasp detection when the direction changes
void GripperController::set_target(bool close)
{
  if (!has_target_ || closing_ != close) {
    overcurrent_time_ = 0.0;
    grasped_ = false;
  }
  target_angle_ = close ? close_angle_ : open_angle_;
  closing_ = close;
  has_target_ = true;
}

void GripperController::clear_target()
{
  has_target_ = false;
  closing_ = false;
  overcurrent_time_ = 0.0;
  grasped_ = false;
}

double GripperController::compute_command(double current_angle) const
{
  const double requested = !has_target_ ? current_angle :
    (closing_ && grasped_) ? grasp_hold_angle_ : target_angle_;
  return std::clamp(requested, min_angle_, max_angle_);
}

// 열기는 열림 각도 도달, 닫기는 닫힘 각도 도달 또는 물체 파지(전류 기반)로 완료.
// 물체를 잡으면 닫힘 각도까지 가지 못하므로 파지도 완료로 본다.
bool GripperController::is_target_reached(double current_angle) const
{
  return has_target_ &&
         (std::abs(current_angle - compute_command(current_angle)) <= tolerance_ ||
         (closing_ && is_grasped()));
}

void GripperController::update_current_feedback(double motor_current_a, double dt, double current_angle)
{
  if (!has_target_ || !closing_ || grasped_ || !grasp_detection_configured_ ||
    !std::isfinite(motor_current_a) || !std::isfinite(dt) || dt <= 0.0 ||
    !std::isfinite(current_angle))
  {
    return;
  }
  if (std::abs(motor_current_a) >= grasp_current_threshold_a_) {
    overcurrent_time_ += dt;
    if (overcurrent_time_ >= grasp_current_debounce_) {
      grasped_ = true;
      // Squeeze toward close_angle only (the motor + direction closes per gripper_close_angle).
      const double toward_close = close_angle_ >= open_angle_ ? 1.0 : -1.0;
      grasp_hold_angle_ = std::clamp(
        current_angle + toward_close * grasp_squeeze_angle_, min_angle_, max_angle_);
    }
  } else {
    overcurrent_time_ = 0.0;
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
