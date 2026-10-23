#include "craft_control/lm_controller.hpp"

#include <algorithm>
#include <cmath>

namespace craft_control
{

bool LmController::configure(
  const std::string & joint_name, double min_angle, double max_angle,
  double direction, double offset, double pulley_radius, double gear_ratio, double tolerance)
{
  if (joint_name.empty() || !std::isfinite(min_angle) || !std::isfinite(max_angle) ||
    min_angle >= max_angle || !std::isfinite(direction) || std::abs(direction) != 1.0 ||
    !std::isfinite(offset))
  {
    return false;
  }
  // All-zero calibration remains a placeholder: hold position and reject linear targets.
  const bool placeholder = pulley_radius == 0.0 && gear_ratio == 0.0;
  const bool calibrated = std::isfinite(pulley_radius) && pulley_radius > 0.0 &&
    std::isfinite(gear_ratio) && gear_ratio > 0.0;
  if (!placeholder && !calibrated) {
    return false;
  }
  joint_name_ = joint_name;
  min_angle_ = min_angle;
  max_angle_ = max_angle;
  direction_ = direction;
  offset_ = offset;
  pulley_radius_ = pulley_radius;
  gear_ratio_ = gear_ratio;
  tolerance_ = tolerance;
  conversion_ready_ = calibrated;
  return true;
}

bool LmController::is_valid_target(const std::string & joint_name, double position) const
{
  return conversion_ready_ && joint_name == joint_name_ &&
         std::isfinite(position) &&
         std::isfinite(offset_ + direction_ * gear_ratio_ * position / pulley_radius_);
}

void LmController::set_target(double position)
{
  target_position_ = position;
  has_target_ = true;
}

void LmController::clear_target()
{
  has_target_ = false;
}

double LmController::compute_command(double current_angle) const
{
  const double requested = has_target_ ?
    offset_ + direction_ * gear_ratio_ * target_position_ / pulley_radius_ : current_angle;
  return std::clamp(requested, min_angle_, max_angle_);
}

std::optional<double> LmController::get_joint_position(double current_angle) const
{
  if (!conversion_ready_) {
    return std::nullopt;
  }
  const double position = (current_angle - offset_) * pulley_radius_ / (direction_ * gear_ratio_);
  return std::isfinite(position) ? std::optional<double>(position) : std::nullopt;
}

// 목표가 있고, LM 모터 피드백이 목표 모터각에서 허용 오차 이내면 완료.
bool LmController::is_target_reached(double current_angle) const
{
  return has_target_ && std::abs(current_angle - compute_command(current_angle)) <= tolerance_;
}

}  // namespace craft_control
