#include "craft_control/lm_controller.hpp"

#include <algorithm>
#include <cmath>

namespace craft_control
{

bool LmController::configure(
  const std::string & joint_name, double min_angle, double max_angle,
  double direction, double offset, double meters_per_rad, double tolerance)
{
  if (joint_name.empty() || !std::isfinite(min_angle) || !std::isfinite(max_angle) ||
    min_angle >= max_angle || !std::isfinite(direction) || std::abs(direction) != 1.0 ||
    !std::isfinite(offset))
  {
    return false;
  }
  // Zero calibration holds position and rejects linear targets.
  if (!std::isfinite(meters_per_rad) || meters_per_rad < 0.0) {
    return false;
  }
  joint_name_ = joint_name;
  min_angle_ = min_angle;
  max_angle_ = max_angle;
  direction_ = direction;
  offset_ = offset;
  meters_per_rad_ = meters_per_rad;
  tolerance_ = tolerance;
  conversion_ready_ = meters_per_rad > 0.0;
  return true;
}

bool LmController::is_valid_target(const std::string & joint_name, double position) const
{
  return conversion_ready_ && joint_name == joint_name_ &&
         std::isfinite(position) &&
         std::isfinite(offset_ + direction_ * position / meters_per_rad_);
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
    offset_ + direction_ * target_position_ / meters_per_rad_ : current_angle;
  return std::clamp(requested, min_angle_, max_angle_);
}

std::optional<double> LmController::get_joint_position(double current_angle) const
{
  if (!conversion_ready_) {
    return std::nullopt;
  }
  const double position = (current_angle - offset_) * meters_per_rad_ / direction_;
  return std::isfinite(position) ? std::optional<double>(position) : std::nullopt;
}

// 목표가 있고, LM 모터 피드백이 목표 모터각에서 허용 오차 이내면 완료.
bool LmController::is_target_reached(double current_angle) const
{
  return has_target_ && std::abs(current_angle - compute_command(current_angle)) <= tolerance_;
}

}  // namespace craft_control
