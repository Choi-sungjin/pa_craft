#include "craft_control/clock_controller.hpp"

#include <cmath>

namespace craft_control
{

bool ClockController::configure(
  int slot_count, double slot_angle, double zero_angle, double direction, double angle_tolerance)
{
  if (slot_count < 2 || !std::isfinite(slot_angle) || slot_angle <= 0.0 ||
    !std::isfinite(slot_count * slot_angle) || !std::isfinite(zero_angle) ||
    !std::isfinite(direction) || std::abs(direction) != 1.0 ||
    !std::isfinite(angle_tolerance) || angle_tolerance <= 0.0 ||
    angle_tolerance >= slot_angle / 2.0)
  {
    return false;
  }
  slot_count_ = slot_count;
  slot_angle_ = slot_angle;
  zero_angle_ = zero_angle;
  direction_ = direction;
  revolution_angle_ = slot_count * slot_angle;
  angle_tolerance_ = angle_tolerance;
  has_target_ = false;
  configured_ = true;
  return true;
}

bool ClockController::set_target_slot(int slot, double clock_angle)
{
  if (!configured_ || slot < 1 || slot > slot_count_ || !std::isfinite(clock_angle)) {
    return false;
  }
  // Repeated planning messages must not change the chosen turn or completed target.
  if (has_target_ && target_slot_ == slot) {
    return true;
  }
  const double slot_angle = zero_angle_ + direction_ * (slot % slot_count_) * slot_angle_;
  const double delta = std::remainder(slot_angle - clock_angle, revolution_angle_);
  target_angle_ = clock_angle + delta;  // nearest equivalent angle; fixed during motion
  target_slot_ = slot;
  has_target_ = true;
  return true;
}

double ClockController::compute_clock_command(double clock_angle) const
{
  if (!has_target_) {
    return clock_angle;  // hold current angle
  }
  return target_angle_;
}

int ClockController::get_current_slot(double clock_angle) const
{
  if (!configured_ || !std::isfinite(clock_angle)) {
    return 0;
  }
  const double angle = std::remainder(direction_ * (clock_angle - zero_angle_), revolution_angle_);
  const int nearest = static_cast<int>(std::round(angle / slot_angle_));
  if (std::abs(angle - nearest * slot_angle_) > angle_tolerance_) {
    return 0;
  }
  const int remainder = nearest % slot_count_;
  const int slot = remainder < 0 ? remainder + slot_count_ : remainder;
  return slot == 0 ? slot_count_ : slot;
}

bool ClockController::is_target_reached(double clock_angle, int clock_state) const
{
  return has_target_ && clock_state == 0 && std::isfinite(clock_angle) &&
         std::abs(target_angle_ - clock_angle) <= angle_tolerance_;
}

int ClockController::get_clock_status(int state) const
{
  return state;
}

}  // namespace craft_control
