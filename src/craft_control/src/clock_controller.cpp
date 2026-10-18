#include "craft_control/clock_controller.hpp"

#include <cmath>

namespace craft_control
{

// ControlMaster가 YAML에서 읽은 슬롯 수, 슬롯 간 모터 각도, 영점, 방향, 정렬 허용 오차를 검증·저장한다.
// 기본 설정은 0 rad = 12번 슬롯, 반시계 방향으로 번호 증가, 슬롯 간격 30도이다.
// 방향은 +/-1, 허용 오차는 슬롯 간격의 절반 미만이어야 한다.
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
  return true;
}

// 목표 슬롯이 1..slot_count 범위인지 검증한다.
bool ClockController::is_valid_target(int slot) const
{
  return slot >= 1 && slot <= slot_count_;
}

// 목표 슬롯을 모터 각도(rad)로 변환하고, 현재 피드백에서 가장 가까운 동등 각도를 목표로 저장한다.
// delta = remainder(슬롯 기준 각도 - 현재 각도, 한 바퀴에 해당하는 모터 각도).
// 기본 설정에서 12 -> 7은 시계방향 150도(-5*pi/6 rad); 정확히 반 바퀴면 어느 방향도 최단이다.
// 같은 슬롯 명령이 반복되면 저장된 목표각을 유지하여 회전 중 목표가 다시 계산되지 않게 한다.
void ClockController::set_target(int slot, double clock_angle)
{
  // Repeated planning messages must not change the chosen turn or completed target.
  if (has_target_ && target_slot_ == slot) {
    return;
  }
  const double slot_angle = zero_angle_ + direction_ * (slot % slot_count_) * slot_angle_;
  const double delta = std::remainder(slot_angle - clock_angle, revolution_angle_);
  target_angle_ = clock_angle + delta;  // nearest equivalent angle; fixed during motion
  target_slot_ = slot;
  has_target_ = true;
}

void ClockController::clear_target()
{
  has_target_ = false;
}

// 저장된 모터 목표각(rad)을 반환한다. 목표가 없으면 현재 피드백 각도를 반환해 위치를 유지한다.
// 최단 회전 계산은 set_target()에서 끝나며, 여기서는 목표를 다시 계산하지 않는다.
// OpenCR/모터는 음수 및 여러 바퀴를 표현하는 연속 각도를 지원하고 이동 프로파일·PID를 담당해야 한다.
double ClockController::compute_command(double clock_angle) const
{
  if (!has_target_) {
    return clock_angle;  // hold current angle
  }
  return target_angle_;
}

// 현재 모터 각도에서 영점·방향을 보정한 뒤 가장 가까운 수납 슬롯 번호(1..slot_count)를 구한다.
// 정렬 오차가 허용 범위를 넘으면 0을 반환한다.
// 모터의 이동/고장 상태는 여기서 검사하지 않으며, ControlMaster가 상태와 함께 사용한다.
int ClockController::get_current_slot(double clock_angle) const
{
  const double angle = std::remainder(direction_ * (clock_angle - zero_angle_), revolution_angle_);
  const int nearest = static_cast<int>(std::round(angle / slot_angle_));
  if (std::abs(angle - nearest * slot_angle_) > angle_tolerance_) {
    return 0;
  }
  const int remainder = nearest % slot_count_;
  const int slot = remainder < 0 ? remainder + slot_count_ : remainder;
  return slot == 0 ? slot_count_ : slot;
}

// 목표가 존재하고, 실제 피드백과 저장된 목표각의 차이가 허용 오차 이내면 완료.
// ControlMaster는 모터 idle 상태, 새 피드백, 명령 전송 여부도 확인해 보고한다.
bool ClockController::is_target_reached(double clock_angle) const
{
  return has_target_ && std::abs(target_angle_ - clock_angle) <= angle_tolerance_;
}

}  // namespace craft_control
