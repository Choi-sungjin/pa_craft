#include "craft_control/lateral_controller.hpp"

#include <cmath>
#include <limits>

namespace craft_control
{

// ControlMaster가 YAML에서 읽은 바퀴 반지름, 좌우 바퀴 중심 간 거리, lookahead, 도착 허용 거리를 저장한다.
// 모두 ControlMaster가 유한한 양수로 검증하며 거리 단위는 m이다.
void LateralController::configure(
  double wheel_radius, double wheel_separation, double lookahead, double goal_tolerance)
{
  wheel_radius_ = wheel_radius;
  wheel_separation_ = wheel_separation;
  lookahead_ = lookahead;
  goal_tolerance_ = goal_tolerance;
}

// odom: frame_id가 있고 위치가 유한하며 quaternion 크기가 0이 아니어야 한다.
bool LateralController::is_valid_odom(const nav_msgs::msg::Odometry & odom) const
{
  const auto & p = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  return !odom.header.frame_id.empty() && std::isfinite(p.x) && std::isfinite(p.y) &&
         std::isfinite(norm) && norm > 1e-9;
}

// 경로: 비어 있지 않고, frame_id가 있으며, 모든 점이 유한하고 같은 frame이어야 한다.
bool LateralController::is_valid_target(const nav_msgs::msg::Path & path) const
{
  if (path.poses.empty() || path.header.frame_id.empty()) {
    return false;
  }
  for (const auto & pose : path.poses) {
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
      (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id))
    {
      return false;
    }
  }
  return true;
}

// 검증된 차축 중심 기준 현재 위치·자세를 저장하고 quaternion을 정규화한다.
// 경로와의 좌표계 일치는 곡률 계산 시 확인한다.
void LateralController::set_odom(const nav_msgs::msg::Odometry & odom)
{
  odom_ = odom;
  auto & q = odom_.pose.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  have_odom_ = true;
}

void LateralController::clear_odom()
{
  have_odom_ = false;
}

// 검증된 planning 경로점(x, y)과 frame_id를 저장한다.
// 경로점의 자세나 trajectory 기준 곡률은 사용하지 않고, 현재 odom과 경로점으로 추종 곡률을 계산한다.
void LateralController::set_target(const nav_msgs::msg::Path & path)
{
  path_ = path;
}

void LateralController::clear_target()
{
  path_ = nav_msgs::msg::Path{};
}

// 경로가 있고 odom과 좌표계가 같으며, 경로 종점이 도착 허용 거리 이내면 완료.
bool LateralController::is_target_reached() const
{
  if (!have_odom_ || path_.poses.empty() || path_.header.frame_id != odom_.header.frame_id) {
    return false;
  }
  const auto & p = odom_.pose.pose.position;
  const auto & goal = path_.poses.back().pose.position;
  return std::hypot(goal.x - p.x, goal.y - p.y) <= goal_tolerance_;
}

// Pure Pursuit로 현재 위치에서 경로를 추종하는 데 필요한 곡률 kappa [1/m]를 계산한다.
// 가장 가까운 경로점부터 진행 방향의 점을 찾고, 차체 좌표(전방 +x, 좌측 +y)로 변환한다.
// lookahead 이상 떨어진 첫 점을 선택하며, 없으면 진행 방향에 있는 마지막 후보점을 사용한다.
// kappa = 2*y / (x*x + y*y). target_v의 부호로 전진/후진을 구분하며 후진 시 차체 뒤의 점을 추종한다.
// odom/경로 없음, 좌표계 불일치, 종점 허용 거리 도달 또는 적합한 점이 없으면 nullopt를 반환한다.
std::optional<double> LateralController::get_curvature(double target_v) const
{
  if (!have_odom_ || path_.poses.empty() || path_.header.frame_id != odom_.header.frame_id) {
    return std::nullopt;
  }
  if (is_target_reached()) {
    return std::nullopt;
  }
  const auto & p = odom_.pose.pose.position;
  const auto & q = odom_.pose.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path_.poses.size(); ++i) {
    const auto & point = path_.poses[i].pose.position;
    const double distance = std::hypot(point.x - p.x, point.y - p.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }
  std::optional<double> curvature;
  for (std::size_t i = nearest; i < path_.poses.size(); ++i) {
    const auto & point = path_.poses[i].pose.position;
    const double dx = point.x - p.x;
    const double dy = point.y - p.y;
    const double x = c * dx + s * dy;
    const double y = -s * dx + c * dy;
    // Reverse targets pursue points behind the chassis. A zero target never turns in place.
    if ((target_v < 0.0 ? -x : x) <= 1e-9) {
      continue;
    }
    const double distance = std::hypot(x, y);
    curvature = 2.0 * (y / distance) / distance;
    if (distance >= lookahead_) {
      break;
    }
  }
  return curvature;
}

// 현재 경로·odom·진행 방향으로 유효한 추종 곡률을 구할 수 있는지 확인한다.
// false이면 ControlMaster가 longitudinal 목표를 정지시킨다.
bool LateralController::can_track(double target_v) const
{
  return get_curvature(target_v).has_value();
}

// 추종 곡률을 차체 회전 속도 omega = linear_speed * kappa [rad/s]로 바꾼다.
// 차동구동의 회전 성분만 {-omega*폭/(2*반지름), +omega*폭/(2*반지름)} [바퀴 rad/s]로 반환한다.
// ControlMaster가 longitudinal의 전진 성분을 더해 최종 좌우 속도를 만들고 공통 비율로 속도를 제한한다.
// 조향각을 구하는 bicycle 식은 사용하지 않는다. linear_speed=0이면 제자리 회전을 생성하지 못한다.
// 곡률을 구할 수 없으면 회전 성분 {0, 0}을 반환한다. 모터 속도 추종은 OpenCR/DYNAMIXEL이 담당한다.
std::array<double, 2> LateralController::compute_command(double linear_speed) const
{
  const auto curvature = get_curvature(linear_speed);
  if (!curvature) {
    return {0.0, 0.0};
  }
  const double omega = linear_speed * *curvature;
  const double wheel_delta = omega * wheel_separation_ / (2.0 * wheel_radius_);
  return {-wheel_delta, wheel_delta};
}

}  // namespace craft_control
