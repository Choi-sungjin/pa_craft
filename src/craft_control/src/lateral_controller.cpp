#include "craft_control/lateral_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace craft_control
{

// ControlMaster가 YAML에서 읽은 바퀴 반지름, 좌우 바퀴 중심 간 거리, 앞뒤 축간거리, 스키드 계수,
// lookahead, 도착 허용 거리, 경로 재샘플 간격을 저장한다. 거리 단위는 m이다.
// 4WD 스키드 스티어는 회전할 때 바퀴가 옆으로 긁혀 차체가 바퀴 속도 차보다 덜 돈다.
// 유효 간격 = 계수 * 좌우 간격으로 이를 반영하며, 계수가 0 이하이면 1 + (축간거리/좌우 간격)^2로 추정한다.
// 실측값(test/drive/rotate.py의 wheel/IMU)이 있으면 그 값을 계수로 쓴다.
// tracking_gain은 경로 추종(Pure Pursuit) 회전 명령에만 곱하는 under steer 보정 계수다(제자리 회전 제외).
void LateralController::configure(
  double wheel_radius, double wheel_track, double wheelbase, double skid_steer_factor,
  double lookahead, double goal_tolerance, double path_spacing,
  double heading_tolerance, double rotate_angular_speed, double tracking_gain)
{
  const double ratio = wheelbase / wheel_track;
  wheel_radius_ = wheel_radius;
  wheel_separation_ = (skid_steer_factor > 0.0 ? skid_steer_factor : 1.0 + ratio * ratio) * wheel_track;
  lookahead_ = lookahead;
  goal_tolerance_ = goal_tolerance;
  path_spacing_ = path_spacing;
  heading_tolerance_ = heading_tolerance;
  rotate_angular_speed_ = rotate_angular_speed;
  tracking_gain_ = tracking_gain;
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
  update_node();
}

void LateralController::clear_odom()
{
  have_odom_ = false;
}

// Preserve planner geometry: interpolate on its segments, never bend a checked path.
void LateralController::set_target(const nav_msgs::msg::Path &path, bool require_goal_heading)
{
  require_goal_heading_ = require_goal_heading;
  const auto previous = path_;
  path_ = nav_msgs::msg::Path{}; path_.header = path.header; arc_.clear();
  if (path.poses.empty()) return;
  path_.poses.push_back(path.poses.front());
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto &a = path.poses[i-1].pose.position;
    const auto &b = path.poses[i].pose.position;
    const double length = std::hypot(b.x-a.x,b.y-a.y);
    if (length < 1e-9) {
      path_.poses.back() = path.poses[i];  // Keep the final heading at repeated vertices.
      continue;
    }
    const auto count = static_cast<size_t>(std::ceil(length / path_spacing_));
    for (size_t k = 1; k <= count; ++k) {
      auto pose = path.poses[i]; pose.header = path.header;
      const double t = static_cast<double>(k) / count;
      pose.pose.position.x = a.x + t*(b.x-a.x);
      pose.pose.position.y = a.y + t*(b.y-a.y);
      path_.poses.push_back(pose);
    }
  }
  arc_.assign(1, 0.0);
  for (std::size_t i = 1; i < path_.poses.size(); ++i) {
    const auto & a = path_.poses[i - 1].pose.position, & b = path_.poses[i].pose.position;
    arc_.push_back(arc_.back() + std::hypot(b.x - a.x, b.y - a.y));
  }
  // The same samples again (a pause, an unchanged update) keep the progress; a new path starts over.
  const bool same = previous.header.frame_id == path_.header.frame_id && previous.poses.size() == path_.poses.size() &&
    std::equal(previous.poses.begin(), previous.poses.end(), path_.poses.begin(), [](const auto & a, const auto & b) {
      return a.pose.position.x == b.pose.position.x && a.pose.position.y == b.pose.position.y;});
  if (!same) {node_ = 0; rotating_ = false; rotating_direction_ = 0;}
  update_node();
}

void LateralController::clear_target()
{
  path_ = nav_msgs::msg::Path{}; arc_.clear(); node_ = 0; rotating_ = false; rotating_direction_ = 0;
}

void LateralController::update_node()
{
  if (!have_odom_ || path_.poses.empty() || path_.header.frame_id != odom_.header.frame_id) return;
  const auto & p = odom_.pose.pose.position;
  const auto distance = [&](std::size_t i) {
      const auto & q = path_.poses[i].pose.position; return std::hypot(q.x - p.x, q.y - p.y);};
  // Ties go to the later sample inside the window (on a U-turn both branches are equally near: take the
  // one ahead) and to the earlier one in a global search.
  const auto nearest = [&](std::size_t lo, std::size_t hi, bool later) {
      std::size_t best = lo;
      for (std::size_t i = lo + 1; i <= hi; ++i) {
        if (distance(i) < distance(best) || (later && distance(i) == distance(best))) best = i;
      }
      return best;};
  node_ = std::min(node_, path_.poses.size() - 1);
  std::size_t next = nearest(node_ > kNodeWindow ? node_ - kNodeWindow : 0,
    std::min(path_.poses.size() - 1, node_ + kNodeWindow), true);
  // Lost the window (a localization jump, a path that does not start here): earliest global nearest.
  if (distance(next) > lookahead_) next = nearest(0, path_.poses.size() - 1, false);
  node_ = next;
}

// First sample at least lookahead along the path from my node, else the last one.
std::size_t LateralController::target_index() const
{
  for (std::size_t i = node_; i < path_.poses.size(); ++i) {
    if (arc_[i] - arc_[node_] >= lookahead_) return i;
  }
  return path_.poses.size() - 1;
}

// Within goal_tolerance of the end and near it along the path too, not where an earlier part passes the goal.
bool LateralController::at_goal() const
{
  const auto & p = odom_.pose.pose.position;
  const auto & goal = path_.poses.back().pose.position;
  return std::hypot(goal.x - p.x, goal.y - p.y) <= goal_tolerance_ &&
         arc_.back() - arc_[node_] <= goal_tolerance_ + kNodeWindow * path_spacing_;
}

bool LateralController::needs_rotation(double target_v) const
{
  const int direction = target_v < 0.0 ? -1 : 1;
  if (direction != rotating_direction_) {rotating_ = false; rotating_direction_ = direction;}
  const auto error = heading_error(target_v);
  rotating_ = error && std::abs(*error) > (rotating_ ? 0.5 * heading_tolerance_ : heading_tolerance_);
  return rotating_;
}

const nav_msgs::msg::Path & LateralController::target_path() const
{
  return path_;
}

// 차체 회전 속도 omega [rad/s]의 회전 성분 {-delta, +delta} [바퀴 rad/s], 유효 간격 사용.
std::array<double, 2> LateralController::wheel_delta(double omega) const
{
  const double delta = omega * wheel_separation_ / (2.0 * wheel_radius_);
  return {-delta, delta};
}

// 경로가 있고 odom과 좌표계가 같으며, 경로 종점이 도착 허용 거리 이내면 완료.
bool LateralController::is_target_reached() const
{
  if (!have_odom_ || path_.poses.empty() || path_.header.frame_id != odom_.header.frame_id) {
    return false;
  }
  if (!at_goal()) return false;
  if (require_goal_heading_) {
    const auto heading = [](const geometry_msgs::msg::Quaternion &q) {
      const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (!std::isfinite(n) || n < 1e-12) return std::numeric_limits<double>::quiet_NaN();
      return std::atan2(2*(q.w*q.z+q.x*q.y)/n, 1-2*(q.y*q.y+q.z*q.z)/n);
    };
    const double error = std::abs(std::remainder(heading(odom_.pose.pose.orientation) -
      heading(path_.poses.back().pose.orientation), 2*std::acos(-1.0)));
    if (!std::isfinite(error) || error > heading_tolerance_) return false;
  }
  return true;
}

std::optional<double> LateralController::heading_error(double target_v) const
{
  if (!have_odom_ || path_.poses.empty() || path_.header.frame_id != odom_.header.frame_id) {
    return std::nullopt;
  }
  const auto & p = odom_.pose.pose.position;
  const auto & q = odom_.pose.pose.orientation;
  const double current = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  double desired;
  if (at_goal()) {
    if (!require_goal_heading_) return std::nullopt;
    const auto & g = path_.poses.back().pose.orientation;
    desired = std::atan2(
      2.0 * (g.w * g.z + g.x * g.y), 1.0 - 2.0 * (g.y * g.y + g.z * g.z));
  } else {
    const auto * target = &path_.poses[target_index()].pose.position;
    desired = std::atan2(target->y - p.y, target->x - p.x);
    if (target_v < 0.0) desired += std::acos(-1.0);
  }
  return std::remainder(desired - current, 2.0 * std::acos(-1.0));
}

// Pure Pursuit로 현재 위치에서 경로를 추종하는 데 필요한 곡률 kappa [1/m]를 계산한다.
// 목표점은 내 노드(update_node)에서 경로를 따라 lookahead 이상 간 첫 점이며, 차체 좌표(전방 +x, 좌측 +y)로
// 변환해 kappa = 2*y / (x*x + y*y)를 구한다. 경로 전체에서 차체 앞쪽 점을 찾지 않으므로 먼 구간을 집지 않고,
// 목표점이 뒤나 옆에 있으면 needs_rotation()이 제자리 회전을 먼저 시킨다. 후진(v < 0)도 같은 원을 따른다.
// odom/경로 없음, 좌표계 불일치, 종점 도달이면 nullopt를 반환한다.
std::optional<double> LateralController::get_curvature(double /*target_v*/) const
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
  const auto & point = path_.poses[target_index()].pose.position;
  const double dx = point.x - p.x, dy = point.y - p.y;
  const double x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double distance = std::hypot(x, y);
  if (distance < 1e-9) return std::nullopt;
  // Circle through the robot tangent to its heading; the same arc serves reverse travel (v < 0).
  return 2.0 * (y / distance) / distance;
}

// 현재 경로·odom·진행 방향으로 유효한 추종 곡률을 구할 수 있는지 확인한다.
// false이면 ControlMaster가 longitudinal 목표를 정지시킨다.
bool LateralController::can_track(double target_v) const
{
  return get_curvature(target_v).has_value() && !needs_rotation(target_v);
}

// 추종 곡률을 차체 회전 속도 omega = tracking_gain * linear_speed * kappa [rad/s]로 바꾼다.
// 스키드 스티어의 회전 성분만 {-omega*유효간격/(2*반지름), +omega*유효간격/(2*반지름)} [바퀴 rad/s]로 반환한다.
// ControlMaster가 longitudinal의 전진 성분을 더해 최종 좌우 속도를 만들고 공통 비율로 속도를 제한한다.
// 조향각을 구하는 bicycle 식은 사용하지 않는다. linear_speed=0이면 제자리 회전을 생성하지 못한다.
// 곡률을 구할 수 없으면 회전 성분 {0, 0}을 반환한다. 모터 속도 추종은 OpenCR/DYNAMIXEL이 담당한다.
std::array<double, 2> LateralController::compute_command(double linear_speed, double target_v_hint) const
{
  if (std::isfinite(target_v_hint) && std::abs(target_v_hint) <= 1e-12) return {0.0, 0.0};
  const double direction = std::isfinite(target_v_hint) ? target_v_hint : linear_speed;
  // ControlMaster always supplies the direction hint; it has also cleared forward motion when can_track()
  // said to turn. Direct callers without a hint never turn in place.
  if (std::isfinite(target_v_hint) && needs_rotation(direction)) {
    return wheel_delta(std::copysign(rotate_angular_speed_, *heading_error(direction)));
  }
  const auto curvature = get_curvature(direction);
  if (!curvature) {
    return {0.0, 0.0};
  }
  return wheel_delta(tracking_gain_ * linear_speed * *curvature);
}

}  // namespace craft_control
