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
  const std::vector<double> & lookahead_speeds, const std::vector<double> & lookahead_distances,
  double goal_tolerance, double path_spacing,
  double heading_tolerance, double rotate_angular_speed, double tracking_gain)
{
  const double ratio = wheelbase / wheel_track;
  wheel_radius_ = wheel_radius;
  wheel_separation_ = (skid_steer_factor > 0.0 ? skid_steer_factor : 1.0 + ratio * ratio) * wheel_track;
  lookahead_speeds_ = lookahead_speeds;
  lookahead_distances_ = lookahead_distances;
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
void LateralController::set_target(
  const nav_msgs::msg::Path &path, bool require_goal_heading, const std::vector<double> & speed_limit)
{
  require_goal_heading_ = require_goal_heading;
  const auto previous = path_;
  path_ = nav_msgs::msg::Path{}; path_.header = path.header; arc_.clear(); speed_limit_.clear();
  if (path.poses.empty()) return;
  const bool limited = speed_limit.size() == path.poses.size();
  path_.poses.push_back(path.poses.front());
  if (limited) speed_limit_.push_back(speed_limit.front());
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto &a = path.poses[i-1].pose.position;
    const auto &b = path.poses[i].pose.position;
    const double length = std::hypot(b.x-a.x,b.y-a.y);
    if (length < 1e-9) {
      path_.poses.back() = path.poses[i];  // Keep the final heading at repeated vertices.
      if (limited) speed_limit_.back() = std::min(speed_limit_.back(), speed_limit[i]);
      continue;
    }
    const auto count = static_cast<size_t>(std::ceil(length / path_spacing_));
    for (size_t k = 1; k <= count; ++k) {
      auto pose = path.poses[i]; pose.header = path.header;
      const double t = static_cast<double>(k) / count;
      pose.pose.position.x = a.x + t*(b.x-a.x);
      pose.pose.position.y = a.y + t*(b.y-a.y);
      path_.poses.push_back(pose);
      if (limited) speed_limit_.push_back(speed_limit[i-1] + t*(speed_limit[i]-speed_limit[i-1]));
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
  if (!same) {node_ = 0; rotating_ = false; rotating_direction_ = 0; rotate_sign_ = 0;}
  update_node();
}

void LateralController::clear_target()
{
  path_ = nav_msgs::msg::Path{}; arc_.clear(); speed_limit_.clear(); node_ = 0; rotating_ = false; rotating_direction_ = 0;
  continuous_tracking_ = false;
}

double LateralController::speed_limit() const
{
  return speed_limit_.empty() ? std::numeric_limits<double>::infinity() : speed_limit_[node_];
}

double LateralController::lookahead() const
{
  if (task_lookahead_ > 0.0) return task_lookahead_;
  const auto & s = lookahead_speeds_;
  const auto & d = lookahead_distances_;
  const double v = std::abs(speed_);
  if (v <= s.front()) return d.front();
  for (std::size_t i = 1; i < s.size(); ++i) {
    if (v <= s[i]) return d[i - 1] + (v - s[i - 1]) / (s[i] - s[i - 1]) * (d[i] - d[i - 1]);
  }
  return d.back();
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
  if (distance(next) > lookahead()) next = nearest(0, path_.poses.size() - 1, false);
  node_ = next;
}

// Arc-length progress of the robot projected onto the path segments adjacent to my node.
// 내 노드 앞뒤 선분에 로봇을 투영한 부호 있는 횡오차 [m]: 경로 진행 방향의 왼쪽이 +.
std::optional<double> LateralController::lateral_offset() const
{
  if (!have_odom_ || path_.poses.size() < 2 || path_.header.frame_id != odom_.header.frame_id) return std::nullopt;
  const auto & p = odom_.pose.pose.position;
  std::optional<double> best;
  const auto lo = node_ > 0 ? node_ - 1 : 0, hi = std::min(path_.poses.size() - 1, node_ + 1);
  for (std::size_t i = lo; i < hi; ++i) {
    const auto & a = path_.poses[i].pose.position, & b = path_.poses[i + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y, len = std::hypot(dx, dy);
    if (len < 1e-9) continue;
    const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / (len * len), 0.0, 1.0);
    const double ex = p.x - (a.x + t * dx), ey = p.y - (a.y + t * dy);
    if (!best || std::hypot(ex, ey) < std::abs(*best)) best = std::copysign(std::hypot(ex, ey), dx * ey - dy * ex);
  }
  return best;
}

double LateralController::projected_arc() const
{
  auto p = odom_.pose.pose.position;
  if (task_lookahead_ > 0.0) {
    const auto &q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    p.x += tracking_reference_x_ * std::cos(yaw);
    p.y += tracking_reference_x_ * std::sin(yaw);
  }
  double best_arc = arc_[node_];
  double best_dist = std::numeric_limits<double>::infinity();
  const auto lo = task_lookahead_ > 0.0 ? 0 : (node_ > 0 ? node_ - 1 : 0);
  const auto hi = task_lookahead_ > 0.0 ? path_.poses.size() - 1 : std::min(path_.poses.size() - 1, node_ + 1);
  for (std::size_t i = lo; i < hi; ++i) {
    const auto & a = path_.poses[i].pose.position;
    const auto & b = path_.poses[i + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    const double t = len2 > 1e-18 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
    const double d = std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
    if (d < best_dist) {
      best_dist = d;
      best_arc = arc_[i] + t * (arc_[i + 1] - arc_[i]);
    }
  }
  // A front reference can be past a short dock path while the base has yet to reach its goal.
  // Project onto the forward ray beyond the final segment as well as the finite path.
  if (task_lookahead_ > 0.0 && path_.poses.size() >= 2) {
    const auto &a = path_.poses[path_.poses.size() - 2].pose.position;
    const auto &b = path_.poses.back().pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    if (len2 > 1e-18) {
      const double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
      if (t > 1.0) {
        const double d = std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
        if (d < best_dist) best_arc = arc_[arc_.size() - 2] + t * std::sqrt(len2);
      }
    }
  }
  return best_arc;
}

std::optional<geometry_msgs::msg::Point> LateralController::lookahead_point() const
{
  if (!have_odom_ || path_.poses.empty() || path_.header.frame_id != odom_.header.frame_id) return std::nullopt;
  return target_point();
}

// The Pure Pursuit target: lookahead along the path from my node. Near the end the path is extended straight along
// its last segment, so the target stays lookahead ahead instead of shrinking onto the goal: a goal a few cm away
// turned the target direction with every mm of pose noise and the robot spun in place (box dock, 2026-11-08).
geometry_msgs::msg::Point LateralController::target_point() const
{
  const double target_arc = projected_arc() + lookahead();
  for (std::size_t i = task_lookahead_ > 0.0 ? 1 : (node_ > 0 ? node_ : 1); i < path_.poses.size(); ++i) {
    if (arc_[i] >= target_arc) {
      const auto & a = path_.poses[i - 1].pose.position;
      const auto & b = path_.poses[i].pose.position;
      const double seg = arc_[i] - arc_[i - 1];
      const double t = seg > 1e-9 ? std::clamp((target_arc - arc_[i - 1]) / seg, 0.0, 1.0) : 0.0;
      auto point = b;
      point.x = a.x + t * (b.x - a.x);
      point.y = a.y + t * (b.y - a.y);
      return point;
    }
  }
  auto point = path_.poses.back().pose.position;
  const double short_by = target_arc - arc_.back();
  if (short_by > 0.0 && path_.poses.size() >= 2) {
    const auto & a = path_.poses[path_.poses.size() - 2].pose.position;
    const double dx = point.x - a.x, dy = point.y - a.y, length = std::hypot(dx, dy);
    if (length > 1e-9) {point.x += dx / length * short_by; point.y += dy / length * short_by;}
  }
  return point;
}

// Within goal_tolerance of the end and near it along the path too, not where an earlier part passes the goal.
// A fine task tolerance can be overshot: past the end of the path (along its last segment, within the configured
// tolerance) also counts, or the robot would turn round for a goal just behind it.
bool LateralController::at_goal() const
{
  const auto & p = odom_.pose.pose.position;
  const auto & goal = path_.poses.back().pose.position;
  const double d = std::hypot(goal.x - p.x, goal.y - p.y), left = arc_.back() - arc_[node_];
  if (d <= goal_tol() && left <= goal_tol() + kNodeWindow * path_spacing_) return true;
  // A fine task tolerance is the distance along the last segment (past the goal included): a robot a little to the
  // side has arrived when it reaches the goal's line, and one just past it does not turn back. The side offset is
  // left to the caller (planning checks the box alignment), within the configured tolerance.
  if (path_.poses.size() < 2 || d > goal_tolerance_ || left > goal_tolerance_ + kNodeWindow * path_spacing_) return false;
  const auto & a = path_.poses[path_.poses.size() - 2].pose.position;
  const double ux = goal.x - a.x, uy = goal.y - a.y, length = std::hypot(ux, uy);
  return length > 1e-9 && ((goal.x - p.x) * ux + (goal.y - p.y) * uy) / length <= goal_tol();
}

void LateralController::set_task_tolerances(double goal_tolerance, double heading_tolerance)
{
  task_goal_tolerance_ = goal_tolerance;
  task_heading_tolerance_ = heading_tolerance;
}

// The task's fine alignment applies before the robot has moved along the path and at the goal heading; on the way
// the configured tolerance (Pure Pursuit steers small errors out).
bool LateralController::needs_rotation(double target_v) const
{
  if (continuous_tracking_ && arc_[node_] > kNodeWindow * path_spacing_ && !at_goal()) {rotating_ = false; rotate_sign_ = 0; return false;}
  const int direction = target_v < 0.0 ? -1 : 1;
  if (direction != rotating_direction_) {rotating_ = false; rotating_direction_ = direction;}
  const auto error = heading_error(target_v);
  const bool fine = task_heading_tolerance_ > 0.0 && !path_.poses.empty() &&
    (arc_[node_] <= kNodeWindow * path_spacing_ || (!camera_distance_mode_ && at_goal()));
  const double tolerance = fine ? task_heading_tolerance_ : heading_tolerance_;
  // One threshold to start and to stop (user, 2026-11-10: 0.5 x tolerance to stop beside tolerance to start was
  // two numbers to think about); the slow approach near the heading (compute_command) keeps the overshoot small.
  rotating_ = error && std::abs(*error) > tolerance;
  if (!rotating_) rotate_sign_ = 0;
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
  if (camera_distance_mode_) return false;
  if (!at_goal()) return false;
  if (require_goal_heading_) {
    const auto heading = [](const geometry_msgs::msg::Quaternion &q) {
      const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (!std::isfinite(n) || n < 1e-12) return std::numeric_limits<double>::quiet_NaN();
      return std::atan2(2*(q.w*q.z+q.x*q.y)/n, 1-2*(q.y*q.y+q.z*q.z)/n);
    };
    const double error = std::abs(std::remainder(heading(odom_.pose.pose.orientation) -
      heading(path_.poses.back().pose.orientation), 2*std::acos(-1.0)));
    const double tolerance = task_heading_tolerance_ > 0.0 ? task_heading_tolerance_ : heading_tolerance_;
    if (!std::isfinite(error) || error > tolerance) return false;
  }
  return true;
}

double LateralController::tracking_yaw() const
{
  const auto & q = odom_.pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return task_lookahead_ > 0.0 || camera_distance_mode_ ? yaw : yaw + heading_offset_;
}

double LateralController::gain_scale(double bearing) const
{
  if (gain_errors_.empty() || gain_errors_.size() != gain_scales_.size()) return 1.0;
  const double e = std::abs(bearing);
  if (e <= gain_errors_.front()) return gain_scales_.front();
  for (std::size_t i = 1; i < gain_errors_.size(); ++i) {
    if (e <= gain_errors_[i]) {
      const double t = (e - gain_errors_[i - 1]) / (gain_errors_[i] - gain_errors_[i - 1]);
      return gain_scales_[i - 1] + t * (gain_scales_[i] - gain_scales_[i - 1]);
    }
  }
  return gain_scales_.back();
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
  if (!camera_distance_mode_ && at_goal()) {
    if (!require_goal_heading_) return std::nullopt;
    const auto & g = path_.poses.back().pose.orientation;
    desired = std::atan2(
      2.0 * (g.w * g.z + g.x * g.y), 1.0 - 2.0 * (g.y * g.y + g.z * g.z));
  } else {
    const auto target = target_point();
    desired = std::atan2(target.y - p.y, target.x - p.x);
    if (target_v < 0.0) desired += std::acos(-1.0);
    // Toward the lookahead point the robot goes where it travels, not where the pose points (heading_offset_).
    return std::remainder(desired - tracking_yaw(), 2.0 * std::acos(-1.0));
  }
  return std::remainder(desired - current, 2.0 * std::acos(-1.0));
}

// Pure Pursuit로 현재 위치에서 경로를 추종하는 데 필요한 곡률 kappa [1/m]를 계산한다.
// 목표점은 내 노드(update_node)에서 경로를 따라 lookahead 이상 간 첫 점이며, 차체 좌표(전방 +x, 좌측 +y)로
// 변환해 kappa = 2*y / (x*x + y*y)를 구한다. 경로 전체에서 차체 앞쪽 점을 찾지 않으므로 먼 구간을 집지 않고,
// 목표점이 뒤나 옆에 있으면 needs_rotation()이 제자리 회전을 먼저 시킨다. 후진(v < 0)도 같은 원을 따른다.
// odom/경로 없음, 좌표계 불일치, 종점 도달이면 nullopt를 반환한다.
std::optional<double> LateralController::get_curvature(double /*target_v*/, double * bearing) const
{
  if (!have_odom_ || path_.poses.empty() || path_.header.frame_id != odom_.header.frame_id) {
    return std::nullopt;
  }
  if (is_target_reached()) {
    return std::nullopt;
  }
  const auto & p = odom_.pose.pose.position;
  const double yaw = tracking_yaw();
  const auto point = target_point();
  const double dx = point.x - p.x, dy = point.y - p.y;
  const double x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double distance = std::hypot(x, y);
  if (distance < 1e-9) return std::nullopt;
  if (bearing) *bearing = std::atan2(y, x);
  // Circle through the robot tangent to its heading; the same arc serves reverse travel (v < 0).
  return 2.0 * (y / distance) / distance;
}

// 현재 경로·odom·진행 방향으로 유효한 추종 곡률을 구할 수 있는지 확인한다.
// false이면 ControlMaster가 longitudinal 목표를 정지시킨다.
bool LateralController::can_track(double target_v) const
{
  if (continuous_tracking_ && !path_.poses.empty() && have_odom_ &&
      arc_[node_] > kNodeWindow * path_spacing_ && !at_goal()) {
    const auto error = heading_error(target_v);
    if (!error || std::abs(*error) >= std::acos(-1.0) / 2) return false;
  }
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
  if (continuous_tracking_ && !path_.poses.empty() && have_odom_ &&
      arc_[node_] > kNodeWindow * path_spacing_ && !at_goal()) {
    const auto error = heading_error(direction);
    if (!error || std::abs(*error) >= std::acos(-1.0) / 2) return {0.0, 0.0};
  }
  // ControlMaster always supplies the direction hint; it has also cleared forward motion when can_track()
  // said to turn. Direct callers without a hint never turn in place.
  if (std::isfinite(target_v_hint) && needs_rotation(direction)) {
    // The shorter way round (user 2026-11-13: a turn kept its first side and went the long way, 323 deg at S4). Only
    // a target within kTurnBack of straight behind keeps the side the turn started on: there pose noise and a path
    // doubling back near itself flip the shorter side (box L1, 2026-11-08: 40 s of alternating turns). Within
    // kTurnSlow of the target heading the turn slows down (a fine tolerance is not overshot).
    constexpr double kTurnSlow = 0.3, kTurnMin = 0.15, kTurnBack = 0.5;  // [rad], [rad/s], [rad]
    // The offset the integral holds belongs to the direction just driven (the pose heading is 0 to 5 deg off depending
    // on place and direction): after a turn on the spot it starts again. tour_i30_1111_1928 kept it and the first
    // 0.5 m of every leg spread 1.9 cm instead of 1.0-1.3.
    integral_ = 0.0;
    const double error = *heading_error(direction);
    if (rotate_sign_ == 0 || std::abs(error) < std::acos(-1.0) - kTurnBack) rotate_sign_ = error >= 0.0 ? 1 : -1;
    // At the goal (the final heading alignment) at most goal_rotate_angular_speed_: a fast turn there overshoots and
    // slips off the goal (2026-11-10: 0.57 rad/s at a stage point, past the heading, 10 cm off, circling).
    const bool goal = !camera_distance_mode_ && goal_rotate_angular_speed_ > 0.0 && at_goal();
    const double top = goal ? std::min(goal_rotate_angular_speed_, rotate_angular_speed_) : rotate_angular_speed_;
    // Slowing down toward the heading as toward a goal point: proportional to the error within kTurnSlow, down to
    // goal_rotate_min_speed_ at the goal (2026-11-10: the 0.15 rad/s floor ran 0.87 deg past a stage heading).
    const double floor = goal && goal_rotate_min_speed_ > 0.0 ? goal_rotate_min_speed_ : kTurnMin;
    const double speed = std::clamp(top * std::abs(error) / kTurnSlow, std::min(floor, top), top);
    return wheel_delta(rotate_sign_ * speed);
  }
  double bearing = 0.0;
  const auto curvature = get_curvature(direction, &bearing);
  if (!curvature) {
    return {0.0, 0.0};
  }
  // Backwards-facing travel sees the lookahead point behind the body: the bearing off the travel direction counts.
  if (direction < 0.0) bearing = std::remainder(bearing - std::acos(-1.0), 2.0 * std::acos(-1.0));
  double kappa = gain_scale(bearing) * *curvature;
  if (integral_gain_ > 0.0 && task_lookahead_ <= 0.0 && !camera_distance_mode_) {
    // A new path can start well beside the robot (a pose jump at the stop before it, 9-11 cm, tour_1111_1716): only
    // kOffsetMax of the offset counts, so one such start cannot fill the integral.
    constexpr double kOffsetMax = 0.05;  // [m]
    if (const auto d = lateral_offset()) {
      integral_ = std::clamp(
        integral_ + integral_gain_ * std::clamp(*d, -kOffsetMax, kOffsetMax) * std::abs(linear_speed) * integral_dt_,
        -integral_limit_, integral_limit_);
    }
    // Left of the path -> turn right of the travel direction; backwards-facing travel (v < 0) mirrors the body curvature.
    kappa -= (direction < 0.0 ? -1.0 : 1.0) * integral_;
  }
  return wheel_delta(tracking_gain_ * linear_speed * kappa);
}

}  // namespace craft_control
