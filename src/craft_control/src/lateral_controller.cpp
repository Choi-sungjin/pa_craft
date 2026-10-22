#include "craft_control/lateral_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace craft_control
{

namespace
{

// 누적 현 길이 s에 대한 natural cubic spline의 각 점 2차 미분값 M을 구한다 (양 끝 M = 0).
// 삼중대각 연립방정식을 Thomas 알고리즘으로 푼다. s는 엄격히 증가해야 한다.
std::vector<double> natural_spline_second_derivatives(
  const std::vector<double> & s, const std::vector<double> & v)
{
  const std::size_t n = s.size();
  std::vector<double> m(n, 0.0);
  if (n < 3) {
    return m;
  }
  std::vector<double> c(n, 0.0);
  std::vector<double> d(n, 0.0);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const double h0 = s[i] - s[i - 1];
    const double h1 = s[i + 1] - s[i];
    const double a = h0;
    const double b = 2.0 * (h0 + h1);
    const double rhs = 6.0 * ((v[i + 1] - v[i]) / h1 - (v[i] - v[i - 1]) / h0);
    const double denom = b - a * c[i - 1];
    c[i] = h1 / denom;
    d[i] = (rhs - a * d[i - 1]) / denom;
  }
  for (std::size_t i = n - 2; i >= 1; --i) {
    m[i] = d[i] - c[i] * m[i + 1];
  }
  return m;
}

// 구간 [s[i], s[i+1]]에서 spline 값과 1차 미분을 계산한다.
std::pair<double, double> eval_spline(
  const std::vector<double> & s, const std::vector<double> & v, const std::vector<double> & m,
  std::size_t i, double t)
{
  const double h = s[i + 1] - s[i];
  const double a = (s[i + 1] - t) / h;
  const double b = (t - s[i]) / h;
  const double value = a * v[i] + b * v[i + 1] +
    ((a * a * a - a) * m[i] + (b * b * b - b) * m[i + 1]) * h * h / 6.0;
  const double slope = (v[i + 1] - v[i]) / h +
    ((1.0 - 3.0 * a * a) * m[i] + (3.0 * b * b - 1.0) * m[i + 1]) * h / 6.0;
  return {value, slope};
}

}  // namespace

// ControlMaster가 YAML에서 읽은 바퀴 반지름, 좌우 바퀴 중심 간 거리, lookahead, 도착 허용 거리,
// 경로 재샘플 간격을 저장한다. 모두 ControlMaster가 유한한 양수로 검증하며 거리 단위는 m이다.
void LateralController::configure(
  double wheel_radius, double wheel_separation, double lookahead, double goal_tolerance,
  double path_spacing)
{
  wheel_radius_ = wheel_radius;
  wheel_separation_ = wheel_separation;
  lookahead_ = lookahead;
  goal_tolerance_ = goal_tolerance;
  path_spacing_ = path_spacing;
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

// 검증된 planning 경로점(x, y)을 지나는 natural cubic spline을 누적 현 길이로 매개화한 뒤,
// spline의 실제 호 길이 기준 path_spacing 간격으로 재샘플해 저장한다 (마지막 간격만 더 짧을 수 있다).
// 호 길이는 각 구간을 path_spacing/10 이하로 잘게 나눈 표에서 선형 보간으로 구한다.
// 첫 점과 마지막 점은 원래 경로점을 그대로 유지한다.
// 연속으로 겹친 점은 하나로 합치고, 서로 다른 점이 하나뿐이면 그 점만 저장한다.
// 재샘플한 점의 자세는 spline 접선 방향 yaw이며 시각화용이다 (추종 곡률 계산에는 쓰지 않는다).
void LateralController::set_target(const nav_msgs::msg::Path & path)
{
  std::vector<double> s;
  std::vector<double> xs;
  std::vector<double> ys;
  for (const auto & pose : path.poses) {
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    if (!xs.empty()) {
      const double ds = std::hypot(x - xs.back(), y - ys.back());
      if (ds <= 1e-9) {
        continue;
      }
      s.push_back(s.back() + ds);
    } else {
      s.push_back(0.0);
    }
    xs.push_back(x);
    ys.push_back(y);
  }

  path_ = nav_msgs::msg::Path{};
  path_.header = path.header;
  if (xs.size() < 2) {
    path_.poses.push_back(path.poses.front());
    path_.poses.back().header = path.header;
    return;
  }

  const auto mx = natural_spline_second_derivatives(s, xs);
  const auto my = natural_spline_second_derivatives(s, ys);

  // (spline 매개변수, 누적 호 길이) 표.
  std::vector<double> table_t = {0.0};
  std::vector<double> table_arc = {0.0};
  double prev_x = xs.front();
  double prev_y = ys.front();
  for (std::size_t i = 0; i + 1 < s.size(); ++i) {
    const double h = s[i + 1] - s[i];
    const auto steps = static_cast<std::size_t>(std::ceil(h / (path_spacing_ / 10.0)));
    for (std::size_t j = 1; j <= steps; ++j) {
      const double t = s[i] + h * static_cast<double>(j) / static_cast<double>(steps);
      const double x = eval_spline(s, xs, mx, i, t).first;
      const double y = eval_spline(s, ys, my, i, t).first;
      table_t.push_back(t);
      table_arc.push_back(table_arc.back() + std::hypot(x - prev_x, y - prev_y));
      prev_x = x;
      prev_y = y;
    }
  }

  const double length = table_arc.back();
  const auto count = static_cast<std::size_t>(std::ceil(length / path_spacing_ - 1e-9));
  std::size_t row = 0;
  std::size_t segment = 0;
  for (std::size_t k = 0; k <= count; ++k) {
    const bool last = k == count;
    const double arc = last ? length : static_cast<double>(k) * path_spacing_;
    while (row + 2 < table_arc.size() && arc > table_arc[row + 1]) {
      ++row;
    }
    const double span = table_arc[row + 1] - table_arc[row];
    const double ratio = span > 0.0 ? (arc - table_arc[row]) / span : 0.0;
    const double t = last ? s.back() :
      table_t[row] + std::clamp(ratio, 0.0, 1.0) * (table_t[row + 1] - table_t[row]);
    while (segment + 2 < s.size() && t > s[segment + 1]) {
      ++segment;
    }
    const auto [x, dx] = eval_spline(s, xs, mx, segment, t);
    const auto [y, dy] = eval_spline(s, ys, my, segment, t);
    const double yaw = std::atan2(dy, dx);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = last ? xs.back() : x;
    pose.pose.position.y = last ? ys.back() : y;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path_.poses.push_back(pose);
  }
}

void LateralController::clear_target()
{
  path_ = nav_msgs::msg::Path{};
}

const nav_msgs::msg::Path & LateralController::target_path() const
{
  return path_;
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
