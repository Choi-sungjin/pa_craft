#include "craft_planning/grasp_region.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <stdexcept>

namespace craft_planning
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

bool measured(const GraspBox & box)
{
  return box[0] < box[1] && box[2] < box[3] && box[4] < box[5];
}

void validate(const GraspRegionConfig & config)
{
  for (const auto * box : {&config.lying, &config.standing}) {
    for (const double v : *box) {
      if (!std::isfinite(v)) {throw std::invalid_argument("grasp_region values must be finite");}
    }
  }
  if (!std::isfinite(config.margin) || config.margin < 0 || !std::isfinite(config.axis_tolerance) ||
    config.axis_tolerance < 0 || config.axis_tolerance > kPi / 2 || config.confirm_count < 1)
  {
    throw std::invalid_argument("grasp_region margin/axis tolerance/confirm_count out of range");
  }
  for (const double v : {config.front_face_x, config.front_face_width, config.good_plane, config.min_box_gap,
       config.w_box, config.w_grasp}) {
    if (!std::isfinite(v)) {throw std::invalid_argument("grasp_region stop cost values must be finite");}
  }
  if (config.front_face_width < 0 || config.good_plane < 0 || config.min_box_gap < 0 || config.w_box < 0 ||
    config.w_grasp < 0)
  {
    throw std::invalid_argument("grasp_region stop cost widths, distances and weights must be nonnegative");
  }
}

bool stop_cost_configured(const GraspRegionConfig & config)
{
  return config.front_face_width > 0 && config.good_plane > 0 && config.min_box_gap > 0;
}

namespace
{
struct P {double x, y;};
double point_segment(P p, P a, P b)
{
  const double vx = b.x - a.x, vy = b.y - a.y, length = vx * vx + vy * vy;
  const double t = length > 0 ? std::clamp(((p.x - a.x) * vx + (p.y - a.y) * vy) / length, 0.0, 1.0) : 0.0;
  return std::hypot(p.x - a.x - t * vx, p.y - a.y - t * vy);
}
bool segments_cross(P a, P b, P c, P d)
{
  const auto side = [](P o, P p, P q) {return (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);};
  return side(a, b, c) * side(a, b, d) < 0 && side(c, d, a) * side(c, d, b) < 0;
}
}  // namespace

double front_face_to_box(const GraspRegionConfig & config, const BoxInBase & box)
{
  const P f0{config.front_face_x, -config.front_face_width / 2}, f1{config.front_face_x, config.front_face_width / 2};
  const double h = box.side / 2, c = std::cos(box.yaw), s = std::sin(box.yaw);
  const double u[4][2] = {{h, h}, {-h, h}, {-h, -h}, {h, -h}};
  std::array<P, 4> k;
  for (int i = 0; i < 4; ++i) {k[i] = {box.x + c * u[i][0] - s * u[i][1], box.y + s * u[i][0] + c * u[i][1]};}
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 4; ++i) {
    const P a = k[i], b = k[(i + 1) % 4];
    if (segments_cross(f0, f1, a, b)) {return 0.0;}
    best = std::min({best, point_segment(f0, a, b), point_segment(f1, a, b), point_segment(a, f0, f1), point_segment(b, f0, f1)});
  }
  return best;
}

StopPlan plan_stop(const GraspRegionConfig & config, const PuckInBase & puck, const BoxInBase & box)
{
  StopPlan plan;
  if (!stop_cost_configured(config)) {plan.reason = "stop cost not configured (front face, good plane, safety gap)"; return plan;}
  const auto & region = puck.standing ? config.standing : config.lying;
  if (!measured(region)) {plan.reason = check_graspable(config, puck).reason; return plan;}
  // Advancing by a moves the puck and the box by -a in x; alignment, y and z do not change.
  const auto at = [&](double a) {
      PuckInBase p = puck; p.center.x -= a;
      BoxInBase b = box; b.x -= a;
      return std::pair{p, b};};
  const double x_lo = region[0] + config.margin, x_hi = region[1] - config.margin;
  // Advances that put the puck centre inside the region's x range (empty if the region is narrower than 2 margins).
  const double a_lo = puck.center.x - x_hi, a_hi = puck.center.x - x_lo;
  if (!(a_lo <= a_hi)) {plan.reason = "grasp region shorter than twice the margin"; return plan;}
  {
    const auto check = check_graspable(config, at(0.5 * (a_lo + a_hi)).first);
    if (!check.graspable) {plan.reason = check.reason; return plan;}  // alignment, y or z: no advance fixes it
  }
  const double sigma = 0.5 * (region[1] - region[0]);
  struct Eval {bool ok; double cost, box_cost, grasp_cost, gap, s;};
  const auto evaluate = [&](double a) {
      const auto [p, b] = at(a);
      const double gap = front_face_to_box(config, b), s = p.center.x - config.front_face_x;
      // Barrier at the safety gap: infinite at d_short = min_box_gap, so that gap is never reached.
      Eval e{gap > config.min_box_gap, 0, 0, 0, gap, s};
      if (e.ok) {
        e.box_cost = config.w_box / (gap - config.min_box_gap);
        e.grasp_cost = config.w_grasp * std::pow((s - config.good_plane) / sigma, 2);
        e.cost = e.box_cost + e.grasp_cost;
      }
      return e;};
  constexpr double kStep = 0.001;
  double best_a = 0; Eval best{false, 0, 0, 0, 0, 0};
  const int n = static_cast<int>(std::ceil((a_hi - a_lo) / kStep));
  for (int i = 0; i <= n; ++i) {
    const double a = std::min(a_lo + i * kStep, a_hi);
    const auto e = evaluate(a);
    if (e.ok && (!best.ok || e.cost < best.cost)) {best = e; best_a = a;}
  }
  if (!best.ok) {
    char text[128];
    std::snprintf(text, sizeof(text), "no stop beyond the %.3f m safety gap inside the grasp region", config.min_box_gap);
    plan.reason = text;
    return plan;
  }
  // Golden section within one grid step either side (still inside the feasible set).
  double lo = std::max(a_lo, best_a - kStep), hi = std::min(a_hi, best_a + kStep);
  const double r = 0.5 * (std::sqrt(5.0) - 1);
  for (int i = 0; i < 30; ++i) {
    const double m1 = hi - r * (hi - lo), m2 = lo + r * (hi - lo);
    const auto e1 = evaluate(m1), e2 = evaluate(m2);
    const double c1 = e1.ok ? e1.cost : std::numeric_limits<double>::infinity();
    const double c2 = e2.ok ? e2.cost : std::numeric_limits<double>::infinity();
    (c1 < c2 ? hi : lo) = c1 < c2 ? m2 : m1;
  }
  const double refined = 0.5 * (lo + hi);
  const auto e = evaluate(refined);
  if (e.ok && e.cost <= best.cost) {best = e; best_a = refined;}
  plan.feasible = true;
  plan.advance = best_a; plan.cost = best.cost; plan.box_cost = best.box_cost; plan.grasp_cost = best.grasp_cost;
  plan.box_gap = best.gap; plan.puck_s = best.s;
  char text[160];
  std::snprintf(text, sizeof(text), "advance %.3f m: box gap %.3f m, puck %.3f m from the front face", best_a, best.gap, best.s);
  plan.reason = text;
  return plan;
}

GraspCheck check_graspable(const GraspRegionConfig & config, const PuckInBase & puck)
{
  const auto & box = puck.standing ? config.standing : config.lying;
  const char * type = puck.standing ? "standing" : "lying";
  char text[160];
  GraspCheck result;
  if (!measured(box)) {
    std::snprintf(text, sizeof(text), "%s grasp region not measured", type);
    result.reason = text;
    return result;
  }
  // 1. Alignment first: the region was measured approaching along the long axis, so it means nothing
  //    until the chassis is aligned. The long axis is 180 degree symmetric. A standing puck's axis is the
  //    shelf normal: any approach direction counts as aligned.
  if (!puck.standing) {
    const double horizontal = std::hypot(puck.axis.x, puck.axis.y);
    if (!(horizontal > 1e-6)) {result.reason = "lying puck axis has no horizontal direction"; return result;}
    const double off = std::atan2(std::abs(puck.axis.y), std::abs(puck.axis.x));
    if (off > config.axis_tolerance) {
      std::snprintf(text, sizeof(text), "not aligned: lying axis %.1f deg off forward (tolerance %.1f deg)",
        off * 180 / kPi, config.axis_tolerance * 180 / kPi);
      result.reason = text;
      return result;
    }
  }
  // 2. Aligned: the puck centre must lie in the measured region, shrunk by the margin.
  const double value[] = {puck.center.x, puck.center.y, puck.center.z};
  const char * const names[] = {"x", "y", "z"};
  double slack = 1e9;
  for (int k = 0; k < 3; ++k) {
    const double lo = box[2 * k] + config.margin, hi = box[2 * k + 1] - config.margin;
    if (!(value[k] >= lo && value[k] <= hi)) {
      std::snprintf(text, sizeof(text), "aligned, %s centre %s %.3f m outside [%.3f, %.3f]",
        type, names[k], value[k], lo, hi);
      result.reason = text;
      return result;
    }
    slack = std::min({slack, value[k] - lo, hi - value[k]});
  }
  result.graspable = true;
  std::snprintf(text, sizeof(text), "aligned, inside %s region, %.3f m to its nearest face", type, slack);
  result.reason = text;
  return result;
}

bool GraspConfirm::update(const GraspCheck & check, int confirm_count)
{
  count_ = check.graspable ? count_ + 1 : 0;
  return count_ >= confirm_count;
}
}  // namespace craft_planning
