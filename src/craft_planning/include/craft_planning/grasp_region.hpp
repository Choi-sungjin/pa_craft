#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <array>
#include <string>

namespace craft_planning
{
// Puck centre ranges in base_link [m]: {x_min, x_max, y_min, y_max, z_min, z_max}, measured by experiment
// for an approach along the puck's long axis. Any min >= max means "not measured": never graspable.
// TODO(grasp region): the axis-aligned box is a placeholder shape. Replace shape and values with the user's
// per-type measurements (see HANDOFF.md); do not guess them.
using GraspBox = std::array<double, 6>;

struct GraspRegionConfig
{
  GraspBox lying{}, standing{};
  double margin = 0.0;           // [m] every face moves inward by this (observation noise)
  double axis_tolerance = 0.17;  // [rad] lying puck long axis vs chassis forward, 180 degree symmetric
  int confirm_count = 3;         // consecutive graspable observations before Graspable::confirmed()
  // Stop cost along the approach (chassis forward), see plan_stop(). Measured values: 0 = not set yet.
  // TODO(grasp region): front_face_x, front_face_width, good_plane, min_box_gap from the user's measurements.
  double front_face_x = 0.0;      // [m] base_link x of the front face (LM guide plane)
  double front_face_width = 0.0;  // [m] its width, centred on base_link y = 0
  double good_plane = 0.0;        // [m] s_good: front face -> puck centre distance that grasps best
  double min_box_gap = 0.03;      // [m] safety gap: box cost w_box / (d_short - min_box_gap) is infinite there
  double w_box = 0.003;           // [m] weight of the box barrier (tuning)
  double w_grasp = 1.0;           // weight of ((s - s_good) / sigma)^2 (tuning)
};

// One puck observation already in base_link. Lying: axis = long axis (horizontal). Standing: axis = shelf
// normal, so the approach direction is free.
struct PuckInBase
{
  geometry_msgs::msg::Point center;
  geometry_msgs::msg::Vector3 axis;
  bool standing = false;
};

struct GraspCheck
{
  bool graspable = false;
  std::string reason;  // why not, or the remaining margins when graspable
};

bool measured(const GraspBox & box);
// Throws invalid_argument for non-finite values, a negative margin/tolerance or confirm_count < 1.
void validate(const GraspRegionConfig & config);
bool stop_cost_configured(const GraspRegionConfig & config);
// Alignment is the precondition: a lying puck's long axis must be within axis_tolerance of chassis forward
// (standing: always aligned). Only then is the centre tested against the measured region of its type (with
// margin). No IK: the measured region already stands for reachability.
GraspCheck check_graspable(const GraspRegionConfig & config, const PuckInBase & puck);

// Box from LiDAR, in base_link: centre and yaw of the square (any side as reference).
struct BoxInBase
{
  double x = 0, y = 0, yaw = 0, side = 0.205;
};

// Best stop along the approach line (chassis forward): advance (m, < 0 = back up) minimising
//   J = w_box / (d_short - min_box_gap) + w_grasp * ((s - good_plane) / sigma)^2
// d_short: shortest distance front-face segment <-> box square; s: front face -> puck centre (x);
// sigma: half the puck type's region length in x. The box barrier is infinite at the safety gap, so
// d_short > min_box_gap always; check_graspable() must hold at the stop (alignment first, then region).
// 1 mm grid over the feasible advances, then golden section.
struct StopPlan
{
  bool feasible = false;
  std::string reason;
  double advance = 0, cost = 0, box_cost = 0, grasp_cost = 0, box_gap = 0, puck_s = 0;
};
double front_face_to_box(const GraspRegionConfig & config, const BoxInBase & box);
StopPlan plan_stop(const GraspRegionConfig & config, const PuckInBase & puck, const BoxInBase & box);

// Debounce: confirmed after confirm_count graspable checks in a row; any other check restarts the count.
class GraspConfirm
{
public:
  bool update(const GraspCheck & check, int confirm_count);
  void reset() {count_ = 0;}
  int count() const {return count_;}

private:
  int count_ = 0;
};
}  // namespace craft_planning
