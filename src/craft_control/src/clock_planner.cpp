#include "craft_control/clock_planner.hpp"

namespace craft_control
{

void ClockPlanner::set_clock_pose(double pose)
{
  clock_pose_ = pose;
  has_target_ = true;
}

double ClockPlanner::get_clock_pose(double clock_angle) const
{
  if (!has_target_) {
    return clock_angle;  // hold current angle
  }
  // TODO: clock target from mission time and clock_angle
  return clock_pose_;
}

int ClockPlanner::get_clock_status(int clock_state) const
{
  return clock_state;
}

}  // namespace craft_control
