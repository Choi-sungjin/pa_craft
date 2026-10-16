#pragma once

namespace craft_control
{

class ClockPlanner
{
public:
  void set_clock_pose(double pose);
  // clock_angle: clock OpenCR feedback
  double get_clock_pose(double clock_angle) const;
  // clock_state: clock OpenCR state (0 idle, 1 using, 2 fault)
  int get_clock_status(int clock_state) const;

private:
  double clock_pose_ = 0.0;
  bool has_target_ = false;
};

}  // namespace craft_control
