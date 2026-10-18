#pragma once

namespace craft_control
{

class ClockController
{
public:
  // Geometry comes from control.yaml through ControlMaster; angles are motor radians.
  bool configure(
    int slot_count, double slot_angle, double zero_angle, double direction, double angle_tolerance);
  // YAML defaults: 0 rad = slot 12, positive (CCW) increases slots, 30 degrees per slot.
  // Command/feedback are unwrapped motor radians.
  bool set_target_slot(int slot, double clock_angle);
  // clock_angle: clock OpenCR feedback
  double compute_clock_command(double clock_angle) const;
  int get_current_slot(double clock_angle) const;  // 1..slot_count; 0 means between slots
  bool is_target_reached(double clock_angle, int clock_state) const;
  // state: clock actuator state (0 idle, 1 using, 2 fault)
  int get_clock_status(int state) const;

private:
  double target_angle_ = 0.0;
  double angle_tolerance_ = 0.0;
  int slot_count_ = 0;
  double slot_angle_ = 0.0;
  double zero_angle_ = 0.0;
  double direction_ = 0.0;
  double revolution_angle_ = 0.0;
  int target_slot_ = 0;
  bool configured_ = false;
  bool has_target_ = false;
};

}  // namespace craft_control
