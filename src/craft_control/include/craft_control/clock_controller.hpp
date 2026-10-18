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
  bool is_valid_target(int slot) const;  // 1..slot_count
  // Store a slot after is_valid_target() succeeds; clock_angle is fresh finite feedback.
  void set_target(int slot, double clock_angle);
  void clear_target();  // hold the current angle
  // clock_angle: clock OpenCR feedback
  double compute_command(double clock_angle) const;
  int get_current_slot(double clock_angle) const;  // 1..slot_count; 0 means between slots
  // Angle only; ControlMaster also requires clock_state idle.
  bool is_target_reached(double clock_angle) const;

private:
  double target_angle_ = 0.0;
  double angle_tolerance_ = 0.0;
  int slot_count_ = 0;
  double slot_angle_ = 0.0;
  double zero_angle_ = 0.0;
  double direction_ = 0.0;
  double revolution_angle_ = 0.0;
  int target_slot_ = 0;
  bool has_target_ = false;
};

}  // namespace craft_control
