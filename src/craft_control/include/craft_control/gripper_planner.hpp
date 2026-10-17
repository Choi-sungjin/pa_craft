#pragma once

namespace craft_control
{

class GripperPlanner
{
public:
  bool configure(
    double min_angle, double max_angle, double open_angle, double close_angle,
    double grasp_current_threshold_a, double grasp_current_debounce);
  bool set_target(bool close);
  double compute_gripper_command(double current_angle) const;
  bool is_target_reached(double current_angle, double tolerance) const;
  void update_current_feedback(double motor_current_a, double dt);
  bool is_grasped() const;
  bool grasp_detection_configured() const;

private:
  double min_angle_ = 0.0;
  double max_angle_ = 0.0;
  double open_angle_ = 0.0;
  double close_angle_ = 0.0;
  double target_angle_ = 0.0;
  double grasp_current_threshold_a_ = 0.0;
  double grasp_current_debounce_ = 0.0;
  double overcurrent_time_ = 0.0;
  bool configured_ = false;
  bool grasp_detection_configured_ = false;
  bool has_target_ = false;
  bool closing_ = false;
  bool grasped_ = false;
};

}  // namespace craft_control