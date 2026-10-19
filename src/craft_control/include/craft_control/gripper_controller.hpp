#pragma once

namespace craft_control
{

class GripperController
{
public:
  bool configure(
    double min_angle, double max_angle, double open_angle, double close_angle,
    double grasp_current_threshold_a, double grasp_current_debounce, double tolerance);
  // Store open/close after ControlMaster validation; configure() must have succeeded.
  void set_target(bool close);
  double compute_command(double current_angle) const;
  // Open: within tolerance of the open angle. Close: within tolerance of the close angle, or grasped.
  bool is_target_reached(double current_angle) const;
  void update_current_feedback(double motor_current_a, double dt);
  bool is_grasped() const;
  bool grasp_detection_configured() const;

private:
  double min_angle_ = 0.0;
  double max_angle_ = 0.0;
  double open_angle_ = 0.0;
  double close_angle_ = 0.0;
  double target_angle_ = 0.0;
  double tolerance_ = 0.0;
  double grasp_current_threshold_a_ = 0.0;
  double grasp_current_debounce_ = 0.0;
  double overcurrent_time_ = 0.0;
  bool grasp_detection_configured_ = false;
  bool has_target_ = false;
  bool closing_ = false;
  bool grasped_ = false;
};

}  // namespace craft_control
