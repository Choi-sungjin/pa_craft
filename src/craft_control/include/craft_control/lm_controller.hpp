#pragma once

#include <optional>
#include <string>

namespace craft_control
{

class LmController
{
public:
  // ControlMaster supplies YAML configuration. Limits/offset are motor rad, target is metres.
  bool configure(
    const std::string & joint_name, double min_angle, double max_angle,
    double direction, double offset, double pulley_radius, double gear_ratio, double tolerance);
  bool is_valid_target(const std::string & joint_name, double position) const;
  void set_target(double position);  // store a validated linear position [m]
  void clear_target();
  double compute_command(double current_angle) const;  // motor rad
  // Invert measured motor radians into slide metres; unavailable without calibration.
  std::optional<double> get_joint_position(double current_angle) const;
  const std::string & get_joint_name() const {return joint_name_;}
  bool is_target_reached(double current_angle) const;  // within tolerance [motor rad]

private:
  std::string joint_name_;
  double min_angle_ = 0.0;
  double max_angle_ = 0.0;
  double direction_ = 0.0;
  double offset_ = 0.0;
  double pulley_radius_ = 0.0;
  double gear_ratio_ = 0.0;
  double tolerance_ = 0.0;
  double target_position_ = 0.0;
  bool conversion_ready_ = false;
  bool has_target_ = false;
};

}  // namespace craft_control
