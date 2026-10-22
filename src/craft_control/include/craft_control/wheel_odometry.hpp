#pragma once

namespace craft_control
{

// Differential-drive dead reckoning from measured wheel speeds [rad/s].
// Same sign convention as LateralController: wr > wl turns left (+yaw).
class WheelOdometry
{
public:
  void configure(double wheel_radius, double wheel_separation);
  // Midpoint integration over dt [s]; non-positive or non-finite inputs are ignored.
  void update(double wheel_left, double wheel_right, double dt);
  double x() const {return x_;}
  double y() const {return y_;}
  double yaw() const {return yaw_;}
  double linear() const {return v_;}
  double angular() const {return w_;}

private:
  double radius_ = 0.0, separation_ = 0.0;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, v_ = 0.0, w_ = 0.0;
};

}  // namespace craft_control
