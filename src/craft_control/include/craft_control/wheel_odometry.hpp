#pragma once

#include <vector>

namespace craft_control
{

// Differential-drive dead reckoning from measured wheel speeds [rad/s].
// Same sign convention as LateralController: wr > wl turns left (+yaw).
// In-place turns (wheels opposite, |wl + wr| < in_place_ratio * |wr - wl|) report zero linear speed:
// the scrubbing wheels slip unevenly, so their mean speed is slip, not travel.
// Yaw rate: radius * (wr - wl) / separation, or with yaw_rate_coeffs c1..cn the measured skid steer map
// sign(D) * sum(c_k * |D|^k), D = wr - wl [rad/s] (test/drive/turn_map.py poly).
class WheelOdometry
{
public:
  void configure(
    double wheel_radius, double wheel_separation, double in_place_ratio = 0.0,
    std::vector<double> yaw_rate_coeffs = {});
  // Midpoint integration over dt [s]; non-positive or non-finite inputs are ignored.
  void update(double wheel_left, double wheel_right, double dt);
  double x() const {return x_;}
  double y() const {return y_;}
  double yaw() const {return yaw_;}
  double linear() const {return v_;}
  double angular() const {return w_;}

private:
  double radius_ = 0.0, separation_ = 0.0, in_place_ratio_ = 0.0;
  std::vector<double> yaw_rate_coeffs_;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, v_ = 0.0, w_ = 0.0;
};

}  // namespace craft_control
