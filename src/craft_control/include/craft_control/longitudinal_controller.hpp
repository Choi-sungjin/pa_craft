#pragma once

#include <array>

namespace craft_control
{

class LongitudinalController
{
public:
  void set_target_v(double v);
  std::array<double, 2> get_wheel_speed() const;  // {left, right}

private:
  double target_v_ = 0.0;
};

}  // namespace craft_control
