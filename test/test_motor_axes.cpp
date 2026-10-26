#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <CraftMotorFirmware.h>

// Keep both boards' production configurations, including their CSV ordering.
namespace wheel_clock {
#include "../firmware/wheel_clock/config.h"
}
namespace lm_arm {
#include "../firmware/lm_arm/config.h"
}

uint32_t test_time_ms = 0;
HardwareSerial Serial3;
TestSerial Serial;

template<unsigned N>
void checkAxes(const craft::AxisConfig (&axes)[N], const unsigned (&ids)[N],
               const char *const (&names)[N], double current_scale = 0) {
  using namespace ControlTableItem;
  Serial = {};
  test_time_ms = 0;
  Dynamixel2Arduino bus(Serial3, 84);
  double baseline[N];
  for (unsigned i = 0; i < N; ++i) {
    assert(axes[i].id == ids[i]);
    assert(axes[i].velocity == (N == 3 && i < 2));
    baseline[i] = axes[i].velocity ? 0 : (axes[i].minimum + axes[i].maximum) / 2;
    bus.registers[{ids[i], PRESENT_POSITION}] = static_cast<int32_t>(std::llround(
        baseline[i] * axes[i].direction / axes[i].units_per_tick + axes[i].zero_tick));
  }
  // Enable only the fake board; production confirmation flags remain untouched.
  craft::MotorBoard<N> board(bus, axes, true, 1000000, current_scale);
  board.begin();
  for (unsigned i = 0; i < N; ++i) {
    assert((bus.registers[{ids[i], TORQUE_ENABLE}] == 1));
    const int item = axes[i].velocity ? GOAL_VELOCITY : GOAL_POSITION;
    assert((bus.registers[{ids[i], item}] == (axes[i].velocity ? 0 :
        bus.registers[{ids[i], PRESENT_POSITION}])));
  }

  for (unsigned selected = 0; selected < N; ++selected) {
    for (int sign : {-1, 1}) {
      double targets[N];
      std::ostringstream command;
      command << std::setprecision(17);
      for (unsigned i = 0; i < N; ++i) {
        targets[i] = baseline[i] + (i == selected ? sign * 0.05 : 0);
        assert(targets[i] >= axes[i].minimum && targets[i] <= axes[i].maximum);
        if (i) command << ',';
        command << targets[i];
      }
      Serial.input = command.str() + "\n";
      test_time_ms += craft::kPeriodMs;
      board.tick();
      for (unsigned i = 0; i < N; ++i) {
        const int item = axes[i].velocity ? GOAL_VELOCITY : GOAL_POSITION;
        const int32_t expected = static_cast<int32_t>(std::llround(
            targets[i] * axes[i].direction / axes[i].units_per_tick +
            (axes[i].velocity ? 0 : axes[i].zero_tick)));
        assert((bus.registers[{ids[i], item}] == expected));
        // Simulate arrival, then verify feedback returns in CSV axis order.
        bus.registers[{ids[i], axes[i].velocity ? PRESENT_VELOCITY : PRESENT_POSITION}] = expected;
      }
      Serial.output.clear();
      test_time_ms += craft::kPeriodMs;
      board.tick();
      std::istringstream feedback(Serial.output);
      for (unsigned i = 0; i < N; ++i) {
        std::string field;
        assert(static_cast<bool>(std::getline(feedback, field, ',')));
        assert(std::fabs(std::stod(field) - targets[i]) <= axes[i].units_per_tick / 2 + 1e-6);
      }
    }
    std::cout << "PASS " << names[selected] << " (ID " << ids[selected]
              << "): startup hold, +/- command isolation, feedback mapping\n";
  }
}

int main() {
  // Hardware observations: raw + drives left forward and right backward.
  assert(wheel_clock::kAxes[0].direction == 1);
  assert(wheel_clock::kAxes[1].direction == -1);
  assert(wheel_clock::kAxes[2].direction == 1);
  const unsigned wheel_ids[] = {1, 2, 3};
  const char *wheel_names[] = {"wheel_l", "wheel_r", "clock"};
  checkAxes(wheel_clock::kAxes, wheel_ids, wheel_names);
  const unsigned arm_ids[] = {1, 2, 3, 4, 5, 6};
  const char *arm_names[] = {"LM", "shoulder", "elbow", "wrist_pitch", "tool_roll", "gripper"};
  checkAxes(lm_arm::kAxes, arm_ids, arm_names, lm_arm::kGripperCurrentAPerTick);
}
