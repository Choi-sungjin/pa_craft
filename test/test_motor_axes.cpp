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

// Rear wheels (IDs 3, 4) copy their side's command; feedback is the side mean; a rear motor
// error latches the board fault and stops every wheel.
void checkFollowers() {
  using namespace ControlTableItem;
  using wheel_clock::kAxes;
  using wheel_clock::kFollowers;
  assert(kFollowers[0].id == 3 && kFollowers[0].leader == 0 && kFollowers[0].direction == 1);
  assert(kFollowers[1].id == 4 && kFollowers[1].leader == 1 && kFollowers[1].direction == -1);
  Serial = {};
  test_time_ms = 0;
  Dynamixel2Arduino bus(Serial3, 84);
  craft::MotorBoard<3> board(bus, kAxes, true, 1000000);
  board.setFollowers(kFollowers);
  board.begin();
  for (int id : {3, 4}) {
    assert((bus.registers[{id, TORQUE_ENABLE}] == 1));
    assert((bus.registers[{id, GOAL_VELOCITY}] == 0));
    assert((bus.registers[{id, PROFILE_ACCELERATION}] == kAxes[0].profile_acceleration));
  }
  const double tick = kAxes[0].units_per_tick;
  Serial.input = "1.5,-2.0,0\n";
  test_time_ms += craft::kPeriodMs;
  board.tick();
  const int32_t left = std::llround(1.5 / tick), right = std::llround(-2.0 * -1 / tick);
  assert((bus.registers[{1, GOAL_VELOCITY}] == left && bus.registers[{3, GOAL_VELOCITY}] == left));
  assert((bus.registers[{2, GOAL_VELOCITY}] == right && bus.registers[{4, GOAL_VELOCITY}] == right));

  // Left rear slips 20 ticks faster: wheel_l reports the mean.
  bus.registers[{1, PRESENT_VELOCITY}] = left;
  bus.registers[{3, PRESENT_VELOCITY}] = left + 20;
  bus.registers[{2, PRESENT_VELOCITY}] = right;
  bus.registers[{4, PRESENT_VELOCITY}] = right;
  Serial.output.clear();
  test_time_ms += craft::kPeriodMs;
  board.tick();
  double wl, wr;
  char comma;
  std::istringstream feedback(Serial.output);
  assert(static_cast<bool>(feedback >> wl >> comma >> wr));
  assert(std::fabs(wl - (left + 10) * tick) < 1e-5 && std::fabs(wr + 2.0) < tick);

  bus.registers[{4, HARDWARE_ERROR_STATUS}] = 0x20;  // overload on the right rear
  Serial.output.clear();
  test_time_ms += craft::kPeriodMs;
  board.tick();
  assert(Serial.output.find(",2,2") != std::string::npos);
  for (int id : {1, 2, 3, 4}) assert((bus.registers[{id, GOAL_VELOCITY}] == 0));
  std::cout << "PASS rear wheels (ID 3, 4): follow side command, mean feedback, fault stop\n";
}

int main() {
  // Hardware observations: raw + drives left forward and right backward.
  assert(wheel_clock::kAxes[0].direction == 1);
  assert(wheel_clock::kAxes[1].direction == -1);
  assert(wheel_clock::kAxes[2].direction == 1);
  const unsigned wheel_ids[] = {1, 2, 5};
  const char *wheel_names[] = {"wheel_l", "wheel_r", "clock"};
  checkAxes(wheel_clock::kAxes, wheel_ids, wheel_names);
  checkFollowers();
  const unsigned arm_ids[] = {1, 2, 3, 4, 5, 6};
  const char *arm_names[] = {"LM", "shoulder", "elbow", "wrist_pitch", "tool_roll", "gripper"};
  checkAxes(lm_arm::kAxes, arm_ids, arm_names, lm_arm::kGripperCurrentAPerTick);
}
