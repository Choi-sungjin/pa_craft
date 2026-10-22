#include <assert.h>
#include <iostream>
#include <CraftMotorFirmware.h>

uint32_t test_time_ms = 0;
HardwareSerial Serial3;
TestSerial Serial;

template<unsigned N>
bool frame(craft::CsvReceiver<N> &receiver, const std::string &line, double (&out)[N]) {
  bool accepted = false;
  for (char c : line) accepted = receiver.push(c, out) || accepted;
  return accepted;
}

void protocol() {
  craft::CsvReceiver<3> receiver;
  double values[3] = {9, 9, 9};
  assert(frame(receiver, "1,-2, 3.25\r\n", values));
  assert(values[0] == 1 && values[1] == -2 && values[2] == 3.25);
  for (const auto &bad : {"1,2\n", "1,2,3,\n", "1,2,nan\n", "1,2,inf\n",
       "1,2,1e999\n", "1,2,3x\n", "1,2,3,4\n", "\n"}) {
    assert(!frame(receiver, bad, values));
    assert(values[0] == 1 && values[1] == -2 && values[2] == 3.25);
  }
  assert(!frame(receiver, std::string(300, '0') + "1,2,3\n", values));
  assert(!frame(receiver, std::string("1,2,3\0junk\n", 11), values));
  assert(frame(receiver, "4,5,6\n", values));
  assert(!craft::commandExpired(0x20, 0xfffffff0, true, 100));
  assert(craft::commandExpired(0x80, 0xfffffff0, true, 100));
  assert(craft::commandExpired(0, 0, false, 100));
}

void wheelClock() {
  using namespace ControlTableItem;
  Serial = {};
  test_time_ms = 0;
  const craft::AxisConfig axes[3] = {
    {1, true, 0.01, 0, 1, -8, 8, 10, 0, 0.05},
    {2, true, 0.01, 0, -1, -8, 8, 10, 0, 0.05},
    {3, false, 0.001, 100, 1, -100, 100, 10, 30, 0.035},
  };
  Dynamixel2Arduino bus(Serial3, 84);
  bus.registers[{3, PRESENT_POSITION}] = 1200;
  craft::MotorBoard<3> board(bus, axes, true, 1000000);
  board.begin();
  assert((bus.registers[{3, GOAL_POSITION}] == 1200)); // startup hold
  Serial.input = "1,2,-7\n";
  board.tick();
  assert((bus.registers[{1, GOAL_VELOCITY}] == 100));
  assert((bus.registers[{2, GOAL_VELOCITY}] == -200));
  assert((bus.registers[{3, GOAL_POSITION}] == -6900)); // unwrapped negative angle
  Serial.input = "1,2,20\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{3, GOAL_POSITION}] == 20100)); // above 2*pi
  test_time_ms += 250;
  bus.registers[{3, PRESENT_POSITION}] = 3000;
  Serial.input = "9,0,0\n"; // entire out-of-range command rejected, no watchdog feed
  board.tick();
  assert((bus.registers[{1, GOAL_VELOCITY}] == 0));
  assert((bus.registers[{3, GOAL_POSITION}] == 3000));
  assert(Serial.output.find(",2,2\n") != std::string::npos);
  test_time_ms += 20;
  bus.registers[{3, PRESENT_POSITION}] = 3005;
  board.tick();
  assert((bus.registers[{3, GOAL_POSITION}] == 3000)); // hold is captured once
  Serial.input = "0,0,4\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{3, GOAL_POSITION}] == 4100)); // timeout can recover
  bus.registers[{3, HARDWARE_ERROR_STATUS}] = 4;
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{3, GOAL_POSITION}] == 3005)); // fault uses fresh hold
  bus.registers[{3, HARDWARE_ERROR_STATUS}] = 0;
  Serial.input = "0,0,5\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{3, GOAL_POSITION}] == 3005)); // motor fault latches

  Serial = {};
  Dynamixel2Arduino unconfirmed_bus(Serial3, 84);
  craft::MotorBoard<3> unconfirmed(unconfirmed_bus, axes, false, 1000000);
  unconfirmed.begin();
  unconfirmed.tick();
  assert(unconfirmed_bus.writes == 0);
  assert(Serial.output.find(",2,2\n") != std::string::npos);
}

void manipulator() {
  using namespace ControlTableItem;
  Serial = {};
  craft::AxisConfig axes[6];
  for (unsigned i = 0; i < 6; ++i) {
    axes[i] = {static_cast<uint8_t>(i + 1), false, 0.001, 0, 1,
               -10, 10, 10, 30, 0.035};
  }
  Dynamixel2Arduino bus(Serial3, 84);
  bus.registers[{6, PRESENT_CURRENT}] = 65526; // uint16 encoding of -10
  craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
  board.begin();
  Serial.input = "1,2,3,4,5,6\n";
  board.tick();
  for (unsigned i = 0; i < 6; ++i) {
    assert((bus.registers[{i + 1, GOAL_POSITION}] == static_cast<int32_t>((i + 1) * 1000)));
  }
  assert(Serial.output.find(",1,-0.026900\n") != std::string::npos);
  bus.failed_item = PRESENT_CURRENT;
  test_time_ms += 20;
  board.tick();
  assert(Serial.output.find(",2,-0.026900\n") != std::string::npos);
}

int main() {
  protocol();
  wheelClock();
  manipulator();
  std::cout << "Firmware protocol, watchdog, fault and channel tests passed\n";
}
