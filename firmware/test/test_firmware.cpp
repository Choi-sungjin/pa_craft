#include <assert.h>
#include <iostream>
#include <CraftMotorFirmware.h>
#include <CraftImu.h>

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
  assert(receiver.takeKeyword() == craft::Keyword::kNone);
  assert(!frame(receiver, "RELAX\r\n", values));
  assert(receiver.takeKeyword() == craft::Keyword::kRelax);
  assert(receiver.takeKeyword() == craft::Keyword::kNone); // read once
  assert(!frame(receiver, "HOLD\n", values));
  assert(receiver.takeKeyword() == craft::Keyword::kHold);
  for (const auto &bad : {"RELAXX\n", "relax\n", " HOLD\n", "HOLD,1,2\n"}) {
    assert(!frame(receiver, bad, values));
    assert(receiver.takeKeyword() == craft::Keyword::kNone);
  }
  assert(values[0] == 4 && values[1] == 5 && values[2] == 6);
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
  const uint16_t i_gains[6] = {0, 100, 100, 100, 100, 0};
  board.setPositionIGains(i_gains);
  board.begin();
  for (unsigned i = 0; i < 6; ++i) {  // zero gains are left unwritten
    assert((bus.registers.count({i + 1, POSITION_I_GAIN}) == (i_gains[i] ? 1u : 0u)));
    if (i_gains[i]) assert((bus.registers[{i + 1, POSITION_I_GAIN}] == i_gains[i]));
  }
  Serial.input = "1,2,3,4,5,6\n";
  board.tick();
  for (unsigned i = 0; i < 6; ++i) {
    assert((bus.registers[{i + 1, GOAL_POSITION}] == static_cast<int32_t>((i + 1) * 1000)));
  }
  assert(Serial.output.find(",1,-0.026900\n") != std::string::npos);

  // RELAX: torque off, commands ignored, moved-by-hand positions tracked without a fault.
  Serial.input = "RELAX\n";
  test_time_ms += 20;
  board.tick();
  for (unsigned i = 1; i <= 6; ++i) assert((bus.registers[{i, TORQUE_ENABLE}] == 0));
  bus.registers[{2, PRESENT_POSITION}] = 7777;
  Serial = {};
  Serial.input = "0,0,0,0,0,0\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{1, GOAL_POSITION}] == 1000));
  assert(Serial.output.find("7.777000") != std::string::npos);
  assert(Serial.output.find(",2,-0.026900\n") != std::string::npos);
  // HOLD: torque back on at the hand-moved position; commands work again.
  Serial.input = "HOLD\n";
  test_time_ms += 20;
  board.tick();
  for (unsigned i = 1; i <= 6; ++i) assert((bus.registers[{i, TORQUE_ENABLE}] == 1));
  assert((bus.registers[{2, GOAL_POSITION}] == 7777));
  Serial.input = "1,2,3,4,5,6\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{2, GOAL_POSITION}] == 2000));
  bus.registers[{2, PRESENT_POSITION}] = 2000;

  bus.failed_item = PRESENT_CURRENT;
  test_time_ms += 20;
  board.tick();
  assert(Serial.output.find(",2,-0.026900\n") != std::string::npos);
}

// Power-up reads less than one turn: a single-turn-range axis read outside its limits is unwrapped
// by the one whole turn that fits; a multi-turn range (LM) with several candidates is left alone.
void bootUnwrap() {
  using namespace ControlTableItem;
  Serial = {};
  const double tick = craft::kTwoPi / 4096;
  craft::AxisConfig axes[6] = {
    {1, false, tick, 0, 1, -3.0543, 3.0543, 10, 30, 0.035},       // tool roll -175..175 deg
    {2, false, tick, 0, -1, -3.0543, 3.0543, 10, 30, 0.035},      // same, mirrored motor
    {3, false, tick, -8192, 1, -2.3806, 14.0167, 10, 30, 0.035},  // LM, 2.6 turns
  };
  for (unsigned i = 3; i < 6; ++i) axes[i] = {static_cast<uint8_t>(i + 1), false, tick, 0, 1, -1, 7, 10, 30, 0.035};
  Dynamixel2Arduino bus(Serial3, 84);
  bus.registers[{1, PRESENT_POSITION}] = 3072;   // 270 deg read -> -90 deg
  bus.registers[{2, PRESENT_POSITION}] = 3072;   // mirrored: -270 deg read -> +90 deg
  bus.registers[{3, PRESENT_POSITION}] = 2048;   // 180 + 720 = 900 deg: over 803, two candidates
  craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
  board.begin();
  for (unsigned id = 1; id <= 6; ++id) {          // startup hold does not move anything
    assert((bus.registers[{id, GOAL_POSITION}] == bus.registers[{id, PRESENT_POSITION}]));
  }
  test_time_ms += 20;
  board.tick();
  assert(Serial.output.find("-1.570796,1.570796,15.707963,") != std::string::npos);
  Serial.input = "-1.0,1.0,10,0,0,0\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{1, GOAL_POSITION}] == 4096 - 652));  // -1 rad on the unwrapped frame
  assert((bus.registers[{2, GOAL_POSITION}] == 4096 - 652));  // mirrored: +1 rad, same raw
  assert((bus.registers[{3, GOAL_POSITION}] == 6519 - 8192));
}

// Raw samples are averaged as floats over one feedback period: sub-LSB offsets survive, nothing is
// zeroed, and a period without a new sample repeats the last mean.
void imuFeedback() {
  cIMU imu;
  craft::ImuFeedback feedback;
  const int16_t gz[] = {47, 48, 48, 48};  // mean 47.75 LSB: the library would keep 47 and zero the rest
  for (int16_t z : gz) {
    imu.SEN.gyroRAW[2] = z;
    imu.SEN.accADC[2] = 16384;
    feedback.add(imu);
  }
  Serial.output.clear();
  feedback.print(imu);
  char expected[96];
  snprintf(expected, sizeof expected, ",1,0.000000,0.000000,%.6f,0.0000,0.0000,1.0000", 47.75 * craft::kGyroRadSPerLsb);
  assert(Serial.output == expected);
  Serial.output.clear();
  feedback.print(imu);  // no new sample
  assert(Serial.output == expected);
  imu.bConnected = false;
  imu.SEN.gyroRAW[2] = 0;
  feedback.add(imu);
  Serial.output.clear();
  feedback.print(imu);
  assert(Serial.output == ",0,0.000000,0.000000,0.000000,0.0000,0.0000,1.0000");
}

int main() {
  protocol();
  imuFeedback();
  wheelClock();
  manipulator();
  bootUnwrap();
  std::cout << "Firmware protocol, IMU feedback, watchdog, fault, relax/hold and channel tests passed\n";
}
