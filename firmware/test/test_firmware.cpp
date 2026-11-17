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
  assert(!frame(receiver, "TIME,1500\r\n", values));
  assert(receiver.takeKeyword() == craft::Keyword::kTime && receiver.keywordValue() == 1500);
  for (const auto &bad : {"TIME\n", "TIME,\n", "TIME,-1\n", "TIME,32768\n", "TIME,1.5\n", "TIME,9,9\n"}) {
    assert(!frame(receiver, bad, values));
    assert(receiver.takeKeyword() == craft::Keyword::kNone);
  }
  assert(!frame(receiver, "TOL,300\n", values));
  assert(receiver.takeKeyword() == craft::Keyword::kTol && receiver.keywordValue() == 300);
  assert(!frame(receiver, "TOL,0\n", values));
  assert(receiver.takeKeyword() == craft::Keyword::kTol && receiver.keywordValue() == 0);
  assert(!frame(receiver, "IGAIN,1,100\n", values));
  assert(receiver.takeKeyword() == craft::Keyword::kIGain && receiver.keywordAxis() == 1 && receiver.keywordValue() == 100);
  assert(!frame(receiver, "IGAIN,BOOT\n", values));
  assert(receiver.takeKeyword() == craft::Keyword::kIGainBoot);
  assert(!frame(receiver, "IGAIN,2,16383\r\n", values));
  assert(receiver.takeKeyword() == craft::Keyword::kIGain && receiver.keywordAxis() == 2 && receiver.keywordValue() == 16383);
  for (const auto &bad : {"IGAIN\n", "IGAIN,\n", "IGAIN,1\n", "IGAIN,1,\n", "IGAIN,3,5\n", "IGAIN,-1,5\n",
                          "IGAIN,1,16384\n", "IGAIN,1,-5\n", "IGAIN,1,5,6\n", "IGAIN,a,5\n", "IGAIN,,5\n",
                          "IGAIN,BOOTX\n", "IGAIN,boot\n"}) {
    assert(!frame(receiver, bad, values));
    assert(receiver.takeKeyword() == craft::Keyword::kNone);
  }
  for (const auto &bad : {"TOL\n", "TOL,\n", "TOL,-1\n", "TOL,501\n", "TOL,3.0\n", "TOL,1,2\n", "TOLL,3\n"}) {
    assert(!frame(receiver, bad, values));
    assert(receiver.takeKeyword() == craft::Keyword::kNone);
  }
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
  bus.registers[{2, PRESENT_CURRENT}] = 372;   // shoulder +1.000680 A
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
  assert(Serial.output.find(",1,0.000000,1.000680,0.000000,0.000000,0.000000,-0.026900\n") != std::string::npos);

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
  assert(Serial.output.find(",2,0.000000,1.000680,0.000000,0.000000,0.000000,-0.026900\n") != std::string::npos);
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
  assert(Serial.output.find(",2,0.000000,1.000680,0.000000,0.000000,0.000000,-0.026900\n") != std::string::npos);
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

// setBootCenters: the power-up turn of the LM is the one nearest the centre (nominal pose 1030.9 deg),
// within half a turn on either side; the limit unwrap then leaves it alone.
void bootCenter() {
  using namespace ControlTableItem;
  const double tick = craft::kTwoPi / 4096;
  const double centers[6] = {17.9925, NAN, NAN, NAN, NAN, NAN};
  const struct { int32_t present; int32_t turns; } cases[] = {
    {3537, 2},  // 310.9 deg read: 1750.9 on zero_tick -4 turns -> 1030.9 (2 turns back)
    {1603, 2},  // 140.9 deg read: 860.9, 170 deg below the centre
    {1365, 1},  // 120.0 deg read: 840.0 is 190.9 below, so the next turn up: 1200.0
  };
  for (const auto &c : cases) {
    Serial = {};
    craft::AxisConfig axes[6] = {{1, false, tick, -16384, 1, 10.2032, 26.2898, 30, 180, 0.0087}};
    for (unsigned i = 1; i < 6; ++i) axes[i] = {static_cast<uint8_t>(i + 1), false, tick, 0, 1, -1, 7, 10, 30, 0.035};
    Dynamixel2Arduino bus(Serial3, 84);
    bus.registers[{1, PRESENT_POSITION}] = c.present;
    craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
    board.setBootCenters(centers);
    board.begin();
    assert((bus.registers[{1, GOAL_POSITION}] == c.present));  // holds where it is
    test_time_ms += 20;
    board.tick();
    char expect[32];
    snprintf(expect, sizeof(expect), "%.6f,", (c.present + 16384 - c.turns * 4096) * tick);
    assert(Serial.output.find(expect) == 0);
  }
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

// Diagnostic line: once a second the lowest and per-motor input voltage; when the fault latches, at once with its
// first cause (motor id, hardware error bits / rebooted motor / no answer).
void diagnostics() {
  using namespace ControlTableItem;
  struct Case { int item; int32_t value; int reason; int id; int code; };
  const Case cases[] = {
    {TORQUE_ENABLE, 0, craft::kFaultTorqueOff, 3, 0},            // motor 3 rebooted: torque came back off
    {HARDWARE_ERROR_STATUS, 32, craft::kFaultHardwareError, 3, 32},  // overload bit
  };
  for (const auto &c : cases) {
    Serial = {};
    craft::AxisConfig axes[6];
    for (unsigned i = 0; i < 6; ++i) axes[i] = {static_cast<uint8_t>(i + 1), false, 0.001, 0, 1, -10, 10, 10, 30, 0.035};
    Dynamixel2Arduino bus(Serial3, 84);
    for (unsigned id = 1; id <= 6; ++id) bus.registers[{id, PRESENT_INPUT_VOLTAGE}] = id == 4 ? 115 : 121;
    bus.registers[{2, PRESENT_TEMPERATURE}] = 61;
    bus.registers[{5, PRESENT_CURRENT}] = -1000;  // 2.69 A the other way: the peak is |current|
    craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
    board.begin();
    for (unsigned k = 0; k < craft::kDiagTicks; ++k) {
      Serial.input = "0,0,0,0,0,0\n";
      test_time_ms += 20;
      board.tick();
    }
    assert(Serial.output.find("\nD,0,0,0,11.5,12.1,12.1,12.1,11.5,12.1,12.1,-1,0,61,0,0,0,0,"
                              "-1,0.00,0.00,0.00,0.00,2.69,0.00\n") != std::string::npos);
    Serial = {};
    bus.registers[{3, static_cast<uint8_t>(c.item)}] = c.value;
    test_time_ms += 20;
    board.tick();
    char expect[64];
    snprintf(expect, sizeof(expect), "\nD,%d,%d,%d,", c.reason, c.id, c.code);
    assert(Serial.output.find(expect) != std::string::npos);  // reported in the tick that latched it
    assert(Serial.output.find(",2,") != std::string::npos);
    Serial = {};
    test_time_ms += 20;
    board.tick();
    assert(Serial.output.find("D,") == std::string::npos);  // then back to once a second
  }
}

// Time-based axes: DRIVE_MODE bit 2; every changed goal gets the same travel time (TIME, raised to the speed cap of
// the farthest axis); an unchanged goal is not rewritten (that would restart the timed move).
void timeBased() {
  using namespace ControlTableItem;
  Serial = {};
  craft::AxisConfig axes[6];
  for (unsigned i = 0; i < 6; ++i) axes[i] = {static_cast<uint8_t>(i + 1), false, 0.001, 0, 1, -10, 10, 10, 30, 0.035};
  const bool timed[6] = {true, true, true, true, true, false};
  Dynamixel2Arduino bus(Serial3, 84);
  craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
  board.setTimeBased(timed);
  board.begin();
  assert((bus.registers[{1, DRIVE_MODE}] == 4 && bus.registers[{6, DRIVE_MODE}] == 0));
  assert((bus.registers[{1, PROFILE_VELOCITY}] == 100 && bus.registers[{6, PROFILE_VELOCITY}] == 30));
  // Cap 30 x 0.229 rpm = 0.7194 rad/s: 1 rad needs 1000 / (0.7194 * 0.75) = 1853.4 ms -> 1854.
  Serial.input = "0,1,0.5,0,0,0\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{2, PROFILE_VELOCITY}] == 1854 && bus.registers[{3, PROFILE_VELOCITY}] == 1854));
  assert((bus.registers[{2, PROFILE_ACCELERATION}] == 463));
  assert((bus.registers[{2, GOAL_POSITION}] == 1000 && bus.registers[{3, GOAL_POSITION}] == 500));
  assert((bus.registers[{1, PROFILE_VELOCITY}] == 100));  // unchanged goal: nothing rewritten
  // The same goal again: no write to the timed axes (a rewrite would restart the move).
  bus.registers[{2, PROFILE_VELOCITY}] = -1;
  Serial.input = "0,1,0.5,0,0,0\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{2, PROFILE_VELOCITY}] == -1));
  // TIME longer than the cap needs: taken as is, for every changed axis.
  Serial.input = "TIME,3000\n0,1.2,0.5,0.1,0,0\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{2, PROFILE_VELOCITY}] == 3000 && bus.registers[{4, PROFILE_VELOCITY}] == 3000));
  assert((bus.registers[{2, PROFILE_ACCELERATION}] == 750));
  assert((bus.registers[{3, PROFILE_VELOCITY}] == 1854));  // elbow goal unchanged: keeps its last move
  assert((bus.registers[{6, PROFILE_VELOCITY}] == 30));    // velocity-based gripper untouched
  // A timed gripper at 0.7 of the move: same start, there at 70 % of the time.
  Serial = {};
  const bool all_timed[6] = {true, true, true, true, true, true};
  const double scale[6] = {1, 1, 1, 1, 1, 0.7};
  Dynamixel2Arduino bus2(Serial3, 84);
  craft::MotorBoard<6> board2(bus2, axes, true, 1000000, 0.00269);
  board2.setTimeBased(all_timed);
  board2.setTimeScale(scale);
  board2.begin();
  Serial.input = "TIME,2000\n0,0.5,0,0,0,0.5\n";
  test_time_ms += 20;
  board2.tick();
  assert((bus2.registers[{2, PROFILE_VELOCITY}] == 2000 && bus2.registers[{6, PROFILE_VELOCITY}] == 1400));
  // Only the gripper moving (a GRIPPER task, TIME 0): its own cap decides. The fake reads 0 (goals do not move it):
  // 1 rad / (0.7194 * 0.75) = 1853.4 ms for the gripper, the move 1853.4 / 0.7.
  Serial.input = "TIME,0\n0,0.5,0,0,0,1.0\n";
  test_time_ms += 20;
  board2.tick();
  assert((bus2.registers[{6, PROFILE_VELOCITY}] == 1853));
}

// Nobody has the port open: no feedback printed (the OpenCR stack would block in every print), commands still work.
void hostClosed() {
  using namespace ControlTableItem;
  Serial = {};
  craft::AxisConfig axes[6];
  for (unsigned i = 0; i < 6; ++i) axes[i] = {static_cast<uint8_t>(i + 1), false, 0.001, 0, 1, -10, 10, 10, 30, 0.035};
  Dynamixel2Arduino bus(Serial3, 84);
  craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
  board.begin();
  Serial.connected = false;
  for (unsigned k = 0; k < 2 * craft::kDiagTicks; ++k) {
    Serial.input = "0,1,0,0,0,0\n";
    test_time_ms += 20;
    board.tick();
  }
  assert(Serial.output.empty());
  assert((bus.registers[{2, GOAL_POSITION}] == 1000));
  Serial.connected = true;
  test_time_ms += 20;
  board.tick();
  assert(!Serial.output.empty());
}

// TOL,<0.01 deg> widens the arrival tolerance of the loose axes only (via poses); TOL,0 restores their own.
static int lastState() {  // state field of the last feedback line (diagnostic "D,..." lines skipped)
  const std::string &out = Serial.output;
  size_t end = out.rfind('\n'), begin = 0;
  std::string line;
  while (end != std::string::npos) {
    const size_t prev = end == 0 ? std::string::npos : out.rfind('\n', end - 1);
    begin = prev == std::string::npos ? 0 : prev + 1;
    line = out.substr(begin, end - begin);
    if (line.compare(0, 2, "D,") != 0) break;
    end = prev;
  }
  for (int k = 0; k < 6; ++k) line = line.substr(line.find(',') + 1);
  return std::stoi(line.substr(0, line.find(',')));
}
void looseTolerance() {
  using namespace ControlTableItem;
  Serial = {};
  Serial.connected = true;
  craft::AxisConfig axes[6];
  for (unsigned i = 0; i < 6; ++i) {
    axes[i] = {static_cast<uint8_t>(i + 1), false, 0.001, 0, 1, -10, 10, 10, 30, 0.0087266};  // 0.5 deg
  }
  const bool loose[6] = {true, true, true, true, true, false};
  Dynamixel2Arduino bus(Serial3, 84);
  craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
  board.setLooseTolerance(loose);
  board.begin();
  auto step = [&](const char *in) { Serial.input = in; test_time_ms += 20; board.tick(); return lastState(); };
  // Shoulder 2.3 deg short of its goal (the motor reads 960 ticks, goal 1000): moving at 0.5 deg.
  bus.registers[{2, PRESENT_POSITION}] = 960;
  assert(step("0,1,0,0,0,0\n") == 1);
  assert(step("TOL,300\n0,1,0,0,0,0\n") == 0);   // 3 deg: arrived
  assert(step("TOL,200\n0,1,0,0,0,0\n") == 1);   // 2 deg: not yet
  assert(step("TOL,0\n0,1,0,0,0,0\n") == 1);     // own 0.5 deg again
  bus.registers[{2, PRESENT_POSITION}] = 1000;
  // The gripper is not loose: 2.3 deg short stays moving under TOL,300.
  bus.registers[{6, PRESENT_POSITION}] = 960;
  assert(step("TOL,300\n0,1,0,0,0,1\n") == 1);
  bus.registers[{6, PRESENT_POSITION}] = 1000;
  assert(step("0,1,0,0,0,1\n") == 0);
}

// IGAIN,<axis>,<gain> rewrites one position axis's Position I gain right away (before the next targets).
void dynamicIGain() {
  using namespace ControlTableItem;
  Serial = {};
  Serial.connected = true;
  craft::AxisConfig axes[6];
  for (unsigned i = 0; i < 6; ++i) axes[i] = {static_cast<uint8_t>(i + 1), false, 0.001, 0, 1, -10, 10, 10, 30, 0.0087266};
  Dynamixel2Arduino bus(Serial3, 84);
  craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
  const uint16_t i_gains[6] = {0, 130, 100, 200, 100, 0};
  board.setPositionIGains(i_gains);
  board.begin();
  assert((bus.registers[{2, POSITION_I_GAIN}] == 130));
  Serial.input = "IGAIN,1,100\n0,1,0,0,0,0\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{2, POSITION_I_GAIN}] == 100 && bus.registers[{3, POSITION_I_GAIN}] == 100));
  assert((bus.registers[{2, GOAL_POSITION}] == 1000));
  // IGAIN,BOOT: every position axis back to its power-up gain.
  Serial.input = "IGAIN,1,60\nIGAIN,2,40\nIGAIN,0,7\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{1, POSITION_I_GAIN}] == 7 && bus.registers[{2, POSITION_I_GAIN}] == 60));
  Serial.input = "IGAIN,BOOT\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{1, POSITION_I_GAIN}] == 0 && bus.registers[{2, POSITION_I_GAIN}] == 130 &&
          bus.registers[{3, POSITION_I_GAIN}] == 100 && bus.registers[{4, POSITION_I_GAIN}] == 200));
  // A failed IGAIN write latches the fault: the targets after it are not taken, state 2, D line reason 4.
  bus.failed_item = POSITION_I_GAIN;
  Serial = {};
  Serial.connected = true;
  Serial.input = "IGAIN,1,100\n0,2,0,0,0,0\n";
  test_time_ms += 20;
  board.tick();
  assert((bus.registers[{2, GOAL_POSITION}] != 2000));  // the fault holds where it is; the new target is not taken
  assert(lastState() == 2);
  test_time_ms += 1000;
  board.tick();
  assert(Serial.output.find("D,4,2,") != std::string::npos);
}

// A lost status packet skips the tick; only kMaxLostReplies ticks in a row latch "no answer" (D,3).
void lostReplies() {
  Serial = {};
  Serial.connected = true;
  craft::AxisConfig axes[6];
  for (unsigned i = 0; i < 6; ++i) axes[i] = {static_cast<uint8_t>(i + 1), false, 0.001, 0, 1, -10, 10, 10, 30, 0.035};
  Dynamixel2Arduino bus(Serial3, 84);
  craft::MotorBoard<6> board(bus, axes, true, 1000000, 0.00269);
  board.begin();
  const auto lose = [&](unsigned ticks) {
    for (unsigned k = 0; k < ticks; ++k) {
      bus.lost_replies = 1;
      Serial.input = "0,0,0,0,0,0\n";
      test_time_ms += 20;
      board.tick();
    }
  };
  const auto good = [&] {
    Serial.input = "0,0,0,0,0,0\n";
    test_time_ms += 20;
    board.tick();
  };
  good();
  lose(2);  // one or two lost replies in a row: ignored
  assert(Serial.output.find("D,3,") == std::string::npos && lastState() != 2);
  good();  // one good tick starts the count again
  lose(2);  // one or two lost replies in a row: ignored
  assert(Serial.output.find("D,3,") == std::string::npos && lastState() != 2);
  lose(3);
  assert(Serial.output.find("D,3,1,") != std::string::npos && lastState() == 2);
}

int main() {
  protocol();
  imuFeedback();
  wheelClock();
  manipulator();
  bootUnwrap();
  bootCenter();
  diagnostics();
  timeBased();
  hostClosed();
  looseTolerance();
  dynamicIGain();
  lostReplies();
  std::cout << "Firmware protocol, IMU feedback, watchdog, fault, relax/hold, channel, diagnostic, time-based profile, closed-port, loose-tolerance dynamic I gain and lost-reply tests passed\n";
}
