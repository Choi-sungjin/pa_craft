// Standalone OpenCR / Protocol 2.0 X-series position-axis commissioning.
// Upload this sketch instead of the normal board firmware. Only one ID is used.
#include <Dynamixel2Arduino.h>
#include <stdint.h>

constexpr uint8_t kMotorId = 1;  // Select ONE physically identified motor.
constexpr uint32_t kBaud = 1000000;
constexpr int32_t kStepTicks = 8;   // 0.70 degrees at 4096 ticks/revolution
constexpr int32_t kTravelTicks = 32; // +/-2.81 degrees from startup position
constexpr uint32_t kTimeoutMs = 250;
Dynamixel2Arduino bus(Serial3, 84);
int32_t origin = 0;
bool ready = false;
bool pending = false;
uint32_t commanded_at = 0;
const char *diagnostic = "STARTING";

bool statusOk() {
  return bus.getLastLibErrCode() == 0 && bus.getLastStatusPacketError() == 0;
}

bool readPosition(int32_t &value) {
  value = bus.readControlTableItem(static_cast<uint8_t>(ControlTableItem::PRESENT_POSITION), kMotorId, 10);
  return statusOk();
}

void fault() {
  diagnostic = "FAULT: check motor and reset board; support the load.";
  ready = false;
  pending = false;
  int32_t position;
  // Hold using a fresh sample; torque-off is a last resort. Support any load.
  if (!readPosition(position) ||
      !bus.writeControlTableItem(static_cast<uint8_t>(ControlTableItem::GOAL_POSITION), kMotorId, position, 10) ||
      !statusOk()) bus.torqueOff(kMotorId);
  Serial.println("FAULT: check motor and reset board; support the load.");
}

void setup() {
  Serial.begin(115200);
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(200);
  bus.begin(kBaud);
  if (!bus.setPortProtocolVersion(2.0) || !bus.ping(kMotorId) || !statusOk()) {
    diagnostic = "NO MOTOR: check ID/baud/model/power/wiring, then reset.";
    Serial.println("No motor. Check ID/baud/model and reset.");
    return;
  }
  // This requires an X-series model supporting extended position mode and these
  // registers. No zero, direction or host joint mapping is needed.
  if (!bus.torqueOff(kMotorId) || !statusOk() ||
      !bus.setOperatingMode(kMotorId, OP_EXTENDED_POSITION) || !statusOk() ||
      !bus.writeControlTableItem(static_cast<uint8_t>(ControlTableItem::DRIVE_MODE), kMotorId, 0, 10) || !statusOk() ||
      !bus.writeControlTableItem(static_cast<uint8_t>(ControlTableItem::PROFILE_ACCELERATION), kMotorId, 1, 10) || !statusOk() ||
      !bus.writeControlTableItem(static_cast<uint8_t>(ControlTableItem::PROFILE_VELOCITY), kMotorId, 1, 10) || !statusOk() ||
      !readPosition(origin) ||
      !bus.writeControlTableItem(static_cast<uint8_t>(ControlTableItem::GOAL_POSITION), kMotorId, origin, 10) || !statusOk() ||
      !bus.torqueOn(kMotorId) || !statusOk()) {
    fault();
    return;
  }
  ready = true;
  diagnostic = "READY: + / - = 8 ticks; s = hold.";
  Serial.println("READY: + / - = 8 raw ticks; s = hold. Travel limited to +/-32 startup ticks.");
}

void loop() {
  static uint32_t last_diagnostic = 0;
  if (static_cast<uint32_t>(millis() - last_diagnostic) >= 2000) {
    last_diagnostic = millis();
    Serial.print("JOG ID="); Serial.print(kMotorId);
    Serial.print(" baud="); Serial.print(kBaud);
    Serial.print(" "); Serial.println(diagnostic);
  }
  if (!ready) return;
  static uint32_t last_poll = 0;
  if (static_cast<uint32_t>(millis() - last_poll) < 20) return;
  last_poll = millis();
  int32_t position;
  const int32_t error = bus.readControlTableItem(static_cast<uint8_t>(ControlTableItem::HARDWARE_ERROR_STATUS), kMotorId, 10);
  if (!statusOk() || error != 0) { fault(); return; }
  const int32_t torque = bus.readControlTableItem(static_cast<uint8_t>(ControlTableItem::TORQUE_ENABLE), kMotorId, 10);
  if (!statusOk() || torque != 1 || !readPosition(position)) { fault(); return; }
  if (static_cast<int64_t>(position) < static_cast<int64_t>(origin) - kTravelTicks ||
      static_cast<int64_t>(position) > static_cast<int64_t>(origin) + kTravelTicks) {
    fault(); return;
  }
  int command = -1;
  // Read one input per tick; do not automatically repeat commands.
  if (Serial.available()) command = Serial.read();
  const bool timed_out = pending && static_cast<uint32_t>(millis() - commanded_at) >= kTimeoutMs;
  if (timed_out || command == 's' || command == '+' || command == '-') {
    int64_t target = position;
    // Timeout takes priority: capture a fresh hold before accepting another jog.
    if (!timed_out && (command == '+' || command == '-')) {
      target += command == '+' ? kStepTicks : -kStepTicks;
      if (target < static_cast<int64_t>(origin) - kTravelTicks ||
          target > static_cast<int64_t>(origin) + kTravelTicks ||
          target < INT32_MIN || target > INT32_MAX) {
        Serial.println("Travel limit: command rejected.");
        return;
      }
    }
    if (!bus.writeControlTableItem(static_cast<uint8_t>(ControlTableItem::GOAL_POSITION), kMotorId,
                                  static_cast<int32_t>(target), 10) || !statusOk()) {
      fault(); return;
    }
    pending = !timed_out && (command == '+' || command == '-');
    commanded_at = millis();
    Serial.print("raw position="); Serial.print(position);
    Serial.print(" target="); Serial.println(static_cast<int32_t>(target));
  }
}
