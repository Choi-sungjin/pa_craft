#pragma once
#include "Arduino.h"
#include <map>
#include <utility>

enum { OP_VELOCITY = 1, OP_EXTENDED_POSITION = 4 };
namespace ControlTableItem {
enum : uint8_t { DRIVE_MODE, PROFILE_ACCELERATION, PROFILE_VELOCITY,
  HARDWARE_ERROR_STATUS, TORQUE_ENABLE, PRESENT_VELOCITY, PRESENT_POSITION,
  MOVING, PRESENT_CURRENT, GOAL_VELOCITY, GOAL_POSITION, POSITION_I_GAIN, PRESENT_INPUT_VOLTAGE,
  PRESENT_TEMPERATURE };
}
struct Dynamixel2Arduino {
  std::map<std::pair<int, int>, int32_t> registers;
  int error = 0, status_error = 0, failed_item = -1;
  int writes = 0;
  int lost_replies = 0;  // the next reads get no status packet (a noisy bus)
  Dynamixel2Arduino(HardwareSerial &, int) {}
  void begin(uint32_t) {}
  bool setPortProtocolVersion(double) { return true; }
  bool ping(uint8_t) { error = 0; return true; }
  bool torqueOff(uint8_t id) { registers[{id, ControlTableItem::TORQUE_ENABLE}] = 0; return true; }
  bool torqueOn(uint8_t id) { registers[{id, ControlTableItem::TORQUE_ENABLE}] = 1; return true; }
  bool setOperatingMode(uint8_t, int) { return true; }
  int getLastLibErrCode() const { return error; }
  int getLastStatusPacketError() const { return status_error; }
  int32_t readControlTableItem(uint8_t item, uint8_t id, uint32_t) {
    error = item == failed_item ? 1 : 0;
    return registers[{id, item}];
  }
  // One read of a control table range (X-series addresses), as the firmware batches them.
  int32_t read(uint8_t id, uint16_t addr, uint16_t length, uint8_t *buf, uint16_t, uint32_t) {
    using namespace ControlTableItem;
    if (lost_replies > 0) { --lost_replies; error = 1; return 0; }
    const struct { uint16_t addr; unsigned bytes; uint8_t item; } table[] = {
      {64, 1, TORQUE_ENABLE}, {70, 1, HARDWARE_ERROR_STATUS}, {122, 1, MOVING}, {126, 2, PRESENT_CURRENT},
      {128, 4, PRESENT_VELOCITY}, {132, 4, PRESENT_POSITION}, {144, 2, PRESENT_INPUT_VOLTAGE}, {146, 1, PRESENT_TEMPERATURE}};
    error = 0;
    for (uint16_t k = 0; k < length; ++k) buf[k] = 0;
    for (const auto &t : table) {
      if (t.addr < addr || t.addr + t.bytes > addr + length) continue;
      if (t.item == failed_item) error = 1;
      const uint32_t v = static_cast<uint32_t>(registers[{id, t.item}]);
      for (unsigned b = 0; b < t.bytes; ++b) buf[t.addr - addr + b] = static_cast<uint8_t>(v >> (8 * b));
    }
    return error ? 0 : length;
  }
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t) {
    ++writes;
    error = item == failed_item ? 1 : 0;
    if (error) return false;
    registers[{id, item}] = value;
    return true;
  }
};
