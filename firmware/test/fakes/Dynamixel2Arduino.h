#pragma once
#include "Arduino.h"
#include <map>
#include <utility>

enum { OP_VELOCITY = 1, OP_EXTENDED_POSITION = 4 };
namespace ControlTableItem {
enum : uint8_t { DRIVE_MODE, PROFILE_ACCELERATION, PROFILE_VELOCITY,
  HARDWARE_ERROR_STATUS, TORQUE_ENABLE, PRESENT_VELOCITY, PRESENT_POSITION,
  MOVING, PRESENT_CURRENT, GOAL_VELOCITY, GOAL_POSITION, POSITION_I_GAIN };
}
struct Dynamixel2Arduino {
  std::map<std::pair<int, int>, int32_t> registers;
  int error = 0, status_error = 0, failed_item = -1;
  int writes = 0;
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
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t) {
    ++writes;
    error = item == failed_item ? 1 : 0;
    if (error) return false;
    registers[{id, item}] = value;
    return true;
  }
};
