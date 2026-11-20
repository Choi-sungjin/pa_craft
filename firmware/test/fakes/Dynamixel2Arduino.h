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
// Sync Read / Sync Write as Dynamixel2Arduino 0.8.2 declares them (only the fields the firmware uses).
namespace DYNAMIXEL {
struct InfoSyncBulkBuffer_t { uint8_t *p_buf; uint16_t buf_capacity; uint16_t gen_length; bool is_completed; };
struct XELInfoSyncRead_t { uint8_t *p_recv_buf; uint8_t id; uint8_t error; };
struct InfoSyncReadInst_t {
  uint16_t addr; uint16_t addr_length; XELInfoSyncRead_t *p_xels; uint8_t xel_count; bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
};
struct XELInfoSyncWrite_t { uint8_t *p_data; uint8_t id; };
struct InfoSyncWriteInst_t {
  uint16_t addr; uint16_t addr_length; XELInfoSyncWrite_t *p_xels; uint8_t xel_count; bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
};
}
struct Dynamixel2Arduino {
  std::map<std::pair<int, int>, int32_t> registers;
  int error = 0, status_error = 0, failed_item = -1;
  int writes = 0;
  int sync_reads = 0, sync_writes = 0;
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
    if (length == 1 && (addr == 12 || addr == 68 || addr == 98)) { error = 0; buf[0] = rawByte(id, addr); return 1; }
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
  // Replies in list order; like the library, stops at the first motor that does not answer.
  uint8_t syncRead(DYNAMIXEL::InfoSyncReadInst_t *p, uint32_t timeout) {
    ++sync_reads;
    uint8_t got = 0;
    for (uint8_t k = 0; k < p->xel_count; ++k) {
      auto &x = p->p_xels[k];
      if (read(x.id, p->addr, p->addr_length, x.p_recv_buf, p->addr_length, timeout) != p->addr_length) break;
      x.error = static_cast<uint8_t>(status_error);
      ++got;
    }
    return got;
  }
  // Broadcast, no reply: true when sent. GOAL_VELOCITY 104, PROFILE_ACCELERATION 108 (+ PROFILE_VELOCITY 112 in
  // an 8-byte write), GOAL_POSITION 116.
  bool syncWrite(DYNAMIXEL::InfoSyncWriteInst_t *p) {
    using namespace ControlTableItem;
    ++sync_writes;
    for (uint8_t k = 0; k < p->xel_count; ++k) {
      const auto &x = p->p_xels[k];
      for (uint16_t off = 0; off < p->addr_length; off += 4) {
        const uint16_t addr = p->addr + off;
        const uint8_t item = addr == 104 ? GOAL_VELOCITY : addr == 108 ? PROFILE_ACCELERATION :
                             addr == 112 ? PROFILE_VELOCITY : addr == 116 ? GOAL_POSITION : 255;
        if (item == 255) return false;
        uint32_t v = 0;
        for (unsigned b = 4; b-- > 0;) v = v << 8 | x.p_data[off + b];
        ++writes;
        if (item == failed_item) return false;
        registers[{x.id, item}] = static_cast<int32_t>(v);
      }
    }
    return true;
  }
  // Raw one-byte settings by address (SECONDARY_ID 12, STATUS_RETURN_LEVEL 68, BUS_WATCHDOG 98), as normalizeMotor
  // uses them; unset ones read as the factory value.
  std::map<std::pair<int, int>, uint8_t> raw_bytes;
  int raw_writes = 0;
  uint8_t rawByte(uint8_t id, uint16_t addr) {
    const auto it = raw_bytes.find({id, addr});
    return it != raw_bytes.end() ? it->second : addr == 12 ? 255 : addr == 68 ? 2 : 0;
  }
  bool write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t length, uint32_t) {
    error = 0;
    if (length != 1) return false;
    ++raw_writes;
    raw_bytes[{id, addr}] = data[0];
    return true;
  }
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t) {
    ++writes;
    error = item == failed_item ? 1 : 0;
    if (error) return false;
    registers[{id, item}] = value;
    return true;
  }
};
