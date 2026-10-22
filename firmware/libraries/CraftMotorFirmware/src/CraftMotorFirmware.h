#pragma once

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <limits.h>
#include "CraftProtocol.h"

namespace craft {

constexpr double kTwoPi = 6.2831853071795864769;
constexpr uint32_t kPeriodMs = 20;
constexpr uint32_t kCommandTimeoutMs = 250;
constexpr uint32_t kIoTimeoutMs = 3;

struct AxisConfig {
  uint8_t id;
  bool velocity;                 // false: extended position, true: velocity
  double units_per_tick;        // radians/tick or (rad/s)/tick
  int32_t zero_tick;            // motor radians = (raw - zero_tick)*units_per_tick
  int8_t direction;             // physical motor sign, +1 or -1
  double minimum;
  double maximum;
  int32_t profile_acceleration; // raw model-specific units, nonzero
  int32_t profile_velocity;     // raw model-specific units, position axes only
  double tolerance;            // radians or rad/s
};

// N=3: wheel_l, wheel_r, clock. N=6: LM, shoulder, elbow, wrist, roll, gripper.
template<unsigned N>
class MotorBoard {
  static_assert(N == 3 || N == 6, "CRAFT board needs 3 or 6 channels");
public:
  MotorBoard(Dynamixel2Arduino &bus, const AxisConfig (&axes)[N],
             bool hardware_confirmed, uint32_t baud, double current_a_per_tick = 0.0)
      : bus_(bus), axes_(axes), confirmed_(hardware_confirmed), baud_(baud),
        current_scale_(current_a_per_tick) {}

  void begin() {
    Serial.begin(115200);
    // USB must not gate feedback: ControlMaster waits for the first sample.
    if (!validConfiguration() || !confirmed_) { fault_ = true; return; }
    pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
    delay(200);
    bus_.begin(baud_);
    bus_.setPortProtocolVersion(2.0);
    initialized_ = true;

    // Configure every axis with torque disabled, then preload a zero/hold goal.
    for (unsigned i = 0; i < N; ++i) {
      const auto &a = axes_[i];
      if (!bus_.ping(a.id) || !bus_.torqueOff(a.id) || !statusOk() ||
          !bus_.setOperatingMode(a.id, a.velocity ? OP_VELOCITY : OP_EXTENDED_POSITION) ||
          !statusOk() ||
          !write(ControlTableItem::DRIVE_MODE, i, 0) ||
          !write(ControlTableItem::PROFILE_ACCELERATION, i, a.profile_acceleration) ||
          (!a.velocity && !write(ControlTableItem::PROFILE_VELOCITY, i, a.profile_velocity))) {
        tripFault(); return;
      }
    }
    if (!readFeedback()) { tripFault(); return; }
    holdTargets();
    if (!writeTargets()) { tripFault(); return; }
    for (unsigned i = 0; i < N; ++i) {
      if (!bus_.torqueOn(axes_[i].id) || !statusOk()) { tripFault(); return; }
    }
  }

  void tick() {
    // Limit work per pass so a noisy USB sender cannot starve motor/watchdog work.
    double incoming[N];
    for (unsigned budget = 0; budget < 512 && Serial.available(); ++budget) {
      if (receiver_.push(static_cast<char>(Serial.read()), incoming) && !fault_) {
        bool valid = true;
        for (unsigned i = 0; i < N; ++i) {
          const auto &a = axes_[i];
          const double raw = incoming[i] * a.direction / a.units_per_tick +
                             (a.velocity ? 0 : a.zero_tick);
          valid = valid && incoming[i] >= a.minimum && incoming[i] <= a.maximum &&
                  raw >= INT32_MIN && raw <= INT32_MAX;
        }
        if (valid) {
          for (unsigned i = 0; i < N; ++i) target_[i] = incoming[i];
          last_command_ms_ = millis();
          received_ = true;
          holding_ = false;
        }
      }
    }
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - last_tick_ms_) < kPeriodMs) return;
    last_tick_ms_ = now;
    if (!fault_) {
      if (!readFeedback()) {
        tripFault();
      } else {
        // Hold is captured once at timeout; no drift-following hold on later ticks.
        expired_ = commandExpired(millis(), last_command_ms_, received_, kCommandTimeoutMs);
        if (expired_ && !holding_) { holdTargets(); holding_ = true; }
        if (!writeTargets()) tripFault();
      }
    }
    publishFeedback();
  }

private:
  bool statusOk() const {
    return bus_.getLastLibErrCode() == 0 && bus_.getLastStatusPacketError() == 0;
  }

  bool write(uint8_t item, unsigned i, int32_t value) {
    return bus_.writeControlTableItem(item, axes_[i].id, value, kIoTimeoutMs) && statusOk();
  }

  bool read(uint8_t item, unsigned i, int32_t &value) {
    value = bus_.readControlTableItem(item, axes_[i].id, kIoTimeoutMs);
    return statusOk();
  }

  bool validConfiguration() const {
    if (N == 6 && (!isfinite(current_scale_) || current_scale_ <= 0)) return false;
    for (unsigned i = 0; i < N; ++i) {
      const auto &a = axes_[i];
      if (a.id > 252 || !isfinite(a.units_per_tick) || a.units_per_tick <= 0 ||
          (a.direction != 1 && a.direction != -1) ||
          !isfinite(a.minimum) || !isfinite(a.maximum) || a.minimum >= a.maximum ||
          a.profile_acceleration <= 0 || (!a.velocity && a.profile_velocity <= 0) ||
          !isfinite(a.tolerance) || a.tolerance <= 0 ||
          a.velocity != (N == 3 && i < 2)) return false;
      for (unsigned j = 0; j < i; ++j) if (axes_[j].id == a.id) return false;
    }
    return true;
  }

  bool readFeedback() {
    double sample[N];
    bool moving[N];
    for (unsigned i = 0; i < N; ++i) {
      int32_t raw, error, torque;
      if (!read(ControlTableItem::HARDWARE_ERROR_STATUS, i, error) || error != 0 ||
          !read(ControlTableItem::TORQUE_ENABLE, i, torque) ||
          (started_ && torque != 1) ||
          !read(axes_[i].velocity ? ControlTableItem::PRESENT_VELOCITY :
                                 ControlTableItem::PRESENT_POSITION, i, raw)) return false;
      // Subtract in double to avoid overflowing signed raw extended positions.
      sample[i] = (static_cast<double>(raw) - (axes_[i].velocity ? 0 : axes_[i].zero_tick)) *
                  axes_[i].units_per_tick * axes_[i].direction;
      int32_t is_moving = 0;
      if (!axes_[i].velocity && !read(ControlTableItem::MOVING, i, is_moving)) return false;
      moving[i] = is_moving != 0;
    }
    if (N == 6) {
      int32_t current;
      if (!read(ControlTableItem::PRESENT_CURRENT, N - 1, current)) return false;
      // PRESENT_CURRENT is a signed 16-bit register on the supported X-series.
      current_a_ = static_cast<int16_t>(current) * current_scale_;
    }
    for (unsigned i = 0; i < N; ++i) {
      feedback_[i] = sample[i];
      moving_[i] = moving[i];
    }
    return true;
  }

  void holdTargets() {
    for (unsigned i = 0; i < N; ++i) target_[i] = axes_[i].velocity ? 0 : feedback_[i];
  }

  bool writeTargets() {
    for (unsigned i = 0; i < N; ++i) {
      const auto &a = axes_[i];
      const double raw = target_[i] * a.direction / a.units_per_tick +
                         (a.velocity ? 0 : a.zero_tick);
      if (!isfinite(raw) || raw < INT32_MIN || raw > INT32_MAX ||
          !write(a.velocity ? ControlTableItem::GOAL_VELOCITY :
                             ControlTableItem::GOAL_POSITION, i, static_cast<int32_t>(llround(raw)))) {
        return false;
      }
    }
    started_ = true;
    return true;
  }

  void tripFault() {
    fault_ = true; // A bus, motor or startup fault requires a board reset.
    if (!initialized_) return;
    // Best effort: stop velocity axes; hold fresh positions, disabling torque only
    // on axes whose position/hold write cannot be obtained. Never reuse a stale goal.
    for (unsigned i = 0; i < N; ++i) {
      int32_t position;
      const bool stopped = axes_[i].velocity ? write(ControlTableItem::GOAL_VELOCITY, i, 0) :
          (read(ControlTableItem::PRESENT_POSITION, i, position) &&
           write(ControlTableItem::GOAL_POSITION, i, position));
      if (!stopped) bus_.torqueOff(axes_[i].id);
    }
  }

  int state(unsigned first, unsigned last) const {
    if (fault_ || (received_ && expired_)) return 2;
    for (unsigned i = first; i < last; ++i) {
      if (axes_[i].velocity) {
        if (fabs(feedback_[i]) > axes_[i].tolerance ||
            fabs(target_[i]) > axes_[i].tolerance) return 1;
      } else if (moving_[i] || fabs(target_[i] - feedback_[i]) > axes_[i].tolerance) {
        return 1;
      }
    }
    return 0;
  }

  void publishFeedback() {
    // On fault, positions are the last complete sample (zero before startup).
    // State=2 makes the host suppress completion reporting.
    for (unsigned i = 0; i < N; ++i) {
      Serial.print(feedback_[i], 6);
      Serial.print(',');
    }
    if (N == 3) {
      Serial.print(state(0, 2)); Serial.print(','); Serial.println(state(2, 3));
    } else {
      Serial.print(state(0, N)); Serial.print(','); Serial.println(current_a_, 6);
    }
  }

  Dynamixel2Arduino &bus_;
  const AxisConfig (&axes_)[N];
  const bool confirmed_;
  const uint32_t baud_;
  const double current_scale_;
  CsvReceiver<N> receiver_;
  double feedback_[N] = {};
  double target_[N] = {};
  bool moving_[N] = {};
  double current_a_ = 0;
  uint32_t last_command_ms_ = 0;
  uint32_t last_tick_ms_ = 0;
  bool received_ = false;
  bool holding_ = true;
  bool expired_ = true;
  bool initialized_ = false;
  bool started_ = false;
  bool fault_ = false;
};

}  // namespace craft
