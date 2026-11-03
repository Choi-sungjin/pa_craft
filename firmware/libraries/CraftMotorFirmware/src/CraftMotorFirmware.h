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
  int32_t zero_tick;            // motor radians = (raw - zero_tick)*units_per_tick (+ whole turns, unwrapTurns)
  int8_t direction;             // physical motor sign, +1 or -1
  double minimum;
  double maximum;
  int32_t profile_acceleration; // raw model-specific units, nonzero
  int32_t profile_velocity;     // raw model-specific units, position axes only
  double tolerance;            // radians or rad/s
};

// An extra velocity motor that copies a velocity channel's command (4WD: rear wheel follows
// its side's channel). Same model as the leader: units and profile acceleration are the leader's.
// Not a CSV channel; the channel's feedback is the mean of its leader and followers.
struct FollowerConfig {
  uint8_t id;
  uint8_t leader;  // CSV channel index
  int8_t direction;
};
constexpr unsigned kMaxFollowers = 4;

// N=3: wheel_l, wheel_r, clock. N=6: LM, shoulder, elbow, wrist, roll, gripper.
template<unsigned N>
class MotorBoard {
  static_assert(N == 3 || N == 6, "CRAFT board needs 3 or 6 channels");
public:
  MotorBoard(Dynamixel2Arduino &bus, const AxisConfig (&axes)[N],
             bool hardware_confirmed, uint32_t baud, double current_a_per_tick = 0.0)
      : bus_(bus), axes_(axes), confirmed_(hardware_confirmed), baud_(baud),
        current_scale_(current_a_per_tick) {}

  // Optional: print ",v1,v2,..." after the state fields of every feedback line.
  void setFeedbackExtension(void (*extension)()) { extension_ = extension; }
  // Optional, before begin(): Position I gain per axis (position axes; 0 keeps the motor default 0).
  void setPositionIGains(const uint16_t (&gains)[N]) { i_gains_ = gains; }
  // Optional, before begin(): follower motors (see FollowerConfig).
  template<unsigned M>
  void setFollowers(const FollowerConfig (&followers)[M]) {
    static_assert(M <= kMaxFollowers, "too many follower motors");
    followers_ = followers;
    follower_count_ = M;
  }

  void begin() {
    Serial.begin(115200);
    // USB must not gate feedback: ControlMaster waits for the first sample.
    for (unsigned i = 0; i < N; ++i) zero_[i] = axes_[i].velocity ? 0 : axes_[i].zero_tick;
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
          (!a.velocity && !write(ControlTableItem::PROFILE_VELOCITY, i, a.profile_velocity)) ||
          (!a.velocity && i_gains_ && i_gains_[i] &&
           !write(ControlTableItem::POSITION_I_GAIN, i, i_gains_[i]))) {
        tripFault(); return;
      }
    }
    for (unsigned k = 0; k < follower_count_; ++k) {
      const auto &f = followers_[k];
      if (!bus_.ping(f.id) || !bus_.torqueOff(f.id) || !statusOk() ||
          !bus_.setOperatingMode(f.id, OP_VELOCITY) || !statusOk() ||
          !writeId(ControlTableItem::DRIVE_MODE, f.id, 0) ||
          !writeId(ControlTableItem::PROFILE_ACCELERATION, f.id,
                   axes_[f.leader].profile_acceleration)) {
        tripFault(); return;
      }
    }
    if (!readFeedback()) { tripFault(); return; }
    unwrapTurns();
    holdTargets();
    if (!writeTargets()) { tripFault(); return; }
    if (!setTorque(true)) { tripFault(); return; }
  }

  void tick() {
    // Limit work per pass so a noisy USB sender cannot starve motor/watchdog work.
    double incoming[N];
    for (unsigned budget = 0; budget < 512 && Serial.available(); ++budget) {
      if (receiver_.push(static_cast<char>(Serial.read()), incoming) && !fault_ && !relaxed_) {
        bool valid = true;
        for (unsigned i = 0; i < N; ++i) {
          const auto &a = axes_[i];
          const double raw = incoming[i] * a.direction / a.units_per_tick + zero_[i];
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
      const Keyword keyword = receiver_.takeKeyword();
      if (keyword == Keyword::kRelax && !fault_) relax();
      if (keyword == Keyword::kHold && !fault_) hold();
    }
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - last_tick_ms_) < kPeriodMs) return;
    last_tick_ms_ = now;
    if (!fault_) {
      if (!readFeedback()) {
        tripFault();
      } else if (!relaxed_) {
        // Hold is captured once at timeout; no drift-following hold on later ticks.
        expired_ = commandExpired(millis(), last_command_ms_, received_, kCommandTimeoutMs);
        if (expired_ && !holding_) { holdTargets(); holding_ = true; }
        if (!writeTargets()) tripFault();
      }
    }
    publishFeedback();
  }

private:
  // Torque off for moving the axes by hand. Feedback keeps tracking, so multi-turn
  // positions stay valid; CSV commands are ignored and state reports 2 until HOLD.
  void relax() {
    if (!started_ || relaxed_) return;
    relaxed_ = true;
    received_ = false;
    holding_ = true;
    if (!setTorque(false)) tripFault();
  }

  // Torque back on, holding wherever the axes were left (wheels stopped).
  void hold() {
    if (!relaxed_) return;
    if (!readFeedback()) { tripFault(); return; }
    holdTargets();
    if (!writeTargets()) { tripFault(); return; }
    if (!setTorque(true)) { tripFault(); return; }
    relaxed_ = false;
  }

  bool statusOk() const {
    return bus_.getLastLibErrCode() == 0 && bus_.getLastStatusPacketError() == 0;
  }

  bool writeId(uint8_t item, uint8_t id, int32_t value) {
    return bus_.writeControlTableItem(item, id, value, kIoTimeoutMs) && statusOk();
  }

  bool readId(uint8_t item, uint8_t id, int32_t &value) {
    value = bus_.readControlTableItem(item, id, kIoTimeoutMs);
    return statusOk();
  }

  bool write(uint8_t item, unsigned i, int32_t value) { return writeId(item, axes_[i].id, value); }
  bool read(uint8_t item, unsigned i, int32_t &value) { return readId(item, axes_[i].id, value); }

  // Axes, then followers.
  bool setTorque(bool on) {
    for (unsigned i = 0; i < N + follower_count_; ++i) {
      const uint8_t id = i < N ? axes_[i].id : followers_[i - N].id;
      if (!(on ? bus_.torqueOn(id) : bus_.torqueOff(id)) || !statusOk()) return false;
    }
    return true;
  }

  // Healthy, torque as expected, and its present value (velocity or position).
  bool readMotor(uint8_t id, bool velocity, int32_t &raw) {
    int32_t error, torque;
    return readId(ControlTableItem::HARDWARE_ERROR_STATUS, id, error) && error == 0 &&
           readId(ControlTableItem::TORQUE_ENABLE, id, torque) &&
           !(started_ && !relaxed_ && torque != 1) &&
           readId(velocity ? ControlTableItem::PRESENT_VELOCITY : ControlTableItem::PRESENT_POSITION,
                  id, raw);
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
    for (unsigned k = 0; k < follower_count_; ++k) {
      const auto &f = followers_[k];
      if (f.id > 252 || f.leader >= N || !axes_[f.leader].velocity ||
          (f.direction != 1 && f.direction != -1)) return false;
      for (unsigned i = 0; i < N; ++i) if (axes_[i].id == f.id) return false;
      for (unsigned j = 0; j < k; ++j) if (followers_[j].id == f.id) return false;
    }
    return true;
  }

  bool readFeedback() {
    double sample[N];
    bool moving[N];
    unsigned count[N];
    for (unsigned i = 0; i < N; ++i) {
      int32_t raw;
      if (!readMotor(axes_[i].id, axes_[i].velocity, raw)) return false;
      // Subtract in double to avoid overflowing signed raw extended positions.
      sample[i] = (static_cast<double>(raw) - zero_[i]) *
                  axes_[i].units_per_tick * axes_[i].direction;
      count[i] = 1;
      int32_t is_moving = 0;
      if (!axes_[i].velocity && !read(ControlTableItem::MOVING, i, is_moving)) return false;
      moving[i] = is_moving != 0;
    }
    for (unsigned k = 0; k < follower_count_; ++k) {
      const auto &f = followers_[k];
      int32_t raw;
      if (!readMotor(f.id, true, raw)) return false;
      sample[f.leader] += raw * axes_[f.leader].units_per_tick * f.direction;
      ++count[f.leader];
    }
    for (unsigned i = 0; i < N; ++i) sample[i] /= count[i];
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

  // Motors forget whole turns at power-up and report less than one turn. A position axis read
  // outside its limits is shifted by the one whole turn that brings it inside; with none or
  // several candidates (the multi-turn LM) it is left for the host checks (checkarm).
  void unwrapTurns() {
    for (unsigned i = 0; i < N; ++i) {
      const auto &a = axes_[i];
      if (a.velocity || (feedback_[i] >= a.minimum && feedback_[i] <= a.maximum)) continue;
      int found = 0, turns = 0;
      for (int k = -2; k <= 2; ++k) {
        const double shifted = feedback_[i] + k * kTwoPi;
        if (k != 0 && shifted >= a.minimum && shifted <= a.maximum) { ++found; turns = k; }
      }
      if (found != 1) continue;
      zero_[i] -= turns * kTwoPi / a.units_per_tick * a.direction;
      feedback_[i] += turns * kTwoPi;
    }
  }

  void holdTargets() {
    for (unsigned i = 0; i < N; ++i) target_[i] = axes_[i].velocity ? 0 : feedback_[i];
  }

  bool writeTargets() {
    for (unsigned i = 0; i < N; ++i) {
      const auto &a = axes_[i];
      const double raw = target_[i] * a.direction / a.units_per_tick + zero_[i];
      if (!isfinite(raw) || raw < INT32_MIN || raw > INT32_MAX ||
          !write(a.velocity ? ControlTableItem::GOAL_VELOCITY :
                             ControlTableItem::GOAL_POSITION, i, static_cast<int32_t>(llround(raw)))) {
        return false;
      }
    }
    for (unsigned k = 0; k < follower_count_; ++k) {
      const auto &f = followers_[k];
      const double raw = target_[f.leader] * f.direction / axes_[f.leader].units_per_tick;
      if (!writeId(ControlTableItem::GOAL_VELOCITY, f.id, static_cast<int32_t>(llround(raw)))) {
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
    for (unsigned k = 0; k < follower_count_; ++k) {
      if (!writeId(ControlTableItem::GOAL_VELOCITY, followers_[k].id, 0)) {
        bus_.torqueOff(followers_[k].id);
      }
    }
  }

  int state(unsigned first, unsigned last) const {
    if (fault_ || relaxed_ || (received_ && expired_)) return 2;
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
      Serial.print(state(0, 2)); Serial.print(','); Serial.print(state(2, 3));
    } else {
      Serial.print(state(0, N)); Serial.print(','); Serial.print(current_a_, 6);
    }
    if (extension_) extension_();
    Serial.println();
  }

  Dynamixel2Arduino &bus_;
  const AxisConfig (&axes_)[N];
  const bool confirmed_;
  const uint32_t baud_;
  const double current_scale_;
  CsvReceiver<N> receiver_;
  void (*extension_)() = nullptr;
  const uint16_t *i_gains_ = nullptr;
  const FollowerConfig *followers_ = nullptr;
  unsigned follower_count_ = 0;
  double zero_[N] = {};  // runtime zero tick: zero_tick plus the boot unwrap
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
  bool relaxed_ = false;
};

}  // namespace craft
