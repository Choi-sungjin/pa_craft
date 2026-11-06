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
constexpr unsigned kDiagTicks = 50;
// Time-based profile (DRIVE_MODE bit 2): PROFILE_VELOCITY is the travel time [ms] and PROFILE_ACCELERATION the
// ramp time [ms]; every time-based axis of one command gets the same time, so they start and arrive together.
constexpr uint8_t kDriveModeTimeBased = 4;
constexpr double kRampFraction = 0.25;     // ramp up and down, each this share of the travel time
constexpr uint32_t kMinMoveMs = 100;
constexpr uint32_t kMaxMoveMs = 32767;
constexpr double kVelocityUnitRpm = 0.229;  // X-series PROFILE_VELOCITY unit (the speed caps in AxisConfig)  // a diagnostic line every second (and at once when a fault latches)

// Why the board latched its fault (diagnostic line, first cause only).
enum FaultReason : uint8_t {
  kFaultNone = 0,
  kFaultHardwareError = 1,  // the motor reports HARDWARE_ERROR_STATUS (code = its bits)
  kFaultTorqueOff = 2,      // torque found off while it should be on: the motor rebooted (power dip)
  kFaultNoResponse = 3,     // a read got no valid status packet
  kFaultWriteFailed = 4,    // a goal / torque write failed
  kFaultStartup = 5,        // ping / configuration at begin() failed
  kFaultConfig = 6,         // invalid axis configuration or hardware not confirmed
};

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
  // Optional: the axes whose arrival tolerance TOL,<0.01 deg> may widen for one move (via poses; the host sends
  // TOL before every arm move, 0 = each axis's own). Without this call TOL changes nothing.
  void setLooseTolerance(const bool (&loose)[N]) { loose_ = loose; }
  // Optional, before begin(): per position axis, the whole turn at power-up is the one that puts the axis
  // nearest this angle [rad] (the motor keeps only its angle within one turn). NAN: zero_tick and the
  // limit unwrap decide, as without this call.
  void setBootCenters(const double (&centers)[N]) { boot_centers_ = centers; }
  // Optional, before begin(): position axes moved with a time-based profile (see kDriveModeTimeBased). Their
  // AxisConfig profile_velocity stays the speed cap [0.229 rpm]: a move never runs faster than that, whatever the
  // TIME asked for. Their goal is written only when it changes (a rewrite restarts the timed move).
  void setTimeBased(const bool (&axes)[N]) { time_based_ = axes; }
  // Optional, with setTimeBased: per time-based axis, the share of the move's travel time it uses (default 1; e.g.
  // 0.7: the gripper is there when the arm has 30 % of the move left).
  void setTimeScale(const double (&scale)[N]) { time_scale_ = scale; }
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
    if (!validConfiguration() || !confirmed_) { fail(kFaultConfig, 0); fault_ = true; return; }
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
          !write(ControlTableItem::DRIVE_MODE, i, timeBased(i) ? kDriveModeTimeBased : 0) ||
          !write(ControlTableItem::PROFILE_ACCELERATION, i,
                 timeBased(i) ? static_cast<int32_t>(kMinMoveMs * kRampFraction) : a.profile_acceleration) ||
          (!a.velocity && !write(ControlTableItem::PROFILE_VELOCITY, i,
                                 timeBased(i) ? static_cast<int32_t>(kMinMoveMs) : a.profile_velocity)) ||
          (!a.velocity && i_gains_ && i_gains_[i] &&
           !write(ControlTableItem::POSITION_I_GAIN, i, i_gains_[i]))) {
        fail(kFaultStartup, a.id); tripFault(); return;
      }
    }
    for (unsigned k = 0; k < follower_count_; ++k) {
      const auto &f = followers_[k];
      if (!bus_.ping(f.id) || !bus_.torqueOff(f.id) || !statusOk() ||
          !bus_.setOperatingMode(f.id, OP_VELOCITY) || !statusOk() ||
          !writeId(ControlTableItem::DRIVE_MODE, f.id, 0) ||
          !writeId(ControlTableItem::PROFILE_ACCELERATION, f.id,
                   axes_[f.leader].profile_acceleration)) {
        fail(kFaultStartup, f.id); tripFault(); return;
      }
    }
    if (!readFeedback()) { tripFault(); return; }
    centerTurns();
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
      if (keyword == Keyword::kTime) move_ms_ = static_cast<uint32_t>(receiver_.keywordValue());
      if (keyword == Keyword::kTol) loose_tolerance_ = receiver_.keywordValue() * kTwoPi / 36000.0;
      // IGAIN: the Position I gain (RAM) of one position axis, before the next targets (host: per move, e.g. a
      // smaller one against gravity); IGAIN,BOOT: every position axis back to its power-up gain (0 without
      // setPositionIGains, the motor default). A failed write latches the fault like any other write.
      if (keyword == Keyword::kIGain && !fault_ && !axes_[receiver_.keywordAxis()].velocity &&
          !write(ControlTableItem::POSITION_I_GAIN, static_cast<unsigned>(receiver_.keywordAxis()),
                 static_cast<int32_t>(receiver_.keywordValue()))) {
        fail(kFaultWriteFailed, axes_[receiver_.keywordAxis()].id);
        tripFault();
      }
      if (keyword == Keyword::kIGainBoot && !fault_) {
        for (unsigned i = 0; i < N; ++i) {
          if (axes_[i].velocity) continue;
          if (!write(ControlTableItem::POSITION_I_GAIN, i, i_gains_ ? i_gains_[i] : 0)) {
            fail(kFaultWriteFailed, axes_[i].id);
            tripFault();
            break;
          }
        }
      }
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
    if (initialized_) sampleVoltage();
    // Print only while a host has the port open (DTR). With nobody reading, the OpenCR USB stack waits up to
    // 100 ms in every print call for buffer space: a tick took seconds, the board stopped reading USB, and the
    // upload (1200-baud request, handled when data arrives) was ignored after control_master closed the port.
    if (Serial) {
      publishFeedback();
      if (++diag_ticks_ >= kDiagTicks || (fault_ && !fault_reported_)) {
        publishDiagnostic();
        diag_ticks_ = 0;
        fault_reported_ = fault_;
      }
    }
  }

private:
  // First cause of the fault, kept for the diagnostic line. Returns false for `return fail(...)`.
  bool fail(FaultReason reason, uint8_t id, int32_t code = 0) {
    if (fault_reason_ == kFaultNone) {
      fault_reason_ = reason;
      fault_id_ = id;
      fault_code_ = code;
    }
    return false;
  }

  // One motor per tick (axes, then followers): PRESENT_INPUT_VOLTAGE [V], 0 when it does not answer.
  void sampleVoltage() {
    const unsigned count = N + follower_count_;
    const unsigned k = voltage_next_ % count;
    const uint8_t id = k < N ? axes_[k].id : followers_[k - N].id;
    int32_t raw;
    const double v = readId(ControlTableItem::PRESENT_INPUT_VOLTAGE, id, raw) ? raw * 0.1 : 0.0;
    voltage_[k] = v;
    if (v > 0 && (voltage_min_ == 0 || v < voltage_min_)) voltage_min_ = v;
    voltage_next_ = (k + 1) % count;
  }

  // "D,reason,motor id,code,lowest V since the last line,V per motor (axes, then followers)". Starts with a
  // letter so the numeric feedback parsers skip it.
  void publishDiagnostic() {
    Serial.print('D'); Serial.print(',');
    Serial.print(static_cast<int>(fault_reason_)); Serial.print(',');
    Serial.print(static_cast<int>(fault_id_)); Serial.print(',');
    Serial.print(static_cast<int>(fault_code_)); Serial.print(',');
    Serial.print(voltage_min_, 1);
    for (unsigned k = 0; k < N + follower_count_; ++k) { Serial.print(','); Serial.print(voltage_[k], 1); }
    Serial.println();
    voltage_min_ = 0;
  }

  // Torque off for moving the axes by hand. Feedback keeps tracking, so multi-turn
  // positions stay valid; CSV commands are ignored and state reports 2 until HOLD.
  void relax() {
    if (!started_ || relaxed_) return;
    for (unsigned i = 0; i < N; ++i) goal_written_[i] = false;
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
      if (!(on ? bus_.torqueOn(id) : bus_.torqueOff(id)) || !statusOk()) return fail(kFaultWriteFailed, id);
    }
    return true;
  }

  // Healthy, torque as expected, and its present value (velocity or position).
  bool readMotor(uint8_t id, bool velocity, int32_t &raw) {
    int32_t error, torque;
    if (!readId(ControlTableItem::HARDWARE_ERROR_STATUS, id, error)) return fail(kFaultNoResponse, id);
    if (error != 0) return fail(kFaultHardwareError, id, error);
    if (!readId(ControlTableItem::TORQUE_ENABLE, id, torque)) return fail(kFaultNoResponse, id);
    if (started_ && !relaxed_ && torque != 1) return fail(kFaultTorqueOff, id);
    if (!readId(velocity ? ControlTableItem::PRESENT_VELOCITY : ControlTableItem::PRESENT_POSITION, id, raw)) {
      return fail(kFaultNoResponse, id);
    }
    return true;
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
      if (!axes_[i].velocity && !read(ControlTableItem::MOVING, i, is_moving)) {
        return fail(kFaultNoResponse, axes_[i].id);
      }
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
    double current[N] = {};
    if (N == 6) {  // every arm/LM motor: the host judges overload from these
      for (unsigned i = 0; i < N; ++i) {
        int32_t raw;
        if (!read(ControlTableItem::PRESENT_CURRENT, i, raw)) return fail(kFaultNoResponse, axes_[i].id);
        // PRESENT_CURRENT is a signed 16-bit register on the supported X-series.
        current[i] = static_cast<int16_t>(raw) * current_scale_;
      }
    }
    for (unsigned i = 0; i < N; ++i) {
      feedback_[i] = sample[i];
      moving_[i] = moving[i];
      current_a_[i] = current[i];
    }
    return true;
  }

  // Motors forget whole turns at power-up and report less than one turn. A position axis read
  // outside its limits is shifted by the one whole turn that brings it inside; with none or
  // several candidates (the multi-turn LM) it is left for the host checks (checkarm).
  void centerTurns() {
    if (!boot_centers_) return;
    for (unsigned i = 0; i < N; ++i) {
      const auto &a = axes_[i];
      if (a.velocity || !isfinite(boot_centers_[i])) continue;
      const int turns = static_cast<int>(lround((boot_centers_[i] - feedback_[i]) / kTwoPi));
      zero_[i] -= turns * kTwoPi / a.units_per_tick * a.direction;
      feedback_[i] += turns * kTwoPi;
    }
  }

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

  // Arrival tolerance [rad]: the axis's own, or TOL's when it is set loose and TOL is larger.
  double tolerance(unsigned i) const {
    return loose_ && loose_[i] ? fmax(axes_[i].tolerance, loose_tolerance_) : axes_[i].tolerance;
  }

  bool timeBased(unsigned i) const { return time_based_ && time_based_[i] && !axes_[i].velocity; }

  double timeScale(unsigned i) const { return time_scale_ ? time_scale_[i] : 1.0; }

  // Travel time [ms] of the move for the time-based axes whose goal changes: the asked TIME, but no axis above its
  // speed cap within its share of it (peak speed of the ramp profile = distance / (time - ramp)).
  uint32_t moveMs(const bool (&changed)[N]) const {
    double ms = move_ms_;
    for (unsigned i = 0; i < N; ++i) {
      if (!changed[i]) continue;
      const double cap = axes_[i].profile_velocity * kVelocityUnitRpm * kTwoPi / 60.0;  // [rad/s]
      ms = fmax(ms, 1000.0 * fabs(target_[i] - feedback_[i]) / (cap * (1.0 - kRampFraction)) / timeScale(i));
    }
    return static_cast<uint32_t>(fmin(fmax(ceil(ms), kMinMoveMs), kMaxMoveMs));
  }

  bool writeTargets() {
    int32_t goal[N];
    bool changed[N] = {};
    bool any = false;
    for (unsigned i = 0; i < N; ++i) {
      const auto &a = axes_[i];
      const double raw = target_[i] * a.direction / a.units_per_tick + zero_[i];
      if (!isfinite(raw) || raw < INT32_MIN || raw > INT32_MAX) return fail(kFaultWriteFailed, a.id);
      goal[i] = static_cast<int32_t>(llround(raw));
      changed[i] = timeBased(i) && !(goal_written_[i] && goal[i] == goal_[i]);
      any = any || changed[i];
    }
    const uint32_t ms = any ? moveMs(changed) : 0;
    for (unsigned i = 0; i < N; ++i) {
      const auto &a = axes_[i];
      if (timeBased(i)) {
        if (!changed[i]) continue;
        const uint32_t axis_ms = static_cast<uint32_t>(fmax(kMinMoveMs, floor(ms * timeScale(i))));
        if (!write(ControlTableItem::PROFILE_ACCELERATION, i, static_cast<int32_t>(axis_ms * kRampFraction)) ||
            !write(ControlTableItem::PROFILE_VELOCITY, i, static_cast<int32_t>(axis_ms)) ||
            !write(ControlTableItem::GOAL_POSITION, i, goal[i])) {
          return fail(kFaultWriteFailed, a.id);
        }
        goal_[i] = goal[i];
        goal_written_[i] = true;
      } else if (!write(a.velocity ? ControlTableItem::GOAL_VELOCITY : ControlTableItem::GOAL_POSITION, i, goal[i])) {
        return fail(kFaultWriteFailed, a.id);
      }
    }
    for (unsigned k = 0; k < follower_count_; ++k) {
      const auto &f = followers_[k];
      const double raw = target_[f.leader] * f.direction / axes_[f.leader].units_per_tick;
      if (!writeId(ControlTableItem::GOAL_VELOCITY, f.id, static_cast<int32_t>(llround(raw)))) {
        return fail(kFaultWriteFailed, f.id);
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
      } else if (moving_[i] || fabs(target_[i] - feedback_[i]) > tolerance(i)) {
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
      Serial.print(state(0, N));
      for (unsigned i = 0; i < N; ++i) { Serial.print(','); Serial.print(current_a_[i], 6); }
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
  const double *boot_centers_ = nullptr;
  const bool *time_based_ = nullptr;
  const double *time_scale_ = nullptr;
  uint32_t move_ms_ = 0;      // TIME of the next move [ms]; 0: the speed caps decide
  const bool *loose_ = nullptr;
  double loose_tolerance_ = 0;  // TOL [rad]; 0: every axis its own
  int32_t goal_[N] = {};      // last goal written to each time-based axis
  bool goal_written_[N] = {};
  double zero_[N] = {};  // runtime zero tick: zero_tick plus the boot unwrap
  double feedback_[N] = {};
  double target_[N] = {};
  bool moving_[N] = {};
  double current_a_[N] = {};  // [A] per axis, N == 6 only
  uint32_t last_command_ms_ = 0;
  uint32_t last_tick_ms_ = 0;
  bool received_ = false;
  bool holding_ = true;
  bool expired_ = true;
  bool initialized_ = false;
  bool started_ = false;
  bool fault_ = false;
  bool relaxed_ = false;
  FaultReason fault_reason_ = kFaultNone;
  uint8_t fault_id_ = 0;
  int32_t fault_code_ = 0;
  bool fault_reported_ = false;
  unsigned diag_ticks_ = 0;
  double voltage_[N + kMaxFollowers] = {};
  double voltage_min_ = 0;
  unsigned voltage_next_ = 0;
};

}  // namespace craft
