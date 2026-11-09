#pragma once
// Stop-on-contact for a timed arm move (2026-11-07, dice put-down): tells when the held puck has met the surface
// below before the move arrived, so control can end the move where the arm is instead of pressing on.
//
// Two rules, each from the move's own data (bags sis_1107_1246 / 1343 / 1451):
//  - current: going down, the shoulder carries the arm (|I| ~0.3 A far, ~0.18 A middle); once the surface takes
//    the weight it falls to ~0 within ~0.1 s. Baseline = mean |I| over the middle of the move; contact when |I|
//    stays under ratio x baseline for `samples` feedback lines. Off when the baseline is too small to tell (near
//    row ~0.04 A: noise made false hits 27-37 mm early).
//  - stall: late in the move, not arrived yet and no joint moved more than stall_rad over stall_window_s (the
//    surface holds the arm; the near row, or a hit the current rule missed).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <utility>

namespace craft_control {

template <std::size_t N>
class ContactDetector {
public:
  struct Config {
    double baseline_from = 0.30;  // fraction of the move time: baseline window start
    double baseline_to = 0.65;    // ... end; the current rule starts after it
    double min_baseline_a = 0.10;  // [A] smaller baseline: current rule off
    int samples = 2;              // consecutive low lines for the current rule
    double stall_after = 0.75;    // fraction of the move time before the stall rule may fire
    double stall_window_s = 0.25;
    double stall_rad = 0.0026;    // 0.15 deg
  };
  enum Cause {None, Current, Stall};

  ContactDetector() = default;
  explicit ContactDetector(const Config & config) : config_(config) {}

  // ratio <= 0 or move_s <= 0: off. ratio: contact when |I| < ratio x baseline.
  void start(double ratio, double move_s) {
    *this = ContactDetector(config_);
    active_ = std::isfinite(ratio) && ratio > 0.0 && ratio < 1.0 && std::isfinite(move_s) && move_s > 0.0;
    ratio_ = ratio; move_s_ = move_s;
  }
  void stop() {active_ = false;}
  bool active() const {return active_;}
  Cause cause() const {return cause_;}
  double baseline_a() const {return base_n_ ? base_sum_ / base_n_ : 0.0;}

  // t: seconds since the move started; current_a: the load-carrying motor [A] (sign ignored); joints: motor angles
  // [rad]; arrived: inside the arrival tolerance (then no contact: the move ends normally). True once, at contact.
  bool update(double t, double current_a, const std::array<double, N> & joints, bool arrived) {
    if (!active_ || !std::isfinite(t) || !std::isfinite(current_a)) return false;
    if (arrived) {low_ = 0; history_.clear(); return false;}
    const double a = std::abs(current_a);
    if (t >= config_.baseline_from * move_s_ && t <= config_.baseline_to * move_s_) {
      base_sum_ += a; ++base_n_;
    } else if (t > config_.baseline_to * move_s_ && base_n_ >= 3 && baseline_a() >= config_.min_baseline_a) {
      low_ = a < ratio_ * baseline_a() ? low_ + 1 : 0;
      if (low_ >= config_.samples) return hit(Current);
    }
    if (t >= config_.stall_after * move_s_) {
      history_.emplace_back(t, joints);
      while (history_.size() > 1 && t - history_[1].first >= config_.stall_window_s) history_.pop_front();
      if (t - history_.front().first >= config_.stall_window_s) {
        double spread = 0.0;
        for (std::size_t j = 0; j < N; ++j) {
          double lo = history_.front().second[j], hi = lo;
          for (const auto & h : history_) {lo = std::min(lo, h.second[j]); hi = std::max(hi, h.second[j]);}
          spread = std::max(spread, hi - lo);
        }
        if (spread < config_.stall_rad) return hit(Stall);
      }
    }
    return false;
  }

private:
  bool hit(Cause c) {cause_ = c; active_ = false; return true;}

  Config config_{};
  bool active_ = false;
  double ratio_ = 0.0, move_s_ = 0.0;
  double base_sum_ = 0.0;
  int base_n_ = 0, low_ = 0;
  Cause cause_ = None;
  std::deque<std::pair<double, std::array<double, N>>> history_;
};

}  // namespace craft_control
