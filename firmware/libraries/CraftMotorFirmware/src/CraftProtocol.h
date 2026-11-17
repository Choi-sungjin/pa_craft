#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

namespace craft {

// Text lines besides the CSV command: RELAX turns torque off, HOLD turns it back on in place, TIME,<ms> sets the
// travel time of the next move of the time-based axes (0..32767 ms, 0 = as fast as their speed caps allow),
// TOL,<0.01 deg> the arrival tolerance of the axes set loose (MotorBoard::setLooseTolerance; 0..500, 0 = their own),
// IGAIN,<axis 0..N-1>,<0..16383> the Position I gain of one position axis from now on, IGAIN,BOOT every position
// axis back to its power-up gain (MotorBoard::setPositionIGains).
enum class Keyword { kNone, kRelax, kHold, kTime, kTol, kIGain, kIGainBoot };

// Fixed-size, newline-framed input. An oversized frame is discarded through '\n'.
template<unsigned N, unsigned Capacity = 256>
class CsvReceiver {
public:
  bool push(char byte, double (&values)[N]) {
    if (byte == '\n') {
      buffer_[used_] = '\0';
      keyword_ = overflow_ ? Keyword::kNone : keyword();
      const bool valid = !overflow_ && keyword_ == Keyword::kNone && parse(values);
      used_ = 0;
      overflow_ = false;
      return valid;
    }
    if (byte == '\0' || used_ == Capacity - 1) overflow_ = true;
    if (!overflow_) buffer_[used_++] = byte;
    return false;
  }

  // TIME's value [ms] / TOL's [0.01 deg], valid after takeKeyword() returned kTime / kTol.
  long keywordValue() const { return keyword_value_; }
  // IGAIN's axis (keywordValue() is the gain), valid after takeKeyword() returned kIGain.
  long keywordAxis() const { return keyword_axis_; }

  // The keyword of the last complete line, cleared once read.
  Keyword takeKeyword() {
    const Keyword k = keyword_;
    keyword_ = Keyword::kNone;
    return k;
  }

private:
  Keyword keyword() {
    unsigned length = used_;
    while (length > 0 && isspace(static_cast<unsigned char>(buffer_[length - 1]))) --length;
    if (length == 5 && strncmp(buffer_, "RELAX", 5) == 0) return Keyword::kRelax;
    if (length == 4 && strncmp(buffer_, "HOLD", 4) == 0) return Keyword::kHold;
    if (number("TIME,", length, 32767)) return Keyword::kTime;
    if (number("TOL,", length, 500)) return Keyword::kTol;
    if (length == 10 && strncmp(buffer_, "IGAIN,BOOT", 10) == 0) return Keyword::kIGainBoot;
    if (length > 6 && strncmp(buffer_, "IGAIN,", 6) == 0) {
      const char *comma = static_cast<const char *>(memchr(buffer_ + 6, ',', length - 6));
      if (comma) {
        char *end = nullptr;
        errno = 0;
        const long axis = strtol(buffer_ + 6, &end, 10);
        const bool axis_ok = end != buffer_ + 6 && end == comma && errno == 0 && axis >= 0 && axis < static_cast<long>(N);
        if (axis_ok && number(comma + 1 - buffer_, length, 16383)) {
          keyword_axis_ = axis;
          return Keyword::kIGain;
        }
      }
    }
    return Keyword::kNone;
  }

  // "<prefix><integer 0..max>" exactly: the integer goes to keyword_value_.
  bool number(const char *prefix, unsigned length, long max) {
    const unsigned n = strlen(prefix);
    if (length <= n || strncmp(buffer_, prefix, n) != 0) return false;
    return number(n, length, max);
  }
  // The integer 0..max from buffer_[n] to the end of the line.
  bool number(long n, unsigned length, long max) {
    if (static_cast<long>(length) <= n) return false;
    char *end = nullptr;
    errno = 0;
    const long v = strtol(buffer_ + n, &end, 10);
    if (end == buffer_ + n || end != buffer_ + length || errno != 0 || v < 0 || v > max) return false;
    keyword_value_ = v;
    return true;
  }

  bool parse(double (&values)[N]) {
    double candidate[N];
    char *cursor = buffer_;
    for (unsigned i = 0; i < N; ++i) {
      char *end = nullptr;
      errno = 0;
      candidate[i] = strtod(cursor, &end);
      if (end == cursor || errno == ERANGE || !isfinite(candidate[i])) return false;
      while (isspace(static_cast<unsigned char>(*end))) ++end;
      if (i + 1 < N) {
        if (*end != ',') return false;
        cursor = end + 1;
      } else if (*end != '\0') {
        return false;
      }
    }
    for (unsigned i = 0; i < N; ++i) values[i] = candidate[i];
    return true;
  }

  char buffer_[Capacity] = {};
  unsigned used_ = 0;
  bool overflow_ = false;
  Keyword keyword_ = Keyword::kNone;
  long keyword_value_ = 0;
  long keyword_axis_ = 0;
};

inline bool commandExpired(uint32_t now, uint32_t last, bool received,
                           uint32_t timeout) {
  return !received || static_cast<uint32_t>(now - last) >= timeout;
}

}  // namespace craft
