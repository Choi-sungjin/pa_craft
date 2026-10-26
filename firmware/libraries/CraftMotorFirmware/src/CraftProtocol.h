#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

namespace craft {

// Text lines besides the CSV command: RELAX turns torque off, HOLD turns it back on in place.
enum class Keyword { kNone, kRelax, kHold };

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
    return Keyword::kNone;
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
};

inline bool commandExpired(uint32_t now, uint32_t last, bool received,
                           uint32_t timeout) {
  return !received || static_cast<uint32_t>(now - last) >= timeout;
}

}  // namespace craft
