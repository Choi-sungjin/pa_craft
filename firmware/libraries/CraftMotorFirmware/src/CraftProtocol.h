#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

namespace craft {

// Fixed-size, newline-framed input. An oversized frame is discarded through '\n'.
template<unsigned N, unsigned Capacity = 256>
class CsvReceiver {
public:
  bool push(char byte, double (&values)[N]) {
    if (byte == '\n') {
      buffer_[used_] = '\0';
      const bool valid = !overflow_ && parse(values);
      used_ = 0;
      overflow_ = false;
      return valid;
    }
    if (byte == '\0' || used_ == Capacity - 1) overflow_ = true;
    if (!overflow_) buffer_[used_++] = byte;
    return false;
  }

private:
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
};

inline bool commandExpired(uint32_t now, uint32_t last, bool received,
                           uint32_t timeout) {
  return !received || static_cast<uint32_t>(now - last) >= timeout;
}

}  // namespace craft
