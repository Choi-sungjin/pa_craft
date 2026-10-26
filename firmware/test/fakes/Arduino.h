#pragma once
#include <stdint.h>
#include <string>
#include <sstream>
#include <iomanip>

constexpr int BDPIN_DXL_PWR_EN = 32;
constexpr int OUTPUT = 1;
constexpr int HIGH = 1;
extern uint32_t test_time_ms;
inline uint32_t millis() { return test_time_ms; }
inline void delay(uint32_t ms) { test_time_ms += ms; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
struct HardwareSerial {};
extern HardwareSerial Serial3;
struct TestSerial {
  std::string input, output;
  void begin(int) {}
  int available() const { return input.size(); }
  int read() { char c = input.front(); input.erase(0, 1); return c; }
  void print(char value) { output += value; }
  void print(int value) { output += std::to_string(value); }
  void print(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    output += stream.str();
  }
  void println() { output += '\n'; }
  void println(int value) { print(value); output += '\n'; }
  void println(double value, int precision) { print(value, precision); output += '\n'; }
};
extern TestSerial Serial;
