#pragma once
// ---------------------------------------------------------------------------
// Portable mini test framework for the Servomotor Arduino library test suite.
//
// Compiles in BOTH environments:
//   - ESP32-S3 sketch build (ARDUINO defined): runs natively on the custom
//     Gearotons ESP32-S3 RS485 controller; motor on Serial1 (RX=GPIO5, TX=GPIO4),
//     results over USB CDC. The harness (the .ino) sequences one test per boot,
//     resetting the ESP32 between tests; progress + stats persist in NVS.
//   - Host build (ARDUINO not defined): compiles on the Mac against
//     ../ArduinoEmulator.h and talks to a motor through a USB-RS485 port.
//     See host_main.cpp / build_host_suite.sh.
//
// Test modules: one tm_*.cpp per command group. Entry point signature:
//     void tm_<name>(void)
// Modules may use ONLY what this header provides:
//     tfGetMotor(), TEST_RESULT(), approxEqual(), checkMotorError(),
//     printf() (redirected to USB CDC on-device), Serial.print/println,
//     delay(), millis(), and the Servomotor API.
// Modules must NOT: define setup()/loop(), call exit()/abort()/ESP.restart(),
// include any other headers (this one provides cstdio/cmath/cstring/string/
// vector), or use Serial.printf (the printf macro below would mangle it).
// File-scope state must be static and re-initialized at entry (the host
// harness can run modules back-to-back in one process).
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <Servomotor.h>
#else
  #include "../ArduinoEmulator.h"
  #include "../Servomotor.h"
  #include <cstdlib>
#endif

// printf redirection: on the ESP32, stdout goes to UART0 (invisible over USB),
// so route printf through Serial (USB CDC). Plain passthrough on the host.
// NOTE: object-like macro — never write `Serial.printf` in code that includes
// this header; use printf() or Serial.print().
int tf_printf(const char* fmt, ...);
#define printf tf_printf

inline bool approxEqual(float a, float b, float tolerance = 0.01f) {
    return std::fabs(a - b) <= tolerance;
}

struct TestResult {
    std::string name;
    bool passed;
};

class TestRunner {
public:
    static void reset();  // clear results; the harness calls this before each test
    static void addResult(const std::string& name, bool passed);
    static int  passCount();
    static int  failCount();
    static void printResults();
private:
    static std::vector<TestResult> results;
};

#define TEST_RESULT(name, condition)                                        \
    do {                                                                    \
        bool tf_passed_ = (condition);                                      \
        std::string tf_name_(name);                                         \
        TestRunner::addResult(tf_name_, tf_passed_);                        \
        tf_printf("%s: %s\n", tf_name_.c_str(), tf_passed_ ? "PASS" : "FAIL"); \
    } while (0)

// Records a FAIL result if the motor has a pending error (does NOT exit —
// on-device there is no process to exit; the test continues).
void checkMotorError(Servomotor& motor, const std::string& commandName);

// The motor under test, addressed by unique ID (extended addressing).
// Created once per boot by the harness; modules just call this.
Servomotor* tfGetMotor();

// Compatibility alias so bodies ported from the old test_ard_* files work.
inline Servomotor* Servomotor_TestModeConvenienceWrapper() { return tfGetMotor(); }
