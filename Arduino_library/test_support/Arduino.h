#ifndef SERVOMOTOR_HOST_TEST_ARDUINO_H
#define SERVOMOTOR_HOST_TEST_ARDUINO_H

// Test-only Arduino surface for the payload-level Communication stand-in.
// No real serial implementation, POSIX headers, or device access is involved.
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string.h>

class ConsoleSerial {
public:
    template <typename T> void print(const T& value) { std::cout << value; }
    template <typename T> void println(const T& value) { std::cout << value << '\n'; }
    void println() { std::cout << '\n'; }
};

class HardwareSerial {};
extern ConsoleSerial Serial;
extern HardwareSerial Serial1;

#endif
