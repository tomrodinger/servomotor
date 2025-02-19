#ifndef ARDUINO_EMULATOR_H
#define ARDUINO_EMULATOR_H

/*
  ArduinoEmulator.h
  Minimal Arduino environment emulator so you can compile 
  and run your code on a Mac (or other desktop) 
  without actual Arduino hardware.
*/

#include <iostream>
#include <string>
#include <cmath>
#include <cstdint>
#include <thread>
#include <chrono>

// Mock definition of Serial_ class
class Serial_ {
public:
    void begin(unsigned long baudRate) {
        std::cout << "[Serial] begin at baud " << baudRate << "\n";
    }

    void print(const char* s) {
        std::cout << s;
    }
    void println(const char* s) {
        std::cout << s << std::endl;
    }

    void print(float val) {
        std::cout << val;
    }
    void println(float val) {
        std::cout << val << std::endl;
    }

    void print(int val) {
        std::cout << val;
    }
    void println(int val) {
        std::cout << val << std::endl;
    }

    void println() {
        std::cout << std::endl;
    }
};

// A global "Serial" object for emulation
static Serial_ Serial;

// Minimal HardwareSerial placeholder (will be used for Arduino-like emulation)
class HardwareSerial {
public:
    void begin(long baud) {
        // Mock initialization for Mac
    }

    void write(uint8_t byte) {
        // Mock write functionality
    }

    void write(const uint8_t *buffer, size_t size) {
        // Mock buffer write functionality
    }

    int available() {
        // Mock available check
        return 1;
    }

    uint8_t read() {
        // Mock read functionality
        return 0;
    }

    void flush() {
        // Mock flush
    }

    void print(const char* str) {
        std::cout << str;
    }

    void println(const char* str) {
        std::cout << str << std::endl;
    }
};

// Replacement for Arduino delay (ms)
inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Macro F()
#ifndef F
#define F(x) (x)
#endif

// Define millis function for time tracking (since Arduino's millis() won't work on macOS)
inline unsigned long millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

#endif // ARDUINO_EMULATOR_H
