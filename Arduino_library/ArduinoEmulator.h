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

// Emulated "Serial" class
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

// A global "Serial" object
static Serial_ Serial;

// Replacement for Arduino delay (ms)
inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Macro F()
#ifndef F
#define F(x) (x)
#endif

// A minimal HardwareSerial placeholder
class HardwareSerial {
public:
    void begin(unsigned long baudRate) {
        // no-op
    }
};

#endif // ARDUINO_EMULATOR_H