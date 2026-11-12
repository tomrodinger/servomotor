#ifndef SERVOMOTOR_FAKE_WIRE_ARDUINO_H
#define SERVOMOTOR_FAKE_WIRE_ARDUINO_H
// Host-only serial/clock fixture. This file cannot open a physical device.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>
#include <algorithm>
#define HEX 16
struct TestClock { static uint32_t now; };
inline uint32_t millis() { return TestClock::now; }
inline void delay(uint32_t ms) { TestClock::now += ms; }
class ConsoleSerial {
public:
    template<class T> void print(const T&) {}
    template<class T> void print(const T&, int) {}
    template<class T> void println(const T&) {}
    template<class T> void println(const T&, int) {}
    void println() {}
};
class HardwareSerial {
public:
    struct Event { uint32_t when; std::vector<uint8_t> bytes; };
    std::vector<uint8_t> tx, rx;
    std::vector<Event> events;
    size_t readAt = 0, eventAt = 0;
    unsigned beginCalls = 0, flushCalls = 0;
    void begin(uint32_t) { ++beginCalls; }
    void reset() { tx.clear(); rx.clear(); events.clear(); readAt = eventAt = 0; TestClock::now = 0; }
    void schedule(uint32_t when, const std::vector<uint8_t>& bytes) {
        events.push_back({when, bytes});
    }
    size_t write(uint8_t value) { tx.push_back(value); return 1; }
    size_t write(const uint8_t* data, size_t size) { tx.insert(tx.end(), data, data + size); return size; }
    int available() {
        ++TestClock::now;
        while(eventAt < events.size() && static_cast<int32_t>(TestClock::now - events[eventAt].when) >= 0) {
            rx.insert(rx.end(), events[eventAt].bytes.begin(), events[eventAt].bytes.end()); ++eventAt;
        }
        return static_cast<int>(rx.size() - readAt);
    }
    int read() { return readAt < rx.size() ? rx[readAt++] : -1; }
    void flush() { ++flushCalls; }
};
extern ConsoleSerial Serial;
extern HardwareSerial Serial1;
#endif
