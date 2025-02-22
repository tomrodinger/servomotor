#ifndef ARDUINO_EMULATOR_H
#define ARDUINO_EMULATOR_H

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdint>
#include <fcntl.h>      // open()
#include <unistd.h>     // read(), write(), close()
#include <termios.h>    // termios
#include <sys/ioctl.h>  // ioctl()
#include <errno.h>      // errno

// Base class for common print/println methods
class SerialBase {
public:
    virtual void print(const char* s) = 0;
    virtual void println(const char* s) = 0;
    virtual void print(float val) = 0;
    virtual void println(float val) = 0;
    virtual void print(int val) = 0;
    virtual void println(int val) = 0;
    virtual void println() = 0;
    virtual ~SerialBase() {}
};

/*
  ConsoleSerial
  - Used for debug output to console (std::cout)
  - Does not attempt to open any actual serial ports
*/
class ConsoleSerial : public SerialBase {
public:
    ConsoleSerial() {}
    
    void begin(long baud) {
        std::cout << "[ConsoleSerial] Debug output initialized\n";
    }

    void print(const char* s) override { std::cout << s; }
    void println(const char* s) override { std::cout << s << std::endl; }
    void print(float val) override { std::cout << val; }
    void println(float val) override { std::cout << val << std::endl; }
    void print(int val) override { std::cout << val; }
    void println(int val) override { std::cout << val << std::endl; }
    void println() override { std::cout << std::endl; }
};

/*
  HardwareSerial
  - Used for actual serial communication with hardware or terminal-based simulators
  - Attempts real serial I/O at 'begin()'
*/
class HardwareSerial : public SerialBase {
public:
    HardwareSerial() : _fd(-1), _portName("(none)") {}
    HardwareSerial(const std::string& portName)
        : _fd(-1), _portName(portName) {}

    // Attempt to open real serial device or terminal at 'baud'
    void begin(long baud) {
        if (_portName == "(none)") {
            std::cerr << "No valid port name set.\n";
            return;
        }

        // Try opening the device
        _fd = ::open(_portName.c_str(), O_RDWR | O_NOCTTY);
        if (_fd < 0) {
            std::cerr << "Failed to open port " << _portName << ": " << strerror(errno) << std::endl;
            return;
        }

        // Get current terminal settings
        struct termios tty;
        if (tcgetattr(_fd, &tty) != 0) {
            std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
            ::close(_fd);
            _fd = -1;
            return;
        }

        // Save original settings for restoration
        tcgetattr(_fd, &_old_tty);

        // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~PARENB;
        // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSTOPB;
        // Clear all bits that set the data size 
        tty.c_cflag &= ~CSIZE;
        // 8 bits per byte (most common)
        tty.c_cflag |= CS8;
        // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag &= ~CRTSCTS;
        // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        tty.c_cflag |= CREAD | CLOCAL;

        // Disable canonical mode
        tty.c_lflag &= ~ICANON;
        // Disable echo
        tty.c_lflag &= ~ECHO;
        // Disable erasure
        tty.c_lflag &= ~ECHOE;
        // Disable new-line echo
        tty.c_lflag &= ~ECHONL;
        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_lflag &= ~ISIG;

        // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        // Disable any special handling of received bytes
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~OPOST;
        // Prevent conversion of newline to carriage return/line feed
        tty.c_oflag &= ~ONLCR;

        // Set baud rate
        speed_t speedVal = B115200; // default
        switch (baud) {
            case 9600:   speedVal = B9600;   break;
            case 19200:  speedVal = B19200;  break;
            case 38400:  speedVal = B38400;  break;
            case 57600:  speedVal = B57600;  break;
            case 115200: speedVal = B115200; break;
            case 230400: speedVal = B230400; break;
            default:
                std::cerr << "Warning: Unsupported baud rate " << baud << ", using 115200\n";
                break;
        }

        cfsetispeed(&tty, speedVal);
        cfsetospeed(&tty, speedVal);

        // Set read timeout
        tty.c_cc[VTIME] = 10;    // Wait for up to 1 second (10 deciseconds)
        tty.c_cc[VMIN] = 0;     // No minimum number of characters

        // Save settings
        if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
            ::close(_fd);
            _fd = -1;
            return;
        }

        std::cout << "[HardwareSerial] Opened " << _portName 
                  << " at baud " << baud << "\n";
    }

    // Write single byte
    void write(uint8_t b) {
        if (_fd >= 0) {
            ::write(_fd, &b, 1);
            tcdrain(_fd);  // Wait for output to be transmitted
        }
    }

    // Write buffer
    void write(const uint8_t* buffer, size_t size) {
        if (_fd >= 0 && buffer && size > 0) {
            ::write(_fd, buffer, size);
            tcdrain(_fd);  // Wait for output to be transmitted
        }
    }

    // Check how many bytes are available
    int available() {
        if (_fd < 0) return 0;
        int nbytes = 0;
        ioctl(_fd, FIONREAD, &nbytes);
        return nbytes;
    }

    // Read one byte
    uint8_t read() {
        if (_fd < 0) return 0;
        uint8_t b;
        ssize_t n = ::read(_fd, &b, 1);
        if (n > 0) return b;
        return 0;
    }

    // Flush
    void flush() {
        if (_fd >= 0) {
            tcdrain(_fd);
        }
    }

    // Print/println methods
    void print(const char* s) override {
        if (_fd >= 0) {
            ::write(_fd, s, strlen(s));
            tcdrain(_fd);
        }
    }
    void println(const char* s) override {
        if (_fd >= 0) {
            ::write(_fd, s, strlen(s));
            ::write(_fd, "\n", 1);
            tcdrain(_fd);
        }
    }

    void print(float val) override {
        if (_fd >= 0) {
            std::string s = std::to_string(val);
            ::write(_fd, s.c_str(), s.length());
            tcdrain(_fd);
        }
    }
    void println(float val) override {
        if (_fd >= 0) {
            std::string s = std::to_string(val) + "\n";
            ::write(_fd, s.c_str(), s.length());
            tcdrain(_fd);
        }
    }

    void print(int val) override {
        if (_fd >= 0) {
            std::string s = std::to_string(val);
            ::write(_fd, s.c_str(), s.length());
            tcdrain(_fd);
        }
    }
    void println(int val) override {
        if (_fd >= 0) {
            std::string s = std::to_string(val) + "\n";
            ::write(_fd, s.c_str(), s.length());
            tcdrain(_fd);
        }
    }

    void println() override {
        if (_fd >= 0) {
            ::write(_fd, "\n", 1);
            tcdrain(_fd);
        }
    }

    // Close if open
    ~HardwareSerial() {
        if (_fd >= 0) {
            // Restore original terminal settings
            tcsetattr(_fd, TCSANOW, &_old_tty);
            ::close(_fd);
        }
    }

private:
    int         _fd;
    std::string _portName;
    struct termios _old_tty;  // Store original terminal settings
};

// We define global 'Serial' (for debug) and 'Serial1' (for hardware) objects
extern ConsoleSerial Serial;
extern HardwareSerial Serial1;

// Replacement for Arduino delay(ms)
inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Arduino-style millis()
inline unsigned long millis() {
    using namespace std::chrono;
    static auto startTime = steady_clock::now();
    auto now = steady_clock::now();
    auto ms  = duration_cast<milliseconds>(now - startTime).count();
    return static_cast<unsigned long>(ms);
}

#endif // ARDUINO_EMULATOR_H
