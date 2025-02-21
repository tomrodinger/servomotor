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

/*
  HardwareSerial
  - If a valid port name is provided, attempts real serial I/O at 'begin()'
  - Also implements print/println methods for debugging
*/
class HardwareSerial {
public:
    HardwareSerial() : _fd(-1), _portName("(none)") {}
    HardwareSerial(const std::string& portName)
        : _fd(-1), _portName(portName) {}

    // Attempt to open real serial device at 'baud'
    void begin(long baud) {
        if (_portName == "(none)") {
            std::cerr << "No valid serial port name set.\n";
            return;
        }
        // Try opening the device
        _fd = ::open(_portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (_fd < 0) {
            std::cerr << "Failed to open port " << _portName << std::endl;
            return;
        }

        // Setup port
        struct termios tty;
        if (tcgetattr(_fd, &tty) != 0) {
            std::cerr << "Error from tcgetattr.\n";
            return;
        }

        speed_t speedVal = B115200; // default
        switch (baud) {
            case 9600:   speedVal = B9600;   break;
            case 19200:  speedVal = B19200;  break;
            case 38400:  speedVal = B38400;  break;
            case 57600:  speedVal = B57600;  break;
            case 115200: speedVal = B115200; break;
            // Add more as you wish
        }

        cfsetospeed(&tty, speedVal);
        cfsetispeed(&tty, speedVal);

        // 8N1
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_cflag |= CLOCAL | CREAD;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(ICRNL | INLCR);
        tty.c_oflag &= ~(OPOST);
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error from tcsetattr.\n";
        }

        // Make blocking
        int flags = fcntl(_fd, F_GETFL);
        flags &= ~O_NONBLOCK;
        fcntl(_fd, F_SETFL, flags);

        std::cout << "[HardwareSerial] Opened " << _portName 
                  << " at baud " << baud << "\n";
    }

    // Write single byte
    void write(uint8_t b) {
        if (_fd >= 0) {
            ::write(_fd, &b, 1);
        }
    }

    // Write buffer
    void write(const uint8_t* buffer, size_t size) {
        if (_fd >= 0 && buffer && size > 0) {
            ::write(_fd, buffer, size);
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

    // Print/println methods for debugging
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

    // Close if open
    ~HardwareSerial() {
        if (_fd >= 0) {
            ::close(_fd);
        }
    }

private:
    int         _fd;
    std::string _portName;
};

// We define a global 'Serial' object. 
extern HardwareSerial Serial;

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

// If not on Arduino, we define main in ArduinoEmulator.cpp

#endif // ARDUINO_EMULATOR_H
