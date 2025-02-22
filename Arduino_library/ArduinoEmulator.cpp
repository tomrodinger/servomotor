// ArduinoEmulator.cpp

#include "ArduinoEmulator.h"

// forward declarations from test_one_move.cpp
extern void setup();
extern void loop();

// Define the global 'Serial' and 'Serial1' objects
ConsoleSerial Serial;  // Debug output to console
HardwareSerial Serial1;  // Hardware communication

#if !defined(ARDUINO)
int main(int argc, char* argv[]) {
    // Check if port is provided
    if (argc < 2) {
        std::cerr << "Error: Serial port must be specified.\n";
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        std::cerr << "Example: " << argv[0] << " /dev/ttys003\n";
        return 1;
    }

    // Initialize Serial1 with the specified port
    Serial1 = HardwareSerial(argv[1]);
    if (!Serial1.begin(230400)) {
        std::cerr << "Failed to initialize serial port. Exiting.\n";
        return 1;  // Exit with error
    }

    setup();
    while (true) {
        loop();
        // small delay so we don't spin too fast
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
#endif
