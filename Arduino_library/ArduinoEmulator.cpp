// ArduinoEmulator.cpp

#include "ArduinoEmulator.h"

// forward declarations from test_one_move.cpp
extern void setup();
extern void loop();

// Define the global 'Serial' object so that 'Motor.cpp' etc. can use it
HardwareSerial Serial;

#if !defined(ARDUINO)
int main(int argc, char* argv[]) {
    // Default port
    std::string port = "/dev/ttyS0";
    if (argc > 1) {
        port = argv[1];
    }

    // Reassign 'Serial' to the chosen port
    Serial = HardwareSerial(port);

    setup(); 
    while (true) {
        loop();
        // small delay so we don't spin too fast
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
#endif
