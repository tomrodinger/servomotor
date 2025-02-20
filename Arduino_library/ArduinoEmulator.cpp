// ArduinoEmulator.cpp
#include "ArduinoEmulator.h"

// Declare the setup and loop functions that should be defined elsewhere
extern void setup();  // Defined in user code
extern void loop();   // Defined in user code

// Ensure that this is only compiled in the emulation environment
#if !defined(ARDUINO)
int main() {
    setup();  // Call setup function
    while (true) {
        loop();  // Call loop function repeatedly
    }
    return 0;
}
#endif
