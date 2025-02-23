#include "ServomotorCommands.h"

// Create ServoMotor instance on the stack
// This is safe because ArduinoEmulator.cpp ensures Serial1 is initialized
// before setup() is called, and exits if initialization fails
ServoMotor motor('X', Serial1);

void setup() {
    Serial.begin(115200);  // Debugging port at 115200 baud

    // Example usage:
    // 1) We want position in shaft rotations, time in seconds
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.setTimeUnit(TimeUnit::SECONDS);

    // 2) Perform trapezoid move
    //    2 shaft rotations over 3 seconds
    motor.trapezoidMove(2.0f, 3.0f);

    // Exit after move is complete
    exit(0);
}

void loop() {
    // Not used
}
