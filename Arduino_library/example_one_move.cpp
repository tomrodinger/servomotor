#include "Servomotor.h"

// Create Servomotor instance on the stack
// This is safe because ArduinoEmulator.cpp ensures Serial1 is initialized
// before setup() is called, and exits if initialization fails
Servomotor motor('X', Serial1);

void setup() {
    Serial.begin(115200);  // Debugging port at 115200 baud

    // Example usage:
    // 1) We want position in shaft rotations, time in seconds
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.setTimeUnit(TimeUnit::SECONDS);

    // Enable mosfets before moving
    motor.enableMosfets();

    // Get initial position
    float start_pos = motor.getPosition();
    Serial.print("Position before move: ");
    Serial.print(start_pos, 2);
    Serial.println(" rotations");

    // 2) Perform trapezoid move
    //    2 shaft rotations over 3 seconds
    motor.trapezoidMove(2.0f, 3.0f);

    // Wait for move to complete
    delay(4000);  // Wait 4 seconds (longer than the move)

    // Get final position
    float end_pos = motor.getPosition();
    Serial.print("Position after move: ");
    Serial.print(end_pos, 2);
    Serial.println(" rotations");

    // Finished
}

void loop() {
    // Not used
}
