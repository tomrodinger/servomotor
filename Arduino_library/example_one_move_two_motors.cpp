#include "Servomotor.h"

// Create two motors that share the same serial bus
Servomotor motorX('X', Serial1);
Servomotor motorY('Y', Serial1);

void setup() {
    Serial.begin(115200);  // Debugging port at 115200 baud

    // Configure both motors
    motorX.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motorX.setTimeUnit(TimeUnit::SECONDS);
    
    motorY.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motorY.setTimeUnit(TimeUnit::SECONDS);

    // Enable mosfets for both motors before moving
    motorX.enableMosfets();
    motorY.enableMosfets();

    // Move both motors
    // X: 2 rotations over 3 seconds
    // Y: 1 rotation over 2 seconds
    motorX.trapezoidMove(2.0f, 3.0f);
    motorY.trapezoidMove(1.0f, 2.0f);

    // Wait for moves to complete
    delay(4000);
}

void loop() {
    // Not used
}
