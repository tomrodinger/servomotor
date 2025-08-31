#include "Servomotor.h"

void setup() {
    // Create motor instance on Serial1
    Servomotor motor(Serial1);
    
    // Reset and enable motor
    motor.systemReset();
    delay(1500);
    motor.enableMosfets();
    
    // Execute a trapezoid move:
    // Move 10 rotations in 3 seconds
    motor.trapezoidMove(10.0, 3.0);
}

void loop() {
    // Empty
}