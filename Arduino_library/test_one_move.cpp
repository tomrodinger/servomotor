#include "Servomotor.h"

// Let's assume we want to test the simpler "Motor" class.
Motor motor;

void setup() {
  Serial.begin(115200);

  // Example usage:
  // 1) We want position in shaft rotations, time in seconds
  motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
  motor.setTimeUnit(TimeUnit::SECONDS);

  // 2) Perform trapezoid move
  //    2 shaft rotations over 3 seconds
  motor.trapezoidMove(2.0f, 3.0f);
}

void loop() {
  // For demonstration, do nothing
}
