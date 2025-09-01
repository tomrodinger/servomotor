#include <M17Servomotor.h>

// Create motor object on Serial1
M17Servomotor motor(&Serial1, 115200);

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize motor communication
  motor.begin();
  
  // Wait for motor to be ready
  delay(1000);
  
  // Enable mosfets (power on)
  motor.enableMosfets();
  Serial.println("Motor enabled");
  
  // Wait for motor to stabilize
  delay(500);
}

void loop() {
  // Move forward 10000 steps
  Serial.println("Moving forward...");
  motor.trapezoidMove(
    10000,  // position in steps
    5000,   // max velocity in steps/sec
    1000    // acceleration in steps/sec^2
  );
  
  // Wait for move to complete
  motor.waitForMoveComplete();
  delay(1000);
  
  // Move back to origin
  Serial.println("Moving back...");
  motor.trapezoidMove(0, 5000, 1000);
  
  // Wait for move to complete
  motor.waitForMoveComplete();
  delay(1000);
}