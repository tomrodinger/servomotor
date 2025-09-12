#include <Servomotor.h>

#define ALIAS 'X'

// Example RS485 pins for ESP32 (adjust if your board uses different pins)
#if defined(ESP32)
#define RS485_TXD 4
#define RS485_RXD 5
#endif

void setup() {
  Serial.begin(115200);

  // Create the motor; serial port opens on first instantiation.
#if defined(ESP32)
  Servomotor motor(ALIAS, Serial1, RS485_RXD, RS485_TXD);
#else
  Servomotor motor(ALIAS, Serial1);
#endif

  // Configure units
  motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
  motor.setTimeUnit(TimeUnit::SECONDS);

  // Enable mosfets before moving
  motor.enableMosfets();

  // Get initial position
  float start_pos = motor.getPosition();
  Serial.print("Position before move: ");
  Serial.print(start_pos, 2);
  Serial.println(" rotations");

  // Perform trapezoid move: 2 rotations over 3 seconds
  motor.trapezoidMove(2.0f, 3.0f);

  // Wait for the move to complete
  delay(4000);  // 4 seconds (slightly longer than the commanded duration)

  // Get final position
  float end_pos = motor.getPosition();
  Serial.print("Position after move: ");
  Serial.print(end_pos, 2);
  Serial.println(" rotations");
}

void loop() {
  // Not used
}
