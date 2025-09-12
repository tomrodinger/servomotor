#include <Servomotor.h>

#define ALIAS_X 'X'
#define ALIAS_Y 'Y'
#define BAUD 230400

// Example RS485 pins for ESP32 (adjust if your board uses different pins)
#if defined(ESP32)
#define RS485_TXD 4
#define RS485_RXD 5
#endif

void setup() {
  Serial.begin(115200);

  // Create the motors; serial port opens on first instantiation (optional pins for ESP32).
#if defined(ESP32)
  Servomotor motorX(ALIAS_X, Serial1, RS485_RXD, RS485_TXD);
  Servomotor motorY(ALIAS_Y, Serial1, RS485_RXD, RS485_TXD);
#else
  Servomotor motorX(ALIAS_X, Serial1);
  Servomotor motorY(ALIAS_Y, Serial1);
#endif

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
