#include <Servomotor.h>

#define ALIAS 'X'
#define BAUD 230400

// Example RS485 pins for ESP32 (adjust if your board uses different pins)
#if defined(ESP32)
#define RS485_TXD 4
#define RS485_RXD 5
#endif

void setup() {
  Serial.begin(115200);

  // Create the motor; serial port opens on first instantiation (optional pins for ESP32).
#if defined(ESP32)
  Servomotor motor(ALIAS, Serial1, RS485_RXD, RS485_TXD);
#else
  Servomotor motor(ALIAS, Serial1);
#endif

  // Configure motor units
  motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
  motor.setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
  motor.setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
  motor.setTimeUnit(TimeUnit::SECONDS);

  // Enable mosfets before moving
  motor.enableMosfets();

  // Get initial position
  float start_pos = motor.getPosition();
  Serial.print("Position before move: ");
  Serial.print(start_pos, 2);
  Serial.println(" rotations");

  // Create a sequence of moves
  const uint8_t multiMoveCount = 4;
  uint32_t multiMoveTypes = 0b1001;  // 1st and 4th are velocity, 2nd and 3rd are acceleration

  // Create move list with user units
  multimoveListConverted_t multiMoveList[multiMoveCount] = {
    {2.0f, 1.0f},   // Velocity: 2 rot/sec for 1 sec
    {2.0f, 1.0f},   // Acceleration: 2 rot/sec² for 1 sec
    {-2.0f, 1.0f},  // Acceleration: -2 rot/sec² for 1 sec
    {0.0f, 0.1f}    // Velocity: 0 rot/sec for 0.1 sec (stop)
  };

  // Execute the multi-move sequence with automatic unit conversion
  Serial.println("Executing multi-move sequence...");
  motor.multimove(multiMoveCount, multiMoveTypes, multiMoveList);

  // Calculate total duration of the sequence (+5% buffer)
  float totalDuration = 0.0f;
  for (int i = 0; i < multiMoveCount; i++) {
    totalDuration += multiMoveList[i].duration;
  }
  float waitTime = totalDuration * 1.05f;
  Serial.print("Waiting for ");
  Serial.print(waitTime, 2);
  Serial.println(" seconds");

  // Wait for the sequence to complete
  delay(static_cast<int>(waitTime * 1000));

  // Get final position
  float end_pos = motor.getPosition();
  Serial.print("Position after move: ");
  Serial.print(end_pos, 2);
  Serial.println(" rotations");

  // Expected end position (see comments in original sketch)
  Serial.print("Expected position: ");
  Serial.print(start_pos + 8.0f, 2);
  Serial.println(" rotations");
}

void loop() {
  // Not used
}