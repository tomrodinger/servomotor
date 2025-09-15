// Minimal Arduino example: Trapezoid move using built‑in unit conversions
// Goal: spin the motor exactly 1 rotation in 1 second, then stop.
// Sequence:
//  enable MOSFETs -> trapezoidMove(1.0 rotations, 1.0 seconds) -> wait 1.1s -> disable MOSFETs.
//
// Notes:
// - This uses the library's unit conversion (no raw counts/timesteps).
// - Configure Serial1 pins for your board (ESP32 example pins below).
// - Motor is created AFTER Serial1.begin(...) so hardware UART pins are set first.

#include <Arduino.h>
#include <Servomotor.h>

#define ALIAS 'X'                   // Device alias
#define BAUD 230400                 // RS485 UART baud rate
#define DISPLACEMENT_ROTATIONS 1.0f // 1 rotation
#define DURATION_SECONDS 1.0f       // 1 second
#define TOLERANCE_PERCENT 10        // +10% wait margin because the motor's clock is not
                                    //  perfectly accurate
#define WAIT_MS ((unsigned long)(DURATION_SECONDS * 1000.0f * (100 + TOLERANCE_PERCENT) / 100))

// Example RS485 pins for ESP32 DevKit (change as needed for your board)
#if defined(ESP32)
#define RS485_TXD 4              // TX pin to RS485 transceiver
#define RS485_RXD 5              // RX pin from RS485 transceiver
#endif

void setup() {
  Serial.begin(115200); // Console serial, set baud rate here
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // Initialize the RS485 UART (set pins before constructing Servomotor)
#if defined(ESP32)
  Serial1.begin(BAUD, SERIAL_8N1, RS485_RXD, RS485_TXD);
#else
  Serial1.begin(BAUD);
#endif

  // Create the motor object AFTER Serial1 is configured
  Servomotor motor(ALIAS, Serial1);

  // Use units: rotations for position, seconds for time
  motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
  motor.setTimeUnit(TimeUnit::SECONDS);

  motor.enableMosfets();
  motor.trapezoidMove(DISPLACEMENT_ROTATIONS, DURATION_SECONDS);
  delay(WAIT_MS);
  motor.disableMosfets();
}

void loop() {
}