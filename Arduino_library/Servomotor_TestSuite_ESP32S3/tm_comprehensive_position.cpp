#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_comprehensive_position.cpp
//
// Arduino-library on-device test for cmd 34 Get comprehensive position, ported
// from the legacy test_get_comprehensive_position.cpp. It moves the shaft to a
// known position (1 rotation) then reads getComprehensivePosition() in each of
// the four position units (SHAFT_ROTATIONS / DEGREES / RADIANS / ENCODER_COUNTS),
// asserting the converted commanded + hall values, that they agree with the raw
// int64 response run through convertPosition(), that ENCODER_COUNTS matches the
// raw counts, and that getPosition()/getHallSensorPosition() cross-check against
// the comprehensive response.
//
// One motor on the bus, addressed by unique-ID (extended addressing) via
// tfGetMotor(), so all methods are called WITHOUT a uniqueId arg.
//
// SAFETY:
//   * The only motion is a single trapezoidMove (ends at rest by design) plus a
//     trapezoid return to origin; no velocity moves, so no zero-velocity queue
//     hazard. The device is left disabled + reset.
// ---------------------------------------------------------------------------

static const int64_t COUNTS_PER_ROTATION = COUNTS_PER_REVOLUTION;  // 3276800

// Wait until the motion queue is empty, then settle.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 1000; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (n == 0) break;
        delay(10);
    }
    delay(200);  // settle margin before position reads
}

void tm_comprehensive_position(void) {
    Serial.println("test_ard_comprehensive_position: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean known state ----
    motor->systemReset();
    delay(1500);  // reset; device does not respond during reset

    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle before zeroing

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    // ---- Move to a known position: 1 rotation over 2 seconds ----
    printf("\n---- Moving motor to test position (1 rotation) ----\n");
    motor->trapezoidMove(1.0f, 2.0f);  // trapezoid ends at rest by design
    checkMotorError(*motor, "trapezoidMove to test position");
    waitForIdle(motor);

    // =====================================================================
    // Raw response is the unit-independent ground truth for all conversions.
    // =====================================================================
    auto rawPos = motor->getComprehensivePositionRaw();
    checkMotorError(*motor, "getComprehensivePositionRaw");
    printf("Raw commanded: %lld  hall: %lld  extEnc: %lld\n",
           (long long)rawPos.commandedPosition,
           (long long)rawPos.hallSensorPosition,
           (long long)rawPos.externalEncoderPosition);

    // =====================================================================
    // Test 1: SHAFT_ROTATIONS
    // =====================================================================
    printf("\n---- getComprehensivePosition in SHAFT_ROTATIONS ----\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    auto positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - ROTATIONS");
    printf("commanded=%.4f hall=%.4f extEnc=%.4f\n",
           positions.commandedPosition, positions.hallSensorPosition,
           positions.externalEncoderPosition);

    TEST_RESULT("Commanded Position in SHAFT_ROTATIONS",
                approxEqual(positions.commandedPosition, 1.0f, 0.05f));
    TEST_RESULT("Hall Sensor Position in SHAFT_ROTATIONS",
                approxEqual(positions.hallSensorPosition, 1.0f, 0.05f));

    float expected_commanded = convertPosition((float)rawPos.commandedPosition,
                                               PositionUnit::SHAFT_ROTATIONS,
                                               ConversionDirection::FROM_INTERNAL);
    TEST_RESULT("SHAFT_ROTATIONS Conversion Accurate (Commanded)",
                approxEqual(positions.commandedPosition, expected_commanded, 0.01f));

    // =====================================================================
    // Test 2: DEGREES  (1 rotation == 360 degrees)
    // =====================================================================
    printf("\n---- getComprehensivePosition in DEGREES ----\n");
    motor->setPositionUnit(PositionUnit::DEGREES);
    positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - DEGREES");
    printf("commanded=%.4f hall=%.4f\n",
           positions.commandedPosition, positions.hallSensorPosition);

    TEST_RESULT("Commanded Position in DEGREES",
                approxEqual(positions.commandedPosition, 360.0f, 5.0f));
    TEST_RESULT("Hall Sensor Position in DEGREES",
                approxEqual(positions.hallSensorPosition, 360.0f, 5.0f));

    float expected_degrees_cmd = convertPosition((float)rawPos.commandedPosition,
                                                 PositionUnit::DEGREES,
                                                 ConversionDirection::FROM_INTERNAL);
    TEST_RESULT("DEGREES Conversion Correct (Commanded)",
                approxEqual(positions.commandedPosition, expected_degrees_cmd, 0.1f));

    // =====================================================================
    // Test 3: RADIANS  (1 rotation == 2*pi radians)
    // =====================================================================
    printf("\n---- getComprehensivePosition in RADIANS ----\n");
    motor->setPositionUnit(PositionUnit::RADIANS);
    positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - RADIANS");
    printf("commanded=%.4f hall=%.4f\n",
           positions.commandedPosition, positions.hallSensorPosition);

    TEST_RESULT("Commanded Position in RADIANS",
                approxEqual(positions.commandedPosition, 6.28f, 0.1f));
    TEST_RESULT("Hall Sensor Position in RADIANS",
                approxEqual(positions.hallSensorPosition, 6.28f, 0.1f));

    float expected_radians_cmd = convertPosition((float)rawPos.commandedPosition,
                                                 PositionUnit::RADIANS,
                                                 ConversionDirection::FROM_INTERNAL);
    TEST_RESULT("RADIANS Conversion Correct (Commanded)",
                approxEqual(positions.commandedPosition, expected_radians_cmd, 0.1f));

    // =====================================================================
    // Test 4: ENCODER_COUNTS  (converted values must match the raw counts)
    // =====================================================================
    printf("\n---- getComprehensivePosition in ENCODER_COUNTS ----\n");
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - ENCODER_COUNTS");
    printf("commanded=%.1f hall=%.1f\n",
           positions.commandedPosition, positions.hallSensorPosition);

    TEST_RESULT("ENCODER_COUNTS Match Raw (Commanded)",
                approxEqual(positions.commandedPosition, (float)rawPos.commandedPosition, 300.0f));
    TEST_RESULT("ENCODER_COUNTS Match Raw (Hall Sensor)",
                approxEqual(positions.hallSensorPosition, (float)rawPos.hallSensorPosition, 300.0f));

    // =====================================================================
    // Test 5: Cross-check against standard getPosition / getHallSensorPosition
    // =====================================================================
    printf("\n---- Comparing with standard position methods ----\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    float std_position = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float hall_position = motor->getHallSensorPosition();
    checkMotorError(*motor, "getHallSensorPosition");
    positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - cross-check");
    printf("getPosition=%.4f getHallSensorPosition=%.4f comp.cmd=%.4f comp.hall=%.4f\n",
           std_position, hall_position,
           positions.commandedPosition, positions.hallSensorPosition);

    TEST_RESULT("getPosition matches getComprehensivePosition",
                approxEqual(std_position, positions.commandedPosition, 0.05f));
    TEST_RESULT("getHallSensorPosition matches getComprehensivePosition",
                approxEqual(hall_position, positions.hallSensorPosition, 0.05f));

    // Commanded and hall positions should agree (shaft actually reached target).
    TEST_RESULT("Commanded vs Hall position",
                approxEqual(positions.commandedPosition, positions.hallSensorPosition, 0.1f));

    // ---- Clean up: return to origin at rest, disable, reset. ----
    printf("\nReturning to origin and cleaning up...\n");
    motor->goToPosition(0.0f, 2.0f);  // absolute return to 0, ends at rest
    checkMotorError(*motor, "goToPosition return to origin");
    waitForIdle(motor);
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(1500);
}
