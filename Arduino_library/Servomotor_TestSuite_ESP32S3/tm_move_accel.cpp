#include "tf_framework.h"

// Per-command Arduino test for cmd 11 "Move with acceleration".
// Ported from the legacy test_move_with_acceleration.cpp.
//
// Exercises acceleration moves across the 6 unit systems plus negative
// acceleration. Each sequence applies an acceleration for a duration, then the
// reverse acceleration for the same duration, which brings the motor back to
// zero velocity (required, else async fatal 18). Net displacement of an
// accelerate-then-decelerate pair is a * t^2 (see derivation in the asserts).
//
// One motor on the bus, addressed by unique-ID (extended addressing) via
// tfGetMotor(), so all methods are called WITHOUT a uniqueId arg.

// Wait until the motion queue is empty (all queued moves consumed), then settle.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 1500; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (n == 0) break;
        delay(10);
    }
    delay(200);  // small settle margin
}

void tm_move_accel(void) {
    Serial.println("test_ard_move_accel: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean known state ----
    motor->systemReset();
    delay(1500);  // reset; motor does not respond during reset, so don't check errors

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");

    delay(300);  // let the commutation-alignment transient settle
    getStatusResponse enabled_status = motor->getStatus();
    checkMotorError(*motor, "getStatus");
    printf("Status after enable: 0x%02X\n", enabled_status.statusFlags);
    TEST_RESULT("MOSFETs Successfully Enabled", (enabled_status.statusFlags & 0x02) == 0x02);

    // =====================================================================
    // Test 1: Move with acceleration in ROTATIONS_PER_SECOND_SQUARED
    // =====================================================================
    printf("\n=== Test 1: ROTATIONS_PER_SECOND_SQUARED ===\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test1 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f rotations\n", initial_pos_test1);
    TEST_RESULT("Initial Position Is Zero", approxEqual(initial_pos_test1, 0.0f, 0.05f));

    const float accel_test1 = 2.0f;     // rotations/sec^2
    const float duration_test1 = 1.0f;  // seconds
    printf("Accelerating %.1f rot/s^2 for %.1f s, then decelerating...\n",
           accel_test1, duration_test1);
    motor->moveWithAcceleration(accel_test1, duration_test1);
    checkMotorError(*motor, "moveWithAcceleration");
    motor->moveWithAcceleration(-accel_test1, duration_test1);  // ramp back to zero velocity
    checkMotorError(*motor, "moveWithAcceleration (stop)");
    waitForIdle(motor);

    float final_pos_test1 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    // Total distance for accelerate-then-decelerate pair = a * t^2.
    float expected_pos_test1 = initial_pos_test1 + (accel_test1 * duration_test1 * duration_test1);
    printf("Final: %.2f, Expected: %.2f rotations\n", final_pos_test1, expected_pos_test1);
    TEST_RESULT("Final Position Is Correct (ROTATIONS_PER_SECOND_SQUARED)",
                approxEqual(final_pos_test1, expected_pos_test1, 0.1f));

    // =====================================================================
    // Test 2: Move with acceleration in DEGREES_PER_SECOND_SQUARED
    // =====================================================================
    printf("\n=== Test 2: DEGREES_PER_SECOND_SQUARED ===\n");
    motor->setPositionUnit(PositionUnit::DEGREES);
    motor->setVelocityUnit(VelocityUnit::DEGREES_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::DEGREES_PER_SECOND_SQUARED);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test2 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f degrees\n", initial_pos_test2);

    const float accel_test2 = 720.0f;   // degrees/sec^2
    const float duration_test2 = 1.0f;  // seconds
    printf("Accelerating %.1f deg/s^2 for %.1f s, then decelerating...\n",
           accel_test2, duration_test2);
    motor->moveWithAcceleration(accel_test2, duration_test2);
    checkMotorError(*motor, "moveWithAcceleration");
    motor->moveWithAcceleration(-accel_test2, duration_test2);
    checkMotorError(*motor, "moveWithAcceleration (stop)");
    waitForIdle(motor);

    float final_pos_test2 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test2 = initial_pos_test2 + (accel_test2 * duration_test2 * duration_test2);
    printf("Final: %.2f, Expected: %.2f degrees\n", final_pos_test2, expected_pos_test2);
    TEST_RESULT("Final Position Is Correct (DEGREES_PER_SECOND_SQUARED)",
                approxEqual(final_pos_test2, expected_pos_test2, 10.0f));

    // =====================================================================
    // Test 3: Move with acceleration in COUNTS_PER_SECOND_SQUARED
    // =====================================================================
    printf("\n=== Test 3: COUNTS_PER_SECOND_SQUARED ===\n");
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setVelocityUnit(VelocityUnit::COUNTS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::COUNTS_PER_SECOND_SQUARED);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test3 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f counts\n", initial_pos_test3);

    const float accel_test3 = 2.0f * COUNTS_PER_REVOLUTION;  // counts/sec^2 (== 2 rot/s^2)
    const float duration_test3 = 1.0f;                       // seconds
    printf("Accelerating %.1f counts/s^2 for %.1f s, then decelerating...\n",
           accel_test3, duration_test3);
    motor->moveWithAcceleration(accel_test3, duration_test3);
    checkMotorError(*motor, "moveWithAcceleration");
    motor->moveWithAcceleration(-accel_test3, duration_test3);
    checkMotorError(*motor, "moveWithAcceleration (stop)");
    waitForIdle(motor);

    float final_pos_test3 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test3 = initial_pos_test3 + (accel_test3 * duration_test3 * duration_test3);
    printf("Final: %.2f, Expected: %.2f counts\n", final_pos_test3, expected_pos_test3);
    TEST_RESULT("Final Position Is Correct (COUNTS_PER_SECOND_SQUARED)",
                approxEqual(final_pos_test3, expected_pos_test3, 100000.0f));

    // =====================================================================
    // Test 4: Move with acceleration in RADIANS_PER_SECOND_SQUARED
    // =====================================================================
    printf("\n=== Test 4: RADIANS_PER_SECOND_SQUARED ===\n");
    motor->setPositionUnit(PositionUnit::RADIANS);
    motor->setVelocityUnit(VelocityUnit::RADIANS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::RADIANS_PER_SECOND_SQUARED);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test4 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f radians\n", initial_pos_test4);

    const float accel_test4 = 12.56f;   // radians/sec^2 (~2*pi, i.e. ~1 rot/s^2)
    const float duration_test4 = 1.0f;  // seconds
    printf("Accelerating %.2f rad/s^2 for %.1f s, then decelerating...\n",
           accel_test4, duration_test4);
    motor->moveWithAcceleration(accel_test4, duration_test4);
    checkMotorError(*motor, "moveWithAcceleration");
    motor->moveWithAcceleration(-accel_test4, duration_test4);
    checkMotorError(*motor, "moveWithAcceleration (stop)");
    waitForIdle(motor);

    float final_pos_test4 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test4 = initial_pos_test4 + (accel_test4 * duration_test4 * duration_test4);
    printf("Final: %.2f, Expected: %.2f radians\n", final_pos_test4, expected_pos_test4);
    TEST_RESULT("Final Position Is Correct (RADIANS_PER_SECOND_SQUARED)",
                approxEqual(final_pos_test4, expected_pos_test4, 0.2f));

    // =====================================================================
    // Test 5: Move with acceleration in COUNTS_PER_TIMESTEP_SQUARED
    // =====================================================================
    printf("\n=== Test 5: COUNTS_PER_TIMESTEP_SQUARED ===\n");
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setVelocityUnit(VelocityUnit::COUNTS_PER_TIMESTEP);
    motor->setAccelerationUnit(AccelerationUnit::COUNTS_PER_TIMESTEP_SQUARED);
    motor->setTimeUnit(TimeUnit::TIMESTEPS);

    float initial_pos_test5 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f counts\n", initial_pos_test5);

    // 0.0067 counts/timestep^2 for 31250 timesteps == ~2 rot/s^2 for 1 s.
    const float accel_test5 = 0.0067f;      // counts/timestep^2
    const float duration_test5 = 31250.0f;  // timesteps (1.0 seconds)
    printf("Accelerating %.6f counts/timestep^2 for %.1f timesteps, then decelerating...\n",
           accel_test5, duration_test5);
    motor->moveWithAcceleration(accel_test5, duration_test5);
    checkMotorError(*motor, "moveWithAcceleration");
    motor->moveWithAcceleration(-accel_test5, duration_test5);
    checkMotorError(*motor, "moveWithAcceleration (stop)");
    waitForIdle(motor);

    float final_pos_test5 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test5 = initial_pos_test5 + (accel_test5 * duration_test5 * duration_test5);
    printf("Final: %.2f, Expected: %.2f counts\n", final_pos_test5, expected_pos_test5);
    TEST_RESULT("Final Position Is Correct (COUNTS_PER_TIMESTEP_SQUARED)",
                approxEqual(final_pos_test5, expected_pos_test5, 100.0f));

    // =====================================================================
    // Test 6: Move with negative acceleration
    // =====================================================================
    printf("\n=== Test 6: Negative acceleration ===\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test6 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f rotations\n", initial_pos_test6);

    const float accel_test6 = -2.0f;    // rotations/sec^2 (negative)
    const float duration_test6 = 1.0f;  // seconds
    printf("Accelerating %.1f rot/s^2 for %.1f s, then decelerating...\n",
           accel_test6, duration_test6);
    motor->moveWithAcceleration(accel_test6, duration_test6);
    checkMotorError(*motor, "moveWithAcceleration");
    motor->moveWithAcceleration(-accel_test6, duration_test6);  // opposite -> back to zero velocity
    checkMotorError(*motor, "moveWithAcceleration (stop)");
    waitForIdle(motor);

    float final_pos_test6 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test6 = initial_pos_test6 + (accel_test6 * duration_test6 * duration_test6);
    printf("Final: %.2f, Expected: %.2f rotations\n", final_pos_test6, expected_pos_test6);
    TEST_RESULT("Negative Acceleration Movement Is Correct",
                approxEqual(final_pos_test6, expected_pos_test6, 0.1f));

    // ---- Clean up: motor is at rest (zero velocity); disable and reset. ----
    printf("\nCleaning up...\n");
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(1500);
}
