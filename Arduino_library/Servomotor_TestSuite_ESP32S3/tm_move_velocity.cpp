#include "tf_framework.h"

// Per-command Arduino test for Move With Velocity (cmd 24).
//
// Ported from the legacy standalone test_move_with_velocity.cpp. Verifies that
// moveWithVelocity honours the selected velocity unit system, that the resulting
// displacement (velocity * duration) accumulates across moves, that raw encoder
// counts agree with the accumulated rotations, and that negative velocity moves
// the shaft the other way. Every velocity move is followed by a zero-velocity
// stop move so the motion queue drains to rest (avoids async fatal 18).
//
// Commanded velocity is kept at 2 rot/s equivalents throughout (as in the source).
//
// One motor on the bus, addressed by unique-ID (extended addressing) via
// tfGetMotor(), so all methods are called WITHOUT a uniqueId arg.

static const int64_t COUNTS_PER_ROTATION = COUNTS_PER_REVOLUTION;  // 3276800

void tm_move_velocity(void) {
    Serial.println("test_ard_move_velocity: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean known state ----
    motor->systemReset();
    delay(1500);  // reset; motor does not respond during reset, so don't check errors

    // Enable MOSFETs
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");

    delay(100);  // let status update
    getStatusResponse enabled_status = motor->getStatus();
    checkMotorError(*motor, "getStatus");
    printf("Status after enable: 0x%02X\n", enabled_status.statusFlags);
    TEST_RESULT("MOSFETs Successfully Enabled", (enabled_status.statusFlags & 0x02) == 0x02);

    // =====================================================================
    // Test 1: Move with velocity in ROTATIONS_PER_SECOND
    // =====================================================================
    printf("\n=== Test 1: Move with velocity in ROTATIONS_PER_SECOND ===\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test1 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f rotations\n", initial_pos_test1);
    TEST_RESULT("Initial Position Is Zero", approxEqual(initial_pos_test1, 0.0f, 0.05f));

    // Move at 2 rotations/sec for 1 second == 2 rotations.
    const float velocity_test1 = 2.0f;  // rotations per second
    const float duration_test1 = 1.0f;  // seconds
    printf("Moving at %.1f rotations/sec for %.1f seconds...\n", velocity_test1, duration_test1);
    motor->moveWithVelocity(velocity_test1, duration_test1);
    checkMotorError(*motor, "moveWithVelocity");

    // Queue a zero-velocity move to bring the motor to rest.
    printf("Stopping with velocity 0 for 0.1 seconds...\n");
    motor->moveWithVelocity(0.0f, 0.1f);
    checkMotorError(*motor, "moveWithVelocity (stop)");

    delay(1200);  // wait for the move to complete

    float final_pos_test1 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test1 = initial_pos_test1 + (velocity_test1 * duration_test1);
    printf("Final position: %.2f rotations (expected %.2f)\n", final_pos_test1, expected_pos_test1);
    TEST_RESULT("Final Position Is Correct (ROTATIONS_PER_SECOND)",
                approxEqual(final_pos_test1, expected_pos_test1, 0.1f));

    // =====================================================================
    // Test 2: Move with velocity in DEGREES_PER_SECOND
    // =====================================================================
    printf("\n=== Test 2: Move with velocity in DEGREES_PER_SECOND ===\n");
    motor->setPositionUnit(PositionUnit::DEGREES);
    motor->setVelocityUnit(VelocityUnit::DEGREES_PER_SECOND);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test2 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f degrees\n", initial_pos_test2);
    // Not expected to be zero: position accumulated from test 1.

    // 720 degrees/sec for 1 second == 2 rotations.
    const float velocity_test2 = 720.0f;  // degrees per second
    const float duration_test2 = 1.0f;    // seconds
    printf("Moving at %.1f degrees/sec for %.1f seconds...\n", velocity_test2, duration_test2);
    motor->moveWithVelocity(velocity_test2, duration_test2);
    checkMotorError(*motor, "moveWithVelocity");

    printf("Stopping with velocity 0 for 0.1 seconds...\n");
    motor->moveWithVelocity(0.0f, 0.1f);
    checkMotorError(*motor, "moveWithVelocity (stop)");

    delay(1200);

    float final_pos_test2 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test2 = initial_pos_test2 + (velocity_test2 * duration_test2);
    printf("Final position: %.2f degrees (expected %.2f)\n", final_pos_test2, expected_pos_test2);
    TEST_RESULT("Final Position Is Correct (DEGREES_PER_SECOND)",
                approxEqual(final_pos_test2, expected_pos_test2, 10.0f));

    // =====================================================================
    // Test 3: Move with velocity in COUNTS_PER_SECOND
    // =====================================================================
    printf("\n=== Test 3: Move with velocity in COUNTS_PER_SECOND ===\n");
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setVelocityUnit(VelocityUnit::COUNTS_PER_SECOND);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test3 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f counts\n", initial_pos_test3);
    // Not expected to be zero: position accumulated from tests 1 and 2.

    // 2 * COUNTS_PER_REVOLUTION counts/sec for 1 second == 2 rotations.
    const float velocity_test3 = 2.0f * COUNTS_PER_REVOLUTION;  // counts per second
    const float duration_test3 = 1.0f;                          // seconds
    printf("Moving at %.1f counts/sec for %.1f seconds...\n", velocity_test3, duration_test3);
    motor->moveWithVelocity(velocity_test3, duration_test3);
    checkMotorError(*motor, "moveWithVelocity");

    printf("Stopping with velocity 0 for 0.1 seconds...\n");
    motor->moveWithVelocity(0.0f, 0.1f);
    checkMotorError(*motor, "moveWithVelocity (stop)");

    delay(1200);

    float final_pos_test3 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test3 = initial_pos_test3 + (velocity_test3 * duration_test3);
    printf("Final position: %.2f counts (expected %.2f)\n", final_pos_test3, expected_pos_test3);
    TEST_RESULT("Final Position Is Correct (COUNTS_PER_SECOND)",
                approxEqual(final_pos_test3, expected_pos_test3, 100000.0f));

    // =====================================================================
    // Test 4: Raw encoder counts agree with accumulated rotations.
    // =====================================================================
    printf("\n=== Test 4: Get position in raw encoder counts ===\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // 2 rotations/sec for 1 second == 2 rotations.
    const float velocity_test4 = 2.0f;  // rotations per second
    const float duration_test4 = 1.0f;  // seconds
    printf("Moving at %.1f rotations/sec for %.1f seconds...\n", velocity_test4, duration_test4);
    motor->moveWithVelocity(velocity_test4, duration_test4);
    checkMotorError(*motor, "moveWithVelocity");

    printf("Stopping with velocity 0 for 0.1 seconds...\n");
    motor->moveWithVelocity(0.0f, 0.1f);
    checkMotorError(*motor, "moveWithVelocity (stop)");

    delay(1200);

    float pos_in_rotations = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Position in rotations: %.2f rotations\n", pos_in_rotations);

    int64_t raw_pos = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw");
    printf("Position in raw encoder counts: %lld\n", (long long)raw_pos);

    // Accumulated movement: 2 (T1) + 2 (T2) + 2 (T3) + 2 (T4) = 8 rotations.
    const int64_t TOTAL_ROTATIONS = 8;
    int64_t expected_raw_pos = TOTAL_ROTATIONS * COUNTS_PER_ROTATION;
    printf("Expected raw position: %lld counts\n", (long long)expected_raw_pos);
    TEST_RESULT("Raw Position Is Correct", llabs(raw_pos - expected_raw_pos) < 1000);

    // =====================================================================
    // Test 5: Move with negative velocity.
    // =====================================================================
    printf("\n=== Test 5: Move with negative velocity ===\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setTimeUnit(TimeUnit::SECONDS);

    float initial_pos_test5 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position: %.2f rotations\n", initial_pos_test5);

    // -2 rotations/sec for 1 second == -2 rotations.
    const float velocity_test5 = -2.0f;  // rotations per second
    const float duration_test5 = 1.0f;   // seconds
    printf("Moving at %.1f rotations/sec for %.1f seconds...\n", velocity_test5, duration_test5);
    motor->moveWithVelocity(velocity_test5, duration_test5);
    checkMotorError(*motor, "moveWithVelocity");

    printf("Stopping with velocity 0 for 0.1 seconds...\n");
    motor->moveWithVelocity(0.0f, 0.1f);
    checkMotorError(*motor, "moveWithVelocity (stop)");

    delay(1200);

    float final_pos_test5 = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    float expected_pos_test5 = initial_pos_test5 + (velocity_test5 * duration_test5);
    printf("Final position: %.2f rotations (expected %.2f)\n", final_pos_test5, expected_pos_test5);
    TEST_RESULT("Negative Velocity Movement Is Correct",
                approxEqual(final_pos_test5, expected_pos_test5, 0.1f));

    // ---- Clean up: motor is already at rest; disable and reset. ----
    printf("\nCleaning up...\n");
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(1500);
}
