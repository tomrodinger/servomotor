#include "tf_framework.h"

// Per-command Arduino test for the Multi-move command (cmd 29):
//   Servomotor::multimove()     -- mixed velocity/acceleration lists, user units
//   Servomotor::multimoveRaw()  -- lists in raw internal units
//
// Ported from the legacy test_multi_move.cpp. Every case is preserved:
//   - velocity moves expressed in four velocity unit systems
//   - acceleration moves expressed in four acceleration unit systems
//   - the same move in three time unit systems
//   - multiMoveRaw with internal units
//   - a complex mixed velocity/acceleration sequence
//   - maximum-count (32) velocity-move and acceleration-move lists
//
// Every move list ends at zero velocity (final velocity move of 0 for a short
// duration, or an accel wiggle that returns velocity to 0), so no async fatal 18.
//
// One motor on the bus, addressed by unique-ID (extended addressing) via
// tfGetMotor(), so all methods are called WITHOUT a uniqueId arg.

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Calculate and perform delay for multiMove sequences (durations in user time
// units of SECONDS -- callers always pass a seconds-valued array here).
template<typename T>
static void multiMoveDelay(uint8_t multiMoveCount, const T* multiMoveList) {
    float totalDuration = 0.0f;
    for (int i = 0; i < multiMoveCount; i++) {
        totalDuration += multiMoveList[i].duration;
    }
    float waitTime = totalDuration * 1.05f;  // 5% buffer to ensure completion
    printf("Waiting for %.2f seconds (sequence duration: %.2f seconds + 5%% buffer)\n",
           waitTime, totalDuration);
    delay(static_cast<int>(waitTime * 1000));
}

// Calculate and perform delay for multiMoveRaw sequences with internal time units.
static void multiMoveDelayInternalTimeInput(uint8_t multiMoveCount,
                                            const multimoveList_t* multiMoveList) {
    float totalDurationInternalUnits = 0.0f;
    for (int i = 0; i < multiMoveCount; i++) {
        totalDurationInternalUnits += multiMoveList[i].timeSteps;
    }
    // Convert from internal time units to seconds (CONVERSION_FACTOR_SECONDS = 31250.0f)
    float totalDuration = totalDurationInternalUnits / 31250.0f;
    float waitTime = totalDuration * 1.05f;
    printf("Waiting for %.2f seconds (sequence duration: %.2f seconds + 5%% buffer)\n",
           waitTime, totalDuration);
    delay(static_cast<int>(waitTime * 1000));
}

// ---------------------------------------------------------------------------
// Test: multiMove with different velocity units
// ---------------------------------------------------------------------------
static void testMultiMoveVelocityUnits(Servomotor* motor) {
    printf("\n=== Testing multiMove with different velocity units ===\n");

    // Test with ROTATIONS_PER_SECOND
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(ROTATIONS_PER_SECOND)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    const uint8_t multiMoveCount = 2;
    uint32_t multiMoveTypes = 0b11;  // Both are velocity moves

    multimoveListConverted_t multiMoveList[multiMoveCount] = {
        {2.0f, 1.0f},    // Velocity move: 2 rot/sec for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 rot/sec for 0.1 sec (stop)
    };

    printf("Testing with ROTATIONS_PER_SECOND...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (ROTATIONS_PER_SECOND)");
    multiMoveDelay(multiMoveCount, multiMoveList);

    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (ROTATIONS_PER_SECOND)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with ROTATIONS_PER_SECOND", approxEqual(end_pos, 2.0f));

    // Test with RPM (120 RPM = 2 RPS)
    motor->setVelocityUnit(VelocityUnit::RPM);
    checkMotorError(*motor, "setVelocityUnit(RPM)");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    multimoveListConverted_t multiMoveListRPM[multiMoveCount] = {
        {120.0f, 1.0f},  // Velocity move: 120 RPM for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 RPM for 0.1 sec (stop)
    };
    printf("Testing with RPM...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListRPM);
    checkMotorError(*motor, "multiMove (RPM)");
    multiMoveDelay(multiMoveCount, multiMoveListRPM);

    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (RPM)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with RPM", approxEqual(end_pos, 2.0f));

    // Test with DEGREES_PER_SECOND (720 deg/sec = 2 RPS)
    motor->setVelocityUnit(VelocityUnit::DEGREES_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(DEGREES_PER_SECOND)");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    multimoveListConverted_t multiMoveListDegrees[multiMoveCount] = {
        {720.0f, 1.0f},  // Velocity move: 720 deg/sec for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 deg/sec for 0.1 sec (stop)
    };
    printf("Testing with DEGREES_PER_SECOND...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListDegrees);
    checkMotorError(*motor, "multiMove (DEGREES_PER_SECOND)");
    multiMoveDelay(multiMoveCount, multiMoveListDegrees);

    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (DEGREES_PER_SECOND)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with DEGREES_PER_SECOND", approxEqual(end_pos, 2.0f));

    // Test with RADIANS_PER_SECOND (4pi rad/sec = 2 RPS)
    motor->setVelocityUnit(VelocityUnit::RADIANS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(RADIANS_PER_SECOND)");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    multimoveListConverted_t multiMoveListRadians[multiMoveCount] = {
        {4.0f * (float)M_PI, 1.0f},  // Velocity move: 4pi rad/sec for 1 sec
        {0.0f, 0.1f}                 // Velocity move: 0 rad/sec for 0.1 sec (stop)
    };
    printf("Testing with RADIANS_PER_SECOND...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListRadians);
    checkMotorError(*motor, "multiMove (RADIANS_PER_SECOND)");
    multiMoveDelay(multiMoveCount, multiMoveListRadians);

    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (RADIANS_PER_SECOND)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with RADIANS_PER_SECOND", approxEqual(end_pos, 2.0f));
}

// ---------------------------------------------------------------------------
// Test: multiMove with different acceleration units
// ---------------------------------------------------------------------------
static void testMultiMoveAccelerationUnits(Servomotor* motor) {
    printf("\n=== Testing multiMove with different acceleration units ===\n");

    // Test with ROTATIONS_PER_SECOND_SQUARED
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(ROTATIONS_PER_SECOND_SQUARED)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    const uint8_t multiMoveCount = 3;
    uint32_t multiMoveTypes = 0b001;  // First is velocity move, others are acceleration moves

    multimoveListConverted_t multiMoveList[multiMoveCount] = {
        {0.0f, 0.1f},     // Velocity move: 0 rot/sec for 0.1 sec (start from rest)
        {2.0f, 2.0f},     // Acceleration move: 2 rot/sec^2 for 2 sec
        {-2.0f, 2.0f}     // Acceleration move: -2 rot/sec^2 for 2 sec (stop)
    };
    printf("Testing with ROTATIONS_PER_SECOND_SQUARED...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (ROTATIONS_PER_SECOND_SQUARED)");
    multiMoveDelay(multiMoveCount, multiMoveList);

    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (ROTATIONS_PER_SECOND_SQUARED)");
    printf("Position after move: %.2f rotations\n", end_pos);
    // Accelerate 0->4 rot/s over 2s = 4 rot; decelerate 4->0 over 2s = 4 rot; total 8 rot.
    TEST_RESULT("multiMove with ROTATIONS_PER_SECOND_SQUARED", approxEqual(end_pos, 8.0f));

    // Test with RPM_PER_SECOND (120 RPM/sec = 2 RPS^2)
    motor->setAccelerationUnit(AccelerationUnit::RPM_PER_SECOND);
    checkMotorError(*motor, "setAccelerationUnit(RPM_PER_SECOND)");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    multimoveListConverted_t multiMoveListRPM[multiMoveCount] = {
        {0.0f, 0.1f},      // Velocity move: 0 rot/sec for 0.1 sec (start from rest)
        {120.0f, 2.0f},    // Acceleration move: 120 RPM/sec for 2 sec
        {-120.0f, 2.0f}    // Acceleration move: -120 RPM/sec for 2 sec (stop)
    };
    printf("Testing with RPM_PER_SECOND...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListRPM);
    checkMotorError(*motor, "multiMove (RPM_PER_SECOND)");
    multiMoveDelay(multiMoveCount, multiMoveListRPM);

    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (RPM_PER_SECOND)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with RPM_PER_SECOND", approxEqual(end_pos, 8.0f));

    // Test with DEGREES_PER_SECOND_SQUARED (720 deg/sec^2 = 2 RPS^2)
    motor->setAccelerationUnit(AccelerationUnit::DEGREES_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(DEGREES_PER_SECOND_SQUARED)");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    multimoveListConverted_t multiMoveListDegrees[multiMoveCount] = {
        {0.0f, 0.1f},      // Velocity move: 0 rot/sec for 0.1 sec (start from rest)
        {720.0f, 2.0f},    // Acceleration move: 720 deg/sec^2 for 2 sec
        {-720.0f, 2.0f}    // Acceleration move: -720 deg/sec^2 for 2 sec (stop)
    };
    printf("Testing with DEGREES_PER_SECOND_SQUARED...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListDegrees);
    checkMotorError(*motor, "multiMove (DEGREES_PER_SECOND_SQUARED)");
    multiMoveDelay(multiMoveCount, multiMoveListDegrees);

    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (DEGREES_PER_SECOND_SQUARED)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with DEGREES_PER_SECOND_SQUARED", approxEqual(end_pos, 8.0f));

    // Test with RADIANS_PER_SECOND_SQUARED (4pi rad/sec^2 = 2 RPS^2)
    motor->setAccelerationUnit(AccelerationUnit::RADIANS_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(RADIANS_PER_SECOND_SQUARED)");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    multimoveListConverted_t multiMoveListRadians[multiMoveCount] = {
        {0.0f, 0.1f},                 // Velocity move: 0 rot/sec for 0.1 sec (start from rest)
        {4.0f * (float)M_PI, 2.0f},   // Acceleration move: 4pi rad/sec^2 for 2 sec
        {-4.0f * (float)M_PI, 2.0f}   // Acceleration move: -4pi rad/sec^2 for 2 sec (stop)
    };
    printf("Testing with RADIANS_PER_SECOND_SQUARED...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListRadians);
    checkMotorError(*motor, "multiMove (RADIANS_PER_SECOND_SQUARED)");
    multiMoveDelay(multiMoveCount, multiMoveListRadians);

    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (RADIANS_PER_SECOND_SQUARED)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with RADIANS_PER_SECOND_SQUARED", approxEqual(end_pos, 8.0f));
}

// ---------------------------------------------------------------------------
// Test: multiMove with different time units
// ---------------------------------------------------------------------------
static void testMultiMoveTimeUnits(Servomotor* motor) {
    printf("\n=== Testing multiMove with different time units ===\n");

    // Test with SECONDS
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(ROTATIONS_PER_SECOND)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    const uint8_t multiMoveCount = 2;
    uint32_t multiMoveTypes = 0b11;  // Both are velocity moves
    const float multiMoveTime0 = 1.0f;
    const float multiMoveTime1 = 0.1f;

    multimoveListConverted_t multiMoveList[multiMoveCount] = {
        {2.0f, multiMoveTime0},    // Velocity move: 2 rot/sec for 1 sec
        {0.0f, multiMoveTime1}     // Velocity move: 0 rot/sec for 0.1 sec (stop)
    };
    printf("Testing with SECONDS...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (SECONDS)");
    multiMoveDelay(multiMoveCount, multiMoveList);

    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (SECONDS)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with SECONDS", approxEqual(end_pos, 2.0f));

    // Test with MILLISECONDS
    motor->setTimeUnit(TimeUnit::MILLISECONDS);
    checkMotorError(*motor, "setTimeUnit(MILLISECONDS)");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    multimoveListConverted_t multiMoveListMS[multiMoveCount] = {
        {2.0f, multiMoveTime0 * 1000.0f},  // Velocity move: 2 rot/sec for 1000 ms
        {0.0f, multiMoveTime1 * 100.0f}    // Velocity move: 0 rot/sec for 100 ms (stop)
    };
    printf("Testing with MILLISECONDS...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListMS);
    checkMotorError(*motor, "multiMove (MILLISECONDS)");
    // Note: wait using the seconds-valued array (durations here are in seconds)
    multiMoveDelay(multiMoveCount, multiMoveList);

    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (MILLISECONDS)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with MILLISECONDS", approxEqual(end_pos, 2.0f));

    // Test with MINUTES
    motor->setTimeUnit(TimeUnit::MINUTES);
    checkMotorError(*motor, "setTimeUnit(MINUTES)");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    multimoveListConverted_t multiMoveListMin[multiMoveCount] = {
        {2.0f, multiMoveTime0 / 60.0f},  // Velocity move: 2 rot/sec for 1/60 minute (1 sec)
        {0.0f, multiMoveTime1 / 60.0f}   // Velocity move: 0 rot/sec for 0.1/60 minute (0.1 sec)
    };
    printf("Testing with MINUTES...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListMin);
    checkMotorError(*motor, "multiMove (MINUTES)");
    // Note: wait using the seconds-valued array (durations here are in seconds)
    multiMoveDelay(multiMoveCount, multiMoveList);

    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (MINUTES)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMove with MINUTES", approxEqual(end_pos, 2.0f));
}

// ---------------------------------------------------------------------------
// Test: multiMoveRaw with internal units
// ---------------------------------------------------------------------------
static void testMultiMoveRaw(Servomotor* motor) {
    printf("\n=== Testing multiMoveRaw with internal units ===\n");

    // Position unit for the readback assertion.
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    const uint8_t multiMoveCount = 2;
    uint32_t multiMoveTypes = 0b11;  // Both are velocity moves

    // Internal units:
    //   velocity: CONVERSION_FACTOR_ROTATIONS_PER_SECOND ~ 109951162.7776
    //   time:     CONVERSION_FACTOR_SECONDS = 31250.0
    multimoveList_t multiMoveList[multiMoveCount] = {
        {219902325, 31250},  // Velocity move: 2 rot/sec for 1 sec in internal units
        {0, 3125}            // Velocity move: 0 rot/sec for 0.1 sec in internal units
    };
    printf("Testing multiMoveRaw...\n");
    motor->multimoveRaw(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMoveRaw");
    multiMoveDelayInternalTimeInput(multiMoveCount, multiMoveList);

    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (Raw)");
    printf("Position after move: %.2f rotations\n", end_pos);
    TEST_RESULT("multiMoveRaw", approxEqual(end_pos, 2.0f));
}

// ---------------------------------------------------------------------------
// Test: complex motion sequence with mixed velocity and acceleration moves
// ---------------------------------------------------------------------------
static void testComplexMotionSequence(Servomotor* motor) {
    printf("\n=== Testing complex motion sequence with mixed moves ===\n");

    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(ROTATIONS_PER_SECOND)");
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(ROTATIONS_PER_SECOND_SQUARED)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    const uint8_t multiMoveCount = 4;
    uint32_t multiMoveTypes = 0b1001;  // 1st and 4th velocity moves, 2nd and 3rd acceleration moves

    multimoveListConverted_t multiMoveList[multiMoveCount] = {
        {2.0f, 1.0f},    // Velocity move: 2 rot/sec for 1 sec
        {2.0f, 1.0f},    // Acceleration move: 2 rot/sec^2 for 1 sec
        {-2.0f, 1.0f},   // Acceleration move: -2 rot/sec^2 for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 rot/sec for 0.1 sec (stop)
    };
    printf("Testing complex motion sequence...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (Complex)");
    multiMoveDelay(multiMoveCount, multiMoveList);

    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (Complex)");
    printf("Position after move: %.2f rotations\n", end_pos);
    // 2 (vel) + 3 (accel up) + 3 (accel down) + 0 (stop) = 8 rotations
    TEST_RESULT("Complex motion sequence", approxEqual(end_pos, 8.0f));
}

// ---------------------------------------------------------------------------
// Test: maximum number of moves (32) with velocity moves
// ---------------------------------------------------------------------------
static void testMaximumNumberOfVelocityMoves(Servomotor* motor) {
    printf("\n=== Testing maximum number of velocity moves (32) ===\n");

    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(ROTATIONS_PER_SECOND)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    const uint8_t multiMoveCount = 32;
    uint32_t multiMoveTypes = 0xFFFFFFFF;  // All velocity moves

    multimoveListConverted_t multiMoveList[multiMoveCount];
    // Alternating positive/negative velocities to wiggle back and forth.
    for (int i = 0; i < multiMoveCount - 1; i++) {
        float velocity = (i % 2 == 0) ? 1.0f : -1.0f;
        multiMoveList[i].value = velocity;
        multiMoveList[i].duration = 0.1f;
    }
    // Last move is always a stop (zero velocity).
    multiMoveList[multiMoveCount - 1].value = 0.0f;
    multiMoveList[multiMoveCount - 1].duration = 0.1f;

    printf("Testing maximum number of velocity moves...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (Max Velocity Moves)");
    multiMoveDelay(multiMoveCount, multiMoveList);

    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (Max Velocity Moves)");
    printf("Position after velocity moves: %.2f rotations\n", end_pos);
    // Only checking that the 32-move command executed without errors.
    TEST_RESULT("Maximum number of velocity moves", motor->getError() == 0);
}

// ---------------------------------------------------------------------------
// Test: maximum number of moves (32) with acceleration moves
// ---------------------------------------------------------------------------
static void testMaximumNumberOfAccelerationMoves(Servomotor* motor) {
    printf("\n=== Testing maximum number of acceleration moves (32) ===\n");

    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(ROTATIONS_PER_SECOND_SQUARED)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    const uint8_t multiMoveCount = 32;
    uint32_t multiMoveTypes = 0x00000000;  // All acceleration moves

    multimoveListConverted_t multiMoveList[multiMoveCount];
    // Each wiggle = 4 acceleration moves that bring velocity 0 -> +peak -> 0 ->
    // -peak -> 0, so the sequence always ends at zero velocity.
    for (int wiggle = 0; wiggle < 8; wiggle++) {
        int baseIndex = wiggle * 4;
        multiMoveList[baseIndex + 0].value = 5.0f;
        multiMoveList[baseIndex + 0].duration = 0.2f;
        multiMoveList[baseIndex + 1].value = -5.0f;
        multiMoveList[baseIndex + 1].duration = 0.2f;
        multiMoveList[baseIndex + 2].value = -5.0f;
        multiMoveList[baseIndex + 2].duration = 0.2f;
        multiMoveList[baseIndex + 3].value = 5.0f;
        multiMoveList[baseIndex + 3].duration = 0.2f;
    }

    printf("Testing maximum number of acceleration moves...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (Max Acceleration Moves)");
    multiMoveDelay(multiMoveCount, multiMoveList);

    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (Max Acceleration Moves)");
    printf("Position after acceleration moves: %.2f rotations\n", end_pos);
    // Only checking that the 32-move command executed without errors.
    TEST_RESULT("Maximum number of acceleration moves", motor->getError() == 0);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
void tm_multimove(void) {
    Serial.println("test_ard_multimove: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    printf("\n=== Starting multiMove Tests ===\n\n");

    // ---- Clean known state ----
    printf("Resetting system...\n");
    motor->systemReset();
    delay(1500);  // reset; motor does not respond during reset, so don't check errors

    // Configure motor units.
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // Enable MOSFETs before moving.
    printf("Enabling MOSFETs...\n");
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");

    delay(300);  // let the commutation-alignment transient settle; status updates
    getStatusResponse enabled_status = motor->getStatus();
    checkMotorError(*motor, "getStatus");
    printf("Status after enable: 0x%02X\n", enabled_status.statusFlags);
    TEST_RESULT("MOSFETs Successfully Enabled", (enabled_status.statusFlags & 0x02) == 0x02);

    // Run tests.
    testMultiMoveVelocityUnits(motor);
    testMultiMoveAccelerationUnits(motor);
    testMultiMoveTimeUnits(motor);
    testMultiMoveRaw(motor);
    testComplexMotionSequence(motor);
    testMaximumNumberOfVelocityMoves(motor);
    testMaximumNumberOfAccelerationMoves(motor);

    printf("\n=== All tests completed ===\n");

    // ---- Clean up: motor at rest, disable, reset. ----
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(1500);
}
