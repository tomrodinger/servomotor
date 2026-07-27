#include "tf_framework.h"

// On-device test for fast, short velocity moves queued back-to-back.
//
// Mirrors the Python test test_fast_short_move_with_velocity.py, reduced to a
// deterministic assertion-based form:
//   - enable MOSFETs, zero the position
//   - queue 20 short velocity moves (0.5 rot/s for 0.1 s each) plus a final
//     zero-velocity item -> total commanded displacement = 20 * 0.05 = 1.0 rot
//   - wait for the queue to drain, assert final position ~1.0 rot and no errors
//   - repeat in the reverse direction, ending back near 0
//
// Exercises: moveWithVelocity (cmd 30), getNQueuedItems (cmd 21),
//            getPositionRaw, enableMosfets, zeroPosition.
//
// One motor on the bus, unique-ID addressed via tfGetMotor(), so every call is
// made without a uniqueId argument (collision-safe on the shared rack).

static const int64_t COUNTS_PER_ROTATION = COUNTS_PER_REVOLUTION;  // 3276800

// Motion parameters (match the Python test's per-move quantities).
static const int   N_MOVES         = 20;
static const float MOVE_VELOCITY    = 0.5f;   // rot/s
static const float MOVE_DURATION    = 0.1f;   // s   -> 0.05 rot per move
static const float FINAL_STOP_DUR   = 0.1f;   // s   final zero-velocity item

// Total commanded displacement per direction = 20 * 0.5 * 0.1 = 1.0 rotation.
static const int64_t EXPECTED_DISPLACEMENT = COUNTS_PER_ROTATION;         // 1.0 rot
// Spec tolerance: +/- 0.1 rotation.
static const int64_t TOLERANCE_COUNTS = COUNTS_PER_ROTATION / 10;         // 327680

// Wait until the motion queue is empty, then settle. The 21 queued items each
// run ~0.1 s (~2.1 s total), so poll generously.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 1000; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (n == 0) break;
        delay(20);
    }
    delay(200);  // settle margin after the last segment finishes
}

// Read the current position in raw encoder counts (unit-independent).
static int64_t getCounts(Servomotor* motor) {
    int64_t raw = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw");
    return raw;
}

// Queue N short velocity moves in one direction plus a final zero-velocity stop.
// Every sequence therefore ends at zero velocity (avoids async fatal error 18).
static void queueBurst(Servomotor* motor, float velocity) {
    for (int i = 0; i < N_MOVES; i++) {
        motor->moveWithVelocity(velocity, MOVE_DURATION);
        checkMotorError(*motor, "moveWithVelocity");
    }
    // Final zero-velocity item guarantees the burst ends at rest.
    motor->moveWithVelocity(0.0f, FINAL_STOP_DUR);
    checkMotorError(*motor, "moveWithVelocity stop");
}

void tm_fast_short_moves(void) {
    Serial.println("tm_fast_short_moves: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean known state ----
    motor->systemReset();
    delay(1500);  // motor does not respond during reset; don't check errors here

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle before zeroing

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    int64_t start = getCounts(motor);
    printf("Start position after enable+zero: %lld counts\n", (long long)start);
    TEST_RESULT("Start Position Is Zero", llabs(start) <= TOLERANCE_COUNTS);

    // =====================================================================
    // PART A: 20 short forward velocity moves -> ~+1.0 rotation.
    // =====================================================================
    printf("\n=== A: %d moves of %.2f rot/s x %.2fs (forward) ===\n",
           N_MOVES, MOVE_VELOCITY, MOVE_DURATION);
    queueBurst(motor, MOVE_VELOCITY);
    waitForIdle(motor);

    int64_t posFwd = getCounts(motor);
    printf("Position after forward burst: %lld (expect ~%lld, tol %lld)\n",
           (long long)posFwd, (long long)EXPECTED_DISPLACEMENT,
           (long long)TOLERANCE_COUNTS);
    TEST_RESULT("Forward Burst Lands At ~1.0 Rotation",
                llabs(posFwd - EXPECTED_DISPLACEMENT) <= TOLERANCE_COUNTS);

    // No queued items should remain, and the burst must not have raised an error.
    uint8_t nqFwd = motor->getNQueuedItems();
    checkMotorError(*motor, "getNQueuedItems fwd");
    printf("Queued items after forward drain: %u\n", (unsigned)nqFwd);
    TEST_RESULT("Forward Queue Fully Drained", nqFwd == 0);
    TEST_RESULT("Forward Burst No Fatal Error", motor->getError() == 0);

    // =====================================================================
    // PART B: 20 short reverse velocity moves -> back to ~0.
    // =====================================================================
    printf("\n=== B: %d moves of %.2f rot/s x %.2fs (reverse) ===\n",
           N_MOVES, -MOVE_VELOCITY, MOVE_DURATION);
    queueBurst(motor, -MOVE_VELOCITY);
    waitForIdle(motor);

    int64_t posRev = getCounts(motor);
    printf("Position after reverse burst: %lld (expect ~0, tol %lld)\n",
           (long long)posRev, (long long)TOLERANCE_COUNTS);
    TEST_RESULT("Reverse Burst Returns To ~0",
                llabs(posRev) <= TOLERANCE_COUNTS);

    uint8_t nqRev = motor->getNQueuedItems();
    checkMotorError(*motor, "getNQueuedItems rev");
    printf("Queued items after reverse drain: %u\n", (unsigned)nqRev);
    TEST_RESULT("Reverse Queue Fully Drained", nqRev == 0);
    TEST_RESULT("Reverse Burst No Fatal Error", motor->getError() == 0);

    // ---- Clean up: motor is already at rest (final zero-velocity item). ----
    printf("\nCleaning up...\n");
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(1500);
}
