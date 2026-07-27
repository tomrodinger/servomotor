#include "tf_framework.h"

// Per-command Arduino test for basic motion commands:
//   cmd 2  Trapezoid move   (RELATIVE move, trapezoidal profile)
//   cmd 4  Go to position   (ABSOLUTE move)
//   cmd 13 Zero position    (redefine current position as the new origin)
//
// Mirrors the Python tests:
//   test_trapezoid_move.py, test_go_to_position.py, test_zero_position.py
//
// One motor on the bus, addressed by unique-ID (extended addressing) via the
// TestModeConvenienceWrapper, so all methods are called WITHOUT a uniqueId arg.

static const int64_t COUNTS_PER_ROTATION = COUNTS_PER_REVOLUTION;  // 3276800
static const int64_t TOLERANCE_COUNTS = 200;  // generous host<->hardware tolerance

// Wait until the motion queue is empty (all queued moves consumed), then settle.
static void waitForIdle(Servomotor* motor) {
    // Poll the queue for up to ~10 s; each queued item is a real motion segment.
    for (int i = 0; i < 1000; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (n == 0) break;
        delay(10);
    }
    delay(150);  // small settle margin
}

// Read the current position in raw encoder counts (unit-independent ground truth).
static int64_t getCounts(Servomotor* motor) {
    int64_t raw = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw");
    return raw;
}

void tm_motion_basic(void) {
    Serial.println("test_ard_motion_basic: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean known state ----
    motor->systemReset();
    delay(1500);  // reset; motor does not respond during reset, so don't check errors

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle before zeroing

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    int64_t start = getCounts(motor);
    printf("Start position after enable+zero: %lld counts\n", (long long)start);
    TEST_RESULT("Start Position Is Zero", llabs(start) <= TOLERANCE_COUNTS);

    // =====================================================================
    // PART A: Trapezoid move (cmd 2) is RELATIVE and accumulates.
    // Mirrors test_trapezoid_move.py.
    // =====================================================================
    const int64_t D = COUNTS_PER_ROTATION / 4;  // 1/4 turn per relative move
    const float duration = 1.0f;

    // [A1] Relative +D over `duration`, timed to prove the duration is honoured.
    printf("\n=== A1: Relative +%lld counts over %.1fs ===\n", (long long)D, duration);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    unsigned long t0 = millis();
    motor->trapezoidMove((float)D, duration);
    checkMotorError(*motor, "trapezoidMove A1");
    waitForIdle(motor);
    float elapsed = (millis() - t0) / 1000.0f;
    int64_t posA1 = getCounts(motor);
    printf("Ended at %lld (expect ~%lld), elapsed %.2fs (expect ~%.1fs)\n",
           (long long)posA1, (long long)D, elapsed, duration);
    TEST_RESULT("A1 Trapezoid Relative +1/4 Turn Lands Correctly",
                llabs(posA1 - D) <= TOLERANCE_COUNTS);
    TEST_RESULT("A1 Trapezoid Move Honours Duration Argument",
                (elapsed >= duration * 0.6f) && (elapsed <= duration + 1.0f));

    // [A2] Another relative +D must accumulate to ~2D (proves RELATIVE semantics).
    printf("\n=== A2: Another relative +%lld must accumulate to ~%lld ===\n",
           (long long)D, (long long)(2 * D));
    motor->trapezoidMove((float)D, duration);
    checkMotorError(*motor, "trapezoidMove A2");
    waitForIdle(motor);
    int64_t posA2 = getCounts(motor);
    printf("Ended at %lld (expect ~%lld)\n", (long long)posA2, (long long)(2 * D));
    TEST_RESULT("A2 Trapezoid Moves Accumulate (Relative)",
                llabs(posA2 - 2 * D) <= TOLERANCE_COUNTS);

    // [A3] Relative -2D must return to the origin.
    printf("\n=== A3: Relative -%lld must return to origin ===\n", (long long)(2 * D));
    motor->trapezoidMove(-2.0f * (float)D, duration);
    checkMotorError(*motor, "trapezoidMove A3");
    waitForIdle(motor);
    int64_t posA3 = getCounts(motor);
    printf("Ended at %lld (expect ~0)\n", (long long)posA3);
    TEST_RESULT("A3 Trapezoid Return To Origin",
                llabs(posA3) <= TOLERANCE_COUNTS);

    // [A4] Same 1/4-turn displacement expressed in shaft_rotations lands identically.
    printf("\n=== A4: Same displacement in shaft_rotations lands identically ===\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->trapezoidMove(0.25f, duration);  // 1/4 rotation == D counts
    checkMotorError(*motor, "trapezoidMove A4");
    waitForIdle(motor);
    int64_t posA4 = getCounts(motor);
    printf("Ended at %lld (expect ~%lld)\n", (long long)posA4, (long long)D);
    TEST_RESULT("A4 Trapezoid Displacement Unit-Independent",
                llabs(posA4 - D) <= TOLERANCE_COUNTS);

    // Return to origin before the absolute-move part.
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->trapezoidMove(-(float)D, duration);
    checkMotorError(*motor, "trapezoidMove return");
    waitForIdle(motor);

    // Re-zero so the absolute-position part starts from a clean origin.
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition before Part B");
    int64_t rezero = getCounts(motor);
    printf("Re-zeroed to %lld counts before Part B\n", (long long)rezero);

    // =====================================================================
    // PART B: Go to position (cmd 4) is ABSOLUTE. Verify one full rotation
    // expressed across several position units, each returning to zero after.
    // Mirrors test_go_to_position.py.
    // =====================================================================
    const int64_t ONE_ROTATION = COUNTS_PER_ROTATION;  // 3276800

    // [B1] shaft_rotations: go to 1.0 rotation.
    printf("\n=== B1: goToPosition 1.0 shaft_rotations (absolute) ===\n");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->goToPosition(1.0f, 2.0f);
    checkMotorError(*motor, "goToPosition B1");
    waitForIdle(motor);
    int64_t posB1 = getCounts(motor);
    printf("Ended at %lld (expect ~%lld)\n", (long long)posB1, (long long)ONE_ROTATION);
    TEST_RESULT("B1 GoToPosition 1 Rotation (shaft_rotations)",
                llabs(posB1 - ONE_ROTATION) <= TOLERANCE_COUNTS);

    motor->goToPosition(0.0f, 1.0f);  // absolute return to 0
    checkMotorError(*motor, "goToPosition B1 return");
    waitForIdle(motor);
    int64_t posB1z = getCounts(motor);
    printf("Returned to %lld (expect ~0)\n", (long long)posB1z);
    TEST_RESULT("B1 GoToPosition Absolute Zero (shaft_rotations)",
                llabs(posB1z) <= TOLERANCE_COUNTS);

    // [B2] degrees: go to 360 degrees == 1 rotation.
    printf("\n=== B2: goToPosition 360 degrees (absolute) ===\n");
    motor->setPositionUnit(PositionUnit::DEGREES);
    motor->goToPosition(360.0f, 2.0f);
    checkMotorError(*motor, "goToPosition B2");
    waitForIdle(motor);
    int64_t posB2 = getCounts(motor);
    printf("Ended at %lld (expect ~%lld)\n", (long long)posB2, (long long)ONE_ROTATION);
    TEST_RESULT("B2 GoToPosition 360 Degrees == 1 Rotation",
                llabs(posB2 - ONE_ROTATION) <= TOLERANCE_COUNTS);

    motor->goToPosition(0.0f, 1.0f);
    checkMotorError(*motor, "goToPosition B2 return");
    waitForIdle(motor);
    int64_t posB2z = getCounts(motor);
    printf("Returned to %lld (expect ~0)\n", (long long)posB2z);
    TEST_RESULT("B2 GoToPosition Absolute Zero (degrees)",
                llabs(posB2z) <= TOLERANCE_COUNTS);

    // [B3] radians: go to 2*pi radians == 1 rotation.
    printf("\n=== B3: goToPosition 2*pi radians (absolute) ===\n");
    motor->setPositionUnit(PositionUnit::RADIANS);
    motor->goToPosition((float)(2.0 * M_PI), 2.0f);
    checkMotorError(*motor, "goToPosition B3");
    waitForIdle(motor);
    int64_t posB3 = getCounts(motor);
    printf("Ended at %lld (expect ~%lld)\n", (long long)posB3, (long long)ONE_ROTATION);
    TEST_RESULT("B3 GoToPosition 2pi Radians == 1 Rotation",
                llabs(posB3 - ONE_ROTATION) <= TOLERANCE_COUNTS);

    motor->goToPosition(0.0f, 1.0f);
    checkMotorError(*motor, "goToPosition B3 return");
    waitForIdle(motor);
    int64_t posB3z = getCounts(motor);
    printf("Returned to %lld (expect ~0)\n", (long long)posB3z);
    TEST_RESULT("B3 GoToPosition Absolute Zero (radians)",
                llabs(posB3z) <= TOLERANCE_COUNTS);

    // [B4] encoder_counts: go to exactly one rotation in counts.
    printf("\n=== B4: goToPosition %lld encoder_counts (absolute) ===\n",
           (long long)ONE_ROTATION);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->goToPosition((float)ONE_ROTATION, 2.0f);
    checkMotorError(*motor, "goToPosition B4");
    waitForIdle(motor);
    int64_t posB4 = getCounts(motor);
    printf("Ended at %lld (expect ~%lld)\n", (long long)posB4, (long long)ONE_ROTATION);
    TEST_RESULT("B4 GoToPosition Absolute (encoder_counts)",
                llabs(posB4 - ONE_ROTATION) <= TOLERANCE_COUNTS);

    // [B5] Absolute semantics: issuing the SAME absolute target again must NOT
    // move further (unlike a relative move, which would double it).
    printf("\n=== B5: repeating same absolute target must not advance ===\n");
    motor->goToPosition((float)ONE_ROTATION, 1.0f);
    checkMotorError(*motor, "goToPosition B5");
    waitForIdle(motor);
    int64_t posB5 = getCounts(motor);
    printf("Ended at %lld (expect still ~%lld, NOT ~%lld)\n",
           (long long)posB5, (long long)ONE_ROTATION, (long long)(2 * ONE_ROTATION));
    TEST_RESULT("B5 GoToPosition Is Absolute (repeat does not advance)",
                llabs(posB5 - ONE_ROTATION) <= TOLERANCE_COUNTS);

    motor->goToPosition(0.0f, 1.0f);
    checkMotorError(*motor, "goToPosition B5 return");
    waitForIdle(motor);

    // =====================================================================
    // PART C: Zero position (cmd 13) redefines the origin without moving.
    // Mirrors test_zero_position.py.
    // =====================================================================
    const int64_t HALF_TURN = COUNTS_PER_ROTATION / 2;

    printf("\n=== C1: move to 1/2 turn, then zero the origin there ===\n");
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->trapezoidMove((float)HALF_TURN, 1.0f);
    checkMotorError(*motor, "trapezoidMove C1");
    waitForIdle(motor);
    int64_t posC_before = getCounts(motor);
    printf("Position before zeroing: %lld (expect ~%lld)\n",
           (long long)posC_before, (long long)HALF_TURN);
    TEST_RESULT("C1 Setup Move To Half Turn",
                llabs(posC_before - HALF_TURN) <= TOLERANCE_COUNTS);

    printf("Calling zeroPosition...\n");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition C");
    int64_t posC_after = getCounts(motor);
    printf("Position immediately after zeroing: %lld (expect ~0)\n", (long long)posC_after);
    TEST_RESULT("C2 Zero Position Redefines Origin To 0",
                llabs(posC_after) <= TOLERANCE_COUNTS);

    // The shaft must not physically move due to zeroing: read again after a wait.
    delay(200);
    int64_t posC_again = getCounts(motor);
    printf("Position again (must stay ~0, no induced motion): %lld\n", (long long)posC_again);
    TEST_RESULT("C3 Zeroing Does Not Move The Shaft",
                llabs(posC_again) <= TOLERANCE_COUNTS);

    // Prove the origin really shifted: a relative move from the new zero.
    const int64_t QUARTER = COUNTS_PER_ROTATION / 4;
    printf("Moving +%lld counts from the new origin...\n", (long long)QUARTER);
    motor->trapezoidMove((float)QUARTER, 1.0f);
    checkMotorError(*motor, "trapezoidMove C4");
    waitForIdle(motor);
    int64_t posC_quarter = getCounts(motor);
    printf("Position after +1/4 turn from new origin: %lld (expect ~%lld)\n",
           (long long)posC_quarter, (long long)QUARTER);
    TEST_RESULT("C4 New Origin Is Real (relative move lands at +1/4 turn)",
                llabs(posC_quarter - QUARTER) <= TOLERANCE_COUNTS);

    // ---- Clean up: return to rest at the new origin, disable, reset. ----
    printf("\nReturning to new origin and cleaning up...\n");
    motor->trapezoidMove(-(float)QUARTER, 1.0f);  // trapezoid ends at rest
    checkMotorError(*motor, "trapezoidMove cleanup");
    waitForIdle(motor);
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(1500);
}
