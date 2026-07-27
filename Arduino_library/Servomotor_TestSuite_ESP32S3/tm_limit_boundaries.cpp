#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_limit_boundaries.cpp
//
// Edge-case probe of the INCLUSIVE limit comparisons the firmware performs when
// a move is queued. The firmware validates a queued move with a STRICTLY-GREATER
// test, so a move exactly AT the configured limit is accepted and only a move
// ABOVE it is rejected. We verify both sides of that boundary for the maximum
// velocity limit and the maximum acceleration limit.
//
// Firmware ground truth (firmware/Src/motor_control.c, add_to_queue()):
//   * Velocity-type move  (MOVE_WITH_VELOCITY branch, ~line 1799):
//         movement_queue[..].velocity <<= VELOCITY_SHIFT_LEFT;   // 12
//         if(llabs(velocity) > max_velocity) fatal_error(ERROR_VEL_TOO_HIGH);
//     -> a direct velocity move that is too fast raises ERROR_VEL_TOO_HIGH (16),
//        NOT the predicted-velocity code 28 (28 is only reached from the
//        acceleration branch's predicted_final_velocity check). The spec sketch
//        offered "16 or 28"; the firmware fires 16 for a direct velocity move,
//        so we assert 16.
//   * Acceleration-type move (MOVE_WITH_ACCELERATION branch, ~line 1748):
//         movement_queue[..].acceleration <<= ACCELERATION_SHIFT_LEFT;  // 8
//         if(llabs(acceleration) > max_acceleration)
//                 fatal_error(ERROR_ACCEL_TOO_HIGH);
//     -> an over-limit acceleration raises ERROR_ACCEL_TOO_HIGH (15). This check
//        precedes the predicted-velocity and safety-zone checks in the same
//        branch, so an acceleration just over the limit trips 15 first.
//   The comparison operator is `>` in every case, so parameter == limit passes.
//
// The limit register and the queued move value are scaled identically: the
// library applies the SAME convertVelocity / convertAcceleration factor (and the
// same integer truncation) to both "Set maximum velocity"/"...acceleration" and
// the move commands (Servomotor.cpp), and the firmware applies the SAME left
// shift (VELOCITY_SHIFT_LEFT / ACCELERATION_SHIFT_LEFT) to the limit and to the
// queued value. So "set the limit to X, then move at exactly X" produces a
// queued value bit-identical to the limit -> the `>` test is false -> accepted.
//
// Error codes asserted (verified against
//   python_programs/servomotor/error_codes.json):
//     ERROR_ACCEL_TOO_HIGH == 15
//     ERROR_VEL_TOO_HIGH   == 16
//
// SAFETY:
//   * The MOSFETs are left DISABLED the whole time (boot state after reset), so
//     the shaft is never driven -> zero physical motion on the bench or the
//     39-motor rack. The motion planner still runs and drains the queue, and the
//     add_to_queue limit checks (which is what we test) run regardless of MOSFET
//     state, so disabling them costs no coverage.
//   * Every accepted sequence ENDS AT ZERO VELOCITY: the velocity test appends a
//     move-with-velocity(0) item, and the acceleration test appends an equal-and-
//     opposite deceleration, so the queue never empties at a nonzero commanded
//     velocity (no async ERROR_RUN_OUT_OF_QUEUE_ITEMS 18). Both accepted-move
//     items are queued back-to-back (within one command round-trip, << the move
//     durations) so the first item cannot drain before the stopping item lands.
//   * Commanded travel per accepted sequence is <= 0.6 rotation and peak
//     commanded velocity <= 2.0 rot/s -- within the gentle-motion guideline and
//     well under the default 2-rotation position-deviation limit (no error 45).
//   * After each DELIBERATE fatal error we systemReset + delay(1500) + drain
//     before proceeding. No Test mode (cmd 36) values are used anywhere.
//   * The motor is addressed by unique ID (tfGetMotor uses extended addressing),
//     so every frame is collision-safe on a multi-motor bus; nothing is
//     broadcast that would elicit responses from multiple devices, and no raw
//     bus frames are injected.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int FATAL_RESET_DELAY_MS  = 2000;  // wait after recovering a fatal
static const int LATCH_SETTLE_MS       = 300;   // let a queued-move fatal latch

static const uint8_t ERROR_ACCEL_TOO_HIGH = 15; // firmware code (error_codes.json)
static const uint8_t ERROR_VEL_TOO_HIGH   = 16; // firmware code (error_codes.json)

// Boundary test values. The at-limit value is set as the limit AND commanded as
// the move, so they convert to the identical internal integer (accepted). The
// over-limit value is strictly larger and clearly distinct after conversion:
//   vel:   2.00 -> 219902325 ,  2.05 -> 225399884  (wire units, truncated)
//   accel: 2.00 -> 112589    ,  2.05 -> 115404     (wire units, truncated)
static const float AT_LIMIT_VELOCITY = 2.00f;   // rot/s
static const float OVER_VELOCITY     = 2.05f;   // rot/s
static const float AT_LIMIT_ACCEL    = 2.00f;   // rot/s^2
static const float OVER_ACCEL        = 2.05f;   // rot/s^2

static const float MOVE_DUR      = 0.30f;       // s, first (accepted) move
static const float STOP_DUR      = 0.10f;       // s, stopping/return item

// Drain any stale bytes left on the RS485 line after a reset.
static void drainBus() {
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}

// systemReset + settle + drain: leaves the device in a clean, known state.
static void cleanReset(Servomotor* motor, int delayMs) {
    motor->systemReset();
    delay(delayMs);   // no response is produced during reset; do not check errors
    drainBus();
}

// Read the latched fatal error code via Get status (answered normally even in
// the fatal-error state). Returns 0xFF if the query failed at the comms layer,
// so 0xFF is distinguishable from a genuine code 0.
static uint8_t readFatalCode(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

// Poll the queue until it drains to empty (bounded ~8 s), then settle. Returns
// the last observed depth (0 on success, nonzero/0xFF if it never drained).
static uint8_t waitForIdle(Servomotor* motor) {
    uint8_t n = 0xFF;
    uint32_t start = millis();
    while (millis() - start < 8000) {
        n = motor->getNQueuedItems();
        if (motor->getError() == 0 && n == 0) break;
        delay(10);
    }
    delay(150);  // settle margin
    return n;
}

void tm_limit_boundaries(void) {
    Serial.println("test_ard_limit_boundaries: BEGIN\n");

    Servomotor* motor = tfGetMotor();
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);

    // =======================================================================
    // SECTION 1: maximum VELOCITY boundary.
    // =======================================================================
    printf("\n=== SECTION 1: max velocity boundary (limit %.2f rot/s) ===\n",
           (double)AT_LIMIT_VELOCITY);

    cleanReset(motor, NORMAL_RESET_DELAY_MS);
    uint8_t vStart = readFatalCode(motor);
    printf("Fatal code after reset: %d\n", vStart);
    TEST_RESULT("V-limit No fatal error at clean start", vStart == 0);

    // Set the ceiling to exactly the boundary value.
    motor->setMaximumVelocity(AT_LIMIT_VELOCITY);
    bool vSetOk = (motor->getError() == 0);
    TEST_RESULT("V-limit setMaximumVelocity(2.0) no comms error", vSetOk);

    // --- At the limit: a move at exactly 2.0 rot/s must be ACCEPTED. ---
    // Queue the constant-velocity item then, back-to-back, the zero-velocity
    // stopping item so the sequence ends at rest.
    motor->moveWithVelocity(AT_LIMIT_VELOCITY, MOVE_DUR);
    bool vMoveComms = (motor->getError() == 0);
    motor->moveWithVelocity(0.0f, STOP_DUR);
    vMoveComms = vMoveComms && (motor->getError() == 0);

    delay(LATCH_SETTLE_MS);
    uint8_t vAtLimitFatal = readFatalCode(motor);
    printf("At-limit velocity move: commsOk=%d fatalCode=%d (expect 0)\n",
           (int)vMoveComms, (int)vAtLimitFatal);
    // ground truth: llabs(velocity) > max_velocity is FALSE at equality -> accepted
    TEST_RESULT("V-limit Move at exactly 2.0 rot/s accepted (no fatal)",
                vMoveComms && vAtLimitFatal == 0);

    uint8_t vDrained = waitForIdle(motor);
    printf("At-limit velocity sequence drained to depth %d (expect 0)\n", vDrained);
    TEST_RESULT("V-limit At-limit sequence ran and drained to empty queue",
                vDrained == 0);
    uint8_t vPostRun = readFatalCode(motor);
    TEST_RESULT("V-limit Still no fatal after at-limit sequence ran",
                vPostRun == 0);

    // --- Over the limit: a move at 2.05 rot/s must FATAL with code 16. ---
    motor->moveWithVelocity(OVER_VELOCITY, MOVE_DUR);
    delay(LATCH_SETTLE_MS);
    uint8_t vOverFatal = readFatalCode(motor);
    printf("Over-limit velocity move (2.05): fatalCode=%d (expect %d)\n",
           (int)vOverFatal, (int)ERROR_VEL_TOO_HIGH);
    TEST_RESULT("V-limit Over-limit move (2.05) latched a fatal error",
                vOverFatal != 0 && vOverFatal != 0xFF);
    // ground truth: MOVE_WITH_VELOCITY over-limit -> ERROR_VEL_TOO_HIGH (16)
    TEST_RESULT("V-limit Over-limit fatal is ERROR_VEL_TOO_HIGH (16)",
                vOverFatal == ERROR_VEL_TOO_HIGH);

    // Recover from the deliberate fatal before proceeding.
    cleanReset(motor, FATAL_RESET_DELAY_MS);
    uint8_t vRecovered = readFatalCode(motor);
    printf("Fatal code after recovery reset: %d (expect 0)\n", vRecovered);
    TEST_RESULT("V-limit Device recovered (no fatal) after reset",
                vRecovered == 0);

    // =======================================================================
    // SECTION 2: maximum ACCELERATION boundary.
    // =======================================================================
    printf("\n=== SECTION 2: max acceleration boundary (limit %.2f rot/s^2) ===\n",
           (double)AT_LIMIT_ACCEL);

    // Fresh reset restores boot-default limits, then lower the accel ceiling.
    cleanReset(motor, NORMAL_RESET_DELAY_MS);
    uint8_t aStart = readFatalCode(motor);
    printf("Fatal code after reset: %d\n", aStart);
    TEST_RESULT("A-limit No fatal error at clean start", aStart == 0);

    motor->setMaximumAcceleration(AT_LIMIT_ACCEL);
    bool aSetOk = (motor->getError() == 0);
    TEST_RESULT("A-limit setMaximumAcceleration(2.0) no comms error", aSetOk);

    // --- At the limit: an accel move at exactly 2.0 rot/s^2 must be ACCEPTED. ---
    // Accelerate then decelerate by the equal and opposite acceleration for the
    // same duration -> ends at exactly zero velocity (peak ~0.6 rot/s, ~0.18 rot).
    motor->moveWithAcceleration(AT_LIMIT_ACCEL, MOVE_DUR);
    bool aMoveComms = (motor->getError() == 0);
    motor->moveWithAcceleration(-AT_LIMIT_ACCEL, MOVE_DUR);
    aMoveComms = aMoveComms && (motor->getError() == 0);

    delay(LATCH_SETTLE_MS);
    uint8_t aAtLimitFatal = readFatalCode(motor);
    printf("At-limit accel move: commsOk=%d fatalCode=%d (expect 0)\n",
           (int)aMoveComms, (int)aAtLimitFatal);
    // ground truth: llabs(acceleration) > max_acceleration is FALSE at equality
    TEST_RESULT("A-limit Move at exactly 2.0 rot/s^2 accepted (no fatal)",
                aMoveComms && aAtLimitFatal == 0);

    uint8_t aDrained = waitForIdle(motor);
    printf("At-limit accel sequence drained to depth %d (expect 0)\n", aDrained);
    TEST_RESULT("A-limit At-limit sequence ran and drained to empty queue",
                aDrained == 0);
    uint8_t aPostRun = readFatalCode(motor);
    TEST_RESULT("A-limit Still no fatal after at-limit sequence ran",
                aPostRun == 0);

    // --- Over the limit: an accel move at 2.05 rot/s^2 must FATAL with code 15. ---
    motor->moveWithAcceleration(OVER_ACCEL, MOVE_DUR);
    delay(LATCH_SETTLE_MS);
    uint8_t aOverFatal = readFatalCode(motor);
    printf("Over-limit accel move (2.05): fatalCode=%d (expect %d)\n",
           (int)aOverFatal, (int)ERROR_ACCEL_TOO_HIGH);
    TEST_RESULT("A-limit Over-limit move (2.05) latched a fatal error",
                aOverFatal != 0 && aOverFatal != 0xFF);
    // ground truth: MOVE_WITH_ACCELERATION over-limit -> ERROR_ACCEL_TOO_HIGH (15)
    TEST_RESULT("A-limit Over-limit fatal is ERROR_ACCEL_TOO_HIGH (15)",
                aOverFatal == ERROR_ACCEL_TOO_HIGH);

    // Recover from the deliberate fatal.
    cleanReset(motor, FATAL_RESET_DELAY_MS);
    uint8_t aRecovered = readFatalCode(motor);
    printf("Fatal code after recovery reset: %d (expect 0)\n", aRecovered);
    TEST_RESULT("A-limit Device recovered (no fatal) after reset",
                aRecovered == 0);

    // -----------------------------------------------------------------------
    // Restore boot defaults and leave the device clean and at rest.
    // -----------------------------------------------------------------------
    cleanReset(motor, NORMAL_RESET_DELAY_MS);
}
