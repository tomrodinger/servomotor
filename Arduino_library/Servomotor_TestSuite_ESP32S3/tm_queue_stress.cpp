#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_queue_stress.cpp
//
// Arduino-library on-device stress test for the MOTION QUEUE:
//   * cmd 2   Trapezoid move      (queues up to 3 items: accel/coast/decel)
//   * cmd 11  Get n queued items  (reports the current queue depth, 0..32)
//   * Get status                  (reports the latched fatal error code)
//
// Two phases:
//   PART A  - Well-behaved queueing. Send 30 tiny trapezoid moves, pacing so
//             the queue never overflows: the depth RISES above zero while
//             moves pile up, every move is accepted (no fatal error), and the
//             queue eventually DRAINS back to 0 as the moves execute.
//   PART B  - Deliberate overflow. Blast tiny moves with no pacing until the
//             firmware latches the fatal error ERROR_QUEUE_IS_FULL. The motion
//             queue holds a maximum of 32 items (MOVEMENT_QUEUE_SIZE); adding
//             one more calls fatal_error(ERROR_QUEUE_IS_FULL) in
//             firmware/Src/motor_control.c. We then recover with systemReset
//             and verify the queue is empty and the device is clean/responsive.
//
// Error code asserted (verified against
//   python_programs/servomotor/error_codes.json):
//       ERROR_QUEUE_IS_FULL == 17   (fatal, recoverable via systemReset)
//
// SAFETY:
//   * All moves are tiny relative trapezoids (0.01 rotation over 0.2 s). Each
//     trapezoid ends at REST by design, so the queue can be abandoned/reset at
//     any point without leaving a nonzero commanded velocity (no async
//     ERROR_RUN_OUT_OF_QUEUE_ITEMS). Cumulative Part-A displacement is only
//     ~0.3 rotation, all under the 1-rotation gentle-motion guideline.
//   * No test-mode (cmd 36) values are used anywhere here.
//   * Overfill is bounded by a hard iteration cap; the device is reset and
//     verified clean at the end.
//
// Runs against a REAL motor addressed by unique-ID (extended addressing), so
// every method is called without a uniqueId argument. Collision-safe on a
// single-motor bench or the 39-motor rack.
// ---------------------------------------------------------------------------

static const int     NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int     FATAL_RESET_DELAY_MS  = 2000;  // wait after recovering from a fatal error

static const uint8_t ERROR_QUEUE_IS_FULL   = 17;    // firmware code (error_codes.json)
static const uint8_t QUEUE_MAX             = 32;    // MOVEMENT_QUEUE_SIZE
static const uint8_t ROOM_THRESHOLD        = 28;    // only add a (<=3-item) move when depth <= this

static const int     N_GOOD_MOVES          = 30;    // Part A: tiny moves that must all be accepted
static const float   TINY_ROT              = 0.01f; // rotations per move
static const float   TINY_DUR              = 0.2f;  // seconds per move
static const int     OVERFILL_MAX_MOVES    = 120;   // Part B: hard cap on overflow blast

// Read the device's latched fatal error code via Get status (answered normally
// even in the fatal-error state). Returns 0xFF if the query failed at the comms
// layer so the caller can distinguish "no info" from "code 0".
static uint8_t readFatalErrorCode(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

// Read the current queue depth; returns 0xFF on a comms error.
static uint8_t readQueueDepth(Servomotor* motor) {
    uint8_t n = motor->getNQueuedItems();
    if (motor->getError() != 0) {
        printf("  (getNQueuedItems comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return n;
}

// Verify a ping payload round-trips unchanged (device is alive and answering).
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 7 + 1);
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  ping comms error %d\n", motor->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

// Poll the queue until it drains to empty (up to ~15 s), then settle.
static uint8_t waitForIdle(Servomotor* motor) {
    uint8_t n = 0xFF;
    for (int i = 0; i < 1500; i++) {
        n = motor->getNQueuedItems();
        if (motor->getError() == 0 && n == 0) break;
        delay(10);
    }
    delay(150);  // settle margin
    return n;
}

void tm_queue_stress(void) {
    Serial.println("test_ard_queue_stress: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no response during reset; don't check errors

    uint8_t startFatal = readFatalErrorCode(motor);
    printf("Fatal error code after reset: %d\n", startFatal);
    TEST_RESULT("No fatal error after reset", startFatal == 0);

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle

    // =======================================================================
    // PART A: well-behaved queueing. 30 tiny moves, all accepted, depth rises
    // then drains to 0. Pace so the queue never overflows: only add a move when
    // the depth leaves room for its (up to 3) items.
    // =======================================================================
    printf("\n--- PART A: queue %d tiny moves with pacing ---\n", N_GOOD_MOVES);

    bool allAccepted   = true;   // no fatal error latched while queueing
    uint8_t maxDepth   = 0;      // proof the queue rose above zero
    bool depthReadsOk  = true;   // all depth reads succeeded at the comms layer

    for (int i = 0; i < N_GOOD_MOVES; i++) {
        // Wait until there is room for a <=3-item move (depth <= ROOM_THRESHOLD).
        uint8_t depth = 0xFF;
        for (int w = 0; w < 500; w++) {
            depth = motor->getNQueuedItems();
            if (motor->getError() != 0) { depthReadsOk = false; depth = 0xFF; break; }
            if (depth <= ROOM_THRESHOLD) break;
            delay(10);
        }
        if (depth != 0xFF && depth > maxDepth) maxDepth = depth;

        motor->trapezoidMove(TINY_ROT, TINY_DUR);
        if (motor->getError() != 0) {
            printf("  move %d: comms error %d\n", i, motor->getError());
        }
        // A latched fatal error here means a move was rejected -> not "all accepted".
        uint8_t f = readFatalErrorCode(motor);
        if (f != 0) {
            printf("  move %d: unexpected fatal code %d\n", i, f);
            allAccepted = false;
            break;
        }
    }

    // Capture the peak depth once more right after the burst (may already be draining).
    uint8_t depthNow = readQueueDepth(motor);
    if (depthNow != 0xFF && depthNow > maxDepth) maxDepth = depthNow;
    printf("Part A: all accepted=%d, max observed queue depth=%d\n", allAccepted, maxDepth);

    TEST_RESULT("Q-A All tiny moves accepted (no fatal error)", allAccepted);
    TEST_RESULT("Q-A Queue depth rose above zero while queueing", maxDepth > 0);
    TEST_RESULT("Q-A Queue depth reads succeeded", depthReadsOk);

    // Let the queue drain completely.
    uint8_t drained = waitForIdle(motor);
    printf("Part A: queue depth after draining = %d (expect 0)\n", drained);
    TEST_RESULT("Q-A Queue drains back to zero", drained == 0);

    uint8_t fatalAfterA = readFatalErrorCode(motor);
    TEST_RESULT("Q-A No fatal error after 30-move sequence", fatalAfterA == 0);

    // =======================================================================
    // PART B: deliberate overflow. Blast tiny moves with no pacing until the
    // firmware latches ERROR_QUEUE_IS_FULL (17). Detect via Get status after
    // each move; stop at the first nonzero fatal code.
    // =======================================================================
    printf("\n--- PART B: overfill the queue to force ERROR_QUEUE_IS_FULL ---\n");

    uint8_t preDepth = readQueueDepth(motor);
    printf("Queue depth before overfill = %d (expect 0)\n", preDepth);
    TEST_RESULT("Q-B Queue empty before overfill", preDepth == 0);

    uint8_t overfillFatal = 0;   // fatal code observed during overfill
    int     movesUntilFull = 0;  // how many moves it took to trip the error
    for (int i = 0; i < OVERFILL_MAX_MOVES; i++) {
        motor->trapezoidMove(TINY_ROT, TINY_DUR);  // ignore comms errors here on purpose
        movesUntilFull++;
        uint8_t f = readFatalErrorCode(motor);
        if (f != 0 && f != 0xFF) { overfillFatal = f; break; }
    }
    printf("Overfill: latched fatal code %d after ~%d move sends\n",
           overfillFatal, movesUntilFull);

    TEST_RESULT("Q-B Overfill latched a fatal error", overfillFatal != 0);
    TEST_RESULT("Q-B Fatal error is ERROR_QUEUE_IS_FULL (17)",
                overfillFatal == ERROR_QUEUE_IS_FULL);

    // -----------------------------------------------------------------------
    // Recover and verify the device is clean.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(FATAL_RESET_DELAY_MS);  // avoid pinning the bootloader window

    uint8_t recoveredFatal = readFatalErrorCode(motor);
    printf("Fatal code after recovery reset = %d (expect 0)\n", recoveredFatal);
    TEST_RESULT("Q-B Recovered (no fatal error) after systemReset", recoveredFatal == 0);

    uint8_t recoveredDepth = readQueueDepth(motor);
    printf("Queue depth after recovery = %d (expect 0)\n", recoveredDepth);
    TEST_RESULT("Q-B Queue empty after recovery", recoveredDepth == 0);

    TEST_RESULT("Q-B Device responsive (ping) after recovery", pingRoundTrips(motor));

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
}
