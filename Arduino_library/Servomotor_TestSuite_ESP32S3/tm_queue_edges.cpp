#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_queue_edges.cpp
//
// Arduino-library on-device EDGE-CASE test for the MOTION QUEUE boundaries.
// Goal: probe the exact full/overflow boundary, the emergency-stop clear path,
// and how the firmware behaves toward a new move while a fatal error is latched.
//
// FIRMWARE GROUND TRUTH (verified in source, cited at each assertion):
//   * The motion queue holds MOVEMENT_QUEUE_SIZE == 32 items
//     (firmware/Src/motor_control.h:31).
//   * add_to_queue() accepts an item only while n_items_in_queue < 32; the 33rd
//     item calls fatal_error(ERROR_QUEUE_IS_FULL) (motor_control.c:1743/1822).
//   * ERROR_QUEUE_IS_FULL == 17 (python_programs/servomotor/error_codes.json).
//   * A single "Trapezoid move" queues UP TO 3 items (accel/coast/decel) via
//     add_trapezoid_move_to_queue() (motor_control.c:1938-1952), so it CANNOT
//     be used to reach a clean depth of exactly 32. To fill the queue with a
//     KNOWN, one-item-per-call primitive we use "Move with velocity" at
//     velocity 0: MOVE_WITH_VELOCITY_COMMAND calls add_to_queue() exactly once
//     (main.c:608-621, motor_control.c:1793-1811). Velocity 0 means the shaft
//     never moves and each item leaves the runtime velocity at 0, so when the
//     queue drains, current_velocity_i64 == 0 and NO ERROR_RUN_OUT_OF_QUEUE_ITEMS
//     (18) is raised (motor_control.c:2057-2083). This is the SPEC's "32 tiny
//     moves" realized with the firmware-correct 1-item primitive (see notes).
//   * The queue drains continuously in the TIM16 control ISR
//     (motor_control.c:3087 -> motor_movement_calculations -> handle_queued_movements),
//     regardless of MOSFET state, so fills are paced against long item durations.
//   * emergency_stop() (motor_control.c:3323) calls disable_mosfets() +
//     clear_the_queue_and_stop_no_disable_interrupt() (n_items_in_queue = 0). It
//     does NOT raise a fatal error.
//   * STATUS_MOSFETS_ENABLED_FLAG_BIT == 1 (common_source_files/device_status.h:17)
//     -> status-flags mask 0x0002.
//   * While a fatal error is latched, the firmware is spinning inside
//     fatal_error()'s while(1) and services packets via handle_packet()
//     (common_source_files/error_handling.c:62-112). ONLY System reset and Get
//     status are handled specially; EVERY other command (incl. Trapezoid move)
//     falls into the default case and is answered with an ERROR packet carrying
//     the latched fatal code -- the command is NOT executed. The Arduino library
//     surfaces that as a POSITIVE getError() equal to the latched code
//     (Communication.cpp:404-413,626-628).
//   * Test mode value V (14..73) injects fatal_error(V-14) (main.c:909-910);
//     testMode(17) -> fatal code 3 (ERROR_FLASH_WRITE_FAIL). Non-motion.
//
// SAFETY:
//   * Parts A/B/D queue only velocity-0 moves with MOSFETs OFF: zero current,
//     zero motion, sequences end at rest. Part C enables MOSFETs to prove
//     emergencyStop turns them off; still only velocity-0 (holding) moves, and
//     the deep queue is emergency-stopped, never allowed to run a real profile.
//   * The Part-D trapezoidMove is tiny (0.01 rot / 0.3 s) AND is issued while
//     the device is in the fatal state, so it is rejected, never executed.
//   * After every deliberately-induced fatal (Part B overflow, Part D
//     injection) we systemReset + delay + drain before proceeding.
//   * NO test-mode values 10..13 are ever sent. All addressing is unique-ID
//     (tfGetMotor), collision-safe on the 39-motor rack; nothing is broadcast.
//     No raw Serial1 frames are written.
// ---------------------------------------------------------------------------

static const int      NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int      FATAL_RESET_DELAY_MS  = 2000;  // wait after recovering from a fatal error (avoid bootloader window)
static const int      INJECT_SETTLE_MS      = 300;   // let a fatal state latch

static const uint8_t  ERROR_QUEUE_IS_FULL   = 17;    // error_codes.json / motor_control.c:1822
static const uint8_t  QUEUE_MAX             = 32;    // MOVEMENT_QUEUE_SIZE (motor_control.h:31)
static const uint8_t  INJECT_TEST_MODE      = 17;    // main.c:909-910 -> fatal_error(17-14)
static const uint8_t  INJECT_FATAL_CODE     = 3;     // 17 - 14 == 3 (ERROR_FLASH_WRITE_FAIL)
static const uint16_t MOSFETS_ENABLED_MASK  = (uint16_t)(1u << 1); // STATUS_MOSFETS_ENABLED_FLAG_BIT == 1

static const float    HOLD_VEL              = 0.0f;  // rot/s: zero-velocity "hold" move (no motion)
static const float    FILL_DUR              = 1.0f;  // s per item: long enough that a fast fill never drains mid-burst
static const float    SHORT_DUR             = 0.5f;  // s per item for the post-recovery re-fill in Part D
static const int      DEEP_QUEUE_MOVES      = 20;    // Part C: deep-but-not-full queue

// Drain any leftover bytes on the motor UART (mandated clean start/end).
static void drainSerial(void) {
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}

// Clean, known starting state.
static void cleanStart(Servomotor* motor) {
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no response during reset; don't check errors
    drainSerial();
}

// Recover after a deliberately-induced fatal error.
static void recoverFromFatal(Servomotor* motor) {
    motor->systemReset();
    delay(FATAL_RESET_DELAY_MS);
    drainSerial();
}

// Read the latched fatal error code via Get status (answered normally even in
// the fatal-error state). Returns 0xFF on a comms-layer failure.
static uint8_t readFatal(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

// Read the status flags via Get status. Returns false on a comms failure.
static bool readFlags(Servomotor* motor, uint16_t* flagsOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return false;
    }
    *flagsOut = st.statusFlags;
    return true;
}

// Read the current queue depth (0..32); returns 0xFF on a comms error.
// NOTE: only valid when the device is NOT in the fatal state (Get n queued
// items is not one of the two fatal-state commands).
static uint8_t readQueueDepth(Servomotor* motor) {
    uint8_t n = motor->getNQueuedItems();
    if (motor->getError() != 0) {
        printf("  (getNQueuedItems comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return n;
}

// Verify the device round-trips a ping payload (alive and answering).
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

// Fire n velocity-0 hold moves as fast as possible (no interleaved reads, so
// the queue does not drain mid-burst). Each is exactly one queue item.
static void blastHoldMoves(Servomotor* motor, int n, float dur) {
    for (int i = 0; i < n; i++) {
        motor->moveWithVelocity(HOLD_VEL, dur);  // one add_to_queue() per call
    }
}

// Poll the queue until it drains to empty, bounded by a wall-clock timeout.
static uint8_t waitForIdle(Servomotor* motor, uint32_t timeout_ms) {
    uint8_t n = 0xFF;
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        n = motor->getNQueuedItems();
        if (motor->getError() == 0 && n == 0) break;
        delay(100);
    }
    delay(150);  // settle margin
    return n;
}

void tm_queue_edges(void) {
    Serial.println("test_ard_queue_edges: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    cleanStart(motor);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);

    uint8_t startFatal = readFatal(motor);
    printf("Fatal code after reset: %d (expect 0)\n", startFatal);
    TEST_RESULT("No fatal error after initial reset", startFatal == 0);

    // =======================================================================
    // PART A: fill the queue with EXACTLY 32 one-item moves. All accepted, peak
    // depth == 32, then it drains naturally to 0 with no fatal error.
    // =======================================================================
    printf("\n--- PART A: fill queue to exactly %d items ---\n", (int)QUEUE_MAX);

    blastHoldMoves(motor, QUEUE_MAX, FILL_DUR);   // 32 velocity-0 items

    uint8_t fatalAfterFill = readFatal(motor);
    uint8_t peakDepth = readQueueDepth(motor);
    printf("After filling %d: fatal=%d, depth=%d (expect fatal 0, depth 32)\n",
           (int)QUEUE_MAX, (int)fatalAfterFill, (int)peakDepth);

    // No overflow occurs at exactly 32 items (motor_control.c:1743 accepts while
    // n_items_in_queue < 32), so no fatal must be latched.
    TEST_RESULT("Q-A All 32 hold moves accepted (no fatal)", fatalAfterFill == 0);
    // Peak depth is exactly the queue capacity (MOVEMENT_QUEUE_SIZE == 32).
    TEST_RESULT("Q-A Queue depth == 32 at peak", peakDepth == QUEUE_MAX);

    // Drain: 32 items * 1.0 s = ~32 s of hold; give it a generous cap.
    uint8_t drained = waitForIdle(motor, 45000);
    printf("Queue depth after draining = %d (expect 0)\n", (int)drained);
    TEST_RESULT("Q-A Queue drains back to zero", drained == 0);

    uint8_t fatalAfterDrain = readFatal(motor);
    // Draining velocity-0 items leaves current_velocity == 0, so no
    // ERROR_RUN_OUT_OF_QUEUE_ITEMS (motor_control.c:2076-2083).
    TEST_RESULT("Q-A No fatal error after full drain", fatalAfterDrain == 0);

    // =======================================================================
    // PART B: overflow boundary. Fill to exactly 32, then a 33rd move must trip
    // the fatal ERROR_QUEUE_IS_FULL (17). The overflowing command itself is
    // answered with the fatal code (via the deferred fatal-error response).
    // =======================================================================
    printf("\n--- PART B: 33rd move must trip ERROR_QUEUE_IS_FULL (17) ---\n");

    cleanStart(motor);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);

    blastHoldMoves(motor, QUEUE_MAX, FILL_DUR);   // fill to 32 (fast, no drain)
    uint8_t preOverflowDepth = readQueueDepth(motor);
    printf("Depth before 33rd move = %d (expect 32)\n", (int)preOverflowDepth);
    TEST_RESULT("Q-B Queue full (32) before overflow", preOverflowDepth == QUEUE_MAX);

    // The 33rd item overflows: add_to_queue() -> fatal_error(17). The firmware
    // answers this very command with the fatal code, surfaced as getError()==17
    // (Communication.cpp:412).
    motor->moveWithVelocity(HOLD_VEL, FILL_DUR);
    int overflowErr = motor->getError();
    printf("33rd move getError() = %d (expect 17)\n", overflowErr);
    TEST_RESULT("Q-B 33rd move rejected with ERROR_QUEUE_IS_FULL (17)",
                overflowErr == (int)ERROR_QUEUE_IS_FULL);

    delay(INJECT_SETTLE_MS);
    uint8_t latchedFull = readFatal(motor);
    printf("getStatus latched fatal = %d (expect 17)\n", (int)latchedFull);
    TEST_RESULT("Q-B getStatus reports latched fatal 17",
                latchedFull == ERROR_QUEUE_IS_FULL);

    recoverFromFatal(motor);
    uint8_t recoveredFatal = readFatal(motor);
    TEST_RESULT("Q-B Recovered (no fatal) after reset", recoveredFatal == 0);
    uint8_t recoveredDepth = readQueueDepth(motor);
    TEST_RESULT("Q-B Queue empty after recovery", recoveredDepth == 0);
    TEST_RESULT("Q-B Device responsive (ping) after recovery", pingRoundTrips(motor));

    // =======================================================================
    // PART C: emergencyStop on a deep queue clears it to 0 and turns MOSFETs
    // off, with NO fatal error.
    // =======================================================================
    printf("\n--- PART C: emergencyStop clears a deep queue + MOSFETs off ---\n");

    cleanStart(motor);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle

    uint16_t flagsOn = 0;
    bool flagsOnOk = readFlags(motor, &flagsOn);
    printf("Flags with MOSFETs enabled = 0x%04X (bit1 expected set)\n", (unsigned)flagsOn);
    // Sanity: MOSFETs really are on before we test that emergencyStop turns them off.
    TEST_RESULT("Q-C MOSFETs on before emergencyStop",
                flagsOnOk && (flagsOn & MOSFETS_ENABLED_MASK) != 0);

    blastHoldMoves(motor, DEEP_QUEUE_MOVES, FILL_DUR);  // deep (~20) hold items
    uint8_t deepDepth = readQueueDepth(motor);
    printf("Deep queue depth before emergencyStop = %d (expect ~%d)\n",
           (int)deepDepth, DEEP_QUEUE_MOVES);
    TEST_RESULT("Q-C Deep queue present before emergencyStop",
                deepDepth != 0xFF && deepDepth >= 10);

    motor->emergencyStop();
    checkMotorError(*motor, "emergencyStop");  // emergency_stop() raises no fatal
    delay(INJECT_SETTLE_MS);

    uint8_t fatalAfterEstop = readFatal(motor);
    // emergency_stop() (motor_control.c:3323) never calls fatal_error().
    TEST_RESULT("Q-C emergencyStop leaves no fatal error", fatalAfterEstop == 0);

    uint16_t flagsOff = 0xFFFF;
    bool flagsOffOk = readFlags(motor, &flagsOff);
    printf("Flags after emergencyStop = 0x%04X (bit1 expected clear)\n", (unsigned)flagsOff);
    // emergency_stop() calls disable_mosfets() first (motor_control.c:3326).
    TEST_RESULT("Q-C emergencyStop turned MOSFETs off",
                flagsOffOk && (flagsOff & MOSFETS_ENABLED_MASK) == 0);

    uint8_t depthAfterEstop = readQueueDepth(motor);
    printf("Queue depth after emergencyStop = %d (expect 0)\n", (int)depthAfterEstop);
    // clear_the_queue_and_stop_no_disable_interrupt() sets n_items_in_queue = 0.
    TEST_RESULT("Q-C emergencyStop cleared queue to zero", depthAfterEstop == 0);

    TEST_RESULT("Q-C Device responsive (ping) after emergencyStop", pingRoundTrips(motor));

    // =======================================================================
    // PART D: with a fatal latched, a new trapezoidMove is rejected (answered
    // with the latched code, never executed); after reset the queue works again.
    // =======================================================================
    printf("\n--- PART D: move rejected while fatal latched; queue works after reset ---\n");

    cleanStart(motor);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);

    // Inject a clean non-motion fatal (code 3) via the test-mode machinery.
    motor->testMode(INJECT_TEST_MODE);
    delay(INJECT_SETTLE_MS);
    uint8_t injected = readFatal(motor);
    printf("Injected fatal code = %d (expect 3)\n", (int)injected);
    TEST_RESULT("Q-D Fatal code 3 latched (testMode 17)", injected == INJECT_FATAL_CODE);

    // A new Trapezoid move while latched: handle_packet()'s default case answers
    // with an ERROR packet carrying the latched code (error_handling.c:104-110),
    // surfaced as a POSITIVE getError() (Communication.cpp:412). The move is NOT
    // queued/executed. (Tiny displacement; it would not run even if it were.)
    motor->trapezoidMove(0.01f, 0.3f);
    int rejectedErr = motor->getError();
    printf("trapezoidMove while latched getError() = %d (expect 3)\n", rejectedErr);
    TEST_RESULT("Q-D trapezoidMove while latched returns error response with code 3",
                rejectedErr == (int)INJECT_FATAL_CODE);

    // The fatal is still latched (the rejected move changed nothing).
    uint8_t stillLatched = readFatal(motor);
    TEST_RESULT("Q-D Fatal still latched (3) after rejected move",
                stillLatched == INJECT_FATAL_CODE);

    // Reset and prove the queue accepts and drains moves again.
    recoverFromFatal(motor);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);

    uint8_t clearedFatal = readFatal(motor);
    TEST_RESULT("Q-D No fatal after recovery reset", clearedFatal == 0);

    blastHoldMoves(motor, 4, SHORT_DUR);  // a few hold items
    uint8_t depthAfterReset = readQueueDepth(motor);
    uint8_t fatalAfterReMove = readFatal(motor);
    printf("Depth after post-reset moves = %d, fatal = %d\n",
           (int)depthAfterReset, (int)fatalAfterReMove);
    TEST_RESULT("Q-D Queue accepts moves again after reset",
                depthAfterReset != 0xFF && depthAfterReset > 0 && fatalAfterReMove == 0);

    uint8_t reDrained = waitForIdle(motor, 8000);
    TEST_RESULT("Q-D Queue drains to zero after recovery", reDrained == 0);

    TEST_RESULT("Q-D Device responsive (ping) at end", pingRoundTrips(motor));

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    drainSerial();
}
