#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_mosfet_interactions.cpp
//
// Edge-case probe of how motion commands interact with the MOSFET (drive)
// state. Every assertion below encodes firmware GROUND TRUTH, cited inline.
//
// THREE interactions are exercised:
//
//   (a) Trapezoid move while the MOSFETs are DISABLED.
//       Firmware ground truth:
//         * The TRAPEZOID_MOVE dispatch (firmware/Src/main.c:253) performs NO
//           MOSFET-state check: it just calls add_trapezoid_move_to_queue().
//           So the move is ACCEPTED (normal no-error reply) regardless of drive
//           state.
//         * The motion profile advances the COMMANDED position every control
//           tick independent of the drive stage; get_motor_position()
//           (motor_control.c:3332) returns current_position_i96.position, which
//           is what Get position / getPositionRaw report. So the commanded
//           position advances even though the de-energized shaft does not move.
//         * check_if_actual_vs_desired_position_deviated_too_much() runs every
//           main-loop pass (main.c:1503) and, when
//               |commanded - hall| > max_allowable_position_deviation
//           raises fatal_error(ERROR_POSITION_DEVIATION_TOO_LARGE)
//           (motor_control.c:3136-3141). The default limit is
//           DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION = ONE_REVOLUTION_MICROSTEPS
//           * 2 (motor_control.c:97), i.e. 2 shaft rotations. With the drive
//           off the shaft stays put, so a commanded move larger than 2 rot
//           drives the deviation past the limit and latches fatal code 45.
//       We prove: a small (<2 rot) disabled move is accepted, advances the
//       commanded position, and does NOT fault; then a >2.5 rot disabled move
//       DELIBERATELY latches fatal 45; then systemReset recovers.
//       ERROR_POSITION_DEVIATION_TOO_LARGE == 45 (error_codes.json).
//       SAFE: throughout (a) the MOSFETs are OFF, so the shaft never moves,
//       regardless of the >2.5 rot commanded distance.
//
//   (b) goToClosedLoop while the MOSFETs are DISABLED.
//       Firmware ground truth (M17 / non-M1 branch,
//       start_go_to_closed_loop_mode() at motor_control.c:1140): the only
//       preconditions are an empty motion queue (else ERROR_QUEUE_NOT_EMPTY)
//       and not-busy (else ERROR_MOTOR_BUSY). It then calls
//       check_current_sensor_and_enable_mosfets() -- which ITSELF enables the
//       MOSFETs -- and set_motor_control_mode(CLOSED_LOOP_POSITION_CONTROL).
//       So goToClosedLoop from a disabled state does NOT error and does NOT
//       require MOSFETs to be pre-enabled: it succeeds, turns the MOSFETs ON,
//       and enters closed-loop mode. (This contradicts a naive "needs MOSFETs
//       first" guess; see notes.) We assert the true behavior via the status
//       flags: STATUS_MOSFETS_ENABLED_FLAG_BIT==1 (mask 0x02) and
//       STATUS_CLOSED_LOOP_FLAG_BIT==2 (mask 0x04) (device_status.h). No motion
//       is commanded, so the shaft only holds position: SAFE.
//
//   (c) disableMosfets DURING a queued move.
//       Firmware ground truth: disable_mosfets() (firmware/Src/mosfets.c:17)
//       only turns off the hardware drive outputs -- it does NOT touch the
//       motion queue or the motor control mode. So a disable issued mid-move is
//       ACCEPTED (normal reply), the drive coasts, and the queue KEEPS RUNNING:
//       the commanded position continues to advance to the move's target. We
//       prove the queue was not cancelled by showing the commanded position
//       keeps climbing AFTER the disable. The move used here is <=1 rot, so the
//       resulting commanded-vs-hall deviation stays well under the 2-rot limit
//       and no fatal 45 occurs. SAFE: gentle <=1 rot / <1 rot/s move that ends
//       at rest.
//
// SAFETY SUMMARY:
//   * Single motor, unique-ID addressed via tfGetMotor() -- collision-safe on
//     the 39-motor rack; no alias-88 or multi-response frames; no raw bus
//     injection.
//   * No Test mode (cmd 36) values used anywhere (never 10..13).
//   * All physically-energized motion is gentle (<=1 rot, <1 rot/s) and ends at
//     rest. The only large commanded move happens with the drive OFF.
//   * After the one deliberately-induced fatal error (part a), the device is
//     systemReset + settled + drained before proceeding.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;   // wait after a plain reset
static const int FATAL_RESET_DELAY_MS  = 2000;   // wait after recovering a fatal error
static const int ENABLE_SETTLE_MS      = 350;    // commutation-alignment settle after enable

static const uint8_t ERROR_POSITION_DEVIATION_TOO_LARGE = 45;  // error_codes.json

// Status-flag masks (common_source_files/device_status.h):
//   STATUS_MOSFETS_ENABLED_FLAG_BIT == 1  -> 1<<1 == 0x02
//   STATUS_CLOSED_LOOP_FLAG_BIT     == 2  -> 1<<2 == 0x04
static const uint16_t MOSFETS_ENABLED_MASK = (1u << 1);
static const uint16_t CLOSED_LOOP_MASK     = (1u << 2);

// Drain any stray bytes on the motor serial line (rule 2).
static void drainSerial() {
    uint32_t t0 = millis();
    while (Serial1.available()) {
        Serial1.read();
        if (millis() - t0 > 500) break;  // bounded, never hang
    }
}

// Full clean reset: systemReset + settle + drain.
static void cleanReset(Servomotor* motor, int settleMs) {
    motor->systemReset();
    delay(settleMs);   // no response is emitted during reset; don't check errors
    drainSerial();
}

// Read the latched fatal error code via Get status (answered normally even in
// the fatal state). Returns 0xFF on a comms-layer failure.
static uint8_t readFatalCode(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

// Read the status flags word; returns 0xFFFF on a comms-layer failure.
static uint16_t readFlags(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFFFF;
    }
    return st.statusFlags;
}

// Poll Get status until a nonzero fatal code appears or the timeout elapses.
// Returns the code (0 if none latched within the window, 0xFF on comms error).
static uint8_t pollForFatal(Servomotor* motor, uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        uint8_t c = readFatalCode(motor);
        if (c == 0xFF) return 0xFF;
        if (c != 0) return c;
        delay(20);
    }
    return 0;
}

// Poll the queue until it drains to 0 (bounded). Returns the last depth read.
static uint8_t waitForIdle(Servomotor* motor, uint32_t timeoutMs) {
    uint8_t n = 0xFF;
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        n = motor->getNQueuedItems();
        if (motor->getError() == 0 && n == 0) break;
        delay(20);
    }
    delay(150);  // settle margin
    return n;
}

void tm_mosfet_interactions(void) {
    Serial.println("test_ard_mosfet_interactions: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // Client-side unit settings persist across device resets (they live on the
    // host object), so set them once up front.
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);

    // ---- Clean, known starting state (MOSFETs off after reset). ----
    cleanReset(motor, NORMAL_RESET_DELAY_MS);

    uint8_t startFatal = readFatalCode(motor);
    printf("Fatal code after initial reset: %d\n", startFatal);
    TEST_RESULT("No fatal error after initial reset", startFatal == 0);

    // counts-per-rotation, so we can reason about the commanded position in raw
    // units without hardcoding the product constant.
    getProductSpecsResponse specs = motor->getProductSpecs();
    uint32_t cpr = specs.countsPerRotation;
    printf("countsPerRotation = %lu\n", (unsigned long)cpr);
    TEST_RESULT("Product specs report a nonzero counts-per-rotation", cpr > 0);

    // =======================================================================
    // (a) Trapezoid move while MOSFETs DISABLED.
    // =======================================================================
    printf("\n--- (a) trapezoidMove while MOSFETs disabled ---\n");

    // Make sure the drive is OFF (reset already leaves it off; be explicit).
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    uint16_t preFlags = readFlags(motor);
    printf("Flags before disabled move = 0x%04X\n", (unsigned)preFlags);
    // Ground truth: after reset+disable the MOSFETS_ENABLED bit must be clear.
    TEST_RESULT("MOSFETs are disabled at start of part (a)",
                preFlags != 0xFFFF && (preFlags & MOSFETS_ENABLED_MASK) == 0);

    // --- Small disabled move: accepted, advances commanded position, no fault.
    // 0.5 rot << 2 rot default deviation limit, so it must NOT latch fatal 45.
    int64_t posBefore = motor->getPositionRaw();
    bool posBeforeOk = (motor->getError() == 0);

    motor->trapezoidMove(0.5f, 1.5f);   // 0.5 rotation over 1.5 s
    bool smallMoveComms = (motor->getError() == 0);
    // Ground truth (main.c:253): no MOSFET check -> accepted with a normal reply.
    TEST_RESULT("Trapezoid move accepted while MOSFETs disabled (no comms error)",
                smallMoveComms);

    waitForIdle(motor, 6000);           // let the (drive-off) profile complete

    int64_t posAfter = motor->getPositionRaw();
    bool posAfterOk = (motor->getError() == 0);
    printf("Commanded position: before=%lld after=%lld (expect ~%ld)\n",
           (long long)posBefore, (long long)posAfter, (long)(cpr / 2));
    // Ground truth: get_motor_position() returns the commanded profile position,
    // which advances even with the drive off. Expect ~0.5 rot of counts.
    TEST_RESULT("Commanded position advanced while MOSFETs disabled",
                posBeforeOk && posAfterOk &&
                (posAfter - posBefore) > (int64_t)(cpr / 4));

    uint8_t smallMoveFatal = readFatalCode(motor);
    printf("Fatal code after small disabled move = %d\n", smallMoveFatal);
    // 0.5 rot deviation < 2 rot limit -> no fatal.
    TEST_RESULT("Small disabled move (<2 rot) does not fault", smallMoveFatal == 0);

    // Reset before the deliberate over-deviation to start from a clean profile.
    cleanReset(motor, NORMAL_RESET_DELAY_MS);

    // --- Large disabled move: DELIBERATELY exceed the 2-rot deviation limit.
    // Give the planner generous limits so a 3-rot / 3-s move is realizable; the
    // drive is OFF so none of this produces shaft motion.
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets(before big move)");
    motor->setMaximumVelocity(5.0f);        // 5 rot/s (non-zero: parameter valid)
    checkMotorError(*motor, "setMaximumVelocity");
    motor->setMaximumAcceleration(50.0f);   // 50 rot/s^2
    checkMotorError(*motor, "setMaximumAcceleration");

    motor->trapezoidMove(3.0f, 3.0f);       // 3 rot commanded, drive OFF -> shaft still
    bool bigMoveComms = (motor->getError() == 0);
    TEST_RESULT("Large (>2.5 rot) disabled move accepted (no comms error)",
                bigMoveComms);

    // The deviation crosses 2 rot partway through; poll for the fatal latch.
    uint8_t devFatal = pollForFatal(motor, 12000);
    printf("Fatal code after large disabled move = %d\n", devFatal);
    // Ground truth: motor_control.c:3136-3141 raises code 45 once deviation > 2 rot.
    TEST_RESULT("Large disabled move latches a fatal error",
                devFatal != 0 && devFatal != 0xFF);
    TEST_RESULT("Fatal error is ERROR_POSITION_DEVIATION_TOO_LARGE (45)",
                devFatal == ERROR_POSITION_DEVIATION_TOO_LARGE);

    // Recover from the deliberate fatal error.
    cleanReset(motor, FATAL_RESET_DELAY_MS);
    uint8_t afterDevReset = readFatalCode(motor);
    printf("Fatal code after recovery reset = %d\n", afterDevReset);
    TEST_RESULT("Deviation fatal cleared by systemReset", afterDevReset == 0);

    // =======================================================================
    // (b) goToClosedLoop while MOSFETs DISABLED.
    // Ground truth (motor_control.c:1140): enables the MOSFETs itself and enters
    // closed-loop mode; no error, no pre-enable requirement.
    // =======================================================================
    printf("\n--- (b) goToClosedLoop while MOSFETs disabled ---\n");

    // Fresh reset already left the drive off and the queue empty.
    uint16_t flagsBeforeGTCL = readFlags(motor);
    printf("Flags before goToClosedLoop = 0x%04X\n", (unsigned)flagsBeforeGTCL);
    TEST_RESULT("MOSFETs disabled before goToClosedLoop",
                flagsBeforeGTCL != 0xFFFF &&
                (flagsBeforeGTCL & MOSFETS_ENABLED_MASK) == 0);

    motor->goToClosedLoop();
    bool gtclComms = (motor->getError() == 0);
    TEST_RESULT("goToClosedLoop accepted from disabled state (no comms error)",
                gtclComms);
    delay(ENABLE_SETTLE_MS);  // let the enable/commutation transient settle

    uint8_t gtclFatal = readFatalCode(motor);
    printf("Fatal code after goToClosedLoop = %d\n", gtclFatal);
    // Ground truth: preconditions (empty queue, not busy) are met, so no fatal.
    TEST_RESULT("goToClosedLoop from disabled state raised no fatal error",
                gtclFatal == 0);

    uint16_t flagsAfterGTCL = readFlags(motor);
    printf("Flags after goToClosedLoop = 0x%04X\n", (unsigned)flagsAfterGTCL);
    // Ground truth: check_current_sensor_and_enable_mosfets() turned the drive ON.
    TEST_RESULT("goToClosedLoop enabled the MOSFETs",
                flagsAfterGTCL != 0xFFFF &&
                (flagsAfterGTCL & MOSFETS_ENABLED_MASK) != 0);
    // Ground truth: set_motor_control_mode(CLOSED_LOOP_POSITION_CONTROL).
    TEST_RESULT("goToClosedLoop entered closed-loop mode",
                flagsAfterGTCL != 0xFFFF &&
                (flagsAfterGTCL & CLOSED_LOOP_MASK) != 0);

    // Leave the drive off and clean before part (c).
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets(after goToClosedLoop)");
    cleanReset(motor, NORMAL_RESET_DELAY_MS);

    // =======================================================================
    // (c) disableMosfets DURING a queued move.
    // Ground truth (mosfets.c:17): disable only toggles the drive outputs; the
    // queue keeps running and the commanded position keeps advancing (coast).
    // =======================================================================
    printf("\n--- (c) disableMosfets during a queued move ---\n");

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(ENABLE_SETTLE_MS);  // settle the commutation-alignment transient

    // Gentle, energized move: 0.8 rot over 2.0 s (peak < 1 rot/s), ends at rest.
    // Max deviation possible (0.8 rot) stays well under the 2-rot limit -> no 45.
    motor->trapezoidMove(0.8f, 2.0f);
    bool coastMoveComms = (motor->getError() == 0);
    TEST_RESULT("Move queued for coast test (no comms error)", coastMoveComms);

    delay(400);  // ~0.4 s into a 2 s move: queue is executing
    int64_t pMid1 = motor->getPositionRaw();
    bool pMid1Ok = (motor->getError() == 0);
    uint8_t depthMid = motor->getNQueuedItems();
    bool depthOk = (motor->getError() == 0);
    printf("Mid-move: commanded=%lld queueDepth=%d\n",
           (long long)pMid1, (int)depthMid);
    TEST_RESULT("Queue is executing before the mid-move disable",
                pMid1Ok && depthOk && depthMid > 0 && pMid1 > 0);

    // Disable the MOSFETs mid-move.
    motor->disableMosfets();
    bool disableComms = (motor->getError() == 0);
    // Ground truth: disable has no queue/mode side effects -> normal reply.
    TEST_RESULT("disableMosfets mid-move accepted (no comms error)", disableComms);

    uint16_t flagsAfterDisable = readFlags(motor);
    printf("Flags after mid-move disable = 0x%04X\n", (unsigned)flagsAfterDisable);
    TEST_RESULT("MOSFETs are off after mid-move disable",
                flagsAfterDisable != 0xFFFF &&
                (flagsAfterDisable & MOSFETS_ENABLED_MASK) == 0);

    delay(400);  // still before the 2 s move end
    int64_t pMid2 = motor->getPositionRaw();
    bool pMid2Ok = (motor->getError() == 0);
    printf("After disable: commanded=%lld (was %lld)\n",
           (long long)pMid2, (long long)pMid1);
    // Ground truth: the queue was NOT cancelled -> commanded position kept
    // advancing toward the target even though the drive is now coasting.
    TEST_RESULT("disableMosfets did NOT cancel the queue (commanded kept advancing)",
                pMid1Ok && pMid2Ok && pMid2 > pMid1);

    uint8_t depthDrained = waitForIdle(motor, 6000);
    printf("Queue depth after drain = %d\n", (int)depthDrained);

    uint8_t coastFatal = readFatalCode(motor);
    printf("Fatal code after coasting move = %d\n", coastFatal);
    // Small (<=0.8 rot) deviation stays under the 2-rot limit -> no fatal.
    TEST_RESULT("No fatal error after coasting move completed (deviation < 2 rot)",
                coastFatal == 0);

    // ---- Leave the device clean and at rest. ----
    cleanReset(motor, NORMAL_RESET_DELAY_MS);
}
