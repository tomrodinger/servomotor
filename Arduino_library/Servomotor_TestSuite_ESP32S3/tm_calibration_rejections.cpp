#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_calibration_rejections.cpp
//
// Probes the pre-flight REJECTION paths of the 'Start calibration' command
// (cmd 6) WITHOUT ever running a real calibration (no ~7 s spin, no auto-reboot,
// no free-shaft requirement). The firmware validates several preconditions at
// the top of start_calibration() (firmware/Src/motor_control.c:535) and latches
// a fatal error if any fails. The checks are sequential `if`s, and fatal_error()
// never returns (it __disable_irq()s and services only Get status / System reset
// until reset), so the FIRST failing check wins deterministically. Order:
//
//   1. motor_control_mode != OPEN_LOOP_POSITION_CONTROL -> ERROR_NOT_IN_OPEN_LOOP
//   2. n_items_in_queue != 0                            -> ERROR_QUEUE_NOT_EMPTY
//   3. motor_busy                                       -> ERROR_MOTOR_BUSY
//   4. test_mode != 0                                   -> ERROR_TEST_MODE_ACTIVE
//
// Error codes (verified in python_programs/servomotor/error_codes.json AND in
// the firmware dispatch at motor_control.c:539-549):
//   ERROR_NOT_IN_OPEN_LOOP  == 7   (code 7,  json line ~101)
//   ERROR_QUEUE_NOT_EMPTY   == 8   (code 8,  json line ~113)
//
// This module exercises checks (1) and (2):
//   (a) startCalibration while in CLOSED-LOOP mode  -> fatal code 7.
//       We reach closed loop with goToClosedLoop(), which on M17/M23/M2 switches
//       to CLOSED_LOOP_POSITION_CONTROL immediately using stored calibration and
//       enables the MOSFETs itself (motor_control.c:1140 start_go_to_closed_loop_mode).
//       We confirm the closed-loop status flag (bit 2) is set BEFORE asserting the
//       rejection, so code 7 is provably attributable to the control mode.
//   (b) startCalibration while a move is queued/executing in OPEN LOOP (motor NOT
//       busy) -> fatal code 8. A normal queued move sets neither closed-loop nor
//       motor_busy, so check (2) fires. We confirm we are still in open loop
//       (closed-loop flag clear) and the queue depth is > 0 before asserting.
//   (c) after both rejections + recovery, the device is fully healthy: status
//       clean (fatal 0, flags 0), queue empty, and a small gentle move executes
//       to completion and moves the shaft.
//
// SAFETY:
//   * tfGetMotor() is unique-ID addressed (extended addressing); every command is
//     collision-safe on the 39-motor rack. No alias-88 or broadcast frame is sent
//     except systemReset (a write-style no-response command) which is fine to
//     broadcast-address but here also goes to the unique-ID target. No raw bus
//     injection at all.
//   * Motion is gentle: a single 0.5-rotation trapezoid over 1.0 s in (b) and a
//     0.25-rotation trapezoid over 0.6 s in (c) -- both << 1 rot and << 3 rot/s.
//     Every trapezoid ends at REST by design, so interrupting (b) with a fatal
//     error and then resetting leaves no residual commanded velocity (no async
//     ERROR_RUN_OUT_OF_QUEUE_ITEMS). MOSFETs are enabled in (b)/(c) so the shaft
//     tracks the commanded position and no ERROR_POSITION_DEVIATION_TOO_LARGE (45)
//     can build up (default 2-rotation deviation limit is never approached).
//   * After every deliberately-induced fatal error we systemReset + delay(2000)
//     (past the post-reset bootloader window) + drain before proceeding.
//   * NO test-mode (cmd 36) values are used anywhere -> the 10..13 permanent-hang
//     trap is impossible to hit here.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int FATAL_RESET_DELAY_MS  = 2000;  // wait after recovering a fatal error

// Firmware fatal codes (error_codes.json + motor_control.c:539-543).
static const uint8_t ERROR_NOT_IN_OPEN_LOOP = 7;
static const uint8_t ERROR_QUEUE_NOT_EMPTY  = 8;

// Status flag bits (common_source_files/device_status.h).
static const uint16_t CLOSED_LOOP_FLAG = (uint16_t)(1u << 2);  // STATUS_CLOSED_LOOP_FLAG_BIT

// Drain any stray bytes left on the RS485 line so they cannot corrupt later reads.
static void drainSerial() {
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}

// Read latched fatal error code AND status flags via Get status (answered
// normally even in the fatal-error state). Returns false on a comms error.
static bool readStatus(Servomotor* motor, uint8_t* codeOut, uint16_t* flagsOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return false;
    }
    *codeOut = st.fatalErrorCode;
    *flagsOut = st.statusFlags;
    return true;
}

// Read the current motion-queue depth; 0xFF on a comms error.
static uint8_t readQueueDepth(Servomotor* motor) {
    uint8_t n = motor->getNQueuedItems();
    if (motor->getError() != 0) {
        printf("  (getNQueuedItems comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return n;
}

// Prove the device is fully responsive by round-tripping a ping payload.
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 3 + 5);
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  ping comms error %d\n", motor->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

// Poll the queue until it drains to empty (bounded ~15 s), then settle.
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

// Fully reset the device to a clean open-loop state and drain the line.
static void cleanReset(Servomotor* motor, int delayMs) {
    motor->systemReset();
    delay(delayMs);  // no response during reset; don't check errors
    drainSerial();
}

void tm_calibration_rejections(void) {
    Serial.println("test_ard_calibration_rejections: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state: open loop, MOSFETs off, no fatal error.
    // -----------------------------------------------------------------------
    cleanReset(motor, NORMAL_RESET_DELAY_MS);

    uint8_t code = 0xFF;
    uint16_t flags = 0xFFFF;
    bool ok = readStatus(motor, &code, &flags);
    printf("After reset: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)ok, (int)code, (unsigned)flags);
    TEST_RESULT("No fatal error after initial reset", ok && code == 0);
    TEST_RESULT("Not in closed loop after reset", ok && (flags & CLOSED_LOOP_FLAG) == 0);

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    // =======================================================================
    // (a) START CALIBRATION WHILE IN CLOSED LOOP -> ERROR_NOT_IN_OPEN_LOOP (7).
    // =======================================================================
    printf("\n--- (a) startCalibration in CLOSED-LOOP mode ---\n");

    // Enter closed loop. On M17 this uses stored calibration, enables the MOSFETs,
    // and sets CLOSED_LOOP_POSITION_CONTROL (motor_control.c:1140). Requires an
    // empty queue (true right after reset) and a calibrated motor (bench + rack
    // units are calibrated on fw 0.15.9.0).
    motor->goToClosedLoop();
    checkMotorError(*motor, "goToClosedLoop");
    delay(400);  // let the closed-loop engagement settle

    uint8_t clCode = 0xFF;
    uint16_t clFlags = 0xFFFF;
    bool clOk = readStatus(motor, &clCode, &clFlags);
    printf("After goToClosedLoop: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)clOk, (int)clCode, (unsigned)clFlags);
    // Precondition proof: no error yet, and we are genuinely in closed loop.
    TEST_RESULT("goToClosedLoop left no fatal error", clOk && clCode == 0);
    TEST_RESULT("Closed-loop flag set after goToClosedLoop",
                clOk && (clFlags & CLOSED_LOOP_FLAG) != 0);

    // Now the rejection: startCalibration must latch fatal code 7.
    motor->startCalibration();
    delay(300);  // let the fatal state latch

    uint8_t rejCode = 0xFF;
    uint16_t rejFlags = 0xFFFF;
    bool rejOk = readStatus(motor, &rejCode, &rejFlags);
    printf("After startCalibration (closed loop): commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)rejOk, (int)rejCode, (unsigned)rejFlags);
    // motor_control.c:539-540 -> fatal_error(ERROR_NOT_IN_OPEN_LOOP), code 7.
    TEST_RESULT("startCalibration in closed loop -> fatal code 7 (ERROR_NOT_IN_OPEN_LOOP)",
                rejOk && rejCode == ERROR_NOT_IN_OPEN_LOOP);
    // fatal_error() refreshes the flags snapshot to 0 (not in the bootloader here).
    TEST_RESULT("Closed-loop rejection refreshes status flags to 0",
                rejOk && rejFlags == 0);

    // Recover fully (past the bootloader window).
    cleanReset(motor, FATAL_RESET_DELAY_MS);
    uint8_t recA = 0xFF; uint16_t recFA = 0xFFFF;
    bool recAok = readStatus(motor, &recA, &recFA);
    TEST_RESULT("Recovered clean after closed-loop rejection", recAok && recA == 0);

    // =======================================================================
    // (b) START CALIBRATION WITH A MOVE QUEUED (OPEN LOOP) -> ERROR_QUEUE_NOT_EMPTY (8).
    // =======================================================================
    printf("\n--- (b) startCalibration with a queued move (open loop) ---\n");

    // Enable MOSFETs (stays OPEN_LOOP_POSITION_CONTROL) so the shaft tracks the
    // commanded position and no deviation (code 45) accrues while a move runs.
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // commutation-alignment transient

    // Confirm we are still in open loop (closed-loop flag clear) before queueing.
    uint8_t olCode = 0xFF; uint16_t olFlags = 0xFFFF;
    bool olOk = readStatus(motor, &olCode, &olFlags);
    printf("Before queueing: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)olOk, (int)olCode, (unsigned)olFlags);
    TEST_RESULT("In open loop (closed-loop flag clear) before queueing",
                olOk && olCode == 0 && (olFlags & CLOSED_LOOP_FLAG) == 0);

    // Queue a gentle 0.5-rotation move over 1.0 s (a trapezoid enqueues up to 3
    // items and ends at rest). It is still executing on the next command.
    motor->trapezoidMove(0.5f, 1.0f);
    checkMotorError(*motor, "trapezoidMove(queue non-empty setup)");

    uint8_t depth = readQueueDepth(motor);
    printf("Queue depth after trapezoidMove = %d (expect > 0)\n", (int)depth);
    // Precondition proof: the queue really is non-empty, in open loop, not busy.
    TEST_RESULT("Move is queued (depth > 0) in open loop", depth != 0xFF && depth > 0);

    // The rejection: startCalibration must latch fatal code 8 (queue not empty),
    // NOT code 19 (motor_busy is only set during calibration/homing, not a normal
    // move) -- firmware order is queue-check before busy-check (motor_control.c:542).
    motor->startCalibration();
    delay(300);  // let the fatal state latch

    uint8_t qrCode = 0xFF; uint16_t qrFlags = 0xFFFF;
    bool qrOk = readStatus(motor, &qrCode, &qrFlags);
    printf("After startCalibration (queue non-empty): commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)qrOk, (int)qrCode, (unsigned)qrFlags);
    // motor_control.c:542-543 -> fatal_error(ERROR_QUEUE_NOT_EMPTY), code 8.
    TEST_RESULT("startCalibration with queued move -> fatal code 8 (ERROR_QUEUE_NOT_EMPTY)",
                qrOk && qrCode == ERROR_QUEUE_NOT_EMPTY);
    TEST_RESULT("Queue-not-empty rejection refreshes status flags to 0",
                qrOk && qrFlags == 0);

    // Recover fully.
    cleanReset(motor, FATAL_RESET_DELAY_MS);
    uint8_t recB = 0xFF; uint16_t recFB = 0xFFFF;
    bool recBok = readStatus(motor, &recB, &recFB);
    TEST_RESULT("Recovered clean after queue-not-empty rejection", recBok && recB == 0);

    // =======================================================================
    // (c) DEVICE FULLY HEALTHY AFTER BOTH REJECTIONS: clean status, empty queue,
    //     and a small gentle move executes to completion and moves the shaft.
    // =======================================================================
    printf("\n--- (c) device healthy after both rejections ---\n");

    uint8_t hCode = 0xFF; uint16_t hFlags = 0xFFFF;
    bool hOk = readStatus(motor, &hCode, &hFlags);
    printf("Health check status: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)hOk, (int)hCode, (unsigned)hFlags);
    TEST_RESULT("Status clean after both rejections (fatal 0, flags 0)",
                hOk && hCode == 0 && hFlags == 0);
    TEST_RESULT("Device responsive (ping) after both rejections", pingRoundTrips(motor));

    uint8_t qEmpty = readQueueDepth(motor);
    TEST_RESULT("Queue empty after both rejections", qEmpty == 0);

    // A small gentle move must work end-to-end.
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets(health move)");
    delay(300);  // commutation-alignment transient

    int64_t startPos = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw(before health move)");

    motor->trapezoidMove(0.25f, 0.6f);
    checkMotorError(*motor, "trapezoidMove(health move)");

    uint8_t drained = waitForIdle(motor);
    printf("Queue depth after health move drained = %d (expect 0)\n", (int)drained);
    TEST_RESULT("Health move queue drains to zero", drained == 0);

    int64_t endPos = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw(after health move)");

    uint8_t postCode = 0xFF; uint16_t postFlags = 0xFFFF;
    bool postOk = readStatus(motor, &postCode, &postFlags);
    printf("After health move: commsOk=%d fatalCode=%d startPos=%lld endPos=%lld\n",
           (int)postOk, (int)postCode, (long long)startPos, (long long)endPos);
    TEST_RESULT("Health move completed with no fatal error", postOk && postCode == 0);
    // 0.25 shaft rotation == 0.25 * 3276800 == 819200 raw counts; the shaft must
    // have advanced substantially (allow slack for tracking/settle).
    TEST_RESULT("Health move advanced the shaft (>= 0.1 rot raw)",
                (endPos - startPos) >= (int64_t)327680);

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest.
    // -----------------------------------------------------------------------
    cleanReset(motor, NORMAL_RESET_DELAY_MS);
}
