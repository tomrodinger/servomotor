#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_pid_dynamics.cpp
//
// Arduino-library on-device test for the PID-error dynamics command group:
//   * cmd 39  Get max PID error   (min/max PID-error window, read-then-reset)
//   * cmd 43  Set PID constants    (kP, kI, kD applied immediately)
//
// Related Python test: python_programs/test_set_pid_constants.py.
//
// This exercises the OBSERVABLE dynamics of the PID-error window rather than
// fragile magnitude thresholds:
//
//   A. Before closed loop (fresh after reset, open-loop) the window is the
//      firmware SENTINEL: min = INT32_MAX, max = INT32_MIN, i.e. min > max.
//      The control loop never seeded it because PID tracking runs only in
//      closed loop.
//   B. After entering closed loop and running a gentle 0.5-rotation trapezoid,
//      the window becomes REAL: min <= max (a proper, non-inverted window).
//   C. Reading again IMMEDIATELY (no motion between) proves read-then-reset
//      semantics: cmd 39 resets the accumulator on every read, so the second
//      read is a freshly re-seeded window, NOT the identical latched extremes
//      the first read returned.
//   D. Setting deliberately WEAK PID constants (1/4 of the firmware defaults)
//      still leaves the motor able to run the same move and still produces a
//      valid (non-sentinel) window with no fatal error - proving cmd 43 is
//      accepted and the motor keeps controlling. Defaults are then restored.
//
// The firmware's compile-time default M17 gains are (2000, 5, 175000); a
// quarter of those (500, 1, 43750) is weak but still stable on a free shaft,
// so the move completes cleanly (no ERROR_POSITION_DEVIATION_TOO_LARGE).
//
// SAFETY: single motor addressed by unique-ID (collision-safe on the rack).
// All motion is a <= 0.5-rotation trapezoid (ends at rest by design), waited
// to completion before any assert. No broadcast reads. Ends reset + MOSFETs
// off with firmware-default gains restored.
// ---------------------------------------------------------------------------

static const int32_t PID_SENTINEL_MIN = 2147483647;   // INT32_MAX (firmware seed for min)
static const int32_t PID_SENTINEL_MAX = -2147483648;  // INT32_MIN (firmware seed for max)

static const uint16_t CLOSED_LOOP_BIT = (1u << 2);    // status flag: closed-loop control active

// Firmware compile-time default M17 gains, and a deliberately weak quarter set.
static const uint32_t GOOD_KP = 2000, GOOD_KI = 5, GOOD_KD = 175000;
static const uint32_t WEAK_KP = 500,  WEAK_KI = 1, WEAK_KD = 43750;

static const int   RESET_DELAY_MS      = 1500;  // wait after systemReset (device silent during reset)
static const int   ENABLE_SETTLE_MS    = 300;   // let commutation-alignment transient settle
static const int   SETTLE_MS           = 300;   // post-move settle before reading window
static const int   CLOSED_LOOP_TIMEOUT_MS = 6000;

// A real (non-sentinel) window has min <= max. The sentinel is inverted (min > max).
static bool isSentinelWindow(const getMaxPidErrorResponse& w) {
    return (w.minPidError == PID_SENTINEL_MIN) && (w.maxPidError == PID_SENTINEL_MAX);
}
static bool isValidWindow(const getMaxPidErrorResponse& w) {
    return w.minPidError <= w.maxPidError;
}

// Wait until the motion queue is empty, then settle.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 1000; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (motor->getError() != 0) break;  // don't spin on a comms/fatal error
        if (n == 0) break;
        delay(10);
    }
    delay(SETTLE_MS);
}

// Read the device fatal-error code via Get status (answered even in fatal state).
static uint8_t readFatalErrorCode(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) return 0xFF;
    return st.fatalErrorCode;
}

// Poll Get status until the closed-loop bit is set (or timeout). Returns true if entered.
static bool waitForClosedLoop(Servomotor* motor, int timeout_ms) {
    for (int t = 0; t < timeout_ms; t += 100) {
        getStatusResponse st = motor->getStatus();
        if (motor->getError() == 0 && (st.statusFlags & CLOSED_LOOP_BIT)) return true;
        delay(100);
    }
    return false;
}

// Bring the motor from a clean reset into closed loop at a zeroed origin, then
// discard the settle-time PID window so the next read reflects only new motion.
static bool enterClosedLoopZeroed(Servomotor* motor) {
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(ENABLE_SETTLE_MS);
    motor->goToClosedLoop();
    checkMotorError(*motor, "goToClosedLoop");
    bool inLoop = waitForClosedLoop(motor, CLOSED_LOOP_TIMEOUT_MS);
    if (!inLoop) return false;
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    motor->getMaxPidErrorRaw();   // discard the closed-loop-entry / settle window
    checkMotorError(*motor, "getMaxPidError discard");
    return true;
}

void tm_pid_dynamics(void) {
    Serial.println("test_ard_pid_dynamics: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean known state ----
    motor->systemReset();
    delay(RESET_DELAY_MS);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    // =====================================================================
    // PART A: sentinel window before closed loop (open-loop, fresh reset).
    // =====================================================================
    printf("\n=== A: max PID error window before closed loop (expect sentinel) ===\n");
    getMaxPidErrorResponse wA = motor->getMaxPidErrorRaw();
    checkMotorError(*motor, "getMaxPidError A");
    printf("  min=%ld max=%ld (sentinel is min=%ld max=%ld)\n",
           (long)wA.minPidError, (long)wA.maxPidError,
           (long)PID_SENTINEL_MIN, (long)PID_SENTINEL_MAX);
    TEST_RESULT("A max PID error is sentinel (min>max) before closed loop",
                isSentinelWindow(wA));

    // =====================================================================
    // PART B: enter closed loop, run a 0.5-rotation trapezoid, read the
    // window -> it must be a REAL, non-inverted window (min <= max).
    // =====================================================================
    printf("\n=== B: closed loop + 0.5-rot trapezoid -> real PID window ===\n");
    bool inLoopB = enterClosedLoopZeroed(motor);
    TEST_RESULT("B closed loop entered (with good gains)", inLoopB);
    if (!inLoopB) {
        // Cannot exercise the window dynamics without closed loop; clean up and stop.
        motor->disableMosfets();
        motor->systemReset();
        delay(RESET_DELAY_MS);
        return;
    }

    motor->trapezoidMove(0.5f, 1.0f);   // gentle half rotation, ends at rest
    checkMotorError(*motor, "trapezoidMove B");
    waitForIdle(motor);

    getMaxPidErrorResponse wB = motor->getMaxPidErrorRaw();
    checkMotorError(*motor, "getMaxPidError B");
    printf("  after move: min=%ld max=%ld\n", (long)wB.minPidError, (long)wB.maxPidError);
    TEST_RESULT("B real PID window after move (min<=max, not sentinel)",
                isValidWindow(wB) && !isSentinelWindow(wB));

    // =====================================================================
    // PART C: read again IMMEDIATELY (no motion) -> read-then-reset means the
    // window was re-seeded, so it is NOT the identical latched extremes from B.
    // =====================================================================
    printf("\n=== C: immediate re-read proves read-then-reset (re-seed) ===\n");
    getMaxPidErrorResponse wC = motor->getMaxPidErrorRaw();
    checkMotorError(*motor, "getMaxPidError C");
    printf("  re-read: min=%ld max=%ld\n", (long)wC.minPidError, (long)wC.maxPidError);
    bool reseeded = (wC.minPidError != wB.minPidError) || (wC.maxPidError != wB.maxPidError);
    TEST_RESULT("C re-read differs from previous (window was reset/re-seeded)", reseeded);
    // The re-seeded window is itself well-formed: sentinel (no tick yet) or valid.
    TEST_RESULT("C re-seeded window is well-formed (sentinel or min<=max)",
                isSentinelWindow(wC) || isValidWindow(wC));

    // Return home and drop closed loop before changing gains.
    motor->trapezoidMove(-0.5f, 1.0f);
    checkMotorError(*motor, "trapezoidMove B return");
    waitForIdle(motor);
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets B");
    motor->systemReset();
    delay(RESET_DELAY_MS);

    // =====================================================================
    // PART D: set WEAK PID (1/4 defaults); the same move still completes and
    // still yields a valid non-sentinel window with no fatal error.
    // =====================================================================
    printf("\n=== D: weak PID (1/4 defaults) still controls -> valid window ===\n");
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    motor->setPidConstants(WEAK_KP, WEAK_KI, WEAK_KD);
    TEST_RESULT("D setPidConstants (weak) accepted (no comms error)",
                motor->getError() == 0);

    bool inLoopD = enterClosedLoopZeroed(motor);
    TEST_RESULT("D closed loop entered (with weak gains)", inLoopD);
    if (inLoopD) {
        motor->trapezoidMove(0.5f, 1.0f);
        checkMotorError(*motor, "trapezoidMove D");
        waitForIdle(motor);

        uint8_t fatalD = readFatalErrorCode(motor);
        printf("  weak-gain move fatal code: %d (expect 0)\n", fatalD);
        TEST_RESULT("D weak-gain move did not trip a fatal error", fatalD == 0);

        getMaxPidErrorResponse wD = motor->getMaxPidErrorRaw();
        checkMotorError(*motor, "getMaxPidError D");
        printf("  weak-gain window: min=%ld max=%ld\n",
               (long)wD.minPidError, (long)wD.maxPidError);
        TEST_RESULT("D weak-gain window still valid (min<=max, not sentinel)",
                    isValidWindow(wD) && !isSentinelWindow(wD));

        // Return home before restoring gains.
        motor->trapezoidMove(-0.5f, 1.0f);
        checkMotorError(*motor, "trapezoidMove D return");
        waitForIdle(motor);
        motor->disableMosfets();
        checkMotorError(*motor, "disableMosfets D");
    }

    // Restore the firmware-default gains (cmd 43 accepted the other direction too).
    motor->setPidConstants(GOOD_KP, GOOD_KI, GOOD_KD);
    TEST_RESULT("D setPidConstants (restore defaults) accepted (no comms error)",
                motor->getError() == 0);

    // ---- Clean end: reset restores compile-time defaults, MOSFETs off. ----
    printf("\nCleaning up (systemReset restores default gains)...\n");
    motor->systemReset();
    delay(RESET_DELAY_MS);
}
