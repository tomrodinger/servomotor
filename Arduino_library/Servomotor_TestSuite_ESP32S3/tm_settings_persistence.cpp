#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_settings_persistence.cpp
//
// Edge-case / robustness module: proves that RAM-only motion settings revert to
// their firmware boot defaults on System reset (they are NOT persisted to
// flash). Three independent settings are exercised:
//
//   1. Maximum velocity + maximum acceleration (read back directly via Get
//      debug values, cmd 45, fields maxVelocity / maxAcceleration).
//   2. Max allowable position deviation (no direct readback -> observed
//      indirectly: a tiny limit makes a normal closed-loop move latch fatal
//      error 45, and after reset the same move is clean).
//   3. PID constants (no direct readback -> observed indirectly: after setting
//      them weak and resetting, a closed-loop move still physically lands on
//      target, proving the strong boot defaults are back).
//
// GROUND TRUTH (all cited inline):
//   * firmware/Src/motor_control.c:163-164  boot defaults are static-initialised
//       (max_acceleration = MAX_ACCELERATION, max_velocity = MAX_VELOCITY); a
//       real MCU reset (System reset) re-runs static init -> revert. RAM only.
//   * firmware/Src/motor_control.c:3259-3294  set_max_velocity shifts the sent
//       value left by VELOCITY_SHIFT_LEFT (=12, motor_control.h:65) and clamps
//       to EXPERIMENTAL_MAX_VELOCITY; set_max_acceleration shifts left by
//       ACCELERATION_SHIFT_LEFT (=8, motor_control.h:64), clamps to
//       EXPERIMENTAL_MAX_ACCELERATION. So debug field == (rawSent << shift).
//   * firmware/Src/motor_control.c:3444-3447  set_max_allowable_position_deviation
//       stores the raw value directly (no shift). Default 2 shaft rotations
//       (DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION, line 97).
//   * firmware/Src/motor_control.c:3127-3143  deviation check raises
//       fatal_error(ERROR_POSITION_DEVIATION_TOO_LARGE). That code == 45
//       (python_programs/servomotor/error_codes.json:663-666, enum
//       ERROR_POSITION_DEVIATION_TOO_LARGE, "position deviation too large").
//   * firmware/Src/main.c:1438  on init set_pid_constants(PROPORTIONAL_CONSTANT_PID,
//       INTEGRAL_CONSTANT_PID, DERIVATIVE_CONSTANT_PID) restores the strong PID
//       defaults; set_pid_constants (motor_control.c:2266) writes RAM only.
//
// SAFETY:
//   * tfGetMotor() is unique-ID addressed -> every command is collision-safe on
//     the 39-motor bus. No alias-88 frames, no broadcast, no raw bus injection.
//   * All motion is gentle (<= 0.5 rotation, trapezoid ends at rest, drained
//     before any position read). Test-mode values 10..13 are never sent (no
//     testMode calls at all here).
//   * The deviation test deliberately induces fatal error 45; per suite policy
//     it is immediately followed by systemReset + delay(1500) + drain before
//     anything else runs.
// ---------------------------------------------------------------------------

static const unsigned long RESET_DELAY_MS        = 1500;
static const unsigned long ENABLE_SETTLE_MS      = 300;
static const unsigned long CLOSED_LOOP_TIMEOUT_MS = 6000;
static const unsigned long DRAIN_TIMEOUT_MS      = 4000;
static const unsigned long DEVIATION_TRIP_TIMEOUT_MS = 4000;

static const uint16_t CLOSED_LOOP_BIT   = 1 << 2;   // status flag: closed-loop engaged
static const int32_t  COUNTS_PER_ROTATION = 3276800; // M17/M23/M3 ONE_REVOLUTION_MICROSTEPS
static const int32_t  VELOCITY_SHIFT_LEFT = 12;      // motor_control.h:65
static const int32_t  ACCEL_SHIFT_LEFT    = 8;       // motor_control.h:64

// Deviation fatal code — verified in error_codes.json:663-666.
static const uint8_t  FATAL_POSITION_DEVIATION = 45;
// Tiny deviation limit (raw internal microsteps): 2000 counts ~= 0.0006 rot, so
// ordinary closed-loop following error trips it almost immediately.
static const int64_t  TINY_DEVIATION = 2000;

// Drain the RS485 RX buffer after an induced-error / reset window.
static void drainSerial() {
#if defined(ARDUINO)
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
#endif
}

// Clean reset: systemReset + settle + drain. Used at start, end, and after any
// deliberately-induced fatal error.
static void cleanReset(Servomotor* motor) {
    motor->systemReset();
    delay(RESET_DELAY_MS);
    drainSerial();
}

// Bring the motor into closed-loop control. enableMosfets, settle ~0.3 s (per
// the enable-commutation-transient note), goToClosedLoop, then wait for the
// closed-loop status bit with no fatal error. Returns true on success.
static bool enterClosedLoop(Servomotor* motor) {
    motor->enableMosfets();
    delay(ENABLE_SETTLE_MS);
    motor->goToClosedLoop();
    unsigned long waited = 0;
    while (waited < CLOSED_LOOP_TIMEOUT_MS) {
        getStatusResponse st = motor->getStatus();
        if (motor->getError() == 0 && st.fatalErrorCode == 0 &&
            (st.statusFlags & CLOSED_LOOP_BIT)) {
            return true;
        }
        delay(100);
        waited += 100;
    }
    return false;
}

// Wait for the motion queue to drain (bounded).
static void drainQueue(Servomotor* motor) {
    unsigned long waited = 0;
    while (waited < DRAIN_TIMEOUT_MS) {
        if (motor->getNQueuedItems() == 0) break;
        delay(20);
        waited += 20;
    }
    delay(200); // let the shaft settle
}

void tm_settings_persistence(void) {
    Serial.println("tm_settings_persistence: BEGIN\n");

    Servomotor* motor = tfGetMotor();
    char buf[200];

    // ---- Clean, known starting state. ----
    cleanReset(motor);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // ========================================================================
    // SECTION A — max velocity / max acceleration revert on reset (readback)
    // ========================================================================
    Serial.println("\n--- Section A: max velocity / acceleration revert ---");

    // Capture the boot defaults straight from the debug snapshot.
    getDebugValuesResponse dBoot = motor->getDebugValues();
    TEST_RESULT("A: getDebugValues (boot) returns no error", motor->getError() == 0);
    int64_t bootMaxVel = dBoot.maxVelocity;
    int64_t bootMaxAcc = dBoot.maxAcceleration;
    snprintf(buf, sizeof(buf), "boot maxVelocity=%lld maxAcceleration=%lld\n",
             (long long)bootMaxVel, (long long)bootMaxAcc);
    Serial.print(buf);
    TEST_RESULT("A: boot maxVelocity is positive", bootMaxVel > 0);
    TEST_RESULT("A: boot maxAcceleration is positive", bootMaxAcc > 0);

    // Choose non-default raw values ~= half the boot default. set_max_velocity
    // shifts the sent raw left by 12, set_max_acceleration by 8; so pre-shift
    // the target to keep the *result* at half and well under the experimental
    // clamp (EXPERIMENTAL = 2x boot for velocity, 4x for acceleration).
    uint32_t rawVel = (uint32_t)((bootMaxVel / 2) >> VELOCITY_SHIFT_LEFT);
    uint32_t rawAcc = (uint32_t)((bootMaxAcc / 2) >> ACCEL_SHIFT_LEFT);
    if (rawVel == 0) rawVel = 1;   // set_max_velocity rejects 0 (fatal); guard
    if (rawAcc == 0) rawAcc = 1;
    int64_t expectVel = (int64_t)((uint64_t)rawVel << VELOCITY_SHIFT_LEFT); // fw:3266
    int64_t expectAcc = (int64_t)((uint64_t)rawAcc << ACCEL_SHIFT_LEFT);    // fw:3285

    motor->setMaximumVelocityRaw(rawVel);
    TEST_RESULT("A: setMaximumVelocityRaw returns no error", motor->getError() == 0);
    motor->setMaximumAccelerationRaw(rawAcc);
    TEST_RESULT("A: setMaximumAccelerationRaw returns no error", motor->getError() == 0);

    getDebugValuesResponse dSet = motor->getDebugValues();
    TEST_RESULT("A: getDebugValues (after set) returns no error", motor->getError() == 0);
    snprintf(buf, sizeof(buf),
             "after set maxVelocity=%lld (expect %lld) maxAcceleration=%lld (expect %lld)\n",
             (long long)dSet.maxVelocity, (long long)expectVel,
             (long long)dSet.maxAcceleration, (long long)expectAcc);
    Serial.print(buf);
    // Exact firmware transform: debug field == (rawSent << shift).
    TEST_RESULT("A: maxVelocity == rawSent<<12 after set", dSet.maxVelocity == expectVel);
    TEST_RESULT("A: maxAcceleration == rawSent<<8 after set", dSet.maxAcceleration == expectAcc);
    // And it genuinely differs from the boot default.
    TEST_RESULT("A: maxVelocity now differs from boot default", dSet.maxVelocity != bootMaxVel);
    TEST_RESULT("A: maxAcceleration now differs from boot default", dSet.maxAcceleration != bootMaxAcc);

    // System reset must drop both RAM settings back to the boot defaults.
    cleanReset(motor);
    getDebugValuesResponse dRev = motor->getDebugValues();
    TEST_RESULT("A: getDebugValues (after reset) returns no error", motor->getError() == 0);
    snprintf(buf, sizeof(buf), "after reset maxVelocity=%lld maxAcceleration=%lld\n",
             (long long)dRev.maxVelocity, (long long)dRev.maxAcceleration);
    Serial.print(buf);
    TEST_RESULT("A: maxVelocity reverted to boot default after reset",
                dRev.maxVelocity == bootMaxVel);
    TEST_RESULT("A: maxAcceleration reverted to boot default after reset",
                dRev.maxAcceleration == bootMaxAcc);

    // ========================================================================
    // SECTION B — max allowable position deviation revert on reset
    // ========================================================================
    Serial.println("\n--- Section B: max allowable position deviation revert ---");

    const int32_t moveTarget = COUNTS_PER_ROTATION / 2;  // 0.5 rot, gentle
    const float   moveDuration = 0.5f;                    // <= 3 rot/s peak

    // B1: With a tiny deviation limit, a normal closed-loop move must latch
    //     fatal 45 (ordinary tracking error exceeds 2000 microsteps at once).
    bool cl1 = enterClosedLoop(motor);
    TEST_RESULT("B1: closed loop engaged (motor calibrated)", cl1);

    bool tripped45 = false;
    uint8_t trippedCode = 0;
    if (cl1) {
        motor->zeroPosition();
        motor->setMaxAllowablePositionDeviationRaw(TINY_DEVIATION); // fw:3444 (RAM, no shift)
        TEST_RESULT("B1: setMaxAllowablePositionDeviationRaw no comms error",
                    motor->getError() == 0);
        motor->trapezoidMove((float)moveTarget, moveDuration);

        unsigned long waited = 0;
        while (waited < DEVIATION_TRIP_TIMEOUT_MS) {
            getStatusResponse st = motor->getStatus();
            if (motor->getError() == 0 && st.fatalErrorCode != 0) {
                tripped45 = true;
                trippedCode = st.fatalErrorCode;
                break;
            }
            delay(50);
            waited += 50;
        }
    }
    snprintf(buf, sizeof(buf), "tiny-limit move latched fatal code=%u (expect 45)\n",
             trippedCode);
    Serial.print(buf);
    // error_codes.json:663-666 -> ERROR_POSITION_DEVIATION_TOO_LARGE == 45.
    TEST_RESULT("B1: tiny deviation limit trips a fatal error", tripped45);
    TEST_RESULT("B1: fatal error is code 45 (position deviation too large)",
                tripped45 && trippedCode == FATAL_POSITION_DEVIATION);

    // Mandatory recovery after a deliberately-induced fatal error.
    cleanReset(motor);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // B2: After reset the default limit (2 rotations) is restored, so the SAME
    //     move completes with no fatal error.
    bool cl2 = enterClosedLoop(motor);
    TEST_RESULT("B2: closed loop re-engaged after reset", cl2);

    bool cleanMove = false;
    if (cl2) {
        motor->zeroPosition();
        motor->trapezoidMove((float)moveTarget, moveDuration);
        drainQueue(motor);
        getStatusResponse st = motor->getStatus();
        snprintf(buf, sizeof(buf), "post-reset same move fatalError=%u\n", st.fatalErrorCode);
        Serial.print(buf);
        cleanMove = (motor->getError() == 0 && st.fatalErrorCode == 0);
    }
    TEST_RESULT("B2: same move is clean after reset (deviation default restored)",
                cleanMove);

    // Return to zero and stand down before section C.
    if (cl2) {
        motor->trapezoidMove(0.0f, moveDuration);
        drainQueue(motor);
    }
    motor->disableMosfets();
    cleanReset(motor);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // ========================================================================
    // SECTION C — PID constants revert on reset (indirect: move must land)
    // ========================================================================
    Serial.println("\n--- Section C: PID constants revert ---");

    // Set the PID constants deliberately weak (RAM only, main.c:1438 restores
    // the strong defaults on reset). We do NOT exercise the weak gains (unsafe
    // / unreliable); we only prove they are gone after reset.
    motor->setPidConstants(1, 0, 0);   // fw motor_control.c:2266 (RAM write)
    TEST_RESULT("C: setPidConstants(weak) returns no error", motor->getError() == 0);

    cleanReset(motor);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    bool cl3 = enterClosedLoop(motor);
    TEST_RESULT("C: closed loop engaged after reset", cl3);

    bool landed = false;
    bool noFatalC = false;
    if (cl3) {
        motor->zeroPosition();
        const int32_t cTarget = COUNTS_PER_ROTATION / 4;  // 0.25 rot, gentle
        motor->trapezoidMove((float)cTarget, moveDuration);
        drainQueue(motor);

        getStatusResponse st = motor->getStatus();
        noFatalC = (motor->getError() == 0 && st.fatalErrorCode == 0);

        // "Lands on target" must be judged on the ACTUAL (hall) position, not
        // the commanded position (which reaches target regardless of PID). With
        // weak gains still active the shaft would fall far short; with the
        // strong defaults restored it settles near target.
        getComprehensivePositionResponse cp = motor->getComprehensivePositionRaw();
        int64_t hall = cp.hallSensorPosition;
        int64_t err = hall - (int64_t)cTarget;
        if (err < 0) err = -err;
        // Tolerance 0.05 rotation: comfortably passes with defaults, fails hard
        // if the weak gains had persisted (shaft would be near 0).
        int64_t tol = COUNTS_PER_ROTATION / 20;
        landed = (err <= tol);
        snprintf(buf, sizeof(buf),
                 "post-reset move: commanded=%lld hall=%lld target=%d err=%lld tol=%lld\n",
                 (long long)cp.commandedPosition, (long long)hall, cTarget,
                 (long long)err, (long long)tol);
        Serial.print(buf);
    }
    TEST_RESULT("C: post-reset closed-loop move has no fatal error", noFatalC);
    TEST_RESULT("C: shaft lands on target (strong PID defaults restored)", landed);

    // ------------------------------------------------------------------------
    // Clean end: back to zero, MOSFETs off, reset, drain.
    // ------------------------------------------------------------------------
    if (cl3) {
        motor->trapezoidMove(0.0f, moveDuration);
        drainQueue(motor);
    }
    motor->disableMosfets();
    cleanReset(motor);

    Serial.println("tm_settings_persistence: END");
}
