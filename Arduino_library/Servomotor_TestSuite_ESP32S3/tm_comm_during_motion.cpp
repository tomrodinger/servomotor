#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_comm_during_motion.cpp
//
// Arduino-library on-device test for COMMUNICATION UNDER LOAD.
// Mirrors python_programs/test_communication_while_high_speed.py, but GENTLER:
// a modest velocity profile (peak 2.5 rot/s instead of 580 RPM) and a shorter
// total duration, so it is safe on a free shaft and quick in the suite.
//
// Sequence:
//   * enable MOSFETs, zero, cap the motor current for gentleness
//   * queue a full velocity profile UP FRONT (accelerate -> coast -> decelerate
//     -> hold zero), engineered to END AT ZERO VELOCITY
//   * WHILE the queued profile runs, hammer the bus continuously with
//     ping + getStatus + getPositionRaw and assert:
//       - ZERO comm errors across >= 30 mid-motion transactions
//       - every ping payload round-trips unchanged
//       - the MOTOR-BUSY status flag stays CLEAR during motion (firmware
//         ground truth motor_control.c: motor_busy is only set for long
//         actions like calibration/homing, NOT ordinary queued moves)
//       - position increases monotonically during the coast phase
//   * after the queue drains, assert a clean final state (no fatal error,
//     motor no longer busy, net forward displacement) and clean up.
//
// ONE motor on the bus, addressed by unique-ID (extended addressing), so every
// transaction here is single-target and collision-safe even on the 39-motor
// rack. Nothing is broadcast that would elicit a chorus of responses.
//
// SAFETY: the profile is queued so it decelerates back to zero velocity and
// then holds zero (a trailing moveWithVelocity(0,...)); it can never run the
// queue dry at speed (no async fatal 18). Displacement is many rotations by
// nature of a coast test, which is fine on a free shaft.
// ---------------------------------------------------------------------------

static const int     NORMAL_RESET_DELAY_MS = 1500;
static const int64_t POS_NOISE_TOL_COUNTS  = 4000;  // encoder jitter tolerance
static const int     MIN_TRANSACTIONS      = 30;    // spec: >= 30 mid-motion

// Velocity profile (velocity unit = ROTATIONS_PER_SECOND, accel = rot/s^2,
// time unit = SECONDS). Peak = ACCEL * T_ACCEL = 2.5 rot/s. Symmetric ramps
// (T_ACCEL == T_DECEL) guarantee the profile ends at exactly zero velocity.
static const float ACCEL    = 1.25f;  // rot/s^2
static const float T_ACCEL  = 2.0f;   // s  -> ramps 0 -> 2.5 rot/s
static const float T_COAST  = 3.0f;   // s  -> constant 2.5 rot/s
static const float T_DECEL  = 2.0f;   // s  -> ramps 2.5 -> 0 rot/s
// Motion-start-relative coast window (with margin) used for the monotonic check.
static const float COAST_LO = T_ACCEL + 0.5f;              // 2.5 s
static const float COAST_HI = T_ACCEL + T_COAST - 0.5f;    // 4.5 s

// Status-flag bit positions (from common_source_files/device_status.h).
static const uint16_t STATUS_MOSFETS_ENABLED = (1u << 1);
static const uint16_t STATUS_MOTOR_BUSY      = (1u << 6);

void tm_comm_during_motion(void) {
    Serial.println("test_ard_comm_during_motion: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean, known starting state ----
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no responses during reset; don't check errors

    getStatusResponse st0 = motor->getStatus();
    bool startComms = (motor->getError() == 0);
    printf("After reset: commsErr=%d fatalCode=%d flags=0x%04X\n",
           motor->getError(), st0.fatalErrorCode, (unsigned)st0.statusFlags);
    TEST_RESULT("No fatal error after reset", startComms && st0.fatalErrorCode == 0);

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor->setCurrentUnit(CurrentUnit::MILLIAMPS);

    motor->enableMosfets();
    bool enableComms = (motor->getError() == 0);
    TEST_RESULT("enableMosfets returns success (comms ok)", enableComms);
    delay(300);  // let the commutation-alignment transient settle before zeroing

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    // Keep it gentle: cap the motor current well below the default.
    motor->setMaximumMotorCurrent(200.0f, 200.0f);
    checkMotorError(*motor, "setMaximumMotorCurrent");

    int64_t startPos = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw start");
    printf("Start position: %lld counts\n", (long long)startPos);

    // ---- Queue the ENTIRE velocity profile up front (ends at zero velocity) ----
    printf("\nQueuing profile: accel %.2f rot/s^2 for %.1fs (-> %.1f rot/s), "
           "coast %.1fs, decel for %.1fs (-> 0), hold 0.\n",
           ACCEL, T_ACCEL, ACCEL * T_ACCEL, T_COAST, T_DECEL);
    motor->moveWithAcceleration(ACCEL, T_ACCEL);   // 0 -> 2.5 rot/s
    checkMotorError(*motor, "moveWithAcceleration accel");
    motor->moveWithAcceleration(0.0f, T_COAST);    // coast at 2.5 rot/s
    checkMotorError(*motor, "moveWithAcceleration coast");
    motor->moveWithAcceleration(-ACCEL, T_DECEL);  // 2.5 -> 0 rot/s
    checkMotorError(*motor, "moveWithAcceleration decel");
    motor->moveWithVelocity(0.0f, 0.1f);           // explicit zero-velocity hold
    checkMotorError(*motor, "moveWithVelocity hold");

    // ---- Hammer the bus while the profile runs ----
    unsigned long t0 = millis();
    int  transactions   = 0;   // ping + getStatus + getPositionRaw each count
    int  commErrors     = 0;
    int  pingMismatches = 0;
    bool sawBusy        = false;
    bool sawEnabled     = false;

    // Coast-window position samples for the monotonic check.
    int    coastSamples   = 0;
    int    coastViolations = 0;
    int64_t coastFirst    = 0;
    int64_t coastLast     = 0;
    int64_t coastPrev     = 0;
    bool    coastHavePrev = false;

    int loopIter = 0;
    // Bound the loop generously (profile is ~7.2 s; ~15 s cap is plenty).
    while (millis() - t0 < 15000) {
        uint8_t nq = motor->getNQueuedItems();
        if (motor->getError() != 0) commErrors++;
        transactions++;
        if (nq == 0) break;  // profile fully consumed

        // 1) Ping with a per-iteration-varying payload; must round-trip exact.
        uint8_t payload[10];
        for (int i = 0; i < 10; i++)
            payload[i] = (uint8_t)((loopIter * 7 + i * 13) & 0xFF);
        pingResponse pr = motor->ping(payload);
        if (motor->getError() != 0) commErrors++;
        else if (memcmp(pr.responsePayload, payload, 10) != 0) pingMismatches++;
        transactions++;

        // 2) Get status; watch for busy/enabled flags during motion.
        getStatusResponse st = motor->getStatus();
        if (motor->getError() != 0) commErrors++;
        else {
            if (st.statusFlags & STATUS_MOTOR_BUSY)      sawBusy = true;
            if (st.statusFlags & STATUS_MOSFETS_ENABLED) sawEnabled = true;
        }
        transactions++;

        // 3) Get position (raw counts, unit-independent).
        int64_t pos = motor->getPositionRaw();
        bool posOk = (motor->getError() == 0);
        if (!posOk) commErrors++;
        transactions++;

        // Monotonic-during-coast tracking.
        float elapsed = (millis() - t0) / 1000.0f;
        if (posOk && elapsed >= COAST_LO && elapsed <= COAST_HI) {
            if (coastSamples == 0) coastFirst = pos;
            coastLast = pos;
            coastSamples++;
            if (coastHavePrev && pos < coastPrev - POS_NOISE_TOL_COUNTS)
                coastViolations++;
            coastPrev = pos;
            coastHavePrev = true;
        }

        loopIter++;
        delay(10);  // ~matches the Python test's 10 ms inter-ping gap
    }

    unsigned long motionMs = millis() - t0;
    printf("\nMotion window: %lu ms, loops=%d, transactions=%d\n",
           motionMs, loopIter, transactions);
    printf("commErrors=%d pingMismatches=%d sawBusy=%d sawEnabled=%d\n",
           commErrors, pingMismatches, (int)sawBusy, (int)sawEnabled);
    printf("coastSamples=%d coastViolations=%d first=%lld last=%lld\n",
           coastSamples, coastViolations,
           (long long)coastFirst, (long long)coastLast);

    // ---- Assertions on the mid-motion communication ----
    TEST_RESULT("At least 30 mid-motion transactions",
                transactions >= MIN_TRANSACTIONS);
    TEST_RESULT("Zero comm errors during motion", commErrors == 0);
    TEST_RESULT("All ping payloads round-tripped during motion",
                pingMismatches == 0);
    // Firmware ground truth (motor_control.c ~line 202): motor_busy is set only
    // during long actions (calibration/homing), NOT during ordinary queued
    // moves — so the busy bit must stay CLEAR throughout this profile.
    TEST_RESULT("Motor-busy flag stays clear during ordinary motion", !sawBusy);
    TEST_RESULT("MOSFETs-enabled flag observed during motion", sawEnabled);

    // ---- Monotonic position during the coast phase ----
    TEST_RESULT("Coast phase sampled (position data captured)",
                coastSamples >= 3);
    TEST_RESULT("Position monotonic (non-decreasing) during coast",
                coastViolations == 0);
    TEST_RESULT("Position advanced forward during coast",
                (coastLast - coastFirst) > POS_NOISE_TOL_COUNTS);

    // ---- Let any residual queue drain, then verify a clean final state ----
    for (int i = 0; i < 300; i++) {
        if (motor->getNQueuedItems() == 0) break;
        delay(10);
    }
    delay(300);  // settle

    getStatusResponse stF = motor->getStatus();
    bool finalComms = (motor->getError() == 0);
    printf("\nFinal: commsErr=%d fatalCode=%d flags=0x%04X\n",
           motor->getError(), stF.fatalErrorCode, (unsigned)stF.statusFlags);
    TEST_RESULT("Clean final state (comms ok, no fatal error)",
                finalComms && stF.fatalErrorCode == 0);
    TEST_RESULT("Motor no longer busy at end of profile",
                finalComms && (stF.statusFlags & STATUS_MOTOR_BUSY) == 0);
    TEST_RESULT("Queue fully drained at end",
                motor->getNQueuedItems() == 0);

    int64_t finalPos = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw final");
    printf("Final position: %lld counts (net %lld from start)\n",
           (long long)finalPos, (long long)(finalPos - startPos));
    TEST_RESULT("Net forward displacement over the profile",
                (finalPos - startPos) > POS_NOISE_TOL_COUNTS);

    // ---- Clean up: at rest, MOSFETs off, reset ----
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
}
