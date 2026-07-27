#include "tf_framework.h"

// ============================================================================
// tm_settings.cpp
//
// Arduino-library mirror of four Python per-command tests:
//   - test_set_maximum_velocity.py                (cmd 3  Set maximum velocity)
//   - test_set_maximum_acceleration.py            (cmd 5  Set maximum acceleration)
//   - test_set_maximum_motor_current.py           (cmd 28 Set maximum motor current)
//   - test_set_max_allowable_position_deviation.py(cmd 44 Set max allowable position deviation)
//
// These are all setters. Their observable behaviour is verified deterministically
// (no torque / velocity measurement): a too-low cap makes a subsequent move be
// REJECTED with a specific firmware error code; a generous cap lets an ordinary
// move complete cleanly.
//
// How firmware errors surface in the Arduino library:
//   When the device rejects a command it replies with an error packet whose
//   error-code byte is non-zero; Communication::getResponse() returns that code
//   as a positive value, which the caller sees via motor->getError() right after
//   the command. A latched fatal error is also readable through
//   getStatus().fatalErrorCode. The position-deviation trip (cmd 44) is raised
//   ASYNCHRONOUSLY inside the motor-control loop, so it is polled for.
//
// SAFETY: every velocity move is followed by moveWithVelocity(0,...) and every
// acceleration ramp-up gets a symmetric ramp-down + a zero-velocity hold, so the
// motor is always at rest before the queue empties (avoids fatal error 18). Every
// rejection latches a fatal error, so we systemReset() to recover between phases.
// Currents are never lowered to a stalling value; the working move uses 200.
// ============================================================================

// Firmware error codes (from common_source_files/error_text.h)
static const int ERROR_ACCEL_TOO_HIGH            = 15;
static const int ERROR_VEL_TOO_HIGH              = 16;
static const int ERROR_MAX_PWM_VOLTAGE_TOO_HIGH  = 23;
static const int ERROR_PARAMETER_OUT_OF_RANGE    = 34;
static const int ERROR_POSITION_DEVIATION_TOO_LARGE = 45;

static const int RESET_DELAY_MS = 1500;
static const float MOVE_TOL_COUNTS = 150.0f;   // tolerance for a real hardware move

// Return the firmware fatal-error code currently observable on the device.
// A freshly-rejected command makes getResponse() return the remote error code
// (positive), which surfaces via getError(). Otherwise a normal getStatus() reply
// carries the latched fatalErrorCode (0 if none).
static int currentFatal(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    int commErr = motor->getError();
    if (commErr > 0) return commErr;   // device replied with an error packet carrying the code
    return (int)st.fatalErrorCode;     // normal status reply carries the latched fatal code
}

// Poll for an asynchronous fatal-error trip (cmd 44 deviation check runs in the
// control loop). Returns true as soon as the expected code is observed.
static bool waitForFatal(Servomotor* motor, int expectedCode, int timeoutMs) {
    int elapsed = 0;
    while (elapsed < timeoutMs) {
        if (currentFatal(motor) == expectedCode) return true;
        delay(50);
        elapsed += 50;
    }
    return false;
}

static void recover(Servomotor* motor) {
    motor->systemReset();
    delay(RESET_DELAY_MS);
}

static void drainQueue(Servomotor* motor) {
    int guard = 0;
    while (motor->getNQueuedItems() > 0 && guard < 500) {
        delay(10);
        guard++;
    }
}

void tm_settings(void) {
    Serial.println("test_ard_settings: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // Clean, known state.
    motor->systemReset();
    delay(RESET_DELAY_MS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // =====================================================================
    // Command 3: Set maximum velocity
    // =====================================================================
    printf("\n=== Command 3: Set maximum velocity ===\n");

    // Scenario A: a LOW velocity cap rejects an over-cap velocity move.
    // A high acceleration cap is set first so velocity, not acceleration, is the limiter.
    printf("Scenario A: low cap (0.5) must reject moveWithVelocity(5.0) with code %d...\n", ERROR_VEL_TOO_HIGH);
    motor->enableMosfets();
    motor->zeroPosition();
    motor->setMaximumAcceleration(100.0f);
    motor->setMaximumVelocity(0.5f);
    motor->moveWithVelocity(5.0f, 1.0f);   // 10x the cap -> must be rejected
    int velImmErr = motor->getError();
    int velObserved = (velImmErr == ERROR_VEL_TOO_HIGH) ? velImmErr : currentFatal(motor);
    printf("  immediate getError=%d, observed fatal=%d\n", velImmErr, velObserved);
    TEST_RESULT("Cmd3 over-cap velocity rejected with ERROR_VEL_TOO_HIGH(16)",
                velObserved == ERROR_VEL_TOO_HIGH);
    recover(motor);

    // Scenario B: a HIGH velocity cap accepts an under-cap velocity move.
    printf("Scenario B: high cap (10) must accept moveWithVelocity(1.0)...\n");
    motor->enableMosfets();
    motor->zeroPosition();
    motor->setMaximumAcceleration(100.0f);
    motor->setMaximumVelocity(10.0f);
    motor->moveWithVelocity(1.0f, 1.0f);
    int velAcceptErr = motor->getError();
    motor->moveWithVelocity(0.0f, 0.5f);   // stop so the motor is at rest before the queue empties
    delay(1000 + 500 + 300);
    drainQueue(motor);
    int velAfter = currentFatal(motor);
    printf("  accept getError=%d, fatal after move=%d\n", velAcceptErr, velAfter);
    TEST_RESULT("Cmd3 under-cap velocity accepted, no fatal error",
                velAcceptErr == 0 && velAfter == 0);
    recover(motor);

    // Scenario C: a cap of 0 is rejected with ERROR_PARAMETER_OUT_OF_RANGE (BUG-21,
    // fixed in fw 0.15.6.0). Bench firmware is newer, so this must pass.
    printf("Scenario C: setMaximumVelocity(0) must be rejected with code %d...\n", ERROR_PARAMETER_OUT_OF_RANGE);
    motor->enableMosfets();
    motor->zeroPosition();
    motor->setMaximumVelocity(0.0f);
    int zeroImmErr = motor->getError();
    int zeroObserved = (zeroImmErr == ERROR_PARAMETER_OUT_OF_RANGE) ? zeroImmErr : currentFatal(motor);
    printf("  immediate getError=%d, observed=%d\n", zeroImmErr, zeroObserved);
    TEST_RESULT("Cmd3 setMaximumVelocity(0) rejected with ERROR_PARAMETER_OUT_OF_RANGE(34)",
                zeroObserved == ERROR_PARAMETER_OUT_OF_RANGE);
    recover(motor);

    // =====================================================================
    // Command 5: Set maximum acceleration
    // =====================================================================
    printf("\n=== Command 5: Set maximum acceleration ===\n");

    // Scenario A: a LOW acceleration cap rejects an over-cap acceleration move.
    // A high velocity cap is set first so acceleration is the limiter.
    printf("Scenario A: low cap (1.0) must reject moveWithAcceleration(50.0) with code %d...\n", ERROR_ACCEL_TOO_HIGH);
    motor->enableMosfets();
    motor->zeroPosition();
    motor->setMaximumVelocity(20.0f);
    motor->setMaximumAcceleration(1.0f);
    motor->moveWithAcceleration(50.0f, 0.5f);   // well above the cap -> must be rejected
    int accImmErr = motor->getError();
    int accObserved = (accImmErr == ERROR_ACCEL_TOO_HIGH) ? accImmErr : currentFatal(motor);
    printf("  immediate getError=%d, observed fatal=%d\n", accImmErr, accObserved);
    TEST_RESULT("Cmd5 over-cap acceleration rejected with ERROR_ACCEL_TOO_HIGH(15)",
                accObserved == ERROR_ACCEL_TOO_HIGH);
    recover(motor);

    // Scenario B: a HIGH acceleration cap accepts an under-cap acceleration move.
    // Ramp up, symmetric ramp down, then hold at zero velocity so the motor is at rest.
    printf("Scenario B: high cap (100) must accept moveWithAcceleration(2.0) ramp...\n");
    motor->enableMosfets();
    motor->zeroPosition();
    motor->setMaximumVelocity(20.0f);
    motor->setMaximumAcceleration(100.0f);
    motor->moveWithAcceleration(2.0f, 0.5f);    // ramp velocity up to ~1.0 rot/s
    int accAcceptErr = motor->getError();
    motor->moveWithAcceleration(-2.0f, 0.5f);   // symmetric ramp back to ~0
    motor->moveWithVelocity(0.0f, 0.1f);        // hold at rest
    delay(500 + 500 + 100 + 300);
    drainQueue(motor);
    int accAfter = currentFatal(motor);
    printf("  accept getError=%d, fatal after move=%d\n", accAcceptErr, accAfter);
    TEST_RESULT("Cmd5 under-cap acceleration accepted, no fatal error",
                accAcceptErr == 0 && accAfter == 0);
    recover(motor);

    // =====================================================================
    // Command 28: Set maximum motor current
    // =====================================================================
    printf("\n=== Command 28: Set maximum motor current ===\n");
    motor->setCurrentUnit(CurrentUnit::INTERNAL_CURRENT_UNITS);  // raw integer values pass straight through
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);

    // A spread of valid working currents; each must be accepted with no fatal error.
    printf("Setting valid currents 50/150/200/0; each must be accepted...\n");
    const float validCurrents[] = {50.0f, 150.0f, 200.0f, 0.0f};
    bool allCurrentsAccepted = true;
    for (int i = 0; i < 4; i++) {
        motor->setMaximumMotorCurrent(validCurrents[i], validCurrents[i]);
        int e = motor->getError();
        int f = currentFatal(motor);
        printf("  current=%.0f: getError=%d fatal=%d\n", validCurrents[i], e, f);
        if (e != 0 || f != 0) allCurrentsAccepted = false;
    }
    TEST_RESULT("Cmd28 valid current values accepted, no fatal error", allCurrentsAccepted);

    // An excessively high current maps above the safe PWM voltage and must be rejected.
    printf("Setting excessive current 65535; must be rejected with code %d...\n", ERROR_MAX_PWM_VOLTAGE_TOO_HIGH);
    motor->setMaximumMotorCurrent(65535.0f, 65535.0f);
    int curImmErr = motor->getError();
    int curObserved = (curImmErr == ERROR_MAX_PWM_VOLTAGE_TOO_HIGH) ? curImmErr : currentFatal(motor);
    printf("  immediate getError=%d, observed=%d\n", curImmErr, curObserved);
    TEST_RESULT("Cmd28 excessive current rejected with ERROR_MAX_PWM_VOLTAGE_TOO_HIGH(23)",
                curObserved == ERROR_MAX_PWM_VOLTAGE_TOO_HIGH);
    recover(motor);

    // With a normal working current the motor still moves and lands on target.
    printf("Setting working current 200 and running a quarter-rotation move...\n");
    motor->setMaximumMotorCurrent(200.0f, 200.0f);
    motor->enableMosfets();
    motor->zeroPosition();
    float curTarget = (float)(COUNTS_PER_REVOLUTION / 4);
    motor->trapezoidMove(curTarget, 1.0f);
    delay(1200);
    drainQueue(motor);
    float curPos = motor->getPosition();
    int curMoveFatal = currentFatal(motor);
    printf("  move ended at %.0f (target %.0f), fatal=%d\n", curPos, curTarget, curMoveFatal);
    TEST_RESULT("Cmd28 working-current move lands on target with no fatal error",
                fabs(curPos - curTarget) < MOVE_TOL_COUNTS && curMoveFatal == 0);
    motor->trapezoidMove(0.0f, 1.0f);   // return to zero (ends at rest)
    delay(1200);
    drainQueue(motor);
    motor->disableMosfets();
    recover(motor);

    // =====================================================================
    // Command 44: Set max allowable position deviation
    // =====================================================================
    printf("\n=== Command 44: Set max allowable position deviation ===\n");
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);

    // Phase 1: a GENEROUS limit lets an ordinary move complete cleanly.
    printf("Phase 1: generous limit (one rotation) must let an ordinary move complete...\n");
    motor->setMaxAllowablePositionDeviation((float)COUNTS_PER_REVOLUTION);
    motor->enableMosfets();
    delay(300);   // let the commutation-alignment transient settle before zeroing
    motor->zeroPosition();
    float devTarget = (float)(COUNTS_PER_REVOLUTION / 4);
    motor->trapezoidMove(devTarget, 1.0f);
    delay(1200);
    drainQueue(motor);
    float devPos = motor->getPosition();
    int devFatal1 = currentFatal(motor);
    printf("  move ended at %.0f (target %.0f), fatal=%d\n", devPos, devTarget, devFatal1);
    TEST_RESULT("Cmd44 generous limit: ordinary move completes with no fatal error",
                devFatal1 == 0);
    TEST_RESULT("Cmd44 generous limit: move lands on target",
                fabs(devPos - devTarget) < MOVE_TOL_COUNTS);
    motor->trapezoidMove(0.0f, 1.0f);
    delay(1200);
    drainQueue(motor);
    motor->disableMosfets();
    recover(motor);

    // Phase 2: a TIGHT positive limit traps a fast move with code 45 (async trip).
    // Enable + zero first (under the default generous limit), THEN set the tight
    // limit, so we don't trip on the MOSFET-enable alignment transient.
    printf("Phase 2: tight positive limit (2000) must trap a fast move with code %d...\n", ERROR_POSITION_DEVIATION_TOO_LARGE);
    motor->enableMosfets();
    delay(300);
    motor->zeroPosition();
    motor->setMaxAllowablePositionDeviation(2000.0f);
    motor->trapezoidMove((float)(COUNTS_PER_REVOLUTION / 2), 0.3f);  // aggressive: desired races ahead of hall
    bool trip2 = (motor->getError() == ERROR_POSITION_DEVIATION_TOO_LARGE)
                 || waitForFatal(motor, ERROR_POSITION_DEVIATION_TOO_LARGE, 4000);
    printf("  deviation trip observed = %s\n", trip2 ? "yes" : "no");
    TEST_RESULT("Cmd44 tight positive limit: fast move trips ERROR_POSITION_DEVIATION_TOO_LARGE(45)",
                trip2);
    recover(motor);

    // Phase 3: a NEGATIVE tight limit behaves like |value| (firmware folds with llabs)
    // and traps the same fast move with code 45.
    printf("Phase 3: tight NEGATIVE limit (-2000) must fold to abs and trip code %d...\n", ERROR_POSITION_DEVIATION_TOO_LARGE);
    motor->enableMosfets();
    delay(300);
    motor->zeroPosition();
    motor->setMaxAllowablePositionDeviation(-2000.0f);
    motor->trapezoidMove((float)(COUNTS_PER_REVOLUTION / 2), 0.3f);
    bool trip3 = (motor->getError() == ERROR_POSITION_DEVIATION_TOO_LARGE)
                 || waitForFatal(motor, ERROR_POSITION_DEVIATION_TOO_LARGE, 4000);
    printf("  deviation trip observed = %s\n", trip3 ? "yes" : "no");
    TEST_RESULT("Cmd44 negative tight limit folds to abs and trips code 45",
                trip3);
    recover(motor);

    // Final cleanup: leave the device reset and de-energised.
    motor->disableMosfets();
    motor->systemReset();
    delay(RESET_DELAY_MS);
}
