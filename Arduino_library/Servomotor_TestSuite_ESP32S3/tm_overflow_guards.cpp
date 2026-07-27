#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_overflow_guards.cpp
//
// Regression tests for the firmware's overflow / zero-guard parameter checks
// (registry BUG-5 / BUG-6, fixed in fw 0.15.4.0). Every offending command must
// latch a CLEAN, DETERMINISTIC fatal error (not a hang, not garbage, not a
// silent success) and the device must fully recover after a System reset.
//
// Ground truth (all codes verified in
//   python_programs/servomotor/error_codes.json  and the firmware dispatch):
//
//   ERROR_PARAMETER_OUT_OF_RANGE == 34  (common_source_files/error_text.h:41)
//   ERROR_PREDICTED_VELOCITY_TOO_HIGH == 28  (error_text.h:35)
//
// PART A - zero-duration move rejections (BUG-6). A move duration of 0 used to
//   be a silent no-op that still returned success; as of 0.15.4.0 each of these
//   raises fatal 34:
//     * Trapezoid move   -> add_trapezoid_move_to_queue(): total_time==0
//                           (firmware/Src/motor_control.c:1944-1945)
//     * Go to position    -> add_go_to_position_to_queue(): move_time==0
//                           (motor_control.c:1966-1967)
//     * Move with velocity -> MOVE_WITH_VELOCITY_COMMAND: time_steps==0
//                           (main.c:616-617)
//     * Move with accel    -> MOVE_WITH_ACCELERATION_COMMAND: time_steps==0
//                           (main.c:535-536)
//
// PART B - zero-limit rejections (BUG-6). A zero maximum silently turns every
//   planned move into a zero-motion dwell (and divides by zero in trapezoid
//   planning). As of 0.15.4.0 each raises fatal 34:
//     * Set maximum velocity     -> set_max_velocity(): new==0
//                                   (motor_control.c:3261-3262)
//     * Set maximum acceleration -> set_max_acceleration(): new==0
//                                   (motor_control.c:3280-3281)
//
// PART C - prediction-overflow guard (BUG-5). A huge acceleration over a very
//   long duration would make the int64 velocity prediction
//   (acceleration<<8) * n_time_steps wrap past the max-velocity safety check,
//   producing garbage motion / a hang. check_queue_item_prediction_overflow()
//   (motor_control.c:1705-1717) now rejects it up front with fatal 28 BEFORE
//   the multiply, when n_time_steps > (INT64_MAX>>1)/|acceleration<<8|.
//
//   We choose RAW parameters that are product-independent:
//     acceleration = 100,000,000  ->  <<8 = 25,600,000,000 (2.56e10)
//     n_time_steps = 3,000,000,000
//   * 2.56e10 is below the boot-default max_acceleration of EVERY product
//     (smallest is the M1 at ~3.37e10, verified from MAX_ACCELERATION in
//     motor_control.h), so the max-acceleration check (ERROR_ACCEL_TOO_HIGH,
//     code 15) does NOT fire first.
//   * limit = (INT64_MAX>>1)/2.56e10 = ~1.80e8; n_time_steps 3e9 far exceeds
//     it, so the overflow guard fires -> fatal 28.
//   * Without the guard, 2.56e10 * 3e9 = 7.68e19 overflows int64 (>9.2e18) and
//     wraps -- exactly the BUG-5 hazard this regression pins down.
//   Raw variants are used so no unit conversion can perturb the values.
//
// SAFETY:
//   * MOSFETs are DISABLED throughout (never enabled). Every case is rejected
//     BEFORE anything is queued, so the shaft never moves and there is no
//     nonzero commanded velocity to leave behind.
//   * No test-mode (cmd 36) values are used anywhere.
//   * The motor is unique-ID addressed via tfGetMotor(), so every command is
//     collision-safe on the 39-motor rack; nothing is broadcast that would
//     solicit responses from multiple devices, and NO raw frames are written to
//     the bus (read-only draining only).
//   * After every deliberately-induced fatal error we System reset, wait, and
//     drain before the next step.
// ---------------------------------------------------------------------------

static const int     NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const uint8_t ERR_PARAM_OUT_OF_RANGE = 34;   // error_codes.json
static const uint8_t ERR_PREDICTED_VEL_HIGH = 28;   // error_codes.json

// Read the latched fatal error code via Get status (answered normally even in
// the fatal-error state). Returns 0xFF on a comms-layer failure so the caller
// can tell "no info" from "code 0".
static uint8_t readFatalErrorCode(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

// Drain any bytes left on the RS485 line.
static void drainBus() {
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}

// System reset + settle + drain: the mandated clean-recovery sequence after a
// deliberately-induced fatal error.
static void recoverClean(Servomotor* motor) {
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no response during reset; don't check errors
    drainBus();
}

// Prove the device is fully responsive at the application level (not hung) by
// round-tripping a ping payload.
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 3 + 2);
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  ping comms error %d\n", motor->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

void tm_overflow_guards(void) {
    Serial.println("test_ard_overflow_guards: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    drainBus();

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);

    // Keep MOSFETs off for the whole module (also the post-reset default).
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");

    uint8_t startFatal = readFatalErrorCode(motor);
    printf("Fatal error code after reset: %d\n", startFatal);
    TEST_RESULT("OG No fatal error after initial reset", startFatal == 0);

    // =======================================================================
    // PART A: zero-duration move rejections -> fatal 34 each; recover each.
    // =======================================================================
    printf("\n--- PART A: zero-duration moves must be rejected (fatal 34) ---\n");

    // A1: Trapezoid move, zero duration.
    motor->trapezoidMove(0.1f, 0.0f);
    {
        uint8_t f = readFatalErrorCode(motor);
        printf("trapezoidMove(0.1, 0.0): fatal=%d (expect 34)\n", f);
        // fatal 34 -> motor_control.c:1944-1945 (add_trapezoid_move_to_queue, total_time==0)
        TEST_RESULT("OG-A trapezoidMove(_,0) latches ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    f == ERR_PARAM_OUT_OF_RANGE);
    }
    recoverClean(motor);
    TEST_RESULT("OG-A recovered after trapezoidMove(_,0)",
                readFatalErrorCode(motor) == 0);

    // A2: Go to position, zero duration.
    motor->goToPosition(0.1f, 0.0f);
    {
        uint8_t f = readFatalErrorCode(motor);
        printf("goToPosition(0.1, 0.0): fatal=%d (expect 34)\n", f);
        // fatal 34 -> motor_control.c:1966-1967 (add_go_to_position_to_queue, move_time==0)
        TEST_RESULT("OG-A goToPosition(_,0) latches ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    f == ERR_PARAM_OUT_OF_RANGE);
    }
    recoverClean(motor);
    TEST_RESULT("OG-A recovered after goToPosition(_,0)",
                readFatalErrorCode(motor) == 0);

    // A3: Move with velocity, zero duration.
    motor->moveWithVelocity(1.0f, 0.0f);
    {
        uint8_t f = readFatalErrorCode(motor);
        printf("moveWithVelocity(1.0, 0.0): fatal=%d (expect 34)\n", f);
        // fatal 34 -> main.c:616-617 (MOVE_WITH_VELOCITY_COMMAND, time_steps==0)
        TEST_RESULT("OG-A moveWithVelocity(_,0) latches ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    f == ERR_PARAM_OUT_OF_RANGE);
    }
    recoverClean(motor);
    TEST_RESULT("OG-A recovered after moveWithVelocity(_,0)",
                readFatalErrorCode(motor) == 0);

    // A4: Move with acceleration, zero duration.
    motor->moveWithAcceleration(1.0f, 0.0f);
    {
        uint8_t f = readFatalErrorCode(motor);
        printf("moveWithAcceleration(1.0, 0.0): fatal=%d (expect 34)\n", f);
        // fatal 34 -> main.c:535-536 (MOVE_WITH_ACCELERATION_COMMAND, time_steps==0)
        TEST_RESULT("OG-A moveWithAcceleration(_,0) latches ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    f == ERR_PARAM_OUT_OF_RANGE);
    }
    recoverClean(motor);
    TEST_RESULT("OG-A recovered after moveWithAcceleration(_,0)",
                readFatalErrorCode(motor) == 0);

    // =======================================================================
    // PART B: zero maximum-limit rejections -> fatal 34 each; recover each.
    // =======================================================================
    printf("\n--- PART B: zero max velocity/acceleration must be rejected (fatal 34) ---\n");

    // B1: Set maximum velocity = 0.
    motor->setMaximumVelocity(0.0f);
    {
        uint8_t f = readFatalErrorCode(motor);
        printf("setMaximumVelocity(0): fatal=%d (expect 34)\n", f);
        // fatal 34 -> motor_control.c:3261-3262 (set_max_velocity, new==0)
        TEST_RESULT("OG-B setMaximumVelocity(0) latches ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    f == ERR_PARAM_OUT_OF_RANGE);
    }
    recoverClean(motor);
    TEST_RESULT("OG-B recovered after setMaximumVelocity(0)",
                readFatalErrorCode(motor) == 0);

    // B2: Set maximum acceleration = 0.
    motor->setMaximumAcceleration(0.0f);
    {
        uint8_t f = readFatalErrorCode(motor);
        printf("setMaximumAcceleration(0): fatal=%d (expect 34)\n", f);
        // fatal 34 -> motor_control.c:3280-3281 (set_max_acceleration, new==0)
        TEST_RESULT("OG-B setMaximumAcceleration(0) latches ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    f == ERR_PARAM_OUT_OF_RANGE);
    }
    recoverClean(motor);
    TEST_RESULT("OG-B recovered after setMaximumAcceleration(0)",
                readFatalErrorCode(motor) == 0);

    // =======================================================================
    // PART C: prediction-overflow guard -> clean fatal 28 (not a hang/garbage).
    // =======================================================================
    printf("\n--- PART C: prediction overflow must be rejected (fatal 28) ---\n");

    // acceleration<<8 = 2.56e10 (below every product's default max_acceleration
    // so ERROR_ACCEL_TOO_HIGH does NOT fire first); n_time_steps 3e9 exceeds the
    // overflow limit ~1.80e8, so check_queue_item_prediction_overflow() fires.
    const int32_t  PATHO_ACCEL      = 100000000;    // raw internal units
    const uint32_t PATHO_TIME_STEPS = 3000000000u;  // raw internal units
    motor->moveWithAccelerationRaw(PATHO_ACCEL, PATHO_TIME_STEPS);
    {
        uint8_t f = readFatalErrorCode(motor);
        printf("moveWithAccelerationRaw(%ld, %lu): fatal=%d (expect 28)\n",
               (long)PATHO_ACCEL, (unsigned long)PATHO_TIME_STEPS, f);
        // fatal 28 -> motor_control.c:1705-1717 (check_queue_item_prediction_overflow)
        TEST_RESULT("OG-C overflow move latches ERROR_PREDICTED_VELOCITY_TOO_HIGH (28)",
                    f == ERR_PREDICTED_VEL_HIGH);
        // The getStatus above already succeeding proves the firmware did not
        // hang on the would-be-overflowing multiply.
        TEST_RESULT("OG-C device answered getStatus (did not hang) after overflow move",
                    f != 0xFF);
    }
    recoverClean(motor);
    TEST_RESULT("OG-C recovered (no fatal error) after overflow move",
                readFatalErrorCode(motor) == 0);
    TEST_RESULT("OG-C device responsive (ping) after overflow recovery",
                pingRoundTrips(motor));

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest (no motion ever occurred).
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    drainBus();
}
