#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_safety_fence.cpp
//
// Arduino-library on-device deep-dive test for "Set safety limits" (cmd 30)
// and its interplay with "Zero position" (cmd 13).
// Mirrors python_programs/test_set_safety_limits.py, extended with the
// documented zeroing interplay.
//
// What a safety fence does (verified against firmware/Src/motor_control.c):
//   * The lower/upper limits are stored in the SAME internal position frame
//     that "Zero position" resets to 0. The limits themselves are NOT shifted
//     by zeroing (set_movement_limits just stores them; zero_position only
//     zeroes the position counters). So after zeroing, a limit numerically
//     unchanged is now measured relative to the NEW origin -- the fence
//     "follows" the zeroed frame.
//   * When a move is queued, the firmware predicts the end position (and, for
//     acceleration moves, the turn point). If either would leave [lower,upper]
//     the move is rejected before executing with a fatal error:
//        25 ERROR_SAFETY_LIMIT_EXCEEDED (real-time enforcement)
//        26 ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE (queue-time prediction)
//        27 ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE (queue-time prediction)
//     Any of these is an acceptable "fence tripped" outcome.
//
// SAFETY / MULTI-MOTOR NOTES:
//   * The MOSFETs stay DISABLED for the whole test (as in the Python test):
//     the safety check acts on the COMMANDED / predicted trajectory at
//     queue-time, so no closed-loop motion is needed. This avoids any physical
//     motion, is collision-safe, and needs no calibration. Commanded excursions
//     are kept well under the 2-rotation default deviation limit so error 45
//     never fires.
//   * tfGetMotor() is unique-ID addressed; systemReset() targets that device
//     and elicits no response, so this is bus-collision-safe.
//   * No test-mode command is used anywhere here.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;  // plain reset
static const int FATAL_RESET_DELAY_MS  = 2000;  // recovering from a fatal error (avoid bootloader pin)

static const uint8_t ERR_SAFETY_LIMIT_EXCEEDED        = 25;
static const uint8_t ERR_TURN_POINT_OUT_OF_SAFETY     = 26;
static const uint8_t ERR_PREDICTED_POS_OUT_OF_SAFETY  = 27;

static const float NARROW_LIMIT_ROT = 0.5f;   // +/- 0.5 shaft rotations
static const float WIDE_LIMIT_ROT   = 1000.0f;

// Read the device's fatal error code via Get status (answered normally even in
// the fatal-error state). Returns 0xFF if the query itself failed at comms.
static uint8_t readFatalErrorCode(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

static bool isSafetyZoneCode(uint8_t code) {
    return code == ERR_SAFETY_LIMIT_EXCEEDED ||
           code == ERR_TURN_POINT_OUT_OF_SAFETY ||
           code == ERR_PREDICTED_POS_OUT_OF_SAFETY;
}

// Drain the motion queue (moves execute open-loop even with MOSFETs disabled),
// then settle briefly. Predictively-rejected moves are never queued, so this
// returns almost immediately for them.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 500; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (motor->getError() != 0) break;
        if (n == 0) break;
        delay(10);
    }
    delay(150);
}

void tm_safety_fence(void) {
    Serial.println("test_ard_safety_fence: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // =====================================================================
    // PART A: narrow fence accepts an in-bounds move and rejects an
    // out-of-bounds move. (mirrors test_set_safety_limits.py scenario A)
    // =====================================================================
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    uint8_t a0 = readFatalErrorCode(motor);
    printf("A0: fatal after reset = %d\n", a0);
    TEST_RESULT("A0 No fatal error after reset", a0 == 0);

    printf("\n--- A: narrow fence [%.1f, +%.1f] rot ---\n",
           -NARROW_LIMIT_ROT, NARROW_LIMIT_ROT);
    motor->setSafetyLimits(-NARROW_LIMIT_ROT, NARROW_LIMIT_ROT);
    bool aSetComms = (motor->getError() == 0);
    uint8_t aSetFatal = readFatalErrorCode(motor);
    TEST_RESULT("A1 setSafetyLimits(narrow) no comms error", aSetComms);
    TEST_RESULT("A1 setSafetyLimits(narrow) no fatal", aSetFatal == 0);

    // In-bounds: +0.3 rot lands inside +/-0.5 -> accepted.
    printf("A2: in-bounds goToPosition(+0.3 rot)\n");
    motor->goToPosition(0.3f, 1.0f);
    bool a2Comms = (motor->getError() == 0);
    waitForIdle(motor);
    uint8_t a2Fatal = readFatalErrorCode(motor);
    printf("   comms=%d fatal=%d\n", a2Comms ? 0 : motor->getError(), a2Fatal);
    TEST_RESULT("A2 In-bounds move (+0.3 rot) accepted under narrow fence",
                a2Comms && a2Fatal == 0);

    // Return to origin before the out-of-bounds attempt.
    motor->goToPosition(0.0f, 1.0f);
    checkMotorError(*motor, "goToPosition A2 return");
    waitForIdle(motor);

    // Out-of-bounds: +5 rot predicted end is 10x the fence -> rejected.
    printf("A3: out-of-bounds goToPosition(+5 rot) must trip the fence\n");
    motor->goToPosition(5.0f, 2.0f);
    delay(200);
    uint8_t a3Fatal = readFatalErrorCode(motor);
    printf("   fatal=%d (expect one of 25/26/27)\n", a3Fatal);
    TEST_RESULT("A3 Out-of-bounds move trips safety-zone fatal (25/26/27)",
                isSafetyZoneCode(a3Fatal));

    // =====================================================================
    // PART B: the fence follows the zeroed frame. Establish the same
    // +/-0.5 fence, move mid-range, zeroPosition there, and show that the
    // fence is now measured relative to the NEW origin.
    // =====================================================================
    motor->systemReset();               // clears the Part-A fatal
    delay(FATAL_RESET_DELAY_MS);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    uint8_t b0 = readFatalErrorCode(motor);
    printf("\n--- B: zeroing interplay ---\nB0: fatal after reset = %d\n", b0);
    TEST_RESULT("B0 No fatal error after reset (before zeroing test)", b0 == 0);

    motor->setSafetyLimits(-NARROW_LIMIT_ROT, NARROW_LIMIT_ROT);
    checkMotorError(*motor, "setSafetyLimits B");

    // Move to +0.4 rot: inside +/-0.5, legal in the ORIGINAL frame.
    printf("B1: goToPosition(+0.4 rot) inside old fence\n");
    motor->goToPosition(0.4f, 1.0f);
    bool b1Comms = (motor->getError() == 0);
    waitForIdle(motor);
    uint8_t b1Fatal = readFatalErrorCode(motor);
    TEST_RESULT("B1 In-bounds move to +0.4 rot accepted (old frame)",
                b1Comms && b1Fatal == 0);

    // Zero the origin here. Commanded position -> 0; fence numbers unchanged,
    // so the fence is now centered on this new origin (which is old +0.4).
    printf("B2: zeroPosition mid-range (new origin == old +0.4 rot)\n");
    motor->zeroPosition();
    bool b2Comms = (motor->getError() == 0);
    uint8_t b2Fatal = readFatalErrorCode(motor);
    TEST_RESULT("B2 zeroPosition mid-range leaves no fatal",
                b2Comms && b2Fatal == 0);

    // New-frame +0.4 == old +0.8, which was OUTSIDE the old fence, yet is
    // inside the fence in the NEW frame -> accepted. Proves the fence moved
    // with the zero.
    printf("B3: goToPosition(+0.4 new frame) == old +0.8 (old-illegal) must be accepted\n");
    motor->goToPosition(0.4f, 1.0f);
    bool b3Comms = (motor->getError() == 0);
    waitForIdle(motor);
    uint8_t b3Fatal = readFatalErrorCode(motor);
    printf("   comms=%d fatal=%d\n", b3Comms ? 0 : motor->getError(), b3Fatal);
    TEST_RESULT("B3 Move now-legal in new frame accepted (was outside old fence)",
                b3Comms && b3Fatal == 0);

    // Return to the new origin, then attempt new-frame -0.8 == old -0.4, which
    // WAS legal in the old fence but is outside the new fence -> must trip.
    motor->goToPosition(0.0f, 1.0f);
    checkMotorError(*motor, "goToPosition B3 return");
    waitForIdle(motor);

    printf("B4: goToPosition(-0.8 new frame) == old -0.4 (old-legal) must trip new fence\n");
    motor->goToPosition(-0.8f, 2.0f);
    delay(200);
    uint8_t b4Fatal = readFatalErrorCode(motor);
    printf("   fatal=%d (expect one of 25/26/27)\n", b4Fatal);
    TEST_RESULT("B4 Move legal in OLD frame trips NEW fence (fence follows zero)",
                isSafetyZoneCode(b4Fatal));

    // =====================================================================
    // PART C: restore wide limits; a normal move that would have exceeded
    // the narrow fence is now accepted. (mirrors scenario B of the Python test)
    // =====================================================================
    motor->systemReset();               // clears the Part-B fatal
    delay(FATAL_RESET_DELAY_MS);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    uint8_t c0 = readFatalErrorCode(motor);
    printf("\n--- C: wide fence [%.0f, +%.0f] rot ---\nC0: fatal after reset = %d\n",
           -WIDE_LIMIT_ROT, WIDE_LIMIT_ROT, c0);
    TEST_RESULT("C0 No fatal error after reset (before wide-limits test)", c0 == 0);

    motor->setSafetyLimits(-WIDE_LIMIT_ROT, WIDE_LIMIT_ROT);
    bool cSetComms = (motor->getError() == 0);
    uint8_t cSetFatal = readFatalErrorCode(motor);
    TEST_RESULT("C1 setSafetyLimits(wide) accepted (no comms/fatal)",
                cSetComms && cSetFatal == 0);

    // +0.8 rot would have exceeded the narrow +/-0.5 fence; under wide limits
    // it is accepted.
    printf("C2: goToPosition(+0.8 rot) (would exceed old narrow fence) must be accepted\n");
    motor->goToPosition(0.8f, 2.0f);
    bool c2Comms = (motor->getError() == 0);
    waitForIdle(motor);
    uint8_t c2Fatal = readFatalErrorCode(motor);
    printf("   comms=%d fatal=%d\n", c2Comms ? 0 : motor->getError(), c2Fatal);
    TEST_RESULT("C2 Normal move under wide limits accepted (exceeds old narrow fence)",
                c2Comms && c2Fatal == 0);

    // Return to origin so we leave at rest.
    motor->goToPosition(0.0f, 1.0f);
    checkMotorError(*motor, "goToPosition C2 return");
    waitForIdle(motor);

    // ---- Clean up: MOSFETs already disabled; reset to power-on defaults. ----
    motor->disableMosfets();
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    uint8_t cEnd = readFatalErrorCode(motor);
    TEST_RESULT("C3 Device clean (no fatal) after final reset", cEnd == 0);
}
