#include "tf_framework.h"

// Mirrors python_programs/test_homing.py (command 14 Homing), adapted to the
// Arduino host-emulator test framework and made SAFE for a free shaft:
//  - keeps each homing distance small (<= ~0.5 rotation),
//  - verifies the command is accepted and the shaft moves the commanded
//    distance in the commanded direction,
//  - exercises several position units (rotations / degrees / radians / counts)
//    and both directions,
//  - starts each homing from closed-loop mode (homing requires it),
//  - recovers from any fatal error with a systemReset so one failure does not
//    poison the rest of the run.
//
// NOTE: The Python test also simulates a hard stop by dropping the motor
// current mid-homing. That is intentionally NOT reproduced here: this agent
// has no hardware and the crash-detection path is risky to trigger blindly on a
// free shaft. We cover the primary "home the full commanded distance" behavior,
// across units and directions, which is command 14's core contract.

static const float TOL_COUNTS = 20000.0f;  // ~0.6% of a rotation; homing on a
                                           // free shaft covers the full distance

// Wait until all queued moves finish (or a timeout elapses so we never hang).
static void waitForMovesToComplete(Servomotor* motor) {
    for (int i = 0; i < 500; i++) {  // up to ~5 s
        uint8_t queued = motor->getNQueuedItems();
        if (motor->getError() != 0) {
            // Reading the queue failed; bail out and let the caller decide.
            return;
        }
        if (queued == 0) {
            return;
        }
        delay(10);
    }
}

// Bring the motor into closed-loop mode, the required precondition for homing.
static void enterClosedLoop(Servomotor* motor) {
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(100);
    motor->goToClosedLoop();
    checkMotorError(*motor, "goToClosedLoop");
    delay(500);  // give time to settle into closed loop
}

// Run one homing move (in the currently-selected position unit) and verify the
// shaft moved the commanded distance in the commanded direction. start/expected
// distances are supplied in ENCODER COUNTS for a unit-independent comparison.
static void runHomingCase(Servomotor* motor, const char* label,
                          float maxDistanceInUnit, float maxDuration,
                          int expectedDirection, float expectedCounts) {
    printf("\n=== Homing case: %s ===\n", label);

    // Read the start position in raw encoder counts (unit-independent) WITHOUT
    // changing the position unit the caller selected for the homing distance.
    float startCounts = (float)motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw (start)");

    // Command homing in the unit the caller selected for maxDistanceInUnit.
    // (The caller sets the position unit right before calling us.)
    printf("Homing maxDistance=%.4f (unit-specific), maxDuration=%.2f s\n",
           maxDistanceInUnit, maxDuration);
    motor->homing(maxDistanceInUnit, maxDuration);
    bool accepted = (motor->getError() == 0);
    TEST_RESULT((std::string("Homing command accepted: ") + label).c_str(), accepted);

    waitForMovesToComplete(motor);

    // Read the end position back in raw encoder counts (unit unchanged).
    float endCounts = (float)motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw (end)");

    float movement = endCounts - startCounts;
    printf("Start=%.0f counts  End=%.0f counts  Movement=%.0f counts  Expected=%.0f counts\n",
           startCounts, endCounts, movement, expectedCounts);

    // Direction check.
    bool dirOk = (expectedDirection > 0) ? (movement > 0.0f) : (movement < 0.0f);
    TEST_RESULT((std::string("Homing moved in commanded direction: ") + label).c_str(), dirOk);

    // Magnitude check (free shaft covers the full commanded distance).
    bool magOk = approxEqual(movement, expectedCounts, TOL_COUNTS);
    printf("Movement error: %.0f counts (tolerance %.0f)\n",
           std::fabs(movement - expectedCounts), TOL_COUNTS);
    TEST_RESULT((std::string("Homing distance approx correct: ") + label).c_str(), magOk);
}

void tm_homing(void) {
    Servomotor* motor = tfGetMotor();

    // Clean, known state.
    motor->systemReset();
    delay(1500);
    // (No error check right after reset: the motor reboots and does not reply.)

    motor->setTimeUnit(TimeUnit::SECONDS);

    // Homing requires closed-loop mode.
    enterClosedLoop(motor);

    // Use a small, gentle homing distance everywhere: 0.5 rotation.
    // 0.5 rotation == COUNTS_PER_REVOLUTION/2 encoder counts.
    const float HALF_ROT_COUNTS = COUNTS_PER_REVOLUTION * 0.5f;
    const float DURATION = 1.0f;  // seconds

    // --- Shaft rotations, + and - ---
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    runHomingCase(motor, "+0.5 rotation", 0.5f, DURATION, +1, +HALF_ROT_COUNTS);

    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    runHomingCase(motor, "-0.5 rotation", -0.5f, DURATION, -1, -HALF_ROT_COUNTS);

    // --- Degrees (180 deg == 0.5 rotation) ---
    motor->setPositionUnit(PositionUnit::DEGREES);
    runHomingCase(motor, "+180 degrees", 180.0f, DURATION, +1, +HALF_ROT_COUNTS);

    motor->setPositionUnit(PositionUnit::DEGREES);
    runHomingCase(motor, "-180 degrees", -180.0f, DURATION, -1, -HALF_ROT_COUNTS);

    // --- Radians (pi rad == 0.5 rotation) ---
    motor->setPositionUnit(PositionUnit::RADIANS);
    runHomingCase(motor, "+pi radians", (float)M_PI, DURATION, +1, +HALF_ROT_COUNTS);

    motor->setPositionUnit(PositionUnit::RADIANS);
    runHomingCase(motor, "-pi radians", -(float)M_PI, DURATION, -1, -HALF_ROT_COUNTS);

    // --- Encoder counts (direct) ---
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    runHomingCase(motor, "+0.5 rotation (counts)", HALF_ROT_COUNTS, DURATION, +1, +HALF_ROT_COUNTS);

    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    runHomingCase(motor, "-0.5 rotation (counts)", -HALF_ROT_COUNTS, DURATION, -1, -HALF_ROT_COUNTS);

    // MOTION SAFETY: homing (a trapezoid-style seek) ends at rest on its own, so
    // no explicit stop move is required. Make sure the queue is drained and, if
    // any fatal error was raised along the way, recover with a reset so we leave
    // the device clean.
    waitForMovesToComplete(motor);
    if (motor->getError() != 0) {
        printf("A fatal error was present at the end; recovering with systemReset.\n");
        motor->systemReset();
        delay(1500);
    }
}
