#include "tf_framework.h"

// Safety-zone fatal error codes the firmware can raise for an out-of-zone move.
static const int ERROR_SAFETY_LIMIT_EXCEEDED = 25;
static const int ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE = 26;
static const int ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE = 27;

static bool isSafetyZoneCode(int code) {
    return code == ERROR_SAFETY_LIMIT_EXCEEDED ||
           code == ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE ||
           code == ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE;
}

// PID-error min/max seed sentinel (min = INT32_MAX, max = INT32_MIN).
static const int32_t SENTINEL_MIN = 2147483647;    // INT32_MAX
static const int32_t SENTINEL_MAX = (-2147483647 - 1); // INT32_MIN

static bool isSentinel(int32_t minV, int32_t maxV) {
    return minV == SENTINEL_MIN && maxV == SENTINEL_MAX;
}

// STATUS_IN_THE_BOOTLOADER_FLAG_BIT
static const uint16_t BOOTLOADER_FLAG = (1u << 0);

// Known-good compile-time PID defaults for the bench M17.
static const uint32_t GOOD_KP = 2000, GOOD_KI = 5, GOOD_KD = 175000;
// Deliberately weak proportional gain, no integral / derivative.
static const uint32_t WEAK_KP = 1, WEAK_KI = 0, WEAK_KD = 0;

void tm_safety_pid(void) {
    Servomotor* motor = tfGetMotor();

    // ------------------------------------------------------------------
    // Clean state.
    // ------------------------------------------------------------------
    motor->systemReset();
    delay(1500);  // Wait for the device to come back up.

    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // Sanity: after reset the device is in application mode with no fatal error.
    getStatusResponse st0 = motor->getStatus();
    checkMotorError(*motor, "getStatus (post-reset)");
    printf("Post-reset status: flags=0x%04X fatalErrorCode=%u\n",
           st0.statusFlags, st0.fatalErrorCode);
    TEST_RESULT("Post-Reset No Fatal Error", st0.fatalErrorCode == 0);
    TEST_RESULT("Post-Reset In Application Mode",
                (st0.statusFlags & BOOTLOADER_FLAG) == 0);

    // ==================================================================
    // cmd 30 - Set safety limits
    // ==================================================================
    printf("\n=== cmd 30: Set safety limits ===\n");

    // --- Scenario A: narrow fence rejects an out-of-zone move ---
    const float NARROW_LIMIT_ROT = 0.5f;   // +/- 0.5 shaft rotations
    const float OUT_OF_ZONE_MOVE_ROT = 5.0f;  // 10x the narrow limit
    const float OUT_OF_ZONE_MOVE_TIME_S = 2.0f;

    printf("Setting narrow safety limits [%.2f, %.2f] rotations (current zeroed frame)...\n",
           -NARROW_LIMIT_ROT, NARROW_LIMIT_ROT);
    motor->setSafetyLimits(-NARROW_LIMIT_ROT, NARROW_LIMIT_ROT);
    checkMotorError(*motor, "setSafetyLimits (narrow)");
    getStatusResponse stNarrowSet = motor->getStatus();
    checkMotorError(*motor, "getStatus (after narrow set)");
    TEST_RESULT("Set Safety Limits (narrow) Accepted",
                stNarrowSet.fatalErrorCode == 0);

    printf("Attempting out-of-zone trapezoidMove(%.1f, %.1fs) - must be rejected...\n",
           OUT_OF_ZONE_MOVE_ROT, OUT_OF_ZONE_MOVE_TIME_S);
    // MOSFETs stay disabled: the safety check is on the predicted trajectory at
    // command-receive time, so the move never needs to physically execute.
    motor->trapezoidMove(OUT_OF_ZONE_MOVE_ROT, OUT_OF_ZONE_MOVE_TIME_S);
    delay(250);  // Let the firmware settle into the fatal-error state.
    getStatusResponse stRejected = motor->getStatus();
    printf("Status after out-of-zone move: flags=0x%04X fatalErrorCode=%u\n",
           stRejected.statusFlags, stRejected.fatalErrorCode);
    // The correct behavior: a safety-zone fatal error fired (25/26/27), and it is
    // definitely NOT 0 (which would mean the fence was not enforced).
    TEST_RESULT("Out-Of-Zone Move Rejected (safety-zone fatal error)",
                isSafetyZoneCode(stRejected.fatalErrorCode));
    TEST_RESULT("Out-Of-Zone Move Not Silently Accepted",
                stRejected.fatalErrorCode != 0);

    // Clear the fatal-error state before the next scenario.
    motor->systemReset();
    delay(1500);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setTimeUnit(TimeUnit::SECONDS);
    getStatusResponse stCleared = motor->getStatus();
    checkMotorError(*motor, "getStatus (after clearing fatal)");
    TEST_RESULT("Fatal Error Cleared By Reset", stCleared.fatalErrorCode == 0);

    // --- Scenario B: wide fence accepts an in-bounds move ---
    const float WIDE_LIMIT_ROT = 1000.0f;   // effectively unlimited
    const float IN_BOUNDS_MOVE_ROT = 0.1f;  // comfortably within the wide fence
    const float IN_BOUNDS_MOVE_TIME_S = 0.5f;

    printf("Setting wide safety limits [%.1f, %.1f] rotations...\n",
           -WIDE_LIMIT_ROT, WIDE_LIMIT_ROT);
    motor->setSafetyLimits(-WIDE_LIMIT_ROT, WIDE_LIMIT_ROT);
    checkMotorError(*motor, "setSafetyLimits (wide)");
    getStatusResponse stWideSet = motor->getStatus();
    checkMotorError(*motor, "getStatus (after wide set)");
    TEST_RESULT("Set Safety Limits (wide) Accepted",
                stWideSet.fatalErrorCode == 0);

    printf("Attempting in-bounds trapezoidMove(%.2f, %.1fs) - must be accepted...\n",
           IN_BOUNDS_MOVE_ROT, IN_BOUNDS_MOVE_TIME_S);
    // Trapezoid move ends at rest, so no ERROR_RUN_OUT_OF_QUEUE_ITEMS risk.
    motor->trapezoidMove(IN_BOUNDS_MOVE_ROT, IN_BOUNDS_MOVE_TIME_S);
    delay((int)(IN_BOUNDS_MOVE_TIME_S * 1000) + 400);  // wait past the move duration
    getStatusResponse stInBounds = motor->getStatus();
    printf("Status after in-bounds move: flags=0x%04X fatalErrorCode=%u\n",
           stInBounds.statusFlags, stInBounds.fatalErrorCode);
    TEST_RESULT("In-Bounds Move Accepted (no fatal error)",
                stInBounds.fatalErrorCode == 0);

    // ==================================================================
    // cmd 43 - Set PID constants
    // ==================================================================
    printf("\n=== cmd 43: Set PID constants ===\n");
    motor->systemReset();
    delay(1500);

    // Set the known-good gains. The firmware applies them immediately and does no
    // command-time validation; the observable contract here is a clean round-trip
    // plus a still-responsive device (no readback command exists).
    printf("Setting good PID constants (%u, %u, %u)...\n", GOOD_KP, GOOD_KI, GOOD_KD);
    motor->setPidConstants(GOOD_KP, GOOD_KI, GOOD_KD);
    TEST_RESULT("Set PID Constants (good) No Comm Error", motor->getError() == 0);

    getStatusResponse stAfterGoodPid = motor->getStatus();
    checkMotorError(*motor, "getStatus (after good PID)");
    TEST_RESULT("Motor Responsive After Good PID (no fatal error)",
                stAfterGoodPid.fatalErrorCode == 0);

    // The device must still answer a data query normally.
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    float posAfterGoodPid = motor->getPosition();
    TEST_RESULT("Get Position Responsive After Good PID", motor->getError() == 0);
    printf("Position after good PID: %.1f counts (errno=%d)\n",
           posAfterGoodPid, motor->getError());

    // Set a deliberately weak set of gains; the command must still round-trip
    // cleanly and leave the device responsive (behavioral degradation only shows
    // up under closed loop, which we do not force here).
    printf("Setting weak PID constants (%u, %u, %u)...\n", WEAK_KP, WEAK_KI, WEAK_KD);
    motor->setPidConstants(WEAK_KP, WEAK_KI, WEAK_KD);
    TEST_RESULT("Set PID Constants (weak) No Comm Error", motor->getError() == 0);
    getStatusResponse stAfterWeakPid = motor->getStatus();
    checkMotorError(*motor, "getStatus (after weak PID)");
    TEST_RESULT("Motor Responsive After Weak PID (no fatal error)",
                stAfterWeakPid.fatalErrorCode == 0);

    // Restore firmware-default gains for anything that runs later.
    motor->setPidConstants(GOOD_KP, GOOD_KI, GOOD_KD);
    TEST_RESULT("Restore PID Constants No Comm Error", motor->getError() == 0);

    // ==================================================================
    // cmd 39 - Get max PID error
    // ==================================================================
    printf("\n=== cmd 39: Get max PID error ===\n");
    motor->systemReset();
    delay(1500);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);

    // Before any closed-loop activity the trackers hold the seed sentinel
    // [INT32_MAX, INT32_MIN]. Use the RAW read for an exact integer comparison.
    getMaxPidErrorResponse pidRaw1 = motor->getMaxPidErrorRaw();
    TEST_RESULT("Get Max PID Error Read Shape (no comm error)", motor->getError() == 0);
    printf("First read (raw): min=%d max=%d\n", pidRaw1.minPidError, pidRaw1.maxPidError);
    TEST_RESULT("Post-Reset PID Error Is Sentinel [INT32_MAX, INT32_MIN]",
                isSentinel(pidRaw1.minPidError, pidRaw1.maxPidError));

    // Reading resets the trackers; with no closed loop / no PID iterations the
    // firmware reseeds to the same sentinel, so a second read is again the
    // sentinel. This exercises the read-then-reset path deterministically.
    getMaxPidErrorResponse pidRaw2 = motor->getMaxPidErrorRaw();
    TEST_RESULT("Second Max PID Error Read (no comm error)", motor->getError() == 0);
    printf("Second read (raw): min=%d max=%d\n", pidRaw2.minPidError, pidRaw2.maxPidError);
    TEST_RESULT("Second Read Reseeds To Sentinel (no PID activity)",
                isSentinel(pidRaw2.minPidError, pidRaw2.maxPidError));

    // The converted accessor must also return a well-formed 2-field struct. In
    // encoder-counts units the conversion is 1:1, so the sentinel maps straight
    // through and min > max still holds (the "no data" marker).
    getMaxPidErrorResponseConverted pidConv = motor->getMaxPidError();
    TEST_RESULT("Get Max PID Error Converted (no comm error)", motor->getError() == 0);
    printf("Converted read: min=%.1f max=%.1f\n", pidConv.minPidError, pidConv.maxPidError);
    TEST_RESULT("Converted Sentinel Has min > max (no-data marker)",
                pidConv.minPidError > pidConv.maxPidError);

    // ------------------------------------------------------------------
    // Leave a clean state.
    // ------------------------------------------------------------------
    motor->systemReset();
    delay(1500);
}
