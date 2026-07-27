#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_wrong_addressing.cpp
//
// Addressing-robustness test, adapted to UNIQUE-ID addressing from
// python_programs/test_correct_and_incorrect_addressing.py.
//
// The suite addresses the real motor by its 64-bit unique ID (extended
// addressing), which is collision-safe on a multi-motor bus. This module
// proves:
//   (a) the correct unique ID responds cleanly (ping + Get status);
//   (b) a WRONG unique ID (the real ID with its low bit flipped) causes
//       Get status to TIME OUT (COMMUNICATION_ERROR_TIMEOUT, -1) and leaves
//       NO fatal error on the real motor (a mismatched extended-address
//       packet is silently ignored by every device);
//   (c) the real unique ID still responds cleanly afterward;
//   (d) a BROADCAST System reset (standard addressing, alias 255, no
//       response expected) leaves the motor responsive after it reboots.
//
// SAFETY:
//   * No motion is commanded, so there is no queue to drain.
//   * The only multi-target packet issued is the broadcast System reset,
//     which elicits NO response and is explicitly permitted. Every other
//     command targets exactly one device (by unique ID) or one wrong unique
//     ID that no device answers.
//   * The wrong-ID instance is a SECOND, local Servomotor sharing the same
//     Serial1. The library opens the RS485 port only once (static guard), so
//     constructing it with default pins does NOT reconfigure the port that
//     the harness already opened — it is safe on device and on host.
// ---------------------------------------------------------------------------

static const int RESET_MS       = 1500;  // wait after a plain / broadcast reset
static const int DETECT_QUIET_MS = 1300; // post-detect packet-discard window

// Read the real motor's fatal error code via Get status (answered even in the
// fatal state). Returns 0xFF if the query itself failed at the comms layer.
static uint8_t readFatalErrorCode(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

// Send a 0..9 ping and verify the 10-byte payload round-trips unchanged.
static bool pingRoundTrips(Servomotor* motor, const char* label) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)i;
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  %s: ping comms error %d\n", label, motor->getError());
        return false;
    }
    if (memcmp(r.responsePayload, payload, 10) != 0) {
        printf("  %s: ping payload mismatch\n", label);
        return false;
    }
    return true;
}

void tm_wrong_addressing(void) {
    Serial.println("tm_wrong_addressing: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state (unique-ID addressed reset).
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(RESET_MS);

    const bool extended = motor->isUsingExtendedAddressing();
    uint64_t realUid = motor->usingThisUniqueId();
    printf("Addressed by extended (unique-ID) addressing: %s\n", extended ? "yes" : "no");

    // If for some reason we are not extended-addressed (or the ID is unset),
    // learn the real unique ID by detecting once, then respect the quiet window.
    if (!extended || realUid == 0) {
        detectDevicesResponse d = motor->detectDevices();
        printf("Detect: error=%d uniqueId=0x%016llX alias=%u\n",
               motor->getError(), (unsigned long long)d.uniqueId, (unsigned)d.alias);
        delay(DETECT_QUIET_MS);
        if (motor->getError() == 0 && d.uniqueId != 0) realUid = d.uniqueId;
    }
    printf("Real unique ID: 0x%016llX\n", (unsigned long long)realUid);
    TEST_RESULT("Real unique ID is known (non-zero)", realUid != 0);

    // =======================================================================
    // (a) Correct unique ID responds cleanly.
    // =======================================================================
    printf("\n--- (a) Correct unique ID ---\n");
    TEST_RESULT("Correct unique ID: ping round-trips",
                pingRoundTrips(motor, "correct-uid-ping"));

    uint8_t fatalBefore = readFatalErrorCode(motor);
    printf("Get status (correct uid): commsErr=%d fatal=%u\n",
           motor->getError(), (unsigned)fatalBefore);
    TEST_RESULT("Correct unique ID: Get status succeeds with no fatal error",
                fatalBefore == 0);

    // =======================================================================
    // (b) WRONG unique ID -> Get status TIMES OUT, and NO fatal on the real
    //     motor. The wrong ID is the real ID with its low bit flipped.
    // =======================================================================
    printf("\n--- (b) Wrong unique ID (low bit flipped) ---\n");
    const uint64_t wrongUid = realUid ^ 0x1ULL;
    printf("Wrong unique ID: 0x%016llX\n", (unsigned long long)wrongUid);

    Servomotor wrong(255, Serial1);
    wrong.useUniqueId(wrongUid);

    getStatusResponse wrongSt = wrong.getStatus();
    int wrongErr = wrong.getError();
    (void)wrongSt;
    printf("Get status (wrong uid): commsErr=%d (expect %d = COMMUNICATION_ERROR_TIMEOUT)\n",
           wrongErr, COMMUNICATION_ERROR_TIMEOUT);
    TEST_RESULT("Wrong unique ID: Get status times out (COMMUNICATION_ERROR_TIMEOUT)",
                wrongErr == COMMUNICATION_ERROR_TIMEOUT);

    // The mismatched extended-address packet must be silently ignored by the
    // real motor: no fatal error should be latched.
    uint8_t fatalAfterWrong = readFatalErrorCode(motor);
    printf("Real motor Get status after wrong-uid attempt: commsErr=%d fatal=%u\n",
           motor->getError(), (unsigned)fatalAfterWrong);
    TEST_RESULT("Wrong unique ID: real motor still answers (no comms error)",
                motor->getError() == 0);
    TEST_RESULT("Wrong unique ID: real motor has NO fatal error",
                fatalAfterWrong == 0);

    // =======================================================================
    // (c) Real unique ID still responds cleanly after the wrong-ID attempt.
    // =======================================================================
    printf("\n--- (c) Real unique ID responds cleanly afterward ---\n");
    getStatusResponse stAfter = motor->getStatus();
    int errAfter = motor->getError();
    printf("Get status (correct uid, post-wrong): commsErr=%d fatal=%u\n",
           errAfter, (unsigned)stAfter.fatalErrorCode);
    TEST_RESULT("Post-wrong: real unique ID Get status succeeds, no fatal",
                errAfter == 0 && stAfter.fatalErrorCode == 0);
    TEST_RESULT("Post-wrong: real unique ID ping round-trips",
                pingRoundTrips(motor, "post-wrong-ping"));

    // =======================================================================
    // (d) BROADCAST System reset (standard addressing, alias 255). No response
    //     is sent for a broadcast reset; the getResponse call simply times out.
    //     After the reboot window the real motor must be responsive again.
    // =======================================================================
    printf("\n--- (d) Broadcast System reset (alias 255) ---\n");
    wrong.useAlias(255);          // switch the local instance to broadcast addressing
    wrong.systemReset();          // no response expected (broadcast)
    printf("Broadcast systemReset issued (commsErr=%d, timeout expected/ignored)\n",
           wrong.getError());
    delay(RESET_MS);              // let the device leave the bootloader window + boot the app

    getStatusResponse stBcast = motor->getStatus();
    int errBcast = motor->getError();
    printf("After broadcast reset: real motor Get status commsErr=%d fatal=%u\n",
           errBcast, (unsigned)stBcast.fatalErrorCode);
    TEST_RESULT("After broadcast reset: real motor responsive (Get status ok, no fatal)",
                errBcast == 0 && stBcast.fatalErrorCode == 0);
    TEST_RESULT("After broadcast reset: real motor ping round-trips",
                pingRoundTrips(motor, "post-broadcast-ping"));

    // -----------------------------------------------------------------------
    // Leave the device clean.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(RESET_MS);
}
