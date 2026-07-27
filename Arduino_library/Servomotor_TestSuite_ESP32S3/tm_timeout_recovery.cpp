#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_timeout_recovery.cpp
//
// Timeout-path robustness for the RS485 receive/parser state machine.
//
// A command addressed to a unique ID that no device on the bus owns elicits
// NO reply, so the library's getResponse() blocks until its timeout expires
// (COMMUNICATION_ERROR_TIMEOUT == -1, ~1 s per call; see Communication.h line
// 11 and receiveBytes timeout_ms default 1000 in Communication.h). The bug
// this module hunts: does a timed-out receive leave lingering bytes or stale
// parser state that corrupts the very NEXT command to a REAL device?
//
// Ground truth for the "no cross-contamination" expectation: a mismatched
// EXTENDED-addressing packet is silently ignored by every device on the bus
// (it addresses a unique ID none of them has), so the real motor latches no
// fatal error and stays fully responsive. This mirrors the (b) case proven in
// tm_wrong_addressing.cpp and the addressing model in error_codes.json.
//
// STRUCTURE:
//   PHASE 1  - 10 cycles of [getStatus to a nonexistent uid (must time out),
//              then immediately getStatus + ping to the REAL uid (must both
//              succeed, no fatal, ping payload intact)]. All 10 wrong-target
//              rounds must time out; all 10 real-target rounds must succeed
//              with zero cross-contamination.
//   PHASE 2  - 5 rapid nonexistent-uid calls back-to-back (each timing out),
//              then 5 real calls (getStatus + ping) that must all succeed.
//   Total wall-clock elapsed is reported.
//
// SAFETY:
//   * NO motion is commanded anywhere, so there is no motion queue to drain
//     and no async fatal risk.
//   * NO test-mode (cmd 36) values are used.
//   * RACK SAFETY: every command targets exactly one device by EXTENDED
//     (unique-ID) addressing. The real motor is tfGetMotor() (unique-ID
//     addressed). The "wrong" target is a SECOND local Servomotor that is
//     ALSO extended-addressed via useUniqueId() to a nonexistent id (the real
//     id with its low bit flipped), so it is silently ignored by all devices
//     and can never command an alias-88 frame that would hit 39 motors at
//     once. No alias frame, no broadcast, is ever sent by this module.
//   * The wrong-id instance shares Serial1; the library opens the RS485 port
//     only once (static guard), so constructing it does not reconfigure the
//     port the harness already opened.
//   * No raw Serial1.write() framing is done here (raw_injection = false).
// ---------------------------------------------------------------------------

static const int RESET_MS = 1500;  // wait after a plain reset

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

// Send an identifiable 10-byte ping and verify it round-trips unchanged.
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 13 + 5);
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  ping comms error %d\n", motor->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

void tm_timeout_recovery(void) {
    Serial.println("tm_timeout_recovery: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state (unique-ID addressed reset + drain).
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(RESET_MS);
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)

    // Learn the real unique ID (the harness addresses tfGetMotor by it).
    uint64_t realUid = motor->usingThisUniqueId();
    if (!motor->isUsingExtendedAddressing() || realUid == 0) {
        detectDevicesResponse d = motor->detectDevices();
        printf("Detect: error=%d uniqueId=0x%016llX\n",
               motor->getError(), (unsigned long long)d.uniqueId);
        delay(1300);  // post-detect quiet window
        for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
        if (motor->getError() == 0 && d.uniqueId != 0) realUid = d.uniqueId;
    }
    printf("Real unique ID: 0x%016llX\n", (unsigned long long)realUid);
    TEST_RESULT("Real unique ID is known (non-zero)", realUid != 0);

    // Nonexistent target: the real id with its low bit flipped. No device on
    // the bus owns it, so extended-addressed packets to it are silently
    // ignored (same construction proven safe in tm_wrong_addressing.cpp).
    const uint64_t wrongUid = realUid ^ 0x1ULL;
    printf("Nonexistent unique ID: 0x%016llX\n", (unsigned long long)wrongUid);
    Servomotor wrong(255, Serial1);
    wrong.useUniqueId(wrongUid);  // EXTENDED addressing -> rack-safe

    unsigned long tStart = millis();

    // =======================================================================
    // PHASE 1: 10 cycles of [wrong-uid getStatus (times out) -> real-uid
    // getStatus + ping (both succeed, no cross-contamination)].
    // =======================================================================
    printf("\n--- PHASE 1: 10 timeout/recover cycles ---\n");

    bool allWrongTimedOut = true;   // every wrong-uid call returned -1
    bool allRealStatusOk  = true;   // every real getStatus: comms ok + fatal 0
    bool allRealPingOk    = true;   // every real ping round-tripped intact
    unsigned long maxTimeoutMs = 0; // longest single wrong-uid timeout observed
    unsigned long minTimeoutMs = 0xFFFFFFFFUL;

    for (int i = 0; i < 10; i++) {
        // --- wrong uid: must time out (~1 s). ---
        unsigned long t0 = millis();
        getStatusResponse wrongSt = wrong.getStatus();
        (void)wrongSt;
        int wrongErr = wrong.getError();
        unsigned long dt = millis() - t0;
        if (dt > maxTimeoutMs) maxTimeoutMs = dt;
        if (dt < minTimeoutMs) minTimeoutMs = dt;
        // COMMUNICATION_ERROR_TIMEOUT == -1 (Communication.h line 11).
        if (wrongErr != COMMUNICATION_ERROR_TIMEOUT) {
            printf("  cycle %d: wrong-uid err=%d (expected %d) dt=%lums\n",
                   i, wrongErr, COMMUNICATION_ERROR_TIMEOUT, dt);
            allWrongTimedOut = false;
        }

        // --- real uid: getStatus must succeed with no fatal error. ---
        getStatusResponse realSt = motor->getStatus();
        int realErr = motor->getError();
        if (realErr != 0 || realSt.fatalErrorCode != 0) {
            printf("  cycle %d: real getStatus err=%d fatal=%u\n",
                   i, realErr, (unsigned)realSt.fatalErrorCode);
            allRealStatusOk = false;
        }

        // --- real uid: ping must round-trip immediately after the timeout. ---
        if (!pingRoundTrips(motor)) {
            printf("  cycle %d: real ping failed\n", i);
            allRealPingOk = false;
        }
    }
    printf("Phase 1: wrong-uid timeout dt range %lu..%lu ms\n",
           minTimeoutMs, maxTimeoutMs);

    TEST_RESULT("P1 Every nonexistent-uid getStatus timed out (COMMUNICATION_ERROR_TIMEOUT)",
                allWrongTimedOut);
    TEST_RESULT("P1 Every real-uid getStatus succeeded with no fatal (no cross-contamination)",
                allRealStatusOk);
    TEST_RESULT("P1 Every real-uid ping round-tripped intact after a timeout",
                allRealPingOk);

    // =======================================================================
    // PHASE 2: 5 rapid nonexistent-uid calls back-to-back, then 5 real calls.
    // Proves multiple consecutive timeouts do not accumulate stale RX state.
    // =======================================================================
    printf("\n--- PHASE 2: 5 rapid timeouts, then 5 real calls ---\n");

    bool burstAllTimedOut = true;
    for (int i = 0; i < 5; i++) {
        getStatusResponse s = wrong.getStatus();
        (void)s;
        int e = wrong.getError();
        if (e != COMMUNICATION_ERROR_TIMEOUT) {
            printf("  burst %d: err=%d (expected %d)\n",
                   i, e, COMMUNICATION_ERROR_TIMEOUT);
            burstAllTimedOut = false;
        }
    }
    TEST_RESULT("P2 Five rapid nonexistent-uid calls all timed out",
                burstAllTimedOut);

    bool realAfterBurstOk = true;
    for (int i = 0; i < 5; i++) {
        getStatusResponse s = motor->getStatus();
        int e = motor->getError();
        if (e != 0 || s.fatalErrorCode != 0) {
            printf("  post-burst %d: getStatus err=%d fatal=%u\n",
                   i, e, (unsigned)s.fatalErrorCode);
            realAfterBurstOk = false;
        }
        if (!pingRoundTrips(motor)) {
            printf("  post-burst %d: ping failed\n", i);
            realAfterBurstOk = false;
        }
    }
    TEST_RESULT("P2 Five real calls after a rapid timeout burst all succeeded (no fatal, ping intact)",
                realAfterBurstOk);

    // Final confirmation the real device is clean and responsive.
    uint8_t finalFatal = readFatalErrorCode(motor);
    printf("Final: real motor fatal=%u\n", (unsigned)finalFatal);
    TEST_RESULT("Real motor clean and responsive at end (no fatal error)",
                finalFatal == 0);

    unsigned long elapsed = millis() - tStart;
    printf("\nTotal elapsed (timeout+recovery workload): %lu ms\n", elapsed);

    // -----------------------------------------------------------------------
    // Leave the device clean.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(RESET_MS);
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}
