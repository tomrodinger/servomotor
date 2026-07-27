#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_uid_edges.cpp
//
// Unique-ID addressing EDGE cases (bug-hunting, all assertions grounded in
// firmware/library source, not guesses):
//
//   (a) The two extreme unique IDs 0x0000000000000000 and 0xFFFFFFFFFFFFFFFF
//       belong to no device on the bus. A 'Get status' extended-addressed to
//       either must produce a CLEAN comms timeout and NOT be mistaken for a
//       response, and it must leave the REAL target unharmed (no fatal error,
//       still fully responsive). Ground truth:
//         - COMMUNICATION_ERROR_TIMEOUT == -1 (Communication.h line 11); the
//           getStatus(uint64_t) path sets _errno to getResponse()'s return,
//           and with no responder the first receiveBytes() times out and
//           returns COMMUNICATION_ERROR_TIMEOUT (Communication.cpp
//           getResponse -> receiveBytes -> COMMUNICATION_ERROR_TIMEOUT).
//         - A mismatched extended-address packet is silently ignored by every
//           device (no reply, no state change) -> the real motor stays clean.
//
//   (b) RESPONSE-ABANDONMENT RECOVERY. We deliberately transmit a 'Get status'
//       request to the REAL unique ID from a SECOND local instance using the
//       low-level Servomotor::sendCommand() (which sends but never reads the
//       reply), let the device's reply land unread in the RX buffer, then
//       DRAIN it. After draining, the next 5 normal commands on the main motor
//       must all succeed. This proves an abandoned reply cannot desynchronize
//       subsequent traffic once the RX buffer is drained.
//
//   (c) 10 alternating wrong-uid / right-uid 'Get status' cycles. Every wrong-
//       uid call must time out; every right-uid call must succeed. Proves the
//       real target keeps answering cleanly interleaved with dead-uid timeouts.
//
// SAFETY:
//   * No motion is commanded anywhere -> no motion queue, no async fatal-18.
//   * No 'Test mode' command is used -> the 10..13 permanent-hang values are
//     structurally impossible here.
//   * Every frame issued is EXTENDED-ADDRESSED (address byte 254 + 8-byte
//     unique ID) via the library send path:
//       - main motor: tfGetMotor() is unique-ID addressed;
//       - getStatus(uint64_t) overload: sends by unique ID directly;
//       - the local probe instance: useUniqueId() before any send.
//     No alias-88 / single-alias / broadcast frame that could command the
//     39-motor rack is ever transmitted. The dead unique IDs (all-zeros and
//     all-ones) match no device on any bus.
//   * NO raw Serial1.write() of hand-built frames is done; all sends go
//     through the library (raw_injection = false).
//   * The local probe instance shares Serial1; the library opens the port
//     only once (static guard in Communication::openSerialPort), so the
//     second instance does not reconfigure the harness-opened port.
// ---------------------------------------------------------------------------

static const int RESET_MS       = 1500;  // wait after a plain reset (no response during reset)
static const int DETECT_QUIET_MS = 1300; // post-detect packet-discard window
static const int ABANDON_SETTLE_MS = 200; // let an abandoned reply fully land before draining

// The two unique IDs that belong to no device.
static const uint64_t DEAD_UID_ZERO = 0x0000000000000000ULL;
static const uint64_t DEAD_UID_ONES = 0xFFFFFFFFFFFFFFFFULL;

// Discard any bytes sitting in the RS485 receive buffer.
static void drainRx() {
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}

// Read the real motor's fatal error code via 'Get status' (answered even in
// the fatal-error state). Returns 0xFF if the query itself failed at the
// comms layer so the caller can distinguish "no info" from "code 0".
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

void tm_uid_edges(void) {
    Serial.println("tm_uid_edges: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state (unique-ID addressed reset + drain).
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(RESET_MS);
    drainRx();

    const bool extended = motor->isUsingExtendedAddressing();
    uint64_t realUid = motor->usingThisUniqueId();
    printf("Addressed by extended (unique-ID) addressing: %s\n", extended ? "yes" : "no");

    // If we somehow are not extended-addressed (or the ID is unset), learn the
    // real unique ID by detecting once, then respect the quiet window.
    if (!extended || realUid == 0) {
        detectDevicesResponse d = motor->detectDevices();
        printf("Detect: error=%d uniqueId=0x%016llX alias=%u\n",
               motor->getError(), (unsigned long long)d.uniqueId, (unsigned)d.alias);
        delay(DETECT_QUIET_MS);
        drainRx();
        if (motor->getError() == 0 && d.uniqueId != 0) realUid = d.uniqueId;
    }
    printf("Real unique ID: 0x%016llX\n", (unsigned long long)realUid);
    // Sanity: the real ID must be known and must NOT coincide with either dead
    // ID (otherwise the "dead uid" probes below would hit the real device).
    TEST_RESULT("Real unique ID is known and distinct from both dead IDs",
                realUid != 0 && realUid != DEAD_UID_ONES);

    // =======================================================================
    // (a) The two extreme (nonexistent) unique IDs time out cleanly, and the
    //     real motor is left completely unharmed.
    // =======================================================================
    printf("\n--- (a) Dead unique IDs 0x0 and 0xFFFFFFFFFFFFFFFF ---\n");

    // 0x0000000000000000 : Get status must time out (COMMUNICATION_ERROR_TIMEOUT == -1).
    getStatusResponse stZero = motor->getStatus(DEAD_UID_ZERO);
    int errZero = motor->getError();
    (void)stZero;
    printf("Get status uid=0x0: commsErr=%d (expect %d = COMMUNICATION_ERROR_TIMEOUT)\n",
           errZero, COMMUNICATION_ERROR_TIMEOUT);
    // COMMUNICATION_ERROR_TIMEOUT == -1, Communication.h line 11.
    TEST_RESULT("Dead uid 0x0: Get status times out (COMMUNICATION_ERROR_TIMEOUT)",
                errZero == COMMUNICATION_ERROR_TIMEOUT);
    drainRx();  // nothing should be here, but keep the bus pristine

    // 0xFFFFFFFFFFFFFFFF : Get status must time out likewise.
    getStatusResponse stOnes = motor->getStatus(DEAD_UID_ONES);
    int errOnes = motor->getError();
    (void)stOnes;
    printf("Get status uid=0xFFFFFFFFFFFFFFFF: commsErr=%d (expect %d)\n",
           errOnes, COMMUNICATION_ERROR_TIMEOUT);
    TEST_RESULT("Dead uid 0xFFFFFFFFFFFFFFFF: Get status times out (COMMUNICATION_ERROR_TIMEOUT)",
                errOnes == COMMUNICATION_ERROR_TIMEOUT);
    drainRx();

    // The real motor must be untouched: no comms error, no latched fatal code.
    uint8_t fatalAfterDead = readFatalErrorCode(motor);
    printf("Real motor after dead-uid probes: commsErr=%d fatal=%u\n",
           motor->getError(), (unsigned)fatalAfterDead);
    TEST_RESULT("After dead-uid probes: real motor answers with no fatal error",
                motor->getError() == 0 && fatalAfterDead == 0);
    TEST_RESULT("After dead-uid probes: real motor ping round-trips",
                pingRoundTrips(motor, "post-dead-uid-ping"));

    // =======================================================================
    // (b) Response-abandonment recovery. Transmit a Get status to the REAL uid
    //     from a local instance WITHOUT reading the reply, let it land unread,
    //     drain it, then verify the next 5 normal commands all succeed.
    // =======================================================================
    printf("\n--- (b) Response-abandonment recovery ---\n");

    Servomotor probe(255, Serial1);
    probe.useUniqueId(realUid);         // extended addressing to the real device
    // Low-level send WITHOUT a matching getResponse(): the reply is abandoned.
    // GET_STATUS == 16 (Commands.h); Get status is answered normally, so the
    // device WILL emit a reply that we deliberately never read.
    probe.sendCommand(GET_STATUS, nullptr, 0);
    delay(ABANDON_SETTLE_MS);           // let the whole reply arrive in the RX buffer
    printf("Abandoned reply left %d byte(s) in RX; draining.\n", (int)Serial1.available());
    drainRx();                          // discard the abandoned reply

    int recoverySuccesses = 0;
    // 5 normal commands on the MAIN motor, alternating command types.
    for (int i = 0; i < 5; i++) {
        bool ok;
        if (i % 2 == 0) {
            getStatusResponse s = motor->getStatus();
            ok = (motor->getError() == 0 && s.fatalErrorCode == 0);
        } else {
            ok = pingRoundTrips(motor, "recovery-ping");
        }
        char name[64];
        snprintf(name, sizeof(name), "Recovery command %d/5 succeeds after abandonment", i + 1);
        TEST_RESULT(name, ok);
        if (ok) recoverySuccesses++;
    }
    printf("Recovery: %d/5 commands succeeded after draining the abandoned reply\n",
           recoverySuccesses);
    // Independently confirm no fatal error was latched by any of the above.
    uint8_t fatalAfterRecovery = readFatalErrorCode(motor);
    TEST_RESULT("After abandonment recovery: real motor has NO fatal error",
                motor->getError() == 0 && fatalAfterRecovery == 0);

    // =======================================================================
    // (c) 10 alternating wrong-uid / right-uid Get status cycles. Every wrong-
    //     uid call must time out; every right-uid call must succeed.
    // =======================================================================
    printf("\n--- (c) 10 alternating wrong-uid / right-uid cycles ---\n");

    int rightSuccesses = 0;   // right-uid Get status calls that returned cleanly
    int wrongTimeouts  = 0;   // wrong-uid Get status calls that timed out
    for (int cyc = 0; cyc < 10; cyc++) {
        // Alternate between the two dead IDs to exercise both extremes.
        uint64_t wrongUid = (cyc & 1) ? DEAD_UID_ONES : DEAD_UID_ZERO;

        getStatusResponse wSt = motor->getStatus(wrongUid);
        int wErr = motor->getError();
        (void)wSt;
        if (wErr == COMMUNICATION_ERROR_TIMEOUT) wrongTimeouts++;
        drainRx();  // keep the bus pristine between calls

        getStatusResponse rSt = motor->getStatus();  // real uid (stored addressing)
        int rErr = motor->getError();
        if (rErr == 0 && rSt.fatalErrorCode == 0) rightSuccesses++;
    }
    printf("Alternating cycles: right-uid successes=%d/10, wrong-uid timeouts=%d/10\n",
           rightSuccesses, wrongTimeouts);
    TEST_RESULT("Alternating: all 10 right-uid Get status calls succeed",
                rightSuccesses == 10);
    TEST_RESULT("Alternating: all 10 wrong-uid Get status calls time out",
                wrongTimeouts == 10);

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(RESET_MS);
    drainRx();
}
