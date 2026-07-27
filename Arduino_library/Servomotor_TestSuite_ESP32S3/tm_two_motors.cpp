#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_two_motors.cpp  --  MULTI-INSTANCE / SHARED-PORT usage pattern
//
// Exercises the way a real sketch drives more than one motor from ONE bus: two
// independent Servomotor objects, each bound to a specific device by unique ID,
// issuing commands interleaved over the shared Serial1. Two things must hold:
//
//   * ISOLATION (>= 2 devices): a command addressed to device A must move ONLY
//     device A; device B must not budge. Unique-ID (extended) addressing must
//     select exactly one target.
//   * SHARED-PORT CORRECTNESS (1 device / bench): two instances pointed at the
//     SAME device, used interleaved, must both transact successfully and AGREE
//     on what they read back (no cross-instance state corruption on the port).
//
// The module adapts to the bus population it finds and prints which path ran.
//
// LOCAL INSTANCES, portably:
//   The harness's tfGetMotor() has ALREADY opened Serial1 with the correct
//   RS485 pins (those pin numbers live only in the .ino and are unreachable
//   here). Communication::openSerialPort() guards the actual begin() behind a
//   static s_commSerialOpened flag, so any Servomotor we construct AFTER
//   tfGetMotor() re-opens nothing -- its openSerialPort() is a no-op and it
//   simply shares the already-configured Serial1. (The .ino itself constructs
//   an extra Servomotor the same way in detectMotorUid().) We call tfGetMotor()
//   first to guarantee the port is up before building our local instances, and
//   we match their CRC32 state to the harness instance so framing agrees.
//
// SAFETY / RACK-SAFE:
//   * Fleet discovery copies tm_multi_device.cpp's robust pattern: broadcast
//     reset + N detect rounds, each round followed by a broadcast reset and a
//     straggler-drain so no late detect reply desyncs the next transaction.
//   * Every single-target command goes out via a unique-ID-addressed instance
//     (mA / mB), so it is collision-safe on the 39-motor rack -- never an
//     alias-88 frame. The only broadcast is systemReset (write-style, no
//     response). No raw frames are injected.
//   * All motion is tiny (0.1 rotation over 1 s, ~0.1 rot/s) and every
//     trapezoid ends at REST; sequences return to ~origin and disable at rest.
//   * No test-mode (cmd 36) values are ever sent.
// ---------------------------------------------------------------------------

static const int    RESET_SETTLE_MS = 2000;   // after a broadcast systemReset
static const int    DETECT_QUIET_MS = 1300;   // post-detect packet-discard window
static const int    N_DETECT_ROUNDS = 5;      // discovery rounds (staggered replies)
static const size_t MAX_UIDS        = 48;     // collection cap (rack has 39)

static const int64_t MOVE_COUNTS = COUNTS_PER_REVOLUTION / 10;  // 0.1 rot = 327680 counts
static const int64_t ZERO_TOL    = 5000;      // "still at origin" tolerance (counts)
static const int64_t LAND_TOL    = 30000;     // "landed at target" tolerance (counts)
static const float   MOVE_ROT    = 0.1f;      // rotations per tiny move
static const float   MOVE_DUR    = 1.0f;      // seconds per tiny move

static bool containsUid(const std::vector<uint64_t>& list, uint64_t uid) {
    for (size_t i = 0; i < list.size(); i++)
        if (list[i] == uid) return true;
    return false;
}

// One discovery round (identical strategy to tm_multi_device.cpp).
static void detectRound(Servomotor* bcast, std::vector<uint64_t>& found) {
    const uint32_t roundStart = millis();  // hard bound: no infinite reply loop
    detectDevicesResponse r = bcast->detectDevices();
    while (bcast->getError() == 0 && (millis() - roundStart) < 8000) {
        uint64_t uid = r.uniqueId;
        if (uid != 0 && !containsUid(found, uid) && found.size() < MAX_UIDS) {
            found.push_back(uid);
            printf("  detected uniqueId=0x%016llX alias=%u\n",
                   (unsigned long long)uid, (unsigned)r.alias);
        }
        r = bcast->detectDevicesGetAnotherResponse();
    }
}

// Poll one instance's motion queue until empty (bounded ~8 s), then settle.
static void waitForIdle(Servomotor* m) {
    unsigned long t0 = millis();
    while (millis() - t0 < 8000) {
        uint8_t n = m->getNQueuedItems();
        if (m->getError() == 0 && n == 0) break;
        delay(10);
    }
    delay(150);  // settle margin
}

static int64_t readCounts(Servomotor* m) {
    int64_t raw = m->getPositionRaw();
    return raw;  // caller checks getError() where it matters
}

static bool pingRoundTrips(Servomotor* m) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 3 + 5);
    pingResponse r = m->ping(payload);
    if (m->getError() != 0) return false;
    return memcmp(r.responsePayload, payload, 10) == 0;
}

void tm_two_motors(void) {
    Serial.println("tm_two_motors: BEGIN\n");

    Servomotor* motor = tfGetMotor();   // guarantees Serial1 is open (pins set)

    const bool     hadExtended = motor->isUsingExtendedAddressing();
    const uint64_t originalUid = motor->usingThisUniqueId();
    const bool     crcOn       = motor->isCRC32Enabled();

    // =======================================================================
    // PHASE 1: broadcast reset + fleet discovery (robust straggler-drain).
    // =======================================================================
    motor->useAlias(255);
    motor->systemReset();
    delay(RESET_SETTLE_MS);
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)

    std::vector<uint64_t> found;
    found.reserve(MAX_UIDS);

    for (int round = 0; round < N_DETECT_ROUNDS; round++) {
        printf("\n--- Detect round %d/%d ---\n", round + 1, N_DETECT_ROUNDS);
        detectRound(motor, found);
        delay(DETECT_QUIET_MS);
        motor->systemReset();          // still broadcast (alias 255)
        delay(1500);
        for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
    }

    const int fleetCount = (int)found.size();
    printf("\n================= FLEET_COUNT n=%d =================\n", fleetCount);
    TEST_RESULT("TwoMotors: at least one device detected", fleetCount >= 1);

    if (fleetCount < 1) {
        // Nothing to test; restore addressing and bail (early abort = return).
        if (hadExtended) motor->useUniqueId(originalUid);
    else             motor->useAlias((uint8_t)originalUid);  // never leave the shared instance at alias 255
        return;
    }

    const bool twoPath = (fleetCount >= 2);
    const uint64_t uidA = found[0];
    const uint64_t uidB = twoPath ? found[1] : found[0];
    printf("PATH: %s (uidA=0x%016llX uidB=0x%016llX)\n",
           twoPath ? "TWO-DEVICE (isolation)" : "ONE-DEVICE (shared-port)",
           (unsigned long long)uidA, (unsigned long long)uidB);

    // -----------------------------------------------------------------------
    // Build two local unique-ID-addressed instances sharing the open Serial1.
    // openSerialPort() is a no-op now (static guard already set by tfGetMotor).
    // -----------------------------------------------------------------------
    Servomotor mA(255, Serial1);
    Servomotor mB(255, Serial1);
    mA.useUniqueId(uidA);
    mB.useUniqueId(uidB);
    if (crcOn) { mA.enableCRC32();  mB.enableCRC32(); }
    else       { mA.disableCRC32(); mB.disableCRC32(); }
    mA.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);  // getPositionRaw() still returns counts
    mA.setTimeUnit(TimeUnit::SECONDS);
    mB.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    mB.setTimeUnit(TimeUnit::SECONDS);

    // Enable both (interleaved) and zero both. enableMosfets can twitch a few
    // degrees, so settle before zeroing (per project know-how).
    mA.enableMosfets();
    mB.enableMosfets();
    delay(300);
    mA.zeroPosition();
    mB.zeroPosition();
    delay(150);

    if (twoPath) {
        // ===================================================================
        // TWO-DEVICE PATH: prove per-device isolation of unique-ID addressing.
        // ===================================================================
        int64_t pA0 = readCounts(&mA);
        int64_t pB0 = readCounts(&mB);
        printf("Start: pA0=%lld pB0=%lld (expect ~0 both)\n",
               (long long)pA0, (long long)pB0);
        TEST_RESULT("TwoMotors: device A starts at origin", llabs(pA0) <= ZERO_TOL);
        TEST_RESULT("TwoMotors: device B starts at origin", llabs(pB0) <= ZERO_TOL);

        // --- Move A ONLY. B must not move. ---
        printf("\n--- Move A only (+%.2f rot) ---\n", (double)MOVE_ROT);
        mA.trapezoidMove(MOVE_ROT, MOVE_DUR);
        checkMotorError(mA, "A trapezoidMove");
        waitForIdle(&mA);
        int64_t pA1 = readCounts(&mA);
        int64_t pB1 = readCounts(&mB);
        printf("After A-move: pA1=%lld (expect ~%lld) pB1=%lld (expect ~0)\n",
               (long long)pA1, (long long)MOVE_COUNTS, (long long)pB1);
        TEST_RESULT("TwoMotors: A moved ~0.1 rot on its own command",
                    llabs(pA1 - MOVE_COUNTS) <= LAND_TOL);
        TEST_RESULT("TwoMotors: B did NOT move while A moved (isolation)",
                    llabs(pB1 - pB0) <= ZERO_TOL);

        // Return A to origin (ends at rest).
        mA.trapezoidMove(-MOVE_ROT, MOVE_DUR);
        checkMotorError(mA, "A return");
        waitForIdle(&mA);

        // --- Move B ONLY. A must not move. ---
        printf("\n--- Move B only (+%.2f rot) ---\n", (double)MOVE_ROT);
        int64_t pA_pre = readCounts(&mA);
        mB.trapezoidMove(MOVE_ROT, MOVE_DUR);
        checkMotorError(mB, "B trapezoidMove");
        waitForIdle(&mB);
        int64_t pB2 = readCounts(&mB);
        int64_t pA2 = readCounts(&mA);
        printf("After B-move: pB2=%lld (expect ~%lld) pA2=%lld (expect ~%lld)\n",
               (long long)pB2, (long long)MOVE_COUNTS, (long long)pA2, (long long)pA_pre);
        TEST_RESULT("TwoMotors: B moved ~0.1 rot on its own command",
                    llabs(pB2 - MOVE_COUNTS) <= LAND_TOL);
        TEST_RESULT("TwoMotors: A did NOT move while B moved (isolation)",
                    llabs(pA2 - pA_pre) <= ZERO_TOL);

        // Return B to origin.
        mB.trapezoidMove(-MOVE_ROT, MOVE_DUR);
        checkMotorError(mB, "B return");
        waitForIdle(&mB);

        // --- Simultaneous tiny moves on both (interleaved dispatch). ---
        printf("\n--- Simultaneous move on both ---\n");
        mA.trapezoidMove(MOVE_ROT, MOVE_DUR);
        mB.trapezoidMove(MOVE_ROT, MOVE_DUR);
        checkMotorError(mA, "A simul move");
        checkMotorError(mB, "B simul move");
        waitForIdle(&mA);
        waitForIdle(&mB);
        int64_t pA3 = readCounts(&mA);
        int64_t pB3 = readCounts(&mB);
        printf("After simul: pA3=%lld pB3=%lld (both expect ~%lld)\n",
               (long long)pA3, (long long)pB3, (long long)MOVE_COUNTS);
        TEST_RESULT("TwoMotors: both moved ~0.1 rot on simultaneous commands",
                    llabs(pA3 - MOVE_COUNTS) <= LAND_TOL &&
                    llabs(pB3 - MOVE_COUNTS) <= LAND_TOL);

        // Return both to origin (end at rest).
        mA.trapezoidMove(-MOVE_ROT, MOVE_DUR);
        mB.trapezoidMove(-MOVE_ROT, MOVE_DUR);
        waitForIdle(&mA);
        waitForIdle(&mB);

        // Neither device latched a fatal error through the sequence.
        getStatusResponse sA = mA.getStatus();
        bool sAok = (mA.getError() == 0) && (sA.fatalErrorCode == 0);
        getStatusResponse sB = mB.getStatus();
        bool sBok = (mB.getError() == 0) && (sB.fatalErrorCode == 0);
        TEST_RESULT("TwoMotors: A clean (no fatal) after sequence", sAok);
        TEST_RESULT("TwoMotors: B clean (no fatal) after sequence", sBok);
    } else {
        // ===================================================================
        // ONE-DEVICE PATH: two instances -> SAME device. Interleaved commands
        // from either instance must succeed and AGREE (shared-port correctness).
        // ===================================================================
        int64_t pA0 = readCounts(&mA);
        bool a0ok = (mA.getError() == 0);
        int64_t pB0 = readCounts(&mB);
        bool b0ok = (mB.getError() == 0);
        printf("Start: pA0=%lld pB0=%lld (same device -> agree, ~0)\n",
               (long long)pA0, (long long)pB0);
        TEST_RESULT("TwoMotors(1dev): both instances read origin cleanly",
                    a0ok && b0ok && llabs(pA0) <= ZERO_TOL && llabs(pB0) <= ZERO_TOL);
        TEST_RESULT("TwoMotors(1dev): both instances agree at origin",
                    a0ok && b0ok && llabs(pA0 - pB0) <= ZERO_TOL);

        // Command via instance A; read back via BOTH instances -> must agree.
        printf("\n--- Move via instance A, read via A and B ---\n");
        mA.trapezoidMove(MOVE_ROT, MOVE_DUR);
        checkMotorError(mA, "A trapezoidMove");
        waitForIdle(&mA);
        int64_t pA1 = readCounts(&mA);
        int64_t pB1 = readCounts(&mB);   // same physical device, other instance
        printf("After A-move: pA1=%lld pB1=%lld (expect ~%lld, agree)\n",
               (long long)pA1, (long long)pB1, (long long)MOVE_COUNTS);
        TEST_RESULT("TwoMotors(1dev): instance-A command moved the device ~0.1 rot",
                    llabs(pA1 - MOVE_COUNTS) <= LAND_TOL);
        TEST_RESULT("TwoMotors(1dev): instance B sees A's motion (shared device, agree)",
                    llabs(pA1 - pB1) <= ZERO_TOL);

        // Command via instance B (the OTHER instance) to return -> proves both
        // instances can drive the shared port. Read back via both.
        printf("\n--- Move via instance B (return), read via A and B ---\n");
        mB.trapezoidMove(-MOVE_ROT, MOVE_DUR);
        checkMotorError(mB, "B trapezoidMove");
        waitForIdle(&mB);
        int64_t pA2 = readCounts(&mA);
        int64_t pB2 = readCounts(&mB);
        printf("After B-return: pA2=%lld pB2=%lld (expect ~0, agree)\n",
               (long long)pA2, (long long)pB2);
        TEST_RESULT("TwoMotors(1dev): instance-B command returned the device ~origin",
                    llabs(pB2) <= LAND_TOL);
        TEST_RESULT("TwoMotors(1dev): instances still agree after B-driven move",
                    llabs(pA2 - pB2) <= ZERO_TOL);

        // Interleaved non-motion transactions from both instances all succeed.
        bool interleavedOk = pingRoundTrips(&mA) && pingRoundTrips(&mB) &&
                             pingRoundTrips(&mA) && pingRoundTrips(&mB);
        TEST_RESULT("TwoMotors(1dev): interleaved pings from both instances round-trip",
                    interleavedOk);

        getStatusResponse st = mA.getStatus();
        TEST_RESULT("TwoMotors(1dev): device clean (no fatal) after sequence",
                    mA.getError() == 0 && st.fatalErrorCode == 0);
    }

    // =======================================================================
    // Cleanup: disable both, broadcast reset, drain, restore addressing.
    // =======================================================================
    mA.disableMosfets();
    mB.disableMosfets();
    motor->useAlias(255);
    motor->systemReset();
    delay(RESET_SETTLE_MS);
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)

    if (hadExtended) motor->useUniqueId(originalUid);
    else             motor->useAlias((uint8_t)originalUid);  // never leave the shared instance at alias 255
}
