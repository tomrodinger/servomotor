#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_multi_device.cpp  -- FLEET TEST
//
// Exercises the multi-device / bus-discovery path against the whole fleet:
//   * cmd 20  Detect devices   (broadcast discovery, dedupe by unique ID)
//   * cmd  8  Ping             (per-device round-trip)
//   * cmd 16  Get status       (per-device, must show no fatal error)
//   * cmd  5  Get temperature  (per-device, sanity range)
//
// Runs correctly on BOTH bus populations:
//   * the ESP32 bench (ONE motor)                -> FLEET_COUNT n=1
//   * the 39-motor rack (all alias 88)           -> FLEET_COUNT n=39
//
// BROADCAST INSTANCE, portably:
//   The spec calls for a "local broadcast Servomotor instance (like tfGetMotor's
//   construction but alias 255)". Constructing a *second* Servomotor here is
//   unsafe on the ESP32 target: the constructor unconditionally calls
//   openSerialPort(), which would re-begin Serial1 with the WRONG (default) UART
//   pins -- the actual RS485 RX/TX pin numbers live only in the .ino and are not
//   reachable from a test module (tf_framework.h is the only allowed include).
//   So instead we take the harness's single Servomotor (already bound to the
//   correct serial/pins) and TOGGLE its addressing: useAlias(255) makes it a
//   broadcast instance for the reset+detect phase; useUniqueId(uid) targets each
//   discovered device for the per-device probes. We capture the original
//   unique-ID addressing at entry and RESTORE it before returning, so later
//   modules in the suite still address the motor exactly as before. Functionally
//   identical to a dedicated alias-255 instance, without the pin hazard.
//
// SAFETY:
//   * NO motion is commanded anywhere -> no queue, no async fatal-18 risk.
//   * NO alias changes to any *device* (setDeviceAlias is never sent). We only
//     change which address OUR instance talks to.
//   * The only broadcast commands sent are systemReset (write-style, no response)
//     -- never a broadcast that would elicit replies from all 39 devices at once
//     except the inherently-broadcast 'Detect devices', whose staggered
//     random-delay replies are exactly what we are collecting.
//   * Detect leaves each device discarding packets for ~1 s; we honour that
//     quiet window before every subsequent transmission.
// ---------------------------------------------------------------------------

static const int   RESET_SETTLE_MS   = 2000;  // after a broadcast systemReset
static const int   DETECT_QUIET_MS   = 1300;  // post-detect packet-discard window
static const int   N_DETECT_ROUNDS   = 5;     // discovery rounds (staggered replies)
static const size_t MAX_UIDS         = 48;    // collection cap (rack has 39)
static const float TEMP_MIN_C        = 5.0f;  // open-interval sanity band...
static const float TEMP_MAX_C        = 85.0f; // ...(5..85) C

// Is `uid` already present in `list`?
static bool containsUid(const std::vector<uint64_t>& list, uint64_t uid) {
    for (size_t i = 0; i < list.size(); i++)
        if (list[i] == uid) return true;
    return false;
}

// One discovery round: broadcast 'Detect devices', then drain every staggered
// reply via detectDevicesGetAnotherResponse() until the read times out. New,
// non-zero unique IDs are appended to `found` (deduped, capped at MAX_UIDS).
static void detectRound(Servomotor* bcast, std::vector<uint64_t>& found) {
    detectDevicesResponse r = bcast->detectDevices();
    while (bcast->getError() == 0) {
        uint64_t uid = r.uniqueId;
        if (uid != 0 && !containsUid(found, uid) && found.size() < MAX_UIDS) {
            found.push_back(uid);
            printf("  detected uniqueId=0x%016llX alias=%u\n",
                   (unsigned long long)uid, (unsigned)r.alias);
        }
        r = bcast->detectDevicesGetAnotherResponse();
    }
}

void tm_multi_device(void) {
    Serial.println("tm_multi_device: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // The harness hands us a unique-ID-addressed instance. Remember it so we can
    // restore that exact addressing when we're done.
    const bool     hadExtended  = motor->isUsingExtendedAddressing();
    const uint64_t originalUid  = motor->usingThisUniqueId();

    // =======================================================================
    // PHASE 1: broadcast reset + fleet discovery.
    // =======================================================================
    motor->useAlias(255);                 // become a broadcast instance
    motor->systemReset();                 // broadcast reset (no response expected)
    delay(RESET_SETTLE_MS);

    std::vector<uint64_t> found;
    found.reserve(MAX_UIDS);

    for (int round = 0; round < N_DETECT_ROUNDS; round++) {
        printf("\n--- Detect round %d/%d ---\n", round + 1, N_DETECT_ROUNDS);
        detectRound(motor, found);
        // A device discards all packets for ~1 s after answering a detect, so
        // wait out that window before the next detect (or the per-device phase).
        delay(DETECT_QUIET_MS);
        // Python's proven fleet flow (detect_devices_iteratively): reset the
        // bus between rounds and DRAIN any straggler detect-responses, so no
        // late reply can desync the next transaction. With 39 devices the
        // response window is long; without this drain the whole RX stream
        // wedges (observed on the rack: every post-detect command failed).
        motor->systemReset();             // still broadcast (alias 255) here
        delay(1500);
        for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
    }

    const int fleetCount = (int)found.size();
    printf("\n================= FLEET_COUNT n=%d =================\n", fleetCount);

    TEST_RESULT("Fleet: at least one device detected", fleetCount >= 1);

    // Non-zero and unique (deduped by construction, but assert explicitly).
    bool allNonZero = true, allUnique = true;
    for (size_t i = 0; i < found.size(); i++) {
        if (found[i] == 0) allNonZero = false;
        for (size_t j = i + 1; j < found.size(); j++)
            if (found[i] == found[j]) allUnique = false;
    }
    TEST_RESULT("Fleet: all detected unique IDs are non-zero", allNonZero);
    TEST_RESULT("Fleet: all detected unique IDs are unique (no duplicates)", allUnique);

    // =======================================================================
    // PHASE 2: per-device sweep -- 3 fast commands each, addressed by unique ID.
    //   (a) ping echoes the 10-byte payload unchanged
    //   (b) getStatus succeeds with NO fatal error
    //   (c) getTemperature reads a sane Celsius value
    // Assert 100% of discovered devices pass ALL three.
    // =======================================================================
    motor->setTemperatureUnit(TemperatureUnit::CELSIUS);  // ensure C for the range check

    int pingOk = 0, statusOk = 0, tempOk = 0, allThreeOk = 0;

    for (size_t i = 0; i < found.size(); i++) {
        uint64_t uid = found[i];
        motor->useUniqueId(uid);   // target this specific device (collision-safe)

        // (a) ping round-trip
        uint8_t payload[10];
        for (int k = 0; k < 10; k++) payload[k] = (uint8_t)(k + (int)(uid & 0x0F));
        pingResponse pr = motor->ping(payload);
        bool pOk = (motor->getError() == 0) &&
                   (memcmp(pr.responsePayload, payload, 10) == 0);

        // (b) getStatus -- comms ok and no fatal error code
        getStatusResponse st = motor->getStatus();
        bool sOk = (motor->getError() == 0) && (st.fatalErrorCode == 0);

        // (c) getTemperature -- sane Celsius reading
        float tC = motor->getTemperature();
        bool tOk = (motor->getError() == 0) && (tC > TEMP_MIN_C) && (tC < TEMP_MAX_C);

        if (pOk) pingOk++;
        if (sOk) statusOk++;
        if (tOk) tempOk++;
        if (pOk && sOk && tOk) allThreeOk++;

        printf("  dev 0x%016llX: ping=%s status=%s(fatal=%u) temp=%.1fC=%s\n",
               (unsigned long long)uid,
               pOk ? "OK" : "BAD",
               sOk ? "OK" : "BAD", (unsigned)st.fatalErrorCode,
               (double)tC, tOk ? "OK" : "BAD");
    }

    printf("\nPer-device pass counts (of %d): ping=%d status=%d temp=%d allThree=%d\n",
           fleetCount, pingOk, statusOk, tempOk, allThreeOk);

    TEST_RESULT("Fleet: every device responds to ping (100%)",
                fleetCount >= 1 && pingOk == fleetCount);
    TEST_RESULT("Fleet: every device getStatus has no fatal error (100%)",
                fleetCount >= 1 && statusOk == fleetCount);
    TEST_RESULT("Fleet: every device temperature in (5..85) C (100%)",
                fleetCount >= 1 && tempOk == fleetCount);
    TEST_RESULT("Fleet: all three per-device checks pass on 100% of devices",
                fleetCount >= 1 && allThreeOk == fleetCount);

    // =======================================================================
    // Cleanup: broadcast reset the fleet, then restore OUR original addressing.
    // =======================================================================
    motor->useAlias(255);
    motor->systemReset();
    delay(RESET_SETTLE_MS);
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)  // leave a clean RX stream

    if (hadExtended) motor->useUniqueId(originalUid);
    else             motor->useAlias((uint8_t)originalUid);  // (harness uses extended; defensive)
}
