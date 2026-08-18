#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_crc32_matrix.cpp
//
// CRC32 IS A FRAMING CHANGE THAT AFFECTS EVERY COMMAND, SO IT SHOULD BE TESTED
// AGAINST EVERY COMMAND -- not just against Get status.
//
// tm_comms already proves the CRC32 control command itself works: enabling it
// makes an unprotected packet get dropped, disabling it makes one succeed, and
// the bus recovers either way. What it does not do -- and what nothing in the
// suite does -- is run the actual COMMAND SET both ways and compare.
//
// That matters because turning CRC32 on adds four bytes to every frame in both
// directions. Any command whose payload sizing is computed slightly differently
// on the CRC path, or whose response length is parsed from a hard-coded offset,
// breaks only when CRC32 is on. Since CRC32 is ON by default in firmware, a
// command that works only with it OFF would be broken for most users -- and a
// command that works only with it ON would be broken for anyone who turned it
// off for bandwidth.
//
// The property asserted here is the same shape as tm_unit_equivalence's:
//
//     Every command must produce the SAME result with CRC32 on as with it off.
//
// So each command is run in both framings and the answers compared. The
// variable-length responses get particular attention: those are where the
// Arduino library had real bugs (BUG-25/26/27, fixed in library 0.10.0), and
// they are the ones most likely to mis-parse when four extra bytes appear.
//
// SAFETY: MOSFETs stay OFF. Motion commands are included, but every one is a
// small move read through the COMMANDED position, which the planner advances
// regardless of the output stage, so nothing turns. No broadcasts and no alias
// changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const int32_t CPR = 3276800;
static const int32_t TPS = 31250;

static void freshStart(Servomotor* m) {
    m->systemReset();
    delay(1500);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
    m->enableCRC32();                       // firmware boots with CRC32 on
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    m->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
}

static bool drainQueue(Servomotor* m, uint32_t budgetMs) {
    uint32_t deadline = millis() + budgetMs;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return false;
        if (q == 0) { delay(150); return true; }
        delay(20);
    }
    return false;
}

// Switch the whole link to no-CRC framing: tell the firmware first (that
// command still goes out WITH a CRC because the firmware still wants one), then
// drop the library's framing to match. Doing it in the other order desynchronises
// the link and everything times out.
static bool crcOff(Servomotor* m) {
    m->crc32Control(0);
    if (m->getError() != 0) return false;
    m->disableCRC32();
    return true;
}

// And back on: the enable command goes out WITHOUT a CRC (the firmware is not
// expecting one yet), then the library starts adding them.
static bool crcOn(Servomotor* m) {
    m->crc32Control(1);
    if (m->getError() != 0) return false;
    m->enableCRC32();
    return true;
}

// Everything one pass observes, so the two passes can be compared field by field.
struct Observations {
    bool     ok;
    uint64_t uniqueId;
    uint32_t fwVersion;          // packed major/minor/patch/dev
    uint16_t statusFlags;
    uint8_t  fatal;
    int64_t  moveDelta;
    uint8_t  slotsNormal;
    uint8_t  slotsFast;
    uint16_t voltageRaw;
    int16_t  temperatureRaw;
    int      descriptionLen;
    uint8_t  pingEchoOk;
    uint8_t  queueAfterStop;
    int64_t  hallDelta;
};

// Run the full battery once, in whatever framing is currently in force.
static Observations runBattery(Servomotor* m, const char* label) {
    Observations o;
    memset(&o, 0, sizeof(o));
    o.ok = true;

    // --- identity, fixed-length responses ---------------------------------
    getProductInfoResponse pi = m->getProductInfo();
    if (m->getError() != 0) { o.ok = false; printf("   [%s] getProductInfo failed\n", label); }
    o.uniqueId = pi.uniqueId;

    getFirmwareVersionResponse fv = m->getFirmwareVersion();
    if (m->getError() != 0) { o.ok = false; printf("   [%s] getFirmwareVersion failed\n", label); }
    o.fwVersion = ((uint32_t)fv.firmwareVersion.major << 24) |
                  ((uint32_t)fv.firmwareVersion.minor << 16) |
                  ((uint32_t)fv.firmwareVersion.patch << 8)  |
                   (uint32_t)fv.firmwareVersion.dev;

    getStatusResponse st = m->getStatus();
    if (m->getError() != 0) { o.ok = false; printf("   [%s] getStatus failed\n", label); }
    o.statusFlags = st.statusFlags;
    o.fatal = st.fatalErrorCode;

    // --- telemetry ---------------------------------------------------------
    o.voltageRaw = m->getSupplyVoltageRaw();
    if (m->getError() != 0) { o.ok = false; printf("   [%s] getSupplyVoltage failed\n", label); }
    o.temperatureRaw = m->getTemperatureRaw();
    if (m->getError() != 0) { o.ok = false; printf("   [%s] getTemperature failed\n", label); }

    // --- a VARIABLE-LENGTH response ---------------------------------------
    // This is the one most likely to mis-parse when four extra bytes appear.
    {
        char buf[128];
        uint16_t actual = 0;
        memset(buf, 0, sizeof(buf));
        m->getProductDescription(buf, sizeof(buf), &actual);
        if (m->getError() != 0) { o.ok = false; printf("   [%s] getProductDescription failed\n", label); }
        o.descriptionLen = (int)actual;
    }

    // --- a round-trip payload command --------------------------------------
    {
        uint8_t out[10];
        for (int i = 0; i < 10; i++) out[i] = (uint8_t)(0xA5 ^ (i * 17));
        pingResponse pr = m->ping(out);
        bool echoed = (m->getError() == 0);
        for (int i = 0; i < 10 && echoed; i++) {
            if (pr.responsePayload[i] != out[i]) echoed = false;
        }
        o.pingEchoOk = echoed ? 1 : 0;
        if (!echoed) { o.ok = false; printf("   [%s] ping did not echo correctly\n", label); }
    }

    // --- hall sensor read ---------------------------------------------------
    {
        int64_t h0 = m->getHallSensorPositionRaw();
        if (m->getError() != 0) { o.ok = false; printf("   [%s] getHallSensorPosition failed\n", label); }
        o.hallDelta = h0;
    }

    // --- an actual move, measured -------------------------------------------
    {
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        m->setTimeUnit(TimeUnit::TIMESTEPS);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(CPR / 8, TPS / 2);
        bool acc = (m->getError() == 0);
        bool drained = acc && drainQueue(m, 8000);
        o.moveDelta = drained ? (m->getPositionRaw() - before) : 0;
        if (!drained) { o.ok = false; printf("   [%s] the move did not complete\n", label); }
    }

    // --- queue behaviour in both regimes ------------------------------------
    {
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw(CPR / 200, 8 * TPS);
        o.slotsNormal = m->getNQueuedItems();
        m->emergencyStop();
        delay(300);
        o.queueAfterStop = m->getNQueuedItems();

        m->setMaximumVelocity(0.3f);
        m->setMaximumAcceleration(12000.0f);
        m->trapezoidMoveRaw(CPR / 200, 8 * TPS);
        o.slotsFast = m->getNQueuedItems();
        m->emergencyStop();
        delay(300);
        if (m->getError() != 0) { o.ok = false; printf("   [%s] queue probing failed\n", label); }
    }

    return o;
}

void tm_crc32_matrix(void) {
    Serial.println("tm_crc32_matrix: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE BATTERY WITH CRC32 ON (the firmware default).
    // =====================================================================
    printf("\n--- 1. the full battery with CRC32 ENABLED ---\n");
    freshStart(m);
    TEST_RESULT("the library is in CRC32 mode after a reset", m->isCRC32Enabled());
    Observations withCrc = runBattery(m, "crc on");
    TEST_RESULT("every command in the battery succeeds with CRC32 enabled", withCrc.ok);

    // =====================================================================
    // 2. THE SAME BATTERY WITH CRC32 OFF.
    // =====================================================================
    printf("\n--- 2. the same battery with CRC32 DISABLED ---\n");
    freshStart(m);
    bool switched = crcOff(m);
    TEST_RESULT("CRC32 can be turned off without losing the link", switched);
    TEST_RESULT("the library is out of CRC32 mode after the switch", !m->isCRC32Enabled());
    Observations noCrc;
    memset(&noCrc, 0, sizeof(noCrc));
    if (switched) {
        noCrc = runBattery(m, "crc off");
        TEST_RESULT("every command in the battery succeeds with CRC32 disabled", noCrc.ok);
    } else {
        TEST_RESULT("every command in the battery succeeds with CRC32 disabled", false);
    }

    // =====================================================================
    // 3. THE TWO PASSES MUST AGREE, FIELD BY FIELD. This is the point of the
    //    module: not that each framing works, but that they are equivalent.
    // =====================================================================
    printf("\n--- 3. the two framings must agree ---\n");
    {
        bool both = withCrc.ok && noCrc.ok;
        printf("   uniqueId      %016llX vs %016llX\n",
               (unsigned long long)withCrc.uniqueId, (unsigned long long)noCrc.uniqueId);
        printf("   firmware      %08lX vs %08lX\n",
               (unsigned long)withCrc.fwVersion, (unsigned long)noCrc.fwVersion);
        printf("   description   %d vs %d bytes\n", withCrc.descriptionLen, noCrc.descriptionLen);
        printf("   move delta    %lld vs %lld\n",
               (long long)withCrc.moveDelta, (long long)noCrc.moveDelta);
        printf("   slots         %u/%u vs %u/%u (normal/fast)\n",
               (unsigned)withCrc.slotsNormal, (unsigned)withCrc.slotsFast,
               (unsigned)noCrc.slotsNormal, (unsigned)noCrc.slotsFast);

        TEST_RESULT("the unique ID is the same in both framings",
                    both && withCrc.uniqueId == noCrc.uniqueId);
        TEST_RESULT("the firmware version is the same in both framings",
                    both && withCrc.fwVersion == noCrc.fwVersion);
        TEST_RESULT("the status flags are the same in both framings",
                    both && withCrc.statusFlags == noCrc.statusFlags);
        TEST_RESULT("neither framing leaves a fatal error",
                    both && withCrc.fatal == 0 && noCrc.fatal == 0);
        // The VARIABLE-LENGTH response is the one four extra bytes can break.
        TEST_RESULT("the product description is the same LENGTH in both framings",
                    both && withCrc.descriptionLen == noCrc.descriptionLen);
        TEST_RESULT("the product description is not empty in either framing",
                    both && withCrc.descriptionLen > 0);
        TEST_RESULT("ping echoes its payload correctly in both framings",
                    both && withCrc.pingEchoOk && noCrc.pingEchoOk);
        TEST_RESULT("the same move travels the same distance in both framings",
                    both && llabs((long long)(withCrc.moveDelta - noCrc.moveDelta)) <= 4);
        TEST_RESULT("the move is the commanded distance in both framings",
                    both && llabs((long long)(withCrc.moveDelta - CPR / 8)) <= 4);
        TEST_RESULT("the normal slot cost is the same in both framings",
                    both && withCrc.slotsNormal == noCrc.slotsNormal);
        TEST_RESULT("the fast-path slot cost is the same in both framings",
                    both && withCrc.slotsFast == noCrc.slotsFast);
        TEST_RESULT("emergency stop empties the queue in both framings",
                    both && withCrc.queueAfterStop == 0 && noCrc.queueAfterStop == 0);
        // Telemetry is a live reading, so compare loosely -- the point is that
        // it parses to a sane value both ways, not that it is bit-identical.
        TEST_RESULT("the supply voltage reads sanely in both framings",
                    both && withCrc.voltageRaw > 0 && noCrc.voltageRaw > 0);
        TEST_RESULT("the supply voltage agrees between framings to within 10%",
                    both && withCrc.voltageRaw > 0 &&
                    labs((long)withCrc.voltageRaw - (long)noCrc.voltageRaw)
                        <= (long)withCrc.voltageRaw / 10 + 2);
    }

    // =====================================================================
    // 4. SWITCHING BACK AND FORTH REPEATEDLY. A single toggle could work by
    //    luck; the link has to survive being switched many times, and a
    //    command must succeed after every switch. This is where a stale
    //    receive buffer or an off-by-four in the framing would show up.
    // =====================================================================
    printf("\n--- 4. toggling the framing repeatedly ---\n");
    {
        freshStart(m);
        bool allOk = true;
        int cycles = 0;
        for (int i = 0; i < 8 && allOk; i++) {
            if (!crcOff(m)) { allOk = false; printf("   cycle %d: could not turn CRC off\n", i); break; }
            getStatusResponse a = m->getStatus();
            if (m->getError() != 0) { allOk = false; printf("   cycle %d: no-CRC status failed\n", i); break; }
            (void)a;
            if (!crcOn(m))  { allOk = false; printf("   cycle %d: could not turn CRC on\n", i); break; }
            getStatusResponse b = m->getStatus();
            if (m->getError() != 0) { allOk = false; printf("   cycle %d: CRC status failed\n", i); break; }
            (void)b;
            cycles++;
        }
        printf("   completed %d off/on cycles\n", cycles);
        TEST_RESULT("eight CRC32 off/on cycles all succeed", allOk && cycles == 8);
        TEST_RESULT("the link still works after all the toggling",
                    (m->getStatus(), m->getError() == 0));
    }

    // =====================================================================
    // 5. THE FRAMING SETTING DOES NOT SURVIVE A RESET. The firmware boots
    //    with CRC32 on, so a reset must put it back regardless of what it was
    //    switched to. A caller that resets and then keeps sending no-CRC
    //    frames would otherwise silently lose the link.
    // =====================================================================
    printf("\n--- 5. a reset restores the default framing ---\n");
    {
        freshStart(m);
        bool off = crcOff(m);
        // Now reset while the firmware is in no-CRC mode. The reset command
        // itself goes out without a CRC, which is correct for the current state.
        m->systemReset();
        delay(1500);
        for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
        // The firmware is back to expecting a CRC. Sending without one must
        // now fail, which is how we prove the reset restored the default.
        getStatusResponse bad = m->getStatus();
        int errWithoutCrc = m->getError();
        (void)bad;
        m->enableCRC32();
        getStatusResponse good = m->getStatus();
        int errWithCrc = m->getError();
        (void)good;
        printf("   after reset: no-CRC err=%d, with-CRC err=%d\n", errWithoutCrc, errWithCrc);
        TEST_RESULT("CRC32 was successfully disabled before the reset", off);
        TEST_RESULT("after a reset an unprotected frame is refused again",
                    errWithoutCrc != 0);
        TEST_RESULT("after a reset a protected frame works again", errWithCrc == 0);
    }

    // =====================================================================
    // 6. Still healthy afterwards, in the default framing.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        freshStart(m);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(CPR / 4, TPS);
        bool ok = (m->getError() == 0) && drainQueue(m, 8000);
        getStatusResponse st = m->getStatus();
        TEST_RESULT("an ordinary move still works after the CRC32 sweep",
                    ok && st.fatalErrorCode == 0 &&
                    llabs((long long)(m->getPositionRaw() - before - CPR / 4)) < 200);
    }

    freshStart(m);
}
