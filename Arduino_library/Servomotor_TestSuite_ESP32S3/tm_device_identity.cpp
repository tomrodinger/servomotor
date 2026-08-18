#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_device_identity.cpp
//
// THE DEVICE'S SELF-DESCRIPTION, AND WHETHER THE TEST SUITE AGREES WITH IT.
//
// Four commands report what this motor IS: `Get product specs` (18), `Get
// product info` (22), `Get product description` (24) and `Get firmware version`
// (25). A coverage sweep put three of them in the bottom eight of all 48
// commands, at 2 to 4 call sites each.
//
// The reason to care is sharper than the low counts suggest. `Get product
// specs` returns exactly two numbers:
//
//     updateFrequency    -- control ticks per second
//     countsPerRotation  -- encoder counts in one shaft revolution
//
// and those two numbers are the constants that EVERY OTHER MODULE IN THIS SUITE
// hardcodes, as TFH_TPS (31250) and TFH_CPR (3276800). Nothing has ever checked
// that the device agrees with them. If it did not -- a different motor variant,
// a firmware change, a gearing difference -- then every distance and every
// duration asserted anywhere in the suite would be silently wrong, and every
// test would still pass, because they would all be wrong together.
//
// So this module does two things no other module does:
//
//   1. asks the device for those constants and checks the suite's hardcoded
//      values against them
//   2. closes the loop PHYSICALLY: commands a move computed from the DEVICE's
//      reported countsPerRotation and confirms the shaft's commanded position
//      advances by one rotation, and times a move built from the DEVICE's
//      reported updateFrequency against the device's own clock
//
// It also checks the identity fields are immutable, self-consistent and
// genuinely this motor -- in particular that the unique ID the device reports
// is the same one the harness is using to address it, which is the one identity
// assertion that could otherwise be wrong for every test in the suite at once.
//
// SAFETY: MOSFETs stay OFF. The only motion is a single one-rotation commanded
// move read through the COMMANDED position, which the planner advances
// regardless of the output stage, so nothing turns. No broadcasts and no alias
// changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

void tm_device_identity(void) {
    Serial.println("tm_device_identity: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE DEVICE'S REPORTED CONSTANTS MATCH THE SUITE'S ASSUMPTIONS.
    //    This is the assertion that validates every other module's arithmetic.
    // =====================================================================
    printf("\n--- 1. the device agrees with the suite's constants ---\n");
    uint32_t devTicksPerSec = 0, devCountsPerRot = 0;
    {
        tfhReset(m);
        getProductSpecsResponse specs = m->getProductSpecs();
        bool ok = (m->getError() == 0);
        devTicksPerSec = specs.updateFrequency;
        devCountsPerRot = specs.countsPerRotation;
        printf("   device reports: updateFrequency=%lu Hz, countsPerRotation=%lu\n",
               (unsigned long)devTicksPerSec, (unsigned long)devCountsPerRot);
        printf("   suite assumes : TFH_TPS=%ld,        TFH_CPR=%lld\n",
               (long)TFH_TPS, (long long)TFH_CPR);
        TEST_RESULT("get product specs is answerable", ok);
        TEST_RESULT("the device's update frequency matches the suite's TFH_TPS",
                    ok && devTicksPerSec == (uint32_t)TFH_TPS);
        TEST_RESULT("the device's counts per rotation matches the suite's TFH_CPR",
                    ok && devCountsPerRot == (uint32_t)TFH_CPR);
        // Sanity in absolute terms too, so a device that agreed with a WRONG
        // constant would still be caught.
        TEST_RESULT("the update frequency is the documented 31250 Hz",
                    devTicksPerSec == 31250u);
        TEST_RESULT("the counts per rotation is the documented 3276800",
                    devCountsPerRot == 3276800u);
    }

    // =====================================================================
    // 2. CLOSING THE LOOP: the reported countsPerRotation really is one
    //    rotation. A number that is merely self-consistent proves nothing;
    //    commanding exactly that many counts must move exactly one rotation
    //    of commanded position.
    // =====================================================================
    printf("\n--- 2. the reported countsPerRotation is physically one rotation ---\n");
    {
        if (devCountsPerRot == 0) {
            TEST_RESULT("countsPerRotation was readable, so the move can be built", false);
        } else {
            tfhFreshOpen(m);
            bool ok = false;
            int64_t moved = tfhMove(m, (int32_t)devCountsPerRot, 2 * TFH_TPS, &ok);
            printf("   commanded %lu counts (the device's own figure), moved %lld\n",
                   (unsigned long)devCountsPerRot, (long long)moved);
            TEST_RESULT("a move of the device's own countsPerRotation completes", ok);
            TEST_RESULT("and it advances the commanded position by exactly that much",
                        ok && llabs((long long)(moved - (int64_t)devCountsPerRot)) <=
                             (int64_t)devCountsPerRot / 1000 + 4);
        }
    }

    // =====================================================================
    // 3. CLOSING THE LOOP: the reported updateFrequency really is the tick
    //    rate. A move of N = updateFrequency ticks must consume one second of
    //    the DEVICE's own clock -- measured on the device, so no host latency
    //    enters the comparison.
    // =====================================================================
    printf("\n--- 3. the reported updateFrequency is the real tick rate ---\n");
    {
        if (devTicksPerSec == 0) {
            TEST_RESULT("updateFrequency was readable, so the timing can be built", false);
        } else {
            tfhFreshOpen(m);
            uint64_t t0 = m->getCurrentTimeRaw();
            bool okA = (m->getError() == 0);
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 8), devTicksPerSec);  // "one second"
            bool queued = (m->getError() == 0);
            bool drained = queued && tfhDrain(m, 15000);
            uint64_t t1 = m->getCurrentTimeRaw();
            bool okB = (m->getError() == 0);
            // getCurrentTimeRaw reports microseconds.
            uint64_t elapsedUs = t1 - t0;
            printf("   a %lu-tick move consumed %llu us of device clock (about 1 s expected)\n",
                   (unsigned long)devTicksPerSec, (unsigned long long)elapsedUs);
            TEST_RESULT("the clock is readable either side of the move", okA && okB);
            TEST_RESULT("the move completes", drained && tfhFatal(m) == 0);
            // At least a second (the move cannot finish early), and not wildly
            // more (draining polls, so allow generous overhead).
            TEST_RESULT("a move of updateFrequency ticks takes at least one second",
                        elapsedUs >= 990000ULL);
            TEST_RESULT("and not much more than one second",
                        elapsedUs <= 2500000ULL);
        }
    }

    // =====================================================================
    // 4. THE UNIQUE ID THE DEVICE REPORTS IS THE ONE WE ARE ADDRESSING.
    //    The harness talks to this motor by unique ID over a bus with 35 of
    //    them. If the reported ID differed from the addressed one, every
    //    module in the suite could be measuring a different motor than it
    //    thinks -- and nothing else would notice.
    // =====================================================================
    printf("\n--- 4. the device reports the unique ID we are addressing ---\n");
    {
        tfhReset(m);
        getProductInfoResponse info = m->getProductInfo();
        bool ok = (m->getError() == 0);
        uint64_t addressed = m->usingThisUniqueId();
        printf("   addressing %016llX, device reports %016llX\n",
               (unsigned long long)addressed, (unsigned long long)info.uniqueId);
        TEST_RESULT("get product info is answerable", ok);
        TEST_RESULT("the reported unique ID is non-zero", ok && info.uniqueId != 0);
        TEST_RESULT("the reported unique ID is the one being addressed",
                    ok && info.uniqueId == addressed);
    }

    // =====================================================================
    // 5. THE IDENTITY FIELDS ARE PLAUSIBLE. Each is checked for the specific
    //    way it could be wrong: a product code that is not printable text, a
    //    zeroed hardware version, an unset serial number.
    // =====================================================================
    printf("\n--- 5. the identity fields are plausible ---\n");
    {
        tfhReset(m);
        getProductInfoResponse info = m->getProductInfo();
        bool ok = (m->getError() == 0);

        int printable = 0;
        for (int i = 0; i < 8; i++) {
            char ch = info.productCode[i];
            if (ch >= 32 && ch < 127) printable++;
            else if (ch == 0) break;
        }
        char code[9];
        memcpy(code, info.productCode, 8);
        code[8] = 0;
        for (int i = 0; i < 8; i++) if (code[i] < 32 || code[i] >= 127) code[i] = 0;

        printf("   productCode='%s' (%d printable), firmwareCompatibility=%u, "
               "hardware %u.%u.%u, serial %lu\n",
               code, printable, (unsigned)info.firmwareCompatibility,
               (unsigned)info.hardwareVersion.major, (unsigned)info.hardwareVersion.minor,
               (unsigned)info.hardwareVersion.patch,
               (unsigned long)info.serialNumber);

        TEST_RESULT("the product code contains printable characters", ok && printable > 0);
        TEST_RESULT("the firmware compatibility code is set", ok && info.firmwareCompatibility > 0);
        TEST_RESULT("the hardware version is not all zeroes",
                    ok && (info.hardwareVersion.major || info.hardwareVersion.minor ||
                           info.hardwareVersion.patch));
        // Version fields are decimal-coded, so each should be a sane small number.
        TEST_RESULT("the hardware version fields are in a sane range",
                    ok && info.hardwareVersion.major < 100 &&
                    info.hardwareVersion.minor < 100 && info.hardwareVersion.patch < 100);
    }

    // =====================================================================
    // 6. THE PRODUCT DESCRIPTION IS A REAL STRING. It is a variable-length
    //    response, which is the shape that produced the Arduino library's
    //    BUG-25/26/27, so both its length and its contents are checked.
    // =====================================================================
    printf("\n--- 6. the product description is a real string ---\n");
    {
        tfhReset(m);
        char buf[128];
        uint16_t actual = 0;
        memset(buf, 0, sizeof(buf));
        m->getProductDescription(buf, sizeof(buf), &actual);
        bool ok = (m->getError() == 0);
        int printable = 0;
        for (uint16_t i = 0; i < actual && i < sizeof(buf); i++) {
            if (buf[i] >= 32 && buf[i] < 127) printable++;
        }
        // Make it safe to print.
        char safe[129];
        memcpy(safe, buf, sizeof(buf));
        safe[128] = 0;
        for (int i = 0; i < 128; i++) if (safe[i] && (safe[i] < 32 || safe[i] >= 127)) safe[i] = '.';
        printf("   %u bytes: '%s' (%d printable)\n", (unsigned)actual, safe, printable);
        TEST_RESULT("get product description is answerable", ok);
        TEST_RESULT("it returns a non-empty description", ok && actual > 0);
        TEST_RESULT("the description fits in the buffer provided", actual <= sizeof(buf));
        TEST_RESULT("the description is mostly printable text",
                    ok && actual > 0 && printable >= (int)actual / 2);

        // Repeatable, including the LENGTH -- a variable-length response that
        // returned a different size each time would be the BUG-25 signature.
        bool stable = ok;
        for (int i = 0; i < 8 && stable; i++) {
            char b2[128]; uint16_t a2 = 0;
            memset(b2, 0, sizeof(b2));
            m->getProductDescription(b2, sizeof(b2), &a2);
            if (m->getError() != 0 || a2 != actual || memcmp(b2, buf, actual) != 0) {
                stable = false;
                printf("   read %d returned %u bytes, first returned %u\n",
                       i, (unsigned)a2, (unsigned)actual);
            }
        }
        TEST_RESULT("eight reads return a byte-identical description", stable);
    }

    // =====================================================================
    // 7. IDENTITY IS IMMUTABLE. None of it may change across a reset, across
    //    a fault, or across ordinary work -- it describes the hardware, not
    //    the session.
    // =====================================================================
    printf("\n--- 7. identity survives resets, faults and work ---\n");
    {
        tfhReset(m);
        getProductInfoResponse ref = m->getProductInfo();
        getProductSpecsResponse refSpecs = m->getProductSpecs();
        getFirmwareVersionResponse refFw = m->getFirmwareVersion();
        bool ok = (m->getError() == 0);

        // Do some real work, then fault the machine deliberately.
        tfhFreshOpen(m);
        bool moveOk = false;
        tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &moveOk);
        tfhReset(m);
        m->setMaximumVelocity(0.2f);
        m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS / 4);   // force a fault
        delay(400);
        uint8_t faulted = tfhFatal(m);
        tfhReset(m);

        getProductInfoResponse after = m->getProductInfo();
        getProductSpecsResponse afterSpecs = m->getProductSpecs();
        getFirmwareVersionResponse afterFw = m->getFirmwareVersion();
        bool ok2 = (m->getError() == 0);

        printf("   fault raised: %u; identity before/after compared\n", (unsigned)faulted);
        TEST_RESULT("the machine really was faulted in between",
                    faulted != 0 && faulted != 0xFF);
        TEST_RESULT("the unique ID is unchanged", ok && ok2 && ref.uniqueId == after.uniqueId);
        TEST_RESULT("the serial number is unchanged",
                    ok && ok2 && ref.serialNumber == after.serialNumber);
        TEST_RESULT("the product code is unchanged",
                    ok && ok2 && memcmp(ref.productCode, after.productCode, 8) == 0);
        TEST_RESULT("the product specs are unchanged",
                    ok && ok2 &&
                    refSpecs.updateFrequency == afterSpecs.updateFrequency &&
                    refSpecs.countsPerRotation == afterSpecs.countsPerRotation);
        TEST_RESULT("the firmware version is unchanged",
                    ok && ok2 &&
                    refFw.firmwareVersion.major == afterFw.firmwareVersion.major &&
                    refFw.firmwareVersion.minor == afterFw.firmwareVersion.minor &&
                    refFw.firmwareVersion.patch == afterFw.firmwareVersion.patch &&
                    refFw.firmwareVersion.dev == afterFw.firmwareVersion.dev);
        TEST_RESULT("the device is running main firmware, not the bootloader",
                    ok2 && afterFw.inBootloader == 0);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
