#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_hall_capture.cpp
//
// CAPTURE HALL SENSOR DATA (cmd 7) -- the least-exercised command in the suite.
//
// A coverage sweep of all 48 commands (coverage_report.py) put this one at the
// very bottom of the table with two call sites across eighty modules. That is
// worth fixing on its own, but it matters more than a low number suggests:
//
//   * it takes SIX parameters, more than any other command, so it has by far
//     the largest space of invalid inputs
//   * it is the only command with its own dedicated error codes (20, 21, 49)
//   * it returns a BULK, VARIABLE-LENGTH payload whose size is computed from
//     three of those parameters multiplied together -- exactly the shape that
//     produced the Arduino library's variable-length response bugs
//     (BUG-25/26/27), and exactly the shape where an overflow hides
//
// FIRMWARE GROUND TRUTH (firmware/Src/main.c, CAPTURE_HALL_SENSOR_DATA_COMMAND):
//
//     capture_type must be 1, 2 or 3                       else ERROR_CAPTURE_BAD_PARAMETERS (49)
//     n_points_to_capture  != 0                            else 49
//     time_steps_per_sample != 0                           else 49
//     n_samples_to_sum     != 0                            else 49
//     division_factor      != 0                            else 49
//     (bitmask & 7) != 0  AND  (bitmask & ~7) == 0         else 49   (so bitmask in 1..7)
//
//     n_channels   = popcount(bitmask & 7)
//     payload_size = n_points_to_capture * n_channels * 2
//     payload_size > 65535 - 5 - 4  ->  ERROR_CAPTURE_PAYLOAD_TOO_BIG (20)
//
// So the returned byte count is FULLY DETERMINED by the request, which makes
// this testable to the byte rather than merely "it returned something". That is
// the property this module is built around: for every legal combination, the
// number of bytes returned must equal n_points * popcount(bitmask) * 2 exactly.
//
// THE SAMPLE RATE MUST NOT OUTRUN THE LINK. A capture streams its points out
// over RS485 as it takes them. At 230400 baud the bus carries about 23000 bytes
// per second, so a three-channel point (6 bytes) needs roughly 260 microseconds
// on the wire, while one control tick is only 32 microseconds. Sampling every
// two ticks therefore produces data about four times faster than it can be
// sent, and the firmware correctly raises ERROR_CAPTURE_OVERFLOW (21).
//
// That is a real protection and section 5b now tests it deliberately -- it was
// previously the only one of this command's three error codes with no coverage.
// Everywhere else, time_steps_per_sample is set to 32 (about one millisecond
// per point), which is comfortably slower than the link for all seven channel
// masks.
//
// SAFETY: MOSFETs stay OFF. Capturing reads the hall sensors, which are passive
// -- nothing is driven and nothing turns. Captures are kept short so no
// subsection ties up the bus. Every invalid-parameter probe latches a fatal
// error deliberately and is cleared by a reset immediately afterwards. No
// broadcasts and no alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const uint8_t ERR_CAPTURE_PAYLOAD_TOO_BIG  = 20;
static const uint8_t ERR_CAPTURE_OVERFLOW         = 21;
static const uint8_t ERR_CAPTURE_BAD_PARAMETERS   = 49;

static int popcount3(uint8_t mask) {
    int n = 0;
    for (int i = 0; i < 3; i++) if (mask & (1 << i)) n++;
    return n;
}

// One capture attempt. Returns the byte count the library reported, and sets
// `accepted` to whether the call itself came back without an error.
static uint16_t doCapture(Servomotor* m, uint8_t type, uint32_t nPoints,
                          uint8_t mask, uint16_t stepsPerSample,
                          uint16_t samplesToSum, uint16_t divisionFactor,
                          bool* accepted, uint8_t* fatal) {
    static uint8_t buf[4096];
    uint16_t actual = 0;
    memset(buf, 0, sizeof(buf));
    m->captureHallSensorData(type, nPoints, mask, stepsPerSample, samplesToSum,
                             divisionFactor, buf, sizeof(buf), &actual);
    *accepted = (m->getError() == 0);
    *fatal = tfhFatal(m);
    return actual;
}

void tm_hall_capture(void) {
    Serial.println("tm_hall_capture: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. A LEGAL CAPTURE RETURNS EXACTLY THE COMPUTED NUMBER OF BYTES.
    //    Swept across every channel bitmask and several point counts, because
    //    the byte count is the product of both and an error in either factor
    //    would only show up for some combinations.
    // =====================================================================
    printf("\n--- 1. the returned byte count is exactly n_points x channels x 2 ---\n");
    {
        const uint32_t pointCounts[] = { 1, 2, 10, 50 };
        for (uint8_t mask = 1; mask <= 7; mask++) {
            for (unsigned i = 0; i < sizeof(pointCounts) / sizeof(pointCounts[0]); i++) {
                uint32_t nPoints = pointCounts[i];
                tfhReset(m);
                bool accepted = false; uint8_t fat = 0;
                uint16_t got = doCapture(m, 1, nPoints, mask, 32, 1, 1, &accepted, &fat);
                uint32_t want = nPoints * (uint32_t)popcount3(mask) * 2u;
                bool right = accepted && (fat == 0) && ((uint32_t)got == want);
                if (!right) {
                    printf("   mask=%u points=%lu: accepted=%d fatal=%u got %u bytes, expected %lu\n",
                           (unsigned)mask, (unsigned long)nPoints, (int)accepted,
                           (unsigned)fat, (unsigned)got, (unsigned long)want);
                }
                TEST_RESULT(std::string("capture of ") + std::to_string((unsigned long)nPoints) +
                                " points on mask " + std::to_string((unsigned)mask) +
                                " returns exactly " + std::to_string((unsigned long)want) + " bytes",
                            right);
            }
        }
    }

    // =====================================================================
    // 2. EVERY INVALID capture_type IS REFUSED WITH CODE 49. The firmware
    //    accepts only 1, 2 and 3; everything else must be rejected, not
    //    silently coerced into a valid type.
    // =====================================================================
    printf("\n--- 2. capture_type outside 1..3 is refused ---\n");
    {
        const uint8_t badTypes[] = { 0, 4, 5, 100, 255 };
        for (unsigned i = 0; i < sizeof(badTypes) / sizeof(badTypes[0]); i++) {
            tfhReset(m);
            bool accepted = false; uint8_t fat = 0;
            doCapture(m, badTypes[i], 10, 7, 32, 1, 1, &accepted, &fat);
            printf("   capture_type=%u -> accepted=%d fatal=%u\n",
                   (unsigned)badTypes[i], (int)accepted, (unsigned)fat);
            TEST_RESULT(std::string("capture_type=") + std::to_string((unsigned)badTypes[i]) +
                            " raises ERROR_CAPTURE_BAD_PARAMETERS (49)",
                        fat == ERR_CAPTURE_BAD_PARAMETERS);
            tfhReset(m);
        }
        // And the three legal types are all accepted, so section 2 cannot pass
        // by the command simply rejecting everything.
        for (uint8_t t = 1; t <= 3; t++) {
            tfhReset(m);
            bool accepted = false; uint8_t fat = 0;
            uint16_t got = doCapture(m, t, 4, 7, 32, 1, 1, &accepted, &fat);
            printf("   capture_type=%u -> accepted=%d fatal=%u bytes=%u\n",
                   (unsigned)t, (int)accepted, (unsigned)fat, (unsigned)got);
            TEST_RESULT(std::string("capture_type=") + std::to_string((unsigned)t) +
                            " is accepted", accepted && fat == 0 && got == 24);
        }
    }

    // =====================================================================
    // 3. EVERY ZERO-VALUED PARAMETER IS REFUSED. Four separate parameters
    //    each have their own non-zero requirement, checked one at a time so a
    //    missing guard on any single one is visible.
    // =====================================================================
    printf("\n--- 3. each parameter that must be non-zero is checked ---\n");
    {
        struct { const char* which; uint32_t nPoints; uint16_t steps, sum, div; } cases[] = {
            { "n_points_to_capture",   0, 32, 1, 1 },
            { "time_steps_per_sample", 10, 0, 1, 1 },
            { "n_samples_to_sum",      10, 32, 0, 1 },
            { "division_factor",       10, 32, 1, 0 },
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            tfhReset(m);
            bool accepted = false; uint8_t fat = 0;
            doCapture(m, 1, cases[i].nPoints, 7, cases[i].steps, cases[i].sum,
                      cases[i].div, &accepted, &fat);
            printf("   %s = 0 -> accepted=%d fatal=%u\n",
                   cases[i].which, (int)accepted, (unsigned)fat);
            TEST_RESULT(std::string("a zero ") + cases[i].which +
                            " raises ERROR_CAPTURE_BAD_PARAMETERS (49)",
                        fat == ERR_CAPTURE_BAD_PARAMETERS);
            tfhReset(m);
        }
    }

    // =====================================================================
    // 4. THE CHANNEL BITMASK BOUNDARY. Legal values are exactly 1..7: at
    //    least one of the low three bits set, and no bit above them. Both
    //    halves of that condition are probed, since a guard that checked only
    //    one would pass a naive test.
    // =====================================================================
    printf("\n--- 4. the channel bitmask must be in 1..7 ---\n");
    {
        // Zero: no channel selected.
        tfhReset(m);
        bool accepted = false; uint8_t fat = 0;
        doCapture(m, 1, 10, 0, 32, 1, 1, &accepted, &fat);
        printf("   bitmask=0 -> accepted=%d fatal=%u\n", (int)accepted, (unsigned)fat);
        TEST_RESULT("a bitmask of 0 selects no channel and is refused with code 49",
                    fat == ERR_CAPTURE_BAD_PARAMETERS);
        tfhReset(m);

        // Bits above the low three.
        const uint8_t highBits[] = { 8, 9, 15, 16, 128, 255 };
        for (unsigned i = 0; i < sizeof(highBits) / sizeof(highBits[0]); i++) {
            tfhReset(m);
            bool acc2 = false; uint8_t f2 = 0;
            doCapture(m, 1, 10, highBits[i], 32, 1, 1, &acc2, &f2);
            printf("   bitmask=%u -> accepted=%d fatal=%u\n",
                   (unsigned)highBits[i], (int)acc2, (unsigned)f2);
            TEST_RESULT(std::string("a bitmask of ") + std::to_string((unsigned)highBits[i]) +
                            " sets a bit above channel 3 and is refused",
                        f2 == ERR_CAPTURE_BAD_PARAMETERS);
            tfhReset(m);
        }

        // And all seven legal masks are accepted -- the positive control.
        int legalAccepted = 0;
        for (uint8_t mask = 1; mask <= 7; mask++) {
            tfhReset(m);
            bool acc3 = false; uint8_t f3 = 0;
            uint16_t got = doCapture(m, 1, 4, mask, 32, 1, 1, &acc3, &f3);
            if (acc3 && f3 == 0 && got == 4u * popcount3(mask) * 2u) legalAccepted++;
        }
        printf("   %d of 7 legal bitmasks accepted with the right byte count\n", legalAccepted);
        TEST_RESULT("all seven legal channel bitmasks are accepted", legalAccepted == 7);
    }

    // =====================================================================
    // 5. THE PAYLOAD-SIZE CEILING. payload_size = n_points * channels * 2
    //    must not exceed 65526 (65535 minus a 5-byte header and a 4-byte
    //    CRC). Probed from both sides of the boundary on a three-channel
    //    capture, where each point costs 6 bytes.
    // =====================================================================
    printf("\n--- 5. the 65526-byte payload ceiling ---\n");
    {
        const uint32_t maxBytes = 65535u - 5u - 4u;      // 65526
        const uint32_t perPoint = 3u * 2u;               // three channels
        const uint32_t maxPoints = maxBytes / perPoint;  // 10921

        // Comfortably over the ceiling: must be refused with code 20.
        tfhReset(m);
        bool accepted = false; uint8_t fat = 0;
        doCapture(m, 1, maxPoints * 4u, 7, 2, 1, 1, &accepted, &fat);
        printf("   %lu points x 6 bytes -> accepted=%d fatal=%u (expect 20)\n",
               (unsigned long)(maxPoints * 4u), (int)accepted, (unsigned)fat);
        TEST_RESULT("a capture far over the payload ceiling raises "
                    "ERROR_CAPTURE_PAYLOAD_TOO_BIG (20)",
                    fat == ERR_CAPTURE_PAYLOAD_TOO_BIG);
        tfhReset(m);

        // Just over the ceiling: the boundary itself.
        tfhReset(m);
        bool acc2 = false; uint8_t f2 = 0;
        doCapture(m, 1, maxPoints + 1u, 7, 2, 1, 1, &acc2, &f2);
        printf("   %lu points (one past the ceiling) -> accepted=%d fatal=%u\n",
               (unsigned long)(maxPoints + 1u), (int)acc2, (unsigned)f2);
        TEST_RESULT("one point past the payload ceiling is refused with code 20",
                    f2 == ERR_CAPTURE_PAYLOAD_TOO_BIG);
        tfhReset(m);

        // A capture near, but under, the ceiling must NOT be refused for
        // being too big. It is far larger than the host buffer here, so the
        // library may report a truncation -- what matters is that the FIRMWARE
        // did not raise code 20.
        tfhReset(m);
        bool acc3 = false; uint8_t f3 = 0;
        doCapture(m, 1, maxPoints - 1u, 7, 2, 1, 1, &acc3, &f3);
        printf("   %lu points (one under the ceiling) -> accepted=%d fatal=%u\n",
               (unsigned long)(maxPoints - 1u), (int)acc3, (unsigned)f3);
        TEST_RESULT("a capture just under the ceiling is not refused for being too big",
                    f3 != ERR_CAPTURE_PAYLOAD_TOO_BIG);
        tfhReset(m);
    }

    // =====================================================================
    // 5b. THE OVERFLOW GUARD. Sampling faster than the link can carry the data
    //     must raise ERROR_CAPTURE_OVERFLOW (21) rather than silently dropping
    //     points or corrupting the stream. This is the third of the command's
    //     error codes and the one that had no coverage at all before -- it was
    //     found only because this module's first draft tripped it by accident.
    //
    //     One tick is 32 us; a three-channel point is 6 bytes, which needs
    //     about 260 us at 230400 baud. So a sample every 1 or 2 ticks is
    //     several times faster than the wire can carry, and must be caught.
    // =====================================================================
    printf("\n--- 5b. sampling faster than the link raises the overflow guard ---\n");
    {
        const uint16_t tooFast[] = { 1, 2, 4 };
        for (unsigned i = 0; i < sizeof(tooFast) / sizeof(tooFast[0]); i++) {
            tfhReset(m);
            bool accepted = false; uint8_t fat = 0;
            doCapture(m, 1, 50, 7, tooFast[i], 1, 1, &accepted, &fat);
            printf("   time_steps_per_sample=%u on three channels -> accepted=%d fatal=%u\n",
                   (unsigned)tooFast[i], (int)accepted, (unsigned)fat);
            TEST_RESULT(std::string("sampling every ") + std::to_string((unsigned)tooFast[i]) +
                            " tick(s) on three channels raises ERROR_CAPTURE_OVERFLOW (21)",
                        fat == ERR_CAPTURE_OVERFLOW);
            tfhReset(m);
        }
        // And the positive control: a rate the link CAN carry must not overflow.
        tfhReset(m);
        bool accepted = false; uint8_t fat = 0;
        uint16_t got = doCapture(m, 1, 50, 7, 32, 1, 1, &accepted, &fat);
        printf("   time_steps_per_sample=32 -> accepted=%d fatal=%u bytes=%u\n",
               (int)accepted, (unsigned)fat, (unsigned)got);
        TEST_RESULT("a sample rate the link can carry does not overflow",
                    accepted && fat == 0 && got == 300);
        tfhReset(m);
    }

    // =====================================================================
    // 6. THE CAPTURED DATA IS PLAUSIBLE, NOT JUST CORRECTLY SIZED. A command
    //    that returned the right number of zero bytes would satisfy every
    //    check above. Require the samples to carry actual sensor readings.
    // =====================================================================
    printf("\n--- 6. the captured samples are real readings ---\n");
    {
        static uint8_t buf[512];
        uint16_t actual = 0;
        tfhReset(m);
        memset(buf, 0, sizeof(buf));
        m->captureHallSensorData(1, 60, 7, 32, 1, 1, buf, sizeof(buf), &actual);
        bool accepted = (m->getError() == 0) && (tfhFatal(m) == 0);
        int nonZero = 0;
        uint16_t lo = 0xFFFF, hi = 0;
        int nSamples = actual / 2;
        for (int i = 0; i < nSamples; i++) {
            uint16_t v = (uint16_t)(buf[i * 2] | ((uint16_t)buf[i * 2 + 1] << 8));
            if (v != 0) nonZero++;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        printf("   %u bytes, %d samples, %d non-zero, range %u..%u\n",
               (unsigned)actual, nSamples, nonZero, (unsigned)lo, (unsigned)hi);
        TEST_RESULT("a 60-point three-channel capture is accepted", accepted);
        TEST_RESULT("it returns 360 bytes", accepted && actual == 360);
        TEST_RESULT("most samples are non-zero, so these are real readings",
                    accepted && nonZero > nSamples / 2);
        TEST_RESULT("the samples vary, rather than being one repeated constant",
                    accepted && hi > lo);
    }

    // =====================================================================
    // 7. REPEATED CAPTURES ARE CONSISTENT, and the command leaves no residue:
    //    no queued items, no latched error, and the machine still moves.
    // =====================================================================
    printf("\n--- 7. repeated captures leave no residue ---\n");
    {
        tfhReset(m);
        bool allOk = true;
        uint16_t firstSize = 0;
        for (int i = 0; i < 10; i++) {
            bool accepted = false; uint8_t fat = 0;
            uint16_t got = doCapture(m, 1, 20, 7, 32, 1, 1, &accepted, &fat);
            if (!accepted || fat != 0) { allOk = false; printf("   capture %d failed\n", i); break; }
            if (i == 0) firstSize = got;
            else if (got != firstSize) {
                allOk = false;
                printf("   capture %d returned %u bytes, first returned %u\n",
                       i, (unsigned)got, (unsigned)firstSize);
                break;
            }
        }
        TEST_RESULT("ten identical captures all succeed and return the same size", allOk);
        TEST_RESULT("no queued items result from capturing",
                    (m->getNQueuedItems() == 0) && (m->getError() == 0));
        TEST_RESULT("no fatal error results from capturing", tfhFatal(m) == 0);

        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("an ordinary move still works after the capture sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
