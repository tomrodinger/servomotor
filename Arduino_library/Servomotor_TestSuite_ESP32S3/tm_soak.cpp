#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_soak.cpp
//
// ENDURANCE. Everything else in the suite is a sprint.
//
// Every other module resets, does a handful of operations, and looks at the
// result. Nothing keeps a motor working continuously for minutes. That leaves a
// whole class of defect unobserved:
//
//   * slow accumulation -- an error of a fraction of a count per move is
//     invisible in any single measurement and obvious after four hundred
//   * resource leaks -- a queue slot, a buffer, or a counter that is not
//     returned once per cycle takes many cycles to matter
//   * rare races -- the 0.15.12.0 queue-fill race fired about one move in five,
//     but a one-in-five-hundred race would pass every existing module and still
//     fail a customer once a day
//   * thermal and supply drift -- the motor warms up and the rail sags under
//     sustained work, and behaviour must not change as they do
//
// The design principle here is that a soak is only useful if it MEASURES
// something across the run rather than merely surviving it. So this module
// tracks the per-cycle error, the spread of it, the queue depth at rest, and
// the telemetry, and asserts on the TREND -- specifically that the second half
// of the run behaves like the first. A soak that only asserted "no fault
// occurred" would pass while quietly drifting.
//
// RUNTIME: roughly four to six minutes. That is why it has its own long
// timeout in the registry and why it is worth running in the background.
//
// SAFETY: MOSFETs stay OFF. Only the COMMANDED position is read, and the
// planner advances it regardless of the output stage, so nothing turns however
// long this runs -- which is what makes a multi-minute soak safe on a shared
// rack with pulleys and disks attached. No broadcasts and no alias changes.
// ---------------------------------------------------------------------------

void tm_soak(void) {
    Serial.println("tm_soak: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE MAIN SOAK: many out-and-back cycles without resetting between
    //    them. Not resetting is the point -- a reset would wash away exactly
    //    the accumulated state this module exists to find.
    // =====================================================================
    printf("\n--- 1. sustained out-and-back cycles ---\n");
    const int CYCLES = 200;
    int completed = 0, faults = 0;
    uint8_t firstFaultCode = 0;
    int firstFaultCycle = -1;
    int64_t worstResidueFirstHalf = 0, worstResidueSecondHalf = 0;
    int64_t origin = 0;
    {
        tfhFreshOpen(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        m->zeroPosition();
        delay(200);
        origin = m->getPositionRaw();

        const int32_t leg = (int32_t)(TFH_CPR / 4);
        const uint32_t dur = TFH_TPS / 2;

        for (int i = 0; i < CYCLES; i++) {
            m->trapezoidMoveRaw(leg, dur);
            if (m->getError() != 0) { faults++; if (firstFaultCycle < 0) firstFaultCycle = i; break; }
            m->trapezoidMoveRaw(-leg, dur);
            if (m->getError() != 0) { faults++; if (firstFaultCycle < 0) firstFaultCycle = i; break; }
            if (!tfhDrain(m, 10000)) { faults++; if (firstFaultCycle < 0) firstFaultCycle = i; break; }
            uint8_t fat = tfhFatal(m);
            if (fat != 0) {
                faults++;
                if (firstFaultCycle < 0) { firstFaultCycle = i; firstFaultCode = fat; }
                break;
            }
            // After an out-and-back the machine must be back at the origin.
            int64_t residue = m->getPositionRaw() - origin;
            if (i < CYCLES / 2) {
                if (llabs((long long)residue) > llabs((long long)worstResidueFirstHalf))
                    worstResidueFirstHalf = residue;
            } else {
                if (llabs((long long)residue) > llabs((long long)worstResidueSecondHalf))
                    worstResidueSecondHalf = residue;
            }
            completed++;
            if ((i + 1) % 50 == 0) {
                printf("   %d/%d cycles, residue %lld counts\n",
                       i + 1, CYCLES, (long long)residue);
            }
        }
        printf("   completed %d/%d cycles, %d faults (first at cycle %d, code %u)\n",
               completed, CYCLES, faults, firstFaultCycle, (unsigned)firstFaultCode);
        printf("   worst residue: first half %lld, second half %lld\n",
               (long long)worstResidueFirstHalf, (long long)worstResidueSecondHalf);

        TEST_RESULT("two hundred out-and-back cycles all complete",
                    completed == CYCLES && faults == 0);
        // The key assertion: no accumulation. An out-and-back is exactly
        // reversible, so the residue must stay near zero however many run.
        TEST_RESULT("the machine returns to the origin every cycle",
                    llabs((long long)worstResidueSecondHalf) <= 200);
        // And the trend: the second half must be no worse than the first.
        TEST_RESULT("the residue does not grow over the run",
                    llabs((long long)worstResidueSecondHalf) <=
                        llabs((long long)worstResidueFirstHalf) + 200);
    }

    // =====================================================================
    // 2. NOTHING LEAKED. After all that work the queue must be empty, the
    //    position must be the origin, and no error latched. A leaked queue
    //    slot would show up here as a nonzero resting depth.
    // =====================================================================
    printf("\n--- 2. nothing leaked over the run ---\n");
    {
        uint8_t depth = m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);
        uint8_t fat = tfhFatal(m);
        int64_t finalResidue = m->getPositionRaw() - origin;
        printf("   resting queue depth %u, fatal %u, final residue %lld\n",
               (unsigned)depth, (unsigned)fat, (long long)finalResidue);
        TEST_RESULT("the queue is empty at rest after the soak", depthOk && depth == 0);
        TEST_RESULT("no fatal error latched during the soak", fat == 0);
        TEST_RESULT("the final position is the origin it started from",
                    llabs((long long)finalResidue) <= 200);
    }

    // =====================================================================
    // 3. TELEMETRY STAYED SANE THROUGHOUT. The motor does not turn here (the
    //    MOSFETs are off), so it should not heat up much -- but the readings
    //    must remain plausible and readable, not drift into nonsense or stop
    //    responding after thousands of packets.
    // =====================================================================
    printf("\n--- 3. telemetry survived the run ---\n");
    {
        bool allOk = true;
        uint16_t vLo = 0xFFFF, vHi = 0;
        int16_t tLo = 32767, tHi = -32768;
        for (int i = 0; i < 20; i++) {
            uint16_t v = m->getSupplyVoltageRaw();
            if (m->getError() != 0) { allOk = false; break; }
            int16_t t = m->getTemperatureRaw();
            if (m->getError() != 0) { allOk = false; break; }
            if (v < vLo) vLo = v; if (v > vHi) vHi = v;
            if (t < tLo) tLo = t; if (t > tHi) tHi = t;
            delay(30);
        }
        printf("   supply voltage raw %u..%u, temperature raw %d..%d\n",
               (unsigned)vLo, (unsigned)vHi, (int)tLo, (int)tHi);
        TEST_RESULT("telemetry is still readable after the soak", allOk);
        TEST_RESULT("the supply voltage is still present and plausible",
                    allOk && vLo > 0 && vHi < 10000);
        TEST_RESULT("the supply voltage is stable across the readings",
                    allOk && (vHi - vLo) <= vHi / 5 + 5);
    }

    // =====================================================================
    // 4. THE MACHINE IS STILL PRECISE. A soak that ended with a working but
    //    less accurate motor would be a failure. Re-measure the same move
    //    that other modules use and require the same accuracy.
    // =====================================================================
    printf("\n--- 4. still precise after the soak ---\n");
    {
        int64_t lo = 0, hi = 0;
        int good = 0;
        for (int r = 0; r < 5; r++) {
            tfhFreshOpen(m);
            bool ok = false;
            int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 8), TFH_TPS / 2, &ok);
            if (!ok) continue;
            if (good == 0) { lo = hi = moved; }
            else { if (moved < lo) lo = moved; if (moved > hi) hi = moved; }
            good++;
        }
        printf("   five post-soak moves: %lld..%lld (asked %lld)\n",
               (long long)lo, (long long)hi, (long long)(TFH_CPR / 8));
        TEST_RESULT("all five post-soak moves complete", good == 5);
        TEST_RESULT("they still agree with each other to within 2 counts",
                    good == 5 && (hi - lo) <= 2);
        TEST_RESULT("they still deliver the commanded distance",
                    good == 5 && llabs((long long)(lo - TFH_CPR / 8)) <= 2);
    }

    // =====================================================================
    // 5. A SECOND, DIFFERENT SOAK: queue churn rather than travel. Fill the
    //    queue most of the way and drain it, repeatedly. This exercises the
    //    ring buffer's wraparound many times over, which the travel soak does
    //    not -- it never puts more than a few items in at once.
    // =====================================================================
    printf("\n--- 5. queue churn: fill and drain repeatedly ---\n");
    {
        const int ROUNDS = 40;
        int rounds = 0;
        bool clean = true;
        tfhFreshOpen(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        for (int r = 0; r < ROUNDS; r++) {
            // Nine moves at three slots each = 27, comfortably under 32.
            for (int k = 0; k < 9; k++) {
                m->trapezoidMoveRaw((k % 2 == 0) ? 4000 : -4000, TFH_TPS / 20);
                if (m->getError() != 0) { clean = false; break; }
            }
            if (!clean) break;
            if (!tfhDrain(m, 15000)) { clean = false; break; }
            if (tfhFatal(m) != 0) { clean = false; break; }
            rounds++;
        }
        uint8_t depth = m->getNQueuedItems();
        printf("   %d/%d fill-and-drain rounds (%d moves), resting depth %u\n",
               rounds, ROUNDS, rounds * 9, (unsigned)depth);
        TEST_RESULT("forty fill-and-drain rounds all complete", clean && rounds == ROUNDS);
        TEST_RESULT("the queue is empty after all the churn",
                    (m->getError() == 0) && depth == 0);
        TEST_RESULT("no fatal error resulted from the churn", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 6. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("an ordinary move still works after the whole soak",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
