#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_determinism.cpp
//
// REPEATABILITY. Every other module asks "is this one answer correct?". This
// one asks "is it the SAME answer every time?", which is a different and
// largely unguarded property.
//
// It matters because the three bugs fixed in 0.15.12.0 were all INTERMITTENT.
// The queue-fill race failed roughly one move in five: a single-shot test had
// an 80% chance of passing and calling the firmware healthy. Anything that
// depends on the timing of the 31.25 kHz control ISR relative to when a packet
// happens to arrive can only be caught by repetition, and nothing in the suite
// repeats an identical operation enough times to see it.
//
// So each check here runs the same operation many times and requires the
// results to be identical -- not merely individually plausible.
//
//     * an identical move, repeated, travels an identical distance
//     * a pure read, repeated, returns an identical value while nothing moves
//     * the same move from the same origin lands at the same absolute position
//     * a reset really does return the machine to the same starting state
//
// A failure here is worth more than a failure anywhere else in the suite,
// because it means the behaviour is not a function of the inputs alone.
//
// SAFETY: MOSFETs stay OFF. Only the COMMANDED position, queue depth and
// telemetry are read, and the planner advances the commanded position
// regardless of the output stage, so nothing turns. No broadcasts and no alias
// changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const int32_t CPR = 3276800;
static const int32_t TPS = 31250;

static void freshStart(Servomotor* m) {
    m->systemReset();
    delay(1500);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
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
        delay(15);
    }
    return false;
}

static uint8_t fatalOf(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

void tm_determinism(void) {
    Serial.println("tm_determinism: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. AN IDENTICAL MOVE TRAVELS AN IDENTICAL DISTANCE. Twenty repetitions
    //    of exactly the same command, each from a fresh reset. The spread
    //    across the twenty is the measurement; a single run tells you nothing.
    // =====================================================================
    printf("\n--- 1. twenty identical moves travel an identical distance ---\n");
    {
        const int REPS = 20;
        const int32_t want = CPR / 8;
        int64_t lo = 0, hi = 0;
        int good = 0, faults = 0;
        for (int i = 0; i < REPS; i++) {
            freshStart(m);
            int64_t before = m->getPositionRaw();
            m->trapezoidMoveRaw(want, TPS / 2);
            if (m->getError() != 0) { faults++; continue; }
            if (!drainQueue(m, 8000))  { faults++; continue; }
            if (fatalOf(m) != 0)       { faults++; continue; }
            int64_t moved = m->getPositionRaw() - before;
            if (good == 0) { lo = hi = moved; }
            else { if (moved < lo) lo = moved; if (moved > hi) hi = moved; }
            good++;
        }
        printf("   %d/%d completed, %d faulted; distance range %lld..%lld (asked %ld)\n",
               good, REPS, faults, (long long)lo, (long long)hi, (long)want);
        TEST_RESULT("all twenty identical moves complete without a fault",
                    good == REPS && faults == 0);
        TEST_RESULT("all twenty travel the same distance to within 2 counts",
                    good > 0 && (hi - lo) <= 2);
        TEST_RESULT("that distance is the one that was asked for",
                    good > 0 && llabs((long long)(lo - want)) <= 2);
    }

    // =====================================================================
    // 2. THE ONE-TICK-RAMP REGIME, REPEATED. This is the exact configuration
    //    that failed about one move in five before 0.15.12.0 -- a slow speed
    //    limit with the factory-default acceleration. Twenty-five repetitions
    //    is enough that the old race would show up with overwhelming
    //    probability (0.8^25 is under 0.4%), so a clean run here is real
    //    evidence rather than luck.
    // =====================================================================
    printf("\n--- 2. the former race regime, twenty-five times ---\n");
    {
        const int REPS = 25;
        const int32_t want = CPR / 100;
        int good = 0, faults = 0;
        uint8_t firstFault = 0;
        int64_t lo = 0, hi = 0;
        for (int i = 0; i < REPS; i++) {
            freshStart(m);
            m->setMaximumVelocity(0.5f);        // with the 12000 rot/s^2 default
            int64_t before = m->getPositionRaw();
            m->trapezoidMoveRaw(want, 2 * TPS);
            if (m->getError() != 0) { faults++; continue; }
            if (!drainQueue(m, 10000)) { faults++; continue; }
            uint8_t f = fatalOf(m);
            if (f != 0) { faults++; if (!firstFault) firstFault = f; continue; }
            int64_t moved = m->getPositionRaw() - before;
            if (good == 0) { lo = hi = moved; }
            else { if (moved < lo) lo = moved; if (moved > hi) hi = moved; }
            good++;
        }
        printf("   %d/%d completed, %d faulted (first fault code %u); range %lld..%lld\n",
               good, REPS, faults, (unsigned)firstFault, (long long)lo, (long long)hi);
        TEST_RESULT("twenty-five moves in the former race regime all complete",
                    good == REPS && faults == 0);
        TEST_RESULT("and all of them travel the same distance",
                    good > 0 && (hi - lo) <= 2);
    }

    // =====================================================================
    // 3. REPEATED READS OF A STATIONARY MACHINE AGREE. With nothing queued
    //    and nothing moving, position must not drift and status must not
    //    flicker. A reading that wanders while the machine is still would
    //    undermine every position assertion in the suite.
    // =====================================================================
    printf("\n--- 3. repeated reads of a stationary machine agree ---\n");
    {
        freshStart(m);
        int64_t first = m->getPositionRaw();
        bool posStable = true, statusStable = true;
        uint8_t firstFatal = fatalOf(m);
        for (int i = 0; i < 40; i++) {
            int64_t p = m->getPositionRaw();
            if (m->getError() != 0 || p != first) {
                posStable = false;
                printf("   position moved while idle: %lld -> %lld on read %d\n",
                       (long long)first, (long long)p, i);
                break;
            }
            uint8_t f = fatalOf(m);
            if (f != firstFatal) { statusStable = false; break; }
            delay(10);
        }
        TEST_RESULT("the commanded position does not drift while the machine is idle",
                    posStable);
        TEST_RESULT("the status does not flicker while the machine is idle", statusStable);

        // Identity reads must be bit-identical every time.
        getProductInfoResponse pi0 = m->getProductInfo();
        bool infoStable = (m->getError() == 0);
        for (int i = 0; i < 10 && infoStable; i++) {
            getProductInfoResponse pi = m->getProductInfo();
            if (m->getError() != 0 || pi.uniqueId != pi0.uniqueId) infoStable = false;
        }
        TEST_RESULT("the unique ID reads back identically ten times", infoStable);

        getFirmwareVersionResponse fv0 = m->getFirmwareVersion();
        bool fwStable = (m->getError() == 0);
        for (int i = 0; i < 10 && fwStable; i++) {
            getFirmwareVersionResponse fv = m->getFirmwareVersion();
            if (m->getError() != 0 ||
                fv.firmwareVersion.major != fv0.firmwareVersion.major ||
                fv.firmwareVersion.minor != fv0.firmwareVersion.minor ||
                fv.firmwareVersion.patch != fv0.firmwareVersion.patch ||
                fv.firmwareVersion.dev   != fv0.firmwareVersion.dev) fwStable = false;
        }
        TEST_RESULT("the firmware version reads back identically ten times", fwStable);
    }

    // =====================================================================
    // 4. THE SAME MOVE FROM THE SAME ORIGIN LANDS AT THE SAME ABSOLUTE
    //    POSITION. Stronger than 1: not just the same distance travelled, but
    //    the same endpoint, so an origin that shifted between runs would be
    //    caught too.
    // =====================================================================
    printf("\n--- 4. the same move lands at the same absolute position ---\n");
    {
        const int REPS = 10;
        int64_t endpoints[REPS];
        int good = 0;
        for (int i = 0; i < REPS; i++) {
            freshStart(m);
            m->zeroPosition();
            if (m->getError() != 0) continue;
            delay(150);
            m->goToPositionRaw(CPR / 8, TPS / 2);
            if (m->getError() != 0) continue;
            if (!drainQueue(m, 8000)) continue;
            if (fatalOf(m) != 0) continue;
            endpoints[good++] = m->getPositionRaw();
        }
        bool same = (good == REPS);
        for (int i = 1; i < good && same; i++) {
            if (llabs((long long)(endpoints[i] - endpoints[0])) > 2) {
                same = false;
                printf("   endpoint %d = %lld, endpoint 0 = %lld\n",
                       i, (long long)endpoints[i], (long long)endpoints[0]);
            }
        }
        printf("   %d/%d runs completed; first endpoint %lld (target %ld)\n",
               good, REPS, good ? (long long)endpoints[0] : 0LL, (long)(CPR / 8));
        TEST_RESULT("all ten zero-then-goto runs complete", good == REPS);
        TEST_RESULT("all ten land at the same absolute position", same);
        TEST_RESULT("that position is the commanded target",
                    good > 0 && llabs((long long)(endpoints[0] - CPR / 8)) <= 2);
    }

    // =====================================================================
    // 5. RESET IS IDEMPOTENT AND COMPLETE. The whole suite depends on reset
    //    giving back the same machine every time; if it did not, every
    //    "fresh start" would be a different starting point and cross-test
    //    interference would be invisible. Checked by resetting from several
    //    DIFFERENT prior states and requiring the same post-reset state.
    // =====================================================================
    printf("\n--- 5. reset returns the same machine from any prior state ---\n");
    {
        // (a) from idle
        freshStart(m);
        int64_t posIdle = m->getPositionRaw();
        uint8_t fatIdle = fatalOf(m);
        uint8_t qIdle = m->getNQueuedItems();

        // (b) from mid-move
        freshStart(m);
        m->trapezoidMoveRaw(CPR, 4 * TPS);
        delay(400);                              // interrupt it deliberately
        freshStart(m);
        int64_t posMid = m->getPositionRaw();
        uint8_t fatMid = fatalOf(m);
        uint8_t qMid = m->getNQueuedItems();

        // (c) from a latched fatal error
        freshStart(m);
        m->setMaximumVelocity(0.2f);
        m->trapezoidMoveRaw(CPR, TPS / 4);       // over the limit -> fatal
        delay(300);
        uint8_t inducedFault = fatalOf(m);
        freshStart(m);
        int64_t posFault = m->getPositionRaw();
        uint8_t fatFault = fatalOf(m);
        uint8_t qFault = m->getNQueuedItems();

        // (d) from a double reset
        freshStart(m);
        freshStart(m);
        int64_t posTwice = m->getPositionRaw();
        uint8_t fatTwice = fatalOf(m);
        uint8_t qTwice = m->getNQueuedItems();

        printf("   idle:%lld/%u/%u  mid:%lld/%u/%u  fault(%u):%lld/%u/%u  twice:%lld/%u/%u\n",
               (long long)posIdle, (unsigned)fatIdle, (unsigned)qIdle,
               (long long)posMid,  (unsigned)fatMid,  (unsigned)qMid,
               (unsigned)inducedFault,
               (long long)posFault,(unsigned)fatFault,(unsigned)qFault,
               (long long)posTwice,(unsigned)fatTwice,(unsigned)qTwice);

        TEST_RESULT("the induced fault really did latch a fatal error", inducedFault != 0);
        TEST_RESULT("reset from idle clears the fatal error and the queue",
                    fatIdle == 0 && qIdle == 0);
        TEST_RESULT("reset from mid-move clears the fatal error and the queue",
                    fatMid == 0 && qMid == 0);
        TEST_RESULT("reset from a latched fatal error clears it and the queue",
                    fatFault == 0 && qFault == 0);
        TEST_RESULT("a second consecutive reset changes nothing",
                    fatTwice == 0 && qTwice == 0);
        // Position after reset must be the same regardless of what came before.
        bool posSame = (llabs((long long)(posMid   - posIdle)) <= 2) &&
                       (llabs((long long)(posFault - posIdle)) <= 2) &&
                       (llabs((long long)(posTwice - posIdle)) <= 2);
        TEST_RESULT("the post-reset position is the same from every prior state", posSame);
    }

    // =====================================================================
    // 6. QUEUEING THE SAME PLAN TWICE GIVES THE SAME QUEUE. The slot cost of
    //    a given plan must be a function of the plan, not of when it was sent.
    //    A cost that varied run to run would make queue-capacity planning
    //    impossible.
    // =====================================================================
    printf("\n--- 6. the same plan always costs the same number of slots ---\n");
    {
        const int REPS = 8;
        uint8_t depths[REPS];
        bool allOk = true;
        for (int i = 0; i < REPS; i++) {
            freshStart(m);
            m->setMaximumVelocity(1.0f);
            m->setMaximumAcceleration(1.0f);     // long ramps: race-free reading
            for (int k = 0; k < 3; k++) m->trapezoidMoveRaw(CPR / 200, 8 * TPS);
            if (m->getError() != 0) { allOk = false; break; }
            depths[i] = m->getNQueuedItems();
            if (m->getError() != 0) { allOk = false; break; }
            m->emergencyStop();
            delay(250);
        }
        bool same = allOk;
        for (int i = 1; i < REPS && same; i++) if (depths[i] != depths[0]) same = false;
        printf("   depths across %d runs: ", REPS);
        for (int i = 0; i < REPS && allOk; i++) printf("%u ", (unsigned)depths[i]);
        printf("\n");
        TEST_RESULT("queueing the same three-move plan always costs the same slots",
                    same);
        TEST_RESULT("and that cost is the expected 9 slots", allOk && depths[0] == 9);
    }

    // =====================================================================
    // 7. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    freshStart(m);
    int64_t bFin = m->getPositionRaw();
    m->trapezoidMoveRaw(CPR / 4, TPS);
    bool finOk = (m->getError() == 0) && drainQueue(m, 8000);
    TEST_RESULT("an ordinary move still works after the determinism sweep",
                finOk && (fatalOf(m) == 0) &&
                llabs((long long)(m->getPositionRaw() - bFin - CPR / 4)) < 200);

    freshStart(m);
}
