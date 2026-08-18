#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_move_composition.cpp
//
// COMPOSITION. Every existing module checks one move at a time. Real callers
// almost never do that -- they queue a sequence and expect the sequence to add
// up. This module checks the algebra of composition:
//
//     * N moves queued back-to-back land where the SUM of them says.
//     * A move decomposed into k pieces lands where the whole one did.
//     * trapezoidMove(d) and goToPosition(p0+d) are the same operation.
//     * Direction reversals inside a sequence cancel exactly.
//     * multimove of k moves == the same k moves issued individually.
//
// These are properties no single-move test can see. An off-by-one in the
// segment arithmetic, an accumulator that saturates, or a queue hand-off that
// drops the last tick would all pass every existing test and fail here.
//
// SAFETY: MOSFETs stay OFF throughout. Every check reads the COMMANDED
// position, which the planner advances regardless of the output stage, so
// nothing physically turns. Rack-safe and load-safe. No broadcasts, no alias
// changes, so it is safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const int32_t CPR = 3276800;    // encoder counts per shaft rotation
static const int32_t TPS = 31250;      // control ticks per second

static void freshStart(Servomotor* m) {
    m->systemReset();
    delay(1500);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
}

// Block until the queue drains, bounded. Never guess a duration: a fixed wait
// silently truncates a long move and reads a partial position, which looks
// exactly like a planner bug.
static bool drainQueue(Servomotor* m, uint32_t budgetMs) {
    uint32_t deadline = millis() + budgetMs;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return false;
        if (q == 0) { delay(250); return true; }
        delay(20);
    }
    return false;
}

static uint8_t fatalOf(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

void tm_move_composition(void) {
    Serial.println("tm_move_composition: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. ADDITIVITY. Queue N moves; the final position must equal the sum of
    //    the individual displacements. Checked at several N so a per-move
    //    constant error (which would scale with N) is distinguishable from a
    //    one-off error (which would not).
    // =====================================================================
    printf("\n--- 1. additivity: N queued moves sum exactly ---\n");
    const int32_t step = CPR / 20;                    // 0.05 rotation each
    const int nCounts[] = { 2, 3, 5, 8 };
    for (unsigned i = 0; i < sizeof(nCounts) / sizeof(nCounts[0]); i++) {
        int n = nCounts[i];
        freshStart(m);
        int64_t before = m->getPositionRaw();
        bool allAccepted = true;
        for (int k = 0; k < n; k++) {
            m->trapezoidMoveRaw(step, TPS / 4);       // 0.25 s each
            if (m->getError() != 0) { allAccepted = false; break; }
        }
        bool drained = allAccepted && drainQueue(m, (uint32_t)n * 400 + 4000);
        int64_t moved = m->getPositionRaw() - before;
        int64_t want  = (int64_t)step * n;
        bool ok = allAccepted && drained && (fatalOf(m) == 0) &&
                  (llabs((long long)(moved - want)) <= 2 * n);
        if (!ok) printf("   n=%d: moved %lld, expected %lld (accepted=%d drained=%d)\n",
                        n, (long long)moved, (long long)want, (int)allAccepted, (int)drained);
        TEST_RESULT(std::string("sum of ") + std::to_string(n) + " queued moves is exact", ok);
    }

    // =====================================================================
    // 2. DECOMPOSITION INVARIANCE. One move of D over T must land in the same
    //    place as k moves of D/k over T/k. This is the property a caller
    //    relies on when it chops a path into segments; if the planner loses a
    //    tick per move, this is where it shows.
    // =====================================================================
    printf("\n--- 2. decomposition invariance ---\n");
    {
        const int32_t whole = CPR / 2;                // 0.5 rotation
        const uint32_t wholeT = 2 * TPS;              // 2 s
        freshStart(m);
        int64_t b1 = m->getPositionRaw();
        m->trapezoidMoveRaw(whole, wholeT);
        bool acc1 = (m->getError() == 0) && drainQueue(m, 8000);
        int64_t movedWhole = m->getPositionRaw() - b1;

        const int ks[] = { 2, 4, 10 };
        for (unsigned i = 0; i < sizeof(ks) / sizeof(ks[0]); i++) {
            int k = ks[i];
            freshStart(m);
            int64_t b2 = m->getPositionRaw();
            bool acc2 = true;
            for (int j = 0; j < k; j++) {
                m->trapezoidMoveRaw(whole / k, wholeT / k);
                if (m->getError() != 0) { acc2 = false; break; }
            }
            bool drained = acc2 && drainQueue(m, wholeT * 1000 / TPS + 6000);
            int64_t movedParts = m->getPositionRaw() - b2;
            // whole/k truncates, so allow the truncation residue plus slack.
            int64_t slack = (int64_t)k * 3 + 4;
            bool ok = acc1 && acc2 && drained && (fatalOf(m) == 0) &&
                      (llabs((long long)(movedParts - movedWhole)) <= slack);
            if (!ok) printf("   k=%d: parts moved %lld, whole moved %lld (slack %lld)\n",
                            k, (long long)movedParts, (long long)movedWhole, (long long)slack);
            TEST_RESULT(std::string("splitting a move into ") + std::to_string(k) +
                            " pieces lands in the same place", ok);
        }
    }

    // =====================================================================
    // 3. trapezoidMove AND goToPosition AGREE. They share the planner (both
    //    call the same queueing helper in firmware 0.15.12.0), so a relative
    //    move of D from p0 must land exactly where an absolute move to p0+D
    //    lands. If they ever diverge, one of the two wrappers is wrong.
    // =====================================================================
    printf("\n--- 3. relative and absolute moves agree ---\n");
    {
        const int32_t deltas[] = { CPR / 8, -CPR / 8, 1000, -1000 };
        for (unsigned i = 0; i < sizeof(deltas) / sizeof(deltas[0]); i++) {
            int32_t d = deltas[i];
            freshStart(m);
            int64_t startA = m->getPositionRaw();
            m->trapezoidMoveRaw(d, TPS);
            bool okA = (m->getError() == 0) && drainQueue(m, 6000);
            int64_t endA = m->getPositionRaw();

            freshStart(m);
            int64_t startB = m->getPositionRaw();
            m->goToPositionRaw((int32_t)(startB + d), TPS);
            bool okB = (m->getError() == 0) && drainQueue(m, 6000);
            int64_t endB = m->getPositionRaw();

            bool ok = okA && okB && (fatalOf(m) == 0) &&
                      (llabs((long long)((endA - startA) - (endB - startB))) <= 2);
            if (!ok) printf("   d=%ld: relative moved %lld, absolute moved %lld\n",
                            (long)d, (long long)(endA - startA), (long long)(endB - startB));
            TEST_RESULT(std::string("relative and absolute agree for d=") +
                            std::to_string((long)d), ok);
        }
    }

    // =====================================================================
    // 4. REVERSAL CANCELS. +D then -D must return exactly to the origin, with
    //    no residue. A rounding error in the segment arithmetic that always
    //    rounds the same way (rather than symmetrically) accumulates here and
    //    nowhere else.
    // =====================================================================
    printf("\n--- 4. reversals cancel exactly ---\n");
    {
        const int32_t amps[] = { 1000, CPR / 16, CPR / 4 };
        for (unsigned i = 0; i < sizeof(amps) / sizeof(amps[0]); i++) {
            int32_t d = amps[i];
            freshStart(m);
            int64_t origin = m->getPositionRaw();
            bool acc = true;
            for (int rep = 0; rep < 3 && acc; rep++) {
                m->trapezoidMoveRaw(d, TPS / 2);
                if (m->getError() != 0) { acc = false; break; }
                m->trapezoidMoveRaw(-d, TPS / 2);
                if (m->getError() != 0) { acc = false; break; }
            }
            bool drained = acc && drainQueue(m, 12000);
            int64_t residue = m->getPositionRaw() - origin;
            bool ok = acc && drained && (fatalOf(m) == 0) && (llabs((long long)residue) <= 4);
            if (!ok) printf("   d=%ld: residue after 3 out-and-back pairs = %lld\n",
                            (long)d, (long long)residue);
            TEST_RESULT(std::string("3 out-and-back pairs of ") + std::to_string((long)d) +
                            " counts leave no residue", ok);
        }
    }

    // =====================================================================
    // 5. MULTIMOVE EQUALS THE SAME MOVES ISSUED ONE BY ONE. Multimove packs
    //    several moves into one packet with a per-move type bit (0 =
    //    acceleration, 1 = velocity -- NOT displacement/duration; see the
    //    MULTIMOVE_COMMAND handler in firmware main.c). It must be a pure
    //    transport optimisation, not a second planner path.
    //
    //    A velocity segment PERSISTS after its time expires, so every velocity
    //    sequence here ends with an explicit zero-velocity segment. That is
    //    the same trailing stop the firmware itself appends in the delta_t1<=1
    //    path of queue_trapezoid_segments().
    // =====================================================================
    printf("\n--- 5. multimove matches individually-issued moves ---\n");
    {
        // Use the library's own converters so the raw values are exactly what
        // the converted API would have sent -- no hand-rolled shift constants.
        const int32_t v0 = (int32_t)convertVelocity(1.0f,  VelocityUnit::ROTATIONS_PER_SECOND,
                                                    ConversionDirection::TO_INTERNAL);
        const int32_t v1 = (int32_t)convertVelocity(-0.5f, VelocityUnit::ROTATIONS_PER_SECOND,
                                                    ConversionDirection::TO_INTERNAL);
        const uint32_t tA = (uint32_t)convertTime(0.4f, TimeUnit::SECONDS,
                                                  ConversionDirection::TO_INTERNAL);
        const uint32_t tB = (uint32_t)convertTime(0.4f, TimeUnit::SECONDS,
                                                  ConversionDirection::TO_INTERNAL);

        freshStart(m);
        int64_t bIndiv = m->getPositionRaw();
        m->moveWithVelocityRaw(v0, tA);
        m->moveWithVelocityRaw(v1, tB);
        m->moveWithVelocityRaw(0, 1);              // explicit stop
        bool okI = (m->getError() == 0) && drainQueue(m, 8000);
        int64_t movedIndiv = m->getPositionRaw() - bIndiv;

        freshStart(m);
        int64_t bMulti = m->getPositionRaw();
        multimoveList_t mv[3];
        mv[0].value = v0; mv[0].timeSteps = tA;
        mv[1].value = v1; mv[1].timeSteps = tB;
        mv[2].value = 0;  mv[2].timeSteps = 1;     // explicit stop
        // Bit k == 1 means move k is a VELOCITY move. All three are.
        m->multimoveRaw(3, 0x00000007, mv);
        bool okM = (m->getError() == 0) && drainQueue(m, 8000);
        int64_t movedMulti = m->getPositionRaw() - bMulti;

        // Physically: 1 rot/s for 0.4 s then -0.5 rot/s for 0.4 s = +0.2 rot.
        const int64_t wantPhysical = (int64_t)(0.2 * (double)CPR);
        printf("   individually: %lld   multimove: %lld   (physically ~%lld)\n",
               (long long)movedIndiv, (long long)movedMulti, (long long)wantPhysical);
        TEST_RESULT("multimove moves the same distance as the moves issued singly",
                    okI && okM && (llabs((long long)(movedMulti - movedIndiv)) <= CPR / 500));
        TEST_RESULT("multimove of velocity segments travels the physically expected distance",
                    okM && (fatalOf(m) == 0) &&
                    (llabs((long long)(movedMulti - wantPhysical)) <= CPR / 50));
    }

    // =====================================================================
    // 5b. MIXED-TYPE MULTIMOVE. The type bitmask is per-move, so a single
    //     packet may mix acceleration and velocity segments. That mixing is
    //     the whole reason the bitmask exists and nothing else exercises it.
    //     Accelerate for a while, then hold a velocity, then stop.
    // =====================================================================
    printf("\n--- 5b. mixed acceleration/velocity multimove ---\n");
    {
        freshStart(m);
        int64_t before = m->getPositionRaw();
        const int32_t accel = (int32_t)convertAcceleration(
                                  2.0f, AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED,
                                  ConversionDirection::TO_INTERNAL);
        const uint32_t tAcc = (uint32_t)convertTime(0.25f, TimeUnit::SECONDS,
                                                    ConversionDirection::TO_INTERNAL);
        const uint32_t tHold = (uint32_t)convertTime(0.25f, TimeUnit::SECONDS,
                                                     ConversionDirection::TO_INTERNAL);
        multimoveList_t mv[4];
        mv[0].value = accel;  mv[0].timeSteps = tAcc;   // accelerate up
        mv[1].value = 0;      mv[1].timeSteps = tHold;  // coast (accel 0)
        mv[2].value = -accel; mv[2].timeSteps = tAcc;   // decelerate back down
        mv[3].value = 0;      mv[3].timeSteps = 1;      // velocity 0: full stop
        // bits 0,1,2 = 0 (acceleration), bit 3 = 1 (velocity).
        m->multimoveRaw(4, 0x00000008, mv);
        bool ok = (m->getError() == 0) && drainQueue(m, 8000);
        int64_t moved = m->getPositionRaw() - before;
        // Symmetric ramp up/down about a coast: net displacement is positive
        // and bounded. Exact value depends on the integrator, so check sign
        // and a generous range rather than a precise figure.
        bool sane = ok && (fatalOf(m) == 0) && (moved > 0) && (moved < (int64_t)CPR * 3);
        printf("   mixed multimove moved %lld counts (ok=%d)\n", (long long)moved, (int)ok);
        TEST_RESULT("a multimove mixing acceleration and velocity segments runs cleanly", sane);

        // And it must come to rest: after the trailing zero-velocity segment,
        // the commanded position must stop advancing.
        int64_t p1 = m->getPositionRaw();
        delay(500);
        int64_t p2 = m->getPositionRaw();
        TEST_RESULT("the trailing zero-velocity segment actually stops the motion",
                    llabs((long long)(p2 - p1)) <= 2);
    }

    // =====================================================================
    // 6. QUEUE DEPTH IS CONSISTENT WITH WHAT WAS QUEUED. Not the exact slot
    //    cost (that is tm_queue_capacity's job) but the ordering property:
    //    depth must be non-increasing while nothing new is queued, and must
    //    reach zero. A depth that sticks above zero is a lost move.
    // =====================================================================
    printf("\n--- 6. queue depth decreases monotonically to zero ---\n");
    {
        freshStart(m);
        for (int k = 0; k < 4; k++) m->trapezoidMoveRaw(CPR / 40, TPS / 4);
        bool accepted = (m->getError() == 0);
        uint8_t prev = m->getNQueuedItems();
        bool monotonic = true, reachedZero = false;
        for (uint32_t t0 = millis(); millis() - t0 < 6000; ) {
            uint8_t q = m->getNQueuedItems();
            if (m->getError() != 0) { monotonic = false; break; }
            if (q > prev) { monotonic = false;
                            printf("   depth went UP: %u -> %u\n", (unsigned)prev, (unsigned)q); }
            prev = q;
            if (q == 0) { reachedZero = true; break; }
            delay(40);
        }
        TEST_RESULT("queue depth never increases on its own", accepted && monotonic);
        TEST_RESULT("queue depth reaches zero after the moves finish", reachedZero);
    }

    // =====================================================================
    // 7. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    freshStart(m);
    int64_t bFin = m->getPositionRaw();
    m->trapezoidMoveRaw(CPR / 4, TPS);
    bool finOk = (m->getError() == 0) && drainQueue(m, 6000);
    TEST_RESULT("an ordinary move still works after the composition sweep",
                finOk && (fatalOf(m) == 0) &&
                llabs((long long)(m->getPositionRaw() - bFin - CPR / 4)) < 200);

    freshStart(m);
}
