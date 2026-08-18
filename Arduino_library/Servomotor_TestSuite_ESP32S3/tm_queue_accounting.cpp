#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_queue_accounting.cpp
//
// EXACT SLOT COST of a move. tm_queue_edges finds the 32-item boundary and
// tm_queue_stress fills and drains the queue, but neither pins down how many
// slots ONE move actually consumes -- they both say "up to 3". That number is
// not decoration: it is the difference between a 10-move plan fitting in the
// queue and faulting with ERROR_QUEUE_IS_FULL, and firmware 0.15.12.0 changed
// it.
//
// GROUND TRUTH (firmware/Src/motor_control.c, queue_trapezoid_segments):
//
//   normal case      delta_t1 >= 2   -> THREE slots: ramp, coast, ramp
//   one-tick ramp    delta_t1 <= 1   -> TWO slots:   velocity hold, 1-tick stop
//     and total_time >= 3
//
// The two-slot path is the 0.15.12.0 fix for the queue-fill race. It has a
// crisp, directly observable signature -- the depth after queueing one move is
// 2 instead of 3 -- and nothing in the suite looks at it. If a future change
// silently reverts to three add_to_queue() calls on a one-tick ramp, the race
// comes back and only this module notices.
//
// MEASURING IT RELIABLY: the segments start executing the moment they are
// queued, so a naive read races the ISR. Every measurement below first makes
// the segments LONG -- by lowering the acceleration limit, so the ramp is
// ~1 second rather than ~0.8 ms -- and only then reads the depth. The queue
// then cannot drain during the round trip.
//
// SAFETY: MOSFETs stay OFF. Only queue depth and the COMMANDED position are
// read, and both advance regardless of the output stage, so nothing turns.
// No broadcasts and no alias changes: safe on the shared 35-motor rack.
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
        if (q == 0) { delay(200); return true; }
        delay(20);
    }
    return false;
}

static uint8_t fatalOf(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

// Queue one trapezoid move under the given limits and report the queue depth
// immediately afterwards. Uses a SLOW acceleration limit so each segment lasts
// about a second, which makes the reading race-free.
static int slotsForOneMove(Servomotor* m, float maxVel, float maxAccel,
                           int32_t counts, uint32_t ticks, uint8_t* fatal) {
    freshStart(m);
    m->setMaximumVelocity(maxVel);
    m->setMaximumAcceleration(maxAccel);
    if (m->getError() != 0) { *fatal = 0xFE; return -1; }
    m->trapezoidMoveRaw(counts, ticks);
    if (m->getError() != 0) { *fatal = 0xFD; return -1; }
    uint8_t depth = m->getNQueuedItems();
    if (m->getError() != 0) { *fatal = 0xFC; return -1; }
    *fatal = 0;
    return (int)depth;
}

void tm_queue_accounting(void) {
    Serial.println("tm_queue_accounting: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE NORMAL PATH COSTS EXACTLY THREE SLOTS.
    //    maxVel 1 rot/s with maxAccel 1 rot/s^2 gives delta_t1 = 1 second =
    //    31250 ticks, far above the one-tick threshold, and long enough that
    //    the queue cannot drain while we read it.
    // =====================================================================
    printf("\n--- 1. an ordinary trapezoid move costs exactly 3 slots ---\n");
    {
        uint8_t f = 0;
        int slots = slotsForOneMove(m, 1.0f, 1.0f, CPR / 4, 4 * TPS, &f);
        printf("   maxVel=1 rot/s maxAccel=1 rot/s^2, 0.25 rot over 4 s -> depth %d (f=%u)\n",
               slots, (unsigned)f);
        TEST_RESULT("a normal trapezoid move queues exactly 3 items", slots == 3);
        drainQueue(m, 12000);
    }

    // =====================================================================
    // 2. THE ONE-TICK-RAMP PATH COSTS EXACTLY TWO SLOTS.
    //    delta_t1 = maxVelocity/maxAcceleration. With the 12000 rot/s^2
    //    factory default and a slow speed limit, that integer division lands
    //    at 0 or 1 -- the 0.15.12.0 fast path -- and the move becomes a
    //    velocity hold plus a one-tick stop.
    //
    //    These are not exotic settings. A 0.3 rot/s limit on a slow, precise
    //    axis with the untouched default acceleration is exactly this case.
    // =====================================================================
    printf("\n--- 2. a one-tick ramp costs exactly 2 slots (the 0.15.12.0 path) ---\n");
    {
        const float slowVels[] = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f };
        for (unsigned i = 0; i < sizeof(slowVels) / sizeof(slowVels[0]); i++) {
            uint8_t f = 0;
            int slots = slotsForOneMove(m, slowVels[i], 12000.0f, CPR / 100, 4 * TPS, &f);
            printf("   maxVel=%.1f rot/s, default accel -> depth %d (f=%u)\n",
                   (double)slowVels[i], slots, (unsigned)f);
            TEST_RESULT(std::string("maxVel=") + std::to_string((double)slowVels[i]).substr(0, 3) +
                            " rot/s takes the 2-slot fast path",
                        slots == 2);
            drainQueue(m, 12000);
        }
    }

    // =====================================================================
    // 3. THE TWO PATHS AGREE ON WHERE THE SHAFT ENDS UP. The fast path is an
    //    optimisation, not a different move: whichever way the planner splits
    //    it, the same displacement must be delivered. A slot count that
    //    changed the DESTINATION would be a far worse bug than the race it
    //    was introduced to fix.
    // =====================================================================
    printf("\n--- 3. both paths deliver the same displacement ---\n");
    {
        const int32_t want = CPR / 100;
        // Three-slot path.
        freshStart(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        int64_t b3 = m->getPositionRaw();
        m->trapezoidMoveRaw(want, 4 * TPS);
        bool ok3 = (m->getError() == 0) && drainQueue(m, 15000);
        int64_t moved3 = m->getPositionRaw() - b3;

        // Two-slot path.
        freshStart(m);
        m->setMaximumVelocity(0.3f);
        m->setMaximumAcceleration(12000.0f);
        int64_t b2 = m->getPositionRaw();
        m->trapezoidMoveRaw(want, 4 * TPS);
        bool ok2 = (m->getError() == 0) && drainQueue(m, 15000);
        int64_t moved2 = m->getPositionRaw() - b2;

        printf("   3-slot path moved %lld, 2-slot path moved %lld, asked %ld\n",
               (long long)moved3, (long long)moved2, (long)want);
        TEST_RESULT("the 3-slot path delivers the commanded displacement",
                    ok3 && llabs((long long)(moved3 - want)) <= want / 100 + 2);
        TEST_RESULT("the 2-slot path delivers the commanded displacement",
                    ok2 && llabs((long long)(moved2 - want)) <= want / 100 + 2);
        TEST_RESULT("the two paths agree with each other",
                    ok2 && ok3 && llabs((long long)(moved2 - moved3)) <= want / 50 + 4);
    }

    // =====================================================================
    // 4. SLOT COST IS ADDITIVE. N moves queued back to back occupy N times the
    //    per-move cost. This is the property a caller needs in order to plan
    //    how many moves fit, and it is what makes the 3-vs-2 distinction
    //    matter: at 3 slots only 10 moves fit in 32, at 2 slots 16 do.
    // =====================================================================
    printf("\n--- 4. slot cost is additive across queued moves ---\n");
    {
        // Three-slot regime: 5 moves must occupy 15 slots.
        freshStart(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        for (int k = 0; k < 5; k++) m->trapezoidMoveRaw(CPR / 200, 4 * TPS);
        bool acc = (m->getError() == 0);
        uint8_t d3 = m->getNQueuedItems();
        printf("   5 moves in the 3-slot regime -> depth %u (expected 15)\n", (unsigned)d3);
        TEST_RESULT("5 normal moves occupy 15 slots", acc && d3 == 15);
        m->emergencyStop();
        delay(300);

        // Two-slot regime: 5 moves must occupy 10 slots.
        freshStart(m);
        m->setMaximumVelocity(0.3f);
        m->setMaximumAcceleration(12000.0f);
        for (int k = 0; k < 5; k++) m->trapezoidMoveRaw(CPR / 200, 4 * TPS);
        bool acc2 = (m->getError() == 0);
        uint8_t d2 = m->getNQueuedItems();
        printf("   5 moves in the 2-slot regime -> depth %u (expected 10)\n", (unsigned)d2);
        TEST_RESULT("5 fast-path moves occupy 10 slots", acc2 && d2 == 10);
        m->emergencyStop();
        delay(300);
    }

    // =====================================================================
    // 5. HOW MANY MOVES ACTUALLY FIT. The practical consequence: in the
    //    3-slot regime the 11th move must not fit (33 > 32), while in the
    //    2-slot regime 16 moves fit exactly and the 17th must not. Getting
    //    this wrong is what makes a plan fault in the field.
    // =====================================================================
    printf("\n--- 5. capacity in moves, both regimes ---\n");
    {
        // 3-slot regime: 10 moves = 30 slots, fits. 11th = 33, must fault.
        freshStart(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        int accepted = 0;
        for (int k = 0; k < 11; k++) {
            m->trapezoidMoveRaw(CPR / 400, 8 * TPS);
            if (m->getError() != 0) break;
            if (fatalOf(m) != 0) break;
            accepted++;
        }
        printf("   3-slot regime accepted %d moves before faulting\n", accepted);
        TEST_RESULT("in the 3-slot regime exactly 10 moves fit in the 32-item queue",
                    accepted == 10);
        freshStart(m);

        // 2-slot regime: 16 moves = 32 slots, fits exactly. 17th must fault.
        m->setMaximumVelocity(0.3f);
        m->setMaximumAcceleration(12000.0f);
        int accepted2 = 0;
        for (int k = 0; k < 17; k++) {
            m->trapezoidMoveRaw(CPR / 400, 8 * TPS);
            if (m->getError() != 0) break;
            if (fatalOf(m) != 0) break;
            accepted2++;
        }
        printf("   2-slot regime accepted %d moves before faulting\n", accepted2);
        TEST_RESULT("in the 2-slot regime exactly 16 moves fit in the 32-item queue",
                    accepted2 == 16);
        freshStart(m);
    }

    // =====================================================================
    // 6. THE FAST PATH NEEDS total_time >= 3. Below that the guard falls
    //    through to the three-segment form, which is the whole reason the
    //    condition is written as a conjunction. Very short moves must still
    //    behave -- either performed or refused, never silently wrong.
    // =====================================================================
    printf("\n--- 6. the short-duration corner of the fast-path guard ---\n");
    {
        const uint32_t shortT[] = { 1, 2, 3, 4 };
        for (unsigned i = 0; i < sizeof(shortT) / sizeof(shortT[0]); i++) {
            freshStart(m);
            m->setMaximumVelocity(0.3f);
            m->setMaximumAcceleration(12000.0f);
            int64_t before = m->getPositionRaw();
            m->trapezoidMoveRaw(10, shortT[i]);
            bool accepted = (m->getError() == 0);
            delay(300);
            uint8_t fat = fatalOf(m);
            int64_t moved = (fat == 0 && accepted) ? (m->getPositionRaw() - before) : 0;
            // Accepted with no fault => it must have moved what was asked.
            bool sane = (!accepted) || (fat != 0) || (llabs((long long)(moved - 10)) <= 2);
            printf("   total_time=%lu: accepted=%d fatal=%u moved=%lld\n",
                   (unsigned long)shortT[i], (int)accepted, (unsigned)fat, (long long)moved);
            TEST_RESULT(std::string("total_time=") + std::to_string((unsigned long)shortT[i]) +
                            " is performed correctly or refused, never silently wrong",
                        sane);
        }
    }

    // =====================================================================
    // 7. EMERGENCY STOP EMPTIES THE QUEUE COMPLETELY, in both regimes. A
    //    stop that left one slot occupied would strand the next plan.
    // =====================================================================
    printf("\n--- 7. emergency stop empties the queue in both regimes ---\n");
    {
        struct { const char* name; float vel; float acc; } regime[] = {
            { "3-slot", 1.0f, 1.0f },
            { "2-slot", 0.3f, 12000.0f },
        };
        for (int r = 0; r < 2; r++) {
            freshStart(m);
            m->setMaximumVelocity(regime[r].vel);
            m->setMaximumAcceleration(regime[r].acc);
            for (int k = 0; k < 4; k++) m->trapezoidMoveRaw(CPR / 200, 8 * TPS);
            uint8_t filled = m->getNQueuedItems();
            m->emergencyStop();
            delay(400);
            uint8_t after = m->getNQueuedItems();
            bool ok = (m->getError() == 0) && (filled > 0) && (after == 0);
            printf("   %s regime: depth %u -> emergency stop -> %u\n",
                   regime[r].name, (unsigned)filled, (unsigned)after);
            TEST_RESULT(std::string("emergency stop empties the queue in the ") +
                            regime[r].name + " regime", ok);
        }
    }

    // =====================================================================
    // 8. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 8. still healthy afterwards ---\n");
    freshStart(m);
    int64_t bFin = m->getPositionRaw();
    m->trapezoidMoveRaw(CPR / 4, TPS);
    bool finOk = (m->getError() == 0) && drainQueue(m, 8000);
    TEST_RESULT("an ordinary move still works after the accounting sweep",
                finOk && (fatalOf(m) == 0) &&
                llabs((long long)(m->getPositionRaw() - bFin - CPR / 4)) < 200);

    freshStart(m);
}
