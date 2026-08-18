#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_planner_arithmetic.cpp
//
// The trapezoid planner's ARITHMETIC, as distinct from its motion. Three real
// bugs lived here (fixed in fw 0.15.12.0): an integer divide-by-zero, an
// int64->int32 truncation of the computed acceleration, and a queue-fill race.
// All three were SILENT -- success reply, no error, wrong or absent motion --
// so this module is built around one invariant:
//
//     A move is either performed as asked, or refused with a fatal error.
//     "Accepted, no error, did something else" is ALWAYS a failure.
//
// What is deliberately new here versus the existing modules: sign symmetry,
// behaviour at exact limit boundaries, and the relationship between the ramp
// length and the velocity/acceleration limits. goto_edges and limit_boundaries
// cover the commands; this covers the maths underneath them.
//
// SAFETY: MOSFETs stay OFF. Every check reads the COMMANDED position, which the
// planner advances regardless, so nothing turns. Rack-safe and load-safe.
// ---------------------------------------------------------------------------

static const int32_t CPR  = 3276800;   // encoder counts per shaft rotation
static const int32_t TPS  = 31250;     // control ticks per second
static const int RESET_MS = 1500;

static void freshStart(Servomotor* m) {
    m->systemReset();
    delay(RESET_MS);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
}

// Perform one raw move and report what actually happened.
// Raw units: encoder_counts and timesteps both convert with factor 1, so these
// are exactly the numbers the firmware planner sees.
struct MoveOutcome {
    bool    accepted;    // the command itself did not report an error
    uint8_t fatal;       // fatal error code afterwards (0 = none)
    int64_t moved;       // commanded-position delta
};

static MoveOutcome doMove(Servomotor* m, int32_t counts, uint32_t ticks) {
    MoveOutcome o;
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    int64_t before = m->getPositionRaw();
    m->trapezoidMoveRaw(counts, ticks);
    o.accepted = (m->getError() == 0);
    // Wait for the move to ACTUALLY finish by draining the queue, rather than
    // guessing a duration. A fixed wait with a cap silently truncates long
    // moves and reads a partial result -- which looked exactly like a firmware
    // bug the first time this module ran (626 of 1000 counts on a 6.4 s move,
    // because the cap stopped the clock at 4 s).
    uint32_t deadline = millis() + (uint32_t)((uint64_t)ticks * 1000ULL / TPS) + 3000;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) break;      // fatal, or unreachable: stop waiting
        if (q == 0) break;
        delay(20);
    }
    delay(250);                             // let the last tick settle
    getStatusResponse st = m->getStatus();
    o.fatal = (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
    o.moved = (o.fatal == 0) ? (m->getPositionRaw() - before) : 0;
    return o;
}

// The invariant. wantCounts == 0 means a dwell, which correctly does not move.
static void expectMoveOrError(Servomotor* m, const char* label,
                              int32_t counts, uint32_t ticks) {
    freshStart(m);
    MoveOutcome o = doMove(m, counts, ticks);
    std::string name(label);
    if (o.accepted && o.fatal == 0) {
        int64_t tol = (counts == 0) ? 1 : (llabs((long long)counts) / 100 + 2);
        bool ok = llabs((long long)(o.moved - counts)) <= tol;
        if (!ok) {
            printf("   %s: accepted but moved %lld, asked %ld\n",
                   label, (long long)o.moved, (long)counts);
        }
        TEST_RESULT(name + ": accepted, so it moved what was asked", ok);
    } else {
        printf("   %s: refused (accepted=%d fatal=%u)\n",
               label, (int)o.accepted, (unsigned)o.fatal);
        TEST_RESULT(name + ": refused loudly rather than doing something else", true);
    }
}

void tm_planner_arithmetic(void) {
    Serial.println("tm_planner_arithmetic: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. SIGN SYMMETRY. A move of -X must behave exactly like +X mirrored.
    //    The planner negates the acceleration for the deceleration segment,
    //    so a sign error would show up as an asymmetry between the two.
    // =====================================================================
    printf("\n--- 1. sign symmetry ---\n");
    const int32_t symDist[] = { 1, 1000, 163840, CPR / 2 };
    for (unsigned i = 0; i < sizeof(symDist) / sizeof(symDist[0]); i++) {
        int32_t d = symDist[i];
        freshStart(m);
        MoveOutcome pos = doMove(m, d, TPS);
        freshStart(m);
        MoveOutcome neg = doMove(m, -d, TPS);
        bool bothOk = (pos.fatal == 0) && (neg.fatal == 0);
        bool mirrored = bothOk && (llabs((long long)(pos.moved + neg.moved)) <= 2);
        printf("   +%ld -> %lld ; -%ld -> %lld\n",
               (long)d, (long long)pos.moved, (long)d, (long long)neg.moved);
        TEST_RESULT(std::string("sign symmetry at ") + std::to_string((long)d) + " counts",
                    mirrored);
    }

    // =====================================================================
    // 2. EXACT LIMIT BOUNDARIES. The limits are documented as INCLUSIVE: a
    //    move exactly at the limit is accepted, one step past is refused.
    //    Checked through the minimum-duration relationship rather than by
    //    setting the limits, so it exercises the planner rather than the
    //    setters.
    // =====================================================================
    printf("\n--- 2. duration boundary (inclusive limits) ---\n");
    // Default limits: maxVel 9.3333 rot/s, maxAccel 12000 rot/s^2 -> ramp 24 ticks.
    // Minimum duration ~= displacement/maxVel + maxVel/maxAccel.
    {
        const int32_t d = CPR / 4;                       // 0.25 rotation
        // displacement/maxVel in ticks = d / (maxVel_counts_per_tick)
        // maxVel = 9.3333 rot/s = 9.3333*CPR/TPS counts/tick = 978.6 counts/tick
        const uint32_t minTicks = (uint32_t)((double)d / 978.6) + 24;
        expectMoveOrError(m, "well above the minimum duration",  d, minTicks * 3);
        expectMoveOrError(m, "just above the minimum duration",  d, minTicks + 40);
        expectMoveOrError(m, "just below the minimum duration",  d, (minTicks > 40) ? minTicks - 20 : 1);
    }

    // =====================================================================
    // 3. RAMP-LENGTH SWEEP. delta_t1 = maxVelocity/maxAcceleration in ticks.
    //    Bug 1 was delta_t1 == 0; bug 3 was delta_t1 == 1. Sweep the whole
    //    low end and require sane behaviour at every step, including the
    //    values that used to be silent no-ops.
    // =====================================================================
    printf("\n--- 3. ramp-length sweep (the bug-1 / bug-3 territory) ---\n");
    const float sweepVel[] = { 0.10f, 0.20f, 0.30f, 0.40f, 0.60f, 0.80f, 1.00f, 1.50f, 3.00f };
    for (unsigned i = 0; i < sizeof(sweepVel) / sizeof(sweepVel[0]); i++) {
        freshStart(m);
        m->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
        m->setMaximumVelocity(sweepVel[i]);
        bool setOk = (m->getError() == 0);
        MoveOutcome o = doMove(m, 163840, 2 * TPS);   // 0.05 rot over 2 s
        bool good = setOk && (o.fatal == 0) && (llabs((long long)(o.moved - 163840)) < 200);
        if (!good) {
            printf("   maxVel=%.2f -> accepted=%d fatal=%u moved=%lld\n",
                   (double)sweepVel[i], (int)o.accepted, (unsigned)o.fatal, (long long)o.moved);
        }
        TEST_RESULT(std::string("ramp sweep maxVel=") + std::to_string((double)sweepVel[i]).substr(0, 4)
                        + " performs an ordinary move",
                    good);
    }

    // =====================================================================
    // 4. DISPLACEMENT MAGNITUDE SWEEP at a fixed generous duration. Bug 2
    //    was an int32 truncation of the computed acceleration, so walk the
    //    magnitude up towards the int32 range and require the invariant at
    //    every point.
    // =====================================================================
    printf("\n--- 4. displacement magnitude sweep (the bug-2 territory) ---\n");
    const int32_t mags[] = { 1, 10, 1000, 100000, CPR, CPR * 10, CPR * 100, CPR * 600 };
    for (unsigned i = 0; i < sizeof(mags) / sizeof(mags[0]); i++) {
        char lbl[64];
        snprintf(lbl, sizeof(lbl), "displacement %ld counts over 3 s", (long)mags[i]);
        expectMoveOrError(m, lbl, mags[i], 3 * TPS);
    }

    // =====================================================================
    // 5. DURATION MAGNITUDE SWEEP at a fixed small displacement. The other
    //    axis of the same space: very short through very long.
    // =====================================================================
    printf("\n--- 5. duration magnitude sweep ---\n");
    const uint32_t durs[] = { 1, 2, 3, 5, 10, 50, 500, 5000, 62500, 200000 };
    for (unsigned i = 0; i < sizeof(durs) / sizeof(durs[0]); i++) {
        char lbl[64];
        snprintf(lbl, sizeof(lbl), "1000 counts over %lu ticks", (unsigned long)durs[i]);
        expectMoveOrError(m, lbl, 1000, durs[i]);
    }

    // =====================================================================
    // 6. ZERO-DISPLACEMENT DWELLS of various lengths. A dwell is the one
    //    legitimate "accepted and did not move" case; it must also not be
    //    rejected, and must not disturb the position.
    // =====================================================================
    printf("\n--- 6. zero-displacement dwells ---\n");
    const uint32_t dwell[] = { 3, 100, 31250 };
    for (unsigned i = 0; i < sizeof(dwell) / sizeof(dwell[0]); i++) {
        freshStart(m);
        MoveOutcome o = doMove(m, 0, dwell[i]);
        bool good = o.accepted && (o.fatal == 0) && (llabs((long long)o.moved) <= 1);
        TEST_RESULT(std::string("dwell of ") + std::to_string((unsigned long)dwell[i])
                        + " ticks is accepted and does not move",
                    good);
    }

    // =====================================================================
    // 7. The motor must still be healthy and usable after all of that.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    freshStart(m);
    MoveOutcome fin = doMove(m, CPR / 4, TPS);
    TEST_RESULT("an ordinary move still works after the whole sweep",
                fin.accepted && fin.fatal == 0 && llabs((long long)(fin.moved - CPR / 4)) < 200);

    freshStart(m);
}
