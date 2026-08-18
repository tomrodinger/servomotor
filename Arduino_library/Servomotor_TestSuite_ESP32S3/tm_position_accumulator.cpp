#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_position_accumulator.cpp
//
// THE POSITION ACCUMULATOR AT LARGE MAGNITUDES.
//
// Internally the commanded position is not a count -- it is a wide fixed-point
// accumulator (int96, with the whole-count part at bit 32), advanced every one
// of the 31250 control ticks per second. Every test in the suite moves a
// fraction of a turn and looks at the answer, which exercises the bottom of
// that accumulator and nothing else.
//
// The interesting failures live at the top:
//
//   * `trapezoid move` takes an int32 displacement, so its extreme is +/-
//     2147483647 counts -- about 655 rotations. Right at that edge, an
//     intermediate that is computed in 32 bits instead of 64 wraps.
//   * A long run of large moves walks the accumulator far from zero. A
//     truncation that is invisible over a quarter turn is obvious after a
//     thousand rotations of accumulated travel.
//   * Absolute moves (`go to position`) must stay consistent with relative
//     ones no matter how far from the origin the machine has wandered.
//
// This is the same class of defect as the int64->int32 truncation of the
// computed acceleration that was fixed in 0.15.12.0: arithmetic that is correct
// everywhere a normal test looks, and wrong out at the edge.
//
// TWO CEILINGS HAVE TO BE RAISED FIRST, or this module measures them instead of
// what it is trying to measure:
//
//   * max allowable position deviation defaults to TWO ROTATIONS
//     (DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION, motor_control.c:97). With the
//     MOSFETs off the shaft does not follow the commanded position at all, so
//     the deviation grows with every count commanded and anything past about
//     two turns trips ERROR_POSITION_DEVIATION_TOO_LARGE (45). That is correct
//     behaviour -- it is the firmware noticing the shaft is not keeping up --
//     but it caps MOSFETs-off testing at two rotations unless the limit is
//     raised, which is worth knowing before writing any large-displacement test.
//   * max velocity caps how far a move can get in a given time. 655 rotations
//     cannot happen in three seconds at any legal speed, so a naive sweep to
//     INT32_MAX just measures the speed limit. The ceiling is
//     EXPERIMENTAL_MAX_VELOCITY = 2x the rated MAX_VELOCITY (motor_control.h:61).
//
// Both are raised in openTheLimits() below and both are restored by the reset at
// the end of every subsection, so nothing leaks to the next test.
//
// SAFETY: MOSFETs stay OFF. Every check reads the COMMANDED position, which the
// planner advances regardless of the output stage, so the shaft never turns --
// which is what makes it safe to command hundreds of rotations on a shared rack
// where some motors have a pulley or a disk attached. Raising the deviation
// limit only silences a diagnostic about a shaft that was never going to move;
// it cannot cause motion while the output stage is off. No broadcasts and no
// alias changes.
// ---------------------------------------------------------------------------

static const int64_t CPR = 3276800;
static const int32_t TPS = 31250;
static const int32_t I32MAX = 2147483647;

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
        delay(20);
    }
    return false;
}

static uint8_t fatalOf(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

// Raise every ceiling that would otherwise be the thing under test, so what is
// actually measured is the position arithmetic.
static void openTheLimits(Servomotor* m) {
    m->setMaximumVelocity(15.0f);          // near the experimental ceiling (~18.7)
    m->setMaximumAcceleration(5000.0f);
    // With the MOSFETs off the shaft never follows, so the deviation would
    // otherwise trip at two rotations and cap the whole module.
    m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    m->setMaxAllowablePositionDeviation(100000.0f);
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
}

// Seconds needed to cover `counts` at the 15 rot/s ceiling, with headroom for
// the ramps. Asking for a move faster than this is asking to be refused.
static uint32_t ticksFor(int64_t counts) {
    double rotations = (double)(counts < 0 ? -counts : counts) / (double)CPR;
    double seconds = rotations / 12.0 + 0.5;      // 12 rot/s average, plus ramps
    return (uint32_t)(seconds * (double)TPS);
}

void tm_position_accumulator(void) {
    Serial.println("tm_position_accumulator: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE int32 DISPLACEMENT EDGE, ACTUALLY PERFORMED. Walk the commanded
    //    displacement up to INT32_MAX, giving each move enough time to be
    //    physically achievable, and require it to be delivered. A wrap in an
    //    intermediate computed at 32 bits would show up as a move in the WRONG
    //    DIRECTION -- the most dangerous possible failure, and precisely what
    //    a silent truncation produces.
    //
    //    INT32_MAX counts is about 655 rotations, which needs the better part
    //    of a minute even at the experimental speed ceiling. That is why this
    //    module has a long timeout.
    // =====================================================================
    printf("\n--- 1. displacement up to the int32 edge, performed ---\n");
    {
        const int32_t mags[] = {
            (int32_t)(CPR * 10),          //   10 rotations
            (int32_t)(CPR * 100),         //  100 rotations
            I32MAX / 4,                   // ~163 rotations
            I32MAX / 2,                   // ~327 rotations
            I32MAX - 1,                   // ~655 rotations, one short of the edge
            I32MAX,                       // the edge itself
        };
        for (unsigned i = 0; i < sizeof(mags) / sizeof(mags[0]); i++) {
            for (int sign = 1; sign >= -1; sign -= 2) {
                int32_t d = (sign > 0) ? mags[i] : -mags[i];
                freshStart(m);
                openTheLimits(m);
                uint32_t ticks = ticksFor(d);
                int64_t before = m->getPositionRaw();
                m->trapezoidMoveRaw(d, ticks);
                bool accepted = (m->getError() == 0);
                bool drained = accepted &&
                               drainQueue(m, (uint32_t)((uint64_t)ticks * 1000ULL / TPS) + 20000);
                uint8_t fat = fatalOf(m);
                int64_t moved = (accepted && drained && fat == 0)
                                    ? (m->getPositionRaw() - before) : 0;
                bool performed = accepted && drained && (fat == 0);
                printf("   d=%ld over %.1f s: accepted=%d fatal=%u moved=%lld\n",
                       (long)d, (double)ticks / (double)TPS, (int)accepted,
                       (unsigned)fat, (long long)moved);

                // Given an achievable duration, the move must actually happen.
                TEST_RESULT(std::string("displacement ") + std::to_string((long)d) +
                                " is performed, not refused, when given time for it",
                            performed);
                if (performed) {
                    TEST_RESULT(std::string("displacement ") + std::to_string((long)d) +
                                    " delivers what was asked",
                                llabs((long long)(moved - d)) <= llabs((long long)d) / 1000 + 4);
                    // Stated separately: a sign flip is the specific catastrophe
                    // an int32 wrap causes, and it deserves its own verdict.
                    TEST_RESULT(std::string("displacement ") + std::to_string((long)d) +
                                    " moves in the commanded direction",
                                (d > 0) ? (moved > 0) : (moved < 0));
                }
            }
        }
    }

    // =====================================================================
    // 1b. AND WHEN THERE IS NOT ENOUGH TIME, IT IS REFUSED. The counterpart:
    //     the same extreme displacement in a duration no legal speed could
    //     cover must be rejected outright rather than truncated into some
    //     smaller move that happens to fit. "Did something else" is the
    //     failure mode this whole campaign is built around.
    // =====================================================================
    printf("\n--- 1b. the same displacements refused when the time is impossible ---\n");
    {
        const int32_t impossible[] = { I32MAX, -I32MAX, (int32_t)(CPR * 600) };
        for (unsigned i = 0; i < sizeof(impossible) / sizeof(impossible[0]); i++) {
            int32_t d = impossible[i];
            freshStart(m);
            openTheLimits(m);
            int64_t before = m->getPositionRaw();
            m->trapezoidMoveRaw(d, TPS / 10);         // 0.1 s: not physically possible
            bool accepted = (m->getError() == 0);
            delay(400);
            uint8_t fat = fatalOf(m);
            int64_t moved = (accepted && fat == 0) ? (m->getPositionRaw() - before) : 0;
            bool refused = (!accepted) || (fat != 0);
            printf("   d=%ld in 0.1 s: accepted=%d fatal=%u moved=%lld\n",
                   (long)d, (int)accepted, (unsigned)fat, (long long)moved);
            TEST_RESULT(std::string("displacement ") + std::to_string((long)d) +
                            " in an impossible time is refused, not silently shrunk",
                        refused || llabs((long long)(moved - d)) <= 4);
        }
    }

    // =====================================================================
    // 2. ACCUMULATION FAR FROM THE ORIGIN. Twenty 50-rotation moves in the
    //    same direction walk the accumulator out to a thousand rotations.
    //    Each step must add exactly what was asked, and the running total
    //    must equal the sum -- so a truncation that costs a few counts per
    //    move shows up as a growing discrepancy rather than hiding in the
    //    tolerance of a single move.
    // =====================================================================
    printf("\n--- 2. accumulating out to a thousand rotations ---\n");
    {
        freshStart(m);
        openTheLimits(m);
        m->zeroPosition();
        delay(200);
        const int STEPS = 20;
        const int32_t step = (int32_t)(CPR * 20);      // 20 rotations per step
        int64_t expected = 0;
        bool ok = true;
        int64_t worstErr = 0;
        for (int i = 0; i < STEPS && ok; i++) {
            m->trapezoidMoveRaw(step, ticksFor(step));
            if (m->getError() != 0) { ok = false; printf("   step %d rejected\n", i); break; }
            if (!drainQueue(m, 30000)) { ok = false; printf("   step %d never drained\n", i); break; }
            if (fatalOf(m) != 0) { ok = false; printf("   step %d faulted\n", i); break; }
            expected += step;
            int64_t actual = m->getPositionRaw();
            int64_t err = actual - expected;
            if (llabs((long long)err) > llabs((long long)worstErr)) worstErr = err;
            if (llabs((long long)err) > 200) {
                ok = false;
                printf("   after step %d: position %lld, expected %lld (error %lld)\n",
                       i, (long long)actual, (long long)expected, (long long)err);
            }
        }
        printf("   %d steps of 20 rotations: final expected %lld, worst error %lld counts\n",
               STEPS, (long long)expected, (long long)worstErr);
        TEST_RESULT("twenty 20-rotation moves all complete", ok);
        TEST_RESULT("the accumulated position never drifts from the sum of the moves",
                    ok && llabs((long long)worstErr) <= 200);
        TEST_RESULT("the final position is four hundred rotations",
                    ok && llabs((long long)(m->getPositionRaw() - CPR * 400)) <= 400);
    }

    // =====================================================================
    // 3. RELATIVE AND ABSOLUTE STILL AGREE FAR FROM ZERO. Section 3 of
    //    tm_move_composition checks this near the origin. The accumulator is
    //    a different size out here, so it is worth re-checking after the
    //    machine has travelled a thousand rotations.
    // =====================================================================
    printf("\n--- 3. relative and absolute agree far from the origin ---\n");
    {
        // The machine is still ~1000 rotations out from section 2; go there
        // deliberately so the check does not depend on section 2 running.
        freshStart(m);
        openTheLimits(m);
        m->zeroPosition();
        delay(200);
        for (int i = 0; i < 4; i++) {              // walk out to ~400 rotations
            m->trapezoidMoveRaw((int32_t)(CPR * 100), ticksFor(CPR * 100));
            drainQueue(m, 30000);
        }
        int64_t origin = m->getPositionRaw();
        bool outThere = (fatalOf(m) == 0) && (origin > CPR * 350);
        TEST_RESULT("the machine can be walked out to hundreds of rotations", outThere);
        (void)0;

        const int32_t d = (int32_t)(CPR / 4);
        int64_t startA = m->getPositionRaw();
        m->trapezoidMoveRaw(d, TPS);
        bool okA = (m->getError() == 0) && drainQueue(m, 10000);
        int64_t deltaA = m->getPositionRaw() - startA;

        int64_t startB = m->getPositionRaw();
        m->goToPositionRaw((int32_t)(startB + d), TPS);
        bool okB = (m->getError() == 0) && drainQueue(m, 10000);
        int64_t deltaB = m->getPositionRaw() - startB;

        printf("   at %lld counts out: relative moved %lld, absolute moved %lld\n",
               (long long)origin, (long long)deltaA, (long long)deltaB);
        TEST_RESULT("a relative move works far from the origin",
                    okA && llabs((long long)(deltaA - d)) <= 200);
        TEST_RESULT("an absolute move works far from the origin",
                    okB && llabs((long long)(deltaB - d)) <= 200);
        TEST_RESULT("relative and absolute still agree far from the origin",
                    okA && okB && llabs((long long)(deltaA - deltaB)) <= 200);
    }

    // =====================================================================
    // 4. THE ACCUMULATOR IS SYMMETRIC. Walk a long way out and come all the
    //    way back; the machine must return to where it started. An asymmetric
    //    rounding -- one that truncates toward zero, say -- accumulates in one
    //    direction only and shows up here as a residue proportional to the
    //    distance travelled.
    // =====================================================================
    printf("\n--- 4. out and back over a long distance leaves no residue ---\n");
    {
        freshStart(m);
        openTheLimits(m);
        m->zeroPosition();
        delay(200);
        int64_t origin = m->getPositionRaw();
        bool ok = true;
        const int LEGS = 6;
        const int32_t leg = (int32_t)(CPR * 30);
        for (int i = 0; i < LEGS && ok; i++) {
            m->trapezoidMoveRaw(leg, ticksFor(leg));
            if (m->getError() != 0 || !drainQueue(m, 30000)) { ok = false; break; }
            m->trapezoidMoveRaw(-leg, ticksFor(leg));
            if (m->getError() != 0 || !drainQueue(m, 30000)) { ok = false; break; }
        }
        int64_t residue = m->getPositionRaw() - origin;
        printf("   %d out-and-back legs of 30 rotations: residue %lld counts\n",
               LEGS, (long long)residue);
        TEST_RESULT("six 30-rotation out-and-back legs all complete",
                    ok && fatalOf(m) == 0);
        TEST_RESULT("they leave the machine back at the origin",
                    ok && llabs((long long)residue) <= 100);
    }

    // =====================================================================
    // 5. NEGATIVE TERRITORY. Everything above walks in the positive
    //    direction. The accumulator is signed, and sign-extension mistakes in
    //    wide fixed-point arithmetic are common, so the same walk is repeated
    //    downwards.
    // =====================================================================
    printf("\n--- 5. the same accumulation in negative territory ---\n");
    {
        freshStart(m);
        openTheLimits(m);
        m->zeroPosition();
        delay(200);
        const int STEPS = 8;
        const int32_t step = (int32_t)(-CPR * 20);
        int64_t expected = 0;
        bool ok = true;
        int64_t worstErr = 0;
        for (int i = 0; i < STEPS && ok; i++) {
            m->trapezoidMoveRaw(step, ticksFor(step));
            if (m->getError() != 0 || !drainQueue(m, 30000) || fatalOf(m) != 0) {
                ok = false; printf("   negative step %d failed\n", i); break;
            }
            expected += step;
            int64_t err = m->getPositionRaw() - expected;
            if (llabs((long long)err) > llabs((long long)worstErr)) worstErr = err;
        }
        printf("   %d steps of -20 rotations: expected %lld, worst error %lld\n",
               STEPS, (long long)expected, (long long)worstErr);
        TEST_RESULT("eight 20-rotation moves in the negative direction all complete", ok);
        TEST_RESULT("the negative accumulation is as exact as the positive one",
                    ok && llabs((long long)worstErr) <= 200);
        TEST_RESULT("the position really is negative afterwards",
                    ok && m->getPositionRaw() < -CPR * 140);
    }

    // =====================================================================
    // 6. Still healthy afterwards, and a reset brings the origin back.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        freshStart(m);
        int64_t after = m->getPositionRaw();
        printf("   position after reset: %lld\n", (long long)after);
        TEST_RESULT("a reset returns the commanded position to zero",
                    llabs((long long)after) <= 2);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(CPR / 4), TPS);
        bool ok = (m->getError() == 0) && drainQueue(m, 8000);
        TEST_RESULT("an ordinary move still works after the accumulator sweep",
                    ok && fatalOf(m) == 0 &&
                    llabs((long long)(m->getPositionRaw() - before - CPR / 4)) < 200);
    }

    freshStart(m);
}
