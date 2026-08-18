#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_acceleration_move_semantics.cpp
//
// `MOVE WITH ACCELERATION` (cmd 19) AS A PRIMITIVE, CHARACTERISED ON ITS OWN
// TERMS.
//
// The whole motion system is built from exactly two primitives: a
// constant-velocity segment and a constant-acceleration segment. `Trapezoid
// move` and `Go to position` are not independent mechanisms -- the planner
// decomposes them into three acceleration segments (motor_control.c:2039-2041)
// or, on the one-tick-ramp fast path, into two velocity segments
// (motor_control.c:2034-2035). So whatever an acceleration segment guarantees,
// every profiled move in the product inherits.
//
// An acceleration segment has five properties, all of them visible in the
// firmware, and all of them counter-intuitive enough to have cost this campaign
// real debugging time:
//
//   1. IT ACCUMULATES. The executor does a `+=`, not an `=`:
//          current_velocity_i64 += movement_queue[queue_read_position].acceleration;
//      (motor_control.c:2132) -- once per control tick, for as many ticks as the
//      segment lasts. A velocity segment, by contrast, ASSIGNS
//      (motor_control.c:2140). That difference is the single most important fact
//      about this command: N consecutive acceleration segments do not each start
//      from rest, they compose into ONE continuous ramp. It is also asserted at
//      queue time, where the validator carries the running velocity forward:
//          predicted_final_velocity = velocity_after_last_queue_item
//                                     + acceleration * n_time_steps;
//      (motor_control.c:1799). Four segments of 20 rot/s^2 x 0.2 s reach 16
//      rot/s, not 4, so a speed ceiling picked for the typical segment REFUSES
//      the worst one. That exact trap broke tm_multimove_bitmask twice during
//      this campaign and is recorded in TEST_EXPANSION_CAMPAIGN_2026-08.md.
//      Nothing in the suite asserts it. Section 5 does, deliberately.
//
//   2. THE SIGN OF THE ACCELERATION IS THE SIGN OF THE VELOCITY *CHANGE*, NOT
//      OF THE MOTION. A negative acceleration applied to a forward-moving shaft
//      carries it FORWARD until the velocity crosses zero, and only then
//      backwards. The firmware knows this explicitly -- it computes the turn
//      point and safety-checks the position there (motor_control.c:1819-1833) --
//      but a caller who reads "move with acceleration -20" as "move backwards"
//      will command the opposite of what they meant.
//
//   3. ZERO ACCELERATION COASTS. `+= 0` holds whatever velocity was inherited
//      for the whole segment, which is how the trapezoid planner spends its
//      cruise phase (motor_control.c:2040 queues acceleration 0 for delta_t2).
//
//   4. ITS DISTANCE IS THE EXACT DISCRETE INTEGRAL, not an approximation. With
//      v0 the inherited velocity and A the stored acceleration, the position
//      accumulator gains the current velocity once per tick
//      (motor_control.c:2191), so over n ticks it gains exactly
//          v0*n + A*n*(n+1)/2
//      in units of 2^-32 count, and the reported position is the top 64 bits of
//      that 96-bit accumulator (motor_control.c:137-152). From rest this is the
//      textbook 0.5*a*t^2 to within one part in n -- and this module asserts the
//      EXACT form, because a fixed-point scaling error of a few parts per
//      million is exactly what a percentage tolerance hides.
//
//   5. ITS ACCELERATION IS VALIDATED INCLUSIVELY AND SYMMETRICALLY AT QUEUE
//      TIME. The check is
//          llabs(parameter << ACCELERATION_SHIFT_LEFT) > max_acceleration
//      (motor_control.c:1795, shift 8 at motor_control.h:64) and
//      set_max_acceleration() stores its limit through the identical shift
//      (motor_control.c:3366-3367). Both sides are therefore raw wire values, so
//      the boundary is exact to ONE least-significant unit and can be tested as
//      such. And it is a PASS/FAIL CHECK, never a clamp: an over-limit
//      acceleration is refused with fatal error 15, not quietly slowed down.
//
// WHAT ALREADY EXISTS AND WHY THIS IS NOT IT:
//   * tm_move_accel is the per-command smoke test for cmd 11: MOSFETs ON, one
//     accelerate/decelerate pair per unit system, checked to +/-0.1 ROTATION. It
//     measures that the motor turns roughly the right way by roughly the right
//     amount. It never queues more than two segments, never coasts, never
//     crosses a turn point, and never touches a limit. This module measures the
//     arithmetic instead of the motor, to the COUNT, which is only affordable
//     because the output stage is off.
//   * tm_limit_boundaries checks the acceleration ceiling with float user units
//     (2.0 vs 2.05 rot/s^2) -- a 2.5% step, and only on the positive side.
//     Section 6 pins the same boundary to ONE RAW UNIT via
//     setMaximumAccelerationRaw() and adds the negative half, which is what
//     makes `llabs(...) >` testable rather than merely plausible.
//   * tm_velocity_move_semantics is the sibling module for the OTHER primitive.
//     It shows that a zero-acceleration coast INHERITS a velocity. Section 2
//     here asks the complementary question -- what an acceleration segment does
//     to the velocity it inherited -- and pins the answer for A=0 at EXACTLY
//     zero change, by leaving an unmodified mirror ramp to land on zero.
//   * tm_queue_underrun owns error 18 as a fault. This module uses its ABSENCE
//     as an instrument (see below).
//   * tm_planner_arithmetic, tm_move_composition and tm_move_type_equivalence
//     all reach the acceleration primitive only through trapezoid moves or
//     multimove, never as a command in its own right.
//
// THE MEASURING TRICK, stated once. Error 18, ERROR_RUN_OUT_OF_QUEUE_ITEMS, is
// raised on the very next control tick if the queue empties while the velocity
// is non-zero (motor_control.c:2158-2164). A chain of acceleration segments that
// is supposed to end at rest therefore carries its own oracle: if the final
// velocity is off by even one least-significant unit, the machine faults. So in
// sections 2, 3 and 5a the terminating zero-velocity segment is deliberately
// OMITTED, and "no fault" is the assertion that the velocity returned to exactly
// zero. That is a far sharper measurement than any position comparison, because
// the velocity itself is not otherwise readable. Every OTHER chain in this
// module ends with an explicit `moveWithVelocityRaw(0, 1)` stop -- the same
// terminator the firmware's own planner emits (motor_control.c:2035).
//
// SAFETY. The MOSFETs stay OFF for the entire module: enableMosfets() and
// goToClosedLoop() are never called, and only the COMMANDED position is read,
// which the planner advances whether or not the output stage is energised.
// Nothing turns, on any motor, at any moment. Every segment is at least 0.2 s
// long (6250 ticks) so the queue cannot run dry in the few milliseconds between
// two commands on the bus, and every chain either ends at zero velocity by
// construction or is explicitly stopped. The deliberate faults are five queue-
// time refusals (codes 28, 15, 15, 15 and 34), each raised before anything
// executes and each cleared by the reset at the head of the next subsection.
// tfhFreshOpen() raises the deviation window first, because with the shaft
// stationary the deviation equals the commanded travel and anything past two
// rotations would trip error 45 (motor_control.c:97). No broadcasts, no alias
// writes, no test modes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// Error codes, from python_programs/servomotor/error_codes.json.
static const uint8_t ERR_ACCEL_TOO_HIGH              = 15;
static const uint8_t ERR_RUN_OUT_OF_QUEUE_ITEMS      = 18;
static const uint8_t ERR_PREDICTED_VELOCITY_TOO_HIGH = 28;
static const uint8_t ERR_PARAM_OUT_OF_RANGE          = 34;

// The two firmware shifts that put the wire values onto the common internal
// scale of 2^32 per count (motor_control.h:64-65). They are mirrored here only
// so the PREDICTION can be computed; every value actually sent to the motor is
// produced by the library's own converters, so a disagreement between the
// library's scaling and the firmware's would fail this module too.
static const int ACCELERATION_SHIFT_LEFT_FW = 8;
static const int VELOCITY_SHIFT_LEFT_FW     = 12;

// Chains start from a freshly reset accumulator, so the prediction is exact
// rather than exact-to-within-a-count. Two counts of slack absorbs nothing more
// than a stray tick.
static const int64_t COUNT_TOL = 2;

enum { AMS_ACC = 0, AMS_VEL = 1 };

struct AmsSeg {
    uint8_t  kind;    // AMS_ACC -> 'Move with acceleration', AMS_VEL -> 'Move with velocity'
    int32_t  value;   // raw wire acceleration or velocity
    uint32_t ticks;   // raw wire duration, in control ticks
};

// --- Wire values, always via the library's converters -----------------------
static int32_t rawAcc(float rotPerSec2) {
    return (int32_t)convertAcceleration(rotPerSec2,
                                        AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED,
                                        ConversionDirection::TO_INTERNAL);
}
static int32_t rawVel(float rotPerSec) {
    return (int32_t)convertVelocity(rotPerSec, VelocityUnit::ROTATIONS_PER_SECOND,
                                    ConversionDirection::TO_INTERNAL);
}
static uint32_t rawTicks(float seconds) {
    return (uint32_t)convertTime(seconds, TimeUnit::SECONDS,
                                 ConversionDirection::TO_INTERNAL);
}
// Back to human units, for the diagnostics printed alongside every measurement.
static float accAsRot(int32_t raw) {
    return convertAcceleration((float)raw, AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED,
                               ConversionDirection::FROM_INTERNAL);
}
static float fineVelAsRot(int64_t fineVel) {
    return convertVelocity((float)(fineVel >> VELOCITY_SHIFT_LEFT_FW),
                           VelocityUnit::ROTATIONS_PER_SECOND,
                           ConversionDirection::FROM_INTERNAL);
}

// ---------------------------------------------------------------------------
// The displacement, in encoder counts, that a chain MUST produce, derived from
// the firmware rather than from physics so that it is exact:
//
//   * an acceleration item stores `value << 8` and the executor adds it to the
//     velocity once per tick (motor_control.c:1794, :2132);
//   * a velocity item stores `value << 12` and the executor assigns it
//     (motor_control.c:1842, :2140);
//   * the position accumulator gains the current velocity once per tick
//     (motor_control.c:2191), with no rounding anywhere;
//   * the reported position is the top 64 bits of the 96-bit accumulator
//     (motor_control.c:137-152), i.e. the fine total arithmetically shifted
//     right by 32 -- a floor, which `>>` on a signed int64 reproduces exactly.
//
// Summing an acceleration segment tick by tick gives v0*n + A*n*(n+1)/2, the
// exact discrete integral. `*endVelOut` is the fine velocity the chain ends on;
// anything other than 0 means the chain needs a terminating stop.
// ---------------------------------------------------------------------------
static int64_t exactCounts(const AmsSeg* s, int n, int64_t* endVelOut) {
    int64_t v = 0;       // fine velocity: 2^32 per count-per-tick
    int64_t fine = 0;    // fine position: 2^32 per count
    for (int i = 0; i < n; i++) {
        int64_t t = (int64_t)s[i].ticks;
        if (s[i].kind == AMS_ACC) {
            int64_t a = (int64_t)s[i].value << ACCELERATION_SHIFT_LEFT_FW;
            fine += v * t + a * ((t * (t + 1)) / 2);
            v    += a * t;
        } else {
            v = (int64_t)s[i].value << VELOCITY_SHIFT_LEFT_FW;
            fine += v * t;
        }
    }
    if (endVelOut != nullptr) *endVelOut = v;
    return fine >> 32;
}

// ---------------------------------------------------------------------------
// Queue a chain exactly as written -- no terminator is added, because whether
// one is present is part of what several sections measure -- then drain and
// report the commanded-position delta.
//
// Draining is done by polling the queue depth with a bounded deadline, NEVER by
// waiting a computed duration: a wait that is even slightly short reads a
// PARTIAL move and reports a distance shorter than commanded, which is
// indistinguishable from a planner bug and was this campaign's first false
// finding.
//
// Returns true only if every segment was accepted, the queue drained, and no
// fatal error was latched. `*refusedAtOut` is the index of the segment the
// motor refused (-1 if none) and `*fatalOut` the latched code, so a failure is
// fully diagnosable from the printout without a rerun.
// ---------------------------------------------------------------------------
static bool runChain(Servomotor* m, const AmsSeg* s, int n,
                     int64_t* movedOut, int* refusedAtOut, uint8_t* fatalOut) {
    *movedOut = 0;
    *refusedAtOut = -1;
    *fatalOut = 0;

    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) { *fatalOut = 0xFE; return false; }

    uint64_t totalTicks = 0;
    for (int i = 0; i < n; i++) {
        if (s[i].kind == AMS_ACC) m->moveWithAccelerationRaw(s[i].value, s[i].ticks);
        else                      m->moveWithVelocityRaw(s[i].value, s[i].ticks);
        if (m->getError() != 0) {           // refused at queue time
            *refusedAtOut = i;
            *fatalOut = tfhFatal(m);
            return false;
        }
        totalTicks += (uint64_t)s[i].ticks;
    }

    uint32_t budget = (uint32_t)(totalTicks * 1000ULL / (uint64_t)TFH_TPS) + 10000;
    bool drained = tfhDrain(m, budget);
    *fatalOut = tfhFatal(m);
    if (!drained || *fatalOut != 0) return false;

    *movedOut = m->getPositionRaw() - before;
    return true;
}

// Reset, raise the ceilings, then run: the usual starting point. The reset also
// clears any fault a previous subsection deliberately provoked.
static bool runChainFresh(Servomotor* m, const AmsSeg* s, int n,
                          int64_t* movedOut, int* refusedAtOut, uint8_t* fatalOut) {
    tfhFreshOpen(m);
    return runChain(m, s, n, movedOut, refusedAtOut, fatalOut);
}

void tm_acceleration_move_semantics(void) {
    Serial.println("tm_acceleration_move_semantics: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // Every segment is 0.2 s or longer. That is not arbitrary: the first
    // segment starts executing the moment it is queued, so the whole rest of
    // the chain has to be sent before it expires. 0.2 s against a few
    // milliseconds per bus round trip is roughly a 40x margin.
    const uint32_t T_UNIT = rawTicks(0.2f);          // 6250 ticks
    const int32_t  A20    = rawAcc(20.0f);           // ~1.13e6 raw, well inside every ceiling used here
    const int32_t  A50    = rawAcc(50.0f);
    printf("wire values: 20 rot/s^2 -> %ld raw, 50 rot/s^2 -> %ld raw, 0.2 s -> %lu ticks\n",
           (long)A20, (long)A50, (unsigned long)T_UNIT);

    // =====================================================================
    // 1. A RAMP FROM REST COVERS THE EXACT INTEGRAL OF ITS ACCELERATION.
    //    The foundational claim: everything else in this module is a
    //    difference of distances, and a difference is only meaningful if the
    //    distances themselves are right. Asserted to within COUNTS out of
    //    millions rather than to a percentage, because the firmware's
    //    integrator has no rounding in it and a percentage tolerance would
    //    conceal a fixed-point scaling error of a few parts per million --
    //    precisely the mistake a shift constant produces.
    // =====================================================================
    printf("\n--- 1. a ramp from rest covers the exact integral of its acceleration ---\n");
    {
        struct { int32_t acc; float secs; const char* tag; } cases[] = {
            { A20,           0.2f, "20 rot/s^2 for 0.2 s"  },
            { A20,           0.4f, "20 rot/s^2 for 0.4 s"  },
            { rawAcc(-50.0f), 0.2f, "-50 rot/s^2 for 0.2 s" },
        };
        int64_t moved[3] = { 0, 0, 0 };
        int64_t predicted[3] = { 0, 0, 0 };
        bool    ok[3] = { false, false, false };

        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            uint32_t n = rawTicks(cases[i].secs);
            // Ramp, then a hard stop. The stop is the same one the firmware's
            // own trapezoid planner emits (motor_control.c:2035); with the
            // output stage off there is no mechanical cost to it, and it lets
            // the ramp's distance be measured in isolation because a
            // zero-velocity segment contributes zero displacement.
            AmsSeg chain[2] = { { AMS_ACC, cases[i].acc, n }, { AMS_VEL, 0, 1 } };
            int64_t endVel = 0;
            predicted[i] = exactCounts(chain, 2, &endVel);

            int refusedAt = -1; uint8_t fatal = 0;
            ok[i] = runChainFresh(m, chain, 2, &moved[i], &refusedAt, &fatal);
            printf("   %-22s: moved %lld, predicted %lld, peak %.2f rot/s, refusedAt=%d fatal=%u\n",
                   cases[i].tag, (long long)moved[i], (long long)predicted[i],
                   (double)fineVelAsRot(((int64_t)cases[i].acc << ACCELERATION_SHIFT_LEFT_FW) *
                                        (int64_t)n),
                   refusedAt, (unsigned)fatal);
            TEST_RESULT(std::string("a ramp of ") + cases[i].tag +
                            " from rest is accepted and completes cleanly",
                        ok[i]);
            TEST_RESULT(std::string("and a ramp of ") + cases[i].tag +
                            " travels exactly the integral of its acceleration",
                        ok[i] && llabs((long long)(moved[i] - predicted[i])) <= COUNT_TOL);
        }

        // The sign of the acceleration, applied from rest, IS the direction of
        // travel -- the only situation in which that shortcut is true, which is
        // exactly why section 4 exists.
        printf("   direction check: +20 rot/s^2 moved %lld, -50 rot/s^2 moved %lld\n",
               (long long)moved[0], (long long)moved[2]);
        TEST_RESULT("from rest, the direction of travel is the sign of the acceleration",
                    ok[0] && ok[2] && moved[0] > 0 && moved[2] < 0);

        // The textbook continuous form is 0.5*a*t^2; the firmware's exact
        // discrete form is a*n*(n+1)/2, larger by one part in n. Reported so
        // that a reader who predicts with physics knows the size of the gap
        // rather than discovering it as a mystery failure.
        {
            uint32_t n = rawTicks(0.4f);
            // a in counts per tick^2 is the wire value divided by 2^24
            // (CONVERSION_FACTOR_COUNTS_PER_TIMESTEP_SQUARED = 16777216).
            double continuous = 0.5 * (double)A20 * (double)n * (double)n / 16777216.0;
            double exact      = (double)predicted[1];
            double relErr     = (exact != 0.0) ? ((continuous - exact) / exact) : 1.0;
            printf("   0.5*a*t^2 = %.0f counts vs exact %.0f counts (%.4f%%)\n",
                   continuous, exact, relErr * 100.0);
            TEST_RESULT("the exact discrete integral agrees with the textbook 0.5*a*t^2 "
                        "to better than 0.05%",
                        ok[1] && std::fabs(relErr) < 0.0005);
        }
        tfhReset(m);
    }

    // =====================================================================
    // 2. ZERO ACCELERATION COASTS AT THE INHERITED VELOCITY.
    //    This is how the trapezoid planner spends its cruise phase
    //    (motor_control.c:2040), so "acceleration 0" being a genuine no-change
    //    rather than a stop or a decay is load-bearing for every profiled move
    //    in the product. Measured two ways at once: the extra DISTANCE a coast
    //    contributes must be velocity x time, and the coast must leave the
    //    velocity so exactly untouched that an unmodified mirror ramp still
    //    lands on zero (no ERROR_RUN_OUT_OF_QUEUE_ITEMS).
    // =====================================================================
    printf("\n--- 2. zero acceleration coasts at the inherited velocity ---\n");
    {
        // No terminating stop on the first three chains: the mirror ramp is
        // supposed to bring the velocity to exactly zero, and error 18 is the
        // oracle for whether it did.
        AmsSeg base[2]   = { { AMS_ACC,  A20, T_UNIT },
                             { AMS_ACC, -A20, T_UNIT } };
        AmsSeg coast1[3] = { { AMS_ACC,  A20, T_UNIT },
                             { AMS_ACC,    0, T_UNIT },
                             { AMS_ACC, -A20, T_UNIT } };
        AmsSeg coast2[3] = { { AMS_ACC,  A20, T_UNIT },
                             { AMS_ACC,    0, T_UNIT * 2 },
                             { AMS_ACC, -A20, T_UNIT } };

        int64_t mBase = 0, m1 = 0, m2 = 0;
        int refA = -1, ref1 = -1, ref2 = -1;
        uint8_t fBase = 0, f1 = 0, f2 = 0;
        bool okBase = runChainFresh(m, base,   2, &mBase, &refA, &fBase);
        bool ok1    = runChainFresh(m, coast1, 3, &m1,    &ref1, &f1);
        bool ok2    = runChainFresh(m, coast2, 3, &m2,    &ref2, &f2);

        int64_t pBase = exactCounts(base,   2, nullptr);
        int64_t p1    = exactCounts(coast1, 3, nullptr);
        int64_t p2    = exactCounts(coast2, 3, nullptr);
        printf("   no coast: moved %lld (predicted %lld) ok=%d fatal=%u\n",
               (long long)mBase, (long long)pBase, (int)okBase, (unsigned)fBase);
        printf("   0.2 s coast: moved %lld (predicted %lld) ok=%d fatal=%u -> coast added %lld\n",
               (long long)m1, (long long)p1, (int)ok1, (unsigned)f1, (long long)(m1 - mBase));
        printf("   0.4 s coast: moved %lld (predicted %lld) ok=%d fatal=%u -> coast added %lld\n",
               (long long)m2, (long long)p2, (int)ok2, (unsigned)f2, (long long)(m2 - mBase));

        TEST_RESULT("a zero-acceleration segment is accepted, and the unchanged mirror ramp "
                    "still lands on zero velocity (no underrun), so the coast changed the "
                    "velocity by exactly zero",
                    ok1 && f1 == 0);
        TEST_RESULT("a chain containing a coast travels exactly its predicted distance",
                    ok1 && llabs((long long)(m1 - p1)) <= COUNT_TOL);
        TEST_RESULT("the coast contributes exactly the held velocity times its duration",
                    okBase && ok1 &&
                    llabs((long long)((m1 - mBase) - (p1 - pBase))) <= 2 * COUNT_TOL);
        TEST_RESULT("doubling the coast doubles the distance it adds",
                    okBase && ok1 && ok2 &&
                    llabs((long long)((m2 - mBase) - 2 * (m1 - mBase))) <= 4 * COUNT_TOL);

        // The degenerate case: zero acceleration inherited from rest. A whole
        // segment of it must be a legal, harmless dwell -- not a no-op that is
        // silently dropped, and not a fault.
        AmsSeg rest0[2] = { { AMS_ACC, 0, T_UNIT }, { AMS_VEL, 0, 1 } };
        int64_t mRest = 0; int refR = -1; uint8_t fRest = 0;
        bool okRest = runChainFresh(m, rest0, 2, &mRest, &refR, &fRest);
        printf("   zero acceleration from rest: moved %lld, ok=%d fatal=%u\n",
               (long long)mRest, (int)okRest, (unsigned)fRest);
        TEST_RESULT("zero acceleration applied from rest is accepted and raises no fault",
                    okRest && fRest == 0);
        TEST_RESULT("and it moves the commanded position by exactly nothing",
                    okRest && mRest == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 3. EQUAL AND OPPOSITE ACCELERATIONS FOR EQUAL TIMES RETURN THE VELOCITY
    //    TO EXACTLY ZERO. This is the only safe way to end an acceleration
    //    plan without a hard velocity stop, so "exactly" matters: a residue of
    //    one least-significant unit is the difference between a clean finish
    //    and a latched fatal error 18 on the next tick. The absence of that
    //    fault is the assertion. The distance is a bonus check with a
    //    beautifully simple closed form -- the triangle's area, a * t^2, where
    //    t is the length of ONE of the two legs.
    // =====================================================================
    printf("\n--- 3. equal and opposite ramps return the velocity to exactly zero ---\n");
    {
        struct { int32_t acc; float secs; const char* tag; } cases[] = {
            { A20,            0.2f, "20 rot/s^2 +/- 0.2 s"  },
            { A50,            0.2f, "50 rot/s^2 +/- 0.2 s"  },
            { rawAcc(-20.0f), 0.4f, "-20 rot/s^2 +/- 0.4 s" },
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            uint32_t n = rawTicks(cases[i].secs);
            AmsSeg pair[2] = { { AMS_ACC,  cases[i].acc, n },
                               { AMS_ACC, -cases[i].acc, n } };
            int64_t endVel = 0;
            int64_t predicted = exactCounts(pair, 2, &endVel);
            int64_t moved = 0; int refusedAt = -1; uint8_t fatal = 0;
            bool ok = runChainFresh(m, pair, 2, &moved, &refusedAt, &fatal);
            printf("   %-24s: moved %lld, predicted %lld, predicted end velocity %lld, fatal=%u\n",
                   cases[i].tag, (long long)moved, (long long)predicted,
                   (long long)endVel, (unsigned)fatal);
            TEST_RESULT(std::string("a pair of ") + cases[i].tag +
                            " ends at exactly zero velocity: no ERROR_RUN_OUT_OF_QUEUE_ITEMS "
                            "even with no terminating stop",
                        ok && fatal != ERR_RUN_OUT_OF_QUEUE_ITEMS && fatal == 0);
            TEST_RESULT(std::string("and a pair of ") + cases[i].tag +
                            " covers exactly the triangle's area",
                        ok && llabs((long long)(moved - predicted)) <= COUNT_TOL);
        }
        tfhReset(m);
    }

    // =====================================================================
    // 4. THE SIGN OF THE ACCELERATION IS THE SIGN OF THE VELOCITY CHANGE, NOT
    //    THE DIRECTION OF TRAVEL. The most dangerous misreading of this
    //    command: applied to a moving shaft, a NEGATIVE acceleration first
    //    carries it FORWARD (decelerating), and only reverses it after the
    //    velocity has crossed zero. The firmware models the crossing
    //    explicitly as the "turn point" and safety-checks the position there
    //    (motor_control.c:1819-1833), so this is a designed behaviour, not an
    //    accident -- but a caller who reads "-20" as "backwards" commands the
    //    opposite of what they meant. Measured as a difference against a
    //    velocity-only reference leg, so the acceleration segment's own
    //    contribution is isolated.
    // =====================================================================
    printf("\n--- 4. the acceleration sign sets the velocity change, not the direction ---\n");
    {
        const int32_t  V4     = rawVel(4.0f);          // 4 rot/s
        const uint32_t N0     = T_UNIT;                // 0.2 s of steady travel first
        const uint32_t NSHORT = rawTicks(0.1f);        // stops short of the turn point
        const uint32_t NLONG  = rawTicks(0.6f);        // well past it: ends near -8 rot/s

        AmsSeg refFwd[2]  = { { AMS_VEL,  V4, N0 }, { AMS_VEL, 0, 1 } };
        AmsSeg shortFwd[3] = { { AMS_VEL,  V4, N0 }, { AMS_ACC, -A20, NSHORT }, { AMS_VEL, 0, 1 } };
        AmsSeg longFwd[3]  = { { AMS_VEL,  V4, N0 }, { AMS_ACC, -A20, NLONG  }, { AMS_VEL, 0, 1 } };
        AmsSeg refBack[2]  = { { AMS_VEL, -V4, N0 }, { AMS_VEL, 0, 1 } };
        AmsSeg shortBack[3]= { { AMS_VEL, -V4, N0 }, { AMS_ACC,  A20, NSHORT }, { AMS_VEL, 0, 1 } };

        int64_t mRef = 0, mShort = 0, mLong = 0, mRefB = 0, mShortB = 0;
        int r = -1; uint8_t f = 0;
        bool okRef    = runChainFresh(m, refFwd,    2, &mRef,    &r, &f); uint8_t fRef = f;
        bool okShort  = runChainFresh(m, shortFwd,  3, &mShort,  &r, &f); uint8_t fShort = f;
        bool okLong   = runChainFresh(m, longFwd,   3, &mLong,   &r, &f); uint8_t fLong = f;
        bool okRefB   = runChainFresh(m, refBack,   2, &mRefB,   &r, &f);
        bool okShortB = runChainFresh(m, shortBack, 3, &mShortB, &r, &f);

        int64_t pRef   = exactCounts(refFwd,    2, nullptr);
        int64_t pShort = exactCounts(shortFwd,  3, nullptr);
        int64_t pLong  = exactCounts(longFwd,   3, nullptr);
        int64_t pRefB  = exactCounts(refBack,   2, nullptr);
        int64_t pShortB= exactCounts(shortBack, 3, nullptr);

        printf("   forward reference leg: moved %lld (predicted %lld) ok=%d fatal=%u\n",
               (long long)mRef, (long long)pRef, (int)okRef, (unsigned)fRef);
        printf("   -20 rot/s^2 for 0.1 s on a forward shaft: leg contributed %lld "
               "(predicted %lld) ok=%d fatal=%u\n",
               (long long)(mShort - mRef), (long long)(pShort - pRef), (int)okShort,
               (unsigned)fShort);
        printf("   -20 rot/s^2 for 0.6 s on a forward shaft: leg contributed %lld "
               "(predicted %lld) ok=%d fatal=%u\n",
               (long long)(mLong - mRef), (long long)(pLong - pRef), (int)okLong,
               (unsigned)fLong);
        printf("   mirror: +20 rot/s^2 for 0.1 s on a backward shaft: leg contributed %lld "
               "(predicted %lld)\n",
               (long long)(mShortB - mRefB), (long long)(pShortB - pRefB));

        TEST_RESULT("a NEGATIVE acceleration applied to a forward-moving shaft still carries "
                    "it FORWARD while the velocity stays positive",
                    okRef && okShort && (mShort - mRef) > 0);
        TEST_RESULT("and the forward distance that decelerating leg adds matches the exact "
                    "integral",
                    okRef && okShort &&
                    llabs((long long)((mShort - mRef) - (pShort - pRef))) <= 2 * COUNT_TOL);
        TEST_RESULT("the SAME negative acceleration carries the shaft BACKWARD once the "
                    "velocity has crossed zero",
                    okRef && okLong && (mLong - mRef) < 0);
        TEST_RESULT("and that backward distance matches the exact integral, turn point "
                    "included",
                    okRef && okLong &&
                    llabs((long long)((mLong - mRef) - (pLong - pRef))) <= 2 * COUNT_TOL);
        TEST_RESULT("the mirror case is identical with every sign exchanged: a POSITIVE "
                    "acceleration on a backward-moving shaft still carries it BACKWARD",
                    okRefB && okShortB && (mShortB - mRefB) < 0 &&
                    llabs((long long)((mShortB - mRefB) - (pShortB - pRefB))) <= 2 * COUNT_TOL);
        tfhReset(m);
    }

    // =====================================================================
    // 5. VELOCITY ACCUMULATES ACROSS CONSECUTIVE ACCELERATION SEGMENTS.
    //    The property that broke tm_multimove_bitmask twice, asserted here
    //    deliberately so it can never be rediscovered the hard way. Three
    //    claims, in increasing order of consequence:
    //      (a) N segments compose into ONE continuous ramp -- splitting a ramp
    //          changes nothing at all;
    //      (b) therefore a speed ceiling must be chosen for the ACCUMULATED
    //          peak: a segment that is legal as the first is REFUSED as the
    //          fourth, with ERROR_PREDICTED_VELOCITY_TOO_HIGH, because
    //          add_to_queue() carries velocity_after_last_queue_item forward
    //          (motor_control.c:1799);
    //      (c) and raising the ceiling above the accumulated peak makes that
    //          same fourth segment legal again -- proving the refusal is about
    //          the accumulation and not about the segment.
    // =====================================================================
    printf("\n--- 5. velocity accumulates across consecutive acceleration segments ---\n");
    {
        // (a) split versus whole. Both reach 12 rot/s at 20 rot/s^2 over 0.6 s,
        //     comfortably inside the 15 rot/s ceiling tfhFreshOpen() sets.
        AmsSeg split[4] = { { AMS_ACC,  A20, T_UNIT },
                            { AMS_ACC,  A20, T_UNIT },
                            { AMS_ACC,  A20, T_UNIT },
                            { AMS_ACC, -A20, T_UNIT * 3 } };   // no stop: must land on zero
        AmsSeg whole[2] = { { AMS_ACC,  A20, T_UNIT * 3 },
                            { AMS_ACC, -A20, T_UNIT * 3 } };
        int64_t mSplit = 0, mWhole = 0; int r = -1; uint8_t fSplit = 0, fWhole = 0;
        bool okSplit = runChainFresh(m, split, 4, &mSplit, &r, &fSplit);
        bool okWhole = runChainFresh(m, whole, 2, &mWhole, &r, &fWhole);
        int64_t pWhole = exactCounts(whole, 2, nullptr);
        printf("   three 0.2 s segments: moved %lld (ok=%d fatal=%u)\n",
               (long long)mSplit, (int)okSplit, (unsigned)fSplit);
        printf("   one 0.6 s segment:    moved %lld (ok=%d fatal=%u), predicted %lld\n",
               (long long)mWhole, (int)okWhole, (unsigned)fWhole, (long long)pWhole);
        TEST_RESULT("three consecutive acceleration segments are indistinguishable from one "
                    "segment of the summed duration",
                    okSplit && okWhole && llabs((long long)(mSplit - mWhole)) <= COUNT_TOL);
        TEST_RESULT("and both cover the distance predicted for one continuous ramp",
                    okSplit && okWhole &&
                    llabs((long long)(mWhole - pWhole)) <= COUNT_TOL &&
                    llabs((long long)(mSplit - pWhole)) <= COUNT_TOL);

        // (b) the trap. Ceiling set explicitly in RAW units so the arithmetic is
        //     exact rather than approximate: at 20 rot/s^2 each 0.2 s segment
        //     adds 4 rot/s, so segments 1-3 sit at 4, 8 and 12 rot/s and the
        //     fourth would predict 16 -- past a 15 rot/s ceiling.
        tfhFreshOpen(m);
        uint32_t velCeil15 = (uint32_t)convertVelocity(15.0f, VelocityUnit::ROTATIONS_PER_SECOND,
                                                       ConversionDirection::TO_INTERNAL);
        m->setMaximumVelocityRaw(velCeil15);
        bool ceilOk = (m->getError() == 0);
        AmsSeg four[4] = { { AMS_ACC, A20, T_UNIT }, { AMS_ACC, A20, T_UNIT },
                           { AMS_ACC, A20, T_UNIT }, { AMS_ACC, A20, T_UNIT } };
        int64_t moved4 = 0; int refusedAt = -1; uint8_t fatal4 = 0;
        runChain(m, four, 4, &moved4, &refusedAt, &fatal4);
        int64_t endVel4 = 0;
        (void)exactCounts(four, 4, &endVel4);
        printf("   ceiling 15 rot/s (raw %lu): four identical 0.2 s segments -> refusedAt=%d "
               "fatal=%u; the fourth would reach %.2f rot/s\n",
               (unsigned long)velCeil15, refusedAt, (unsigned)fatal4,
               (double)fineVelAsRot(endVel4));
        TEST_RESULT("the first three identical acceleration segments are each accepted",
                    ceilOk && refusedAt == 3);
        TEST_RESULT("the FOURTH identical segment is refused, even though it is byte-for-byte "
                    "the same command that was accepted as the first",
                    refusedAt == 3 && fatal4 != 0);
        TEST_RESULT("and the refusal is ERROR_PREDICTED_VELOCITY_TOO_HIGH (28), naming the "
                    "ACCUMULATED velocity rather than the segment",
                    fatal4 == ERR_PREDICTED_VELOCITY_TOO_HIGH);

        // (c) the same four segments, with a ceiling chosen for the WORST case
        //     in the sequence rather than the typical one. 17 rot/s clears the
        //     16 rot/s peak and stays under EXPERIMENTAL_MAX_VELOCITY
        //     (2 x MAX_VELOCITY, about 18.7 rot/s, motor_control.h:61).
        tfhFreshOpen(m);
        uint32_t velCeil17 = (uint32_t)convertVelocity(17.0f, VelocityUnit::ROTATIONS_PER_SECOND,
                                                       ConversionDirection::TO_INTERNAL);
        m->setMaximumVelocityRaw(velCeil17);
        AmsSeg fourOk[5] = { { AMS_ACC,  A20, T_UNIT }, { AMS_ACC,  A20, T_UNIT },
                             { AMS_ACC,  A20, T_UNIT }, { AMS_ACC,  A20, T_UNIT },
                             { AMS_ACC, -A20, T_UNIT * 4 } };   // mirror ramp back to zero
        int64_t movedOk = 0; int refOk = -1; uint8_t fatalOk = 0;
        bool okFour = runChain(m, fourOk, 5, &movedOk, &refOk, &fatalOk);
        int64_t pFour = exactCounts(fourOk, 5, nullptr);
        printf("   ceiling 17 rot/s (raw %lu): moved %lld (predicted %lld) ok=%d refusedAt=%d "
               "fatal=%u\n",
               (unsigned long)velCeil17, (long long)movedOk, (long long)pFour,
               (int)okFour, refOk, (unsigned)fatalOk);
        TEST_RESULT("raising the ceiling above the ACCUMULATED peak makes the identical fourth "
                    "segment legal and the whole ramp run to the predicted distance",
                    okFour && llabs((long long)(movedOk - pFour)) <= COUNT_TOL);
        tfhReset(m);
    }

    // =====================================================================
    // 6. THE ACCELERATION LIMIT IS AN EXACT, SYMMETRIC, PASS/FAIL BOUNDARY.
    //    The limit is set in RAW units here so the comparison the firmware
    //    makes -- llabs(value << 8) > max_acceleration, both sides shifted
    //    identically (motor_control.c:1795 and :3366-3367) -- can be probed one
    //    least-significant unit at a time. And it is a CHECK, not a clamp: an
    //    over-limit acceleration is REFUSED with fatal error 15, never quietly
    //    slowed down to fit. If the firmware ever did clamp, the over-limit
    //    chain below would be accepted and would then drain with a non-zero
    //    velocity, latching error 18 instead of 15 -- so the assertion
    //    distinguishes the two outcomes rather than merely detecting "a fault".
    // =====================================================================
    printf("\n--- 6. the acceleration limit is an exact, symmetric pass/fail boundary ---\n");
    {
        const int32_t LIM = A20;   // 20 rot/s^2, far below EXPERIMENTAL_MAX_ACCELERATION
        printf("   limit set to %ld raw (%.2f rot/s^2)\n", (long)LIM, (double)accAsRot(LIM));

        struct { int32_t acc; bool expectAccept; const char* tag; } cases[] = {
            {  LIM,     true,  "exactly the limit"            },
            {  LIM + 1, false, "one raw unit above the limit" },
            { -LIM,     true,  "exactly minus the limit"      },
            { -LIM - 1, false, "one raw unit below minus the limit" },
        };
        bool accepted[4] = { false, false, false, false };
        uint8_t code[4]  = { 0, 0, 0, 0 };

        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            tfhFreshOpen(m);
            m->setMaximumAccelerationRaw((uint32_t)LIM);
            bool setOk = (m->getError() == 0);
            // An accepted segment is stopped; a refused one never executes, so
            // the terminator would be pointless (and unsendable: after a fatal
            // error only get_status answers).
            AmsSeg chain[2] = { { AMS_ACC, cases[i].acc, T_UNIT }, { AMS_VEL, 0, 1 } };
            int nSeg = cases[i].expectAccept ? 2 : 1;
            int64_t moved = 0; int refusedAt = -1; uint8_t fatal = 0;
            accepted[i] = setOk && runChain(m, chain, nSeg, &moved, &refusedAt, &fatal);
            code[i] = fatal;
            printf("   %-36s: accepted=%d moved %lld refusedAt=%d fatal=%u\n",
                   cases[i].tag, (int)accepted[i], (long long)moved, refusedAt,
                   (unsigned)fatal);
        }

        TEST_RESULT("an acceleration exactly at the configured limit is accepted "
                    "(the comparison is strictly-greater, so equality passes)",
                    accepted[0] && code[0] == 0);
        TEST_RESULT("one raw unit above the limit is refused outright rather than clamped "
                    "to the limit",
                    !accepted[1]);
        TEST_RESULT("and that refusal is ERROR_ACCEL_TOO_HIGH (15), not an underrun from a "
                    "silently clamped move",
                    code[1] == ERR_ACCEL_TOO_HIGH);
        TEST_RESULT("the limit is symmetric: exactly minus the limit is accepted",
                    accepted[2] && code[2] == 0);
        TEST_RESULT("and one raw unit below minus the limit is refused with the same "
                    "ERROR_ACCEL_TOO_HIGH (15)",
                    !accepted[3] && code[3] == ERR_ACCEL_TOO_HIGH);
        tfhReset(m);
    }

    // =====================================================================
    // 7. PARAMETER EDGES AND RECOVERY. A zero-duration acceleration move was a
    //    SILENT NO-OP before firmware 0.15.4.0 -- accepted, acknowledged, and
    //    never executed, which is the worst possible failure mode for a motion
    //    command. It is now rejected explicitly at main.c:535-537, before
    //    add_to_queue() is even reached. This is the regression guard for that
    //    fix. Then a plain ramp, to show the machine is fully usable again
    //    after the five deliberate faults above.
    // =====================================================================
    printf("\n--- 7. parameter edges and recovery ---\n");
    {
        AmsSeg zeroDur[1] = { { AMS_ACC, A20, 0 } };
        int64_t moved = 0; int refusedAt = -1; uint8_t fatal = 0;
        runChainFresh(m, zeroDur, 1, &moved, &refusedAt, &fatal);
        printf("   zero duration: refusedAt=%d fatal=%u (expect 0 and %u)\n",
               refusedAt, (unsigned)fatal, (unsigned)ERR_PARAM_OUT_OF_RANGE);
        TEST_RESULT("a zero-duration acceleration move is refused rather than silently "
                    "accepted and dropped",
                    refusedAt == 0);
        TEST_RESULT("and it reports ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    fatal == ERR_PARAM_OUT_OF_RANGE);

        AmsSeg again[2] = { { AMS_ACC, A20, T_UNIT }, { AMS_ACC, -A20, T_UNIT } };
        int64_t movedAgain = 0; int refAgain = -1; uint8_t fatalAgain = 0;
        bool okAgain = runChainFresh(m, again, 2, &movedAgain, &refAgain, &fatalAgain);
        int64_t pAgain = exactCounts(again, 2, nullptr);
        printf("   recovery ramp: moved %lld (predicted %lld) ok=%d fatal=%u\n",
               (long long)movedAgain, (long long)pAgain, (int)okAgain, (unsigned)fatalAgain);
        TEST_RESULT("a normal acceleration ramp works again, to the count, after every "
                    "deliberate fault in this module",
                    okAgain && llabs((long long)(movedAgain - pAgain)) <= COUNT_TOL);
        tfhReset(m);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
