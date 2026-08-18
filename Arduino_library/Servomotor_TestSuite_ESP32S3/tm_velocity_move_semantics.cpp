#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_velocity_move_semantics.cpp
//
// `MOVE WITH VELOCITY` (cmd 26) AS A PRIMITIVE, CHARACTERISED ON ITS OWN TERMS.
//
// Everything the planner emits is built from two primitives: a constant-velocity
// segment and a constant-acceleration segment. `Trapezoid move` and `Go to
// position` are convenience wrappers that decompose into them
// (motor_control.c:2034-2041). So the guarantees the whole motion system rests
// on are the guarantees of these two primitives, and a velocity segment has
// exactly four of them, all visible in the firmware:
//
//   1. IT IS AN INSTANTANEOUS STEP, NOT A RAMP. On the tick the item is picked
//      up, `handle_queued_movements()` does an assignment, not an accumulation:
//          current_velocity_i64 = movement_queue[queue_read_position].velocity;
//      (motor_control.c:2140). There is no ramp in or out, so a velocity segment
//      REPLACES whatever speed was in flight rather than adding to it -- the
//      opposite of an acceleration segment, which does `+=` (motor_control.c:2132)
//      and therefore ACCUMULATES across a queued sequence.
//
//   2. ITS DISTANCE IS EXACTLY SPEED x TIME. Once per tick the position
//      accumulator gains the current velocity (motor_control.c:2191) with no
//      rounding: the accumulator is 96 bits and the reported position is its top
//      64 (motor_control.c:137-152). The only truncation in the whole path is
//      reading out the integer part.
//
//   3. IT HOLDS ITS VELOCITY WHEN ITS TIME EXPIRES. Nothing zeroes the velocity
//      at a segment boundary; the item is simply dropped from the queue. That is
//      WHY a terminating stop is mandatory, and it is the mechanism -- not the
//      resulting error -- that this module measures.
//
//   4. IT IS VALIDATED AGAINST `max_velocity` AT QUEUE TIME, INCLUSIVELY. The
//      check is `llabs(velocity << VELOCITY_SHIFT_LEFT) > max_velocity`
//      (motor_control.c:1846), and `set_max_velocity()` stores the limit through
//      the identical shift (motor_control.c:3347-3348). The two therefore compare
//      raw wire value against raw wire value, which makes the boundary EXACT and
//      testable to one least-significant bit rather than approximately.
//
// WHAT ALREADY EXISTS AND WHY THIS IS NOT IT:
//   * tm_queue_underrun owns ERROR_RUN_OUT_OF_QUEUE_ITEMS (18) -- the CONSEQUENCE
//     of property 3. This module measures the property itself, by handing the
//     held velocity to a following zero-acceleration coast segment, so the hold
//     is observed as motion rather than inferred from a fault.
//   * tm_move_type_equivalence only ever compares a velocity chain to other
//     routes; it never asks whether the primitive is right in absolute terms.
//   * tm_move_velocity and tm_velocity_accuracy drive the shaft with the MOSFETs
//     ON and check the physical rate to within 15-20%. Useful, and completely
//     different: they measure the motor. This measures the arithmetic, to the
//     count, and can afford to because the output stage is off.
//   * tm_error_taxonomy establishes that an over-speed TRAPEZOID move faults with
//     one of 15/16/28. Nothing anywhere pins down the velocity-move boundary --
//     that at the limit is accepted and one raw step past is refused with 16.
//
// Every velocity and duration here is produced by the library's own
// convertVelocity()/convertTime(), never by hand-rolled shift constants, so a
// disagreement between the LIBRARY's scaling and the FIRMWARE's would fail this
// module too.
//
// SAFETY: the MOSFETs stay OFF for the whole module -- enableMosfets() and
// goToClosedLoop() are never called. Only the COMMANDED position is read, and
// the planner advances that whether or not the output stage is energised, so
// nothing turns on any motor at any point. Every chain is terminated with an
// explicit zero-velocity stop, so no segment is ever left running. Deliberate
// faults are limited to SEVEN refusals at queue time -- six of code 16 (section
// 8: two per limit, three limits) and one of code 34 (section 9) -- each raised
// before anything executes and each cleared by a reset immediately afterwards.
// tfhFreshOpen() raises the deviation window first, because with the shaft
// stationary the deviation equals the commanded travel and anything past two
// rotations would trip error 45. No broadcasts and no alias writes: safe on the
// shared 35-motor rack.
//
// TIMING RULE OBSERVED THROUGHOUT, and the one that is easy to get wrong: the
// FIRST item of a chain begins executing on the very next control tick after it
// is queued, so the whole remainder of the chain -- including the mandatory stop
// -- has to reach the motor before that first item expires. If it does not, the
// queue empties at non-zero velocity and the motor latches error 18 roughly a
// millisecond before the stop arrives. Every chain below therefore opens with a
// segment of at least 0.15 s (about 4700 ticks) against a bus round trip of a
// few milliseconds. Section 9, whose whole subject is a ONE-TICK segment,
// satisfies the rule by putting a zero-velocity dwell in front of it; see the
// note there.
// ---------------------------------------------------------------------------

// Codes from python_programs/servomotor/error_codes.json.
static const uint8_t ERR_RUN_OUT_OF_QUEUE_ITEMS = 18;
static const uint8_t ERR_VEL_TOO_HIGH           = 16;
static const uint8_t ERR_PARAM_OUT_OF_RANGE     = 34;

// Wire velocity is counts-per-timestep scaled by 2^20
// (CONVERSION_FACTOR_COUNTS_PER_TIMESTEP = 1048576). Always go through the
// library's converter rather than that constant, so the library's own scaling is
// under test alongside the firmware's.
static int32_t rawVel(float rotPerSec) {
    return (int32_t)convertVelocity(rotPerSec, VelocityUnit::ROTATIONS_PER_SECOND,
                                    ConversionDirection::TO_INTERNAL);
}

static uint32_t rawTicks(float seconds) {
    return (uint32_t)convertTime(seconds, TimeUnit::SECONDS,
                                 ConversionDirection::TO_INTERNAL);
}

// ---------------------------------------------------------------------------
// The displacement, in encoder counts, that a chain of velocity segments MUST
// produce. Derived from the firmware, not from physics, so it is exact:
//
//   * the queued item stores `velocity << VELOCITY_SHIFT_LEFT` with
//     VELOCITY_SHIFT_LEFT = 12 (motor_control.c:1842, motor_control.h:65), which
//     puts it at 2^32 per count-per-tick;
//   * once per tick the 96-bit position accumulator gains exactly that
//     (motor_control.c:2191);
//   * the reported position is the top 64 bits of the accumulator
//     (motor_control.c:137-152), i.e. the fine total divided by 2^32.
//
// Net: counts = (velocity * ticks * 2^12) / 2^32 = (velocity * ticks) >> 20,
// truncated toward negative infinity exactly as reading the upper words of a
// two's-complement accumulator truncates.
// ---------------------------------------------------------------------------
static int64_t exactCounts(const int32_t* vel, const uint32_t* ticks, int nSeg) {
    int64_t fine = 0;
    for (int i = 0; i < nSeg; i++) fine += (int64_t)vel[i] * (int64_t)ticks[i];
    return fine >> 20;
}

// ---------------------------------------------------------------------------
// Queue a chain of constant-velocity segments, terminate it with the MANDATORY
// zero-velocity stop, drain, and report the commanded-position delta.
//
// The stop is not optional tidiness: a velocity segment holds its velocity when
// its time expires (property 3 above), so a chain that ends at speed with an
// empty queue raises fatal error 18 on the next tick. Draining is done by
// polling the queue depth, never by waiting a computed duration -- a wait that
// is even slightly short reads a PARTIAL move and looks exactly like a planner
// bug.
// ---------------------------------------------------------------------------
static bool velChain(Servomotor* m, const int32_t* vel, const uint32_t* ticks,
                     int nSeg, int64_t* movedOut) {
    *movedOut = 0;
    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) return false;
    uint64_t totalTicks = 0;
    for (int i = 0; i < nSeg; i++) {
        m->moveWithVelocityRaw(vel[i], ticks[i]);
        if (m->getError() != 0) return false;
        totalTicks += (uint64_t)ticks[i];
    }
    m->moveWithVelocityRaw(0, 1);                  // the mandatory stop
    if (m->getError() != 0) return false;
    uint32_t budget = (uint32_t)(totalTicks * 1000ULL / (uint64_t)TFH_TPS) + 10000;
    if (!tfhDrain(m, budget)) return false;
    if (tfhFatal(m) != 0) return false;
    *movedOut = m->getPositionRaw() - before;
    return true;
}

// A chain measured from a freshly reset machine, so the position accumulator's
// fractional word starts at zero and the prediction above is exact rather than
// exact-to-within-a-count.
static bool velChainFresh(Servomotor* m, const int32_t* vel, const uint32_t* ticks,
                          int nSeg, int64_t* movedOut) {
    tfhFreshOpen(m);
    return velChain(m, vel, ticks, nSeg, movedOut);
}

void tm_velocity_move_semantics(void) {
    Serial.println("tm_velocity_move_semantics: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. A VELOCITY SEGMENT TRAVELS EXACTLY SPEED x TIME.
    //    The foundational claim, and the one every other section leans on. It
    //    is asserted to within a few COUNTS out of millions rather than to a
    //    percentage, because the firmware's integrator is exact: there is no
    //    ramp, no rounding per tick, and only one truncation in the whole path.
    //    A percentage tolerance here would hide a genuine scaling error of a
    //    few parts per million, which is precisely the kind of mistake a
    //    fixed-point shift constant produces.
    // =====================================================================
    printf("\n--- 1. a velocity segment travels exactly speed x time ---\n");
    {
        struct { float rps; float secs; } cases[] = {
            { 1.0f, 0.5f },      // 0.5 rotation
            { 3.0f, 0.25f },     // 0.75 rotation
            { 0.4f, 1.5f },      // 0.6 rotation
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            int32_t  v[1] = { rawVel(cases[i].rps) };
            uint32_t t[1] = { rawTicks(cases[i].secs) };
            int64_t moved = 0;
            bool ok = velChainFresh(m, v, t, 1, &moved);
            int64_t want = exactCounts(v, t, 1);
            printf("   %.2f rot/s x %.2f s (raw v=%ld, %lu ticks) -> %lld counts "
                   "(exactly %lld expected)\n",
                   (double)cases[i].rps, (double)cases[i].secs,
                   (long)v[0], (unsigned long)t[0],
                   (long long)moved, (long long)want);
            TEST_RESULT(std::string("a velocity segment at ") +
                            std::to_string((double)cases[i].rps).substr(0, 4) +
                            " rot/s completes without fault", ok);
            TEST_RESULT(std::string("a velocity segment at ") +
                            std::to_string((double)cases[i].rps).substr(0, 4) +
                            " rot/s travels speed x time to the count",
                        ok && llabs((long long)(moved - want)) <= 3);
        }
    }

    // =====================================================================
    // 2. THE SIGN OF THE VELOCITY CHOOSES THE DIRECTION, AND MIRRORS EXACTLY.
    //    The velocity field is a SIGNED int32 on the wire and stays signed all
    //    the way into the accumulator, so negation must be perfectly symmetric
    //    -- not merely "goes the other way". An asymmetry of even one count
    //    would mean a sign-extension or shift-of-negative defect, which would
    //    accumulate over a long job and is invisible to any percentage check.
    // =====================================================================
    printf("\n--- 2. the sign of the velocity chooses the direction ---\n");
    {
        int32_t  vp[1] = { rawVel(2.0f) };
        int32_t  vn[1] = { -rawVel(2.0f) };
        uint32_t t[1]  = { rawTicks(0.5f) };
        int64_t fwd = 0, rev = 0;
        bool okF = velChainFresh(m, vp, t, 1, &fwd);
        bool okR = velChainFresh(m, vn, t, 1, &rev);
        printf("   +2.0 rot/s -> %lld counts; -2.0 rot/s -> %lld counts\n",
               (long long)fwd, (long long)rev);
        TEST_RESULT("a positive velocity advances the commanded position and a "
                    "negative one retreats it",
                    okF && okR && fwd > 0 && rev < 0);
        TEST_RESULT("the two directions mirror each other to within a count",
                    okF && okR && llabs((long long)(fwd + rev)) <= 3);

        // A second pair at a different speed, so the symmetry is a property of
        // the primitive rather than a coincidence at one magnitude.
        int32_t  wp[1] = { rawVel(0.75f) };
        int32_t  wn[1] = { -rawVel(0.75f) };
        uint32_t u[1]  = { rawTicks(1.0f) };
        int64_t f2 = 0, r2 = 0;
        // Deliberately not short-circuited: both legs run even if one fails, so
        // the printed pair is always diagnosable without a rerun.
        bool okF2 = velChainFresh(m, wp, u, 1, &f2);
        bool okR2 = velChainFresh(m, wn, u, 1, &r2);
        bool ok2 = okF2 && okR2;
        printf("   +0.75 rot/s -> %lld counts; -0.75 rot/s -> %lld counts\n",
               (long long)f2, (long long)r2);
        TEST_RESULT("a second, slower pair mirrors just as exactly",
                    ok2 && llabs((long long)(f2 + r2)) <= 3);
    }

    // =====================================================================
    // 3. VELOCITY ZERO IS A TIMED DWELL, NOT A NO-OP.
    //    A zero-velocity segment is the only way to express "wait here" in the
    //    motion queue, and it is the exact construct every chain in this suite
    //    uses as its terminator. So it has to do two things at once: move
    //    nothing, and OCCUPY THE QUEUE for the full commanded duration. If it
    //    were dropped as a no-op, the terminator would stop terminating and
    //    every velocity chain in the suite would become an error-18 generator.
    //    It also has to be self-terminating: expiring at zero velocity is the
    //    one case that must NOT raise an underrun.
    // =====================================================================
    printf("\n--- 3. velocity zero is a timed dwell ---\n");
    {
        tfhFreshOpen(m);
        int64_t p0 = m->getPositionRaw();
        uint32_t shortTicks = rawTicks(0.5f);
        uint32_t longTicks  = rawTicks(1.25f);

        uint32_t tA = millis();
        m->moveWithVelocityRaw(0, shortTicks);
        bool acceptedA = (m->getError() == 0);
        uint8_t depth = m->getNQueuedItems();
        bool drainedA = acceptedA && tfhDrain(m, 12000);
        uint32_t elapsedShort = millis() - tA;
        int64_t movedA = m->getPositionRaw() - p0;

        tfhFreshOpen(m);
        int64_t p1 = m->getPositionRaw();
        uint32_t tB = millis();
        m->moveWithVelocityRaw(0, longTicks);
        bool acceptedB = (m->getError() == 0);
        bool drainedB = acceptedB && tfhDrain(m, 12000);
        uint32_t elapsedLong = millis() - tB;
        int64_t movedB = m->getPositionRaw() - p1;
        uint8_t fatB = tfhFatal(m);

        long diff = (long)elapsedLong - (long)elapsedShort;
        printf("   dwell 0.50 s: queued depth %u, held the queue %lu ms, moved %lld\n",
               (unsigned)depth, (unsigned long)elapsedShort, (long long)movedA);
        printf("   dwell 1.25 s: held the queue %lu ms, moved %lld, fatal %u; "
               "difference %ld ms (0.75 s expected)\n",
               (unsigned long)elapsedLong, (long long)movedB, (unsigned)fatB, diff);

        TEST_RESULT("a zero-velocity segment is accepted and takes a queue slot",
                    acceptedA && depth == 1);
        TEST_RESULT("a zero-velocity segment moves the commanded position by "
                    "exactly nothing",
                    drainedA && drainedB && movedA == 0 && movedB == 0);
        // The absolute times carry the drain poll interval and the post-drain
        // settle, so the DIFFERENCE is the sharp claim; both offsets cancel.
        TEST_RESULT("a dwell holds the queue for its full commanded duration",
                    drainedA && drainedB && diff > 550 && diff < 950);
        TEST_RESULT("a dwell expiring on an empty queue raises no underrun, "
                    "which is why it works as a terminator",
                    fatB != ERR_RUN_OUT_OF_QUEUE_ITEMS && fatB == 0);
    }

    // =====================================================================
    // 4. A VELOCITY SEGMENT HANDS ITS VELOCITY ON WHEN ITS TIME EXPIRES.
    //    THE reason a terminating stop is mandatory, measured directly rather
    //    than inferred from the fault it eventually causes (tm_queue_underrun
    //    owns the fault). Nothing in the firmware zeroes the velocity at a
    //    segment boundary -- the item is just dropped -- so a following
    //    ZERO-ACCELERATION segment, whose whole action is `+= 0`
    //    (motor_control.c:2132), must coast on at the inherited speed.
    //
    //    The discrimination is total: if the velocity were carried across, the
    //    chain covers both segments; if it were reset at the boundary, it
    //    covers only the first. The two answers differ by a factor of three
    //    here, so there is no tolerance question to argue about.
    // =====================================================================
    printf("\n--- 4. a velocity segment hands its velocity to what follows ---\n");
    {
        tfhFreshOpen(m);
        int32_t  v  = rawVel(1.0f);
        uint32_t n1 = rawTicks(0.4f);
        uint32_t n2 = rawTicks(0.8f);
        int64_t before = m->getPositionRaw();
        m->moveWithVelocityRaw(v, n1);
        bool accepted = (m->getError() == 0);
        m->moveWithAccelerationRaw(0, n2);      // pure coast: keeps whatever it inherits
        if (m->getError() != 0) accepted = false;
        m->moveWithVelocityRaw(0, 1);           // and then actually stop
        if (m->getError() != 0) accepted = false;
        bool drained = accepted && tfhDrain(m, 12000);
        uint8_t fat = tfhFatal(m);
        int64_t moved = (drained && fat == 0) ? (m->getPositionRaw() - before) : 0;

        int32_t  vv[1] = { v };
        uint32_t both[1] = { n1 + n2 };
        uint32_t only[1] = { n1 };
        int64_t ifHeld    = exactCounts(vv, both, 1);
        int64_t ifDropped = exactCounts(vv, only, 1);
        printf("   velocity 0.4 s then zero-acceleration coast 0.8 s -> %lld counts "
               "(held: %lld, dropped: %lld)\n",
               (long long)moved, (long long)ifHeld, (long long)ifDropped);

        TEST_RESULT("a velocity segment followed by a coast segment completes cleanly",
                    drained && fat == 0);
        TEST_RESULT("the coast segment inherits the velocity instead of starting "
                    "from rest",
                    drained && fat == 0 && llabs((long long)(moved - ifHeld)) <= 3);
        TEST_RESULT("the chain is not merely the first segment's distance, which "
                    "is what a velocity that reset at the boundary would give",
                    drained && fat == 0 &&
                    llabs((long long)(moved - ifDropped)) > ifDropped / 2);
    }

    // =====================================================================
    // 5. A VELOCITY SEGMENT REPLACES THE VELOCITY, IT DOES NOT ADD TO IT.
    //    The counterpart to section 4 and the sharpest distinction between the
    //    two primitives. Acceleration segments ACCUMULATE across a queued
    //    sequence -- four of them in a row reach four times the speed, which
    //    has caught this campaign out twice. Velocity segments must not: the
    //    firmware assigns rather than accumulates (motor_control.c:2140).
    //
    //    Tested with a chain that goes forward and then back at the same speed
    //    for the same time. If velocity replaced correctly, it returns to where
    //    it started. If it accumulated, the second segment would command zero
    //    and the machine would end a whole segment's travel away -- a mistake
    //    that would be invisible in any single-direction test.
    // =====================================================================
    printf("\n--- 5. a velocity segment replaces rather than accumulates ---\n");
    {
        int32_t  v[2] = { rawVel(1.5f), -rawVel(1.5f) };
        uint32_t t[2] = { rawTicks(0.4f), rawTicks(0.4f) };
        int64_t moved = 0;
        bool ok = velChainFresh(m, v, t, 2, &moved);
        int32_t  firstOnly[1] = { v[0] };
        uint32_t firstTime[1] = { t[0] };
        int64_t ifAccumulated = exactCounts(firstOnly, firstTime, 1);
        printf("   +1.5 rot/s then -1.5 rot/s, 0.4 s each -> %lld counts "
               "(0 if it replaces, about %lld if it accumulated)\n",
               (long long)moved, (long long)ifAccumulated);
        TEST_RESULT("an out-and-back velocity pair completes", ok);
        TEST_RESULT("an out-and-back velocity pair returns exactly to its start",
                    ok && llabs((long long)moved) <= 3);

        // The asymmetric case, which also rules out the milder error of adding
        // the two velocities: 1.0 then 0.5 must cover 1.5 segment-lengths, not 2.5.
        int32_t  w[2] = { rawVel(1.0f), rawVel(0.5f) };
        uint32_t u[2] = { rawTicks(0.5f), rawTicks(0.5f) };
        int64_t moved2 = 0;
        bool ok2 = velChainFresh(m, w, u, 2, &moved2);
        int64_t want2 = exactCounts(w, u, 2);
        printf("   1.0 rot/s then 0.5 rot/s, 0.5 s each -> %lld counts "
               "(expected exactly %lld)\n", (long long)moved2, (long long)want2);
        TEST_RESULT("a slower second segment travels its own speed x time, not "
                    "the sum of both speeds",
                    ok2 && llabs((long long)(moved2 - want2)) <= 3);
    }

    // =====================================================================
    // 6. CHAINING SEGMENTS PRODUCES THE SUM OF SPEED x TIME.
    //    The composition property. Because each segment contributes exactly and
    //    the accumulator carries full sub-count precision between them, a chain
    //    of five different speeds must land on the sum to within the single
    //    read-out truncation -- no per-segment rounding error that would
    //    accumulate over a long program. Reordering the same segments must give
    //    the identical total, which is a real claim about the machine: it says
    //    no segment's contribution depends on what preceded it.
    // =====================================================================
    printf("\n--- 6. a chain travels the sum of speed x time ---\n");
    {
        int32_t  v[5] = { rawVel(1.0f), rawVel(-0.5f), rawVel(2.0f),
                          rawVel(0.25f), rawVel(-1.25f) };
        uint32_t t[5] = { rawTicks(0.30f), rawTicks(0.20f), rawTicks(0.15f),
                          rawTicks(0.40f), rawTicks(0.25f) };
        int64_t moved = 0;
        bool ok = velChainFresh(m, v, t, 5, &moved);
        int64_t want = exactCounts(v, t, 5);
        printf("   five-segment chain -> %lld counts (expected exactly %lld)\n",
               (long long)moved, (long long)want);
        TEST_RESULT("a five-segment velocity chain completes without fault", ok);
        TEST_RESULT("a five-segment chain travels the sum of speed x time",
                    ok && llabs((long long)(moved - want)) <= 4);

        // Same five segments, reversed order.
        int32_t  rv[5] = { v[4], v[3], v[2], v[1], v[0] };
        uint32_t rt[5] = { t[4], t[3], t[2], t[1], t[0] };
        int64_t movedR = 0;
        bool okR = velChainFresh(m, rv, rt, 5, &movedR);
        printf("   the same five segments reversed -> %lld counts\n", (long long)movedR);
        TEST_RESULT("reordering the segments of a chain gives the same total",
                    ok && okR && llabs((long long)(moved - movedR)) <= 4);
    }

    // =====================================================================
    // 7. DISTANCE IS STRICTLY LINEAR IN DURATION -- THERE IS NO HIDDEN RAMP.
    //    A velocity move is documented as an instantaneous step. If the
    //    firmware quietly ramped into the commanded speed, short segments would
    //    fall short by the ramp area while long ones would not, so the distance
    //    would be sub-linear in duration. Doubling and quadrupling the time at
    //    a fixed speed is the direct test: any ramp shows up as a shortfall
    //    that shrinks with duration.
    // =====================================================================
    printf("\n--- 7. distance is linear in duration, so there is no ramp ---\n");
    {
        int32_t  v[1] = { rawVel(2.0f) };
        uint32_t base = rawTicks(0.2f);
        uint32_t t1[1] = { base };
        uint32_t t2[1] = { base * 2 };
        uint32_t t4[1] = { base * 4 };
        int64_t d1 = 0, d2 = 0, d4 = 0;
        // Each leg runs unconditionally so all three numbers are printed even
        // when one of them is the one that failed.
        bool ok1 = velChainFresh(m, v, t1, 1, &d1);
        bool ok2 = velChainFresh(m, v, t2, 1, &d2);
        bool ok4 = velChainFresh(m, v, t4, 1, &d4);
        bool ok = ok1 && ok2 && ok4;
        printf("   2.0 rot/s for 1x/2x/4x of %lu ticks -> %lld / %lld / %lld counts\n",
               (unsigned long)base, (long long)d1, (long long)d2, (long long)d4);
        TEST_RESULT("all three durations complete", ok);
        TEST_RESULT("twice the duration travels exactly twice as far",
                    ok && llabs((long long)(d2 - 2 * d1)) <= 4);
        TEST_RESULT("four times the duration travels exactly four times as far",
                    ok && llabs((long long)(d4 - 4 * d1)) <= 6);
    }

    // =====================================================================
    // 8. THE VELOCITY LIMIT IS ENFORCED ON THE EXACT BOUNDARY, INCLUSIVELY.
    //    `set_max_velocity()` stores its argument shifted left by
    //    VELOCITY_SHIFT_LEFT (motor_control.c:3347-3348) and `add_to_queue()`
    //    compares the queued velocity through the identical shift
    //    (motor_control.c:1842-1848). The comparison is `>`, so the limit is
    //    INCLUSIVE and the two raw wire values can be compared bit for bit.
    //    That makes the boundary exact: raw L must be accepted and raw L+1 must
    //    be refused, with no grey zone.
    //
    //    A limit is a PASS/FAIL CHECK, never a clamp -- the firmware does not
    //    slow an over-ambitious move down to fit, it refuses it. Note that the
    //    refusal is a latched fatal error, after which get_status is the only
    //    command that returns real data, so the position cannot be read back to
    //    confirm nothing moved; the reset that follows is the recovery.
    //
    //    Limits are all kept well under EXPERIMENTAL_MAX_VELOCITY (2 x
    //    MAX_VELOCITY, about 18.7 rot/s, motor_control.h:61), because above
    //    that the setter silently clamps and L would no longer be the limit.
    // =====================================================================
    printf("\n--- 8. the velocity limit is exact and inclusive ---\n");
    {
        const float limits[] = { 1.0f, 2.5f, 6.0f };
        for (unsigned i = 0; i < sizeof(limits) / sizeof(limits[0]); i++) {
            int32_t L = rawVel(limits[i]);
            uint32_t ticks = rawTicks(0.3f);

            // (a) EXACTLY at the limit: accepted, completes, delivers speed x time.
            tfhFreshOpen(m);
            m->setMaximumVelocityRaw((uint32_t)L);
            bool limitSet = (m->getError() == 0);
            int32_t  atv[1] = { L };
            uint32_t att[1] = { ticks };
            int64_t moved = 0;
            bool okAt = limitSet && velChain(m, atv, att, 1, &moved);
            int64_t want = exactCounts(atv, att, 1);

            // (b) ONE RAW STEP past the limit: refused with error 16.
            tfhFreshOpen(m);
            m->setMaximumVelocityRaw((uint32_t)L);
            m->moveWithVelocityRaw(L + 1, ticks);
            bool rejectedPos = (m->getError() != 0);
            uint8_t codePos = tfhFatal(m);

            // (c) The same one step past, negative. llabs() is applied before
            //     the comparison, so the limit must be symmetric.
            tfhFreshOpen(m);
            m->setMaximumVelocityRaw((uint32_t)L);
            m->moveWithVelocityRaw(-(L + 1), ticks);
            bool rejectedNeg = (m->getError() != 0);
            uint8_t codeNeg = tfhFatal(m);
            tfhReset(m);

            printf("   limit %.2f rot/s (raw %ld): at limit moved %lld of %lld; "
                   "L+1 -> refused=%d code %u; -(L+1) -> refused=%d code %u\n",
                   (double)limits[i], (long)L, (long long)moved, (long long)want,
                   (int)rejectedPos, (unsigned)codePos,
                   (int)rejectedNeg, (unsigned)codeNeg);

            std::string tag = std::to_string((double)limits[i]).substr(0, 4);
            TEST_RESULT(std::string("a velocity of exactly the limit (") + tag +
                            " rot/s) is accepted and delivers speed x time",
                        okAt && llabs((long long)(moved - want)) <= 3);
            TEST_RESULT(std::string("one raw step past the ") + tag +
                            " rot/s limit is refused with ERROR_VEL_TOO_HIGH (16)",
                        rejectedPos && codePos == ERR_VEL_TOO_HIGH);
            TEST_RESULT(std::string("one raw step past the ") + tag +
                            " rot/s limit in the negative direction is refused too",
                        rejectedNeg && codeNeg == ERR_VEL_TOO_HIGH);
        }
    }

    // =====================================================================
    // 9. THE DURATION EDGES. A velocity move must last at least one control
    //    tick: zero is rejected outright in the command handler, BEFORE the
    //    item reaches the queue (main.c:616-618), because a zero-duration item
    //    was once a silent no-op that left callers waiting for motion that
    //    never came. One tick, at the other end, is perfectly legal and must
    //    deliver exactly one tick's worth of travel -- a couple of hundred
    //    counts, small but not zero, and therefore a real check that the
    //    smallest possible segment is not quietly dropped.
    //
    //    A one-tick segment cannot be the FIRST item of a chain. It is consumed
    //    on the very next control tick (32 us), the queue then empties while the
    //    velocity is still non-zero, and the motor latches
    //    ERROR_RUN_OUT_OF_QUEUE_ITEMS (motor_control.c:2148-2164) about a
    //    millisecond before the terminating stop can get across the bus. That is
    //    a property of the queue, not of one-tick segments, and it is already
    //    owned by tm_queue_underrun -- so it is designed out here rather than
    //    rediscovered: a zero-velocity DWELL goes in front to hold the queue open
    //    while the one-tick segment and its stop are sent. The dwell contributes
    //    exactly zero displacement (section 3), so the measured travel is still
    //    exactly one tick's worth and nothing else.
    // =====================================================================
    printf("\n--- 9. the duration edges: zero is refused, one tick works ---\n");
    {
        tfhFreshOpen(m);
        m->moveWithVelocityRaw(rawVel(1.0f), 0);
        bool refused = (m->getError() != 0);
        uint8_t code = tfhFatal(m);
        printf("   zero duration -> refused=%d code %u (expected %u)\n",
               (int)refused, (unsigned)code, (unsigned)ERR_PARAM_OUT_OF_RANGE);
        TEST_RESULT("a zero-duration velocity move is refused rather than "
                    "silently dropped", refused && code != 0 && code != 0xFF);
        TEST_RESULT("and it reports ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    code == ERR_PARAM_OUT_OF_RANGE);

        // Segment 0 is the queue-holding dwell (zero velocity, zero distance);
        // segment 1 is the subject. velChain() appends the stop after both.
        int32_t  v[2] = { 0,             rawVel(2.0f) };
        uint32_t t[2] = { rawTicks(0.5f), 1           };
        int64_t moved = 0;
        bool ok = velChainFresh(m, v, t, 2, &moved);
        int64_t want = exactCounts(v, t, 2);   // == (rawVel(2.0) * 1) >> 20
        printf("   0.5 s dwell then a one-tick segment at 2.0 rot/s -> %lld counts "
               "(expected %lld)\n", (long long)moved, (long long)want);
        TEST_RESULT("a one-tick velocity segment is legal and delivers exactly "
                    "one tick of travel",
                    ok && want > 0 && llabs((long long)(moved - want)) <= 2);
    }

    // =====================================================================
    // 10. ONE VELOCITY SEGMENT COSTS ONE QUEUE SLOT. Unlike a trapezoid move,
    //     which the planner expands into three queue items (two on the
    //     0.15.12.0 one-tick-ramp fast path), a velocity move is a single
    //     add_to_queue() call (main.c:619) and therefore a single slot. That is
    //     the whole reason to reach for the primitive: it is how a caller fits
    //     a long, finely-shaped profile into a 32-slot queue.
    // =====================================================================
    printf("\n--- 10. one velocity segment costs one queue slot ---\n");
    {
        tfhFreshOpen(m);
        int32_t v = rawVel(1.0f);
        uint32_t t = rawTicks(1.0f);          // long enough that none expires while we look
        bool accepted = true;
        for (int i = 0; i < 5 && accepted; i++) {
            m->moveWithVelocityRaw(v, t);
            if (m->getError() != 0) accepted = false;
        }
        uint8_t depthFive = m->getNQueuedItems();
        m->moveWithVelocityRaw(0, 1);
        if (m->getError() != 0) accepted = false;
        uint8_t depthSix = m->getNQueuedItems();
        bool drained = accepted && tfhDrain(m, 20000);
        uint8_t fat = tfhFatal(m);
        printf("   five segments -> depth %u; plus the stop -> depth %u; fatal %u\n",
               (unsigned)depthFive, (unsigned)depthSix, (unsigned)fat);
        TEST_RESULT("five velocity segments occupy exactly five queue slots",
                    accepted && depthFive == 5);
        TEST_RESULT("the terminating stop costs one slot like any other segment, "
                    "and the whole chain drains cleanly",
                    depthSix == 6 && drained && fat == 0);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
