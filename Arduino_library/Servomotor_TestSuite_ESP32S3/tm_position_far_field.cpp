#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_position_far_field.cpp
//
// THE POSITION ACCUMULATOR TEN THOUSAND ROTATIONS FROM THE ORIGIN.
//
// The commanded position is not an int64 counter. It is a 96-bit fixed-point
// accumulator whose whole-count part starts at bit 32 (motor_control.c:137-152):
//
//     typedef struct {
//         uint32_t low;                 // 32 fractional bits
//         union { struct { uint32_t mid; int32_t high; }; int64_t position; };
//     } int96_t;
//
// `position` -- the number "Get position" (cmd 34) returns, via
// get_motor_position() at motor_control.c:3421 -- is the upper 64 bits: the
// UNSIGNED word `mid` glued underneath the SIGNED word `high`. Every advance of
// the machine goes through add_int96() (motor_control.c:385-408), which adds a
// 64-bit delta to the low two words and propagates a carry by hand:
//
//     sum = (uint64_t)a_i96->mid + b_mid + carry;
//     result->mid = (uint32_t)sum;
//     carry = sum >> 32;
//     sum = (int64_t)a_i96->high + b_high + (int64_t)carry;
//
// That hand-written carry has two structurally interesting moments, and BOTH of
// them are past where any existing test goes:
//
//   * 2^31 counts (655.36 rotations) -- `mid` runs into its own top bit. Read
//     as a signed 32-bit word it goes negative here, which is exactly what the
//     firmware's superseded position code did (`((int32_t*)&position)[1]`, still
//     visible commented out at motor_control.c:1811 and :2638). The union is
//     what makes it right; nothing checks that it stays right.
//   * 2^32 counts (1310.72 rotations) -- the FIRST carry out of `mid` into
//     `high`. Below this the high word is 0 (or -1) forever and the carry path
//     never runs with a nonzero result.
//
// HOW THIS DIFFERS FROM tm_position_accumulator, which is the nearest module.
// That one proves a SINGLE move of INT32_MAX counts is performed correctly and
// then accumulates out to 400 rotations. Its furthest RESTING position is
// 2,147,483,647 counts -- one count short of 2^31, so it stops immediately
// before the first of the two boundaries above and never approaches the second.
// This module starts where that one stops: it walks to 2^35 counts
// (34,359,738,368 -- 10,485.76 rotations, sixteen times beyond the int32 edge),
// crossing 2^31, 2^32, 2^33 and 2^34 on the way and asserting the reported
// position against the exact running sum at every leg. It then asks whether the
// machine is still USABLE out there, which no module asks at any distance.
//
// The other three near neighbours, and what is new here:
//   * tm_goto_absolute_semantics section 7 proves that a target more than
//     INT32_MAX counts from the planner's baseline is refused with
//     ERROR_MOVE_TOO_FAR (46) rather than truncated -- measured a couple of
//     rotations from the origin. Section 3 below draws the far-field
//     consequence, which is a different and more alarming statement: once the
//     machine is further than INT32_MAX counts out, THE ORIGIN ITSELF IS NO
//     LONGER AN ADDRESSABLE TARGET. "Go to position 0" is refused. The payload
//     field is int32 (main.c:277) while the accumulator is not, so the absolute
//     command's reach is a 4.3-billion-count window that travels with the
//     machine, and only "Zero position" can bring the origin back inside it.
//   * tm_safety_fence_arithmetic exercises the fence across the whole int64
//     line, but with the machine parked near zero. Sections 2 and 5 put the
//     POSITION and the FENCE BOUNDS beyond int32 at the same time, which is the
//     only configuration in which the fence's 64-bit comparison actually has to
//     be 64-bit.
//   * tm_deviation_tracking characterises the deviation watchdog. Here it is
//     deliberately raised out of the way, and the module asserts it never fired
//     across 10,485 rotations of commanded travel.
//
// WHY A LEG IS EXACTLY 2^30 COUNTS. Every queue item predicts its own end
// position as a plain int64 (motor_control.c:1808 and :1852), in internal units
// where one encoder count is 2^32 (MICROSTEPS_PER_ROTATION, motor_control.h:42).
// So a single segment carrying D counts computes D * 2^32, and D is therefore
// bounded by about 2^31 before that int64 wraps -- which is why a single
// INT32_MAX move sits right at the edge. 2^30 counts (327.68 rotations) is half
// of that bound: comfortably safe, a power of two so that 2, 4, 8, 16 and 32
// legs land exactly on 2^31 ... 2^35, and small enough that no leg runs long
// enough to be interesting for any other reason.
//
// WHY THIS MODULE IS SLOW, AND WHY THAT IS IRREDUCIBLE. The velocity ceiling is
// EXPERIMENTAL_MAX_VELOCITY = 2 x MAX_VELOCITY (motor_control.h:61), about 18.7
// rotations/second, and set_max_velocity() CLAMPS to it (motor_control.c:3349).
// Ten thousand rotations therefore cannot be commanded in less than about nine
// minutes no matter how the test is written; the whole module takes roughly
// sixteen, inside the 1800 s registry watchdog. Nothing here is padding: it is
// travel time, and travel time is the subject.
//
// WHAT IS ASSERTED, as one property: the commanded position is a genuine wide
// accumulator that behaves identically 34 billion counts from the origin as it
// does at the origin -- same displacement per move, no drift against the exact
// running sum, correct across both carry boundaries and in both signs -- while
// the 32-bit-payload commands layered on top of it (absolute go-to) fail LOUDLY
// rather than wrapping when their reach runs out, and the 64-bit ones (safety
// fence, zero position) keep working.
//
// SAFETY. The MOSFETs stay OFF for the whole module: enableMosfets() and
// goToClosedLoop() are never called, and disableMosfets() is issued explicitly
// after every reset. Every distance is read from the COMMANDED position, which
// the planner advances whether or not the output stage is energised, so ten
// thousand commanded rotations turn nothing -- section 1 asserts that directly
// by showing the hall reading is unchanged at the end. Raising the deviation
// window only silences a diagnostic about a shaft that was never going to
// follow; it cannot cause motion. There are no broadcasts, no alias changes and
// no test modes, so this is safe on the shared 35-motor rack. Every fault it
// raises deliberately is cleared by the reset that ends its section.
// ---------------------------------------------------------------------------

static const uint8_t PFF_ERR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE = 27;
static const uint8_t PFF_ERR_MOVE_TOO_FAR                          = 46;

static const int64_t PFF_LEG      = 1073741824LL;    // 2^30 counts = 327.68 rotations
static const int     PFF_LEGS     = 32;              // 32 legs -> 2^35 counts
static const int64_t PFF_POW2_31  = 2147483648LL;
static const int64_t PFF_POW2_32  = 4294967296LL;
static const int64_t PFF_POW2_33  = 8589934592LL;
static const int64_t PFF_POW2_34  = 17179869184LL;
static const int64_t PFF_POW2_35  = 34359738368LL;   // the destination

// Ceilings for this module. 18.5 rot/s is just under the firmware's clamp so it
// is accepted verbatim; the legs are planned for a 16.0 rot/s average, leaving
// 14% of headroom. That margin matters: the limits are PASS/FAIL CHECKS, never
// clamps -- an over-ambitious move is refused with fatal error 15 or 16, not
// stretched to fit.
static const float  PFF_CEILING_ROT_S = 18.5f;
static const double PFF_PLAN_ROT_S    = 16.0;

// A cumulative-drift budget for the whole 32-leg walk. compute_trapezoid_move()
// truncates the derived acceleration (motor_control.c:485), which costs at most
// (delta_t1 + delta_t2) * delta_t1 / 2^24 -- about four counts per leg here, so
// a few hundred over the walk. 4096 counts is generous for that and still a
// MILLION times smaller than the 2^32 counts a mishandled carry out of `mid`
// would cost, which is the failure this bound is looking for.
static const int64_t PFF_DRIFT_BUDGET = 4096;

// Reset, open every ceiling, and state the output stage is off.
static void pffOpen(Servomotor* m) {
    tfhFreshOpen(m);                          // reset + 15 rot/s, 5000 rot/s^2, 100000-rotation deviation window
    m->disableMosfets();                      // nothing in this module may turn a shaft
    m->setMaximumVelocity(PFF_CEILING_ROT_S);
    m->setMaximumAcceleration(8000.0f);
    delay(50);
}

// Ticks for a move of `counts` at the planned average, plus ramp headroom.
static uint32_t pffTicks(int64_t counts) {
    double rotations = (double)(counts < 0 ? -counts : counts) / (double)TFH_CPR;
    double seconds = rotations / PFF_PLAN_ROT_S + 0.3;
    return (uint32_t)(seconds * (double)TFH_TPS);
}

// The velocity ceiling the firmware actually adopted, in rotations/second.
// max_velocity is reported raw by "Get debug values" (motor_control.c:3680) in
// internal units of 2^32 per count per time step.
static double pffCeilingRotPerSec(Servomotor* m) {
    getDebugValuesResponse dv = m->getDebugValues();
    if (m->getError() != 0) return -1.0;
    return (double)dv.maxVelocity * (double)TFH_TPS / ((double)TFH_CPR * 4294967296.0);
}

void tm_position_far_field(void) {
    Serial.println("tm_position_far_field: BEGIN\n");
    Servomotor* m = tfGetMotor();
    uint32_t moduleStart = millis();

    // Measured in section 1 and reused in section 4 for the sign-symmetry check.
    int64_t positiveLegDelta = 0;

    // =====================================================================
    // 1. THE WALK: 2^35 COUNTS, IN THIRTY-TWO EQUAL LEGS.
    //
    //    Thirty-two identical trapezoid moves of 2^30 counts each. After every
    //    leg the reported position is compared against the exact running sum of
    //    what was commanded, so a truncation that costs a handful of counts per
    //    move shows up as a growing discrepancy instead of hiding inside the
    //    tolerance of any single move -- and a mishandled carry out of `mid`,
    //    which costs 4,294,967,296 counts at once, cannot hide at all.
    //
    //    The legs are also compared against EACH OTHER. Every leg is the same
    //    command with the same limits, so the planner's arithmetic must deliver
    //    the same distance for leg 32 (starting 10,158 rotations out) as for
    //    leg 1 (starting at zero). If the delivered displacement depended on the
    //    magnitude of the starting position, that dependence would be the bug.
    //    The tolerance is two counts rather than zero because int96_t.low is a
    //    32-bit FRACTIONAL accumulator that is not reset between moves: a leg
    //    that begins with a partial count already banked can legitimately hand
    //    over one whole count more than its predecessor.
    //
    //    The landmarks are read one leg AFTER each power of two, not at it. A
    //    leg is a few counts short of the 2^30 it was commanded (the planner
    //    truncates its derived acceleration), so the position after two legs
    //    would sit just BELOW 2^31 and the boundary claim would be false by a
    //    handful of counts. Reading at legs 3, 5, 9 and 17 puts the machine a
    //    whole 2^30 counts past 2^31, 2^32, 2^33 and 2^34 respectively, and has
    //    the further advantage that each boundary is crossed in the MIDDLE of a
    //    move -- by the control loop advancing the accumulator tick by tick,
    //    not merely by the queue-time prediction.
    // =====================================================================
    printf("\n--- 1. walking out to 2^35 counts (10,485.76 rotations) in 32 legs ---\n");
    {
        pffOpen(m);

        int64_t start = m->getPositionRaw();
        bool startRead = (m->getError() == 0);
        TEST_RESULT("a reset leaves the machine at exactly zero",
                    startRead && start == 0);

        double ceiling = pffCeilingRotPerSec(m);
        printf("   velocity ceiling adopted by the firmware: %.3f rotations/second\n", ceiling);
        TEST_RESULT("the velocity ceiling really was raised, so the legs are planned "
                    "against the speed the firmware will actually allow",
                    ceiling >= 17.5);

        int64_t hallBefore = m->getHallSensorPositionRaw();
        bool hallReadOk = (m->getError() == 0);

        const uint32_t legTicks = pffTicks(PFF_LEG);
        printf("   leg: %lld counts over %u ticks (%.2f s), planned average %.1f rot/s\n",
               (long long)PFF_LEG, (unsigned)legTicks,
               (double)legTicks / (double)TFH_TPS, PFF_PLAN_ROT_S);

        int64_t deltas[PFF_LEGS];
        int64_t past31 = 0, past32 = 0, past33 = 0, past34 = 0;
        int64_t expected = 0;
        int64_t worstDrift = 0;
        int64_t worstLegErr = 0;
        int completed = 0;
        bool allOk = true;

        for (int i = 0; i < PFF_LEGS; i++) {
            bool ok = false;
            int64_t d = tfhMove(m, (int32_t)PFF_LEG, legTicks, &ok);
            if (!ok) {
                allOk = false;
                printf("   leg %d did not complete (fatal=%u)\n", i + 1, (unsigned)tfhFatal(m));
                break;
            }
            deltas[i] = d;
            completed++;
            expected += PFF_LEG;

            int64_t pos = m->getPositionRaw();
            if (m->getError() != 0) { allOk = false; printf("   leg %d: position unreadable\n", i + 1); break; }

            int64_t drift = pos - expected;
            if (llabs((long long)drift) > llabs((long long)worstDrift)) worstDrift = drift;
            int64_t legErr = d - PFF_LEG;
            if (llabs((long long)legErr) > llabs((long long)worstLegErr)) worstLegErr = legErr;

            if (i == 2)  past31 = pos;      // leg 3:  ~3.22e9, a full 2^30 past 2^31
            if (i == 4)  past32 = pos;      // leg 5:  ~5.37e9, a full 2^30 past 2^32
            if (i == 8)  past33 = pos;      // leg 9:  ~9.66e9, a full 2^30 past 2^33
            if (i == 16) past34 = pos;      // leg 17: ~1.83e10, a full 2^30 past 2^34

            if (((i + 1) % 8) == 0 || i == 2 || i == 4) {
                printf("   after leg %2d: position %lld (%.2f rotations), drift %lld counts\n",
                       i + 1, (long long)pos, (double)pos / (double)TFH_CPR, (long long)drift);
            }
        }

        int64_t minDelta = 0, maxDelta = 0;
        for (int i = 0; i < completed; i++) {
            if (i == 0 || deltas[i] < minDelta) minDelta = deltas[i];
            if (i == 0 || deltas[i] > maxDelta) maxDelta = deltas[i];
        }
        if (completed > 0) positiveLegDelta = deltas[0];

        int64_t finalPos = allOk ? m->getPositionRaw() : 0;
        uint8_t fat = tfhFatal(m);
        int64_t hallAfter = m->getHallSensorPositionRaw();
        bool hallOk = hallReadOk && (m->getError() == 0);

        printf("   %d/%d legs completed; per-leg delivered min %lld max %lld (spread %lld)\n",
               completed, PFF_LEGS, (long long)minDelta, (long long)maxDelta,
               (long long)(maxDelta - minDelta));
        printf("   worst per-leg shortfall %lld counts, worst cumulative drift %lld counts\n",
               (long long)worstLegErr, (long long)worstDrift);
        printf("   final position %lld counts = %.2f rotations (target %lld)\n",
               (long long)finalPos, (double)finalPos / (double)TFH_CPR, (long long)PFF_POW2_35);
        printf("   hall position: %lld before, %lld after (moved %lld counts)\n",
               (long long)hallBefore, (long long)hallAfter,
               (long long)(hallAfter - hallBefore));

        TEST_RESULT("all thirty-two legs are accepted and complete without a fault",
                    allOk && completed == PFF_LEGS);
        TEST_RESULT("every leg delivers the same displacement, however far from the "
                    "origin it starts",
                    completed == PFF_LEGS && (maxDelta - minDelta) <= 2);
        TEST_RESULT("each leg delivers the displacement it was commanded",
                    completed == PFF_LEGS && llabs((long long)worstLegErr) <= 16);
        TEST_RESULT("the position passes 2^31 counts, where the mid word runs into its "
                    "own top bit, and is still reported positive",
                    past31 > PFF_POW2_31 && past31 > 0);
        TEST_RESULT("the position passes 2^32 counts, so the carry out of the mid word "
                    "into the high word has run, and it is still reported positive",
                    past32 > PFF_POW2_32 && past32 > 0);
        TEST_RESULT("the position goes on to pass 2^33 and then 2^34 counts",
                    past33 > PFF_POW2_33 && past34 > PFF_POW2_34);
        TEST_RESULT("the machine ends up at 2^35 counts, ten thousand four hundred "
                    "rotations from the origin",
                    allOk && llabs((long long)(finalPos - PFF_POW2_35)) <= PFF_DRIFT_BUDGET);
        TEST_RESULT("the reported position never drifts from the exact running sum of "
                    "the legs",
                    allOk && llabs((long long)worstDrift) <= PFF_DRIFT_BUDGET);
        TEST_RESULT("no fatal error is latched after ten thousand rotations of "
                    "commanded travel",
                    fat == 0);
        TEST_RESULT("the shaft never turned: the hall reading is unchanged while the "
                    "commanded position advanced thirty-four billion counts",
                    hallOk && llabs((long long)(hallAfter - hallBefore)) < TFH_CPR / 4);

        // =================================================================
        // 2. AND OUT THERE, THE ORDINARY OPERATIONS STILL WORK.
        //
        //    This runs on section 1's END STATE and deliberately does not
        //    reset first: re-walking would cost eleven minutes per subsection.
        //    Everything here is therefore chosen to be NON-fatal, so the far
        //    position survives from one check to the next; the two deliberate
        //    faults this module raises live in sections 3 and 5, each with its
        //    own much shorter walk.
        //
        //    Order matters in one place. The fence is restored to the full
        //    int64 line BEFORE zeroPosition, because zeroing moves the reported
        //    position to 0 while a fence pinned around 2^35 counts would still
        //    be in force -- and the per-tick check at motor_control.c:2640
        //    would fault the machine for being outside it a tick later.
        // =================================================================
        //    Each check below is written ONCE and carries `farOut &&` in its
        //    condition, rather than being duplicated into a skip branch: if the
        //    walk never reached the far field these must FAIL, not silently
        //    measure something at some other distance.
        printf("\n--- 2. what still works 34 billion counts from the origin ---\n");
        {
            const bool farOut = allOk && (completed == PFF_LEGS) &&
                                (finalPos > PFF_POW2_34);
            const int32_t small = (int32_t)(TFH_CPR / 4);      // a quarter turn
            if (!farOut) printf("   the walk did not reach the far field; "
                                "the checks below must fail\n");

            int64_t base = m->getPositionRaw();
            bool okOut = false;
            int64_t movedOut = 0;
            if (farOut) {
                movedOut = tfhMove(m, small, pffTicks(small), &okOut);
                printf("   quarter-turn move at %lld counts out: delivered %lld of %ld\n",
                       (long long)base, (long long)movedOut, (long)small);
            }
            TEST_RESULT("a small relative move is accepted thirty-four billion counts "
                        "from the origin", farOut && okOut);
            TEST_RESULT("and it delivers its full displacement out there",
                        farOut && okOut && llabs((long long)(movedOut - small)) <= 16);

            bool okBack = false;
            int64_t returned = 0;
            if (farOut) {
                int64_t movedBack = tfhMove(m, -small, pffTicks(small), &okBack);
                returned = m->getPositionRaw();
                printf("   and back: delivered %lld, position %lld (was %lld)\n",
                       (long long)movedBack, (long long)returned, (long long)base);
            }
            TEST_RESULT("moving back returns to the position it started from",
                        farOut && okBack && llabs((long long)(returned - base)) <= 32);

            // A fence pinned around the far position. Both bounds are ~3.4e10,
            // sixteen times outside int32, so only a 64-bit fence can express
            // them at all -- the superseded int32 comparison could not have.
            bool fenceSet = false;
            int64_t upper = 0;
            if (farOut) {
                int64_t here  = m->getPositionRaw();
                int64_t lower = here - 2 * TFH_CPR;
                upper = here + 2 * TFH_CPR;
                m->setSafetyLimitsRaw(lower, upper);
                fenceSet = (m->getError() == 0) && (tfhFatal(m) == 0);
                printf("   far fence: [%lld, %lld]\n", (long long)lower, (long long)upper);
            }
            TEST_RESULT("a safety fence whose bounds need 64 bits is accepted out there",
                        farOut && fenceSet && upper > 2147483647LL);

            bool okInside = false;
            int64_t movedInside = 0;
            if (farOut && fenceSet) {
                movedInside = tfhMove(m, small, pffTicks(small), &okInside);
                printf("   move inside the far fence: delivered %lld, fatal=%u\n",
                       (long long)movedInside, (unsigned)tfhFatal(m));
            }
            TEST_RESULT("a move that stays inside that far fence completes untouched",
                        farOut && okInside && llabs((long long)(movedInside - small)) <= 16);

            bool fenceOpen = false;
            if (farOut) {
                m->setSafetyLimitsRaw(INT64_MIN, INT64_MAX);
                fenceOpen = (m->getError() == 0) && (tfhFatal(m) == 0);
            }
            TEST_RESULT("the fence can be reopened to the whole int64 line",
                        farOut && fenceOpen);

            // Zero position from the far field. It writes all three words of the
            // accumulator (motor_control.c:3311-3338), so the answer must be an
            // exact zero, not a small remainder.
            bool zeroAccepted = false;
            uint8_t zeroFatal = 0xFF;
            int64_t afterZero = -1;
            if (farOut) {
                int64_t beforeZero = m->getPositionRaw();
                m->zeroPosition();
                zeroAccepted = (m->getError() == 0);
                delay(200);
                zeroFatal = tfhFatal(m);
                afterZero = m->getPositionRaw();
                printf("   zeroPosition from %lld counts -> %lld (fatal=%u)\n",
                       (long long)beforeZero, (long long)afterZero, (unsigned)zeroFatal);
            }
            TEST_RESULT("zeroPosition is accepted thirty-four billion counts from the "
                        "origin", farOut && zeroAccepted && zeroFatal == 0);
            TEST_RESULT("and it brings the reported position back to exactly zero",
                        farOut && zeroAccepted && zeroFatal == 0 && afterZero == 0);

            // The absolute command's reach is measured from the planner's
            // baseline, so re-zeroing the frame restores it (section 3 shows
            // what it is like before the frame is re-zeroed).
            bool gotoDrained = false;
            uint8_t gotoFatal = 0xFF;
            int64_t landed = 0;
            const int32_t target = (int32_t)(TFH_CPR / 4);
            if (farOut && afterZero == 0) {
                m->goToPositionRaw(target, pffTicks(target));
                gotoDrained = (m->getError() == 0) && tfhDrain(m, 12000);
                gotoFatal = tfhFatal(m);
                landed = m->getPositionRaw();
                printf("   goToPosition(%ld) after re-zeroing: landed %lld (fatal=%u)\n",
                       (long)target, (long long)landed, (unsigned)gotoFatal);
            }
            TEST_RESULT("an absolute goToPosition works again once the frame has been "
                        "re-zeroed", farOut && gotoDrained && gotoFatal == 0);
            TEST_RESULT("and it lands on the target it was given",
                        farOut && gotoDrained && gotoFatal == 0 &&
                        llabs((long long)(landed - target)) <= 16);
        }

        tfhReset(m);
    }

    // =====================================================================
    // 3. THE ABSOLUTE COMMAND'S REACH TRAVELS WITH THE MACHINE, AND THE
    //    ORIGIN FALLS OUT OF IT.
    //
    //    "Go to position" carries an int32 target on the wire (main.c:277)
    //    while the accumulator behind it is not int32 at all. The firmware
    //    computes the displacement in int64 and then bounds it explicitly
    //    (motor_control.c:2074-2078):
    //
    //        int64_t total_displacement = absolute_position - position_after_last_queue_item_i64;
    //        if((total_displacement > INT32_MAX) || (total_displacement < INT32_MIN))
    //            fatal_error(ERROR_MOVE_TOO_FAR);              // code 46
    //
    //    So the set of reachable absolute targets is a 4.3-billion-count window
    //    centred on wherever the machine currently is. Park it 983 rotations
    //    out -- past INT32_MAX counts -- and that window no longer contains
    //    zero: "go to position 0" becomes a fatal error. It is refused, loudly,
    //    rather than being truncated into a move in some arbitrary direction,
    //    which is the point. tm_goto_absolute_semantics proves the refusal near
    //    the origin; what is new here is that the ORIGIN is the thing being
    //    refused, and that the same command with a nearer target is accepted
    //    from the very same position.
    //
    //    The accepted move is stopped rather than drained: it only needs to be
    //    shown to pass queue-time validation, and running it would undo the
    //    walk that set the test up.
    // =====================================================================
    printf("\n--- 3. out past INT32_MAX counts, the origin is not addressable ---\n");
    {
        pffOpen(m);
        const uint32_t legTicks = pffTicks(PFF_LEG);
        bool walked = true;
        for (int i = 0; i < 3 && walked; i++) {
            bool ok = false;
            tfhMove(m, (int32_t)PFF_LEG, legTicks, &ok);
            if (!ok) walked = false;
        }
        int64_t here = walked ? m->getPositionRaw() : 0;
        printf("   parked at %lld counts (%.2f rotations); INT32_MAX is %ld\n",
               (long long)here, (double)here / (double)TFH_CPR, (long)2147483647L);
        TEST_RESULT("the machine reaches nine hundred rotations, past INT32_MAX counts",
                    walked && here > 2147483647LL);

        // A target 1.07 billion counts away: inside the reach window.
        m->goToPositionRaw((int32_t)2147483647L, legTicks);
        bool nearAccepted = (m->getError() == 0);
        delay(150);
        uint8_t nearFatal = tfhFatal(m);
        printf("   goToPosition(INT32_MAX) from here: accepted=%d fatal=%u\n",
               (int)nearAccepted, (unsigned)nearFatal);
        TEST_RESULT("an absolute target that is within int32 reach is still accepted "
                    "out there", walked && nearAccepted && nearFatal == 0);

        m->emergencyStop();
        delay(400);
        int64_t stopped = m->getPositionRaw();
        bool stoppedOk = (m->getError() == 0) && (tfhFatal(m) == 0);
        printf("   after emergency stop: %lld counts\n", (long long)stopped);
        TEST_RESULT("the machine is still past INT32_MAX counts after the stop",
                    stoppedOk && stopped > 2147483647LL);

        // The origin: more than INT32_MAX counts away, in the negative direction.
        m->goToPositionRaw(0, legTicks);
        delay(250);
        uint8_t tooFar = tfhFatal(m);
        printf("   goToPosition(0) from %lld counts: fatal=%u (expect %u)\n",
               (long long)stopped, (unsigned)tooFar, (unsigned)PFF_ERR_MOVE_TOO_FAR);
        TEST_RESULT("from out there the origin itself is unreachable: go to position 0 "
                    "raises ERROR_MOVE_TOO_FAR (46) instead of wrapping",
                    tooFar == PFF_ERR_MOVE_TOO_FAR);

        tfhReset(m);
        TEST_RESULT("a reset clears the reach fault", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 4. THE SAME WALK DOWNWARDS.
    //
    //    Sign extension in hand-written wide arithmetic is a classic source of
    //    asymmetry, and add_int96() sign-extends its 64-bit operand by hand:
    //
    //        int32_t b_high = (*b_i64 < 0) ? -1 : 0;
    //
    //    Going negative exercises a different path entirely. `high` becomes -1
    //    the instant the position drops below zero; `mid` runs into its top bit
    //    at -2^31; and `high` only borrows down to -2 BELOW -2^32 -- at exactly
    //    -2^32 the words are mid = 0, high = -1, so the landmarks here are again
    //    read a leg past each boundary rather than at it. Five legs cover all of
    //    that, so this section is cheap; what it buys on top is the assertion
    //    that a negative leg covers EXACTLY the distance a positive one does,
    //    compared against the number measured in section 1.
    // =====================================================================
    printf("\n--- 4. the same walk in negative territory ---\n");
    {
        pffOpen(m);
        const uint32_t legTicks = pffTicks(PFF_LEG);
        const int NEG_LEGS = 5;
        int64_t expected = 0;
        int64_t negDelta = 0;
        int64_t worstDrift = 0;
        int64_t below31 = 0, below32 = 0;
        bool allOk = true;
        int completed = 0;

        for (int i = 0; i < NEG_LEGS; i++) {
            bool ok = false;
            int64_t d = tfhMove(m, (int32_t)(-PFF_LEG), legTicks, &ok);
            if (!ok) { allOk = false; printf("   negative leg %d failed (fatal=%u)\n",
                                             i + 1, (unsigned)tfhFatal(m)); break; }
            if (i == 0) negDelta = d;
            completed++;
            expected -= PFF_LEG;
            int64_t pos = m->getPositionRaw();
            int64_t drift = pos - expected;
            if (llabs((long long)drift) > llabs((long long)worstDrift)) worstDrift = drift;
            if (i == 2) below31 = pos;      // leg 3: ~-3.22e9, a full 2^30 below -2^31
            if (i == 4) below32 = pos;      // leg 5: ~-5.37e9, a full 2^30 below -2^32
            printf("   after negative leg %d: position %lld (%.2f rotations), drift %lld\n",
                   i + 1, (long long)pos, (double)pos / (double)TFH_CPR, (long long)drift);
        }

        printf("   negative leg delivered %lld; positive leg delivered %lld\n",
               (long long)negDelta, (long long)positiveLegDelta);

        TEST_RESULT("all five negative legs complete", allOk && completed == NEG_LEGS);
        TEST_RESULT("the position passes -2^31 counts and is still reported negative",
                    below31 < -PFF_POW2_31 && below31 < 0);
        TEST_RESULT("the position passes -2^32 counts, so the borrow out of the mid word "
                    "into the high word has run",
                    below32 < -PFF_POW2_32);
        TEST_RESULT("the reported position tracks the exact running sum in negative "
                    "territory too",
                    allOk && llabs((long long)worstDrift) <= PFF_DRIFT_BUDGET);
        TEST_RESULT("a negative leg covers exactly the same distance as a positive one",
                    completed > 0 && positiveLegDelta > 0 &&
                    llabs((long long)(-negDelta - positiveLegDelta)) <= 2);

        int64_t beforeZero = m->getPositionRaw();
        m->zeroPosition();
        bool zeroAccepted = (m->getError() == 0);
        delay(200);
        int64_t afterZero = m->getPositionRaw();
        printf("   zeroPosition from %lld counts -> %lld\n",
               (long long)beforeZero, (long long)afterZero);
        TEST_RESULT("zeroPosition returns to exactly zero from negative territory too",
                    zeroAccepted && tfhFatal(m) == 0 && afterZero == 0);

        tfhReset(m);
    }

    // =====================================================================
    // 5. THE FENCE IS 64-BIT WHERE THE ABSOLUTE COMMAND IS 32-BIT.
    //
    //    Section 2 showed a far fence ACCEPTING a move. The other half of the
    //    contract is that it still REFUSES one, out where both the fence bounds
    //    and the predicted position need more than 32 bits. The queue-time
    //    check is a plain int64 comparison (motor_control.c:1812):
    //
    //        if((predicted_final_position_i64 < position_lower_safety_limit_i64) ||
    //           (predicted_final_position_i64 > position_upper_safety_limit_i64))
    //            fatal_error(ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE);   // 27
    //
    //    A fence pinned at about 3.2 billion counts cannot be expressed in the
    //    int32 pair the firmware used to keep (still visible commented out at
    //    motor_control.c:190-191), so this is the configuration in which the
    //    widening actually earns its keep. The refusal is at queue time, before
    //    anything moves, which is what makes a fence a fence rather than a
    //    report.
    // =====================================================================
    printf("\n--- 5. a fence beyond int32 still refuses a departure ---\n");
    {
        pffOpen(m);
        const uint32_t legTicks = pffTicks(PFF_LEG);
        bool walked = true;
        for (int i = 0; i < 3 && walked; i++) {
            bool ok = false;
            tfhMove(m, (int32_t)PFF_LEG, legTicks, &ok);
            if (!ok) walked = false;
        }
        int64_t here = walked ? m->getPositionRaw() : 0;
        int64_t lower = here - 2 * TFH_CPR;
        int64_t upper = here + 2 * TFH_CPR;
        m->setSafetyLimitsRaw(lower, upper);
        bool fenceSet = (m->getError() == 0) && (tfhFatal(m) == 0);
        printf("   parked at %lld; fence [%lld, %lld], both outside int32\n",
               (long long)here, (long long)lower, (long long)upper);
        TEST_RESULT("a fence whose bounds do not fit in int32 is accepted far from the "
                    "origin",
                    walked && fenceSet && lower > 2147483647LL);

        bool okInside = false;
        int64_t movedInside = tfhMove(m, (int32_t)TFH_CPR, pffTicks(TFH_CPR), &okInside);
        printf("   one rotation inside the fence: delivered %lld, fatal=%u\n",
               (long long)movedInside, (unsigned)tfhFatal(m));
        TEST_RESULT("a move that stays inside it completes",
                    okInside && llabs((long long)(movedInside - TFH_CPR)) <= 16);

        // Ends about four rotations above `here`, two past the upper bound.
        m->trapezoidMoveRaw((int32_t)(3 * TFH_CPR), pffTicks(3 * TFH_CPR));
        delay(300);
        uint8_t fenceFatal = tfhFatal(m);
        printf("   a move ending outside the fence: fatal=%u (expect %u)\n",
               (unsigned)fenceFatal,
               (unsigned)PFF_ERR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE);
        TEST_RESULT("a move that would leave it raises "
                    "ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE (27) at queue time",
                    fenceFatal == PFF_ERR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE);

        tfhReset(m);
        TEST_RESULT("a reset clears the fence fault", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 6. STILL AN ORDINARY MACHINE AFTERWARDS. About fourteen thousand rotations
    //    of commanded travel and two deliberate faults later (ERROR_MOVE_TOO_FAR
    //    in section 3, ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE in section 5),
    //    the reset that every other module in the suite depends on must still
    //    produce a plain working motor sitting at the origin.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        pffOpen(m);
        int64_t after = m->getPositionRaw();
        printf("   position after reset: %lld\n", (long long)after);
        TEST_RESULT("a reset returns the commanded position to zero", after == 0);

        bool ok = false;
        int32_t quarter = (int32_t)(TFH_CPR / 4);
        int64_t moved = tfhMove(m, quarter, pffTicks(quarter), &ok);
        printf("   quarter-turn move: delivered %lld of %ld\n",
               (long long)moved, (long)quarter);
        TEST_RESULT("an ordinary quarter-turn move still works after the far-field sweep",
                    ok && llabs((long long)(moved - quarter)) <= 16);
    }

    tfhReset(m);
    printf("\n   module elapsed: %.1f s\n", (double)(millis() - moduleStart) / 1000.0);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
