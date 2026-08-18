#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_goto_absolute_semantics.cpp
//
// "GO TO POSITION" IS AN ABSOLUTE-FRAME COMMAND. THAT IS THE WHOLE PROPERTY.
//
// Every other move the firmware accepts is RELATIVE: a trapezoid move, a
// velocity segment and an acceleration segment all say "add this much to
// wherever you are". "Go to position" (cmd 4) is the only one that names a
// point in a coordinate frame. Firmware ground truth
// (motor_control.c:2062 add_go_to_position_to_queue):
//
//     position_after_last_queue_item_i64 = position_after_last_queue_item_i96.position;
//     if(move_time == 0) fatal_error(ERROR_PARAMETER_OUT_OF_RANGE);
//     int64_t total_displacement = absolute_position - position_after_last_queue_item_i64;
//     if((total_displacement > INT32_MAX) || (total_displacement < INT32_MIN))
//         fatal_error(ERROR_MOVE_TOO_FAR);                       // code 46
//     compute_trapezoid_move((int32_t)total_displacement, move_time, ...);
//     queue_trapezoid_segments((int32_t)total_displacement, move_time, ...);
//
// Three consequences follow from that one subtraction, and this module asserts
// all of them:
//
//   * IDEMPOTENCE. The target is a place, not a distance. Commanding the same
//     target again must displace nothing, because the subtraction yields ~0.
//     A relative move repeated five times travels five times as far; an
//     absolute one must not. This is the single easiest thing for a caller to
//     get wrong and the single most valuable thing to guarantee.
//   * THE BASELINE IS THE END OF THE QUEUE, NOT THE LIVE SHAFT. The
//     subtraction is against position_after_last_queue_item_i96 -- the
//     planner's PREDICTED end-of-queue position -- so a go-to issued while
//     other moves are still pending still lands on its target, and can even
//     undo a move that has not finished executing yet.
//   * THE FRAME IS OWNED BY zeroPosition. zero_position()
//     (motor_control.c:3311) zeroes BOTH current_position_i96 (lines 3319-3321)
//     and position_after_last_queue_item_i96 (lines 3323-3325). So it does not
//     merely relabel the reported number: it moves the origin that every future
//     absolute target is measured from. "Go to 0" after a zero returns to where
//     zeroPosition was called, not to where the machine booted.
//
// WHAT THIS IS DISTINCT FROM.
//   * tm_move_composition section 3 checks only that goToPosition(p0 + d)
//     agrees with trapezoidMove(d) for four small d, all within 0.125 rotation
//     of the origin. It never repeats a target, never leaves the origin's
//     neighbourhood, and never touches zeroPosition. It establishes that the
//     two commands share a planner, not that one of them is absolute.
//   * tm_goto_edges probes the same command with the MOSFETs ENERGISED at
//     0.25 rotation and in four unit systems. This module runs with the output
//     stage OFF, which is what makes the large-displacement and far-from-origin
//     cases possible at all, and it asserts the frame properties that module
//     does not reach: both-directions convergence, negative targets, the
//     zeroPosition re-basing, arithmetic 16 rotations out, and the int32 reach
//     limit (ERROR_MOVE_TOO_FAR, 46 -- asserted nowhere else in the suite).
//   * tm_position_frames covers zeroPosition itself (including its
//     queue-not-empty guard) on a closed-loop motor. It never asks what
//     zeroPosition does to a subsequent ABSOLUTE target, which is section 5
//     here.
//
// SAFETY. The MOSFETs are OFF for the entire module and are never enabled;
// goToClosedLoop() is never called. Every measurement reads the COMMANDED
// position (getPositionRaw), which the motion planner advances whether or not
// the output stage is energised, so no shaft turns anywhere on the shared
// 35-motor rack. Nothing is broadcast and no alias is ever written. No
// testMode() value is ever sent. Because the rotor never follows, the
// commanded-vs-hall deviation grows with every commanded count and the default
// two-rotation window would trip error 45, so every subsection starts from
// tfhFreshOpen() which raises that window (and the speed/acceleration
// ceilings, so the test measures its subject rather than a limit). Queue waits
// are always tfhDrain() with a bounded deadline -- never a computed duration,
// which reads a partial move and looks exactly like a planner bug. Every
// subsection ends with tfhReset(m), and a reset restores all of those
// defaults.
// ---------------------------------------------------------------------------

static const uint8_t GA_ERR_MOVE_TOO_FAR = 46;   // error_codes.json "move too far"

// Tolerances in encoder counts. The commanded position is a planner integer, so
// the only error is the planner's rounding: measured elsewhere in this campaign
// as 1 count in 4,096,000 and 7 counts in 2,147,483,647. 64 counts is roughly
// 0.00002 of a rotation -- five orders of magnitude below every target used
// here, so it cannot hide a real frame error.
static const int64_t GA_LAND_TOL  = 64;   // landed on an absolute target
static const int64_t GA_STAY_TOL  = 64;   // a no-op must not have moved
static const int64_t GA_AGREE_TOL = 64;   // two routes to one target must agree

static int64_t gaAbs64(int64_t v) { return (v < 0) ? -v : v; }
static bool gaNear(int64_t a, int64_t b, int64_t tol) { return gaAbs64(a - b) <= tol; }

// ---------------------------------------------------------------------------
// Issue ONE absolute go-to-position, wait for the queue to drain, and return
// the commanded position afterwards.
//
// The wire field is int32 (main.c:277 `int32_t end_position`), so the target is
// cast down here deliberately -- every target in this module is well inside
// int32 except the one in section 7, which is INT32_MAX on purpose.
// ---------------------------------------------------------------------------
static int64_t gaGoto(Servomotor* m, int64_t target, uint32_t ticks, bool* ok) {
    *ok = false;
    m->goToPositionRaw((int32_t)target, ticks);
    if (m->getError() != 0) {
        printf("   goto %lld: refused at the bus layer (err %d)\n",
               (long long)target, m->getError());
        return 0;
    }
    // Bounded deadline derived from the commanded duration plus generous slack.
    // The DRAIN is what decides completion; the number below is only a ceiling.
    uint32_t budget = (uint32_t)((uint64_t)ticks * 1000ULL / TFH_TPS) + 10000;
    if (!tfhDrain(m, budget)) {
        printf("   goto %lld: queue did not drain within %lu ms\n",
               (long long)target, (unsigned long)budget);
        return 0;
    }
    uint8_t f = tfhFatal(m);
    if (f != 0) {
        printf("   goto %lld: fatal error %u\n", (long long)target, (unsigned)f);
        return 0;
    }
    *ok = true;
    return m->getPositionRaw();
}

void tm_goto_absolute_semantics(void) {
    Serial.println("tm_goto_absolute_semantics: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 0. PREFLIGHT. State the premise the whole module rests on: the output
    //    stage is OFF, so every number below is the planner's commanded
    //    position and no shaft is being driven. A reset leaves the MOSFETs
    //    disabled; disabling them explicitly makes the intent unmistakable to
    //    anyone reading a log, and the status flag proves it rather than
    //    assuming it (STATUS_MOSFETS_ENABLED_FLAG_BIT = 1, mask 0x02,
    //    common_source_files/device_status.h).
    // =====================================================================
    printf("\n--- 0. preflight: the output stage is off ---\n");
    {
        tfhFreshOpen(m);
        m->disableMosfets();
        delay(200);
        getStatusResponse st = m->getStatus();
        bool read = (m->getError() == 0);
        printf("   status flags 0x%04X, fatal %u\n",
               read ? (unsigned)st.statusFlags : 0xFFFFu,
               read ? (unsigned)st.fatalErrorCode : 0xFFu);
        TEST_RESULT("the MOSFETs are off and no fault is latched before the frame tests",
                    read && ((st.statusFlags & 0x02) == 0) && (st.fatalErrorCode == 0));
    }

    // =====================================================================
    // 1. IDEMPOTENCE. The defining property. Command one absolute target five
    //    times in a row: the first must travel to it, the other four must
    //    travel nothing, because total_displacement = target - (where the
    //    planner already is) is ~0 by then.
    //
    //    The failure this catches is the one that matters commercially: a
    //    firmware or library change that treats the payload as a DISPLACEMENT
    //    would sail through every single-move test in the suite and quietly
    //    send a machine five times too far the first time a caller re-issued a
    //    setpoint. So the "did not add up to five displacements" claim is
    //    asserted explicitly, not merely implied by the tolerance.
    //
    //    Run at a positive and a large negative target so the property is not
    //    an accident of sign or of magnitude.
    // =====================================================================
    printf("\n--- 1. the same absolute target commanded five times displaces once ---\n");
    {
        const int64_t targets[] = { TFH_CPR * 3 / 4, -TFH_CPR * 4 };
        const char*   labels[]  = { "+0.75 rotation", "-4 rotations" };
        for (unsigned i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
            const int64_t T = targets[i];
            tfhFreshOpen(m);
            const uint32_t ticks = tfhTicksFor(T);

            bool ok1 = false;
            int64_t first = gaGoto(m, T, ticks, &ok1);

            bool allOk = ok1;
            int64_t after = first;
            int64_t worstStep = 0;
            for (int r = 0; r < 4; r++) {
                bool okR = false;
                int64_t before = after;
                after = gaGoto(m, T, ticks, &okR);
                if (!okR) { allOk = false; break; }
                int64_t step = gaAbs64(after - before);
                if (step > worstStep) worstStep = step;
            }

            printf("   %s: first landed %lld (target %lld), after 4 repeats %lld, "
                   "worst repeat step %lld, five displacements would be %lld\n",
                   labels[i], (long long)first, (long long)T, (long long)after,
                   (long long)worstStep, (long long)(T * 5));

            TEST_RESULT(std::string("the first go-to lands on the absolute target ") + labels[i],
                        ok1 && gaNear(first, T, GA_LAND_TOL));
            TEST_RESULT(std::string("four more go-tos to ") + labels[i] +
                            " leave the position exactly where it was",
                        allOk && gaNear(after, first, GA_STAY_TOL));
            // The alternative hypothesis, rejected out loud: if the target were
            // read as a displacement the machine would be at 5T by now.
            TEST_RESULT(std::string("five go-tos to ") + labels[i] +
                            " do not add up to five displacements",
                        allOk && gaAbs64(after - T * 5) >= gaAbs64(T) * 3);
            tfhReset(m);
        }
    }

    // =====================================================================
    // 2. A GO-TO TO THE CURRENT POSITION IS A LEGAL DWELL, NOT AN ERROR AND
    //    NOT A DISCARDED COMMAND. total_displacement is exactly 0 (the queue
    //    is drained, so the reported position and the planner's end-of-queue
    //    baseline are identical -- handle_queued_movements enforces that with
    //    ERROR_POSITION_DISCREPANCY, motor_control.c:2152). The planner then
    //    computes acceleration 0 and queues three real segments that occupy
    //    the commanded time. That is a genuine, useful dwell primitive, so the
    //    queue occupancy is asserted as well as the stillness -- a command
    //    that was silently dropped would also "not move".
    // =====================================================================
    printf("\n--- 2. a go-to to the current position is a dwell that occupies the queue ---\n");
    {
        const int64_t places[] = { 0, -TFH_CPR };
        const char*   labels[] = { "at the origin", "one rotation below the origin" };
        const uint32_t dwellTicks = (uint32_t)(TFH_TPS / 2);   // 0.5 s
        uint8_t depthSeen = 0;

        for (unsigned i = 0; i < sizeof(places) / sizeof(places[0]); i++) {
            tfhFreshOpen(m);
            int64_t here = 0;
            if (places[i] != 0) {
                bool okMove = false;
                here = gaGoto(m, places[i], tfhTicksFor(places[i]), &okMove);
                if (!okMove) here = m->getPositionRaw();
            } else {
                here = m->getPositionRaw();
            }

            // Target the exact current commanded position: displacement 0.
            m->goToPositionRaw((int32_t)here, dwellTicks);
            bool accepted = (m->getError() == 0);
            uint8_t depth = m->getNQueuedItems();
            if (m->getError() == 0 && depth > depthSeen) depthSeen = depth;
            bool drained = accepted && tfhDrain(m, 6000);
            uint8_t fat = tfhFatal(m);
            int64_t afterDwell = m->getPositionRaw();

            printf("   %s: dwell from %lld -> %lld, accepted=%d depth=%u fatal=%u\n",
                   labels[i], (long long)here, (long long)afterDwell,
                   (int)accepted, (unsigned)depth, (unsigned)fat);

            if (i == 0) {
                TEST_RESULT("a zero-distance go-to is accepted and completes without a fault",
                            accepted && drained && fat == 0);
            }
            TEST_RESULT(std::string("a go-to to the current position ") + labels[i] +
                            " does not move the machine",
                        drained && fat == 0 && gaNear(afterDwell, here, GA_STAY_TOL));
            tfhReset(m);
        }
        printf("   deepest queue seen during a dwell: %u items\n", (unsigned)depthSeen);
        TEST_RESULT("the dwell really occupies motion-queue slots rather than being discarded",
                    depthSeen > 0);
    }

    // =====================================================================
    // 3. NEGATIVE ABSOLUTE TARGETS. The frame is signed and unbounded in both
    //    directions: absolute_position is an int32 on the wire, sign-extended
    //    into an int64 subtraction, and the commanded position accumulator is
    //    a signed int96. A target below the origin must therefore land on a
    //    NEGATIVE reported position -- "go to -3 rotations" is a place, not a
    //    3-rotation journey backwards.
    //
    //    The last check crosses zero, where a sign bug in the subtraction
    //    would show up as a displacement of |target| instead of |target-start|.
    // =====================================================================
    printf("\n--- 3. negative absolute targets ---\n");
    {
        const int64_t targets[] = { -TFH_CPR / 2, -TFH_CPR * 3, -TFH_CPR * 7 };
        const char*   labels[]  = { "-0.5 rotations", "-3 rotations", "-7 rotations" };
        for (unsigned i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
            tfhFreshOpen(m);
            bool ok = false;
            int64_t landed = gaGoto(m, targets[i], tfhTicksFor(targets[i]), &ok);
            printf("   %s: landed %lld (target %lld)\n",
                   labels[i], (long long)landed, (long long)targets[i]);
            TEST_RESULT(std::string("a target of ") + labels[i] +
                            " lands on a negative absolute position",
                        ok && gaNear(landed, targets[i], GA_LAND_TOL) && landed < 0);
            tfhReset(m);
        }

        // Cross zero: +2 rotations, then -2 rotations. The second leg must
        // travel the SIGNED DIFFERENCE (4 rotations), not the target's
        // magnitude (2 rotations).
        tfhFreshOpen(m);
        const int64_t up = TFH_CPR * 2, down = -TFH_CPR * 2;
        bool okUp = false, okDown = false;
        int64_t atUp = gaGoto(m, up, tfhTicksFor(up), &okUp);
        int64_t atDown = gaGoto(m, down, tfhTicksFor(up * 2), &okDown);
        int64_t leg = atDown - atUp;
        printf("   crossing zero: %lld -> %lld, second leg travelled %lld "
               "(signed difference %lld, target magnitude %lld)\n",
               (long long)atUp, (long long)atDown, (long long)leg,
               (long long)(down - up), (long long)down);
        TEST_RESULT("a target on the far side of zero travels the signed difference, "
                    "not the target's magnitude",
                    okUp && okDown && gaNear(atDown, down, GA_LAND_TOL) &&
                    gaNear(leg, down - up, GA_LAND_TOL));
        tfhReset(m);
    }

    // =====================================================================
    // 4. A TARGET IS A PLACE, SO IT IS PATH-INDEPENDENT. Approach the same
    //    absolute target from below and from above; both must land on the same
    //    point, to within the planner's rounding. Then reach it through six
    //    intermediate hops and land there too.
    //
    //    This is the property a caller depends on when it homes a machine from
    //    an arbitrary state: a residue that depended on the approach direction
    //    (a rounding rule that always rounds toward zero, say, or an asymmetric
    //    ramp) would make repeatability direction-dependent and would be
    //    invisible to any test that only ever approaches from one side.
    // =====================================================================
    printf("\n--- 4. a target lands identically from either direction ---\n");
    {
        const int64_t targets[] = { TFH_CPR * 2, -TFH_CPR * 2 };
        const char*   labels[]  = { "+2 rotations", "-2 rotations" };
        int64_t fromBelowRef = 0;
        bool fromBelowRefOk = false;

        for (unsigned i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
            const int64_t T = targets[i];
            const int64_t approach = TFH_CPR / 2;    // start half a rotation away

            tfhFreshOpen(m);
            bool okA = false, okB = false;
            gaGoto(m, T - approach, tfhTicksFor(T - approach), &okA);
            int64_t below = gaGoto(m, T, tfhTicksFor(approach), &okB);
            bool belowOk = okA && okB;
            tfhReset(m);

            tfhFreshOpen(m);
            bool okC = false, okD = false;
            gaGoto(m, T + approach, tfhTicksFor(T + approach), &okC);
            int64_t above = gaGoto(m, T, tfhTicksFor(approach), &okD);
            bool aboveOk = okC && okD;
            tfhReset(m);

            printf("   %s: from below %lld, from above %lld, difference %lld\n",
                   labels[i], (long long)below, (long long)above,
                   (long long)(below - above));

            TEST_RESULT(std::string("a target of ") + labels[i] +
                            " approached from below lands on it",
                        belowOk && gaNear(below, T, GA_LAND_TOL));
            TEST_RESULT(std::string("a target of ") + labels[i] +
                            " approached from above lands in the same place",
                        aboveOk && gaNear(above, below, GA_AGREE_TOL) &&
                        gaNear(above, T, GA_LAND_TOL));

            if (i == 0) { fromBelowRef = below; fromBelowRefOk = belowOk; }
        }

        // Six hops that wander either side of the target, ending on it.
        const int64_t T = targets[0];
        const int64_t hops[6] = { TFH_CPR / 4, TFH_CPR * 3, -TFH_CPR / 2,
                                  TFH_CPR * 5 / 2, 0, T };
        tfhFreshOpen(m);
        bool hopsOk = true;
        int64_t viaHops = 0;
        for (int h = 0; h < 6 && hopsOk; h++) {
            bool okH = false;
            // Duration sized for the WORST hop in the list, never a computed
            // guess at the actual one: an under-timed hop is refused outright
            // (limits are pass/fail checks, not clamps), it is never slowed.
            viaHops = gaGoto(m, hops[h], tfhTicksFor(TFH_CPR * 4), &okH);
            if (!okH) hopsOk = false;
        }
        printf("   six hops ended at %lld; the direct route ended at %lld\n",
               (long long)viaHops, (long long)fromBelowRef);
        TEST_RESULT("a target reached through six intermediate hops lands where the "
                    "direct route landed",
                    hopsOk && fromBelowRefOk && gaNear(viaHops, fromBelowRef, GA_AGREE_TOL) &&
                    gaNear(viaHops, T, GA_LAND_TOL));
        tfhReset(m);
    }

    // =====================================================================
    // 5. zeroPosition REDEFINES THE FRAME THAT ABSOLUTE TARGETS ARE MEASURED
    //    IN. This is the deepest claim in the module and the one with a
    //    genuinely falsifiable alternative.
    //
    //    zero_position() (motor_control.c:3311) zeroes current_position_i96
    //    (3319-3321) AND position_after_last_queue_item_i96 (3323-3325). If it
    //    zeroed only the first -- i.e. only the number that "get position"
    //    reports -- then a later "go to 0" would compute
    //    0 - (stale baseline) and travel all the way back to the BOOT origin
    //    instead of to the point where zeroPosition was called. Both outcomes
    //    leave the reported position reading 0, so the reported position alone
    //    cannot tell them apart. The DISPLACEMENT of the returning leg can, and
    //    that is what is measured here.
    //
    //    Sequence: travel A, zero (new origin), travel B, then go to 0.
    //    Correct behaviour: the returning leg is -B.
    //    Stale-baseline behaviour: the returning leg would be -(A+B).
    //
    //    zeroPosition is only legal on an empty queue -- otherwise fatal
    //    error 8, ERROR_QUEUE_NOT_EMPTY (motor_control.c:3313). Every zero
    //    below therefore follows a completed drain. (That guard itself is
    //    already covered by tm_position_frames and is not re-tested here.)
    // =====================================================================
    printf("\n--- 5. zeroPosition moves the origin that absolute targets are measured from ---\n");
    {
        const int64_t A = TFH_CPR * 3 / 2;    // 1.5 rotations before the zero
        const int64_t B = TFH_CPR * 3 / 4;    // 0.75 rotations after the zero

        tfhFreshOpen(m);
        bool okA = false;
        gaGoto(m, A, tfhTicksFor(A), &okA);

        m->zeroPosition();                    // queue is drained: legal
        bool zeroAccepted = (m->getError() == 0);
        delay(200);
        uint8_t zeroFatal = tfhFatal(m);
        int64_t afterZero = m->getPositionRaw();
        printf("   travelled %lld, zeroed: reported position now %lld, fatal %u\n",
               (long long)A, (long long)afterZero, (unsigned)zeroFatal);
        TEST_RESULT("zeroPosition on a drained queue reports a commanded position of zero",
                    okA && zeroAccepted && zeroFatal == 0 && gaNear(afterZero, 0, GA_STAY_TOL));

        // Immediately asking for 0 in the new frame must be a no-op. If the
        // planner's baseline had NOT been re-based this would already move.
        bool okNoop = false;
        int64_t noop = gaGoto(m, 0, (uint32_t)(TFH_TPS / 2), &okNoop);
        printf("   go-to 0 straight after the zero moved %lld counts\n",
               (long long)(noop - afterZero));
        TEST_RESULT("a go-to 0 immediately after zeroing does not move",
                    okNoop && gaNear(noop, afterZero, GA_STAY_TOL));

        // Travel B in the new frame, then return to the new zero.
        bool okB = false, okBack = false;
        int64_t atB = gaGoto(m, B, tfhTicksFor(B), &okB);
        int64_t legStart = atB;
        int64_t back = gaGoto(m, 0, tfhTicksFor(B), &okBack);
        int64_t leg = back - legStart;
        printf("   returning leg travelled %lld; -B is %lld, -(A+B) would be %lld\n",
               (long long)leg, (long long)(-B), (long long)(-(A + B)));
        TEST_RESULT("a go-to 0 returns to the point where zeroPosition was called",
                    okB && okBack && gaNear(leg, -B, GA_LAND_TOL));
        // The falsifiable alternative, rejected explicitly.
        TEST_RESULT("a go-to 0 does NOT return to the pre-zero origin",
                    okB && okBack && gaAbs64(leg - (-(A + B))) > A / 2);

        // A second zero re-bases again, from a different place.
        bool okC = false;
        gaGoto(m, TFH_CPR / 2, tfhTicksFor(TFH_CPR / 2), &okC);
        m->zeroPosition();
        delay(200);
        uint8_t zero2Fatal = tfhFatal(m);
        int64_t afterZero2 = m->getPositionRaw();
        printf("   second zero: reported position %lld, fatal %u\n",
               (long long)afterZero2, (unsigned)zero2Fatal);
        TEST_RESULT("a second zeroPosition re-bases the frame again",
                    okC && zero2Fatal == 0 && gaNear(afterZero2, 0, GA_STAY_TOL));

        // And an ordinary absolute target is now measured from that new origin.
        const int64_t T2 = TFH_CPR / 4;
        bool okT2 = false;
        int64_t landedT2 = gaGoto(m, T2, tfhTicksFor(T2), &okT2);
        printf("   after the second zero, go-to %lld landed at %lld\n",
               (long long)T2, (long long)landedT2);
        TEST_RESULT("an absolute target after zeroing is measured from the new origin",
                    okT2 && gaNear(landedT2, T2, GA_LAND_TOL));
        tfhReset(m);
    }

    // =====================================================================
    // 6. THE FRAME STAYS SHARP FAR FROM THE ORIGIN. The subtraction that
    //    defines the command is done in int64 while the reported position
    //    lives in an int96 accumulator, so accuracy must not degrade with
    //    distance from zero. Sixteen rotations out is 52,428,800 counts --
    //    well past the point where a 32-bit intermediate or a float would
    //    start shedding low bits.
    //
    //    The sharpest check is the last one: a 1000-count step taken sixteen
    //    rotations from the origin. That is a difference of two numbers around
    //    5.2e7 that must come out exactly 1000. A float baseline (24-bit
    //    mantissa) could not represent either operand, and would lose the step
    //    entirely.
    //
    //    Only possible with the deviation window raised, which tfhFreshOpen
    //    does: with the output stage off the rotor never follows, so sixteen
    //    commanded rotations is sixteen rotations of deviation and the factory
    //    two-rotation window would fire error 45 long before the target.
    // =====================================================================
    printf("\n--- 6. absolute targets far from the origin ---\n");
    {
        // Near-origin reference for the accuracy comparison.
        tfhFreshOpen(m);
        const int64_t nearT = TFH_CPR / 4;
        bool okNear = false;
        int64_t nearLanded = gaGoto(m, nearT, tfhTicksFor(nearT), &okNear);
        int64_t nearErr = gaAbs64(nearLanded - nearT);
        tfhReset(m);

        tfhFreshOpen(m);
        const int64_t far8 = TFH_CPR * 8;
        bool ok8 = false;
        int64_t landed8 = gaGoto(m, far8, tfhTicksFor(far8), &ok8);
        printf("   8 rotations out: landed %lld (target %lld, error %lld)\n",
               (long long)landed8, (long long)far8, (long long)(landed8 - far8));
        TEST_RESULT("an absolute target eight rotations from the origin lands on it",
                    ok8 && gaNear(landed8, far8, GA_LAND_TOL));

        const int64_t far16 = TFH_CPR * 16;
        bool ok16 = false;
        int64_t landed16 = gaGoto(m, far16, tfhTicksFor(far16 - far8), &ok16);
        int64_t farErr = gaAbs64(landed16 - far16);
        printf("   16 rotations out: landed %lld (target %lld, error %lld); "
               "near-origin error was %lld\n",
               (long long)landed16, (long long)far16, (long long)farErr,
               (long long)nearErr);
        TEST_RESULT("an absolute target sixteen rotations from the origin lands on it",
                    ok16 && gaNear(landed16, far16, GA_LAND_TOL));
        TEST_RESULT("landing accuracy sixteen rotations out is no worse than near the origin",
                    ok16 && okNear && farErr <= nearErr + 32);

        // A 1000-count step, taken from 52,428,800 counts out.
        const int64_t stepTarget = far16 + 1000;
        bool okStep = false;
        int64_t landedStep = gaGoto(m, stepTarget, (uint32_t)(TFH_TPS / 2), &okStep);
        int64_t stepMoved = landedStep - landed16;
        printf("   a 1000-count step from 16 rotations out moved %lld counts\n",
               (long long)stepMoved);
        // NOT an exact-equality check, deliberately. compute_trapezoid_move
        // derives an INTEGER acceleration, (displacement << 24) / ((dt1+dt2)*dt1),
        // and truncates it, so a move always lands a fraction of a count short
        // (here about 0.06). The reported position is the integer part of the
        // int96 accumulator, so the measured step is 999 or 1001 depending on
        // where the carried fraction sits -- 1000 exactly is not reliably
        // attainable, and asserting it would fail on correct firmware. What IS
        // verifiable is that the step is a 1000-count step and not a lost one,
        // a doubled one, or a 5.2e7-sized re-travel from the origin, all of
        // which are orders of magnitude outside this window.
        TEST_RESULT("a 1000-count step taken sixteen rotations out displaces 1000 counts "
                    "to within the planner's rounding",
                    okStep && gaNear(stepMoved, 1000, 16));

        // Idempotence still holds out here.
        bool okRep = false;
        int64_t repeated = gaGoto(m, stepTarget, (uint32_t)(TFH_TPS / 2), &okRep);
        printf("   repeating that far target moved a further %lld counts\n",
               (long long)(repeated - landedStep));
        TEST_RESULT("repeating an absolute target sixteen rotations out is still a no-op",
                    okRep && gaNear(repeated, landedStep, GA_STAY_TOL));
        tfhReset(m);
    }

    // =====================================================================
    // 7. THE FRAME HAS A REACH LIMIT, AND IT IS A PASS/FAIL CHECK.
    //    total_displacement is computed in int64 and then explicitly bounded
    //    (motor_control.c:2076-2078): a target more than INT32_MAX counts away
    //    from the planner's baseline raises ERROR_MOVE_TOO_FAR (46) rather
    //    than being truncated into a move in some arbitrary direction. That
    //    check exists precisely because the value is about to be cast to
    //    int32 for compute_trapezoid_move, and nothing else in this suite
    //    asserts it.
    //
    //    Both sides are checked, because "refuses everything big" would also
    //    pass a one-sided test: a displacement of 2,000,000,000 counts (~610
    //    rotations, comfortably inside int32) must be ACCEPTED.
    //
    //    That accepted move is deliberately NOT drained -- at a safe speed it
    //    would run for well over a minute. Queue-time validation has already
    //    happened by the time the reply comes back, so the acceptance is
    //    established at once and the move is abandoned by the reset. This is
    //    the one place in the module that abandons a move rather than draining
    //    it, and it is safe for exactly the same reason everything else here
    //    is: the output stage is off.
    // =====================================================================
    printf("\n--- 7. targets beyond int32 reach are refused, not truncated ---\n");
    {
        const int64_t offset = -2000000;      // 0.61 rotations below the origin

        // (a) Inside the limit: displacement 2,000,000,000 counts.
        tfhFreshOpen(m);
        bool okOff = false;
        gaGoto(m, offset, tfhTicksFor(offset), &okOff);
        m->goToPositionRaw((int32_t)(offset + 2000000000LL), 3000000u);  // 96 s
        bool acceptedBig = (m->getError() == 0);
        delay(150);
        uint8_t bigFatal = tfhFatal(m);
        printf("   displacement 2,000,000,000: accepted=%d fatal=%u\n",
               (int)acceptedBig, (unsigned)bigFatal);
        TEST_RESULT("a displacement of two billion counts is inside the reach limit "
                    "and is accepted",
                    okOff && acceptedBig && bigFatal == 0);
        tfhReset(m);

        // (b) Past the limit: from the same offset, INT32_MAX is
        //     2,149,483,647 counts away -- 2,000,000 counts too far.
        tfhFreshOpen(m);
        bool okOff2 = false;
        gaGoto(m, offset, tfhTicksFor(offset), &okOff2);
        m->goToPositionRaw((int32_t)2147483647LL, 3000000u);
        delay(200);
        uint8_t tooFar = tfhFatal(m);
        printf("   displacement 2,149,483,647: fatal=%u (expect %u)\n",
               (unsigned)tooFar, (unsigned)GA_ERR_MOVE_TOO_FAR);
        TEST_RESULT("a target more than INT32_MAX counts away raises "
                    "ERROR_MOVE_TOO_FAR (46) instead of being truncated",
                    okOff2 && tooFar == GA_ERR_MOVE_TOO_FAR);

        tfhReset(m);
        TEST_RESULT("a reset clears the reach-limit fault", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 8. THE TARGET IS RESOLVED AGAINST THE END OF THE QUEUE, NOT THE LIVE
    //    SHAFT. add_go_to_position_to_queue subtracts
    //    position_after_last_queue_item_i96 -- the position the planner
    //    PREDICTS it will be at once everything already queued has run
    //    (updated at motor_control.c:1865 as each item is queued) -- not
    //    current_position_i96.
    //
    //    The second half is the sharp discriminator and is worth reading
    //    carefully. A relative move of +0.5 rotation is queued, and then,
    //    while it is still pending, a go-to is issued targeting the position
    //    the shaft is at RIGHT NOW:
    //      * resolved against the end of the queue (correct): displacement is
    //        -0.5 rotation, so the machine ends back where it started;
    //      * resolved against the live position (wrong): displacement is ~0,
    //        a dwell, and the machine ends 0.5 rotation away.
    //    The two hypotheses predict endpoints half a rotation apart, so this
    //    cannot be satisfied by a lucky tolerance.
    // =====================================================================
    printf("\n--- 8. absolute targets resolve against the end of the queue ---\n");
    {
        // Three absolute targets queued back to back without draining.
        tfhFreshOpen(m);
        const int64_t T1 = TFH_CPR * 2 / 5;
        const int64_t T2 = TFH_CPR * 6 / 5;
        const int64_t T3 = TFH_CPR * 3 / 5;
        const uint32_t legTicks = (uint32_t)(TFH_TPS * 3 / 5);   // 0.6 s each

        int64_t start = m->getPositionRaw();
        m->goToPositionRaw((int32_t)T1, legTicks);
        bool a1 = (m->getError() == 0);
        m->goToPositionRaw((int32_t)T2, legTicks);
        bool a2 = (m->getError() == 0);
        m->goToPositionRaw((int32_t)T3, legTicks);
        bool a3 = (m->getError() == 0);
        uint8_t depth = m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);
        bool drained = a1 && a2 && a3 && tfhDrain(m, 12000);
        uint8_t fat = tfhFatal(m);
        int64_t landed = m->getPositionRaw();

        printf("   three queued targets from %lld: depth after sending %u, ended at %lld "
               "(last target %lld, sum would be %lld)\n",
               (long long)start, (unsigned)depth, (long long)landed, (long long)T3,
               (long long)(T1 + T2 + T3));
        TEST_RESULT("three absolute targets queued back to back are all accepted",
                    a1 && a2 && a3 && drained && fat == 0);
        // Each go-to costs 3 slots (accelerate / coast / decelerate, verified in
        // tm_queue_accounting), so 9 were queued. The threshold is NOT 9: the
        // first slot is the delta_t1 ramp, which at these limits is 93 ticks =
        // 3 ms, and the three sends plus this read take longer than that, so the
        // reading is normally 8. Demanding 9 would be a race that fails on a
        // healthy machine. 6 is what is actually verifiable -- at most one move's
        // worth of slots can have drained, so two complete go-tos were pending
        // simultaneously, which is all this assertion needs to establish that the
        // targets were resolved against a NON-EMPTY queue.
        TEST_RESULT("at least two full go-tos' worth of segments were pending at once, "
                    "so the later targets were planned against a non-empty queue",
                    depthOk && depth >= 6);
        TEST_RESULT("the machine ends on the LAST absolute target, not the sum of the three",
                    drained && fat == 0 && gaNear(landed, T3, GA_LAND_TOL) &&
                    gaAbs64(landed - (T1 + T2 + T3)) > TFH_CPR / 4);
        tfhReset(m);

        // The discriminator: a go-to that must undo a move still in the queue.
        tfhFreshOpen(m);
        const int64_t half = TFH_CPR / 2;
        int64_t here = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)half, (uint32_t)TFH_TPS);   // RELATIVE, 1 s
        bool relOk = (m->getError() == 0);
        // Issued immediately: the relative move is still queued and unexecuted.
        m->goToPositionRaw((int32_t)here, (uint32_t)TFH_TPS);
        bool gotoOk = (m->getError() == 0);
        bool bothDrained = relOk && gotoOk && tfhDrain(m, 12000);
        uint8_t fat2 = tfhFatal(m);
        int64_t ended = m->getPositionRaw();

        printf("   go-to behind a pending +%lld move: started %lld, ended %lld "
               "(end-of-queue prediction says %lld, live-position reading would say %lld)\n",
               (long long)half, (long long)here, (long long)ended,
               (long long)here, (long long)(here + half));
        TEST_RESULT("a go-to issued behind a pending relative move undoes it, because the "
                    "target is resolved against the end of the queue",
                    bothDrained && fat2 == 0 && gaNear(ended, here, GA_LAND_TOL));
        TEST_RESULT("the go-to did not merely dwell where the shaft was when it arrived",
                    bothDrained && gaAbs64(ended - (here + half)) > TFH_CPR / 4);
        tfhReset(m);
    }

    // =====================================================================
    // 9. Leave the machine exactly as it was found: reset, no fault, default
    //    limits and deviation window restored, MOSFETs off.
    // =====================================================================
    tfhReset(m);
    uint8_t finalFatal = tfhFatal(m);
    int64_t finalPos = m->getPositionRaw();
    printf("\n   final state: fatal %u, commanded position %lld\n",
           (unsigned)finalFatal, (long long)finalPos);
    TEST_RESULT("the motor is left clean, at rest and free of faults",
                finalFatal == 0 && gaNear(finalPos, 0, GA_STAY_TOL));
}
