#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_homing_arithmetic.cpp
//
// THE TWO NUMBERS YOU HAND TO 'Homing' (command 14), AND THE ORDER IN WHICH
// THE FIRMWARE JUDGES THEM.
//
// Homing takes exactly two parameters -- maxDistance (int32, signed: the sign
// picks the direction) and maxDuration (uint32) -- and the interesting thing
// about them is that they are NOT special. The firmware dispatch is three
// lines long:
//
//     main.c:428   case HOMING_COMMAND:
//     main.c:439       start_homing(homing_input.max_homing_travel_displacement,
//                                   homing_input.max_homing_time);
//
// and start_homing (motor_control.c:1434) does only this:
//
//     motor_control.c:1436-1438   if(motor_control_mode != CLOSED_LOOP_POSITION_CONTROL)
//                                     fatal_error(ERROR_NOT_IN_CLOSED_LOOP);   // 13
//     motor_control.c:1440-1442   if(n_items_in_queue != 0)
//                                     fatal_error(ERROR_QUEUE_NOT_EMPTY);      // 8
//     motor_control.c:1444        add_trapezoid_move_to_queue(max_homing_displacement,
//                                                             max_homing_time);
//
// That last line is the whole point of this module. `add_trapezoid_move_to_queue`
// is the SAME function, with the SAME (int32 displacement, uint32 time)
// signature, that the ordinary 'Trapezoid move' command (command 2) calls at
// main.c:261. The firmware says so in its own words at motor_control.c:1985-1989:
// "Shared by 'Trapezoid move' and 'Go to position' (and therefore by 'Homing',
// which goes through add_trapezoid_move_to_queue) so the shape is decided in
// exactly one place."
//
// So homing has no arithmetic of its own. Its (maxDistance, maxDuration) pair
// is governed by exactly the laws that govern any other move:
//
//     * zero duration is refused outright         (motor_control.c:2051-2053, code 34)
//     * the ramp/coast shape is computed          (motor_control.c:2055 -> :445)
//     * an acceleration too big to represent SATURATES rather than truncating
//       (motor_control.c:510-515) so it is caught, not silently mangled
//     * and the segments are then validated against the configured ceilings:
//         acceleration     motor_control.c:1795-1797  -> code 15
//         predicted speed  motor_control.c:1802-1804  -> code 28
//         segment speed    motor_control.c:1846-1848  -> code 16
//
// Limits are PASS/FAIL CHECKS, never clamps: an over-ambitious distance/duration
// pair is REFUSED, not stretched to fit. That is the property this module
// asserts for homing's parameters.
//
// WHAT MAKES THIS DISTINCT FROM THE TWO EXISTING HOMING MODULES
//   * tm_homing.cpp drives REAL homing moves in closed loop with the MOSFETs
//     energised, across four position units and both directions, and measures
//     the physical outcome. It is about homing as a motion.
//   * tm_homing_edges.cpp covers three preconditions/boundaries -- no closed
//     loop, zero duration in closed loop, and one tiny homing that completes --
//     and it too needs the output stage on. (Its header labels homing "cmd 30";
//     the correct number is 14. Command 30 is 'Set safety limits'. Verified in
//     python_programs/servomotor/motor_commands.json. Not edited here.)
//   * This module never energises anything and never enters closed loop. It
//     asserts the PARAMETER contract and the GUARD ORDERING: that the
//     closed-loop precondition is evaluated first and unconditionally, so no
//     value of either parameter -- zero, negative, INT32_MIN, UINT32_MAX, or an
//     arithmetically impossible pair -- can change the answer; and that the law
//     those parameters would then face is the shared planner's law, measured on
//     the delegate path the firmware itself routes homing through.
//
// The delegate measurement is not a substitute for testing homing -- it is the
// only way to reach homing's arithmetic at all without the output stage on,
// because the closed-loop guard sits in front of it. The two halves together
// say something neither says alone: the guard cannot be bypassed by any
// parameter, and the arithmetic behind it is the arithmetic already
// characterised for ordinary moves.
//
// WHICH FIRMWARE THIS IS WRITTEN AGAINST. Sections 5b, 5c and 5f assert
// 0.15.12.0 planner behaviour -- the saturation at motor_control.c:510-515, the
// denominator==0 repair at :456-484, and the zero-duration guard at :2051. On
// the ESP32-S3 BENCH motor, which is still on fw 0.15.9.0 and cannot be
// reflashed from the host, those same moves are SILENTLY ACCEPTED with
// acceleration 0 (see TEST_EXPANSION_CAMPAIGN_2026-08.md, "The ESP32-S3 is a
// NEGATIVE CONTROL"). Failures of 5b/5c/5f there are EXPECTED and are not a new
// finding: the rack (0.15.12.0) is the authority. Sections 1-4, 5a, 5d, 5e and
// 6 touch none of the fixed code and should pass identically on both.
//
// RUNTIME. Roughly 100 s: 49 tfhReset/tfhFreshOpen calls at 1.5 s each dominate
// everything else. Register it with a timeout of at least 240 s, or the harness
// will record a CRASH that is really just the clock.
//
// SAFETY. The MOSFETs stay OFF for the entire module: goToClosedLoop() and
// enableMosfets() are never called, which is also precisely why every homing
// attempt here stops dead at motor_control.c:1436 without ever queueing a
// segment -- a rejected homing is not motion, it is a refusal. The only things
// that move are COMMANDED positions in the delegate sections, which the planner
// advances whether or not the output stage is energised, so no shaft turns on
// any motor. Every move is well inside the deviation window opened by
// tfhFreshOpen (tf_motion_helpers.h), so error 45 is not in play. No broadcast,
// no setDeviceAlias, no test modes: safe on the shared 35-motor rack. Every
// deliberately-latched fault is cleared with tfhReset before moving on, and the
// module ends by asserting the motor is clean.
// ---------------------------------------------------------------------------

// Error codes, all from python_programs/servomotor/error_codes.json.
static const uint8_t ERR_QUEUE_NOT_EMPTY    = 8;   // start_homing guard #2
static const uint8_t ERR_NOT_IN_CLOSED_LOOP = 13;  // start_homing guard #1
static const uint8_t ERR_ACCEL_TOO_HIGH     = 15;
static const uint8_t ERR_VEL_TOO_HIGH       = 16;
static const uint8_t ERR_PRED_VEL_TOO_HIGH  = 28;
static const uint8_t ERR_PARAM_OUT_OF_RANGE = 34;  // zero duration

// A quarter turn: small, fast, and far inside every ceiling tfhFreshOpen sets.
static const int32_t  QUARTER_ROT = (int32_t)(TFH_CPR / 4);   // 819200 counts
// A tenth of that again, used where the value only has to be "some legal
// distance" and the point is the parameter, not the motion.
static const int32_t  SMALL_DIST  = 65536;                    // 0.02 rotation

// True if `code` is one of the three refusal codes the planner uses for a
// distance/duration pair that cannot be flown inside the configured limits.
static bool isPlannerRefusal(uint8_t code) {
    return code == ERR_ACCEL_TOO_HIGH ||
           code == ERR_VEL_TOO_HIGH ||
           code == ERR_PRED_VEL_TOO_HIGH;
}

// What one attempt to start homing produced.
struct HaHomingAttempt {
    int     reported;   // what the LIBRARY told the caller (0 = clean success reply)
    uint8_t latched;    // what Get status reports afterwards (0xFF = unreadable)
};

// Ask the motor to home, then interrogate it. The short settle exists because
// fatal_error() takes over the CPU and hand-pumps the UART; reading status
// immediately can race the transition.
static HaHomingAttempt tryHoming(Servomotor* m, int32_t maxDistance, uint32_t maxDuration) {
    HaHomingAttempt r;
    m->homingRaw(maxDistance, maxDuration);
    r.reported = m->getError();
    delay(300);
    r.latched = tfhFatal(m);
    return r;
}

// What one attempt to queue a trapezoid move produced. `moved` is only
// meaningful when nothing latched: once a fatal error is set, Get status is the
// only command that returns real data, so the position read fails by design.
struct HaPlannerAttempt {
    bool    accepted;   // the enqueue reply came back clean
    uint8_t latched;    // fatal code afterwards (0 = none, 0xFF = unreadable)
    int64_t moved;      // commanded-position delta, valid only when latched == 0
};

static HaPlannerAttempt tryTrapezoid(Servomotor* m, int32_t displacement, uint32_t ticks) {
    HaPlannerAttempt r;
    r.accepted = false;
    r.latched  = 0xFF;
    r.moved    = 0;

    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) return r;

    m->trapezoidMoveRaw(displacement, ticks);
    r.accepted = (m->getError() == 0);
    if (r.accepted) {
        // NEVER a fixed wait: poll the queue to empty with a bounded deadline.
        uint64_t ms = (uint64_t)ticks * 1000ULL / (uint64_t)TFH_TPS;
        if (ms > 20000ULL) ms = 20000ULL;
        tfhDrain(m, (uint32_t)ms + 8000);
    } else {
        delay(300);
    }
    r.latched = tfhFatal(m);
    if (r.latched == 0) {
        int64_t after = m->getPositionRaw();
        if (m->getError() == 0) r.moved = after - before;
    }
    return r;
}

// Milliseconds taken for the queue to drain, or 0 if it never did. Used to show
// that maxDuration is a PACING budget -- the move really does occupy the time
// it was given -- not merely an upper bound that a fast move finishes early.
static uint32_t timedDrain(Servomotor* m, uint32_t budgetMs) {
    uint32_t t0 = millis();
    while (millis() - t0 < budgetMs) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return 0;
        if (q == 0) return millis() - t0;
        delay(10);
    }
    return 0;
}

void tm_homing_arithmetic(void) {
    Serial.println("tm_homing_arithmetic: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE BASELINE. After a reset the machine is in
    //    OPEN_LOOP_POSITION_CONTROL (motor_control.c:170, restored at :3735),
    //    so the very first guard in start_homing fires. Establish that plainly
    //    before building anything on it, and establish that the CALLER is told
    //    -- a refusal the caller cannot see would be worse than no refusal.
    // =====================================================================
    printf("\n--- 1. homing from open loop is refused, and the caller hears about it ---\n");
    {
        tfhReset(m);
        m->disableMosfets();          // already off after a reset; make it explicit
        delay(150);

        TEST_RESULT("the motor starts with no latched fatal error", tfhFatal(m) == 0);

        HaHomingAttempt a = tryHoming(m, SMALL_DIST, TFH_TPS / 2);   // 0.02 rot, 0.5 s
        printf("   homing(%lld counts, %llu ticks): library reported %d, status says %u\n",
               (long long)SMALL_DIST, (unsigned long long)(TFH_TPS / 2),
               a.reported, (unsigned)a.latched);
        TEST_RESULT("homing from open loop latches ERROR_NOT_IN_CLOSED_LOOP (13)",
                    a.latched == ERR_NOT_IN_CLOSED_LOOP);
        TEST_RESULT("the caller is told it failed rather than getting a success reply",
                    a.reported != 0);

        tfhReset(m);
        TEST_RESULT("a system reset clears the homing rejection", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 2. NO PARAMETER CAN REACH THE ARITHMETIC. The guard ordering in
    //    start_homing is not incidental -- it decides which error a caller
    //    sees. Because the closed-loop test at motor_control.c:1436 comes
    //    FIRST and looks at neither parameter, every one of these pairs must
    //    produce code 13 and nothing else. Two of them are chosen to be
    //    diagnostic: a zero duration would give 34 if the planner ran first,
    //    and an arithmetically impossible pair would give 15/16/28. Seeing 13
    //    for those is the ordering, measured rather than assumed.
    // =====================================================================
    printf("\n--- 2. the closed-loop guard is evaluated before any parameter ---\n");
    {
        struct Case { const char* label; int32_t dist; uint32_t dur; };
        const Case cases[] = {
            { "an ordinary small distance",     SMALL_DIST,   TFH_TPS / 2 },
            { "a zero distance",                0,            TFH_TPS / 2 },
            { "a zero duration",                SMALL_DIST,   0           },
            { "an impossible distance/duration pair", 2000000, 1          },
            { "the most negative int32 distance", INT32_MIN,  (uint32_t)TFH_TPS },
            { "the largest possible duration",  SMALL_DIST,   UINT32_MAX  },
            { "a negative distance",            -SMALL_DIST,  TFH_TPS / 2 },
        };
        const unsigned N = sizeof(cases) / sizeof(cases[0]);

        bool anyReportedSuccess = false;
        bool anyCarriedCode13   = false;

        for (unsigned i = 0; i < N; i++) {
            tfhReset(m);
            HaHomingAttempt a = tryHoming(m, cases[i].dist, cases[i].dur);
            printf("   %-40s dist=%lld dur=%llu -> reported %d, latched %u\n",
                   cases[i].label, (long long)cases[i].dist,
                   (unsigned long long)cases[i].dur, a.reported, (unsigned)a.latched);
            if (a.reported == 0) anyReportedSuccess = true;
            if (a.reported == (int)ERR_NOT_IN_CLOSED_LOOP) anyCarriedCode13 = true;
            TEST_RESULT(std::string("homing is refused with code 13 for ") + cases[i].label,
                        a.latched == ERR_NOT_IN_CLOSED_LOOP);
        }

        // The regression this guards against is the ugliest failure mode there
        // is: a command that is refused by the machine but reported as fine.
        TEST_RESULT("no parameter combination produced a success reply",
                    !anyReportedSuccess);
        // The error PACKET should carry the firmware's own number, so a caller
        // learns the reason without a second round trip. Asserted as "at least
        // one", because a link-level timeout instead of the code is a plausible
        // and harmless alternative for any single attempt.
        TEST_RESULT("at least one rejection carried the firmware's own code 13 to the caller",
                    anyCarriedCode13);

        tfhReset(m);
    }

    // =====================================================================
    // 3. GUARD #1 BEATS GUARD #2. The queue-not-empty check
    //    (motor_control.c:1440-1442, code 8) sits BELOW the closed-loop check,
    //    so with a move in flight AND the machine in open loop the answer must
    //    still be 13. This is the only way to observe the relative order of
    //    the two guards from outside, and it matters: a caller who sees 8 will
    //    go looking at their queue when the real problem is their mode.
    // =====================================================================
    printf("\n--- 3. with a queued move AND open loop, the closed-loop guard still wins ---\n");
    {
        tfhFreshOpen(m);
        // Three seconds of commanded travel, a quarter turn, MOSFETs off.
        m->trapezoidMoveRaw(QUARTER_ROT, (uint32_t)(3 * TFH_TPS));
        bool queued = (m->getError() == 0);
        uint8_t depth = m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);
        printf("   queue depth while the move is in flight: %u (queued=%d)\n",
               (unsigned)depth, (int)queued);
        TEST_RESULT("a trapezoid move really is sitting in the queue",
                    queued && depthOk && depth > 0);

        HaHomingAttempt a = tryHoming(m, SMALL_DIST, TFH_TPS / 2);
        printf("   homing with a non-empty queue -> latched %u (13 = mode, 8 = queue)\n",
               (unsigned)a.latched);
        TEST_RESULT("a non-empty queue does not change the answer: still code 13, not 8",
                    a.latched == ERR_NOT_IN_CLOSED_LOOP && a.latched != ERR_QUEUE_NOT_EMPTY);

        tfhReset(m);
        TEST_RESULT("reset clears the rejection raised with a move in flight",
                    tfhFatal(m) == 0);
    }

    // =====================================================================
    // 4. A FAULTED MACHINE STAYS REFUSED, AND KEEPS ITS ORIGINAL DIAGNOSIS.
    //    Once fatal_error() has taken over, Get status is the only command
    //    that returns real data (campaign finding F2). A second homing must
    //    therefore neither succeed nor OVERWRITE the code that explains what
    //    went wrong first -- an error code that the next command can clobber
    //    is not a diagnosis.
    // =====================================================================
    printf("\n--- 4. a second homing on a faulted machine changes nothing ---\n");
    {
        tfhReset(m);
        HaHomingAttempt first = tryHoming(m, SMALL_DIST, TFH_TPS / 2);

        getStatusResponse st = m->getStatus();
        bool statusAnswers = (m->getError() == 0);
        printf("   after the fault: status readable=%d, flags=0x%04X, code %u\n",
               (int)statusAnswers, (unsigned)st.statusFlags, (unsigned)st.fatalErrorCode);
        TEST_RESULT("Get status still answers while the machine is faulted", statusAnswers);

        // Different parameters this time; the latched code must not move.
        HaHomingAttempt second = tryHoming(m, -QUARTER_ROT, (uint32_t)(2 * TFH_TPS));
        printf("   first latched %u, second attempt latched %u\n",
               (unsigned)first.latched, (unsigned)second.latched);
        TEST_RESULT("a second homing does not overwrite the first latched code",
                    first.latched == ERR_NOT_IN_CLOSED_LOOP &&
                    second.latched == first.latched);

        tfhReset(m);
        TEST_RESULT("reset recovers from a repeatedly-refused homing", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 5. THE SHARED PLANNER. Everything from here on exercises
    //    add_trapezoid_move_to_queue (motor_control.c:2045) through the
    //    'Trapezoid move' command, which reaches it at main.c:261 with the
    //    same two arguments homing passes at motor_control.c:1444. This is
    //    homing's arithmetic, reached the only way it can be reached with the
    //    output stage off.
    // =====================================================================

    // ---------------------------------------------------------------------
    // 5a. THE SIGN OF maxDistance SELECTS THE DIRECTION. It is a signed int32
    //     on the wire (homingPayload in Servomotor.h; the firmware's
    //     max_homing_travel_displacement at main.c:431), and the sign is the
    //     only thing that says which way to look for the end stop.
    // ---------------------------------------------------------------------
    printf("\n--- 5a. the sign of the distance selects the direction ---\n");
    {
        tfhFreshOpen(m);
        uint32_t ticks = tfhTicksFor(QUARTER_ROT);

        bool fwdOk = false;
        int64_t fwd = tfhMove(m, QUARTER_ROT, ticks, &fwdOk);
        bool revOk = false;
        int64_t rev = tfhMove(m, -QUARTER_ROT, ticks, &revOk);
        printf("   +%lld counts moved %lld; -%lld counts moved %lld\n",
               (long long)QUARTER_ROT, (long long)fwd,
               (long long)QUARTER_ROT, (long long)rev);

        TEST_RESULT("a positive distance drives the commanded position forward",
                    fwdOk && fwd > 0);
        TEST_RESULT("and a positive distance delivers the whole of it",
                    fwdOk && llabs((long long)(fwd - QUARTER_ROT)) < 200);
        TEST_RESULT("a negative distance drives the commanded position backward",
                    revOk && rev < 0);
        TEST_RESULT("and a negative distance delivers the whole of it",
                    revOk && llabs((long long)(rev + QUARTER_ROT)) < 200);

        tfhReset(m);
    }

    // ---------------------------------------------------------------------
    // 5b. THE DURATION BOUNDS THE DISTANCE, AND AN IMPOSSIBLE PAIR IS
    //     REFUSED -- NOT STRETCHED. A quarter turn in one control tick, or in
    //     100 of them, needs an acceleration far outside int32; the planner
    //     SATURATES it (motor_control.c:510-515) precisely so the ceiling
    //     check at :1795 rejects the move loudly instead of truncating it into
    //     a different, arbitrary move. Silent acceptance with no motion was a
    //     real fw 0.15.9.0 defect; this is its regression guard on the path
    //     homing shares.
    // ---------------------------------------------------------------------
    printf("\n--- 5b. an impossible duration is refused; a generous one is flown ---\n");
    {
        const uint32_t impossible[] = { 1, 100 };
        bool anySilentlyAccepted = false;

        for (unsigned i = 0; i < 2; i++) {
            tfhFreshOpen(m);
            HaPlannerAttempt p = tryTrapezoid(m, QUARTER_ROT, impossible[i]);
            printf("   %lld counts in %llu ticks: accepted=%d latched=%u\n",
                   (long long)QUARTER_ROT, (unsigned long long)impossible[i],
                   (int)p.accepted, (unsigned)p.latched);
            if (p.accepted && p.latched == 0) anySilentlyAccepted = true;
            TEST_RESULT(std::string("a quarter turn in ") +
                            std::to_string((long long)impossible[i]) +
                            " ticks is refused with a planner code (15/16/28)",
                        isPlannerRefusal(p.latched));
            tfhReset(m);
        }
        TEST_RESULT("neither impossible pair was silently accepted and then ignored",
                    !anySilentlyAccepted);

        const uint32_t generous[] = { (uint32_t)TFH_TPS, (uint32_t)(2 * TFH_TPS) };
        for (unsigned i = 0; i < 2; i++) {
            tfhFreshOpen(m);
            HaPlannerAttempt p = tryTrapezoid(m, QUARTER_ROT, generous[i]);
            printf("   %lld counts in %llu ticks: accepted=%d latched=%u moved=%lld\n",
                   (long long)QUARTER_ROT, (unsigned long long)generous[i],
                   (int)p.accepted, (unsigned)p.latched, (long long)p.moved);
            TEST_RESULT(std::string("the same quarter turn over ") +
                            std::to_string((long long)generous[i]) +
                            " ticks is accepted and delivered in full",
                        p.accepted && p.latched == 0 &&
                        llabs((long long)(p.moved - QUARTER_ROT)) < 200);
            tfhReset(m);
        }
    }

    // ---------------------------------------------------------------------
    // 5c. A ZERO DURATION IS REFUSED WITH ERROR_PARAMETER_OUT_OF_RANGE (34).
    //     This is the single guard at motor_control.c:2051-2053, the first
    //     statement of add_trapezoid_move_to_queue -- the very line homing
    //     jumps to from :1444. tm_homing_edges observes code 34 from homing
    //     itself in closed loop; measuring the identical code here, from the
    //     delegate, is the behavioural evidence that it is one guard and not
    //     two lookalikes.
    // ---------------------------------------------------------------------
    printf("\n--- 5c. a zero duration is refused with code 34, the guard homing shares ---\n");
    {
        tfhFreshOpen(m);
        HaPlannerAttempt p = tryTrapezoid(m, QUARTER_ROT, 0);
        printf("   %lld counts in 0 ticks: accepted=%d latched=%u\n",
               (long long)QUARTER_ROT, (int)p.accepted, (unsigned)p.latched);
        TEST_RESULT("a zero duration is refused with ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    p.latched == ERR_PARAM_OUT_OF_RANGE);

        tfhReset(m);
        TEST_RESULT("reset clears the zero-duration rejection", tfhFatal(m) == 0);
    }

    // ---------------------------------------------------------------------
    // 5d. A ZERO DISTANCE IS LEGAL AND MOVES NOTHING. The asymmetry with 5c is
    //     deliberate in the firmware: zero time divides by zero, zero distance
    //     is simply a move of nothing (delta_d = 0 -> acceleration 0 -> three
    //     segments that each add nothing). What that means for HOMING -- that a
    //     zero maxDistance holds the busy state for the full duration without
    //     travelling -- is an INFERENCE from the shared planner, not something
    //     stated in motor_commands.json; it is not asserted here, only the
    //     delegate behaviour that is actually measured is.
    // ---------------------------------------------------------------------
    printf("\n--- 5d. a zero distance over a legal duration is a legal no-op ---\n");
    {
        tfhFreshOpen(m);
        HaPlannerAttempt p = tryTrapezoid(m, 0, (uint32_t)TFH_TPS);
        printf("   0 counts in %llu ticks: accepted=%d latched=%u moved=%lld\n",
               (unsigned long long)TFH_TPS, (int)p.accepted,
               (unsigned)p.latched, (long long)p.moved);
        TEST_RESULT("a zero distance over a legal duration is accepted",
                    p.accepted && p.latched == 0);
        // One count of slack rather than exact equality. The arithmetic really
        // is exact (acceleration 0 means current_velocity_i64 stays 0 and
        // current_position_i96 is never advanced), but 'Get position' reports
        // the whole-count part of a fixed-point accumulator, so one count is
        // the resolution of the instrument and not of the planner. This matches
        // the dwell convention already used in tm_planner_arithmetic section 6.
        // A real dwell defect would move thousands of counts, not one.
        TEST_RESULT("and it moves the commanded position nowhere",
                    llabs((long long)p.moved) <= 1);

        tfhReset(m);
    }

    // ---------------------------------------------------------------------
    // 5e. THE DURATION IS A PACING BUDGET, NOT A TIMEOUT. This is the claim
    //     that makes maxDuration matter to a homing caller: the approach speed
    //     is roughly maxDistance/maxDuration, so a generous budget makes the
    //     approach GENTLER rather than merely allowing more time. If the move
    //     ran at full speed and simply finished early, a long budget would be
    //     no safer than a short one for finding a hard stop.
    // ---------------------------------------------------------------------
    printf("\n--- 5e. the duration paces the move rather than merely bounding it ---\n");
    {
        tfhFreshOpen(m);
        m->trapezoidMoveRaw(QUARTER_ROT, (uint32_t)TFH_TPS);
        uint32_t oneSecond = (m->getError() == 0) ? timedDrain(m, 15000) : 0;
        tfhReset(m);

        tfhFreshOpen(m);
        m->trapezoidMoveRaw(QUARTER_ROT, (uint32_t)(2 * TFH_TPS));
        uint32_t twoSeconds = (m->getError() == 0) ? timedDrain(m, 20000) : 0;
        tfhReset(m);

        // A ZERO in either reading is timedDrain reporting that the queue never
        // emptied or that a getNQueuedItems() round trip failed -- i.e. a link
        // problem, NOT the planner mispacing the move. Read the printed numbers
        // before concluding anything about the firmware from these two lines.
        double ratio = (oneSecond > 0) ? (double)twoSeconds / (double)oneSecond : 0.0;
        printf("   1 s budget took %llu ms; 2 s budget took %llu ms; ratio %.2f\n",
               (unsigned long long)oneSecond, (unsigned long long)twoSeconds, ratio);
        TEST_RESULT("a one-second budget really takes about one second",
                    oneSecond > 700 && oneSecond < 1600);
        TEST_RESULT("doubling the budget roughly doubles the time on the same distance",
                    ratio > 1.5 && ratio < 2.5);
    }

    // ---------------------------------------------------------------------
    // 5f. IT IS THE PAIR THAT IS JUDGED, NOT EITHER NUMBER ALONE. The same
    //     duration accepts one distance and refuses another, exactly as the
    //     same distance was accepted at one duration and refused at another in
    //     5b. Neither parameter has a legal range of its own.
    // ---------------------------------------------------------------------
    printf("\n--- 5f. at one fixed duration, the distance decides ---\n");
    {
        tfhFreshOpen(m);
        HaPlannerAttempt small = tryTrapezoid(m, QUARTER_ROT, (uint32_t)TFH_TPS);
        printf("   quarter turn in 1 s: accepted=%d latched=%u moved=%lld\n",
               (int)small.accepted, (unsigned)small.latched, (long long)small.moved);
        TEST_RESULT("a quarter turn is legal in one second",
                    small.accepted && small.latched == 0 &&
                    llabs((long long)(small.moved - QUARTER_ROT)) < 200);
        tfhReset(m);

        // 100 rotations in the same second: about 100 rot/s against a 15 rot/s
        // ceiling, so the ramp acceleration blows past max_acceleration.
        tfhFreshOpen(m);
        HaPlannerAttempt huge = tryTrapezoid(m, (int32_t)(TFH_CPR * 100), (uint32_t)TFH_TPS);
        printf("   100 rotations in 1 s: accepted=%d latched=%u\n",
               (int)huge.accepted, (unsigned)huge.latched);
        TEST_RESULT("a hundred rotations in the same one second is refused (15/16/28)",
                    isPlannerRefusal(huge.latched));

        tfhReset(m);
    }

    // =====================================================================
    // 6. RECOVERY. Every refusal in this module is a LATCHED fatal error, so
    //    the machine is only usable again if a reset genuinely restores the
    //    planner. Checked after both kinds of refusal -- the precondition kind
    //    and the arithmetic kind -- and then repeatedly, because a recovery
    //    that works once is not the same as a recovery that works.
    // =====================================================================
    printf("\n--- 6. the machine is fully usable again after every refusal ---\n");
    {
        // (i) after a homing precondition refusal
        tfhReset(m);
        tryHoming(m, SMALL_DIST, TFH_TPS / 2);
        tfhFreshOpen(m);
        bool ok1 = false;
        int64_t moved1 = tfhMove(m, QUARTER_ROT, tfhTicksFor(QUARTER_ROT), &ok1);
        printf("   after a homing refusal, a real move delivered %lld of %lld\n",
               (long long)moved1, (long long)QUARTER_ROT);
        TEST_RESULT("after a homing refusal a full move delivers its exact displacement",
                    ok1 && llabs((long long)(moved1 - QUARTER_ROT)) < 200);
        tfhReset(m);

        // (ii) after an arithmetic refusal from the shared planner
        tfhFreshOpen(m);
        tryTrapezoid(m, QUARTER_ROT, 1);
        tfhFreshOpen(m);
        bool ok2 = false;
        int64_t moved2 = tfhMove(m, QUARTER_ROT, tfhTicksFor(QUARTER_ROT), &ok2);
        printf("   after a planner refusal, a real move delivered %lld of %lld\n",
               (long long)moved2, (long long)QUARTER_ROT);
        TEST_RESULT("after a planner refusal a full move delivers its exact displacement",
                    ok2 && llabs((long long)(moved2 - QUARTER_ROT)) < 200);
        tfhReset(m);

        // (iii) repeatedly
        bool cyclesOk = true;
        for (int cycle = 0; cycle < 3 && cyclesOk; cycle++) {
            tfhReset(m);
            HaHomingAttempt a = tryHoming(m, SMALL_DIST, TFH_TPS / 2);
            if (a.latched != ERR_NOT_IN_CLOSED_LOOP) {
                printf("   cycle %d did not fault (latched %u)\n", cycle, (unsigned)a.latched);
                cyclesOk = false;
                break;
            }
            tfhReset(m);
            if (tfhFatal(m) != 0) {
                printf("   cycle %d did not clear\n", cycle);
                cyclesOk = false;
            }
        }
        TEST_RESULT("three refuse-and-recover cycles each fault and each clear", cyclesOk);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
