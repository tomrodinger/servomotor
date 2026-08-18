#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_zero_position_semantics.cpp
//
// "ZERO POSITION" (command 13) AS AN OPERATION, CHARACTERISED END TO END.
//
// Zero position is the only command in the whole API that redefines what every
// other position number MEANS. It is what a user calls after homing, after
// mounting a tool, after every power cycle -- and everything downstream (the
// absolute go-to target, the safety fence, the deviation watchdog, the number
// printed on a screen) is measured from whatever origin it last established.
// A defect here does not produce a wrong error code; it produces a machine that
// is silently in the wrong place.
//
// The firmware side is very small, which is exactly why it is worth pinning
// down precisely (firmware/Src/motor_control.c:3311-3338):
//
//     void zero_position(void) {
//         if(n_items_in_queue != 0) {
//             fatal_error(ERROR_QUEUE_NOT_EMPTY);          // code 8
//         }
//         __disable_irq();
//         current_position_i96 = 0;                        // :3319-3321
//         position_after_last_queue_item_i96 = 0;          // :3323-3325
//         current_velocity_i64 = 0;                        // :3327
//         hall_position_i64 = 0;  hall_position_delta = 0; // :3329-3330
//         bound_the_sensor_position(...);                  // :3331
//         velocity = 0; integral_term = 0;                 // :3333-3334
//         previous_error = 0; low_pass_filtered_error_change = 0;
//         __enable_irq();
//     }
//
// Four separate claims fall out of those twenty lines, and this module asserts
// all four behaviourally rather than by reading the source:
//
//   1. EXACTNESS. current_position_i96 is assigned zero, not nudged toward it.
//      With the queue empty the control loop adds current_velocity_i64 (also
//      zero) to it every tick (motor_control.c:2189), so the commanded position
//      must read EXACTLY 0 afterwards -- not "about 0". Every position assert
//      here demands exact zero, which is a strictly sharper claim than the
//      "within a couple of counts" the suite settles for elsewhere.
//
//   2. THE PRECONDITION IS QUEUE OCCUPANCY, NOT MOTION. The guard tests
//      n_items_in_queue, and nothing else -- not the MOSFETs, not the velocity,
//      not whether anything is actually turning. Section 3 proves the
//      distinction with a queued ZERO-velocity segment: one occupied slot, a
//      completely stationary machine, and the command is still refused.
//
//   3. THE FRAME MOVES, THE PLANNER DOES NOT. position_after_last_queue_item
//      is zeroed alongside the live position, and that is the value
//      add_go_to_position_to_queue() subtracts from an absolute target
//      (motor_control.c:2074-2079). So ABSOLUTE moves are re-based and RELATIVE
//      moves (add_trapezoid_move_to_queue, :2045) are untouched. Sections 5 and
//      6 assert both halves; either one alone could be satisfied by a bug.
//
//   4. ZEROING RE-ARMS THE DEVIATION WATCHDOG. The commanded position and the
//      hall position are zeroed in the SAME critical section, so the deviation
//      they are compared on (motor_control.c:3217) collapses to zero. Section 7
//      turns that into a striking observable: with the output stage off and the
//      FACTORY two-rotation window, six rotations are commanded without a
//      fault, purely because a zeroing sits between the legs -- while the same
//      sequence without the zeroings trips error 45, as it must.
//
// HOW THIS DIFFERS FROM WHAT ALREADY EXISTS:
//   * tm_position_frames covers zeroing on a CLOSED-LOOP, calibrated motor with
//     the MOSFETs ON, and asks whether the frame re-bases to "~0" within 0.03
//     of a rotation. This module is the open-loop, output-stage-OFF counterpart
//     and asks for exact counts; it also covers idempotence, the queue-depth
//     boundary, the drain-then-zero race and the watchdog interaction, none of
//     which appear there.
//   * tm_error_taxonomy provokes the queue-not-empty fault exactly ONCE, as one
//     entry in a sweep of many different errors. Section 3 here is about the
//     shape of that boundary: which states refuse, whether the code is stable,
//     and the stationary-but-occupied case that distinguishes "busy queue" from
//     "moving machine".
//   * tm_command_state_matrix records zeroing as one cell of a command/state
//     grid (accepted / refused). It never checks what a successful zeroing
//     actually DID, which is this module's whole subject.
//   * tm_safety_fence already owns the zeroing-versus-safety-fence interaction
//     (the fence numbers are NOT rebased, so the fence appears to follow the new
//     origin). That is deliberately not repeated here; no safety limits are set
//     anywhere in this module.
//
// SAFETY. The MOSFETs stay OFF for the entire module and are never enabled --
// no enableMosfets(), no goToClosedLoop(). Every position number read here is
// the COMMANDED position, which the planner advances whether or not the output
// stage is energised (motor_control.c:2611 calls handle_queued_movements()
// unconditionally), so nothing physically turns on any motor at any point; the
// hall reading is sampled precisely to confirm that. Section 7 deliberately
// trips ERROR_POSITION_DEVIATION_TOO_LARGE and section 3 deliberately trips
// ERROR_QUEUE_NOT_EMPTY; both are ordinary latched faults, each is cleared by a
// system reset immediately afterwards, and no command other than get_status is
// sent while a fault is latched. No broadcasts, no alias writes and no test
// modes: safe to run on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// Codes from python_programs/servomotor/error_codes.json.
static const uint8_t ERR_QUEUE_NOT_EMPTY              = 8;
static const uint8_t ERR_POSITION_DEVIATION_TOO_LARGE = 45;

// Sentinels returned by the helpers below, chosen well above any real code so
// "the probe itself failed" can never be mistaken for "the device said 0".
static const uint8_t PROBE_REFUSED_AT_QUEUE_TIME = 0xFD;
static const uint8_t PROBE_COMMS_FAILED          = 0xFE;

// A commanded move lands within a count or two of its target; this is the
// slack used for "delivered its displacement" checks. Position assertions
// about zeroing itself are EXACT and use no tolerance at all.
static const int64_t MOVE_TOL = 200;

// ---------------------------------------------------------------------------
// Issue Zero position and report both halves of the outcome: whether the
// device answered normally, and what fatal code (if any) it latched.
//
// The 200 ms is not padding. fatal_error() disables interrupts and never
// returns (common_source_files/error_handling.c:169); the refusal is reported
// by the hand-pumped UART loop inside it, so the status has to be asked for
// after that loop has taken over, not in the same breath.
// ---------------------------------------------------------------------------
static uint8_t zeroAndFatal(Servomotor* m, bool* answeredNormally) {
    m->zeroPosition();
    *answeredNormally = (m->getError() == 0);
    delay(200);
    return tfhFatal(m);
}

// ---------------------------------------------------------------------------
// Run one trapezoid move and report the LATCHED FATAL CODE rather than a
// pass/fail flag, so a deliberately-tripped watchdog is measurable instead of
// merely "the move did not work". `moved` receives the commanded-position
// delta, or 0 when the move did not complete.
//
// The wait is always tfhDrain(), never a computed duration: a fixed wait that
// is even slightly short reads a PARTIAL move and looks exactly like a planner
// bug. That mistake produced this campaign's first false finding.
// ---------------------------------------------------------------------------
static uint8_t legFatal(Servomotor* m, int32_t counts, uint32_t ticks, int64_t* moved) {
    *moved = 0;
    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) return PROBE_COMMS_FAILED;
    m->trapezoidMoveRaw(counts, ticks);
    if (m->getError() != 0) return PROBE_REFUSED_AT_QUEUE_TIME;
    uint32_t budget = (uint32_t)((uint64_t)ticks * 1000ULL / TFH_TPS) + 10000;
    tfhDrain(m, budget);                 // may return false: the fault refuses reads
    uint8_t f = tfhFatal(m);
    if (f == 0) *moved = m->getPositionRaw() - before;
    return f;
}

void tm_zero_position_semantics(void) {
    Serial.println("tm_zero_position_semantics: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. ZEROING AN ALREADY-ZERO MACHINE IS EXACT AND IDEMPOTENT.
    //
    //    The degenerate case is worth asserting first because it is the one a
    //    real caller performs most often -- zero at start-up, zero again after
    //    a homing routine that ended where it began -- and because "exactly
    //    zero" is the premise every later section measures against. An
    //    operation that drifted by a count per invocation would be invisible
    //    in a single call and ruinous over a shift.
    // =====================================================================
    printf("\n--- 1. zeroing an already-zero machine is exact and idempotent ---\n");
    {
        tfhReset(m);
        m->disableMosfets();             // explicit: the output stage stays off
        delay(200);

        int64_t p0 = m->getPositionRaw();
        int64_t h0 = m->getHallSensorPositionRaw();
        printf("   after reset: commanded=%lld hall=%lld\n",
               (long long)p0, (long long)h0);
        TEST_RESULT("a freshly reset machine already reads exactly zero", p0 == 0);

        bool answered = false;
        uint8_t f = zeroAndFatal(m, &answered);
        int64_t p1 = m->getPositionRaw();
        printf("   first zeroing: answered=%d fatal=%u commanded=%lld\n",
               (int)answered, (unsigned)f, (long long)p1);
        TEST_RESULT("zeroing an already-zero machine is accepted", answered);
        TEST_RESULT("zeroing an already-zero machine raises no fatal error", f == 0);
        TEST_RESULT("and it leaves the commanded position at exactly zero", p1 == 0);

        // Idempotence: five more in a row must change nothing at all.
        bool allAccepted = true;
        bool allClean = true;
        for (int i = 0; i < 5; i++) {
            bool a = false;
            uint8_t ff = zeroAndFatal(m, &a);
            if (!a) allAccepted = false;
            if (ff != 0) { allClean = false; printf("   repeat %d latched fatal %u\n", i, (unsigned)ff); }
        }
        int64_t p2 = m->getPositionRaw();
        int64_t h2 = m->getHallSensorPositionRaw();
        printf("   after six zeroings in a row: commanded=%lld hall=%lld\n",
               (long long)p2, (long long)h2);
        TEST_RESULT("five further consecutive zeroings are all accepted with no fatal error",
                    allAccepted && allClean);
        TEST_RESULT("and the commanded position is still exactly zero after all of them",
                    p2 == 0);
    }

    // =====================================================================
    // 2. ZEROING FAR FROM THE ORIGIN GENUINELY RESETS A LARGE POSITION.
    //
    //    The interesting failure mode is not "zero does nothing" but "zero
    //    only works near zero" -- a truncation, a 32-bit assignment into a
    //    96-bit accumulator, a mid/high word left behind. current_position_i96
    //    is three words and zero_position clears all three by hand, so this is
    //    checked at magnitudes big enough that a half-cleared accumulator
    //    would show. Both signs, because a sign-extension slip only appears on
    //    one of them.
    //
    //    tfhFreshOpen() is required here: four rotations commanded with the
    //    output stage off would otherwise trip the two-rotation deviation
    //    window (error 45) before the zeroing was ever reached.
    // =====================================================================
    printf("\n--- 2. zeroing far from the origin resets a large position exactly ---\n");
    {
        const int64_t offsets[] = { TFH_CPR, -TFH_CPR, TFH_CPR * 4 };
        for (unsigned i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
            tfhFreshOpen(m);
            int64_t moved = 0;
            uint8_t f = legFatal(m, (int32_t)offsets[i], tfhTicksFor(offsets[i]), &moved);
            int64_t atOffset = m->getPositionRaw();

            bool answered = false;
            uint8_t zf = zeroAndFatal(m, &answered);
            int64_t afterZero = m->getPositionRaw();

            printf("   offset %lld: moved=%lld position=%lld -> after zeroing %lld (moveFatal=%u zeroFatal=%u)\n",
                   (long long)offsets[i], (long long)moved, (long long)atOffset,
                   (long long)afterZero, (unsigned)f, (unsigned)zf);

            char name[128];
            snprintf(name, sizeof(name),
                     "the machine reaches %lld counts before being zeroed there",
                     (long long)offsets[i]);
            TEST_RESULT(name, f == 0 && llabs((long long)(atOffset - offsets[i])) <= MOVE_TOL);

            snprintf(name, sizeof(name),
                     "zeroing at %lld counts resets the position to exactly zero",
                     (long long)offsets[i]);
            TEST_RESULT(name, answered && zf == 0 && afterZero == 0);
        }

        // Idempotence away from the origin: having just zeroed at a large
        // offset, a second zeroing must be a no-op like any other.
        tfhFreshOpen(m);
        int64_t moved = 0;
        legFatal(m, (int32_t)(TFH_CPR * 3), tfhTicksFor(TFH_CPR * 3), &moved);
        bool a1 = false, a2 = false;
        uint8_t f1 = zeroAndFatal(m, &a1);
        int64_t after1 = m->getPositionRaw();
        uint8_t f2 = zeroAndFatal(m, &a2);
        int64_t after2 = m->getPositionRaw();
        printf("   three rotations out: first zeroing -> %lld, second -> %lld\n",
               (long long)after1, (long long)after2);
        TEST_RESULT("a second zeroing at a far offset is accepted and changes nothing",
                    a1 && a2 && f1 == 0 && f2 == 0 && after1 == 0 && after2 == 0);
    }

    // =====================================================================
    // 3. THE PRECONDITION IS AN OCCUPIED QUEUE, NOT A MOVING MACHINE.
    //
    //    zero_position() guards on n_items_in_queue and on nothing else. The
    //    sharp case is a queued segment with velocity ZERO: exactly one slot
    //    occupied, the shaft and the commanded position both perfectly still,
    //    and the command must STILL be refused. If the guard were ever
    //    "loosened" to test motion instead of occupancy it would pass every
    //    other test in the suite and fail this one -- and it would corrupt the
    //    planner, because position_after_last_queue_item still points at the
    //    end of the queued segment.
    //
    //    Note what cannot be checked: once error 8 is latched, get_status is
    //    the ONLY command that answers (see tm_fatal_state_commands), so
    //    whether the refusal was partially applied is unobservable from the
    //    bus. The reset that follows zeroes the position regardless.
    // =====================================================================
    printf("\n--- 3. the refusal is about queue occupancy, not motion ---\n");
    {
        // 3a/3b/3c: one occupied slot, nothing moving.
        tfhFreshOpen(m);
        m->moveWithVelocityRaw(0, 10 * (uint32_t)TFH_TPS);   // 1 slot, ten seconds, zero speed
        bool queued = (m->getError() == 0);
        delay(200);
        uint8_t depth = m->getNQueuedItems();
        int64_t s0 = m->getPositionRaw();
        delay(400);
        int64_t s1 = m->getPositionRaw();
        printf("   zero-velocity segment: queued=%d depth=%u position %lld -> %lld\n",
               (int)queued, (unsigned)depth, (long long)s0, (long long)s1);
        TEST_RESULT("a zero-velocity segment occupies exactly one queue slot",
                    queued && depth == 1);
        TEST_RESULT("nothing moves while that segment runs", s0 == 0 && s1 == 0);

        bool answered = false;
        uint8_t f = zeroAndFatal(m, &answered);
        printf("   zeroing with one idle slot occupied -> fatal %u (expect %u)\n",
               (unsigned)f, (unsigned)ERR_QUEUE_NOT_EMPTY);
        TEST_RESULT("zeroing is refused with ERROR_QUEUE_NOT_EMPTY (8) even though nothing is moving",
                    f == ERR_QUEUE_NOT_EMPTY);

        // 3d: a real move in flight, three times, to show the code is stable
        // rather than incidental.
        uint8_t codes[3];
        bool recovered = true;
        for (int t = 0; t < 3; t++) {
            tfhFreshOpen(m);
            // Slow ceilings stretch each queued segment to about a second, so
            // the queue is reliably still occupied when the zeroing lands.
            m->setMaximumVelocity(1.0f);
            m->setMaximumAcceleration(1.0f);
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), 12 * (uint32_t)TFH_TPS);
            delay(150);
            bool ans = false;
            codes[t] = zeroAndFatal(m, &ans);
            tfhReset(m);
            if (tfhFatal(m) != 0) recovered = false;
        }
        printf("   in-flight trials: %u %u %u\n",
               (unsigned)codes[0], (unsigned)codes[1], (unsigned)codes[2]);
        TEST_RESULT("zeroing during a move in flight raises ERROR_QUEUE_NOT_EMPTY every time",
                    codes[0] == ERR_QUEUE_NOT_EMPTY &&
                    codes[1] == ERR_QUEUE_NOT_EMPTY &&
                    codes[2] == ERR_QUEUE_NOT_EMPTY);
        TEST_RESULT("a system reset clears the refusal after every trial", recovered);

        // 3e: a deep queue refuses identically. 10 moves x 3 slots = 30 of 32.
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        for (int k = 0; k < 10; k++) {
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 400), 12 * (uint32_t)TFH_TPS);
        }
        uint8_t deep = m->getNQueuedItems();
        bool deepAns = false;
        uint8_t deepFatal = zeroAndFatal(m, &deepAns);
        printf("   deep queue: depth=%u -> fatal %u\n",
               (unsigned)deep, (unsigned)deepFatal);
        TEST_RESULT("a deeply filled queue refuses zeroing with the same code",
                    deep >= 30 && deepFatal == ERR_QUEUE_NOT_EMPTY);
        tfhReset(m);
    }

    // =====================================================================
    // 4. ACCEPTED THE INSTANT THE QUEUE EMPTIES.
    //
    //    Section 3 would be satisfied by a command that simply never worked
    //    while anything had ever been queued, so the complementary half has to
    //    be asserted too -- and asserted tightly. The zeroing here is issued
    //    with NO settling delay at all, in the same breath as the queue-depth
    //    read that returned zero. n_items_in_queue is decremented inside the
    //    control ISR the moment an item's last time step is consumed
    //    (motor_control.c:2136/:2144), so a depth of zero on the bus means the
    //    guard has already been released; any lag or lock-out between the two
    //    would show up here as an intermittent error 8.
    // =====================================================================
    printf("\n--- 4. accepted the instant the queue reads empty ---\n");
    {
        tfhFreshOpen(m);
        int accepted = 0, clean = 0, exact = 0, trials = 0;
        for (int t = 0; t < 5; t++) {
            int32_t d = (int32_t)(TFH_CPR / 8);
            m->trapezoidMoveRaw(d, tfhTicksFor(d));
            if (m->getError() != 0) continue;

            // Poll the depth with a bounded deadline, then zero immediately.
            // The 5 ms polling cadence does not weaken the claim: what is
            // being asserted is that NOTHING sits between the depth read that
            // returned zero and the zeroing, not that the poll was fast.
            bool empty = false;
            uint32_t deadline = millis() + 8000;
            while (millis() < deadline) {
                uint8_t q = m->getNQueuedItems();
                if (m->getError() != 0) break;
                if (q == 0) { empty = true; break; }
                delay(5);
            }
            if (!empty) continue;
            trials++;

            m->zeroPosition();                       // no delay: that is the point
            bool ans = (m->getError() == 0);
            delay(150);
            uint8_t f = tfhFatal(m);
            int64_t p = m->getPositionRaw();
            printf("   trial %d: answered=%d fatal=%u position=%lld\n",
                   t, (int)ans, (unsigned)f, (long long)p);
            if (ans) accepted++;
            if (f == 0) clean++;
            if (p == 0) exact++;
        }
        TEST_RESULT("zeroing issued the instant the queue depth reads zero is always accepted",
                    trials == 5 && accepted == 5);
        TEST_RESULT("no drain-then-zero trial raised a fatal error", clean == 5);
        TEST_RESULT("every drain-then-zero trial left the position at exactly zero", exact == 5);

        int64_t moved = 0;
        uint8_t f = legFatal(m, (int32_t)(TFH_CPR / 4), tfhTicksFor(TFH_CPR / 4), &moved);
        printf("   move after five drain-and-zero cycles: moved=%lld fatal=%u\n",
               (long long)moved, (unsigned)f);
        TEST_RESULT("an ordinary move still works after five drain-and-zero cycles",
                    f == 0 && llabs((long long)(moved - TFH_CPR / 4)) <= MOVE_TOL);
    }

    // =====================================================================
    // 5. ZEROING REDEFINES THE FRAME FOR ABSOLUTE MOVES.
    //
    //    add_go_to_position_to_queue() computes its displacement as
    //    target - position_after_last_queue_item (motor_control.c:2074-2079),
    //    and zero_position() clears that very accumulator. So an unchanged
    //    absolute target means something different after every zeroing.
    //
    //    The demonstration is deliberately dramatic: the SAME go-to command,
    //    with the same number in it, is issued four times. Sent twice in a row
    //    it is a no-op the second time -- the machine is already there. With a
    //    zeroing in between it walks a further half rotation each time. That
    //    is the entire semantics of the command in one measurement, and it is
    //    also the trap a user falls into when they zero inside a loop.
    // =====================================================================
    printf("\n--- 5. zeroing re-bases absolute moves ---\n");
    {
        tfhFreshOpen(m);
        const int32_t D = (int32_t)(TFH_CPR / 2);
        const uint32_t T = tfhTicksFor(D);
        bool anyFatal = false;

        m->goToPositionRaw(D, T);
        tfhDrain(m, 15000);
        if (tfhFatal(m) != 0) anyFatal = true;
        int64_t p1 = m->getPositionRaw();

        // The same absolute target again, WITHOUT zeroing: displacement is
        // zero, so the machine must stay exactly where it is.
        m->goToPositionRaw(D, T);
        tfhDrain(m, 15000);
        if (tfhFatal(m) != 0) anyFatal = true;
        int64_t p2 = m->getPositionRaw();

        printf("   goToPosition(%ld) once -> %lld, twice -> %lld\n",
               (long)D, (long long)p1, (long long)p2);
        TEST_RESULT("an absolute move to +D from the origin lands at +D",
                    llabs((long long)(p1 - D)) <= MOVE_TOL);
        // The tolerance is not slack for a re-based frame: a frame bug would
        // move a further D (1638400 counts). It is there because a completed
        // trapezoid lands a count or two short of its target, so the repeat
        // legitimately mops up that residue.
        TEST_RESULT("repeating the same absolute target without zeroing leaves the machine where it is",
                    llabs((long long)(p2 - p1)) <= 64);

        // Now interleave zeroings: the same target advances the machine every
        // time, because the origin keeps moving with it.
        int64_t legs[3];
        int64_t total = 0;
        bool legsClean = true;
        for (int i = 0; i < 3; i++) {
            bool ans = false;
            if (zeroAndFatal(m, &ans) != 0 || !ans) legsClean = false;
            if (m->getPositionRaw() != 0) legsClean = false;   // exact, per section 1
            m->goToPositionRaw(D, T);
            tfhDrain(m, 15000);
            if (tfhFatal(m) != 0) { anyFatal = true; legsClean = false; }
            legs[i] = m->getPositionRaw();
            total += legs[i];
        }
        printf("   three zero-then-goto legs: %lld %lld %lld (total travel %lld, expect %lld)\n",
               (long long)legs[0], (long long)legs[1], (long long)legs[2],
               (long long)total, (long long)(3LL * D));
        TEST_RESULT("after zeroing, the unchanged absolute target advances the machine by +D again",
                    legsClean &&
                    llabs((long long)(legs[0] - D)) <= MOVE_TOL &&
                    llabs((long long)(legs[1] - D)) <= MOVE_TOL &&
                    llabs((long long)(legs[2] - D)) <= MOVE_TOL);
        TEST_RESULT("three such legs travel three times D although the target never changed",
                    llabs((long long)(total - 3LL * D)) <= 3 * MOVE_TOL);
        TEST_RESULT("no fatal error anywhere in the re-basing sequence", !anyFatal);
    }

    // =====================================================================
    // 6. RELATIVE MOVES ARE INDIFFERENT TO THE FRAME.
    //
    //    The other half of claim 3, and the one that would be silently broken
    //    by a "fix" that rebased displacements as well as targets.
    //    add_trapezoid_move_to_queue() takes a DISPLACEMENT
    //    (motor_control.c:2045) and never consults the origin, so the same
    //    relative command must deliver the same distance whether the machine
    //    is at zero, two rotations away, or freshly zeroed at that offset.
    //    Measuring all three in one section is what makes it a comparison
    //    rather than three independent accuracy checks.
    // =====================================================================
    printf("\n--- 6. relative moves ignore the frame ---\n");
    {
        tfhFreshOpen(m);
        const int32_t D = (int32_t)(TFH_CPR / 4);
        const uint32_t T = tfhTicksFor(D);

        int64_t atOrigin = 0, atOffset = 0, afterZero = 0, jump = 0;
        uint8_t f1 = legFatal(m, D, T, &atOrigin);
        uint8_t fj = legFatal(m, (int32_t)(TFH_CPR * 2), tfhTicksFor(TFH_CPR * 2), &jump);
        uint8_t f2 = legFatal(m, D, T, &atOffset);

        bool ans = false;
        uint8_t zf = zeroAndFatal(m, &ans);
        uint8_t f3 = legFatal(m, D, T, &afterZero);

        printf("   same relative move of %ld counts: at origin %lld, at +2 rot %lld, after zeroing %lld\n",
               (long)D, (long long)atOrigin, (long long)atOffset, (long long)afterZero);
        printf("   (jump fatal=%u, zero fatal=%u, leg fatals %u/%u/%u)\n",
               (unsigned)fj, (unsigned)zf, (unsigned)f1, (unsigned)f2, (unsigned)f3);

        TEST_RESULT("a relative move at the origin delivers its commanded displacement",
                    f1 == 0 && llabs((long long)(atOrigin - D)) <= MOVE_TOL);
        TEST_RESULT("the same relative move two rotations from the origin delivers the same distance",
                    fj == 0 && f2 == 0 && llabs((long long)(atOffset - D)) <= MOVE_TOL);
        TEST_RESULT("and the same again immediately after zeroing at that offset",
                    ans && zf == 0 && f3 == 0 &&
                    llabs((long long)(afterZero - D)) <= MOVE_TOL);
        TEST_RESULT("the three distances agree with each other to within a few counts",
                    llabs((long long)(atOrigin - atOffset)) <= 8 &&
                    llabs((long long)(atOrigin - afterZero)) <= 8);
    }

    // =====================================================================
    // 7. ZEROING DOES NOT TURN THE SHAFT -- AND IT RE-ARMS THE DEVIATION
    //    WATCHDOG.
    //
    //    zero_position() clears current_position_i96 and hall_position_i64 in
    //    the same critical section (motor_control.c:3319/:3329), so the
    //    difference the watchdog checks (:3217) becomes zero. With the output
    //    stage off the rotor never follows, which normally caps every test in
    //    this suite at the factory two-rotation window -- so the effect is
    //    directly observable: SIX rotations are commanded here under that
    //    default window, in four legs with a zeroing between them, and no
    //    fault occurs. The identical sequence without the zeroings must trip
    //    ERROR_POSITION_DEVIATION_TOO_LARGE, and that negative control is what
    //    stops the positive result being vacuous.
    //
    //    The window is left at its factory value throughout this section:
    //    tfhReset(), never tfhFreshOpen(). Only the speed and acceleration
    //    ceilings are opened, exactly as tm_deviation_tracking does, so the
    //    watchdog is the only thing under test.
    //
    //    The hall reading is sampled at the end of each leg, BEFORE the
    //    zeroing that would reset the counter, so it is a genuine measurement
    //    of whether the shaft moved rather than a reading of the zeroing's own
    //    side effect.
    // =====================================================================
    printf("\n--- 7. zeroing re-arms the deviation watchdog without turning the shaft ---\n");
    {
        const int32_t LEG = (int32_t)(TFH_CPR * 3 / 2);   // 1.5 rotations: inside 2
        const uint32_t LEG_T = 2 * (uint32_t)TFH_TPS;

        // Single leg first: it must complete, and the shaft must not turn.
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        int64_t hallStart = m->getHallSensorPositionRaw();
        int64_t moved = 0;
        uint8_t f = legFatal(m, LEG, LEG_T, &moved);
        int64_t hallAfterLeg = m->getHallSensorPositionRaw();
        printf("   one 1.5-rotation leg: moved=%lld fatal=%u hall %lld -> %lld\n",
               (long long)moved, (unsigned)f,
               (long long)hallStart, (long long)hallAfterLeg);
        TEST_RESULT("with the factory deviation window a 1.5-rotation move completes",
                    f == 0 && llabs((long long)(moved - LEG)) <= MOVE_TOL);
        TEST_RESULT("the shaft does not turn while it is commanded",
                    llabs((long long)(hallAfterLeg - hallStart)) < TFH_CPR / 50);

        // Four legs with a zeroing between them: six rotations commanded under
        // a two-rotation window.
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        int64_t commandedTotal = 0;
        int64_t worstHall = 0;
        bool allLegsClean = true;
        for (int leg = 0; leg < 4; leg++) {
            int64_t d = 0;
            uint8_t lf = legFatal(m, LEG, LEG_T, &d);
            int64_t hall = m->getHallSensorPositionRaw();
            if (llabs((long long)hall) > llabs((long long)worstHall)) worstHall = hall;
            if (lf != 0 || llabs((long long)(d - LEG)) > MOVE_TOL) {
                allLegsClean = false;
                printf("   leg %d failed: moved=%lld fatal=%u\n", leg, (long long)d, (unsigned)lf);
                break;
            }
            commandedTotal += d;
            bool ans = false;
            uint8_t zf = zeroAndFatal(m, &ans);
            if (!ans || zf != 0 || m->getPositionRaw() != 0) {
                allLegsClean = false;
                printf("   zeroing after leg %d failed: answered=%d fatal=%u\n",
                       leg, (int)ans, (unsigned)zf);
                break;
            }
        }
        printf("   four legs with zeroing between: total commanded travel %lld counts (%.2f rotations), worst hall %lld\n",
               (long long)commandedTotal, (double)commandedTotal / (double)TFH_CPR,
               (long long)worstHall);
        TEST_RESULT("four 1.5-rotation legs separated by zeroings all complete", allLegsClean);
        TEST_RESULT("six rotations are commanded under a two-rotation window without a fault",
                    allLegsClean && tfhFatal(m) == 0 &&
                    commandedTotal >= (int64_t)(TFH_CPR * 5.5));
        TEST_RESULT("and the shaft never left the origin: nothing physically turned",
                    llabs((long long)worstHall) < TFH_CPR / 50);

        // Negative control: the same two legs with NO zeroing between them
        // must trip the watchdog, or the result above proves nothing.
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        int64_t d1 = 0, d2 = 0;
        uint8_t nf1 = legFatal(m, LEG, LEG_T, &d1);
        uint8_t nf2 = legFatal(m, LEG, LEG_T, &d2);
        printf("   negative control (no zeroing): leg1 fatal=%u moved=%lld, leg2 fatal=%u\n",
               (unsigned)nf1, (long long)d1, (unsigned)nf2);
        TEST_RESULT("without a zeroing between them the same two legs trip error 45",
                    nf1 == 0 && nf2 == ERR_POSITION_DEVIATION_TOO_LARGE);

        tfhReset(m);
        TEST_RESULT("a system reset clears the deliberately tripped watchdog",
                    tfhFatal(m) == 0);
    }

    // Leave the machine clean for whatever module runs next.
    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0 && m->getPositionRaw() == 0);
}
