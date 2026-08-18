#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_recovery_sequences.cpp
//
// THE ORDER OF THE RECOVERY STEPS, NOT JUST THE STEPS.
//
// tm_error_taxonomy proves that ONE particular recovery works: fault, then
// System reset (cmd 27), then the machine is usable again. tm_fatal_state_commands
// proves WHICH commands a faulted motor still answers. Neither asks the question
// an operator actually faces at the machine:
//
//     I hit the emergency stop first. I reset twice because I was not sure the
//     first one took. I sat and read the status for a minute deciding what to
//     do. I walked away and came back. Does the motor still come back cleanly?
//
// That is a question about ORDERING, and orderings are exactly where recovery
// paths rot: a step that is harmless on its own can leave a flag, a queue entry
// or a stale limit behind that only bites when it precedes or follows another
// step. This module walks five different recovery sequences from the same
// latched fault and requires all five to land in the SAME fully-usable machine.
//
// GROUND TRUTH (why these five and not others). While a fatal error is latched
// the firmware is spinning inside fatal_error() with interrupts disabled and
// hand-pumping the UART (common_source_files/error_handling.c:169 onwards). The
// packet handler it calls in that state is a THREE-WAY switch
// (error_handling.c:82-110):
//
//     SYSTEM_RESET_COMMAND (27) -> replies, sets reset_requested, reboots  (:83)
//     GET_STATUS_COMMAND   (16) -> replies with the real status           (:92)
//     everything else           -> replies with the latched error code    (:104)
//
// So of the steps an operator would try, only reset and get_status do anything
// at all. Emergency stop (cmd 12), Disable MOSFETs (cmd 0) and Zero position
// (cmd 13) fall into the `default` branch: they are ANSWERED but never executed.
// That is the "fails cleanly rather than wedging" property section 3 asserts --
// the operator's instinctive first move is refused with a diagnosis instead of
// being silently swallowed or ignored into a timeout.
//
// HOW A FAULT IS LATCHED HERE, AND WHY THE ERROR REPLY IS PROOF. Every sequence
// starts from an over-speed trapezoid move: main.c:238 arms
// if_fatal_error_then_respond() before dispatching a non-broadcast command, and
// the queue-validation checks in add_to_queue() call fatal_error(), which then
// transmits the error packet from its own loop (error_handling.c:125-130). An
// error REPLY to that move is therefore direct evidence that fatal_error() ran
// -- which lets each sequence be timed from the fault with no probe in between,
// so "reset immediately with no delay" really means no delay.
//
// The exact path, traced rather than assumed. A 0.2 rotation/second ceiling
// against the factory acceleration makes max_velocity/max_acceleration truncate
// to zero, so compute_trapezoid_move()'s zero-denominator repair
// (motor_control.c:457) gives the ramp its minimum length of one tick, and
// queue_trapezoid_segments() (motor_control.c:2015) then emits the move as one
// held VELOCITY segment instead of three acceleration segments. That segment
// fails add_to_queue()'s velocity check (motor_control.c:1846) and the latched
// code is 16, ERROR_VEL_TOO_HIGH. The assertion below accepts 15/16/28 rather
// than pinning 16, because which of the three fires depends on how the limits
// are configured and all three mean the same thing here: a limit refused the
// move. NOTE this needs firmware carrying that zero-denominator repair
// (0.15.10.0 onwards; the rack is on 0.15.12.0). On 0.15.9.0 the same move
// divides by zero, yields acceleration 0 and is ACCEPTED silently, so the
// module reports failures there by design -- that is the old bug, not a test
// bug.
//
// WHAT COUNTS AS RECOVERED. Not "the status byte cleared". Every sequence must
// end with a REFERENCE MOVE completing correctly: one rotation in one second at
// the FACTORY DEFAULT limits. Planned out, that is a 24-tick ramp to a 1.0008
// rot/s coast against the 9.33 rot/s default ceiling, and its peak deviation of
// one rotation is half the two-rotation default deviation window, so it passes
// on a healthy machine -- and it is REFUSED outright if any
// of the crippled settings used to induce the fault survived the recovery
// (limits are pass/fail checks, never clamps). One move therefore measures
// usability AND the absence of residual limits at the same time.
//
// SAFETY: MOSFETs stay OFF throughout, and a fatal error force-disables them
// anyway. Every fault is raised at queue-validation time or by the deviation
// watchdog, so nothing turns; only the COMMANDED position is read, and the
// planner advances that regardless of the output stage. No broadcasts, no alias
// changes, no test modes: safe on the shared 35-motor rack. Every subsection
// ends with tfhReset(), and the module ends leaving the motor clean.
// ---------------------------------------------------------------------------

static const uint8_t ERR_ACCEL_TOO_HIGH      = 15;   // ERROR_ACCEL_TOO_HIGH
static const uint8_t ERR_VEL_TOO_HIGH        = 16;   // ERROR_VEL_TOO_HIGH
static const uint8_t ERR_PRED_VEL_TOO_HIGH   = 28;   // ERROR_PREDICTED_VELOCITY_TOO_HIGH

// "A long wait" from an operator's point of view: long enough that a fault which
// timed out, self-cleared or was garbage-collected would have done so by now.
static const uint32_t LONG_WAIT_MS = 8000;

// ---------------------------------------------------------------------------
// Latch a fatal error and read NOTHING back. Returns whether the move was
// refused, which (see the header) is proof that fatal_error() ran -- obtained
// without spending a single extra bus transaction or millisecond, so a sequence
// that must start "immediately" really can.
// ---------------------------------------------------------------------------
static bool armFault(Servomotor* m) {
    tfhReset(m);
    m->setMaximumVelocity(0.2f);                          // ceiling far below
    m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS / 4);   // 1 rotation in 0.25 s
    return (m->getError() != 0);
}

// ---------------------------------------------------------------------------
// Read the commanded position and the queue depth SAFELY. A faulted motor
// refuses both reads, and the library then returns a value that means nothing
// -- for the position, plausibly 0, which is exactly the value every check
// below treats as "recovered". A refused read must therefore be turned into
// something that FAILS every check, never into something that passes one.
// ---------------------------------------------------------------------------
static const int64_t POS_UNREADABLE = 1000000000LL;   // fails every "at the origin" test
static const uint8_t QUEUE_UNREADABLE = 0xFF;         // fails every "empty queue" test

static int64_t posOrSentinel(Servomotor* m) {
    int64_t p = m->getPositionRaw();
    return (m->getError() == 0) ? p : POS_UNREADABLE;
}

static uint8_t queueOrSentinel(Servomotor* m) {
    uint8_t q = m->getNQueuedItems();
    return (m->getError() == 0) ? q : QUEUE_UNREADABLE;
}

// The reference move described in the header: one rotation in one second at the
// factory defaults. Reports the commanded-position delta it achieved.
static bool referenceMove(Servomotor* m, int64_t* deltaOut) {
    *deltaOut = 0;
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) return false;
    m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS);
    if (m->getError() != 0) return false;                 // a surviving limit refuses it
    if (!tfhDrain(m, 12000)) return false;
    if (tfhFatal(m) != 0) return false;
    *deltaOut = m->getPositionRaw() - before;
    return llabs((long long)(*deltaOut - TFH_CPR)) <= TFH_CPR / 100;
}

// --- the five recovery sequences --------------------------------------------
// tfhReset() is the reset step: it sends System reset, waits out the 250 ms
// bootloader window with margin, drains the RX line and restores the LIBRARY's
// unit settings (which are host-side only and put no traffic on the bus).

static void recEstopThenReset(Servomotor* m) {
    m->emergencyStop();                    // refused while faulted; harmless
    tfhReset(m);
}

static void recResetThenReset(Servomotor* m) {
    tfhReset(m);
    tfhReset(m);                           // the "I am not sure the first took" case
}

static void recPollThenReset(Servomotor* m) {
    for (int i = 0; i < 20; i++) { m->getStatus(); delay(25); }
    tfhReset(m);
}

static void recWaitThenReset(Servomotor* m) {
    delay(LONG_WAIT_MS);                   // the "walked away and came back" case
    tfhReset(m);
}

static void recImmediateReset(Servomotor* m) {
    tfhReset(m);                           // nothing at all between fault and reset
}

// What one sequence produced.
struct SeqOutcome {
    bool     armed;      // the fault really was latched before the recovery ran
    uint8_t  fatalAfter;
    uint8_t  queueAfter;
    int64_t  posAfter;
    bool     moveOk;
    int64_t  moveDelta;
};

static SeqOutcome runSequence(Servomotor* m, const char* label,
                              void (*apply)(Servomotor*)) {
    SeqOutcome o;
    o.armed = armFault(m);
    apply(m);
    o.fatalAfter = tfhFatal(m);
    o.queueAfter = queueOrSentinel(m);
    o.posAfter   = posOrSentinel(m);
    o.moveOk     = referenceMove(m, &o.moveDelta);
    printf("   %-34s armed=%d fatal=%u queue=%u pos=%lld move=%d delta=%lld\n",
           label, (int)o.armed, (unsigned)o.fatalAfter, (unsigned)o.queueAfter,
           (long long)o.posAfter, (int)o.moveOk, (long long)o.moveDelta);
    return o;
}

void tm_recovery_sequences(void) {
    Serial.println("tm_recovery_sequences: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE PREMISE. A fault is inducible on demand, it reports a documented
    //    limit code, and it genuinely renders the machine unusable. Without
    //    this last part every "ends fully usable" verdict below would be
    //    satisfied by a machine that was never broken in the first place.
    // =====================================================================
    printf("\n--- 1. the fault is real and the machine is genuinely unusable ---\n");
    {
        bool armed = armFault(m);
        delay(300);
        uint8_t code = tfhFatal(m);

        // A move that a healthy machine performs happily must now be refused.
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        m->setTimeUnit(TimeUnit::TIMESTEPS);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), TFH_TPS);
        bool moveRefused = (m->getError() != 0);

        printf("   armed=%d fatal=%u, an ordinary move afterwards refused=%d\n",
               (int)armed, (unsigned)code, (int)moveRefused);
        TEST_RESULT("an over-speed move latches a fatal error and is answered with it",
                    armed && code != 0 && code != 0xFF);
        TEST_RESULT("the latched code is one of the documented limit codes (15/16/28)",
                    code == ERR_ACCEL_TOO_HIGH || code == ERR_VEL_TOO_HIGH ||
                    code == ERR_PRED_VEL_TOO_HIGH);
        TEST_RESULT("while faulted the machine refuses work it would normally accept",
                    moveRefused);
        tfhReset(m);
    }

    // =====================================================================
    // 2. FIVE RECOVERY ORDERINGS, ALL FROM THE SAME FAULT. Each is a sequence
    //    a real operator would try. The verdict for each is the same three
    //    things: the fault was really there, the sequence ends with a clean
    //    machine, and a full reference move completes correctly afterwards.
    //
    //    The interesting one is the FIRST. Reaching for the emergency stop is
    //    the reflex, and by the firmware's own three-way switch that command
    //    is refused while faulted -- so this asserts that the reflex is
    //    harmless and does not spoil the reset that follows it.
    // =====================================================================
    printf("\n--- 2. five recovery orderings from the same latched fault ---\n");
    SeqOutcome outcomes[5];
    {
        struct { const char* label; void (*apply)(Servomotor*); } seqs[5] = {
            { "emergency stop, then reset",   recEstopThenReset  },
            { "reset, then reset again",      recResetThenReset  },
            { "read the status 20x, then reset", recPollThenReset },
            { "wait 8 seconds, then reset",   recWaitThenReset   },
            { "reset immediately, no delay",  recImmediateReset  },
        };
        for (int i = 0; i < 5; i++) {
            outcomes[i] = runSequence(m, seqs[i].label, seqs[i].apply);
            TEST_RESULT(std::string(seqs[i].label) +
                            ": the fault really was latched before the recovery ran",
                        outcomes[i].armed);
            TEST_RESULT(std::string(seqs[i].label) +
                            ": ends with no latched fault, an empty queue and a zeroed position",
                        outcomes[i].fatalAfter == 0 && outcomes[i].queueAfter == 0 &&
                        llabs((long long)outcomes[i].posAfter) <= 2);
            TEST_RESULT(std::string(seqs[i].label) +
                            ": a full move at the factory limits completes correctly afterwards",
                        outcomes[i].moveOk);
            tfhReset(m);
        }
    }

    // =====================================================================
    // 3. WHAT DOES NOT WORK -- AND FAILS CLEANLY. The complement of section 2:
    //    the steps an operator might HOPE clear a fault, which do not. The
    //    requirement is not that they succeed; it is that they fail visibly
    //    and harmlessly:
    //
    //      * the step is REFUSED, with the error code as the diagnosis, rather
    //        than being silently swallowed or answered with a plausible lie
    //      * the ORIGINAL code is still latched afterwards, so the diagnosis
    //        is not destroyed by the attempt to fix it
    //      * the machine has not wedged -- get_status still answers, and a
    //        reset still works after every failed attempt in a row
    // =====================================================================
    printf("\n--- 3. steps that do NOT clear a fault fail cleanly ---\n");
    {
        const char* names[3] = { "emergency stop", "disable MOSFETs", "zero position" };
        for (int i = 0; i < 3; i++) {
            armFault(m);
            delay(300);
            uint8_t original = tfhFatal(m);
            switch (i) {
                case 0: m->emergencyStop();  break;
                case 1: m->disableMosfets(); break;
                case 2: m->zeroPosition();   break;
            }
            bool refused = (m->getError() != 0);
            getStatusResponse st = m->getStatus();
            bool stillAnswers = (m->getError() == 0);
            // `original != 0` is part of the claim, not decoration: without it a
            // machine that never faulted at all would satisfy
            // st.fatalErrorCode == original == 0 and this section would pass
            // vacuously for all three commands. 0xFF is tfhFatal()'s
            // read-failed sentinel and must never count as "the same code".
            bool armedHere = (original != 0) && (original != 0xFF);
            bool sameCode = stillAnswers && armedHere && (st.fatalErrorCode == original);
            printf("   %-16s -> refused=%d, status still answers=%d, code %u -> %u\n",
                   names[i], (int)refused, (int)stillAnswers,
                   (unsigned)original, (unsigned)st.fatalErrorCode);
            TEST_RESULT(std::string(names[i]) +
                            " does not clear the fault: the same code is still latched",
                        sameCode);
            TEST_RESULT(std::string(names[i]) +
                            " is refused with the error code and leaves the motor answering",
                        refused && stillAnswers);
            tfhReset(m);
        }

        // Waiting is not a recovery step either: a latched fault has no
        // timeout and no self-clearing path out of fatal_error()'s while(1).
        armFault(m);
        delay(300);
        uint8_t before = tfhFatal(m);
        delay(LONG_WAIT_MS);
        uint8_t after = tfhFatal(m);
        printf("   waiting %lu ms -> code %u -> %u\n",
               (unsigned long)LONG_WAIT_MS, (unsigned)before, (unsigned)after);
        TEST_RESULT("waiting does not clear a latched fault, however long you wait",
                    before != 0 && before != 0xFF && after == before);
        TEST_RESULT("the motor is still answering after the long wait", after != 0xFF);

        // And after ALL of those failed attempts stacked onto a single fault,
        // a reset still recovers. This is the "no wedging" claim at its
        // strongest: nothing tried above can poison the way out.
        armFault(m);
        delay(300);
        m->emergencyStop();
        m->disableMosfets();
        m->zeroPosition();
        for (int k = 0; k < 5; k++) m->getStatus();
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 100), TFH_TPS);
        tfhReset(m);
        int64_t d = 0;
        bool ok = (tfhFatal(m) == 0) && referenceMove(m, &d);
        printf("   after every failed attempt stacked on one fault: recovered=%d delta=%lld\n",
               (int)ok, (long long)d);
        TEST_RESULT("a reset still recovers after every failed attempt in a row", ok);
        tfhReset(m);
    }

    // =====================================================================
    // 4. RECOVERY IS IDEMPOTENT. Applying it again to a machine that has
    //    already recovered must be a no-op, not a disturbance -- an operator
    //    or a supervisory script will do exactly that. Checked in both
    //    directions: extra resets after a successful recovery, and repeated
    //    fault-and-recover rounds with no reset in between them.
    // =====================================================================
    printf("\n--- 4. recovery is idempotent ---\n");
    {
        armFault(m);
        tfhReset(m);
        uint8_t f1 = tfhFatal(m);
        uint8_t q1 = queueOrSentinel(m);
        int64_t p1 = posOrSentinel(m);

        tfhReset(m); tfhReset(m); tfhReset(m);      // three more, for luck
        uint8_t f2 = tfhFatal(m);
        uint8_t q2 = queueOrSentinel(m);
        int64_t p2 = posOrSentinel(m);

        int64_t d = 0;
        bool moveOk = referenceMove(m, &d);
        printf("   after 1 reset: fatal=%u queue=%u pos=%lld; after 4: fatal=%u queue=%u pos=%lld\n",
               (unsigned)f1, (unsigned)q1, (long long)p1,
               (unsigned)f2, (unsigned)q2, (long long)p2);
        TEST_RESULT("one reset already leaves a clean machine",
                    f1 == 0 && q1 == 0 && llabs((long long)p1) <= 2);
        // The commanded position is compared with the same +/-2 count window the
        // rest of the suite uses for "a reset returns the position to zero"
        // (tm_reset_completeness section 5), NOT with exact equality. Two
        // independent reads of a quantity that is only ever asserted to within
        // 2 counts must not be required to agree to the bit, or this assertion
        // is stricter than anything the firmware actually promises. The fatal
        // code and the queue depth ARE exact -- those are discrete.
        TEST_RESULT("three further resets produce exactly the same state as the first",
                    p1 != POS_UNREADABLE && p2 != POS_UNREADABLE &&
                    f2 == f1 && q2 == q1 &&
                    llabs((long long)(p2 - p1)) <= 2);
        TEST_RESULT("a real move still works after four consecutive resets", moveOk);

        // Fault, recover, fault, recover -- five rounds, nothing accumulating.
        bool allClean = true;
        for (int round = 0; round < 5 && allClean; round++) {
            if (!armFault(m)) { allClean = false; printf("   round %d did not fault\n", round); break; }
            tfhReset(m);
            if (tfhFatal(m) != 0)          { allClean = false; printf("   round %d did not clear\n", round); break; }
            if (m->getNQueuedItems() != 0) { allClean = false; printf("   round %d left the queue dirty\n", round); break; }
        }
        TEST_RESULT("five fault-and-recover rounds each fault and each clear", allClean);
        tfhReset(m);
    }

    // =====================================================================
    // 5. THE ORDERING DOES NOT CHANGE THE OUTCOME. Sections 2's five sequences
    //    must not merely each work -- they must all land in the SAME machine.
    //    A recovery path whose result depended on which order the operator
    //    happened to use would be unusable in a real procedure, and would make
    //    every other module in this suite order-dependent too, since they all
    //    open with a reset.
    // =====================================================================
    printf("\n--- 5. all five orderings land in the same machine ---\n");
    {
        bool sameState = true;
        // A sequence whose reference move never ran reports moveDelta 0. Five
        // zeroes have a spread of zero, so the distance comparison below would
        // PASS on a machine where nothing moved at all -- the accumulator would
        // be holding the empty sentinel, not a measurement. Require every move
        // to have actually happened before comparing the distances.
        bool allMoved = true;
        int64_t lo = outcomes[0].moveDelta, hi = outcomes[0].moveDelta;
        for (int i = 0; i < 5; i++) {
            if (outcomes[i].fatalAfter != 0 || outcomes[i].queueAfter != 0 ||
                llabs((long long)outcomes[i].posAfter) > 2 || !outcomes[i].moveOk) {
                sameState = false;
            }
            if (!outcomes[i].moveOk) allMoved = false;
            if (outcomes[i].moveDelta < lo) lo = outcomes[i].moveDelta;
            if (outcomes[i].moveDelta > hi) hi = outcomes[i].moveDelta;
        }
        printf("   reference-move deltas: %lld %lld %lld %lld %lld (spread %lld)\n",
               (long long)outcomes[0].moveDelta, (long long)outcomes[1].moveDelta,
               (long long)outcomes[2].moveDelta, (long long)outcomes[3].moveDelta,
               (long long)outcomes[4].moveDelta, (long long)(hi - lo));
        TEST_RESULT("all five orderings leave identical directly-readable state",
                    sameState);
        TEST_RESULT("all five orderings deliver the same reference-move distance",
                    allMoved && (hi - lo) <= TFH_CPR / 200);
    }

    // =====================================================================
    // 6. NO ORDERING LEAVES RESIDUE, EVEN FROM THE DIRTIEST POSSIBLE FAULT.
    //    Sections 2-5 fault a machine that was otherwise pristine. Here the
    //    machine is made as dirty as it can safely be made FIRST -- position
    //    driven two rotations from the origin, twelve queue slots occupied by
    //    moves still running, and the speed and acceleration ceilings both
    //    crippled -- and only then is it faulted, from inside the control loop
    //    rather than at queue-validation time, by shrinking the deviation
    //    window below the deviation the MOSFETs-off shaft has already
    //    accumulated (motor_control.c:3218 -> ERROR_POSITION_DEVIATION_TOO_LARGE).
    //
    //    The reference move afterwards is the residue check: it is refused if
    //    the speed ceiling, the acceleration ceiling OR the deviation window
    //    survived, so one move covers all three at once.
    // =====================================================================
    printf("\n--- 6. the dirtiest fault still leaves no residue ---\n");
    {
        tfhFreshOpen(m);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 2), 2 * TFH_TPS);   // wander from the origin
        tfhDrain(m, 15000);
        int64_t wandered = m->getPositionRaw();
        bool wanderedOk = (m->getError() == 0);

        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        for (int k = 0; k < 4; k++) m->trapezoidMoveRaw((int32_t)(TFH_CPR / 100), 8 * TFH_TPS);
        uint8_t depth = m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);

        // Fault from inside the control loop: the commanded position is two
        // rotations from a hall reading that never followed it, so a window of
        // a twentieth of a rotation is breached on the very next tick.
        m->setMaxAllowablePositionDeviationRaw(TFH_CPR / 20);
        delay(400);
        uint8_t code = tfhFatal(m);

        recEstopThenReset(m);                                       // the reflex ordering

        uint8_t fat = tfhFatal(m);
        uint8_t q   = queueOrSentinel(m);
        int64_t pos = posOrSentinel(m);
        int64_t d   = 0;
        bool moveOk = referenceMove(m, &d);

        printf("   dirty: pos %lld, queue %u, fault %u -> after recovery: fatal=%u queue=%u pos=%lld move=%d delta=%lld\n",
               (long long)wandered, (unsigned)depth, (unsigned)code,
               (unsigned)fat, (unsigned)q, (long long)pos, (int)moveOk, (long long)d);

        TEST_RESULT("the machine really was dirty and faulted before the recovery",
                    wanderedOk && depthOk &&
                    llabs((long long)wandered) > TFH_CPR && depth > 0 &&
                    code != 0 && code != 0xFF);
        TEST_RESULT("recovery returns the commanded position to the origin",
                    llabs((long long)pos) <= 2);
        TEST_RESULT("recovery empties the motion queue", q == 0);
        TEST_RESULT("recovery clears the latched fault", fat == 0);
        TEST_RESULT("the speed, acceleration and deviation limits are all back to factory defaults",
                    moveOk);
        tfhReset(m);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
