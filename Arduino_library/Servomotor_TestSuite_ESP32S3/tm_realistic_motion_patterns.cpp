#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_realistic_motion_patterns.cpp
//
// WHOLE MOTION PROGRAMS, END TO END.
//
// Every other motion module in this suite tests ONE primitive in isolation:
// tm_planner_arithmetic checks a single trapezoid's arithmetic, tm_queue_*
// check slot bookkeeping, tm_goto_absolute_semantics checks what one absolute
// target means, tm_velocity_move_semantics checks one velocity segment.
// tm_move_composition is the closest relative, but it checks ALGEBRAIC
// identities (D = d1+d2, +D then -D cancels, multimove == individual moves) --
// deliberately abstract, deliberately two or three moves long.
//
// Nothing in the suite runs a PROGRAM: the six- to nine-item sequence with
// dwells, speed changes, reversals and absolute waypoints that an actual
// machine executes every cycle of its working life. That is exactly where
// INTERACTION bugs live -- a planner baseline that is right for one move and
// stale for the next, a dwell that leaks a count, a queue hand-off that only
// misbehaves when a slow segment follows a fast one, an absolute waypoint that
// resolves against the wrong end of the queue. Each of those passes every
// single-primitive test and ruins every real program.
//
// Four programs are built here, each one a shape a customer would actually
// write, and each one has an EXACTLY COMPUTABLE endpoint:
//
//   (a) INDEXING   -- five equal forward steps with dwells between them.
//                     Ends at exactly 5 x step. Run again without the dwells:
//                     must land on the identical count and take 0.4 s less.
//   (b) SCAN       -- four out-and-back sweeps. Ends at exactly the origin,
//                     never past the outward limit, never behind the origin.
//   (c) PICK-AND-PLACE -- fast long move, slow short approach, dwell, slow
//                     retract, fast return. Ends at exactly the origin and
//                     takes exactly as long as its five phases add up to.
//   (d) SPEED RAMP -- five legs of identical displacement over shrinking
//                     durations. Ends at exactly 5 x leg, and each leg really
//                     does run quicker than the one before.
//
// Plus a fifth, mixed relative/absolute production sequence, because that is
// how real code is written: index relatively, then correct to an absolute
// waypoint, then return home absolutely.
//
// WHY THE ENDPOINTS ARE EXACT, and how tight the assertions can honestly be:
//   * A symmetric program (+d over T, then -d over T) cancels to the COUNT.
//     compute_trapezoid_move (motor_control.c:485) derives the acceleration by
//     C integer division, which truncates toward zero, so the -d leg's
//     acceleration is the exact negative of the +d leg's whenever both legs are
//     given the same duration and the same ceilings. The runtime integration is
//     a pure integer recurrence (v += a; p += v, motor_control.c:2129 and 2191)
//     and every trapezoid both starts and ends at exactly zero velocity
//     (a*t1 + 0 - a*t1), so the reverse leg subtracts precisely what the
//     forward leg added, fractional bits included. Programs (b) and (c) pair
//     every leg with a same-duration opposite leg, so they are asserted at zero
//     residue exactly. (tm_soak and tm_move_composition assert the same
//     property with slack; the slack there is defensive, not measured.)
//   * A net-displacement program loses that truncation instead of cancelling
//     it -- at most about one count per move (measured repeatedly during this
//     campaign: 409599 of a commanded 409600). Programs (a), (d) and the mixed
//     sequence are therefore asserted within rmpTol() = 4 counts per move + 8,
//     which is under 0.002% of the distances used here and still an order of
//     magnitude smaller than any dropped or duplicated segment.
//
// PRIMITIVE CHOICE IS DELIBERATE: every program is built from trapezoid moves
// and go-to-position moves only. Both self-terminate (queue_trapezoid_segments,
// motor_control.c:2014, always ends at zero velocity). A raw velocity segment
// HOLDS its velocity after expiry and would underrun the queue with error 18,
// which is a property of that primitive, not of program composition, and is
// already covered by tm_queue_underrun and tm_velocity_move_semantics.
//
// SAFETY: THE MOSFETs STAY OFF for the whole module. Every measurement reads
// the COMMANDED position (getPositionRaw), which the planner advances whether
// or not the output stage is energised, so nothing physically turns on any
// motor at any point. No enableMosfets, no goToClosedLoop, no homing, no
// calibration. No broadcasts and no alias changes, so this is safe on the
// shared 35-motor rack. The largest excursion any program makes is 0.52
// rotations, and tfhFreshOpen raises the deviation window anyway so the
// MOSFETs-off two-rotation ceiling (error 45) is never in play. Every limit is
// left far above what the programs ask for, since limits here are pass/fail
// checks and never clamps -- an over-ambitious move is REFUSED (error 15/16/28),
// which would make the module measure the ceiling instead of the program.
// Each subsection ends with a reset, and the module ends with a clean motor.
// ---------------------------------------------------------------------------

// Working ceilings applied by tfhFreshOpen: 15 rot/s and 5000 rot/s^2. Those
// give compute_trapezoid_move a ramp of delta_t1 = max_velocity/max_acceleration
// = 93 control ticks (motor_control.c:449), comfortably above 1, so every move
// in this module takes the ordinary three-segment path and costs 3 queue slots.
// The one-tick-ramp two-slot fast path (motor_control.c:2017) is a separate
// regime with its own module (tm_queue_accounting) and is avoided here on
// purpose: the programs below are about composition, not about which path a
// single move takes. Slot budgets: the biggest program is 9 items = 27 slots,
// inside MOVEMENT_QUEUE_SIZE = 32 (motor_control.h:31).
static const int SLOTS_PER_MOVE = 3;

static int64_t rmpAbs64(int64_t v) { return (v < 0) ? -v : v; }

// Tolerance for a program that ends with a NET displacement: the acceleration
// truncation described above costs up to about one count per move and does not
// cancel. Four counts per move plus eight is generous against that and still
// far tighter than any structural error could be.
static int64_t rmpTol(int nMoves) { return 4 * (int64_t)nMoves + 8; }

// Milliseconds a program of `ticks` control ticks should take.
static uint32_t rmpMsFor(uint32_t ticks) {
    return (uint32_t)((uint64_t)ticks * 1000ULL / (uint64_t)TFH_TPS);
}

// What one run of a program measured.
struct RmpRun {
    bool     drained;    // the queue emptied inside the budget
    bool     monotonic;  // the commanded position never went backwards
    uint32_t elapsedMs;  // from the caller's t0 (see below) to an empty queue
    int64_t  lo;         // lowest commanded position seen, relative to origin
    int64_t  hi;         // highest commanded position seen, relative to origin
};

// ---------------------------------------------------------------------------
// Watch an already-queued program to completion.
//
// This polls the queue depth rather than waiting a computed duration. A fixed
// wait that is even slightly short reads a PARTIAL program and reports a
// distance shorter than commanded, which is indistinguishable from a planner
// bug -- that exact mistake was the first false finding of this campaign.
//
// It also tracks the extremes of the commanded position along the way, which
// is what makes "the scan never travels past its outward limit" and "the
// pick-and-place never overshoots the place point" checkable at all. Sampling
// costs two commands per iteration, so the trajectory is sampled at roughly
// 14 ms; that is fine for endpoint bounds and is accounted for in the
// tolerances that use `hi`.
//
// `startedMs` is the caller's clock reading from BEFORE the FIRST enqueue,
// which is when motion actually begins. Timing from there rather than from the
// end of the batch is what makes the duration assertions honest: the later
// enqueues overlap with a queue that is already running, so they add nothing
// to the total, whereas timing from the end of the batch would SUBTRACT the
// send time from the measurement and make every program look quicker than it
// was. The measured figure can therefore only be late (by at most one poll
// period), never early -- so a lower bound at the commanded duration is safe.
// ---------------------------------------------------------------------------
static void rmpWatch(Servomotor* m, int64_t origin, uint32_t startedMs,
                     uint32_t budgetMs, RmpRun* r) {
    r->drained   = false;
    r->monotonic = true;
    r->elapsedMs = 0;
    r->lo = 0;
    r->hi = 0;

    int64_t prev = m->getPositionRaw() - origin;
    if (m->getError() == 0) { r->lo = prev; r->hi = prev; }

    uint32_t deadline = millis() + budgetMs;
    while (millis() < deadline) {
        int64_t p = m->getPositionRaw() - origin;
        if (m->getError() != 0) break;              // a fault refuses reads
        if (p < prev) r->monotonic = false;
        if (p < r->lo) r->lo = p;
        if (p > r->hi) r->hi = p;
        prev = p;

        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) break;
        if (q == 0) { r->drained = true; r->elapsedMs = millis() - startedMs; break; }
        delay(10);
    }
    if (r->drained) delay(150);                     // let the last tick settle
}

// Queue one trapezoid move; false if the command itself was refused.
static bool rmpTrap(Servomotor* m, int32_t counts, uint32_t ticks) {
    m->trapezoidMoveRaw(counts, ticks);
    return m->getError() == 0;
}

// Queue one absolute go-to; false if the command itself was refused.
static bool rmpGoto(Servomotor* m, int64_t absolutePosition, uint32_t ticks) {
    m->goToPositionRaw((int32_t)absolutePosition, ticks);
    return m->getError() == 0;
}

void tm_realistic_motion_patterns(void) {
    Serial.println("tm_realistic_motion_patterns: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 0. PREFLIGHT. Establish the premise the whole module rests on: the
    //    output stage is off and nothing is latched. A reset already leaves
    //    the MOSFETs disabled, but this asks for it explicitly and checks,
    //    because every "safe on the rack" claim below depends on it.
    // =====================================================================
    printf("\n--- 0. preflight: the output stage is off and the machine is clean ---\n");
    {
        tfhFreshOpen(m);
        m->disableMosfets();
        bool offOk = (m->getError() == 0);
        delay(200);
        uint8_t fat = tfhFatal(m);
        uint8_t depth = m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);
        printf("   disableMosfets accepted=%d, fatal=%u, resting queue depth=%u\n",
               (int)offOk, (unsigned)fat, (unsigned)depth);
        TEST_RESULT("the module starts on a clean machine with the output stage off",
                    offOk && fat == 0 && depthOk && depth == 0);
    }

    // =====================================================================
    // 1. AN INDEXING PATTERN (a): five equal forward steps with a dwell
    //    between each pair. This is the single most common real program --
    //    a rotary index table, a filler, a capper. Two things must be true
    //    and neither is checked anywhere else:
    //
    //      * the whole nine-item program is accepted as ONE queue load and
    //        lands on exactly five step lengths;
    //      * the dwells cost TIME and nothing else. A dwell is a trapezoid
    //        move of zero displacement, so compute_trapezoid_move yields
    //        acceleration 0 (motor_control.c:485 with delta_d = 0) and the
    //        three segments occupy their ticks without moving. If a dwell
    //        leaked even one count, an indexer would walk away from its
    //        stations over a shift.
    //
    //    The dwell-free run is the control: same five steps, no dwells. It
    //    must land on the IDENTICAL count, not merely a nearby one.
    // =====================================================================
    printf("\n--- 1. indexing: five steps with dwells between them ---\n");
    {
        const int32_t  STEP        = (int32_t)(TFH_CPR / 10);       // 327680 counts = 0.1 rot
        const uint32_t STEP_TICKS  = (uint32_t)(TFH_TPS * 2 / 5);   // 12500 ticks = 0.4 s
        const uint32_t DWELL_TICKS = (uint32_t)(TFH_TPS / 10);      // 3125 ticks = 0.1 s
        const int      STEPS       = 5;
        const int64_t  EXPECTED    = (int64_t)STEP * STEPS;         // 1638400 counts
        const int      ITEMS_A     = STEPS + (STEPS - 1);           // 9 items, 27 slots
        const uint32_t TICKS_A     = STEP_TICKS * STEPS + DWELL_TICKS * (STEPS - 1);
        const uint32_t TICKS_B     = STEP_TICKS * STEPS;

        // --- run A: with dwells -------------------------------------------
        tfhFreshOpen(m);
        int64_t originA = m->getPositionRaw();
        bool acceptedA = (m->getError() == 0);
        uint32_t t0A = millis();                 // motion begins at the first enqueue
        for (int i = 0; i < STEPS && acceptedA; i++) {
            acceptedA = rmpTrap(m, STEP, STEP_TICKS);
            if (acceptedA && i < STEPS - 1) acceptedA = rmpTrap(m, 0, DWELL_TICKS);
        }
        uint8_t depthA = m->getNQueuedItems();
        if (m->getError() != 0) depthA = 0;

        RmpRun runA;
        rmpWatch(m, originA, t0A, rmpMsFor(TICKS_A) + 12000, &runA);
        uint8_t fatA = tfhFatal(m);
        uint8_t restA = m->getNQueuedItems();
        bool restAok = (m->getError() == 0);
        int64_t movedA = m->getPositionRaw() - originA;

        // --- run B: the same steps, no dwells -----------------------------
        tfhFreshOpen(m);
        int64_t originB = m->getPositionRaw();
        bool acceptedB = (m->getError() == 0);
        uint32_t t0B = millis();
        for (int i = 0; i < STEPS && acceptedB; i++) acceptedB = rmpTrap(m, STEP, STEP_TICKS);

        RmpRun runB;
        rmpWatch(m, originB, t0B, rmpMsFor(TICKS_B) + 12000, &runB);
        uint8_t fatB = tfhFatal(m);
        int64_t movedB = m->getPositionRaw() - originB;

        printf("   with dwells:    accepted=%d depth=%u moved=%lld (expected %lld) "
               "fatal=%u elapsed=%lums monotonic=%d\n",
               (int)acceptedA, (unsigned)depthA, (long long)movedA, (long long)EXPECTED,
               (unsigned)fatA, (unsigned long)runA.elapsedMs, (int)runA.monotonic);
        printf("   without dwells: accepted=%d moved=%lld fatal=%u elapsed=%lums\n",
               (int)acceptedB, (long long)movedB, (unsigned)fatB,
               (unsigned long)runB.elapsedMs);
        printf("   program time predicted %lums with dwells, %lums without; "
               "dwells should add %lums\n",
               (unsigned long)rmpMsFor(TICKS_A), (unsigned long)rmpMsFor(TICKS_B),
               (unsigned long)rmpMsFor(DWELL_TICKS * (STEPS - 1)));

        TEST_RESULT("the whole nine-item indexing program is accepted without a refusal",
                    acceptedA && runA.drained);
        TEST_RESULT("the indexing program leaves the queue empty when it finishes",
                    restAok && restA == 0);
        TEST_RESULT("the indexing program raises no fatal error", fatA == 0);
        TEST_RESULT("the indexing program lands on exactly five step lengths",
                    rmpAbs64(movedA - EXPECTED) <= rmpTol(ITEMS_A));
        TEST_RESULT("a forward-only index never runs the commanded position backwards",
                    runA.monotonic && runA.lo >= -rmpTol(ITEMS_A));
        // The whole program really was resident in the queue at once, rather
        // than the motor having executed it move by move as it arrived.
        TEST_RESULT("the indexing program is resident in the motion queue as one batch",
                    depthA >= 12 && depthA <= (uint8_t)(ITEMS_A * SLOTS_PER_MOVE));
        // The control: dwells must not shift the endpoint by a single count.
        TEST_RESULT("removing the dwells lands the index on the identical count",
                    acceptedB && runB.drained && fatB == 0 && movedA == movedB);
        // And they must cost the time they were given, near enough to see.
        // Both runs are timed from their first enqueue, so the difference is
        // the four dwells and nothing else: nominally 400 ms.
        // The window is wide on purpose. Each run's finish is detected to
        // within one poll period, so the measured DIFFERENCE carries up to one
        // poll period of error in either direction -- ~14 ms on the ESP32-S3,
        // but up to ~70 ms in host mode where USB and scheduler jitter
        // dominate. [200, 800] tolerates a poll period up to 200 ms and still
        // fails loudly if the dwells cost nothing (0 ms) or cost the wrong
        // order of magnitude.
        TEST_RESULT("the dwells cost the running time they were given and nothing else",
                    runA.drained && runB.drained &&
                    (runA.elapsedMs > runB.elapsedMs + 200) &&
                    (runA.elapsedMs < runB.elapsedMs + 800));
        tfhReset(m);
    }

    // =====================================================================
    // 2. A SCAN (b): out, back, repeat. A scanner, a wiper, a reciprocating
    //    axis. The endpoint is the strongest assertion available anywhere in
    //    this module -- an out-and-back is exactly reversible in integer
    //    arithmetic, so after four cycles the residue must be ZERO COUNTS,
    //    not "close to zero". Any asymmetry in the planner (a rounding mode
    //    that is not truncate-toward-zero, a sign handled differently on the
    //    reverse leg, a baseline updated only on one side) shows up here as a
    //    nonzero residue that no single-move test would ever reveal.
    //
    //    The trajectory bounds matter as much as the endpoint: a scan that
    //    reaches the right place by overshooting and coming back would fail a
    //    real machine (it crashes into the end stop) while passing an
    //    endpoint-only test. So the outward limit and the origin are asserted
    //    as bounds on the whole path, not just on the finish.
    // =====================================================================
    printf("\n--- 2. scan: four out-and-back sweeps returning to the origin ---\n");
    {
        const int32_t  LEG       = (int32_t)(TFH_CPR / 4);         // 819200 counts = 0.25 rot
        const uint32_t LEG_TICKS = (uint32_t)(TFH_TPS * 2 / 5);    // 12500 ticks = 0.4 s
        const int      CYCLES    = 4;                              // 8 legs, 24 slots
        const uint32_t TICKS_S   = LEG_TICKS * 2 * CYCLES;

        tfhFreshOpen(m);
        int64_t origin = m->getPositionRaw();
        bool accepted = (m->getError() == 0);
        uint32_t t0 = millis();
        for (int c = 0; c < CYCLES && accepted; c++) {
            accepted = rmpTrap(m, LEG, LEG_TICKS);
            if (accepted) accepted = rmpTrap(m, -LEG, LEG_TICKS);
        }
        RmpRun run;
        rmpWatch(m, origin, t0, rmpMsFor(TICKS_S) + 12000, &run);
        uint8_t fat = tfhFatal(m);
        uint8_t rest = m->getNQueuedItems();
        bool restOk = (m->getError() == 0);
        int64_t residue = m->getPositionRaw() - origin;

        printf("   scan: accepted=%d residue=%lld counts, path spanned %lld..%lld "
               "(outward limit %ld), fatal=%u elapsed=%lums (predicted %lums)\n",
               (int)accepted, (long long)residue, (long long)run.lo, (long long)run.hi,
               (long)LEG, (unsigned)fat,
               (unsigned long)run.elapsedMs, (unsigned long)rmpMsFor(TICKS_S));

        TEST_RESULT("the whole eight-leg scan program is accepted without a refusal",
                    accepted && run.drained);
        TEST_RESULT("the scan program leaves the queue empty when it finishes",
                    restOk && rest == 0);
        TEST_RESULT("the scan program raises no fatal error", fat == 0);
        // Exact, and justified: the +leg and -leg truncations are mirror images.
        TEST_RESULT("four out-and-back sweeps return to the origin with zero residue",
                    residue == 0);
        // The path bounds. Reaching the limit is SAMPLING-LIMITED and is the
        // one assertion in this module that a slow link could break, so it is
        // deliberately loose. The scan coasts at about 66 counts/tick, so the
        // trajectory stays within LEG/10 of the peak for only ~80 ms; a poll
        // iteration is two commands plus delay(10), which is ~14 ms on the
        // ESP32-S3 but can reach 70 ms in host mode where a USB-serial
        // adapter's latency timer dominates. At LEG/10 that leaves barely one
        // sample of margin, so the bound is relaxed to LEG/4 (a ~200 ms
        // window, guaranteed to catch a sample at any plausible poll rate).
        // It still proves the axis really swept out; the ZERO-residue and
        // never-past-the-limit assertions carry the precision.
        // EXCEEDING the limit stays asserted tightly, because that is the
        // failure that would wreck a machine and sampling can only
        // under-report a peak, never over-report it.
        TEST_RESULT("the scan actually reaches its outward limit",
                    run.hi >= (int64_t)LEG - (int64_t)LEG / 4);
        TEST_RESULT("the scan never travels past its outward limit",
                    run.hi <= (int64_t)LEG + rmpTol(CYCLES * 2));
        TEST_RESULT("the scan never travels behind its origin",
                    run.lo >= -rmpTol(CYCLES * 2));

        // Three whole scans back to back, with no reset between them: the
        // residue must still be zero, so nothing accumulates across programs.
        tfhFreshOpen(m);
        int64_t origin3 = m->getPositionRaw();
        bool ok3 = (m->getError() == 0);
        for (int rep = 0; rep < 3 && ok3; rep++) {
            for (int c = 0; c < CYCLES && ok3; c++) {
                ok3 = rmpTrap(m, LEG, LEG_TICKS);
                if (ok3) ok3 = rmpTrap(m, -LEG, LEG_TICKS);
            }
            if (ok3) ok3 = tfhDrain(m, rmpMsFor(TICKS_S) + 12000);
            if (ok3) ok3 = (tfhFatal(m) == 0);
        }
        int64_t residue3 = m->getPositionRaw() - origin3;
        printf("   three scans back to back: cumulative residue %lld counts (ok=%d)\n",
               (long long)residue3, (int)ok3);
        TEST_RESULT("three scans run back to back leave zero cumulative residue",
                    ok3 && residue3 == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 3. PICK AND PLACE (c): the classic five-phase shape -- fast long move,
    //    slow short approach, dwell at the work point, slow retract, fast
    //    return. It is here because it mixes SPEEDS inside one queue load,
    //    which the other programs do not: a 1 rot/s phase is followed
    //    immediately by a 0.04 rot/s phase, so the planner has to hand over
    //    between two very different segment shapes with no gap. A hand-off
    //    that lost or duplicated a tick would show up as a nonzero residue.
    //
    //    The duration is asserted as well as the endpoint. Real programs are
    //    written to a cycle time, and a machine that lands correctly but takes
    //    a different length of time is still broken. The phase durations are
    //    chosen so that dropping ANY single phase would put the total outside
    //    the accepted window (2400 ms nominal; the shortest phase is 400 ms).
    // =====================================================================
    printf("\n--- 3. pick and place: fast move, slow approach, dwell, retract, return ---\n");
    {
        const int32_t  LONG_MOVE      = (int32_t)(TFH_CPR / 2);      // 1638400 = 0.5 rot, fast
        const uint32_t LONG_TICKS     = (uint32_t)(TFH_TPS / 2);     // 15625 ticks = 0.5 s
        const int32_t  APPROACH       = (int32_t)(TFH_CPR / 50);     // 65536 = 0.02 rot, slow
        const uint32_t APPROACH_TICKS = (uint32_t)(TFH_TPS / 2);     // 15625 ticks = 0.5 s
        const uint32_t DWELL_TICKS    = (uint32_t)(TFH_TPS * 2 / 5); // 12500 ticks = 0.4 s
        const int64_t  PLACE_POINT    = (int64_t)LONG_MOVE + (int64_t)APPROACH;
        const uint32_t TICKS_P        = LONG_TICKS * 2 + APPROACH_TICKS * 2 + DWELL_TICKS;
        const int      ITEMS_P        = 5;                           // 15 slots

        tfhFreshOpen(m);
        int64_t origin = m->getPositionRaw();
        bool accepted = (m->getError() == 0);
        uint32_t t0 = millis();
        if (accepted) accepted = rmpTrap(m,  LONG_MOVE, LONG_TICKS);      // fast approach
        if (accepted) accepted = rmpTrap(m,  APPROACH,  APPROACH_TICKS);  // slow final approach
        if (accepted) accepted = rmpTrap(m,  0,         DWELL_TICKS);     // dwell on the part
        if (accepted) accepted = rmpTrap(m, -APPROACH,  APPROACH_TICKS);  // slow retract
        if (accepted) accepted = rmpTrap(m, -LONG_MOVE, LONG_TICKS);      // fast return

        RmpRun run;
        rmpWatch(m, origin, t0, rmpMsFor(TICKS_P) + 12000, &run);
        uint8_t fat = tfhFatal(m);
        uint8_t rest = m->getNQueuedItems();
        bool restOk = (m->getError() == 0);
        int64_t residue = m->getPositionRaw() - origin;
        uint32_t predictedMs = rmpMsFor(TICKS_P);

        printf("   pick and place: accepted=%d residue=%lld, reached %lld of place point "
               "%lld, fatal=%u elapsed=%lums (predicted %lums)\n",
               (int)accepted, (long long)residue, (long long)run.hi,
               (long long)PLACE_POINT, (unsigned)fat,
               (unsigned long)run.elapsedMs, (unsigned long)predictedMs);

        TEST_RESULT("the whole five-phase pick-and-place program is accepted without a refusal",
                    accepted && run.drained);
        TEST_RESULT("the pick-and-place program leaves the queue empty when it finishes",
                    restOk && rest == 0);
        TEST_RESULT("the pick-and-place program raises no fatal error", fat == 0);
        TEST_RESULT("a pick-and-place cycle returns to its start with zero residue",
                    residue == 0);
        TEST_RESULT("the pick-and-place reaches the place point at the end of its slow approach",
                    run.hi >= PLACE_POINT - PLACE_POINT / 10);
        TEST_RESULT("the pick-and-place never overshoots the place point",
                    run.hi <= PLACE_POINT + rmpTol(ITEMS_P));
        // The cycle time. Timed from the first enqueue, so the measurement can
        // only run LATE (by at most one poll period), never early: the lower
        // bound sits just under the commanded 2400 ms and is safe. Dropping any
        // one phase would remove at least 400 ms and break it.
        TEST_RESULT("the pick-and-place takes as long as its five phases add up to",
                    run.drained &&
                    run.elapsedMs > predictedMs - 100 &&
                    run.elapsedMs < predictedMs + 800);

        // Three cycles with no reset between them: a machine runs this shape
        // thousands of times a shift, so the per-cycle residue must not creep.
        tfhFreshOpen(m);
        int64_t origin3 = m->getPositionRaw();
        bool ok3 = (m->getError() == 0);
        for (int rep = 0; rep < 3 && ok3; rep++) {
            ok3 = rmpTrap(m,  LONG_MOVE, LONG_TICKS);
            if (ok3) ok3 = rmpTrap(m,  APPROACH,  APPROACH_TICKS);
            if (ok3) ok3 = rmpTrap(m,  0,         DWELL_TICKS);
            if (ok3) ok3 = rmpTrap(m, -APPROACH,  APPROACH_TICKS);
            if (ok3) ok3 = rmpTrap(m, -LONG_MOVE, LONG_TICKS);
            if (ok3) ok3 = tfhDrain(m, rmpMsFor(TICKS_P) + 12000);
            if (ok3) ok3 = (tfhFatal(m) == 0);
        }
        int64_t residue3 = m->getPositionRaw() - origin3;
        printf("   three pick-and-place cycles: cumulative residue %lld counts (ok=%d)\n",
               (long long)residue3, (int)ok3);
        TEST_RESULT("three pick-and-place cycles leave zero cumulative residue",
                    ok3 && residue3 == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 4. A SPEED RAMP (d): five legs of IDENTICAL displacement over shrinking
    //    durations, so each leg runs faster than the last. This is how a real
    //    program feeds material in gently and then gets on with it, and it is
    //    the one shape that varies the segment geometry from item to item
    //    within a single queue load: compute_trapezoid_move produces a
    //    different acceleration and a different coast length for every leg
    //    (motor_control.c:445), while the ramp length delta_t1 stays fixed at
    //    93 ticks. If the planner carried anything over between items -- a
    //    stale coast length, a velocity baseline -- the legs would not all
    //    deliver the same distance.
    //
    //    Two runs. First the whole ramp as one program, to assert the total.
    //    Then the same five legs one at a time, to assert that the speeds
    //    really did increase: the endpoint alone cannot tell a ramp from five
    //    identical moves, so the elapsed time of each leg is measured.
    // =====================================================================
    printf("\n--- 4. speed ramp: five equal legs over shrinking durations ---\n");
    {
        const int32_t  RAMP_LEG = (int32_t)(TFH_CPR / 10);   // 327680 counts = 0.1 rot
        const uint32_t RAMP_TICKS[5] = { 50000, 37500, 25000, 15625, 7812 };
        const int      LEGS     = 5;                          // 15 slots
        const int64_t  EXPECTED = (int64_t)RAMP_LEG * LEGS;   // 1638400 counts
        uint32_t totalTicks = 0;
        for (int i = 0; i < LEGS; i++) totalTicks += RAMP_TICKS[i];

        // --- the ramp as one queued program -------------------------------
        tfhFreshOpen(m);
        int64_t origin = m->getPositionRaw();
        bool accepted = (m->getError() == 0);
        uint32_t t0 = millis();
        for (int i = 0; i < LEGS && accepted; i++) accepted = rmpTrap(m, RAMP_LEG, RAMP_TICKS[i]);

        RmpRun run;
        rmpWatch(m, origin, t0, rmpMsFor(totalTicks) + 15000, &run);
        uint8_t fat = tfhFatal(m);
        uint8_t rest = m->getNQueuedItems();
        bool restOk = (m->getError() == 0);
        int64_t moved = m->getPositionRaw() - origin;

        printf("   ramp as one program: accepted=%d moved=%lld (expected %lld) fatal=%u "
               "elapsed=%lums (predicted %lums)\n",
               (int)accepted, (long long)moved, (long long)EXPECTED, (unsigned)fat,
               (unsigned long)run.elapsedMs, (unsigned long)rmpMsFor(totalTicks));

        TEST_RESULT("the whole five-leg speed ramp is accepted without a refusal",
                    accepted && run.drained);
        TEST_RESULT("the speed-ramp program leaves the queue empty when it finishes",
                    restOk && rest == 0);
        TEST_RESULT("the speed-ramp program raises no fatal error", fat == 0);
        TEST_RESULT("the speed ramp lands on exactly five leg displacements",
                    rmpAbs64(moved - EXPECTED) <= rmpTol(LEGS));

        // --- the same five legs, timed individually -----------------------
        tfhFreshOpen(m);
        int64_t legMoved[5];
        uint32_t legMs[5];
        bool legsOk = true;
        for (int i = 0; i < LEGS && legsOk; i++) {
            int64_t before = m->getPositionRaw();
            if (m->getError() != 0) { legsOk = false; break; }
            uint32_t legT0 = millis();
            if (!rmpTrap(m, RAMP_LEG, RAMP_TICKS[i])) { legsOk = false; break; }
            RmpRun legRun;
            rmpWatch(m, before, legT0, rmpMsFor(RAMP_TICKS[i]) + 10000, &legRun);
            if (!legRun.drained || tfhFatal(m) != 0) { legsOk = false; break; }
            legMoved[i] = m->getPositionRaw() - before;
            legMs[i]    = legRun.elapsedMs;
        }

        bool sameDistance = legsOk;
        bool strictlyQuicker = legsOk;
        int64_t widest = 0;
        if (legsOk) {
            for (int i = 0; i < LEGS; i++) {
                if (rmpAbs64(legMoved[i] - (int64_t)RAMP_LEG) > rmpTol(1)) sameDistance = false;
                for (int j = 0; j < LEGS; j++) {
                    int64_t d = rmpAbs64(legMoved[i] - legMoved[j]);
                    if (d > widest) widest = d;
                }
                if (i > 0 && legMs[i] >= legMs[i - 1]) strictlyQuicker = false;
            }
            printf("   legs: ");
            for (int i = 0; i < LEGS; i++) {
                printf("%lu ticks -> %lld counts in %lums   ",
                       (unsigned long)RAMP_TICKS[i], (long long)legMoved[i],
                       (unsigned long)legMs[i]);
            }
            printf("\n   widest spread between legs: %lld counts\n", (long long)widest);
        }

        TEST_RESULT("every leg of the ramp delivers the same displacement whatever its speed",
                    legsOk && sameDistance && widest <= rmpTol(1));
        TEST_RESULT("each successive leg of the ramp really does run quicker",
                    legsOk && strictlyQuicker);
        TEST_RESULT("the fastest leg of the ramp is at least three times quicker than the slowest",
                    legsOk && legMs[LEGS - 1] > 0 && legMs[0] >= legMs[LEGS - 1] * 3);
        tfhReset(m);
    }

    // =====================================================================
    // 5. A MIXED RELATIVE/ABSOLUTE PRODUCTION SEQUENCE. Real programs are not
    //    written purely in one frame: they index RELATIVELY (three more
    //    parts) and then correct ABSOLUTELY (back to the fixture datum). The
    //    interaction is subtle, because add_go_to_position_to_queue resolves
    //    its target against position_after_last_queue_item
    //    (motor_control.c:2070) -- the planner's prediction of where the queue
    //    ENDS -- and not against the live position. So an absolute waypoint
    //    queued BEHIND three relative moves that have not run yet must still
    //    land on the waypoint.
    //
    //    That is the property that makes an absolute waypoint useful at all:
    //    it CORRECTS whatever the relative moves ahead of it accumulated,
    //    rather than adding to it. It is asserted here inside a program, with
    //    all four moves queued as one batch before any of them executes.
    // =====================================================================
    printf("\n--- 5. a mixed relative/absolute production sequence ---\n");
    {
        const int32_t  APPROACH = (int32_t)(TFH_CPR / 5);    // 655360 counts = 0.2 rot
        const int32_t  INDEX    = (int32_t)(TFH_CPR / 20);   // 163840 counts = 0.05 rot
        const int32_t  RETRACT  = (int32_t)(TFH_CPR / 4);    // 819200 counts = 0.25 rot
        const uint32_t T        = (uint32_t)(TFH_TPS / 2);   // 15625 ticks = 0.5 s

        tfhFreshOpen(m);
        int64_t home = m->getPositionRaw();
        bool ok = (m->getError() == 0);
        int64_t waypoint = home + TFH_CPR / 2;               // 0.5 rot from home

        // Batch one, queued in full before anything runs: relative approach,
        // two relative index steps, then an absolute correction to the datum.
        uint32_t t1 = millis();
        if (ok) ok = rmpTrap(m, APPROACH, T);
        if (ok) ok = rmpTrap(m, INDEX,    T);
        if (ok) ok = rmpTrap(m, INDEX,    T);
        if (ok) ok = rmpGoto(m, waypoint, T);
        RmpRun run1;
        rmpWatch(m, home, t1, rmpMsFor(T * 4) + 12000, &run1);
        uint8_t fat1 = tfhFatal(m);
        int64_t atWaypoint = m->getPositionRaw();

        // Batch two: retract relatively, then go home absolutely.
        bool ok2 = ok && run1.drained && (fat1 == 0);
        uint32_t t2 = millis();
        if (ok2) ok2 = rmpTrap(m, -RETRACT, T);
        if (ok2) ok2 = rmpGoto(m, home,     T);
        RmpRun run2;
        rmpWatch(m, home, t2, rmpMsFor(T * 2) + 12000, &run2);
        uint8_t fat2 = tfhFatal(m);
        uint8_t rest = m->getNQueuedItems();
        bool restOk = (m->getError() == 0);
        int64_t atHome = m->getPositionRaw();

        printf("   relative sum before the waypoint was %lld; waypoint %lld, landed %lld "
               "(error %lld)\n",
               (long long)((int64_t)APPROACH + 2 * (int64_t)INDEX),
               (long long)waypoint, (long long)atWaypoint,
               (long long)(atWaypoint - waypoint));
        printf("   home %lld, returned to %lld (error %lld), fatal=%u/%u, resting depth %u\n",
               (long long)home, (long long)atHome, (long long)(atHome - home),
               (unsigned)fat1, (unsigned)fat2, (unsigned)rest);

        TEST_RESULT("the mixed relative/absolute program is accepted end to end",
                    ok && ok2 && run1.drained && run2.drained);
        TEST_RESULT("the mixed program leaves the queue empty when it finishes",
                    restOk && rest == 0);
        TEST_RESULT("the mixed program raises no fatal error", fat1 == 0 && fat2 == 0);
        TEST_RESULT("an absolute waypoint queued behind three relative moves lands on the waypoint",
                    run1.drained && rmpAbs64(atWaypoint - waypoint) <= rmpTol(4));
        TEST_RESULT("the absolute return brings the program back to where it started",
                    run2.drained && rmpAbs64(atHome - home) <= rmpTol(6));
        tfhReset(m);
    }

    // =====================================================================
    // 6. STILL A GOOD MACHINE AFTERWARDS. Everything above ran without a
    //    reset inside a program, so this is the check that none of it left
    //    residue in the planner: one plain quarter-rotation move must still
    //    be as accurate as it is on a cold machine.
    // =====================================================================
    printf("\n--- 6. still exact after every program has run ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   plain quarter-rotation move: ok=%d moved=%lld of %lld\n",
               (int)ok, (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("a plain quarter-rotation move is still exact after every program",
                    ok && rmpAbs64(moved - TFH_CPR / 4) <= rmpTol(1));
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
