#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_queue_ordering.cpp
//
// THE MOTION QUEUE IS A FIFO, AND A REFUSAL IS ALL-OR-NOTHING.
//
// Eighty-odd modules queue moves and check where the shaft ended up. Not one of
// them checks that the moves ran in the ORDER they were queued. That is a real
// gap: the endpoint of a plan is the SUM of its moves, and a sum does not care
// about order. Every existing displacement assertion would pass unchanged if
// the firmware executed a queued plan backwards, or shuffled it, or ran the
// third move first. Order only becomes visible if you look at the trajectory
// WHILE it is being executed.
//
// GROUND TRUTH (firmware/Src/motor_control.c) -- the queue really is a ring:
//
//   :112-115   static movement_queue_t movement_queue[MOVEMENT_QUEUE_SIZE];
//              static uint8_t queue_write_position = 0;
//              static uint8_t queue_read_position  = 0;
//              static uint8_t n_items_in_queue     = 0;
//   :1860      queue_write_position = (queue_write_position + 1) & (MOVEMENT_QUEUE_SIZE - 1);
//   :2131-2146 ... movement_queue[queue_read_position] ...
//              queue_read_position = (queue_read_position + 1) & (MOVEMENT_QUEUE_SIZE - 1);
//              (the advance itself is at :2135 for acceleration items and :2143
//               for velocity items; n_items_in_queue-- follows each)
//   motor_control.h:31   #define MOVEMENT_QUEUE_SIZE 32   // "has to be a power of 2"
//
// Producer writes at the write index, consumer reads at the read index, both
// advance by one modulo 32. That is a textbook FIFO -- but the masking is index
// arithmetic that a future change could get wrong, and the failure mode of a
// broken ring is not a wrong endpoint, it is a wrong ORDER. Section 5 therefore
// keeps queueing past the 32-item wrap deliberately.
//
// WHAT THIS MODULE ASSERTS
//   1. moves execute strictly in the order they were queued, proved by giving
//      each leg a distinct SIGNATURE (alternating direction, growing magnitude)
//      and sampling the commanded position while the plan runs
//   2. the position is monotonic WITHIN each leg -- a queued move never
//      backtracks, so the reversals in the trajectory are exactly the sign
//      changes in the plan and nothing else
//   3. the queue DEPTH and the position agree about which move is running: by
//      the time k items have been consumed, exactly the first k items' worth of
//      displacement has been delivered
//   4. ordering survives the ring's 32-item wrap
//   5. ATOMICITY: when a move is refused because the queue is full, what
//      happens to the moves already queued behind it
//
// HOW THIS DIFFERS FROM ITS NEIGHBOURS
//   tm_queue_accounting  - how many SLOTS one move costs (3, or 2 on the
//                          one-tick-ramp fast path). Says nothing about order.
//   tm_queue_edges       - where the 32-item boundary is, and the emergency
//                          stop clear path.
//   tm_queue_stress      - fills and drains the queue, counts acceptances.
//   tm_queue_underrun    - error 18, i.e. what happens when the queue empties
//                          with a nonzero velocity.
//   tm_error_taxonomy    - that overflowing the queue reports code 17 and that
//                          a reset recovers. It established that a fatal error
//                          aborts the queue; section 6 here pins down exactly
//                          what that costs the caller and which parts of it are
//                          observable at all.
//   tm_move_composition  - that a sequence SUMS correctly, which is the
//                          order-blind property this module complements.
//
// THE ATOMICITY QUESTION, AND ITS HONEST ANSWER (section 6)
// add_to_queue() refuses only one way (motor_control.c:1790 / :1869):
//     if(n_items_in_queue < MOVEMENT_QUEUE_SIZE) { ...enqueue... }
//     else fatal_error(ERROR_QUEUE_IS_FULL);            // code 17
// and fatal_error() (common_source_files/error_handling.c:198, :231) calls
// __disable_irq() and then never returns -- it spins forever, hand-pumping the
// UART. So the control ISR stops dead at that instant: the moves queued BEFORE
// the refused one do not survive, they simply stop mid-flight, and the only
// exit is a system reset, which clears the whole queue
// (clear_the_queue_and_stop_no_disable_interrupt, motor_control.c:420) and puts
// the commanded position back to its static zero (motor_control.c:153, :3714).
// The caller loses the ENTIRE plan, not just the move that did not fit.
//
// One consequence worth stating because it is deliberately NOT asserted: a
// trapezoid move is three separate add_to_queue() calls
// (queue_trapezoid_segments, motor_control.c:2039-2041), so a move that finds
// only one or two free slots gets PARTIALLY written before the third call
// faults. That partial write is unobservable from outside -- once the fault is
// latched, get_status is the only command that answers at all (campaign finding
// F2) -- and it is irrelevant, because the machine is halted and the reset that
// follows clears the ring. Section 6 asserts what can actually be measured and
// says so, rather than inventing a probe for something no caller can see.
//
// SAFETY: the MOSFETs stay OFF for the whole module. Every measurement reads
// the COMMANDED position (cmd 34) and the queue depth (cmd 11), and the planner
// advances the commanded position whether or not the output stage is energised,
// so nothing turns on any motor at any point. Total commanded excursion never
// exceeds about half a rotation from the origin, and tfhFreshOpen() raises the
// deviation window first so the two-rotation watchdog is not the thing under
// test. The one deliberate fault is a queue overflow, cleared by a reset like
// any other. No broadcasts, no alias changes, no test modes: safe on the shared
// 35-motor rack.
// ---------------------------------------------------------------------------

static const uint8_t ERR_QUEUE_IS_FULL = 17;   // error_codes.json

// ---------------------------------------------------------------------------
// Trajectory sampling.
//
// File-scope state is static and re-initialised at entry, because the host
// harness can run modules back-to-back inside one process.
//
// Each sample is a (queue depth, commanded position) pair. The depth is read
// FIRST, so the recorded position is never OLDER than the recorded depth --
// i.e. a sample can show a position that has just crossed into the next leg
// while still carrying the previous leg's depth, but never the reverse. Every
// depth-versus-position assertion below is written to tolerate that one-sided
// lag (about one command round trip, a few milliseconds).
// ---------------------------------------------------------------------------
static const int SAMPLE_CAP = 512;
static int32_t sPos[SAMPLE_CAP];
static uint8_t sDepth[SAMPLE_CAP];
static int     sN;

// Sample until the queue empties or the budget runs out. Returns true only if
// the queue really did reach zero -- a budget expiry is a failure, never a
// silently truncated trajectory. (A short wait that reads a PARTIAL move is the
// single most expensive mistake available in this suite; see tfhDrain.)
static bool sampleUntilIdle(Servomotor* m, int64_t origin, uint32_t budgetMs,
                            uint32_t periodMs) {
    sN = 0;
    uint32_t deadline = millis() + budgetMs;
    while ((millis() < deadline) && (sN < SAMPLE_CAP)) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return false;
        int64_t p = m->getPositionRaw();
        if (m->getError() != 0) return false;
        sDepth[sN] = q;
        sPos[sN]   = (int32_t)(p - origin);
        sN++;
        if (q == 0) return true;
        delay(periodMs);
    }
    return false;
}

// Index of the first sample at or beyond a threshold, or -1 if never reached.
static int firstAtOrAbove(int32_t threshold) {
    for (int i = 0; i < sN; i++) if (sPos[i] >= threshold) return i;
    return -1;
}
static int firstAtOrBelow(int32_t threshold) {
    for (int i = 0; i < sN; i++) if (sPos[i] <= threshold) return i;
    return -1;
}

// All present and strictly increasing: the ordering verdict.
static bool strictlyIncreasing(const int* idx, int n) {
    for (int i = 0; i < n; i++) if (idx[i] < 0) return false;
    for (int i = 1; i < n; i++) if (idx[i] <= idx[i - 1]) return false;
    return true;
}

// ---------------------------------------------------------------------------
// Queue a plan of alternating direction and growing magnitude:
//     leg k = (-1)^(k+1) * k * u,  k = 1..nLegs
// so the position after leg k is +j*u for k = 2j-1 and -j*u for k = 2j. Every
// leg therefore ends somewhere no earlier leg ever reached, which is what makes
// a first-crossing milestone a unique fingerprint for one specific leg.
// ---------------------------------------------------------------------------
static bool queueAlternatingPlan(Servomotor* m, int nLegs, int32_t u, uint32_t ticksPerLeg) {
    for (int k = 1; k <= nLegs; k++) {
        int32_t d = (int32_t)((int64_t)u * (int64_t)k);
        if ((k % 2) == 0) d = -d;
        m->trapezoidMoveRaw(d, ticksPerLeg);
        if (m->getError() != 0) return false;
    }
    return true;
}

// Limits used throughout. delta_t1 = max_velocity / max_acceleration
// (compute_trapezoid_move, motor_control.c:449), so 3 rot/s over 15 rot/s^2 is
// a 0.2 s ramp = 6250 ticks. Two things follow, and both are deliberate:
//
//   * the ramp is far longer than one control tick, so every move takes the
//     ordinary THREE-slot path (motor_control.c:2039-2041) rather than the
//     two-slot one-tick fast path (queue_trapezoid_segments takes the fast path
//     only when delta_t1 <= 1). Slot counts below are therefore predictable,
//     which is what makes "depth 7..9 is leg 1" meaningful in section 3. It
//     also requires every leg duration below to be comfortably more than
//     2 * delta_t1 = 12500 ticks (0.4 s), otherwise delta_t1 is clamped to
//     total_time/2, delta_t2 becomes 0, and add_to_queue silently DROPS the
//     zero-duration coast segment -- a move that costs 2 slots, not 3.
//     The shortest leg used with these limits is 0.8 s. Keep it that way.
//   * the FIRST queued segment lasts 0.2 s, which is much longer than the time
//     it takes to send the rest of the plan over the bus. The queue cannot
//     start draining while it is still being filled, so a depth read taken
//     straight after the fill is race-free. The worst case under these limits
//     is section 4: eight moves plus one depth read is nine bus round trips,
//     and a round trip on this link is a few milliseconds (see the same
//     argument in tm_acceleration_move_semantics), so ~45 ms against a 200 ms
//     window. Section 6 needs eleven round trips AND its whole premise
//     collapses if even one slot frees, so it uses the slower limits below.
//
// Both ceilings are far above anything any leg below asks for (the most
// demanding is 5.7 rot/s^2 at 1.1 rot/s, in section 4). That matters because
// limits are pass/fail CHECKS, never clamps: a ceiling chosen too tightly does
// not slow a move down, it refuses it with fatal error 15, 16 or 28, and the
// module would end up measuring the ceiling instead of the queue.
static void setWorkingLimits(Servomotor* m) {
    m->setMaximumVelocity(3.0f);          // rot/s
    m->setMaximumAcceleration(15.0f);     // rot/s^2
}

// Section 6 only. 1 rot/s over 2 rot/s^2 is a 0.5 s ramp (15625 ticks), so the
// first queued segment lasts half a second and eleven round trips have a ~9x
// margin instead of ~3.6x. This is not fussiness: section 6 asserts that the
// ELEVENTH move is refused because only two slots remain. If one segment
// retires while the fill is still being sent, three slots are free, the
// eleventh move is ACCEPTED, and every assertion in the second half of the
// section fails on perfectly healthy firmware. The hardware-verified queue-full
// provocations elsewhere in the suite (tm_error_taxonomy trigFillQueue,
// tm_queue_accounting section 5) use a one-second ramp for exactly this reason.
//
// The cost of a long ramp is coarser planner rounding: the queued acceleration
// is delta_d/((delta_t1+delta_t2)*delta_t1) truncated toward zero, so each move
// under-delivers by at most (delta_t1*(delta_t1+delta_t2)) >> 24 counts -- about
// 20 counts for the 1.2 s control moves here, versus about 9 under
// setWorkingLimits. The displacement tolerance in section 6 is widened to suit.
//
// Every leg used with these limits must still be longer than 2 * 15625 ticks
// (1.0 s) or the coast segment vanishes and the move costs 2 slots (see above).
static void setSlowRampLimits(Servomotor* m) {
    m->setMaximumVelocity(1.0f);          // rot/s
    m->setMaximumAcceleration(2.0f);      // rot/s^2
}

void tm_queue_ordering(void) {
    Serial.println("tm_queue_ordering: BEGIN\n");
    Servomotor* m = tfGetMotor();
    sN = 0;

    // Analysis results shared between sections 1 and 2.
    int  mile4[4] = { -1, -1, -1, -1 };
    int  rev[3]   = { -1, -1, -1 };
    int  nRev     = 0;
    const int32_t U1 = (int32_t)(TFH_CPR / 5);        // 0.2 rotation
    const uint32_t LEG1_TICKS = (uint32_t)(TFH_TPS * 6 / 5);   // 1.2 s

    // =====================================================================
    // 1. FOUR MOVES, FOUR SIGNATURES, ONE ORDER.
    //
    //    Queue +1u, -2u, +3u, -4u. The plan visits +1u, -1u, +2u, -2u in that
    //    order, and each of those is a place no EARLIER leg ever reached. So
    //    the first time the trajectory crosses +0.9u it must be leg 1 doing
    //    it, the first time it crosses -0.9u it must be leg 2, and so on.
    //    Four first-crossing timestamps that come out in queue order are a
    //    direct proof of FIFO execution; any other order -- LIFO, shuffled,
    //    one move skipped -- produces a different sequence, or fails to reach
    //    a milestone at all.
    //
    //    The endpoint alone cannot see any of this: every permutation of the
    //    four legs lands at exactly -2u.
    // =====================================================================
    printf("\n--- 1. four legs with distinct signatures run in queue order ---\n");
    {
        tfhFreshOpen(m);
        setWorkingLimits(m);
        int64_t origin = m->getPositionRaw();
        bool accepted = queueAlternatingPlan(m, 4, U1, LEG1_TICKS);
        uint8_t depth = m->getNQueuedItems();

        sN = 0;                       // never analyse a previous section's samples
        bool drained = accepted && sampleUntilIdle(m, origin, 4 * 1200 + 9000, 20);
        drained = drained && tfhDrain(m, 6000);
        uint8_t fat = tfhFatal(m);
        int64_t finalPos = m->getPositionRaw() - origin;

        mile4[0] = firstAtOrAbove((int32_t)((int64_t)U1 *  9 / 10));   // +0.9u, leg 1
        mile4[1] = firstAtOrBelow((int32_t)((int64_t)U1 * -9 / 10));   // -0.9u, leg 2
        mile4[2] = firstAtOrAbove((int32_t)((int64_t)U1 * 19 / 10));   // +1.9u, leg 3
        mile4[3] = firstAtOrBelow((int32_t)((int64_t)U1 * -19 / 10));  // -1.9u, leg 4

        int32_t hi = 0, lo = 0;
        for (int i = 0; i < sN; i++) {
            if (sPos[i] > hi) hi = sPos[i];
            if (sPos[i] < lo) lo = sPos[i];
        }
        const int32_t tol = U1 / 20;                        // 0.01 rotation

        printf("   %d samples; milestones at samples %d %d %d %d\n",
               sN, mile4[0], mile4[1], mile4[2], mile4[3]);
        printf("   envelope %lld .. %lld (predicted %lld .. %lld); final %lld (predicted %lld)\n",
               (long long)lo, (long long)hi,
               (long long)(-2 * (int64_t)U1), (long long)(2 * (int64_t)U1),
               (long long)finalPos, (long long)(-2 * (int64_t)U1));

        TEST_RESULT("four trapezoid moves are accepted and occupy 12 queue slots",
                    accepted && depth == 12);
        TEST_RESULT("the whole plan drains with no fatal error", drained && fat == 0);
        TEST_RESULT("every leg reaches the milestone only it can reach",
                    mile4[0] >= 0 && mile4[1] >= 0 && mile4[2] >= 0 && mile4[3] >= 0);
        TEST_RESULT("the milestones are crossed in the order the moves were queued",
                    strictlyIncreasing(mile4, 4));
        TEST_RESULT("the trajectory never leaves the envelope the queued order predicts",
                    (hi <= 2 * U1 + tol) && (lo >= -2 * U1 - tol));
        TEST_RESULT("the plan ends where the sum of the four queued moves says",
                    drained && llabs((long long)(finalPos + 2 * (int64_t)U1)) <= 300);
    }

    // =====================================================================
    // 2. NOTHING HAPPENS BETWEEN THE LEGS THAT THE PLAN DID NOT ASK FOR.
    //
    //    Ordering is only half the contract. The other half is that each
    //    queued move is executed as ONE monotonic excursion: a trapezoid ramps
    //    up, coasts and ramps down, all in one direction, so the commanded
    //    position must never turn round inside a leg. Which means the whole
    //    four-leg trajectory must contain EXACTLY three direction reversals --
    //    one per sign change in the plan -- and they must fall between the
    //    milestones, alternating with them.
    //
    //    A queue that ran the right moves in the right order but re-entered a
    //    finished item, double-counted a segment, or dropped a tick at the
    //    hand-off would show up here as a fourth reversal or a backtrack, and
    //    nowhere else in the suite.
    //
    //    This section re-uses section 1's samples: the analysis is what is
    //    being added, not another run on the hardware.
    // =====================================================================
    printf("\n--- 2. each leg is one monotonic excursion, with three reversals ---\n");
    {
        // A direction change is only believed when the step is bigger than the
        // deadband. During the SLOWEST leg's coast the position moves about
        // 17000 counts per sample; at a leg boundary the velocity passes
        // through zero, so the steps either side of a turning point are much
        // smaller than that. 2500 counts cleanly separates "a real reversal"
        // from "the turning point", and a deadband can only ever cause a
        // reversal to be noticed a sample or two LATE -- it cannot invent one,
        // because an against-direction step of any size is real motion.
        const int32_t DEADBAND = 2500;
        int dir = 0;
        nRev = 0;
        for (int i = 1; i < sN; i++) {
            int32_t d = sPos[i] - sPos[i - 1];
            if (llabs((long long)d) < DEADBAND) continue;
            int nd = (d > 0) ? 1 : -1;
            if (dir == 0) { dir = nd; continue; }
            if (nd == dir) continue;
            if (nRev < 3) rev[nRev] = i;
            nRev++;
            dir = nd;
        }

        // Run boundaries from the reversals, then check monotonicity strictly
        // INSIDE each run. Five samples either side of a boundary are skipped.
        // That is not slack, it is the deadband's cost: a reversal is only
        // recognised a sample or two after the true turning point, so a run's
        // nominal end index sits slightly INSIDE the following leg. Skipping
        // five covers that lag with room to spare while still leaving about
        // thirty-five samples of interior per leg to check.
        const int GUARD = 5;
        int start[4] = { 1, 0, 0, 0 };
        int end[4]   = { 0, 0, 0, 0 };
        int runLen[4] = { 0, 0, 0, 0 };
        int32_t worstBacktrack = 0;
        int shortestRun = 100000;
        bool shapeOk = (nRev == 3) && (sN > 40);
        if (shapeOk) {
            start[0] = 1;          end[0] = rev[0] - 1;
            start[1] = rev[0];     end[1] = rev[1] - 1;
            start[2] = rev[1];     end[2] = rev[2] - 1;
            start[3] = rev[2];     end[3] = sN - 1;
            for (int r = 0; r < 4; r++) {
                int wantDir = ((r % 2) == 0) ? 1 : -1;
                runLen[r] = end[r] - start[r] + 1;
                if (runLen[r] < shortestRun) shortestRun = runLen[r];
                for (int i = start[r] + GUARD; i <= end[r] - GUARD; i++) {
                    if (i < 1 || i >= sN) continue;
                    int32_t d = sPos[i] - sPos[i - 1];
                    if (((wantDir > 0) && (d < 0)) || ((wantDir < 0) && (d > 0))) {
                        int32_t back = (int32_t)llabs((long long)d);
                        if (back > worstBacktrack) worstBacktrack = back;
                    }
                }
            }
        }

        // Per-leg displacement, measured at the turning points. The velocity is
        // zero there, so a sampled turning point is an accurate one.
        int64_t legMoved[4] = { 0, 0, 0, 0 };
        bool legsMatch = false;
        if (shapeOk) {
            legMoved[0] = (int64_t)sPos[end[0]] - (int64_t)sPos[0];
            legMoved[1] = (int64_t)sPos[end[1]] - (int64_t)sPos[end[0]];
            legMoved[2] = (int64_t)sPos[end[2]] - (int64_t)sPos[end[1]];
            legMoved[3] = (int64_t)sPos[end[3]] - (int64_t)sPos[end[2]];
            const int64_t want[4] = { (int64_t)U1, -2 * (int64_t)U1,
                                      3 * (int64_t)U1, -4 * (int64_t)U1 };
            legsMatch = true;
            for (int r = 0; r < 4; r++)
                if (llabs((long long)(legMoved[r] - want[r])) > U1 / 10) legsMatch = false;
        }

        bool interleaved = shapeOk &&
                           (mile4[0] < rev[0]) && (rev[0] < mile4[1]) &&
                           (mile4[1] < rev[1]) && (rev[1] < mile4[2]) &&
                           (mile4[2] < rev[2]) && (rev[2] < mile4[3]);

        printf("   reversals: %d (at samples %d %d %d); run lengths %d %d %d %d\n",
               nRev, rev[0], rev[1], rev[2],
               runLen[0], runLen[1], runLen[2], runLen[3]);
        printf("   per-leg displacement %lld %lld %lld %lld (queued %lld %lld %lld %lld)\n",
               (long long)legMoved[0], (long long)legMoved[1],
               (long long)legMoved[2], (long long)legMoved[3],
               (long long)U1, (long long)(-2 * (int64_t)U1),
               (long long)(3 * (int64_t)U1), (long long)(-4 * (int64_t)U1));
        printf("   worst backtrack inside a leg: %lld counts\n", (long long)worstBacktrack);

        TEST_RESULT("the plan reverses direction exactly three times, once per queued sign change",
                    nRev == 3);
        TEST_RESULT("no leg backtracks: each queued move is one monotonic excursion",
                    shapeOk && worstBacktrack == 0);
        TEST_RESULT("the reversals and the milestones alternate, in queue order",
                    interleaved);
        TEST_RESULT("each leg delivers its own queued displacement, in its own turn",
                    legsMatch);
        TEST_RESULT("every leg is resolved by more than a handful of samples",
                    shapeOk && shortestRun >= 12);
    }

    // =====================================================================
    // 3. THE QUEUE DEPTH AND THE POSITION AGREE ON WHICH MOVE IS RUNNING.
    //
    //    A stronger statement than "the milestones came out in order": by the
    //    time the queue has given back k items, exactly the FIRST k items'
    //    worth of displacement must have been delivered. Reading the depth
    //    alongside the position turns the queue into a clock and lets each
    //    prefix sum be checked directly.
    //
    //    The signature here is MAGNITUDE rather than direction: three moves in
    //    the same direction, tiny / huge / medium. Their prefix sums are far
    //    apart, so the depth bucket a sample falls into pins down which leg
    //    produced it. Three same-direction moves are also the case where the
    //    endpoint is most obviously order-blind -- 0.05 + 0.6 + 0.15 rotations
    //    is 0.8 rotations in any order whatsoever.
    //
    //    Each trapezoid costs three slots (motor_control.c:2039-2041), so with
    //    9 items queued: depth 7..9 is leg 1, 4..6 is leg 2, 1..3 is leg 3.
    // =====================================================================
    printf("\n--- 3. queue depth and position agree on which move is executing ---\n");
    {
        const int32_t A = (int32_t)(TFH_CPR / 20);        // 0.05 rotation
        const int32_t B = (int32_t)(TFH_CPR * 3 / 5);     // 0.60 rotation
        const int32_t C = (int32_t)(TFH_CPR * 3 / 20);    // 0.15 rotation
        const uint32_t legTicks = (uint32_t)(TFH_TPS * 6 / 5);   // 1.2 s
        const int32_t tol = (int32_t)(TFH_CPR / 50);      // 0.02 rotation

        tfhFreshOpen(m);
        setWorkingLimits(m);
        int64_t origin = m->getPositionRaw();
        m->trapezoidMoveRaw(A, legTicks);
        bool accepted = (m->getError() == 0);
        m->trapezoidMoveRaw(B, legTicks);
        accepted = accepted && (m->getError() == 0);
        m->trapezoidMoveRaw(C, legTicks);
        accepted = accepted && (m->getError() == 0);
        uint8_t depth = m->getNQueuedItems();

        sN = 0;
        bool drained = accepted && sampleUntilIdle(m, origin, 3 * 1200 + 9000, 20);
        drained = drained && tfhDrain(m, 6000);
        uint8_t fat = tfhFatal(m);
        int64_t finalPos = m->getPositionRaw() - origin;

        // EMPTY SENTINEL: a bucket that never received a sample keeps
        // min = INT32_MAX and max = INT32_MIN, which deliberately violates
        // min <= max. Emptiness is asserted separately, before any comparison
        // that would be meaningless on an empty bucket.
        int32_t bMin[3] = { INT32_MAX, INT32_MAX, INT32_MAX };
        int32_t bMax[3] = { INT32_MIN, INT32_MIN, INT32_MIN };
        int     bCount[3] = { 0, 0, 0 };
        for (int i = 0; i < sN; i++) {
            int b = -1;
            if (sDepth[i] >= 7)      b = 0;
            else if (sDepth[i] >= 4) b = 1;
            else if (sDepth[i] >= 1) b = 2;
            if (b < 0) continue;
            if (sPos[i] < bMin[b]) bMin[b] = sPos[i];
            if (sPos[i] > bMax[b]) bMax[b] = sPos[i];
            bCount[b]++;
        }
        bool allSeen = (bCount[0] > 0) && (bCount[1] > 0) && (bCount[2] > 0);
        int64_t span[3] = { 0, 0, 0 };
        if (allSeen) for (int b = 0; b < 3; b++) span[b] = (int64_t)bMax[b] - (int64_t)bMin[b];

        printf("   depth 7..9 (leg 1): %d samples, %lld .. %lld\n",
               bCount[0], (long long)(allSeen ? bMin[0] : 0), (long long)(allSeen ? bMax[0] : 0));
        printf("   depth 4..6 (leg 2): %d samples, %lld .. %lld\n",
               bCount[1], (long long)(allSeen ? bMin[1] : 0), (long long)(allSeen ? bMax[1] : 0));
        printf("   depth 1..3 (leg 3): %d samples, %lld .. %lld\n",
               bCount[2], (long long)(allSeen ? bMin[2] : 0), (long long)(allSeen ? bMax[2] : 0));
        printf("   prefix sums A=%lld A+B=%lld A+B+C=%lld; final %lld; fatal %u\n",
               (long long)A, (long long)((int64_t)A + B), (long long)((int64_t)A + B + C),
               (long long)finalPos, (unsigned)fat);

        TEST_RESULT("three same-direction moves are accepted and occupy 9 queue slots",
                    accepted && depth == 9);
        TEST_RESULT("all three moves are observed while they are the one executing",
                    drained && allSeen);
        TEST_RESULT("while the FIRST queued move runs, the position never passes its endpoint",
                    allSeen && bMax[0] <= A + tol);
        TEST_RESULT("the SECOND queued move begins where the first one ended",
                    allSeen && bMin[1] >= A - tol);
        TEST_RESULT("and while it runs the position never passes the sum of the first two",
                    allSeen && bMax[1] <= (int32_t)((int64_t)A + B) + tol);
        TEST_RESULT("the THIRD queued move begins from the sum of the first two",
                    allSeen && bMin[2] >= (int32_t)((int64_t)A + B) - tol);
        TEST_RESULT("the large move is the one that was queued second, not first or third",
                    allSeen && (span[1] > 3 * span[0]) && (span[1] > 2 * span[2]));
        TEST_RESULT("the plan ends at the sum of the three queued moves",
                    drained && fat == 0 &&
                    llabs((long long)(finalPos - ((int64_t)A + B + C))) <= 300);
    }

    // =====================================================================
    // 4. THE SAME PROPERTY, EIGHT LEGS DEEP.
    //
    //    Four legs can be got right by accident. Eight alternating legs of
    //    growing magnitude produce eight milestones that must come out in
    //    exactly one of 40320 possible orders, and 24 of the queue's 32 slots
    //    are occupied at once, so the plan spans most of the ring in a single
    //    fill. This is the ordering claim at full depth.
    // =====================================================================
    printf("\n--- 4. eight legs, eight milestones, one order ---\n");
    {
        const int32_t U = (int32_t)(TFH_CPR / 10);                 // 0.1 rotation
        const uint32_t legTicks = (uint32_t)(TFH_TPS * 9 / 10);    // 0.9 s

        tfhFreshOpen(m);
        setWorkingLimits(m);
        int64_t origin = m->getPositionRaw();
        bool accepted = queueAlternatingPlan(m, 8, U, legTicks);
        uint8_t depth = m->getNQueuedItems();

        sN = 0;
        bool drained = accepted && sampleUntilIdle(m, origin, 8 * 900 + 9000, 20);
        drained = drained && tfhDrain(m, 8000);
        uint8_t fat = tfhFatal(m);
        int64_t finalPos = m->getPositionRaw() - origin;

        int mile[8];
        for (int j = 1; j <= 4; j++) {
            int32_t up   = (int32_t)((int64_t)U * (10 * j - 1) / 10);
            mile[2 * j - 2] = firstAtOrAbove(up);
            mile[2 * j - 1] = firstAtOrBelow(-up);
        }
        bool allSeen = true;
        for (int i = 0; i < 8; i++) if (mile[i] < 0) allSeen = false;

        printf("   %d samples; milestones at %d %d %d %d %d %d %d %d\n", sN,
               mile[0], mile[1], mile[2], mile[3], mile[4], mile[5], mile[6], mile[7]);
        printf("   final %lld (predicted %lld); depth after queueing %u; fatal %u\n",
               (long long)finalPos, (long long)(-4 * (int64_t)U),
               (unsigned)depth, (unsigned)fat);

        TEST_RESULT("eight trapezoid moves are accepted and occupy 24 queue slots",
                    accepted && depth == 24);
        TEST_RESULT("all eight signature milestones are reached", drained && allSeen);
        TEST_RESULT("all eight are crossed in the order the moves were queued",
                    strictlyIncreasing(mile, 8));
        TEST_RESULT("an eight-leg plan ends at the sum of its legs with no fault",
                    drained && fat == 0 &&
                    llabs((long long)(finalPos + 4 * (int64_t)U)) <= 400);
    }

    // =====================================================================
    // 5. ORDERING SURVIVES THE RING'S WRAP.
    //
    //    queue_write_position advances modulo 32 (motor_control.c:1860) and so
    //    does queue_read_position (:2135, :2143). Everything above ran inside
    //    one pass of the ring, so a broken wrap would go unnoticed. Here the
    //    same three-leg signature is repeated FIVE times back to back with no
    //    reset in between -- 5 x 9 = 45 items through a 32-slot ring, so the
    //    write index wraps during round 4 and rounds 4 and 5 execute out of the
    //    wrapped region.
    //
    //    The reset is deliberately omitted between rounds: a system reset, and
    //    equally an emergency stop, calls
    //    clear_the_queue_and_stop_no_disable_interrupt() (motor_control.c:420),
    //    which puts BOTH indices back to 0 and would hide the wrap entirely.
    //    Letting the queue drain naturally is the only way to reach it.
    // =====================================================================
    printf("\n--- 5. ordering survives the 32-slot ring wrap ---\n");
    {
        const int32_t U = (int32_t)(TFH_CPR / 20);                 // 0.05 rotation
        const uint32_t legTicks = (uint32_t)(TFH_TPS * 4 / 5);     // 0.8 s

        tfhFreshOpen(m);
        setWorkingLimits(m);

        bool roundOk[5] = { false, false, false, false, false };
        int  itemsQueued = 0;
        for (int r = 0; r < 5; r++) {
            int64_t origin = m->getPositionRaw();
            bool accepted = queueAlternatingPlan(m, 3, U, legTicks);   // +1u, -2u, +3u
            uint8_t depth = m->getNQueuedItems();
            itemsQueued += 9;

            sN = 0;
            bool drained = accepted && sampleUntilIdle(m, origin, 3 * 800 + 8000, 20);
            drained = drained && tfhDrain(m, 6000);
            int64_t moved = m->getPositionRaw() - origin;

            int mile[3];
            mile[0] = firstAtOrAbove((int32_t)((int64_t)U *  9 / 10));   // +0.9u, leg 1
            mile[1] = firstAtOrBelow((int32_t)((int64_t)U * -9 / 10));   // -0.9u, leg 2
            mile[2] = firstAtOrAbove((int32_t)((int64_t)U * 19 / 10));   // +1.9u, leg 3

            roundOk[r] = drained && (depth == 9) && strictlyIncreasing(mile, 3) &&
                         (llabs((long long)(moved - 2 * (int64_t)U)) <= 300);
            printf("   round %d: %d items queued so far, depth %u, milestones %d %d %d, moved %lld\n",
                   r + 1, itemsQueued, (unsigned)depth,
                   mile[0], mile[1], mile[2], (long long)moved);
            TEST_RESULT(std::string("round ") + std::to_string(r + 1) +
                            " runs its three legs in the queued order",
                        roundOk[r]);
        }
        // 45 items through a 32-slot ring: rounds 4 and 5 are on the far side
        // of the wrap by construction, so this is a statement about the index
        // arithmetic, not about repetition.
        TEST_RESULT("the ring wrapped part-way through and ordering was unaffected",
                    (itemsQueued > 32) && roundOk[3] && roundOk[4] && (tfhFatal(m) == 0));
    }

    // =====================================================================
    // 6. ATOMICITY: WHAT A QUEUE-FULL REFUSAL COSTS.
    //
    //    The question this section exists to answer: when a move is refused
    //    because the queue is full, do the moves queued BEFORE it survive?
    //
    //    The answer is no, and the reason is structural rather than a policy
    //    decision. add_to_queue()'s only refusal is
    //        else fatal_error(ERROR_QUEUE_IS_FULL);   (motor_control.c:1869)
    //    and fatal_error() disables interrupts and spins forever
    //    (error_handling.c:198, :231). The control ISR that drains the queue
    //    stops at that instant, so the plan is frozen mid-flight; the only exit
    //    is a system reset, which clears the ring and returns the commanded
    //    position to zero (motor_control.c:420, :3714). The caller loses the
    //    WHOLE plan, not merely the move that did not fit.
    //
    //    A control run is what makes that meaningful: the same ten moves, left
    //    alone, deliver their full displacement. The only difference in the
    //    second run is an eleventh move that does not fit, and it costs
    //    everything.
    //
    //    Note also what is asserted about TIMING. The refusal arrives while a
    //    plan with eighty seconds left to run is still in the queue, so it is a
    //    genuine refusal and not a request that blocks until space appears.
    //
    //    This section runs on setSlowRampLimits, not setWorkingLimits: the
    //    half-second first segment is what guarantees no slot frees while the
    //    ten-move fill is still being sent. See that helper for the argument.
    // =====================================================================
    printf("\n--- 6. a queue-full refusal discards the whole plan ---\n");
    {
        const int32_t D = (int32_t)(TFH_CPR / 20);      // 0.05 rotation per move
        // 1.2 s per control move: comfortably above 2 * delta_t1 = 1.0 s, so
        // each move is still the ordinary three-segment shape (delta_t1 = 15625,
        // delta_t2 = 6250) and ten of them occupy exactly 30 slots.
        const uint32_t CTRL_TICKS = (uint32_t)(TFH_TPS * 6 / 5);   // 1.2 s
        const uint32_t FILL_TICKS = (uint32_t)(8 * TFH_TPS);       // 8 s

        // --- control: ten moves, no eleventh -----------------------------
        tfhFreshOpen(m);
        setSlowRampLimits(m);
        int64_t origin = m->getPositionRaw();
        bool accepted = true;
        for (int k = 0; k < 10 && accepted; k++) {
            m->trapezoidMoveRaw(D, CTRL_TICKS);
            accepted = (m->getError() == 0);
        }
        uint8_t depth = m->getNQueuedItems();
        bool drained = accepted && tfhDrain(m, 30000);   // the plan itself is 12 s
        uint8_t fat = tfhFatal(m);
        int64_t moved = m->getPositionRaw() - origin;
        printf("   control: depth %u after queueing, moved %lld of %lld, fatal %u\n",
               (unsigned)depth, (long long)moved, (long long)(10 * (int64_t)D), (unsigned)fat);
        TEST_RESULT("ten trapezoid moves fill 30 of the queue's 32 slots",
                    accepted && depth == 30);
        // Tolerance is 600, not 400: the planner truncates the queued
        // acceleration, so each of these ten moves under-delivers by up to
        // (15625 * 21875) >> 24 = 20 counts, i.e. up to ~204 counts over the
        // plan. The claim being made is "all ten legs ran", and one missing leg
        // is 163840 counts -- three orders of magnitude above the tolerance.
        TEST_RESULT("left alone, all ten are delivered in full",
                    drained && fat == 0 &&
                    llabs((long long)(moved - 10 * (int64_t)D)) <= 600);

        // --- the same plan, plus one move that does not fit ---------------
        // Eight-second legs so the queued plan has ~80 s left to run when the
        // refusal lands: that is what distinguishes "refused" from "deferred".
        tfhFreshOpen(m);
        setSlowRampLimits(m);
        bool filled = true;
        for (int k = 0; k < 10 && filled; k++) {
            m->trapezoidMoveRaw(D, FILL_TICKS);
            filled = (m->getError() == 0);
        }
        uint8_t fullDepth = m->getNQueuedItems();

        uint32_t t0 = millis();
        m->trapezoidMoveRaw(D, FILL_TICKS);                // the 11th: needs 3, has 2
        int errOnOffending = m->getError();
        // Measured BEFORE the status read, so a slow (or timed-out) get_status
        // cannot be mistaken for a slow refusal.
        uint32_t elapsed = millis() - t0;
        uint8_t fatalCode = tfhFatal(m);

        // Once the fault is latched, get_status is the ONLY command that
        // answers (campaign finding F2), so the queue depth becomes unreadable.
        // That unreadability IS the observation: the machine is halted, not
        // merely holding a full queue.
        m->getNQueuedItems();
        int errOnDepthRead = m->getError();

        printf("   fill depth %u; 11th move -> getError %d, fatal %u, after %lu ms\n",
               (unsigned)fullDepth, errOnOffending, (unsigned)fatalCode,
               (unsigned long)elapsed);
        printf("   queue-depth read while faulted -> getError %d (0 would mean it still answers)\n",
               errOnDepthRead);
        TEST_RESULT("the eleventh move is refused rather than accepted",
                    filled && fullDepth == 30 && errOnOffending != 0);
        TEST_RESULT("the refusal arrives at once, not deferred until the 80-second plan drains",
                    elapsed < 2000);
        TEST_RESULT("the refusal is reported as ERROR_QUEUE_IS_FULL (17)",
                    fatalCode == ERR_QUEUE_IS_FULL);
        TEST_RESULT("after the refusal the machine is halted: the queue depth stops answering",
                    errOnDepthRead != 0);

        // --- recovery: the plan is gone, not resumed ----------------------
        tfhReset(m);
        uint8_t afterDepth = m->getNQueuedItems();
        int64_t afterPos = m->getPositionRaw();
        bool readOk = (m->getError() == 0);
        delay(3000);                                   // longer than any leg
        uint8_t laterDepth = m->getNQueuedItems();
        int64_t laterPos = m->getPositionRaw();
        readOk = readOk && (m->getError() == 0);

        printf("   after reset: depth %u position %lld; 3 s later: depth %u position %lld\n",
               (unsigned)afterDepth, (long long)afterPos,
               (unsigned)laterDepth, (long long)laterPos);
        TEST_RESULT("a reset is the only exit, and it leaves an empty queue at the origin",
                    readOk && afterDepth == 0 && llabs((long long)afterPos) <= 300);
        TEST_RESULT("the moves queued before the refusal are discarded, not resumed",
                    readOk && laterDepth == 0 &&
                    llabs((long long)(laterPos - afterPos)) <= 4);

        tfhFreshOpen(m);
        setWorkingLimits(m);
        bool ok = false;
        int64_t again = tfhMove(m, (int32_t)(TFH_CPR / 8), TFH_TPS, &ok);
        printf("   a fresh move after the whole sequence: ok=%d moved %lld of %lld\n",
               (int)ok, (long long)again, (long long)(TFH_CPR / 8));
        TEST_RESULT("the machine accepts and completes a new plan afterwards",
                    ok && llabs((long long)(again - TFH_CPR / 8)) <= 300);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
