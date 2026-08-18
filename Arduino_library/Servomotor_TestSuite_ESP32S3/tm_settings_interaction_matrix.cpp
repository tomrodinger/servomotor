#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_settings_interaction_matrix.cpp
//
// SETTINGS THAT INTERACT. Every limit in this machine is tested on its own
// somewhere in this suite. Nothing tests them in COMBINATION, and the planner's
// behaviour is decided by their RATIOS, not by any one of them:
//
//   1. MAX VELOCITY vs MAX ACCELERATION decide the shape of the plan.
//      compute_trapezoid_move() sizes the ramp as an integer division of one
//      limit by the other (motor_control.c:449):
//
//          int64_t delta_t1 = max_velocity / max_acceleration;
//
//      and queue_trapezoid_segments() then branches on the result
//      (motor_control.c:2017):
//
//          if((delta_t1 <= 1) && (total_time >= 3))   -> TWO velocity slots
//          else                                       -> THREE acceleration slots
//
//      Both limits are stored SHIFTED -- max_velocity by VELOCITY_SHIFT_LEFT
//      (12) and max_acceleration by ACCELERATION_SHIFT_LEFT (8),
//      motor_control.h:64-65, applied in set_max_velocity() at :3348 and
//      set_max_acceleration() at :3367 -- so with the raw wire values v and a
//      the ramp is exactly
//
//          delta_t1 = (v << 12) / (a << 8) = (16 * v) / a          [integer]
//
//      which puts the 3-slot/2-slot frontier at exactly a == 8 * v: one raw
//      acceleration count above that, the plan changes shape. In physical
//      terms that is an acceleration ceiling 15625x the velocity ceiling. This
//      module asserts the frontier at that predicted ratio, to the single
//      count, from BOTH sides.
//
//   2. THE SAFETY FENCE vs THE DEVIATION WINDOW both stop a move that goes too
//      far, but at different MOMENTS, and for a move that violates both, which
//      one fires is not a race -- it is decided by the firmware's structure:
//        * the fence is a QUEUE-TIME prediction, checked inside add_to_queue()
//          before the item is counted into the queue (motor_control.c:1813 for
//          acceleration segments, :1856 for velocity ones) -> error 27, and it
//          is reported as the failure of the move command itself;
//        * the deviation watchdog is a RUN-TIME observation, checked in the
//          control loop against where the shaft actually is
//          (motor_control.c:3218) -> error 45, and it can only happen after
//          motion has begun.
//      So the fence must win, always, and the observable difference is sharp:
//      the fence makes the move command fail, the watchdog lets it succeed and
//      faults afterwards. That is asserted here as a deterministic contract,
//      not as a probability.
//
//   3. MAX VELOCITY vs THE COMMANDED DURATION decide whether a move exists at
//      all. On the 2-slot path the planner's whole arithmetic is one line
//      (motor_control.c:2024):
//
//          int64_t velocity = ((int64_t)total_displacement << 20) / (total_time - 1);
//
//      checked against the ceiling at motor_control.c:1847. Both sides are
//      integers, so the shortest legal duration for a given displacement is
//      computable TO THE TICK -- and it must halve when the ceiling doubles.
//
// WHAT THIS MODULE ASSERTS, as one property: the planner's behaviour is a
// deterministic function of the RATIOS between the settings, matching the
// firmware's integer arithmetic exactly at each cell of a small matrix, and
// repeatable across three independent trials of every cell.
//
// HOW IT IS DISTINCT FROM WHAT ALREADY EXISTS.
//   * tm_queue_accounting establishes that a move costs 3 slots or 2, using two
//     well-separated settings pairs (1 rot/s + 1 rot/s^2, and 0.3 rot/s +
//     12000 rot/s^2). It never varies the RATIO systematically and cannot reach
//     the frontier at all -- see the measurement note below. Nothing there
//     predicts WHERE the change happens; this module does, and pins it to one
//     raw count.
//   * tm_limit_boundaries checks each limit's own inclusive `>` comparison in
//     isolation. It does not ask which limit a trapezoid move trips FIRST, and
//     the answer turns out to depend on the ratio: the 3-slot path can only
//     ever refuse with ERROR_ACCEL_TOO_HIGH (15) and the 2-slot path can only
//     ever refuse with ERROR_VEL_TOO_HIGH (16), because they queue different
//     KINDS of segment.
//   * tm_safety_fence and tm_safety_fence_arithmetic characterise the fence
//     alone; tm_deviation_tracking characterises the watchdog alone. Neither
//     puts them in conflict, which is the only way to learn which is dominant.
//
// THE MEASUREMENT PROBLEM, AND HOW IT IS SOLVED. Queued segments start
// executing immediately, so reading the queue depth after a move races the
// 31.25 kHz ISR. tm_queue_accounting handles this by choosing settings that
// make the first segment about a second long -- which works, but makes the
// interesting configurations unmeasurable: at the frontier delta_t1 is 2 ticks,
// i.e. 64 microseconds, so a naive read would always see the ramp already
// consumed and report 2 slots for a 3-slot plan.
//
// This module instead HOLDS THE QUEUE with a zero-velocity dwell
// (`moveWithVelocityRaw(0, 8 seconds)`, cmd 26) queued ahead of the move under
// test. A velocity segment of zero occupies exactly one slot, commands no
// motion, and expires safely because the underrun check only fires on a
// NONZERO velocity (motor_control.c:2158-2164) -- all three properties already
// measured on hardware by tm_velocity_move_semantics section 3. Nothing behind
// it can execute until it expires, so the depth can be read at leisure and the
// slot cost is (depth after) - (depth before), exactly, for ANY settings pair.
// The move under test is then discarded with an emergency stop, so it never
// runs at all.
//
// WHY RAW SETTERS. Every prediction here is integer arithmetic on the wire
// values, so the module sets the limits with setMaximumVelocityRaw /
// setMaximumAccelerationRaw (cmds 3 and 5) rather than the float unit
// conversions. At the frontier the two differ: the float path rounds
// 109951162.7776 to the nearest representable float and can land on the wrong
// side of a one-count boundary. Raw values make the assertions exact.
//
// SAFETY, and why this is rack-safe. The MOSFETs are never enabled and
// goToClosedLoop is never called; every measurement reads the COMMANDED
// position or the queue depth, both of which the planner advances whether or
// not the output stage is energised, so NOTHING TURNS on any motor. Most cells
// never execute their move at all (it is held behind the dwell and then
// discarded). The section that does let moves run commands at most three
// rotations and deliberately trips watchdogs that a reset clears. No
// broadcasts, no alias changes, no test modes: safe on the shared 35-motor
// rack.
//
// THE EXPERIMENTAL CLAMPS, AND WHY EVERY SETTING HERE STAYS UNDER THEM.
// set_max_velocity() and set_max_acceleration() do NOT refuse an over-large
// limit -- they are the one pair of setters in this firmware that silently
// CLAMP (motor_control.c:3349-3351 and :3368-3370, against
// EXPERIMENTAL_MAX_VELOCITY = 2 x MAX_VELOCITY and EXPERIMENTAL_MAX_ACCELERATION
// = 4 x MAX_ACCELERATION, motor_control.h:61-62). planTrapezoid() below does
// not model that clamp, so a clamped setting would make every prediction in
// this module wrong in a way no assertion could see. Recomputed from the
// firmware's own macros for M17 (560 RPM, 12000 rot/s^2, 3276800 counts/rot,
// 31250 ticks/s), the ceilings in RAW wire units are:
//
//     velocity      2052421705 raw  = 18.67 rot/s
//     acceleration  2702159776 raw  = 48000 rot/s^2
//
// The largest values this module ever sets are 4 x VRAW_PER_ROT_PER_S
// (439804648 raw, 4 rot/s) and 20 x VRAW_PER_ROT_PER_S used as an ACCELERATION
// (2199023240 raw, 39062 rot/s^2) -- 4.7x and 1.23x of headroom respectively.
// ANY new cell must be re-checked against those two numbers before it is added.
//
// FIRMWARE VERSION THIS DESCRIBES: 0.15.12.0 or later. Sections 2 (cells 5-7),
// 3 and 4 all depend on the 2-slot fast path and the divide-by-zero correction
// that landed in 0.15.12.0. On the ESP32-S3 bench motor, still on 0.15.9.0,
// those cells are EXPECTED to fail -- the fast path does not exist there and an
// over-speed move is silently accepted instead of refused. That is the
// documented negative control (TEST_EXPANSION_CAMPAIGN_2026-08.md, "The
// ESP32-S3 is a NEGATIVE CONTROL"), not a new finding. Rack results are the
// authority.
// ---------------------------------------------------------------------------

static const uint8_t ERR_ACCEL_TOO_HIGH        = 15;  // add_to_queue, motor_control.c:1796
static const uint8_t ERR_VEL_TOO_HIGH          = 16;  // add_to_queue, motor_control.c:1847
static const uint8_t ERR_PREDICTED_VEL_TOO_HIGH = 28; // add_to_queue, motor_control.c:1803
static const uint8_t ERR_PREDICTED_POS_OUT_OF_ZONE = 27; // motor_control.c:1813 / :1856
static const uint8_t ERR_DEVIATION_TOO_LARGE   = 45;  // control loop, motor_control.c:3223

// The wire scales, as integers. Velocity is Q20 encoder counts per tick and
// acceleration is Q24 encoder counts per tick squared, so one rotation/second
// is (2^20 * 3276800 / 31250) = 109951162.7776 and one rotation/second^2 is
// (2^24 * 3276800 / 31250^2) = 56294.995. Truncated here; only the RATIO
// between the two settings matters for the frontier, and it is exact.
static const uint32_t VRAW_PER_ROT_PER_S   = 109951162;
static const uint32_t ARAW_PER_ROT_PER_S2  = 56294;

// The queue-holding dwell: long enough that the round trip of a few commands
// cannot exhaust it, short enough that a stranded motor frees itself.
static const uint32_t BLOCK_TICKS = 8 * (uint32_t)TFH_TPS;   // 8 seconds

// ---------------------------------------------------------------------------
// The firmware's plan, recomputed here in the same integer arithmetic.
// Every line is a transcription of compute_trapezoid_move() and
// queue_trapezoid_segments() with the checks add_to_queue() then applies. The
// point of duplicating it is that each matrix cell below gets a PREDICTION
// derived from the source rather than a hand-copied expected value.
// ---------------------------------------------------------------------------
struct Plan {
    int64_t  deltaT1;      // ramp length in ticks
    uint32_t deltaT2;      // coast length in ticks
    int      slots;        // queue slots the move should occupy
    uint8_t  refusedWith;  // 0 if the move should be accepted
};

static Plan planTrapezoid(uint32_t vraw, uint32_t araw, int32_t counts, uint32_t ticks) {
    Plan p;
    const int64_t maxV = (int64_t)vraw << 12;   // VELOCITY_SHIFT_LEFT, motor_control.c:3348
    const int64_t maxA = (int64_t)araw << 8;    // ACCELERATION_SHIFT_LEFT, motor_control.c:3367

    int64_t dt1 = maxV / maxA;                              // :449
    if ((dt1 << 1) > (int64_t)ticks) dt1 = (int64_t)(ticks >> 1);   // :450-452
    uint32_t dt2 = ticks - (uint32_t)(dt1 << 1);            // :453
    int64_t den = (dt1 + (int64_t)dt2) * dt1;               // :455
    if (den == 0) {                                          // :456-484, the divide-by-zero fix
        dt1 = 1;
        dt2 = (ticks >= 2) ? (ticks - 2) : 0;
        den = (dt1 + (int64_t)dt2) * dt1;
    }
    int64_t accel = ((int64_t)counts << 24) / den;          // :485
    if (accel > INT32_MAX) accel = INT32_MAX;               // the saturation at :510-515
    else if (accel < INT32_MIN) accel = INT32_MIN;

    p.deltaT1 = dt1;
    p.deltaT2 = dt2;

    if ((dt1 <= 1) && (ticks >= 3)) {
        // The 2-slot path: one long velocity hold plus a one-tick stop.
        int64_t v = ((int64_t)counts << 20) / (int64_t)(ticks - 1);   // :2024
        if (v > INT32_MAX) v = INT32_MAX;
        else if (v < INT32_MIN) v = INT32_MIN;
        p.slots = 2;
        // Checked as (v << 12) > (vraw << 12), which is v > vraw. :1847
        p.refusedWith = (llabs(v) > (int64_t)vraw) ? ERR_VEL_TOO_HIGH : 0;
    }
    else {
        // The 3-slot path: ramp, coast, ramp. A coast of zero duration is
        // silently dropped rather than queued (add_to_queue, :1782), which is
        // the one way a 3-segment plan still costs only 2 slots.
        p.slots = (dt2 == 0) ? 2 : 3;
        const int64_t accInternal = accel << 8;
        if (llabs(accInternal) > maxA)                  p.refusedWith = ERR_ACCEL_TOO_HIGH;
        else if (llabs(accInternal * dt1) > maxV)       p.refusedWith = ERR_PREDICTED_VEL_TOO_HIGH;
        else                                            p.refusedWith = 0;
    }
    return p;
}

// ---------------------------------------------------------------------------
// One matrix cell: set the two limits, hold the queue, offer the move, and
// report exactly what the machine did with it. The move is never allowed to
// execute.
// ---------------------------------------------------------------------------
struct Cell {
    bool    infraOk;    // the dwell was queued and the depth read back cleanly
    bool    accepted;   // the move command itself returned success
    int     slots;      // slots the move occupied (only meaningful if accepted)
    uint8_t fatal;      // latched fatal code after a refusal
};

static Cell measureCell(Servomotor* m, uint32_t vraw, uint32_t araw,
                        int32_t counts, uint32_t ticks) {
    Cell c;
    c.infraOk = false; c.accepted = false; c.slots = -1; c.fatal = 0;

    tfhReset(m);
    m->setMaximumVelocityRaw(vraw);
    m->setMaximumAccelerationRaw(araw);
    if (m->getError() != 0) return c;

    // The dwell is a VELOCITY item, so add_to_queue records
    // velocity_after_last_queue_item = its velocity = 0 (motor_control.c:1843,
    // :1866). The move under test therefore still plans from a zero velocity
    // baseline and the accumulate-across-segments trap does not apply -- which
    // is the other reason a zero-velocity dwell is the right instrument here.
    m->moveWithVelocityRaw(0, BLOCK_TICKS);      // hold the queue: nothing behind this runs
    if (m->getError() != 0) return c;
    uint8_t before = m->getNQueuedItems();
    if (m->getError() != 0 || before != 1) return c;
    c.infraOk = true;

    m->trapezoidMoveRaw(counts, ticks);
    if (m->getError() != 0) { c.fatal = tfhFatal(m); return c; }

    uint8_t after = m->getNQueuedItems();
    if (m->getError() != 0) { c.infraOk = false; return c; }
    c.accepted = true;
    c.slots = (int)after - (int)before;

    m->emergencyStop();   // discard the whole plan; it never executed
    delay(200);
    return c;
}

// ---------------------------------------------------------------------------
// One fence-versus-watchdog cell. This one is NOT held behind a dwell: the
// whole question is which check fires when the move is genuinely offered, so
// the move must be free to start if the firmware lets it.
// ---------------------------------------------------------------------------
struct FenceCell {
    bool    refusedAtCall;  // the trapezoid command itself reported the fault
    uint8_t fatal;          // the latched code (0xFF if nothing ever faulted)
    int64_t travelled;      // commanded counts covered before the fault, if any
};

static FenceCell runFenceCell(Servomotor* m, int64_t fenceCounts, int64_t windowCounts,
                              int32_t counts, uint32_t ticks) {
    FenceCell r;
    r.refusedAtCall = false; r.fatal = 0xFF; r.travelled = 0;

    tfhReset(m);
    m->zeroPosition();                       // the fence is absolute: fix the origin
    m->setMaximumVelocityRaw(4 * VRAW_PER_ROT_PER_S);
    m->setMaximumAccelerationRaw(2000 * ARAW_PER_ROT_PER_S2);
    m->setSafetyLimitsRaw(-fenceCounts, fenceCounts);
    m->setMaxAllowablePositionDeviationRaw(windowCounts);
    if (m->getError() != 0) { r.fatal = 0xFE; return r; }

    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) { r.fatal = 0xFE; return r; }

    m->trapezoidMoveRaw(counts, ticks);
    if (m->getError() != 0) {                // the queue-time prediction refused it
        r.refusedAtCall = true;
        r.fatal = tfhFatal(m);
        return r;
    }

    // Accepted. Watch for a run-time fault while it executes.
    uint32_t deadline = millis() + (uint32_t)((uint64_t)ticks * 1000ULL / TFH_TPS) + 6000;
    while (millis() < deadline) {
        getStatusResponse st = m->getStatus();
        if (m->getError() == 0 && st.fatalErrorCode != 0) { r.fatal = st.fatalErrorCode; break; }
        int64_t p = m->getPositionRaw();
        if (m->getError() == 0) r.travelled = p - before;
        uint8_t q = m->getNQueuedItems();
        if (m->getError() == 0 && q == 0) { r.fatal = tfhFatal(m); break; }
        delay(15);
    }
    return r;
}

void tm_settings_interaction_matrix(void) {
    Serial.println("tm_settings_interaction_matrix: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // The move used for the whole slot matrix: 0.1 rotation over 2 seconds.
    // Small enough and slow enough that every settings pair below accepts it,
    // so the ONLY thing that varies across the matrix is the plan's shape.
    const int32_t  MATRIX_COUNTS = (int32_t)(TFH_CPR / 10);   // 327680
    const uint32_t MATRIX_TICKS  = 2 * (uint32_t)TFH_TPS;     // 62500

    // =====================================================================
    // 1. THE INSTRUMENT ITSELF. Before trusting any slot count, prove the
    //    queue-holding dwell does what the whole module rests on: it occupies
    //    exactly one slot, and it stops what is queued behind it from running.
    //    If it did not, every measurement below would be reading a partially
    //    drained queue and reporting 2 where the plan says 3.
    // =====================================================================
    printf("\n--- 1. the queue-holding dwell is a valid instrument ---\n");
    {
        tfhReset(m);
        m->setMaximumVelocityRaw(VRAW_PER_ROT_PER_S);
        m->setMaximumAccelerationRaw(2 * VRAW_PER_ROT_PER_S);   // delta_t1 = 8 ticks
        int64_t p0 = m->getPositionRaw();
        m->moveWithVelocityRaw(0, BLOCK_TICKS);
        uint8_t depth1 = m->getNQueuedItems();
        m->trapezoidMoveRaw(MATRIX_COUNTS, MATRIX_TICKS);
        bool queued = (m->getError() == 0);
        uint8_t depth2 = m->getNQueuedItems();
        // A FIXED delay is correct here, not tfhDrain(): this assertion is a
        // NEGATIVE -- that the queue does NOT drain. tfhDrain waits FOR the
        // queue to empty and would simply burn its whole budget. 400 ms is
        // 12500 control ticks, enough to consume an 8-tick ramp 1500 times over
        // if anything behind the dwell were running, and a small fraction of
        // the dwell's own 8 s.
        delay(400);
        uint8_t depth3 = m->getNQueuedItems();
        int64_t p1 = m->getPositionRaw();
        bool readsOk = (m->getError() == 0);
        m->emergencyStop();
        delay(200);

        printf("   dwell alone: depth %u; after the move: depth %u; 400 ms later: depth %u\n",
               (unsigned)depth1, (unsigned)depth2, (unsigned)depth3);
        printf("   commanded position moved %lld counts while held\n", (long long)(p1 - p0));
        TEST_RESULT("a zero-velocity dwell occupies exactly one queue slot",
                    readsOk && depth1 == 1);
        TEST_RESULT("a held three-segment move reads exactly three extra slots",
                    queued && depth2 == 4);
        TEST_RESULT("and nothing behind the dwell executes: depth is unchanged and "
                    "the commanded position has not moved",
                    readsOk && depth3 == depth2 && (p1 - p0) == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 2. THE MATRIX: THE VELOCITY-TO-ACCELERATION RATIO DECIDES THE PLAN.
    //    The velocity ceiling is held at one raw value throughout and only
    //    the acceleration ceiling moves, as exact integer multiples of it, so
    //    the raw division 16 * v / a takes the values 16, 8, 4, 2, 1, 1, 0 --
    //    and the predicted slot cost falls from 3 to 2 as it crosses 2.
    //    (The last cell's raw 0 is corrected to 1 inside compute_trapezoid_move
    //    by the divide-by-zero fix, so the printed delta_t1 reads 1, not 0.
    //    Either way it is on the <= 1 side of the frontier.)
    //
    //    Each cell is measured THREE times, from a fresh reset every time.
    //    The claim is not just "the count is right" but "the count is a
    //    function of the settings alone", which only repetition can show.
    // =====================================================================
    printf("\n--- 2. slot cost across the velocity:acceleration matrix ---\n");
    int cellSlots[7];
    {
        const uint32_t V = VRAW_PER_ROT_PER_S;                 // ~1.000 rot/s
        struct { const char* label; uint32_t araw; } cells[7] = {
            { "a = 1 x v   (1953 rot/s^2)",      1 * V     },
            { "a = 2 x v   (3906 rot/s^2)",      2 * V     },
            { "a = 4 x v   (7812 rot/s^2)",      4 * V     },
            { "a = 8 x v   (15625 rot/s^2)",     8 * V     },   // last 3-slot cell
            { "a = 8 x v + 1 raw count",         8 * V + 1 },   // first 2-slot cell
            { "a = 16 x v  (31250 rot/s^2)",    16 * V     },
            { "a = 20 x v  (39062 rot/s^2)",    20 * V     },
        };

        for (int i = 0; i < 7; i++) {
            Plan p = planTrapezoid(V, cells[i].araw, MATRIX_COUNTS, MATRIX_TICKS);
            int measured[3] = { -2, -2, -2 };
            bool infra = true;
            bool acceptedAll = true;
            for (int t = 0; t < 3; t++) {
                Cell c = measureCell(m, V, cells[i].araw, MATRIX_COUNTS, MATRIX_TICKS);
                if (!c.infraOk) infra = false;
                if (!c.accepted) acceptedAll = false;
                measured[t] = c.slots;
            }
            cellSlots[i] = measured[0];
            printf("   %-28s delta_t1=%lld -> predicted %d slots; trials %d %d %d "
                   "(infra %d, accepted %d)\n",
                   cells[i].label, (long long)p.deltaT1, p.slots,
                   measured[0], measured[1], measured[2],
                   (int)infra, (int)acceptedAll);
            TEST_RESULT(std::string("with ") + cells[i].label +
                            " the move costs the predicted " +
                            std::to_string(p.slots) + " slots",
                        infra && acceptedAll && measured[0] == p.slots);
            // `infra` is part of the condition so this cannot pass VACUOUSLY:
            // three infrastructure failures in a row all report -1 and would
            // otherwise "agree" with each other.
            TEST_RESULT(std::string("with ") + cells[i].label +
                            " three independent trials agree",
                        infra && acceptedAll &&
                        measured[0] == measured[1] && measured[1] == measured[2]);
        }

        // The shape of the sweep as a whole: the cost never rises as the
        // acceleration ceiling rises, and it changes exactly once.
        int transitions = 0;
        bool monotone = true;
        for (int i = 1; i < 7; i++) {
            if (cellSlots[i] > cellSlots[i - 1]) monotone = false;
            if (cellSlots[i] != cellSlots[i - 1]) transitions++;
        }
        printf("   sweep: %d %d %d %d %d %d %d  (transitions %d)\n",
               cellSlots[0], cellSlots[1], cellSlots[2], cellSlots[3],
               cellSlots[4], cellSlots[5], cellSlots[6], transitions);
        TEST_RESULT("the slot cost falls with the ratio and changes exactly once",
                    monotone && transitions == 1);
        // The frontier itself: cells 3 and 4 differ by ONE raw acceleration
        // count and nothing else, and that one count changes the plan.
        TEST_RESULT("one raw acceleration count is the whole difference between "
                    "a 3-slot and a 2-slot plan",
                    cellSlots[3] == 3 && cellSlots[4] == 2);
        tfhReset(m);
    }

    // =====================================================================
    // 3. MAX VELOCITY AGAINST THE COMMANDED DURATION. On the 2-slot path the
    //    planner's whole arithmetic is one integer division, so the shortest
    //    legal duration for a given displacement is exactly predictable. Here
    //    65536 counts at a 1 rot/s ceiling: the velocity the planner computes
    //    is (65536 << 20) / (ticks - 1), and the move is refused the moment
    //    that exceeds the raw ceiling. The frontier lands between 625 and 626
    //    ticks -- a ONE TICK distinction, 32 microseconds.
    // =====================================================================
    printf("\n--- 3. the speed ceiling sets the shortest legal duration, to the tick ---\n");
    {
        const uint32_t V = VRAW_PER_ROT_PER_S;
        const uint32_t A = 20 * V;              // firmly on the 2-slot path
        const int32_t  D = 65536;               // 0.02 rotation
        const uint32_t durations[4] = { 624, 625, 626, 630 };
        int refusals = 0, correctCode = 0, infraFails = 0;
        for (int i = 0; i < 4; i++) {
            Plan p = planTrapezoid(V, A, D, durations[i]);
            Cell c = measureCell(m, V, A, D, durations[i]);
            bool predictedRefusal = (p.refusedWith != 0);
            // Count only cells whose INSTRUMENT worked. measureCell() reports
            // accepted=false and fatal=0 for a bus hiccup or a bad dwell too,
            // and folding those into `refusals` below would make an
            // infrastructure failure read as "the firmware returned the wrong
            // error code" -- a misdiagnosis that costs a debugging cycle. Infra
            // failures are counted and printed separately, and already fail the
            // per-duration assertion on their own.
            if (!c.infraOk) infraFails++;
            else if (!c.accepted) {
                refusals++;
                if (c.fatal == ERR_VEL_TOO_HIGH) correctCode++;
            }
            printf("   %lu ticks: predicted %s, measured accepted=%d fatal=%u (infra %d)\n",
                   (unsigned long)durations[i],
                   predictedRefusal ? "REFUSED" : "accepted",
                   (int)c.accepted, (unsigned)c.fatal, (int)c.infraOk);
            TEST_RESULT(std::string("a 65536-count move over ") +
                            std::to_string((unsigned long)durations[i]) +
                            " ticks is " + (predictedRefusal ? "refused" : "accepted") +
                            " exactly as the integer arithmetic predicts",
                        c.infraOk && (c.accepted == !predictedRefusal));
        }
        printf("   refusals %d, of which ERROR_VEL_TOO_HIGH %d; infra failures %d\n",
               refusals, correctCode, infraFails);
        TEST_RESULT("every duration-driven refusal on the 2-slot path is "
                    "ERROR_VEL_TOO_HIGH (16)",
                    infraFails == 0 && refusals > 0 && refusals == correctCode);

        // Double the ceiling: the frontier must move to 314 ticks, i.e. the
        // coasting time (ticks - 1) halves from 625 to 313.
        const uint32_t V2 = 2 * V;
        const uint32_t durations2[2] = { 313, 314 };
        bool halvedOk = true;
        for (int i = 0; i < 2; i++) {
            Plan p = planTrapezoid(V2, A, D, durations2[i]);
            Cell c = measureCell(m, V2, A, D, durations2[i]);
            bool predictedRefusal = (p.refusedWith != 0);
            printf("   at twice the ceiling, %lu ticks: predicted %s, measured accepted=%d fatal=%u\n",
                   (unsigned long)durations2[i],
                   predictedRefusal ? "REFUSED" : "accepted",
                   (int)c.accepted, (unsigned)c.fatal);
            bool cellOk = c.infraOk && (c.accepted == !predictedRefusal);
            if (!cellOk) halvedOk = false;
            TEST_RESULT(std::string("at twice the speed ceiling, ") +
                            std::to_string((unsigned long)durations2[i]) +
                            " ticks behaves as predicted",
                        cellOk);
        }
        TEST_RESULT("doubling the speed ceiling halves the shortest legal duration "
                    "(626 ticks -> 314, coast 625 -> 313)",
                    halvedOk);
        tfhReset(m);
    }

    // =====================================================================
    // 4. WHICH LIMIT AN OVER-AMBITIOUS MOVE TRIPS DEPENDS ON THE RATIO. The
    //    same impossible move -- one rotation in 0.1 s, roughly 10 rot/s
    //    against a 1 rot/s ceiling -- is refused by BOTH regimes, but with
    //    DIFFERENT error codes, because they queue different kinds of segment:
    //    acceleration items are checked against max_acceleration (15) and
    //    velocity items against max_velocity (16). A caller that keys on the
    //    error code is therefore keying on a settings ratio it may not know it
    //    has chosen.
    // =====================================================================
    printf("\n--- 4. the refusal code depends on which path the ratio selected ---\n");
    {
        const uint32_t V = VRAW_PER_ROT_PER_S;
        const int32_t  D = (int32_t)TFH_CPR;                 // one rotation
        const uint32_t T = (uint32_t)(TFH_TPS / 10);         // 3125 ticks = 0.1 s

        Plan pSlow = planTrapezoid(V, 2 * V, D, T);          // 3-slot regime
        Cell cSlow = measureCell(m, V, 2 * V, D, T);
        Plan pFast = planTrapezoid(V, 20 * V, D, T);         // 2-slot regime
        Cell cFast = measureCell(m, V, 20 * V, D, T);

        // infraOk is printed on both lines so that a failure here is instantly
        // attributable: infra 0 means the dwell/queue instrument did not work
        // and says nothing about the firmware's refusal code.
        printf("   3-slot regime (delta_t1=%lld): predicted code %u, measured %u "
               "(accepted=%d, infra %d)\n",
               (long long)pSlow.deltaT1, (unsigned)pSlow.refusedWith,
               (unsigned)cSlow.fatal, (int)cSlow.accepted, (int)cSlow.infraOk);
        printf("   2-slot regime (delta_t1=%lld): predicted code %u, measured %u "
               "(accepted=%d, infra %d)\n",
               (long long)pFast.deltaT1, (unsigned)pFast.refusedWith,
               (unsigned)cFast.fatal, (int)cFast.accepted, (int)cFast.infraOk);

        TEST_RESULT("in the 3-slot regime an impossible move is refused with "
                    "ERROR_ACCEL_TOO_HIGH (15)",
                    cSlow.infraOk && !cSlow.accepted && cSlow.fatal == ERR_ACCEL_TOO_HIGH);
        TEST_RESULT("in the 2-slot regime the same move is refused with "
                    "ERROR_VEL_TOO_HIGH (16)",
                    cFast.infraOk && !cFast.accepted && cFast.fatal == ERR_VEL_TOO_HIGH);
        TEST_RESULT("either way the move is refused rather than silently reshaped",
                    cSlow.infraOk && cFast.infraOk &&
                    !cSlow.accepted && !cFast.accepted && cSlow.fatal != cFast.fatal);
        tfhReset(m);
    }

    // =====================================================================
    // 5. THE SAFETY FENCE AGAINST THE DEVIATION WINDOW. Three cells, same
    //    move (three rotations in two seconds) and same limits except for
    //    which guard is armed:
    //
    //      a) fence tight, window wide  -> refused when commanded, code 27
    //      b) fence wide,  window tight -> accepted, then faults mid-move, 45
    //      c) BOTH tight                -> must behave exactly like (a)
    //
    //    (c) is the question the module exists to answer, and it is settled by
    //    structure rather than by timing: the fence is evaluated inside
    //    add_to_queue() before the item is counted into the queue, while the
    //    watchdog cannot speak until the shaft has been commanded past its
    //    window. Measured margin: the first ramp segment is 62 ticks and
    //    covers under 5000 counts before the fence rejects the coast segment,
    //    against a 819200-count window -- so the outcome is not a close call.
    // =====================================================================
    printf("\n--- 5. the fence is a queue-time check and always beats the watchdog ---\n");
    {
        const int64_t  TIGHT = (int64_t)(TFH_CPR / 4);        // 0.25 rotation
        const int64_t  WIDE_FENCE  = (int64_t)TFH_CPR * 1000;
        const int64_t  WIDE_WINDOW = (int64_t)TFH_CPR * 1000;
        const int32_t  MOVE = (int32_t)(TFH_CPR * 3);         // three rotations
        const uint32_t MOVE_TICKS = 2 * (uint32_t)TFH_TPS;

        FenceCell a = runFenceCell(m, TIGHT, WIDE_WINDOW, MOVE, MOVE_TICKS);
        printf("   a) fence 0.25 rot, window wide : refusedAtCall=%d fatal=%u\n",
               (int)a.refusedAtCall, (unsigned)a.fatal);
        TEST_RESULT("a move past the fence alone is refused at the moment it is commanded",
                    a.refusedAtCall);
        TEST_RESULT("and the fence reports ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE (27)",
                    a.fatal == ERR_PREDICTED_POS_OUT_OF_ZONE);
        tfhReset(m);

        FenceCell b = runFenceCell(m, WIDE_FENCE, TIGHT, MOVE, MOVE_TICKS);
        printf("   b) fence wide, window 0.25 rot : refusedAtCall=%d fatal=%u after %lld counts\n",
               (int)b.refusedAtCall, (unsigned)b.fatal, (long long)b.travelled);
        // `b.fatal != 0xFE` is the guard against a VACUOUS pass: runFenceCell
        // reports refusedAtCall=false both when the move was genuinely accepted
        // and when its own setup (zero / limits / first position read) failed,
        // and 0xFE is the sentinel for the latter.
        TEST_RESULT("a move past the deviation window alone is ACCEPTED when commanded",
                    !b.refusedAtCall && b.fatal != 0xFE);
        TEST_RESULT("and it faults only once it is moving, with "
                    "ERROR_POSITION_DEVIATION_TOO_LARGE (45)",
                    b.fatal == ERR_DEVIATION_TOO_LARGE);
        tfhReset(m);

        // (c) three trials, because "which one fires" is exactly the kind of
        // claim that deserves more than a single observation.
        bool allRefusedAtCall = true;
        bool allCode27 = true;
        for (int t = 0; t < 3; t++) {
            FenceCell c = runFenceCell(m, TIGHT, TIGHT, MOVE, MOVE_TICKS);
            printf("   c) both tight, trial %d      : refusedAtCall=%d fatal=%u\n",
                   t + 1, (int)c.refusedAtCall, (unsigned)c.fatal);
            if (!c.refusedAtCall) allRefusedAtCall = false;
            if (c.fatal != ERR_PREDICTED_POS_OUT_OF_ZONE) allCode27 = false;
            tfhReset(m);
        }
        TEST_RESULT("a move violating BOTH guards is refused at the call, three times "
                    "out of three",
                    allRefusedAtCall);
        TEST_RESULT("and it always reports the fence's code 27, never the watchdog's 45",
                    allCode27);
        TEST_RESULT("so the queue-time fence strictly precedes the run-time watchdog",
                    a.refusedAtCall && !b.refusedAtCall && allRefusedAtCall && allCode27);
        tfhReset(m);
    }

    // =====================================================================
    // 6. STILL HEALTHY. The matrix deliberately faulted the machine many
    //    times over; an ordinary move at the factory limits must still work.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), (uint32_t)TFH_TPS, &ok);
        printf("   post-matrix move delivered %lld of %ld counts\n",
               (long long)moved, (long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move still works after the whole interaction matrix",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
        tfhReset(m);
    }

    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
