#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_safety_fence_arithmetic.cpp
//
// THE ARITHMETIC OF THE SAFETY FENCE, AT ITS EXTREMES AND AT ITS EXACT EDGE.
//
// "Set safety limits" (cmd 30) is the only mechanism in this machine that can
// be told "never go past here". It is not advisory and it is not a clamp: the
// firmware compares against it in two independent places, and both of them are
// plain integer comparisons whose exact form is the whole contract.
//
//   * QUEUE TIME, on the predicted end of the queue -- motor_control.c:1812 in
//     add_to_queue()'s acceleration branch and :1855 in its velocity branch
//     (the identical-looking pair at :1910 / :1953 belongs to
//     add_to_queue_test(), the cmd-120 dry-run validator, which nothing here
//     ever sends):
//         if((predicted_final_position_i64 < position_lower_safety_limit_i64) ||
//            (predicted_final_position_i64 > position_upper_safety_limit_i64))
//             fatal_error(ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE);   // 27
//     and, for an acceleration segment that reverses direction, the same
//     comparison on the turn point (motor_control.c:1831) -> error 26.
//
//   * EVERY CONTROL TICK, on where the machine actually is
//     (motor_control.c:2640):
//         if((current_position_i64 > position_upper_safety_limit_i64) ||
//            (current_position_i64 < position_lower_safety_limit_i64))
//             fatal_error(ERROR_SAFETY_LIMIT_EXCEEDED);                   // 25
//
//   * and a THIRD, earlier check in the command handler itself
//     (main.c:709-711):
//         if (limits_input.lower_limit > limits_input.upper_limit)
//             fatal_error(ERROR_PARAMETER_OUT_OF_RANGE);                  // 34
//
// Those are `<` and `>`, never `<=` or `>=`. So the limits are INCLUSIVE: a
// position exactly equal to a limit is legal, and it is the very next count
// that is not. And the parameter check is `>`, not `>=`, so a degenerate fence
// with lower == upper is a legal fence rather than a rejected one. Nothing in
// the suite tests either of those single characters, and both are exactly the
// kind of detail a refactor silently inverts.
//
// WHAT THIS MODULE ASSERTS, as one property: the fence is a pair of ordinary
// int64 comparisons that behave correctly across the ENTIRE int64 line --
// permissive at the extremes, degenerate but valid at zero width, rejected when
// inverted, one-sided when asymmetric, and exact to the individual count at the
// boundary -- and that it lives only until the next reset
// (motor_control.c:3726-3727 restore INT64_MIN / INT64_MAX).
//
// HOW THIS IS DISTINCT FROM tm_safety_fence.cpp. That module establishes that
// the fence exists and works at all (a +/-0.5 rotation fence accepts an
// in-bounds move and trips on an out-of-bounds one) and then spends most of its
// length on the interplay with "Zero position" -- how the fence follows a
// re-zeroed frame. This module does the opposite: it NEVER calls zeroPosition,
// works in raw encoder counts rather than shaft rotations, and attacks the
// comparison arithmetic itself. Everything here is out of scope there:
// int64-extreme fences, zero-width fences, inverted fences and their distinct
// error code, one-sided fences, count-exact boundary inclusivity, last-write-
// wins, and non-persistence across a reset.
//
// WHY THE MEASUREMENTS ARE EXACT. Most sections drive the machine with
// CONSTANT-VELOCITY segments rather than trapezoid moves. A velocity segment
// advances the position accumulator by (raw_velocity << VELOCITY_SHIFT_LEFT)
// per tick (motor_control.h:65, motor_control.c:1842) into a value carrying 32
// fractional bits (motor_control.c:142-152), so with a raw velocity that is a
// multiple of 2^20 the displacement is a whole number of counts with no
// rounding whatsoever: counts per tick == raw_velocity / 2^20. The velocity
// branch of add_to_queue also has NO turn-point check, so the only fence code
// it can possibly raise is 27 -- a sharper claim than "one of 25/26/27".
// Finally, the firmware itself guarantees the measured endpoint equals the
// predicted one: when the queue empties it compares the two and raises
// ERROR_POSITION_DISCREPANCY if they differ (motor_control.c:2152). So reading
// the landed position with getPositionRaw() reads precisely the number the
// fence check was comparing.
//
// SAFETY. The MOSFETs stay OFF for the entire module: enableMosfets() and
// goToClosedLoop() are never called, and nothing here can turn a shaft. Every
// distance is read from the COMMANDED position, which the planner advances
// whether or not the output stage is energised, so the fence -- which acts on
// commanded and predicted positions -- is fully exercised with the machine
// mechanically inert. This runs on a SHARED 35-motor rack: there is no
// broadcast anywhere, no setDeviceAlias, and no testMode; every command is
// addressed to the one motor tfGetMotor() holds by unique ID. Deliberate faults
// (25, 27, 34) are the point of several sections; each is cleared by tfhReset()
// before the next, and the module ends by leaving the motor reset and clean.
// Commanded excursions are at most four rotations, and tfhFreshOpen() raises
// the two-rotation deviation window first so error 45 never masquerades as a
// fence result.
// ---------------------------------------------------------------------------

static const uint8_t ERR_SAFETY_LIMIT_EXCEEDED       = 25;
static const uint8_t ERR_TURN_POINT_OUT_OF_SAFETY    = 26;
static const uint8_t ERR_PREDICTED_POS_OUT_OF_SAFETY = 27;
static const uint8_t ERR_PARAMETER_OUT_OF_RANGE      = 34;

// counts per control tick == raw velocity / 2^20 (see the header comment).
static const int64_t VRAW_PER_COUNT_PER_TICK = 1048576;   // 2^20

static const int32_t  LEG_CPT    = 32;                              // counts per tick
static const uint32_t LEG_TICKS  = (uint32_t)TFH_TPS;               // exactly one second
static const int64_t  LEG_COUNTS = (int64_t)LEG_CPT * (int64_t)TFH_TPS;  // 1,000,000 exactly

static const int32_t BIG_MOVE_COUNTS = (int32_t)(TFH_CPR * 4);      // 13,107,200
static const int64_t NARROW_COUNTS   = 100000;                      // ~0.03 rotations

// Build an assertion name that carries the numbers it is talking about, so a
// failure line is self-describing in the log.
static std::string sfmt(const char* fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return std::string(buf);
}

// What a command attempt actually did.
//   replyErr : the code carried by the command's OWN reply. The library returns
//              a device-reported error code POSITIVELY and reserves -1..-9 for
//              local comms failures (Communication.h:11-20), so a positive value
//              here is the firmware's error number verbatim.
//   fatal    : the code latched in the machine afterwards, read via get_status
//              (the only command that still answers once a fault is latched).
struct Outcome {
    bool     accepted;    // queued/accepted with no error reply
    bool     completed;   // ran to an empty queue with nothing latched
    int      replyErr;
    uint8_t  fatal;
    int64_t  moved;       // commanded-position delta; meaningful only if completed
};

static Outcome blankOutcome() {
    Outcome o;
    o.accepted = false;
    o.completed = false;
    o.replyErr = 0;
    o.fatal = 0;
    o.moved = 0;
    return o;
}

// ---------------------------------------------------------------------------
// Install a fence in RAW ENCODER COUNTS -- the same domain the firmware
// compares in (Servomotor.cpp setSafetyLimitsRaw sends the two int64s
// unconverted). The float setSafetyLimits() would round through a float and
// could not express a limit that is exact to one count, which is the whole
// point of section 6.
//
// The short settle afterwards is not padding: a fence that excludes the CURRENT
// position is perfectly well-formed and is caught by the per-tick check
// (motor_control.c:2640) some microseconds later, not by the command handler.
// Reading the latched code immediately would miss it.
// ---------------------------------------------------------------------------
static void setFence(Servomotor* m, int64_t lo, int64_t hi, int* replyErr, uint8_t* fatal) {
    m->setSafetyLimitsRaw(lo, hi);
    *replyErr = m->getError();
    delay(250);
    *fatal = tfhFatal(m);
}

// ---------------------------------------------------------------------------
// One constant-velocity leg of `countsPerTick` counts per control tick for
// `ticks` ticks, terminated by an explicit zero-velocity segment.
//
// The terminator is mandatory, not tidiness: a velocity segment HOLDS its
// velocity after its duration expires, and if the queue empties while the
// velocity is nonzero the firmware raises ERROR_RUN_OUT_OF_QUEUE_ITEMS
// (motor_control.c:2164). Legs here are a full second long, so the terminator
// is always queued with a huge margin before the first segment runs out.
// ---------------------------------------------------------------------------
static Outcome velocityLeg(Servomotor* m, int32_t countsPerTick, uint32_t ticks) {
    Outcome o = blankOutcome();

    int64_t before = m->getPositionRaw();
    bool haveBefore = (m->getError() == 0);

    int32_t raw = (int32_t)((int64_t)countsPerTick * VRAW_PER_COUNT_PER_TICK);
    m->moveWithVelocityRaw(raw, ticks);
    o.replyErr = m->getError();
    if (o.replyErr != 0) { o.fatal = tfhFatal(m); return o; }

    m->moveWithVelocityRaw(0, 1);          // terminator: see the comment above
    int termErr = m->getError();
    if (termErr != 0) { o.replyErr = termErr; o.fatal = tfhFatal(m); return o; }

    o.accepted = true;
    uint32_t budget = (uint32_t)((uint64_t)ticks * 1000ULL / (uint64_t)TFH_TPS) + 8000;
    if (!tfhDrain(m, budget)) { o.fatal = tfhFatal(m); return o; }
    o.fatal = tfhFatal(m);
    if (o.fatal != 0) return o;

    o.completed = true;
    if (haveBefore) o.moved = m->getPositionRaw() - before;
    return o;
}

// One trapezoid move, reported the same way. Used where the point is the shape
// of a real move rather than count-exact arithmetic; a trapezoid can trip 26 as
// well as 27 because its deceleration segment has a turn point.
static Outcome trapLeg(Servomotor* m, int32_t counts, uint32_t ticks) {
    Outcome o = blankOutcome();

    int64_t before = m->getPositionRaw();
    bool haveBefore = (m->getError() == 0);

    m->trapezoidMoveRaw(counts, ticks);
    o.replyErr = m->getError();
    if (o.replyErr != 0) { o.fatal = tfhFatal(m); return o; }

    o.accepted = true;
    uint32_t budget = (uint32_t)((uint64_t)ticks * 1000ULL / (uint64_t)TFH_TPS) + 10000;
    if (!tfhDrain(m, budget)) { o.fatal = tfhFatal(m); return o; }
    o.fatal = tfhFatal(m);
    if (o.fatal != 0) return o;

    o.completed = true;
    if (haveBefore) o.moved = m->getPositionRaw() - before;
    return o;
}

// Any of the three ways the fence can refuse motion.
static bool isFenceCode(uint8_t code) {
    return code == ERR_SAFETY_LIMIT_EXCEEDED ||
           code == ERR_TURN_POINT_OUT_OF_SAFETY ||
           code == ERR_PREDICTED_POS_OUT_OF_SAFETY;
}

// ---------------------------------------------------------------------------
// "The move was not obstructed": did a TRAPEZOID move deliver what was asked?
//
// A trapezoid move does NOT land on an exact count, so this is a band and not
// an equality. compute_trapezoid_move() truncates the derived acceleration
// (motor_control.c:485) and queue_trapezoid_segments() then delivers exactly
// a_truncated * delta_t1 * (delta_t1 + delta_t2), so the shortfall is up to
// delta_t1 * (delta_t1 + delta_t2) / 2^24 counts, plus one more for the
// fraction the int96 accumulator still holds below the reported count
// (motor_control.c:143-152). Under the ceilings tfhOpenCeilings installs that
// works out to about one count for the moves in this module -- but it scales
// with the DURATION and with the ramp length, and it would grow by orders of
// magnitude if the profile ever became triangular, which is exactly the sort of
// thing a changed default would do. The campaign has already measured 7 counts
// short on a long move, so a bare "within 8 counts" is not a safe standing
// claim, and this module is not the place that owns planner precision
// (tm_position_accumulator is). Use the suite's usual 0.1%-plus-a-few band: it
// still fails loudly on the outcomes these sections actually care about -- a
// refusal, no motion at all, a truncated move, or the wrong direction.
// ---------------------------------------------------------------------------
static bool deliveredInFull(int64_t moved, int64_t commanded) {
    int64_t tolerance = llabs((long long)commanded) / 1000 + 8;
    return llabs((long long)(moved - commanded)) <= tolerance;
}

void tm_safety_fence_arithmetic(void) {
    Serial.println("tm_safety_fence_arithmetic: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE FACTORY DEFAULT IS THE WHOLE int64 LINE, AND SAYING SO
    //    EXPLICITLY CHANGES NOTHING.
    //
    //    position_lower/upper_safety_limit_i64 start at INT64_MIN/INT64_MAX
    //    (motor_control.c:192-193). This section is the control for every
    //    later section: it shows that a four-rotation move is unobstructed
    //    when there is no fence, so that a refusal later is attributable to
    //    the fence and not to some other ceiling. It also shows that writing
    //    the extreme values by hand is a no-op, which is the cheapest
    //    possible check that the int64 payload survives the wire intact --
    //    a byte-order or width bug in the 8-byte limits would land somewhere
    //    other than the extremes and start refusing this move.
    // =====================================================================
    printf("\n--- 1. the default fence is the entire int64 range ---\n");
    {
        tfhFreshOpen(m);
        Outcome free1 = trapLeg(m, BIG_MOVE_COUNTS, tfhTicksFor(BIG_MOVE_COUNTS));
        printf("   unfenced: accepted=%d completed=%d moved=%lld of %ld fatal=%u\n",
               (int)free1.accepted, (int)free1.completed,
               (long long)free1.moved, (long)BIG_MOVE_COUNTS, (unsigned)free1.fatal);
        TEST_RESULT("with no fence set, a four-rotation move runs to completion",
                    free1.completed);
        TEST_RESULT("and it delivers its full four rotations",
                    free1.completed && deliveredInFull(free1.moved, BIG_MOVE_COUNTS));

        tfhFreshOpen(m);
        int reply = 0; uint8_t fatal = 0;
        setFence(m, INT64_MIN, INT64_MAX, &reply, &fatal);
        printf("   fence [INT64_MIN, INT64_MAX]: reply=%d fatal=%u\n", reply, (unsigned)fatal);
        TEST_RESULT("a fence spanning the whole int64 range is accepted without fault",
                    reply == 0 && fatal == 0);

        Outcome free2 = trapLeg(m, BIG_MOVE_COUNTS, tfhTicksFor(BIG_MOVE_COUNTS));
        printf("   fenced at the extremes: completed=%d moved=%lld fatal=%u\n",
               (int)free2.completed, (long long)free2.moved, (unsigned)free2.fatal);
        // This one CAN stay tight: it compares two runs of the identical
        // computation from the identical starting state, so the difference is
        // not a rounding question at all -- the planner is deterministic
        // (established by tm_determinism) and the two figures should be equal.
        TEST_RESULT("naming the int64 extremes explicitly is indistinguishable from no fence",
                    free2.completed &&
                    llabs((long long)(free2.moved - free1.moved)) <= 8);
        tfhReset(m);
    }

    // =====================================================================
    // 2. A FENCE NEAR THE int64 EXTREMES PERMITS EVERYTHING.
    //
    //    The comparisons at motor_control.c:1812 and :2640 are signed int64
    //    against signed int64. Limits within one count of INT64_MIN/INT64_MAX,
    //    and limits at +/-2^62, must therefore behave identically to no fence
    //    at all. If any of these started refusing a four-rotation move it
    //    would mean the limits are being truncated, sign-extended wrongly, or
    //    compared after an arithmetic step that overflows -- the classic way a
    //    64-bit guard band silently becomes a 32-bit one.
    // =====================================================================
    printf("\n--- 2. fences at the int64 extremes obstruct nothing ---\n");
    {
        const int64_t los[3] = { INT64_MIN + 1, -((int64_t)1 << 62), INT64_MIN };
        const int64_t his[3] = { INT64_MAX - 1,  ((int64_t)1 << 62), ((int64_t)1 << 62) };
        const char*   tags[3] = { "INT64_MIN+1 .. INT64_MAX-1",
                                  "-2^62 .. +2^62",
                                  "INT64_MIN .. +2^62" };
        for (int i = 0; i < 3; i++) {
            tfhFreshOpen(m);
            int reply = 0; uint8_t fatal = 0;
            setFence(m, los[i], his[i], &reply, &fatal);
            Outcome o = trapLeg(m, BIG_MOVE_COUNTS, tfhTicksFor(BIG_MOVE_COUNTS));
            printf("   [%s]: reply=%d fatal=%u -> completed=%d moved=%lld fatal=%u\n",
                   tags[i], reply, (unsigned)fatal,
                   (int)o.completed, (long long)o.moved, (unsigned)o.fatal);
            TEST_RESULT(sfmt("an enormous fence [%s] is installed without fault", tags[i]),
                        reply == 0 && fatal == 0);
            TEST_RESULT(sfmt("a four-rotation move is unobstructed by the fence [%s]", tags[i]),
                        o.completed && deliveredInFull(o.moved, BIG_MOVE_COUNTS));
        }
        tfhReset(m);
    }

    // =====================================================================
    // 3. A ZERO-WIDTH FENCE PERMITS A DWELL AND NOTHING ELSE.
    //
    //    lower == upper is NOT rejected -- the parameter check at main.c:709
    //    is `>`, not `>=`. It leaves exactly one legal position, so the machine
    //    is pinned: it may sit there indefinitely, and any commanded
    //    displacement in either direction is refused before it is queued.
    //    This is the strongest available statement that the fence is a genuine
    //    two-sided closed interval and not, say, a half-open one.
    //
    //    The fence is installed at [0, 0] immediately after a reset, when the
    //    commanded position is exactly 0, so the pinned position IS the current
    //    one and the per-tick check has nothing to complain about.
    // =====================================================================
    printf("\n--- 3. a zero-width fence permits only a dwell ---\n");
    {
        tfhFreshOpen(m);
        int reply = 0; uint8_t fatal = 0;
        setFence(m, 0, 0, &reply, &fatal);
        printf("   fence [0,0]: reply=%d fatal=%u\n", reply, (unsigned)fatal);
        TEST_RESULT("a fence with lower == upper is accepted, not rejected as a bad parameter",
                    reply == 0);
        TEST_RESULT("and a zero-width fence around the present position raises no fault",
                    fatal == 0);

        // A dwell: one second of exactly zero velocity. The predicted end of
        // the queue is 0, which is inside the closed interval [0, 0].
        Outcome dwell = velocityLeg(m, 0, LEG_TICKS);
        int64_t restingPos = dwell.completed ? m->getPositionRaw() : -1;
        printf("   dwell: completed=%d moved=%lld resting position=%lld fatal=%u\n",
               (int)dwell.completed, (long long)dwell.moved,
               (long long)restingPos, (unsigned)dwell.fatal);
        TEST_RESULT("a one-second dwell is legal inside a zero-width fence",
                    dwell.completed && dwell.moved == 0);
        TEST_RESULT("and the dwell leaves the machine exactly on the pinned position",
                    dwell.completed && restingPos == 0);

        // Any displacement at all, either way, is refused at queue time.
        Outcome fwd = velocityLeg(m, LEG_CPT, LEG_TICKS);
        printf("   +%lld counts under [0,0]: reply=%d fatal=%u\n",
               (long long)LEG_COUNTS, fwd.replyErr, (unsigned)fwd.fatal);
        TEST_RESULT("a forward move inside a zero-width fence is refused with error 27",
                    fwd.fatal == ERR_PREDICTED_POS_OUT_OF_SAFETY &&
                    fwd.replyErr == (int)ERR_PREDICTED_POS_OUT_OF_SAFETY);

        tfhFreshOpen(m);
        setFence(m, 0, 0, &reply, &fatal);
        Outcome rev = velocityLeg(m, -LEG_CPT, LEG_TICKS);
        printf("   -%lld counts under [0,0]: reply=%d fatal=%u\n",
               (long long)LEG_COUNTS, rev.replyErr, (unsigned)rev.fatal);
        TEST_RESULT("a reverse move inside a zero-width fence is refused just the same",
                    rev.fatal == ERR_PREDICTED_POS_OUT_OF_SAFETY);
        tfhReset(m);
    }

    // =====================================================================
    // 4. A FENCE THAT EXCLUDES THE PRESENT POSITION IS VALID, AND FAULTS ON
    //    THE NEXT CONTROL TICK.
    //
    //    This is the case that separates the two rejection mechanisms, and
    //    getting it backwards would be a serious safety regression. A fence
    //    the machine is ALREADY outside of is well-formed: lower <= upper, so
    //    the handler stores it. It is the per-tick check (motor_control.c:2640)
    //    that then notices, within 32 microseconds, that the present position
    //    is out of bounds and raises ERROR_SAFETY_LIMIT_EXCEEDED (25) -- a
    //    different code from the inverted-fence rejection in section 5, and
    //    the correct one, because nothing about the parameters was malformed.
    //
    //    The command's own reply is deliberately NOT asserted here: the fault
    //    is raised from the control ISR and races the reply, so the host may
    //    legitimately receive either a success or the code 25. Exactly one
    //    packet is sent either way (error_handling.c:216-219 drains an
    //    in-flight reply and cancels the deferred one), so the bus stays in
    //    step; only the latched code is a stable claim.
    // =====================================================================
    printf("\n--- 4. a fence excluding the present position faults on the next tick ---\n");
    {
        tfhFreshOpen(m);
        int reply = 0; uint8_t fatal = 0;
        setFence(m, 1000, 1000, &reply, &fatal);      // pinned somewhere we are not
        printf("   zero-width fence [1000,1000] at position 0: reply=%d fatal=%u\n",
               reply, (unsigned)fatal);
        TEST_RESULT("a zero-width fence away from the present position raises error 25",
                    fatal == ERR_SAFETY_LIMIT_EXCEEDED);
        TEST_RESULT("it is a valid fence, not a rejected parameter (not error 34)",
                    fatal != ERR_PARAMETER_OUT_OF_RANGE);

        tfhFreshOpen(m);
        setFence(m, 1000, 2000, &reply, &fatal);      // a wide fence, just not here
        printf("   wide fence [1000,2000] at position 0: reply=%d fatal=%u\n",
               reply, (unsigned)fatal);
        TEST_RESULT("a wide fence that still excludes the present position also raises 25",
                    fatal == ERR_SAFETY_LIMIT_EXCEEDED);
        tfhReset(m);
    }

    // =====================================================================
    // 5. AN INVERTED FENCE IS REJECTED OUTRIGHT AS A PARAMETER ERROR.
    //
    //    lower > upper describes an empty interval: EVERY position is out of
    //    bounds, including the one the machine is standing on. The firmware
    //    refuses to store it at all (main.c:709-711) and answers
    //    ERROR_PARAMETER_OUT_OF_RANGE (34), rather than storing it and letting
    //    the control loop discover the contradiction one tick later. That
    //    distinction is not cosmetic: the comment on that line records that
    //    the deferred route used to race this command's own reply, and on
    //    firmware up to 0.15.4.0 it could truncate the reply and leave the
    //    device answering nothing at all until a power cycle. So "34, from the
    //    handler" is the regression guard for a hang, not just a nicer code.
    //
    //    The cases below include both ends of the int64 line, so a comparison
    //    that was accidentally unsigned would fail here loudly.
    // =====================================================================
    printf("\n--- 5. an inverted fence is rejected with error 34 ---\n");
    {
        const int64_t los[4] = { 1, 0, INT64_MAX, INT64_MIN + 1 };
        const int64_t his[4] = { 0, -1, INT64_MIN, INT64_MIN };
        const char*   tags[4] = { "[1, 0]", "[0, -1]",
                                  "[INT64_MAX, INT64_MIN]", "[INT64_MIN+1, INT64_MIN]" };
        bool allAnsweredWith34 = true;
        for (int i = 0; i < 4; i++) {
            tfhFreshOpen(m);
            int reply = 0; uint8_t fatal = 0;
            setFence(m, los[i], his[i], &reply, &fatal);
            printf("   inverted %s: reply=%d fatal=%u\n", tags[i], reply, (unsigned)fatal);
            if (reply != (int)ERR_PARAMETER_OUT_OF_RANGE) allAnsweredWith34 = false;
            TEST_RESULT(sfmt("the inverted fence %s is rejected with ERROR_PARAMETER_OUT_OF_RANGE (34)",
                             tags[i]),
                        fatal == ERR_PARAMETER_OUT_OF_RANGE);
        }
        // The host must learn about it from the reply to the command it sent,
        // not by having to go and ask afterwards.
        TEST_RESULT("every inverted fence answers the setting command itself with code 34",
                    allAnsweredWith34);
        tfhReset(m);
    }

    // =====================================================================
    // 6. THE BOUNDARY IS INCLUSIVE, AND EXACT TO ONE COUNT.
    //
    //    The method: run a constant-velocity leg with no fence and MEASURE
    //    where it lands, then repeat the identical leg twice more -- once with
    //    the fence set exactly to that landing point, once with it one single
    //    count tighter. The first must be accepted (`>` is not `>=`), the
    //    second must be refused. Nothing is predicted from theory except the
    //    displacement itself, and the firmware guarantees the measurement is
    //    the same number the fence check compares, because it raises
    //    ERROR_POSITION_DISCREPANCY if the executed and predicted endpoints
    //    ever disagree (motor_control.c:2152).
    //
    //    A one-count experiment is only meaningful if the arithmetic is exact,
    //    which is why this uses velocity segments rather than a trapezoid: the
    //    displacement is an exact multiple of one count (header comment), and
    //    the velocity branch of add_to_queue has no turn-point check, so the
    //    only possible refusal code is 27.
    // =====================================================================
    printf("\n--- 6. the boundary is inclusive, to the individual count ---\n");
    {
        tfhFreshOpen(m);
        Outcome ref = velocityLeg(m, LEG_CPT, LEG_TICKS);
        // Fall back to the arithmetic prediction rather than to 0 if the
        // reference leg did not complete: a landing of 0 would silently turn
        // every fence below into a zero-width one and produce a cascade of
        // failures that say nothing about the boundary. With the predicted
        // value the rest of the section still asks a real question.
        int64_t landing = ref.completed ? ref.moved : LEG_COUNTS;
        printf("   unfenced leg landed at %lld (expected exactly %lld)\n",
               (long long)landing, (long long)LEG_COUNTS);
        TEST_RESULT("a constant-velocity leg advances an exact whole number of counts",
                    ref.completed && landing == LEG_COUNTS);

        // Upper limit set to the exact landing point: legal.
        tfhFreshOpen(m);
        int reply = 0; uint8_t fatal = 0;
        setFence(m, -TFH_CPR, landing, &reply, &fatal);
        Outcome onEdge = velocityLeg(m, LEG_CPT, LEG_TICKS);
        int64_t restedAt = onEdge.completed ? m->getPositionRaw() : -1;
        printf("   fence upper == %lld: completed=%d rested at %lld fatal=%u\n",
               (long long)landing, (int)onEdge.completed,
               (long long)restedAt, (unsigned)onEdge.fatal);
        TEST_RESULT("a move landing exactly on the upper limit is accepted",
                    onEdge.completed && onEdge.moved == landing);
        TEST_RESULT("and it comes to rest precisely on the limit, not short of it",
                    onEdge.completed && restedAt == landing);

        // One count tighter: refused.
        tfhFreshOpen(m);
        setFence(m, -TFH_CPR, landing - 1, &reply, &fatal);
        Outcome overEdge = velocityLeg(m, LEG_CPT, LEG_TICKS);
        printf("   fence upper == %lld (one count tighter): reply=%d fatal=%u\n",
               (long long)(landing - 1), overEdge.replyErr, (unsigned)overEdge.fatal);
        TEST_RESULT("moving the upper limit down by ONE count refuses the same move (27)",
                    overEdge.fatal == ERR_PREDICTED_POS_OUT_OF_SAFETY);

        // The mirror image on the lower limit.
        tfhFreshOpen(m);
        setFence(m, -landing, TFH_CPR, &reply, &fatal);
        Outcome onEdgeNeg = velocityLeg(m, -LEG_CPT, LEG_TICKS);
        printf("   fence lower == %lld: completed=%d moved=%lld fatal=%u\n",
               (long long)(-landing), (int)onEdgeNeg.completed,
               (long long)onEdgeNeg.moved, (unsigned)onEdgeNeg.fatal);
        TEST_RESULT("a move landing exactly on the lower limit is accepted too",
                    onEdgeNeg.completed && onEdgeNeg.moved == -landing);

        tfhFreshOpen(m);
        setFence(m, -landing + 1, TFH_CPR, &reply, &fatal);
        Outcome overEdgeNeg = velocityLeg(m, -LEG_CPT, LEG_TICKS);
        printf("   fence lower == %lld (one count tighter): reply=%d fatal=%u\n",
               (long long)(-landing + 1), overEdgeNeg.replyErr, (unsigned)overEdgeNeg.fatal);
        TEST_RESULT("raising the lower limit by ONE count refuses the mirror move (27)",
                    overEdgeNeg.fatal == ERR_PREDICTED_POS_OUT_OF_SAFETY);
        tfhReset(m);
    }

    // =====================================================================
    // 7. AN ASYMMETRIC FENCE BLOCKS ONLY THE SIDE IT SHOULD.
    //
    //    The two limits are independent. A fence that is tight below and open
    //    above must let the machine run freely upward while stopping it dead
    //    downward, and vice versa. This is the shape a real installation
    //    usually has -- one hard stop and one free direction -- and a fence
    //    that blocked both directions, or the wrong one, would be either
    //    useless or actively dangerous. Trapezoid moves are used here because
    //    this is about ordinary motion rather than count-exact arithmetic, so
    //    any of the three fence codes is a correct refusal.
    // =====================================================================
    printf("\n--- 7. an asymmetric fence blocks only its own side ---\n");
    {
        const int32_t oneRot = (int32_t)TFH_CPR;
        const uint32_t rotTicks = tfhTicksFor(TFH_CPR);

        // Tight below, open above.
        tfhFreshOpen(m);
        int reply = 0; uint8_t fatal = 0;
        setFence(m, -NARROW_COUNTS, (int64_t)BIG_MOVE_COUNTS, &reply, &fatal);
        Outcome up = trapLeg(m, oneRot, rotTicks);
        printf("   [-%lld, +%ld]: upward completed=%d moved=%lld fatal=%u\n",
               (long long)NARROW_COUNTS, (long)BIG_MOVE_COUNTS,
               (int)up.completed, (long long)up.moved, (unsigned)up.fatal);
        TEST_RESULT("a fence tight below and open above lets a full rotation upward through",
                    up.completed && deliveredInFull(up.moved, oneRot));

        tfhFreshOpen(m);
        setFence(m, -NARROW_COUNTS, (int64_t)BIG_MOVE_COUNTS, &reply, &fatal);
        Outcome down = trapLeg(m, -oneRot, rotTicks);
        printf("   [-%lld, +%ld]: downward accepted=%d fatal=%u\n",
               (long long)NARROW_COUNTS, (long)BIG_MOVE_COUNTS,
               (int)down.accepted, (unsigned)down.fatal);
        TEST_RESULT("the same fence stops the identical move in the downward direction",
                    isFenceCode(down.fatal));

        // Tight above, open below: the mirror image.
        tfhFreshOpen(m);
        setFence(m, -(int64_t)BIG_MOVE_COUNTS, NARROW_COUNTS, &reply, &fatal);
        Outcome down2 = trapLeg(m, -oneRot, rotTicks);
        printf("   [-%ld, +%lld]: downward completed=%d moved=%lld fatal=%u\n",
               (long)BIG_MOVE_COUNTS, (long long)NARROW_COUNTS,
               (int)down2.completed, (long long)down2.moved, (unsigned)down2.fatal);
        TEST_RESULT("mirroring the fence mirrors which direction is free",
                    down2.completed && deliveredInFull(down2.moved, -(int64_t)oneRot));

        tfhFreshOpen(m);
        setFence(m, -(int64_t)BIG_MOVE_COUNTS, NARROW_COUNTS, &reply, &fatal);
        Outcome up2 = trapLeg(m, oneRot, rotTicks);
        printf("   [-%ld, +%lld]: upward accepted=%d fatal=%u\n",
               (long)BIG_MOVE_COUNTS, (long long)NARROW_COUNTS,
               (int)up2.accepted, (unsigned)up2.fatal);
        TEST_RESULT("and mirroring it mirrors which direction is blocked",
                    isFenceCode(up2.fatal));
        tfhReset(m);
    }

    // =====================================================================
    // 8. THE FENCE IS A LATCH: LAST WRITE WINS, IT SURVIVES UNRELATED
    //    TRAFFIC, AND IT DIES AT RESET.
    //
    //    set_movement_limits() (motor_control.c:3494-3500) is a plain pair of
    //    assignments -- no accumulation, no ratcheting, no "tightest wins".
    //    The consequences are worth pinning down explicitly, because an
    //    operator's mental model of a safety fence usually assumes at least
    //    one of them is false:
    //      * a later, WIDER fence really does relax an earlier tighter one;
    //      * the fence is not sticky against ordinary supervisory polling and
    //        setting changes, but it is also not disturbed by them;
    //      * and it is not saved anywhere. A reset returns the limits to
    //        INT64_MIN/INT64_MAX (motor_control.c:3726-3727), which means a
    //        fence must be re-sent after every reset or the machine comes back
    //        completely unprotected. That is the single most important
    //        operational consequence in this module.
    // =====================================================================
    printf("\n--- 8. last write wins; survives traffic; gone after reset ---\n");
    {
        const int32_t oneRot = (int32_t)TFH_CPR;
        const uint32_t rotTicks = tfhTicksFor(TFH_CPR);

        // A narrow fence refuses the move...
        tfhFreshOpen(m);
        int reply = 0; uint8_t fatal = 0;
        setFence(m, -NARROW_COUNTS, NARROW_COUNTS, &reply, &fatal);
        Outcome blocked = trapLeg(m, oneRot, rotTicks);
        printf("   narrow fence: accepted=%d fatal=%u\n",
               (int)blocked.accepted, (unsigned)blocked.fatal);
        TEST_RESULT("a fence narrower than the move refuses it",
                    isFenceCode(blocked.fatal));

        // ...and a reset removes it entirely.
        tfhFreshOpen(m);
        Outcome afterReset = trapLeg(m, oneRot, rotTicks);
        printf("   after reset, no fence re-sent: completed=%d moved=%lld fatal=%u\n",
               (int)afterReset.completed, (long long)afterReset.moved,
               (unsigned)afterReset.fatal);
        TEST_RESULT("a reset removes the fence completely: the same move now runs",
                    afterReset.completed && deliveredInFull(afterReset.moved, oneRot));

        // The fence is unmoved by ordinary supervisory traffic.
        tfhFreshOpen(m);
        setFence(m, -NARROW_COUNTS, NARROW_COUNTS, &reply, &fatal);
        bool trafficClean = true;
        for (int i = 0; i < 6 && trafficClean; i++) {
            m->getStatus();               if (m->getError() != 0) trafficClean = false;
            m->getPositionRaw();          if (m->getError() != 0) trafficClean = false;
            m->getNQueuedItems();         if (m->getError() != 0) trafficClean = false;
            m->setMaximumVelocity(12.0f); if (m->getError() != 0) trafficClean = false;
            m->setMaximumVelocity(15.0f); if (m->getError() != 0) trafficClean = false;
            delay(10);
        }
        // A short legal leg well inside the fence, to prove the machine is
        // still functional rather than merely quiet.
        Outcome inside = velocityLeg(m, 1, LEG_TICKS);   // 31250 counts, inside +/-100000
        printf("   after 30 unrelated commands: clean=%d, inside leg completed=%d moved=%lld\n",
               (int)trafficClean, (int)inside.completed, (long long)inside.moved);
        TEST_RESULT("thirty unrelated commands under a fence are all answered cleanly",
                    trafficClean && inside.completed && inside.moved == TFH_TPS);
        Outcome stillBlocked = trapLeg(m, oneRot, rotTicks);
        printf("   fence after the traffic: accepted=%d fatal=%u\n",
               (int)stillBlocked.accepted, (unsigned)stillBlocked.fatal);
        TEST_RESULT("and the fence is still enforced afterwards, unchanged",
                    isFenceCode(stillBlocked.fatal));

        // Last write wins, in both directions.
        tfhFreshOpen(m);
        setFence(m, -NARROW_COUNTS, NARROW_COUNTS, &reply, &fatal);
        setFence(m, -(int64_t)BIG_MOVE_COUNTS, (int64_t)BIG_MOVE_COUNTS, &reply, &fatal);
        Outcome relaxed = trapLeg(m, oneRot, rotTicks);
        printf("   narrow then wide: completed=%d moved=%lld fatal=%u\n",
               (int)relaxed.completed, (long long)relaxed.moved, (unsigned)relaxed.fatal);
        TEST_RESULT("a later wider fence really does relax an earlier tighter one",
                    relaxed.completed && deliveredInFull(relaxed.moved, oneRot));

        tfhFreshOpen(m);
        setFence(m, -(int64_t)BIG_MOVE_COUNTS, (int64_t)BIG_MOVE_COUNTS, &reply, &fatal);
        setFence(m, -NARROW_COUNTS, NARROW_COUNTS, &reply, &fatal);
        Outcome tightened = trapLeg(m, oneRot, rotTicks);
        printf("   wide then narrow: accepted=%d fatal=%u\n",
               (int)tightened.accepted, (unsigned)tightened.fatal);
        TEST_RESULT("and a later narrower fence really does tighten an earlier wider one",
                    isFenceCode(tightened.fatal));
        // (this section's cleanup reset is the module-level one immediately below)
    }

    // Leave the machine as it was found: no fence, no fault, nothing queued.
    tfhReset(m);
    TEST_RESULT("the motor is left clean, with the fence back at its defaults",
                tfhFatal(m) == 0);
}
