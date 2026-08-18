#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_multimove_capacity.cpp
//
// HOW BIG A MULTIMOVE CAN BE, AND WHAT IT COSTS.
//
// Multimove (command 29) is the only way to hand the motor a whole plan in one
// packet, and its entire value proposition is DENSITY: every entry costs ONE
// motion-queue slot, where a trapezoid move costs three. That single fact is
// what decides whether a 30-step plan fits in the machine or faults halfway
// through with "queue is full" -- and it is nowhere asserted in this suite.
//
// GROUND TRUTH, all read from the firmware rather than the JSON descriptions:
//
//   firmware/Src/config.h:6
//       #define MAX_MULTIMOVES (sizeof(uint32_t) * 8)          // == 32
//   firmware/Src/main.c:656..662  (MULTIMOVE_COMMAND handler)
//       uint8_t n_moves_in_this_command = ((int8_t*)payload)[0];
//       if(n_moves_in_this_command > MAX_MULTIMOVES) {
//           fatal_error(ERROR_MULTIMOVE_MORE_THAN_32_MOVES);    // code 24
//       }
//     Note the count byte is READ as int8_t and then held in a uint8_t, so a
//     count with the high bit set (128..255) still compares as 128..255 and is
//     still refused. There is no signed-comparison hole. Section 3 proves it.
//   firmware/Src/main.c:681..694
//       for(i = 0; i < n_moves_in_this_command; i++)
//           add_to_queue(... MOVE_WITH_ACCELERATION or MOVE_WITH_VELOCITY);
//     ONE add_to_queue() per entry -- hence one slot per move. Contrast
//     queue_trapezoid_segments(), which calls add_to_queue() three times.
//   firmware/Src/motor_control.h:31
//       #define MOVEMENT_QUEUE_SIZE 32
//   firmware/Src/motor_control.c:1790 / :1869
//       if(n_items_in_queue < MOVEMENT_QUEUE_SIZE) { ...enqueue... }
//       else fatal_error(ERROR_QUEUE_IS_FULL);                  // code 17
//     So MAX_MULTIMOVES and MOVEMENT_QUEUE_SIZE are two DIFFERENT 32s that
//     merely happen to be equal: the first bounds the packet, the second bounds
//     the machine. A 32-entry multimove only fits when the queue is empty.
//   firmware/Src/motor_control.c:1782..1789
//       if(n_time_steps == 0) return;   // silently dropped, costs no slot
//
// WHAT THIS MODULE ASSERTS: the size axis of Multimove is exact and linear.
// N entries are accepted for every N up to 32, occupy exactly N slots, are
// refused at 33 and beyond, are refused when N will not fit on top of work
// already queued, and deliver exactly the sum of their segments.
//
// DISTINCT FROM ITS NEIGHBOURS:
//   * tm_multimove_bitmask walks the TYPE MASK (which int32 means what). It
//     never varies the length beyond eleven entries and never counts slots.
//   * tm_multimove_edges covers the REJECTION cases one at a time (a 33-move
//     packet, a 1-move packet, a count-0 packet). It never establishes the
//     accepted range, the per-entry slot cost, or the capacity interaction with
//     an already-occupied queue -- the three things a caller actually has to
//     plan around. The 32/33 boundary is re-measured here as an adjacent PAIR,
//     which is a different claim from "33 faults" on its own.
//   * tm_queue_accounting pins down the slot cost of a TRAPEZOID move (3, or 2
//     on the 0.15.12.0 one-tick-ramp fast path). Section 2 deliberately re-uses
//     its exact recipe as the baseline for the 3-vs-1 comparison.
//
// WHY THE MEASUREMENTS ARE RACE-FREE: queued segments start executing on the
// next control tick, so a naive depth read can catch the queue mid-drain.
// n_items_in_queue only decrements when an entry's LAST time step is consumed
// (motor_control.c:2141..2145), so the safe window for any capacity measurement
// is the duration of the entry at the HEAD of the queue -- and it has to cover
// the WHOLE measurement, not just the depth read. That is why there are two
// segment lengths:
//
//   FILL_TICKS (31250 ticks = 1 s) is used by every subsection that measures
//     capacity across several round trips -- sections 1, 2 and 4. Section 4 is
//     the demanding one: "16 accepted on top of 16" spans two 133-byte multimove
//     packets and two depth reads, roughly 25 ms of wire time, and "16 then 17
//     faults" is only a fault at all if NOTHING has retired in between. A single
//     retirement would make 15 + 17 == 32 fit and the test would fail on a
//     perfectly healthy motor. One second per entry leaves ~40x margin. None of
//     these subsections ever drains, so the long entries cost no wall time: each
//     ends in an emergency stop or in the deliberate fault.
//   SEG_TICKS (5000 ticks = 160 ms) is used by the subsections that DO drain to
//     the end -- sections 5 and 6 -- where the whole measurement is one send
//     plus one read (~14 ms, still >10x margin) and a longer entry would only
//     make the module slower.
//
// No depth is ever inferred from a computed wait, and every completion is
// confirmed with tfhDrain().
//
// Depth reads that EXPECT zero are also checked for a comms error, because
// Servomotor::getNQueuedItems() returns 0 when the round trip fails
// (Servomotor.cpp, getNQueuedItems: `getNQueuedItemsResponse defaultResponse =
// {0}` on the error path) -- an unguarded `depth == 0` would pass for the wrong
// reason on a dead link.
//
// WHY THE DISTANCES ARE PREDICTED EXACTLY RATHER THAN APPROXIMATELY: a velocity
// entry sets current_velocity_i64 = value << VELOCITY_SHIFT_LEFT (12)
// (motor_control.c:1842, :2140) and each tick adds that to a 96-bit accumulator
// whose reported position is its top 64 bits (motor_control.c:2191). So one tick
// advances the commanded position by value / 2^20 counts. Choosing every value
// as an exact multiple of 2^20 makes the displacement a whole number of counts
// with no fractional residue at all, so the expected result is an integer and
// the tolerance can be +/- 2 counts instead of a percentage.
//
// SAFETY. The MOSFETs are never enabled and go_to_closed_loop is never called;
// every measurement reads the COMMANDED position and the queue depth, both of
// which the planner advances whether or not the output stage is energised, so
// nothing turns on any motor at any point. EVERY subsection starts with
// tfhFreshOpen(), which raises the two-rotation deviation window: with the
// output stage off the shaft never follows, so the deviation grows with every
// count commanded and any plan past ~2 rotations would trip fatal error 45 and
// turn this into a deviation test. The largest plan that actually runs to
// completion is section 5's 32-entry sweep at ~9.3 rotations; sections 1 and 4
// enqueue plans of up to ~30 rotations but stop them within milliseconds, so
// the window has to cover both and tfhFreshOpen's 100000 rotations does. No
// broadcasts, no alias writes and no test modes: safe on the shared 35-motor
// rack. Every deliberately induced fault (24, 17) is cleared with tfhReset()
// before the next subsection, and the module ends clean.
// ---------------------------------------------------------------------------

static const uint8_t  ERR_QUEUE_IS_FULL                = 17;  // error_text.h:24
static const uint8_t  ERR_MULTIMOVE_MORE_THAN_32_MOVES = 24;  // error_text.h:31
static const uint8_t  CMD_MULTIMOVE                    = 29;  // Commands.h
static const int      QUEUE_SIZE                       = 32;  // motor_control.h:31
static const int      MAX_MULTIMOVES                   = 32;  // config.h:6

// One entry lasts 5000 ticks = 160 ms. Long enough that nothing retires during
// a single send-then-read, short enough that a full 32-entry plan drains in
// ~5 seconds. Used only where the plan is actually drained (sections 5 and 6).
static const uint32_t SEG_TICKS = 5000;

// One entry lasts 31250 ticks = 1 s. Used by every subsection that measures
// queue capacity across MORE THAN ONE round trip (sections 1, 2 and 4), so that
// no entry can retire mid-measurement and turn a correct motor into a failure.
// Free: none of those subsections drains -- each ends in an emergency stop or in
// a deliberate fault, both of which discard the remaining entries immediately.
static const uint32_t FILL_TICKS = 31250;

// ---------------------------------------------------------------------------
// Internal velocity value for a whole number of encoder counts per control tick.
//
// The firmware advances the commanded position by value / 2^20 counts per tick
// (see the header comment), so value = countsPerTick << 20 gives exact integer
// motion. countsPerTick must stay under ~1572 to keep the value inside int32
// and under the ~15 rot/s ceiling tfhFreshOpen installs; everything here uses
// 50..350, i.e. 0.48..3.34 rot/s.
// ---------------------------------------------------------------------------
static int32_t velValue(int32_t countsPerTick) {
    return (int32_t)((int64_t)countsPerTick << 20);
}

// ---------------------------------------------------------------------------
// Type mask with the low n bits set: every entry in this module is a VELOCITY
// entry (mask bit 1), because a velocity entry's displacement is exactly
// value * timeSteps whereas an acceleration entry's depends on the velocity
// left over from the previous entry.
//
// The shift is written out rather than (1u << n) - 1 because n reaches 32 and
// a 32-bit shift by 32 is undefined behaviour in C++ -- the exact kind of
// silent miscompile that would make the largest, most interesting case pass
// for the wrong reason.
// ---------------------------------------------------------------------------
static uint32_t allVelocityMask(int n) {
    return (n >= 32) ? 0xFFFFFFFFu : ((1u << n) - 1u);
}

// ---------------------------------------------------------------------------
// Build an n-entry list: (n-1) moving segments at countsPerTick, then a final
// ZERO-velocity segment.
//
// The trailing stop is mandatory, not decoration. A velocity segment HOLDS its
// velocity after its time expires, so when the queue empties at nonzero speed
// the firmware raises fatal ERROR_RUN_OUT_OF_QUEUE_ITEMS (18) from inside the
// control loop (motor_control.c:2159..2164). Without the stop, every list here
// would end in a fault and the slot counts would be measured on a dying machine.
//
// Returns the exact predicted commanded displacement, in encoder counts.
// ---------------------------------------------------------------------------
static int64_t buildUniform(multimoveList_t* mv, int n, int32_t countsPerTick,
                            uint32_t dur) {
    int64_t predicted = 0;
    for (int i = 0; i < n - 1; i++) {
        mv[i].value = velValue(countsPerTick);
        mv[i].timeSteps = dur;
        predicted += (int64_t)countsPerTick * (int64_t)dur;
    }
    // The stop. For n == 1 this is the ONLY entry, which is the degenerate but
    // perfectly legal one-entry plan "hold still for dur ticks".
    mv[n - 1].value = 0;
    mv[n - 1].timeSteps = dur;
    return predicted;
}

// Send an n-entry all-velocity multimove and read the queue depth straight
// afterwards. `accepted` reports the command reply; the depth is the measurement.
static int sendAndMeasureDepth(Servomotor* m, int n, multimoveList_t* mv,
                               bool* accepted) {
    m->multimoveRaw((uint8_t)n, allVelocityMask(n), mv);
    *accepted = (m->getError() == 0);
    if (!*accepted) return -1;
    uint8_t depth = m->getNQueuedItems();
    if (m->getError() != 0) { *accepted = false; return -1; }
    return (int)depth;
}

// ---------------------------------------------------------------------------
// Inject a raw MULTIMOVE frame carrying nothing but an oversized count byte.
//
// This CANNOT go through the library: Servomotor::multimoveRaw() does
//     memcpy(payload.moveList /* multimoveList_t[32] */, moveList, count * 8)
// so a count above 32 overruns a host-side buffer and corrupts the ESP32
// controller before a single byte reaches the motor. The firmware's guard runs
// BEFORE copy_input_parameters_and_check_size() (main.c:660 vs :676), so a
// one-byte payload is enough to exercise it and is the smallest, lowest-risk
// frame that can.
//
// The framing mirrors Communication::sendCommandCore exactly, WITH CRC32 (both
// the device and the library default to CRC enabled here, and this module never
// toggles that). Extended addressing (254 + the 8-byte unique ID) keeps the
// frame aimed at exactly one motor: on the shared rack it can never be mistaken
// for an alias-88 frame and it is not a broadcast.
//
//   frame = sizeByte | 254 | uniqueId[8 LE] | 29 | count | crc32[4 LE]
//
// Adapted from tm_multimove_edges::injectRawMultimoveCount, which established
// this recipe.
// ---------------------------------------------------------------------------
static void injectRawMultimoveCount(Servomotor* m, uint8_t moveCount) {
    uint64_t uid = m->usingThisUniqueId();
    const uint16_t totalPacketSize = 1 + 1 + 8 + 1 + 1 + 4;   // 16 bytes

    uint8_t frame[16];
    int n = 0;
    frame[n++] = encodeFirstByte((uint8_t)totalPacketSize);   // (size << 1) | 1
    frame[n++] = EXTENDED_ADDRESSING;                         // 254
    for (int i = 0; i < 8; i++) {
        frame[n++] = (uint8_t)((uid >> (8 * i)) & 0xFF);      // little-endian
    }
    frame[n++] = CMD_MULTIMOVE;
    frame[n++] = moveCount;
    uint32_t crc = calculate_crc32(frame, n);                 // over sizeByte..payload
    for (int i = 0; i < 4; i++) {
        frame[n++] = (uint8_t)((crc >> (8 * i)) & 0xFF);
    }

    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
    Serial1.write(frame, (size_t)n);
    Serial1.flush();
}

void tm_multimove_capacity(void) {
    Serial.println("tm_multimove_capacity: BEGIN\n");
    Servomotor* m = tfGetMotor();
    multimoveList_t mv[32];

    // =====================================================================
    // 1. ONE ENTRY, ONE SLOT -- FOR EVERY LENGTH UP TO THE MAXIMUM.
    //
    //    This is the module's central claim and the one a caller plans
    //    around. The handler calls add_to_queue() once per entry
    //    (main.c:681..694), so an N-entry list must land as exactly N items
    //    and never as 2N or 3N. The sweep covers powers of two plus the two
    //    lengths that matter most -- 31 (one below the maximum) and 32 (the
    //    maximum itself, which is also the only routine command large enough
    //    to need extended-size packet framing: 1 + 4 + 32*8 = 261 payload
    //    bytes, well past the 127-byte single-byte size field).
    //
    //    Each list is emergency-stopped rather than drained: the slot count is
    //    the measurement, and an emergency stop clears the queue and zeroes
    //    the velocity atomically with interrupts disabled
    //    (motor_control.c:420..429), so it cannot leave a partial state behind.
    // =====================================================================
    printf("\n--- 1. an N-entry multimove occupies exactly N queue slots ---\n");
    {
        const int sizes[] = { 1, 2, 4, 8, 16, 31, 32 };
        for (unsigned s = 0; s < sizeof(sizes) / sizeof(sizes[0]); s++) {
            const int n = sizes[s];
            tfhFreshOpen(m);
            // FILL_TICKS, not SEG_TICKS: the 32-entry packet alone is ~12 ms on
            // the wire and the depth read follows it, so the head entry has to
            // outlive both. The list is emergency-stopped, never drained.
            buildUniform(mv, n, 100, FILL_TICKS);

            bool accepted = false;
            int depth = sendAndMeasureDepth(m, n, mv, &accepted);
            uint8_t fat = tfhFatal(m);
            printf("   n=%d -> accepted=%d depth=%d fatal=%u\n",
                   n, (int)accepted, depth, (unsigned)fat);

            TEST_RESULT(std::string("a ") + std::to_string(n) +
                            "-entry multimove is accepted without faulting",
                        accepted && fat == 0);
            TEST_RESULT(std::string("a ") + std::to_string(n) +
                            "-entry multimove occupies exactly " +
                            std::to_string(n) + " queue slots",
                        depth == n);

            m->emergencyStop();
            delay(300);
        }

        // The degenerate low end. A count of 0 is not > MAX_MULTIMOVES, so it
        // passes the guard, the enqueue loop runs zero times and the handler
        // still replies success (main.c:681, :695). It must cost nothing.
        tfhFreshOpen(m);
        uint8_t before = m->getNQueuedItems();
        bool beforeOk = (m->getError() == 0);
        mv[0].value = 0; mv[0].timeSteps = FILL_TICKS;   // never read at count 0
        m->multimoveRaw(0, 0, mv);
        bool zeroAccepted = (m->getError() == 0);
        uint8_t after = m->getNQueuedItems();
        // A failed depth read also returns 0, which is the value this case
        // expects -- without the error check the assertion would pass on a dead
        // link. Same guard wherever a depth of zero is the expected answer.
        bool afterOk = (m->getError() == 0);
        printf("   n=0 -> accepted=%d depth %u -> %u (reads ok: %d/%d)\n",
               (int)zeroAccepted, (unsigned)before, (unsigned)after,
               (int)beforeOk, (int)afterOk);
        TEST_RESULT("a zero-entry multimove is accepted and queues nothing",
                    zeroAccepted && beforeOk && afterOk &&
                    before == 0 && after == 0 && tfhFatal(m) == 0);
    }

    // =====================================================================
    // 2. A MULTIMOVE ENTRY IS THREE TIMES CHEAPER THAN A TRAPEZOID MOVE.
    //
    //    The comparison, not the individual numbers, is the point: it is the
    //    reason Multimove exists at all. Both halves run under the SAME
    //    limits (maxVel 1 rot/s, maxAccel 1 rot/s^2), which force the
    //    three-segment planner path -- delta_t1 = maxVel/maxAccel = 31250
    //    ticks, far above the one-tick fast-path threshold -- so the trapezoid
    //    baseline is 3 on both current firmware and the pre-0.15.12.0
    //    firmware the ESP32-S3 bench motor still runs. The segments last about
    //    a second each, so the depth read cannot race the drain.
    //
    //    tm_queue_accounting owns the trapezoid slot cost; re-measuring it here
    //    is deliberate, because a comparison against a remembered number is not
    //    a comparison.
    // =====================================================================
    printf("\n--- 2. one multimove entry costs one slot, one trapezoid move costs three ---\n");
    {
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), 4 * TFH_TPS);
        bool trapOk = (m->getError() == 0);
        int trapDepth = trapOk ? (int)m->getNQueuedItems() : -1;
        m->emergencyStop();
        delay(300);

        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        // 50 counts/tick = 0.48 rot/s, comfortably under the 1 rot/s ceiling
        // so the ceiling cannot become the thing being measured. FILL_TICKS
        // matches the ~1 s first segment of the trapezoid above, so both halves
        // of the comparison are read under the same race margin.
        buildUniform(mv, 3, 50, FILL_TICKS);
        bool mmOk = false;
        int mmDepth = sendAndMeasureDepth(m, 3, mv, &mmOk);
        m->emergencyStop();
        delay(300);

        printf("   one trapezoid move -> depth %d; a three-entry multimove -> depth %d\n",
               trapDepth, mmDepth);
        TEST_RESULT("an ordinary trapezoid move still costs three queue slots",
                    trapOk && trapDepth == 3);
        TEST_RESULT("a three-move multimove costs three queue slots, one per move",
                    mmOk && mmDepth == 3);
        // Three moves for the price of one: same slots occupied, three times
        // the moves expressed. That is the whole reason to reach for Multimove.
        TEST_RESULT("three multimove moves fit in the space of one trapezoid move",
                    trapOk && mmOk && trapDepth == mmDepth && mmDepth == 3);
    }

    // =====================================================================
    // 3. THIRTY-TWO IS THE CEILING AND EVERYTHING ABOVE IT IS REFUSED.
    //
    //    Section 1 already showed 32 is accepted; this is the other side of
    //    the same boundary. The three counts are chosen to probe the guard's
    //    arithmetic, not just repeat it:
    //      33  -- one past the limit, the ordinary mistake
    //      128 -- the first value whose high bit is set, i.e. the first value
    //             that would read as NEGATIVE if the int8_t in main.c:656 were
    //             compared as a signed number. A signed comparison would let
    //             it through and then copy 128 * 8 = 1024 bytes into a
    //             256-byte stack buffer.
    //      255 -- the largest byte, the worst case of the same hole.
    //    All three must land on fatal 24, and the motor must survive each one.
    // =====================================================================
    printf("\n--- 3. a count above 32 is refused, high bit or not ---\n");
    {
        const uint8_t counts[] = { 33, 128, 255 };
        bool allAlive = true, allCleared = true;
        for (unsigned i = 0; i < sizeof(counts) / sizeof(counts[0]); i++) {
            tfhReset(m);
            injectRawMultimoveCount(m, counts[i]);
            delay(300);   // let the fault latch; fatal_error() sends no reply
            for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();

            getStatusResponse st = m->getStatus();
            bool alive = (m->getError() == 0);
            uint8_t code = alive ? st.fatalErrorCode : 0xFF;
            if (!alive) allAlive = false;
            printf("   count byte %u -> alive=%d fatal=%u\n",
                   (unsigned)counts[i], (int)alive, (unsigned)code);
            TEST_RESULT(std::string("a multimove claiming ") +
                            std::to_string((int)counts[i]) +
                            " moves is refused with ERROR_MULTIMOVE_MORE_THAN_32_MOVES (24)",
                        code == ERR_MULTIMOVE_MORE_THAN_32_MOVES);

            tfhReset(m);
            if (tfhFatal(m) != 0) allCleared = false;
        }
        TEST_RESULT("the motor still answers get_status after every oversized count",
                    allAlive);
        TEST_RESULT("a reset clears every oversized-multimove fault", allCleared);
    }

    // =====================================================================
    // 4. THE PACKET LIMIT AND THE QUEUE LIMIT ARE DIFFERENT LIMITS.
    //
    //    MAX_MULTIMOVES bounds the packet; MOVEMENT_QUEUE_SIZE bounds the
    //    machine. They are both 32, which makes it easy to assume a 32-entry
    //    multimove always fits -- it only fits when the queue is EMPTY. On top
    //    of anything already queued the enqueue loop walks into
    //    motor_control.c:1869 and raises ERROR_QUEUE_IS_FULL (17).
    //
    //    Worth knowing, and asserted only as far as it is observable: the loop
    //    faults on the entry that does not fit, AFTER the earlier entries have
    //    already been added, so a multimove is NOT atomic. Once fatal 17 is
    //    latched only get_status answers (fatal_error() never returns), so the
    //    partial state cannot be read back; what is asserted here is the
    //    refusal itself and full recovery.
    // =====================================================================
    printf("\n--- 4. a multimove that will not fit on top of queued work is refused ---\n");
    {
        // Exact fit: 16 + 16 = 32. The queue must end up exactly full, with no
        // fault -- the boundary case that must NOT be refused.
        //
        // FILL_TICKS throughout this subsection. Every claim here is about what
        // is in the queue at a given moment, and each spans two multimove
        // packets plus one or two depth reads (~25 ms). At SEG_TICKS the head
        // entry lives only 160 ms; at FILL_TICKS it lives a full second, so a
        // retirement cannot rewrite the arithmetic under the test. Nothing here
        // is ever drained, so the longer entries cost nothing.
        tfhFreshOpen(m);
        buildUniform(mv, 16, 100, FILL_TICKS);
        m->multimoveRaw(16, allVelocityMask(16), mv);
        bool firstOk = (m->getError() == 0);
        int depth1 = firstOk ? (int)m->getNQueuedItems() : -1;
        // Second half: 15 moving entries plus the stop, so the plan as a whole
        // still ends at zero velocity.
        buildUniform(mv, 16, 100, FILL_TICKS);
        m->multimoveRaw(16, allVelocityMask(16), mv);
        bool secondOk = (m->getError() == 0);
        int depth2 = secondOk ? (int)m->getNQueuedItems() : -1;
        uint8_t fatFit = tfhFatal(m);
        printf("   16 entries -> depth %d, then 16 more -> depth %d, fatal=%u\n",
               depth1, depth2, (unsigned)fatFit);
        TEST_RESULT("sixteen entries on top of sixteen are accepted",
                    firstOk && secondOk && fatFit == 0);
        TEST_RESULT("and they fill the 32-slot queue exactly",
                    depth1 == 16 && depth2 == QUEUE_SIZE);
        m->emergencyStop();
        delay(300);

        // One entry too many: 16 + 17. The 33rd add_to_queue() must fault.
        // This is the assertion most sensitive to a retirement: if even ONE of
        // the first 16 entries had completed before the second packet landed,
        // 15 + 17 == 32 would fit and there would be no fault at all. Hence
        // FILL_TICKS.
        tfhFreshOpen(m);
        buildUniform(mv, 16, 100, FILL_TICKS);
        m->multimoveRaw(16, allVelocityMask(16), mv);
        bool preOk = (m->getError() == 0);
        buildUniform(mv, 17, 100, FILL_TICKS);
        m->multimoveRaw(17, allVelocityMask(17), mv);
        uint8_t fatOver = tfhFatal(m);
        printf("   16 entries then 17 more (33 total) -> fatal=%u\n", (unsigned)fatOver);
        TEST_RESULT("one entry more than the queue can hold is refused with "
                    "ERROR_QUEUE_IS_FULL (17)",
                    preOk && fatOver == ERR_QUEUE_IS_FULL);
        tfhReset(m);
        TEST_RESULT("the queue-full refusal is cleared by a reset", tfhFatal(m) == 0);

        // A full-size 32-entry multimove needs an EMPTY queue. Even one slot
        // already taken is enough to refuse it, and twenty certainly are.
        const int prefill[] = { 1, 20 };
        for (unsigned p = 0; p < sizeof(prefill) / sizeof(prefill[0]); p++) {
            tfhFreshOpen(m);
            // FILL_TICKS again: with prefill == 1 the single occupied slot is
            // the ONLY thing making the 32-entry packet impossible, so it must
            // still be occupied when that packet arrives.
            buildUniform(mv, prefill[p], 100, FILL_TICKS);
            m->multimoveRaw((uint8_t)prefill[p], allVelocityMask(prefill[p]), mv);
            bool pre = (m->getError() == 0);
            // A packet of exactly MAX_MULTIMOVES moves -- legal as a PACKET,
            // yet impossible as a QUEUE operation with slots already taken.
            buildUniform(mv, MAX_MULTIMOVES, 100, FILL_TICKS);
            m->multimoveRaw((uint8_t)MAX_MULTIMOVES, allVelocityMask(MAX_MULTIMOVES), mv);
            uint8_t fat = tfhFatal(m);
            printf("   %d slots already taken, then a full 32-entry multimove -> fatal=%u\n",
                   prefill[p], (unsigned)fat);
            TEST_RESULT(std::string("a full 32-move multimove is refused when ") +
                            std::to_string(prefill[p]) +
                            " slot(s) are already taken",
                        pre && fat == ERR_QUEUE_IS_FULL);
            tfhReset(m);
        }
    }

    // =====================================================================
    // 5. A BIG MULTIMOVE DELIVERS THE SUM OF ITS SEGMENTS, TO THE COUNT.
    //
    //    Capacity is worthless if the plan is not executed faithfully at that
    //    size. Every value is an exact multiple of 2^20, so the expected
    //    displacement is a whole number of counts and can be checked to +/- 2
    //    rather than to a percentage.
    //
    //    Draining also exercises the firmware's own arithmetic cross-check:
    //    when the queue empties it compares the position it PREDICTED at
    //    enqueue time against the position it actually reached and raises
    //    ERROR_POSITION_DISCREPANCY (42) on any mismatch
    //    (motor_control.c:2152..2153). A clean drain of a 32-entry plan is
    //    therefore the planner agreeing with itself over 32 chained segments.
    // =====================================================================
    printf("\n--- 5. a 32-entry plan delivers the exact sum of its segments ---\n");
    {
        // Deliberately non-uniform, so a bug that averaged, reordered or
        // dropped a segment could not cancel out.
        int32_t cpt[31];
        uint32_t dur[31];
        int64_t predicted = 0;
        for (int i = 0; i < 31; i++) {
            cpt[i] = 50 + 10 * i;            // 50..350 counts/tick = 0.48..3.34 rot/s
            dur[i] = 3000 + 100 * (uint32_t)i;
            predicted += (int64_t)cpt[i] * (int64_t)dur[i];
        }

        // One shot: all 31 segments plus the stop, in a single 32-entry packet.
        tfhFreshOpen(m);
        for (int i = 0; i < 31; i++) { mv[i].value = velValue(cpt[i]); mv[i].timeSteps = dur[i]; }
        mv[31].value = 0; mv[31].timeSteps = 100;
        int64_t b1 = m->getPositionRaw();
        m->multimoveRaw(32, allVelocityMask(32), mv);
        bool oneShotOk = (m->getError() == 0) && tfhDrain(m, 20000);
        uint8_t fatOne = tfhFatal(m);
        int64_t movedOne = (oneShotOk && fatOne == 0) ? (m->getPositionRaw() - b1) : 0;
        printf("   one 32-entry packet: moved %lld, predicted %lld, fatal=%u\n",
               (long long)movedOne, (long long)predicted, (unsigned)fatOne);
        TEST_RESULT("a 32-entry multimove delivers the exact sum of its segments",
                    oneShotOk && llabs((long long)(movedOne - predicted)) <= 2);
        TEST_RESULT("and it drains with no position-discrepancy fault",
                    oneShotOk && fatOne == 0);

        // The same 32 entries as four packets of eight. Same slots, same
        // arithmetic, same destination -- the packet boundary must be invisible.
        tfhFreshOpen(m);
        int64_t b2 = m->getPositionRaw();
        bool splitOk = true;
        for (int chunk = 0; chunk < 4 && splitOk; chunk++) {
            multimoveList_t part[8];
            for (int j = 0; j < 8; j++) {
                int idx = chunk * 8 + j;
                if (idx < 31) { part[j].value = velValue(cpt[idx]); part[j].timeSteps = dur[idx]; }
                else          { part[j].value = 0;                  part[j].timeSteps = 100; }
            }
            m->multimoveRaw(8, allVelocityMask(8), part);
            if (m->getError() != 0) splitOk = false;
        }
        splitOk = splitOk && tfhDrain(m, 20000) && (tfhFatal(m) == 0);
        int64_t movedSplit = splitOk ? (m->getPositionRaw() - b2) : 0;
        printf("   four 8-entry packets: moved %lld\n", (long long)movedSplit);
        TEST_RESULT("the same plan split into four eight-entry packets travels "
                    "the identical distance",
                    splitOk && llabs((long long)(movedSplit - predicted)) <= 2);

        // Linearity in the number of segments: N identical segments must travel
        // exactly N times as far as one. A capacity bug that quietly dropped
        // the last entry of a long list would show up here and nowhere else.
        const int counts[] = { 2, 16 };
        for (unsigned c = 0; c < sizeof(counts) / sizeof(counts[0]); c++) {
            const int nMoving = counts[c];
            tfhFreshOpen(m);
            int64_t want = buildUniform(mv, nMoving + 1, 100, SEG_TICKS);
            int64_t b = m->getPositionRaw();
            m->multimoveRaw((uint8_t)(nMoving + 1), allVelocityMask(nMoving + 1), mv);
            bool ok = (m->getError() == 0) && tfhDrain(m, 20000) && (tfhFatal(m) == 0);
            int64_t moved = ok ? (m->getPositionRaw() - b) : 0;
            printf("   %d moving segments: moved %lld, predicted %lld\n",
                   nMoving, (long long)moved, (long long)want);
            TEST_RESULT(std::string("a plan of ") + std::to_string(nMoving) +
                            " identical segments travels exactly " +
                            std::to_string(nMoving) + " times one segment",
                        ok && llabs((long long)(moved - want)) <= 2);
        }
    }

    // =====================================================================
    // 6. ZERO-DURATION ENTRIES ARE FREE, AND ARE NEVER EVEN VALIDATED.
    //
    //    add_to_queue() returns immediately on n_time_steps == 0
    //    (motor_control.c:1782..1789) -- before the queue-full check, before
    //    the acceleration and velocity limit checks, before everything. So a
    //    zero-duration entry costs no slot, contributes no motion, and cannot
    //    raise an error no matter what value it carries.
    //
    //    That makes the list length an UPPER bound on the slot cost rather
    //    than the cost itself, which is exactly the kind of asymmetry a
    //    capacity calculation gets wrong. The proof is sharp: the dropped
    //    entries here carry INT32_MAX, which is an ILLEGAL velocity under the
    //    ceiling this module runs at. If even one of them reached the validator
    //    it would raise fatal ERROR_VEL_TOO_HIGH (16) instead of being ignored.
    //
    //    The margin, stated exactly rather than hand-waved, because it is
    //    tighter than it looks. add_to_queue compares (value << 12) against
    //    max_velocity (motor_control.c:1849). INT32_MAX << 12 = 8.796e12.
    //    tfhFreshOpen sets 15 rot/s, i.e. 6.755e12 -- so INT32_MAX is 1.30x over
    //    the limit and is refused. Even at the absolute ceiling the firmware
    //    permits, EXPERIMENTAL_MAX_VELOCITY = 2 x MAX_VELOCITY at MAX_RPM 560
    //    = 8.406e12 (motor_control.h:44,:51,:61), INT32_MAX is still over -- but
    //    only by 4.6%. Do not raise the ceiling in tfhOpenCeilings expecting
    //    this to keep working by a wide margin; there isn't one.
    // =====================================================================
    printf("\n--- 6. zero-duration entries cost nothing and bypass validation ---\n");
    {
        // 16 real entries, 15 illegal-but-zero-duration entries, 1 stop.
        tfhFreshOpen(m);
        int64_t want = 0;
        for (int i = 0; i < 16; i++) {
            mv[i].value = velValue(100); mv[i].timeSteps = SEG_TICKS;
            want += (int64_t)100 * (int64_t)SEG_TICKS;
        }
        for (int i = 16; i < 31; i++) { mv[i].value = INT32_MAX; mv[i].timeSteps = 0; }
        mv[31].value = 0; mv[31].timeSteps = 100;

        int64_t b = m->getPositionRaw();
        m->multimoveRaw(32, allVelocityMask(32), mv);
        bool accepted = (m->getError() == 0);
        int depth = accepted ? (int)m->getNQueuedItems() : -1;
        bool drained = accepted && tfhDrain(m, 20000);
        uint8_t fat = tfhFatal(m);
        int64_t moved = (drained && fat == 0) ? (m->getPositionRaw() - b) : 0;
        printf("   32 entries, 15 of them zero-duration at INT32_MAX -> depth %d, "
               "moved %lld (predicted %lld), fatal=%u\n",
               depth, (long long)moved, (long long)want, (unsigned)fat);
        TEST_RESULT("zero-duration entries are dropped before validation, so an "
                    "INT32_MAX velocity in one raises no error",
                    accepted && fat == 0);
        TEST_RESULT("a 32-entry list with fifteen zero-duration entries occupies "
                    "only seventeen slots",
                    depth == 17);
        TEST_RESULT("and the dropped entries contribute no motion at all",
                    drained && llabs((long long)(moved - want)) <= 2);

        // The whole list dropped: accepted, and nothing whatsoever queued.
        tfhFreshOpen(m);
        for (int i = 0; i < 32; i++) { mv[i].value = velValue(100); mv[i].timeSteps = 0; }
        int64_t bz = m->getPositionRaw();
        m->multimoveRaw(32, allVelocityMask(32), mv);
        bool zAccepted = (m->getError() == 0);
        uint8_t zDepth = m->getNQueuedItems();
        // Guarded for the same reason as the count-0 case above: a depth of 0
        // is both the expected answer and the value a failed read returns.
        bool zDepthOk = (m->getError() == 0);
        delay(300);   // not a wait for a move -- nothing was queued; this is a
                      // settling window in which the position must NOT change
        int64_t zMoved = m->getPositionRaw() - bz;
        bool zPosOk = (m->getError() == 0);
        printf("   32 zero-duration entries -> accepted=%d depth %u moved %lld "
               "(reads ok: %d/%d)\n",
               (int)zAccepted, (unsigned)zDepth, (long long)zMoved,
               (int)zDepthOk, (int)zPosOk);
        TEST_RESULT("a 32-entry list of only zero-duration entries is accepted "
                    "and queues nothing",
                    zAccepted && zDepthOk && zPosOk &&
                    zDepth == 0 && zMoved == 0 && tfhFatal(m) == 0);
    }

    // =====================================================================
    // 7. STILL HEALTHY, AND LEFT CLEAN. The rack is shared; the next module
    //    and the next user get a machine with no latched fault, a default
    //    deviation window and an empty queue.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   post-sweep quarter turn: moved %lld of %lld\n",
               (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move still works after the capacity sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
