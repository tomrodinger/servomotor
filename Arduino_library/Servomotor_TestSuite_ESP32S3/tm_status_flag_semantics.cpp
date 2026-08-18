#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_status_flag_semantics.cpp
//
// WHAT THE STATUS FLAGS WORD ACTUALLY MEANS.
//
// Get status (command 16, motor_commands.json "Get status") returns three
// bytes: a 16-bit FLAGS word and an 8-bit fatal error code
// (getStatusResponse in Arduino_library/Servomotor.h). Almost every test in
// this suite reads the error byte; hardly anything interrogates the flags word,
// and nothing at all asks what it does during ordinary operation.
//
// That matters because the flags word is the only supervisory channel a caller
// has for "what is this machine doing right now", and a caller who guesses its
// meaning wrong writes a supervisor that silently never fires.
//
// THE BIT DEFINITIONS, from the firmware (common_source_files/device_status.h:16-22):
//
//     bit 0  STATUS_IN_THE_BOOTLOADER_FLAG_BIT   in the bootloader
//     bit 1  STATUS_MOSFETS_ENABLED_FLAG_BIT     output stage energised
//     bit 2  STATUS_CLOSED_LOOP_FLAG_BIT         closed-loop position control
//     bit 3  STATUS_CALIBRATING_FLAG_BIT         calibration running
//     bit 4  STATUS_HOMING_FLAG_BIT              homing running
//     bit 5  STATUS_GO_TO_CLOSED_LOOP_FLAG_BIT   go-to-closed-loop running
//     bit 6  STATUS_MOTOR_BUSY_FLAG_BIT          long exclusive task in progress
//     bits 7..15                                 undefined, always 0
//
// The word is composed in get_motor_status_flags_without_disable_enable_irq()
// (firmware/Src/motor_control.c:3442-3466) and shipped by the GET_STATUS_COMMAND
// handler (firmware/Src/main.c:488-505), which recomputes it on every request.
//
// THE PROPERTY THIS MODULE ASSERTS, and it is a surprising one:
//
//     THE STATUS FLAGS WORD SAYS NOTHING ABOUT THE MOTION QUEUE.
//
// It is tempting to read bit 6 as "busy = moving". It is not. In the firmware,
// motor_busy is set in only three places reachable on this product --
// start_calibration (motor_control.c:648), start_go_to_closed_loop_mode (:1005)
// and start_homing (:1446). (There is a fourth assignment, in vibrate()
// at :1237, but that whole function is inside #ifdef PRODUCT_NAME_M1 and is
// compiled out on M17/M23; this module never calls vibrate in any case.)
// An ordinary trapezoid move, a queue filled to
// its 32-slot capacity, a move refused for overflow: none of them touch a single
// bit. So the ONLY way to know the machine is moving is getNQueuedItems
// (command 11), and a supervisor that polls the flags word instead will watch a
// constant 0x0000 forever while the shaft turns. This module measures that,
// rather than leaving it as a plausible reading of the header comment.
//
// The second property is the fault behaviour, which is the opposite of what a
// "status" word usually does: fatal_error() FORCES the flags to zero
// (common_source_files/error_handling.c:227,
//   set_device_status_flags(flags & (1 << STATUS_IN_THE_BOOTLOADER_FLAG_BIT)))
// and the fatal-state get_status handler (error_handling.c:92-103) replies with
// that frozen snapshot instead of recomputing it. A faulted motor therefore
// reports flags 0x0000 -- identical to a healthy idle one -- and the error byte
// is the only thing that distinguishes them. Anyone who tests "flags == 0" to
// decide a machine is healthy has written a bug, and this module pins that down
// across five different fatal errors.
//
// HOW THIS IS DISTINCT FROM THE TWO EXISTING STATUS MODULES:
//   * tm_status_matrix walks the MOSFET/closed-loop state machine -- it calls
//     enableMosfets and goToClosedLoop and asserts bits 1, 2 and 5 turning on
//     and off. It never queues a move, never fills the queue, and never faults.
//   * tm_system_status mirrors the Python per-command tests: reset, the MOSFET
//     bit, and one safety-zone fatal error. Four assertions on the flags word.
//   * Neither one ever reads the flags word DURING MOTION, with a FULL QUEUE, or
//     in more than one fault; neither checks the flags against getNQueuedItems;
//     and neither asserts that the faulted word is frozen rather than live.
//   This module covers exactly those gaps and deliberately touches no MOSFET
//   state at all, so the two do not overlap even at the edges.
//
// SAFETY. The MOSFETs stay OFF for the entire module: enableMosfets,
// goToClosedLoop, startCalibration and vibrate are never called. With the output
// stage de-energised the planner still advances the COMMANDED position, which is
// what every measurement here reads (getPositionRaw), so nothing turns on any
// motor at any point. The homing attempt in section 6 is refused by
// start_homing's very first check (motor_control.c:1436-1438, ERROR_NOT_IN_CLOSED_LOOP)
// before anything is queued, so it cannot move the shaft either. Displacements
// are a fraction of a rotation. No broadcasts, no alias writes, no test modes:
// safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// device_status.h:16-22, as bit masks.
static const uint16_t BIT_BOOTLOADER        = 1u << 0;  // 0x0001
static const uint16_t BIT_MOSFETS_ENABLED   = 1u << 1;  // 0x0002
static const uint16_t BIT_CLOSED_LOOP       = 1u << 2;  // 0x0004
static const uint16_t BIT_CALIBRATING       = 1u << 3;  // 0x0008
static const uint16_t BIT_HOMING            = 1u << 4;  // 0x0010
static const uint16_t BIT_GO_TO_CLOSED_LOOP = 1u << 5;  // 0x0020
static const uint16_t BIT_MOTOR_BUSY        = 1u << 6;  // 0x0040
// Structurally impossible, not merely unused: the flags are accumulated in a
// uint8_t local before being widened to the uint16_t wire field
// (motor_control.c:3444). If any of these is ever observed set, either the
// firmware changed or the response is being mis-decoded.
static const uint16_t BITS_RESERVED         = 0xFF80u;  // bits 7..15

// Fatal error codes, from python_programs/servomotor/error_codes.json.
static const uint8_t ERR_QUEUE_NOT_EMPTY   = 8;
static const uint8_t ERR_NOT_IN_CLOSED_LOOP = 13;
static const uint8_t ERR_ACCEL_TOO_HIGH    = 15;
static const uint8_t ERR_VEL_TOO_HIGH      = 16;
static const uint8_t ERR_QUEUE_IS_FULL     = 17;
static const uint8_t ERR_PREDICTED_VEL_TOO_HIGH = 28;
static const uint8_t ERR_POSITION_DEVIATION_TOO_LARGE = 45;

// ---------------------------------------------------------------------------
// Every status word this module ever sees passes through readFlags(), which
// folds it into these accumulators. Section 9 then makes one global claim over
// the whole run rather than repeating the same bit checks in every section.
// File-scope state must be re-initialised at entry: the host harness can run
// modules back-to-back in one process (tf_framework.h).
// ---------------------------------------------------------------------------
static uint32_t g_samples;
static uint32_t g_readFailures;
static uint16_t g_flagsUnion;      // OR of every word observed
static uint16_t g_flagsIntersect;  // AND of every word observed

struct FlagSample {
    uint16_t flags;
    uint8_t  code;
    bool     ok;      // false means the status read itself failed
};

static FlagSample readFlags(Servomotor* m) {
    FlagSample s;
    getStatusResponse st = m->getStatus();
    if (m->getError() != 0) {
        s.flags = 0xFFFF;
        s.code = 0xFF;
        s.ok = false;
        g_readFailures++;
        return s;
    }
    s.flags = st.statusFlags;
    s.code = st.fatalErrorCode;
    s.ok = true;
    g_samples++;
    g_flagsUnion |= s.flags;
    g_flagsIntersect &= s.flags;
    return s;
}

// ---------------------------------------------------------------------------
// The five fatal errors used in section 6. Each is reachable with the output
// stage off and each comes from a DIFFERENT place in the firmware -- queue-time
// validation, the ring-buffer capacity check, the main loop's deviation
// supervisor (check_if_actual_vs_desired_position_deviated_too_much, called
// from main.c:1503 -- the main loop, NOT the 31.25 kHz control ISR), a
// precondition check and a mode check -- so "the flags collapse to zero on a
// fault" is tested against the whole family rather than one code path.
// ---------------------------------------------------------------------------
enum { FAULT_QUEUE_FULL = 0, FAULT_LIMIT, FAULT_DEVIATION,
       FAULT_QUEUE_NOT_EMPTY, FAULT_NOT_CLOSED_LOOP, N_FAULTS };

static const char* faultName(int k) {
    switch (k) {
        case FAULT_QUEUE_FULL:      return "queue overflow (17)";
        case FAULT_LIMIT:           return "over-ambitious move (15/16/28)";
        case FAULT_DEVIATION:       return "position deviation (45)";
        case FAULT_QUEUE_NOT_EMPTY: return "zero position mid-move (8)";
        case FAULT_NOT_CLOSED_LOOP: return "homing in open loop (13)";
    }
    return "?";
}

static bool codeIsExpected(int k, uint8_t code) {
    switch (k) {
        case FAULT_QUEUE_FULL:      return code == ERR_QUEUE_IS_FULL;
        // Which of the three fires depends on which check in add_to_queue()
        // trips first: accel (motor_control.c:1796), predicted velocity
        // (:1803), or the velocity-segment check on the one-tick-ramp fast
        // path (:1847). For the exact parameters used below the accel check
        // (15) is the one that fires on fw 0.15.12.0, but the property under
        // test is only that the machine REFUSES the move rather than clamping
        // it, so any of the three is accepted -- the split between them is an
        // arithmetic detail of the planner and is not worth pinning here.
        case FAULT_LIMIT:           return code == ERR_ACCEL_TOO_HIGH ||
                                           code == ERR_VEL_TOO_HIGH ||
                                           code == ERR_PREDICTED_VEL_TOO_HIGH;
        case FAULT_DEVIATION:       return code == ERR_POSITION_DEVIATION_TOO_LARGE;
        case FAULT_QUEUE_NOT_EMPTY: return code == ERR_QUEUE_NOT_EMPTY;
        case FAULT_NOT_CLOSED_LOOP: return code == ERR_NOT_IN_CLOSED_LOOP;
    }
    return false;
}

// Drive the machine into the requested fault. Always starts from a reset, so a
// fault is never induced on top of a previous one (fatal_error() returns early
// if one is already latched -- error_handling.c:191 -- and the first code wins).
static void induceFault(Servomotor* m, int k) {
    switch (k) {
    case FAULT_QUEUE_FULL:
        // Slow limits make each queued segment long, so the queue saturates
        // instead of draining underneath the loop. Ten trapezoid moves at three
        // slots each fill 30 of 32; the next one cannot fit and raises 17
        // (motor_control.c:1869).
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        for (int i = 0; i < 24; i++) {
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 400), 12 * TFH_TPS);
            if (m->getError() != 0) break;
        }
        break;
    case FAULT_LIMIT:
        // One full rotation in 0.1 s against a 1 rot/s^2 acceleration ceiling.
        // Limits are pass/fail checks, never clamps: the move is REFUSED, not
        // slowed down, so nothing turns.
        tfhReset(m);
        m->setMaximumVelocity(0.5f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS / 10);
        break;
    case FAULT_DEVIATION: {
        // tfhReset (NOT tfhFreshOpen) leaves the factory two-rotation deviation
        // window in place -- a system reset restores it unconditionally
        // (motor_control.c:3728), so section 5's temporary three-rotation
        // setting cannot leak in here. With the MOSFETs off the hall position
        // never follows, so a three-rotation command is three rotations of
        // deviation and the main loop's supervisor faults
        // (motor_control.c:3209-3223). The commanded position crosses two
        // rotations about 1.3 s into a 2 s move, so the 15 s deadline below has
        // an order of magnitude of headroom. This one is asynchronous, so poll
        // for it rather than waiting a fixed time.
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 3), 2 * TFH_TPS);
        uint32_t deadline = millis() + 15000;
        while (millis() < deadline) {
            getStatusResponse st = m->getStatus();
            if (m->getError() == 0 && st.fatalErrorCode != 0) break;
            delay(25);
        }
        break;
    }
    case FAULT_QUEUE_NOT_EMPTY:
        // zero_position() refuses outright when the queue is not empty
        // (motor_control.c:3313-3315).
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 8), 10 * TFH_TPS);
        delay(250);
        m->zeroPosition();
        break;
    case FAULT_NOT_CLOSED_LOOP:
        // start_homing()'s first check, before anything is queued
        // (motor_control.c:1436-1438). No motion is possible from this path.
        tfhReset(m);
        m->homingRaw((int32_t)(TFH_CPR / 10), TFH_TPS);
        break;
    }
}

void tm_status_flag_semantics(void) {
    Serial.println("tm_status_flag_semantics: BEGIN\n");
    Servomotor* m = tfGetMotor();

    g_samples = 0;
    g_readFailures = 0;
    g_flagsUnion = 0x0000;
    g_flagsIntersect = 0xFFFF;

    // =====================================================================
    // 1. THE IDLE WORD, BIT BY BIT. Every one of the seven defined bits is
    //    checked individually rather than as a single "== 0x0000" comparison,
    //    because a single comparison tells a failing run nothing about WHICH
    //    bit went wrong. This is the reference value the rest of the module
    //    compares against, and it is also the value a caller has to recognise
    //    as "idle and healthy".
    // =====================================================================
    printf("\n--- 1. the idle status word, bit by bit ---\n");
    {
        tfhReset(m);
        FlagSample s = readFlags(m);
        printf("   after reset: flags=0x%04X fatal=%u (read ok=%d)\n",
               (unsigned)s.flags, (unsigned)s.code, (int)s.ok);

        TEST_RESULT("after a reset the status word is exactly 0x0000",
                    s.ok && s.flags == 0x0000);
        TEST_RESULT("after a reset there is no latched fatal error",
                    s.ok && s.code == 0);
        TEST_RESULT("the bootloader bit (bit 0) is clear, so this is the application",
                    s.ok && (s.flags & BIT_BOOTLOADER) == 0);
        TEST_RESULT("the MOSFETs-enabled bit (bit 1) is clear with the output stage off",
                    s.ok && (s.flags & BIT_MOSFETS_ENABLED) == 0);
        TEST_RESULT("the closed-loop bit (bit 2) is clear after a reset",
                    s.ok && (s.flags & BIT_CLOSED_LOOP) == 0);
        TEST_RESULT("neither the calibrating (bit 3) nor the homing (bit 4) bit is set at rest",
                    s.ok && (s.flags & (BIT_CALIBRATING | BIT_HOMING)) == 0);
        TEST_RESULT("the go-to-closed-loop bit (bit 5) is clear at rest",
                    s.ok && (s.flags & BIT_GO_TO_CLOSED_LOOP) == 0);
        TEST_RESULT("the motor-busy bit (bit 6) is clear at rest",
                    s.ok && (s.flags & BIT_MOTOR_BUSY) == 0);
        TEST_RESULT("the undefined bits 7..15 are clear at rest",
                    s.ok && (s.flags & BITS_RESERVED) == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 2. THE WORD IS STABLE, AND READING IT CHANGES NOTHING. A status word
    //    that flickered between reads would make every other assertion in
    //    this module -- and every supervisor in the field -- unreliable, and
    //    a status read that consumed a queue slot or nudged the position
    //    would make polling unsafe. Both are cheap to measure and neither is
    //    measured anywhere else against the FLAGS word specifically.
    // =====================================================================
    printf("\n--- 2. repeated reads are identical and inert ---\n");
    {
        tfhReset(m);
        int64_t posBefore = m->getPositionRaw();
        uint8_t depthBefore = m->getNQueuedItems();
        bool baselineOk = (m->getError() == 0);

        const int N_READS = 25;
        uint16_t first = 0;
        int differing = 0, failed = 0;
        for (int i = 0; i < N_READS; i++) {
            FlagSample s = readFlags(m);
            if (!s.ok) { failed++; continue; }
            if (i == 0) first = s.flags;
            else if (s.flags != first) differing++;
            delay(10);
        }
        int64_t posAfter = m->getPositionRaw();
        uint8_t depthAfter = m->getNQueuedItems();
        bool afterOk = (m->getError() == 0);

        printf("   %d reads: first=0x%04X differing=%d failed=%d\n",
               N_READS, (unsigned)first, differing, failed);
        printf("   position %lld -> %lld, queue depth %u -> %u\n",
               (long long)posBefore, (long long)posAfter,
               (unsigned)depthBefore, (unsigned)depthAfter);

        TEST_RESULT("twenty-five consecutive status reads all succeed and agree",
                    failed == 0 && differing == 0 && first == 0x0000);
        TEST_RESULT("polling the status word does not consume a queue slot",
                    baselineOk && afterOk && depthBefore == 0 && depthAfter == 0);
        TEST_RESULT("polling the status word does not move the commanded position",
                    baselineOk && afterOk && posAfter == posBefore);
        tfhReset(m);
    }

    // =====================================================================
    // 3. AN ORDINARY MOVE SETS NO FLAG AT ALL. The headline result. The
    //    status word is sampled repeatedly alongside getNQueuedItems while a
    //    six-second move runs; every sample taken with a non-empty queue must
    //    still read 0x0000. In particular the motor-busy bit stays clear,
    //    because motor_busy is set only by calibration (motor_control.c:648),
    //    go-to-closed-loop (:1005) and homing (:1446) -- never by a queued
    //    move. A caller cannot detect motion from this word; getNQueuedItems
    //    is the only motion indicator the protocol offers.
    // =====================================================================
    printf("\n--- 3. an ordinary queued move sets no flag ---\n");
    {
        tfhFreshOpen(m);
        // Slow ceilings stretch the move out so it is still running while the
        // sampling loop runs; a quarter rotation keeps the displacement small.
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);

        const int32_t dist = (int32_t)(TFH_CPR / 4);
        const uint32_t ticks = 6 * TFH_TPS;
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(dist, ticks);
        bool accepted = (m->getError() == 0);

        uint32_t busySamples = 0, nonZeroSamples = 0;
        uint16_t unionDuring = 0;
        uint8_t maxCodeDuring = 0;
        uint8_t maxDepth = 0;
        uint32_t deadline = millis() + 20000;
        while (accepted && millis() < deadline) {
            uint8_t q = m->getNQueuedItems();
            if (m->getError() != 0) break;
            FlagSample s = readFlags(m);
            if (!s.ok) break;
            if (q > 0) {
                busySamples++;
                if (q > maxDepth) maxDepth = q;
                unionDuring |= s.flags;
                if (s.flags != 0x0000) nonZeroSamples++;
                if (s.code > maxCodeDuring) maxCodeDuring = s.code;
            } else {
                break;  // the queue drained; the move is over
            }
            delay(25);
        }

        bool drained = accepted && tfhDrain(m, 20000);
        int64_t moved = drained ? (m->getPositionRaw() - before) : 0;
        FlagSample after = readFlags(m);

        printf("   accepted=%d, %lu samples with a non-empty queue (max depth %u)\n",
               (int)accepted, (unsigned long)busySamples, (unsigned)maxDepth);
        printf("   union of flags during motion = 0x%04X, non-zero samples = %lu\n",
               (unsigned)unionDuring, (unsigned long)nonZeroSamples);
        printf("   moved %lld of %ld counts; flags after = 0x%04X fatal=%u\n",
               (long long)moved, (long)dist, (unsigned)after.flags, (unsigned)after.code);

        TEST_RESULT("a six-second move is accepted and observed with a non-empty queue",
                    accepted && busySamples >= 5);
        TEST_RESULT("the status word reads 0x0000 in every sample taken while the queue was busy",
                    accepted && busySamples >= 5 && nonZeroSamples == 0);
        TEST_RESULT("an ordinary queued move never sets the motor-busy bit (bit 6)",
                    (unionDuring & BIT_MOTOR_BUSY) == 0);
        TEST_RESULT("no fatal error is reported at any point during the move",
                    maxCodeDuring == 0);
        TEST_RESULT("the move completes and delivers its full displacement",
                    drained && llabs((long long)(moved - dist)) <= dist / 100 + 4);
        TEST_RESULT("the status word after the move is the same idle word as before it",
                    after.ok && after.flags == 0x0000 && after.code == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 4. A FULL QUEUE IS INVISIBLE IN THE FLAGS. The other half of the same
    //    property, and the one most likely to trip a caller up: a motor whose
    //    32-slot queue is saturated -- so saturated that the next move will be
    //    refused -- still reports exactly the idle word. Flow control has to
    //    be done with getNQueuedItems (command 11); there is no "queue full"
    //    bit to watch.
    // =====================================================================
    printf("\n--- 4. a saturated queue still reads as the idle word ---\n");
    {
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);

        // Fill by measurement, not by arithmetic: a trapezoid move costs three
        // slots normally but two on the 0.15.12.0 one-tick-ramp fast path, so a
        // hard-coded move count would either under-fill or overflow.
        //
        // The only non-error exit from this loop is `depth > 28`, so the
        // depth >= 29 assertion below is structurally guaranteed rather than
        // hopeful. If you change the 28 threshold, change that assertion with
        // it. The 20-move cap is only a runaway guard: with 1 rot/s and
        // 1 rot/s^2 each move is three segments of 1 s / 10 s / 1 s, so the
        // earliest a slot can be released is one second in, which is an order
        // of magnitude longer than the ten or so round trips it takes to fill.
        int queued = 0;
        uint8_t depth = 0;
        bool fillOk = true;
        while (queued < 20) {
            depth = m->getNQueuedItems();
            if (m->getError() != 0) { fillOk = false; break; }
            if (depth > 28) break;               // no room for another 3-slot move
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 400), 12 * TFH_TPS);
            if (m->getError() != 0) { fillOk = false; break; }
            queued++;
        }
        depth = m->getNQueuedItems();
        if (m->getError() != 0) fillOk = false;
        FlagSample s = readFlags(m);

        printf("   queued %d moves, depth %u of 32, flags=0x%04X fatal=%u\n",
               queued, (unsigned)depth, (unsigned)s.flags, (unsigned)s.code);

        TEST_RESULT("the movement queue can be filled to within one move of capacity",
                    fillOk && depth >= 29);
        TEST_RESULT("a saturated queue leaves the status word at exactly 0x0000",
                    s.ok && s.flags == 0x0000);
        TEST_RESULT("a saturated queue is not a fault: the error byte is still 0",
                    s.ok && s.code == 0);

        m->emergencyStop();
        delay(200);
        tfhReset(m);
    }

    // =====================================================================
    // 5. SETTINGS WRITES NEVER DISTURB THE WORD. Configuration commands do not
    //    appear in get_motor_status_flags_without_disable_enable_irq()
    //    (motor_control.c:3442-3466) at all, so none of them may perturb a
    //    single bit. Worth asserting because these are the commands a
    //    supervisor issues most often between moves.
    //
    //    DO NOT REMOVE the tfhReset at the end of this section, and do not move
    //    this section after section 6: the deviation limit written here (3
    //    rotations) would otherwise still be in force when section 6 induces
    //    fault 45 with a 3-rotation move, which is exactly the marginal case
    //    that would flake. The reset restores the 2-rotation factory value
    //    (motor_control.c:3728) and the safety fence to INT64_MIN/INT64_MAX.
    // =====================================================================
    printf("\n--- 5. configuration writes leave the status word alone ---\n");
    {
        tfhReset(m);
        bool allAccepted = true;

        m->setMaximumVelocity(3.0f);            if (m->getError() != 0) allAccepted = false;
        m->setMaximumAcceleration(1500.0f);     if (m->getError() != 0) allAccepted = false;
        m->setPidConstants(1000, 100, 10000);   if (m->getError() != 0) allAccepted = false;
        m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
        m->setSafetyLimits(-100.0f, 100.0f);    if (m->getError() != 0) allAccepted = false;
        m->setMaxAllowablePositionDeviation(3.0f);
        if (m->getError() != 0) allAccepted = false;
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        m->zeroPosition();                      if (m->getError() != 0) allAccepted = false;

        FlagSample s = readFlags(m);
        printf("   six settings written (all accepted=%d); flags=0x%04X fatal=%u\n",
               (int)allAccepted, (unsigned)s.flags, (unsigned)s.code);

        TEST_RESULT("six configuration writes are all accepted with an empty queue",
                    allAccepted);
        TEST_RESULT("configuration writes leave the status word at exactly 0x0000",
                    s.ok && s.flags == 0x0000 && s.code == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 6. A FAULT FORCES THE WORD TO ZERO. fatal_error() masks the flags down
    //    to the bootloader bit alone (error_handling.c:227) and the fatal-state
    //    handler replies with that snapshot rather than recomputing
    //    (error_handling.c:92-103). So a faulted motor and a healthy idle one
    //    report the SAME flags word, and only the error byte tells them apart.
    //    Five different fatal errors, from five different places in the
    //    firmware, are each induced from a clean reset and checked.
    // =====================================================================
    printf("\n--- 6. five different faults, all collapsing the flags to zero ---\n");
    {
        for (int k = 0; k < N_FAULTS; k++) {
            induceFault(m, k);
            FlagSample s = readFlags(m);
            printf("   %-34s -> flags=0x%04X fatal=%u (read ok=%d)\n",
                   faultName(k), (unsigned)s.flags, (unsigned)s.code, (int)s.ok);

            char name[128];
            snprintf(name, sizeof(name), "%s latches the expected fatal error code",
                     faultName(k));
            TEST_RESULT(name, s.ok && codeIsExpected(k, s.code));

            snprintf(name, sizeof(name),
                     "%s leaves the flags word at 0x0000, indistinguishable from idle",
                     faultName(k));
            TEST_RESULT(name, s.ok && s.flags == 0x0000);

            tfhReset(m);
        }
    }

    // =====================================================================
    // 7. THE FAULTED WORD IS FROZEN, NOT LIVE. In the application the flags
    //    are recomputed on every request (main.c:497). In the fatal-error loop
    //    they are not: the handler memcpy's the cached device_status
    //    (error_handling.c:100). The observable consequence is that the word
    //    cannot change while the fault is latched, and neither can the error
    //    byte -- a later illegal command does not overwrite the first cause,
    //    because fatal_error() returns early once fatal_error_occurred is set
    //    (error_handling.c:191). That is what makes the reported code
    //    trustworthy as a diagnosis.
    // =====================================================================
    printf("\n--- 7. the faulted status word is a frozen snapshot ---\n");
    {
        induceFault(m, FAULT_NOT_CLOSED_LOOP);
        FlagSample first = readFlags(m);
        int changed = 0, failed = 0;
        for (int i = 0; i < 5; i++) {
            FlagSample s = readFlags(m);
            if (!s.ok) { failed++; continue; }
            if (s.flags != first.flags || s.code != first.code) changed++;
            delay(40);
        }

        // Provoke a second, different violation while the fault is latched.
        m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS / 10);
        delay(150);
        FlagSample afterSecond = readFlags(m);

        printf("   first: flags=0x%04X fatal=%u; %d of 5 later reads differed, %d failed\n",
               (unsigned)first.flags, (unsigned)first.code, changed, failed);
        printf("   after a further illegal move: flags=0x%04X fatal=%u\n",
               (unsigned)afterSecond.flags, (unsigned)afterSecond.code);

        TEST_RESULT("six status reads in the faulted state all return the identical word",
                    first.ok && failed == 0 && changed == 0 &&
                    first.code == ERR_NOT_IN_CLOSED_LOOP && first.flags == 0x0000);
        TEST_RESULT("a further illegal command does not overwrite the first fatal error",
                    afterSecond.ok && afterSecond.code == ERR_NOT_IN_CLOSED_LOOP &&
                    afterSecond.flags == 0x0000);
        tfhReset(m);
    }

    // =====================================================================
    // 8. RESET RESTORES THE IDLE WORD, EVERY TIME. The recovery contract: a
    //    system reset (command 27) must put both halves of the status reply
    //    back to their factory values, repeatedly, so a supervisor can rely on
    //    "reset, then re-read" as its way out of any fault.
    // =====================================================================
    printf("\n--- 8. reset returns the status reply to its idle value ---\n");
    {
        bool allLatched = true, allCleared = true;
        for (int cycle = 0; cycle < 3; cycle++) {
            induceFault(m, FAULT_LIMIT);
            FlagSample faulted = readFlags(m);
            if (!faulted.ok || faulted.code == 0) allLatched = false;

            tfhReset(m);
            FlagSample clean = readFlags(m);
            if (!clean.ok || clean.flags != 0x0000 || clean.code != 0) allCleared = false;

            printf("   cycle %d: faulted fatal=%u flags=0x%04X -> after reset fatal=%u flags=0x%04X\n",
                   cycle, (unsigned)faulted.code, (unsigned)faulted.flags,
                   (unsigned)clean.code, (unsigned)clean.flags);
        }
        TEST_RESULT("three fault-and-reset cycles each latch a fatal error", allLatched);
        TEST_RESULT("and each reset returns the status reply to flags 0x0000 with error 0",
                    allCleared);
        tfhReset(m);
    }

    // =====================================================================
    // 9. THE GLOBAL CLAIM OVER EVERY SAMPLE THE MODULE TOOK. Idle, mid-move,
    //    queue-saturated and faulted: the union of every status word observed
    //    must still be 0x0000. That single number is the whole module's result
    //    -- with the output stage off, NOTHING the planner or the queue does
    //    is visible in the flags word, and no undefined bit is ever set.
    // =====================================================================
    printf("\n--- 9. the union of every status word observed ---\n");
    {
        printf("   %lu samples, %lu failed reads, union=0x%04X, intersection=0x%04X\n",
               (unsigned long)g_samples, (unsigned long)g_readFailures,
               (unsigned)g_flagsUnion, (unsigned)g_flagsIntersect);

        // The >= 50 floor only guards against "the union is 0x0000 because we
        // never actually sampled". Sections 1, 2, 4, 5, 6, 7, 8 and the epilogue
        // contribute 48 samples on their own, before section 3's mid-move loop
        // adds its ~150, so this is a floor and not a measurement.
        TEST_RESULT("every status read in the module succeeded",
                    g_readFailures == 0 && g_samples >= 50);
        TEST_RESULT("the union of every status word observed is exactly 0x0000, "
                    "so no defined or undefined bit is ever set with the MOSFETs off",
                    g_flagsUnion == 0x0000 && (g_flagsUnion & BITS_RESERVED) == 0);
    }

    tfhReset(m);
    FlagSample end = readFlags(m);
    printf("\n   final state: flags=0x%04X fatal=%u\n",
           (unsigned)end.flags, (unsigned)end.code);
    TEST_RESULT("the motor is left clean: idle word, no latched error",
                end.ok && end.flags == 0x0000 && end.code == 0);
}
