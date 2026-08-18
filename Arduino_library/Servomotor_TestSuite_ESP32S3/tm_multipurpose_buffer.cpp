#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_multipurpose_buffer.cpp
//
// READ MULTIPURPOSE BUFFER (command 35) AS A PROTOCOL SURFACE, NOT AS A
// DIAGNOSTIC FEATURE.
//
// Command 35 is the only routinely-reachable command in the whole API whose
// reply has NO FIXED SHAPE. Everything else answers with a struct of known
// size; this one answers with whatever the last writer left in a shared blob,
// in whichever of two completely different packet formats fits it
// (firmware/Src/main.c:782-851):
//
//   * buffer EMPTY  -> a normal short-format packet whose entire payload is one
//                      byte, the data-type tag 0 (main.c:792-801)
//   * buffer FULL   -> an EXTENDED-size packet: first size byte 255 followed by
//                      a u16 total length, then the tag byte, then the raw
//                      dataset (main.c:803-845, `packet_size = 255` at :811)
//
// That branch is the exact shape that produced the Arduino library's BUG-25,
// BUG-26 and BUG-27, where variable-length replies were forced through a
// fixed one-byte struct and the surplus bytes were left on the wire to be
// mis-parsed as the head of the NEXT reply. The fix gave the caller ownership
// of the buffer:
//
//     void readMultipurposeBuffer(uint8_t* buffer, uint16_t bufferSize,
//                                 uint16_t* actualSize);
//
// and that signature introduces its own contract -- how much is written, what
// `actualSize` means, what happens when the caller's buffer is too small --
// which nothing currently asserts.
//
// WHAT THIS MODULE ASSERTS. Command 35 is a well-behaved, boring, TOTAL
// function of the machine's state:
//
//   1. it is ANSWERABLE in every machine state reachable with the output stage
//      off, and refused in exactly one state (faulted) for exactly the reason
//      finding F2 documents;
//   2. the length it reports is SELF-CONSISTENT and never exceeds the buffer
//      the caller supplied, at any buffer size;
//   3. repeated reads with nothing writing in between are IDENTICAL;
//   4. a deliberately undersized buffer is refused SAFELY -- nothing is written
//      past it, and the partial frame is drained so the link stays in sync;
//   5. reading is PURE: it queues nothing, latches nothing, and does not
//      perturb a move that is already running.
//
// HOW THIS DIFFERS FROM tm_varlen_bugs, THE ONLY OTHER MODULE THAT TOUCHES
// COMMAND 35. That module is a regression guard for the BUG-26/27 fix and its
// interesting half needs a POPULATED buffer, so it enables the MOSFETs, drives
// the motor into closed loop and asserts test mode 3 to make the PID write a
// snapshot. This module deliberately never does any of that: it characterises
// the command with the output stage OFF, which is the state the shared rack
// runs in, and it is about the protocol contract (lengths, buffers, purity,
// state coverage) rather than about the dataset. The two do not overlap in a
// single assertion. tm_readonly_purity, which checks exactly this purity
// property for ten OTHER getters, explicitly EXCLUDES readMultipurposeBuffer
// from its list ("contents depend on what last wrote to it") -- this module is
// what fills that hole.
//
// WHY THE EMPTY TAG IS THE ONLY ANSWER WE CAN SEE HERE, and why that is fine.
// On current products the blob is written from exactly one place that a test
// can reach: the PID debug snapshot at motor_control.c:2509-2517. That code
// lives inside PID_controller(), which the control loop calls ONLY when
// motor_control_mode == CLOSED_LOOP_POSITION_CONTROL (motor_control.c:2673,
// :2724, :2783). With the MOSFETs off there is no closed loop, so the blob can
// never be filled and every read here returns the one-byte empty tag. The
// other writers are out of reach as well:
//
//   * test mode 4's GC6609 register dump is inside `#ifdef GC6609`
//     (motor_control.c:3081-3087), so it is not even compiled on current units;
//   * the go-to-closed-loop dataset comes from process_go_to_closed_loop_data(),
//     which the main loop calls only under `#ifdef PRODUCT_NAME_M1`
//     (main.c:1506-1508);
//   * the VIBRATE dataset (motor_control.c:1245) is NOT product-specific -- the
//     Vibrate command, 40, populates the buffer on every product. It is out of
//     reach here only because this module never sends it, and must not: vibrate
//     calls check_current_sensor_and_enable_mosfets() and energises the motor;
//   * MULTIPURPOSE_DATA_TYPE_CALIBRATION (1) is declared but never assigned
//     anywhere in the firmware, so calibration cannot populate the buffer either.
//
// None of those can leak in from an earlier module either: every section here
// starts from a system reset, and motor_control_init() sets the type back to 0
// (motor_control.c:3794).
//
// That makes the empty tag an EXACT, PREDICTABLE ANSWER rather than a
// limitation: one byte, value 0, every single time, in every state. Almost
// every property above is sharper against a known constant than it would be
// against a live dataset. The one property it cannot exercise -- a payload
// LARGER than the caller's buffer -- is covered in section 5 through
// getProductDescription (command 24), which returns "Servomotor\0", 11 bytes
// (main.c:60, :580-590), through the IDENTICAL caller-buffer path in
// Communication::getResponse (Communication.cpp:444-452). Same code, same
// contract, a payload big enough to overflow something.
//
// SAFETY. The MOSFETs are never enabled and goToClosedLoop() is never called;
// every distance measured is the COMMANDED position, which the planner advances
// whether or not the output stage is energised, so nothing turns on any motor
// at any point. The only motion commanded is under one rotation per section,
// well inside the deviation window. One fatal error is induced deliberately in
// section 6 -- with an over-limit move that is refused at queue time, so again
// nothing turns -- and is cleared by the reset that follows it. Nothing is ever
// broadcast, no alias is ever set, and test mode is never used, so this is safe
// on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// The data-type tag that means "nothing stored" (motor_control.h:142-146 define
// 1..5 for the real datasets; 0 is the cleared value set by
// clear_multipurpose_data(), motor_control.c:3631, and by the power-on init at
// motor_control.c:3794).
static const uint8_t MPB_TAG_EMPTY = 0;

// The empty reply carries exactly one payload byte: the tag (main.c:800).
static const uint16_t MPB_EMPTY_PAYLOAD_BYTES = 1;

// "Servomotor" plus its null terminator (main.c:60, transmitted with the
// terminator at main.c:589).
static const uint16_t PRODUCT_DESCRIPTION_BYTES = 11;

// MOVEMENT_QUEUE_SIZE, motor_control.h:31. A trapezoid move costs 3 slots.
static const int QUEUE_SLOTS = 32;

// Fill value for the caller buffer before every call. Any byte the library
// writes that it should not have written shows up as a change from this.
static const uint8_t CANARY = 0xA5;

static void fillCanary(uint8_t* buf, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) buf[i] = CANARY;
}

// True if every byte in [from, to) is still the canary, i.e. untouched.
static bool canaryIntact(const uint8_t* buf, uint16_t from, uint16_t to) {
    for (uint16_t i = from; i < to; i++) {
        if (buf[i] != CANARY) return false;
    }
    return true;
}

// One read into a canaried buffer. Reports the library error, the length the
// library claims, and whether everything past that length is still untouched.
struct MpbRead {
    int      err;          // Servomotor::getError() after the call
    uint16_t len;          // *actualSize
    uint8_t  tag;          // first payload byte, or 0xFF if none was returned
    bool     tailIntact;   // nothing written at or beyond `len`
};

static MpbRead mpbRead(Servomotor* m, uint16_t bufferSize) {
    // Always allocate the FULL scratch area and only tell the library about
    // `bufferSize` of it, so an overrun past the declared size lands in memory
    // we can inspect rather than in something else's variables.
    static uint8_t scratch[256];
    MpbRead r;
    fillCanary(scratch, sizeof(scratch));
    uint16_t len = 0xFFFF;                 // must be overwritten by the call
    m->readMultipurposeBuffer(scratch, bufferSize, &len);
    r.err = m->getError();
    r.len = len;
    r.tag = (len >= 1 && bufferSize >= 1) ? scratch[0] : 0xFF;
    uint16_t from = (len == 0xFFFF) ? 0 : len;
    if (from > sizeof(scratch)) from = sizeof(scratch);
    r.tailIntact = canaryIntact(scratch, from, sizeof(scratch));
    return r;
}

// A normal, generous-buffer read: the shape used whenever the point of the call
// is "does it answer", not "how does it handle my buffer".
static MpbRead mpbReadNormal(Servomotor* m) {
    return mpbRead(m, 128);
}

// True if a read is the well-formed empty answer: succeeded, one byte, tag 0.
static bool isEmptyAnswer(const MpbRead& r) {
    return r.err == 0 && r.len == MPB_EMPTY_PAYLOAD_BYTES && r.tag == MPB_TAG_EMPTY;
}

// Drop CRC32 framing on BOTH ends, device first. Doing it in the other order
// desynchronises the link and everything times out.
static bool crcOff(Servomotor* m) {
    m->crc32Control(0);
    if (m->getError() != 0) return false;
    m->disableCRC32();
    return true;
}

// And back on: the enable command goes out WITHOUT a CRC (the firmware is not
// expecting one yet), then the library starts adding them.
static bool crcOn(Servomotor* m) {
    m->crc32Control(1);
    if (m->getError() != 0) return false;
    m->enableCRC32();
    return true;
}

// Latch a fatal error deterministically and without motion: a move needing far
// more speed than the ceiling allows is refused at queue time. Same technique
// as tm_fatal_state_commands, which established that it is reliable.
static uint8_t induceFault(Servomotor* m) {
    tfhReset(m);
    m->setMaximumVelocity(0.2f);
    m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS / 4);
    delay(400);
    return tfhFatal(m);
}

void tm_multipurpose_buffer(void) {
    Serial.println("tm_multipurpose_buffer: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE EMPTY ANSWER IS WELL FORMED. Before firmware 0.15.4.0 an empty
    //    buffer produced NO RESPONSE AT ALL and the host simply timed out --
    //    a read that hangs is the worst possible failure mode for a
    //    diagnostic command, because it looks like a dead motor. The fix was
    //    to always answer, with a single tag byte of 0 (main.c:792-801).
    //    Everything else in this module assumes that answer, so it is pinned
    //    down first: it must succeed, be exactly one byte long, contain the
    //    tag 0, latch nothing, and write nothing beyond that one byte.
    // =====================================================================
    printf("\n--- 1. an empty buffer answers with a single tag byte ---\n");
    {
        tfhReset(m);
        MpbRead r = mpbReadNormal(m);
        uint8_t fat = tfhFatal(m);
        printf("   err=%d len=%u tag=%u tailIntact=%d fatal=%u\n",
               r.err, (unsigned)r.len, (unsigned)r.tag, (int)r.tailIntact, (unsigned)fat);
        TEST_RESULT("reading the multipurpose buffer succeeds on a freshly reset motor",
                    r.err == 0);
        TEST_RESULT("an empty multipurpose buffer returns exactly one payload byte",
                    r.len == MPB_EMPTY_PAYLOAD_BYTES);
        TEST_RESULT("and that byte is the empty data-type tag, 0",
                    r.tag == MPB_TAG_EMPTY);
        TEST_RESULT("nothing is written past the length the library reports",
                    r.tailIntact);
        TEST_RESULT("reading the multipurpose buffer latches no fatal error", fat == 0);
    }

    // =====================================================================
    // 2. REPEATED READS ARE IDENTICAL. A successful read CLEARS the buffer
    //    (main.c:844), so this command is one of the very few reads in the
    //    API with a documented side effect. That makes "what does the second
    //    read return?" a real question rather than a trivial one, and the
    //    answer has to be stable: with nothing writing in between, twelve
    //    consecutive reads must be indistinguishable. A drift here would mean
    //    a stale length or a stale pointer surviving the clear -- note that
    //    clear_multipurpose_data() zeroes only the TYPE and deliberately
    //    leaves the SIZE behind (motor_control.c:3631-3634), so a regression
    //    that dropped the type check would resurrect a stale byte count.
    // =====================================================================
    printf("\n--- 2. repeated reads with nothing writing in between ---\n");
    {
        tfhReset(m);
        const int nReads = 12;
        bool allOk = true, sameLen = true, sameTag = true;
        MpbRead first = mpbReadNormal(m);
        if (first.err != 0) allOk = false;
        for (int i = 1; i < nReads; i++) {
            MpbRead r = mpbReadNormal(m);
            if (r.err != 0) { allOk = false; printf("   read %d failed: err=%d\n", i, r.err); }
            if (r.len != first.len) {
                sameLen = false;
                printf("   read %d length %u != first %u\n", i, (unsigned)r.len, (unsigned)first.len);
            }
            if (r.tag != first.tag) {
                sameTag = false;
                printf("   read %d tag %u != first %u\n", i, (unsigned)r.tag, (unsigned)first.tag);
            }
        }
        uint8_t fat = tfhFatal(m);
        printf("   %d reads: first len=%u tag=%u; allOk=%d sameLen=%d sameTag=%d fatal=%u\n",
               nReads, (unsigned)first.len, (unsigned)first.tag,
               (int)allOk, (int)sameLen, (int)sameTag, (unsigned)fat);
        TEST_RESULT("twelve consecutive multipurpose reads all succeed", allOk);
        TEST_RESULT("every one of them reports the same length", sameLen);
        TEST_RESULT("every one of them returns the same data-type tag", sameTag);
        TEST_RESULT("twelve reads in a row latch no fatal error", fat == 0);
    }

    // =====================================================================
    // 3. THE REPORTED LENGTH IS A PROPERTY OF THE DATA, NOT OF THE BUFFER.
    //    The caller-buffer API has one obvious way to go wrong: report back
    //    how much room was OFFERED rather than how much data there IS. A
    //    caller that trusted such a length would read uninitialised memory.
    //    Sweeping the buffer size from the exact fit (1) up to 128 must
    //    change nothing about the answer, and must never write past the size
    //    that was declared.
    // =====================================================================
    printf("\n--- 3. the reported length does not depend on the buffer size ---\n");
    {
        tfhReset(m);
        const uint16_t sizes[] = { 1, 2, 3, 4, 8, 16, 20, 21, 64, 128 };
        const int nSizes = (int)(sizeof(sizes) / sizeof(sizes[0]));
        bool allOk = true, withinBuffer = true, sameLen = true, sameTag = true, noOverrun = true;
        for (int i = 0; i < nSizes; i++) {
            MpbRead r = mpbRead(m, sizes[i]);
            if (r.err != 0) {
                allOk = false;
                printf("   bufferSize %u: err=%d\n", (unsigned)sizes[i], r.err);
            }
            if (r.len > sizes[i]) {
                withinBuffer = false;
                printf("   bufferSize %u: reported length %u EXCEEDS the buffer\n",
                       (unsigned)sizes[i], (unsigned)r.len);
            }
            if (r.len != MPB_EMPTY_PAYLOAD_BYTES) {
                sameLen = false;
                printf("   bufferSize %u: length %u, expected %u\n",
                       (unsigned)sizes[i], (unsigned)r.len, (unsigned)MPB_EMPTY_PAYLOAD_BYTES);
            }
            if (r.tag != MPB_TAG_EMPTY) {
                sameTag = false;
                printf("   bufferSize %u: tag %u\n", (unsigned)sizes[i], (unsigned)r.tag);
            }
            if (!r.tailIntact) {
                noOverrun = false;
                printf("   bufferSize %u: bytes were written past the reported length\n",
                       (unsigned)sizes[i]);
            }
        }
        printf("   swept %d buffer sizes from %u to %u bytes\n",
               nSizes, (unsigned)sizes[0], (unsigned)sizes[nSizes - 1]);
        TEST_RESULT("every buffer size from an exact fit to 128 bytes is answered", allOk);
        TEST_RESULT("the reported length never exceeds the buffer supplied", withinBuffer);
        TEST_RESULT("the answer is identical whatever buffer size is offered",
                    sameLen && sameTag);
        TEST_RESULT("no byte past the reported length is written at any buffer size", noOverrun);
    }

    // =====================================================================
    // 4. A BUFFER THAT CANNOT WORK IS REFUSED BEFORE ANYTHING IS SENT. The
    //    library guards zero-length and null buffers up front
    //    (Servomotor.cpp:1473-1477) and returns without putting a byte on the
    //    wire. That ordering matters on a half-duplex bus: a command sent and
    //    then abandoned leaves a reply nobody reads, which the NEXT call
    //    parses as its own -- historically surfacing as a spurious -7
    //    BAD_FIRST_BYTE on a command that was itself perfectly fine. So the
    //    refusal must be visible to the caller AND invisible to the link.
    // =====================================================================
    printf("\n--- 4. impossible buffers are refused without touching the wire ---\n");
    {
        tfhReset(m);
        static uint8_t scratch[64];

        fillCanary(scratch, sizeof(scratch));
        uint16_t zeroLen = 0xFFFF;
        m->readMultipurposeBuffer(scratch, 0, &zeroLen);
        int zeroErr = m->getError();
        bool zeroUntouched = canaryIntact(scratch, 0, sizeof(scratch));

        uint16_t nullLen = 0xFFFF;
        m->readMultipurposeBuffer(nullptr, 16, &nullLen);
        int nullErr = m->getError();

        // The link must be exactly where it was: a normal read has to work.
        MpbRead after = mpbReadNormal(m);
        uint8_t fat = tfhFatal(m);

        printf("   bufferSize=0: err=%d len=%u untouched=%d\n",
               zeroErr, (unsigned)zeroLen, (int)zeroUntouched);
        printf("   buffer=null:  err=%d len=%u\n", nullErr, (unsigned)nullLen);
        printf("   next normal read: err=%d len=%u tag=%u; fatal=%u\n",
               after.err, (unsigned)after.len, (unsigned)after.tag, (unsigned)fat);
        TEST_RESULT("a zero-length buffer is refused with BUFFER_TOO_SMALL",
                    zeroErr == COMMUNICATION_ERROR_BUFFER_TOO_SMALL);
        TEST_RESULT("a refused zero-length read reports length 0 and touches nothing",
                    zeroLen == 0 && zeroUntouched);
        TEST_RESULT("a null buffer is refused the same way, reporting length 0",
                    nullErr == COMMUNICATION_ERROR_BUFFER_TOO_SMALL && nullLen == 0);
        TEST_RESULT("the refusals put nothing on the bus: the next read is still correct",
                    isEmptyAnswer(after));
        TEST_RESULT("refused reads latch no fatal error", fat == 0);
    }

    // =====================================================================
    // 5. A PAYLOAD BIGGER THAN THE BUFFER TRUNCATES SAFELY. This is the case
    //    the multipurpose buffer itself cannot produce with the output stage
    //    off (see the header: its only reachable writer runs inside the
    //    closed-loop PID), so it is exercised through the ONE other command
    //    that returns a variable-length payload into a caller buffer:
    //    getProductDescription, command 24, "Servomotor\0" = 11 bytes
    //    (main.c:60, :580-590). Both commands hand the caller's buffer to the
    //    same Communication::getResponse, and the too-small case is handled
    //    there once, for both (Communication.cpp:444-452).
    //
    //    The contract has three parts and all three matter. Refuse rather
    //    than silently truncate, so the caller cannot mistake a fragment for
    //    the whole thing. Report the FULL required size, so the caller can
    //    resize and retry rather than guess. And drain the rest of the frame,
    //    so the oversized reply does not become the head of the next one --
    //    which is precisely the BUG-25/26/27 failure mode this whole family
    //    of fixes was about.
    // =====================================================================
    printf("\n--- 5. an oversized payload is refused, sized, and drained ---\n");
    {
        tfhReset(m);

        // NOTE ON THE TWO DIFFERENT LENGTHS BELOW -- they are not a typo, and
        // they are what the library actually does, so do not "fix" one to match
        // the other. On SUCCESS getProductDescription reports strlen() of the
        // string it just stored (Servomotor.cpp:1005), i.e. 10 for
        // "Servomotor". On the BUFFER_TOO_SMALL path there is no string to
        // measure, so it passes through the full payload byte count that
        // getResponse recovered (Servomotor.cpp:1010, Communication.cpp:450),
        // i.e. 11 -- which is exactly the number the caller needs in order to
        // size a retry buffer. Same command, two deliberately different
        // conventions, one per outcome.
        char big[64];
        uint16_t bigLen = 0xFFFF;
        m->getProductDescription(big, sizeof(big), &bigLen);
        int bigErr = m->getError();
        bool bigOk = (bigErr == 0) && (bigLen == PRODUCT_DESCRIPTION_BYTES - 1);

        // Now offer four bytes for an eleven-byte reply.
        static uint8_t small[64];
        fillCanary(small, sizeof(small));
        uint16_t smallLen = 0xFFFF;
        m->getProductDescription((char*)small, 4, &smallLen);
        int smallErr = m->getError();
        // Index 0 is deliberately set to '\0' by the library on this path
        // (Servomotor.cpp:1011), so the untouched region starts at 1.
        bool nothingPastBuffer = canaryIntact(small, 1, sizeof(small));

        // If the surplus bytes were left on the wire, this read gets garbage.
        MpbRead after = mpbReadNormal(m);
        uint8_t fat = tfhFatal(m);

        printf("   full buffer: err=%d len=%u text='%s'\n", bigErr, (unsigned)bigLen,
               bigOk ? big : "?");
        printf("   4-byte buffer: err=%d len=%u (want err=%d len=%u) canaryPast=%d\n",
               smallErr, (unsigned)smallLen, COMMUNICATION_ERROR_BUFFER_TOO_SMALL,
               (unsigned)PRODUCT_DESCRIPTION_BYTES, (int)nothingPastBuffer);
        printf("   multipurpose read afterwards: err=%d len=%u tag=%u; fatal=%u\n",
               after.err, (unsigned)after.len, (unsigned)after.tag, (unsigned)fat);
        TEST_RESULT("a generous buffer receives the whole variable-length reply", bigOk);
        TEST_RESULT("a four-byte buffer is refused rather than silently truncated",
                    smallErr == COMMUNICATION_ERROR_BUFFER_TOO_SMALL);
        TEST_RESULT("the refusal reports the full size needed, so the caller can retry",
                    smallLen == PRODUCT_DESCRIPTION_BYTES);
        TEST_RESULT("not one byte is written past the buffer the caller declared",
                    nothingPastBuffer);
        TEST_RESULT("the surplus is drained: the next multipurpose read is still correct",
                    isEmptyAnswer(after) && fat == 0);
    }

    // =====================================================================
    // 6. ANSWERABLE IN EVERY STATE THAT CAN SAFELY BE REACHED. A diagnostic
    //    command is only worth anything if it works when the machine is busy,
    //    and this one is reached from the same main loop that services the
    //    motion planner. Each state below is a different point in that loop:
    //    a queue nearly full of pending moves, the instant after an emergency
    //    stop has torn the queue down, immediately after the position origin
    //    has been rewritten, and with CRC32 framing off and back on -- the
    //    last of those matters because the populated branch of command 35
    //    hand-rolls its own CRC32 accumulation over a multi-chunk transmit
    //    (main.c:824-841) instead of using the usual finalize-and-transmit
    //    helper, so CRC state is a genuine axis for this command.
    //
    //    The one state where it is REFUSED is the faulted one, and that is
    //    not a defect: fatal_error() disables interrupts and hand-pumps the
    //    UART, and get_status is the only handler that completes. That is
    //    finding F2 of the August 2026 campaign, and asserting it here means
    //    command 35 is pinned to the documented contract rather than
    //    silently drifting into a timeout.
    // =====================================================================
    printf("\n--- 6. answerable in every safely reachable state ---\n");
    {
        // --- a queue nearly full of pending moves ---
        tfhFreshOpen(m);
        const int32_t step = (int32_t)(TFH_CPR / 32);   // 1/32 rotation per move
        const uint32_t dur = 2 * TFH_TPS;               // 2 s each: the queue stays deep
        for (int i = 0; i < 12; i++) {
            uint8_t d = m->getNQueuedItems();
            if (m->getError() != 0) break;
            if ((int)d + 3 > QUEUE_SLOTS) break;        // never overfill: that faults
            m->trapezoidMoveRaw(step, dur);
            if (m->getError() != 0) break;
        }
        uint8_t depth = m->getNQueuedItems();
        MpbRead queued = mpbReadNormal(m);
        printf("   queue depth %u of %d slots: err=%d len=%u tag=%u\n",
               (unsigned)depth, QUEUE_SLOTS, queued.err,
               (unsigned)queued.len, (unsigned)queued.tag);
        // Why 24 is a safe floor rather than a lucky number, and why it should
        // not be tightened: the loop above stops at a pre-move depth of 30 (ten
        // moves x 3 slots; the eleventh would need 33). With the ceilings
        // tfhOpenCeilings sets, delta_t1 = 15 x 31250 / 5000 = 93 ticks, so the
        // segments of each move are 3 ms / 1.994 s / 3 ms -- the machine retires
        // only ONE segment in the first two seconds. Even if the twenty-one
        // command round trips of the fill loop took a full three seconds, at
        // most four segments could have been consumed, leaving 26. Asserting
        // equality with 30 would instead be a bet on bus latency.
        TEST_RESULT("the multipurpose buffer is readable with the motion queue nearly full",
                    depth >= 24 && isEmptyAnswer(queued));

        // --- the instant after an emergency stop ---
        m->emergencyStop();
        delay(300);
        uint8_t depthAfterStop = m->getNQueuedItems();
        MpbRead stopped = mpbReadNormal(m);
        printf("   after emergency stop: depth=%u err=%d len=%u tag=%u\n",
               (unsigned)depthAfterStop, stopped.err,
               (unsigned)stopped.len, (unsigned)stopped.tag);
        TEST_RESULT("it is readable immediately after an emergency stop",
                    depthAfterStop == 0 && isEmptyAnswer(stopped));

        // --- straight after the position origin is rewritten ---
        // zero_position() raises ERROR_QUEUE_NOT_EMPTY if anything is pending
        // (motor_control.c:3313), so this only runs on the drained queue above.
        m->zeroPosition();
        bool zeroOk = (m->getError() == 0);
        MpbRead zeroed = mpbReadNormal(m);
        printf("   after zero position (accepted=%d): err=%d len=%u tag=%u\n",
               (int)zeroOk, zeroed.err, (unsigned)zeroed.len, (unsigned)zeroed.tag);
        TEST_RESULT("it is readable right after the position origin is rewritten",
                    zeroOk && isEmptyAnswer(zeroed));

        // --- with CRC32 framing dropped on both ends, and restored ---
        tfhReset(m);
        bool wentOff = crcOff(m);
        MpbRead noCrc = wentOff ? mpbReadNormal(m) : MpbRead{-99, 0, 0xFF, false};
        bool cameBack = crcOn(m);
        if (!cameBack) m->enableCRC32();   // never leave the library out of sync
        MpbRead withCrc = mpbReadNormal(m);
        printf("   CRC32 off (ok=%d): err=%d len=%u tag=%u | back on (ok=%d): err=%d len=%u tag=%u\n",
               (int)wentOff, noCrc.err, (unsigned)noCrc.len, (unsigned)noCrc.tag,
               (int)cameBack, withCrc.err, (unsigned)withCrc.len, (unsigned)withCrc.tag);
        TEST_RESULT("it is readable with CRC32 framing disabled on both ends",
                    wentOff && isEmptyAnswer(noCrc));
        TEST_RESULT("and identical once CRC32 framing is switched back on",
                    cameBack && isEmptyAnswer(withCrc) &&
                    withCrc.len == noCrc.len && withCrc.tag == noCrc.tag);

        // --- addressed explicitly by unique ID ---
        // A different send path (Communication::sendCommandByUniqueId) with its
        // own copy of the reply handling, so it is worth its own probe.
        tfhReset(m);
        static uint8_t uidBuf[64];
        fillCanary(uidBuf, sizeof(uidBuf));
        uint16_t uidLen = 0xFFFF;
        m->readMultipurposeBuffer(m->usingThisUniqueId(), uidBuf, sizeof(uidBuf), &uidLen);
        int uidErr = m->getError();
        printf("   by unique ID: err=%d len=%u tag=%u tailIntact=%d\n",
               uidErr, (unsigned)uidLen, uidLen >= 1 ? (unsigned)uidBuf[0] : 255u,
               (int)canaryIntact(uidBuf, uidLen == 0xFFFF ? 0 : uidLen, sizeof(uidBuf)));
        TEST_RESULT("the explicit unique-ID overload gives the same answer",
                    uidErr == 0 && uidLen == MPB_EMPTY_PAYLOAD_BYTES &&
                    uidBuf[0] == MPB_TAG_EMPTY &&
                    canaryIntact(uidBuf, MPB_EMPTY_PAYLOAD_BYTES, sizeof(uidBuf)));

        // --- while a fatal error is latched: refused, per finding F2 ---
        uint8_t code = induceFault(m);
        MpbRead faulted = mpbReadNormal(m);
        printf("   faulted (code %u): err=%d len=%u\n",
               (unsigned)code, faulted.err, (unsigned)faulted.len);
        TEST_RESULT("a fault is induced without motion so the faulted state is testable",
                    code != 0 && code != 0xFF);
        TEST_RESULT("a faulted motor refuses the multipurpose read instead of hanging",
                    faulted.err != 0 && faulted.len == 0);

        // --- and it answers again as soon as the fault is cleared ---
        tfhReset(m);
        MpbRead recovered = mpbReadNormal(m);
        uint8_t fat = tfhFatal(m);
        printf("   after the recovering reset: err=%d len=%u tag=%u fatal=%u\n",
               recovered.err, (unsigned)recovered.len, (unsigned)recovered.tag, (unsigned)fat);
        TEST_RESULT("and it answers normally again once the reset clears the fault",
                    fat == 0 && isEmptyAnswer(recovered));

        tfhReset(m);
    }

    // =====================================================================
    // 7. READING IT IS PURE. tm_readonly_purity establishes this property for
    //    ten getters and deliberately leaves this one out, because its
    //    contents depend on what last wrote to them. But purity is exactly
    //    what makes a diagnostic command safe to poll, and this command has
    //    more reason than most to fail it: it is the only read with a
    //    documented SIDE EFFECT (a successful read clears the buffer,
    //    main.c:844), and on the populated path it takes over the transmitter
    //    for a multi-chunk send in the middle of the main loop
    //    (main.c:828-839). Three separate things are checked: it consumes no
    //    queue slot on an idle machine, it consumes none while a move is
    //    pending, and hammering it for the whole duration of a move leaves
    //    that move landing exactly where an undisturbed one did.
    // =====================================================================
    printf("\n--- 7. reading it perturbs nothing ---\n");
    {
        // Idle: thirty reads must change neither the queue nor the position.
        tfhFreshOpen(m);
        int64_t posBefore = m->getPositionRaw();
        uint8_t depthBefore = m->getNQueuedItems();
        bool allOk = true;
        for (int i = 0; i < 30 && allOk; i++) {
            MpbRead r = mpbReadNormal(m);
            if (!isEmptyAnswer(r)) {
                allOk = false;
                printf("   idle read %d: err=%d len=%u tag=%u\n",
                       i, r.err, (unsigned)r.len, (unsigned)r.tag);
            }
        }
        int64_t posAfter = m->getPositionRaw();
        uint8_t depthAfter = m->getNQueuedItems();
        printf("   idle: 30 reads, queue %u -> %u, position %lld -> %lld\n",
               (unsigned)depthBefore, (unsigned)depthAfter,
               (long long)posBefore, (long long)posAfter);
        TEST_RESULT("thirty reads on an idle motor all return the same empty answer", allOk);
        TEST_RESULT("thirty reads occupy no queue slot and move nothing",
                    depthBefore == 0 && depthAfter == 0 && posAfter == posBefore);

        // With a long move pending: the depth must not budge.
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);          // long segments: depth is stable
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 200), 8 * TFH_TPS);
        uint8_t qBefore = m->getNQueuedItems();
        bool pendingOk = true;
        for (int i = 0; i < 40 && pendingOk; i++) {
            pendingOk = (mpbReadNormal(m).err == 0);
        }
        uint8_t qAfter = m->getNQueuedItems();
        printf("   with a move pending: queue %u -> 40 reads -> %u\n",
               (unsigned)qBefore, (unsigned)qAfter);
        TEST_RESULT("forty reads succeed while a move is pending", pendingOk);
        // WEAKENED DELIBERATELY, from `qAfter == qBefore`. That equality is not
        // a property of the command, it is a property of the clock. With
        // maxVelocity 1 and maxAcceleration 1 the planner queues THREE segments
        // -- ramp / coast / ramp -- of 31250, 187500 and 31250 timesteps
        // (queue_trapezoid_segments, motor_control.c:2014-2042; delta_t1 =
        // max_velocity/max_acceleration = 31250 ticks = 1 s). The control loop
        // is entitled to retire that first one-second segment WHILE the forty
        // reads are in flight, and how long forty round trips take depends
        // entirely on the link: fast on the ESP32's own RS485 bus, much slower
        // in host mode through a USB adapter. An equality here would fail on a
        // perfectly healthy machine simply because the bus was slow that day.
        //
        // What is actually being tested is that reading is PURE, i.e. that a
        // read never ADDS a queue item; the queue shrinking on its own is the
        // motor doing its job. So: never grows, and the move is still pending
        // at the end (which also proves the reads did not abort it).
        TEST_RESULT("and reading never adds a queue item while a move is pending",
                    qBefore > 0 && qAfter <= qBefore && qAfter > 0);
        m->emergencyStop();
        delay(300);

        // The endpoint test: an undisturbed reference move, then the same move
        // under continuous polling. NEVER a fixed wait -- tfhDrain, always.
        const int32_t dist = (int32_t)(TFH_CPR / 2);
        const uint32_t dur = 2 * TFH_TPS;

        tfhFreshOpen(m);
        bool refOk = false;
        int64_t reference = tfhMove(m, dist, dur, &refOk);

        tfhFreshOpen(m);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(dist, dur);
        bool launched = (m->getError() == 0);
        int polls = 0;
        bool everyPollGood = launched;
        uint32_t deadline = millis() + 2200;
        while (millis() < deadline) {
            MpbRead r = mpbReadNormal(m);
            polls++;
            if (!isEmptyAnswer(r)) {
                everyPollGood = false;
                printf("   poll %d during the move: err=%d len=%u tag=%u\n",
                       polls, r.err, (unsigned)r.len, (unsigned)r.tag);
                break;
            }
        }
        bool drained = launched && tfhDrain(m, 12000);
        uint8_t fat = tfhFatal(m);
        int64_t moved = (drained && fat == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   %d polls during a %ld-count move: moved %lld vs reference %lld, fatal=%u\n",
               polls, (long)dist, (long long)moved, (long long)reference, (unsigned)fat);
        TEST_RESULT("it can be polled continuously for the whole duration of a move",
                    everyPollGood && polls > 20);
        TEST_RESULT("a move under continuous polling still completes cleanly",
                    refOk && drained && fat == 0);
        TEST_RESULT("and lands exactly where the undisturbed reference move landed",
                    refOk && drained && llabs((long long)(moved - reference)) <= 4);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
