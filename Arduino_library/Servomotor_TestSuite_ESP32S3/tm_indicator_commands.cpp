#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_indicator_commands.cpp
//
// THE TWO COMMANDS WITH NO READABLE OUTPUT.
//
// Vibrate (command 40) and Identify (command 41) are the only two commands in
// the API whose entire purpose is to signal to a HUMAN. Identify flashes the
// green LED; Vibrate shakes the shaft. Neither returns a measurement, neither
// changes a value another command can read back, and neither can be confirmed
// by looking at the reply -- the reply is the same "no error" packet whether the
// command did its job or did nothing at all. On this product it genuinely does
// nothing at all: the whole body of vibrate() is wrapped in
// `#ifdef PRODUCT_NAME_M1` (firmware/Src/motor_control.c:1219-1257), so on M2,
// M17 and M23 the function is EMPTY and still answers with success.
//
// That combination -- no output, no observable effect, and an unconditional
// success reply -- means these two commands could be completely broken and
// every existing test would still pass. They are the easiest commands in the
// whole API to break silently.
//
// So this module tests the things that CAN be established without a human in
// the loop, and it deliberately picks the ones that would actually catch a
// regression:
//
//   1. THE COMMAND NUMBERS THEMSELVES. Both numbers are verified against the
//      firmware ON THE WIRE, not merely against a header. A command byte the
//      firmware does not implement falls through to `default: rs485_done_with_
//      this_packet(); break;` (firmware/Src/main.c:1146-1148) and answers
//      NOTHING at all, so a reply proves the byte is implemented. Better, the
//      two commands have DIFFERENT payload-size contracts -- 40 takes exactly
//      one byte (main.c:937) and 41 takes exactly none (main.c:952) -- so
//      sending each one the OTHER's payload must raise fatal error 51,
//      ERROR_COMMAND_SIZE_WRONG. That distinguishes 40 from 41 positively:
//      if the two numbers were ever swapped, this section fails.
//
//   2. THE PARAMETER DOMAIN. All 256 values of the level byte, not a sample.
//
//   3. THAT THEY EXECUTE IMMEDIATELY rather than queueing, which is the
//      property that makes them usable as indicators at all.
//
//   4. THAT NEITHER DISTURBS A MOVE ALREADY RUNNING, measured the only way that
//      really settles it: against an undisturbed reference move, the technique
//      tm_readonly_purity uses for the getters.
//
//   5. THE QUEUE-EMPTY PRECONDITION, checked where it actually lives. There is
//      a standing note that turning vibration OFF can raise
//      ERROR_QUEUE_NOT_EMPTY. That is true, and it is worse than it sounds --
//      but only on M1. motor_control.c:1222 tests `n_items_in_queue != 0`
//      BEFORE the `vibration_level == 0` turn-off branch at :1226, so on an M1
//      even vibrate(0) faults while moves are queued. On every other product
//      the function is empty and no such precondition exists. This module
//      asserts whichever of the two is correct for the product it finds.
//
// HOW THIS DIFFERS FROM THE TWO EXISTING MODULES ON THESE COMMANDS.
//   * tm_actuators.cpp is the happy path: one on/off cycle, one identify, and a
//     ping. It asserts nothing about numbers, ranges, queueing or interference.
//   * tm_actuator_edges.cpp sweeps NINE level values and sends identify during a
//     move -- but with the MOSFETs ENABLED and the shaft actually turning, and
//     it compares the endpoint against a computed target rather than against an
//     undisturbed run of the same move. It states outright that fatal error 51
//     is "out of reach" for the typed API.
//   This module keeps the MOSFETs OFF throughout, sweeps all 256 levels, reaches
//   error 51 on both commands (via Servomotor::sendCommand, the public raw
//   sender), verifies both command NUMBERS on the wire, and uses a reference
//   move rather than a predicted one. No assertion here is a restatement of one
//   in those two files.
//
// SAFETY.
//   The MOSFETs are never enabled: enableMosfets() and goToClosedLoop() are
//   never called, so the output stage stays dead and the shaft cannot turn. Every
//   distance measured is the COMMANDED position, which the planner advances
//   whether or not the coils are energised, so the moves here are arithmetic and
//   not motion. Nothing is broadcast, no alias is ever set, and every frame --
//   including the hand-addressed raw ones -- is emitted through the unique-ID
//   addressed motor object, so the other 34 motors on the shared rack cannot
//   match any of them. Test mode (command 36) is never sent, so the 10..13 hang
//   is unreachable. On a product where vibrate produces REAL motion (M1) the
//   module stops after the sections that are provably motionless there, rather
//   than shaking a machine nobody is watching.
// ---------------------------------------------------------------------------

// Error and status constants, all verified against the sources named.
static const uint8_t ERR_COMMAND_SIZE_WRONG = 51;  // error_codes.json code 51
static const uint8_t ERR_QUEUE_NOT_EMPTY    = 8;   // error_codes.json code 8
static const uint16_t FLAG_MOSFETS_ENABLED  = (1u << 1);  // device_status.h bit 1
static const uint16_t FLAG_MOTOR_BUSY       = (1u << 6);  // device_status.h bit 6

// The command numbers under test. Cross-checked in THREE places:
//   python_programs/servomotor/motor_commands.json  -> 40 Vibrate, 41 Identify
//   firmware/Src/commands.h:44-45                   -> VIBRATE_COMMAND 40,
//                                                      IDENTIFY_COMMAND 41
//   Arduino_library/Commands.h:50-51                -> VIBRATE = 40, IDENTIFY = 41
static const uint8_t CMD_VIBRATE  = 40;
static const uint8_t CMD_IDENTIFY = 41;

// Bounded RX drain. Never unbounded: a stuck line must not eat the test slot.
static void drainRx(uint32_t maxMs) {
    uint32_t t0 = millis();
    while (Serial1.available() && (millis() - t0) < maxMs) Serial1.read();
}

// ---------------------------------------------------------------------------
// Send a hand-chosen command byte with a hand-chosen payload and capture the
// raw reply frame.
//
// Servomotor::sendCommand() emits exactly the frame the typed methods emit
// (size | 254 | 8-byte unique ID little-endian | command | payload | CRC32) but
// lets us choose the command byte and the payload length independently, which
// is what makes the payload-size discrimination in section 2 possible. It does
// NOT consume the reply, so we read the frame off the wire ourselves and then
// leave the buffer empty for the next typed call.
//
// Returns the number of reply bytes STORED in `reply`: 0 means the device
// answered nothing. The count is capped at replyMax deliberately -- the frame
// recognisers below scan the buffer, and handing them a length larger than the
// buffer would read past the end of it. If the device ever sent more than
// replyMax bytes the surplus is reported on the console and discarded.
// ---------------------------------------------------------------------------
static int rawProbe(Servomotor* m, uint8_t cmd, const uint8_t* payload,
                    uint16_t payloadSize, uint8_t* reply, int replyMax) {
    drainRx(300);
    m->sendCommand(cmd, payload, payloadSize);
    int n = 0;
    uint32_t lastByteAt = millis();
    uint32_t deadline = millis() + 400;          // generous; a real reply is ~1 ms
    while (millis() < deadline) {
        if (Serial1.available()) {
            int b = Serial1.read();
            if (n < replyMax) reply[n] = (uint8_t)b;
            n++;
            lastByteAt = millis();
        }
        else if (n > 0 && (millis() - lastByteAt) > 40) {
            break;                                // the frame is complete
        }
    }
    drainRx(100);
    if (n > replyMax) {
        printf("   (reply was %d bytes; only the first %d were kept)\n", n, replyMax);
        n = replyMax;
    }
    return n;
}

// ---------------------------------------------------------------------------
// Recognise the two reply frames the firmware can send for these commands.
//
// A "no error" reply is the precomputed constant NO_ERROR_RESPONSE_CRC32_ENABLED
// -- 6 bytes, "\x0d\xFD" plus 4 CRC bytes (common_source_files/RS485.h:23) -- or
// NO_ERROR_RESPONSE_CRC32_DISABLED, 2 bytes, "\x05\xFC" (:24). An error reply is
// {size, response_character, error_code, crc32} = 7 bytes, or 3 without the CRC
// (RS485.c:549-577); byte [2] carries the code in BOTH framings. The leading
// size byte is encode_first_byte(total) = (total << 1) | 1, so the four possible
// frames start with 0x0d, 0x05, 0x0f and 0x07 respectively -- which is what
// tells a success frame apart from an error frame in the same framing.
//
// Both framings are accepted deliberately. Which one is in use is the subject of
// tm_crc32_matrix, not of this module, and hard-coding one would make these
// assertions fail for a reason that has nothing to do with the command numbers
// they are testing.
//
// WEAKENED DELIBERATELY (adversarial review, 2026-08). These used to demand that
// the capture be EXACTLY 6/2 or 7/3 bytes long. The firmware promises no such
// thing about the TOTAL number of bytes standing on the wire at that moment: a
// straggler from an earlier transaction that the bounded drain did not yet see,
// or a second frame emitted by the fatal-error loop once its transmitter goes
// idle (error_handling.c handle_packet, default case), would change the COUNT
// without changing the fact under test -- and a healthy machine would fail. What
// section 1 needs to establish is only that a frame carrying this response
// character (and, for an error, this code) came back, so the capture is SEARCHED
// for that signature instead of being length-matched. There is no ambiguity to
// lose: the no-error frame's fixed CRC (13 e2 7b 37) contains neither error
// signature, and no error frame for any code 0..255 contains either success
// signature. The raw byte count is still printed, so a desynchronized bus stays
// diagnosable from the log.
// ---------------------------------------------------------------------------
static bool findFrame(const uint8_t* r, int n, uint8_t sizeByte,
                      uint8_t responseChar, bool wantCode, uint8_t code) {
    const int need = wantCode ? 3 : 2;
    for (int i = 0; i + need <= n; i++) {
        if (r[i] != sizeByte) continue;
        if (r[i + 1] != responseChar) continue;
        if (wantCode && r[i + 2] != code) continue;
        return true;
    }
    return false;
}

static bool isSuccessFrame(const uint8_t* r, int n) {
    return findFrame(r, n, 0x0d, 253, false, 0) ||   // 6 bytes, CRC32 framing
           findFrame(r, n, 0x05, 252, false, 0);     // 2 bytes, no CRC32
}

static bool isErrorFrame(const uint8_t* r, int n, uint8_t code) {
    return findFrame(r, n, 0x0f, 253, true, code) || // 7 bytes, CRC32 framing
           findFrame(r, n, 0x07, 252, true, code);   // 3 bytes, no CRC32
}

// The status flags word; false if the read itself failed.
static bool flagsOf(Servomotor* m, uint16_t* out) {
    getStatusResponse st = m->getStatus();
    if (m->getError() != 0) return false;
    *out = st.statusFlags;
    return true;
}

// A ping whose 10-byte payload must come back unchanged: proves the device is
// alive AND that request/reply pairing on the bus is still in step.
static bool pingRoundTrips(Servomotor* m) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 13 + 7);
    pingResponse r = m->ping(payload);
    if (m->getError() != 0) {
        printf("   ping comms error %d\n", m->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

// True if the multipurpose buffer is EMPTY, i.e. the firmware replies with the
// single data-type byte 0 (main.c:792-802). On an M1 a vibration overwrites
// this buffer with data type 3, MULTIPURPOSE_DATA_TYPE_VIBRATE
// (motor_control.c:1245, motor_control.h:144); on every other product nothing
// should ever write it.
static bool multipurposeBufferEmpty(Servomotor* m, uint16_t* lenOut, uint8_t* typeOut) {
    uint8_t buf[64];
    uint16_t len = 0;
    m->readMultipurposeBuffer(buf, sizeof(buf), &len);
    *lenOut = len;
    *typeOut = (len > 0) ? buf[0] : 0xFF;
    return (m->getError() == 0) && (len == 1) && (buf[0] == 0);
}

void tm_indicator_commands(void) {
    Serial.println("tm_indicator_commands: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 0. WHO WE ARE TALKING TO, AND WHETHER IT CAN HEAR US ALONE.
    //
    //    This module hand-addresses frames in section 2. If the motor object
    //    were alias-addressed, every one of those frames would be heard by all
    //    35 motors on the shared rack (they all carry alias 88). Establish
    //    unique-ID addressing before sending anything, and bail out rather
    //    than risk it.
    //
    //    The product matters too: vibrate() is a real, motion-producing
    //    routine on M1 and an empty function everywhere else, so the correct
    //    expected result for half this module depends on which one is on the
    //    other end of the wire.
    // =====================================================================
    printf("\n--- 0. the machine under test ---\n");

    bool extended = m->isUsingExtendedAddressing();
    TEST_RESULT("the motor is addressed by its unique ID, so no other motor on "
                "the bus can hear this module", extended);
    if (!extended) {
        printf("   ABORT: not unique-ID addressed; refusing to send anything.\n");
        return;
    }

    tfhReset(m);
    uint8_t startFatal = tfhFatal(m);
    printf("   fatal code after reset: %u\n", (unsigned)startFatal);
    TEST_RESULT("the machine starts with no latched fault", startFatal == 0);

    getProductInfoResponse info = m->getProductInfo();
    bool infoOk = (m->getError() == 0);
    char code[9];
    memcpy(code, info.productCode, 8);
    code[8] = '\0';
    // "M1" proper, not "M17": the third character of M17 is a digit.
    bool m1Real = infoOk && code[0] == 'M' && code[1] == '1' &&
                  !(code[2] >= '0' && code[2] <= '9');
    bool vibrateIsNoOp = infoOk && !m1Real;
    printf("   product code \"%s\": vibrate is %s here\n",
           code, vibrateIsNoOp ? "an empty function (no motion possible)"
                               : "a REAL motion-producing routine");
    TEST_RESULT("the product identifies itself, so the right vibrate behaviour "
                "can be expected of it", infoOk);
    tfhReset(m);

    // =====================================================================
    // 1. THE TWO COMMAND NUMBERS ARE WHAT THE DOCUMENTATION SAYS.
    //
    //    Command numbers are the one part of an API that cannot be checked by
    //    reading the reply, because a wrong number usually still produces a
    //    plausible-looking result -- or, here, no result at all. Three things
    //    are established, in increasing strength:
    //
    //      (a) the library's own constants are 40 and 41;
    //      (b) both bytes are IMPLEMENTED in the firmware -- an unimplemented
    //          byte hits main.c:1146 and answers nothing whatsoever, so a
    //          reply is proof of implementation (this is the same silent-drop
    //          behaviour tm_bad_command_byte characterises for bytes 48..255);
    //      (c) each byte enforces ITS OWN payload contract, which tells the
    //          two apart. Command 40 copies exactly one byte
    //          (main.c:937 -> copy_input_parameters_and_check_size, :194) and
    //          command 41 demands exactly zero (main.c:952 ->
    //          check_payload_size_is_zero, :202). Feed each the other's
    //          payload and both must raise fatal 51.
    //
    //    (c) is the assertion that would fail if the numbers were ever
    //    transposed, which no other test in the suite would notice.
    // =====================================================================
    printf("\n--- 1. command 40 is Vibrate and command 41 is Identify ---\n");
    {
        TEST_RESULT("the library sends Vibrate as command number 40",
                    (int)VIBRATE == (int)CMD_VIBRATE && (int)CMD_VIBRATE == 40);
        TEST_RESULT("the library sends Identify as command number 41",
                    (int)IDENTIFY == (int)CMD_IDENTIFY && (int)CMD_IDENTIFY == 41);

        // Frame lengths are 6/2 (success) and 7/3 (error) depending on whether
        // CRC32 framing is in use; isSuccessFrame/isErrorFrame accept either.
        // The library's own view is printed so a mismatch is diagnosable.
        bool crcOn = m->isCRC32Enabled();
        uint8_t reply[16] = {0};

        // (b) Both bytes answer when given the payload they expect. An
        // unimplemented byte would produce ZERO reply bytes here.
        uint8_t zeroLevel = 0;            // vibrate(0): the off value, safe anywhere
        int n40 = rawProbe(m, CMD_VIBRATE, &zeroLevel, 1, reply, sizeof(reply));
        printf("   cmd 40 + 1-byte payload: %d reply bytes, indicator 0x%02X "
               "(library crc32=%d)\n",
               n40, n40 > 1 ? reply[1] : 0, (int)crcOn);
        TEST_RESULT("command number 40 is implemented and accepts a one-byte payload",
                    isSuccessFrame(reply, n40));

        int n41 = rawProbe(m, CMD_IDENTIFY, nullptr, 0, reply, sizeof(reply));
        printf("   cmd 41 + empty payload: %d reply bytes, indicator 0x%02X\n",
               n41, n41 > 1 ? reply[1] : 0);
        TEST_RESULT("command number 41 is implemented and accepts an empty payload",
                    isSuccessFrame(reply, n41));

        // (c) Each byte refuses the other's payload. Fatal 51 latches; the
        // faulted device still answers Get status, which is the one command
        // that keeps working after a fault.
        tfhReset(m);
        int nBad40 = rawProbe(m, CMD_VIBRATE, nullptr, 0, reply, sizeof(reply));
        uint8_t wire40 = (nBad40 >= 3) ? reply[2] : 0xFF;
        uint8_t latched40 = tfhFatal(m);
        printf("   cmd 40 + EMPTY payload: %d reply bytes, error byte %u, "
               "latched %u (expect %u)\n",
               nBad40, (unsigned)wire40, (unsigned)latched40,
               (unsigned)ERR_COMMAND_SIZE_WRONG);
        TEST_RESULT("command number 40 refuses an empty payload with "
                    "ERROR_COMMAND_SIZE_WRONG (51), so 40 is the one that takes "
                    "a parameter",
                    isErrorFrame(reply, nBad40, ERR_COMMAND_SIZE_WRONG) &&
                    latched40 == ERR_COMMAND_SIZE_WRONG);

        tfhReset(m);
        uint8_t junk = 1;
        int nBad41 = rawProbe(m, CMD_IDENTIFY, &junk, 1, reply, sizeof(reply));
        uint8_t wire41 = (nBad41 >= 3) ? reply[2] : 0xFF;
        uint8_t latched41 = tfhFatal(m);
        printf("   cmd 41 + 1-byte payload: %d reply bytes, error byte %u, "
               "latched %u (expect %u)\n",
               nBad41, (unsigned)wire41, (unsigned)latched41,
               (unsigned)ERR_COMMAND_SIZE_WRONG);
        TEST_RESULT("command number 41 refuses a one-byte payload with "
                    "ERROR_COMMAND_SIZE_WRONG (51), so 41 is the one that takes "
                    "none",
                    isErrorFrame(reply, nBad41, ERR_COMMAND_SIZE_WRONG) &&
                    latched41 == ERR_COMMAND_SIZE_WRONG);

        tfhReset(m);
        TEST_RESULT("a reset clears both size faults and the machine answers again",
                    tfhFatal(m) == 0 && pingRoundTrips(m));
        tfhReset(m);
    }

    // =====================================================================
    // 2. THE QUEUE-EMPTY PRECONDITION, ASSERTED WHERE IT ACTUALLY LIVES.
    //
    //    There is a standing note that turning vibration OFF can raise
    //    ERROR_QUEUE_NOT_EMPTY, which sounds like nonsense -- why would
    //    STOPPING something need an empty queue? The firmware explains it:
    //
    //        void vibrate(uint8_t vibration_level) {          // :1219
    //        #ifdef PRODUCT_NAME_M1
    //            if(n_items_in_queue != 0) {
    //                fatal_error(ERROR_QUEUE_NOT_EMPTY);      // :1222-1224
    //            }
    //            if(vibration_level == 0) { ... return; }     // :1226-1231
    //
    //    The guard precedes the turn-off branch, so on an M1 vibrate(0) really
    //    does fault while moves are queued. And the whole body is inside the
    //    `#ifdef`, so on M2/M17/M23 the function is empty and there is NO
    //    precondition at all. Both are legitimate; what matters is that the
    //    machine does the one its own firmware specifies, so that is what is
    //    asserted -- with the expectation chosen from the product code read in
    //    section 0.
    //
    //    Safe on either product. On an M1 the guard fires BEFORE
    //    check_current_sensor_and_enable_mosfets() at :1239, so the output
    //    stage is never energised; and on an M1 we send only the OFF value, so
    //    even if the queue drained underneath us nothing could start vibrating.
    // =====================================================================
    printf("\n--- 2. vibrate with moves still in the queue ---\n");
    {
        tfhFreshOpen(m);
        const int32_t dist = (int32_t)(TFH_CPR / 4);
        const uint32_t dur = 3 * TFH_TPS;          // long enough to stay queued
        const uint8_t expectFatal = m1Real ? ERR_QUEUE_NOT_EMPTY : 0;
        // On a no-op product exercise the interesting ON value; on an M1 send
        // only OFF, which is the documented quirk and cannot start motion.
        const uint8_t onLevel = vibrateIsNoOp ? 1 : 0;

        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(dist, dur);
        bool queued1 = (m->getError() == 0);
        m->trapezoidMoveRaw(dist, dur);
        bool queued2 = (m->getError() == 0);
        uint8_t depth = m->getNQueuedItems();

        m->vibrate(onLevel);
        bool onComms = (m->getError() == 0);
        uint8_t onFatal = tfhFatal(m);
        printf("   queue depth %u, vibrate(%u): comms ok=%d fatal=%u (expect %u)\n",
               (unsigned)depth, (unsigned)onLevel, (int)onComms,
               (unsigned)onFatal, (unsigned)expectFatal);
        TEST_RESULT("vibrate sent with moves still queued behaves exactly as this "
                    "firmware's vibrate() specifies",
                    queued1 && queued2 && depth > 0 && onFatal == expectFatal);

        m->vibrate(0);
        uint8_t offFatal = tfhFatal(m);
        printf("   vibrate(0) with the queue still loaded: fatal=%u (expect %u)\n",
               (unsigned)offFatal, (unsigned)expectFatal);
        TEST_RESULT("turning vibration off with moves still queued behaves the "
                    "same way", offFatal == expectFatal);

        bool drained = (expectFatal == 0) && tfhDrain(m, 20000);
        uint8_t endFatal = tfhFatal(m);
        int64_t moved = (drained && endFatal == 0) ? (m->getPositionRaw() - before) : 0;
        int64_t want = 2 * (int64_t)dist;
        printf("   after the queue drained: fatal=%u moved %lld of %lld\n",
               (unsigned)endFatal, (long long)moved, (long long)want);
        TEST_RESULT("the moves that were queued are unharmed exactly when vibrate "
                    "raised no fault",
                    (expectFatal == 0)
                        ? (drained && endFatal == 0 && llabs((long long)(moved - want)) <= 8)
                        : (endFatal == ERR_QUEUE_NOT_EMPTY));
        tfhReset(m);
    }

    // =====================================================================
    //    STOP HERE ON A PRODUCT WHERE VIBRATE REALLY VIBRATES.
    //
    //    Everything below sends nonzero vibration levels at a machine whose
    //    queue is empty. On an M1 that enables the MOSFETs (motor_control.c:1239)
    //    and drives the coils to +/-50 PWM, which is real motion on a rack
    //    nobody is watching. Every section above was chosen to be motionless on
    //    BOTH products; the rest are not, so on an M1 the module stops here.
    //    Sections 0-2 contribute 13 assertions on any product; the remainder
    //    contribute 25 more on M2/M17/M23.
    // =====================================================================
    if (!vibrateIsNoOp) {
        printf("\n   vibrate produces REAL motion on this product; stopping before "
               "the sections that would shake it.\n");
        TEST_RESULT("the motion-producing sections are skipped on a product where "
                    "vibrate really vibrates", true);
        tfhReset(m);
        return;
    }

    // =====================================================================
    // 3. EVERY VALUE OF THE LEVEL BYTE IS IN RANGE -- ALL 256 OF THEM.
    //
    //    The parameter is documented as "0 turns vibration off; any nonzero
    //    value turns it on ... only tested for zero versus nonzero"
    //    (motor_commands.json, command 40), and the amplitude is the fixed
    //    constant VIBRATE_MAGNITUDE = 50 (motor_control.c:1211). So the
    //    documented domain is the WHOLE byte and there is no out-of-range
    //    value to refuse. That is a claim worth pinning down rather than
    //    assuming: if a future firmware added a range check, the API would
    //    start faulting on values callers are entitled to send.
    //
    //    The only refusable thing about this command is its payload SIZE, and
    //    that was already established in section 1 -- which is where the
    //    "out of range is rejected" half of the contract really lives.
    //
    //    Sweeping all 256 rather than a sample also proves the no-op is a
    //    genuine no-op 256 times over: nothing queued, nothing energised,
    //    nothing moved, nothing written.
    // =====================================================================
    printf("\n--- 3. all 256 vibration-level values ---\n");
    {
        tfhReset(m);
        int64_t posBefore = m->getPositionRaw();
        bool posBeforeOk = (m->getError() == 0);

        int accepted = 0;
        bool allAccepted = true;
        bool anyFault = false;
        for (int v = 0; v <= 255; v++) {
            m->vibrate((uint8_t)v);
            if (m->getError() != 0) {
                printf("   level %d: comms error %d\n", v, m->getError());
                allAccepted = false;
                break;
            }
            accepted++;
            m->vibrate(0);                       // leave it off after every value
            if (m->getError() != 0) {
                printf("   level %d: the following off command failed (%d)\n",
                       v, m->getError());
                allAccepted = false;
                break;
            }
            if ((v & 31) == 31) {                // spot-check the fault latch
                if (tfhFatal(m) != 0) { anyFault = true; break; }
            }
        }
        uint8_t sweepFatal = tfhFatal(m);
        printf("   %d of 256 levels accepted; fatal after the sweep = %u\n",
               accepted, (unsigned)sweepFatal);
        TEST_RESULT("every one of the 256 vibration-level values is accepted",
                    allAccepted && accepted == 256);
        TEST_RESULT("the whole level sweep latches no fault",
                    !anyFault && sweepFatal == 0);

        uint8_t depth = m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);
        printf("   queue depth after the sweep: %u\n", (unsigned)depth);
        TEST_RESULT("the level sweep occupies no queue slot", depthOk && depth == 0);

        uint16_t flags = 0xFFFF;
        bool flagsOk = flagsOf(m, &flags);
        printf("   status flags after the sweep: 0x%04X\n", (unsigned)flags);
        TEST_RESULT("the level sweep leaves the output stage off and the motor "
                    "not busy",
                    flagsOk && (flags & FLAG_MOSFETS_ENABLED) == 0 &&
                    (flags & FLAG_MOTOR_BUSY) == 0);

        int64_t posAfter = m->getPositionRaw();
        bool posAfterOk = (m->getError() == 0);
        printf("   commanded position across the sweep: %lld -> %lld\n",
               (long long)posBefore, (long long)posAfter);
        TEST_RESULT("the level sweep leaves the commanded position exactly where "
                    "it was",
                    posBeforeOk && posAfterOk && posBefore == posAfter);

        // On an M1 a vibration overwrites the shared buffer with data type 3.
        // Here nothing should have written it, so it must still report empty.
        uint16_t mpLen = 0; uint8_t mpType = 0xFF;
        bool mpEmpty = multipurposeBufferEmpty(m, &mpLen, &mpType);
        printf("   multipurpose buffer after the sweep: len=%u type=%u\n",
               (unsigned)mpLen, (unsigned)mpType);
        TEST_RESULT("the level sweep writes nothing into the multipurpose buffer",
                    mpEmpty);
        tfhReset(m);
    }

    // =====================================================================
    // 4. BOTH COMMANDS EXECUTE IMMEDIATELY; NEITHER EVER QUEUES.
    //
    //    This is the property that makes an indicator usable at all. If either
    //    command took a slot in the 32-entry motion queue, then identifying a
    //    motor during a long queued sequence would either be delayed until the
    //    sequence finished -- useless, since the point is to find the motor NOW
    //    -- or would fill the queue and cause a later legitimate move to be
    //    refused for no visible reason. The firmware runs both inline in the
    //    command handler (main.c:939, :955-956), never through add_to_queue().
    //
    //    Checked twice: on an idle machine the depth must stay at zero, and
    //    with a chain of moves running the depth must never GROW across either
    //    command. Depth is sampled either side of each command; it may legally
    //    fall as the chain executes, but an increase would mean a slot was
    //    taken.
    // =====================================================================
    printf("\n--- 4. neither command ever occupies a queue slot ---\n");
    {
        tfhReset(m);
        bool vibOk = true;
        for (int i = 0; i < 20 && vibOk; i++) {
            m->vibrate((uint8_t)(i % 2));
            if (m->getError() != 0) vibOk = false;
        }
        m->vibrate(0);                           // never leave a level latched on
        uint8_t dVib = m->getNQueuedItems();
        bool dVibOk = (m->getError() == 0);
        printf("   after 20 vibrate commands: queue depth %u, fatal %u\n",
               (unsigned)dVib, (unsigned)tfhFatal(m));
        TEST_RESULT("twenty vibrate commands on an idle machine occupy no queue slot",
                    vibOk && dVibOk && dVib == 0);

        bool idOk = true;
        for (int i = 0; i < 20 && idOk; i++) {
            m->identify();
            if (m->getError() != 0) idOk = false;
        }
        uint8_t dId = m->getNQueuedItems();
        bool dIdOk = (m->getError() == 0);
        printf("   after 20 identify commands: queue depth %u, fatal %u\n",
               (unsigned)dId, (unsigned)tfhFatal(m));
        TEST_RESULT("twenty identify commands on an idle machine occupy no queue slot",
                    idOk && dIdOk && dId == 0);

        // Now with work in the queue. Three chained moves; alternate the two
        // commands and require the depth never to rise across one.
        tfhFreshOpen(m);
        const int32_t leg = (int32_t)(TFH_CPR / 4);
        const uint32_t legDur = 2 * TFH_TPS;
        int64_t before = m->getPositionRaw();
        bool queued = true;
        for (int i = 0; i < 3; i++) {
            m->trapezoidMoveRaw(leg, legDur);
            if (m->getError() != 0) queued = false;
        }
        int probes = 0;
        bool neverGrew = true;
        uint32_t deadline = millis() + 5000;
        while (millis() < deadline && queued) {
            uint8_t d0 = m->getNQueuedItems();
            if (m->getError() != 0) { neverGrew = false; break; }
            if (d0 == 0) break;
            if (probes & 1) m->identify(); else m->vibrate((uint8_t)(probes & 7));
            if (m->getError() != 0) { neverGrew = false; break; }
            uint8_t d1 = m->getNQueuedItems();
            if (m->getError() != 0) { neverGrew = false; break; }
            if (d1 > d0) {
                printf("   queue GREW from %u to %u across a %s command\n",
                       (unsigned)d0, (unsigned)d1, (probes & 1) ? "identify" : "vibrate");
                neverGrew = false;
                break;
            }
            probes++;
        }
        m->vibrate(0);                           // never leave a level latched on
        bool drained = queued && tfhDrain(m, 20000);
        uint8_t fat = tfhFatal(m);
        int64_t moved = (drained && fat == 0) ? (m->getPositionRaw() - before) : 0;
        int64_t want = 3 * (int64_t)leg;
        printf("   %d probes during the chain; moved %lld of %lld, fatal %u\n",
               probes, (long long)moved, (long long)want, (unsigned)fat);
        TEST_RESULT("neither command ever increases the queue depth while moves "
                    "are queued",
                    queued && neverGrew && probes > 5);
        TEST_RESULT("and the queued chain still delivers its full displacement",
                    drained && fat == 0 && llabs((long long)(moved - want)) <= 8);
        tfhReset(m);
    }

    // =====================================================================
    // 5. NEITHER COMMAND DISTURBS A MOVE THAT IS ALREADY RUNNING.
    //
    //    The strongest thing that can be said about a command with no output:
    //    run the SAME move twice, once undisturbed and once while hammering the
    //    bus with the command under test, and require the same endpoint to the
    //    count. That is the technique tm_readonly_purity uses for the getters,
    //    and it is the only way to catch a command that perturbs the planner
    //    without reporting anything.
    //
    //    There is a specific reason to suspect identify in particular. Its
    //    handler switches off the SysTick interrupt to update the flash
    //    counters atomically and switches it back on (main.c:954-957). SysTick
    //    is a 100 Hz timer that also decrements the detect-devices delays, and
    //    a regression that left it disabled, or that blocked long enough inside
    //    the critical section, would be invisible in every existing test and
    //    would show up here as a move landing somewhere else. Motion itself is
    //    driven by TIM16 at 31250 Hz, not SysTick, so a correct implementation
    //    must land in exactly the same place.
    // =====================================================================
    printf("\n--- 5. an undisturbed move versus a hammered one ---\n");
    {
        const int32_t dist = (int32_t)(TFH_CPR / 2);
        const uint32_t dur = 2 * TFH_TPS;

        // The reference: the same move three times, so the comparison is
        // against a measured-stable baseline rather than one sample.
        int64_t reference = 0, lo = 0, hi = 0;
        bool refOk = true;
        for (int r = 0; r < 3; r++) {
            tfhFreshOpen(m);
            bool ok = false;
            int64_t moved = tfhMove(m, dist, dur, &ok);
            if (!ok) { refOk = false; break; }
            if (r == 0) { lo = hi = moved; }
            else { if (moved < lo) lo = moved; if (moved > hi) hi = moved; }
            reference = moved;
        }
        printf("   undisturbed reference: %lld counts, spread %lld\n",
               (long long)reference, (long long)(hi - lo));
        TEST_RESULT("the undisturbed reference move is repeatable",
                    refOk && (hi - lo) <= 2);

        // Hammered with identify.
        tfhFreshOpen(m);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(dist, dur);
        bool queued = (m->getError() == 0);
        int idCalls = 0;
        bool idAllOk = true;
        uint32_t deadline = millis() + 2200;
        while (millis() < deadline) {
            m->identify();
            if (m->getError() != 0) { idAllOk = false; break; }
            idCalls++;
        }
        bool idDrained = queued && tfhDrain(m, 15000);
        uint8_t idFat = tfhFatal(m);
        int64_t idMoved = (idDrained && idFat == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   %d identify commands during the move: fatal=%u moved %lld "
               "vs reference %lld\n",
               idCalls, (unsigned)idFat, (long long)idMoved, (long long)reference);
        TEST_RESULT("identify can be sent continuously throughout a move",
                    idAllOk && idCalls > 20);
        TEST_RESULT("identify does not change where the move lands",
                    refOk && idDrained && idFat == 0 &&
                    llabs((long long)(idMoved - reference)) <= 8);

        // Hammered with vibrate, alternating on and off.
        tfhFreshOpen(m);
        before = m->getPositionRaw();
        m->trapezoidMoveRaw(dist, dur);
        queued = (m->getError() == 0);
        int vibCalls = 0;
        bool vibAllOk = true;
        deadline = millis() + 2200;
        while (millis() < deadline) {
            m->vibrate((uint8_t)(vibCalls & 1));
            if (m->getError() != 0) { vibAllOk = false; break; }
            vibCalls++;
        }
        m->vibrate(0);                            // leave it definitively off
        bool vibDrained = queued && tfhDrain(m, 15000);
        uint8_t vibFat = tfhFatal(m);
        int64_t vibMoved = (vibDrained && vibFat == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   %d vibrate commands during the move: fatal=%u moved %lld "
               "vs reference %lld\n",
               vibCalls, (unsigned)vibFat, (long long)vibMoved, (long long)reference);
        TEST_RESULT("vibrate can be cycled continuously throughout a move",
                    vibAllOk && vibCalls > 20);
        TEST_RESULT("vibrate does not change where the move lands",
                    refOk && vibDrained && vibFat == 0 &&
                    llabs((long long)(vibMoved - reference)) <= 8);
        tfhReset(m);
    }

    // =====================================================================
    // 6. IDENTIFY IS IDEMPOTENT AND REPEATABLE.
    //
    //    Identify is the command an operator sends when they have 35 identical
    //    motors on one bus and need to know which is which. In practice it gets
    //    pressed repeatedly and impatiently, well inside the ~2.4 s flash
    //    window (30 flashes of 8 SysTick ticks each, main.c:148-159), so
    //    re-issuing while already flashing is the NORMAL case, not an edge
    //    case. The handler simply rewrites green_led_action_counter and
    //    n_identify_flashes (main.c:955-956), so every repeat must be accepted
    //    and must leave the machine bit-for-bit as it was.
    //
    //    The status word is compared as a WHOLE 16-bit value rather than
    //    flag-by-flag: that catches a bit being disturbed that nobody thought
    //    to name.
    // =====================================================================
    printf("\n--- 6. identify, thirty times over ---\n");
    {
        tfhReset(m);
        uint16_t flagsBefore = 0xFFFF, flagsAfter = 0xFFFE;
        bool fbOk = flagsOf(m, &flagsBefore);
        int64_t posBefore = m->getPositionRaw();
        bool posBeforeOk = (m->getError() == 0);

        int calls = 0;
        bool allOk = true;
        for (int i = 0; i < 30; i++) {
            m->identify();
            if (m->getError() != 0) { allOk = false; break; }
            calls++;
            // The first few land back-to-back, the rest are spread across the
            // flash window so both the re-arm and the natural-expiry paths are
            // exercised.
            if (i >= 5) delay(100);
        }
        bool faOk = flagsOf(m, &flagsAfter);
        int64_t posAfter = m->getPositionRaw();
        bool posAfterOk = (m->getError() == 0);
        uint8_t depth = m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);
        printf("   %d identify commands accepted; flags 0x%04X -> 0x%04X; "
               "position %lld -> %lld; queue %u\n",
               calls, (unsigned)flagsBefore, (unsigned)flagsAfter,
               (long long)posBefore, (long long)posAfter, (unsigned)depth);
        TEST_RESULT("thirty identify commands in a row are all accepted",
                    allOk && calls == 30);
        TEST_RESULT("identify leaves the status word bit-for-bit identical",
                    fbOk && faOk && flagsBefore == flagsAfter);
        TEST_RESULT("identify never moves the commanded position",
                    posBeforeOk && posAfterOk && posBefore == posAfter);
        TEST_RESULT("identify never occupies a queue slot, however many times it "
                    "is sent", depthOk && depth == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 7. THE MACHINE IS STILL A MACHINE AFTERWARDS.
    //
    //    An indicator command that left the motor subtly unusable would be the
    //    worst possible failure: the operator would press it precisely when
    //    they were already confused about which motor was which, and would then
    //    be debugging two problems. So the last thing checked is that after
    //    everything above -- and after a further burst of the two commands
    //    interleaved, which is what a nervous operator actually produces -- a
    //    real move still works and still delivers its full displacement.
    // =====================================================================
    printf("\n--- 7. still usable after all of it ---\n");
    {
        tfhReset(m);
        bool cyclesOk = true;
        for (int i = 0; i < 5 && cyclesOk; i++) {
            m->vibrate(1);
            if (m->getError() != 0) { cyclesOk = false; break; }
            delay(50);
            m->vibrate(0);
            if (m->getError() != 0) cyclesOk = false;
        }
        tfhOpenCeilings(m);
        bool moveOk = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 2), 2 * TFH_TPS, &moveOk);
        printf("   after five on/off cycles: move ok=%d moved %lld of %lld\n",
               (int)moveOk, (long long)moved, (long long)(TFH_CPR / 2));
        TEST_RESULT("five vibrate on/off cycles leave the machine able to move again",
                    cyclesOk && moveOk);
        TEST_RESULT("and the move after them delivers its full displacement",
                    moveOk && llabs((long long)(moved - TFH_CPR / 2)) <= 8);

        tfhReset(m);
        int burst = 0;
        bool burstOk = true;
        for (int i = 0; i < 50; i++) {
            if (i & 1) m->identify(); else m->vibrate((uint8_t)(i * 5 + 1));
            if (m->getError() != 0) { burstOk = false; break; }
            burst++;
        }
        m->vibrate(0);
        uint8_t burstFatal = tfhFatal(m);
        printf("   %d interleaved commands: fatal=%u\n", burst, (unsigned)burstFatal);
        TEST_RESULT("fifty interleaved vibrate and identify commands are all accepted",
                    burstOk && burst == 50);
        TEST_RESULT("the interleaved burst latches no fault", burstFatal == 0);
        TEST_RESULT("the machine still round-trips a ping after everything",
                    pingRoundTrips(m));
        tfhReset(m);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
