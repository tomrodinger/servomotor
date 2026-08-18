#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_communication_statistics.cpp
//
// GET COMMUNICATION STATISTICS (command 47) TREATED AS A STATEFUL PRIMITIVE,
// NOT AS A GETTER.
//
// Command 47 (verified against python_programs/servomotor/motor_commands.json,
// "Get communication statistics"; firmware/Src/commands.h:52;
// handler firmware/Src/main.c:1128-1145) is the only window a host has into
// what the RS485 receiver silently threw away. A packet that fails CRC32
// validation gets NO error reply -- it is dropped and the host sees only a
// timeout (firmware/Src/main.c:226-230). The counters are the difference
// between "the motor is broken" and "the cable is noisy", so if they lie, every
// field diagnosis built on them is wrong.
//
// It also takes an argument -- resetCounter -- which makes it the one "read"
// in the API that MUTATES the thing it reads. The firmware snapshots all six
// counters into a local struct and only then, if the flag is nonzero, zeroes
// them (common_source_files/RS485.c:179-199). So the reply to a resetting call
// carries the PRE-reset values, and nothing that happened before the call can
// be lost between the read and the clear. That contract -- snapshot first,
// clear second, atomically, all six together -- is what this module pins down.
//
// THE PROPERTY ASSERTED:
//
//     The six counters count ERRORS and only errors; a read without the reset
//     flag changes nothing; a read with the flag returns exactly what a plain
//     read would have returned and then leaves all six at zero; and the count
//     for a given cause equals the number of times that cause occurred.
//
// The last clause is the one worth the trouble, so it is measured by deliberate
// injection at two different points in the receive path, with the injected
// count varied (1, 2, 3, 5) and the counter required to match EXACTLY. A
// counter that merely "went up" would pass a lazy test while being useless.
//
// NOTE ON A TEMPTING BUT WRONG TEST: "send N pings and check the counters go up
// by N". They must not. These are error counters, not traffic counters -- there
// is no packet counter anywhere in RS485.c. Section 2 asserts the opposite: 30
// clean commands leave every counter at zero. A counter that ticked on healthy
// traffic would be a real bug, and this is the regression guard for it.
//
// HOW THIS DIFFERS FROM WHAT ALREADY EXISTS:
//   * tm_raw_protocol also injects raw frames, but it uses the counters as an
//     ORACLE for two frame-corruption scenarios (a bad CRC32, and 8 garbage
//     bytes), asserting >= 1 or == 1 once each. It never tests the command's
//     own semantics: the reset flag, the snapshot ordering, non-destructive
//     reads, accumulation, exact scaling, or clearing at reset.
//   * tm_crc32_matrix exercises the CRC32 FRAMING of every command; it never
//     reads a counter.
//   * tm_readonly_purity proves the ordinary getters do not perturb the motor.
//     Command 47 is precisely the getter that is NOT pure, so it was excluded
//     there; this module is where its impurity is characterised.
//
// SAFETY. The MOSFETs are never enabled and no closed-loop command is sent;
// section 10 commands half a rotation and reads the COMMANDED position, which
// the planner advances whether or not the output stage is energised, so nothing
// turns. Nothing is broadcast and no alias is touched.
//
// Rack safety of the injected frames, which is the only unusual thing here:
//   * The undersized frames of sections 3, 5-7 and 9 are EXTENDED-ADDRESSED
//     (byte 254 + this motor's 8-byte unique ID). Every other motor on the bus
//     compares the unique ID and returns from rs485_get_next_packet() before
//     any counter is touched (RS485.c:236-243), so their counters do not even
//     move. Section 9 turns that into an assertion.
//   * Those frames declare a length too small to hold a command plus a CRC32,
//     so they are discarded at RS485.c:255-259 on ANY device that reads them.
//     They can never be executed as a command, on any motor, at any address.
//   * Section 8 writes bytes with the LSB clear, which are invalid first bytes
//     on every device (RS485.c:399-403) and therefore cannot form a packet for
//     anyone. They do bump firstBitErrorCount on every listening motor; those
//     counters are RAM-only diagnostics that any reset or power cycle zeroes,
//     and tm_raw_protocol already does the same thing.
//   * Injections are spaced by at least INJECT_GAP_MS. There are only
//     N_RECEIVE_BUFFERS == 2 receive buffers (firmware/Src/config.h:7) and
//     overflowing them is FATAL (ERROR_COMMAND_OVERFLOW, RS485.c:395-397), so
//     frames must never be blasted back to back.
//
// Response field order (Servomotor.h getCommunicationStatisticsResponse, which
// mirrors rs485_error_statistics_t in common_source_files/RS485.h:49-56):
//     crc32ErrorCount        RS485.c:326   software, CRC32 mismatch
//     packetDecodeErrorCount RS485.c:257   software, declared size impossible
//     firstBitErrorCount     RS485.c:402   software, first byte LSB not 1
//     framingErrorCount      RS485.c:362   hardware, USART_ISR_FE
//     overrunErrorCount      RS485.c:367   hardware, USART_ISR_ORE
//     noiseErrorCount        RS485.c:372   hardware, USART_ISR_NE
// The first three are decided by packet CONTENT and are exactly reproducible.
// The last three are electrical and are asserted only as deltas, never as
// absolute values, because a real cable is entitled to the occasional glitch.
// ---------------------------------------------------------------------------

typedef getCommunicationStatisticsResponse Stats;

static const uint8_t  CMD_PING           = 31;   // motor_commands.json: Ping
static const uint32_t INJECT_GAP_MS      = 25;   // >= one main-loop pass; see N_RECEIVE_BUFFERS above
static const uint32_t NO_REPLY_WAIT_MS   = 200;  // a dropped frame answers with silence
static const uint32_t RESYNC_SILENCE_MS  = 250;  // > the 100 ms receiver timeout (RS485.c:162)

// Drain pending RX bytes, always bounded: a stuck line must never hang the slot.
static void statsDrainRx(uint32_t maxMs) {
    uint32_t start = millis();
    while (Serial1.available()) {
        Serial1.read();
        if (millis() - start > maxMs) break;
    }
}

// Reset to a known machine AND re-align the host's framing with the firmware's.
// tfhReset restores the units but says nothing about CRC32; the firmware always
// comes up with CRC32 enabled (RS485.c:49), and every injected frame in this
// module is built for the CRC32-enabled receiver, so the library must agree.
static void freshLink(Servomotor* m) {
    m->enableCRC32();
    tfhReset(m);
    m->enableCRC32();
    statsDrainRx(300);
}

static bool readStats(Servomotor* m, uint8_t resetFlag, Stats* out) {
    Stats s = m->getCommunicationStatistics(resetFlag);
    if (m->getError() != 0) {
        printf("   getCommunicationStatistics(%u) comms error %d\n",
               (unsigned)resetFlag, m->getError());
        return false;
    }
    *out = s;
    return true;
}

static void printStats(const char* label, const Stats& s) {
    printf("   %-22s crc32=%u decode=%u firstBit=%u | framing=%u overrun=%u noise=%u\n",
           label,
           (unsigned)s.crc32ErrorCount, (unsigned)s.packetDecodeErrorCount,
           (unsigned)s.firstBitErrorCount, (unsigned)s.framingErrorCount,
           (unsigned)s.overrunErrorCount, (unsigned)s.noiseErrorCount);
}

// The three content-decided counters, which are exactly reproducible.
static bool softwareCountersZero(const Stats& s) {
    return s.crc32ErrorCount == 0 && s.packetDecodeErrorCount == 0 &&
           s.firstBitErrorCount == 0;
}

// The three electrical counters, only ever compared as a delta.
static uint32_t hardwareTotal(const Stats& s) {
    return s.framingErrorCount + s.overrunErrorCount + s.noiseErrorCount;
}

static bool sameStats(const Stats& a, const Stats& b) {
    return a.crc32ErrorCount == b.crc32ErrorCount &&
           a.packetDecodeErrorCount == b.packetDecodeErrorCount &&
           a.firstBitErrorCount == b.firstBitErrorCount &&
           a.framingErrorCount == b.framingErrorCount &&
           a.overrunErrorCount == b.overrunErrorCount &&
           a.noiseErrorCount == b.noiseErrorCount;
}

static bool allSixZero(const Stats& s) {
    return softwareCountersZero(s) && hardwareTotal(s) == 0;
}

// ---------------------------------------------------------------------------
// Build a frame that is well formed byte-for-byte but declares a length too
// small to be decodable, which is the cleanest way to drive exactly one
// packetDecodeErrorCount increment on exactly one motor.
//
// Layout (matching Communication::sendCommandCore, and tm_raw_protocol's
// builder, minus the CRC32):
//     [0]      encoded size  = (12 << 1) | 1
//     [1]      254           = EXTENDED_ADDRESSING
//     [2..9]   unique ID, little endian
//     [10]     command byte
//     [11]     one filler byte
//
// The receiver parses the size, matches the unique ID, and then reaches
// RS485.c:255: it needs at least bytes_parsed(10) + command(1) + crc32(4) = 15
// bytes, but the frame claims 12. So packet_decode_error_count++ and the frame
// is dropped with no reply. The command byte is never looked at -- which is why
// this injection cannot make any motor DO anything.
//
// It also does not jam the receiver: the declared length matches the bytes
// actually sent, so receiveIndex returns to 0 and firstByteInvalid stays clear
// (RS485.c:417-423). That is the difference from a TRUNCATED frame, which holds
// the receiver hostage until the 100 ms silence timeout.
// ---------------------------------------------------------------------------
static uint16_t buildUndersizedFrame(uint8_t* buf, uint64_t uid) {
    const uint16_t total = 12;
    uint16_t i = 0;
    buf[i++] = (uint8_t)((total << 1) | 0x01);
    buf[i++] = 254;
    for (int b = 0; b < 8; b++) buf[i++] = (uint8_t)((uid >> (8 * b)) & 0xFF);
    buf[i++] = CMD_PING;   // never reached; the frame dies at the size check
    buf[i++] = 0x00;
    return i;              // == total
}

static void injectUndersized(uint64_t uid, int count) {
    uint8_t frame[16];
    uint16_t len = buildUndersizedFrame(frame, uid);
    for (int i = 0; i < count; i++) {
        Serial1.write(frame, len);
        delay(INJECT_GAP_MS);   // only two receive buffers exist; never blast
    }
    statsDrainRx(100);          // there should be nothing, but stay bounded
}

// Inject `count` bytes whose LSB is 0. Each one is rejected as an invalid first
// byte and bumps first_bit_error_count exactly once (RS485.c:399-405): the
// decoded "size" of 0x00 is 0, so receiveIndex >= receivePacketSize is true
// immediately and the receiver is back at byte 0 for the next one.
static void injectBadFirstBytes(int count) {
    uint8_t junk[16];
    if (count > (int)sizeof(junk)) count = (int)sizeof(junk);
    memset(junk, 0x00, sizeof(junk));
    Serial1.write(junk, (size_t)count);
    // firstByteInvalid is cleared ONLY by the receiver timeout (RS485.c:381-385,
    // RTOR = 0.1 s at RS485.c:162), so anything sent sooner is eaten.
    delay(RESYNC_SILENCE_MS);
    statsDrainRx(100);
}

void tm_communication_statistics(void) {
    Serial.println("tm_communication_statistics: BEGIN\n");
    Servomotor* m = tfGetMotor();

    uint64_t uid = m->usingThisUniqueId();
    bool extAddr = m->isUsingExtendedAddressing() && uid != 0;
    printf("Target uniqueId=0x%016llX extendedAddressing=%d\n",
           (unsigned long long)uid, (int)extAddr);
    TEST_RESULT("the motor under test is addressed by unique ID, so injected frames reach only it",
                extAddr);
    if (!extAddr) {
        // Without a unique-ID target an injected frame could be picked up by
        // every motor on the bus. Refuse to inject and leave the device clean.
        freshLink(m);
        return;
    }

    // =====================================================================
    // 1. THE COUNTERS ARE READABLE AND A FRESHLY RESET MACHINE HAS NONE.
    //    The counters live in .bss (RS485.c:50-55) and a system reset is a
    //    real MCU reboot (NVIC_SystemReset, main.c:628), so every one of them
    //    must read zero here. This is the baseline the whole module measures
    //    against, so it is asserted rather than assumed.
    // =====================================================================
    printf("\n--- 1. a freshly reset machine reports no errors at all ---\n");
    {
        freshLink(m);
        Stats s = {};
        bool ok = readStats(m, 0, &s);
        if (ok) printStats("after reset:", s);
        TEST_RESULT("the statistics command answers", ok);
        TEST_RESULT("a freshly reset motor reports zero CRC32, decode and first-bit errors",
                    ok && softwareCountersZero(s));
        TEST_RESULT("a freshly reset motor reports a quiet receiver (no framing, overrun or noise)",
                    ok && hardwareTotal(s) == 0);
        freshLink(m);
    }

    // =====================================================================
    // 2. HEALTHY TRAFFIC MOVES NOTHING. These are ERROR counters -- there is
    //    no packet counter anywhere in RS485.c -- so a burst of perfectly
    //    good commands must leave all six exactly where they were. If any of
    //    them ticked on clean traffic, every field diagnosis that reads a
    //    nonzero counter as "you have a wiring problem" would be wrong.
    // =====================================================================
    printf("\n--- 2. thirty clean commands leave every counter alone ---\n");
    {
        freshLink(m);
        Stats before = {};
        bool ok = readStats(m, 0, &before);

        int good = 0;
        uint8_t payload[10];
        for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 7 + 1);
        for (int i = 0; i < 10 && ok; i++) {
            pingResponse pr = m->ping(payload);
            if (m->getError() == 0 && memcmp(pr.responsePayload, payload, 10) == 0) good++;
            (void)m->getPositionRaw();
            if (m->getError() == 0) good++;
            getStatusResponse st = m->getStatus();
            if (m->getError() == 0 && st.fatalErrorCode == 0) good++;
        }

        Stats after = {};
        bool ok2 = ok && readStats(m, 0, &after);
        if (ok2) printStats("after 30 commands:", after);
        printf("   %d of 30 clean commands answered correctly\n", good);

        TEST_RESULT("thirty ordinary commands all succeed", ok2 && good == 30);
        TEST_RESULT("clean traffic does not increment the CRC32 error counter",
                    ok2 && after.crc32ErrorCount == before.crc32ErrorCount);
        TEST_RESULT("clean traffic does not increment the packet-decode error counter",
                    ok2 && after.packetDecodeErrorCount == before.packetDecodeErrorCount);
        TEST_RESULT("clean traffic does not increment the first-bit error counter",
                    ok2 && after.firstBitErrorCount == before.firstBitErrorCount);
        TEST_RESULT("clean traffic does not disturb the receiver's hardware counters",
                    ok2 && hardwareTotal(after) == hardwareTotal(before));
        freshLink(m);
    }

    // =====================================================================
    // 3. ONE DELIBERATE ERROR PRODUCES EXACTLY ONE COUNT, ON THE RIGHT
    //    COUNTER. The injected frame is well formed apart from declaring a
    //    length that cannot hold a command plus a CRC32, so it must land on
    //    packetDecodeErrorCount (RS485.c:257) and on nothing else. Getting
    //    the wrong counter would be worse than no counter: it would point a
    //    diagnosis at the wrong layer.
    // =====================================================================
    printf("\n--- 3. one undersized frame == exactly one packet-decode error ---\n");
    {
        freshLink(m);
        statsDrainRx(200);
        injectUndersized(uid, 1);

        delay(NO_REPLY_WAIT_MS);
        int avail = Serial1.available();
        printf("   bytes waiting after the injection: %d (expect 0)\n", avail);
        statsDrainRx(200);

        Stats s = {};
        bool ok = readStats(m, 0, &s);
        if (ok) printStats("after 1 bad frame:", s);

        TEST_RESULT("a frame that declares an impossible length gets no reply", avail == 0);
        TEST_RESULT("one undersized frame raises exactly one packet-decode error",
                    ok && s.packetDecodeErrorCount == 1);
        TEST_RESULT("an undersized frame is not miscounted as a CRC32 or first-bit error",
                    ok && s.crc32ErrorCount == 0 && s.firstBitErrorCount == 0);

        // A truncated frame jams the receiver until the 100 ms silence; this
        // one does not, because its declared length matches what was sent.
        getStatusResponse st = m->getStatus();
        TEST_RESULT("the very next command works with no resynchronisation wait",
                    m->getError() == 0 && st.fatalErrorCode == 0);
        freshLink(m);
    }

    // =====================================================================
    // 4. READING WITHOUT THE RESET FLAG IS NON-DESTRUCTIVE. resetCounter == 0
    //    must be a pure observation: the firmware only zeroes the counters
    //    inside `if (reset)` (RS485.c:190-197). Five consecutive reads of a
    //    known-nonzero state must therefore all return the same thing -- which
    //    also proves the statistics command does not count ITSELF as an error.
    // =====================================================================
    printf("\n--- 4. reading without the reset flag changes nothing ---\n");
    {
        freshLink(m);
        injectUndersized(uid, 3);

        Stats reads[5] = {};
        bool ok = true;
        for (int i = 0; i < 5 && ok; i++) ok = readStats(m, 0, &reads[i]);
        if (ok) {
            printStats("read 1:", reads[0]);
            printStats("read 5:", reads[4]);
        }
        bool identical = ok;
        for (int i = 1; i < 5 && identical; i++) identical = sameStats(reads[0], reads[i]);

        TEST_RESULT("five consecutive plain reads all answer", ok);
        TEST_RESULT("five consecutive plain reads return an identical snapshot", identical);
        TEST_RESULT("the injected errors are still there after being read five times",
                    ok && reads[4].packetDecodeErrorCount == 3);
        TEST_RESULT("reading the statistics does not itself count as a communication error",
                    ok && reads[4].crc32ErrorCount == 0 && reads[4].firstBitErrorCount == 0);
        freshLink(m);
    }

    // =====================================================================
    // 5. THE RESET FLAG SNAPSHOTS FIRST AND CLEARS SECOND. This is the whole
    //    reason the command takes an argument: the firmware copies all six
    //    counters into a local struct and only then zeroes the originals
    //    (RS485.c:181-198), so a resetting read loses nothing that happened
    //    before it. A host that polls with reset therefore sees every event
    //    exactly once -- no double counting, no gap.
    // =====================================================================
    printf("\n--- 5. a resetting read returns the pre-reset values, then zeroes ---\n");
    {
        freshLink(m);
        injectUndersized(uid, 2);

        Stats plain = {}, snapshot = {}, after = {}, again = {};
        bool ok = readStats(m, 0, &plain);
        ok = ok && readStats(m, 1, &snapshot);
        ok = ok && readStats(m, 0, &after);
        ok = ok && readStats(m, 1, &again);
        if (ok) {
            printStats("plain read:", plain);
            printStats("resetting read:", snapshot);
            printStats("after the reset:", after);
        }

        TEST_RESULT("a resetting read returns exactly what a plain read would have returned",
                    ok && sameStats(plain, snapshot));
        TEST_RESULT("the pre-reset snapshot still carries the two injected errors",
                    ok && snapshot.packetDecodeErrorCount == 2);
        TEST_RESULT("all six counters read zero immediately after a resetting read",
                    ok && allSixZero(after));
        TEST_RESULT("a resetting read on an already-clear machine returns zeros",
                    ok && allSixZero(again));
        freshLink(m);
    }

    // =====================================================================
    // 6. ANY NONZERO FLAG RESETS. The firmware tests the byte with a bare
    //    `if (reset)` (RS485.c:190), so 1, 2, 128 and 255 must behave
    //    identically. A caller that passes `true` from a language where true
    //    is -1, or passes a count, must not silently get a plain read.
    // =====================================================================
    printf("\n--- 6. every nonzero reset flag clears the counters ---\n");
    {
        const uint8_t flags[] = { 1, 2, 128, 255 };
        bool snapshotsOk = true;
        for (unsigned f = 0; f < sizeof(flags) / sizeof(flags[0]); f++) {
            freshLink(m);
            injectUndersized(uid, 1);
            Stats snap = {}, after = {};
            bool ok = readStats(m, flags[f], &snap);
            ok = ok && readStats(m, 0, &after);
            printf("   flag %3u: snapshot decode=%u, afterwards decode=%u\n",
                   (unsigned)flags[f],
                   (unsigned)(ok ? snap.packetDecodeErrorCount : 999),
                   (unsigned)(ok ? after.packetDecodeErrorCount : 999));
            if (!ok || snap.packetDecodeErrorCount != 1) snapshotsOk = false;
            TEST_RESULT(std::string("a reset flag of ") + std::to_string((int)flags[f]) +
                            " clears all six counters",
                        ok && allSixZero(after));
        }
        TEST_RESULT("every nonzero reset flag also returned the pre-reset snapshot",
                    snapshotsOk);
        freshLink(m);
    }

    // =====================================================================
    // 7. THE COUNT EQUALS THE NUMBER OF EVENTS. A counter that merely goes up
    //    is nearly useless -- "17 CRC errors since Tuesday" is a maintenance
    //    decision, "some CRC errors" is not. So the same injection is repeated
    //    1, 2, 3 and 5 times from a zeroed baseline and the counter is
    //    required to match EXACTLY each time. The final round then injects in
    //    two instalments without an intervening reset, to show the counter
    //    accumulates rather than latching or overwriting.
    // =====================================================================
    printf("\n--- 7. the counter equals the number of injected errors ---\n");
    {
        const int rounds[] = { 1, 2, 3, 5 };
        for (unsigned r = 0; r < sizeof(rounds) / sizeof(rounds[0]); r++) {
            freshLink(m);
            injectUndersized(uid, rounds[r]);
            Stats s = {};
            bool ok = readStats(m, 0, &s);
            printf("   injected %d -> packetDecodeErrorCount = %u\n",
                   rounds[r], (unsigned)(ok ? s.packetDecodeErrorCount : 9999));
            TEST_RESULT(std::string("injecting ") + std::to_string(rounds[r]) +
                            " undersized frames counts exactly " + std::to_string(rounds[r]),
                        ok && s.packetDecodeErrorCount == (uint32_t)rounds[r]);
        }

        // Accumulation across a read, with no reset in between.
        freshLink(m);
        injectUndersized(uid, 2);
        Stats mid = {}, end = {};
        bool ok = readStats(m, 0, &mid);
        injectUndersized(uid, 3);
        ok = ok && readStats(m, 0, &end);
        printf("   2 injected -> %u, then 3 more -> %u\n",
               (unsigned)(ok ? mid.packetDecodeErrorCount : 9999),
               (unsigned)(ok ? end.packetDecodeErrorCount : 9999));
        TEST_RESULT("counts accumulate across reads until they are explicitly reset",
                    ok && mid.packetDecodeErrorCount == 2 && end.packetDecodeErrorCount == 5);
        freshLink(m);
    }

    // =====================================================================
    // 8. A DIFFERENT CAUSE LANDS ON A DIFFERENT COUNTER. Six counters are only
    //    worth having if they discriminate. A byte whose LSB is clear fails
    //    is_valid_first_byte_format() and bumps first_bit_error_count once per
    //    byte (RS485.c:399-405) -- a completely different point in the receive
    //    path from the decode check. Both the count and the SILENCE of the
    //    other counters are asserted.
    // =====================================================================
    printf("\n--- 8. invalid first bytes land on their own counter ---\n");
    {
        freshLink(m);
        statsDrainRx(200);
        injectBadFirstBytes(5);

        Stats s = {};
        bool ok = readStats(m, 0, &s);
        if (ok) printStats("after 5 bad bytes:", s);

        TEST_RESULT("five bytes with the low bit clear count exactly five first-bit errors",
                    ok && s.firstBitErrorCount == 5);
        TEST_RESULT("invalid first bytes are not miscounted as CRC32 or decode errors",
                    ok && s.crc32ErrorCount == 0 && s.packetDecodeErrorCount == 0);
        TEST_RESULT("the link is usable again once the receiver has resynchronised",
                    ok && (m->getStatus(), m->getError() == 0));

        // Two different causes, one machine: the totals must stay separate.
        // The clearing read's own success is folded into ok2: if it silently
        // failed, the counters would never be zeroed and the mixed assertion
        // below would fail with 8/2 instead of 3/2 -- a real comms fault
        // wearing the costume of a counting bug.
        bool cleared = readStats(m, 1, &s);
        injectBadFirstBytes(3);
        injectUndersized(uid, 2);
        Stats mixed = {};
        bool ok2 = cleared && readStats(m, 0, &mixed);
        if (ok2) printStats("mixed causes:", mixed);
        TEST_RESULT("two different causes are tallied separately and neither leaks into the other",
                    ok2 && mixed.firstBitErrorCount == 3 &&
                    mixed.packetDecodeErrorCount == 2 && mixed.crc32ErrorCount == 0);
        freshLink(m);
    }

    // =====================================================================
    // 9. A FRAME FOR SOMEBODY ELSE IS NOT COUNTED AT ALL. The unique-ID
    //    comparison happens at RS485.c:236-243, BEFORE the size check at
    //    RS485.c:255 that would otherwise count this frame. So a motor must
    //    stay at zero while a malformed frame addressed to a different device
    //    goes past on the wire. This is the property that makes every other
    //    injection in this module safe on a shared 35-motor bus, so it is
    //    measured rather than trusted.
    // =====================================================================
    printf("\n--- 9. a malformed frame addressed elsewhere is ignored entirely ---\n");
    {
        freshLink(m);
        Stats before = {}, after = {};
        bool ok = readStats(m, 1, &before);
        // Same frame shape, a unique ID that is definitely not ours. It is
        // still undersized, so even a miraculous ID collision could only
        // produce a discarded frame on that device, never a command.
        injectUndersized(uid ^ 0x5A5A5A5A5A5A5A5AULL, 3);
        ok = ok && readStats(m, 0, &after);
        if (ok) printStats("after 3 foreign frames:", after);

        // Only the content-decided counters are asserted here: the claim is
        // that the addressing filter runs first, and a stray electrical glitch
        // on the hardware counters would be a different (and separately
        // reported) matter -- see section 1.
        TEST_RESULT("frames addressed to another unique ID leave this motor's counters untouched",
                    ok && softwareCountersZero(after));
        TEST_RESULT("and the motor still answers normally afterwards",
                    ok && (m->getStatus(), m->getError() == 0));
        freshLink(m);
    }

    // =====================================================================
    // 10. THE COMMAND EXECUTES IMMEDIATELY, IT IS NOT QUEUED. main.c:1128
    //     handles it inline and never touches the motion queue, so it must
    //     answer promptly in the middle of a long move and must not consume a
    //     queue slot. Diagnostics that only work when the machine is idle are
    //     diagnostics you cannot use during the fault you are chasing.
    //
    //     MOSFETs stay off: this reads the COMMANDED position, which the
    //     planner advances regardless of the output stage, so nothing turns.
    //     Half a rotation is well inside the deviation window even at the
    //     factory default of two rotations.
    // =====================================================================
    printf("\n--- 10. the statistics can be read in the middle of a move ---\n");
    {
        tfhFreshOpen(m);
        m->enableCRC32();
        int32_t distance = (int32_t)(TFH_CPR / 2);
        uint32_t ticks = 3 * (uint32_t)TFH_TPS;      // about three seconds

        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(distance, ticks);
        bool queued = (m->getError() == 0);
        // Asserted separately so that a REFUSED move (which would make every
        // later assertion in this section fail under a misleading name) is
        // reported as what it is. Half a rotation in three seconds is 0.17
        // rot/s against the 15 rot/s ceiling tfhFreshOpen just set, so this
        // must be accepted.
        TEST_RESULT("a half-rotation three-second move is accepted", queued);

        uint8_t depthBefore = m->getNQueuedItems();
        uint32_t worstRoundTrip = 0;
        bool allAnswered = queued;
        for (int i = 0; i < 5 && allAnswered; i++) {
            uint32_t t0 = millis();
            Stats s = {};
            allAnswered = readStats(m, 0, &s);
            uint32_t dt = millis() - t0;
            if (dt > worstRoundTrip) worstRoundTrip = dt;
            delay(60);
        }
        uint8_t depthAfter = m->getNQueuedItems();

        bool drained = tfhDrain(m, 12000);
        int64_t moved = drained ? (m->getPositionRaw() - before) : 0;
        printf("   queue depth %u -> %u, worst stats round trip %u ms, moved %lld of %ld\n",
               (unsigned)depthBefore, (unsigned)depthAfter,
               (unsigned)worstRoundTrip, (long long)moved, (long)distance);

        TEST_RESULT("the statistics answer while a three-second move is still running",
                    allAnswered);
        // WEAKENED from 300 ms. The discriminating figure is the MOVE's 3000 ms,
        // not a round-trip budget: a queued command would answer after the move
        // or not at all. And a tight budget here is not measuring the firmware --
        // the timed region includes the library's own per-call Serial.println to
        // USB CDC, which blocks whenever the host is slow to drain, so a healthy
        // machine can exceed a few hundred milliseconds for reasons that have
        // nothing to do with command 47. 900 ms sits just under Communication's
        // 1000 ms response timeout (Communication.cpp:6), so any read that
        // SUCCEEDED is already bounded; this only pins that it beat the move.
        TEST_RESULT("the answer comes back promptly instead of waiting for the move to finish",
                    allAnswered && worstRoundTrip < 900);
        TEST_RESULT("reading the statistics consumes no motion queue slot",
                    depthAfter <= depthBefore);
        TEST_RESULT("the move still delivers its full displacement despite the polling",
                    drained && llabs((long long)(moved - distance)) <= distance / 100 + 4);
        freshLink(m);
    }

    // =====================================================================
    // 11. A SYSTEM RESET CLEARS THE COUNTERS. They are plain file-scope
    //     statics (RS485.c:50-55) and a system reset is a genuine MCU reboot
    //     (NVIC_SystemReset, main.c:628), so the counters are RAM-only and
    //     must come back at zero. A host that resets a motor and then reads a
    //     stale error count would chase a fault that is no longer there.
    // =====================================================================
    printf("\n--- 11. a system reset wipes the counters ---\n");
    {
        freshLink(m);
        injectBadFirstBytes(4);
        injectUndersized(uid, 3);
        Stats dirty = {}, clean = {};
        bool ok = readStats(m, 0, &dirty);
        if (ok) printStats("before the reset:", dirty);

        // Deliberately do NOT use the reset flag: the point is that the system
        // reset alone clears them.
        freshLink(m);
        bool ok2 = readStats(m, 0, &clean);
        if (ok2) printStats("after the reset:", clean);

        TEST_RESULT("the counters really were dirty before the reset",
                    ok && dirty.firstBitErrorCount == 4 && dirty.packetDecodeErrorCount == 3);
        TEST_RESULT("a system reset clears all six counters without the reset flag",
                    ok2 && allSixZero(clean));
    }

    // =====================================================================
    // 12. LEFT CLEAN. The shared rack is handed back reset, with no latched
    //     fault and no counters left standing.
    // =====================================================================
    printf("\n--- 12. the machine is left clean ---\n");
    {
        freshLink(m);
        Stats s = {};
        bool ok = readStats(m, 1, &s);
        TEST_RESULT("the motor is left with no fatal error and no residual counts",
                    ok && allSixZero(s) && tfhFatal(m) == 0);
    }
}
