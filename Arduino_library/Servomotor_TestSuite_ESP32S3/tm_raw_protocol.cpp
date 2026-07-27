#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_raw_protocol.cpp
//
// RAW-FRAME protocol robustness. We hand-build RS485 frames byte-for-byte and
// Serial1.write() them at the target motor, then use the firmware's own
// communication-statistics counters (cmd 47, GET_COMMUNICATION_STATISTICS) as
// the oracle for what the receiver actually did with each frame. Mirrors the
// ideas in python_programs/test_trigger_framing_error.py.
//
// Response struct field order (Servomotor.h getCommunicationStatisticsResponse,
// mirrors common_source_files/RS485.c rs485_get_error_statistics...):
//     [0] crc32ErrorCount        crc32_error_count      (RS485.c:326)
//     [1] packetDecodeErrorCount  packet_decode_error_count (RS485.c:257,309)
//     [2] firstBitErrorCount      first_bit_error_count  (RS485.c:402)
//     [3] framingErrorCount       framing_error_count    (USART_ISR_FE, hw)
//     [4] overrunErrorCount       overrun_error_count    (USART_ISR_ORE, hw)
//     [5] noiseErrorCount         noise_error_count      (USART_ISR_NE, hw)
//
// getCommunicationStatistics(resetCounter): the firmware snapshots the counters
// BEFORE optionally zeroing them (RS485.c:181-198). So getCommunicationStatistics(1)
// returns the pre-reset values and leaves all counters at 0 afterwards; a
// following getCommunicationStatistics(0) reads the fresh accumulation. We use
// (1) to zero the slate before an injection and (0) to read the result, so every
// asserted count is an absolute delta from a known-zero baseline.
//
// GROUND-TRUTH DIVERGENCE FROM THE SPEC SKETCH:
//   The sketch loosely says garbage bumps a "framing/first-byte" counter. In the
//   firmware, framing_error_count is a UART hardware flag (USART_ISR_FE) that
//   only fires on an actual electrical framing violation (bad stop bit). Bytes we
//   write in normal 8N1 do NOT set it. A leading byte with LSB=0 instead fails
//   is_valid_first_byte_format() and bumps first_bit_error_count (RS485.c:399-403).
//   So scenario (b) asserts firstBitErrorCount, NOT framingErrorCount. (This
//   subtest cannot produce a true UART framing error from software.)
//
// The wrong-CRC path (a): a well-framed, correctly-addressed packet whose CRC32
// fails is dropped with NO response and crc32_error_count++ (main.c:226-230 ->
// RS485.c:324-327). crc32_enabled defaults to 1 after reset (RS485.c:49,676), so
// the device does validate our CRC.
//
// SAFETY / RACK:
//   * Every injected frame in scenario (a) is EXTENDED-ADDRESSED (byte 254 +
//     8-byte unique ID of THIS motor, little-endian, exactly as
//     Communication::sendCommandCore lays it out). Other motors on the 39-motor
//     bus see a unique-ID mismatch and silently ignore it (RS485.c:240-242) --
//     no counter change, no response elsewhere.
//   * Scenario (b) writes 8x 0x00. 0x00 has LSB=0, so it is an INVALID first byte
//     on EVERY motor and can never parse as an alias-88 (or any) packet -- it is
//     rejected byte-by-byte and commands no device. It is the safest possible
//     garbage for a shared bus.
//   * No motion is commanded anywhere; the MOSFETs are never enabled. Nothing to
//     drain, no async fatal risk.
//   * No test-mode command (cmd 36) is used, so values 10..13 are impossible.
//   * The stat-reads and pings that verify recovery are unique-ID addressed via
//     the motor object, so they are collision-safe on the shared bus.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int NO_RESPONSE_WAIT_MS   = 200;   // window to prove nothing came back

static const uint8_t PING_COMMAND = 31;         // Commands.h: PING = 31

// Drain any pending RX bytes, bounded by a wall-clock timeout so a stuck line can
// never hang the whole test slot.
static void drainRx(uint32_t maxMs) {
    uint32_t start = millis();
    while (Serial1.available()) {
        Serial1.read();
        if (millis() - start > maxMs) break;
    }
}

// Verify a 0..9 ping payload round-trips unchanged through the motor object
// (proves the device is alive and the bus is healthy again).
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)i;
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  ping comms error %d\n", motor->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

// Zero the counters (getCommunicationStatistics(1) snapshots then zeroes) and
// report whether the command itself succeeded at the comms layer.
static bool resetStats(Servomotor* motor) {
    motor->getCommunicationStatistics(1);
    return motor->getError() == 0;
}

// Read the counters without resetting; returns false on a comms error.
static bool readStats(Servomotor* motor, getCommunicationStatisticsResponse* out) {
    getCommunicationStatisticsResponse s = motor->getCommunicationStatistics(0);
    if (motor->getError() != 0) {
        printf("  getCommunicationStatistics comms error %d\n", motor->getError());
        return false;
    }
    *out = s;
    return true;
}

// Build an extended-addressed frame (254 + uid) exactly as
// Communication::sendCommandCore does with CRC32 enabled, then optionally
// corrupt the CRC32 by flipping its lowest bit. Returns the total frame length.
// Layout for payloadSize<=some small value (total<=127, single size byte):
//   [0]        encodeFirstByte(total) = (total<<1)|1
//   [1]        254 (EXTENDED_ADDRESSING)
//   [2..9]     uniqueId, little-endian
//   [10]       commandID
//   [11..]     payload
//   [last-4..] CRC32 over bytes [0 .. payloadEnd], little-endian
static uint16_t buildExtendedFrame(uint8_t* buf, uint64_t uid, uint8_t commandID,
                                   const uint8_t* payload, uint16_t payloadSize,
                                   bool corruptCrc) {
    uint16_t total = 1 + 1 + 8 + 1 + payloadSize + 4;  // size + addr flag + uid + cmd + payload + crc
    uint16_t i = 0;
    buf[i++] = (uint8_t)((total << 1) | 0x01);          // encoded first byte, LSB=1
    buf[i++] = 254;                                      // EXTENDED_ADDRESSING
    for (int b = 0; b < 8; b++) buf[i++] = (uint8_t)((uid >> (8 * b)) & 0xFF);  // uid LE
    buf[i++] = commandID;
    for (uint16_t p = 0; p < payloadSize; p++) buf[i++] = payload[p];

    // CRC32 over everything written so far (size byte through payload), matching
    // sendCommandCore which seeds with the size byte and folds in addr+cmd+payload.
    uint32_t crc = calculate_crc32(buf, i);
    if (corruptCrc) crc ^= 0x00000001u;                 // flip one bit -> guaranteed mismatch
    buf[i++] = (uint8_t)(crc & 0xFF);
    buf[i++] = (uint8_t)((crc >> 8) & 0xFF);
    buf[i++] = (uint8_t)((crc >> 16) & 0xFF);
    buf[i++] = (uint8_t)((crc >> 24) & 0xFF);
    return i;
}

void tm_raw_protocol(void) {
    Serial.println("test_ard_raw_protocol: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);   // no response during reset; don't check errors
    drainRx(500);

    // The whole module depends on extended (unique-ID) addressing being in use so
    // every raw frame is safe on the shared 39-motor bus.
    uint64_t uid = motor->usingThisUniqueId();
    bool extAddr = motor->isUsingExtendedAddressing() && uid != 0;
    printf("Target uniqueId=0x%016llX extendedAddressing=%d\n",
           (unsigned long long)uid, (int)extAddr);
    TEST_RESULT("Motor is unique-ID (extended) addressed", extAddr);
    if (!extAddr) {
        // Refuse to inject raw frames without a unique-ID target: an alias frame
        // could hit all 39 motors. Abort by returning (leave device reset).
        motor->systemReset();
        delay(NORMAL_RESET_DELAY_MS);
        return;
    }

    // =======================================================================
    // (a) Correctly-framed, correctly-addressed Ping with a DELIBERATELY WRONG
    //     CRC32. Expect: no response, crc32ErrorCount incremented by exactly 1,
    //     and no false first-byte / packet-decode increments (the frame is well
    //     formed apart from the CRC). A normal ping must still work afterwards.
    // =======================================================================
    printf("\n--- (a) wrong-CRC32 extended Ping ---\n");

    TEST_RESULT("(a) Reset stats before injection", resetStats(motor));
    drainRx(200);

    uint8_t frame[64];
    uint8_t pingPayload[10];
    for (int i = 0; i < 10; i++) pingPayload[i] = (uint8_t)i;
    uint16_t frameLen = buildExtendedFrame(frame, uid, PING_COMMAND,
                                           pingPayload, 10, /*corruptCrc=*/true);
    printf("Injecting %u-byte wrong-CRC ping frame\n", (unsigned)frameLen);
    Serial1.write(frame, frameLen);

    delay(NO_RESPONSE_WAIT_MS);       // a dropped packet yields no reply
    int availAfterBadCrc = Serial1.available();
    printf("Bytes available after wrong-CRC ping = %d (expect 0)\n", availAfterBadCrc);
    TEST_RESULT("(a) Wrong-CRC ping elicits no response", availAfterBadCrc == 0);
    drainRx(200);

    getCommunicationStatisticsResponse sa;
    bool okA = readStats(motor, &sa);
    if (okA) {
        printf("(a) counts: crc32=%u decode=%u firstBit=%u framing=%u overrun=%u noise=%u\n",
               (unsigned)sa.crc32ErrorCount, (unsigned)sa.packetDecodeErrorCount,
               (unsigned)sa.firstBitErrorCount, (unsigned)sa.framingErrorCount,
               (unsigned)sa.overrunErrorCount, (unsigned)sa.noiseErrorCount);
    }
    // crc32_error_count++ exactly once (RS485.c:324-327).
    TEST_RESULT("(a) crc32ErrorCount == 1 after one bad-CRC frame",
                okA && sa.crc32ErrorCount == 1);
    // A well-framed frame must NOT trip the first-byte or packet-decode paths.
    TEST_RESULT("(a) No first-byte/packet-decode errors from a valid frame",
                okA && sa.firstBitErrorCount == 0 && sa.packetDecodeErrorCount == 0);
    // Device still fully answers a normal (library-generated, correct-CRC) ping.
    TEST_RESULT("(a) Normal ping works after wrong-CRC injection", pingRoundTrips(motor));

    // =======================================================================
    // (b) GARBAGE bytes: 8x 0x00. 0x00 has LSB=0 -> invalid first byte, rejected
    //     byte-by-byte. Expect: no response, firstBitErrorCount increases (>=1;
    //     each rejected byte bumps it, so ~8), crc32ErrorCount stays 0 (garbage
    //     never reaches CRC validation). Bus must recover (valid ping works).
    // =======================================================================
    printf("\n--- (b) garbage bytes (0x00 x8) ---\n");

    TEST_RESULT("(b) Reset stats before garbage", resetStats(motor));
    drainRx(200);

    uint8_t garbage[8];
    memset(garbage, 0x00, sizeof(garbage));
    printf("Injecting 8 x 0x00\n");
    Serial1.write(garbage, sizeof(garbage));

    delay(NO_RESPONSE_WAIT_MS);        // also lets the receiver-timeout reset RX state
    int availAfterGarbage = Serial1.available();
    printf("Bytes available after garbage = %d (expect 0)\n", availAfterGarbage);
    TEST_RESULT("(b) Garbage elicits no response", availAfterGarbage == 0);
    drainRx(200);

    getCommunicationStatisticsResponse sb;
    bool okB = readStats(motor, &sb);
    if (okB) {
        printf("(b) counts: crc32=%u decode=%u firstBit=%u framing=%u overrun=%u noise=%u\n",
               (unsigned)sb.crc32ErrorCount, (unsigned)sb.packetDecodeErrorCount,
               (unsigned)sb.firstBitErrorCount, (unsigned)sb.framingErrorCount,
               (unsigned)sb.overrunErrorCount, (unsigned)sb.noiseErrorCount);
    }
    // Each 0x00 (invalid first byte) bumps first_bit_error_count (RS485.c:399-403).
    TEST_RESULT("(b) firstBitErrorCount increased by garbage",
                okB && sb.firstBitErrorCount >= 1);
    // Garbage never reaches CRC validation, so crc32ErrorCount must stay 0.
    TEST_RESULT("(b) crc32ErrorCount stays 0 for garbage",
                okB && sb.crc32ErrorCount == 0);
    // Bus recovered: a valid ping round-trips.
    TEST_RESULT("(b) Bus recovers: normal ping works after garbage", pingRoundTrips(motor));

    // -----------------------------------------------------------------------
    // Leave the device clean: zero the counters, reset, drain.
    // -----------------------------------------------------------------------
    resetStats(motor);
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    drainRx(500);
}
