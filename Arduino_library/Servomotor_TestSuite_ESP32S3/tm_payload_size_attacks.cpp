#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_payload_size_attacks.cpp
//
// Wrong-payload-size and oversized-packet handling, probed by injecting RAW
// extended-addressed frames on the wire (Serial1.write) with the framing copied
// byte-for-byte from Communication.cpp::sendCommandCore. We deliberately send
// packets the library API cannot construct (wrong payload length for a command,
// and a packet claiming a size larger than the device's receive buffer) and
// assert the EXACT firmware behaviour for each.
//
// GROUND TRUTH (verified in firmware source):
//
//   (a) 'Enable MOSFETs' (cmd 1) expects a 0-byte payload. main.c ENABLE_MOSFETS
//       case calls check_payload_size_is_zero(payload_size) which does
//       fatal_error(ERROR_COMMAND_SIZE_WRONG) when payload_size != 0
//       (firmware/Src/main.c:200-205, dispatch main.c:247-252). ERROR_COMMAND_
//       SIZE_WRONG == 51 (error_codes.json code 51 / error_text.h). We send it
//       with a 4-byte payload -> fatal 51.
//
//   (b) 'Trapezoid move' (cmd 2) expects an 8-byte payload (int32 displacement +
//       uint32 time). main.c TRAPEZOID_MOVE case calls
//       copy_input_parameters_and_check_size(..., sizeof=8, source_size) which
//       does fatal_error(ERROR_COMMAND_SIZE_WRONG) when 8 != source_size
//       (main.c:192-198, dispatch main.c:253-264). We send it with a 4-byte
//       payload -> fatal 51.
//
//   In both fatal cases: fatal_error() (error_handling.c:169) stores the code
//   with set_device_error_code(), refreshes the status-flags snapshot to just
//   the in-bootloader bit (0 here, so flags==0), and thereafter answers only
//   Get status and System reset normally; every OTHER command (and the command
//   that faulted, since it was non-broadcast -> if_fatal_error_then_respond) gets
//   an error-code reply packet. So the device DOES transmit a reply, and Get
//   status reports EXACTLY code 51 with flags 0. System reset clears it.
//
//   (c) A frame claiming a huge size (extended-size encoding: first byte decodes
//       to 127, next two bytes are the 16-bit little-endian total size) with an
//       actual payload larger than the device's receive buffer.
//       MAX_RECEIVE_BUFFER_SIZE = 3 + 9 + 1 + MAX_VALUE_BUFFER_LENGTH(261) + 4 =
//       278 (RS485.c:39). When receiveIndex reaches 278 the ISR sets
//       receiveBufferOverflow and, when the (claimed) packet length is reached,
//       DROPS the packet without committing it (RS485.c:410-423). No command is
//       dispatched, so NO fatal error is raised and NO reply is sent.
//
//       *** This CONTRADICTS the spec sketch, which expected ERROR_TOO_MANY_BYTES
//       (4) or ERROR_COMMAND_TOO_LONG (6). Per the task rules, firmware ground
//       truth wins: error_codes.json documents codes 4 and 6 as RESERVED / never
//       raised ("A received packet that is too long for the receive buffer is
//       silently discarded instead"), and RS485.c confirms the silent-drop path.
//       So the correct assertion is: the device stays SILENT and latches NO fatal
//       error. This module documents that as the ground-truth behaviour. ***
//
// RACK SAFETY:
//   * tfGetMotor() is unique-ID addressed. Every raw frame uses EXTENDED
//     addressing (address byte 254 + the target's 8-byte unique ID, little-endian
//     exactly as sendCommandCore emits it), so no other motor on a 39-motor bus
//     can match, and none replies. The oversized frame is byte-count-dropped by
//     EVERY device's ISR identically, so it elicits zero replies bus-wide.
//   * No alias-88 or bare-alias single-target frame is ever emitted.
//   * No motion is commanded. The malformed 'Enable MOSFETs' faults on the size
//     check BEFORE the MOSFETs are enabled, so the shaft is never energized.
//   * No 'Test mode' command (cmd 36) is ever sent.
//   * After each induced fatal error: systemReset + delay(1500) + drain before
//     the next step. The module ends reset, disabled, and drained.
// ---------------------------------------------------------------------------

static const int      RESET_DELAY_MS  = 1500;   // wait after a plain reset
static const int      SETTLE_MS       = 300;    // let a fatal state latch / reply arrive
static const uint8_t  EXPECT_CODE     = 51;     // ERROR_COMMAND_SIZE_WRONG (error_codes.json)

// Command IDs (firmware/Src/commands.h)
static const uint8_t  CMD_DISABLE_MOSFETS = 0;  // harmless byte for the dropped oversized frame
static const uint8_t  CMD_ENABLE_MOSFETS  = 1;  // expects 0-byte payload
static const uint8_t  CMD_TRAPEZOID_MOVE  = 2;  // expects 8-byte payload

// MAX_RECEIVE_BUFFER_SIZE from RS485.c:39 (3 + 9 + 1 + 261 + 4). A claimed packet
// size above this is overflow-dropped by the receive ISR.
static const uint16_t MAX_RECEIVE_BUFFER_SIZE = 278;

// Drain the RX stream until it has been quiet for `quietMs` (bounded absolutely
// so a wedged/streaming bus can never hang the slot).
static void drainRx(uint32_t quietMs) {
    uint32_t lastByte = millis();
    uint32_t absStart = millis();
    while ((millis() - lastByte < quietMs) && (millis() - absStart < 3000)) {
        if (Serial1.available()) {
            Serial1.read();
            lastByte = millis();
        }
    }
}

// Read the device's latched fatal error code AND status flags via Get status
// (answered normally even in the fatal-error state). Returns false on comms error.
static bool readStatus(Servomotor* motor, uint8_t* codeOut, uint16_t* flagsOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return false;
    }
    *codeOut = st.fatalErrorCode;
    *flagsOut = st.statusFlags;
    return true;
}

// Prove the device is fully responsive at the application level (ping round-trip).
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 7 + 1);
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  ping comms error %d\n", motor->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

// Emit a RAW extended-addressed frame with a deliberately wrong payload length.
// Framing matches Communication.cpp::sendCommandCore for the single-byte-size,
// CRC32-enabled case (the device boots CRC32-enabled, matching the library
// default): [encoded size][254][uid(8, LE)][command][payload...][crc32(4, LE)].
// The CRC32 covers the size byte through the payload (calculate_crc32 is the same
// routine the library uses to build accepted packets), so the frame passes the
// device's CRC check and reaches the command's own size validator.
static void sendWrongSizeFrame(uint64_t uid, uint8_t command, uint16_t payloadSize) {
    uint8_t frame[32];
    uint16_t idx = 0;

    // total = size(1) + addr(1) + uid(8) + command(1) + payload + crc32(4).
    // Small enough for the single-byte size form (<= 127).
    uint16_t total = (uint16_t)(1 + 1 + sizeof(uint64_t) + 1 + payloadSize + 4);

    frame[idx++] = encodeFirstByte((uint8_t)total);   // LSB=1, value = total
    frame[idx++] = EXTENDED_ADDRESSING;               // 254
    memcpy(&frame[idx], &uid, sizeof(uid)); idx += (uint16_t)sizeof(uid);  // 8 bytes, little-endian
    frame[idx++] = command;
    for (uint16_t i = 0; i < payloadSize; i++) frame[idx++] = 0x00;  // junk extra payload bytes

    uint32_t crc = calculate_crc32(frame, idx);       // over size..payload (excl CRC)
    memcpy(&frame[idx], &crc, sizeof(crc)); idx += (uint16_t)sizeof(crc);  // little-endian

    Serial1.write(frame, idx);
}

// Emit a RAW extended-addressed frame that CLAIMS `claimedTotal` bytes (via the
// extended-size encoding) and actually sends that many bytes, exceeding
// MAX_RECEIVE_BUFFER_SIZE so the receive ISR overflow-drops the whole packet.
// No CRC32 is appended: the packet is discarded at the byte-count ISR stage,
// long before any CRC validation, so its content is irrelevant. The command byte
// is Disable MOSFETs (harmless even in the impossible case it were executed).
static void sendOversizedFrame(uint64_t uid, uint16_t claimedTotal) {
    static uint8_t frame[512];
    uint16_t idx = 0;

    frame[idx++] = encodeFirstByte(DECODED_FIRST_BYTE_EXTENDED_SIZE);  // decodes to 127 -> extended size
    frame[idx++] = (uint8_t)(claimedTotal & 0xFF);          // extended size, low byte  (little-endian)
    frame[idx++] = (uint8_t)((claimedTotal >> 8) & 0xFF);   // extended size, high byte
    frame[idx++] = EXTENDED_ADDRESSING;                     // 254
    memcpy(&frame[idx], &uid, sizeof(uid)); idx += (uint16_t)sizeof(uid);  // 8 bytes, little-endian
    frame[idx++] = CMD_DISABLE_MOSFETS;                     // harmless command byte
    while (idx < claimedTotal) frame[idx++] = 0xA5;         // junk payload out to the claimed length

    Serial1.write(frame, claimedTotal);
}

// One wrong-payload-size case: send the malformed frame, confirm the device
// actively replied (an error packet, not silence), confirm Get status latches
// exactly code 51 with flags 0, then recover with System reset and confirm the
// device is clean and responsive again.
static void runWrongSizeCase(Servomotor* motor, uint64_t uid,
                             const char* label, uint8_t command, uint16_t payloadSize) {
    char name[112];
    printf("\n--- %s: cmd %d with %u-byte payload -> expect fatal %d ---\n",
           label, (int)command, (unsigned)payloadSize, (int)EXPECT_CODE);

    drainRx(50);                       // clean RX before the attack
    sendWrongSizeFrame(uid, command, payloadSize);
    delay(SETTLE_MS);                  // let the device process + transmit its reply

    // The device is non-broadcast addressed, so a fatal error produces an error
    // reply packet on the wire (error_handling.c handle_packet default case).
    bool replied = (Serial1.available() > 0);
    printf("  device replied on wire: %d (bytes avail before drain)\n", (int)replied);
    snprintf(name, sizeof(name), "%s: device replies (does not silently drop)", label);
    TEST_RESULT(name, replied);

    drainRx(100);                      // discard that auto error reply before Get status

    uint8_t code = 0xFF;
    uint16_t flags = 0xFFFF;
    bool ok = readStatus(motor, &code, &flags);
    printf("  latched: commsOk=%d fatalCode=%d (expect %d) flags=0x%04X\n",
           (int)ok, (int)code, (int)EXPECT_CODE, (unsigned)flags);

    snprintf(name, sizeof(name), "%s: latches fatal code 51 (ERROR_COMMAND_SIZE_WRONG)", label);
    TEST_RESULT(name, ok && code == EXPECT_CODE);

    // fatal_error() zeroes the flags snapshot (not in bootloader here) -> flags == 0.
    snprintf(name, sizeof(name), "%s: fatal refreshes status flags to 0", label);
    TEST_RESULT(name, ok && flags == 0);

    // Recover.
    motor->systemReset();
    delay(RESET_DELAY_MS);
    drainRx(50);

    uint8_t code2 = 0xFF;
    uint16_t flags2 = 0xFFFF;
    bool ok2 = readStatus(motor, &code2, &flags2);
    printf("  after reset: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)ok2, (int)code2, (unsigned)flags2);
    snprintf(name, sizeof(name), "%s: System reset clears the fatal state", label);
    TEST_RESULT(name, ok2 && code2 == 0);

    snprintf(name, sizeof(name), "%s: device responsive after recovery", label);
    TEST_RESULT(name, pingRoundTrips(motor));
}

void tm_payload_size_attacks(void) {
    Serial.println("test_ard_payload_size_attacks: BEGIN\n");

    Servomotor* motor = tfGetMotor();
    const uint64_t uid = motor->usingThisUniqueId();

    // Defense-in-depth (matches tm_raw_protocol): raw injection is only safe
    // when we have a real extended-addressing target uid — never fall through
    // to anything that could look like an alias frame on a 39-motor bus.
    if (!motor->isUsingExtendedAddressing() || uid == 0) {
        TEST_RESULT("Extended (unique-ID) addressing available for raw injection", false);
        return;
    }

    // ---- Clean, known starting state. ----
    motor->systemReset();
    delay(RESET_DELAY_MS);
    drainRx(50);

    uint8_t startCode = 0xFF;
    uint16_t startFlags = 0xFFFF;
    bool startOk = readStatus(motor, &startCode, &startFlags);
    printf("After reset: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)startOk, (int)startCode, (unsigned)startFlags);
    TEST_RESULT("No fatal error after initial reset", startOk && startCode == 0);

    // =======================================================================
    // (a) Enable MOSFETs (cmd 1, expects 0 bytes) sent with a 4-byte payload.
    // (b) Trapezoid move (cmd 2, expects 8 bytes) sent with a 4-byte payload.
    // Both -> fatal 51 (ERROR_COMMAND_SIZE_WRONG).
    // =======================================================================
    runWrongSizeCase(motor, uid, "A enable+4B", CMD_ENABLE_MOSFETS, 4);
    runWrongSizeCase(motor, uid, "B trapezoid-4B", CMD_TRAPEZOID_MOVE, 4);

    // =======================================================================
    // (c) Oversized packet: claims 400 bytes (> MAX_RECEIVE_BUFFER_SIZE 278) and
    // sends all 400. GROUND TRUTH (RS485.c overflow-drop): the receive ISR drops
    // it without committing -> NO command dispatched -> NO fatal error, NO reply.
    // (This is the firmware reality that CONTRADICTS the spec's expectation of
    // ERROR_TOO_MANY_BYTES / ERROR_COMMAND_TOO_LONG; those codes are reserved and
    // never raised. See error_codes.json codes 4 and 6.)
    // =======================================================================
    const uint16_t CLAIMED = 400;   // 400 > 278 -> overflow-dropped
    printf("\n--- C oversized: claim %u bytes (> %u) -> expect silent discard, NO fatal ---\n",
           (unsigned)CLAIMED, (unsigned)MAX_RECEIVE_BUFFER_SIZE);

    drainRx(50);
    sendOversizedFrame(uid, CLAIMED);
    delay(SETTLE_MS);               // give the device ample time to (not) reply

    bool silent = (Serial1.available() == 0);
    printf("  bytes available after oversized frame: %d -> silent=%d\n",
           Serial1.available(), (int)silent);
    // Firmware silently discards oversized packets (no reply). Documents that
    // neither ERROR_TOO_MANY_BYTES nor ERROR_COMMAND_TOO_LONG is emitted.
    TEST_RESULT("C oversized packet is silently discarded (no reply)", silent);

    drainRx(50);                    // belt-and-suspenders before the status read

    uint8_t cCode = 0xFF;
    uint16_t cFlags = 0xFFFF;
    bool cOk = readStatus(motor, &cCode, &cFlags);
    printf("  after oversized: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)cOk, (int)cCode, (unsigned)cFlags);
    // No fatal error latched: the packet never reached command dispatch.
    TEST_RESULT("C oversized packet latches NO fatal error (code 0)", cOk && cCode == 0);

    TEST_RESULT("C device responsive after oversized frame", pingRoundTrips(motor));

    // ---- Leave the device clean. ----
    motor->systemReset();
    delay(RESET_DELAY_MS);
    drainRx(50);
}
