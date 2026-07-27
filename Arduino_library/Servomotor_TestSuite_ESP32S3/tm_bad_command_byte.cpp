#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_bad_command_byte.cpp
//
// Robustness probe for UNKNOWN / RESERVED command bytes.
//
// GROUND TRUTH (firmware/Src/main.c, process_packet() switch):
//   * The command switch defines cases for command bytes 0..47 only
//     (Arduino_library/Commands.h: highest enum GET_COMMUNICATION_STATISTICS
//     == 47). Every other command byte (48..255) falls through to:
//         default:
//             rs485_done_with_this_packet();
//             break;                                 // main.c ~line 1146
//     i.e. the packet is consumed and DROPPED SILENTLY. The default case
//     transmits NOTHING (no rs485_transmit_no_error_packet, no error packet)
//     and raises NO fatal error. So a non-broadcast unknown-command request
//     gets NO response at all -> the host simply times out waiting.
//
//   * An unknown command byte is a perfectly VALID packet at the framing/CRC
//     layer: rs485_get_next_packet() (common_source_files/RS485.c ~line 260)
//     returns the command without inspecting whether it is a defined command,
//     and rs485_validate_packet_crc32() passes. Therefore NO error-statistics
//     counter is charged for an unknown command: neither packet_decode_error_count
//     (RS485.c lines 257/309, only for undersized/CRC-less packets) nor
//     crc32_error_count (line 326, only for a CRC mismatch) advances.
//
//   * Command bytes 254 / 253 / 252 are "reserved" ONLY in other frame
//     positions (254 = EXTENDED_ADDRESSING address byte, 253/252 = response
//     indicators — Communication.h). In the COMMAND position they carry no
//     special meaning and hit the same default -> silent drop. This test
//     proves the firmware does not confuse a command-position 254/253/252 with
//     its address/response-indicator role.
//
// WHAT WE ASSERT for each probed byte:
//   1. The device sends NO response (we poll Serial1 for a silence window and
//      require zero bytes back).
//   2. The device stays HEALTHY: Get status still answers, fatal code == 0.
//   3. The next VALID command still works: a ping payload round-trips.
// And once, across the whole burst:
//   4. crc32ErrorCount and packetDecodeErrorCount did NOT advance (unknown
//      commands are not charged as protocol errors).
//
// DELIBERATE DEVIATION FROM THE SPEC SKETCH (raw injection):
//   The spec suggested hand-building a raw extended-addressed Serial1.write()
//   frame. Instead we SEND via the library's Servomotor::sendCommand(cmd,
//   nullptr, 0), which for a unique-ID-addressed motor emits EXACTLY that frame
//   (size | 254 | 8-byte unique ID LE | command | CRC32) with the CRC and CRC-
//   enable state already in lock-step with the device. This produces the same
//   bytes a correct hand-built empty-payload extended frame would, while making
//   it IMPOSSIBLE to accidentally emit an alias-88 frame that would command all
//   39 rack motors. We still read (never write) Serial1 to detect the silence.
//   Hence raw_injection == false. Rack-safe: every frame is unique-ID addressed;
//   nothing is broadcast; no single-alias frame is ever sent.
//
// SAFETY:
//   * No motion is ever commanded; there is no motion queue and no async fatal.
//   * No Test mode (cmd 36) values are used, so 10..13 cannot be hit.
//   * A hard millis() timeout bounds every wait.
// ---------------------------------------------------------------------------

static const int      NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const uint32_t SILENCE_WINDOW_MS     = 250;   // window to confirm no reply
                                                     // (a real reply arrives in a
                                                     //  few ms; 250 ms is generous)

// Command bytes to probe. 47 is the highest DEFINED command (Commands.h), so
// every value here has no switch case -> firmware default -> silent drop.
// Includes the reserved response/address indicators 254/253/252 used as COMMANDS.
static const uint8_t UNKNOWN_CMDS[] = { 48, 100, 200, 250, 255, 254, 253, 252 };
static const int NUM_UNKNOWN = (int)(sizeof(UNKNOWN_CMDS) / sizeof(UNKNOWN_CMDS[0]));

// Drain any pending RX bytes so the next observation starts from silence.
static void drainSerial() {
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}

// Poll Serial1 for a fixed window; return true iff NOT ONE byte arrived
// (i.e. the device stayed silent). Bounded by millis().
static bool observedSilence(uint32_t windowMs) {
    uint32_t start = millis();
    while ((millis() - start) < windowMs) {
        if (Serial1.available()) {
            return false;  // an unexpected response byte appeared
        }
    }
    return true;
}

// Read the device's latched fatal error code AND comms-OK via Get status.
static bool readStatusHealthy(Servomotor* motor, uint8_t* codeOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return false;
    }
    *codeOut = st.fatalErrorCode;
    return true;
}

// Prove the application layer is alive by round-tripping a ping payload.
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 3 + 5);
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  ping comms error %d\n", motor->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

void tm_bad_command_byte(void) {
    Serial.println("tm_bad_command_byte: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // RACK-SAFETY GUARD: this module must only ever emit unique-ID-addressed
    // frames. If the motor is not in extended addressing, abort (return) rather
    // than risk an alias frame that could hit all 39 rack motors.
    // -----------------------------------------------------------------------
    if (!motor->isUsingExtendedAddressing()) {
        printf("ABORT: motor is not unique-ID addressed; refusing to send.\n");
        TEST_RESULT("Motor is unique-ID addressed (rack-safe)", false);
        return;
    }
    TEST_RESULT("Motor is unique-ID addressed (rack-safe)", true);

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no response during reset; don't check errors
    drainSerial();

    uint8_t startCode = 0xFF;
    bool startOk = readStatusHealthy(motor, &startCode);
    printf("After reset: commsOk=%d fatalCode=%d\n", (int)startOk, (int)startCode);
    TEST_RESULT("No fatal error after initial reset", startOk && startCode == 0);

    // Baseline error statistics (resetCounter = 0 -> read without clearing).
    getCommunicationStatisticsResponse baseStats = motor->getCommunicationStatistics(0);
    bool baseStatsOk = (motor->getError() == 0);
    printf("Baseline stats: crc32Err=%lu decodeErr=%lu\n",
           (unsigned long)baseStats.crc32ErrorCount,
           (unsigned long)baseStats.packetDecodeErrorCount);
    TEST_RESULT("Baseline comm statistics readable", baseStatsOk);

    // =======================================================================
    // Probe each unknown / reserved command byte.
    // =======================================================================
    for (int i = 0; i < NUM_UNKNOWN; i++) {
        uint8_t cmd = UNKNOWN_CMDS[i];
        char name[96];

        printf("\n--- Unknown command byte %d ---\n", (int)cmd);

        // Start from a clean RX buffer, then send the unknown command with an
        // EMPTY payload via the library's unique-ID-addressed frame.
        drainSerial();
        motor->sendCommand(cmd, nullptr, 0);

        // Firmware default case transmits nothing -> the bus must stay silent.
        bool silent = observedSilence(SILENCE_WINDOW_MS);
        drainSerial();  // discard anything stray so it can't corrupt later reads
        snprintf(name, sizeof(name),
                 "Cmd byte %d silently dropped (no response)", (int)cmd);
        TEST_RESULT(name, silent);

        // Device must remain healthy: Get status answers, fatal code still 0.
        uint8_t code = 0xFF;
        bool ok = readStatusHealthy(motor, &code);
        snprintf(name, sizeof(name),
                 "Device healthy after cmd byte %d (no fatal error)", (int)cmd);
        TEST_RESULT(name, ok && code == 0);

        // The next VALID command must still work.
        snprintf(name, sizeof(name),
                 "Valid ping works after cmd byte %d", (int)cmd);
        TEST_RESULT(name, pingRoundTrips(motor));
    }

    // =======================================================================
    // Unknown commands must NOT be charged as protocol errors.
    // =======================================================================
    printf("\n--- Verify error statistics did not advance ---\n");
    getCommunicationStatisticsResponse endStats = motor->getCommunicationStatistics(0);
    bool endStatsOk = (motor->getError() == 0);
    printf("Final stats: crc32Err=%lu decodeErr=%lu\n",
           (unsigned long)endStats.crc32ErrorCount,
           (unsigned long)endStats.packetDecodeErrorCount);
    TEST_RESULT("Final comm statistics readable", endStatsOk);

    // crc32_error_count only advances on a CRC mismatch (RS485.c line 326);
    // packet_decode_error_count only on undersized/CRC-less packets (lines
    // 257/309). A valid-framed unknown command charges neither.
    TEST_RESULT("Unknown commands did not raise crc32ErrorCount",
                baseStatsOk && endStatsOk &&
                endStats.crc32ErrorCount == baseStats.crc32ErrorCount);
    TEST_RESULT("Unknown commands did not raise packetDecodeErrorCount",
                baseStatsOk && endStatsOk &&
                endStats.packetDecodeErrorCount == baseStats.packetDecodeErrorCount);

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    drainSerial();
}
