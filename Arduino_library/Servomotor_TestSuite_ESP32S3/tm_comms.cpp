#include "tf_framework.h"

// tm_comms.cpp
// Mirrors the Python per-command tests:
//   - test_ping.py                      (cmd 31 Ping)
//   - test_get_communication_statistics.py (cmd 47 Get communication statistics)
//   - test_crc32_control.py             (cmd 46 CRC32 control)
//
// Addressing is by unique ID (the wrapper is built from the run args), so all
// methods are called WITHOUT a uniqueId argument. There is exactly one motor
// on the bus.

static const int N_STATS_FIELDS = 6;

// Fill a 10-byte buffer with pseudo-random bytes.
static void fillRandom10(uint8_t out[10]) {
    for (int i = 0; i < 10; i++) {
        out[i] = (uint8_t)(rand() & 0xFF);
    }
}

void tm_comms(void) {
    srand(12345);  // deterministic sequence for reproducibility

    // Create a Servomotor instance using the wrapper (unique-ID addressed)
    Servomotor* motor = tfGetMotor();

    // Start from a clean state.
    motor->systemReset();
    delay(1500);  // Wait for the device to come back up
    // (No error check after systemReset: the device reboots and does not reply.)

    char buf[160];

    // =====================================================================
    // PART 1: Ping (cmd 31) — the device echoes its 10-byte payload back.
    // =====================================================================
    printf("\n=== Part 1: Ping (cmd 31) ===\n");

    const int N_PINGS = 20;
    int successful_pings = 0;
    int failed_pings = 0;

    for (int p = 0; p < N_PINGS; p++) {
        uint8_t sent[10];
        fillRandom10(sent);

        // ping() takes a non-const uint8_t[10] and returns a 10-byte response.
        uint8_t sentCopy[10];
        memcpy(sentCopy, sent, sizeof(sentCopy));
        pingResponse resp = motor->ping(sentCopy);

        bool ok = (motor->getError() == 0) &&
                  (memcmp(resp.responsePayload, sent, 10) == 0);
        if (ok) {
            successful_pings++;
        } else {
            failed_pings++;
            if (motor->getError() != 0) {
                printf("  ping %d: error code %d\n", p, motor->getError());
            } else {
                printf("  ping %d: response mismatch\n", p);
            }
        }
    }
    snprintf(buf, sizeof(buf), "Pings: %d sent, %d successful, %d failed\n",
             N_PINGS, successful_pings, failed_pings);
    Serial.print(buf);

    // Every ping must round-trip its exact payload back (Python: all succeed).
    TEST_RESULT("All pings echoed their exact payload",
                (successful_pings == N_PINGS) && (failed_pings == 0));

    // Spot-check a single, fixed, known payload for clarity.
    {
        uint8_t known[10] = {0, 1, 2, 3, 4, 250, 251, 252, 253, 254};
        uint8_t knownCopy[10];
        memcpy(knownCopy, known, sizeof(knownCopy));
        pingResponse resp = motor->ping(knownCopy);
        bool ok = (motor->getError() == 0) &&
                  (memcmp(resp.responsePayload, known, 10) == 0);
        TEST_RESULT("Ping echoes a known fixed payload", ok);
    }

    // =====================================================================
    // PART 2: Get communication statistics (cmd 47)
    //   - response shape: 6 non-negative counters
    //   - non-decreasing under correct traffic
    //   - resetCounter=1 zeros them
    //   - valid traffic on a quiet bus does not bump any counter
    // =====================================================================
    printf("\n=== Part 2: Get communication statistics (cmd 47) ===\n");

    // Fresh reset so the firmware's RS485 counters start at zero.
    motor->systemReset();
    delay(1500);

    // Read once (reset=0). The counters are all uint32_t in the response
    // struct, so they are non-negative by type; assert the read succeeds and
    // the response framing round-trips (no DATA_WRONG_SIZE error).
    getCommunicationStatisticsResponse s0 = motor->getCommunicationStatistics(0);
    bool s0_ok = (motor->getError() == 0);
    printf("  initial -> crc32=%u decode=%u firstBit=%u framing=%u overrun=%u noise=%u\n",
           s0.crc32ErrorCount, s0.packetDecodeErrorCount, s0.firstBitErrorCount,
           s0.framingErrorCount, s0.overrunErrorCount, s0.noiseErrorCount);
    TEST_RESULT("Initial stats read succeeds (6-field response round-trips)", s0_ok);

    // Drive well-formed traffic; counters must not decrease.
    const int N_TRAFFIC = 30;
    for (int i = 0; i < N_TRAFFIC; i++) {
        uint8_t zeros[10] = {0};
        motor->ping(zeros);
    }
    getCommunicationStatisticsResponse s1 = motor->getCommunicationStatistics(0);
    bool s1_ok = (motor->getError() == 0);
    printf("  after %d pings -> crc32=%u decode=%u firstBit=%u framing=%u overrun=%u noise=%u\n",
           N_TRAFFIC, s1.crc32ErrorCount, s1.packetDecodeErrorCount, s1.firstBitErrorCount,
           s1.framingErrorCount, s1.overrunErrorCount, s1.noiseErrorCount);
    bool nondecreasing =
        s1.crc32ErrorCount        >= s0.crc32ErrorCount &&
        s1.packetDecodeErrorCount >= s0.packetDecodeErrorCount &&
        s1.firstBitErrorCount     >= s0.firstBitErrorCount &&
        s1.framingErrorCount      >= s0.framingErrorCount &&
        s1.overrunErrorCount      >= s0.overrunErrorCount &&
        s1.noiseErrorCount        >= s0.noiseErrorCount;
    TEST_RESULT("Counters do not decrease under correct traffic", s1_ok && nondecreasing);

    // Reset (reset=1) then read (reset=0); all six counters must be zero.
    motor->getCommunicationStatistics(1);
    bool reset_call_ok = (motor->getError() == 0);
    getCommunicationStatisticsResponse s2 = motor->getCommunicationStatistics(0);
    bool s2_ok = (motor->getError() == 0);
    printf("  post-reset -> crc32=%u decode=%u firstBit=%u framing=%u overrun=%u noise=%u\n",
           s2.crc32ErrorCount, s2.packetDecodeErrorCount, s2.firstBitErrorCount,
           s2.framingErrorCount, s2.overrunErrorCount, s2.noiseErrorCount);
    bool all_zero_after_reset =
        s2.crc32ErrorCount == 0 && s2.packetDecodeErrorCount == 0 &&
        s2.firstBitErrorCount == 0 && s2.framingErrorCount == 0 &&
        s2.overrunErrorCount == 0 && s2.noiseErrorCount == 0;
    TEST_RESULT("resetCounter=1 zeros all six counters",
                reset_call_ok && s2_ok && all_zero_after_reset);

    // More valid traffic must keep the counters at zero on a quiet bus.
    for (int i = 0; i < N_TRAFFIC; i++) {
        uint8_t zeros[10] = {0};
        motor->ping(zeros);
    }
    getCommunicationStatisticsResponse s3 = motor->getCommunicationStatistics(0);
    bool s3_ok = (motor->getError() == 0);
    printf("  post-reset+traffic -> crc32=%u decode=%u firstBit=%u framing=%u overrun=%u noise=%u\n",
           s3.crc32ErrorCount, s3.packetDecodeErrorCount, s3.firstBitErrorCount,
           s3.framingErrorCount, s3.overrunErrorCount, s3.noiseErrorCount);
    bool still_zero =
        s3.crc32ErrorCount == 0 && s3.packetDecodeErrorCount == 0 &&
        s3.firstBitErrorCount == 0 && s3.framingErrorCount == 0 &&
        s3.overrunErrorCount == 0 && s3.noiseErrorCount == 0;
    TEST_RESULT("Valid-only traffic does not bump any counter", s3_ok && still_zero);

    // =====================================================================
    // PART 3: CRC32 control (cmd 46)
    // The Arduino library exposes CRC control directly:
    //   motor->enableCRC32() / disableCRC32() / isCRC32Enabled()
    // These toggle the OUTGOING framing; crc32Control(0/1) tells the firmware
    // which framing it expects. Keep the two in sync when switching.
    // =====================================================================
    printf("\n=== Part 3: CRC32 control (cmd 46) ===\n");

    // Fresh reset: firmware defaults to CRC enabled; library default is enabled too.
    motor->systemReset();
    delay(1500);
    motor->enableCRC32();  // make the library side explicit / in sync with firmware default

    TEST_RESULT("Library defaults to CRC32 enabled", motor->isCRC32Enabled());

    // [1] With CRC enabled on both sides, a normal command succeeds.
    getStatusResponse st1 = motor->getStatus();
    bool step1_ok = (motor->getError() == 0);
    printf("  [1] getStatus WITH CRC (both enabled): err=%d flags=0x%04X\n",
           motor->getError(), st1.statusFlags);
    TEST_RESULT("CRC enabled: command WITH a CRC succeeds", step1_ok);

    // [2] Enforcement: firmware still requires a CRC, but we send WITHOUT one.
    //     The firmware silently drops the packet -> the library times out.
    motor->disableCRC32();               // send without a CRC while firmware still wants one
    getStatusResponse st2 = motor->getStatus();
    int step2_err = motor->getError();
    (void)st2;
    printf("  [2] getStatus WITHOUT CRC (firmware still enabled): err=%d (expect nonzero/timeout)\n",
           step2_err);
    TEST_RESULT("CRC enabled: command WITHOUT a CRC is rejected (enforcement on)",
                step2_err != 0);
    motor->enableCRC32();                // restore library framing to match firmware (still enabled)

    // Confirm the bus recovered after the deliberate timeout.
    getStatusResponse st2b = motor->getStatus();
    bool recovered = (motor->getError() == 0);
    (void)st2b;
    TEST_RESULT("Bus recovers after the rejected no-CRC command", recovered);

    // [3] Disable CRC on the firmware (command sent WITH a CRC, still enabled),
    //     then drop the library framing and confirm a no-CRC command works.
    motor->crc32Control(0);              // firmware disables; ack already comes back no-CRC
    bool disable_cmd_ok = (motor->getError() == 0);
    motor->disableCRC32();               // library now sends without a CRC to match
    getStatusResponse st3 = motor->getStatus();
    bool step3_ok = (motor->getError() == 0);
    printf("  [3] crc32Control(0) then getStatus WITHOUT CRC: disableCmdErr=%s statusErr=%d flags=0x%04X\n",
           disable_cmd_ok ? "0" : "nonzero", motor->getError(), st3.statusFlags);
    TEST_RESULT("crc32Control(0) disables; no-CRC command then succeeds",
                disable_cmd_ok && step3_ok);

    // [4] Re-enable CRC on the firmware (command sent WITHOUT a CRC, currently
    //     disabled), then restore library framing and confirm a with-CRC command works.
    motor->crc32Control(1);              // firmware re-enables; ack already comes back with-CRC
    bool enable_cmd_ok = (motor->getError() == 0);
    motor->enableCRC32();                // library now sends with a CRC to match
    getStatusResponse st4 = motor->getStatus();
    bool step4_ok = (motor->getError() == 0);
    printf("  [4] crc32Control(1) then getStatus WITH CRC: enableCmdErr=%s statusErr=%d flags=0x%04X\n",
           enable_cmd_ok ? "0" : "nonzero", motor->getError(), st4.statusFlags);
    TEST_RESULT("crc32Control(1) re-enables; with-CRC command then succeeds",
                enable_cmd_ok && step4_ok);

    // =====================================================================
    // Cleanup: leave the device (and library) with CRC enabled — the default
    // the rest of the suite relies on. A reset restores the firmware default.
    // =====================================================================
    motor->enableCRC32();
    motor->systemReset();
    delay(1500);
}
