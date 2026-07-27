#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_alias_edges.cpp
//
// Edge-case sweep of the 'Set device alias' command (cmd 24), probing the
// boundaries of the valid alias range and the reserved-alias error path.
//
// GROUND TRUTH (verified in firmware + JSON, cited at each assertion):
//   * Valid aliases are 0..251 and 255. On a valid value the firmware sends a
//     success response first (in reply to the OLD address), then writes the new
//     alias to flash and REBOOTS itself, dropping off the bus for ~1.5..2 s
//     before returning under the new alias.  (firmware/Src/main.c:548-563,
//     Servomotor.cpp:902-911 setDeviceAlias-by-uniqueId doc comment.)
//   * Reserved aliases 252, 253, 254 raise fatal_error(ERROR_BAD_ALIAS). The
//     firmware compares new_alias against RESPONSE_CHARACTER_CRC32_DISABLED(252),
//     RESPONSE_CHARACTER_CRC32_ENABLED(253) and EXTENDED_ADDRESSING(254) and
//     faults before saving anything  (main.c:553-556). ERROR_BAD_ALIAS == 50
//     (python_programs/servomotor/error_codes.json code 50, enum
//     ERROR_BAD_ALIAS). Because the fault replies to the triggering command with
//     an error packet (common_source_files/error_handling.c fatal_error path),
//     the library's getResponse returns the remote code 50 directly, so
//     getError()==50 immediately (matches test_set_device_alias.py which catches
//     FatalError(50)). The alias is NOT changed by a rejected value.
//   * A latched fatal error answers only 'Get status' and 'System reset';
//     Get status then reports fatalErrorCode==50 and, because fatal_error()
//     force-disables the MOSFETs and refreshes the status-flags snapshot,
//     statusFlags==0 (same property verified in tm_error_injection.cpp).
//     System reset clears it.
//
// We test: alias 0 (lower valid boundary) and 251 (upper valid boundary) --
// each set, confirmed by Detect devices after the reboot, then restored to the
// home alias 88; then reserved 252 and 253 -- each must fault with code 50,
// recover via System reset, leaving the original alias 88 untouched. (254 is
// already covered elsewhere per the module spec and is skipped here.)
//
// SAFETY:
//   * NO motion is ever commanded, so there is no motion queue and no async
//     fatal risk; sequences are trivially at rest.
//   * NO test-mode (cmd 36) is used, so the 10..13 permanent-hang values are
//     structurally impossible here.
//   * RACK-SAFE: every command goes out via tfGetMotor(), which is unique-ID
//     (extended, address byte 254 + 8-byte ID) addressed, so setDeviceAlias,
//     detectDevices and getStatus all target exactly ONE device. detectDevices
//     is sent by unique ID (never broadcast), so only the one motor replies --
//     no multi-device response storm on the 39-motor bus. No raw Serial1.write
//     frames are emitted (RX is only DRAINED, never written).
//   * The device is ALWAYS returned to home alias 88 through a single cleanup
//     tail; the linear flow uses no early returns, so every path reaches it.
// ---------------------------------------------------------------------------

static const uint8_t  ORIGINAL_ALIAS        = 88;    // home alias ('X'); restored at the end
static const uint8_t  ERROR_BAD_ALIAS       = 50;    // error_codes.json code 50, ERROR_BAD_ALIAS
static const int      NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int      FATAL_RESET_DELAY_MS  = 2000;  // wait after recovering from a fatal error
static const int      ALIAS_REBOOT_DELAY_MS = 2000;  // set-alias saves to flash + self-reboots (~1.5..2 s)
static const int      LATCH_SETTLE_MS       = 300;   // let a fatal state latch before Get status
static const int      DETECT_QUIET_MS       = 1200;  // device ignores packets ~1 s after a Detect

// Drain any bytes sitting in the RS485 receive buffer (bounded so a stuck line
// can never hang the whole test slot). Reads only -- never writes the bus.
static void drainSerial() {
    uint32_t t0 = millis();
    while (Serial1.available()) {
        Serial1.read();
        if (millis() - t0 > 2000) break;
    }
}

// System reset + settle + drain. Leaves the device clean and responsive.
static void cleanReset(Servomotor* motor, int settleMs) {
    motor->systemReset();          // unique-ID addressed; no response expected
    delay(settleMs);               // do NOT check errors: nothing answers during reset
    drainSerial();
}

// Detect the single target device by its unique ID and return its current
// alias, or 0xFF if it could not be detected. Retries a few times because the
// Detect reply arrives after a pseudo-random 0..950 ms delay and can graze the
// 1 s response timeout. After EVERY attempt we wait out the ~1 s window during
// which the device silently discards incoming packets, so on return the bus is
// settled and the next command is safe to send.
static uint8_t detectAlias(Servomotor* motor, uint64_t uid, bool* commsOk) {
    *commsOk = false;
    uint8_t found = 0xFF;
    for (int attempt = 0; attempt < 4; attempt++) {
        detectDevicesResponse r = motor->detectDevices();   // by unique ID -> exactly one reply
        bool got = (motor->getError() == 0 && r.uniqueId == uid);
        delay(DETECT_QUIET_MS);    // ride out the post-detect discard window
        drainSerial();
        if (got) {
            *commsOk = true;
            found = r.alias;
            break;
        }
    }
    return found;
}

// Set a VALID alias by unique ID, wait out the flash-write + self-reboot, and
// report whether the pre-reboot success response was received cleanly.
static bool setValidAlias(Servomotor* motor, uint8_t alias) {
    motor->setDeviceAlias(alias);           // by unique ID; success reply precedes the reboot
    bool ok = (motor->getError() == 0);
    delay(ALIAS_REBOOT_DELAY_MS);           // device is off the bus while it saves + reboots
    drainSerial();
    return ok;
}

// Read the latched fatal error code and status flags via Get status (answered
// normally even in the fatal state). Returns false on a comms-layer failure.
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

// Prove the device answers at the application level by round-tripping a ping.
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

// Exercise one RESERVED alias: it must fault with ERROR_BAD_ALIAS (50) in the
// immediate reply, latch that code (flags refreshed to 0), and clear on reset.
static void testReservedAlias(Servomotor* motor, uint8_t badAlias) {
    char name[96];
    printf("\n--- Reserved alias %d -> expect fatal ERROR_BAD_ALIAS (50) ---\n",
           (int)badAlias);

    // The firmware faults BEFORE saving (main.c:553-556) and replies to this very
    // command with an error packet; getResponse surfaces the remote code as
    // getError(). Ground truth: error_codes.json code 50 == ERROR_BAD_ALIAS.
    motor->setDeviceAlias(badAlias);        // by unique ID
    int immediateErr = motor->getError();
    printf("  immediate getError after set(%d) = %d (expect %d)\n",
           (int)badAlias, immediateErr, (int)ERROR_BAD_ALIAS);
    snprintf(name, sizeof(name),
             "Reserved alias %d rejected with error 50 in reply", (int)badAlias);
    TEST_RESULT(name, immediateErr == (int)ERROR_BAD_ALIAS);

    delay(LATCH_SETTLE_MS);                 // let the fatal state latch

    // Get status must report exactly code 50, and fatal_error() refreshes the
    // flags snapshot to 0 (MOSFETs off, nothing running, not in bootloader) --
    // same flag-refresh property asserted in tm_error_injection.cpp.
    uint8_t code = 0xFF;
    uint16_t flags = 0xFFFF;
    bool ok = readStatus(motor, &code, &flags);
    printf("  latched: commsOk=%d fatalCode=%d (expect 50) flags=0x%04X\n",
           (int)ok, (int)code, (unsigned)flags);

    snprintf(name, sizeof(name),
             "Reserved alias %d latches fatal ERROR_BAD_ALIAS (50)", (int)badAlias);
    TEST_RESULT(name, ok && code == (int)ERROR_BAD_ALIAS);

    snprintf(name, sizeof(name),
             "Fatal from alias %d refreshes status flags to 0", (int)badAlias);
    TEST_RESULT(name, ok && flags == 0);

    // System reset clears the fatal state (one of the two commands honoured
    // while latched).
    cleanReset(motor, FATAL_RESET_DELAY_MS);

    uint8_t clearedCode = 0xFF;
    uint16_t clearedFlags = 0xFFFF;
    bool clearedOk = readStatus(motor, &clearedCode, &clearedFlags);
    printf("  after reset: commsOk=%d fatalCode=%d\n",
           (int)clearedOk, (int)clearedCode);
    snprintf(name, sizeof(name),
             "System reset clears fatal from alias %d", (int)badAlias);
    TEST_RESULT(name, clearedOk && clearedCode == 0);
}

void tm_alias_edges(void) {
    Serial.println("test_ard_alias_edges: BEGIN\n");

    Servomotor* motor = tfGetMotor();
    const uint64_t uid = motor->usingThisUniqueId();

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    cleanReset(motor, NORMAL_RESET_DELAY_MS);

    uint8_t startCode = 0xFF;
    uint16_t startFlags = 0xFFFF;
    bool startStatusOk = readStatus(motor, &startCode, &startFlags);
    printf("After reset: commsOk=%d fatalCode=%d\n", (int)startStatusOk, (int)startCode);
    TEST_RESULT("No fatal error at entry", startStatusOk && startCode == 0);

    // Capture the device's ACTUAL current (home) alias by unique-ID Detect and
    // use THAT as the value to restore. The shared 39-motor rack may leave this
    // motor at any valid alias (not necessarily 88), and the test must return it
    // EXACTLY as found — never force a specific alias.
    bool startDetOk = false;
    uint8_t startAlias = detectAlias(motor, uid, &startDetOk);
    const uint8_t homeAlias = (startDetOk && startAlias <= 251) ? startAlias : ORIGINAL_ALIAS;
    printf("Home alias detected = %d (commsOk=%d)\n", (int)startAlias, (int)startDetOk);
    TEST_RESULT("Device starts at a valid, detectable home alias (0..251)",
                startDetOk && startAlias <= 251);

    // =======================================================================
    // Lower valid boundary: alias 0.
    // =======================================================================
    printf("\n--- Valid boundary: alias 0 ---\n");
    bool set0Ok = setValidAlias(motor, 0);
    TEST_RESULT("Alias 0 (lower valid boundary) accepted", set0Ok);

    bool det0Ok = false;
    uint8_t alias0 = detectAlias(motor, uid, &det0Ok);
    printf("Detected alias after set(0) = %d (commsOk=%d)\n", (int)alias0, (int)det0Ok);
    TEST_RESULT("Alias 0 detected after reboot", det0Ok && alias0 == 0);

    // Restore the detected home alias before the next case.
    setValidAlias(motor, homeAlias);

    // =======================================================================
    // Upper valid boundary: alias 251.
    // =======================================================================
    printf("\n--- Valid boundary: alias 251 ---\n");
    bool set251Ok = setValidAlias(motor, 251);
    TEST_RESULT("Alias 251 (upper valid boundary) accepted", set251Ok);

    bool det251Ok = false;
    uint8_t alias251 = detectAlias(motor, uid, &det251Ok);
    printf("Detected alias after set(251) = %d (commsOk=%d)\n", (int)alias251, (int)det251Ok);
    TEST_RESULT("Alias 251 detected after reboot", det251Ok && alias251 == 251);

    // Restore the detected home alias before the reserved-value cases.
    setValidAlias(motor, homeAlias);

    // =======================================================================
    // Reserved values 252 and 253: each must fault ERROR_BAD_ALIAS (50).
    // (254 is covered by the spec elsewhere and intentionally not sent here.)
    // =======================================================================
    testReservedAlias(motor, 252);
    testReservedAlias(motor, 253);

    // A rejected alias never changes the stored value, so the original home
    // alias 88 must still be in effect after both reserved attempts.
    bool intactOk = false;
    uint8_t intactAlias = detectAlias(motor, uid, &intactOk);
    printf("Alias after reserved-value rejections = %d (commsOk=%d, expect %d)\n",
           (int)intactAlias, (int)intactOk, (int)homeAlias);
    TEST_RESULT("Home alias intact after reserved-alias rejections",
                intactOk && intactAlias == homeAlias);

    // Device fully responsive at the application level.
    TEST_RESULT("Device responsive (ping) at end", pingRoundTrips(motor));

    // -----------------------------------------------------------------------
    // Single cleanup tail: guarantee the device is left at home alias 88, at
    // rest, and clean. (No early returns above, so this is always reached.)
    // -----------------------------------------------------------------------
    bool finalOk = false;
    uint8_t finalAlias = detectAlias(motor, uid, &finalOk);
    if (!(finalOk && finalAlias == homeAlias)) {
        printf("Cleanup: alias is %d, forcing restore to %d\n",
               (int)finalAlias, (int)homeAlias);
        setValidAlias(motor, homeAlias);
    }
    cleanReset(motor, NORMAL_RESET_DELAY_MS);
}
