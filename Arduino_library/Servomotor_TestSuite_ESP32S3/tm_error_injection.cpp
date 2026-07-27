#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_error_injection.cpp
//
// Fatal-error machinery sweep via the Test mode command (cmd 36).
//
// Firmware behaviour (firmware/Src/main.c): test_mode values 14..73 route to
//   fatal_error(test_mode - 14)
// deliberately raising fatal error codes 0..59 for diagnostic self-test. The
// fatal_error() routine (common_source_files/error_handling.c) stores the raw
// code with set_device_error_code() and refreshes the status flags snapshot to
// 0 (MOSFETs off, nothing running; only the in-bootloader bit could survive,
// and we are not in the bootloader), so Get status reports EXACTLY the injected
// code with flags == 0. Get status and System reset are the only two commands
// answered normally while latched in the fatal-error state; System reset clears
// it (code back to 0, device responsive).
//
// We inject 6 representative codes spread across the range, verify each latches
// exactly, verify the flags are refreshed, then verify System reset clears it
// and the device is fully responsive again. Finally we confirm the safe clear
// value 0 is still accepted cleanly.
//
// SAFETY:
//   * NO motion is commanded anywhere, so there is no motion queue to drain and
//     no async fatal-18 risk.
//   * Test-mode values 10..13 (permanent LED-test hang, power-cycle only) are
//     NEVER sent. Only 14..73 (recoverable via systemReset) and 0 (safe clear).
//   * tfGetMotor() is unique-ID addressed, so every command here is
//     collision-safe on a multi-motor bus; nothing is broadcast that would
//     elicit responses from all devices.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int INJECT_SETTLE_MS      = 300;   // let the fatal state latch

// One injection case: the Test mode value to send and the fatal error code the
// firmware must latch (code = value - 14).
struct InjectionCase {
    uint8_t testModeValue;
    uint8_t expectedCode;
};

// Read the device's latched fatal error code AND status flags via Get status
// (one of the two commands still answered normally in the fatal-error state).
// Returns false if the query itself failed at the comms layer.
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

// Prove the device is fully responsive at the application level by round-
// tripping a 0..9 ping payload.
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

void tm_error_injection(void) {
    Serial.println("test_ard_error_injection: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // 6 representative injected codes spread across the whole 0..59 range.
    // testModeValue - 14 == expectedCode.
    static const InjectionCase CASES[] = {
        { 17, 3  },   // ERROR_FLASH_WRITE_FAIL
        { 30, 16 },   // ERROR_VEL_TOO_HIGH
        { 44, 30 },   // ERROR_CONTROL_LOOP_TOOK_TOO_LONG
        { 55, 41 },   // ERROR_TEST_MODE_ACTIVE
        { 64, 50 },   // ERROR_BAD_ALIAS
        { 73, 59 },   // top of the injectable range (code 59)
    };
    const int NUM_CASES = (int)(sizeof(CASES) / sizeof(CASES[0]));

    // ---- Clean, known starting state. ----
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no response during reset; don't check errors

    uint8_t startCode = 0xFF;
    uint16_t startFlags = 0xFFFF;
    bool startOk = readStatus(motor, &startCode, &startFlags);
    printf("After reset: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)startOk, (int)startCode, (unsigned)startFlags);
    TEST_RESULT("No fatal error after initial reset", startOk && startCode == 0);

    // =======================================================================
    // Sweep each injected fatal error code.
    // =======================================================================
    for (int i = 0; i < NUM_CASES; i++) {
        uint8_t tm = CASES[i].testModeValue;
        uint8_t expect = CASES[i].expectedCode;
        char name[96];

        printf("\n--- Inject test_mode %d -> expect fatal code %d ---\n",
               (int)tm, (int)expect);

        // Inject the fatal error and let it latch.
        motor->testMode(tm);
        delay(INJECT_SETTLE_MS);

        // Get status must report EXACTLY the injected code.
        uint8_t code = 0xFF;
        uint16_t flags = 0xFFFF;
        bool ok = readStatus(motor, &code, &flags);
        printf("  latched: commsOk=%d fatalCode=%d (expect %d) flags=0x%04X\n",
               (int)ok, (int)code, (int)expect, (unsigned)flags);

        snprintf(name, sizeof(name),
                 "Test mode %d latches fatal code %d", (int)tm, (int)expect);
        TEST_RESULT(name, ok && code == expect);

        // Flags refreshed: fatal_error() zeroes the flags snapshot (not in the
        // bootloader here), so the reported status flags must be 0.
        snprintf(name, sizeof(name),
                 "Fatal code %d refreshes status flags to 0", (int)expect);
        TEST_RESULT(name, ok && flags == 0);

        // System reset must clear the fatal state.
        motor->systemReset();
        delay(NORMAL_RESET_DELAY_MS);

        uint8_t clearedCode = 0xFF;
        uint16_t clearedFlags = 0xFFFF;
        bool clearedOk = readStatus(motor, &clearedCode, &clearedFlags);
        printf("  after reset: commsOk=%d fatalCode=%d flags=0x%04X\n",
               (int)clearedOk, (int)clearedCode, (unsigned)clearedFlags);

        snprintf(name, sizeof(name),
                 "System reset clears fatal code %d", (int)expect);
        TEST_RESULT(name, clearedOk && clearedCode == 0);

        // Device must be fully responsive again (application-level ping).
        snprintf(name, sizeof(name),
                 "Device responsive after clearing fatal code %d", (int)expect);
        TEST_RESULT(name, pingRoundTrips(motor));
    }

    // =======================================================================
    // Safe clear value 0 must still be accepted cleanly at the end.
    // =======================================================================
    printf("\n--- Test mode 0 (safe clear) ---\n");
    motor->testMode(0);
    bool clearComms = (motor->getError() == 0);
    uint8_t clearCode = 0xFF;
    uint16_t clearFlags = 0xFFFF;
    bool clearRead = readStatus(motor, &clearCode, &clearFlags);
    printf("testMode(0): commsOk=%d fatalCode=%d\n",
           (int)clearComms, (int)clearCode);
    TEST_RESULT("Test mode 0 (safe clear) returns success", clearComms);
    TEST_RESULT("Test mode 0 leaves no fatal error", clearRead && clearCode == 0);

    // ---- Leave the device clean. ----
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
}
