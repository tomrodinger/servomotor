#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_actuators.cpp
//
// Arduino-library on-device test for the "actuator / device-signalling"
// command group:
//   * cmd 40  Vibrate   (mirrors python_programs/test_vibrate.py)
//   * cmd 41  Identify  (mirrors python_programs/test_identify.py)
//   * cmd 36  Test mode (mirrors python_programs/test_test_mode.py)
//
// This runs against a REAL motor via the serial port; only ONE device is on
// the bus and it is addressed by unique-ID (so all methods are called without
// a uniqueId argument).
//
// SAFETY:
//   * No motion is commanded anywhere in this test, so there is no queue to
//     drain and no ERROR_RUN_OUT_OF_QUEUE_ITEMS risk.
//   * Test-mode values 10..13 (the permanent LED-test hang) are NEVER sent.
//     Only value 0 (the safe clear) and the deliberately fatal-but-fully-
//     recoverable values used by test_test_mode.py (255 and N+14) are used,
//     each immediately followed by a systemReset to recover.
// ---------------------------------------------------------------------------

static const int   NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int   FATAL_RESET_DELAY_MS  = 2000;  // wait after recovering from a fatal error (avoid bootloader pin)
static const uint8_t ERROR_INVALID_TEST_MODE = 53; // firmware code for test_mode > 76

// Read the device's fatal error code via Get status (one of the two commands
// still answered normally in the fatal-error state). Returns 0xFF if the query
// itself failed at the comms layer.
static uint8_t readFatalErrorCode(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

// Send a 0..9 ping and verify the 10-byte payload round-trips unchanged.
static bool pingRoundTrips(Servomotor* motor, const char* label) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)i;
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  %s: ping comms error %d\n", label, motor->getError());
        return false;
    }
    if (memcmp(r.responsePayload, payload, 10) != 0) {
        printf("  %s: ping payload mismatch\n", label);
        return false;
    }
    return true;
}

void tm_actuators(void) {
    Serial.println("test_ard_actuators: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    uint8_t startCode = readFatalErrorCode(motor);
    printf("Fatal error code after reset: %d\n", startCode);
    TEST_RESULT("No fatal error after reset", startCode == 0);

    // =======================================================================
    // cmd 40  VIBRATE  (mirrors test_vibrate.py comms layer)
    //
    // On the bench M17 vibrate is a no-op that still returns success; the
    // point is that the whole on->off comms cycle round-trips and leaves the
    // device in a clean, responsive state. We start a brief vibration then
    // STOP it (vibrate 0) and assert no fatal error, then ping.
    // =======================================================================
    printf("\n--- Vibrate (cmd 40) ---\n");

    motor->vibrate(1);
    bool vibrateOnComms = (motor->getError() == 0);
    delay(200);  // brief settle (a real vibration on M1; harmless no-op on M17)
    uint8_t vibrateOnFatal = readFatalErrorCode(motor);
    printf("vibrate(1): commsErr=%d fatalCode=%d\n", motor->getError(), vibrateOnFatal);
    TEST_RESULT("Vibrate ON returns success (comms ok)", vibrateOnComms);
    TEST_RESULT("Vibrate ON leaves no fatal error", vibrateOnFatal == 0);

    motor->vibrate(0);
    bool vibrateOffComms = (motor->getError() == 0);
    uint8_t vibrateOffFatal = readFatalErrorCode(motor);
    printf("vibrate(0): commsErr=%d fatalCode=%d\n", motor->getError(), vibrateOffFatal);
    TEST_RESULT("Vibrate OFF returns success (comms ok)", vibrateOffComms);
    TEST_RESULT("Vibrate OFF leaves no fatal error", vibrateOffFatal == 0);

    // After the on/off cycle the device must remain responsive.
    TEST_RESULT("Device responsive (ping) after vibrate cycle",
                pingRoundTrips(motor, "post-vibrate"));

    // =======================================================================
    // cmd 41  IDENTIFY  (mirrors test_identify.py comms layer)
    //
    // identify() triggers a ~2.7 s green-LED blink animation driven by the
    // SysTick ISR. We assert no fatal error, then prove the device stays
    // responsive DURING the blink (mid-blink ping) and AFTER it (post-blink
    // ping) - a regression that starved the RS485 ISR during identify would
    // show up here.
    // =======================================================================
    printf("\n--- Identify (cmd 41) ---\n");

    motor->identify();
    bool identifyComms = (motor->getError() == 0);
    uint8_t identifyFatal = readFatalErrorCode(motor);
    printf("identify(): commsErr=%d fatalCode=%d\n", motor->getError(), identifyFatal);
    TEST_RESULT("Identify returns success (comms ok)", identifyComms);
    TEST_RESULT("Identify leaves no fatal error", identifyFatal == 0);

    // Mid-blink responsiveness (~1 s into the ~2.7 s animation).
    delay(1000);
    TEST_RESULT("Device responsive (ping) mid-blink",
                pingRoundTrips(motor, "mid-blink"));

    // Wait out the rest of the blink window, then confirm post-blink health.
    delay(1800);
    TEST_RESULT("Device responsive (ping) post-blink",
                pingRoundTrips(motor, "post-blink"));
    uint8_t afterBlinkFatal = readFatalErrorCode(motor);
    TEST_RESULT("No fatal error after blink window", afterBlinkFatal == 0);

    // =======================================================================
    // cmd 36  TEST MODE  (mirrors test_test_mode.py -- SAFE values only)
    //
    // Values used, exactly mirroring test_test_mode.py:
    //   * 0   -> safe clear of all test modes (returns success, no fatal error)
    //   * 255 -> > 76, deliberately triggers ERROR_INVALID_TEST_MODE (53)
    //   * 44  -> 14 + 30, deliberately triggers fatal_error(30)
    // Values 10..13 (permanent hang) are NEVER sent. Each fatal-triggering
    // value is followed by a Get status check (fatal error is answered
    // normally) and a systemReset to recover.
    // =======================================================================
    printf("\n--- Test mode (cmd 36) ---\n");

    // (a) Safe clear: value 0 must succeed and leave no fatal error.
    motor->testMode(0);
    bool clearComms = (motor->getError() == 0);
    uint8_t clearFatal = readFatalErrorCode(motor);
    printf("testMode(0): commsErr=%d fatalCode=%d\n", motor->getError(), clearFatal);
    TEST_RESULT("Test mode 0 (safe clear) returns success", clearComms);
    TEST_RESULT("Test mode 0 leaves no fatal error", clearFatal == 0);

    // (b) Invalid value 255 -> ERROR_INVALID_TEST_MODE (53).
    motor->testMode(255);
    uint8_t invalidFatal = readFatalErrorCode(motor);
    printf("testMode(255): fatalCode=%d (expect %d)\n", invalidFatal, ERROR_INVALID_TEST_MODE);
    TEST_RESULT("Test mode 255 triggers ERROR_INVALID_TEST_MODE (53)",
                invalidFatal == ERROR_INVALID_TEST_MODE);
    motor->systemReset();
    delay(FATAL_RESET_DELAY_MS);
    TEST_RESULT("Recovered (no fatal error) after invalid test mode reset",
                readFatalErrorCode(motor) == 0);

    // (c) Deterministic fatal trigger: value 44 -> fatal_error(30).
    motor->testMode(44);
    uint8_t fatal30 = readFatalErrorCode(motor);
    printf("testMode(44): fatalCode=%d (expect 30)\n", fatal30);
    TEST_RESULT("Test mode 44 triggers fatal_error(30)", fatal30 == 30);
    motor->systemReset();
    delay(FATAL_RESET_DELAY_MS);
    TEST_RESULT("Recovered (no fatal error) after fatal-trigger test mode reset",
                readFatalErrorCode(motor) == 0);

    // (d) Final safe clear + reset, leaving the device clean.
    motor->testMode(0);
    TEST_RESULT("Final test mode 0 leaves no fatal error",
                readFatalErrorCode(motor) == 0);
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
}
