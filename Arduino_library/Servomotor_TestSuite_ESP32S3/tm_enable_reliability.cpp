#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_enable_reliability.cpp
//
// Arduino-library on-device functional test for Enable/Disable MOSFETs
// reliability. Mirrors python_programs/test_enable_disable_reliability.py
// (which itself modernizes the obsolete test_enable_disable.py and
// test_iterate_reset_and_enable.py).
//
// The per-command basics for Enable MOSFETs (cmd 1) and Disable MOSFETs
// (cmd 0) are covered elsewhere; this test focuses on RELIABILITY:
//
//   Phase A: 20 rapid enable/disable cycles. The status MOSFETs-enabled bit
//            must track each transition (set after enable, clear after
//            disable) with no fatal error, and after the loop the bit must be
//            clear (the final disable stuck).
//   Phase B: 3 cycles of enable -> disable -> systemReset -> wait -> verify
//            the device comes back in application mode with no fatal error.
//   Phase C: a final ping round-trips, confirming comms still work.
//
// SAFETY:
//   * No motion is ever commanded, so there is no motion queue to drain and no
//     ERROR_RUN_OUT_OF_QUEUE_ITEMS / async fatal-18 risk. Enabling MOSFETs on a
//     free shaft may cause a tiny commutation-alignment twitch; that is
//     harmless here because we never issue a precise zero or deviation limit.
//   * Single motor addressed by unique ID (collision-safe on the 39-motor rack).
//   * Ends disabled + systemReset in a clean state.
// ---------------------------------------------------------------------------

// Bit positions in the get_status flags word.
// Must match common_source_files/device_status.h (same as the Python test).
static const uint16_t STATUS_IN_THE_BOOTLOADER_BIT = (1u << 0);
static const uint16_t STATUS_MOSFETS_ENABLED_BIT   = (1u << 1);

static const int TOGGLE_CYCLES     = 20;   // Phase A rapid enable/disable cycles
static const int RESET_CYCLES      = 3;    // Phase B reset/enable cycles
static const int NORMAL_RESET_MS   = 1500; // let the device come back in app mode

// Read get_status; report comms failures. Returns false if the query itself
// failed at the comms layer (out params left untouched in that case).
static bool readStatus(Servomotor* motor, uint16_t* flags, uint8_t* fatal) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  getStatus comms error: %d\n", motor->getError());
        return false;
    }
    *flags = st.statusFlags;
    *fatal = st.fatalErrorCode;
    return true;
}

// Read status and require: comms ok, no fatal error, not in the bootloader.
static bool appModeNoError(Servomotor* motor, const char* label) {
    uint16_t flags = 0;
    uint8_t fatal = 0xFF;
    if (!readStatus(motor, &flags, &fatal)) {
        printf("  %s: get_status unusable\n", label);
        return false;
    }
    if (fatal != 0) {
        printf("  %s: fatal_error_code=%d (expected 0), flags=0x%x\n", label, fatal, flags);
        return false;
    }
    if (flags & STATUS_IN_THE_BOOTLOADER_BIT) {
        printf("  %s: device in bootloader (flags=0x%x)\n", label, flags);
        return false;
    }
    return true;
}

void tm_enable_reliability(void) {
    Serial.println("test_ard_enable_reliability: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_MS);  // device does not respond during reset; no error check
    TEST_RESULT("No fatal error / app mode after initial reset",
                appModeNoError(motor, "after initial reset"));

    // =======================================================================
    // Phase A: rapid enable/disable cycling. The MOSFETs-enabled status bit
    // must track each transition, with no fatal error throughout.
    // =======================================================================
    printf("\n--- Phase A: %d rapid enable/disable cycles ---\n", TOGGLE_CYCLES);

    bool allEnableSetBit  = true;   // enable -> bit set every cycle
    bool allDisableClrBit = true;   // disable -> bit clear every cycle
    bool allNoFatal       = true;   // no fatal error at any read
    bool allComms         = true;   // every command + status read answered

    for (int i = 0; i < TOGGLE_CYCLES; i++) {
        uint16_t flags = 0;
        uint8_t fatal = 0xFF;

        // --- Enable, verify the bit sets ---
        motor->enableMosfets();
        if (motor->getError() != 0) { allComms = false; }
        if (!readStatus(motor, &flags, &fatal)) {
            allComms = false;
        } else {
            if (fatal != 0) allNoFatal = false;
            if (!(flags & STATUS_MOSFETS_ENABLED_BIT)) {
                allEnableSetBit = false;
                printf("  cycle %d: MOSFETs bit NOT set after enable (flags=0x%x)\n", i, flags);
            }
        }

        // --- Disable, verify the bit clears ---
        motor->disableMosfets();
        if (motor->getError() != 0) { allComms = false; }
        if (!readStatus(motor, &flags, &fatal)) {
            allComms = false;
        } else {
            if (fatal != 0) allNoFatal = false;
            if (flags & STATUS_MOSFETS_ENABLED_BIT) {
                allDisableClrBit = false;
                printf("  cycle %d: MOSFETs bit still SET after disable (flags=0x%x)\n", i, flags);
            }
        }
    }
    printf("  completed %d cycles\n", TOGGLE_CYCLES);

    TEST_RESULT("Phase A: all commands/status reads answered", allComms);
    TEST_RESULT("Phase A: MOSFETs bit sets after every enable", allEnableSetBit);
    TEST_RESULT("Phase A: MOSFETs bit clears after every disable", allDisableClrBit);
    TEST_RESULT("Phase A: no fatal error during cycling", allNoFatal);

    // After the loop the device must be in app mode with no fatal error and the
    // final disable must have stuck (MOSFETs bit clear).
    {
        uint16_t flags = 0;
        uint8_t fatal = 0xFF;
        bool ok = readStatus(motor, &flags, &fatal);
        TEST_RESULT("Phase A end: get_status answered", ok);
        TEST_RESULT("Phase A end: no fatal error", ok && fatal == 0);
        TEST_RESULT("Phase A end: not in bootloader (app mode)",
                    ok && !(flags & STATUS_IN_THE_BOOTLOADER_BIT));
        TEST_RESULT("Phase A end: MOSFETs-enabled bit clear (final disable stuck)",
                    ok && !(flags & STATUS_MOSFETS_ENABLED_BIT));
    }

    // =======================================================================
    // Phase B: reset/enable cycling. Each iteration: enable -> disable ->
    // systemReset -> wait -> the device must come back in app mode, no fatal.
    // =======================================================================
    printf("\n--- Phase B: %d reset/enable cycles ---\n", RESET_CYCLES);

    bool allResetRecovered = true;
    for (int i = 0; i < RESET_CYCLES; i++) {
        motor->enableMosfets();
        if (motor->getError() != 0) allResetRecovered = false;
        motor->disableMosfets();
        if (motor->getError() != 0) allResetRecovered = false;

        motor->systemReset();          // broadcast-style: no response expected
        delay(NORMAL_RESET_MS);        // let it come back up in app mode

        char label[48];
        snprintf(label, sizeof(label), "phase B iteration %d", i + 1);
        bool ok = appModeNoError(motor, label);
        if (!ok) allResetRecovered = false;
        printf("  iteration %d: %s\n", i + 1, ok ? "OK" : "FAIL");
    }
    TEST_RESULT("Phase B: device recovers in app mode after every reset",
                allResetRecovered);

    // =======================================================================
    // Phase C: final ping round-trip confirms comms still work.
    // =======================================================================
    printf("\n--- Phase C: final ping ---\n");
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)i;
    pingResponse pr = motor->ping(payload);
    bool pingOk = (motor->getError() == 0) &&
                  (memcmp(pr.responsePayload, payload, 10) == 0);
    printf("  ping round-trip: %s (commsErr=%d)\n", pingOk ? "OK" : "MISMATCH", motor->getError());
    TEST_RESULT("Phase C: final ping round-trips", pingOk);

    // -----------------------------------------------------------------------
    // Clean up: leave the device disabled and reset.
    // -----------------------------------------------------------------------
    motor->disableMosfets();
    motor->systemReset();
    delay(NORMAL_RESET_MS);
}
