#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_status_matrix.cpp
//
// Status-flag transition matrix for Get status (cmd 16). Walks the device
// state machine and asserts the full status-flag word (u16) plus the
// fatalErrorCode (u8) at every step:
//
//   reset            -> flags 0x0000, application mode, no fatal error
//   enableMosfets    -> bit1 (MOSFETs enabled) set
//   goToClosedLoop   -> bit2 (closed loop) set, bit1 still set
//   disableMosfets   -> bit1 clears; bit2 SURVIVES (observed firmware spec)
//   systemReset      -> flags back to 0x0000
//
// Bit semantics (python_programs/test_get_status.py and firmware
// common_source_files/device_status.h):
//   bit 0  in the bootloader (if set, all other flags are 0)
//   bit 1  MOSFETs are enabled          (STATUS_MOSFETS_ENABLED_FLAG_BIT)
//   bit 2  motor is in closed-loop mode (STATUS_CLOSED_LOOP_FLAG_BIT)
//   bit 3  executing calibration
//   bit 4  executing homing
//   bit 5  executing go-to-closed-loop  (STATUS_GO_TO_CLOSED_LOOP_FLAG_BIT)
//   bit 6  busy with a time-consuming task
//   bits 7..15  unused, must be 0
//
// WHY bit2 survives disable: the DISABLE_MOSFETS_COMMAND handler in firmware
// main.c calls only the low-level disable_mosfets() (which drops the gate GPIO
// and thus clears the live bit1), but it does NOT touch motor_control_mode.
// bit2 is derived from motor_control_mode == CLOSED_LOOP_POSITION_CONTROL,
// which is only reset to OPEN_LOOP by a full device init (systemReset). So the
// correct, spec'd behavior is: after disable, bit1=0 but bit2 remains 1, and
// only the subsequent reset returns the word to 0x0000. We assert exactly that.
//
// SAFETY: no motion is commanded anywhere here, so there is no motion queue to
// drain and no async fatal-18 risk. goToClosedLoop energises the motor and
// holds position in closed loop; we settle briefly, then disable + reset to
// leave the device at rest with MOSFETs off. One motor addressed by unique ID
// (collision-safe on the 39-motor rack).
// ---------------------------------------------------------------------------

static const uint16_t BOOTLOADER_BIT        = 1 << 0;  // 0x0001
static const uint16_t MOSFETS_ENABLED_BIT   = 1 << 1;  // 0x0002
static const uint16_t CLOSED_LOOP_BIT       = 1 << 2;  // 0x0004
static const uint16_t GO_TO_CLOSED_LOOP_BIT = 1 << 5;  // 0x0020
static const uint16_t UNUSED_BITS_MASK      = 0xFF80;  // bits 7..15 must be 0

static const int NORMAL_RESET_DELAY_MS = 1500;

// Read (flags, err) via Get status. On a comms error, reports it and returns
// false so the caller records a FAIL rather than trusting stale values.
static bool readStatus(Servomotor* motor, const char* label,
                       uint16_t* flagsOut, uint8_t* errOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  %s: getStatus comms error %d\n", label, motor->getError());
        *flagsOut = 0xFFFF;
        *errOut = 0xFF;
        return false;
    }
    *flagsOut = st.statusFlags;
    *errOut = st.fatalErrorCode;
    printf("  %s: flags=0x%04X fatal=%d\n", label, (unsigned)st.statusFlags, st.fatalErrorCode);
    return true;
}

// Invariants that must hold at EVERY application-mode step: not in bootloader,
// no unused high bits, no fatal error.
static void assertCommonInvariants(const char* step, bool ok,
                                   uint16_t flags, uint8_t err) {
    char name[96];
    snprintf(name, sizeof(name), "%s: status read OK (comms)", step);
    TEST_RESULT(name, ok);

    snprintf(name, sizeof(name), "%s: bootloader bit clear (app mode)", step);
    TEST_RESULT(name, ok && ((flags & BOOTLOADER_BIT) == 0));

    snprintf(name, sizeof(name), "%s: unused high bits (7..15) clear", step);
    TEST_RESULT(name, ok && ((flags & UNUSED_BITS_MASK) == 0));

    snprintf(name, sizeof(name), "%s: fatalErrorCode == 0", step);
    TEST_RESULT(name, ok && (err == 0));
}

void tm_status_matrix(void) {
    Serial.println("tm_status_matrix: BEGIN\n");

    Servomotor* motor = tfGetMotor();
    uint16_t flags = 0;
    uint8_t err = 0;
    bool ok = false;

    // =====================================================================
    // Step 0: RESET -> clean application-mode state, flags == 0x0000.
    // =====================================================================
    printf("\n--- Step 0: reset ---\n");
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no response during reset; don't check errors

    ok = readStatus(motor, "after reset", &flags, &err);
    assertCommonInvariants("Reset", ok, flags, err);
    TEST_RESULT("Reset: MOSFET-enabled bit clear",
                ok && ((flags & MOSFETS_ENABLED_BIT) == 0));
    TEST_RESULT("Reset: closed-loop bit clear",
                ok && ((flags & CLOSED_LOOP_BIT) == 0));
    TEST_RESULT("Reset: flags exactly 0x0000",
                ok && (flags == 0x0000));

    // =====================================================================
    // Step 1: ENABLE MOSFETS -> bit1 set, bit2 still clear.
    // =====================================================================
    printf("\n--- Step 1: enableMosfets ---\n");
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle

    ok = readStatus(motor, "after enable", &flags, &err);
    assertCommonInvariants("Enable", ok, flags, err);
    TEST_RESULT("Enable: MOSFET-enabled bit (bit1) set",
                ok && ((flags & MOSFETS_ENABLED_BIT) != 0));
    TEST_RESULT("Enable: closed-loop bit (bit2) still clear",
                ok && ((flags & CLOSED_LOOP_BIT) == 0));

    // =====================================================================
    // Step 2: GO TO CLOSED LOOP -> bit2 set, bit1 still set.
    // Poll the go-to-closed-loop procedure bit (bit5) clear before asserting,
    // so a firmware variant that runs a timed procedure has fully settled.
    // =====================================================================
    printf("\n--- Step 2: goToClosedLoop ---\n");
    motor->goToClosedLoop();
    checkMotorError(*motor, "goToClosedLoop");
    for (int i = 0; i < 200; i++) {  // up to ~4 s
        uint16_t f = 0; uint8_t e = 0;
        if (readStatus(motor, "waiting", &f, &e) && ((f & GO_TO_CLOSED_LOOP_BIT) == 0))
            break;
        delay(20);
    }
    delay(300);  // extra settle margin

    ok = readStatus(motor, "after goToClosedLoop", &flags, &err);
    assertCommonInvariants("ClosedLoop", ok, flags, err);
    TEST_RESULT("ClosedLoop: closed-loop bit (bit2) set",
                ok && ((flags & CLOSED_LOOP_BIT) != 0));
    TEST_RESULT("ClosedLoop: MOSFET-enabled bit (bit1) still set",
                ok && ((flags & MOSFETS_ENABLED_BIT) != 0));
    TEST_RESULT("ClosedLoop: go-to-closed-loop procedure bit (bit5) clear",
                ok && ((flags & GO_TO_CLOSED_LOOP_BIT) == 0));

    // =====================================================================
    // Step 3: DISABLE MOSFETS -> bit1 clears; bit2 SURVIVES (observed spec).
    // =====================================================================
    printf("\n--- Step 3: disableMosfets ---\n");
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    delay(200);

    ok = readStatus(motor, "after disable", &flags, &err);
    assertCommonInvariants("Disable", ok, flags, err);
    TEST_RESULT("Disable: MOSFET-enabled bit (bit1) clears",
                ok && ((flags & MOSFETS_ENABLED_BIT) == 0));
    // Firmware ground truth: disable_mosfets() leaves motor_control_mode
    // unchanged, so the closed-loop bit persists until a full reset.
    TEST_RESULT("Disable: closed-loop bit (bit2) survives disable",
                ok && ((flags & CLOSED_LOOP_BIT) != 0));

    // =====================================================================
    // Step 4: FINAL RESET -> flags return to 0x0000, clean state.
    // =====================================================================
    printf("\n--- Step 4: final reset ---\n");
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);

    ok = readStatus(motor, "after final reset", &flags, &err);
    assertCommonInvariants("FinalReset", ok, flags, err);
    TEST_RESULT("FinalReset: closed-loop bit cleared by reset",
                ok && ((flags & CLOSED_LOOP_BIT) == 0));
    TEST_RESULT("FinalReset: flags exactly 0x0000",
                ok && (flags == 0x0000));

    // Leave the device clean and at rest (MOSFETs already disabled by reset).
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
}
