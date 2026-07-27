#include "tf_framework.h"

// Mirrors the Python per-command tests:
//   test_system_reset.py  (cmd 27 System reset)
//   test_get_status.py    (cmd 16 Get status)
//
// Cmd 16 output: (u16 statusFlags, u8 fatalErrorCode).
// statusFlags bits:
//   bit 0: in the bootloader (if set, all other flags are 0)
//   bit 1: MOSFETs are enabled
//   bit 2: motor is in closed-loop mode
//   bit 3: executing calibration
//   bit 4: executing homing
//   bit 5: executing go-to-closed-loop procedure
//   bit 6: busy with a time-consuming task
//   bits 7..15: unused, must be 0

static const uint16_t BOOTLOADER_BIT       = 1u << 0;
static const uint16_t MOSFETS_ENABLED_BIT  = 1u << 1;
static const uint16_t UNUSED_BITS_MASK     = 0xFF80u; // bits 7..15 must be 0

static const unsigned long RESET_DELAY_MS  = 1500;

// Narrow safety limits + an out-of-zone move latch a safety-zone fatal error.
// Firmware safety-zone fatal error codes: 25 (SAFETY_LIMIT), 26 (TURN_POINT), 27 (PREDICTED).
static const float NARROW_LIMIT_ROT        = 0.5f;
static const float OUT_OF_ZONE_MOVE_ROT    = 5.0f;
static const float OUT_OF_ZONE_MOVE_TIME_S = 2.0f;

static bool isSafetyZoneError(uint8_t err) {
    return err == 25 || err == 26 || err == 27;
}

void tm_system_status(void) {
    Serial.println("test_ard_system_status: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean baseline ----
    motor->systemReset();
    delay(RESET_DELAY_MS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // ================================================================
    // Part 1: Get status after reset (mirror test_get_status.py step 1/2)
    // ================================================================
    getStatusResponse s0 = motor->getStatus();
    printf("[1] after reset: flags=0x%04X, fatal_error=%u\n", s0.statusFlags, s0.fatalErrorCode);

    TEST_RESULT("After reset: not in bootloader (application mode)",
                (s0.statusFlags & BOOTLOADER_BIT) == 0);
    TEST_RESULT("After reset: no fatal error (code == 0)",
                s0.fatalErrorCode == 0);
    TEST_RESULT("After reset: unused high bits (7..15) are zero",
                (s0.statusFlags & UNUSED_BITS_MASK) == 0);
    TEST_RESULT("After reset: MOSFET-enabled bit clear",
                (s0.statusFlags & MOSFETS_ENABLED_BIT) == 0);

    // ================================================================
    // Part 2: MOSFET-enabled flag tracks live state (mirror test_get_status.py step 3)
    // ================================================================
    motor->enableMosfets();
    delay(100);
    getStatusResponse sEnabled = motor->getStatus();
    printf("[2] after enableMosfets:  flags=0x%04X, fatal_error=%u\n",
           sEnabled.statusFlags, sEnabled.fatalErrorCode);
    TEST_RESULT("After enableMosfets: MOSFET-enabled bit set",
                (sEnabled.statusFlags & MOSFETS_ENABLED_BIT) != 0);
    TEST_RESULT("After enableMosfets: no fatal error",
                sEnabled.fatalErrorCode == 0);
    TEST_RESULT("After enableMosfets: unused high bits still zero",
                (sEnabled.statusFlags & UNUSED_BITS_MASK) == 0);

    motor->disableMosfets();
    delay(100);
    getStatusResponse sDisabled = motor->getStatus();
    printf("[2] after disableMosfets: flags=0x%04X, fatal_error=%u\n",
           sDisabled.statusFlags, sDisabled.fatalErrorCode);
    TEST_RESULT("After disableMosfets: MOSFET-enabled bit clear",
                (sDisabled.statusFlags & MOSFETS_ENABLED_BIT) == 0);

    // ================================================================
    // Part 3: System reset clears the live enabled state (mirror test_system_reset.py step 2)
    // ================================================================
    motor->enableMosfets();
    delay(100);
    getStatusResponse sBeforeReset = motor->getStatus();
    printf("[3] before reset (mosfets on): flags=0x%04X\n", sBeforeReset.statusFlags);
    TEST_RESULT("Setup for reset test: MOSFET-enabled bit set before reset",
                (sBeforeReset.statusFlags & MOSFETS_ENABLED_BIT) != 0);

    motor->systemReset();
    delay(RESET_DELAY_MS);
    getStatusResponse sAfterReset = motor->getStatus();
    printf("[3] after reset following enable: flags=0x%04X, fatal_error=%u\n",
           sAfterReset.statusFlags, sAfterReset.fatalErrorCode);
    TEST_RESULT("Reset clears MOSFET-enabled state",
                (sAfterReset.statusFlags & MOSFETS_ENABLED_BIT) == 0);
    TEST_RESULT("Reset following enable: not in bootloader",
                (sAfterReset.statusFlags & BOOTLOADER_BIT) == 0);
    TEST_RESULT("Reset following enable: no fatal error",
                sAfterReset.fatalErrorCode == 0);

    // ================================================================
    // Part 4: System reset clears a latched fatal error (mirror test_system_reset.py step 3)
    //   Deliberately drive the device into a safety-zone fatal error with
    //   narrow safety limits + an out-of-zone move, confirm it is latched,
    //   then system_reset and confirm the error clears. This is the documented
    //   way out of a fatal-error state -- the property that matters most.
    // ================================================================
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setSafetyLimits(-NARROW_LIMIT_ROT, NARROW_LIMIT_ROT);
    // Firmware rejects the out-of-zone move with a fatal error; no motion occurs.
    motor->trapezoidMove(OUT_OF_ZONE_MOVE_ROT, OUT_OF_ZONE_MOVE_TIME_S);
    delay(300);
    getStatusResponse sFatal = motor->getStatus();
    printf("[4] after out-of-zone move: flags=0x%04X, fatal_error=%u\n",
           sFatal.statusFlags, sFatal.fatalErrorCode);
    TEST_RESULT("Out-of-zone move latches a safety-zone fatal error (25/26/27)",
                isSafetyZoneError(sFatal.fatalErrorCode));

    motor->systemReset();
    delay(RESET_DELAY_MS);
    getStatusResponse sCleared = motor->getStatus();
    printf("[4] after reset clearing fatal error: flags=0x%04X, fatal_error=%u\n",
           sCleared.statusFlags, sCleared.fatalErrorCode);
    TEST_RESULT("Reset clears the latched fatal error (code back to 0)",
                sCleared.fatalErrorCode == 0);
    TEST_RESULT("After reset clearing error: not in bootloader (device responsive)",
                (sCleared.statusFlags & BOOTLOADER_BIT) == 0);
    TEST_RESULT("After reset clearing error: MOSFETs disabled",
                (sCleared.statusFlags & MOSFETS_ENABLED_BIT) == 0);
    TEST_RESULT("After reset clearing error: unused high bits zero",
                (sCleared.statusFlags & UNUSED_BITS_MASK) == 0);

    // ---- Leave the device in a clean state ----
    motor->systemReset();
    delay(RESET_DELAY_MS);
}
