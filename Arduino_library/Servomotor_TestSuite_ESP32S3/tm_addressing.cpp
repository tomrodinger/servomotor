#include "tf_framework.h"

// tm_addressing.cpp
//
// Converted from test_ard_addressing.cpp: RS485 addressing / device
// management commands:
//   - Command 20: Detect devices
//   - Command 21: Set device alias
//
// Mirrors the intent of the Python tests:
//   - test_detect_devices.py                 (at least one device found; uniqueId valid)
//   - test_set_device_alias.py               (set an alias, verify by detect, reserved
//                                             aliases 252..254 raise fatal error 50)
//   - test_correct_and_incorrect_addressing.py (unique-ID addressing keeps working
//                                             regardless of the current alias)
//
// SAFETY: this test is inherently safe because the motor is addressed by its
// unique ID (extended addressing), which does not depend on the alias. We learn
// the current alias by detecting first, temporarily change the alias to a scratch
// value, verify, then RESTORE the original alias before finishing so later tests
// in the suite can still address the device by alias. No motion is commanded.
//
// TIMING NOTES (firmware ground truth):
//   - After a 'Detect devices' command a device silently discards ALL incoming
//     packets for ~1 second, so we wait ~1.3 s after every detect before the next
//     command.
//   - After a successful 'Set device alias' the device saves to flash and reboots,
//     dropping off the bus for ~1.5..2 s, so we wait ~2.5 s.
//   - A reserved alias (252..254) raises fatal error 50 and does NOT reboot; the
//     device answers only 'Get status' and 'System reset' until reset.

// Reserved alias that must raise fatal error 50 (ERROR_BAD_ALIAS).
static const uint8_t RESERVED_ALIAS = 254;
static const uint8_t ERROR_BAD_ALIAS = 50;

// Delays (ms)
static const int DETECT_QUIET_MS = 1300;  // packet-discard window after detect
static const int ALIAS_REBOOT_MS = 2500;  // reboot after successful set-alias
static const int RESET_MS        = 1500;  // reboot after system reset

void tm_addressing(void) {
    // Get the motor (addressed by unique ID because the harness passes a unique-ID arg).
    Servomotor* motor = tfGetMotor();

    // Clean, known state.
    motor->systemReset();
    delay(RESET_MS);
    // Defensive: drain stale RX bytes a previous module may have left behind
    // (e.g. straggler detect-responses on a 39-motor bus).
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)

    const bool extended = motor->isUsingExtendedAddressing();
    const uint64_t addressedUniqueId = motor->usingThisUniqueId();
    printf("Addressed by extended (unique-ID) addressing: %s\n", extended ? "yes" : "no");

    // -------------------------------------------------------------------------
    // Command 20: Detect devices  -- learn the current alias and unique ID.
    // -------------------------------------------------------------------------
    // Note: the wrapper motor addresses by unique ID; the firmware still replies
    // to 'Detect devices' even when addressed to one specific device.
    detectDevicesResponse d1 = motor->detectDevices();
    int detectErr = motor->getError();
    printf("Detect #1: error=%d uniqueId=0x%016llX alias=%u\n",
           detectErr, (unsigned long long)d1.uniqueId, (unsigned)d1.alias);
    delay(DETECT_QUIET_MS);  // respect the post-detect packet-discard window

    TEST_RESULT("Detect devices: at least one device found (no comm error)",
                detectErr == 0);
    TEST_RESULT("Detect devices: detected uniqueId is valid (non-zero)",
                d1.uniqueId != 0);
    // In extended addressing the detected uniqueId must equal the one we address by.
    TEST_RESULT("Detect devices: detected uniqueId matches the addressed uniqueId",
                (!extended) || (d1.uniqueId == addressedUniqueId));

    const uint8_t originalAlias = d1.alias;
    // Pick a scratch alias that differs from the original one. 100 and 101 are
    // both valid (valid aliases are 0..251 and 255).
    const uint8_t tempAlias = (originalAlias == 100) ? 101 : 100;
    printf("Original alias = %u ; temporary scratch alias = %u\n",
           (unsigned)originalAlias, (unsigned)tempAlias);

    // -------------------------------------------------------------------------
    // Command 21: Set device alias -- set the scratch alias, verify.
    // -------------------------------------------------------------------------
    printf("Setting device alias to temporary value %u (via unique ID)...\n",
           (unsigned)tempAlias);
    motor->setDeviceAlias(tempAlias);
    delay(ALIAS_REBOOT_MS);  // flash write + reboot

    // Unique-ID addressing is independent of the alias, so Get status must still
    // work after the alias change.
    getStatusResponse st_temp = motor->getStatus();
    int statusErrTemp = motor->getError();
    printf("After set temp alias: getStatus error=%d flags=0x%04X fatal=%u\n",
           statusErrTemp, st_temp.statusFlags, (unsigned)st_temp.fatalErrorCode);
    TEST_RESULT("Set device alias: unique-ID addressing still works after alias change",
                statusErrTemp == 0);
    TEST_RESULT("Set device alias: no fatal error after setting a valid alias",
                statusErrTemp == 0 && st_temp.fatalErrorCode == 0);

    // Detect again to confirm the alias actually changed to the scratch value.
    detectDevicesResponse d2 = motor->detectDevices();
    int detectErr2 = motor->getError();
    printf("Detect #2 (verify temp): error=%d uniqueId=0x%016llX alias=%u\n",
           detectErr2, (unsigned long long)d2.uniqueId, (unsigned)d2.alias);
    delay(DETECT_QUIET_MS);
    TEST_RESULT("Set device alias: detect confirms the new (temporary) alias",
                detectErr2 == 0 && d2.alias == tempAlias);

    // -------------------------------------------------------------------------
    // Command 21: Set device alias -- RESTORE the original alias.
    // -------------------------------------------------------------------------
    printf("Restoring device alias to original value %u (via unique ID)...\n",
           (unsigned)originalAlias);
    motor->setDeviceAlias(originalAlias);
    delay(ALIAS_REBOOT_MS);

    getStatusResponse st_restore = motor->getStatus();
    int statusErrRestore = motor->getError();
    printf("After restore alias: getStatus error=%d flags=0x%04X fatal=%u\n",
           statusErrRestore, st_restore.statusFlags, (unsigned)st_restore.fatalErrorCode);
    TEST_RESULT("Restore alias: unique-ID addressing still works",
                statusErrRestore == 0);

    detectDevicesResponse d3 = motor->detectDevices();
    int detectErr3 = motor->getError();
    printf("Detect #3 (verify restore): error=%d uniqueId=0x%016llX alias=%u\n",
           detectErr3, (unsigned long long)d3.uniqueId, (unsigned)d3.alias);
    delay(DETECT_QUIET_MS);
    TEST_RESULT("Restore alias: detect confirms the original alias is restored",
                detectErr3 == 0 && d3.alias == originalAlias);

    // -------------------------------------------------------------------------
    // Command 21: reserved alias must raise fatal error 50 (ERROR_BAD_ALIAS).
    // A reserved alias does NOT change the stored alias and does NOT reboot; the
    // device is left answering only 'Get status' and 'System reset' until reset.
    // -------------------------------------------------------------------------
    printf("Setting reserved alias %u (expecting fatal error %u)...\n",
           (unsigned)RESERVED_ALIAS, (unsigned)ERROR_BAD_ALIAS);
    motor->setDeviceAlias(RESERVED_ALIAS);
    delay(200);  // no reboot on the fatal path; brief settle only

    getStatusResponse st_fatal = motor->getStatus();
    printf("After reserved alias: getStatus error=%d flags=0x%04X fatal=%u\n",
           motor->getError(), st_fatal.statusFlags, (unsigned)st_fatal.fatalErrorCode);
    TEST_RESULT("Set device alias: reserved alias 254 raises fatal error 50",
                st_fatal.fatalErrorCode == ERROR_BAD_ALIAS);

    // Recover from the fatal state. The reserved-alias attempt did not persist,
    // so after reset the device is back at the (already-restored) original alias.
    motor->systemReset();
    delay(RESET_MS);

    detectDevicesResponse d4 = motor->detectDevices();
    int detectErr4 = motor->getError();
    printf("Detect #4 (post-recovery): error=%d uniqueId=0x%016llX alias=%u\n",
           detectErr4, (unsigned long long)d4.uniqueId, (unsigned)d4.alias);
    delay(DETECT_QUIET_MS);
    TEST_RESULT("Recovery: system reset clears fatal state; original alias intact",
                detectErr4 == 0 && d4.alias == originalAlias);

    // Leave the device in a clean, reset state at its original alias.
    motor->systemReset();
    delay(RESET_MS);
}
