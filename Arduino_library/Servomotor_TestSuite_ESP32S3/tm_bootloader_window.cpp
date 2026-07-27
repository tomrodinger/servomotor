#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_bootloader_window.cpp
//
// Exercises the 250 ms BOOTLOADER LAUNCH WINDOW protocol path on a single
// device (unique-ID addressed, self-healing) WITHOUT flashing anything.
//
// FIRMWARE GROUND TRUTH (verified in source, not guessed):
//   * After 'System reset' the app calls NVIC_SystemReset(); the device reboots
//     into the bootloader. The bootloader (bootloader_STM32G031/Src/main.c):
//       - line 345: set_device_status_flags(1 << STATUS_IN_THE_BOOTLOADER_FLAG_BIT)
//                   => Get status reports status bit 0 SET while in the bootloader.
//       - line 362: launch_applicaiton = LAUNCH_APPLICATION_DELAY (25 systicks at
//                   100 Hz = 250 ms) => the app auto-launches after 250 ms of silence.
//       - line 137: launch_applicaiton = -1 for EVERY valid command received
//                   => the FIRST valid-CRC packet addressed to the device cancels
//                   the launch and PERMANENTLY pins it in the bootloader until
//                   another 'System reset' or a power cycle.
//       - GET_STATUS (case ~140): returns struct device_status_struct (flags:u16,
//                   error_code:u8) exactly as getStatusResponse; in the bootloader
//                   flags == 0x0001 (bit 0 only; device_status.h documents that all
//                   other flags are 0 while in the bootloader) and error_code == 0.
//       - GET_FIRMWARE_VERSION (case ~231): sets in_bootloader = 1 (the APP sets it
//                   to 0, firmware/Src/main.c line 604).
//   * STATUS_IN_THE_BOOTLOADER_FLAG_BIT == 0  (common_source_files/device_status.h).
//   * Timing: python_programs/upgrade_firmware.py WAIT_FOR_RESET_TIME = 0.07 s,
//     "empirically ... between 0.002 and 0.13" after sending reset the bootloader
//     is listening and the window is still open. We catch at ~50 ms: safely inside.
//   * systemReset() in the library reads its own success ack (Servomotor.cpp), so
//     no stale ack lingers; we still drain Serial1 on misses.
//
// SEQUENCE:
//   (a) systemReset by uid, then within ~50 ms send Get status by uid: expect
//       status bit 0 SET (in bootloader). The early packet cancels the app launch
//       and pins the device. Retry the catch up to 3 times if the timing misses.
//   (b) while pinned: Get firmware version reports inBootloader == 1, and Get
//       status still shows bit 0 set (the pin is permanent).
//   (c) systemReset again, wait 1600 ms (> 250 ms, sending nothing): the app
//       relaunches. Verify application mode (bit 0 CLEAR), inBootloader == 0, no
//       fatal error, and that a gentle move is accepted and drains to rest.
//
// SAFETY:
//   * Every command is unique-ID (extended) addressed via tfGetMotor(); no
//     alias-88 or single-alias frame is ever emitted, so this is collision-safe
//     on the 39-motor rack. NO raw Serial1.write() frames (we only READ Serial1
//     to drain). systemReset() by uid is a single-target reset (not broadcast).
//   * No test-mode (cmd 36) values are used anywhere (never 10..13).
//   * The only motion is one tiny trapezoid (0.1 rot over 0.5 s) that ends at
//     rest by design; well under the 1-rot / 3-rot-s gentle guideline.
//   * Every wait loop is millis()-bounded.
// ---------------------------------------------------------------------------

static const int      NORMAL_RESET_DELAY_MS = 1500; // wait after a plain reset (app relaunch)
static const int      APP_RELAUNCH_DELAY_MS = 1600; // (c): > 250 ms window, so app launches
static const int      CATCH_DELAY_MS        = 50;   // send the catch packet inside the 250 ms window
static const int      MISS_SETTLE_MS        = 400;  // let the app finish booting after a missed catch
static const int      MAX_CATCH_ATTEMPTS    = 3;

static const uint16_t STATUS_BIT_IN_BOOTLOADER = 0x0001; // STATUS_IN_THE_BOOTLOADER_FLAG_BIT == 0
static const float    MOVE_ROT              = 0.1f; // gentle proof-of-life move
static const float    MOVE_DUR              = 0.5f; // seconds

// Drain any bytes sitting in the RS485 receive buffer (READ ONLY; never writes).
static void drainSerial() {
    unsigned long t0 = millis();
    while (Serial1.available()) {
        Serial1.read();
        if (millis() - t0 > 500) break; // bounded
    }
}

// Bring the device to a clean, known state: reset, wait out the window so the
// app relaunches, then drain.
static void cleanReset(Servomotor* motor) {
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS); // no meaningful response during reset
    drainSerial();
}

// Read Get status; returns false on a comms-layer failure.
static bool readStatus(Servomotor* motor, uint16_t* flagsOut, uint8_t* codeOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return false;
    }
    *flagsOut = st.statusFlags;
    *codeOut  = st.fatalErrorCode;
    return true;
}

// (a) Catch the device inside the bootloader launch window: reset, then send
// Get status ~50 ms later (well inside the 250 ms window). On success the launch
// is cancelled and the device is permanently pinned in the bootloader. Retry up
// to MAX_CATCH_ATTEMPTS if the timing misses. Returns true if caught; the caught
// status flags/code are written out.
static bool catchBootloaderWindow(Servomotor* motor, uint16_t* flagsOut, uint8_t* codeOut) {
    for (int attempt = 1; attempt <= MAX_CATCH_ATTEMPTS; attempt++) {
        motor->systemReset();      // reboots into bootloader; ack is read by systemReset()
        delay(CATCH_DELAY_MS);     // land the catch packet inside the 250 ms window
        getStatusResponse st = motor->getStatus();
        if (motor->getError() == 0 && (st.statusFlags & STATUS_BIT_IN_BOOTLOADER)) {
            printf("  catch attempt %d: PINNED, flags=0x%04X code=%d\n",
                   attempt, (unsigned)st.statusFlags, (int)st.fatalErrorCode);
            *flagsOut = st.statusFlags;
            *codeOut  = st.fatalErrorCode;
            return true;
        }
        printf("  catch attempt %d missed: err=%d flags=0x%04X\n",
               attempt, motor->getError(), (unsigned)st.statusFlags);
        // Missed: either the packet was lost mid-reboot, or the app already
        // launched. Let it settle, drain, and try a fresh window.
        delay(MISS_SETTLE_MS);
        drainSerial();
    }
    return false;
}

// Poll the motion queue until it empties (millis()-bounded), then settle.
static uint8_t waitForIdle(Servomotor* motor) {
    uint8_t n = 0xFF;
    unsigned long t0 = millis();
    while (millis() - t0 < 8000) {
        n = motor->getNQueuedItems();
        if (motor->getError() == 0 && n == 0) break;
        delay(10);
    }
    delay(150); // settle margin
    return n;
}

// Prove application-level responsiveness by round-tripping a ping payload.
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

void tm_bootloader_window(void) {
    Serial.println("test_ard_bootloader_window: BEGIN\n");

    Servomotor* motor = tfGetMotor();
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    // -----------------------------------------------------------------------
    // Clean, known starting state (application mode).
    // -----------------------------------------------------------------------
    cleanReset(motor);

    uint16_t flags = 0xFFFF;
    uint8_t  code  = 0xFF;
    bool baseOk = readStatus(motor, &flags, &code);
    printf("Baseline: commsOk=%d flags=0x%04X code=%d\n", (int)baseOk, (unsigned)flags, (int)code);
    // In application mode bit 0 (in-bootloader) must be CLEAR.
    TEST_RESULT("BW Baseline: application mode at start (bit 0 clear, no fatal)",
                baseOk && (flags & STATUS_BIT_IN_BOOTLOADER) == 0 && code == 0);

    // =======================================================================
    // (a) Catch the device inside the 250 ms bootloader launch window.
    // =======================================================================
    printf("\n--- (a) catch the bootloader launch window ---\n");
    uint16_t bootFlags = 0xFFFF;
    uint8_t  bootCode  = 0xFF;
    bool caught = catchBootloaderWindow(motor, &bootFlags, &bootCode);
    printf("Caught=%d flags=0x%04X code=%d\n", (int)caught, (unsigned)bootFlags, (int)bootCode);

    // Get status bit 0 SET => device pinned in the bootloader (firmware line 345).
    TEST_RESULT("BW (a) Caught device in bootloader window (status bit 0 set)",
                caught && (bootFlags & STATUS_BIT_IN_BOOTLOADER) != 0);
    // device_status.h: while in the bootloader all other status flags are 0,
    // so the flags word must be exactly 0x0001.
    TEST_RESULT("BW (a) In bootloader, only bit 0 set (flags == 0x0001)",
                caught && bootFlags == STATUS_BIT_IN_BOOTLOADER);
    // Fresh bootloader boot => no latched fatal error (error_codes.json: code 0 == ERROR_NONE).
    TEST_RESULT("BW (a) In bootloader, no fatal error code (== 0)",
                caught && bootCode == 0);

    // =======================================================================
    // (b) While pinned: Get firmware version reports inBootloader == 1, and the
    //     pin is permanent (Get status still shows bit 0 set).
    // =======================================================================
    printf("\n--- (b) query while pinned in the bootloader ---\n");
    bool fvBootOk = false;
    uint8_t fvBootFlag = 0xFF;
    if (caught) {
        getFirmwareVersionResponse fv = motor->getFirmwareVersion();
        if (motor->getError() == 0) {
            fvBootOk = true;
            fvBootFlag = fv.inBootloader;
        } else {
            printf("  getFirmwareVersion comms error %d\n", motor->getError());
        }
    }
    printf("Firmware version while pinned: commsOk=%d inBootloader=%d\n",
           (int)fvBootOk, (int)fvBootFlag);
    // Bootloader sets in_bootloader = 1 (bootloader main.c ~line 240).
    TEST_RESULT("BW (b) getFirmwareVersion reports inBootloader == 1 while pinned",
                fvBootOk && fvBootFlag == 1);

    // The pin is permanent (launch already cancelled): status must still show bit 0.
    uint16_t stillFlags = 0xFFFF;
    uint8_t  stillCode  = 0xFF;
    bool stillOk = caught && readStatus(motor, &stillFlags, &stillCode);
    printf("Still pinned: commsOk=%d flags=0x%04X\n", (int)stillOk, (unsigned)stillFlags);
    TEST_RESULT("BW (b) Still pinned in bootloader after queries (bit 0 set)",
                stillOk && (stillFlags & STATUS_BIT_IN_BOOTLOADER) != 0);

    // =======================================================================
    // (c) systemReset again + wait out the window: the application relaunches.
    // =======================================================================
    printf("\n--- (c) reset out of the bootloader back into the application ---\n");
    motor->systemReset();
    delay(APP_RELAUNCH_DELAY_MS); // > 250 ms with no packets => app auto-launches
    drainSerial();

    uint16_t appFlags = 0xFFFF;
    uint8_t  appCode  = 0xFF;
    bool appOk = readStatus(motor, &appFlags, &appCode);
    printf("After relaunch: commsOk=%d flags=0x%04X code=%d\n",
           (int)appOk, (unsigned)appFlags, (int)appCode);
    // Application mode => bit 0 CLEAR.
    TEST_RESULT("BW (c) Back in application mode after reset (bit 0 clear)",
                appOk && (appFlags & STATUS_BIT_IN_BOOTLOADER) == 0);
    TEST_RESULT("BW (c) No fatal error after returning to application",
                appOk && appCode == 0);

    // Get firmware version now reports the APPLICATION (in_bootloader == 0, main.c line 604).
    getFirmwareVersionResponse fvApp = motor->getFirmwareVersion();
    bool fvAppOk = (motor->getError() == 0);
    printf("Firmware version in app: commsOk=%d inBootloader=%d\n",
           (int)fvAppOk, (int)fvApp.inBootloader);
    TEST_RESULT("BW (c) getFirmwareVersion reports inBootloader == 0 in application",
                fvAppOk && fvApp.inBootloader == 0);

    // Prove the application is fully functional: a gentle move is accepted, runs,
    // and drains to rest with no fatal error.
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300); // let the commutation-alignment transient settle

    motor->trapezoidMove(MOVE_ROT, MOVE_DUR);
    bool moveCommsOk = (motor->getError() == 0);

    uint8_t drained = waitForIdle(motor);
    uint16_t postFlags = 0xFFFF;
    uint8_t  postCode  = 0xFF;
    bool postOk = readStatus(motor, &postFlags, &postCode);
    printf("After move: moveCommsOk=%d drained=%d fatalCode=%d\n",
           (int)moveCommsOk, (int)drained, (int)postCode);
    // Move accepted (no comms error) AND no fatal error latched during execution.
    TEST_RESULT("BW (c) Gentle move accepted in application (no comms/fatal error)",
                moveCommsOk && postOk && postCode == 0);
    TEST_RESULT("BW (c) Motion queue drained to zero after the move", drained == 0);

    // Final application-level liveness check.
    TEST_RESULT("BW (c) Device responsive (ping) after full bootloader cycle",
                pingRoundTrips(motor));

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest (application mode).
    // -----------------------------------------------------------------------
    cleanReset(motor);
}
