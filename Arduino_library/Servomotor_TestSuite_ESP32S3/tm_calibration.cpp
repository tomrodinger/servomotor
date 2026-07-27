#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_calibration.cpp
//
// Arduino-library on-device test for "Start calibration" (cmd 6).
// Mirrors python_programs/test_start_calibration.py.
//
// Calibration spins the rotor and captures hall-sensor reference data that
// every later closed-loop / position test depends on. The command returns a
// success response immediately, then the firmware spins the motor through
// several cycles while STATUS bit 3 (calibrating) is set. When it finishes
// cleanly the bit clears, MOSFETs are turned off -- and the firmware performs
// an AUTOMATIC MCU REBOOT.
//
// CRITICAL gotcha (verified on this hardware, see [[calibration-auto-reboots]]):
// the poll that detects the calibrating-bit clear can land inside the
// post-reset bootloader window (status = 0x0001). Any command sent during that
// window pins the device in the bootloader. So after the calibrating bit
// clears (or we see the bootloader bit), we delay(2500), then systemReset +
// delay(1500) to force the application to start cleanly BEFORE any other
// command.
//
// One motor on the bus, addressed by unique-ID (extended addressing) via
// tfGetMotor(), so all methods are called WITHOUT a uniqueId arg. This is
// collision-safe on the 39-motor rack; no broadcast query is ever issued.
//
// SAFETY: the only motion is a small (1/8-turn) closed-loop trapezoid move at
// the very end, which ends at rest by design. No moveWithVelocity, no test
// modes. Device is left reset/clean.
// ---------------------------------------------------------------------------

static const uint16_t BOOTLOADER_BIT      = 1 << 0;
static const uint16_t MOSFETS_ENABLED_BIT = 1 << 1;
static const uint16_t CLOSED_LOOP_BIT     = 1 << 2;
static const uint16_t CALIBRATING_BIT     = 1 << 3;
static const uint16_t MOTOR_BUSY_BIT      = 1 << 6;

static const int NORMAL_RESET_DELAY_MS = 1500;

// bit 3 must light up within this window after start_calibration.
static const unsigned long CALIBRATION_START_TIMEOUT_MS = 3000;
// Generous: M17 typically completes in ~30-60s.
static const unsigned long CALIBRATION_TIMEOUT_MS = 120000;
static const unsigned long POLL_INTERVAL_MS = 1000;
// After the calibrating bit clears the MCU auto-reboots; give the bootloader
// window time, then reset into the application.
static const int POST_CALIBRATION_REBOOT_DELAY_MS = 2500;

static const unsigned long CLOSED_LOOP_TIMEOUT_MS = 6000;
static const int64_t COUNTS_PER_ROTATION = COUNTS_PER_REVOLUTION;  // 3276800
static const int64_t CLOSED_LOOP_TOLERANCE_COUNTS = 4000;

// Read status flags. Sets ok=false if the query itself failed at the comms
// layer (expected while the device is mid-reboot). fatalCode is filled too.
static uint16_t readStatusFlags(Servomotor* motor, bool* ok, uint8_t* fatalCode) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        *ok = false;
        *fatalCode = 0xFF;
        return 0;
    }
    *ok = true;
    *fatalCode = st.fatalErrorCode;
    return st.statusFlags;
}

// Wait until the motion queue is empty, then settle.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 1000; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (motor->getError() == 0 && n == 0) break;
        delay(10);
    }
    delay(200);
}

void tm_calibration(void) {
    Serial.println("test_ard_calibration: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state: open loop, no fatal error, MOSFETs off.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no response during reset; don't check errors

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);

    bool ok = false;
    uint8_t fatalCode = 0xFF;
    uint16_t flags = readStatusFlags(motor, &ok, &fatalCode);
    printf("Pre-calibration: commsOk=%d flags=0x%04x fatal=%d\n",
           (int)ok, (unsigned)flags, (int)fatalCode);
    TEST_RESULT("Pre-calibration status readable", ok);
    TEST_RESULT("Pre-calibration no fatal error", ok && fatalCode == 0);
    TEST_RESULT("Pre-calibration not in bootloader", ok && !(flags & BOOTLOADER_BIT));
    TEST_RESULT("Pre-calibration: calibrating bit clear", ok && !(flags & CALIBRATING_BIT));
    TEST_RESULT("Pre-calibration: MOSFETs disabled", ok && !(flags & MOSFETS_ENABLED_BIT));

    // -----------------------------------------------------------------------
    // Issue start_calibration -> success response, then calibration runs.
    // -----------------------------------------------------------------------
    printf("\n--- Start calibration (cmd 6) ---\n");
    motor->startCalibration();
    bool startComms = (motor->getError() == 0);
    printf("startCalibration(): commsErr=%d\n", motor->getError());
    TEST_RESULT("startCalibration returns success (comms ok)", startComms);

    // (1) Calibrating bit (bit 3) must set within a short window.
    bool calibratingStarted = false;
    unsigned long t0 = millis();
    while (millis() - t0 < CALIBRATION_START_TIMEOUT_MS) {
        flags = readStatusFlags(motor, &ok, &fatalCode);
        if (ok && (flags & CALIBRATING_BIT)) {
            calibratingStarted = true;
            break;
        }
        delay(100);
    }
    printf("Calibrating bit set: %d (flags=0x%04x)\n",
           (int)calibratingStarted, (unsigned)flags);
    TEST_RESULT("Calibrating bit (bit 3) set after start_calibration", calibratingStarted);

    // (2) Poll until the calibrating bit clears. Tolerate the post-reboot
    // bootloader state: when calibration completes the MCU resets and our very
    // next poll may hit the bootloader window (flags=0x0001, or comms error) --
    // treat that as "calibration finished" too.
    bool calibrationFinished = false;
    t0 = millis();
    unsigned long lastPrint = 0;
    while (millis() - t0 < CALIBRATION_TIMEOUT_MS) {
        flags = readStatusFlags(motor, &ok, &fatalCode);
        unsigned long elapsed = millis() - t0;
        if (elapsed - lastPrint >= 5000) {
            printf("  ...still calibrating after %lus (commsOk=%d flags=0x%04x)\n",
                   elapsed / 1000, (int)ok, (unsigned)flags);
            lastPrint = elapsed;
        }
        if (!ok) {
            // Comms dropped -- almost certainly the auto-reboot in progress.
            printf("  status comms error after %lus; calibration likely finished + rebooting\n",
                   elapsed / 1000);
            calibrationFinished = true;
            break;
        }
        if (flags & BOOTLOADER_BIT) {
            printf("  in bootloader after %lus (flags=0x%04x); calibration finished + auto-rebooted\n",
                   elapsed / 1000, (unsigned)flags);
            calibrationFinished = true;
            break;
        }
        if (!(flags & CALIBRATING_BIT)) {
            printf("  calibrating bit cleared after %lus (flags=0x%04x)\n",
                   elapsed / 1000, (unsigned)flags);
            calibrationFinished = true;
            break;
        }
        delay(POLL_INTERVAL_MS);
    }
    TEST_RESULT("Calibration finished within timeout (no hall/overflow fatal)",
                calibrationFinished);

    // -----------------------------------------------------------------------
    // Post-calibration auto-reboot handling: delay for the bootloader window,
    // then systemReset + delay to force the application to start BEFORE any
    // other command.
    // -----------------------------------------------------------------------
    printf("\nHandling post-calibration auto-reboot...\n");
    delay(POST_CALIBRATION_REBOOT_DELAY_MS);
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);

    // Post-calibration sanity: application running, no fatal, MOSFETs off,
    // motor not busy, calibrating bit clear.
    flags = readStatusFlags(motor, &ok, &fatalCode);
    printf("Post-calibration: commsOk=%d flags=0x%04x fatal=%d\n",
           (int)ok, (unsigned)flags, (int)fatalCode);
    TEST_RESULT("Post-calibration status readable (app running)", ok);
    TEST_RESULT("Post-calibration no fatal error (calibration succeeded)",
                ok && fatalCode == 0);
    TEST_RESULT("Post-calibration not in bootloader", ok && !(flags & BOOTLOADER_BIT));
    TEST_RESULT("Post-calibration: calibrating bit clear", ok && !(flags & CALIBRATING_BIT));
    TEST_RESULT("Post-calibration: MOSFETs disabled", ok && !(flags & MOSFETS_ENABLED_BIT));
    TEST_RESULT("Post-calibration: motor not busy", ok && !(flags & MOTOR_BUSY_BIT));

    // -----------------------------------------------------------------------
    // End-to-end usability: the new calibration data must actually work.
    // Enable MOSFETs, enter closed loop, then a small closed-loop move must
    // land on target.
    // -----------------------------------------------------------------------
    printf("\n--- Verifying calibration is usable (closed loop) ---\n");
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle

    motor->goToClosedLoop();
    checkMotorError(*motor, "goToClosedLoop");

    bool inClosedLoop = false;
    t0 = millis();
    while (millis() - t0 < CLOSED_LOOP_TIMEOUT_MS) {
        flags = readStatusFlags(motor, &ok, &fatalCode);
        if (ok && fatalCode != 0) break;  // a fatal here means bad calibration
        if (ok && (flags & CLOSED_LOOP_BIT)) {
            inClosedLoop = true;
            break;
        }
        delay(100);
    }
    printf("In closed loop: %d (flags=0x%04x fatal=%d)\n",
           (int)inClosedLoop, (unsigned)flags, (int)fatalCode);
    TEST_RESULT("go_to_closed_loop sets closed-loop bit (calibration usable)",
                inClosedLoop);
    TEST_RESULT("No fatal error entering closed loop", ok && fatalCode == 0);

    // Small closed-loop move: +1/8 turn must land on target.
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    const int64_t target = COUNTS_PER_ROTATION / 8;
    printf("Closed-loop move +%lld counts...\n", (long long)target);
    motor->trapezoidMove((float)target, 1.0f);  // trapezoid ends at rest
    checkMotorError(*motor, "trapezoidMove");
    waitForIdle(motor);

    int64_t pos = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw");
    flags = readStatusFlags(motor, &ok, &fatalCode);
    printf("Ended at %lld (target ~%lld), flags=0x%04x fatal=%d\n",
           (long long)pos, (long long)target, (unsigned)flags, (int)fatalCode);
    TEST_RESULT("Still in closed loop after move", ok && (flags & CLOSED_LOOP_BIT));
    TEST_RESULT("No fatal error after closed-loop move", ok && fatalCode == 0);
    TEST_RESULT("Closed-loop move lands on target (calibration accurate)",
                llabs(pos - target) <= CLOSED_LOOP_TOLERANCE_COUNTS);

    // -----------------------------------------------------------------------
    // Clean up: disable, reset.
    // -----------------------------------------------------------------------
    printf("\nCleaning up: disable, reset.\n");
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
}
