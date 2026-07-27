#include "tf_framework.h"

// Mirrors the Python tests test_go_to_closed_loop.py and test_closed_loop_motion.py.
//
// Cmd 17 "Go to closed loop" takes an already-MOSFET-enabled (open loop) motor
// into closed-loop (field-oriented) control. It requires a CALIBRATED motor.
// We verify:
//   (A) Entry: before the command the closed-loop flag is 0; after it, the
//       closed-loop status flag (bit 2) sets, the MOSFETs-enabled flag (bit 1)
//       stays set, and there is no fatal error.
//   (B) Motion: a small closed-loop trapezoid move lands on the commanded target,
//       proving closed-loop control is actually functional (not just a flag flip).
//
// If the motor is NOT calibrated, go_to_closed_loop raises a fatal error / the
// closed-loop flag never sets. We DETECT that via getStatus()/getError(), record
// a clear FAIL, systemReset, and STILL exit cleanly (never hang).

// Status flag bit masks (see firmware status word)
static const uint16_t BOOTLOADER_BIT      = 1 << 0;
static const uint16_t MOSFETS_ENABLED_BIT = 1 << 1;
static const uint16_t CLOSED_LOOP_BIT     = 1 << 2;

static const int   CLOSED_LOOP_TIMEOUT_MS = 6000;
static const int   POLL_PERIOD_MS         = 100;
// closed-loop holding is hall-based, not exact -> generous tolerance (~4000 counts)
static const float MOVE_TOLERANCE_COUNTS  = 4000.0f;

void tm_closed_loop(void) {
    Servomotor* motor = tfGetMotor();

    // --- Clean, known state ---
    motor->systemReset();
    delay(1500);  // motor reboots and does not respond during this window

    // Work in encoder counts / seconds for the motion check.
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // --- (A) Enter closed loop ---
    // Enable MOSFETs (open loop) first.
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // settle after enable

    getStatusResponse open_status = motor->getStatus();
    checkMotorError(*motor, "getStatus (open loop)");
    printf("Open-loop status flags: 0x%04X, fatalError: %u\n",
           open_status.statusFlags, open_status.fatalErrorCode);

    TEST_RESULT("Not in bootloader (open loop)",
                (open_status.statusFlags & BOOTLOADER_BIT) == 0);
    TEST_RESULT("MOSFETs enabled before closed loop",
                (open_status.statusFlags & MOSFETS_ENABLED_BIT) != 0);
    TEST_RESULT("Closed-loop flag clear before go_to_closed_loop",
                (open_status.statusFlags & CLOSED_LOOP_BIT) == 0);

    // Issue the command and poll for the closed-loop flag.
    printf("\nIssuing goToClosedLoop and polling for the closed-loop flag...\n");
    motor->goToClosedLoop();
    // Deliberately do NOT checkMotorError here: on an uncalibrated motor this may
    // raise a fatal error, which we must detect and report gracefully, not exit.

    uint16_t flags = 0;
    uint8_t  fatalError = 0;
    bool     closedLoopEntered = false;
    bool     fatalDetected = false;
    int      elapsed = 0;
    while (elapsed < CLOSED_LOOP_TIMEOUT_MS) {
        getStatusResponse s = motor->getStatus();
        flags = s.statusFlags;
        fatalError = s.fatalErrorCode;
        if (motor->getError() != 0 || fatalError != 0) {
            fatalDetected = true;
            break;
        }
        if (flags & CLOSED_LOOP_BIT) {
            closedLoopEntered = true;
            break;
        }
        delay(POLL_PERIOD_MS);
        elapsed += POLL_PERIOD_MS;
    }
    printf("After procedure: flags=0x%04X, fatalError=%u, getError=%d\n",
           flags, fatalError, motor->getError());

    if (fatalDetected) {
        // Motor almost certainly not calibrated. Report clearly, do not hang.
        printf("\n*** FATAL ERROR during goToClosedLoop (fatalError=%u, getError=%d).\n"
               "*** This usually means the motor is NOT CALIBRATED. "
               "Recovering with systemReset and exiting cleanly.\n",
               fatalError, motor->getError());
        TEST_RESULT("No fatal error during go_to_closed_loop (motor calibrated?)", false);

        motor->systemReset();
        delay(1500);
        return;
    }

    TEST_RESULT("No fatal error during go_to_closed_loop (motor calibrated?)", true);
    TEST_RESULT("Closed-loop flag set after go_to_closed_loop", closedLoopEntered);
    TEST_RESULT("MOSFETs-enabled flag still set after go_to_closed_loop",
                (flags & MOSFETS_ENABLED_BIT) != 0);

    if (!closedLoopEntered) {
        // Closed loop never engaged (e.g. uncalibrated) but no explicit fatal error.
        printf("\n*** Closed-loop flag never set within %d ms. "
               "Is the motor calibrated? Recovering and exiting.\n", CLOSED_LOOP_TIMEOUT_MS);
        motor->systemReset();
        delay(1500);
        return;
    }

    // --- (B) Small closed-loop move lands on target ---
    printf("\nRunning a small closed-loop trapezoid move to confirm control is functional...\n");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");

    const float target = (float)(COUNTS_PER_REVOLUTION / 8);  // ~1/8 rotation, gentle
    const float duration = 1.0f;                              // seconds
    printf("Trapezoid move to %.0f counts over %.1f s...\n", target, duration);
    motor->trapezoidMove(target, duration);
    checkMotorError(*motor, "trapezoidMove");

    // Wait for the queue to drain (trapezoid move ends at rest).
    int waited = 0;
    while (motor->getNQueuedItems() > 0 && waited < 4000) {
        delay(50);
        waited += 50;
    }
    delay(300);  // settle

    float pos = motor->getPosition();
    checkMotorError(*motor, "getPosition");

    getStatusResponse after = motor->getStatus();
    checkMotorError(*motor, "getStatus (after move)");
    printf("Closed-loop move ended at %.0f counts (target ~%.0f), flags=0x%04X\n",
           pos, target, after.statusFlags);

    TEST_RESULT("Still in closed loop after the move",
                (after.statusFlags & CLOSED_LOOP_BIT) != 0);
    TEST_RESULT("Closed-loop move landed on target",
                approxEqual(pos, target, MOVE_TOLERANCE_COUNTS));

    // --- Clean up: leave motor at rest, disabled, reset ---
    motor->disableMosfets();
    motor->systemReset();
    delay(1500);
}
