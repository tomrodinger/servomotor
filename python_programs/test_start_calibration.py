#!/usr/bin/env python3
"""
Per-command test for "Start calibration" (cmd 6).

Calibration spins the motor and captures hall-sensor data so closed-loop
control will work; without it, closed-loop misbehaves. The command has no
input, returns a success response immediately, then the firmware spins
the rotor through several cycles. While it runs, status bit 3
(STATUS_CALIBRATING_FLAG_BIT) is set; when it finishes cleanly the bit
clears and MOSFETs are turned off (see motor_control.c:start_calibration
and handle_calibration_logic).

Calibration writes the hall-sensor reference values into the settings
page; that data is what every later closed-loop test depends on. This is
the reason TEST_SUMMARY.md orders this test BEFORE every closed-loop and
position-dependent test (the firmware upgrade test is the only one that
runs earlier).

What this test verifies:
  1. Pre-call: device is clean (no fatal error, calibrating bit clear,
     MOSFETs disabled).
  2. Issue start_calibration → success response, calibrating bit (bit 3)
     becomes set within a short window.
  3. Calibrating bit eventually clears within CALIBRATION_TIMEOUT_S with
     no fatal error logged (the firmware fatals ERROR_HALL_SENSOR (9),
     ERROR_CALIBRATION_OVERFLOW (10), etc. on real problems).
  4. After completion: MOSFETs are off, motor is not busy, status flags
     are clean.
  5. End-to-end usability: go_to_closed_loop succeeds and a small
     closed-loop move lands on target — i.e. the new calibration data is
     usable, not just "computed without crashing".

Important: when calibration finishes the firmware auto-reboots the MCU.
The polling `get_status` that detects the calibrating-bit clear arrives
during the bootloader's post-reset window, so the device is pinned in
the bootloader (`get_status` returns 0x0001) — that is normal, not a
regression. We push it back into the application with a `system_reset`
+ long delay before continuing. See the [[calibration-auto-reboots]]
memory note.

Heavy: the M17 calibration takes ~30-60 s; budget CALIBRATION_TIMEOUT_S
generously. This test (together with test_firmware_upgrade.py) is the
slowest in the suite, and that was approved in the 2026-05-19 review.
"""

import argparse
import sys
import time

import servomotor

BOOTLOADER_BIT       = 1 << 0
MOSFETS_ENABLED_BIT  = 1 << 1
CLOSED_LOOP_BIT      = 1 << 2
CALIBRATING_BIT      = 1 << 3
HOMING_BIT           = 1 << 4
GO_TO_CLOSED_LOOP_BIT = 1 << 5
MOTOR_BUSY_BIT       = 1 << 6

RESET_DELAY_S          = 1.5
ENABLE_SETTLE_S        = 0.3
CALIBRATION_START_TIMEOUT_S = 3.0   # bit 3 must light up within this window
CALIBRATION_TIMEOUT_S       = 120.0 # generous: M17 typically completes in ~30-60s
POLL_INTERVAL_S        = 1.0
# Calibration ends with an automatic MCU reboot — the poll that catches the
# calibrating-bit clear lands inside the bootloader window and pins the device
# there. We push it back into the application with a system_reset and this
# delay. Matches DONT_GO_TO_BOOTlOADER_RESET_TIME used elsewhere.
POST_CALIBRATION_REBOOT_DELAY_S = 2.0
CLOSED_LOOP_TIMEOUT_S  = 6.0
COUNTS_PER_ROTATION    = 3276800
CLOSED_LOOP_TOLERANCE_COUNTS = 4000


def get_status_raw(motor):
    """Read status without interpreting flags. Returns (flags, err) or raises if unparseable."""
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"get_status returned unusable value: {status!r}")
    return status[0], status[1]


def get_status_or_fail(motor, label):
    """Read status, fail fast on bootloader / fatal error / unparseable response."""
    flags, err = get_status_raw(motor)
    if flags & BOOTLOADER_BIT:
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#06x})")
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err}, expected 0 (flags={flags:#06x})")
    return flags, err


def wait_for_calibration_to_start(motor):
    deadline = time.time() + CALIBRATION_START_TIMEOUT_S
    while time.time() < deadline:
        flags, _ = get_status_or_fail(motor, "waiting for calibrating bit to set")
        if flags & CALIBRATING_BIT:
            return flags
        time.sleep(0.1)
    raise AssertionError(f"calibrating bit (bit 3) never set within {CALIBRATION_START_TIMEOUT_S}s "
                         f"of start_calibration; cmd may not have entered the calibration state")


def wait_for_calibration_to_finish(motor):
    """Poll until the calibrating bit clears. Tolerate the post-reboot bootloader
    state — see the [[calibration-auto-reboots]] memory note: when calibration
    completes the MCU resets, and our very next poll may hit the bootloader
    window (status=0x0001). Treat that as "calibration finished" too.
    """
    deadline = time.time() + CALIBRATION_TIMEOUT_S
    started = time.time()
    last_print = 0.0
    while time.time() < deadline:
        flags, _ = get_status_raw(motor)   # tolerate bootloader / errors here
        elapsed = time.time() - started
        if elapsed - last_print >= 5.0:
            print(f"  ...still calibrating after {elapsed:5.1f}s (flags={flags:#06x})")
            last_print = elapsed
        if flags & BOOTLOADER_BIT:
            print(f"  device is in the bootloader after {elapsed:.1f}s (flags={flags:#06x}); "
                  f"calibration finished and auto-rebooted")
            return flags
        if not (flags & CALIBRATING_BIT):
            print(f"  calibrating bit cleared after {elapsed:.1f}s (flags={flags:#06x})")
            return flags
        time.sleep(POLL_INTERVAL_S)
    raise AssertionError(f"calibration did not finish within {CALIBRATION_TIMEOUT_S}s "
                         f"(calibrating bit still set)")


def kick_out_of_bootloader_if_needed(motor):
    """If the post-calibration auto-reboot left us in the bootloader, system_reset
    + long delay so the application starts cleanly."""
    flags, _ = get_status_raw(motor)
    if flags & BOOTLOADER_BIT:
        print(f"  device is in the bootloader (flags={flags:#06x}); "
              f"sending system_reset and waiting {POST_CALIBRATION_REBOOT_DELAY_S}s for app to start...")
        motor.system_reset()
        time.sleep(POST_CALIBRATION_REBOOT_DELAY_S)


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Start calibration' (cmd 6).")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    success = False
    failure_message = ""
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              verbose=verbose_level)

        print("\nResetting the device into a clean open-loop state...")
        motor.system_reset()
        time.sleep(RESET_DELAY_S)
        flags, _ = get_status_or_fail(motor, "pre-calibration")
        print(f"  pre-call flags={flags:#06x}")
        if flags & CALIBRATING_BIT:
            raise AssertionError("calibrating bit is set BEFORE start_calibration was issued")
        if flags & MOSFETS_ENABLED_BIT:
            raise AssertionError("MOSFETs are enabled before start_calibration; "
                                 "firmware would refuse with ERROR_MOTOR_BUSY/etc.")

        print("Issuing start_calibration (success response expected, then calibration runs)...")
        motor.start_calibration()

        flags = wait_for_calibration_to_start(motor)
        print(f"  calibrating bit set (flags={flags:#06x}); polling until it clears "
              f"(timeout {CALIBRATION_TIMEOUT_S:.0f}s)...")

        flags = wait_for_calibration_to_finish(motor)

        # Calibration auto-reboots the MCU; the device may be in the bootloader.
        # Push it back into the application before doing anything else.
        kick_out_of_bootloader_if_needed(motor)

        # Post-calibration sanity: MOSFETs off, motor not busy, no fatal error.
        print("Verifying post-calibration state...")
        flags, _ = get_status_or_fail(motor, "post-calibration")
        print(f"  post-calibration flags={flags:#06x}")
        if flags & CALIBRATING_BIT:
            raise AssertionError(f"calibrating bit still set after wait loop (flags={flags:#06x})")
        if flags & MOSFETS_ENABLED_BIT:
            raise AssertionError(f"MOSFETs still enabled after calibration "
                                 f"(firmware should disable_mosfets() on completion; "
                                 f"flags={flags:#06x})")
        if flags & MOTOR_BUSY_BIT:
            raise AssertionError(f"motor_busy bit still set after calibration (flags={flags:#06x})")

        # End-to-end usability: closed loop + a small move must work.
        print("Confirming the new calibration is usable: enabling and entering closed loop...")
        motor.enable_mosfets()
        time.sleep(ENABLE_SETTLE_S)
        motor.go_to_closed_loop()

        deadline = time.time() + CLOSED_LOOP_TIMEOUT_S
        in_closed_loop = False
        while time.time() < deadline:
            flags, _ = get_status_or_fail(motor, "during go_to_closed_loop")
            if flags & CLOSED_LOOP_BIT:
                in_closed_loop = True
                break
            time.sleep(0.1)
        if not in_closed_loop:
            raise AssertionError(f"go_to_closed_loop did not set the closed-loop bit within "
                                 f"{CLOSED_LOOP_TIMEOUT_S}s — calibration data may be invalid")
        print(f"  in closed loop (flags={flags:#06x})")

        motor.zero_position()
        target = COUNTS_PER_ROTATION // 8
        print(f"Running a small closed-loop move of +{target} counts to confirm control is functional...")
        motor.trapezoid_move(target, 1.0)
        while motor.get_n_queued_items() > 0:
            time.sleep(0.01)
        time.sleep(0.3)
        pos = motor.get_position()
        flags, _ = get_status_or_fail(motor, "after closed-loop move")
        print(f"  ended at {pos} (target ~{target}), flags={flags:#06x}")
        if not (flags & CLOSED_LOOP_BIT):
            raise AssertionError("dropped out of closed loop during the verification move")
        if abs(pos - target) > CLOSED_LOOP_TOLERANCE_COUNTS:
            raise AssertionError(f"closed-loop move ended at {pos}, expected ~{target} "
                                 f"(diff {abs(pos - target)} > {CLOSED_LOOP_TOLERANCE_COUNTS}); "
                                 f"calibration data may be poor")

        print("Cleaning up: disable, reset.")
        motor.disable_mosfets()
        motor.system_reset()
        time.sleep(RESET_DELAY_S)

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        if motor is not None:
            try:
                motor.disable_mosfets()
            except Exception:
                pass
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
