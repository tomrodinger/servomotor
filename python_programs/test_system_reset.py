#!/usr/bin/env python3
"""
Per-command test for "System reset" (cmd 27).

System reset reboots the device's MCU. Given a long-enough settle delay the
device comes back up in the application (not the bootloader). A reset must
return the device to a clean baseline: application mode, no fatal error,
MOSFETs disabled, and any latched fatal-error state cleared.

What this test verifies:
  1. After reset the device is in application mode (status bit 0 clear) and
     reports no fatal error.
  2. Reset clears live state: after enabling MOSFETs (bit 1 set), a reset
     brings bit 1 back to 0.
  3. Reset clears a *latched fatal error*. We deliberately drive the device
     into a fatal-error state (narrow safety limits + an out-of-zone move,
     which the firmware rejects with a safety-zone fatal error), confirm the
     error is latched, then system_reset and confirm fatal_error returns to 0
     and the device is responsive again. This is the property that matters
     most — the documented way out of a fatal-error state.
"""

import argparse
import sys
import time
import servomotor
from servomotor.communication import FatalError

BOOTLOADER_BIT = 1 << 0
MOSFETS_ENABLED_BIT = 1 << 1
RESET_DELAY_S = 1.5

SAFETY_ZONE_ERROR_CODES = {25, 26, 27}  # SAFETY_LIMIT / TURN_POINT / PREDICTED out of zone
NARROW_LIMIT_ROT = 0.5
OUT_OF_ZONE_MOVE_ROT = 5.0
OUT_OF_ZONE_MOVE_TIME_S = 2.0


def get_flags_err(motor, label):
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    return status[0], status[1]


def assert_clean_baseline(motor, label):
    flags, err = get_flags_err(motor, label)
    print(f"  {label}: flags={flags:#06x}, fatal_error={err}")
    if flags & BOOTLOADER_BIT:
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#06x})")
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err}, expected 0")
    if flags & MOSFETS_ENABLED_BIT:
        raise AssertionError(f"{label}: MOSFETs enabled after reset, expected disabled")
    return flags, err


def drive_into_fatal_error(motor):
    """Use narrow safety limits + an out-of-zone move to latch a fatal error."""
    motor.set_position_unit("shaft_rotations")
    motor.set_safety_limits(-NARROW_LIMIT_ROT, NARROW_LIMIT_ROT)
    try:
        motor.trapezoid_move(OUT_OF_ZONE_MOVE_ROT, OUT_OF_ZONE_MOVE_TIME_S)
    except FatalError:
        pass  # firmware may raise immediately; the latched state is checked below
    time.sleep(0.2)
    flags, err = get_flags_err(motor, "after out-of-zone move")
    print(f"  after out-of-zone move: flags={flags:#06x}, fatal_error={err}")
    if err not in SAFETY_ZONE_ERROR_CODES:
        raise AssertionError(f"could not latch a fatal error to test reset against; "
                             f"fatal_error={err}, expected one of {sorted(SAFETY_ZONE_ERROR_CODES)}")
    return err


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'System reset' (cmd 27).")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    success = False
    failure_message = ""
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="shaft_rotations",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\n[1] Reset and verify clean baseline...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_clean_baseline(motor, "after first reset")

            print("[2] Enable MOSFETs, then reset must clear the enabled state...")
            motor.enable_mosfets()
            flags, _ = get_flags_err(motor, "after enable_mosfets")
            if not (flags & MOSFETS_ENABLED_BIT):
                raise AssertionError("enable_mosfets did not set the MOSFET-enabled bit; "
                                     "cannot test that reset clears it")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_clean_baseline(motor, "after reset following enable")

            print("[3] Latch a fatal error, then reset must clear it...")
            err = drive_into_fatal_error(motor)
            print(f"      latched fatal_error={err}; issuing system_reset...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_clean_baseline(motor, "after reset clearing fatal error")

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        # Best-effort: leave the device clean regardless of outcome.
        try:
            servomotor.M3(args.alias, verbose=0).system_reset()
            time.sleep(RESET_DELAY_S)
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
