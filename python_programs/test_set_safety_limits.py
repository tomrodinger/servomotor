#!/usr/bin/env python3
"""
Per-command test for "Set safety limits" (cmd 30).

Modernized from the obsolete test_safety_limit.py — which despite its name
actually exercised the developer-only `ADD_TO_QUEUE_TEST_COMMAND` (cmd 120,
internal firmware-math verification), not cmd 30. This file replaces it with
a real per-command test of cmd 30, dropping the raw-`serial` usage.

Cmd 30 input: `(i64 lower_limit, i64 upper_limit)` in position units. Once
set, any subsequent move whose predicted trajectory would leave the
[lower, upper] zone is rejected by the firmware with one of:

  * ERROR_SAFETY_LIMIT_EXCEEDED              (code 25)
  * ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE      (code 26)
  * ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE (code 27)

What this test does:

  1. Reset; set narrow limits (±0.5 rotation); attempt a move to
     +5 rotations (10× the limit) — must be rejected with a safety-zone
     fatal error.
  2. Reset; set wide limits (±1000 rotations); attempt a small in-bounds
     move (+0.1 rotation) — must succeed and leave the device with no
     fatal error.

The motor does not need to physically execute the moves — the safety check
is on the predicted trajectory at command-receive time, so MOSFETs stay
disabled throughout (faster, no closed-loop / calibration dependency).
"""

import argparse
import sys
import time
import servomotor
from servomotor.communication import FatalError

STATUS_IN_THE_BOOTLOADER_FLAG_BIT = 0

ERROR_SAFETY_LIMIT_EXCEEDED = 25
ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE = 26
ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE = 27
SAFETY_ZONE_ERROR_CODES = {
    ERROR_SAFETY_LIMIT_EXCEEDED,
    ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE,
    ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE,
}

RESET_DELAY_S = 1.5
NARROW_LIMIT_ROT = 0.5            # ± 0.5 shaft rotations — small enough that even a small move blows past it
OUT_OF_ZONE_MOVE_ROT = 5.0        # destination 10× the narrow limit
OUT_OF_ZONE_MOVE_TIME_S = 2.0     # 2-second move (plenty for the firmware to predict)
WIDE_LIMIT_ROT = 1000.0           # effectively unlimited for a small in-bounds move
IN_BOUNDS_MOVE_ROT = 0.1          # 0.1 rotation, comfortably within wide limits
IN_BOUNDS_MOVE_TIME_S = 0.5


def assert_app_mode_no_error(motor, label):
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err} (expected 0). flags={flags:#x}")
    if flags & (1 << STATUS_IN_THE_BOOTLOADER_FLAG_BIT):
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#x}, expected application mode)")
    return flags, err


def expect_safety_zone_rejection(motor, move_value_rot, move_time_s, label):
    """Send a trapezoid_move that should be rejected by the safety zone.
    Accept either a raised FatalError or a fatal error code reported via
    get_status (the firmware path can produce either depending on timing)."""
    try:
        motor.trapezoid_move(move_value_rot, move_time_s)
    except FatalError as e:
        code = e.args[0] if e.args else None
        if code in SAFETY_ZONE_ERROR_CODES:
            print(f"  {label}: move correctly rejected with FatalError({code}).")
            return
        raise AssertionError(f"{label}: move rejected with FatalError({code}); expected one of {sorted(SAFETY_ZONE_ERROR_CODES)}.")

    # The command did not raise. Maybe the firmware accepted the encoding
    # but is now in the fatal-error state — check.
    time.sleep(0.2)
    status = motor.get_status()
    if not status or len(status) < 2:
        raise AssertionError(f"{label}: trapezoid_move returned without exception; get_status afterwards is {status!r}")
    err = status[1]
    if err in SAFETY_ZONE_ERROR_CODES:
        print(f"  {label}: device entered fatal_error({err}) as expected.")
        return
    if err == 0:
        raise AssertionError(f"{label}: move was NOT rejected — safety limit is not being enforced. status={status!r}")
    raise AssertionError(f"{label}: device fatal_error={err}, expected one of {sorted(SAFETY_ZONE_ERROR_CODES)}.")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Set safety limits' (cmd 30).")
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
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="shaft_rotations",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            # --- Scenario A: narrow limits reject an out-of-zone move ---
            print("\nResetting device to clear any pre-existing state...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_app_mode_no_error(motor, "before narrow set_safety_limits")

            print(f"Setting narrow safety limits: [{-NARROW_LIMIT_ROT}, +{NARROW_LIMIT_ROT}] rotations.")
            motor.set_safety_limits(-NARROW_LIMIT_ROT, NARROW_LIMIT_ROT)
            assert_app_mode_no_error(motor, "after set_safety_limits (narrow)")

            print(f"Attempting trapezoid_move({OUT_OF_ZONE_MOVE_ROT}, {OUT_OF_ZONE_MOVE_TIME_S}s) — predicted end well outside the limit, must be rejected.")
            expect_safety_zone_rejection(motor, OUT_OF_ZONE_MOVE_ROT, OUT_OF_ZONE_MOVE_TIME_S,
                                         "narrow limits + out-of-zone move")

            # --- Scenario B: wide limits accept an in-bounds move ---
            print("\nResetting device to clear the safety-zone fatal error...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_app_mode_no_error(motor, "after reset clearing fatal error")

            print(f"Setting wide safety limits: [{-WIDE_LIMIT_ROT}, +{WIDE_LIMIT_ROT}] rotations.")
            motor.set_safety_limits(-WIDE_LIMIT_ROT, WIDE_LIMIT_ROT)
            assert_app_mode_no_error(motor, "after set_safety_limits (wide)")

            print(f"Attempting in-bounds trapezoid_move(+{IN_BOUNDS_MOVE_ROT}, {IN_BOUNDS_MOVE_TIME_S}s) — must be accepted.")
            try:
                motor.trapezoid_move(IN_BOUNDS_MOVE_ROT, IN_BOUNDS_MOVE_TIME_S)
            except FatalError as e:
                raise AssertionError(f"In-bounds move was rejected with FatalError({e.args[0] if e.args else None}); expected to be accepted.")
            time.sleep(IN_BOUNDS_MOVE_TIME_S + 0.3)
            assert_app_mode_no_error(motor, "after in-bounds move")

            # Leave a clean state for whatever runs next.
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
