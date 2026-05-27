#!/usr/bin/env python3
"""
Per-command test for "Multimove" (cmd 29).

Cmd 29 input: (u8 moveCount, u32 moveTypes, list_2d moveList). It queues up to
32 moves in one packet. Each bit of `moveTypes` selects the type of the
corresponding move: bit = 0 → move-with-acceleration, bit = 1 →
move-with-velocity. `moveList` is a list of [value, duration] pairs.

The high-level M3.multimove() wrapper cannot perform the mixed
acceleration/velocity/time unit conversion (its generic conversion path has no
`_mixed_..._unit` attribute — true on the committed library too), so the
command is driven here through the library's own
servomotor.communication.execute_command with values already expressed in the
firmware's *internal* units. Internal values are computed from the published
conversion factors (unit_conversions_M3.json), so no fixed-point constants are
hard-coded.

What this test verifies:
  1. A two-segment acceleration multimove (+A for t, then -A for t) is
     accepted, runs, drains the queue, leaves no fatal error, and ends at
     rest. With a symmetric accel/decel the net displacement is A*t^2
     (rotations), which the test checks against the measured position.
  2. A multimove declaring more than 32 moves is rejected with
     ERROR_MULTIMOVE_MORE_THAN_32_MOVES (24), proving the per-packet limit is
     enforced.
"""

import argparse
import json
import os
import sys
import time
import servomotor
from servomotor import communication
from servomotor.communication import FatalError

MULTIMOVE_COMMAND_ID = 29
ERROR_MULTIMOVE_MORE_THAN_32_MOVES = 24
COUNTS_PER_ROTATION = 3276800
RESET_DELAY_S = 1.5

ACCEL_ROT_S2 = 5.0      # acceleration magnitude, rotations/s^2
SEG_TIME_S = 0.5        # duration of each of the two segments, seconds
MAX_VEL = 10.0          # rotations/s   — headroom above peak velocity (A*t = 2.5)
MAX_ACCEL = 50.0        # rotations/s^2 — headroom above ACCEL_ROT_S2
TOLERANCE_COUNTS = 6000  # generous: the firmware discretises the profile


def load_factors():
    # Resolve servomotor package directory portably (works on CPython on any
    # OS, and on MicroPython where `os.path` does not exist).
    pkg_file = servomotor.__file__
    sep_idx = max(pkg_file.rfind('/'), pkg_file.rfind('\\'))
    here = pkg_file[:sep_idx] if sep_idx >= 0 else '.'
    sep = pkg_file[sep_idx] if sep_idx >= 0 else '/'
    cf = json.load(open(here + sep + "unit_conversions_M3.json"))["conversion_factors"]
    return cf


def get_err(motor):
    status = motor.get_status()
    return status[1] if status and len(status) >= 2 else None


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Multimove' (cmd 29).")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    cf = load_factors()
    accel_internal = round(ACCEL_ROT_S2 * cf["rotations_per_second_squared"])
    dur_internal = round(SEG_TIME_S * cf["seconds"])
    expected_counts = round(ACCEL_ROT_S2 * SEG_TIME_S * SEG_TIME_S * COUNTS_PER_ROTATION)

    success = False
    failure_message = ""
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              velocity_unit="rotations_per_second",
                              acceleration_unit="rotations_per_second_squared", verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting, enabling MOSFETs, zeroing, setting limits...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            motor.zero_position()
            motor.set_maximum_velocity(MAX_VEL)
            motor.set_maximum_acceleration(MAX_ACCEL)

            # --- Scenario A: a valid 2-segment acceleration multimove ---
            move_list = f"[[{accel_internal}, {dur_internal}], [{-accel_internal}, {dur_internal}]]"
            print(f"[A] multimove(2 moves, types=0, {move_list})")
            print(f"    expected net displacement: A*t^2 = {ACCEL_ROT_S2}*{SEG_TIME_S}^2 rot "
                  f"= {expected_counts} counts")
            communication.execute_command(MULTIMOVE_COMMAND_ID, [2, 0, move_list],
                                          alias_or_unique_id=motor.alias_or_unique_id,
                                          verbose=verbose_level)
            # Wait for the queue to drain.
            deadline = time.time() + (2 * SEG_TIME_S + 3.0)
            while time.time() < deadline:
                if motor.get_n_queued_items() == 0:
                    break
                time.sleep(0.02)
            if motor.get_n_queued_items() != 0:
                raise AssertionError("multimove did not drain the queue in time")

            err = get_err(motor)
            if err != 0:
                raise AssertionError(f"multimove left fatal_error={err}, expected 0")

            pos1 = motor.get_position()
            time.sleep(0.2)
            pos2 = motor.get_position()
            print(f"    ended at {pos1} counts (re-read {pos2}); expected ~{expected_counts}")
            if abs(pos1 - pos2) > TOLERANCE_COUNTS:
                raise AssertionError(f"motor not at rest after multimove ({pos1} vs {pos2})")
            if abs(pos1 - expected_counts) > TOLERANCE_COUNTS:
                raise AssertionError(f"multimove ended at {pos1}, expected ~{expected_counts} "
                                     f"(diff {abs(pos1 - expected_counts)} > {TOLERANCE_COUNTS})")
            print("    valid multimove executed correctly.")

            # --- Scenario B: more than 32 moves must be rejected ---
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            big_list = "[" + ", ".join("[0, 1000]" for _ in range(33)) + "]"
            print("[B] multimove declaring 33 moves — must be rejected with code 24...")
            rejected = False
            try:
                communication.execute_command(MULTIMOVE_COMMAND_ID, [33, 0, big_list],
                                              alias_or_unique_id=motor.alias_or_unique_id,
                                              verbose=verbose_level)
            except FatalError as e:
                code = e.args[0] if e.args else None
                if code != ERROR_MULTIMOVE_MORE_THAN_32_MOVES:
                    raise AssertionError(f"33-move multimove rejected with FatalError({code}); "
                                         f"expected {ERROR_MULTIMOVE_MORE_THAN_32_MOVES}")
                print(f"    correctly rejected with FatalError({code}) = ERROR_MULTIMOVE_MORE_THAN_32_MOVES.")
                rejected = True
            if not rejected:
                err = get_err(motor)
                if err == ERROR_MULTIMOVE_MORE_THAN_32_MOVES:
                    print(f"    correctly entered fatal_error({err}) = ERROR_MULTIMOVE_MORE_THAN_32_MOVES.")
                else:
                    raise AssertionError(f"33-move multimove was NOT rejected (fatal_error={err}).")

            # A latched fatal error makes every subsequent command echo that error,
            # so reset FIRST (which also disables the MOSFETs) before anything else.
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
