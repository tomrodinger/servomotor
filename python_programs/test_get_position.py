#!/usr/bin/env python3
"""
Per-command test for "Get position" (cmd 34).

Cmd 34 output: (i64 position) — the current *desired* position, in
encoder_counts internally (1 shaft rotation = 3,276,800 counts).

What this test verifies:
  1. After reset + zero_position the reported position is 0 in every unit.
  2. The same underlying position read back in different units
     (encoder_counts / shaft_rotations / degrees / radians) is mutually
     consistent — proving the command's value and the unit conversion layer
     both work.
  3. After a real move to a known target, get_position reports that target
     (within a small tolerance) — proving it reflects live desired position,
     not a constant.

This is the "desired position" command, not the hall-sensed position, so it
matches the commanded target closely and does not require closed-loop mode —
enabling the MOSFETs for an open-loop trapezoid move is enough.
"""

import argparse
import math
import sys
import time
import servomotor

COUNTS_PER_ROTATION = 3276800
RESET_DELAY_S = 1.5
TOLERANCE_COUNTS = 50  # desired position should land essentially exactly on target


def get_position_counts(motor):
    motor.set_position_unit("encoder_counts")
    return motor.get_position()


def assert_units_consistent(motor, label):
    """Read the same position in 4 units; assert they all agree (in counts)."""
    motor.set_position_unit("encoder_counts")
    counts = motor.get_position()
    motor.set_position_unit("shaft_rotations")
    rotations = motor.get_position()
    motor.set_position_unit("degrees")
    degrees = motor.get_position()
    motor.set_position_unit("radians")
    radians = motor.get_position()

    rotations_as_counts = rotations * COUNTS_PER_ROTATION
    degrees_as_counts = (degrees / 360.0) * COUNTS_PER_ROTATION
    radians_as_counts = (radians / (2 * math.pi)) * COUNTS_PER_ROTATION

    spread = max(abs(counts - rotations_as_counts),
                 abs(counts - degrees_as_counts),
                 abs(counts - radians_as_counts))
    print(f"  {label}: counts={counts}, rot={rotations}, deg={degrees}, rad={radians} "
          f"(unit spread={spread:.1f} counts)")
    if spread > 1.0:
        raise AssertionError(f"{label}: unit conversions disagree by {spread:.1f} counts "
                             f"(counts={counts}, rot->{rotations_as_counts}, "
                             f"deg->{degrees_as_counts}, rad->{radians_as_counts})")
    return counts


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get position' (cmd 34).")
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
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("Enabling MOSFETs and zeroing position...")
            motor.enable_mosfets()
            motor.zero_position()

            print("Reading position at zero (must read 0 in all units)...")
            counts = assert_units_consistent(motor, "at zero")
            if abs(counts) > TOLERANCE_COUNTS:
                raise AssertionError(f"position after zero_position is {counts} counts, expected ~0")

            target_counts = COUNTS_PER_ROTATION // 4  # quarter turn
            print(f"Moving to {target_counts} counts (1/4 rotation) and re-reading...")
            motor.set_position_unit("encoder_counts")
            motor.trapezoid_move(target_counts, 1.0)
            # Wait for the move to finish.
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            time.sleep(0.1)

            counts = assert_units_consistent(motor, "after 1/4-turn move")
            if abs(counts - target_counts) > TOLERANCE_COUNTS:
                raise AssertionError(f"position after move is {counts} counts, expected ~{target_counts} "
                                     f"(diff {abs(counts - target_counts)} > {TOLERANCE_COUNTS})")

            print("Returning to zero and disabling MOSFETs...")
            motor.set_position_unit("encoder_counts")
            motor.trapezoid_move(0, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
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
