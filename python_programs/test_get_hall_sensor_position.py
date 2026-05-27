#!/usr/bin/env python3
"""
Per-command test for "Get hall sensor position" (cmd 15).

Cmd 15 has no input and returns i64 — the position measured by the hall
sensors (the *actual* physical position), in encoder_counts internally
(1 shaft rotation = 3,276,800 counts). On a calibrated motor with no
missed steps the hall position closely follows the desired position
reported by cmd 34 (Get position) — the JSON description explicitly says
"this should be the actual position of the motor and if everything is ok
then it will be about the same as the desired position".

What this test verifies:
  1. After reset + enable + zero_position the hall reading is near zero
     (within hall-jitter tolerance) in every unit.
  2. The same hall reading taken in different position units
     (encoder_counts / shaft_rotations / degrees / radians) converts
     consistently — proves the command's value and the unit conversion
     layer agree.
  3. After an open-loop trapezoid move to a known target the hall position
     moved substantially (proves it's live, not a stuck/constant value)
     and tracks the desired position from cmd 34 within a tolerance that
     allows for hall jitter and ordinary open-loop lag — proves the
     command's documented purpose.

This test stays in open loop because cmd 15 is meaningful even there: the
hall sensor measures real motion regardless of control mode, and an
open-loop move with sufficient current and a calibrated commutation table
follows the trajectory closely enough for a meaningful comparison.
"""

import argparse
import math
import sys
import time
import servomotor

COUNTS_PER_ROTATION = 3276800
RESET_DELAY_S = 1.5
ENABLE_SETTLE_S = 0.3
ZERO_TOLERANCE_COUNTS = 500       # hall jitter near rest is small but nonzero
TRACK_TOLERANCE_COUNTS = 20000    # hall vs desired after a 1/8-turn open-loop move (~0.6% of a rotation)
MIN_MOTION_COUNTS = COUNTS_PER_ROTATION // 20  # hall must have visibly moved
# The unit-consistency check below reads hall four times (once per unit), so the
# "spread" mixes float round-trip error with real hall jitter across sequential
# reads. A wholly broken unit conversion would show > 1 rotation of disagreement;
# 1000 counts (0.03% of a rotation) is loose enough for jitter, tight enough to
# catch a real conversion regression.
UNIT_SPREAD_TOLERANCE_COUNTS = 1000


def assert_units_consistent(motor, label):
    """Read the same hall position in 4 units; assert they all agree (in counts)."""
    motor.set_position_unit("encoder_counts")
    counts = motor.get_hall_sensor_position()
    motor.set_position_unit("shaft_rotations")
    rotations = motor.get_hall_sensor_position()
    motor.set_position_unit("degrees")
    degrees = motor.get_hall_sensor_position()
    motor.set_position_unit("radians")
    radians = motor.get_hall_sensor_position()

    rotations_as_counts = rotations * COUNTS_PER_ROTATION
    degrees_as_counts = (degrees / 360.0) * COUNTS_PER_ROTATION
    radians_as_counts = (radians / (2 * math.pi)) * COUNTS_PER_ROTATION

    spread = max(abs(counts - rotations_as_counts),
                 abs(counts - degrees_as_counts),
                 abs(counts - radians_as_counts))
    print(f"  {label}: counts={counts}, rot={rotations}, deg={degrees}, rad={radians} "
          f"(unit spread={spread:.1f} counts)")
    if spread > UNIT_SPREAD_TOLERANCE_COUNTS:
        raise AssertionError(f"{label}: unit conversions disagree by {spread:.1f} counts "
                             f"(> {UNIT_SPREAD_TOLERANCE_COUNTS}) — conversion regression?")
    motor.set_position_unit("encoder_counts")
    return counts


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get hall sensor position' (cmd 15).")
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

            print("\nResetting and enabling MOSFETs (open loop)...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            # Let the rotor finish its commutation-alignment transient *before*
            # zeroing — otherwise zero_position runs while the rotor is still
            # physically moving and the hall accumulator drifts away from 0
            # over the settle window. (Standalone runs hide this because the
            # rotor starts close to a commutation step; suite runs catch it
            # because earlier tests can leave the rotor at any angle.)
            time.sleep(ENABLE_SETTLE_S)
            motor.zero_position()
            time.sleep(0.1)  # short post-zero settle

            print("Reading hall position at zero (must read ~0 in all units)...")
            hall_at_zero = assert_units_consistent(motor, "at zero")
            if abs(hall_at_zero) > ZERO_TOLERANCE_COUNTS:
                raise AssertionError(f"hall position at rest is {hall_at_zero} counts, "
                                     f"expected within ±{ZERO_TOLERANCE_COUNTS}")

            target_counts = COUNTS_PER_ROTATION // 8  # 1/8 turn
            print(f"Open-loop move to {target_counts} counts (1/8 rotation); then compare hall vs desired...")
            motor.set_position_unit("encoder_counts")
            motor.trapezoid_move(target_counts, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            time.sleep(0.3)  # let the motor settle physically

            desired = motor.get_position()
            hall = motor.get_hall_sensor_position()
            tracking_err = abs(hall - desired)
            motion = abs(hall - hall_at_zero)
            print(f"  desired={desired}, hall={hall} (tracking err={tracking_err} counts, "
                  f"motion since zero={motion} counts)")
            if motion < MIN_MOTION_COUNTS:
                raise AssertionError(f"hall barely moved ({motion} counts < {MIN_MOTION_COUNTS}); "
                                     f"is the command reporting a live value?")
            if tracking_err > TRACK_TOLERANCE_COUNTS:
                raise AssertionError(f"hall ({hall}) does not track desired ({desired}); "
                                     f"err {tracking_err} > {TRACK_TOLERANCE_COUNTS}")

            assert_units_consistent(motor, "after 1/8-turn move")

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
