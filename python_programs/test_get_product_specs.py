#!/usr/bin/env python3
"""
Per-command test for "Get product specs" (cmd 18).

Cmd 18 has no input and returns two u32 fields:
  - updateFrequency   — how often the motor control loop runs, in Hz
  - countsPerRotation — encoder counts per one shaft rotation

What this test verifies:
  1. The response is a two-element list of integers.
  2. updateFrequency is positive and in a plausible range (motor control
     loops run in the tens of kHz on these motors — the M17 is 31250 Hz;
     the bound here is wide enough that any sane value passes and any
     wildly wrong value fails).
  3. countsPerRotation equals exactly the value the host Python library
     uses (3,276,800 = the "shaft_rotations" conversion factor from
     servomotor/unit_conversions_*.json). A mismatch here would silently
     misconvert every position the rest of the suite reads or writes, so
     this cross-check is worth its own assertion.
  4. The values are static: a second read returns the exact same numbers.
"""

import argparse
import json
import os
import sys
import time
import servomotor

RESET_DELAY_S = 1.5
MIN_UPDATE_FREQUENCY_HZ = 1000      # generous lower bound
MAX_UPDATE_FREQUENCY_HZ = 200000    # generous upper bound
EXPECTED_COUNTS_PER_ROTATION = 3276800


def load_lib_counts_per_rotation():
    """Read the 'shaft_rotations' conversion factor the host Python library uses.
    If multiple unit-conversion files exist (M1/M2/M3/M17) we cross-check that
    they all agree, then return that common value."""
    # Resolve servomotor package directory portably (works on CPython on any
    # OS, and on MicroPython where `os.path` does not exist).
    pkg_file = servomotor.__file__
    sep_idx = max(pkg_file.rfind('/'), pkg_file.rfind('\\'))
    base = pkg_file[:sep_idx] if sep_idx >= 0 else '.'
    sep = pkg_file[sep_idx] if sep_idx >= 0 else '/'
    factors = set()
    for name in os.listdir(base):
        if name.startswith("unit_conversions_") and name.endswith(".json"):
            with open(base + sep + name) as fh:
                d = json.load(fh)
            factors.add(d["conversion_factors"]["shaft_rotations"])
    if not factors:
        raise AssertionError("could not locate any servomotor/unit_conversions_*.json")
    if len(factors) > 1:
        raise AssertionError(f"unit_conversions_*.json files disagree on shaft_rotations factor: {factors}")
    return int(factors.pop())


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get product specs' (cmd 18).")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    lib_counts_per_rotation = load_lib_counts_per_rotation()
    print(f"Host Python library uses {lib_counts_per_rotation} counts/rotation "
          f"(must match what the firmware reports).")

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

            print("Reading product specs...")
            specs = motor.get_product_specs()
            print(f"  get_product_specs -> {specs}")
            if not isinstance(specs, list) or len(specs) != 2:
                raise AssertionError(f"expected a 2-element list, got: {specs!r}")
            update_freq, counts_per_rotation = specs
            if not isinstance(update_freq, int) or not isinstance(counts_per_rotation, int):
                raise AssertionError(f"expected two integers, got types "
                                     f"{type(update_freq).__name__}, {type(counts_per_rotation).__name__}")

            print(f"  updateFrequency = {update_freq} Hz")
            if not (MIN_UPDATE_FREQUENCY_HZ <= update_freq <= MAX_UPDATE_FREQUENCY_HZ):
                raise AssertionError(f"updateFrequency {update_freq} Hz out of plausible range "
                                     f"[{MIN_UPDATE_FREQUENCY_HZ}, {MAX_UPDATE_FREQUENCY_HZ}]")

            print(f"  countsPerRotation = {counts_per_rotation}")
            if counts_per_rotation != EXPECTED_COUNTS_PER_ROTATION:
                raise AssertionError(f"countsPerRotation {counts_per_rotation} != "
                                     f"expected constant {EXPECTED_COUNTS_PER_ROTATION}")
            if counts_per_rotation != lib_counts_per_rotation:
                raise AssertionError(f"firmware countsPerRotation ({counts_per_rotation}) does not "
                                     f"match host library shaft_rotations factor "
                                     f"({lib_counts_per_rotation}); unit conversions will be wrong")

            print("Re-reading; values must be identical (static)...")
            specs2 = motor.get_product_specs()
            print(f"  second read -> {specs2}")
            if specs2 != specs:
                raise AssertionError(f"product specs changed between reads: {specs} -> {specs2}")

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
