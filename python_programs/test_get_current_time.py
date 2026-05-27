#!/usr/bin/env python3
"""
Per-command test for "Get current time" (cmd 9).

Cmd 9 has no input and returns u64 — the device's absolute time in
microseconds (the same clock that reset_time / cmd 8 zeroes). It is the
clock the move queue uses to time motion segments, so it has to be
monotonic, plausibly close to real wall-clock time, and consistent across
unit conversions.

What this test verifies:
  1. Returns a non-negative numeric scalar (not None / not a list).
  2. Monotonic: a sequence of back-to-back reads is non-decreasing.
  3. Tracks wall-clock: a measured elapsed of SLEEP_S seconds reads back as
     SLEEP_S within a loose absolute tolerance — proves the clock is real,
     not a counter that ticks at the command rate.
  4. Unit conversion is self-consistent: reading in seconds and in
     microseconds at the same moment converts (microseconds / 1e6 ≈
     seconds) within a small absolute tolerance — proves the time unit
     layer in the M3 wrapper is wired up correctly for this command.

(Tightly coupled to cmd 8, which has its own dedicated test for the reset
semantics of the same clock.)
"""

import argparse
import sys
import time
import servomotor


def _monotonic_us():
    """Microsecond-resolution monotonic clock that works on CPython and
    MicroPython (which lacks time.monotonic but has time.ticks_us)."""
    if hasattr(time, 'ticks_us'):
        return time.ticks_us()
    return int(time.monotonic() * 1_000_000)

# The firmware returns the clock as a raw u64 microsecond count. The host
# library's per-product time-conversion table is set up for an internal unit
# of "timesteps" (factor 1.0 in unit_conversions_M3.json), so we read the
# clock in time_unit="timesteps" to get an identity passthrough — the raw
# microsecond value straight from the firmware. The internal-consistency
# check at the end exercises set_time_unit() across two units that share that
# same conversion table, so their relative ratio is still meaningful.
RESET_DELAY_S = 1.5
SLEEP_S = 1.0
ELAPSED_TOLERANCE_US = 200_000   # |measured µs - wall-clock µs| must be within this
N_MONOTONIC_SAMPLES = 20


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get current time' (cmd 9).")
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
        # time_unit="timesteps" → identity passthrough; see the comment block above.
        motor = servomotor.M3(args.alias, time_unit="timesteps", position_unit="encoder_counts",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device and resetting the device clock...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.reset_time()

            print("Reading once; must be a non-negative scalar (µs)...")
            t0 = motor.get_current_time()
            print(f"  initial read: {t0} µs")
            if not isinstance(t0, (int, float)):
                raise AssertionError(f"get_current_time returned non-numeric: {t0!r}")
            if t0 < 0:
                raise AssertionError(f"get_current_time returned negative value {t0}")

            print(f"Reading {N_MONOTONIC_SAMPLES} samples back-to-back; sequence must be non-decreasing...")
            prev = t0
            for i in range(N_MONOTONIC_SAMPLES):
                t = motor.get_current_time()
                if t < prev:
                    raise AssertionError(f"non-monotonic: sample {i} went backwards "
                                         f"({prev} -> {t})")
                prev = t
            print(f"  {N_MONOTONIC_SAMPLES} samples non-decreasing, last = {prev} µs")

            print(f"Sleeping {SLEEP_S} s; measured elapsed must match wall-clock "
                  f"within {ELAPSED_TOLERANCE_US} µs...")
            t_before = motor.get_current_time()
            wall_start = _monotonic_us()
            time.sleep(SLEEP_S)
            t_after = motor.get_current_time()
            # ticks_diff handles wraparound on MicroPython; on CPython
            # _monotonic_us returns a regular int that we subtract directly.
            wall_now = _monotonic_us()
            if hasattr(time, 'ticks_diff'):
                wall_elapsed_us = time.ticks_diff(wall_now, wall_start)
            else:
                wall_elapsed_us = wall_now - wall_start
            motor_elapsed_us = t_after - t_before
            print(f"  motor elapsed {motor_elapsed_us:.0f} µs, wall elapsed {wall_elapsed_us:.0f} µs")
            if motor_elapsed_us <= 0:
                raise AssertionError(f"motor clock did not advance over sleep "
                                     f"(before={t_before}, after={t_after})")
            if abs(motor_elapsed_us - wall_elapsed_us) > ELAPSED_TOLERANCE_US:
                raise AssertionError(f"motor elapsed ({motor_elapsed_us:.0f} µs) disagrees with wall "
                                     f"({wall_elapsed_us:.0f} µs) by more than {ELAPSED_TOLERANCE_US} µs")

            # Cross-unit consistency: the conversion table assigns "microseconds" factor
            # 0.03125 and "seconds" factor 31250, so the two are 1e6 apart regardless of
            # which raw unit the firmware actually reports. Reading the same clock in
            # both units must satisfy us_value ≈ s_value * 1e6, with the only slack
            # coming from the wall-clock gap between the two reads (one round trip on
            # this bus is up to ~100 ms = 0.1 s of motor time). Asserting on the
            # absolute difference rather than the ratio makes this tolerance independent
            # of how long the motor has been running before this check.
            print("Cross-unit consistency check: us reading must be ≈ 1e6 × s reading...")
            motor.set_time_unit("seconds")
            v_s = motor.get_current_time()
            motor.set_time_unit("microseconds")
            v_us = motor.get_current_time()
            motor.set_time_unit("timesteps")  # restore identity passthrough
            diff_us = v_us - v_s * 1_000_000.0  # always > 0; second read is later
            print(f"  seconds-unit read = {v_s}, microseconds-unit read = {v_us}; "
                  f"v_us - v_s*1e6 = {diff_us:.0f} µs")
            if diff_us < 0:
                raise AssertionError(f"second read went backwards in time: diff = {diff_us} µs")
            if diff_us > 500_000:  # 0.5 s of motor time is far more than any single round trip
                raise AssertionError(f"us-vs-s gap of {diff_us:.0f} µs implies the cross-unit "
                                     f"conversion is broken (round-trip gap alone is ~100 ms)")

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
