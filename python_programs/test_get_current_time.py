#!/usr/bin/env python3
"""
Per-command test for "Get current time" (cmd 9).

Cmd 9 has no input and returns u64 — the device's absolute time (the same
clock that reset_time / cmd 8 zeroes). It is the clock the move queue uses
to time motion segments, so it has to be monotonic, plausibly close to
real wall-clock time, and consistent across unit conversions.

This test reads the clock in the library's default time unit, "seconds".
The library converts clock commands correctly in ANY time unit (an earlier
library version applied a wrong conversion factor for the clock commands,
which forced this test to use a timesteps identity-passthrough workaround;
that bug is fixed).

What this test verifies:
  1. Returns a non-negative numeric scalar (not None / not a list).
  2. Monotonic: a sequence of back-to-back reads is non-decreasing.
  3. Tracks wall-clock: a measured elapsed of SLEEP_S seconds reads back as
     SLEEP_S within a loose absolute tolerance — proves the clock is real,
     not a counter that ticks at the command rate.
  4. Unit conversion is physically correct: reading the same clock via
     set_time_unit() in seconds, milliseconds, microseconds and timesteps
     yields the physically correct ratios (milliseconds ≈ seconds × 1000,
     microseconds ≈ seconds × 1e6, timesteps ≈ seconds × 31250, since one
     timestep = 32 µs) — proves the time unit layer in the M3 wrapper is
     wired up correctly for this command.

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

# Since the library fix, clock-command time conversions are correct in all
# units; this test uses the library default, time_unit="seconds". The
# cross-unit check at the end exercises set_time_unit() and asserts the
# physically correct ratios between units (one timestep = 32 µs, i.e.
# 31250 timesteps per second).
RESET_DELAY_S = 1.5
SLEEP_S = 1.0
ELAPSED_TOLERANCE_S = 0.2        # |measured s - wall-clock s| must be within this
N_MONOTONIC_SAMPLES = 20
TIMESTEPS_PER_SECOND = 31250.0   # one timestep = 32 µs


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
        # time_unit="seconds" is the library default; conversions are correct in all
        # units since the library fix.
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device and resetting the device clock...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.reset_time()

            print("Reading once; must be a non-negative scalar (seconds)...")
            t0 = motor.get_current_time()
            print(f"  initial read: {t0} s")
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
            print(f"  {N_MONOTONIC_SAMPLES} samples non-decreasing, last = {prev} s")

            print(f"Sleeping {SLEEP_S} s; measured elapsed must match wall-clock "
                  f"within {ELAPSED_TOLERANCE_S} s...")
            t_before = motor.get_current_time()
            wall_start = _monotonic_us()
            time.sleep(SLEEP_S)
            t_after = motor.get_current_time()
            # ticks_diff handles wraparound on MicroPython; on CPython
            # _monotonic_us returns a regular int that we subtract directly.
            wall_now = _monotonic_us()
            if hasattr(time, 'ticks_diff'):
                wall_elapsed_s = time.ticks_diff(wall_now, wall_start) / 1_000_000.0
            else:
                wall_elapsed_s = (wall_now - wall_start) / 1_000_000.0
            motor_elapsed_s = t_after - t_before
            print(f"  motor elapsed {motor_elapsed_s:.3f} s, wall elapsed {wall_elapsed_s:.3f} s")
            if motor_elapsed_s <= 0:
                raise AssertionError(f"motor clock did not advance over sleep "
                                     f"(before={t_before}, after={t_after})")
            if abs(motor_elapsed_s - wall_elapsed_s) > ELAPSED_TOLERANCE_S:
                raise AssertionError(f"motor elapsed ({motor_elapsed_s:.3f} s) disagrees with wall "
                                     f"({wall_elapsed_s:.3f} s) by more than {ELAPSED_TOLERANCE_S} s")

            # Cross-unit physical-correctness check: since the library fix, each time
            # unit converts the same device clock to its physically correct value —
            # milliseconds ≈ seconds × 1000, microseconds ≈ seconds × 1e6, and
            # timesteps ≈ seconds × 31250 (one timestep = 32 µs). The reads happen at
            # slightly different moments (one round trip on this bus is up to ~100 ms
            # of motor time), so we normalize every reading to seconds and assert the
            # sequence is non-decreasing with each successive gap under 0.5 s.
            # Asserting on absolute differences rather than ratios keeps the tolerance
            # independent of how long the motor has been running before this check.
            print("Cross-unit physical-correctness check: ms ≈ s×1000, µs ≈ s×1e6, "
                  "timesteps ≈ s×31250...")
            motor.set_time_unit("seconds")
            v_s = motor.get_current_time()
            motor.set_time_unit("milliseconds")
            v_ms = motor.get_current_time()
            motor.set_time_unit("microseconds")
            v_us = motor.get_current_time()
            motor.set_time_unit("timesteps")
            v_ts = motor.get_current_time()
            motor.set_time_unit("seconds")  # restore the library default
            normalized = [
                ("seconds", v_s, v_s),
                ("milliseconds", v_ms, v_ms / 1_000.0),
                ("microseconds", v_us, v_us / 1_000_000.0),
                ("timesteps", v_ts, v_ts / TIMESTEPS_PER_SECOND),
            ]
            for name, raw, norm in normalized:
                print(f"  {name}-unit read = {raw} -> normalized {norm:.4f} s")
            for (name_a, _, norm_a), (name_b, _, norm_b) in zip(normalized, normalized[1:]):
                gap_s = norm_b - norm_a  # must be >= 0; the second read is later
                if gap_s < 0:
                    raise AssertionError(f"{name_b} read normalized to {norm_b:.4f} s, which is "
                                         f"BEFORE the earlier {name_a} read ({norm_a:.4f} s) — "
                                         f"cross-unit conversion is broken")
                if gap_s > 0.5:  # 0.5 s of motor time is far more than any single round trip
                    raise AssertionError(f"{name_a}->{name_b} normalized gap of {gap_s:.4f} s "
                                         f"implies the cross-unit conversion is broken "
                                         f"(round-trip gap alone is ~100 ms)")

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
