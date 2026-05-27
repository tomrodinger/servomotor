#!/usr/bin/env python3
"""
Per-command test for "Reset time" (cmd 8).

Cmd 8 has no input and returns a success response. It zeroes the device's
absolute-microsecond clock — the same clock that cmd 9 (Get current time)
reads and that the move queue uses to time motion segments.

What this test verifies:
  1. Immediately after reset_time(), get_current_time() reads a value very
     close to zero — proving the reset actually took effect (a single round
     trip is sub-millisecond; even with verbose tracing the gap is well
     under 100 ms).
  2. The clock advances afterwards: a second read after a wall-clock sleep
     is materially larger than the first, and the elapsed delta is within a
     loose tolerance of the wall-clock sleep — proving reset_time() does
     not stop the clock, it only zeroes it.
  3. A second reset_time() after the clock has been running drops the
     reading back near zero — proving reset is repeatable, not a
     one-shot-after-boot oddity.

(Tightly coupled to cmd 9, which has its own dedicated test that focuses on
the monotonicity / unit-conversion properties of the time read itself.)
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

# The firmware returns the clock as a raw u64 microsecond count. To verify
# the command end-to-end without depending on the host library's per-product
# time-conversion table (which is set up for timesteps internal units, not
# microseconds — using e.g. time_unit="seconds" applies a misleading factor),
# this test reads the clock in time_unit="timesteps", whose factor is 1.0 in
# unit_conversions_M3.json — i.e. an identity passthrough that yields the raw
# microsecond count straight from the firmware.
RESET_DELAY_S = 1.5
NEAR_ZERO_US = 300_000       # post-reset read must be under this (~300 ms)
                             # one round trip is ~3 ms; verbose tracing can stretch it
SLEEP_S = 1.0                # wall-clock sleep between reads
ELAPSED_TOLERANCE_US = 200_000  # |measured µs - wall-clock µs| must be within this


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Reset time' (cmd 8).")
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
        # time_unit="timesteps" gives an identity passthrough — see the comment block
        # above. Numbers below are in raw microseconds.
        motor = servomotor.M3(args.alias, time_unit="timesteps", position_unit="encoder_counts",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("Calling reset_time(); reading current time; must be near zero µs...")
            motor.reset_time()
            t_after_reset = motor.get_current_time()
            print(f"  get_current_time after reset_time: {t_after_reset} µs")
            if not isinstance(t_after_reset, (int, float)):
                raise AssertionError(f"get_current_time returned non-numeric: {t_after_reset!r}")
            if t_after_reset < 0:
                raise AssertionError(f"get_current_time returned negative value {t_after_reset}")
            if t_after_reset > NEAR_ZERO_US:
                raise AssertionError(f"after reset_time the clock reads {t_after_reset} µs, "
                                     f"expected < {NEAR_ZERO_US} µs")

            print(f"Sleeping {SLEEP_S} s, then re-reading; clock must have advanced ~{int(SLEEP_S*1e6)} µs...")
            wall_start = _monotonic_us()
            time.sleep(SLEEP_S)
            t_later = motor.get_current_time()
            wall_now = _monotonic_us()
            if hasattr(time, 'ticks_diff'):
                wall_elapsed_us = time.ticks_diff(wall_now, wall_start)
            else:
                wall_elapsed_us = wall_now - wall_start
            motor_elapsed_us = t_later - t_after_reset
            print(f"  get_current_time after sleep: {t_later} µs (motor elapsed {motor_elapsed_us:.0f} µs, "
                  f"wall elapsed {wall_elapsed_us:.0f} µs)")
            if t_later <= t_after_reset:
                raise AssertionError(f"clock did not advance: t1={t_after_reset}, t2={t_later}")
            if abs(motor_elapsed_us - wall_elapsed_us) > ELAPSED_TOLERANCE_US:
                raise AssertionError(f"motor elapsed ({motor_elapsed_us:.0f} µs) disagrees with wall "
                                     f"({wall_elapsed_us:.0f} µs) by more than {ELAPSED_TOLERANCE_US} µs")

            print("Calling reset_time() again; clock must drop back near zero µs...")
            motor.reset_time()
            t_after_reset2 = motor.get_current_time()
            print(f"  get_current_time after second reset_time: {t_after_reset2} µs")
            if t_after_reset2 > NEAR_ZERO_US:
                raise AssertionError(f"after second reset_time the clock reads {t_after_reset2} µs, "
                                     f"expected < {NEAR_ZERO_US} µs")
            if t_after_reset2 >= t_later:
                raise AssertionError(f"second reset_time did not drop the clock: "
                                     f"pre={t_later}, post={t_after_reset2}")

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
