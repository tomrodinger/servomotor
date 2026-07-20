#!/usr/bin/env python3
"""
Per-command test for "Reset time" (cmd 8).

Cmd 8 has no input and returns a success response. It zeroes the device's
absolute clock — the same clock that cmd 9 (Get current time) reads and
that the move queue uses to time motion segments.

This test reads the clock in the library's default time unit, "seconds".
The library converts clock commands correctly in ANY time unit (an earlier
library version applied a wrong conversion factor for the clock commands,
which forced this test to use a timesteps identity-passthrough workaround;
that bug is fixed).

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

# Since the library fix, clock-command time conversions are correct in all
# units; this test uses the library default, time_unit="seconds".
RESET_DELAY_S = 1.5
NEAR_ZERO_S = 0.3            # post-reset read must be under this (~300 ms)
                             # one round trip is ~3 ms; verbose tracing can stretch it
SLEEP_S = 1.0                # wall-clock sleep between reads
ELAPSED_TOLERANCE_S = 0.2    # |measured s - wall-clock s| must be within this


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
        # time_unit="seconds" is the library default; conversions are correct in all
        # units since the library fix. Numbers below are in seconds.
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("Calling reset_time(); reading current time; must be near zero seconds...")
            motor.reset_time()
            t_after_reset = motor.get_current_time()
            print(f"  get_current_time after reset_time: {t_after_reset} s")
            if not isinstance(t_after_reset, (int, float)):
                raise AssertionError(f"get_current_time returned non-numeric: {t_after_reset!r}")
            if t_after_reset < 0:
                raise AssertionError(f"get_current_time returned negative value {t_after_reset}")
            if t_after_reset > NEAR_ZERO_S:
                raise AssertionError(f"after reset_time the clock reads {t_after_reset} s, "
                                     f"expected < {NEAR_ZERO_S} s")

            print(f"Sleeping {SLEEP_S} s, then re-reading; clock must have advanced ~{SLEEP_S} s...")
            wall_start = _monotonic_us()
            time.sleep(SLEEP_S)
            t_later = motor.get_current_time()
            wall_now = _monotonic_us()
            if hasattr(time, 'ticks_diff'):
                wall_elapsed_s = time.ticks_diff(wall_now, wall_start) / 1_000_000.0
            else:
                wall_elapsed_s = (wall_now - wall_start) / 1_000_000.0
            motor_elapsed_s = t_later - t_after_reset
            print(f"  get_current_time after sleep: {t_later} s (motor elapsed {motor_elapsed_s:.3f} s, "
                  f"wall elapsed {wall_elapsed_s:.3f} s)")
            if t_later <= t_after_reset:
                raise AssertionError(f"clock did not advance: t1={t_after_reset}, t2={t_later}")
            if abs(motor_elapsed_s - wall_elapsed_s) > ELAPSED_TOLERANCE_S:
                raise AssertionError(f"motor elapsed ({motor_elapsed_s:.3f} s) disagrees with wall "
                                     f"({wall_elapsed_s:.3f} s) by more than {ELAPSED_TOLERANCE_S} s")

            print("Calling reset_time() again; clock must drop back near zero seconds...")
            motor.reset_time()
            t_after_reset2 = motor.get_current_time()
            print(f"  get_current_time after second reset_time: {t_after_reset2} s")
            if t_after_reset2 > NEAR_ZERO_S:
                raise AssertionError(f"after second reset_time the clock reads {t_after_reset2} s, "
                                     f"expected < {NEAR_ZERO_S} s")
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
