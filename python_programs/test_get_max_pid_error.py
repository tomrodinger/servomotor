#!/usr/bin/env python3
"""
Per-command test for "Get max PID error" (cmd 39).

Cmd 39 has no input and returns two i32 fields: the *minimum* and
*maximum* PID controller error observed since the last read. Reading
also resets the trackers: the firmware seeds min := INT32_MAX and
max := INT32_MIN immediately after extracting the values, so the next
PID-loop iteration starts a fresh observation window.

Key firmware property: the min/max trackers are only updated inside
PID_controller(), which only runs in closed-loop mode. Between a fresh
system_reset and entering closed loop, no PID iterations have happened
— so the read returns the seed sentinel [INT32_MAX, INT32_MIN] in
that order (i.e. min > max, meaning "no data").

What this test verifies:
  1. The response is a 2-element list of numeric values.
  2. After a fresh system_reset (no closed-loop entry, no PID activity)
     the values are exactly the sentinel [INT32_MAX, INT32_MIN]. This is
     the canonical "no samples observed yet" state and is testable
     because it is deterministic — any other return value means the
     min/max init / update logic regressed.
  3. After entering closed loop and executing a small move, the next
     read returns a *real* range: both values are finite int32, no
     longer the sentinel, and min ≤ max. This proves the trackers are
     wired up to PID_controller() and capture actual control error.
  4. Read-then-reset semantics: a second read taken immediately after
     the post-closed-loop read returns a *different* min/max window
     than the first (because the trackers were reset by the first
     read). On a calibrated motor holding position the new window is
     also a real, narrower range — proves the reset side-effect of the
     read actually fires.

This test needs a calibrated motor (closed loop requires it). The
bench M17 is calibrated per WORK_CHECKLIST.md.
"""

import argparse
import sys
import time
import servomotor

COUNTS_PER_ROTATION = 3276800
INT32_MAX = 2147483647
INT32_MIN = -2147483648
CLOSED_LOOP_BIT = 1 << 2
RESET_DELAY_S = 1.5
ENABLE_SETTLE_S = 0.3
CLOSED_LOOP_TIMEOUT_S = 6.0
PLAUSIBLE_ERROR_BOUND_COUNTS = COUNTS_PER_ROTATION  # |error| < 1 rotation is sane


def is_sentinel(min_v, max_v):
    return int(min_v) == INT32_MAX and int(max_v) == INT32_MIN


def wait_for_closed_loop(motor, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status = motor.get_status()
        if status and len(status) >= 2 and (status[0] & CLOSED_LOOP_BIT):
            return status[0]
        time.sleep(0.1)
    raise AssertionError(f"closed-loop flag never set within {timeout_s} s; "
                         f"is the motor calibrated?")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get max PID error' (cmd 39).")
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

            print("\nResetting device; PID error trackers must be at the sentinel...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            r = motor.get_max_pid_error()
            print(f"  get_max_pid_error after reset -> {r}")
            if not isinstance(r, list) or len(r) != 2:
                raise AssertionError(f"expected 2-element list, got: {r!r}")
            min_v, max_v = r
            if not is_sentinel(min_v, max_v):
                raise AssertionError(f"post-reset min/max ({min_v}, {max_v}) is not the sentinel "
                                     f"({INT32_MAX}, {INT32_MIN}); PID activity before closed-loop?")

            print("Entering closed loop and doing a small move...")
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            motor.go_to_closed_loop()
            wait_for_closed_loop(motor, CLOSED_LOOP_TIMEOUT_S)
            motor.zero_position()
            target = COUNTS_PER_ROTATION // 8
            motor.trapezoid_move(target, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            time.sleep(0.3)

            print("Reading max PID error; must be a real (non-sentinel) range with min <= max...")
            r1 = motor.get_max_pid_error()
            print(f"  first post-move read -> {r1}")
            if len(r1) != 2:
                raise AssertionError(f"expected 2 values, got: {r1!r}")
            min1, max1 = r1
            if is_sentinel(min1, max1):
                raise AssertionError(f"closed-loop move produced no PID samples (still sentinel): {r1}")
            if min1 > max1:
                raise AssertionError(f"PID error min > max after closed-loop activity: "
                                     f"min={min1}, max={max1}")
            if not (INT32_MIN <= int(min1) <= INT32_MAX) or not (INT32_MIN <= int(max1) <= INT32_MAX):
                raise AssertionError(f"PID error values outside i32: min={min1}, max={max1}")
            if abs(min1) > PLAUSIBLE_ERROR_BOUND_COUNTS or abs(max1) > PLAUSIBLE_ERROR_BOUND_COUNTS:
                raise AssertionError(f"PID error magnitude unreasonably large "
                                     f"(|min|={abs(min1)}, |max|={abs(max1)}, bound={PLAUSIBLE_ERROR_BOUND_COUNTS})")

            print("Reading again immediately; the read must have reset the trackers (new window)...")
            time.sleep(0.05)  # let a few PID cycles run so the new window has samples
            r2 = motor.get_max_pid_error()
            print(f"  second post-move read -> {r2}")
            if len(r2) != 2:
                raise AssertionError(f"expected 2 values, got: {r2!r}")
            min2, max2 = r2
            # Holding still in closed loop, the new window has run for ~50 ms of PID cycles, so
            # it should still be a real (non-sentinel) min <= max range, and the values should
            # be distinct from the previous window (proving the reset happened, not a cached read).
            if is_sentinel(min2, max2):
                raise AssertionError(f"second read returned sentinel — PID loop stopped running?")
            if min2 > max2:
                raise AssertionError(f"second-read min > max: min={min2}, max={max2}")
            if (min1, max1) == (min2, max2):
                raise AssertionError(f"second read returned identical values to the first "
                                     f"({r1}); the read did not reset the trackers")

            print("Returning to zero, dropping closed loop, resetting...")
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
