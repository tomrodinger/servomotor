#!/usr/bin/env python3
"""
Per-command test for "Set max allowable position deviation" (cmd 44).

Cmd 44 takes one i64 (in encoder_counts internally) and returns a
success response. The firmware does two notable things with it:

  - It stores `llabs(input)` — negative inputs are folded to their
    absolute magnitude, not rejected.
  - The motor control loop (always running once calibration is done)
    computes `position_deviation = desired - hall` every cycle, and if
    |deviation| exceeds the stored value it raises
    ERROR_POSITION_DEVIATION_TOO_LARGE (45) and goes into the fatal-
    error state — i.e. setting the limit *small* turns ordinary motion
    into an immediate trip.

What this test verifies:

  1. Setting a generous limit (e.g. one full shaft rotation) lets a
     normal trapezoid move complete with no fatal error — proves the
     command accepts the value and does not gratuitously trip.
  2. Setting a tiny positive limit traps a fast trapezoid move with
     ERROR_POSITION_DEVIATION_TOO_LARGE (45) — proves the limit is
     actively enforced by the control loop, not just stored and
     ignored.
  3. Setting a *negative* tiny limit also traps the same fast move with
     code 45 — proves the firmware's `llabs()` fold of the input.

The motor doesn't have to be in closed loop for the deviation check;
the firmware runs the check whenever calibration is complete, so this
test stays in open loop.
"""

import argparse
import sys
import time
import servomotor
from servomotor.communication import FatalError

COUNTS_PER_ROTATION = 3276800
RESET_DELAY_S = 1.5
ENABLE_SETTLE_S = 0.3
GENEROUS_LIMIT_COUNTS = COUNTS_PER_ROTATION         # one full rotation; ordinary moves never hit this
TIGHT_LIMIT_COUNTS = 2000                           # ~0.06% of a rotation; any fast move trips
FAST_MOVE_TARGET = COUNTS_PER_ROTATION // 2         # half rotation
FAST_MOVE_DURATION_S = 0.3                          # aggressive — desired races ahead of hall
TOLERANCE_COUNTS = 80
ERROR_POSITION_DEVIATION_TOO_LARGE = 45
TRIP_POLL_TIMEOUT_S = 4.0


def is_in_deviation_fatal(motor):
    """Return True iff the device reports fatal error code 45.
    In fatal-error state the firmware replies with an error packet to
    almost every command, which surfaces as FatalError in Python."""
    try:
        status = motor.get_status()
    except FatalError as e:
        return e.args and e.args[0] == ERROR_POSITION_DEVIATION_TOO_LARGE
    if status and len(status) >= 2 and status[1] == ERROR_POSITION_DEVIATION_TOO_LARGE:
        return True
    return False


def expect_deviation_trip(motor, label):
    """Poll for up to TRIP_POLL_TIMEOUT_S waiting for fatal code 45 to appear.
    Raises AssertionError if no trip is observed."""
    deadline = time.time() + TRIP_POLL_TIMEOUT_S
    last_seen = None
    while time.time() < deadline:
        try:
            status = motor.get_status()
            last_seen = status[1] if status and len(status) >= 2 else last_seen
            if status and len(status) >= 2 and status[1] == ERROR_POSITION_DEVIATION_TOO_LARGE:
                return
        except FatalError as e:
            code = e.args[0] if e.args else None
            if code == ERROR_POSITION_DEVIATION_TOO_LARGE:
                return
            raise AssertionError(f"{label}: unexpected fatal error {code}, "
                                 f"expected {ERROR_POSITION_DEVIATION_TOO_LARGE}")
        time.sleep(0.05)
    raise AssertionError(f"{label}: no deviation fatal observed within {TRIP_POLL_TIMEOUT_S} s "
                         f"(last seen fatal_error_code = {last_seen}); the tight limit was not enforced")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Set max allowable position deviation' (cmd 44).")
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

            # ---- Phase 1: generous limit -> normal move completes cleanly ----
            print(f"\nPhase 1: generous limit ({GENEROUS_LIMIT_COUNTS} counts) — ordinary move must complete.")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.set_max_allowable_position_deviation(GENEROUS_LIMIT_COUNTS)
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            motor.zero_position()
            target = COUNTS_PER_ROTATION // 4
            motor.trapezoid_move(target, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            time.sleep(0.2)
            status = motor.get_status()
            if not status or len(status) < 2:
                raise AssertionError(f"unusable get_status response: {status!r}")
            if status[1] != 0:
                raise AssertionError(f"Phase 1: ordinary move tripped fatal_error_code={status[1]} "
                                     f"with a generous deviation limit")
            pos = motor.get_position()
            if abs(pos - target) > TOLERANCE_COUNTS:
                raise AssertionError(f"Phase 1: pos {pos} missed target {target} by "
                                     f"{abs(pos - target)} > {TOLERANCE_COUNTS}")
            print(f"  pos={pos}, no fatal error — accepted.")

            motor.trapezoid_move(0, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            motor.disable_mosfets()
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            # ---- Phase 2: tight positive limit -> fast move trips code 45 ----
            # Enable + zero first under the default generous limit, then set the
            # tight limit, then issue the fast move. This avoids tripping on the
            # transient produced when MOSFETs first energise (the firmware steps
            # the rotor into commutation alignment, which can momentarily move the
            # hall reading by more than a few thousand counts).
            print(f"\nPhase 2: tight positive limit ({TIGHT_LIMIT_COUNTS} counts) — fast move must trip code 45.")
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            motor.zero_position()
            motor.set_max_allowable_position_deviation(TIGHT_LIMIT_COUNTS)
            # Queue the fast move; the trapezoid_move command itself just queues, so it
            # succeeds — the deviation trip happens asynchronously inside the control loop.
            try:
                motor.trapezoid_move(FAST_MOVE_TARGET, FAST_MOVE_DURATION_S)
            except FatalError as e:
                code = e.args[0] if e.args else None
                if code != ERROR_POSITION_DEVIATION_TOO_LARGE:
                    raise AssertionError(f"Phase 2: trapezoid_move raised unexpected fatal {code}")
                print(f"  trapezoid_move itself raised fatal {code} — trip observed immediately.")
            else:
                expect_deviation_trip(motor, "Phase 2 (tight positive)")
                print("  deviation fatal 45 fired as expected.")

            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            # ---- Phase 3: negative tight limit -> same trip (proves llabs fold) ----
            print(f"\nPhase 3: tight NEGATIVE limit ({-TIGHT_LIMIT_COUNTS} counts) — must behave like |value| and trip code 45.")
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            motor.zero_position()
            motor.set_max_allowable_position_deviation(-TIGHT_LIMIT_COUNTS)
            try:
                motor.trapezoid_move(FAST_MOVE_TARGET, FAST_MOVE_DURATION_S)
            except FatalError as e:
                code = e.args[0] if e.args else None
                if code != ERROR_POSITION_DEVIATION_TOO_LARGE:
                    raise AssertionError(f"Phase 3: trapezoid_move raised unexpected fatal {code}")
                print(f"  trapezoid_move itself raised fatal {code} — negative input folded to abs as expected.")
            else:
                expect_deviation_trip(motor, "Phase 3 (tight negative)")
                print("  deviation fatal 45 fired — negative input folded to abs as expected.")

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
            try:
                motor.system_reset()
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
