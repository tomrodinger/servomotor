#!/usr/bin/env python3
"""
Edge cases for the trapezoid-move planner: degenerate limits, very short
durations, and very small displacements.

THE INVARIANT THIS TEST ENFORCES
    A move command must do exactly one of two things:
      (a) be accepted AND actually move the commanded position by what was
          asked for, or
      (b) be rejected with a fatal error explaining why.
    Being accepted, reporting success, raising no error, and then not moving is
    ALWAYS a failure. That silent third outcome is the bug this test was written
    for, and it is the worst possible behaviour: the caller has no way to know.

    The one legitimate exception is a commanded displacement of zero, which is a
    timed dwell -- accepted, and correctly does not move.

WHAT WENT WRONG (the regression being locked down)
    compute_trapezoid_move() computes
        delta_t1   = max_velocity / max_acceleration        (integer division)
        denominator = (delta_t1 + delta_t2) * delta_t1
        acceleration = numerator / denominator
    A delta_t1 of 0 makes the denominator 0. That division does not trap on the
    Cortex-M0+; it silently yields acceleration 0, so the move was accepted and
    the shaft never moved. Two independent ways to reach it, both confirmed on
    hardware on firmware 0.15.9.0 -- on the bench motor and on all 35 rack
    motors:
      1. The acceleration limit exceeding roughly 31250x the velocity limit.
         This needs NO exotic settings: a 0.3 rotation/second speed limit with
         the 12000 rotation/second^2 FACTORY DEFAULT acceleration triggers it,
         so any slow, delicate axis hits it.
      2. A duration of 1 timestep, which makes (total_time >> 1) zero
         regardless of the limits.
    Fixed in firmware 0.15.10.0 by detecting the zero denominator and giving the
    ramp its shortest physical length, one control tick.

SAFETY
    Every check below reads the COMMANDED position, which the planner advances
    whether or not the MOSFETs are on, so the MOSFETs are left OFF and nothing
    physically turns. That also makes the test safe to run on a shared rack and
    on a motor with a load attached. Each motor is left reset and clean.
"""

import argparse
import sys
import time

import servomotor
from servomotor import M3

COUNTS_PER_ROTATION = 3276800
TIMESTEPS_PER_SECOND = 31250
RESET_WAIT = 1.4          # golden rule 1: keep the bus quiet after a reset


class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.failures = []

    def check(self, name, ok, detail=""):
        if ok:
            self.passed += 1
            print(f"  PASS  {name}")
        else:
            self.failed += 1
            self.failures.append(f"{name}: {detail}")
            print(f"  FAIL  {name}   {detail}")


def fresh(motor):
    """Reset to a known state and wait out the bootloader window."""
    motor.system_reset()
    time.sleep(RESET_WAIT)


def attempt_move(motor, disp_counts, dur_timesteps, max_vel=None, max_accel=None):
    """Try one raw move. Returns (accepted, fatal_code, counts_moved).

    accepted is False when the command itself raised, which is how a rejection
    normally surfaces. fatal_code is read afterwards because some checks are
    asynchronous and report through Get status instead.
    """
    fresh(motor)
    if max_vel is not None:
        motor.set_maximum_velocity(max_vel)
    if max_accel is not None:
        motor.set_maximum_acceleration(max_accel)

    motor.set_position_unit("encoder_counts")
    before = motor.get_position()

    accepted = True
    try:
        # encoder_counts and timesteps both convert with a factor of 1, so these
        # are the exact raw values the firmware planner sees -- no unit-conversion
        # rounding to muddy the edge cases.
        motor.set_time_unit("timesteps")
        motor.trapezoid_move(int(disp_counts), int(dur_timesteps))
    except Exception:
        accepted = False
    finally:
        motor.set_time_unit("seconds")

    # Let the move run to completion (plus margin), then look.
    time.sleep(min(dur_timesteps / TIMESTEPS_PER_SECOND, 5.0) + 0.35)

    try:
        fatal = motor.get_status()[1]
    except Exception:
        fatal = -1
    try:
        after = motor.get_position()
    except Exception:
        after = before
    return accepted, fatal, after - before


def judge(res, label, accepted, fatal, moved, want_counts):
    """Apply the invariant: accepted => moved correctly; else => an error."""
    if want_counts == 0:
        ok = (fatal == 0) or (not accepted)
        res.check(f"{label}: zero-displacement dwell is accepted and still",
                  ok and abs(moved) <= 1,
                  f"accepted={accepted} fatal={fatal} moved={moved}")
        return

    if accepted and fatal == 0:
        # Claimed success -- it must actually have moved what was asked.
        tol = max(2, abs(want_counts) // 100)
        ok = abs(moved - want_counts) <= tol
        res.check(f"{label}: accepted, so it must move {want_counts} counts",
                  ok, f"moved={moved} (silent no-op)" if moved == 0
                      else f"moved={moved}, wanted {want_counts}")
    else:
        # Refused. That is fine -- the point is that it is not silent.
        res.check(f"{label}: refused loudly rather than silently doing nothing",
                  True, "")
        print(f"          (accepted={accepted} fatal={fatal})")


def main(args):
    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port()
    res = Results()
    motor = None
    try:
        motor = M3(args.alias, verbose=0)
        motor.set_velocity_unit("rotations_per_second")
        motor.set_acceleration_unit("rotations_per_second_squared")

        # ------------------------------------------------------------------
        # 1. The regression: a low speed limit with the DEFAULT acceleration.
        #    Every one of these silently did nothing before 0.15.10.0.
        # ------------------------------------------------------------------
        print("\n1. Low velocity limit, factory-default acceleration (the regression)")
        for vel in (0.1, 0.2, 0.3, 0.38, 0.5, 1.0):
            want = int(0.05 * COUNTS_PER_ROTATION)
            a, f, moved = attempt_move(motor, want, int(2.0 * TIMESTEPS_PER_SECOND),
                                       max_vel=vel)   # acceleration untouched
            judge(res, f"maxVel={vel} rot/s, default accel", a, f, moved, want)

        # ------------------------------------------------------------------
        # 2. The regression from the other side: acceleration far above the
        #    velocity limit, including the exact ratio boundary.
        # ------------------------------------------------------------------
        print("\n2. Acceleration limit far above the velocity limit")
        for vel, acc in ((1.0, 31250.0), (1.0, 48000.0), (0.5, 20000.0), (2.0, 48000.0)):
            want = int(0.05 * COUNTS_PER_ROTATION)
            a, f, moved = attempt_move(motor, want, int(2.0 * TIMESTEPS_PER_SECOND),
                                       max_vel=vel, max_accel=acc)
            judge(res, f"maxVel={vel} maxAccel={acc}", a, f, moved, want)

        # ------------------------------------------------------------------
        # 3. Very short DURATIONS, at the factory defaults. 1 timestep = 32 us.
        # ------------------------------------------------------------------
        print("\n3. Very short durations (default limits)")
        for ts in (1, 2, 3, 10, 100, 1000):
            want = 1000
            a, f, moved = attempt_move(motor, want, ts)
            judge(res, f"duration={ts} timestep(s), {want} counts", a, f, moved, want)

        # ------------------------------------------------------------------
        # 4. Very small DISPLACEMENTS over a comfortable duration.
        # ------------------------------------------------------------------
        print("\n4. Very small displacements (1 second)")
        for counts in (0, 1, 2, 10, 100):
            a, f, moved = attempt_move(motor, counts, TIMESTEPS_PER_SECOND)
            judge(res, f"displacement={counts} counts over 1 s", a, f, moved, counts)

        # ------------------------------------------------------------------
        # 5. Both extremes at once: tiny distance in tiny time, and a large
        #    distance in tiny time (which must be refused, not attempted).
        # ------------------------------------------------------------------
        print("\n5. Combined extremes")
        for counts, ts, note in ((1, 1, "1 count in 1 timestep"),
                                 (1, 2, "1 count in 2 timesteps"),
                                 (COUNTS_PER_ROTATION, 10, "1 rotation in 10 timesteps"),
                                 (COUNTS_PER_ROTATION * 10, 100, "10 rotations in 100 timesteps")):
            a, f, moved = attempt_move(motor, counts, ts)
            judge(res, note, a, f, moved, counts)

        # ------------------------------------------------------------------
        # 6. The motor must still be healthy and usable after all of that.
        # ------------------------------------------------------------------
        print("\n6. Motor still healthy afterwards")
        want = int(0.25 * COUNTS_PER_ROTATION)
        a, f, moved = attempt_move(motor, want, TIMESTEPS_PER_SECOND)
        judge(res, "ordinary move still works after the edge cases", a, f, moved, want)

    except Exception as e:
        print(f"\nTest aborted: {type(e).__name__}: {e}", file=sys.stderr)
        res.failed += 1
        res.failures.append(f"aborted: {e}")
    finally:
        if motor is not None:
            try:
                fresh(motor)
            except Exception:
                pass
        servomotor.close_serial_port()
        print("Closed the serial port")

    print(f"\n{res.passed} passed, {res.failed} failed")
    for f in res.failures:
        print(f"  - {f}")
    if res.failed == 0:
        print("\nPASSED")
        sys.exit(0)
    print("\nFAILED")
    sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Trapezoid-move planner edge cases: degenerate limits, very "
                    "short durations, very small displacements.")
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', action='store_true',
                        help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', help='Alias of the device to control', default='X')
    args = parser.parse_args()
    main(args)
