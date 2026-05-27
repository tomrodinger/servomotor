#!/usr/bin/env python3
"""
Per-command test for "Set PID constants" (cmd 43).

Cmd 43 takes three u32 fields (kP, kI, kD) and returns a success
response. The firmware applies the new constants immediately (it
recomputes the PID parameter bundle inside __disable_irq()/__enable_irq()
so the very next control-loop iteration uses them). There is no readback
command, and the firmware does no command-time validation of the values,
so this test asserts the *observable* control-behaviour difference:

  1. The command is accepted with no fatal error (round-trip works).
  2. With a known-good set of PID constants the motor enters closed loop
     and a trapezoid move lands on target with a bounded max-PID-error
     window. This is the "set good gains -> healthy control" path.
  3. With a deliberately weak P (kP = 1, kI = 0, kD = 0) the same move
     either misses target by far more than the good case, registers a
     materially larger max-PID-error magnitude, or trips
     ERROR_POSITION_DEVIATION_TOO_LARGE (45) — any of which proves the
     set actually changed control behavior, not just acknowledged. The
     test asserts that at least one of these degradation signatures
     fired.

The "known-good" gains hard-coded here (2000, 5, 175000) are the
compile-time defaults the firmware applies at boot for PRODUCT_NAME_M17
(see firmware/Src/motor_control.h). The bench M17 is the targeted
hardware (see WORK_CHECKLIST.md). If this test ever runs against a
different model, those constants should be updated to that model's
working values.
"""

import argparse
import sys
import time
import servomotor
from servomotor.communication import FatalError

COUNTS_PER_ROTATION = 3276800
CLOSED_LOOP_BIT = 1 << 2
RESET_DELAY_S = 1.5
ENABLE_SETTLE_S = 0.3
CLOSED_LOOP_TIMEOUT_S = 6.0
ERROR_POSITION_DEVIATION_TOO_LARGE = 45

# Known working gains for the bench M17 (firmware compile-time defaults).
GOOD_KP, GOOD_KI, GOOD_KD = 2000, 5, 175000
# Deliberately too-weak proportional, no integral / derivative.
WEAK_KP, WEAK_KI, WEAK_KD = 1, 0, 0

GOOD_TOLERANCE_COUNTS = 4000          # closed-loop holding tolerance (hall-based, not exact)
DEGRADATION_FACTOR = 3                # weak |error| or |miss| must exceed good by this factor
MIN_WEAK_MISS_COUNTS = 12000          # absolute threshold for "clearly missed"


def wait_for_closed_loop(motor, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            status = motor.get_status()
        except FatalError as e:
            raise AssertionError(f"fatal error {e.args[0]} while waiting for closed loop")
        if status and len(status) >= 2 and (status[0] & CLOSED_LOOP_BIT):
            return
        time.sleep(0.1)
    raise AssertionError(f"closed-loop flag never set within {timeout_s} s")


def run_move_and_measure(motor, target):
    """Move to target, settle, then read final position and max PID error.
    Returns (pos, |pid_err_max|, fatal_code_or_None).
    fatal_code_or_None == 45 if a position-deviation fatal fired."""
    try:
        motor.trapezoid_move(target, 1.0)
        while motor.get_n_queued_items() > 0:
            time.sleep(0.01)
        time.sleep(0.3)
        pos = motor.get_position()
        r = motor.get_max_pid_error()
        if len(r) != 2:
            raise AssertionError(f"get_max_pid_error returned: {r!r}")
        peak = max(abs(r[0]), abs(r[1]))
        return pos, peak, None
    except FatalError as e:
        return None, None, e.args[0] if e.args else None


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Set PID constants' (cmd 43).")
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

            target = COUNTS_PER_ROTATION // 8

            print(f"\nPhase A: set good PID ({GOOD_KP}, {GOOD_KI}, {GOOD_KD}); move must land on target.")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.set_pid_constants(GOOD_KP, GOOD_KI, GOOD_KD)
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            motor.go_to_closed_loop()
            wait_for_closed_loop(motor, CLOSED_LOOP_TIMEOUT_S)
            motor.zero_position()
            # Discard the pre-move PID window (motor was settling after closed-loop entry).
            motor.get_max_pid_error()
            pos_good, peak_good, fatal_good = run_move_and_measure(motor, target)
            print(f"  good gains: pos={pos_good}, peak |PID err|={peak_good}, "
                  f"fatal={fatal_good}")
            if fatal_good is not None:
                raise AssertionError(f"good gains tripped fatal error {fatal_good}; "
                                     f"the 'known-good' constants are not right for this motor")
            if abs(pos_good - target) > GOOD_TOLERANCE_COUNTS:
                raise AssertionError(f"good gains: pos {pos_good} missed target {target} by "
                                     f"{abs(pos_good - target)} > {GOOD_TOLERANCE_COUNTS}")

            # Drop closed loop / return home before phase B.
            motor.trapezoid_move(0, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            motor.disable_mosfets()

            print(f"\nPhase B: set weak PID ({WEAK_KP}, {WEAK_KI}, {WEAK_KD}); "
                  f"same move must show degradation.")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.set_pid_constants(WEAK_KP, WEAK_KI, WEAK_KD)
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            try:
                motor.go_to_closed_loop()
            except FatalError as e:
                fatal_code = e.args[0] if e.args else None
                print(f"  weak gains: go_to_closed_loop tripped fatal {fatal_code} — "
                      f"degradation observed.")
                # Confirm the device is genuinely degraded and recover.
                motor.system_reset()
                time.sleep(RESET_DELAY_S)
                continue

            # Closed loop entered. Try the move and look for degradation.
            try:
                wait_for_closed_loop(motor, CLOSED_LOOP_TIMEOUT_S)
                motor.zero_position()
                motor.get_max_pid_error()  # discard settle window
                pos_weak, peak_weak, fatal_weak = run_move_and_measure(motor, target)
            except AssertionError as e:
                # e.g. closed-loop flag never set, or fatal error during status polling.
                print(f"  weak gains: closed-loop/control degraded ({e}); degradation observed.")
                motor.system_reset()
                time.sleep(RESET_DELAY_S)
                continue

            print(f"  weak gains: pos={pos_weak}, peak |PID err|={peak_weak}, fatal={fatal_weak}")
            degraded = False
            if fatal_weak == ERROR_POSITION_DEVIATION_TOO_LARGE:
                print("  -> degradation: position deviation fatal error fired (expected).")
                degraded = True
            elif fatal_weak is not None:
                print(f"  -> degradation: some fatal error fired (code {fatal_weak}).")
                degraded = True
            else:
                miss_good = abs(pos_good - target)
                miss_weak = abs(pos_weak - target)
                print(f"  miss_good={miss_good}, miss_weak={miss_weak}; "
                      f"peak_good={peak_good}, peak_weak={peak_weak}")
                if miss_weak >= MIN_WEAK_MISS_COUNTS and miss_weak >= DEGRADATION_FACTOR * max(miss_good, 1):
                    print("  -> degradation: weak gains missed target by far more than good.")
                    degraded = True
                if peak_weak >= DEGRADATION_FACTOR * max(peak_good, 1):
                    print("  -> degradation: weak gains produced much larger PID error.")
                    degraded = True

            if not degraded:
                raise AssertionError("weak PID constants produced no observable degradation; "
                                     "set_pid_constants does not appear to have taken effect")

            print("Resetting (restores firmware-default constants for any later test)...")
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
                # Always leave the device on firmware-default gains.
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
