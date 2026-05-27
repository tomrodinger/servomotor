#!/usr/bin/env python3
"""
Functional test: position telemetry cross-check.

Modernized from the obsolete
test_motor_position_vs_hall_sensor_position_vs_external_encoder_position.py.

What this test does:

  1. Reset, enable MOSFETs, set PID, enter closed loop (the test motor is
     calibrated; see WORK_CHECKLIST.md environment notes).
  2. Read the initial comprehensive position (commanded, hall-sensed,
     external-encoder).
  3. Execute a sequence of trapezoid moves of varying signed magnitudes.
     After each move settles, read the comprehensive position again and
     compute the per-move delta for each of the three channels.
  4. Cross-check: the hall-sensed delta must track the commanded delta
     within `--hall-tolerance-rot` (default 0.10 rotation = 36 degrees).
  5. External-encoder check is **conditional** (per TEST_MODERNIZATION_PLAN.md
     "Hardware availability"): if the encoder is absent (the channel is
     all-zero across the whole sequence) the check is skipped with a clear
     message — a skip is not a failure. When present, the encoder delta
     must track the commanded delta within `--encoder-tolerance-rot`.
"""

import argparse
import sys
import time
import servomotor

STATUS_IN_THE_BOOTLOADER_FLAG_BIT = 0
STATUS_MOSFETS_ENABLED_FLAG_BIT = 1

RESET_DELAY_S = 1.5
SETTLE_AFTER_MOVE_S = 0.5
GO_TO_CLOSED_LOOP_SETTLE_S = 0.3
DEFAULT_MAX_CURRENT = 200          # M17 (QC reject, stiff shaft) needs the headroom
DEFAULT_PID_KP = 3000
DEFAULT_PID_KI = 2
DEFAULT_PID_KD = 175000
MOVE_SEQUENCE_ROT = [+0.25, +0.50, -0.25, -1.00, +0.50]
MOVE_DURATION_S = 1.5
DEFAULT_HALL_TOLERANCE_ROT = 0.10  # 36 degrees -- generous; closed-loop is typically << this
DEFAULT_ENCODER_TOLERANCE_ROT = 0.10


def assert_app_mode_no_error(motor, label):
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err} (expected 0). flags={flags:#x}")
    if flags & (1 << STATUS_IN_THE_BOOTLOADER_FLAG_BIT):
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#x}, expected application mode)")
    return flags, err


def read_position_triple(motor):
    """Return (commanded, hall, external) as floats in the motor's position_unit."""
    triple = motor.get_comprehensive_position()
    if not triple or len(triple) < 3:
        raise AssertionError(f"get_comprehensive_position returned unusable value: {triple!r}")
    return float(triple[0]), float(triple[1]), float(triple[2])


def main():
    parser = argparse.ArgumentParser(description="Functional test: motor commanded vs hall-sensed (and optionally external-encoder) position.")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--max-current', type=int, default=DEFAULT_MAX_CURRENT,
                        help=f'Maximum motor current (default: {DEFAULT_MAX_CURRENT})')
    parser.add_argument('--hall-tolerance-rot', type=float, default=DEFAULT_HALL_TOLERANCE_ROT,
                        help=f'Per-move tolerance for |hall_delta - commanded_delta| in rotations (default: {DEFAULT_HALL_TOLERANCE_ROT})')
    parser.add_argument('--encoder-tolerance-rot', type=float, default=DEFAULT_ENCODER_TOLERANCE_ROT,
                        help=f'Per-move tolerance for |encoder_delta - commanded_delta| in rotations (default: {DEFAULT_ENCODER_TOLERANCE_ROT})')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    success = False
    failure_message = ""
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="shaft_rotations",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            # 1. Clean state, enter closed loop.
            print("Resetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_app_mode_no_error(motor, "after reset")

            print("Enabling MOSFETs and entering closed loop...")
            motor.enable_mosfets()
            time.sleep(0.3)
            motor.set_pid_constants(DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD)
            motor.set_maximum_motor_current(args.max_current, args.max_current)
            motor.go_to_closed_loop()
            time.sleep(GO_TO_CLOSED_LOOP_SETTLE_S)
            assert_app_mode_no_error(motor, "after go_to_closed_loop")

            # 2. Reference position triple.
            cmd_ref, hall_ref, enc_ref = read_position_triple(motor)
            print(f"\nInitial: commanded={cmd_ref:+.4f} hall={hall_ref:+.4f} external={enc_ref:+.4f} rotations")

            # 3. Per-move sequence, collecting cross-check rows.
            rows = []  # each row: (target, cmd_delta, hall_delta, enc_delta)
            external_values_seen = [enc_ref]
            for i, target in enumerate(MOVE_SEQUENCE_ROT):
                print(f"\n[{i+1}/{len(MOVE_SEQUENCE_ROT)}] trapezoid_move({target:+.2f} rot, {MOVE_DURATION_S}s)...")
                motor.trapezoid_move(target, MOVE_DURATION_S)
                time.sleep(MOVE_DURATION_S + SETTLE_AFTER_MOVE_S)
                cmd_now, hall_now, enc_now = read_position_triple(motor)
                cmd_d = cmd_now - cmd_ref
                hall_d = hall_now - hall_ref
                enc_d = enc_now - enc_ref
                rows.append((target, cmd_d, hall_d, enc_d))
                external_values_seen.append(enc_now)
                print(f"  cumulative deltas: commanded={cmd_d:+.4f} hall={hall_d:+.4f} external={enc_d:+.4f} rotations")
                cmd_ref, hall_ref, enc_ref = cmd_now, hall_now, enc_now

            # 4. Cross-check hall against commanded (per-move increment).
            print("\nHall vs commanded cross-check:")
            print("  iter   target_rot   cmd_delta_rot   hall_delta_rot   diff_rot   verdict")
            hall_breaches = []
            for idx, (target, cmd_d, hall_d, _enc_d) in enumerate(rows):
                diff = hall_d - cmd_d
                ok = abs(diff) <= args.hall_tolerance_rot
                if not ok:
                    hall_breaches.append((idx, target, cmd_d, hall_d, diff))
                print(f"  {idx:>4d}   {target:>+10.4f}   {cmd_d:>+13.4f}   {hall_d:>+14.4f}   {diff:>+8.4f}   {'OK' if ok else f'FAIL (>{args.hall_tolerance_rot})'}")
            if hall_breaches:
                raise AssertionError(
                    f"{len(hall_breaches)} of {len(rows)} hall-vs-commanded diffs exceeded ±{args.hall_tolerance_rot} rot. "
                    f"Worst: iter {hall_breaches[0][0]}, diff {hall_breaches[0][4]:+.4f}."
                )
            print(f"OK: all {len(rows)} hall-vs-commanded diffs are within ±{args.hall_tolerance_rot} rot.")

            # 5. External encoder check is conditional. Treat the channel as
            #    "absent" when every reading is exactly 0.0.
            encoder_present = any(abs(v) > 1e-9 for v in external_values_seen)
            print()
            if not encoder_present:
                print("SKIPPED external-encoder cross-check: encoder not present (every reading was 0.0).")
                print("(This is not a failure — see TEST_MODERNIZATION_PLAN.md, 'Hardware availability'.)")
            else:
                print("External vs commanded cross-check:")
                print("  iter   target_rot   cmd_delta_rot   enc_delta_rot    diff_rot   verdict")
                enc_breaches = []
                for idx, (target, cmd_d, _hall_d, enc_d) in enumerate(rows):
                    diff = enc_d - cmd_d
                    ok = abs(diff) <= args.encoder_tolerance_rot
                    if not ok:
                        enc_breaches.append((idx, target, cmd_d, enc_d, diff))
                    print(f"  {idx:>4d}   {target:>+10.4f}   {cmd_d:>+13.4f}   {enc_d:>+13.4f}   {diff:>+9.4f}   {'OK' if ok else f'FAIL (>{args.encoder_tolerance_rot})'}")
                if enc_breaches:
                    raise AssertionError(
                        f"{len(enc_breaches)} of {len(rows)} encoder-vs-commanded diffs exceeded ±{args.encoder_tolerance_rot} rot. "
                        f"Worst: iter {enc_breaches[0][0]}, diff {enc_breaches[0][4]:+.4f}."
                    )
                print(f"OK: all {len(rows)} encoder-vs-commanded diffs are within ±{args.encoder_tolerance_rot} rot.")

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        # Leave the device in a quiet state.
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
