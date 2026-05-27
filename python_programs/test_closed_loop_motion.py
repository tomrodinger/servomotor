#!/usr/bin/env python3
"""
Functional test: closed-loop motion (entry + accuracy + holding).

Modernized from the obsolete test_go_to_closed_loop_mode_and_spin_motor.py.
That file used the deprecated `communication` module and gathered closed-loop
entry statistics based on phase angle. This modernized version uses the
`servomotor` library and focuses on three concrete claims about closed-loop
behaviour:

  (A) **Entry**. `Go to closed loop` succeeds and leaves the device in
      application mode with the closed-loop status flag set, no fatal error.
  (B) **Move accuracy**. Successive trapezoid moves leave the commanded
      position exactly at the target (per-move tolerance
      `--position-tolerance-rot`).
  (C) **Position holding**. After the final move settles, the hall-sensed
      position is sampled `--holding-samples` times and must stay within
      `--holding-tolerance-rot` of the settled position — i.e. the
      controller is not oscillating or drifting at rest.

Telemetry-channel cross-checks (hall vs encoder) are the job of
`test_position_telemetry.py`, not this test.
"""

import argparse
import sys
import time
import servomotor

STATUS_IN_THE_BOOTLOADER_FLAG_BIT = 0
STATUS_MOSFETS_ENABLED_FLAG_BIT = 1
STATUS_CLOSED_LOOP_FLAG_BIT = 2

RESET_DELAY_S = 1.5
ENABLE_SETTLE_S = 0.3
GO_TO_CLOSED_LOOP_SETTLE_S = 0.3
SETTLE_AFTER_MOVE_S = 0.5
DEFAULT_MAX_CURRENT = 200
DEFAULT_PID_KP = 3000
DEFAULT_PID_KI = 2
DEFAULT_PID_KD = 175000
MOVE_SEQUENCE_ROT = [+1.0, -1.0, +0.5, -0.5, +0.25, -0.25]
MOVE_DURATION_S = 1.5
DEFAULT_POSITION_TOLERANCE_ROT = 0.05
DEFAULT_HOLDING_TOLERANCE_ROT = 0.05
DEFAULT_HOLDING_SAMPLES = 20
HOLDING_SAMPLE_PERIOD_S = 0.05


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


def main():
    parser = argparse.ArgumentParser(description="Functional test: closed-loop entry + move accuracy + position holding.")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--max-current', type=int, default=DEFAULT_MAX_CURRENT,
                        help=f'Maximum motor current (default: {DEFAULT_MAX_CURRENT})')
    parser.add_argument('--position-tolerance-rot', type=float, default=DEFAULT_POSITION_TOLERANCE_ROT,
                        help=f'Per-move tolerance for |commanded - target| in rotations (default: {DEFAULT_POSITION_TOLERANCE_ROT})')
    parser.add_argument('--holding-tolerance-rot', type=float, default=DEFAULT_HOLDING_TOLERANCE_ROT,
                        help=f'Tolerance for hall-position drift during holding (default: {DEFAULT_HOLDING_TOLERANCE_ROT})')
    parser.add_argument('--holding-samples', type=int, default=DEFAULT_HOLDING_SAMPLES,
                        help=f'Number of hall-position samples taken at rest after the final move (default: {DEFAULT_HOLDING_SAMPLES})')
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

            # --- (A) Closed-loop entry ---
            print("Resetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_app_mode_no_error(motor, "after reset")

            print("Enabling MOSFETs...")
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            motor.set_pid_constants(DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD)
            motor.set_maximum_motor_current(args.max_current, args.max_current)

            print("Entering closed loop...")
            motor.go_to_closed_loop()
            time.sleep(GO_TO_CLOSED_LOOP_SETTLE_S)
            flags, _ = assert_app_mode_no_error(motor, "after go_to_closed_loop")
            if not (flags & (1 << STATUS_CLOSED_LOOP_FLAG_BIT)):
                raise AssertionError(f"After go_to_closed_loop the closed-loop status flag is NOT set (flags={flags:#x}).")
            if not (flags & (1 << STATUS_MOSFETS_ENABLED_FLAG_BIT)):
                raise AssertionError(f"After go_to_closed_loop the MOSFETs-enabled flag is NOT set (flags={flags:#x}).")
            print(f"OK: closed-loop flag set (flags={flags:#x}).")

            # --- (B) Move accuracy ---
            print("\nExecuting the move sequence and checking commanded position lands on target...")
            commanded_running = 0.0
            triple = motor.get_comprehensive_position()
            base_commanded = float(triple[0])
            print(f"  baseline commanded position = {base_commanded:+.4f} rot")

            move_breaches = []
            for i, target in enumerate(MOVE_SEQUENCE_ROT):
                commanded_running += target
                motor.trapezoid_move(target, MOVE_DURATION_S)
                time.sleep(MOVE_DURATION_S + SETTLE_AFTER_MOVE_S)
                triple = motor.get_comprehensive_position()
                commanded_now = float(triple[0])
                actual_delta = commanded_now - base_commanded
                diff = actual_delta - commanded_running
                ok = abs(diff) <= args.position_tolerance_rot
                if not ok:
                    move_breaches.append((i, target, commanded_running, actual_delta, diff))
                print(f"  [{i+1}/{len(MOVE_SEQUENCE_ROT)}] target_step={target:+.3f} expected_cum={commanded_running:+.4f} actual_cum={actual_delta:+.4f} diff={diff:+.4f}  {'OK' if ok else 'FAIL'}")

            if move_breaches:
                worst = max(move_breaches, key=lambda r: abs(r[4]))
                raise AssertionError(
                    f"{len(move_breaches)} moves exceeded position tolerance ±{args.position_tolerance_rot} rot. "
                    f"Worst: iter {worst[0]}, diff {worst[4]:+.4f}."
                )
            print(f"OK: all {len(MOVE_SEQUENCE_ROT)} moves landed within ±{args.position_tolerance_rot} rot.")

            # --- (C) Position holding ---
            print(f"\nHolding test: {args.holding_samples} hall samples over "
                  f"{args.holding_samples * HOLDING_SAMPLE_PERIOD_S:.2f} s after the last move...")
            samples = []
            for _ in range(args.holding_samples):
                t = motor.get_comprehensive_position()
                samples.append(float(t[1]))
                time.sleep(HOLDING_SAMPLE_PERIOD_S)
            ref = samples[0]
            deviations = [s - ref for s in samples]
            max_dev = max(abs(d) for d in deviations)
            spread = max(samples) - min(samples)
            print(f"  hall at ref:        {ref:+.4f} rot")
            print(f"  max |dev from ref|: {max_dev:.4f} rot")
            print(f"  sample spread:      {spread:.4f} rot")
            if max_dev > args.holding_tolerance_rot:
                raise AssertionError(
                    f"Hall position drifted {max_dev:.4f} rot from the reference during holding, "
                    f"exceeds tolerance ±{args.holding_tolerance_rot} rot."
                )
            print(f"OK: holding within ±{args.holding_tolerance_rot} rot.")

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
