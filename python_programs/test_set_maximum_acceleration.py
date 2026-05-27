#!/usr/bin/env python3
"""
Per-command test for "Set maximum acceleration" (cmd 5).

Cmd 5 input: (u32 maximumAcceleration) in acceleration units. It clamps the
acceleration the firmware will allow. The firmware enforces it by *rejecting*
a move that would exceed the cap with ERROR_ACCEL_TOO_HIGH (code 15) — it does
not silently clamp.

What this test verifies (deterministic, no acceleration measurement needed):
  1. With a low cap, a move_with_acceleration commanding above the cap is
     rejected with ERROR_ACCEL_TOO_HIGH (15). This proves the cap took effect.
  2. With a high cap (after reset), a move_with_acceleration commanding below
     the cap is accepted and runs with no fatal error. This proves the cap
     value actually changed behaviour rather than always rejecting.

A high maximum velocity is set first so the acceleration limit, not the
velocity limit, is the thing under test.
"""

import argparse
import sys
import time
import servomotor
from servomotor.communication import FatalError

ERROR_ACCEL_TOO_HIGH = 15
RESET_DELAY_S = 1.5

HIGH_VEL = 20.0             # rotations_per_second — high so velocity is not the limiter
LOW_ACCEL_CAP = 1.0         # rotations_per_second_squared
OVER_CAP_CMD_ACCEL = 50.0   # well above the low cap → must be rejected
HIGH_ACCEL_CAP = 100.0
UNDER_CAP_CMD_ACCEL = 2.0   # comfortably under the high cap → must be accepted
MOVE_TIME_S = 0.5


def get_err(motor):
    status = motor.get_status()
    return status[1] if status and len(status) >= 2 else None


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Set maximum acceleration' (cmd 5).")
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
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="shaft_rotations",
                              velocity_unit="rotations_per_second",
                              acceleration_unit="rotations_per_second_squared", verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            # --- Scenario A: low cap rejects an over-cap acceleration ---
            print("\nResetting, enabling MOSFETs, high velocity, LOW acceleration cap...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            motor.zero_position()
            motor.set_maximum_velocity(HIGH_VEL)
            motor.set_maximum_acceleration(LOW_ACCEL_CAP)

            print(f"Commanding move_with_acceleration({OVER_CAP_CMD_ACCEL}) — above the {LOW_ACCEL_CAP} cap, must be rejected...")
            rejected = False
            try:
                motor.move_with_acceleration(OVER_CAP_CMD_ACCEL, MOVE_TIME_S)
            except FatalError as e:
                code = e.args[0] if e.args else None
                if code != ERROR_ACCEL_TOO_HIGH:
                    raise AssertionError(f"over-cap move rejected with FatalError({code}); expected {ERROR_ACCEL_TOO_HIGH} (ERROR_ACCEL_TOO_HIGH)")
                print(f"  correctly rejected with FatalError({code}) = ERROR_ACCEL_TOO_HIGH.")
                rejected = True
            if not rejected:
                err = get_err(motor)
                if err == ERROR_ACCEL_TOO_HIGH:
                    print(f"  correctly entered fatal_error({err}) = ERROR_ACCEL_TOO_HIGH.")
                else:
                    raise AssertionError(f"over-cap move was NOT rejected — max acceleration not enforced (fatal_error={err}).")

            # --- Scenario B: high cap accepts an under-cap acceleration ---
            print("\nResetting, enabling MOSFETs, high velocity, HIGH acceleration cap...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            motor.zero_position()
            motor.set_maximum_velocity(HIGH_VEL)
            motor.set_maximum_acceleration(HIGH_ACCEL_CAP)

            print(f"Commanding move_with_acceleration({UNDER_CAP_CMD_ACCEL}) — under the {HIGH_ACCEL_CAP} cap, must be accepted...")
            try:
                # Accelerate, then decelerate by the same amount so velocity ends
                # at ~0 before the queue empties (avoids ERROR_RUN_OUT_OF_QUEUE_ITEMS).
                motor.move_with_acceleration(UNDER_CAP_CMD_ACCEL, MOVE_TIME_S)
                motor.move_with_acceleration(-UNDER_CAP_CMD_ACCEL, MOVE_TIME_S)
            except FatalError as e:
                raise AssertionError(f"under-cap move was rejected with FatalError({e.args[0] if e.args else None}); expected acceptance")
            time.sleep(2 * MOVE_TIME_S + 0.5)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            err = get_err(motor)
            if err != 0:
                raise AssertionError(f"under-cap move left fatal_error={err}, expected 0")
            print("  under-cap move accepted and completed cleanly.")

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
