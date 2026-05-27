#!/usr/bin/env python3
"""
Per-command test for "Set maximum velocity" (cmd 3).

Cmd 3 input: (u32 maximumVelocity) in velocity units. It clamps the velocity
the firmware will allow. The firmware enforces it by *rejecting* a move that
would exceed the cap with ERROR_VEL_TOO_HIGH (code 16) — it does not silently
clamp.

What this test verifies (deterministic, no velocity measurement needed):
  1. With a low cap, a move_with_velocity commanding above the cap is rejected
     with ERROR_VEL_TOO_HIGH (16). This proves the cap took effect.
  2. With a high cap (after reset), a move_with_velocity commanding below the
     cap is accepted and runs with no fatal error. This proves the cap value
     actually changed behaviour rather than always rejecting.

A high maximum acceleration is set first so that the velocity limit, not the
acceleration limit, is the thing under test.
"""

import argparse
import sys
import time
import servomotor
from servomotor.communication import FatalError

ERROR_VEL_TOO_HIGH = 16
RESET_DELAY_S = 1.5

HIGH_ACCEL = 100.0          # rotations_per_second_squared — plenty to reach any velocity quickly
LOW_VEL_CAP = 0.5           # rotations_per_second
OVER_CAP_CMD_VEL = 5.0      # 10x the low cap → must be rejected
HIGH_VEL_CAP = 10.0
UNDER_CAP_CMD_VEL = 1.0     # comfortably under the high cap → must be accepted
MOVE_TIME_S = 1.0


def get_err(motor):
    status = motor.get_status()
    return status[1] if status and len(status) >= 2 else None


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Set maximum velocity' (cmd 3).")
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

            # --- Scenario A: low cap rejects an over-cap velocity ---
            print("\nResetting, enabling MOSFETs, high accel, LOW velocity cap...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            motor.zero_position()
            motor.set_maximum_acceleration(HIGH_ACCEL)
            motor.set_maximum_velocity(LOW_VEL_CAP)

            print(f"Commanding move_with_velocity({OVER_CAP_CMD_VEL}) — above the {LOW_VEL_CAP} cap, must be rejected...")
            rejected = False
            try:
                motor.move_with_velocity(OVER_CAP_CMD_VEL, MOVE_TIME_S)
            except FatalError as e:
                code = e.args[0] if e.args else None
                if code != ERROR_VEL_TOO_HIGH:
                    raise AssertionError(f"over-cap move rejected with FatalError({code}); expected {ERROR_VEL_TOO_HIGH} (ERROR_VEL_TOO_HIGH)")
                print(f"  correctly rejected with FatalError({code}) = ERROR_VEL_TOO_HIGH.")
                rejected = True
            if not rejected:
                err = get_err(motor)
                if err == ERROR_VEL_TOO_HIGH:
                    print(f"  correctly entered fatal_error({err}) = ERROR_VEL_TOO_HIGH.")
                else:
                    raise AssertionError(f"over-cap move was NOT rejected — max velocity not enforced (fatal_error={err}).")

            # --- Scenario B: high cap accepts an under-cap velocity ---
            print("\nResetting, enabling MOSFETs, high accel, HIGH velocity cap...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            motor.zero_position()
            motor.set_maximum_acceleration(HIGH_ACCEL)
            motor.set_maximum_velocity(HIGH_VEL_CAP)

            print(f"Commanding move_with_velocity({UNDER_CAP_CMD_VEL}) — under the {HIGH_VEL_CAP} cap, must be accepted...")
            try:
                motor.move_with_velocity(UNDER_CAP_CMD_VEL, MOVE_TIME_S)
                # A velocity move holds its velocity; queue a stop so the motor is
                # at rest before the queue empties (otherwise ERROR_RUN_OUT_OF_QUEUE_ITEMS).
                motor.move_with_velocity(0, 0.5)
            except FatalError as e:
                raise AssertionError(f"under-cap move was rejected with FatalError({e.args[0] if e.args else None}); expected acceptance")
            time.sleep(MOVE_TIME_S + 0.8)
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
