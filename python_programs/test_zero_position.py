#!/usr/bin/env python3
"""
Per-command test for "Zero position" (cmd 13).

Cmd 13 has no input and returns a success response. It redefines the current
position as the new origin (0) — the desired-position counter is reset to 0
without the motor moving.

What this test verifies:
  1. After a move to a non-zero position, zero_position makes get_position
     read ~0.
  2. The shaft does not move as a result of zeroing — a second get_position
     read immediately after stays at ~0 (zeroing redefines the origin, it
     does not command a move back).
  3. A subsequent relative move from the new origin lands where expected,
     proving the origin really shifted (a move of +Q from the new zero ends
     near +Q counts, not near old_position+Q).
"""

import argparse
import sys
import time
import servomotor

COUNTS_PER_ROTATION = 3276800
RESET_DELAY_S = 1.5
TOLERANCE_COUNTS = 50


def wait_for_idle(motor):
    while motor.get_n_queued_items() > 0:
        time.sleep(0.01)
    time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Zero position' (cmd 13).")
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

            print("\nResetting, enabling MOSFETs, zeroing baseline...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            motor.zero_position()

            half_turn = COUNTS_PER_ROTATION // 2
            print(f"Moving to {half_turn} counts (1/2 rotation)...")
            motor.trapezoid_move(half_turn, 1.0)
            wait_for_idle(motor)
            pos = motor.get_position()
            print(f"  position before zeroing: {pos}")
            if abs(pos - half_turn) > TOLERANCE_COUNTS:
                raise AssertionError(f"setup move ended at {pos}, expected ~{half_turn}")

            print("Calling zero_position...")
            motor.zero_position()
            pos = motor.get_position()
            print(f"  position immediately after zeroing: {pos}")
            if abs(pos) > TOLERANCE_COUNTS:
                raise AssertionError(f"position after zero_position is {pos}, expected ~0")

            # The shaft must not have physically moved due to zeroing: read again.
            time.sleep(0.2)
            pos2 = motor.get_position()
            print(f"  position again (must stay ~0, no motion induced): {pos2}")
            if abs(pos2) > TOLERANCE_COUNTS:
                raise AssertionError(f"position drifted to {pos2} after zeroing; zeroing should not move the shaft")

            # Prove the origin really shifted: a relative move from new zero.
            quarter = COUNTS_PER_ROTATION // 4
            print(f"Moving +{quarter} counts from the new origin...")
            motor.trapezoid_move(quarter, 1.0)
            wait_for_idle(motor)
            pos3 = motor.get_position()
            print(f"  position after +1/4-turn from new origin: {pos3}")
            if abs(pos3 - quarter) > TOLERANCE_COUNTS:
                raise AssertionError(f"move from new origin ended at {pos3}, expected ~{quarter} "
                                     f"(origin did not shift correctly)")

            print("Returning to new origin and cleaning up...")
            motor.trapezoid_move(0, 1.0)
            wait_for_idle(motor)
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
