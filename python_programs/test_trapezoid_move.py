#!/usr/bin/env python3
"""
Per-command test for "Trapezoid move" (cmd 2).

Cmd 2 input: (i32 displacement, u32 duration). It performs a *relative* move
of `displacement` (position units) over `duration` (time units), with a
trapezoidal velocity profile. It returns a success response. Unlike
go_to_position (absolute), trapezoid_move is relative to the current desired
position.

What this test verifies:
  1. A relative move of +D ends at start+D (within tolerance).
  2. Moves are relative and accumulate: two consecutive +D moves end at +2D,
     and a -2D move returns to the origin.
  3. The move takes approximately the requested duration (a trapezoid move's
     wall-clock time should match `duration`, not finish instantly), proving
     the duration argument is honoured.
  4. The relative displacement is consistent across position units (a move
     expressed in shaft_rotations lands at the same place as the equivalent
     move in encoder_counts).
"""

import argparse
import sys
import time
import servomotor

COUNTS_PER_ROTATION = 3276800
RESET_DELAY_S = 1.5
TOLERANCE_COUNTS = 80


def wait_for_idle(motor):
    while motor.get_n_queued_items() > 0:
        time.sleep(0.01)
    time.sleep(0.1)


def get_counts(motor):
    motor.set_position_unit("encoder_counts")
    return motor.get_position()


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Trapezoid move' (cmd 2).")
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

            print("\nResetting, enabling MOSFETs, zeroing...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            motor.zero_position()

            D = COUNTS_PER_ROTATION // 4   # 1/4 turn per relative move
            duration = 1.0

            print(f"[1] Relative +{D} counts over {duration}s, timing the move...")
            t0 = time.time()
            motor.set_position_unit("encoder_counts")
            motor.trapezoid_move(D, duration)
            wait_for_idle(motor)
            elapsed = time.time() - t0
            pos = get_counts(motor)
            print(f"    ended at {pos} (expect ~{D}), elapsed {elapsed:.2f}s (expect ~{duration}s)")
            if abs(pos - D) > TOLERANCE_COUNTS:
                raise AssertionError(f"first move ended at {pos}, expected ~{D}")
            if not (duration * 0.6 <= elapsed <= duration + 1.0):
                raise AssertionError(f"move took {elapsed:.2f}s, expected ~{duration}s "
                                     f"(duration argument not honoured)")

            print(f"[2] Another relative +{D} must accumulate to ~{2*D}...")
            motor.trapezoid_move(D, duration)
            wait_for_idle(motor)
            pos = get_counts(motor)
            print(f"    ended at {pos} (expect ~{2*D})")
            if abs(pos - 2 * D) > TOLERANCE_COUNTS:
                raise AssertionError(f"after two +{D} moves position is {pos}, expected ~{2*D}")

            print(f"[3] Relative -{2*D} must return to the origin...")
            motor.trapezoid_move(-2 * D, duration)
            wait_for_idle(motor)
            pos = get_counts(motor)
            print(f"    ended at {pos} (expect ~0)")
            if abs(pos) > TOLERANCE_COUNTS:
                raise AssertionError(f"after returning, position is {pos}, expected ~0")

            print("[4] Same displacement expressed in shaft_rotations lands identically...")
            motor.set_position_unit("shaft_rotations")
            motor.trapezoid_move(0.25, duration)   # 1/4 rotation == D counts
            wait_for_idle(motor)
            pos = get_counts(motor)
            print(f"    ended at {pos} (expect ~{D})")
            if abs(pos - D) > TOLERANCE_COUNTS:
                raise AssertionError(f"shaft_rotations move ended at {pos}, expected ~{D}")

            print("Returning to origin and cleaning up...")
            motor.set_position_unit("encoder_counts")
            motor.trapezoid_move(-D, duration)
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
