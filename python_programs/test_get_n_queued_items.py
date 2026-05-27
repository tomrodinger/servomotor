#!/usr/bin/env python3
"""
Per-command test for "Get n queued items" (cmd 11).

Cmd 11 output: (u8 queueSize) — number of items in the movement queue,
between 0 and 32. If < 32 you can add more moves.

What this test verifies:
  1. At rest (just reset, MOSFETs enabled) the queue is empty (0).
  2. Queueing N non-overlapping timed moves makes the reported count rise to
     N (read fast, before the first move drains) and the count is never
     reported above the firmware's 32-slot maximum.
  3. The queue drains back to 0 once the moves finish — proving the count
     tracks live queue depth, not a constant.
"""

import argparse
import sys
import time
import servomotor

RESET_DELAY_S = 1.5
MAX_QUEUE = 32
N_MOVES = 5
MOVE_TIME_S = 0.4          # each queued move lasts this long
SMALL_MOVE_COUNTS = 20000  # tiny displacement so total motion is negligible


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get n queued items' (cmd 11).")
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

            print("\nResetting device and enabling MOSFETs...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()

            n = motor.get_n_queued_items()
            print(f"  queue at rest: {n}")
            if not isinstance(n, int):
                raise AssertionError(f"get_n_queued_items returned non-int: {n!r}")
            if n != 0:
                raise AssertionError(f"queue at rest is {n}, expected 0")

            print(f"Queueing {N_MOVES} moves of {MOVE_TIME_S}s each (alternating direction)...")
            for i in range(N_MOVES):
                disp = SMALL_MOVE_COUNTS if (i % 2 == 0) else -SMALL_MOVE_COUNTS
                motor.trapezoid_move(disp, MOVE_TIME_S)

            n = motor.get_n_queued_items()
            print(f"  queue right after queueing {N_MOVES} moves: {n}")
            if n > MAX_QUEUE:
                raise AssertionError(f"queue reported {n} > firmware max {MAX_QUEUE}")
            # A trapezoid move expands into up to 3 queue entries (accel / cruise /
            # decel), so N_MOVES enqueues roughly 3*N_MOVES slots. We only assert
            # the queue is non-trivially populated (most moves still pending; one
            # may have begun draining between the last enqueue and this read) and
            # never exceeds the firmware's 32-slot cap.
            if n < N_MOVES - 1:
                raise AssertionError(f"queue is {n} right after queueing {N_MOVES} moves; "
                                     f"expected the queue to be populated")

            print("Waiting for the queue to drain...")
            deadline = time.time() + (N_MOVES * MOVE_TIME_S + 3.0)
            while time.time() < deadline:
                if motor.get_n_queued_items() == 0:
                    break
                time.sleep(0.02)
            n = motor.get_n_queued_items()
            print(f"  queue after draining: {n}")
            if n != 0:
                raise AssertionError(f"queue did not drain to 0; still {n}")

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
