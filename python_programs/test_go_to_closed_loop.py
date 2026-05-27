#!/usr/bin/env python3
"""
Per-command test for "Go to closed loop" (cmd 17).

Cmd 17 has no input and returns a success response. It runs the procedure
that takes the motor from open-loop to closed-loop (field-oriented) control.
This requires a *calibrated* motor; on an uncalibrated motor closed-loop
control misbehaves. The firmware reflects the result in status bit 2
(closed-loop mode), and bit 5 marks the procedure being in progress.

What this test verifies:
  1. Before the command (just enabled, open loop) the closed-loop flag is 0.
  2. After go_to_closed_loop the closed-loop flag (bit 2) becomes 1, the
     MOSFETs-enabled flag (bit 1) stays 1, and there is no fatal error.
  3. A small move executes in closed-loop mode and lands on target, proving
     closed-loop control is actually functional (not just the flag flipped).

If the motor is not calibrated the closed-loop entry will fail or report a
fatal error; that is a real failure of the test's precondition and is
reported as such (the bench M17 is calibrated — see WORK_CHECKLIST.md).
"""

import argparse
import sys
import time
import servomotor

BOOTLOADER_BIT = 1 << 0
MOSFETS_ENABLED_BIT = 1 << 1
CLOSED_LOOP_BIT = 1 << 2
COUNTS_PER_ROTATION = 3276800

RESET_DELAY_S = 1.5
ENABLE_SETTLE_S = 0.3
CLOSED_LOOP_TIMEOUT_S = 6.0
TOLERANCE_COUNTS = 4000   # closed-loop holding tolerance (hall-based, not exact)


def get_flags_err(motor, label):
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if flags & BOOTLOADER_BIT:
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#06x})")
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err}, expected 0")
    return flags, err


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Go to closed loop' (cmd 17).")
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

            print("\nResetting and enabling MOSFETs (open loop)...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            flags, _ = get_flags_err(motor, "open loop (before go_to_closed_loop)")
            if flags & CLOSED_LOOP_BIT:
                raise AssertionError("closed-loop flag set before go_to_closed_loop was issued")
            print(f"  open loop: flags={flags:#06x}")

            print("Issuing go_to_closed_loop and polling for the closed-loop flag...")
            motor.go_to_closed_loop()
            deadline = time.time() + CLOSED_LOOP_TIMEOUT_S
            flags = 0
            while time.time() < deadline:
                flags, _ = get_flags_err(motor, "during go_to_closed_loop")
                if flags & CLOSED_LOOP_BIT:
                    break
                time.sleep(0.1)
            print(f"  after procedure: flags={flags:#06x}")
            if not (flags & CLOSED_LOOP_BIT):
                raise AssertionError(f"closed-loop flag never set within {CLOSED_LOOP_TIMEOUT_S}s "
                                     f"(flags={flags:#06x}); is the motor calibrated?")
            if not (flags & MOSFETS_ENABLED_BIT):
                raise AssertionError(f"MOSFETs-enabled flag dropped after go_to_closed_loop (flags={flags:#06x})")

            print("Running a small closed-loop move to confirm control is functional...")
            motor.zero_position()
            target = COUNTS_PER_ROTATION // 8
            motor.trapezoid_move(target, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            time.sleep(0.3)
            pos = motor.get_position()
            flags, _ = get_flags_err(motor, "after closed-loop move")
            print(f"  closed-loop move ended at {pos} (target ~{target}), flags={flags:#06x}")
            if not (flags & CLOSED_LOOP_BIT):
                raise AssertionError("dropped out of closed loop during the move")
            if abs(pos - target) > TOLERANCE_COUNTS:
                raise AssertionError(f"closed-loop move ended at {pos}, expected ~{target} "
                                     f"(diff {abs(pos - target)} > {TOLERANCE_COUNTS})")

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
