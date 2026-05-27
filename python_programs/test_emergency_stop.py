#!/usr/bin/env python3
"""
Per-command test for "Emergency stop" (cmd 12).

Cmd 12 has no input and returns a success response. In firmware
(motor_control.c: emergency_stop) it disables the MOSFETs and clears the
movement queue immediately. It does not latch a fatal error.

What this test verifies:
  1. With MOSFETs enabled and several moves queued (queue > 0, status bit 1
     set), emergency_stop clears the queue to 0 and clears the MOSFET-enabled
     bit, leaving no fatal error.
  2. After an emergency stop the device is still responsive and can be
     re-enabled and commanded again (the stop is recoverable without a reset).
"""

import argparse
import sys
import time
import servomotor

BOOTLOADER_BIT = 1 << 0
MOSFETS_ENABLED_BIT = 1 << 1
RESET_DELAY_S = 1.5

N_MOVES = 5
MOVE_TIME_S = 0.5
SMALL_MOVE_COUNTS = 20000


def get_flags_err(motor, label):
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    print(f"  {label}: flags={flags:#06x}, fatal_error={err}, queue={motor.get_n_queued_items()}")
    if flags & BOOTLOADER_BIT:
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#06x})")
    return flags, err


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Emergency stop' (cmd 12).")
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

            print("\nResetting, enabling MOSFETs, queueing moves...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            for i in range(N_MOVES):
                disp = SMALL_MOVE_COUNTS if (i % 2 == 0) else -SMALL_MOVE_COUNTS
                motor.trapezoid_move(disp, MOVE_TIME_S)

            flags, err = get_flags_err(motor, "before emergency_stop")
            if not (flags & MOSFETS_ENABLED_BIT):
                raise AssertionError("MOSFETs not enabled before emergency_stop; setup failed")
            if motor.get_n_queued_items() == 0:
                raise AssertionError("queue empty before emergency_stop; setup failed")

            print("Issuing emergency_stop...")
            motor.emergency_stop()
            time.sleep(0.2)

            flags, err = get_flags_err(motor, "after emergency_stop")
            queue = motor.get_n_queued_items()
            if queue != 0:
                raise AssertionError(f"queue not cleared by emergency_stop; still {queue}")
            if flags & MOSFETS_ENABLED_BIT:
                raise AssertionError("MOSFETs still enabled after emergency_stop")
            if err != 0:
                raise AssertionError(f"emergency_stop latched a fatal error ({err}); expected 0 "
                                     f"(it should be recoverable without a reset)")

            print("Verifying the device is still responsive (re-enable + small move)...")
            motor.enable_mosfets()
            motor.zero_position()
            motor.trapezoid_move(SMALL_MOVE_COUNTS, 0.5)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            flags, err = get_flags_err(motor, "after recovery move")
            if err != 0:
                raise AssertionError(f"device unhealthy after emergency-stop recovery; fatal_error={err}")

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
