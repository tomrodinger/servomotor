#!/usr/bin/env python3
"""
Per-command test for "Disable MOSFETs" (cmd 0).

Cmd 0 has no input and returns a success response. Disabling the MOSFETs
de-energises the motor driver bridge; the firmware reflects this by clearing
status bit 1 (MOSFETs enabled).

What this test verifies:
  1. From an enabled state, disable_mosfets clears status bit 1, with no
     fatal error.
  2. Disabling again while already disabled is idempotent (stays disabled,
     no fatal error).
  3. A reset leaves the MOSFETs disabled (the baseline disable_mosfets
     drives toward).

(The complementary set behaviour is covered by test_enable_mosfets.py.)
"""

import argparse
import sys
import time
import servomotor

BOOTLOADER_BIT = 1 << 0
MOSFETS_ENABLED_BIT = 1 << 1
RESET_DELAY_S = 1.5


def get_flags_err(motor, label):
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    print(f"  {label}: flags={flags:#06x}, fatal_error={err}")
    if flags & BOOTLOADER_BIT:
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#06x})")
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err}, expected 0")
    return flags, err


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Disable MOSFETs' (cmd 0).")
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
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device and enabling MOSFETs (to set up a disable)...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            motor.enable_mosfets()
            flags, _ = get_flags_err(motor, "after enable_mosfets")
            if not (flags & MOSFETS_ENABLED_BIT):
                raise AssertionError("enable_mosfets did not set the MOSFET-enabled bit; "
                                     "cannot test disable from a known-enabled state")

            print("Disabling MOSFETs...")
            motor.disable_mosfets()
            flags, _ = get_flags_err(motor, "after disable_mosfets")
            if flags & MOSFETS_ENABLED_BIT:
                raise AssertionError("disable_mosfets did not clear the MOSFET-enabled bit")

            print("Disabling again (must be idempotent)...")
            motor.disable_mosfets()
            flags, _ = get_flags_err(motor, "after second disable_mosfets")
            if flags & MOSFETS_ENABLED_BIT:
                raise AssertionError("MOSFET-enabled bit set after a redundant disable_mosfets")

            print("Resetting; baseline must also be disabled...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            flags, _ = get_flags_err(motor, "after reset")
            if flags & MOSFETS_ENABLED_BIT:
                raise AssertionError("MOSFETs enabled after reset, expected disabled")

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
