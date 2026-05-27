#!/usr/bin/env python3
"""
Per-command test for "Get status" (cmd 16).

Cmd 16 output: (u16 statusFlags, u8 fatalErrorCode).

statusFlags bits (from motor_commands.json):
  bit 0: in the bootloader (if set, all other flags are 0)
  bit 1: MOSFETs are enabled
  bit 2: motor is in closed-loop mode
  bit 3: executing calibration
  bit 4: executing homing
  bit 5: executing go-to-closed-loop procedure
  bit 6: busy with a time-consuming task
  bits 7..15: unused, must be 0

What this test verifies:
  1. After reset the device reports a usable (flags, err) pair, is in
     application mode (bit 0 clear), and has no fatal error (err == 0).
  2. The unused high bits (7..15) are 0.
  3. The MOSFET-enabled flag (bit 1) tracks reality: it is 0 after reset,
     becomes 1 after enable_mosfets, and 0 again after disable_mosfets.
     This proves the status command reports live state, not a constant.
"""

import argparse
import sys
import time
import servomotor

BOOTLOADER_BIT = 1 << 0
MOSFETS_ENABLED_BIT = 1 << 1
UNUSED_BITS_MASK = 0xFF80  # bits 7..15 must be 0

RESET_DELAY_S = 1.5


def read_status(motor, label):
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if not isinstance(flags, int) or not isinstance(err, int):
        raise AssertionError(f"{label}: get_status types wrong: flags={flags!r}, err={err!r}")
    if not (0 <= flags <= 0xFFFF):
        raise AssertionError(f"{label}: statusFlags out of u16 range: {flags}")
    if not (0 <= err <= 0xFF):
        raise AssertionError(f"{label}: fatalErrorCode out of u8 range: {err}")
    print(f"  {label}: flags={flags:#06x}, fatal_error={err}")
    return flags, err


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get status' (cmd 16).")
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
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="shaft_rotations",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("Reading status after reset (expect application mode, no fatal error)...")
            flags, err = read_status(motor, "after reset")
            if flags & BOOTLOADER_BIT:
                raise AssertionError(f"after reset: device is in the bootloader (flags={flags:#06x})")
            if err != 0:
                raise AssertionError(f"after reset: fatal_error_code={err}, expected 0")
            if flags & UNUSED_BITS_MASK:
                raise AssertionError(f"after reset: unused high bits set (flags={flags:#06x}, mask={UNUSED_BITS_MASK:#06x})")
            if flags & MOSFETS_ENABLED_BIT:
                raise AssertionError(f"after reset: MOSFETs report enabled, expected disabled (flags={flags:#06x})")

            print("Enabling MOSFETs; status bit 1 must become 1...")
            motor.enable_mosfets()
            flags, err = read_status(motor, "after enable_mosfets")
            if not (flags & MOSFETS_ENABLED_BIT):
                raise AssertionError(f"after enable_mosfets: MOSFET-enabled bit not set (flags={flags:#06x})")
            if err != 0:
                raise AssertionError(f"after enable_mosfets: fatal_error_code={err}, expected 0")

            print("Disabling MOSFETs; status bit 1 must become 0...")
            motor.disable_mosfets()
            flags, err = read_status(motor, "after disable_mosfets")
            if flags & MOSFETS_ENABLED_BIT:
                raise AssertionError(f"after disable_mosfets: MOSFET-enabled bit still set (flags={flags:#06x})")

            # Leave a clean state.
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
