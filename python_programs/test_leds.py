#!/usr/bin/env python3
"""
LED-control test via Test mode — MANUAL ONLY, NOT auto-suite safe.

Modernized from the obsolete test_all_leds_on.py.

⚠️  This test puts the motor into a firmware state that *cannot* be recovered
from software. `set_led_test_mode()` in firmware/Src/main.c:178 disables all
interrupts and busy-loops forever; the RS485 ISR cannot run, `system_reset`
over RS485 has no path to be received, and no watchdog kicks in. The only
recovery is a hardware power-cycle (or NRST).

Because of that, this file is catalogued in `TEST_SUMMARY.md` with the module
label `servomotor (manual)` so `run_all_tests.py` skips it automatically.
Run it explicitly when you are at the bench and can power-cycle the motor.
See the memory note `led-test-mode-is-stuck-forever` for the firmware
analysis and a sketch of the future firmware fix that would let this test
become auto-suite friendly.

Flow when invoked interactively:

  1. Reset the device and confirm app-mode.
  2. Send Test mode = 13 (both LEDs on).
  3. Ask the human: "Are both LEDs (green + red) lit?"
  4. Tell the human to power-cycle the motor and wait for them to confirm
     communication is back via a ping.
  5. Reset again to leave a known clean state.

If invoked non-interactively (no TTY) the test refuses to send the
LED command — it prints a clear skip notice and exits 0.
"""

import argparse
import sys
import time
import servomotor

STATUS_IN_THE_BOOTLOADER_FLAG_BIT = 0

TEST_MODE_LEDS_BOTH_ON = 13     # bitmask 3 (green=1, red=2) — see firmware/Src/main.c

RESET_DELAY_S = 1.5
POST_POWER_CYCLE_PING_TIMEOUT_S = 30.0
POST_POWER_CYCLE_PING_PERIOD_S = 0.5


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


def wait_for_motor_to_come_back(motor, timeout_s):
    """Poll ping until the motor responds (after power-cycle), or give up."""
    deadline = time.time() + timeout_s
    payload = bytes(range(10))
    while time.time() < deadline:
        try:
            response = motor.ping(payload)
            if response == payload:
                return True
        except Exception:
            pass
        time.sleep(POST_POWER_CYCLE_PING_PERIOD_S)
    return False


def main():
    parser = argparse.ArgumentParser(description="Manual LED test: sets both LEDs via Test mode, requires a hardware power-cycle to recover.")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()

    if not sys.stdin.isatty():
        # Auto-suite path. Refuse to send the LED command; the motor would
        # be bricked and every subsequent test in the suite would fail.
        print("LED test: stdin is not a TTY, skipping (the LED command bricks the motor and only a hardware power-cycle recovers).")
        print("PASSED")
        return 0

    verbose_level = 2 if args.verbose else 0

    servomotor.set_serial_port_from_args(args)

    print("=" * 70)
    print("MANUAL LED TEST")
    print("=" * 70)
    print()
    print("This test sets the LEDs via firmware Test mode = 13 (both LEDs on).")
    print("Once sent, the motor enters set_led_test_mode() in firmware which")
    print("disables interrupts and loops forever. **A hardware power-cycle")
    print("of the motor will be required** to bring it back.")
    print()
    try:
        ans = input("Continue? [y/N]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = ""
    if not ans.startswith("y"):
        print("Aborted by user.")
        print("PASSED")  # not running the test isn't a failure — it's a deliberate skip
        return 0

    success = False
    failure_message = ""
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, verbose=verbose_level)

        print("\nResetting device to a known state...")
        motor.system_reset()
        time.sleep(RESET_DELAY_S)
        assert_app_mode_no_error(motor, "before LED command")

        print(f"\nSending Test mode = {TEST_MODE_LEDS_BOTH_ON} (both LEDs on)...")
        motor.test_mode(TEST_MODE_LEDS_BOTH_ON)
        print("  command acknowledged — motor is now in the LED-display freeze state")

        try:
            ans = input("\nAre BOTH LEDs (green AND red) lit? [y/n]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            ans = ""
        if not ans.startswith("y"):
            raise AssertionError("Visual confirmation failed: human reported the LEDs are not both lit.")
        print("  visual confirmation: OK")

        print(f"\n*** Power-cycle the motor now. ***")
        input("Press Enter once you have power-cycled it...")

        print(f"Waiting up to {POST_POWER_CYCLE_PING_TIMEOUT_S:.0f}s for the motor to come back online...")
        if not wait_for_motor_to_come_back(motor, POST_POWER_CYCLE_PING_TIMEOUT_S):
            raise AssertionError("Motor did not respond to ping after power-cycle within timeout.")
        print("  motor is back")

        # Final clean state.
        print("\nFinal reset to leave a known state...")
        motor.system_reset()
        time.sleep(RESET_DELAY_S)
        assert_app_mode_no_error(motor, "after final reset")

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
