#!/usr/bin/env python3
"""
Test: Set Device Alias Command

Requirements:
1. This test picks a random alias (0–251) and attempts to set the device alias to that value using the M3.set_device_alias() method.
2. After setting the alias, it calls M3.detect_devices() to verify that the alias was set correctly.
3. Some aliases are not allowed (254, 253, 252) and should trigger a fatal error; the test checks for this by using M3.get_status() after the timeout, using the previous valid alias.
4. The test accepts a command-line argument (--bootloader) to optionally enter bootloader mode before running the test.
5. To enter bootloader mode: call M3.system_reset(), wait WAIT_FOR_RESET_TIME (empirically determined to be 0.07s), then call M3.get_status() to confirm bootloader mode.
6. If M3.get_status() is called within WAIT_FOR_RESET_TIME after reset, the application will not start and the device will remain in bootloader mode.
7. All steps, results, and errors are printed for verification.

"""

import servomotor
from servomotor import M3
import argparse
import random
import time
import sys

WAIT_FOR_RESET_TIME = 0.07  # Empirically determined: between 0.002 and 0.13 for firmware upgrade to work
DELAY_AFTER_SET_DEVICE_ALIAS = 2.0
INVALID_ALIASES = [254, 253, 252]
ERROR_BAD_ALIAS = 50  # From bootloader C code (based on observed output)
BOOTLOADER_FLAG_BIT = 0x01  # STATUS_IN_THE_BOOTLOADER_FLAG_BIT

def reset_device(motor, alias, go_to_bootloader, verbose):
    """
    General reset function.
    If alias == 255 (broadcast), perform reset, call get_status, but do NOT check the result.
    If alias != 255, check bootloader status as appropriate.
    """
    motor.set_alias(alias)
    print(f"Resetting device at alias {alias} (go_to_bootloader={go_to_bootloader})...")
    motor.system_reset(verbose=verbose)
    if go_to_bootloader:
        print("Trying to go into the bootloader, so we will have only a very short delay after the reset")
        print(f"Sleeping for WAIT_FOR_RESET_TIME={WAIT_FOR_RESET_TIME}s after reset (bootloader mode).")
        time.sleep(WAIT_FOR_RESET_TIME)
        print("Calling get_status after reset (bootloader mode).")
        status = motor.get_status(verbose=verbose)
        print(f"Status after reset (bootloader): {status}")
        if alias == 255:
            print("No status check performed for broadcast alias 255 after reset.")
        else:
            if not status or not isinstance(status, list) or len(status) < 2:
                raise AssertionError("No response after reset: cannot confirm device is in bootloader mode!")
            flags = status[0]
            in_bootloader = (flags & BOOTLOADER_FLAG_BIT) != 0
            assert in_bootloader, "Device is NOT in bootloader mode after reset!"
            print("Device is in bootloader mode after reset.")
    else:
        print("Not trying to go into the bootloader, so we will have a long delay to allow the applicaiton to start up")
        print("Sleeping for 2.0s after reset (normal mode).")
        time.sleep(2.0)
        print("Calling get_status after reset (normal mode).")
        status = motor.get_status()
        print(f"Status after reset (normal): {status}")
        if alias == 255:
            print("No status check performed for broadcast alias 255 after reset.")
        else:
            if not status or not isinstance(status, list) or len(status) < 2:
                in_bootloader = False
            else:
                flags = status[0]
                in_bootloader = (flags & BOOTLOADER_FLAG_BIT) != 0
            assert not in_bootloader, "Device is still in bootloader mode after reset!"
            assert status[1] == 0, "Device did not reset correctly to a working state (possibly out of the fatal error condition)"
            print("Device is in normal mode after reset.")

def test_set_device_alias(motor, current_alias, alias_to_set, expect_fatal_error, verbose, go_to_bootloader, description=""):
    """
    General test for setting device alias.
    Parameters:
        motor: M3 object
        current_alias: int, alias to use for sending the set command
        alias_to_set: int, alias to set
        expect_fatal_error: bool, whether a fatal error is expected
        verbose: bool, print detailed output
        go_to_bootloader: bool, whether to reset to bootloader after fatal error
        description: str, description of the test case
    Returns:
        (passed: bool, new_alias: int)
    """
    try:
        print(f"\n--- {description} ---")
        print(f"Setting alias to {alias_to_set} (expect fatal error: {expect_fatal_error})")
        motor.set_alias(current_alias)
        set_alias_timeout = False
        try:
            print(f"Calling set_device_alias({alias_to_set})")
            motor.set_device_alias(alias_to_set)
            print(f"set_device_alias({alias_to_set}) returned")
        except Exception as e:
            # Timeout is only expected for set_device_alias when expect_fatal_error is True
            if "timeout" in str(e).lower() and expect_fatal_error:
                set_alias_timeout = True
                print(f"Set alias to {alias_to_set} timed out as expected for invalid alias.")
            else:
                print(f"set_device_alias({alias_to_set}) exception: {e}")
                raise

        if expect_fatal_error:
            print("Sleeping for 0.1s after setting invalid alias before get_status.")
            time.sleep(0.1)
            # Do NOT update current_alias; use previous valid alias for get_status and reset
            motor.set_alias(current_alias)
            try:
                print(f"Calling get_status after invalid alias set (should use previous valid alias {current_alias})")
                status = motor.get_status()
                print(f"get_status returned: {status}")
            except Exception as e:
                print(f"FAILED: get_status timed out after setting invalid alias {alias_to_set}, which is NOT expected.")
                return False, current_alias
            if not status or not isinstance(status, list) or len(status) < 2:
                print(f"FAILED: No valid response after setting invalid alias {alias_to_set}, cannot confirm fatal error.")
                return False, current_alias
            fatal_error_code = status[1]
            if fatal_error_code != ERROR_BAD_ALIAS:
                print(f"FAILED: Expected fatal error code {ERROR_BAD_ALIAS} for alias {alias_to_set}, but got {fatal_error_code}.")
                return False, current_alias
            print(f"Correct fatal error detected for alias {alias_to_set} (fatal_error_code={fatal_error_code}). Resetting device...")
            reset_device(motor, current_alias, go_to_bootloader, verbose)
            return True, current_alias  # Alias does NOT change!
        else:
            print(f"Sleeping for {DELAY_AFTER_SET_DEVICE_ALIAS}s after setting alias before detect_devices.")
            time.sleep(DELAY_AFTER_SET_DEVICE_ALIAS)
            motor.set_alias(alias_to_set)
            print(f"Calling detect_devices after setting alias {alias_to_set}")
            detected_devices = motor.detect_devices()
            print(f"detect_devices returned: {detected_devices}")
            found = any(dev[1] == alias_to_set for dev in detected_devices)
            if alias_to_set == 255:
                # After alias removal, confirm with detect_devices that the device appears with alias 255
                found_255 = any(dev[1] == 255 for dev in detected_devices)
                if found_255:
                    print("Alias removal confirmed: device found with alias 255.")
                    return True, alias_to_set
                else:
                    print("FAILED: Alias removal not confirmed, device with alias 255 not found.")
                    return False, alias_to_set
            if not found:
                print(f"FAILED: Alias {alias_to_set} was not found in detected devices!")
                return False, alias_to_set
            print(f"Alias {alias_to_set} successfully set and detected.")
            return True, alias_to_set
    except Exception as e:
        print(f"FAILED: {description}: {e}")
        return False, current_alias  # Always return the previous valid alias on failure

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the 'Set device alias' command.")
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port device name (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('--bootloader', action='store_true', help='Enter bootloader mode before running the test')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    args = parser.parse_args()

    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port(timeout=1.5)
    try:
        # Create a single M3 object, start with broadcast alias
        motor = M3(255, verbose=args.verbose)
        # Initial reset with broadcast alias (no status check)
        reset_device(motor, 255, args.bootloader, args.verbose)
        all_passed = True

        # (1) Test setting a random valid alias (0–251) first
        random_valid_alias = random.randint(0, 251)
        passed, current_alias = test_set_device_alias(motor, 255, random_valid_alias, False, args.verbose, args.bootloader, f"Test random valid alias {random_valid_alias}")
        all_passed &= passed

        # (2) Test invalid aliases (254, 253, 252)
        for invalid_alias in INVALID_ALIASES:
            passed, current_alias = test_set_device_alias(motor, current_alias, invalid_alias, True, args.verbose, args.bootloader, f"Test invalid alias {invalid_alias}")
            all_passed &= passed

        # (3) Test alias removal (set to 255) last
        passed, current_alias = test_set_device_alias(motor, current_alias, 255, False, args.verbose, args.bootloader, "Test alias removal (set to 255)")
        all_passed &= passed

        if all_passed:
            print("PASSED")
            sys.exit(0)
        else:
            print("FAILED: One or more alias tests failed.")
            sys.exit(1)
    finally:
        servomotor.close_serial_port()