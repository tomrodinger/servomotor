#!/usr/bin/env python3
"""
Test: Set Device Alias Command

Requirements:
1. This test picks a random alias (0–251) and attempts to set the device alias to that value using the M3.set_device_alias() method.
2. After setting the alias, it calls M3.detect_devices() to verify that the alias was set correctly.
3. Some aliases are not allowed (254, 253, 252) and should trigger a fatal error; the test checks for this by using M3.get_status() after the timeout, using the previous valid alias.
4. The test accepts a command-line argument (--bootloader) to optionally enter bootloader mode before running the test.
5. To enter bootloader mode: call M3.system_reset(), wait GO_TO_BOOTlOADER_RESET_TIME (empirically determined to be 0.07s), then call M3.get_status() to confirm bootloader mode.
6. If M3.get_status() is called within GO_TO_BOOTlOADER_RESET_TIME after reset, the application will not start and the device will remain in bootloader mode.
7. All steps, results, and errors are printed for verification.
"""

import servomotor
from servomotor import M3
import argparse
import random
import time
import sys

GO_TO_BOOTlOADER_RESET_TIME = 0.07  # Empirically determined: between 0.002 and 0.13 for firmware upgrade to work
DONT_GO_TO_BOOTlOADER_RESET_TIME = 2.0
FLASH_WRITE_TIME = 0.02 # when we set a new alias, it will be saved to flash, which take a little bit of time
INVALID_ALIASES = [254, 253, 252]
ERROR_BAD_ALIAS = 50  # From bootloader C code (based on observed output)
BOOTLOADER_FLAG_BIT = 0x01  # STATUS_IN_THE_BOOTLOADER_FLAG_BIT

def check_the_status(motor, alias_to_set, go_to_bootloader, expect_fatal_error, verbose=2):
    print("Calling get_status after reset so that we can check if we are in the mode that we expect (bootloader or application).")
    status = motor.get_status(verbose=verbose)
    print(f"Status after reset (bootloader): {status}")
    if alias_to_set == 255:
        print("No status check performed for broadcast alias 255 after reset.")
    else:
        if not status or not isinstance(status, list) or len(status) < 2:
            raise AssertionError("No response after reset: cannot confirm device is in bootloader mode!")
        flags = status[0]
        fatal_error_code = status[1]

        in_bootloader = (flags & BOOTLOADER_FLAG_BIT) != 0
        if go_to_bootloader:
            assert in_bootloader, "Device is NOT in bootloader mode after reset, which is unexpected"
            print("Device is in bootloader mode after reset, which is correct")
        else:
            assert not in_bootloader, "Device IS in bootloader mode after reset, which is unexpected"
            print("Device is not in bootloader mode, which is correct")

        if expect_fatal_error:
            if fatal_error_code != ERROR_BAD_ALIAS:
                raise AssertionError(f"FAILED: Expected fatal error code {ERROR_BAD_ALIAS} for alias {alias_to_set}, but got {fatal_error_code}.")            
            print(f"Correct fatal error detected for alias {alias_to_set} (fatal_error_code={fatal_error_code}).")
        else:
            if fatal_error_code != 0:
                raise AssertionError(f"FAILED: Expected fatal error code 0 for alias {alias_to_set}, but got {fatal_error_code}.")            
            print(f"Correct fatal error detected for alias {alias_to_set} (fatal_error_code={fatal_error_code}).")

def reset_device(motor, alias, go_to_bootloader, verbose=2):
    """
    General reset function.
    If alias == 255 (broadcast), perform reset, call get_status, but do NOT check the result.
    If alias != 255, check bootloader status as appropriate.
    """
    motor.use_this_alias_or_unique_id(alias)
    print(f"Resetting device at alias {alias} (go_to_bootloader={go_to_bootloader})...")
    motor.system_reset(verbose=verbose)
    if go_to_bootloader:
        print("Trying to go into the bootloader, so we will have only a very short delay after the reset")
        print(f"Sleeping for {GO_TO_BOOTlOADER_RESET_TIME}s after reset (bootloader mode).")
        time.sleep(GO_TO_BOOTlOADER_RESET_TIME)
    else:
        print("Not trying to go into the bootloader, so we will have a long delay to allow the applicaiton to start up")
        print(f"Sleeping for {DONT_GO_TO_BOOTlOADER_RESET_TIME}s after reset (normal mode).")
        time.sleep(DONT_GO_TO_BOOTlOADER_RESET_TIME)
    check_the_status(motor, alias, go_to_bootloader, False, verbose=verbose)

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
        motor.use_this_alias_or_unique_id(current_alias)
        try:
            print(f"Calling set_device_alias({alias_to_set})")
            ret = motor.set_device_alias(alias_to_set)
            print(f"set_device_alias({alias_to_set}) returned: {ret}")
        except Exception as e:
            # Timeout is only expected for set_device_alias when expect_fatal_error is True
            if "timeout" in str(e).lower() and expect_fatal_error:
                print(f"Set alias to {alias_to_set} timed out as expected for invalid alias.")
            else:
                print(f"set_device_alias({alias_to_set}) exception: {e}")
                raise

        if expect_fatal_error:
            print("Sleeping for 0.1s after setting invalid alias before get_status.")
            time.sleep(0.1)
            # Do NOT update current_alias; use previous valid alias for get_status and reset
            motor.use_this_alias_or_unique_id(current_alias)
            try:
                check_the_status(motor, current_alias, go_to_bootloader, expect_fatal_error, verbose=verbose)
            except:
                return False, current_alias
            print("Resetting...")
            reset_device(motor, current_alias, go_to_bootloader, verbose)
            return True, current_alias  # Alias does NOT change!
        else:
            if go_to_bootloader:
                delay_time = GO_TO_BOOTlOADER_RESET_TIME + FLASH_WRITE_TIME
                print(f"Sleeping for {delay_time:0.3}s after setting alias before detect_devices. We will try to stay in the bootloader, so we will use a short delay")
            else:
                delay_time = DONT_GO_TO_BOOTlOADER_RESET_TIME + FLASH_WRITE_TIME
                print(f"Sleeping for {delay_time:0.3}s after setting alias before detect_devices. We will not go to the bootloader, so we will use a long delay")
            time.sleep(delay_time)
            motor.use_this_alias_or_unique_id(alias_to_set)

            check_the_status(motor, alias_to_set, go_to_bootloader, expect_fatal_error, verbose=verbose)

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
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('--bootloader', action='store_true', help='Enter bootloader mode before running the test')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat all tests (default: 1)')
    args = parser.parse_args()

    if args.verbose:
        args.verbose = 2
    # Statistics: {test_case_name: {'pass': int, 'fail': int}}
    test_stats = {}

    overall_passed = True

    for repeat_idx in range(args.repeat):
        if args.repeat > 1:
            print(f"\n========== REPEAT {repeat_idx + 1} of {args.repeat} ==========")
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
            test_name_valid = f"Random valid alias {random_valid_alias}"
            passed, current_alias = test_set_device_alias(motor, 255, random_valid_alias, False, args.verbose, args.bootloader, f"Test random valid alias {random_valid_alias}")
            if test_name_valid not in test_stats:
                test_stats[test_name_valid] = {'pass': 0, 'fail': 0}
            if passed:
                test_stats[test_name_valid]['pass'] += 1
            else:
                test_stats[test_name_valid]['fail'] += 1
            all_passed &= passed

            # (2) Test invalid aliases (254, 253, 252)
            for invalid_alias in INVALID_ALIASES:
                test_name_invalid = f"Invalid alias {invalid_alias}"
                passed, current_alias = test_set_device_alias(motor, current_alias, invalid_alias, True, args.verbose, args.bootloader, f"Test invalid alias {invalid_alias}")
                if test_name_invalid not in test_stats:
                    test_stats[test_name_invalid] = {'pass': 0, 'fail': 0}
                if passed:
                    test_stats[test_name_invalid]['pass'] += 1
                else:
                    test_stats[test_name_invalid]['fail'] += 1
                all_passed &= passed

            # (3) Test alias removal (set to 255) last
            test_name_removal = "Alias removal (set to 255)"
            passed, current_alias = test_set_device_alias(motor, current_alias, 255, False, args.verbose, args.bootloader, "Test alias removal (set to 255)")
            if test_name_removal not in test_stats:
                test_stats[test_name_removal] = {'pass': 0, 'fail': 0}
            if passed:
                test_stats[test_name_removal]['pass'] += 1
            else:
                test_stats[test_name_removal]['fail'] += 1
            all_passed &= passed

            # Print per-repeat statistics
            print("\n--- Statistics after this repeat ---")
            for test_name, stats in test_stats.items():
                total = stats['pass'] + stats['fail']
                print(f"{test_name}: {stats['pass']} passed, {stats['fail']} failed (total: {total})")

            if all_passed:
                print("PASSED (this repeat)")
            else:
                print("FAILED: One or more alias tests failed (this repeat).")
            overall_passed &= all_passed
        finally:
            servomotor.close_serial_port()

    # Grand summary
    print("\n========== GRAND SUMMARY ==========")
    for test_name, stats in test_stats.items():
        total = stats['pass'] + stats['fail']
        print(f"{test_name}: {stats['pass']} passed, {stats['fail']} failed (total: {total})")
    if overall_passed:
        print("ALL REPEATS PASSED")
        sys.exit(0)
    else:
        print("FAILED: One or more tests failed in at least one repeat.")
        sys.exit(1)