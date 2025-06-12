#!/usr/bin/env python3

"""
Test: Test Mode Command (test_test_mode.py)

This test verifies the correct behavior of the "Test mode" command in the servomotor firmware/bootloader.
It covers both invalid and valid (fatal error-triggering) scenarios, and ensures the device responds as expected.

Test Scenarios:
1. Invalid Test Mode Parameter:
   - Sends a test_mode value >= 72 (e.g., 255), which is not supported by the firmware.
   - Expects the device to enter the ERROR_INVALID_TEST_MODE (code 53) fatal error state.
   - The device should NOT respond to the test_mode command (timeout is expected).
   - The test then calls get_status to confirm the fatal error code is set.

2. Valid Test Mode Parameter (Triggers Fatal Error):
   - Randomly selects a fatal error code N in the range 0–59.
   - Sends test_mode = N + 12, which, per firmware logic, triggers fatal_error(N).
   - The device should NOT respond to the test_mode command (timeout is expected).
   - The test then calls get_status to confirm the correct fatal error code is set.

Key Implementation Details:
- The test uses only the device's unique ID for all commands after initial detection, ensuring robust addressing even if the alias is 255.
- After each fatal error is triggered, the test performs a system reset to return the device to a known state before the next scenario.
- The test explicitly checks that a timeout occurs after sending a test_mode command that triggers a fatal error. If no timeout occurs, the test fails.
- The test supports --bootloader and --repeat options for flexibility and repeated validation.
- The test prints clear PASS/FAIL results and exits with code 0 on success, 1 on failure.

Firmware Mapping Reference:
- test_mode == 0: disables test mode
- test_mode 1–9: set_motor_test_mode
- test_mode 10–11: set_led_test_mode
- test_mode 12–71: triggers fatal_error(test_mode - 12), i.e., test_mode 12 triggers fatal_error(0), 13 triggers fatal_error(1), ..., 71 triggers fatal_error(59)
- test_mode >= 72: triggers fatal_error(ERROR_INVALID_TEST_MODE)

This test ensures that the "Test mode" command is robustly implemented and that the device's fatal error handling is working as intended.
"""

import servomotor
from servomotor import M3
import argparse
import time
import sys
import random
import os

GO_TO_BOOTLOADER_RESET_TIME = 0.07
DONT_GO_TO_BOOTLOADER_RESET_TIME = 2.0
ERROR_INVALID_TEST_MODE = 53
BOOTLOADER_FLAG_BIT = 0x01
MAX_FATAL_ERROR = 59  # test_mode 12–71 triggers fatal_error 0–59

def check_the_status(motor, go_to_bootloader, expect_fatal_error, verbose=2, expected_fatal_error_code=None):
    print("Calling get_status to check the status")
    if go_to_bootloader:
        print("Calling get_status to cause us to stay in the bootloader (has to be called fast after system reset)")
    else:
        print("Calling get_status to check the status")
    status = motor.get_status(verbose=verbose)
    print(f"Status after reset (bootloader): {status}")
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
        if expected_fatal_error_code is not None:
            if fatal_error_code != expected_fatal_error_code:
                raise AssertionError(f"FAILED: Expected fatal error code {expected_fatal_error_code} but got {fatal_error_code}.")
            print(f"Correct fatal error detected (fatal_error_code={fatal_error_code}).")
        else:
            if fatal_error_code == 0:
                raise AssertionError("FAILED: Expected a fatal error but got 0.")
            print(f"Fatal error detected (fatal_error_code={fatal_error_code}).")
    else:
        if fatal_error_code != 0:
            raise AssertionError(f"FAILED: Expected fatal error code 0 but got {fatal_error_code}.")
        print(f"Correct fatal error detected (fatal_error_code={fatal_error_code}).")

def reset_device(motor, go_to_bootloader, verbose=2):
    """
    General reset function using unique ID addressing.
    """
    print(f"Resetting device (go_to_bootloader={go_to_bootloader})...")
    motor.system_reset(verbose=verbose)
    if go_to_bootloader:
        print("Trying to go into the bootloader, so we will have only a very short delay after the reset")
        print(f"Sleeping for {GO_TO_BOOTLOADER_RESET_TIME}s after reset (bootloader mode).")
        time.sleep(GO_TO_BOOTLOADER_RESET_TIME)
    else:
        print("Not trying to go into the bootloader, so we will have a long delay to allow the application to start up")
        print(f"Sleeping for {DONT_GO_TO_BOOTLOADER_RESET_TIME}s after reset (normal mode).")
        time.sleep(DONT_GO_TO_BOOTLOADER_RESET_TIME)
    check_the_status(motor, go_to_bootloader, False, verbose=verbose)

def detect_device_and_get_unique_id(motor, verbose=False):
    print("Detecting devices...")
    devices = motor.detect_devices()
    if not devices or len(devices) == 0:
        raise RuntimeError("No devices detected on the bus.")
    # Use the first detected device
    unique_id, alias = devices[0]
    print(f"Detected device: alias={alias}, unique_id={unique_id}")
    return unique_id

def test_invalid_test_mode(motor, go_to_bootloader, verbose):
    print("\n--- Test: Invalid Test Mode Parameter ---")
    try:
        invalid_mode = 255  # >= 72 triggers ERROR_INVALID_TEST_MODE
        print(f"Sending invalid test mode: {invalid_mode}")
        timeout_occurred = False
        try:
            motor.test_mode(invalid_mode)
            time.sleep(0.1)
        except Exception as e:
            if "timeout" in str(e).lower():
                print(f"Expected exception (should be a timeout) after triggering fatal error: {e}")
                timeout_occurred = True
            else:
                print(f"Unexpected exception after triggering fatal error: {e}")
                raise
        if not timeout_occurred:
            raise AssertionError("FAIL: Expected a timeout after triggering fatal error, but did not get one.")
        else:
            print("Confirmwad that the timeout happed as expected")
        check_the_status(motor, go_to_bootloader, True, verbose=verbose, expected_fatal_error_code=ERROR_INVALID_TEST_MODE)
        print("PASS: Invalid test mode triggered ERROR_INVALID_TEST_MODE as expected.")
        return True
    except Exception as e:
        print(f"FAIL: Invalid test mode did not trigger expected error: {e}")
        return False

def test_random_fatal_test_mode(motor, go_to_bootloader, verbose):
    print("\n--- Test: Random Fatal Test Mode ---")
    try:
        fatal_error = random.randint(0, MAX_FATAL_ERROR)
        test_mode_value = fatal_error + 12
        print(f"Sending test mode: {test_mode_value} (should trigger fatal_error {fatal_error})")
        timeout_occurred = False
        try:
            motor.test_mode(test_mode_value)
            time.sleep(0.1)
        except Exception as e:
            if "timeout" in str(e).lower():
                print(f"Expected exception (should be a timeout) after triggering fatal error: {e}")
                timeout_occurred = True
            else:
                print(f"Unexpected exception after triggering fatal error: {e}")
                raise
        if not timeout_occurred:
            raise AssertionError("FAIL: Expected a timeout after triggering fatal error, but did not get one.")
        else:
            print("Confirmwad that the timeout happed as expected")
        check_the_status(motor, go_to_bootloader, True, verbose=verbose, expected_fatal_error_code=fatal_error)
        print(f"PASS: Test mode {test_mode_value} triggered fatal_error {fatal_error} as expected.")
        return True
    except Exception as e:
        print(f"FAIL: Random fatal test mode did not trigger expected fatal error: {e}")
        return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the 'Test mode' command.")
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port device name (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('--bootloader', action='store_true', help='Enter bootloader mode before running the test')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()

    if args.verbose:
        args.verbose = 2

    overall_passed = True

    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port(timeout=1.5)
    try:
        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} of {args.repeat} ==========")
            # Step 1: Reset with broadcast alias
            motor = M3(255, verbose=args.verbose)
            print("Initial reset with broadcast alias 255...")
            motor.use_alias(255)
            motor.system_reset(verbose=args.verbose)
            if args.bootloader:
                print(f"Sleeping for {GO_TO_BOOTLOADER_RESET_TIME}s after reset (bootloader mode).")
                time.sleep(GO_TO_BOOTLOADER_RESET_TIME)
            else:
                print(f"Sleeping for {DONT_GO_TO_BOOTLOADER_RESET_TIME}s after reset (normal mode).")
                time.sleep(DONT_GO_TO_BOOTLOADER_RESET_TIME)
            # Step 2: Detect device and get unique_id
            unique_id = detect_device_and_get_unique_id(motor, verbose=args.verbose)
            motor.use_unique_id(unique_id)

            # All further operations use unique ID
            all_passed = True

            # Test 1: Invalid test mode
            passed = test_invalid_test_mode(motor, args.bootloader, args.verbose)
            all_passed &= passed

            # Reset after fatal error using unique ID
            reset_device(motor, args.bootloader, args.verbose)

            # Test 2: Random fatal error test mode
            passed = test_random_fatal_test_mode(motor, args.bootloader, args.verbose)
            all_passed &= passed

            # Reset after fatal error using unique ID
            reset_device(motor, args.bootloader, args.verbose)

            if all_passed:
                print("PASSED (this repeat)")
            else:
                print("FAILED: One or more test mode scenarios failed (this repeat).")
            overall_passed &= all_passed

        if overall_passed:
            print("\nALL REPEATS PASSED")
            sys.exit(0)
        else:
            print("\nFAILED: One or more tests failed in at least one repeat.")
            sys.exit(1)
    finally:
        servomotor.close_serial_port()