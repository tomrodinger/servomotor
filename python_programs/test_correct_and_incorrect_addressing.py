#!/usr/bin/env python3

"""
Test: Correct and Incorrect Addressing (test_correct_and_incorrect_addressing.py)

This test verifies the ability to communicate with the device using both alias and unique ID addressing.
It checks that the device responds to the correct alias and unique ID, and does not respond (timeout) to incorrect alias, incorrect unique ID, or alias 255.

Test Steps:
1. Detect the device (get unique_id and alias).
2. If alias is 255, set it to a valid alias (1).
3. Test "Get status" with:
   - Correct alias (should succeed)
   - Incorrect alias (should timeout)
   - Correct unique ID (should succeed)
   - Incorrect unique ID (should timeout)
   - Alias 255 (should timeout)
4. Print PASS/FAIL for each scenario and exit with 0 on success, 1 on failure.
"""

import servomotor
from servomotor import M3
import argparse
import time
import sys
import random

GO_TO_BOOTLOADER_RESET_TIME = 0.07
DONT_GO_TO_BOOTLOADER_RESET_TIME = 2.0
ERROR_BAD_ALIAS = 50  # From bootloader C code (based on observed output)
BOOTLOADER_FLAG_BIT = 0x01  # STATUS_IN_THE_BOOTLOADER_FLAG_BIT
FLASH_WRITE_TIME = 0.02  # Time for flash write after setting alias

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
            # Explicitly check for [1, 0]
            if status != [1, 0]:
                raise AssertionError(f"Expected status [1, 0] in bootloader, got {status}")
            print("Status is exactly [1, 0] as expected in bootloader.")
        else:
            assert not in_bootloader, "Device IS in bootloader mode after reset, which is unexpected"
            print("Device is not in bootloader mode, which is correct")
            # Explicitly check for [0, 0]
            if status != [0, 0]:
                raise AssertionError(f"Expected status [0, 0] in application, got {status}")
            print("Status is exactly [0, 0] as expected in application.")

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
    motor.use_alias(alias)
    print(f"Resetting device at alias {alias} (go_to_bootloader={go_to_bootloader})...")
    motor.system_reset(verbose=verbose)
    if go_to_bootloader:
        print("Trying to go into the bootloader, so we will have only a very short delay after the reset")
        print(f"Sleeping for {GO_TO_BOOTLOADER_RESET_TIME}s after reset (bootloader mode).")
        time.sleep(GO_TO_BOOTLOADER_RESET_TIME)
    else:
        print("Not trying to go into the bootloader, so we will have a long delay to allow the applicaiton to start up")
        print(f"Sleeping for {DONT_GO_TO_BOOTLOADER_RESET_TIME}s after reset (normal mode).")
        time.sleep(DONT_GO_TO_BOOTLOADER_RESET_TIME)
    check_the_status(motor, alias, go_to_bootloader, False, verbose=verbose)

def detect_device(motor, verbose=False):
    print("Detecting devices...")
    devices = motor.detect_devices()
    if not devices or len(devices) == 0:
        raise RuntimeError("No devices detected on the bus.")
    unique_id, alias = devices[0]
    print(f"Detected device: alias={alias}, unique_id={unique_id:016X}")
    return alias, unique_id

def set_alias_if_needed(motor, current_alias, unique_id, go_to_bootloader=False, verbose=False):
    if current_alias != 255:
        print(f"Device alias is already {current_alias}, no need to change.")
        return current_alias
    # Set alias to 1 (or any valid alias 0-251)
    new_alias = random.randint(0, 251)
    print(f"Setting device alias from 255 to {new_alias}...")
    motor.use_alias(255)
    ret = motor.set_device_alias(new_alias)
    print(f"set_device_alias({new_alias}) returned: {ret}")
    # Wait for flash write and device to restart, with correct timing for bootloader/app mode
    if go_to_bootloader:
        delay_time = GO_TO_BOOTLOADER_RESET_TIME + FLASH_WRITE_TIME
        print(f"Sleeping for {delay_time:0.3f}s after setting alias (bootloader mode).")
    else:
        delay_time = DONT_GO_TO_BOOTLOADER_RESET_TIME + FLASH_WRITE_TIME
        print(f"Sleeping for {delay_time:0.3f}s after setting alias (normal mode).")
    time.sleep(delay_time)
    motor.use_alias(new_alias)
    # Call check_the_status to ensure device is in correct mode
    check_the_status(motor, new_alias, go_to_bootloader, False, verbose=verbose)
    return new_alias

def pick_random_wrong_alias(correct_alias):
    while True:
        wrong_alias = random.randint(0, 251)
        if wrong_alias != correct_alias:
            return wrong_alias

def pick_random_wrong_unique_id(correct_unique_id):
    while True:
        wrong_unique_id = random.getrandbits(64)
        if wrong_unique_id != correct_unique_id:
            return wrong_unique_id

def validate_product_info(info, expected_unique_id):
    import string
    # info is expected to be a list or tuple of 6 elements
    if not isinstance(info, (list, tuple)) or len(info) != 6:
        raise AssertionError(f"Product info response has unexpected format: {info}")

    product_code, firmware_compat, hardware_version, serial_number, unique_id, reserved = info

    # product_code: string8, should be 8 printable ASCII characters
    assert isinstance(product_code, str), "productCode is not a string"
    assert len(product_code) == 8, f"productCode length is not 8: {product_code!r}"
    assert all(c in string.printable and c not in '\r\n\t\x0b\x0c' for c in product_code), f"productCode contains non-printable characters: {product_code!r}"
    assert product_code.strip() == "M3"

    # firmware_compat: u8, should be 0-255
    assert isinstance(firmware_compat, int), "firmwareCompatibility is not int"
    assert 0 <= firmware_compat <= 255, f"firmwareCompatibility out of range: {firmware_compat}"

    # hardware_version: u24, 3 bytes, patch/minor/major, this is a list with three integers
    assert isinstance(hardware_version, list), "hardwareVersion is not a list"
    assert len(hardware_version) == 3, "hardwareVersion list does not have three items"
    patch = hardware_version[0]
    minor = hardware_version[1]
    major = hardware_version[2]
    for v, name in zip([patch, minor, major], ["patch", "minor", "major"]):
        print(f"Checking this part of the hardware version: {v}")
        assert 0 <= v <= 99, f"hardwareVersion {name} out of range: {v}"

    # serial_number: u32, positive integer, nonzero
    assert isinstance(serial_number, int), "serialNumber is not int"
    assert 0 < serial_number < 2**32, f"serialNumber out of range: {serial_number}"

    # unique_id: u64, should match detected unique_id
    assert isinstance(unique_id, int), "uniqueId is not int"
    assert unique_id == expected_unique_id, f"uniqueId mismatch: got {unique_id}, expected {expected_unique_id}"

    # reserved: u32, should be 0 (not used)
    assert isinstance(reserved, int), "reserved is not int"
    assert reserved == 0, f"reserved field is not zero: {reserved}"

def try_get_status(motor, addressing_method, value, expect_success, verbose=False):
    try:
        if addressing_method == "alias":
            motor.use_alias(value)
        elif addressing_method == "unique_id":
            motor.use_unique_id(value)
        else:
            raise ValueError("Unknown addressing method")
        status = motor.get_status(verbose=verbose)
        if addressing_method == "unique_id":
            value_str = f"{value:016X}"
        else:
            value_str = str(value)
        print(f"{addressing_method}={value_str}: get_status() returned: {status}")
        # If we expect failure (timeout/no response), treat empty list or None as PASS
        if not expect_success:
            if status is None or (isinstance(status, list) and len(status) == 0):
                print(f"{addressing_method}={value_str}: No response (empty result) as expected.")
                return True
            print(f"FAILED: {addressing_method}={value_str}: Expected timeout/no response, but got response: {status}")
            return False
        # If we expect success, treat non-empty result as PASS
        if status is None or (isinstance(status, list) and len(status) == 0):
            print(f"FAILED: {addressing_method}={value_str}: Expected response, but got no response.")
            return False
        return True
    except Exception as e:
        if "timeout" in str(e).lower():
            if addressing_method == "unique_id":
                value_str = f"{value:016X}"
            else:
                value_str = str(value)
            print(f"{addressing_method}={value_str}: Timeout occurred as expected.")
            if expect_success:
                print(f"FAILED: {addressing_method}={value_str}: Expected response, but got timeout.")
                return False
            return True
        else:
            if addressing_method == "unique_id":
                value_str = f"{value:016X}"
            else:
                value_str = str(value)
            print(f"{addressing_method}={value_str}: Unexpected exception: {e}")
            return False

def main():
    parser = argparse.ArgumentParser(description="Test correct and incorrect addressing (alias and unique ID).")
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port device name (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('--bootloader', action='store_true', help='Enter bootloader mode before running the test')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()

    if args.verbose:
        args.verbose = 2

    overall_passed = True
    repeat_results = []

    for repeat_idx in range(args.repeat):
        if args.repeat > 1:
            print(f"\n========== REPEAT {repeat_idx + 1} of {args.repeat} ==========")
        servomotor.set_serial_port_from_args(args)
        servomotor.open_serial_port(timeout=1.5)
        try:
            # Step 1: Reset device (broadcast)
            motor = M3(255, verbose=args.verbose)
            reset_device(motor, 255, args.bootloader, verbose=args.verbose)

            # Step 2: Detect device and get alias/unique_id
            alias, unique_id = detect_device(motor, verbose=args.verbose)

            # Step 3: If alias is 255, set to 1
            alias = set_alias_if_needed(motor, alias, unique_id, go_to_bootloader=args.bootloader, verbose=args.verbose)

            # Step 4: Pick wrong alias and wrong unique ID at random
            wrong_alias = pick_random_wrong_alias(alias)
            wrong_unique_id = pick_random_wrong_unique_id(unique_id)

            # Step 5: Test all addressing scenarios using the same motor object
            results = []

            # Correct alias
            results.append(try_get_status(motor, "alias", alias, expect_success=True, verbose=args.verbose))
            # Wrong alias
            results.append(try_get_status(motor, "alias", wrong_alias, expect_success=False, verbose=args.verbose))
            # Correct unique ID
            results.append(try_get_status(motor, "unique_id", unique_id, expect_success=True, verbose=args.verbose))
            # Wrong unique ID
            results.append(try_get_status(motor, "unique_id", wrong_unique_id, expect_success=False, verbose=args.verbose))
            # Alias 255 (broadcast, should not respond)
            results.append(try_get_status(motor, "alias", 255, expect_success=False, verbose=args.verbose))

            # Additional check: after alias=255, test get_product_info with correct unique ID
            print("\nNow testing get_product_info with correct unique ID after alias=255 test...")
            try:
                motor.use_unique_id(unique_id)
                info = motor.get_product_info(verbose=args.verbose)
                print(f"get_product_info() returned: {info}")
                validate_product_info(info, unique_id)
                print("get_product_info: PASS")
                results.append(True)
            except Exception as e:
                print(f"get_product_info: FAIL ({e})")
                results.append(False)

            all_passed = all(results)
            repeat_results.append(all_passed)
            print("\n========== SUMMARY (Repeat %d) ==========" % (repeat_idx + 1))
            for desc, passed in zip(
                ["Correct alias", "Wrong alias", "Correct unique ID", "Wrong unique ID", "Alias 255 (broadcast)", "Get product info after alias=255"], results
            ):
                print(f"{desc}: {'PASS' if passed else 'FAIL'}")

            if all_passed:
                print("\nPASSED (this repeat)")
            else:
                print("\nFAILED: One or more addressing scenarios failed (this repeat).")
            overall_passed &= all_passed
        finally:
            servomotor.close_serial_port()

    # Grand summary
    if args.repeat > 1:
        print("\n========== GRAND SUMMARY ==========")
        for i, passed in enumerate(repeat_results):
            print(f"Repeat {i+1}: {'PASS' if passed else 'FAIL'}")
    if overall_passed:
        print("\nALL REPEATS PASSED")
        sys.exit(0)
    else:
        print("\nFAILED: One or more tests failed in at least one repeat.")
        sys.exit(1)

if __name__ == "__main__":
    main()