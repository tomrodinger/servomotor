#!/usr/bin/env python3

"""
Test: Get Product Info Command

- Resets the device at the beginning.
- Detects the device alias and unique ID.
- Uses extended addressing to talk to the device.
- Runs the "Get product info" command, supporting --repeat.
- Validates that the result is as expected (field ranges, formats).
- Ensures the result is the same across all repeated calls.
"""

import servomotor
from servomotor import M3
import argparse
import time
import sys
import string

GO_TO_BOOTlOADER_RESET_TIME = 0.07  # Empirically determined: between 0.002 and 0.13 for firmware upgrade to work
DONT_GO_TO_BOOTlOADER_RESET_TIME = 2.0
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
    motor.use_alias(alias)
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

def detect_device_and_get_alias(motor_broadcast, verbose=False):
    print("Detecting devices...")
    devices = motor_broadcast.detect_devices()
    if not devices or len(devices) == 0:
        raise RuntimeError("No devices detected on the bus.")
    # Use the first detected device
    unique_id, alias = devices[0]
    print(f"Detected device: alias={alias}, unique_id={unique_id}")
    return alias, unique_id

def validate_product_info(info, expected_unique_id):
    # info is expected to be a list or tuple of 6 elements
    if not isinstance(info, (list, tuple)) or len(info) != 6:
        raise AssertionError(f"Product info response has unexpected format: {info}")

    product_code, firmware_compat, hardware_version, serial_number, unique_id, reserved = info

    # product_code: string8, should be 8 printable ASCII characters
    assert isinstance(product_code, str), "productCode is not a string"
    assert len(product_code) == 8, f"productCode length is not 8: {product_code!r}"
    assert all(c in string.printable and c not in '\r\n\t\x0b\x0c' for c in product_code), f"productCode contains non-printable characters: {product_code!r}"

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

def test_get_product_info(motor, expected_unique_id, repeat=1, verbose=False):
    print(f"Running 'Get product info' command {repeat} time(s)...")
    results = []
    for i in range(repeat):
        info = motor.get_product_info(verbose=verbose)
        print(f"Result {i+1}: {info}")
        validate_product_info(info, expected_unique_id)
        results.append(info)
#        time.sleep(0.1)
    # Check all results are identical
    for i in range(1, repeat):
        assert results[i] == results[0], f"Result at repeat {i+1} does not match first result"
    print("All repeated results are identical and valid.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the 'Get product info' command.")
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port device name (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('--bootloader', action='store_true', help='Enter bootloader mode before running the test')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()

    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port(timeout=1.5)
    try:
        # Step 1: Reset device (broadcast)
        motor_broadcast = M3(255, verbose=args.verbose)
        reset_device(motor_broadcast, 255, args.bootloader, args.verbose)

        # Step 2: Detect device and get alias/unique_id
        alias, unique_id = detect_device_and_get_alias(motor_broadcast, verbose=args.verbose)

        # Step 3: Create M3 object and use extended addressing to talk to the device (by unique ID)
        motor = M3(255, verbose=args.verbose)
        motor.use_unique_id(unique_id)

        # Step 4: Run the test
        test_get_product_info(motor, unique_id, repeat=args.repeat, verbose=args.verbose)

        print("\nPASSED")
        sys.exit(0)
    except Exception as e:
        print(f"\nFAILED: {e}")
        sys.exit(1)
    finally:
        servomotor.close_serial_port()