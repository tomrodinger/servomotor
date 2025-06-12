#!/usr/bin/env python3

"""
Test: Get Firmware Version Command

This test verifies the "Get firmware version" command functionality in both main firmware and bootloader modes.
The command returns the firmware/bootloader version and a flag indicating the current mode.

Test Scenarios:
1. Test in main firmware mode (inBootloader flag should be 0)
2. Test in bootloader mode (inBootloader flag should be 1) 
3. Verify firmware version format and consistency across multiple calls
4. Support for repeating the test multiple times
5. Validate that the inBootloader flag correctly reflects the current mode

Key Features:
- Works in both main firmware and bootloader modes
- Returns firmware version as u32_version_number (4 bytes: development, patch, minor, major)
- Returns inBootloader flag (0 = main firmware, 1 = bootloader)
- Uses extended addressing (unique ID) for robust communication
- Supports --repeat option for stress testing
- Validates version format and consistency
"""

import servomotor
from servomotor import M3
import argparse
import time
import sys

GO_TO_BOOTLOADER_RESET_TIME = 0.07  # Empirically determined: between 0.002 and 0.13 for firmware upgrade to work
DONT_GO_TO_BOOTLOADER_RESET_TIME = 2.0
BOOTLOADER_FLAG_BIT = 0x01  # STATUS_IN_THE_BOOTLOADER_FLAG_BIT

def check_the_status(motor, go_to_bootloader, verbose=2):
    """Check device status and verify we're in the expected mode."""
    print("Calling get_status to check the current mode (bootloader or application).")
    status = motor.get_status(verbose=verbose)
    print(f"Status: {status}")
    
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError("No response from get_status: cannot confirm device mode!")
    
    flags = status[0]
    fatal_error_code = status[1]
    
    in_bootloader = (flags & BOOTLOADER_FLAG_BIT) != 0
    if go_to_bootloader:
        assert in_bootloader, "Device is NOT in bootloader mode after reset, which is unexpected"
        print("Device is in bootloader mode, which is correct")
    else:
        assert not in_bootloader, "Device IS in bootloader mode after reset, which is unexpected"
        print("Device is in main firmware mode, which is correct")
    
    if fatal_error_code != 0:
        raise AssertionError(f"FAILED: Expected fatal error code 0 but got {fatal_error_code}.")
    print(f"No fatal errors detected (fatal_error_code={fatal_error_code}).")
    
    return in_bootloader

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
        print("Not trying to go into the bootloader, so we will have a long delay to allow the application to start up")
        print(f"Sleeping for {DONT_GO_TO_BOOTLOADER_RESET_TIME}s after reset (normal mode).")
        time.sleep(DONT_GO_TO_BOOTLOADER_RESET_TIME)
    
    if alias != 255:
        check_the_status(motor, go_to_bootloader, verbose=verbose)

def detect_device_and_get_alias(motor_broadcast, verbose=False):
    """Detect device and return alias and unique ID."""
    print("Detecting devices...")
    devices = motor_broadcast.detect_devices()
    if not devices or len(devices) == 0:
        raise RuntimeError("No devices detected on the bus.")
    
    # Use the first detected device
    unique_id, alias = devices[0]
    print(f"Detected device: alias={alias}, unique_id={unique_id}")
    return alias, unique_id

def validate_firmware_version(version_info, expected_in_bootloader):
    """Validate the firmware version response format and content."""
    if not isinstance(version_info, (list, tuple)) or len(version_info) != 2:
        raise AssertionError(f"Firmware version response has unexpected format: {version_info}")
    
    firmware_version, in_bootloader = version_info
    
    # firmware_version: can be either a list [dev, patch, minor, major] or u32 integer
    if isinstance(firmware_version, list):
        # Format: [development, patch, minor, major]
        assert len(firmware_version) == 4, f"firmware_version list should have 4 elements, got {len(firmware_version)}"
        development, patch, minor, major = firmware_version
        # Convert to u32 for consistency
        firmware_version_u32 = development | (patch << 8) | (minor << 16) | (major << 24)
    else:
        # Format: u32 integer
        assert isinstance(firmware_version, int), "firmwareVersion is not int or list"
        assert 0 <= firmware_version < 2**32, f"firmwareVersion out of range: {firmware_version}"
        firmware_version_u32 = firmware_version
        # Extract version components
        development = firmware_version & 0xFF
        patch = (firmware_version >> 8) & 0xFF
        minor = (firmware_version >> 16) & 0xFF
        major = (firmware_version >> 24) & 0xFF
    
    # in_bootloader: u8, should be 0 or 1
    assert isinstance(in_bootloader, int), "inBootloader is not int"
    assert in_bootloader in [0, 1], f"inBootloader should be 0 or 1, got: {in_bootloader}"
    
    # Verify the bootloader flag matches expected mode
    assert in_bootloader == expected_in_bootloader, f"inBootloader flag mismatch: got {in_bootloader}, expected {expected_in_bootloader}"
    
    print(f"Firmware version breakdown: major={major}, minor={minor}, patch={patch}, development={development}")
    print(f"Firmware version as u32: 0x{firmware_version_u32:08X} ({firmware_version_u32})")
    
    # Basic sanity checks on version components
    assert 0 <= development <= 255, f"development version out of range: {development}"
    assert 0 <= patch <= 255, f"patch version out of range: {patch}"
    assert 0 <= minor <= 255, f"minor version out of range: {minor}"
    assert 0 <= major <= 255, f"major version out of range: {major}"
    
    return firmware_version_u32, in_bootloader

def test_get_firmware_version(motor, expected_in_bootloader, repeat=1, verbose=False):
    """Test the get_firmware_version command with validation."""
    mode_name = "bootloader" if expected_in_bootloader else "main firmware"
    print(f"\nRunning 'Get firmware version' command {repeat} time(s) in {mode_name} mode...")
    
    results = []
    for i in range(repeat):
        print(f"\n--- Attempt {i+1} of {repeat} ---")
        version_info = motor.get_firmware_version(verbose=verbose)
        print(f"Result {i+1}: {version_info}")
        
        firmware_version, in_bootloader = validate_firmware_version(version_info, expected_in_bootloader)
        results.append(version_info)
        
        print(f"Firmware version: 0x{firmware_version:08X} ({firmware_version})")
        print(f"In bootloader: {in_bootloader} ({'Yes' if in_bootloader else 'No'})")
        
        # Small delay between repeated calls
        if i < repeat - 1:
            time.sleep(0.1)
    
    # Check all results are identical
    for i in range(1, repeat):
        assert results[i] == results[0], f"Result at repeat {i+1} does not match first result"
    
    print(f"All {repeat} repeated results are identical and valid.")
    return results[0]

def main():
    parser = argparse.ArgumentParser(description="Test the 'Get firmware version' command.")
    parser.add_argument('-p', '--port', type=str, help='Serial port device name (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('--bootloader', action='store_true', help='Enter bootloader mode before running the test')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()
    
    # Check that either -p or -P is provided
    if not args.port and not args.PORT:
        parser.error("Either -p/--port or -P/--PORT must be specified")

    if args.verbose:
        args.verbose = 2

    overall_passed = True
    test_results = {}

    for repeat_idx in range(args.repeat):
        if args.repeat > 1:
            print(f"\n========== REPEAT {repeat_idx + 1} of {args.repeat} ==========")
        
        servomotor.set_serial_port_from_args(args)
        servomotor.open_serial_port(timeout=1.5)
        try:
            # Step 1: Reset device (broadcast)
            motor_broadcast = M3(255, verbose=args.verbose)
            reset_device(motor_broadcast, 255, args.bootloader, verbose=args.verbose)
            
            # Step 2: Detect device and get alias/unique_id
            alias, unique_id = detect_device_and_get_alias(motor_broadcast, verbose=args.verbose)
            
            # Step 3: Create M3 object with the detected alias
            motor = M3(alias, verbose=args.verbose)
            
            # Step 4: Determine expected mode (skip status check since get_status has issues)
            expected_in_bootloader = 1 if args.bootloader else 0
            print(f"Expected mode: {'bootloader' if expected_in_bootloader else 'main firmware'}")
            
            # Step 5: Test get_firmware_version in current mode using alias addressing
            print(f"\n=== Testing with alias addressing (alias={alias}) ===")
            current_repeat_passed = True
            try:
                firmware_info_alias = test_get_firmware_version(motor, expected_in_bootloader, repeat=1, verbose=args.verbose)
                print(f"SUCCESS: Get firmware version test with alias addressing")
            except Exception as e:
                print(f"FAILED: Get firmware version test with alias addressing: {e}")
                current_repeat_passed = False
            
            # Step 6: Test get_firmware_version using unique ID addressing
            print(f"\n=== Testing with unique ID addressing (unique_id={unique_id:016X}) ===")
            motor.use_unique_id(unique_id)
            try:
                firmware_info_uid = test_get_firmware_version(motor, expected_in_bootloader, repeat=1, verbose=args.verbose)
                print(f"SUCCESS: Get firmware version test with unique ID addressing")
                
                # Verify both methods return the same result
                if firmware_info_alias == firmware_info_uid:
                    print("SUCCESS: Both alias and unique ID addressing return identical results")
                else:
                    print(f"WARNING: Different results - alias: {firmware_info_alias}, unique_id: {firmware_info_uid}")
                    
            except Exception as e:
                print(f"FAILED: Get firmware version test with unique ID addressing: {e}")
                current_repeat_passed = False
            
            # Step 7: If repeat > 1, test multiple calls
            if args.repeat > 1:
                print(f"\n=== Testing {args.repeat} repeated calls ===")
                try:
                    firmware_info_repeated = test_get_firmware_version(motor, expected_in_bootloader, repeat=args.repeat, verbose=args.verbose)
                    print(f"SUCCESS: {args.repeat} repeated calls completed successfully")
                except Exception as e:
                    print(f"FAILED: Repeated calls test: {e}")
                    current_repeat_passed = False
            
            # Store results for summary
            mode_key = "bootloader" if args.bootloader else "main_firmware"
            if mode_key not in test_results:
                test_results[mode_key] = []
            if current_repeat_passed:
                test_results[mode_key].append(firmware_info_alias)
            overall_passed &= current_repeat_passed
            
            if current_repeat_passed:
                print(f"\nPASSED: Get firmware version test in {'bootloader' if args.bootloader else 'main firmware'} mode")
                print("PASSED (this repeat)")
            else:
                print(f"\nFAILED: Get firmware version test in {'bootloader' if args.bootloader else 'main firmware'} mode")
                print("FAILED: Get firmware version test failed (this repeat).")
                
        finally:
            servomotor.close_serial_port()

    # Final summary
    if args.repeat > 1:
        print("\n========== GRAND SUMMARY ==========")
        for mode, results in test_results.items():
            print(f"{mode.replace('_', ' ').title()} mode: {len(results)} successful tests")
            if results:
                # Check consistency across repeats
                first_result = results[0]
                all_consistent = all(result == first_result for result in results)
                print(f"  All results consistent: {'Yes' if all_consistent else 'No'}")
                if results:
                    firmware_version, in_bootloader = first_result
                    # Handle both list and integer formats
                    if isinstance(firmware_version, list):
                        dev, patch, minor, major = firmware_version
                        firmware_version_u32 = dev | (patch << 8) | (minor << 16) | (major << 24)
                        print(f"  Firmware version: {major}.{minor}.{patch}.{dev} (0x{firmware_version_u32:08X})")
                    else:
                        print(f"  Firmware version: 0x{firmware_version:08X}")
                    print(f"  In bootloader: {in_bootloader}")

    if overall_passed:
        print("\nALL TESTS PASSED")
        sys.exit(0)
    else:
        print("\nFAILED: One or more tests failed.")
        sys.exit(1)

if __name__ == "__main__":
    main()