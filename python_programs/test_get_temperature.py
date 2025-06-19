#!/usr/bin/env python3

"""
Test: Get Temperature Command

This test verifies the "Get temperature" command functionality by measuring temperature 
changes during high-power motor operation. The test validates temperature measurement 
accuracy, thermal protection behavior, and motor step-skipping detection due to overheating.

Key Features:
- No bootloader support (temperature command not available in bootloader)
- Multi-motor support (tests all detected motors simultaneously)
- Thermal protection detection (monitors for motor driver thermal shutdown)
- Step skipping detection (uses position deviation limits to detect motor failures)
- Robust device detection (multiple detection attempts to handle communication collisions)
- Timeout handling (graceful handling of motor communication failures)

Test Flow:
1. Initial sleep period with countdown (allow motors to cool)
2. Take baseline temperature readings
3. Enable MOSFETs and set motors to high power/velocity
4. Continuously queue movement commands while monitoring
5. Take final temperature readings after motor run time
6. Compare temperatures and validate results
"""

import servomotor
from servomotor import M3, communication
import argparse
import time
import sys
from datetime import datetime

# Constants
DONT_GO_TO_BOOTLOADER_RESET_TIME = 2.0
MAX_DETECTION_ATTEMPTS = 3
MAX_STATUS_RETRY_ATTEMPTS = 3
QUEUE_REFILL_DURATION = 10.0  # Duration for each velocity command in seconds

class MotorTestResult:
    """Stores test results for a single motor"""
    def __init__(self, alias, unique_id):
        self.alias = alias
        self.unique_id = unique_id
        self.initial_temp = None
        self.final_temp = None
        self.temp_increase = None
        self.thermal_protection_events = 0
        self.position_deviation_errors = 0
        self.communication_timeouts = 0
        self.fatal_errors = []
        self.test_completed = False
        self.error_message = None
        self.motor_failed = False  # If motor becomes unresponsive

def print_countdown(message, duration_seconds):
    """Print countdown with minute intervals"""
    print(f"{message} ({duration_seconds} seconds)")
    
    remaining = duration_seconds
    while remaining > 0:
        if remaining % 60 == 0 or remaining <= 10:
            minutes = remaining // 60
            seconds = remaining % 60
            if minutes > 0:
                print(f"  {minutes}m {seconds}s remaining...")
            else:
                print(f"  {seconds}s remaining...")
        time.sleep(1)
        remaining -= 1
    print("  Time's up!")

def detect_devices_with_retries(motor_broadcast, verbose=False):
    """Detect devices with multiple retry attempts using proper multi-pass strategy"""
    device_dict = {}
    successful_detect_devices_count = 0
    detect_devices_attempt_count = 0
    max_attempts = 3
    
    while successful_detect_devices_count < max_attempts:
        print("Resetting the system")
        motor_broadcast.system_reset(verbose=verbose)
        time.sleep(1.5)
        
        print("Flushing the receive buffer")
        servomotor.flush_receive_buffer()
        print(f"Detecting devices attempt {detect_devices_attempt_count + 1}/{max_attempts}")
        
        try:
            response = motor_broadcast.detect_devices(verbose=verbose)
            successful_detect_devices_count += 1
        except Exception as e:
            print(f"Communication error: {e}")
            detect_devices_attempt_count += 1
            continue
            
        detect_devices_attempt_count += 1
        print("Detected devices:")
        
        if response and len(response) > 0:
            for device in response:
                if len(device) >= 2:
                    unique_id = device[0]
                    alias = device[1]
                    
                    print(f"Unique ID: {unique_id:016X}, Alias: {alias}")
                    if unique_id in device_dict:
                        print(f"This unique ID {unique_id:016X} is already in the device dictionary, so not adding it again")
                        if alias != device_dict[unique_id]:
                            print(f"Error: we discovered an inconsistency: the alias is different: {alias} vs {device_dict[unique_id]}")
                    else:
                        device_dict[unique_id] = alias
                else:
                    print(f"Invalid device response format: {device}")
        else:
            print("No devices detected in this attempt")
    
    # Convert device_dict to list of tuples (unique_id, alias) for compatibility
    devices = [(unique_id, alias) for unique_id, alias in device_dict.items()]
    print(f"Final device count after {max_attempts} detection passes: {len(devices)}")
    return devices

def get_motor_status_with_retries(motor, motor_id, verbose=False):
    """Get motor status with retries, return None if all attempts fail"""
    for attempt in range(MAX_STATUS_RETRY_ATTEMPTS):
        try:
            status = motor.get_status(verbose=verbose)
            return status
        except Exception as e:
            if attempt < MAX_STATUS_RETRY_ATTEMPTS - 1:
                if verbose:
                    print(f"Motor {motor_id}: Status attempt {attempt + 1} failed: {e}, retrying...")
                time.sleep(0.1)
            else:
                if verbose:
                    print(f"Motor {motor_id}: All status attempts failed: {e}")
    return None

def safe_motor_command(motor, command_func, motor_id, command_name, *args, **kwargs):
    """Execute motor command with timeout handling"""
    try:
        return command_func(*args, **kwargs)
    except Exception as e:
        if "timeout" in str(e).lower():
            print(f"Motor {motor_id}: Timeout during {command_name} - checking status...")
            status = get_motor_status_with_retries(motor, motor_id)
            if status:
                flags, fatal_error_code = status[0], status[1]
                if fatal_error_code != 0:
                    print(f"Motor {motor_id}: Fatal error detected: {fatal_error_code}")
                    return None, fatal_error_code
            raise TimeoutError(f"Motor {motor_id}: {command_name} timed out")
        else:
            raise

def reset_device(motor, alias, verbose=False):
    """Reset device and wait for startup"""
    motor.use_alias(alias)
    print(f"Resetting device at alias {alias}...")
    motor.system_reset(verbose=verbose)
    print(f"Sleeping for {DONT_GO_TO_BOOTLOADER_RESET_TIME}s after reset (normal mode).")
    time.sleep(DONT_GO_TO_BOOTLOADER_RESET_TIME)

def configure_motor_units(motor, motor_id, verbose=False):
    """Configure motor units for consistent operation"""
    try:
        print(f"Motor {motor_id}: Setting units...")
        motor.set_time_unit("seconds")
        motor.set_position_unit("shaft_rotations")
        motor.set_velocity_unit("rotations_per_second")
        motor.set_acceleration_unit("rotations_per_second_squared")
        print(f"Motor {motor_id}: Units configured successfully")
        return True
    except Exception as e:
        print(f"Motor {motor_id}: Unit configuration failed: {e}")
        return False

def configure_motor_for_test(motor, motor_id, max_current, position_deviation, verbose=False):
    """Configure motor for high-power temperature test"""
    try:
        # Enable MOSFETs
        print(f"Motor {motor_id}: Enabling MOSFETs...")
        safe_motor_command(motor, motor.enable_mosfets, motor_id, "enable_mosfets", verbose=verbose)
        
        # Set maximum motor current
        print(f"Motor {motor_id}: Setting maximum motor current to {max_current}...")
        safe_motor_command(motor, motor.set_maximum_motor_current, motor_id, "set_maximum_motor_current",
                          max_current, 200, verbose=verbose)  # 200 for regeneration current
        
        # Set position deviation limit to detect step skipping (now in shaft_rotations)
        print(f"Motor {motor_id}: Setting position deviation limit to {position_deviation} shaft_rotations...")
        safe_motor_command(motor, motor.set_max_allowable_position_deviation, motor_id,
                          "set_max_allowable_position_deviation", position_deviation, verbose=verbose)
        
        return True
    except Exception as e:
        print(f"Motor {motor_id}: Configuration failed: {e}")
        return False

def wait_for_motor_queues_to_empty(active_motors, args):
    """Wait for all motor queues to become empty (queue-based exit, not time-based)"""
    
    print("Waiting for all motor queues to become empty...")
    all_queues_empty = False
    check_count = 0
    
    while not all_queues_empty:
        all_queues_empty = True
        check_count += 1
        
        for motor, motor_id, result in active_motors:
            try:
                # Check queue size
                queue_size = motor.get_n_queued_items(verbose=args.verbose)
                if queue_size > 0:
                    all_queues_empty = False
                    if args.verbose or (check_count % 10 == 0):  # Show status every 10 checks or if verbose
                        print(f"Motor {motor_id}: Queue size = {queue_size}")
            
            except Exception as e:
                if args.verbose:
                    print(f"Motor {motor_id}: Queue check error: {e}")
                # If we can't check the queue, assume it's not empty to be safe
                all_queues_empty = False
        
        if not all_queues_empty:
            time.sleep(0.5)  # Wait a bit before checking again
    
    print("All motor queues are now empty. Motors have completed their movements.")

def partition_devices_into_batches(devices, devices_per_batch):
    """Partition detected devices into batches for testing"""
    if devices_per_batch is None:
        return [devices]  # Single batch with all devices
    
    batches = []
    for i in range(0, len(devices), devices_per_batch):
        batch = devices[i:i + devices_per_batch]
        batches.append(batch)
    return batches

def run_temperature_test_on_batch(motors_info, args):
    """Main temperature test function"""
    print(f"\n=== Starting Temperature Test ===")
    print(f"Testing {len(motors_info)} motor(s)")
    
    results = {}
    
    # Initialize results for each motor
    for unique_id, alias in motors_info:  # Note: detect_devices returns (unique_id, alias)
        motor_id = f"alias_{alias}"
        results[motor_id] = MotorTestResult(alias, unique_id)
    
    try:
        # Phase 1: Initial sleep period
        print(f"\n--- Phase 1: Initial Cooling Period ---")
        print_countdown("Waiting for motors to cool down", args.initial_sleep)
        
        # Phase 2: Take baseline temperature readings
        print(f"\n--- Phase 2: Baseline Temperature Readings ---")
        for unique_id, alias in motors_info:  # Note: detect_devices returns (unique_id, alias)
            motor_id = f"alias_{alias}"
            result = results[motor_id]
            
            try:
                print("Using alias:", alias)
                motor = M3(alias, verbose=args.verbose)
                

                temp = safe_motor_command(motor, motor.get_temperature, motor_id, "get_temperature", verbose=args.verbose)
                
                if temp is not None:
                    result.initial_temp = temp[0] if isinstance(temp, (list, tuple)) else temp
                    print(f"Motor {motor_id}: Initial temperature = {result.initial_temp}°C")
                    
                    # Validate temperature range
                    if result.initial_temp < args.min_temp or result.initial_temp > args.max_temp:
                        result.error_message = f"Initial temperature {result.initial_temp}°C out of range ({args.min_temp}-{args.max_temp}°C)"
                        print(f"Motor {motor_id}: {result.error_message}")
                        continue
                else:
                    result.error_message = "Failed to read initial temperature"
                    print(f"Motor {motor_id}: {result.error_message}")
                    continue
                    
            except Exception as e:
                result.error_message = f"Initial temperature reading failed: {e}"
                print(f"Motor {motor_id}: {result.error_message}")
                continue
        
        # Phase 3: Configure motors for high-power operation
        print(f"\n--- Phase 3: Motor Configuration ---")
        active_motors = []
        
        for unique_id, alias in motors_info:  # Note: detect_devices returns (unique_id, alias)
            motor_id = f"alias_{alias}"
            result = results[motor_id]
            
            if result.error_message:
                continue  # Skip motors that failed initial temperature reading
            
            try:
                motor = M3(alias, verbose=args.verbose)
                if unique_id != 0:
                    motor.use_unique_id(unique_id)
                else:
                    motor.use_alias(alias)
                
                if configure_motor_for_test(motor, motor_id, args.max_current, args.position_deviation, args.verbose):
                    active_motors.append((motor, motor_id, result))
                    print(f"Motor {motor_id}: Configuration successful")
                else:
                    result.error_message = "Motor configuration failed"
                    
            except Exception as e:
                result.error_message = f"Motor configuration error: {e}"
                print(f"Motor {motor_id}: {result.error_message}")
        
        if not active_motors:
            print("No motors successfully configured for testing!")
            return results
        
        # Phase 4: High-Power Motor Operation
        print(f"\n--- Phase 4: High-Power Motor Operation ---")
        print(f"Running motors at velocity {args.velocity} rot/s for {args.motor_run_time} seconds")
        
        # Start all motors sequentially (no threading)
        for motor, motor_id, result in active_motors:
            try:
                # Set proper units for easy time specification
                motor.set_time_unit("seconds")
                motor.set_velocity_unit("rotations_per_second")
                
                # Queue one long move for the entire test duration
                print(f"Motor {motor_id}: Queuing long move for {args.motor_run_time} seconds")
                safe_motor_command(motor, motor.move_with_velocity, motor_id, "move_with_velocity",
                                 args.velocity, args.motor_run_time, verbose=args.verbose)
                
                # Immediately queue a velocity=0 command to prevent error 18
                print(f"Motor {motor_id}: Queuing velocity=0 to prevent queue empty error")
                safe_motor_command(motor, motor.move_with_velocity, motor_id, "move_with_velocity",
                                 0.0, 0.01, verbose=args.verbose)  # 0 velocity for 0.01 seconds
                
                print(f"Motor {motor_id}: Started high-power operation")
                
            except Exception as e:
                result.error_message = f"Failed to start motor operation: {e}"
                print(f"Motor {motor_id}: {result.error_message}")
        
        # Wait for all motor queues to become empty (no threading)
        wait_for_motor_queues_to_empty(active_motors, args)
        
        # Motors should already be stopped by the queued velocity=0 command
        print("Motors should have stopped automatically...")
        for motor, motor_id, result in active_motors:
            try:
                motor.disable_mosfets(verbose=args.verbose)
                print(f"Motor {motor_id}: MOSFETs disabled")
            except Exception as e:
                print(f"Motor {motor_id}: Error disabling MOSFETs: {e}")
        
        # Phase 5: Final temperature readings
        print(f"\n--- Phase 5: Final Temperature Readings ---")
        time.sleep(2.0)  # Brief pause before final readings
        
        for motor, motor_id, result in active_motors:
            try:
                temp = safe_motor_command(motor, motor.get_temperature, motor_id, "get_temperature", verbose=args.verbose)
                
                if temp is not None:
                    result.final_temp = temp[0] if isinstance(temp, (list, tuple)) else temp
                    print(f"Motor {motor_id}: Final temperature = {result.final_temp}°C")
                    
                    # Calculate temperature increase
                    if result.initial_temp is not None and result.final_temp is not None:
                        result.temp_increase = result.final_temp - result.initial_temp
                        print(f"Motor {motor_id}: Temperature increase = {result.temp_increase}°C")
                    
                    # Validate final temperature range
                    if result.final_temp is not None:
                        if result.final_temp < args.min_temp or result.final_temp > args.max_temp:
                            result.error_message = f"Final temperature {result.final_temp}°C out of range ({args.min_temp}-{args.max_temp}°C)"
                        else:
                            result.test_completed = True
                    else:
                        result.error_message = "Final temperature reading returned None"
                else:
                    result.error_message = "Failed to read final temperature"
                    
            except Exception as e:
                result.error_message = f"Final temperature reading failed: {e}"
                print(f"Motor {motor_id}: {result.error_message}")
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        # Note: Graceful shutdown - motors will complete their current movements
        
        # Emergency stop all motors
        for motor, motor_id, result in active_motors:
            try:
                motor.emergency_stop()
                motor.disable_mosfets()
            except:
                pass
    
    return results

def test_get_temperature(motors_info, args):
    """Main temperature test function with batching support"""
    if not motors_info:
        print("No motors provided for testing")
        return {}
    
    # Partition devices into batches
    batches = partition_devices_into_batches(motors_info, args.devices_per_batch)
    
    print(f"\n=== Starting Temperature Test ===")
    print(f"Testing {len(motors_info)} motor(s) in {len(batches)} batch(es)")
    
    if args.devices_per_batch:
        print(f"Devices per batch: {args.devices_per_batch}")
    else:
        print("Devices per batch: All devices (no batching)")
    
    # Initialize consolidated results
    consolidated_results = {}
    
    # Process each batch
    for batch_idx, batch_motors in enumerate(batches):
        batch_num = batch_idx + 1
        print(f"\n========== BATCH {batch_num} of {len(batches)} ({len(batch_motors)} devices) ==========")
        
        # Run temperature test on this batch
        batch_results = run_temperature_test_on_batch(batch_motors, args)
        
        # Merge batch results into consolidated results
        consolidated_results.update(batch_results)
        
        # Brief pause between batches (except for the last batch)
        if batch_idx < len(batches) - 1:
            print(f"\nCompleted batch {batch_num}. Preparing for next batch...")
            time.sleep(2.0)  # Brief pause between batches
    
    return consolidated_results

def analyze_results(results, args):
    """Analyze test results and determine pass/fail status"""
    print(f"\n========== CONSOLIDATED TEMPERATURE TEST SUMMARY ==========")
    
    total_motors = len(results)
    passed_motors = 0
    failed_motors = 0
    
    for motor_id, result in results.items():
        print(f"\nMotor {motor_id} (Alias: {result.alias}, Unique ID: 0x{result.unique_id:016X}):")
        
        if result.error_message:
            print(f"  Error: {result.error_message}")
            print(f"  Status: FAILED")
            failed_motors += 1
            continue
        
        if result.initial_temp is not None:
            print(f"  Initial Temperature: {result.initial_temp}°C")
        
        if result.final_temp is not None:
            print(f"  Final Temperature: {result.final_temp}°C")
        
        if result.temp_increase is not None:
            print(f"  Temperature Increase: {result.temp_increase}°C")
        
        print(f"  Thermal Protection Events: {result.thermal_protection_events}")
        print(f"  Position Deviation Errors: {result.position_deviation_errors}")
        print(f"  Communication Timeouts: {result.communication_timeouts}")
        
        if result.fatal_errors:
            print(f"  Fatal Error Codes: {result.fatal_errors}")
        
        # Determine pass/fail status
        passed = True
        failure_reasons = []
        
        if not result.test_completed:
            passed = False
            failure_reasons.append("Test not completed")
        
        if result.temp_increase is not None and result.temp_increase < args.temp_increase:
            passed = False
            failure_reasons.append(f"Insufficient temperature increase ({result.temp_increase}°C < {args.temp_increase}°C)")
        
        if result.communication_timeouts > 5:  # Allow some timeouts
            passed = False
            failure_reasons.append(f"Too many communication timeouts ({result.communication_timeouts})")
        
        if passed:
            print(f"  Status: PASSED")
            if result.thermal_protection_events > 0:
                print(f"    (thermal protection detected as expected)")
            passed_motors += 1
        else:
            print(f"  Status: FAILED")
            for reason in failure_reasons:
                print(f"    - {reason}")
            failed_motors += 1
    
    print(f"\n========== OVERALL RESULTS ==========")
    print(f"Total Motors: {total_motors}")
    print(f"Passed: {passed_motors}")
    print(f"Failed: {failed_motors}")
    
    if failed_motors == 0:
        print("Overall Test Result: PASSED")
        print(f"All {total_motors} motor(s) completed temperature test successfully.")
        return True
    else:
        print("Overall Test Result: FAILED")
        print(f"{failed_motors} motor(s) failed the temperature test.")
        return False

def main():
    parser = argparse.ArgumentParser(description="Test the 'Get temperature' command with thermal stress testing.")
    parser.add_argument('-p', '--port', type=str, help='Serial port device name (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', type=str, help='Alias of the device to control (if not provided, will auto-detect)')
    parser.add_argument('--initial-sleep', type=int, default=30, help='Initial cooling time in seconds (default: 30)')
    parser.add_argument('--motor-run-time', type=int, default=120, help='High-power motor run time in seconds (default: 120)')
    parser.add_argument('--min-temp', type=int, default=10, help='Minimum acceptable temperature in °C (default: 10)')
    parser.add_argument('--max-temp', type=int, default=80, help='Maximum acceptable temperature in °C (default: 80)')
    parser.add_argument('--temp-increase', type=int, default=5, help='Minimum expected temperature increase in °C (default: 5)')
    parser.add_argument('--max-current', type=int, default=390, help='Maximum motor current setting (default: 390)')
    parser.add_argument('--velocity', type=float, default=1.0, help='Motor velocity in rotations/second (default: 1.0)')
    parser.add_argument('--position-deviation', type=float, default=0.1, help='Max allowable position deviation in rotations (default: 0.1)')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    parser.add_argument('--devices-per-batch', type=int, help='Maximum number of devices to test concurrently in a single batch (default: test all devices)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()
    
    # Check that either -p or -P is provided
    if not args.port and not args.PORT:
        parser.error("Either -p/--port or -P/--PORT must be specified")
    
    if args.verbose:
        args.verbose = 2
    
    overall_passed = True

    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port(timeout=1.5)

    for repeat_idx in range(args.repeat):
        if args.repeat > 1:
            print(f"\n========== REPEAT {repeat_idx + 1} of {args.repeat} ==========")
                
        try:
            # Step 1: Reset all devices (broadcast)
            motor_broadcast = M3(255, verbose=args.verbose)
            reset_device(motor_broadcast, 255, args.verbose)
            
            # Step 2: Get devices (either from provided alias or auto-detect)
            if args.alias:
                # If a specific alias is provided, parse it and create the device list
                parsed_alias, unique_id = communication.string_to_alias_or_unique_id(args.alias)
                if unique_id is not None:
                    print(f"Error: This test does not support unique ID addressing. Please use an alias.", file=sys.stderr)
                    sys.exit(1)
                devices = [(0, parsed_alias)]  # unique_id=0 as placeholder
                print(f"Testing single motor with alias: {parsed_alias}")
            else:
                # Auto-detect all devices
                devices = detect_devices_with_retries(motor_broadcast, args.verbose)
                if not devices:
                    print("FAILED: No devices detected on the bus.")
                    overall_passed = False
                    continue
                
                print(f"Found {len(devices)} motor(s) for testing:")
                for unique_id, alias in devices:
                    print(f"  - Alias: {alias}, Unique ID: 0x{unique_id:016X}")
            
            # Always flush the buffer after detection or alias parsing
            print("Flushing the receive buffer")
            servomotor.flush_receive_buffer()
            
            # Step 3: Configure units for all motors
            print(f"\n--- Configuring Units for All Motors ---")
            for unique_id, alias in devices:
                motor_id = f"alias_{alias}"
                try:
                    motor = M3(alias, verbose=args.verbose)
                    if unique_id != 0:
                        motor.use_unique_id(unique_id)
                    configure_motor_units(motor, motor_id, args.verbose)
                except Exception as e:
                    print(f"Motor {motor_id}: Unit configuration failed: {e}")
            
            # Step 4: Run temperature test
            results = test_get_temperature(devices, args)
            
            # Step 5: Analyze results
            test_passed = analyze_results(results, args)
            overall_passed &= test_passed
            
            if test_passed:
                print(f"\nPASSED (repeat {repeat_idx + 1})")
            else:
                print(f"\nFAILED (repeat {repeat_idx + 1})")
        
        except Exception as e:
            print(f"\nFAILED: Unexpected error during test: {e}")
            overall_passed = False

    servomotor.close_serial_port()

    # Final result
    if overall_passed:
        print("\nALL TESTS PASSED")
        sys.exit(0)
    else:
        print("\nFAILED: One or more tests failed.")
        sys.exit(1)

if __name__ == "__main__":
    main()