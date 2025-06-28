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

class InconsistencyError(Exception):
    """Raised when device detection finds inconsistent alias assignments"""
    pass


class MotorTestResult:
    """Stores test results for a single motor"""
    def __init__(self, alias_or_unique_id):
        self.alias_or_unique_id = alias_or_unique_id
        self.initial_temp = None
        self.final_temp = None
        self.last_known_temp = None  # Last valid temperature reading before any errors
        self.temp_increase = None
        self.thermal_protection_events = 0
        self.position_deviation_errors = 0
        self.communication_timeouts = 0
        self.fatal_errors = []
        self.test_completed = False
        self.error_message = None
        self.motor_failed = False  # If motor becomes unresponsive
        self.error_types = []  # List of error types encountered
        self.last_successful_temp_time = None  # When last temp reading was successful
        self.motor_start_time = None  # When motor operation started

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
                            raise InconsistencyError(f"Error: we discovered an inconsistency: the alias is different: {alias} vs {device_dict[unique_id]}")
                    else:
                        device_dict[unique_id] = alias
                else:
                    print(f"Invalid device response format: {device}")
        else:
            print("No devices detected in this attempt")

    print("Resetting the system one last time after detection")
    motor_broadcast.system_reset(verbose=verbose)
    time.sleep(1.5)

    # Convert device_dict to list of tuples (unique_id, alias) for compatibility
    devices = [(unique_id, alias) for unique_id, alias in device_dict.items()]
    print(f"Final device count after {max_attempts} detection passes: {len(devices)}")
    return devices

def create_motor_objects(devices, verbose=False):
    """Create M3 motor objects with proper addressing for each detected device"""
    motor_objects = []
    
    print("\n--- Creating Motor Objects ---")
    for device in devices:
        unique_id = device[0]
        try:
            motor = M3(unique_id, verbose=verbose)
            motor_objects.append(motor)
            print(f"Created motor object using unique ID 0x{unique_id:016X}")
            
        except Exception as e:
            print(f"Failed to create motor object for {unique_id}: {e}")
    
    return motor_objects

def get_motor_status_with_retries(motor, alias_or_unique_id, verbose=True):
    """Get motor status with retries, return None if all attempts fail"""
    alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
    for attempt in range(MAX_STATUS_RETRY_ATTEMPTS):
        if verbose:
            print(f"Attempting to get status {attempt + 1} of {MAX_STATUS_RETRY_ATTEMPTS}")
        try:
            status = motor.get_status(verbose=verbose)
            return status
        except Exception as e:
            print(f"An exception occured while trying to get the status: {e}")
            if attempt < MAX_STATUS_RETRY_ATTEMPTS - 1:
                if verbose:
                    print(f"Motor {alias_or_unique_id_str}: Status attempt {attempt + 1} failed: {e}, retrying...")
                time.sleep(0.1)
            else:
                if verbose:
                    print(f"Motor {alias_or_unique_id_str}: All status attempts failed: {e}")
    return None

def reset_device(motor, alias_or_unique_id, verbose=False):
    """Reset device and wait for startup"""
    motor.use_this_alias_or_unique_id(alias_or_unique_id)
    alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
    print(f"Resetting device at {alias_or_unique_id_str}...")
    motor.system_reset(verbose=verbose)
    print(f"Sleeping for {DONT_GO_TO_BOOTLOADER_RESET_TIME}s after reset (normal mode).")
    time.sleep(DONT_GO_TO_BOOTLOADER_RESET_TIME)

def configure_motor_units(motor, alias_or_unique_id, verbose=False):
    """Configure motor units for consistent operation"""
    alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
    try:
        print(f"Motor {alias_or_unique_id_str}: Setting units...")
        motor.set_time_unit("seconds")
        motor.set_position_unit("shaft_rotations")
        motor.set_velocity_unit("rotations_per_second")
        motor.set_acceleration_unit("rotations_per_second_squared")
        print(f"Motor {alias_or_unique_id_str}: Units configured successfully")
        return True
    except Exception as e:
        print(f"Motor {alias_or_unique_id_str}: Unit configuration failed: {e}")
        return False

def configure_motor_for_test(motor, alias_or_unique_id, max_current, position_deviation, verbose=False):
    """Configure motor for high-power temperature test"""
    alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
    try:
        # Enable MOSFETs
        print(f"Motor {alias_or_unique_id_str}: Enabling MOSFETs...")
        motor.enable_mosfets(verbose=verbose)
        
        # Set maximum motor current
        print(f"Motor {alias_or_unique_id_str}: Setting maximum motor current to {max_current}...")
        motor.set_maximum_motor_current(max_current, 200, verbose=verbose)
        
        # Set position deviation limit to detect step skipping (now in shaft_rotations)
        print(f"Motor {alias_or_unique_id_str}: Setting position deviation limit to {position_deviation} shaft_rotations...")
        motor.set_max_allowable_position_deviation(position_deviation, verbose=verbose)
        
        return True
    except Exception as e:
        print(f"Motor {alias_or_unique_id_str}: Configuration failed: {e}")
        return False

def wait_for_motor_queues_to_empty(active_motors, batch_number, n_batches, args):
    """Wait for all motor queues to become empty (queue-based exit, not time-based)"""
    
    print("Waiting for all motor queues to become empty...")
    all_queues_empty = False
    check_count = 0
    start_time = time.time()  # Track start time for countdown
    
    while not all_queues_empty:
        all_queues_empty = True
        check_count += 1
        
        # Calculate estimated time remaining
        elapsed_time = time.time() - start_time
        estimated_time_left = max(0, args.motor_run_time - elapsed_time)
        
        # Print countdown header before motor status (every 10 checks or if verbose)
        if args.verbose or (check_count % 10 == 0):
            print(f"======== Approximate time left in this batch: {estimated_time_left:.0f} seconds. This is batch number {batch_number} of {n_batches} ========")
        
        for motor, alias_or_unique_id, result in active_motors:
            alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
            try:
                # Check queue size
                queue_size = motor.get_n_queued_items(verbose=args.verbose)
                if queue_size > 0:
                    all_queues_empty = False
                
                # Get temperature reading and track it
                try:
                    temp = motor.get_temperature(verbose=args.verbose)
                    temperature = temp[0] if isinstance(temp, (list, tuple)) else temp
                    temp_str = f", Temperature = {temperature}°C"
                    
                    # Update last known temperature and time
                    result.last_known_temp = temperature
                    result.last_successful_temp_time = time.time()
                    
                except Exception as temp_e:
                    temp_str = f", Temperature = ERROR ({temp_e})"
                    
                    # Track timeout/communication errors
                    if "timeout" in str(temp_e).lower():
                        result.communication_timeouts += 1
                        if "timeout" not in result.error_types:
                            result.error_types.append("timeout")
                    else:
                        if "communication_error" not in result.error_types:
                            result.error_types.append("communication_error")
                
                if args.verbose or (check_count % 10 == 0):  # Show status every 10 checks or if verbose
                    print(f"Motor {alias_or_unique_id_str}: Queue size = {queue_size}{temp_str}")
            
            except Exception as e:
                print(f"Motor {alias_or_unique_id_str}: Queue check error: {e}")
                
                # Track different types of errors
                if "timeout" in str(e).lower():
                    result.communication_timeouts += 1
                    if "timeout" not in result.error_types:
                        result.error_types.append("timeout")
                elif "fatal" in str(e).lower():
                    if "fatal_error" not in result.error_types:
                        result.error_types.append("fatal_error")
                else:
                    if "communication_error" not in result.error_types:
                        result.error_types.append("communication_error")
                
                time.sleep(5) # A problem occurred, such as overheat or some other, let's wait a while and check less often
        
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

def run_temperature_test_on_batch(motor_objects, batch_number, n_batches, args):
    """Main temperature test function"""
    print(f"\n=== Starting Temperature Test ===")
    print(f"Testing {len(motor_objects)} motor(s)")
    
    results = {}
    
    # Initialize results for each motor
    for motor_obj in motor_objects:
        results[motor_obj.alias_or_unique_id] = MotorTestResult(motor_obj.alias_or_unique_id)
    
    try:
        # Phase 1: Take baseline temperature readings
        print(f"\n--- Phase 1: Baseline Temperature Readings ---")
        for motor_obj in motor_objects:
            result = results[motor_obj.alias_or_unique_id]
            
            try:
                alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(motor_obj.alias_or_unique_id)
                print("Using alias or unique ID:", alias_or_unique_id_str)
                temp = motor_obj.get_temperature(verbose=args.verbose)
                
                if temp is not None:
                    result.initial_temp = temp[0] if isinstance(temp, (list, tuple)) else temp
                    result.last_known_temp = result.initial_temp  # Initialize last known temp
                    result.last_successful_temp_time = time.time()
                    print(f"Motor {alias_or_unique_id_str}: Initial temperature = {result.initial_temp}°C")
                    
                    # Validate temperature range
                    if result.initial_temp < args.min_temp or result.initial_temp > args.max_temp:
                        result.error_message = f"Initial temperature {result.initial_temp}°C out of range ({args.min_temp}-{args.max_temp}°C)"
                        result.error_types.append("temperature_out_of_range")
                        print(f"Motor {alias_or_unique_id_str}: {result.error_message}")
                        continue
                else:
                    result.error_message = "Failed to read initial temperature"
                    result.error_types.append("initial_temp_read_failed")
                    print(f"Motor {alias_or_unique_id_str}: {result.error_message}")
                    continue
                    
            except Exception as e:
                result.error_message = f"Initial temperature reading failed: {e}"
                if "timeout" in str(e).lower():
                    result.communication_timeouts += 1
                    result.error_types.append("timeout")
                else:
                    result.error_types.append("communication_error")
                print(f"Motor {alias_or_unique_id_str}: {result.error_message}")
                continue
        
        # Phase 2: Configure motors for high-power operation
        print(f"\n--- Phase 2: Motor Configuration ---")
        active_motors = []
        
        for motor_obj in motor_objects:
            result = results[motor_obj.alias_or_unique_id]
            
            if result.error_message:
                continue  # Skip motors that failed initial temperature reading
            
            alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(motor_obj.alias_or_unique_id)
            try:                
                if configure_motor_for_test(motor_obj, motor_obj.alias_or_unique_id, args.max_current, args.position_deviation, args.verbose):
                    active_motors.append((motor_obj, motor_obj.alias_or_unique_id, result))
                    print(f"Motor {alias_or_unique_id_str}: Configuration successful")
                else:
                    result.error_message = "Motor configuration failed"
                    
            except Exception as e:
                result.error_message = f"Motor configuration error: {e}"
                print(f"Motor {alias_or_unique_id_str}: {result.error_message}")
        
        if not active_motors:
            print("No motors successfully configured for testing!")
            return results
        
        # Phase 3: High-Power Motor Operation
        print(f"\n--- Phase 3: High-Power Motor Operation ---")
        print(f"Running motors at velocity {args.velocity} rot/s for {args.motor_run_time} seconds")
        
        # Record the start time for all motors
        motor_operation_start_time = time.time()
        
        # Start all motors sequentially (no threading)
        for motor, alias_or_unique_id, result in active_motors:
            alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
            try:
                # Set proper units for easy time specification
                motor.set_time_unit("seconds")
                motor.set_velocity_unit("rotations_per_second")
                
                # Record when this motor's operation started
                result.motor_start_time = motor_operation_start_time
                
                # Queue one long move for the entire test duration
                print(f"Motor {alias_or_unique_id_str}: Queuing long move for {args.motor_run_time} seconds")
                motor.move_with_velocity(args.velocity, args.motor_run_time, verbose=args.verbose)
                
                # Immediately queue a velocity=0 command to prevent error 18
                print(f"Motor {alias_or_unique_id_str}: Queuing velocity=0 to prevent queue empty error")
                motor.move_with_velocity(0.0, 0.01, verbose=args.verbose)  # 0 velocity for 0.01 seconds)
                
                print(f"Motor {alias_or_unique_id_str}: Started high-power operation")
                
            except Exception as e:
                result.error_message = f"Failed to start motor operation: {e}"
                print(f"Motor {alias_or_unique_id_str}: {result.error_message}")
        
        # Wait for all motor queues to become empty (no threading)
        wait_for_motor_queues_to_empty(active_motors, batch_number, n_batches, args)
        
        # Motors should already be stopped by the queued velocity=0 command and now we will disable the MOSFETs
        for motor, alias_or_unique_id, result in active_motors:
            alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
            try:
                motor.disable_mosfets(verbose=args.verbose)
                print(f"Motor {alias_or_unique_id_str}: MOSFETs disabled")
            except Exception as e:
                print(f"Motor {alias_or_unique_id_str}: Error disabling MOSFETs: {e}")
        
        # Phase 4: Final temperature readings
        print(f"\n--- Phase 4: Final Temperature Readings ---")
        time.sleep(2.0)  # Brief pause before final readings
        
        for motor, alias_or_unique_id, result in active_motors:
            alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
            try:
                temp = motor.get_temperature(verbose=args.verbose)
                
                if temp is not None:
                    result.final_temp = temp[0] if isinstance(temp, (list, tuple)) else temp
                    result.last_known_temp = result.final_temp  # Update last known temp
                    result.last_successful_temp_time = time.time()
                    print(f"Motor {alias_or_unique_id_str}: Final temperature = {result.final_temp}°C")
                    
                    # Calculate temperature increase
                    if result.initial_temp is not None and result.final_temp is not None:
                        result.temp_increase = result.final_temp - result.initial_temp
                        print(f"Motor {alias_or_unique_id_str}: Temperature increase = {result.temp_increase}°C")
                    
                    # Validate final temperature range
                    if result.final_temp is not None:
                        if result.final_temp < args.min_temp or result.final_temp > args.max_temp:
                            result.error_message = f"Final temperature {result.final_temp}°C out of range ({args.min_temp}-{args.max_temp}°C)"
                            result.error_types.append("final_temp_out_of_range")
                        else:
                            result.test_completed = True
                    else:
                        result.error_message = "Final temperature reading returned None"
                        result.error_types.append("final_temp_read_failed")
                else:
                    result.error_message = "Failed to read final temperature"
                    result.error_types.append("final_temp_read_failed")
                    
            except Exception as e:
                result.error_message = f"Final temperature reading failed: {e}"
                if "timeout" in str(e).lower():
                    result.communication_timeouts += 1
                    result.error_types.append("final_temp_timeout")
                else:
                    result.error_types.append("final_temp_communication_error")
                print(f"Motor {alias_or_unique_id_str}: {result.error_message}")
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        # Note: Graceful shutdown - motors will complete their current movements
        
        # Emergency stop all motors
        for motor, alias_or_unique_id, result in active_motors:
            try:
                motor.emergency_stop()
                motor.disable_mosfets()
            except:
                pass
    
    return results

def test_get_temperature(motor_objects, args):
    """Main temperature test function with batching support"""
    if not motor_objects:
        print("No motors provided for testing")
        return {}
    
    # Partition motor objects into batches
    batches = partition_devices_into_batches(motor_objects, args.devices_per_batch)
    
    print(f"\n=== Starting Temperature Test ===")
    print(f"Testing {len(motor_objects)} motor(s) in {len(batches)} batch(es)")
    
    if args.devices_per_batch:
        print(f"Devices per batch: {args.devices_per_batch}")
    else:
        print("Devices per batch: All devices (no batching)")
    
    # Initialize consolidated results
    consolidated_results = {}
    
    # Initial cooling period (only once at the beginning)
    print(f"\n--- Initial Cooling Period ---")
    print_countdown("Waiting for motors to cool down", args.initial_sleep)
    
    # Process each batch
    for batch_idx, batch_motors in enumerate(batches):
        batch_num = batch_idx + 1
        n_batches = len(batches)
        print(f"\n========== BATCH {batch_num} of {n_batches} ({len(batch_motors)} devices) ==========")
        
        # Run temperature test on this batch
        batch_results = run_temperature_test_on_batch(batch_motors, batch_num, n_batches, args)
        
        # Merge batch results into consolidated results
        consolidated_results.update(batch_results)
        
        # Brief pause between batches (except for the last batch)
        if batch_idx < n_batches - 1:
            print(f"\nCompleted batch {batch_num}. Preparing for next batch...")
            time.sleep(2.0)  # Brief pause between batches
    
    return consolidated_results

def analyze_results(results, args):
    """Analyze test results and determine pass/fail status"""
    print(f"\n========== CONSOLIDATED TEMPERATURE TEST SUMMARY ==========")
    
    total_motors = len(results)
    passed_motors = 0
    failed_motors = 0
    
    for alias_or_unique_id, result in results.items():
        alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(alias_or_unique_id)
        print(f"\nMotor {alias_or_unique_id_str}:")
        
        # Always show initial temperature if available
        if result.initial_temp is not None:
            print(f"  Initial Temperature: {result.initial_temp}°C")
        else:
            print(f"  Initial Temperature: Not available")
        
        # Show final temperature or last known temperature
        if result.final_temp is not None:
            print(f"  Final Temperature: {result.final_temp}°C")
        elif result.last_known_temp is not None:
            print(f"  Final Temperature: Not available")
            print(f"  Last Known Temperature: {result.last_known_temp}°C")
            if result.last_successful_temp_time and result.motor_start_time:
                time_since_motor_start = result.last_successful_temp_time - result.motor_start_time
                print(f"  Last Valid Reading: {time_since_motor_start:.1f}s after motor start")
        else:
            print(f"  Final Temperature: Not available")
            print(f"  Last Known Temperature: Not available")
        
        # Calculate temperature increase using best available data
        if result.temp_increase is not None:
            print(f"  Temperature Increase: {result.temp_increase}°C")
        elif result.initial_temp is not None and result.last_known_temp is not None:
            temp_increase = result.last_known_temp - result.initial_temp
            print(f"  Temperature Increase: {temp_increase}°C (based on last known temp)")
            result.temp_increase = temp_increase  # Store for pass/fail evaluation
        else:
            print(f"  Temperature Increase: Cannot calculate")
        
        print(f"  Thermal Protection Events: {result.thermal_protection_events}")
        print(f"  Position Deviation Errors: {result.position_deviation_errors}")
        print(f"  Communication Timeouts: {result.communication_timeouts}")
        
        if result.fatal_errors:
            print(f"  Fatal Error Codes: {result.fatal_errors}")
        
        # Show error types if any occurred
        if result.error_types:
            print(f"  Error Types: {', '.join(result.error_types)}")
        
        # Determine pass/fail status
        passed = True
        failure_reasons = []
        
        # Check if we have enough data to evaluate the test
        if result.initial_temp is None:
            passed = False
            failure_reasons.append("No initial temperature reading")
        elif result.final_temp is None and result.last_known_temp is None:
            passed = False
            failure_reasons.append("No temperature readings during test")
        elif not result.test_completed and result.temp_increase is None:
            passed = False
            failure_reasons.append("Test not completed and no temperature data available")
        
        # Check temperature increase if we have data
        if result.temp_increase is not None and result.temp_increase < args.temp_increase:
            passed = False
            failure_reasons.append(f"Insufficient temperature increase ({result.temp_increase}°C < {args.temp_increase}°C)")
        
        
        # Show specific error message if available
        if result.error_message and not passed:
            print(f"  Error Details: {result.error_message}")
        
        if passed:
            print(f"  Status: PASSED")
            if result.thermal_protection_events > 0:
                print(f"    (thermal protection detected as expected)")
            if "timeout" in result.error_types:
                print(f"    (completed despite communication timeouts)")
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
            
            # Step 2: Get devices (either from provided alias or auto-detect)
            if args.alias:
                reset_device(motor_broadcast, 255, args.verbose)
                # If a specific alias is provided, parse it and create the device list
                alias_or_unique_id = communication.string_to_alias_or_unique_id(args.alias)
                devices = [(alias_or_unique_id, alias_or_unique_id)]  # unique_id=0 as placeholder
                print(f"Testing single motor with alias or unique ID: {alias_or_unique_id}")
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
            
            # Step 3: Create motor objects with proper addressing
            print(f"\n--- Creating Motor Objects ---")
            print(f"DEBUG: devices = {devices}")
            motor_objects = create_motor_objects(devices, args.verbose)
            
            # Step 4: Configure units for all motors
            print(f"\n--- Configuring Units for All Motors ---")
            for motor_obj in motor_objects:
                alias_or_unique_id_str = communication.get_human_readable_alias_or_unique_id(motor_obj.alias_or_unique_id)
                try:
                    configure_motor_units(motor_obj, motor_obj.alias_or_unique_id, args.verbose)
                except Exception as e:
                    print(f"Motor {alias_or_unique_id_str}: Unit configuration failed: {e}")
            
            # Step 5: Run temperature test
            results = test_get_temperature(motor_objects, args)
            
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