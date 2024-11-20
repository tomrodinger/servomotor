#!/usr/bin/env python3

import random
import time
import argparse
import servomotor
from collections import defaultdict
import zlib

# Configuration parameters
N_ENABLED_DEVICES_AT_SAME_TIME = 2  # Number of devices to enable simultaneously
REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT = 3
MIN_ALIAS = 1
MAX_ALIAS = 6
MAX_MOTOR_CURRENT = 390  # Maximum current value

def print_test_description():
    """Print a detailed description of the test"""
    print("\n" + "="*80)
    print("SERVOMOTOR STRESS TEST DESCRIPTION")
    print("="*80)
    print("""
This test program performs comprehensive stress testing of servomotors with the following steps:

1. System Reset and Device Detection:
   - Performs initial system reset
   - Detects all connected devices with CRC verification
   - Requires 3 successful device detections for reliability

2. Alias Assignment:
   - Assigns unique aliases to all detected devices
   - Ensures aliases are within range 1-6
   - Resolves any alias conflicts

3. Communication Verification:
   - Performs 10 rounds of ping tests on all devices
   - Verifies data integrity with random test payloads
   - Reports any communication failures

4. Random Speed Stress Testing:
   - Tests {0} devices simultaneously
   - Sets maximum motor current to {1}mA
   - Randomly varies speeds between 0.1 and 3.0 rotations per second
   - Movement pattern:
     * 2 seconds at random speed
     * 0.01 seconds deceleration to stop
   - Continuously monitors device status

5. Error Monitoring:
   - Tracks fatal errors per device
   - Maintains error statistics
   - Triggers system reset on any fatal error
   - Reports error counts every 10 seconds

6. Periodic Maintenance:
   - Performs system reset every 60 seconds
   - Re-detects all devices (3 attempts)
   - Prints statistics every 10 seconds
   - Prints this description every 30 seconds

7. Safety Features:
   - Proper MOSFET control (enable/disable)
   - Controlled acceleration/deceleration
   - Immediate error response
   - Clean shutdown on interrupt

This test verifies:
- Motor control reliability
- Communication robustness
- Error handling
- System recovery
- Long-term stability
""".format(N_ENABLED_DEVICES_AT_SAME_TIME, MAX_MOTOR_CURRENT))
    print("="*80 + "\n")

def compute_crc32(data_64bit, data_8bit):
    # Convert to bytes in little-endian format
    data_bytes = data_64bit.to_bytes(8, 'little') + data_8bit.to_bytes(1, 'little')
    # Compute the CRC32
    crc32_value = zlib.crc32(data_bytes)
    return crc32_value

class Device:
    def __init__(self, unique_id, alias):
        self.unique_id = unique_id
        self.alias = alias
        self.fatal_error_counts = defaultdict(int)  # Track counts of each error code
        self.total_fatal_errors = 0

def detect_devices(motor255):
    """Detect all devices and return a dictionary of devices"""
    device_dict = {}
    successful_detect_devices_count = 0
    detect_devices_attempt_count = 0
    
    while True:
        if successful_detect_devices_count >= REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT:
            break

        print(f"Detecting devices attempt {detect_devices_attempt_count+1}")
        try:
            response = motor255.detect_devices()
            successful_detect_devices_count += 1
            
            # The response is a list of [unique_id, alias, crc] values
            if isinstance(response, list) and len(response) >= 3:
                # The response contains the values directly
                unique_id = response[0]  # This is already an integer
                alias = response[1]      # This is already an integer
                crc = response[2]        # This is already an integer
                
                # Verify CRC
                crc32_value = compute_crc32(unique_id, alias)
                print(f"Unique ID: {unique_id:016X}, Alias: {alias}, CRC: {crc:08X}")
                
                if crc == crc32_value:
                    print("CRC32 matches")
                    if unique_id not in device_dict:
                        new_device = Device(unique_id, alias)
                        device_dict[unique_id] = new_device
                        print(f"Added device - Unique ID: {unique_id:016X}, Alias: {alias}")
                    else:
                        print(f"Device {unique_id:016X} already in dictionary")
                else:
                    print("CRC32 does not match:")
                    print(f"   Computed: {crc32_value:08X}")
                    print(f"   Received: {crc:08X}")
            else:
                print(f"Unexpected response format: {response}")
            
        except Exception as e:
            print(f"Communication error: {e}")
            continue
        
        detect_devices_attempt_count += 1
    
    return device_dict

def assign_unique_aliases(motor255, device_dict):
    """Assign unique aliases to all devices"""
    used_aliases = []
    for unique_id, device in device_dict.items():
        if device.alias >= MIN_ALIAS and device.alias <= MAX_ALIAS and device.alias not in used_aliases:
            used_aliases.append(device.alias)
            continue
        
        # Need to assign new alias
        new_alias = next(i for i in range(MIN_ALIAS, MAX_ALIAS + 1) if i not in used_aliases)
        used_aliases.append(new_alias)
        print(f"Reassigning device {unique_id:016X} from alias {device.alias} to {new_alias}")
        response = motor255.set_device_alias(device.unique_id, new_alias)
        device.alias = new_alias

def run_ping_test(motor255, device_dict):
    """Run ping test on all devices"""
    print("\nRunning ping test on all devices...")
    for _ in range(10):
        for unique_id, device in device_dict.items():
            motor = servomotor.M3(device.alias, motor_type="M3", time_unit="seconds", position_unit="degrees", 
                                velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                                current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius")
            try:
                # Send random data as ping payload
                ping_data = bytes([random.randint(0, 255) for _ in range(10)])
                response = motor.ping(ping_data)
                if response != ping_data:  # Compare entire response with entire ping_data
                    print(f"Ping test failed for device {unique_id:016X}")
            except Exception as e:
                print(f"Ping test failed for device {unique_id:016X}: {e}")
        time.sleep(0.1)  # Small delay between ping rounds

def print_statistics(device_dict, start_time):
    """Print statistics about devices and errors"""
    print("\n=== Test Statistics ===")
    print(f"Test running for: {int(time.time() - start_time)} seconds")
    print(f"Total devices: {len(device_dict)}")
    
    for unique_id, device in device_dict.items():
        print(f"\nDevice {unique_id:016X} (Alias: {device.alias}):")
        print(f"Total fatal errors: {device.total_fatal_errors}")
        if device.fatal_error_counts:
            print("Fatal error breakdown:")
            for error_code, count in device.fatal_error_counts.items():
                print(f"  Error code {error_code}: {count} occurrences")

def main():
    parser = argparse.ArgumentParser(description='Servomotor random speed stress test')
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
    args = parser.parse_args()

    # Print initial test description
    print_test_description()

    # Initialize communication
    servomotor.set_serial_port_from_args(args)
    motor255 = servomotor.M3(255, motor_type="M3", time_unit="seconds", position_unit="degrees", 
                            velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                            current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius", 
                            verbose=args.verbose)
    servomotor.open_serial_port()

    try:
        # Initial system reset
        print("Performing initial system reset...")
        motor255.system_reset()
        time.sleep(1.5)

        # Detect all devices
        print("\nDetecting devices...")
        device_dict = detect_devices(motor255)
        if not device_dict:
            print("No devices detected!")
            return

        # Assign unique aliases
        print("\nAssigning unique aliases...")
        assign_unique_aliases(motor255, device_dict)

        # Run ping test
        run_ping_test(motor255, device_dict)

        # Main test loop
        print("\nStarting main test loop...")
        start_time = time.time()
        last_stats_time = start_time
        last_reset_time = start_time
        last_description_time = start_time

        while True:
            current_time = time.time()
            
            # Every minute, do system reset and device detection
            if current_time - last_reset_time >= 60:
                print("\nPerforming periodic system reset and device detection...")
                motor255.system_reset()
                time.sleep(1.5)
                
                # Detect devices 3 times
                for i in range(3):
                    print(f"\nDevice detection attempt {i+1}/3")
                    device_dict = detect_devices(motor255)
                    if not device_dict:
                        print("Warning: No devices detected!")
                
                last_reset_time = current_time

            # Every 30 seconds, print test description
            if current_time - last_description_time >= 30:
                print_test_description()
                last_description_time = current_time

            # Every 10 seconds, print statistics
            if current_time - last_stats_time >= 10:
                print_statistics(device_dict, start_time)
                last_stats_time = current_time

            # Randomly select N devices to test
            test_devices = random.sample(list(device_dict.values()), 
                                      min(N_ENABLED_DEVICES_AT_SAME_TIME, len(device_dict)))
            
            # Enable MOSFETs and set current for selected devices
            for device in test_devices:
                motor = servomotor.M3(device.alias, motor_type="M3", time_unit="seconds", position_unit="degrees", 
                                    velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                                    current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius")
                motor.enable_mosfets()
                motor.set_maximum_motor_current(MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT)

            # Apply random velocity movement
            for device in test_devices:
                motor = servomotor.M3(device.alias, motor_type="M3", time_unit="seconds", position_unit="degrees", 
                                    velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                                    current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius")
                
                # Random speed between 0.1 and 3 rotations per second (convert to degrees/second)
                speed = random.uniform(0.1, 3.0) * 360  # Convert rotations/sec to degrees/sec
                
                # Move at random speed for 2 seconds
                motor.move_with_velocity(speed, 2.0)  # Pass as positional args
                
                # Stop over 0.01 seconds
                motor.move_with_velocity(0, 0.01)  # Pass as positional args

            # Small delay to let movements complete
            time.sleep(2.1)  # 2 seconds + 0.01 seconds + small buffer

            # Check status and disable MOSFETs
            for device in test_devices:
                motor = servomotor.M3(device.alias, motor_type="M3", time_unit="seconds", position_unit="degrees", 
                                    velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                                    current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius")
                
                # Get status and check for errors
                status = motor.get_status()
                if len(status) >= 2 and status[1] != 0:
                    error_code = status[1]
                    device.fatal_error_counts[error_code] += 1
                    device.total_fatal_errors += 1
                    print(f"\nFatal error {error_code} detected on device {device.unique_id:016X}")
                    
                    # Reset system if any fatal error occurs
                    print("Resetting system due to fatal error...")
                    motor255.system_reset()
                    time.sleep(1.5)
                    break
                
                # Disable MOSFETs
                motor.disable_mosfets()

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        # Clean up
        servomotor.close_serial_port()
        del motor255

if __name__ == "__main__":
    main()
