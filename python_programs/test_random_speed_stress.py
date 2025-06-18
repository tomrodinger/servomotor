#!/usr/bin/env python3

import random
import time
import argparse
import servomotor
from servomotor import get_human_readable_alias
from collections import defaultdict
import zlib
import paho.mqtt.client as mqtt
import json
import uuid

# Configuration parameters
N_ENABLED_DEVICES_AT_SAME_TIME = 8  # Number of devices to enable simultaneously and make them spin
REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT = 3  # Number of successful device detections required
MIN_ALIAS = 1
MAX_ALIAS = 253  # Maximum possible alias value
MAX_MOTOR_CURRENT = 390  # Maximum current value
PING_TIMEOUT_RETRIES = 3  # Number of retries on ping timeout
DETECT_DEVICES_INTERVAL = 60 * 5  # Reset the system and Detect devices every this number of seconds
PRINT_TEST_STATISTICS_INTERVAL = 30  # Print test statistics every this number of seconds
PRINT_TEST_DESCRIPTION_INTERVAL = 60 * 5  # Print test description every this number of seconds, so that the user knows what test is running
MIN_VELOCITY = -4.0  # Minimum velocity in rotations per second
MAX_VELOCITY = 4.0  # Maximum velocity in rotations per second
STATUS_CHECK_PASSES = 5

# MQTT Configuration
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_TOPIC = "servomotor/statistics"
mqtt_client = None

def setup_mqtt():
    """Setup MQTT client with non-blocking connection"""
    global mqtt_client
    # Generate a unique client ID
    client_id = f'servomotor_publisher_{str(uuid.uuid4())[:8]}'
    mqtt_client = mqtt.Client(client_id=client_id)
    
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT broker")
        else:
            print(f"Failed to connect to MQTT broker with code: {rc}")
    
    mqtt_client.on_connect = on_connect
    
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()  # Start the loop in a background thread
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}")
        mqtt_client = None

def send_statistics_to_mqtt(device_dict, start_time, movement_iterations_done, iterations_with_errors, n_devices_detected_historgram):
    """Send statistics to MQTT broker"""
    if mqtt_client is None:
        return
    
    try:
        # Prepare the data in a similar format to print_statistics
        data = {
            "test_duration": int(time.time() - start_time),
            "total_iterations": movement_iterations_done,
            "iterations_with_errors": iterations_with_errors,
            "total_devices": len(device_dict),
            "devices_histogram": {str(k): v for k, v in n_devices_detected_historgram.items()},
            "devices": [],
            "total_error_counts": defaultdict(int)
        }
        
        # Add per-device statistics
        for unique_id, device in device_dict.items():
            device_data = {
                "unique_id": f"{unique_id:016X}",
                "alias": get_human_readable_alias(device.alias),
                "total_fatal_errors": device.total_fatal_errors,
                "error_counts": {str(k): v for k, v in device.fatal_error_counts.items()}
            }
            data["devices"].append(device_data)
            
            # Accumulate total error counts
            for error_code, count in device.fatal_error_counts.items():
                data["total_error_counts"][str(error_code)] += count
        
        # Convert defaultdict to regular dict for JSON serialization
        data["total_error_counts"] = dict(data["total_error_counts"])
        
        # Publish with a 1-second timeout
        mqtt_client.publish(MQTT_TOPIC, json.dumps(data), qos=1).wait_for_publish(timeout=1.0)
    except Exception as e:
        print(f"Failed to send statistics to MQTT: {e}")

class Device:
    def __init__(self, unique_id, alias):
        self.unique_id = unique_id
        self.alias = alias
        self.fatal_error_counts = defaultdict(int)  # Track counts of each error code
        self.total_fatal_errors = 0


def detect_all_devices_multiple_passes(motor255, n_passes):
    # Let's detect all devices (possible multiple times) and store the data (unique_id and alias) in a dictionary
    device_dict = {}
    successful_detect_devices_count = 0
    detect_devices_attempt_count = 0
    while 1:
        print("Resetting the system")
        motor255.system_reset()
        time.sleep(1.5)

        if successful_detect_devices_count >= n_passes:
            break

        print("Flushing the receive buffer")
        servomotor.flush_receive_buffer()
        print(f"Detecting devices attempt {detect_devices_attempt_count+1}/{n_passes}")
        try:
            response = motor255.detect_devices()
            successful_detect_devices_count += 1
        except Exception as e:
            print(f"Communication error: {e}")
            continue
        detect_devices_attempt_count += 1
        print("Detected devices:")
        for device in response:
            unique_id = device[0]
            alias = device[1]
            print(f"Unique ID: {unique_id:016X}, Alias: {get_human_readable_alias(alias)}")
            if unique_id in device_dict:
                print(f"This unique ID {unique_id:016X} is already in the device dictionary, so not adding it again")
                if alias != device_dict[unique_id].alias:
                    print(f"Error: we discovered an inconsistency: the alias is different: {get_human_readable_alias(alias)} vs {get_human_readable_alias(device_dict[unique_id].alias)}")
            else:
                new_device = Device(unique_id, alias)
                device_dict[unique_id] = new_device
    return device_dict


# This function will merge data from new_device_dict into device_dict keeping existing statistics intact. If there is a conflict, it will print an error message
def merge_device_dict(device_dict, new_device_dict):
    print("\nMerging new devices into the existing device dictionary...")
    for unique_id, new_device in new_device_dict.items():
        if unique_id in device_dict:
            print(f"Unique ID {unique_id:016X} is already in the device dictionary, so not adding it again")
            if new_device.alias != device_dict[unique_id].alias:
                print(f"Error: we discovered an inconsistency: the alias is different: {get_human_readable_alias(new_device.alias)} vs {get_human_readable_alias(device_dict[unique_id].alias)}")
        else:
            device_dict[unique_id] = new_device


def assign_unique_aliases(motor255, device_dict):
    """Assign unique aliases to all devices"""
    if len(device_dict) > MAX_ALIAS - MIN_ALIAS + 1:
        raise ValueError(f"Too many devices ({len(device_dict)}) for available alias range ({get_human_readable_alias(MIN_ALIAS)}-{get_human_readable_alias(MAX_ALIAS)})")
        
    used_aliases = []
    reassigned_devices = []
    
    # First pass: identify devices that need new aliases
    for unique_id, device in device_dict.items():
        if device.alias >= MIN_ALIAS and device.alias <= MAX_ALIAS and device.alias not in used_aliases:
            used_aliases.append(device.alias)
            continue
        reassigned_devices.append(device)
    
    # Second pass: assign new aliases
    for device in reassigned_devices:
        try:
            new_alias = next(i for i in range(MIN_ALIAS, MAX_ALIAS + 1) if i not in used_aliases)
            used_aliases.append(new_alias)
            print(f"Reassigning device {device.unique_id:016X} from alias {get_human_readable_alias(device.alias)} to {get_human_readable_alias(new_alias)}")
            response = motor255.set_device_alias(device.unique_id, new_alias)
            device.alias = new_alias
            # Add delay after each reassignment
            time.sleep(0.1)
        except StopIteration:
            print(f"Error: No more available aliases in range {get_human_readable_alias(MIN_ALIAS)}-{get_human_readable_alias(MAX_ALIAS)}")
            raise ValueError(f"No more available aliases in range {get_human_readable_alias(MIN_ALIAS)}-{get_human_readable_alias(MAX_ALIAS)}")
    
    # If any devices were reassigned, do a system reset
    if reassigned_devices:
        print("\nPerforming system reset after alias reassignment...")
        motor255.system_reset()
        time.sleep(1.5)

def run_ping_test(motor255, device_dict, verbose=2):
    """Run ping test on all devices"""
    ping_test_passed = True
    print("\nRunning ping test on all devices...")

    motor = servomotor.M3(255, time_unit="seconds", position_unit="degrees", 
                        velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                        current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius", verbose=verbose)

    for ping_round in range(10):
        for unique_id, device in device_dict.items():
            if device.unique_id != 0:
                motor.use_unique_id(device.unique_id)
            elif device.alias != None:
                motor.use_alias(device.alias)
            else:
                print("Invalid alias or unique id")
                exit(1)
            # Try ping with retries
            for retry in range(PING_TIMEOUT_RETRIES):
                try:
                    # Send random data as ping payload
                    ping_data = bytes([random.randint(0, 255) for _ in range(10)])
                    response = motor.ping(ping_data)
                    if response != ping_data:
                        ping_test_passed = False
                        print(f"Ping test round {ping_round + 1}/10 failed for device {unique_id:016X} (Alias: {get_human_readable_alias(device.alias)})")
                        print(f"Sent: {ping_data}")
                        print(f"Received: {response}")
                    else:
                        print(f"Ping test round {ping_round + 1}/10 successful for device {unique_id:016X} (Alias: {get_human_readable_alias(device.alias)})")
                    break  # Success, exit retry loop
                except Exception as e:
                    ping_test_passed = False
                    print(f"Ping test round {ping_round + 1}/10 attempt {retry + 1}/{PING_TIMEOUT_RETRIES} failed for device {unique_id:016X} (Alias: {get_human_readable_alias(device.alias)}): {e}")
                    if retry == PING_TIMEOUT_RETRIES - 1:  # Last retry
                        print(f"All ping retries failed for device {unique_id:016X} (Alias: {get_human_readable_alias(device.alias)})")
                    else:
                        time.sleep(0.1)  # Delay before retry
        
        # Delay between ping rounds
        time.sleep(0.1)
    return ping_test_passed

def print_statistics(device_dict, start_time, movement_iterations_done, iterations_with_errors, n_devices_detected_historgram):
    """Print statistics about devices and errors"""
    print("\n============================================================ Test Statistics ============================================================")
    print(f"Test running for: {int(time.time() - start_time)} seconds")
    print(f"Total movement iterations: {movement_iterations_done}")
    print(f"Total iterations with errors: {iterations_with_errors}")
    if movement_iterations_done > 0:
        print(f"Percentage of iterations with errors: {100 * iterations_with_errors / movement_iterations_done:.2f}%")
    print(f"Total unique devices detected in all detections: {len(device_dict)}")
    # Print a histogram from 0 to len(device_dict) of the number of devices detected and show the number and the bar graph as a bunch of # signs
    # Make sure to normalize to a bar length of 50 if the number of devices detected is greater than 50
    max_devices_detected = max(n_devices_detected_historgram.keys())
    if max_devices_detected > 50:
        scale_factor = 50 / max_devices_detected
    else:
        scale_factor = 1
    print("Histogram of number of devices detected:")
    for i in range(max_devices_detected + 1):
        if i in n_devices_detected_historgram:
            print(f"{i:2d} devices: {'#' * int(n_devices_detected_historgram[i] * scale_factor)} ({n_devices_detected_historgram[i]})")
        else:
            print(f"{i:2d} devices: 0")
    
    error_type_counts = {}
    for unique_id, device in device_dict.items():
        print(f"\nDevice {unique_id:016X} (Alias: {get_human_readable_alias(device.alias)}):")
        print(f"   Total fatal errors: {device.total_fatal_errors}")
        if device.fatal_error_counts:
            print("      Fatal error breakdown:")
            for error_code, count in device.fatal_error_counts.items():
                print(f"      Error code {error_code}: {count} occurrences")
                if error_code in error_type_counts:
                    error_type_counts[error_code] += count
                else:
                    error_type_counts[error_code] = count
    print("\nSum over error types:")
    for error_code, count in error_type_counts.items():
        print(f"   Error code {error_code}: {count} total occurrences")
    print("\n=========================================================================================================================================")
    
    # Send statistics to MQTT
    send_statistics_to_mqtt(device_dict, start_time, movement_iterations_done, iterations_with_errors, n_devices_detected_historgram)

def print_test_description():
    """Print a detailed description of the test"""
    print("\n" + "-" * 80)
    print("SERVOMOTOR STRESS TEST DESCRIPTION")
    print("-" * 80)
    print("""
This test program performs comprehensive stress testing of servomotors with the following steps:

1. System Reset and Device Detection:
   - Performs initial system reset
   - Detects all connected devices with CRC verification
   - Requires 3 successful device detections for reliability

2. Alias Assignment:
   - Assigns unique aliases to all detected devices
   - Ensures aliases are within range 1-253
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
    print("-" * 80 + "\n")

def main():
    parser = argparse.ArgumentParser(description='Servomotor random speed stress test')
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('-a', '--alias', help='Alias of a specific device to test (will filter auto-detected devices)', default=None)
    parser.add_argument('-d', '--duration', type=int, default=30, help='Total duration of the test in seconds.')
    parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
    args = parser.parse_args()

    # Print initial test description
    print_test_description()

    if args.verbose:
        verbose_level = 2
    else:
        verbose_level = 0

    # Initialize communication
    servomotor.set_serial_port_from_args(args)
    motor255 = servomotor.M3(255, time_unit="seconds", position_unit="degrees", 
                            velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                            current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius", 
                            verbose=verbose_level)
    servomotor.open_serial_port()
    servomotor.flush_receive_buffer()
    
    # Setup MQTT connection
    setup_mqtt()

    # Main test loop
    print("\nStarting main test loop...")
    start_time = time.time()
    last_stats_time = start_time
    last_reset_time = 0
    last_description_time = start_time
    unique_aliases_assigned = False
    device_dict = {}
    fatal_error_detected = False
    movement_iterations_done = 0
    iterations_with_errors = 0
    n_devices_detected_historgram = defaultdict(int)

    motor = servomotor.M3(255, time_unit="seconds", position_unit="degrees",
                    velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared",
                    current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius", verbose=verbose_level)

    # If an alias is provided, skip detection and test that device directly
    if args.alias:
        try:
            alias, unique_id = servomotor.communication.string_to_alias_or_unique_id(args.alias)
            # Use a placeholder for unique_id as it's not known without detection
            if unique_id == None:
                unique_id = 0
            device_dict[unique_id] = Device(unique_id, alias)
            unique_aliases_assigned = True # Skip alias assignment logic
            n_devices_detected_historgram[1] += 1
            print(f"Bypassing auto-detection. Testing single device with alias: {alias}")
            print("Resetting the system")
            motor255.system_reset()
            time.sleep(1.5)
        except Exception as e:
            print(f"Error: Could not process alias '{args.alias}': {e}")
            exit(1)

    while time.time() - start_time < args.duration:
        current_time = time.time()

        # At the start and also every once in a while, do system reset and device detection
        # This block is skipped if a specific alias was provided
        if not args.alias and ((current_time - last_reset_time >= DETECT_DEVICES_INTERVAL) or fatal_error_detected):
            # Initial system reset
            print("Performing initial or periodic system reset...")
            motor255.system_reset()
            time.sleep(1.5)
            last_reset_time = current_time
            fatal_error_detected = False

            # Detect devices (multiple times)
            new_device_dict = detect_all_devices_multiple_passes(motor255, REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT)
            print(f"\nDetected {len(new_device_dict)} devices:")
            for unique_id, device in new_device_dict.items():
                print(f"  Device {unique_id:016X} (Alias: {get_human_readable_alias(device.alias)})")
            n_devices_detected_historgram[len(new_device_dict)] += 1
            merge_device_dict(device_dict, new_device_dict)

#            motor255.set_max_allowable_position_deviation(360 * 1000000) # This will prevent a fatal error if the motor tries to spin too fast and loose too many steps

        # If no devices detected, don't go on, instead go back to the beginning to try resetting and detecting devices again
        if not device_dict:
            print("Warning: No devices detected!")
            print("Will retry in a while...")
            time.sleep(10)
            continue

        # Assign unique aliases, but do this just one time at the start
        # This block is skipped if a specific alias was provided
        if not unique_aliases_assigned:
            print("\nAssigning unique aliases...")
            assign_unique_aliases(motor255, device_dict)
            unique_aliases_assigned = True
            print("\nUnique aliases assigned successfully")

            # Run ping test, also do this just once at the start
            ping_test_passed = run_ping_test(motor255, device_dict, verbose=verbose_level)
            if not ping_test_passed:
                print("\nPing test failed! Please check the communication with the devices")
                print("Exiting test...")
                exit(1)

        # Every certain number of seconds, print test description
        if current_time - last_description_time >= PRINT_TEST_DESCRIPTION_INTERVAL:
            print_test_description()
            last_description_time = current_time

        # Every 10 seconds, print statistics
        if current_time - last_stats_time >= PRINT_TEST_STATISTICS_INTERVAL:
            print_statistics(device_dict, start_time, movement_iterations_done, iterations_with_errors, n_devices_detected_historgram)
            last_stats_time = current_time

        # Randomly select N devices to test
        test_devices = random.sample(list(device_dict.values()), min(N_ENABLED_DEVICES_AT_SAME_TIME, len(device_dict)))
        
        try:
            # Enable MOSFETs and set current for selected devices
            for device in test_devices:
                if device.unique_id != 0:
                    motor.use_unique_id(device.unique_id)
                elif device.alias != None:
                    motor.use_alias(device.alias)
                else:
                    print("Invalid alias or unique id")
                    exit(1)
                motor.enable_mosfets()
                motor.set_maximum_motor_current(MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT)

            # Apply random velocity movement
            for device in test_devices:
                if device.unique_id != 0:
                    motor.use_unique_id(device.unique_id)
                elif device.alias != None:
                    motor.use_alias(device.alias)
                else:
                    print("Invalid alias or unique id")
                    exit(1)
                
                # Random speed between 0.1 and 3 rotations per second (convert to degrees/second)
                speed = random.uniform(MIN_VELOCITY, MAX_VELOCITY) * 360  # Convert rotations/sec to degrees/sec
                
                # Move at random speed for 2 seconds
                if device.unique_id != 0:
                    print(f"Moving device {device.unique_id:016X} at {speed:.2f} degrees/sec for 2 seconds...")
                elif device.alias != None:
                    print(f"Moving device with alias {get_human_readable_alias(device.alias)} at {speed:.2f} degrees/sec for 2 seconds...")

                print(f"DEBUG: move_with_velocity args: speed={speed}, duration=2.0")
                motor.move_with_velocity(speed, 2.0)  # Pass as positional args
                # Stop over 0.01 seconds
                print(f"DEBUG: move_with_velocity args: speed=0, duration=0.01")
                motor.move_with_velocity(0, 0.01)  # Pass as positional args

        except Exception as e:
            print(f"Error during random speed stress test: {e}")
            fatal_error_detected = True

        # Small delay to let movements complete
        time.sleep(2.1)  # 2 seconds + 0.01 seconds + small buffer

        movement_iterations_done += 1

        # disable all MOSFETs at once. This cannot fail as it is a multicast command and not devices will respond
        motor255.disable_mosfets()
        time.sleep(0.2)  # Small delay to make sure transmitted commands are flushed out before checking status
        motor255.disable_mosfets()
        time.sleep(0.2)  # Small delay to make sure transmitted commands are flushed out before checking status
        motor255.disable_mosfets()
        time.sleep(0.2)  # Small delay to make sure transmitted commands are flushed out before checking status
        motor255.get_status() # this won't return any response, but it may help any corrupted messages get flushed. i am not sure if this is needed
        # Check status of all motors. If any fatal error is detected then set a flag, and we will reset the whole system later if this flag is set
        time.sleep(0.2)  # Small delay to make sure transmitted commands are flushed out before checking status
        servomotor.flush_receive_buffer()
        motor255.get_status() # this won't return any response, but it may help any corrupted messages get flushed. i am not sure if this is needed
        unknown_errors_detected = {}
        known_errors_detected = {}
        # Let's do multiple passes to try to read the status of all the devices, ince I found that this step is unreliable
        for i in range(STATUS_CHECK_PASSES):
            for device in device_dict.values():
                if device.unique_id != 0:
                    motor.use_unique_id(device.unique_id)
                elif device.alias != None:
                    motor.use_alias(device.alias)
                else:
                    print("Invalid alias or unique id")
                    exit(1)
                # Get status and check for errors
                try:
                    time.sleep(0.15)
                    if device.unique_id != 0:
                        print(f"DEBUG: get_status for unique_id {device.unique_id:016X}")
                    elif device.alias != None:
                        print(f"DEBUG: get_status for alias {get_human_readable_alias(device.alias)}")
                    status = motor.get_status()
                    print(f"DEBUG: status = {status}")
                    assert len(status) == 2
                    error_code = status[1]
                    if error_code != 0:
                        known_errors_detected[device.unique_id] = error_code
#                        device.fatal_error_counts[error_code] += 1
#                        device.total_fatal_errors += 1
                        print(f"\nFatal error {error_code} detected on device {device.unique_id:016X}")
                        fatal_error_detected = True
                except Exception as e:
                    unknown_errors_detected[device.unique_id] = 1
#                    device.fatal_error_counts[0] += 1
#                    device.total_fatal_errors += 1
                    print(f"Error checking status for device {device.unique_id:016X}: {e}")
                    fatal_error_detected = True
            if len(unknown_errors_detected.keys()) == 0:
                print("No unknown errors detected this time, so no need to check again")
                break
        # Now that we have gathered the errors, let's increment our error statistics appropriately
        if fatal_error_detected:
            for device in device_dict.values():
                if device.unique_id in known_errors_detected:
                    device.fatal_error_counts[known_errors_detected[device.unique_id]] += 1
                    device.total_fatal_errors += 1
                    print(f"After all checking, we found that device {device.unique_id:016X} suffered a {known_errors_detected[device.unique_id]} fatal error ")
                elif device.unique_id in unknown_errors_detected:
                    device.fatal_error_counts[0] += 1
                    device.total_fatal_errors += 1
                    print(f"After all checking, we found that device {device.unique_id:016X} suffered an unknown fatal error ")
            iterations_with_errors += 1

if __name__ == "__main__":
    test_passed = True
    start_time = time.time()
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        test_passed = False
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        test_passed = False
    finally:
        # Ensure MQTT client is properly shut down
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        # Ensure serial port is closed
        servomotor.close_serial_port()
        end_time = time.time()
        print(f"\nTest finished in {end_time - start_time:.2f} seconds.")

    if test_passed:
        print("\nPASSED")
        exit(0)
    else:
        print("\nFAILED")
        exit(1)
