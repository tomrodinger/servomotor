#!/usr/bin/env python3
import servomotor
from servomotor import communication
import time
import argparse
import zlib
from collections import defaultdict
import sys


SPEED_STEP = 8.0
STEP_TIME = 0.1
MAX_VELOCITY = 340.0 * 10
REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT = 3  # Number of successful device detections required
MAX_MOTOR_CURRENT = 390  # Maximum motor current value


def print_test_description():
    pass

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
            unique_id, alias = device
            print(f"Unique ID: {unique_id:016X}, Alias: {alias}")
            if unique_id in device_dict:
                print(f"This unique ID {unique_id:016X} is already in the device dictionary, so not adding it again")
                if alias != device_dict[unique_id].alias:
                    print(f"Error: we discovered an inconsistency: the alias is different: {alias} vs {device_dict[unique_id].alias}")
            else:
                new_device = Device(unique_id, alias)
                device_dict[unique_id] = new_device
    return device_dict


parser = argparse.ArgumentParser(description='Servomotor random speed stress test')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
parser.add_argument('-a', '--alias', help='Alias of the device (ignored by this test)', default=None) # Added to accept the argument
args = parser.parse_args()

# Print initial test description
print_test_description()

if args.verbose:
    verbose_level = 2
else:
    verbose_level = 0


motor = servomotor.M3(servomotor.ALL_ALIAS, time_unit="seconds", position_unit="degrees", velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius", verbose=verbose_level)
servomotor.set_serial_port_from_args(args)
servomotor.open_serial_port()

# Detect devices (multiple times)
new_device_dict = detect_all_devices_multiple_passes(motor, REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT)

print(f"\nDetected {len(new_device_dict)} devices:")
for unique_id, device in new_device_dict.items():
    print(f"  Device {unique_id:016X} (Alias: {device.alias})")


# Iterate through all devices
test_passed = True
for unique_id, device in new_device_dict.items():
    print(f"\n=== Testing device {unique_id:016X} (Alias: {device.alias}) ===")
    try:
        motor.use_this_alias_or_unique_id(device.alias)
        motor.enable_mosfets()
        time.sleep(0.05)
        motor.set_maximum_motor_current(MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT)

        current_speed = 0.0
        while 1:
            print(f"=== alias = {device.alias} speed = {current_speed} degrees per second = {current_speed * 60 / 360} RPM =================================")
            current_speed += SPEED_STEP

            temperature = motor.get_temperature()
            print(f"Temperature: {temperature}")

            while 1:
                n_queued_items = motor.get_n_queued_items()
        #        print(f"Queue items: {n_queued_items}")
                if n_queued_items >= 3:
        #            print("We will wait for a moment before queing more")
                    time.sleep(0.05)
                else:
                    break

            status = motor.get_status()
        #    print(f"Status: {status}")
            if len(status) == 2 and status[1] != 0:
                print(f"A fatal error occured with the motor. The error code is: {status[1]}")
                test_passed = False
                break

            # Move with velocity in degrees per second
            motor.move_with_velocity(current_speed, STEP_TIME)
            if current_speed > MAX_VELOCITY:
                motor.move_with_velocity(0, STEP_TIME)
                print("Reached maximum velocity")
                break

    except communication.TimeoutError as e:
        print(f"Timeout error: {e}", file=sys.stderr)
        test_passed = False
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
        test_passed = False
    finally:
        try:
            motor.disable_mosfets()
        except communication.TimeoutError:
            print("Timeout while disabling motor. Motor is likely unresponsive.", file=sys.stderr)

servomotor.close_serial_port()
del motor

if test_passed:
    print("PASSED")
    sys.exit(0)
else:
    print("FAILED")
    sys.exit(1)
