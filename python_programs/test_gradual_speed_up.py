#!/usr/bin/env python3
import servomotor
import time
import argparse
import zlib
from collections import defaultdict


SPEED_STEP = 4.0
STEP_TIME = 0.1
MAX_VELOCITY = 360.0 * 10
REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT = 3  # Number of successful device detections required
MAX_MOTOR_CURRENT = 390  # Maximum motor current value


def print_test_description():
    pass

def compute_crc32(unique_id, alias):
    """Compute CRC32 for device detection response"""
    try:
        # Convert to integers if needed
        if isinstance(unique_id, (list, tuple)):
            unique_id = unique_id[0]
        if isinstance(alias, (list, tuple)):
            alias = alias[0]
            
        # Handle large numbers by using modulo
        unique_id = unique_id % (2**64)  # Ensure it fits in 64 bits
        alias = alias % 256  # Ensure it fits in 8 bits
        
        # Convert to bytes
        data_bytes = unique_id.to_bytes(8, 'little') + alias.to_bytes(1, 'little')
        return zlib.crc32(data_bytes)
    except Exception as e:
        print(f"Error computing CRC32: {e}")
        print(f"unique_id type: {type(unique_id)}, value: {unique_id}")
        print(f"alias type: {type(alias)}, value: {alias}")
        return None

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
            crc = device[2]
            # We need to compute the CRC32 of the unique ID and the alias and then compare it to the CRC value
            crc32_value = compute_crc32(unique_id, alias)
            # The unique ID should be printed as a 16 digit hexadecimal and the CRC should be printed as a 8 digit hexadecimal
            if crc == crc32_value:
                print(f"Unique ID: {unique_id:016X}, Alias: {alias}, CRC: {crc:08X} (CHECK IS OK)")
                if unique_id in device_dict:
                    print(f"This unique ID {unique_id:016X} is already in the device dictionary, so not adding it again")
                    if alias != device_dict[unique_id].alias:
                        print(f"Error: we discovered an inconsistency: the alias is different: {alias} vs {device_dict[unique_id].alias}")
                else:
                    new_device = Device(unique_id, alias)
                    device_dict[unique_id] = new_device
            else:
                print(f"Unique ID: {unique_id:016X}, Alias: {alias}, CRC: {crc:08X} (CHECK FAILED: computed crc: {crc32_value:08X} vs. received crc: {crc:08X})")
    return device_dict


parser = argparse.ArgumentParser(description='Servomotor random speed stress test')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
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
for unique_id, device in new_device_dict.items():
    print(f"\n=== Testing device {unique_id:016X} (Alias: {device.alias}) ===")

    motor.set_alias(device.alias)
    motor.enable_mosfets()
    time.sleep(0.05)
    motor.set_maximum_motor_current(MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT)

    current_speed = 0.0
    while 1:
        print(f"=== alias = {device.alias} speed = {current_speed} =============================================================================")
        current_speed += SPEED_STEP

        temperature = motor.get_temperature()
        print(f"Temperature: {temperature}")

        while 1:
            n_queued_items = motor.get_n_queued_items()
    #        print(f"Queue items: {n_queued_items}")
            if n_queued_items >= 2:
    #            print("We will wait for a moment before queing more")
                time.sleep(0.05)
            else:
                break

        status = motor.get_status()
    #    print(f"Status: {status}")
        if len(status) == 2 and status[1] != 0:
            print(f"A fatal error occured with the motor. The error code is: {status[1]}")
            break

        # Move with velocity in degrees per second
        motor.move_with_velocity(current_speed, STEP_TIME)
        if current_speed > MAX_VELOCITY:
            motor.move_with_velocity(0, STEP_TIME)
            print("Reached maximum velocity")
            break

    motor.disable_mosfets()

servomotor.close_serial_port()
del motor
