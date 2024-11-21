#!/usr/bin/env python3

import random
import struct
import argparse
import time
import zlib
import servomotor

REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT = 3

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
        self.product_code = None
        self.firmware_compatibility_code = None
        self.hardware_version = None
        self.serial_number = None
        self.unique_id_2 = None
        self.unused = None
        self.firmware_version = None


def detect_all_devices_multiple_passes(n_passes):
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


def find_unused_alias(alias_dict, min_alias, max_alias):
    for alias in range(min_alias, max_alias + 1):
        if alias not in alias_dict:
            return alias
    print("Error: no more aliases available for all the devices. You should consider to increase the range of allowed aliases")
    print("The current range is from", min_alias, "to", max_alias)
    exit(1)


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Add some random moves to the queue to test the calculations of the safety limits')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
args = parser.parse_args()

serial_port = args.port

motor255 = servomotor.M3(255, motor_type="M3", time_unit="seconds", position_unit="degrees", 
                        velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                        current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius", 
                        verbose=args.verbose)
servomotor.open_serial_port()

device_dict = detect_all_devices_multiple_passes(REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT)

# Let's count how many of each alias there is in all the devices and the print out a report
# At the same time, let's check if there are any duplicate aliases
print()
alias_count = {}
duplicate_alias_list = []
for unique_id, device in device_dict.items():
    alias = device.alias
    if alias in alias_count:
        alias_count[alias] += 1
        if alias not in duplicate_alias_list:
            duplicate_alias_list.append(alias)
    else:
        alias_count[alias] = 1
for alias, count in alias_count.items():
    if alias >= 33 and alias <= 126:
        alias_str = "the ASCII character %c (or the decimal number %d)" % (alias, alias)
    else:
        alias_str = "the single byte integer %d or 0x%02x in hex" % (alias, alias)
    print(f"There are {count} devices with alias {alias_str}")

# Now, lets run the Get Product Info command for all the detected devices one by one
# Skip any where the alias is in the duplicate_alias_list, as communication withe such devices is not possible until the alias is reassigned so there is no conflict
for unique_id, device in device_dict.items():
    if device.alias in duplicate_alias_list:
        print(f"Skipping device with alias {device.alias} as it is in the duplicate alias list")
        continue
    if device.alias == 255:
        print(f"Skipping device with alias {device.alias} as it wont respond to this multicast alias")
        continue
    alias_str = servomotor.get_human_readable_alias(device.alias)
    print(f"Getting product info for device with unique ID {unique_id:016X} and alias {alias_str}")
    motor = servomotor.M3(device.alias, motor_type="M3", time_unit="seconds", position_unit="degrees", 
                    velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                    current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius")
    try:
        response = motor.get_product_info()
        device.product_code = response[0]
        device.firmware_compatibility_code = response[1]
        device.hardware_version = response[2]
        device.serial_number = response[3]
        unique_id_2 = response[4]
        if unique_id_2 != unique_id:
            print(f"Error: the unique ID 2 value is different from the unique ID: {unique_id_2:016X} vs {unique_id:016X}")
            print("This is a sanity check and this error should never occur")
            exit(1)
        device.unused = response[5]
    except Exception as e:
        print(f"Communication error: {e}")
        exit(1)
    try:
        response = motor.get_firmware_version()
        device.firmware_version = response[0]
    except Exception as e:
        print(f"Communication error: {e}")
        exit(1)

# Let's print a report showing all the devices and their aliases and any potential reassignments. Let's print in table format where columns are separated by | characters
# Include a header and make sure to print such that all the columns are aligned
print("\nDevice report:")
print("Unique ID        |         Alias  | Product Code   | Firmware Compatibility Code | Hardware Version   | Serial Number | Firmware Version")
print("----------------------------------------------------------------------------------------------------------------------------------------")
for unique_id, device in device_dict.items():
    alias_str = servomotor.get_human_readable_alias(device.alias)
    if device.alias == 255:
        print(f"{unique_id:016X} | {alias_str:14s} | *** Warning: No alias assigned: Cannot communicate until an alias is set ***")
    elif device.alias in duplicate_alias_list:
        print(f"{unique_id:016X} | {alias_str:14s} | *** Warning: Conflicting alias: Cannot communicate until a unique alias is set ***")
    else:
        hardware_version_str = f"{device.hardware_version[2]}.{device.hardware_version[1]}.{device.hardware_version[0]}"
        firmware_version_str = f"{device.firmware_version[3]}.{device.firmware_version[2]}.{device.firmware_version[1]}.{device.firmware_version[0]}"
        print(f"{unique_id:016X} | {alias_str:14s} | {device.product_code:14s} | {device.firmware_compatibility_code:27d} | {hardware_version_str:18s} | {device.serial_number:13d} | {firmware_version_str:16s}")
print("----------------------------------------------------------------------------------------------------------------------------------------")
print(f"A total of {len(device_dict)} devices were detected")
print("----------------------------------------------------------------------------------------------------------------------------------------\n")



servomotor.close_serial_port()
del motor255

