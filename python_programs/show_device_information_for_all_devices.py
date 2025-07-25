#!/usr/bin/env python3

import random
import struct
import argparse
import time
import zlib
import servomotor
from servomotor.device_detection import detect_devices_iteratively

REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT = 4

def compute_crc32(data_64bit, data_8bit):
    # Convert to bytes in little-endian format
    data_bytes = data_64bit.to_bytes(8, 'little') + data_8bit.to_bytes(1, 'little')
    
    # Compute the CRC32
    crc32_value = zlib.crc32(data_bytes)
    return crc32_value


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
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
args = parser.parse_args()

servomotor.set_serial_port_from_args(args)

motor255 = servomotor.M3(alias_or_unique_id=255, verbose=args.verbose)
servomotor.open_serial_port()

devices = detect_devices_iteratively(REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT, verbose=args.verbose)
# Let's count how many of each alias there is in all the devices and the print out a report
# At the same time, let's check if there are any duplicate aliases
print()
alias_count = {}
duplicate_alias_list = []
for device in devices:
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
for device in devices:
    if device.alias in duplicate_alias_list:
        print(f"Skipping device with alias {device.alias} as it is in the duplicate alias list")
        continue
    if device.alias == 255:
        print(f"Skipping device with alias {device.alias} as it wont respond to this multicast alias")
        continue
    alias_str = servomotor.get_human_readable_alias_or_unique_id(device.alias)
    print(f"Getting product info for device with unique ID {device.unique_id:016X} and alias {alias_str}")
    motor = servomotor.M3(device.unique_id, verbose=args.verbose)
    try:
        response = motor.get_product_info()
        device.product_code = response[0]
        device.firmware_compatibility_code = response[1]
        device.hardware_version = response[2]
        device.serial_number = response[3]
        unique_id_2 = response[4]
        if unique_id_2 != device.unique_id:
            print(f"Error: the unique ID 2 value is different from the unique ID: {unique_id_2:016X} vs {device.unique_id:016X}")
            print("This is a sanity check and this error should never occur")
            exit(1)
        device.unused = response[5]
    except Exception as e:
        print(f"Communication error: {e}")
        exit(1)
    try:
        response = motor.get_firmware_version()
        device.firmware_version = response[0]
        in_bootloader = response[1]
        assert(in_bootloader == 0)
    except Exception as e:
        print(f"Communication error: {e}")
        exit(1)

# Let's print a report showing all the devices and their aliases and any potential reassignments. Let's print in table format where columns are separated by | characters
# Include a header and make sure to print such that all the columns are aligned
print("\nDevice report:")
print("Unique ID        |         Alias  | Product Code   | Firmware Compatibility Code | Hardware Version   | Serial Number | Firmware Version")
print("----------------------------------------------------------------------------------------------------------------------------------------")
for device in devices:
    alias_str = servomotor.get_human_readable_alias_or_unique_id(device.alias)
    if device.alias == 255:
        print(f"{device.unique_id:016X} | {alias_str:14s} | *** Warning: No alias assigned: Cannot communicate until an alias is set ***")
    elif device.alias in duplicate_alias_list:
        print(f"{device.unique_id:016X} | {alias_str:14s} | *** Warning: Conflicting alias: Cannot communicate until a unique alias is set ***")
    else:
        hardware_version_str = f"{device.hardware_version[2]}.{device.hardware_version[1]}.{device.hardware_version[0]}"
        firmware_version_str = f"{device.firmware_version[3]}.{device.firmware_version[2]}.{device.firmware_version[1]}.{device.firmware_version[0]}"
        print(f"{device.unique_id:016X} | {alias_str:14s} | {device.product_code:14s} | {device.firmware_compatibility_code:27d} | {hardware_version_str:18s} | {device.serial_number:13d} | {firmware_version_str:16s}")
print("----------------------------------------------------------------------------------------------------------------------------------------")
print(f"A total of {len(devices)} devices were detected")
print("----------------------------------------------------------------------------------------------------------------------------------------\n")



servomotor.close_serial_port()
del motor255

