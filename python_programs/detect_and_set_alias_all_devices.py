#!/usr/bin/env python3

import random
import struct
import argparse
import time
import zlib
import servomotor

REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT = 3
MIN_ALIAS = 1
#MAX_ALIAS = 253
MAX_ALIAS = 6

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
        self.need_to_reassign_alias = False
        self.reassigned_alias = None


def find_unused_alias(alias_list, min_alias, max_alias):
    for alias in range(min_alias, max_alias + 1):
        if alias not in alias_list:
            return alias
    print("Error: no more aliases available for all the devices. You should consider to increase the range of allowed aliases")
    print("The current range is from", min_alias, "to", max_alias)
    exit(1)


def get_human_readable_alias(alias):
    if alias >= 33 and alias <= 126:
        alias_str = "%c (%d)" % (alias, alias)
    else:
        alias_str = "%d (0x%02x)" % (alias, alias)
    return alias_str


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Add some random moves to the queue to test the calculations of the safety limits')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-r', '--reassign', help='reassign aliases (if necessary) so that there is no conflict and all aliases fall in the given range from MIN_ALIAS to MAX_ALIAS (which is specified inside the program)', action='store_true')
parser.add_argument('-c', '--calibration', help='do the calibration of all motors one by one (so as to not overload the power supply)', action='store_true')
parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
args = parser.parse_args()

servomotor.set_serial_port_from_args(args)
reassign_aliases = args.reassign
do_calibration = args.calibration

motor255 = servomotor.M3(255, motor_type="M3", time_unit="seconds", position_unit="degrees", velocity_unit="degrees/s", acceleration_unit="degree/s^2", current_unit="mA", voltage_unit="V", temperature_unit="C", verbose=args.verbose)
servomotor.open_serial_port()

# Let's detect all devices (possible multiple times) and store the data (unique_id and alias) in a dictionary
device_dict = {}
successful_detect_devices_count = 0
detect_devices_attempt_count = 0
while 1:
    print("Resetting the system")
    motor255.system_reset()
    time.sleep(1.5)

    if successful_detect_devices_count >= REQUIRED_SUCCESSFUL_DETECT_DEVICES_COUNT:
        break

    print("Flushing the receive buffer")
    servomotor.flush_receive_buffer()
    print(f"Detecting devices attempt {detect_devices_attempt_count+1}")
    try:
        response = motor255.detect_devices()
        successful_detect_devices_count += 1
    except Exception as e:
        print(f"Communication error: {e}")
        continue
    detect_devices_attempt_count += 1
    print("Detected devices:")
    for device in response:
        print("Device:", device)
        unique_id = device[0]
        alias = device[1]
        crc = device[2]
        # We need to compute the CRC32 of the unique ID and the alias and then compare it to the CRC value
        crc32_value = compute_crc32(unique_id, alias)
        # The unique ID should be printed as a 16 digit hexadecimal and the CRC should be printed as a 8 digit hexadecimal
        print(f"Unique ID: {unique_id:016X}, Alias: {alias}, CRC: {crc:08X}")
        if crc == crc32_value:
            print("CRC32 matches")
            if unique_id in device_dict:
                print(f"This unique ID {unique_id:016X} is already in the device dictionary, so not adding it again")
                if alias != device_dict[unique_id].alias:
                    print(f"Error: we discovered an inconsistency: the alias is different: {alias} vs {device_dict[unique_id].alias}")
            else:
                new_device = Device(unique_id, alias)
                device_dict[unique_id] = new_device
        else:
            print("CRC32 does not match:")
            print("   The CRC32 value computed is:", crc32_value)
            print("   The CRC32 value received from the device is:", crc)

# Let's count how many of each alias there is in all the devices and the print out a report
print()
alias_count = {}
for unique_id, device in device_dict.items():
    alias = device.alias
    if alias in alias_count:
        alias_count[alias] += 1
    else:
        alias_count[alias] = 1
for alias, count in alias_count.items():
    if alias >= 33 and alias <= 126:
        alias_str = "the ASCII character %c (or the decimal number %d)" % (alias, alias)
    else:
        alias_str = "the single byte integer %d or 0x%02x in hex" % (alias, alias)
    print(f"There are {count} devices with alias {alias_str}")

# Now, we need to check which aliases are used more than once and reassign them as necessary to avoid conflicts (and to adhere to the allowed alias range from MIN_ALIAS to MAX_ALIAS)
valid_used_alias_list = []
for unique_id, device in device_dict.items():
    if device.alias >= MIN_ALIAS and device.alias <= MAX_ALIAS and device.alias not in valid_used_alias_list:
        valid_used_alias_list.append(device.alias)
    else:
        device.need_to_reassign_alias = True
for unique_id, device in device_dict.items():
    if device.need_to_reassign_alias:
        new_alias = find_unused_alias(valid_used_alias_list, MIN_ALIAS, MAX_ALIAS)
        valid_used_alias_list.append(new_alias)
        device.reassigned_alias = new_alias

# Let's print a report showing all the devices and their aliases and any potential reassignments. Let's print in table format where columns are separated by | characters
# Include a header and make sure to print such that all the columns are aligned
print("\nDevice report:")
print("Unique ID        | Original Alias  | Reassigned Alias")
print("-----------------------------------------------------")
for unique_id, device in device_dict.items():
    alias_str = get_human_readable_alias(device.alias)
    if device.reassigned_alias is None:
        reassigned_alias_str = "                 "
    else:
        reassigned_alias_str = get_human_readable_alias(device.reassigned_alias)
    print(f"{unique_id:016X} | {alias_str:15s} | {reassigned_alias_str:15s}")
print("-----------------------------------------------------")
print(f"A total of {len(device_dict)} devices were detected")
print("-----------------------------------------------------\n")

# Now, let's reassign the aliases (if the user has specified to do so) to the devices by running the "Set device alias" command for each device that needs to be reassigned
if reassign_aliases:
    for unique_id, device in device_dict.items():
        if device.reassigned_alias is not None:
            print(f"Reassigning the alias of the device with unique ID {unique_id:016X} from {device.alias} to {device.reassigned_alias}")
            response = motor255.set_device_alias(device.unique_id, device.reassigned_alias)
            print("Response:", response)

# If the user has specified to do so, then we will calibrate all the devices one by one (so as to not overload the power supply)
failed_calibrations = {}
if do_calibration:
    print("Calibration mode activated")
    for unique_id, device in device_dict.items():
        print(f"Calibrating the device with unique ID {unique_id:016X} and alias {device.alias}")
        motor_to_calibrate = servomotor.M3(device.alias, motor_type="M3", time_unit="seconds", position_unit="degrees", velocity_unit="degrees/s", acceleration_unit="degree/s^2", current_unit="mA", voltage_unit="V", temperature_unit="C", verbose=args.verbose)
        response = motor_to_calibrate.start_calibration()
        print("Response from Start Calibration command:", response)
        time.sleep(30)
        response = motor_to_calibrate.get_status()
        print("Response from Get Status command:", response)
        if response[1] == 0:
            print("Calibration completed successfully")
        else:
            failed_calibrations[unique_id] = True
            print("Calibration failed")
    # Let's print a report of all the failed calibrations (if there were some), otherwise print a message that all calibrations were successful
    if len(failed_calibrations) > 0:
        print("The following devices failed calibration:")
        for unique_id in failed_calibrations:
            print(f"   Device with unique ID {unique_id:016X} and alias {device_dict[unique_id].alias}")
    else:
        print("All devices were successfully calibrated")

servomotor.close_serial_port()
del motor255
