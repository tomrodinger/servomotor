#!/usr/bin/env python3

'''
###############################################################################
# Firmware Upgrade Utility for Servo Motor Controllers
###############################################################################
#
# This program upgrades the firmware on servo motor controllers using the RS485
# communication protocol. It supports both old and new protocol versions for
# compatibility with different bootloader and firmware combinations.
#
# Key features:
# - Supports both old and new protocol versions for reset and firmware transfer
# - Can handle mixed configurations (e.g., old bootloader with new firmware)
# - Reads firmware binary files with model code and compatibility information
# - Programs firmware page by page to the device's flash memory using a broadcast
#   method and in this way all devices with a matching model code and software
#   compatibility code will be updated simultaneously.
# - In the case of the new protocol, a CRC32 value is calculated and transmitted
#   to ensure data integrity in the firmware packets
#
# Protocol details:
# - Old protocol: Simple command format without size encoding or CRC32
# - New protocol: Uses proper size encoding with LSB=1 validation and CRC32 checksums
#
# Usage:
#   python upgrade_firmware.py [options] firmware-file.firmware
#
# Options:
#   -p, --port PORT           Serial port device
#   -P, --PORT                Show all ports on the system and select from menu
#   --firmware-protocol PROTO Protocol to use for resetting device (old/new)
#   --bootloader-protocol PROTO Protocol to use for transferring firmware (old/new)
#
# Process flow:
# 1. Parse command line arguments and read the firmware binary file
# 2. Open serial connection to the device
# 3. Reset the device into bootloader mode using specified protocol, and before
#    a timeout of the bootloader,
# 4. Program the firmware page by page using specified protocol
# 5. Reset the device to start the new firmware
#
# The program handles the case where a device might have an old bootloader but
# new firmware or vice versa, allowing for flexible upgrade paths.
#
###############################################################################
'''

import sys
import argparse
import time
import binascii # in the future, if you eliminate using binascii.crc32 then remove this import
import zlib
import struct
import servomotor

FIRMWARE_UPGRADE_COMMAND = 23
SYSTEM_RESET_COMMAND = 27
FLASH_BASE_ADDRESS = 0x8000000
FLASH_PAGE_SIZE = 2048
BOOTLOADER_N_PAGES = 5    # 10kB bootloader
FIRST_FIRMWARE_PAGE_NUMBER = (BOOTLOADER_N_PAGES)
LAST_FIRMWARE_PAGE_NUMBER = 30
FLASH_SETTINGS_PAGE_NUMBER = 31

MODEL_CODE_LENGTH = 8
FIRMWARE_COMPATIBILITY_CODE_LENGTH = 1
FIRMWARE_PAGE_NUMBER_LENGTH = 1
CRC32_SIZE = 4
MINIMUM_FIRWARE_SIZE = FLASH_PAGE_SIZE - CRC32_SIZE

def print_data(data):
    print(data)
    for d in data:
        print("0x%02X %d" % (d, d))


def print_usage():
    print("Usage: %s firmware-file.bin" % (sys.argv[0]))
    exit(1)


def read_binary(filename):
    print("Reading firmware file from:", filename)
    with open(filename, "rb") as fh:
        data = fh.read()
    firmware_data_size = len(data) - MODEL_CODE_LENGTH - FIRMWARE_COMPATIBILITY_CODE_LENGTH
    if firmware_data_size < MINIMUM_FIRWARE_SIZE:
        print("Error: the firmware size (%d) is less than one page of flash memory (%d)" % (firmware_size, FLASH_PAGE_SIZE))
        exit(1)
    print("The firmware file, including the header contents, has size:", len(data))
    model_code = data[0 : MODEL_CODE_LENGTH]
    firmware_compatibility_code = int.from_bytes(data[MODEL_CODE_LENGTH : MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH], byteorder = 'little')
    firmware_data = data[MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH : ]
    return model_code, firmware_compatibility_code, firmware_data


def get_response(ser):
    response = ser.read(3)
    if len(response) != 3:
        print("Error: didn't receive enough bytes in the response")
        exit(1)
    print("Received a response: ", response)
    if response[0] != RESPONSE_CHARACTER:
        print(f"Error: the first is not the expected {RESPONSE_CHARACTER}")
        exit(1)
    payload_size = response[2]
    if payload_size == 0:
        if response[1] != 0:
            print("Error: the second byte should be 0 if there is no payload")
            exit(1)
    else:
        if response[1] != 1:
            print("Error: the second byte should be 1 if there is a payload")
            exit(1)

    payload = ser.read(payload_size)
    if len(payload) != payload_size:
        print("Error: didn't receive the right length payload")
        print("Received this payload: ", payload)
        exit(1)
    print("Got a valid payload:", payload)
    return payload


def program_one_page(ser, model_code, firmware_compatibility_code, page_number, data, protocol='new'):
    assert len(data) == FLASH_PAGE_SIZE
    assert len(model_code) == MODEL_CODE_LENGTH
    print(f"Writing to page {page_number} using {protocol} protocol")
    
    if protocol == 'old':
        # Old protocol implementation
        command = int(255).to_bytes(1, "little") + FIRMWARE_UPGRADE_COMMAND.to_bytes(1, "little")
        command = command + int(255).to_bytes(1, "little") + (MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH + FIRMWARE_PAGE_NUMBER_LENGTH + FLASH_PAGE_SIZE).to_bytes(2, "little")
        command = command + model_code
        command = command + int(firmware_compatibility_code).to_bytes(FIRMWARE_COMPATIBILITY_CODE_LENGTH, "little")
        command = command + int(page_number).to_bytes(FIRMWARE_PAGE_NUMBER_LENGTH, "little")
        command = command + data
        print(f"Writing {len(command)} bytes using old protocol")

        # write the bytes in three shots with a time delay between, otherwise there is a strange bug where bytes get dropped
        ser.write(command[0:1000])
        time.sleep(0.05)
        ser.write(command[1000:2000])
        time.sleep(0.05)
        ser.write(command[2000:])
    else:
        # New protocol implementation with proper size encoding and CRC32
        # Address part (broadcast to all devices)
        address_part = int(255).to_bytes(1, "little")  # ALL_ALIAS
        
        # Command part (firmware upgrade command)
        command_part = FIRMWARE_UPGRADE_COMMAND.to_bytes(1, "little")
        
        # Payload part
        payload = model_code
        payload += int(firmware_compatibility_code).to_bytes(FIRMWARE_COMPATIBILITY_CODE_LENGTH, "little")
        payload += int(page_number).to_bytes(FIRMWARE_PAGE_NUMBER_LENGTH, "little")
        payload += data
        
        # Combine address, command, and payload
        packet_content = address_part + command_part + payload
        
        # Calculate total packet size
        packet_size = 1 + len(packet_content)  # +1 for the size byte itself
        # Add 4 bytes for CRC32
        packet_size += 4
        
        # Check if we need extended size format (size > 127)
        if packet_size > 127:
            # Since we cannot encode the size in just 1 byte, we will use extended size format
            # which adds 2 more bytes to the packet size
            packet_size += 2
            # Extended size format: first byte = 127 (encoded), followed by 2-byte size
            size_bytes = struct.pack('<BH', (127 << 1) | 0x01, packet_size)
        else:
            # Standard size format: first byte = encoded size
            size_bytes = bytearray([(packet_size << 1) | 0x01])
        
        # Create the packet without CRC32
        packet = size_bytes + packet_content
        
        # Calculate CRC32
        crc32_value = binascii.crc32(packet)
        
        # Add CRC32 to the packet
        packet += crc32_value.to_bytes(4, "little")
        
        print(f"Writing {len(packet)} bytes using new protocol")
        
        # For large packets, split the transmission to avoid buffer issues
        chunk_size = 1000
        for i in range(0, len(packet), chunk_size):
            ser.write(packet[i:i+chunk_size])
            time.sleep(0.05)

#    payload = get_response(ser)
#    if len(payload) != 0:
#        print("Error: didn't receive a payload with zero length")
#        exit(1)


def system_reset_command(ser, protocol='new'):
    print(f"Resetting the device using {protocol} protocol...")
    
    if protocol == 'old':
        # Old protocol: simple command format without CRC32
        command = int(255).to_bytes(1, "little") + SYSTEM_RESET_COMMAND.to_bytes(1, "little") + int(0).to_bytes(1, "little")
        print(f"Writing {len(command)} bytes using old protocol")
        ser.write(command)
    else:
        # New protocol: includes proper size encoding and CRC32
        # First byte: encoded size (using the protocol's encoding scheme)
        # For the new protocol, we need to encode the first byte by shifting left and setting LSB to 1
        # The packet will contain: size byte + address byte + command byte + CRC32 (4 bytes)
        
        # Address part (broadcast to all devices)
        address_part = int(255).to_bytes(1, "little")  # ALL_ALIAS
        
        # Command part (system reset command with no payload)
        command_part = SYSTEM_RESET_COMMAND.to_bytes(1, "little")
        
        # Combine address and command parts
        packet_content = address_part + command_part
        
        # Calculate total packet size (including CRC32)
        packet_size = 1 + len(packet_content) + 4  # +1 for size byte, +4 for CRC32
        
        # Encode the first byte (size byte)
        encoded_size = ((packet_size << 1) | 0x01).to_bytes(1, "little")
        
        # Create the packet without CRC32
        packet = encoded_size + packet_content
        
        # Calculate CRC32
        crc32_value = binascii.crc32(packet)
        
        # Add CRC32 to the packet
        packet += crc32_value.to_bytes(4, "little")
        
        print(f"Writing {len(packet)} bytes using new protocol")
        ser.write(packet)


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='Upgrade the firmware on a device')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('--firmware-protocol', choices=['old', 'new'], default='new', 
                    help='Protocol to use for resetting the device into bootloader mode (default: new)')
parser.add_argument('--bootloader-protocol', choices=['old', 'new'], default='new', 
                    help='Protocol to use for transferring the firmware (default: new)')
parser.add_argument('firmware_filename', help='new firmware file to send to the device')
args = parser.parse_args()

if args.PORT == True:
    serial_port = "MENU"
else:
    serial_port = args.port
firmware_filename = args.firmware_filename

# Store protocol choices
firmware_protocol = args.firmware_protocol
bootloader_protocol = args.bootloader_protocol

print(f"Using firmware protocol: {firmware_protocol}")
print(f"Using bootloader protocol: {bootloader_protocol}")

model_code, firmware_compatibility_code, data = read_binary(firmware_filename)

print("This firmware is for a device with model [%s] and firmware compatibility code [%d]" % (model_code, firmware_compatibility_code))

# pad zeros until the length of the data is divisable by 4
while len(data) & 0x03 != 0:
    data = data + b'\x00'

print("The firmware size after padding zeros to make the firmware size divisible by 4 is:", len(data))

data_uint32 = []
for item in struct.iter_unpack('<I', data):  # unpack as little endian unsigned 32-bit integers
    data_uint32.append(item[0])

# we are finished manipulating, so now repack it back into bytes
data2 = b''
for item in data_uint32:
    data2 = data2 + struct.pack('<I', item)

firmware_size = (len(data) >> 2) - 1
firmware_crc = binascii.crc32(data[4:])
second_firmware_crc = zlib.crc32(data[4:])
if (firmware_crc != second_firmware_crc):
    print("Error: binascii.crc32() and binascii.crc32() produced different results, which is unexpected")
    exit(1)
else:
    print("The upgrade_firmware progran has tested that binascii.crc32() and zlib.crc32() produced the same result. In the future we can eliminate usage of ascii.crc32 because it is slightly slower.")
print("Firmware size is %u 32-bit values. Firmware CRC32 is 0x%08X." % (firmware_size, firmware_crc))

# replacing the first 32-bit number with the firmware size. this first number contained the stack location, but we have moved this stack location to the 9th position in the startup script
data = firmware_size.to_bytes(4, "little") + data[4:] + firmware_crc.to_bytes(4, "little")

print("Will write this many bytes:", len(data))

ser = servomotor.serial_functions.open_serial_port(serial_port, 230400, 0.05)

system_reset_command(ser, protocol=firmware_protocol)
time.sleep(0.07) # wait for it to reset. I empirically determined that this delay needs to be between 0.002 and 0.13 for the firmware upgrade to work
page_number = FIRST_FIRMWARE_PAGE_NUMBER
while len(data) > 0:
    if page_number > LAST_FIRMWARE_PAGE_NUMBER:
        print("Error: the firmware is too big to fit in the flash")
        exit(1)
    print("Size left:", len(data))
    if len(data) < FLASH_PAGE_SIZE:
        data = data + bytearray([0]) * (FLASH_PAGE_SIZE - len(data))
        print("Size left after append:", len(data))
    assert len(data) >= FLASH_PAGE_SIZE
    program_one_page(ser, model_code, firmware_compatibility_code, page_number, data[0 : FLASH_PAGE_SIZE], protocol=bootloader_protocol)
    time.sleep(0.1)
    data = data[FLASH_PAGE_SIZE:]
    page_number = page_number + 1

system_reset_command(ser, protocol=bootloader_protocol)

time.sleep(0.1)

ser.close()
