#!/usr/bin/env python3

import sys
import argparse
import time
import binascii
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
    if response[0] != ENCODED_RESPONSE_CHARACTER:
        print(f"Error: the first is not the expected {ENCODED_RESPONSE_CHARACTER}")
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


def program_one_page(ser, model_code, firmware_compatibility_code, page_number, data):
    assert len(data) == FLASH_PAGE_SIZE
    assert len(model_code) == MODEL_CODE_LENGTH
    print("Writing to page:", page_number)
    command = int(255).to_bytes(1, "little") + FIRMWARE_UPGRADE_COMMAND.to_bytes(1, "little")
    command = command + int(255).to_bytes(1, "little") + (MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH + FIRMWARE_PAGE_NUMBER_LENGTH + FLASH_PAGE_SIZE).to_bytes(2, "little")
    command = command + model_code
    command = command + int(firmware_compatibility_code).to_bytes(FIRMWARE_COMPATIBILITY_CODE_LENGTH, "little")
    command = command + int(page_number).to_bytes(FIRMWARE_PAGE_NUMBER_LENGTH, "little")
    command = command + data
    print("Writing %d bytes" % (len(command)))

#    print_data(command)

    # write the bytes in three shots with a time delay betwoen, otherwise there is a strange bug where bytes get dropped
    ser.write(command[0:1000])
    time.sleep(0.05)
    ser.write(command[1000:2000])
    time.sleep(0.05)
    ser.write(command[2000:])

#    payload = get_response(ser)
#    if len(payload) != 0:
#        print("Error: didn't receive a payload with zero length")
#        exit(1)


def system_reset_command(ser):
    print("Resettting the newly programmed device...")
    command = int(255).to_bytes(1, "little") + SYSTEM_RESET_COMMAND.to_bytes(1, "little") + int(0).to_bytes(1, "little")
    print("Writing %d bytes" % (len(command)))
    ser.write(command)


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='Upgrade the firmware on a device')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('firmware_filename', help='new firmware file to send to the device')
args = parser.parse_args()

if args.PORT == True:
    serial_port = "MENU"
else:
    serial_port = args.port
firmware_filename = args.firmware_filename

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
print("Firmware size is %u 32-bit values. Firmware CRC32 is 0x%08X." % (firmware_size, firmware_crc))

# replacing the first 32-bit number with the firmware size. this first number contained the stack location, but we have moved this stack location to the 9th position in the startup script
data = firmware_size.to_bytes(4, "little") + data[4:] + firmware_crc.to_bytes(4, "little")

print("Will write this many bytes:", len(data))

ser = servomotor.serial_functions.open_serial_port(serial_port, 230400, 0.05)

system_reset_command(ser)
time.sleep(0.1) # wait for it to reset

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
    program_one_page(ser, model_code, firmware_compatibility_code, page_number, data[0 : FLASH_PAGE_SIZE])
    time.sleep(0.1)
    data = data[FLASH_PAGE_SIZE:]
    page_number = page_number + 1

system_reset_command(ser)

time.sleep(0.1)

ser.close()
