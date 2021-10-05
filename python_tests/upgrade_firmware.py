#!/usr/local/bin/python3

import sys
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import time
import binascii
import struct
import random
import os

PRODUCT_CODE = "M1      " # This should be 8 characters

HARDWARE_REVISION_MAJOR = 0
HARDWARE_REVISION_MINOR = 8
HARDWARE_REVISION_BUGFIX = 0
HARDWARE_REVISION = ((HARDWARE_REVISION_MAJOR << 16) | (HARDWARE_REVISION_MINOR << 8) | HARDWARE_REVISION_BUGFIX)

SERIAL_NUMBER_FILENAME = os.path.expanduser("~") + "/.motor_serial_number"

#SERIAL_PORT = "/dev/tty.SLAB_USBtoUART"
SERIAL_PORT = "/dev/tty.usbserial-1420"
FIRMWARE_UPGRADE_COMMAND = 23
FLASH_BASE_ADDRESS = 0x8000000
FLASH_PAGE_SIZE = 2048
BOOTLOADER_N_PAGES = 5    # 10kB bootloader
FIRST_FIRMWARE_PAGE_NUMBER = (BOOTLOADER_N_PAGES)
LAST_FIRMWARE_PAGE_NUMBER = 30
FLASH_SETTINGS_PAGE_NUMBER = 31

# The avaialble positions in this particular chip are from 4 to 10
PRODUCT_CODE_POSITION = 4 # the product code is 8 bytes, so it takes two positions in the 32-bit data representation
HARDWARE_REVISION_POSITION = 6
SERIAL_NUMBER_POSITION = 7
UNIQUE_ID_POSITION = 8 # this takes two positions

def open_serial_port(serial_device, baud_rate, timeout = 0.05):
    print("Opening serial device:", serial_device)
    try:
        ser = serial.Serial(serial_device, baud_rate, timeout = timeout)
    except:
        print("Could not open the serial port")
        print("Most likely, this is because the hardware is not connected properly")
        print("So, make sure you plugged in your USB to serial adapeter")
        print("Otherwise, make sure thet the correct serial port is defined in this program")
        print("Here are the current serial ports detected on your computer:")
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print(p[0])
        exit(1)
    print("Opened:", ser.name)
    return ser


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
    print("Firmware has size:", len(data))
    return data


def get_response(ser):
    response = ser.read(3)
    if len(response) != 3:
        print("Error: didn't receive enough bytes in the response")
        exit(1)
    print("Received a response: ", response)
    if response[0] != ord('R'):
        print("Error: the first is not the expected R")
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


def program_one_page(ser, page_number, data):
    assert len(data) == FLASH_PAGE_SIZE
    print("Writing to page:", page_number)
    command = int(255).to_bytes(1, "little") + FIRMWARE_UPGRADE_COMMAND.to_bytes(1, "little") + int(255).to_bytes(1, "little") + (FLASH_PAGE_SIZE + 1).to_bytes(2, "little") + int(page_number).to_bytes(1, "little")
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


def read_serial_number():
    print("Reading the serial number from the file:", SERIAL_NUMBER_FILENAME)
    try:
        fh = open(SERIAL_NUMBER_FILENAME, "r")
    except:
        print("Error: could not open the file containing the serial number:", SERIAL_NUMBER_FILENAME)
        print("Make sure that the file is there and contains an integer in ascii text format")
        exit(1)
    try:
        serial_number_data = fh.read()
    except:
        print("Error: could not read the data in the serial number file")
        exit(1)
    fh.close()
    try:
        serial_number = int(serial_number_data.strip())
    except:
        print("Error: the file containg the serial number does not contain a valid integer in ascii text format")
        exit(1)
    if serial_number < 0 or serial_number > (1 << 32) - 1:
        print("Error: the serial number contained in the file must fit into a 32-bit unsigned number")
        exit(1)
    return serial_number


def increment_serial_number(serial_number):
    serial_number = serial_number + 1
    if serial_number > (1 << 32) - 1:
        print("Error: exceeded the number of possible serial numbers that can be stored in the device")
        print("The serial number reached:", serial_number)
    return serial_number


def write_serial_number(serial_number):
    print("Saving the serial number %d to the file: %s" % (serial_number, SERIAL_NUMBER_FILENAME))
    try:
        fh = open(SERIAL_NUMBER_FILENAME, "w")
    except:
        print("Error: could not open the file for writing the serial number:", SERIAL_NUMBER_FILENAME)
        print("Make sure that the locaiton is writable and that the filename is reasonable")
        exit(1)
    serial_number_text = str(serial_number)
    try:
        fh.write(serial_number_text)
    except:
        print("Error: could not write the serial number to the file")
        print("Make sure that it is a file that can be written. ie. the disk is not full and there are write permissions")
        exit(1)
    fh.close()



if len(sys.argv) != 2:
    print_usage()

firmware_filename = sys.argv[1]

data = read_binary(firmware_filename)
# pad zeros until the length of the data is divisable by 4
while len(data) & 0x03 != 0:
    data.append(0)

print(len(data))

data_uint32 = []
for item in struct.iter_unpack('<I', data):  # unpack as little endian unsigned 32-bit integers
    data_uint32.append(item[0])
print(len(data_uint32))

serial_number = read_serial_number()
serial_number = increment_serial_number(serial_number)

assert len(PRODUCT_CODE) == 8
(product_code_L, product_code_H) = struct.unpack('<II', bytes(PRODUCT_CODE, encoding='utf8'))
data_uint32[PRODUCT_CODE_POSITION] = product_code_L
data_uint32[PRODUCT_CODE_POSITION + 1] = product_code_H
data_uint32[HARDWARE_REVISION_POSITION] = HARDWARE_REVISION
data_uint32[SERIAL_NUMBER_POSITION] = serial_number
unique_id_L = random.randint(0, (1 << 32) - 1)
unique_id_H = random.randint(0, (1 << 32) - 1)
data_uint32[UNIQUE_ID_POSITION] = unique_id_L
data_uint32[UNIQUE_ID_POSITION + 1] = unique_id_H

for i in range(20):
    print("data item [%2d]: 0x%08X %10u" % (i, data_uint32[i], data_uint32[i]))


write_serial_number(serial_number)

exit(1)



# we are finished manipulating, so now repack it back into bytes
data2 = b''
for item in data_uint32:
    data2 = data2 + struct.pack('<I', item)

print(len(data2))

firmware_size = (len(data) >> 2) - 1
firmware_crc = binascii.crc32(data[4:])
print("Firmware size is %u 32-bit values. Firmware CRC32 is 0x%08X." % (firmware_size, firmware_crc))

# replacing the first 32-bit number with the firmware size. this first number contained the stack location, but we have moved this stack location to the 9th position in the startup script
data = firmware_size.to_bytes(4, "little") + data[4:] + firmware_crc.to_bytes(4, "little")

print("Will write this many bytes:", len(data))

ser = open_serial_port(SERIAL_PORT, 230400, 0.05)

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
    program_one_page(ser, page_number, data[0 : FLASH_PAGE_SIZE])
    time.sleep(0.1)
    data = data[FLASH_PAGE_SIZE:]
    page_number = page_number + 1


ser.close()
