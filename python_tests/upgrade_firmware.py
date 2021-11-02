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


SERIAL_PORT = "/dev/cu.SLAB_USBtoUART"
#SERIAL_PORT = "/dev/tty.usbserial-1420"
FIRMWARE_UPGRADE_COMMAND = 23
SYSTEM_RESET_COMMAND = 27
FLASH_BASE_ADDRESS = 0x8000000
FLASH_PAGE_SIZE = 2048
BOOTLOADER_N_PAGES = 5    # 10kB bootloader
FIRST_FIRMWARE_PAGE_NUMBER = (BOOTLOADER_N_PAGES)
LAST_FIRMWARE_PAGE_NUMBER = 30
FLASH_SETTINGS_PAGE_NUMBER = 31


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


def system_reset_command(ser):
    print("Resettting the newly programmed device...")
    command = int(255).to_bytes(1, "little") + SYSTEM_RESET_COMMAND.to_bytes(1, "little") + int(0).to_bytes(1, "little")
    print("Writing %d bytes" % (len(command)))
    ser.write(command)



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

for i in range(20):
    print("data item [%2d]: 0x%08X %10u" % (i, data_uint32[i], data_uint32[i]))

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
    program_one_page(ser, page_number, data[0 : FLASH_PAGE_SIZE])
    time.sleep(0.1)
    data = data[FLASH_PAGE_SIZE:]
    page_number = page_number + 1

system_reset_command(ser)

time.sleep(0.1)

ser.close()
