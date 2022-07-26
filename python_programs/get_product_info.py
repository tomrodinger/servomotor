#!/usr/local/bin/python3

import sys
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import struct

#SERIAL_PORT = "/dev/tty.SLAB_USBtoUART"
SERIAL_PORT = "/dev/cu.usbserial-1410"

DETECT_DEVICES_COMMAND = 20
SET_DEVICE_ALIAS_COMMAND = 21
GET_PRODUCT_INFO_COMMAND = 22


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

ser = open_serial_port(SERIAL_PORT, 230400, 0.1)

command = bytearray([ord('Z'), GET_PRODUCT_INFO_COMMAND, 0])
print("Writing %d bytes" % (len(command)))
print_data(command)
ser.write(command)

response = ser.read(3)
if len(response) != 3:
    print("Error: didn't receive an expected three bytes")
    print("Instead, received:", response)
    exit(1)
print("Received a response: ", response)
if response[0] != 'R' and response[1] != 1:
    print("Error: the first two bytes are not as expected. Expecting R 1")
    exit(1)
payload_size = response[2]
response = ser.read(payload_size)
if len(response) != payload_size:
    print("Error: didn't receive the right length payload")
    print("Received this payload: ", response)
    exit(1)
print(response)
for c in response:
    if c >= 33 and c <= 126:
        print("Character: %1c -> %3d 0x%02x" % (c, c, c))
    else:
        print("Character:   -> %3d 0x%02x" % (c, c))

(product_code, hardware_revision_bugfix, hardware_revision_minor, hardware_revision_major,
 hardware_revision_unused, serial_number, unique_id, unused) = struct.unpack("<8sBBBBIQI", response)
print("")
print("The product information is:")
print("   Product code: [%s]" % (product_code.decode('UTF-8')))
print("   Hardware version: %u.%u.%u" % (hardware_revision_major, hardware_revision_minor, hardware_revision_bugfix))
print("   Serial number:", serial_number)
print("   Unique ID: %016X" % (unique_id))
print("")

ser.close()
