#!/usr/bin/env python3

import sys
import argparse
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import time
import serial_functions


DETECT_DEVICES_COMMAND = 20
SET_DEVICE_ALIAS_COMMAND = 21
GET_PRODUCT_INFO_COMMAND = 22

DETECT_DEVICES_MAX_TIME = 1.2


def send_set_device_alias_command(ser, unique_id, alias):
    command = bytearray([255, SET_DEVICE_ALIAS_COMMAND, 9])
    command = command + unique_id.to_bytes(8, "little")
    command = command + alias.to_bytes(1, "little")
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)


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


def parse_response(response):
    if len(response) != 13:
        return None, None
    unique_id = int.from_bytes(response[0:8], "little")
    alias = int(response[4])
    crc32 = int.from_bytes(response[5:9], "little")
    if crc32 != 0x04030201:
        return None, None
    return unique_id, alias


def print_data(data):
    print(data)
    for d in data:
        print("0x%02X %d" % (d, d))

# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Set the alias of the device using its unique ID to identify it')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('unique_id', help='This is the unique ID of the device and it can be found with the detect devices command')
parser.add_argument('alias', help='This is the alias to set in this device')
args = parser.parse_args()

serial_port = args.port
unique_id = int(args.unique_id, 16)
alias = args.alias
if alias == "255":
    alias = 255
elif len(alias) == 1:
    alias = ord(alias)
else:
    print("Error: the alias nust be just one character, not:", alias)
    print("The alias can also be 255 (which means no alias)")
    exit(1)

print("We will attempt to change the alias of the device with unique id %016X to %c" % (unique_id, alias))

ser = serial_functions.open_serial_port(serial_port, 230400, 0.05)

send_set_device_alias_command(ser, unique_id, alias)
payload = get_response(ser)
if (payload == None) or (len(payload) != 0):
    print("Received an invalid response")
    exit(1)
print("Command succeeded")
ser.close()
