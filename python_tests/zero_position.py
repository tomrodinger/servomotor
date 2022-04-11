#!/usr/local/bin/python3

import sys
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import time
import argparse
import serial_functions

TRAPEZOID_MOVE_COMMAND = 2
ZERO_POSITION_COMMAND = 13
HOMING_COMMAND = 14
DETECT_DEVICES_COMMAND = 20
SET_DEVICE_ALIAS_COMMAND = 21
GET_PRODUCT_INFO_COMMAND = 22
MOVE_WITH_ACCELERATION_COMMAND = 19
MOVE_WITH_VELOCITY_COMMAND = 26

DETECT_DEVICES_MAX_TIME = 1.2

def send_zero_position_command(ser):
    command = bytearray([255, ZERO_POSITION_COMMAND, 0])
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)

def send_move_with_acceleration_command(ser, acceleration, time_steps):
    command = bytearray([255, MOVE_WITH_ACCELERATION_COMMAND, 8])
    command = command + acceleration.to_bytes(4, byteorder = "little", signed = True)
    command = command + time_steps.to_bytes(4, "little", signed = False)
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)

def send_move_with_velocity_command(ser, velocity, time_steps):
    command = bytearray([255, MOVE_WITH_VELOCITY_COMMAND, 8])
    command = command + velocity.to_bytes(4, byteorder = "little", signed = True)
    command = command + time_steps.to_bytes(4, "little", signed = False)
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)

def send_trapezoid_move_command(ser, displacement, time_steps):
    command = bytearray([255, TRAPEZOID_MOVE_COMMAND, 8])
    command = command + displacement.to_bytes(4, byteorder = "little", signed = True)
    command = command + time_steps.to_bytes(4, "little", signed = False)
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)

def send_homing_command(ser, max_displacement, max_time_steps):
    command = bytearray([255, HOMING_COMMAND, 8])
    command = command + max_displacement.to_bytes(4, byteorder = "little", signed = True)
    command = command + max_time_steps.to_bytes(4, "little", signed = False)
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
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='Set the position of the motor to zero')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
args = parser.parse_args()

if args.PORT == True:
    serial_port = "MENU"
else:
    serial_port = args.port

alias = args.alias
if alias == None:
    print("Error: you must specify an alias with the -a option. Run this command with -h to get more detailed help.")
    exit(1)
elif alias == "255":
    alias = 255
elif len(alias) == 1:
    alias = ord(alias)
else:
    print("Error: the alias nust be just one character, not:", alias)
    print("The alias can also be 255 (which means no alias)")
    exit(1)

ser = serial_functions.open_serial_port(serial_port, 230400, 0.05)


send_zero_position_command(ser)

payload = get_response(ser)
if (payload == None) or (len(payload) != 0):
    print("Received an invalid response")
    exit(1)
print("Command succeeded")

ser.close()
