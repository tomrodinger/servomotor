#!/usr/local/bin/python3

import sys
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import argparse
import serial_functions


DETECT_DEVICES_COMMAND = 20
SET_DEVICE_ALIAS_COMMAND = 21
GET_PRODUCT_INFO_COMMAND = 22
MOVE_WITH_ACCELERATION_COMMAND = 19

DETECT_DEVICES_MAX_TIME = 1.2


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


def send_move_with_acceleration_command(ser, alias, acceleration, time_steps):
    command = bytearray([alias, MOVE_WITH_ACCELERATION_COMMAND, 8])
    command = command + acceleration.to_bytes(4, byteorder = "little", signed = True)
    command = command + time_steps.to_bytes(4, "little")
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


def print_usage():
    print("Usage: %s acceleration time" % (sys.argv[0]))
    print("The acceleration is in units of mm per second per second")
    print("The time in in units of seconds")
    print("For example, this is a sensible command: %s 100000 1000" % (sys.argv[0]))
    exit(1)


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Control the motor to make a move given the specified acceleration and time')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
parser.add_argument('acceleration', help='This is the acceleration of the move in mm per second squared')
parser.add_argument('time', help='This is the amount of time to accelerate for in seconds')
args = parser.parse_args()

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

acceleration = float(args.acceleration)
time = float(args.time)
if time < 0:
    print("The time steps cannot be negative")
    exit(1)


TIME_STEPS_PER_SECOND = 31250
mm_per_rotation = 20
microsteps_per_rotation = 360 * 256 * 7
microsteps_per_mm = microsteps_per_rotation / mm_per_rotation
max_rpm = 2000
max_rps = max_rpm / 60
max_microsteps_per_second = max_rps * microsteps_per_rotation
max_microsteps_per_time_step = max_microsteps_per_second / TIME_STEPS_PER_SECOND
max_velocity = max_microsteps_per_time_step
max_acceleration_mm_per_second_squared = 10000
max_acceleration_rotations_per_second_squared = max_acceleration_mm_per_second_squared / mm_per_rotation
max_acceleration_microsteps_per_second_squared = max_acceleration_rotations_per_second_squared * microsteps_per_rotation
max_acceleration_microsteps_per_time_step_squared = max_acceleration_microsteps_per_second_squared / (TIME_STEPS_PER_SECOND * TIME_STEPS_PER_SECOND)
max_acceleration = max_acceleration_microsteps_per_time_step_squared

print("The maximum microsteps per time step (velocity) is:", max_velocity)
print("The maximum microsteps per time step squared (max acceleration) is:", max_acceleration)

acceleration_mm_per_second_squared = acceleration
acceleration_rotations_per_second_squared = acceleration_mm_per_second_squared / mm_per_rotation
acceleration_microsteps_per_second_squared = acceleration_rotations_per_second_squared * microsteps_per_rotation
acceleration_microsteps_per_time_step_squared = acceleration_microsteps_per_second_squared / (TIME_STEPS_PER_SECOND * TIME_STEPS_PER_SECOND)

print("The microsteps per time step squared (acceleration) is:", acceleration_microsteps_per_time_step_squared)
acceleration_to_send = int(acceleration_microsteps_per_time_step_squared * (1 << 24) + 0.5)
print("The acceleration to send:", acceleration_to_send)

time_steps = int(time * TIME_STEPS_PER_SECOND + 0.5)

ser = serial_functions.open_serial_port(serial_port, 230400, 0.05)

send_move_with_acceleration_command(ser, alias, acceleration_to_send, time_steps)
payload = get_response(ser)
if (payload == None) or (len(payload) != 0):
    print("Received an invalid response")
    exit(1)
print("Command succeeded")

ser.close()
