#!/usr/local/bin/python3

import sys
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import argparse
import serial_functions


#SERIAL_PORT = "/dev/tty.SLAB_USBtoUART"
SERIAL_PORT = "/dev/tty.usbserial-142220"

DETECT_DEVICES_COMMAND = 20
SET_DEVICE_ALIAS_COMMAND = 21
GET_PRODUCT_INFO_COMMAND = 22
MOVE_WITH_ACCELERATION_COMMAND = 19
MOVE_WITH_VELOCITY_COMMAND = 26

DETECT_DEVICES_MAX_TIME = 1.2


def send_move_with_acceleration_command(ser, acceleration, time_steps):
    command = bytearray([ord('X'), MOVE_WITH_ACCELERATION_COMMAND, 8])
    command = command + acceleration.to_bytes(4, byteorder = "little", signed = True)
    command = command + time_steps.to_bytes(4, "little")
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)


def send_move_with_velocity_command(ser, velocity, time_steps):
    command = bytearray([ord('X'), MOVE_WITH_VELOCITY_COMMAND, 8])
    command = command + velocity.to_bytes(4, byteorder = "little", signed = True)
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


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Control the motor to make a move given the specified velocity and time')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
parser.add_argument('velocity', help='This is the velociuty of the move in mm per second')
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

mm_per_second = float(args.velocity)
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

rps = mm_per_second / mm_per_rotation
microsteps_per_second = rps * microsteps_per_rotation
microsteps_per_time_step = microsteps_per_second / TIME_STEPS_PER_SECOND

print("The microsteps per time step (velocity) is:", microsteps_per_time_step)
velocity_to_send = int(microsteps_per_time_step * (1 << 20) + 0.5)
print("The velocity to send:", velocity_to_send)

time_steps = int(time * TIME_STEPS_PER_SECOND + 0.5)
print("The timesteps to send:", time_steps)

ser = serial_functions.open_serial_port(serial_port, 230400, 0.05)

send_move_with_velocity_command(ser, velocity_to_send, time_steps)
payload = get_response(ser)
if (payload == None) or (len(payload) != 0):
    print("Received an invalid response")
    exit(1)
print("Command succeeded")

ser.close()
