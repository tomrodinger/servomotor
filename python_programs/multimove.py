#!/usr/bin/env python3

import sys
import argparse
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import time
import binascii
import struct
import random
import os
import serial_functions

FIRMWARE_UPGRADE_COMMAND = 23
SYSTEM_RESET_COMMAND = 27
SET_MAXIMUM_MOTOR_CURRENT_COMMAND = 28
MULTI_MOVE_COMMAND = 29
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


def set_maximum_motor_current_command(ser, max_motor_current, max_motor_regen_current):
    print("Setting the maximum motor current and the maximum motor regen current")
    command = int(255).to_bytes(1, "little") + SET_MAXIMUM_MOTOR_CURRENT_COMMAND.to_bytes(1, "little") + int(4).to_bytes(1, "little")
    command = command + int(max_motor_current).to_bytes(2, "little")
    command = command + int(max_motor_regen_current).to_bytes(2, "little")
    print("Writing %d bytes" % (len(command)))
    ser.write(command)

def send_multimove_command(ser, alias, parameters_to_send):
    n_moves = len(parameters_to_send) // 2
    parameter_length = int(1 + 4 + n_moves * 8)
    command = bytearray([alias, MULTI_MOVE_COMMAND, parameter_length])
    command = command + n_moves.to_bytes(1, byteorder = "little", signed = False)
    command = command + int(0).to_bytes(4, byteorder = "little", signed = False)
    for i in range(n_moves):
        command = command + parameters_to_send[i * 2].to_bytes(4, byteorder = "little", signed = True)
        command = command + parameters_to_send[i * 2 + 1].to_bytes(4, byteorder = "little", signed = False)
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)



# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='Reset the device(s)')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
parser.add_argument('acceleration1', help='This is the acceleration for move one')
parser.add_argument('time1', help='This is the number of time steps for move one')
parser.add_argument('acceleration2', help='This is the acceleration for move two')
parser.add_argument('time2', help='This is the number of time steps for move two')
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



acceleration1 = float(args.acceleration1)
time1 = float(args.time1)
if time1 < 0:
    print("The time time cannot be negative")
    exit(1)

acceleration2 = float(args.acceleration2)
time2 = float(args.time2)
if time2 < 0:
    print("The time time cannot be negative")
    exit(1)

acceleration = [acceleration1, acceleration2]
time_seconds = [time1, time2]
parameters_to_send = []

for i in range(len(acceleration)):
    print("Calculating the parameters for move %d" % (i+1))
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

    acceleration_mm_per_second_squared = acceleration[i]
    acceleration_rotations_per_second_squared = acceleration_mm_per_second_squared / mm_per_rotation
    acceleration_microsteps_per_second_squared = acceleration_rotations_per_second_squared * microsteps_per_rotation
    acceleration_microsteps_per_time_step_squared = acceleration_microsteps_per_second_squared / (TIME_STEPS_PER_SECOND * TIME_STEPS_PER_SECOND)

    print("The microsteps per time step squared (acceleration) is:", acceleration_microsteps_per_time_step_squared)
    acceleration_to_send = round(acceleration_microsteps_per_time_step_squared * (1 << 24))
    print("The acceleration to send:", acceleration_to_send)

    time_steps = int(time_seconds[i] * TIME_STEPS_PER_SECOND + 0.5)
    print("The time steps to send:", time_steps)

    parameters_to_send.append(acceleration_to_send)
    parameters_to_send.append(time_steps)



ser = serial_functions.open_serial_port(serial_port, 230400, 0.05)

send_multimove_command(ser, alias, parameters_to_send)

time.sleep(0.1) # wait for it to reset

ser.close()
