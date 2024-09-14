#!/usr/bin/env python3

import math
import os
import time
import serial
import sys
import argparse
import matplotlib.pyplot as plt

SERIAL_PORT_DEVICE = "/dev/tty.usbserial-1120"
OUT_BIN_FILENAME = "hall_calibration_raw_data.bin"
OUT_TEXT_FILENAME = "hall_calibration_one_rotation.txt"
CALIBRATION_START_TEXT = bytearray(b'Calibration start\n')
CALIBRATION_DONE_TEXT = bytearray(b'Calibration capture done\n')
CAPTURE_HALL_SENSOR_DATA_COMMAND = 7
CAPTURE_HALL_SENSOR_READINGS = 1
CAPTURE_HALL_SENSOR_READINGS_WHILE_TURNING = 4


def plot_data(data):
    col1, col2, col3 = zip(*data)

    X = list(range(len(data)))

    fig, axs = plt.subplots(4, 1, figsize=(8, 12))

    axs[0].plot(X, col1)
    axs[0].set_title('Column 1')
    axs[0].set_ylabel('Y-axis value')

    axs[1].plot(X, col2)
    axs[1].set_title('Column 2')
    axs[1].set_ylabel('Y-axis value')

    axs[2].plot(X, col3)
    axs[2].set_title('Column 3')
    axs[2].set_ylabel('Y-axis value')

    axs[3].plot(X, col1, label="Column 1")
    axs[3].plot(X, col2, label="Column 2")
    axs[3].plot(X, col3, label="Column 3")
    axs[3].set_title('Hall Calibration Data')
    axs[3].set_xlabel('Row number')
    axs[3].set_ylabel('Y-axis value')

    plt.tight_layout()
    plt.show()

# Adding the argparse logic
arg_parser = argparse.ArgumentParser(description="Hall Calibration Data Plotter")
arg_parser.add_argument('--no-graph', action='store_true', help="Disable the plotting of the graph")
arg_parser.add_argument('axis', metavar='AXIS', help='Axis to calibrate (for example: X, Y, Z, E, 0, 1, etc.)')
args = arg_parser.parse_args()

MY_AXIS = args.axis

ser = serial.Serial(SERIAL_PORT_DEVICE, 230400, timeout = 0.1)

print("Sending the command to start the calibration motor movements")
command = bytearray([ord(MY_AXIS), CAPTURE_HALL_SENSOR_DATA_COMMAND, 1, CAPTURE_HALL_SENSOR_READINGS_WHILE_TURNING])
ser.write(command)

data = bytearray()
while 1:
    some_data = ser.read(1000000)
    data = data + some_data
    end_index = data.find(CALIBRATION_DONE_TEXT, 0)
    if end_index >= 0:
        break

start_index = data.find(CALIBRATION_START_TEXT, 0)
if start_index < 0:
    print("Did not find the calibration start text:", CALIBRATION_START_TEXT)
    exit(1)

print("Found the calibration done text at index:", end_index)
data = data[start_index + len(CALIBRATION_START_TEXT) : end_index]

print("Sending the command to disable the motor")
command = bytearray([ord(MY_AXIS), 0, 0])
ser.write(command)

if len(data) % 6 != 0:
    print("Error: The length of the data is not a multiple of 6")
    print("The length of the data is:", len(data))
    exit(1)

with open(OUT_BIN_FILENAME, "wb") as fh:
    fh.write(data)

i = 0
with open(OUT_TEXT_FILENAME, "w") as fh:
    while(i < len(data)):
        hall1 = int.from_bytes(data[i + 0: i + 2], byteorder = "little", signed=False)
        hall2 = int.from_bytes(data[i + 2: i + 4], byteorder = "little", signed=False)
        hall3 = int.from_bytes(data[i + 4: i + 6], byteorder = "little", signed=False)
        if (hall1 != 0) or (hall2 != 0) or (hall3 != 0):
            s = "%5d %5d %5d\n" % (hall1, hall2, hall3)
            fh.write(s)
            print(s)
        i = i + 6
    
ser.close()

with open("MY_AXIS", "w") as fh:
    fh.write(MY_AXIS)

# Reading the generated text file and preparing the data
if not args.no_graph:
    file_data = []

    with open(OUT_TEXT_FILENAME, 'r') as fh:
        for line in fh:
            values = list(map(int, line.split()))
            file_data.append(values)

    plot_data(file_data)
