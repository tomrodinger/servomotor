#!/usr/bin/env python3

import math
import os
import time
import serial
import sys
import argparse
import matplotlib.pyplot as plt

SERIAL_PORT_DEVICE = "/dev/tty.usbserial-0001"
OUT_BIN_FILENAME = "hall_calibration_raw_data.bin"
OUT_TEXT_FILENAME = "hall_calibration_one_rotation.txt"
CAPTURE_HALL_SENSOR_DATA_COMMAND = 7
CAPTURE_HALL_SENSOR_READINGS = 1
CAPTURE_HALL_SENSOR_READINGS_WHILE_TURNING = 4
STOP_CAPTURE_HALL_SENSOR_READINGS = 0
HALL_SENSOR_CAPTURE_LENGTH_MULTIPLIER = 1000 # make sure that this value is the samee as in the C program for the capture board 
HALL_SENSOR_CAPTURE_LENGTH = 6
OVERHEAD_SIZE = 3
BYTES_PER_HALL_SENSOR_READING = OVERHEAD_SIZE + 6
TOTAL_CAPTURE_BYTES = HALL_SENSOR_CAPTURE_LENGTH * HALL_SENSOR_CAPTURE_LENGTH_MULTIPLIER * BYTES_PER_HALL_SENSOR_READING

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
arg_parser = argparse.ArgumentParser(description="Magnetic Ring Test Jig Program")
arg_parser.add_argument('--no-graph', action='store_true', help="Disable the plotting of the graph")
args = arg_parser.parse_args()

MOTOR_AXIS = "M"
HALL_SENSOR_AXIS = "H"

ser = serial.Serial(SERIAL_PORT_DEVICE, 230400, timeout = 0.1)

print("Resetting all devices")
ser.write(bytearray([0xFF, 0x1B, 0x00]))
time.sleep(2.0)

print("Flushing the serial port")
some_data = ser.read(1000000)

print("Enabling the steper motor driver")
ser.write(bytearray([ord(MOTOR_AXIS), 0x01, 0x00]))
response = ser.read(3)
print("Response:", response)

print("Commanding the motor to rotate a bit more than 1 rotation")
ser.write(bytearray([ord(MOTOR_AXIS), 0x02, 0x08, 0x00, 0x50, 0x27, 0x01, 0x36, 0x6E, 0x01, 0x00]))

response = ser.read(3)
print("Response:", response)

print("Sending the command to start the capture for hall sensor data")
command = bytearray([ord(HALL_SENSOR_AXIS), CAPTURE_HALL_SENSOR_DATA_COMMAND, 1, HALL_SENSOR_CAPTURE_LENGTH])
ser.write(command)

response = ser.read(3)
print("Response:", response)

data = bytearray()
while 1:
    some_data = ser.read(TOTAL_CAPTURE_BYTES)
    print("Received", len(some_data), "bytes")
    data = data + some_data
    if len(data) >= TOTAL_CAPTURE_BYTES:
        break

print(f"Received a total of {len(data)} bytes")

if len(data) % BYTES_PER_HALL_SENSOR_READING != 0:
    print(f"Error: The length of the data is not a multiple of {BYTES_PER_HALL_SENSOR_READING}")
    print("The length of the data is:", len(data))
    exit(1)

# Now, let's remove the overhead from the data. The overhead is OVERHEAD_SIZE bytes from the beginning of each BYTE_PER_HALL_SENSOR_READING block
# loop through all the data and grab BYTES_PER_HALL_SENSOR_READING bytes at a time
# and remove the first OVERHEAD_SIZE bytes from each block and save it in a new array
new_data = bytearray()
for i in range(0, len(data), BYTES_PER_HALL_SENSOR_READING):
    new_data = new_data + data[i + OVERHEAD_SIZE: i + BYTES_PER_HALL_SENSOR_READING]
data = new_data

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
    fh.write(HALL_SENSOR_AXIS)

# Reading the generated text file and preparing the data
if not args.no_graph:
    file_data = []

    with open(OUT_TEXT_FILENAME, 'r') as fh:
        for line in fh:
            values = list(map(int, line.split()))
            file_data.append(values)

    plot_data(file_data)
