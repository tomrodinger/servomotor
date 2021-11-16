#!/usr/local/bin/python3

import argparse
import serial
import time
import serial_functions

# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Initiate a calibration procedure')
parser.add_argument('-p', '--port', help='serial port device', default=None)
args = parser.parse_args()

serial_port = args.port

ser = serial_functions.open_serial_port(serial_port, 230400, 0.05)


MY_AXIS = 'X'
CALIBRATION_DONE_TEXT = bytearray(b'Calibration done\n')



print("Sending the command to enable the motor")
command = bytearray([ord(MY_AXIS), 1, 0])
ser.write(command)
time.sleep(0.1)

print("Sending the command to start the calibration motor movements")
command = bytearray([ord(MY_AXIS), 6, 0])
ser.write(command)

data = bytearray()
while 1:
    some_data = ser.read(1000000)
    data = data + some_data
    end_index = data.find(CALIBRATION_DONE_TEXT, 0)
    if end_index >= 0:
        break

ser.close()
