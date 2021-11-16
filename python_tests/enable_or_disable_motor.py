#!/usr/local/bin/python3

import serial
import argparse
import serial_functions



# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Control the motor to make a move given the specified acceleration and time')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
parser.add_argument('disable_or_enable', help='A flag 0 or 1 to disable or enable the motor MOSFETs, respectively')
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

disable_or_enable = args.disable_or_enable

ser = serial_functions.open_serial_port(serial_port, 230400, 0.05)

if disable_or_enable == "0":
    ser.write(bytearray([alias, 0, 0]))
elif disable_or_enable == "1":
    ser.write(bytearray([alias, 1, 0]))
else:
    print("Error: you can only specify 0 or 1 to disable or enable the motor")

data = ser.read(1000)
print("Received %d bytes" % (len(data)))
print(data)

for d in data:
    print("0x%02X %d" % (d, d))

ser.close()
