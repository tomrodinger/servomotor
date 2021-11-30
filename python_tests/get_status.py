#!/usr/local/bin/python3

import serial
import argparse
import serial_functions

# Define the arguments for this program and then check some conditions after
parser = argparse.ArgumentParser(description='Get the status of the device')
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



ser.write(bytearray([alias, 16, 0]))


data = ser.read(1000)
print("Received %d bytes" % (len(data)))
print(data)

for d in data:
    print("0x%02X %d" % (d, d))

ser.close()
