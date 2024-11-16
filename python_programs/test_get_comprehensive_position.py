#!/usr/bin/env python3

import argparse
import time
import servomotor


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Continually get the comprehensive position and print it to the screen')
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
args = parser.parse_args()

alias = args.alias
if alias == None:
    print("Error: you must specify an alias with the -a option. Run this command with -h to get more detailed help.")
    exit(1)
elif alias == "255" or alias == "254":
    print("Error: invalid alias. The alias cannot be 255 or 254.")
    exit(1)
elif len(alias) > 1:
    print("Error: the alias nust be just one character, not:", alias)
    exit(1)

alias = ord(alias)

servomotor.set_serial_port_from_args(args)

motorX = servomotor.M3(alias, motor_type="M3", time_unit="seconds", position_unit="degrees", velocity_unit="degrees/s", acceleration_unit="degree/s^2", current_unit="mA", voltage_unit="V", temperature_unit="C", verbose=args.verbose)
servomotor.open_serial_port()

while 1:
    response = motorX.get_comprehensive_position()
    print(response)
    time.sleep(0.05)