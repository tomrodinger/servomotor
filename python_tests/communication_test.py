#!/usr/local/bin/python3

import sys
import argparse
import motor_commands

# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='This program will continuously ping a device and check that the respons it correct')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)

args = parser.parse_args()

args.command = "PING_COMMAND"

command_id = motor_commands.get_command_id(args.command)
if command_id == None:
    print("ERROR: The command", args.command, "is not supported")
    print("Please run this program with the -c option to see all supported commands")
    exit(1)

gathered_inputs = motor_commands.gather_inputs(command_id, "hellohello")

motor_commands.set_standard_options_from_args(args) # This will find out the port to use and the alias of the device and store those in the motor_commands module
motor_commands.open_serial_port()
response = motor_commands.send_command(command_id, gathered_inputs)
motor_commands.interpret_response(command_id, response)
motor_commands.close_serial_port()
