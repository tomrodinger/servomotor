#!/usr/local/bin/python3

import sys
import argparse
import motor_commands

# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='This program will let you send any supported command to the motor')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
parser.add_argument('-c', '--commands', help='list all supported commands with detailed descriptions', action="store_true")
parser.add_argument('command',
                    help = 'This is the command to be send to the motor. For example ENABLE_MOSFETS_COMMAND. Run this program with the -c option to see all supported commands.',
                    nargs='?',
                    default=None)
# Add a variable number of parameters to specify all the required inputs (if any)
#parser.add_argument('parameters', nargs='*', help='parameters for the command')
parser.add_argument('inputs',
                    help = 'These are the inputs for the command. The number of inputs depends on the command. Some commands have no inputs.',
                    nargs='*',
                    default=None)

args = parser.parse_args()

if args.commands == True:
    motor_commands.print_registered_commands()
    exit(0)

if args.command == None:
    print("ERROR: You didn't specify the command to run.")
    print("Please run this program with the -c option to see all supported commands or run this program with the -h option to see the usage information")
    exit(1)

print("The command is:", args.command)

command_id = motor_commands.get_command_id(args.command)
if command_id == None:
    print("ERROR: The command", args.command, "is not supported")
    print("Please run this program with the -c option to see all supported commands")
    exit(1)

gathered_inputs = motor_commands.gather_inputs(command_id, args.inputs)

motor_commands.set_standard_options_from_args(args) # This will find out the port to use and the alias of the device and store those in the motor_commands module
motor_commands.open_serial_port()
response = motor_commands.send_command(command_id, gathered_inputs)
motor_commands.interpret_response(command_id, response)
motor_commands.close_serial_port()
