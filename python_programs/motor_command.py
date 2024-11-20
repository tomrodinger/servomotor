#!/usr/bin/env python3

import sys
import argparse
import servomotor
from terminal_formatting import format_error, format_info, format_warning, format_success, format_debug

# Define the arguments for this program
parser = argparse.ArgumentParser(description='This program will let you send any supported command to the motor')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
parser.add_argument('-c', '--commands', help='list all supported commands with detailed descriptions', action="store_true")
parser.add_argument('-v', '--verbose', help='equivalent to --verbose-level 2', action="store_true")
parser.add_argument('--verbose-level', type=int, choices=[0, 1, 2], help='Set verbosity level (0: no output, 1: minimal output, 2: detailed output)', default=None)
parser.add_argument('command',
                    help = 'This is the command to be send to the motor. For example ENABLE_MOSFETS_COMMAND. Run this program with the -c option to see all supported commands.',
                    nargs='?',
                    default=None)
parser.add_argument('inputs',
                    help = 'These are the inputs for the command. The number of inputs depends on the command. Some commands have no inputs.',
                    nargs='*',
                    default=None)

args = parser.parse_args()

if args.commands == True:
    print(format_info("=== Protocol and Command Information ==="))
    servomotor.print_protocol_version()
    servomotor.print_data_type_descriptions()
    servomotor.print_registered_commands()
    exit(0)

if args.command == None:
    print(format_error("You didn't specify the command to run."))
    print(format_info("Please run this program with the -c option to see all supported commands or run this program with the -h option to see the usage information"))
    exit(1)

command_id = servomotor.get_command_id(args.command)
if command_id == None:
    print(format_error(f"The command '{args.command}' is not supported"))
    print(format_info("Please run this program with the -c option to see all supported commands"))
    exit(1)

# Determine verbosity level
if args.verbose_level is not None:
    verbose_level = args.verbose_level
elif args.verbose:
    verbose_level = 2
else:
    verbose_level = 1

if verbose_level >= 1:
    print(format_info(f"Command ID: {command_id}"))

gathered_inputs = servomotor.gather_inputs(command_id, args.inputs, verbose=verbose_level)

servomotor.set_standard_options_from_args(args) # This will find out the port to use and the alias of the device and store those in the communication module
servomotor.open_serial_port()
try:
    response = servomotor.send_command(command_id, gathered_inputs, verbose=verbose_level)
except servomotor.communication.TimeoutError:
    print(format_error("Timeout Error: The device did not respond in time"))
    print(format_warning(
        "This may be that your device is not connected and powered on, "
        "or that the serial port is not correct, "
        "or that the alias is not correct for your target device."
    ))
    servomotor.close_serial_port()
    exit(1)

try:
    result = servomotor.interpret_response(command_id, response, verbose=verbose_level)
    if result:  # If the command was successful
        print(format_success("Command executed successfully"))
except Exception as e:
    print(format_error(f"Error interpreting response: {str(e)}"))
finally:
    servomotor.close_serial_port()
