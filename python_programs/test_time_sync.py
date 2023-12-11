#!/usr/bin/env python3

import sys
import argparse
import motor_commands
import communication
import time

VERBOSE = True

def execute_command(alias, command_str, inputs, verbose=True):
    communication.alias = alias
    command_id = communication.get_command_id(command_str)
    if command_id == None:
        print("ERROR: The command", command_str, "is not supported")
        exit(1)
    if verbose:
        print("The command is: %s and it has ID %d" % (command_str, command_id))
    gathered_inputs = communication.gather_inputs(command_id, inputs, verbose=verbose)
    response = communication.send_command(command_id, gathered_inputs, verbose=verbose)
    parsed_response = communication.interpret_response(command_id, response, verbose=verbose)
    return parsed_response

# Define the arguments for this program
parser = argparse.ArgumentParser(description='This program will let you send any supported command to the motor')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
args = parser.parse_args()

communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)
communication.set_standard_options_from_args(args) # This will find out the port to use and the alias of the device and store those in the communication module
communication.open_serial_port()

# Send a RESET_TIME_COMMAND
master_start_time = time.time()
execute_command(ord(args.alias), "RESET_TIME_COMMAND", [], verbose=VERBOSE)

while 1:
    current_time = time.time() - master_start_time
    current_time_us = int(current_time * 1000000)
    execute_command(ord(args.alias), "TIME_SYNC_COMMAND", [current_time_us], verbose=VERBOSE)
    time.sleep(0.1)

communication.close_serial_port()
