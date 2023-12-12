#!/usr/bin/env python3

import sys
import argparse
import motor_commands
import communication
import time

VERBOSE = False
PRINT_STATISTICS_INTERVAL = 60

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
parser = argparse.ArgumentParser(description='This program will syncronize time across multiple devices')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-R', '--reset-device', help='reset the device before starting the time sync', action="store_true")
# And now, accept a variable number of aliases (but do not accept zero aliases)
parser.add_argument('alias', metavar='ALIAS', help='aliases of the devices to control (list as many as needed)', nargs='+')
args = parser.parse_args()

# Print out all the aliases that were specified
print("The specified aliases are:", args.alias)

# Convert the aliases to integers
alias_list = []
for alias in args.alias:
    alias_int = communication.string_to_u8_alias(alias)
    alias_list.append(alias_int)
print("The specified aliases as integers are:", alias_list)
args.alias = None

communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)
communication.set_serial_port_from_args(args) # This will find out the port to use and store it in the communication module
communication.open_serial_port()

# If the user wanted to reset all the devices first, then do it here
if args.reset_device:
    for alias in alias_list:
        execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
time.sleep(1.5)

# Send a RESET_TIME_COMMAND
master_start_time = time.time()
execute_command(255, "RESET_TIME_COMMAND", [], verbose=VERBOSE)

min_error_stats = {}
max_error_stats = {}
for alias in alias_list:
    min_error_stats[alias] = 0
    max_error_stats[alias] = 0
next_statistics_print_time = time.time() + PRINT_STATISTICS_INTERVAL
while 1:
    for alias in alias_list:
        current_time = time.time() - master_start_time
        current_time_us = int(current_time * 1000000)
        response = execute_command(alias, "TIME_SYNC_COMMAND", [current_time_us], verbose=VERBOSE)
        time_error_us = response[0]
        print(f"The time error for alias {alias} is {time_error_us} microseconds")
        if time_error_us < min_error_stats[alias]:
            min_error_stats[alias] = time_error_us
        if time_error_us > max_error_stats[alias]:
            max_error_stats[alias] = time_error_us
    if time.time() >= next_statistics_print_time:
        print("=============================================================================================================================")
        print("Statistics:")
        for alias in alias_list:
            print(f"  Alias {alias}: min error = {min_error_stats[alias]} microseconds, max error = {max_error_stats[alias]} microseconds")
        print("=============================================================================================================================")
        for alias in alias_list:
            min_error_stats[alias] = 0
            max_error_stats[alias] = 0
        next_statistics_print_time = time.time() + PRINT_STATISTICS_INTERVAL
    time.sleep(0.1)

communication.close_serial_port()
