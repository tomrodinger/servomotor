#!/usr/bin/env python3

import sys
import motor_commands
import communication
import time
import os
import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

VERBOSE = 2

ALL_ALIASES = 255

def write_data(filename, int32_list):
    with open(filename, "w") as fh:
        for i in range(len(int32_list)):
            fh.write(str(i) + " " + str(int32_list[i]) + "\n")


def execute_command(alias, command_str, inputs, verbose=2):
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

communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

communication.serial_port = "/dev/tty.usbserial-140"
communication.open_serial_port()

while 1:
    # let's reset all devices to start from a clean state
    execute_command(ALL_ALIASES, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

    # give time for the devices to reset and boot up
    time.sleep(1.5)

    # now lets turn on all the LEDs using an LED test mode
    parsed_response = execute_command(ALL_ALIASES, "TEST_MODE_COMMAND", [12], verbose=VERBOSE)
    if len(parsed_response) != 0:
        print("ERROR: The device with alias", ALL_ALIASES, "did not respond correctly to the GO_TO_CLOSED_LOOP_COMMAND")
        exit(1)

time.sleep(0.3)
