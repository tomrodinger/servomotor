#!/usr/bin/env python3

import sys
import redirect_motor_commands as motor_commands
import redirect_communication as communication
import time
import os
import random
import math

VERBOSE = True

ALL_ALIASES = 255

communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

communication.serial_port = "/dev/tty.usbserial-140"
communication.open_serial_port()

# let's reset all devices to start from a clean state
communication.execute_command(communication.ALL_ALIASES, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

# give time for the devices to reset and boot up
time.sleep(1.5)

# now lets turn on all the LEDs using an LED test mode
parsed_response = communication.execute_command(ALL_ALIASES, "TEST_MODE_COMMAND", [12], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", communication.ALL_ALIASES, "did not respond correctly to the GO_TO_CLOSED_LOOP_COMMAND")
    exit(1)

time.sleep(0.3)


# User verification loop
print("This test program needs human verification. Please follow the instructions below.")
while True:
    user_input = input("Are both the green and red LEDs solidly lit? (y/n): ").strip().lower()
    if user_input == 'y':
        print("*** PASSED ***")
        break
    elif user_input == 'n':
        print("*** FAILED ***")
        break
    else:
        print("Invalid input. Please enter 'y' or 'n' and press enter")
print("You will need to power cycle the motor now or press the physical reset button on the motor in order to run more tests or commands.")
