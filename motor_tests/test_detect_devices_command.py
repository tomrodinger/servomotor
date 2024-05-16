#!/usr/bin/env python3

import redirect_motor_commands as motor_commands
import redirect_communication as communication
import time
import common_functions

test_description = '''
Here we will test the DETECT_DEVICES_COMMAND. At least one device needs to be connected but it is fine if more than
one is connected. This test will reset all the devices and then detect them. It will pass if at least one device is
detected.
'''

VERBOSE = False

def check_parsed_response(parsed_response):
    if isinstance(parsed_response, list):
        if all(isinstance(i, int) for i in parsed_response) and len(parsed_response) == 3:
            common_functions.test_passed()
            return
        elif all(isinstance(sublist, list) and len(sublist) == 3 and all(isinstance(i, int) for i in sublist) for sublist in parsed_response):
            common_functions.test_passed()
            return
    common_functions.test_failed()

common_functions.print_test_description(test_description)

communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

#communication.serial_port = "/dev/tty.usbserial-140"
communication.open_serial_port()

# Let's reset all devices to start from a clean state
print("Resetting all devices")
communication.execute_command(communication.ALL_ALIASES, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

# Give time for the devices to reset and boot up
print("Waiting for devices to reset")
time.sleep(1.5)

print("Detecting devices")
parsed_response = communication.execute_command(communication.ALL_ALIASES, "DETECT_DEVICES_COMMAND", [], verbose=VERBOSE)
alias_list = common_functions.get_alias_list(parsed_response)
common_functions.print_alias_list(alias_list)
if len(alias_list) == 0:
    print("Error: No devices were detected")
    common_functions.test_failed()

check_parsed_response(parsed_response)

