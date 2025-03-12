#!/usr/bin/env python3

import redirect_motor_commands as motor_commands
import redirect_communication as communication
import time
import random
import argparse
import common_functions

test_description = '''
Here we will test the SET_DEVICE_ALIAS_COMMAND. At least one device needs to be connected but the more the merrier.
The program will detect all devices and then assign unique aliases (generated randomly) and then reset the devices
and make sure that the new aliases are set correctly. The test will pass if all the above conditions are met.
'''

VERBOSE = False
ALIASES_TO_AVOID = [254, 255]

common_functions.print_test_description(test_description)

parser = argparse.ArgumentParser(description='This program is for testing a motor')
args = common_functions.add_standard_option_to_parser(parser)
print("args:", args)
communication.set_serial_port_from_args(args)

communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

communication.open_serial_port()

# Let's reset all devices to start from a clean state
print("Resetting all devices")
communication.execute_command(communication.ALL_ALIAS, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

# Give time for the devices to reset and boot up
print("Waiting for devices to reset")
time.sleep(1.5)

# Detect all devices
print("Detecting devices")
parsed_response = communication.execute_command(communication.ALL_ALIAS, "DETECT_DEVICES_COMMAND", [], verbose=VERBOSE)
alias_list = common_functions.get_alias_list(parsed_response)
unique_id_list = common_functions.get_unique_id_list(parsed_response)
if len(alias_list) == 0:
    print("Error: No devices were detected")
    common_functions.test_failed()

# Generate random aliases that avoid the aliases in ALIASES_TO_AVOID and are all unique
new_aliases = []
for alias in alias_list:
    new_alias = None
    while new_alias is None or new_alias in ALIASES_TO_AVOID or new_alias in new_aliases:
        new_alias = random.randint(0, 255)
    new_aliases.append(new_alias)

print("We will use the following aliases for the devices:")
for alias in new_aliases:
    s = common_functions.get_human_readable_alias(alias)
    print(f"   {s}")

for i, alias in enumerate(alias_list):
    new_alias = new_aliases[i]
    unique_id = unique_id_list[i]
    human_readable_new_alias = common_functions.get_human_readable_alias(new_alias)
    human_readable_unique_id = common_functions.get_human_readable_unique_id(unique_id)
    print(f"Setting alias of device with unique ID {human_readable_unique_id} to {human_readable_new_alias}")
    parsed_response = communication.execute_command(communication.ALL_ALIAS, "SET_DEVICE_ALIAS_COMMAND", [unique_id, new_alias], verbose=VERBOSE)

# Give time for the devices to reset and boot up
print("Waiting for devices to all set the new alias and reset")
time.sleep(1.5)

# Detect all devices
print("Detecting devices")
parsed_response = communication.execute_command(communication.ALL_ALIAS, "DETECT_DEVICES_COMMAND", [], verbose=VERBOSE)
second_alias_list = common_functions.get_alias_list(parsed_response)
second_unique_id_list = common_functions.get_unique_id_list(parsed_response)
if len(second_alias_list) != len(alias_list):
    print("Error: Some or all of the devices were not detected in the second round of detection")
    print("An error might have occurred. Please check if the red LED on the device is flashing and if so, check the error code.")
    common_functions.test_failed()
print("And here are the aliases of the devices after the second round of detection:")
for alias in new_aliases:
    s = common_functions.get_human_readable_alias(alias)
    print(f"   {s}")

# Now let's search through the second list of unique IDs and make sure that they exist in the original list and then make sure that
# the alias is the same as the new alias
for i, unique_id in enumerate(second_unique_id_list):
    if unique_id not in unique_id_list:
        print("Error: The unique ID", common_functions.get_human_readable_unique_id(unique_id), "was not found in the original list of unique IDs")
        common_functions.test_failed()
    index = unique_id_list.index(unique_id)
    if second_alias_list[i] != new_aliases[index]:
        print("Error: The alias of the device with unique ID", common_functions.get_human_readable_unique_id(unique_id), "was not set correctly")
        common_functions.test_failed()
print("All devices were detected and the aliases were set as expected")
common_functions.test_passed()