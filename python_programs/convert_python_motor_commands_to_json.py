#!/usr/bin/env python3

# This program will read in a dictionary containing a bunch of data for the various motor commands
# and it will write out the same data in JSON format to a .json file

import json
import motor_commands

DATA_TYPE_OUTPUT_FILENAME = "data_types.json"
MOTOR_COMMANDS_OUTPUT_FILENAME = "motor_commands.test.json"

def convert_to_list_with_text_types(items):
    converted = []
    for item in items:
        item_type = item[0]
        item_description = item[1]
        item_type_text = motor_commands.data_type_dict[item_type]
        converted.append([item_type_text, item_description])
        print(item_type_text, item_description)
    return converted


output_data_list = []

# iterate through all the members of the class
for data_type_name in dir(motor_commands.command_data_types):
    # ignore the members that start with an underscore
    if data_type_name.startswith("_"):
        continue
    # get the value of the member
    value = getattr(motor_commands.command_data_types, data_type_name)
    # ignore the members that are not integers
    if not isinstance(value, int):
        continue

    size = motor_commands.data_type_to_size_dict[value]
    min_value = motor_commands.data_type_min_value_dict[value]
    max_value = motor_commands.data_type_max_value_dict[value]
    is_integer = motor_commands.data_type_is_integer_dict[value]
    description = motor_commands.data_type_description_dict[value]


    # create a list of the name, value, and description
    output_data_list.append({"data_type_id:": value, "data_type": data_type_name, "size": size, "min_value": min_value, "max_value": max_value, "is_integer": is_integer, "description": description})


with open(DATA_TYPE_OUTPUT_FILENAME, 'w') as fh:
    json.dump(output_data_list, fh, indent=4)



output_data_list = []

for key in motor_commands.registered_commands:
#    print(motor_commands.registered_commands[key])
    command_id = key
    command_name = motor_commands.registered_commands[key][0]
    description = motor_commands.registered_commands[key][1]
    inputs = motor_commands.registered_commands[key][2]
    inputs = convert_to_list_with_text_types(inputs)
    response = motor_commands.registered_commands[key][3]
    response = convert_to_list_with_text_types(response)
    multiple_responses = motor_commands.registered_commands[key][4]
    output_data_list.append({"command_id": command_id, "command_name": command_name, "description": description, "inputs": inputs, "response": response, "multiple_responses": multiple_responses})


with open(MOTOR_COMMANDS_OUTPUT_FILENAME, 'w') as fh:
    json.dump(output_data_list, fh, indent=4)
