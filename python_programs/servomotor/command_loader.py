import json
import os

# The data_types.json file has the data types as a list of dictionaries. Here is an example of some data types:
#[
#    {
#        "data_type_id:": 19,
#        "data_type": "buf10",
#        "size": 10,
#        "min_value": null,
#        "max_value": null,
#        "is_integer": false,
#        "description": "10 byte long buffer containing any binary data"
#    },
#    {
#        "data_type_id:": 17,
#        "data_type": "crc32",
#        "size": 4,
#        "min_value": null,
#        "max_value": null,
#        "is_integer": false,
#        "description": "32-bit CRC"
#    }
#]
# We want to access the data using the data_type_id as the key. We will convert this to a dictionary where the key is the data type id
# and the rest of the data is stored in a structure that can be accessed like follows
# data_type = data_type_dict[data_type_id].data_type
# size = data_type_dict[data_type_id].size
# min_value = data_type_dict[data_type_id].min_value
# max_value = data_type_dict[data_type_id].max_value
# is_integer = data_type_dict[data_type_id].is_integer
# description = data_type_dict[data_type_id].description

class DataType:
    def __init__(self, data_type_str, size, min_value, max_value, is_integer, description):
        self.data_type_str = data_type_str
        self.size = size
        self.min_value = min_value
        self.max_value = max_value
        self.is_integer = is_integer
        self.description = description

    def __str__(self):
        return f"DataType(data_type_str={self.data_type_str}, size={self.size}, min_value={self.min_value}, max_value={self.max_value}, is_integer={self.is_integer}, description={self.description})"

    def __repr__(self):
        return self.__str__()
    

class InputOrOutput:
    def __init__(self, data_type_id, description):
        self.data_type_id = data_type_id
        self.description = description

    def __str__(self):
        return f"InputOrOutput(data_type_id={self.data_type_id}, description={self.description})"

    def __repr__(self):
        return self.__str__()
    
def convert_input_or_output(json_format, data_type_dict, success_response_data_type_id, success_response_description):
    converted_format = []
    if json_format == None:
        return []
    elif type(json_format) == str and json_format == "success_response":
        return [InputOrOutput(success_response_data_type_id, success_response_description)]
    elif type(json_format) != list:
        json_format = [json_format] # convert it to a list so that we can iterate over it
    for item in json_format:
        data_type_and_description = item["Description"]
        # extract the data type, which is the part before the colon, but if there is no colon then print an error and exit
        # also if there is nothing before or after the colon, then print an error and exit
        if ":" not in data_type_and_description:
            print("ERROR: The input string", data_type_and_description, "does not contain a colon. The input string should be in the format 'data_type: description'")
            exit(1)
        if data_type_and_description.split(":")[0] == "" or data_type_and_description.split(":")[1] == "":
            print("ERROR: The input string", data_type_and_description, "contains an empty data type or description. The input string should be in the format 'data_type: description'")
            exit(1)
        data_type_str = data_type_and_description.split(":")[0]
        # because there is likely a space after the colon, whiche precedes the description, we need to remove it
        description = data_type_and_description.split(":")[1].strip()
        # now that we have the data type in string format, we need to convert it to the actual data type id. we need to look it up
        # in the data_types_dict. If it is not found, then print an error and exit
        data_type_id = None
        for key in data_type_dict.keys():
            if data_type_dict[key].data_type_str == data_type_str:
                data_type_id = key
                break
        if data_type_id == None:
            print("ERROR: The data type", data_type_str, "is not found in the data types dictionary")
            exit(1)
        converted_format.append(InputOrOutput(data_type_id, description))
    return converted_format


def load_data_types_and_commands(data_type_json_file, commands_json_file, verbose=False):
    if verbose:
        print("\nLoading data types and commands...")
    
    # Load data types
    data_type_dict = {}
    success_response_data_type_id = None
    success_response_data_type_description = None
    
    if verbose:
        print(f"Loading data types from {data_type_json_file}")
    with open(data_type_json_file, 'r') as file:
        data_types_list = json.load(file)
    if verbose:
        print(f"Found {len(data_types_list)} data types")
    
    for data_type in data_types_list:
        size = data_type['size']
        min_value = data_type['min_value']
        max_value = data_type['max_value']
        is_integer = data_type['is_integer']
        description = data_type['description']
        data_type_id = data_type['data_type_id']
        data_type_str = data_type['data_type']
        # and now let's store this appropriately in the data_type_dict
        data_type_dict[data_type_id] = DataType(data_type_str, size, min_value, max_value, is_integer, description)
        if data_type_str == "success_response":
            success_response_data_type_id = data_type_id # this is frequently used in many commands, so let's store the data type id (for efficiency)
            success_response_data_type_description = description
            if verbose:
                print("Found success_response data type")

    # Load commands
    if verbose:
        print(f"\nLoading commands from {commands_json_file}")
    with open(commands_json_file, 'r') as file:
        commands = json.load(file)
    if verbose:
        print(f"Found {len(commands)} commands in JSON")

    # go through all the commands and convert the inputs and outputs to tuples where the first element is the data type and the second element is the description
    # The json file has the input either as null or as a list of dictionaries. The output can be a string saying "success_response" or ot may
    # be a list of dictionaries. Here is an example of some commands:
    #{
    #    "CommandString": "Ping",
    #    "CommandEnum": 31,
    #    "Description": "Send a payload containing any data and the device will respond with the same data back",
    #    "Input": [
    #        {
    #            "Description": "buf10: Any binary data payload to send to the device.",
    #            "TooltipDisplayFormat": "%s"
    #        }
    #    ],
    #    "Output": [
    #        {
    #            "Description": "buf10: The same data that was sent to the device will be returned if all went well.",
    #            "TooltipDisplayFormat": "%s"
    #        }
    #    ]
    #},
    if verbose:
        print("\nConverting commands...")
    converted_commands = []
    for command in commands:
        try:
            if verbose:
                print(f"Converting command: {command['CommandString']} (ID: {command['CommandEnum']})")
            command_copy = command.copy()
            command_copy["Input"] = convert_input_or_output(
                command["Input"], data_type_dict, 
                success_response_data_type_id, success_response_data_type_description)
            command_copy["Output"] = convert_input_or_output(
                command["Output"], data_type_dict, 
                success_response_data_type_id, success_response_data_type_description)
            if "MultipleResponses" not in command_copy:
                command_copy["MultipleResponses"] = False
            converted_commands.append(command_copy)
        except Exception as e:
            print(f"Error converting command {command.get('CommandString', 'unknown')}: {e}")
            continue

    if verbose:
        print(f"\nSuccessfully converted {len(converted_commands)} commands")
    return data_type_dict, converted_commands
