#!/usr/bin/env python3

import time
import struct
import argparse
import motor_commands
import communication

VERBOSE = False
MAX_BAR_LENGTH = 80
BAR_PLOT_SCALE_FACTOR = MAX_BAR_LENGTH / 10000.0

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

def print_statistics(response):
    #motor_commands.interpret_response(command_id, response)
    #print("The response is:", response)
    # unpack this response into a tuple
    all_statistics = struct.unpack("<HHHHHHQQQI", response)
    hall1_max = all_statistics[0]
    hall2_max = all_statistics[1]
    hall3_max = all_statistics[2]
    hall1_min = all_statistics[3]
    hall2_min = all_statistics[4]
    hall3_min = all_statistics[5]
    hall1_mean = all_statistics[6] / all_statistics[9]
    hall2_mean = all_statistics[7] / all_statistics[9]
    hall3_mean = all_statistics[8] / all_statistics[9]
    hall1_delta = hall1_max - hall1_min
    hall2_delta = hall2_max - hall2_min
    hall3_delta = hall3_max - hall3_min
    hall1_delta_fraction = hall1_delta / hall1_mean
    hall2_delta_fraction = hall2_delta / hall2_mean
    hall3_delta_fraction = hall3_delta / hall3_mean
    print("The statistics are:")
    print("   Hall 1 maximum value: ", hall1_max)
    print("   Hall 2 maximum value: ", hall2_max)
    print("   Hall 3 maximum value: ", hall3_max)
    print("   Hall 1 minimum value: ", hall1_min)
    print("   Hall 2 minimum value: ", hall2_min)
    print("   Hall 3 minimum value: ", hall3_min)
    print("   Hall 1 mean value:    ", hall1_mean)
    print("   Hall 2 mean value:    ", hall2_mean)
    print("   Hall 3 mean value:    ", hall3_mean)
    print("   Hall 1 delta:         ", hall1_delta)
    print("   Hall 2 delta:         ", hall2_delta)
    print("   Hall 3 delta:         ", hall3_delta)
    print("   Hall 1 delta fraction:", hall1_delta_fraction)
    print("   Hall 2 delta fraction:", hall2_delta_fraction)
    print("   Hall 3 delta fraction:", hall3_delta_fraction)


def make_text_based_bar_plot(int32_list):
    # print the items in the list by showing a plot, where the number of "#" symbols is proportional to the value.
    # 0 is in the middle of the screen (ie. 80 characters in). Negative values are a bar graph to the left and positive
    # values are a bar graph to the right. Before drawing graphs, we need to find the min and max values in the list
    # and then scale the list such that the maximum length of any bar to the left or right is 80 characters.
    min_value = min(int32_list)
    max_value = max(int32_list)
    if BAR_PLOT_SCALE_FACTOR == None:
        max_abs_value = max(abs(min_value), abs(max_value))
        scale_factor = MAX_BAR_LENGTH / max_abs_value
    else:
        scale_factor = BAR_PLOT_SCALE_FACTOR
    print("The min value is:", min_value)
    print("The max value is:", max_value)
    # now scale the list
    scaled_list = []
    for value in int32_list:
        scaled_value = int(value * scale_factor)
        if scaled_value < -MAX_BAR_LENGTH:
            scaled_value = -MAX_BAR_LENGTH
        elif scaled_value > MAX_BAR_LENGTH:
            scaled_value = MAX_BAR_LENGTH
        scaled_list.append(scaled_value)
    # now print the list
    for value in scaled_list:
        if value < 0:
            print(" " * (MAX_BAR_LENGTH + value) + "#" * (-value))
        else:
            print(" " * MAX_BAR_LENGTH + "#" * value)


communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='This program will let you send any supported command to the motor')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
args = parser.parse_args()

communication.set_standard_options_from_args(args) # This will find out the port to use and the alias of the device and store those in the motor_commands module
communication.open_serial_port()

alias = communication.alias

command_text = "SET_MAXIMUM_MOTOR_CURRENT"
print("Running the command:", command_text)
command_id = communication.get_command_id(command_text)
assert(command_id != None)
gathered_inputs = communication.gather_inputs(command_id, [250, 250])
response = communication.send_command(command_id, gathered_inputs)
communication.interpret_response(command_id, response)

command_text = "ZERO_POSITION_COMMAND"
print("Running the command:", command_text)
command_id = communication.get_command_id(command_text)
assert(command_id != None)
gathered_inputs = communication.gather_inputs(command_id, [])
response = communication.send_command(command_id, gathered_inputs)
communication.interpret_response(command_id, response)

parsed_response = execute_command(alias, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("Error: The device with alias", alias, "did not respond correctly to the ENABLE_MOSFETS_COMMAND and did not return zero bytes")
    exit(1)
time.sleep(3)

while 1:
    command_text = "VIBRATE_COMMAND"
    print("Running the command:", command_text)
    command_id = communication.get_command_id(command_text)
    assert(command_id != None)
    gathered_inputs = communication.gather_inputs(command_id, [1])
    response = communication.send_command(command_id, gathered_inputs)
    communication.interpret_response(command_id, response)
    time.sleep(1)

    command_text = "VIBRATE_COMMAND"
    print("Running the command:", command_text)
    command_id = communication.get_command_id(command_text)
    assert(command_id != None)
    gathered_inputs = communication.gather_inputs(command_id, [0])
    response = communication.send_command(command_id, gathered_inputs)
    communication.interpret_response(command_id, response)

    parsed_response = execute_command(alias, "READ_MULTIPURPOSE_BUFFER_COMMAND", [], verbose=VERBOSE)
    parsed_response = parsed_response[0]
    if len(parsed_response) < 1:
        print("Error: The device with alias", alias, "did not respond correctly to the READ_MULTIPURPOSE_BUFFER_COMMAND and did not return at least one byte")
        exit(1)
    data_type = parsed_response[0]
    parsed_response = parsed_response[1:]
    print("Go to closed loop mode number of data elements:", len(parsed_response))
    parsed_response = bytearray(parsed_response)
    int32_list = [int.from_bytes(parsed_response[i:i+4], byteorder='little', signed=True) for i in range(0, len(parsed_response), 4)]
    print("The length of the data items is:", len(int32_list))
    make_text_based_bar_plot(int32_list)

    time.sleep(0.5)


communication.close_serial_port()
