#!/usr/local/bin/python3

import sys
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import time
import struct
import argparse
import shutil
import serial_functions
import commands
import textwrap

ser = None
alias = 255

def get_response():
    response = ser.read(3)
    if len(response) == 0:
        return None, "Error: timeout"
    if len(response) != 3:
        return None, "Error: the response is not 3 bytes long"
    print("Received a response: ", response)
    if response[0] != ord('R'):
        return None, "Error: the first is not the expected R"
    payload_size = response[2]
    if payload_size == 0:
        if response[1] != 0:
            return None, "Error: the second byte should be 0 if there is no payload"
    else:
        if response[1] != 1:
            return None, "Error: the second byte should be 1 if there is a payload"

    if payload_size == 0:
        print("This response indicates SUCCESS and has no payload")
        return b''

    payload = ser.read(payload_size)
    if len(payload) != payload_size:
        return None, "Error: didn't receive the right length payload. Received: %s" % (payload)

    print("Got a valid payload:")
    print_data(payload)

    return payload


def parse_response(response):
    if len(response) != 13:
        return None, None
    unique_id = int.from_bytes(response[0:8], "little")
    alias = int(response[4])
    crc32 = int.from_bytes(response[5:9], "little")
    if crc32 != 0x04030201:
        return None, None
    return unique_id, alias


def print_data(data):
    print(data)
    for d in data:
        print("0x%02X %d" % (d, d))


def send_command(command_id, gathered_inputs):
    command_payload_len = len(gathered_inputs)
    command = bytearray([alias, command_id, command_payload_len]) + gathered_inputs
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)
    if commands.registered_commands[command_id][3] == []:
        print("This command has no response whatsoever")
        return None
    all_response_payloads = []
    while(1):
        response_payload = get_response()
        if commands.registered_commands[command_id][4] == False:
            if isinstance(response_payload, tuple):
                print(response_payload[1]) # An error occured and the error message is the second element of the tuple
                exit(1)
            return response_payload
        else:
            if isinstance(response_payload, tuple):
                if response_payload[1] == "Error: timeout":
                    break
                else:
                    print(response_payload[1]) # An error occured and the error message is the second element of the tuple
                    exit(1)
        all_response_payloads.append(response_payload)
    return all_response_payloads


def add_standard_option_to_parser(parser):
    # Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
    # and it also takes a mandatory firmware file name
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
    args = parser.parse_args()
    return 0

def string_to_u8_alias(input):
    if input == "255":
        converted_input = 255
    elif len(input) == 1:
        converted_input = ord(input)
    else:
        try:
            converted_input = int(input)
        except ValueError:
            print("Error: it is not a single character nor is it an integer")
            exit(1)
        if converted_input < 0 or converted_input > 255:
            print("Error: it is not within the allowed range. The allowed range is 0 to 255")
            exit(1)
    if converted_input == ord('R'):
        print("Error: the alias R is not allowed because it is reserved to indicate a response")
        exit(1)
    return converted_input


def set_standard_options_from_args(args):
    global serial_port
    global alias

    if args.PORT == True:
        serial_port = "MENU"
    else:
        serial_port = args.port

    alias = args.alias
    if alias == None:
        print("Error: you must specify an alias with the -a option. Run this command with -h to get more detailed help.")
        exit(1)

    print("Getting the alias of the device we want to communicate with (%s)" % (alias))
    alias = string_to_u8_alias(alias)


def open_serial_port():
    global ser
    ser = serial_functions.open_serial_port(serial_port, 230400, timeout = 1.2)


def close_serial_port():
    ser.close


def input_or_response_to_string(data):
    if not isinstance(data, tuple):
        print("Error: the data is not a tuple")
        exit(1)
    if len(data) != 2:
        print("Error: the data is not a tuple of length 2")
        exit(1)
    data_type = commands.data_type_dict[data[0]]
    data_description = data[1]
    return "%s: %s" % (data_type, data_description)

def print_wrapped_text(first_line_message, subsequent_lines_message, text, terminal_window_columns):
    wrapped_text = textwrap.wrap(text, initial_indent = first_line_message, subsequent_indent = subsequent_lines_message, width = terminal_window_columns)
    for line in wrapped_text:
        print(line)

def print_inputs_or_responses(message, data, terminal_window_columns):
    subsequent_spaces = 12
    if data == None or (isinstance(data, list) and len(data) == 0):
        print(message, "None")
    elif isinstance(data, list):
        for i in range(len(data)):
            text = input_or_response_to_string(data[i])
            print_wrapped_text(message, " " * subsequent_spaces, text, terminal_window_columns)
    elif isinstance(data, tuple):
        text = input_or_response_to_string(data)
        print_wrapped_text(message, " " * subsequent_spaces, text, terminal_window_columns)
    else:
        print("Error: this is not correctly formatted:", data)

def print_registered_commands():
    print("\nThese are the registered commands:\n")
    # First, find out the width of the terminal window
    terminal_window_columns = shutil.get_terminal_size().columns
    # iterate through all the sorted keys in the dictionary
    for key in sorted(commands.registered_commands.keys()):
        print("%3d: %s" % (key, commands.registered_commands[key][0]))
        print_wrapped_text("     ", "     ", commands.registered_commands[key][1], terminal_window_columns)
        inputs = commands.registered_commands[key][2]
        print_inputs_or_responses("     Input: ", inputs, terminal_window_columns)
        response = commands.registered_commands[key][3]
        print_inputs_or_responses("     Response: ", response, terminal_window_columns)
        print()

def get_command_id(command):
    for key in sorted(commands.registered_commands.keys()):
        registered_command = commands.registered_commands[key]
        if command == registered_command[0]:
            return key
    return None

def list_2d_string_to_packed_bytes(input):
    # convert this string representation of a python two dimensional list to a python list of lists
    try:
        input_list_2d = eval(input)
    except NameError:
        print("Error: the input could not be successfully translated using the eval function in Python")
        exit(1)
    # check that the shape of this list is correct (ie. is a 2D list or in other words a list of lists)
    if not isinstance(input_list_2d, list):
        print("Error: the input could not be translated into a valid list object")
        exit(1)
    if len(input_list_2d) < 1:
        print("Error: the provided input appears to be an empty list")
        exit(1)
    is_signed = True
    input_packed = b''
    for list_item in input_list_2d:
        print("Here is a list item:", list_item)
        if not isinstance(list_item, list):
            print("Error: the sub-item in the list could not be translated into a valid list object")
            exit(1)
        if len(list_item) != 2:
            print("Error: the sub-item in the list is not a list with two elements in it")
            exit(1)
        for list_sub_item in list_item:
            if not isinstance(list_sub_item, int):
                print("Error: the sub-sub-item in the list is not an integer")
                exit(1)
            try:
                input_packed = input_packed + list_sub_item.to_bytes(4, byteorder="little", signed=is_signed)
            except OverflowError:
                print("Error: the integer value is too large to be represented in the given number of bytes")
                exit(1)
            is_signed = not is_signed
    return input_packed


def convert_input_to_right_type(data_type_id, input, input_data_size, is_integer, input_data_min_value, input_data_max_value):
    if is_integer:
        try:
            input_int = int(input)
        except ValueError:
            print("Error: the input is not an integer")
            exit(1)
        if input_int < input_data_min_value or input_int > input_data_max_value:
            print("Error: the input is not within the allowed range")
            print("The allowed range is:", input_data_min_value, "to", input_data_max_value)
            exit(1)
        if input_data_min_value < 0:
            input_signed = True
        else:
            input_signed = False
        input_packed = input_int.to_bytes(input_data_size, byteorder = "little", signed = input_signed)
        print("The converted input is:", input_packed)
    else:
        if data_type_id == commands.u8_alias:
            input_packed = string_to_u8_alias(input).to_bytes(1, byteorder = "little")
        elif data_type_id == commands.list_2d:
            input_packed = list_2d_string_to_packed_bytes(input)
        else:
            print("Error: didn't yet implement a converter to handle the input type:", commands.data_type_dict[data_type_id])
            exit(1)
    return input_packed

def gather_inputs(command_id, inputs):
    expected_inputs = commands.registered_commands[command_id][2]
    if len(expected_inputs) == 0:
        print("This command takes no inputs")
    else:
        if len(expected_inputs) == 1:
            print("The expected input for this command is:"),
        else:
            print("The expected inputs for this command are:"),
        for i in range(len(expected_inputs)):
            input_text = input_or_response_to_string(expected_inputs[i])
            print("  ", input_text)
    print("%d input(s) were given" % (len(inputs)))
    if len(inputs) != len(expected_inputs):
        print("Error: the number of inputs given for this command is not the expected number")
        exit(1)
    print("Good. You gave the correct number of inputs.")
    
    concatenated_inputs = bytearray()
    for i in range(len(inputs)):
        data_type_id = expected_inputs[i][0]
        input_data_size = commands.data_type_to_size_dict[data_type_id]
        input_data_is_integer = commands.data_type_is_integer_dict[data_type_id]
        if input_data_is_integer:
            input_data_min_value = commands.data_type_min_value_dict[data_type_id]
            input_data_max_value = commands.data_type_max_value_dict[data_type_id]
        input_data_type_string = commands.data_type_dict[data_type_id]
        input_data_description = expected_inputs[i][1]
        print("Now converting input number %d (%s)" % (i + 1, inputs[i]))
        converted_data = convert_input_to_right_type(data_type_id, inputs[i], input_data_size, input_data_is_integer, input_data_min_value, input_data_max_value)
        if converted_data == None:
            print("Error: bad input. This input must follow this convention:")
            print("  %s: %s (size: %d unpack type: %s)" % (input_data_type_string, input_data_description, input_data_size, input_data_min_value, input_data_max_value))
            exit(1)
        concatenated_inputs += converted_data
    return concatenated_inputs

def interpret_single_response(command_id, response):
    if response == None:
        print("This command has no other response and there is nothing more to say")
        return
    expected_response = commands.registered_commands[command_id][3]
    if len(expected_response) == 0:
        print("We are expecting this command to have no response whatsoever")
        if response != None:
            print("Error: there was some sort of response")
            exit(1)
        print("Good. There was no response.")
    elif len(expected_response) == 1 and expected_response[0][0] == commands.success_response:
        if response != commands.success_response:
            print("Error: the response was not the expected success response")
            exit(1)
        print("Good. The command was successful and the payload is empty as expected")
    else:
        print("The expected response for this command is (along with the decoded value(s) below):"),
        for i in range(len(expected_response)):
            response_text = input_or_response_to_string(expected_response[i])
            print("  ", response_text)
            data_type = expected_response[i][0]
            if data_type == commands.string_null_term:
                # find the first occurance of the null terminator in the byte array response
                null_terminator_index = response.find(b'\x00')
                # if it didn't fina a null terminator then throw an error
                if null_terminator_index == -1:
                    print("Error: the response from the device is not null terminated. This is a bug in the device or some sort of communication error")
                    exit(1)
                data_type_size = null_terminator_index + 1
            else:
                data_type_size = commands.data_type_to_size_dict[data_type]
            if len(response) < data_type_size:
                print("Error: the response does not contain enough bytes to decode this data type")
                exit(1)
            data_item = response[:data_type_size]
            response = response[data_type_size:]
            data_item_is_integer = commands.data_type_is_integer_dict[data_type]
            if data_item_is_integer:
                data_item_min_value = commands.data_type_min_value_dict[data_type]
                data_item_max_value = commands.data_type_max_value_dict[data_type]
                if data_item_min_value < 0:
                    data_item_signed = True
                else:
                    data_item_signed = False
                from_bytes_result = int.from_bytes(data_item, byteorder = "little", signed = data_item_signed)
                print("   --->", from_bytes_result)
            else:
                if data_type == commands.string8:
                    # remove the null terminator from the string
                    data_item = data_item[:-1]
                    print("   --->", data_item.decode("utf-8"))
                if data_type == commands.string_null_term:
                    print("   --->", data_item.decode("utf-8"))
                elif data_type == commands.u24_version_number:
                    print("   ---> %d.%d.%d" % (data_item[2], data_item[1], data_item[0]))
                elif data_type == commands.u32_version_number:
                    print("   ---> %d.%d.%d.%d" % (data_item[3], data_item[2], data_item[1], data_item[0]))
                elif data_type == commands.u64_unique_id:
                    print("   ---> %d" % (int.from_bytes(data_item, byteorder = "little")))
                elif data_type == commands.u8_alias:
                    if data_item >= bytearray([33]) and data_item <= bytearray([126]):
                        print("   ---> the ASCII character %c" % (data_item[0]))
                    else:
                        print("   ---> the single byte integer %d or 0x%02x in hex" % (data_item[0], data_item[0]))
                elif data_type == commands.crc32:
                    print("   ---> 0x%08X" % (int.from_bytes(data_item, byteorder = "little")))
                else:
                    print("Error: did not implement the part that interprets non-integer data yet")
                    exit(1)
        if len(response) != 0:
            print("Error: there unexpected bytes left in the response after interpreting the expected response")
            exit(1)


def interpret_response(command_id, response):
    if not isinstance(response, list):
        interpret_single_response(command_id, response)
    else:
        if len(response) == 0:
            print("There was no response from any device(s)")
        elif len(response) == 1:
            interpret_single_response(command_id, response[0])
        else:
            print("The response is a list containing multiple responses. Interpreting them:")
            for i in range(len(response)):
                interpret_single_response(command_id, response[i])

