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
import textwrap

protocol_version = None
registered_commands = None
command_data_types = None
data_type_dict = None
data_type_to_size_dict = None
data_type_min_value_dict = None
data_type_max_value_dict = None
data_type_is_integer_dict = None
data_type_description_dict = None
ser = None
alias = 255
detect_devices_command_id = None


# When we try to communicate with some external device, a number of things can go wrong. Here are some custom exceptions
# Timeout on the communication
class TimeoutError(Exception):
    pass
# Communication failure
class CommunicationError(Exception):
    pass
# The payload appears to be wrong or invalid
class PayloadError(Exception):
    pass



def get_response(verbose=True):
    response = ser.read(3)
    if len(response) == 0:
        # Throw and exception if we don't get a response
        raise TimeoutError("Error: timeout")
    if len(response) != 3:
        # Thow a general communication error
        raise CommunicationError("Error: the response is not 3 bytes long")
    if verbose:
        print("Received a response: ", response)
    if response[0] != ord('R'):
        raise CommunicationError("Error: the first byte is not the expected R")
    payload_size = response[2]
    if payload_size == 0xff:
        response2 = ser.read(2)
        if len(response2) == 0:
            # Throw and exception if we don't get a response
            raise TimeoutError("Error: timeout")
        if len(response2) != 2:
            # Thow a general communication error
            raise CommunicationError("Error: the response indicated a size of 255 and then the second response is not 2 bytes long")
        payload_size = response2[0] + (response2[1] << 8)
        print("Received an extended size:", payload_size)
    if payload_size == 0:
        if response[1] != 0:
            raise CommunicationError("Error: the second byte should be 0 if there is no payload")
    else:
        if response[1] != 1:
            raise CommunicationError("Error: the second byte should be 1 if there is a payload")

    if payload_size == 0:
        if verbose:
            print("This response indicates SUCCESS and has no payload")
        return b''

    payload = ser.read(payload_size)
    if len(payload) != payload_size:
        raise PayloadError("Error: didn't receive the right length (%d bytes) payload. Received (%d bytes): %s" % (payload_size, len(payload), payload))

    if verbose:
        print("Got a valid payload:")
        print_data(payload)

    return payload


def sniffer():
    n_bytes_received = 0
    while True:
        new_byte = ser.read(1)
        if len(new_byte) != 1:
            if n_bytes_received > 0:
                print("---------------------------------------------------------------")
                print("Timed out after incomplete communication. Here is what we know:")
                print("   Received %d bytes" % (n_bytes_received))
                if n_bytes_received >= 1:
                    if alias >= 33 and alias <= 126:
                        alias = chr(alias)
                    else:
                        alias = str(alias)
                    print("   Alias: %s" % (alias))
                if n_bytes_received >= 2:
                    print("   Command ID: %d" % (command_id))
                if n_bytes_received >= 3:
                    print("   Payload length: %d" % (payload_len))
                if n_bytes_received >= 4:
                    print("   Payload:", payload)
                print("---------------------------------------------------------------")
                return None, None, None
            continue
        new_byte = new_byte[0]
        n_bytes_received += 1
        if n_bytes_received == 1:
            alias = new_byte
            payload = bytearray()
        elif n_bytes_received == 2:
            command_id = new_byte
        else:
            if n_bytes_received == 3:
                payload_len = new_byte
                if payload_len == 0:
                    break
            else:
                payload.append(new_byte)
                if len(payload) >= payload_len:
                    break

    return alias, command_id, payload


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


def flush_receive_buffer():
    ser.reset_input_buffer()

def send_command(command_id, gathered_inputs, verbose=True):
    command_payload_len = len(gathered_inputs)
    command = bytearray([alias, command_id, command_payload_len]) + gathered_inputs
    if verbose:
        print("Writing %d bytes" % (len(command)))
        print_data(command)
    ser.write(command)
    if (alias == 255) and (command_id != detect_devices_command_id):
        if verbose:
            print("Sending a command to all devices (alias %d) and there will be no response" % (alias))
        return None
    if registered_commands[command_id][3] == []:
        if verbose:
            print("This command has no response whatsoever")
        return None
    all_response_payloads = []
    while(1):
        try:
            response_payload = get_response(verbose=verbose)
        except TimeoutError:
            if alias == 255: # we are sending a command to all devices and so we are expecting to get no response and rather a timeout
                break
            if registered_commands[command_id][4] == True: # check if this commmand may have any number of responses (including none)
                break
            # There is no legitamate reason that we should get a timeout here, so we need to raise this Timeout error again
            raise TimeoutError("Error: timeout")
#        except CommunicationError as e:
#            print("Communication Error:", e)
#            exit(1)
#        except PayloadError as e:
#            print("Payload Error:", e)
#            exit(1)
        if registered_commands[command_id][4] == False:
            return response_payload
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


def string_to_u64_unique_id(input):
    # conver the string representation of a base 16 hex number to a positive integer that fits into a u64
    try:
        converted_input = int(input, 16)
    except ValueError:
        print("Error: cannot convert the hexadecimal number %s to an integer" % (input))
        exit(1)
    if converted_input < 0 or converted_input > 0xFFFFFFFFFFFFFFFF:
        print("Error: it is not within the allowed range. The allowed range is 0 to 0xFFFFFFFFFFFFFFFF")
        exit(1)
    return converted_input


def set_serial_port_from_args(args):
    global serial_port

    if args.PORT == True:
        serial_port = "MENU"
    else:
        serial_port = args.port


def set_standard_options_from_args(args):
    global alias

    set_serial_port_from_args(args)

    alias = args.alias
    if alias == None:
        print("Error: you must specify an alias with the -a option. Run this command with -h to get more detailed help.")
        exit(1)

    print("Getting the alias of the device we want to communicate with (%s)" % (alias))
    alias = string_to_u8_alias(alias)


def set_command_data(new_protocol_version, new_registered_commands, new_command_data_types, new_data_type_dict, new_data_type_to_size_dict, new_data_type_min_value_dict,
                     new_data_type_max_value_dict, new_data_type_is_integer_dict, new_data_type_description_dict):
    global protocol_version
    global registered_commands
    global command_data_types
    global data_type_dict
    global data_type_to_size_dict
    global data_type_min_value_dict
    global data_type_max_value_dict
    global data_type_is_integer_dict
    global data_type_description_dict
    global detect_devices_command_id
    protocol_version = new_protocol_version
    registered_commands = new_registered_commands
    command_data_types = new_command_data_types
    data_type_dict = new_data_type_dict
    data_type_to_size_dict = new_data_type_to_size_dict
    data_type_min_value_dict = new_data_type_min_value_dict
    data_type_max_value_dict = new_data_type_max_value_dict
    data_type_is_integer_dict = new_data_type_is_integer_dict
    data_type_description_dict = new_data_type_description_dict
    detect_devices_command_id = get_command_id("DETECT_DEVICES_COMMAND")
    

def open_serial_port(timeout = 1.2):
    global ser
    ser = serial_functions.open_serial_port(serial_port, 230400, timeout = timeout)


def close_serial_port():
    ser.close


def input_or_response_to_string(data):
    if not isinstance(data, tuple):
        print("Error: the data is not a tuple")
        exit(1)
    if len(data) != 2:
        print("Error: the data is not a tuple of length 2")
        exit(1)
    data_type = data_type_dict[data[0]]
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


def print_protocol_version():
    print("Currently using protocol version v%d" % (protocol_version))


def print_data_type_descriptions():
    print("\nThese are the descriptions of the various input and output data types currently supported:\n")
    header1 = "   Data type ID   "
    header2 = "Size (bytes)"
    header3 = "      Max Value      "
    header4 = "      Min Value      "
    header5 = "Description"
    header_format_string = "%%%ds | %%%ds | %%%ds | %%%ds | %%%ds" % (len(header1), len(header2), len(header3), len(header4), len(header5))
    data_format_string = "%%%ds | %%%ds | %%%ds | %%%ds | %%%ds" % (len(header1), len(header2), len(header3), len(header4), len(header5))
    header = header_format_string % (header1, header2, header3, header4, header5)
    # print the header
    print(header)
    header_len = len(header)
    # print a bar that has the length of the header to underline the header
    print("-" * header_len)

    for data_type_id in data_type_dict.keys():
        data_type = data_type_dict[data_type_id]
        data_size = data_type_to_size_dict[data_type_id]
        if(data_size != None):
            data_size = str(data_size)
        else:
            data_size = "Variable"
        data_type_is_integer = data_type_is_integer_dict[data_type_id]
        if(data_type_is_integer):
            data_max_value = str(data_type_max_value_dict[data_type_id])
            data_min_value = str(data_type_min_value_dict[data_type_id])
        else:
            data_max_value = "Not Applicable"
            data_min_value = "Not Applicable"
        data_description = data_type_description_dict[data_type_id]
        print(data_format_string % (data_type, data_size, data_max_value, data_min_value, data_description))

def print_registered_commands():
    print("\nThese are the supported commands:\n")
    # First, find out the width of the terminal window
    terminal_window_columns = shutil.get_terminal_size().columns
    # iterate through all the sorted keys in the dictionary
    for key in sorted(registered_commands.keys()):
        print("%3d: %s" % (key, registered_commands[key][0]))
        print_wrapped_text("     ", "     ", registered_commands[key][1], terminal_window_columns)
        inputs = registered_commands[key][2]
        print_inputs_or_responses("     Input: ", inputs, terminal_window_columns)
        response = registered_commands[key][3]
        print_inputs_or_responses("     Response: ", response, terminal_window_columns)
        print()

def get_command_id(command):
    for key in sorted(registered_commands.keys()):
        registered_command = registered_commands[key]
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


def buf10_to_packed_bytes(input):
    if(isinstance(input, str)):
        # convert the string into a bytearray
        input_packed = bytearray(input, "utf-8")
    elif(isinstance(input, bytes) or isinstance(input, bytearray)):
        input_packed = input
    else:
        print("Error: the input is not a string nor is it a bytearray. It is a:", type(input))
        exit(1)
    # check that the length of the bytearray is correct
    if len(input_packed) != 10:
        print("Error: the input is not 10 bytes long")
        exit(1)
    return input_packed


def convert_input_to_right_type(data_type_id, input, input_data_size, is_integer, input_data_min_value, input_data_max_value, verbose=True):
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
        if verbose:
            print("The converted input is:", input_packed)
    else:
        if data_type_id == command_data_types.u8_alias:
            input_packed = string_to_u8_alias(input).to_bytes(1, byteorder = "little")
        elif data_type_id == command_data_types.list_2d:
            input_packed = list_2d_string_to_packed_bytes(input)
        elif data_type_id == command_data_types.buf10:
            input_packed = buf10_to_packed_bytes(input)
        elif data_type_id == command_data_types.u64_unique_id:
            input_packed = string_to_u64_unique_id(input).to_bytes(8, byteorder = "little")
        else:
            print("Error: didn't yet implement a converter to handle the input type:", data_type_dict[data_type_id])
            exit(1)
    return input_packed

def gather_inputs(command_id, inputs, verbose=True):
    expected_inputs = registered_commands[command_id][2]
    if len(expected_inputs) == 0:
        if verbose:
            print("This command takes no inputs")
    else:
        if len(expected_inputs) == 1:
            if verbose:
                print("The expected input for this command is:"),
        else:
            if verbose:
                print("The expected inputs for this command are:"),
        for i in range(len(expected_inputs)):
            input_text = input_or_response_to_string(expected_inputs[i])
            if verbose:
                print("  ", input_text)
    if verbose:
        print("%d input(s) were given" % (len(inputs)))
    if len(inputs) != len(expected_inputs):
        print("Error: the number of inputs given for this command is not the expected number")
        exit(1)
    if verbose:
        print("Good. You gave the correct number of inputs.")
    
    concatenated_inputs = bytearray()
    for i in range(len(inputs)):
        data_type_id = expected_inputs[i][0]
        input_data_size = data_type_to_size_dict[data_type_id]
        input_data_is_integer = data_type_is_integer_dict[data_type_id]
        if input_data_is_integer:
            input_data_min_value = data_type_min_value_dict[data_type_id]
            input_data_max_value = data_type_max_value_dict[data_type_id]
        else:
            input_data_min_value = None
            input_data_max_value = None
        input_data_type_string = data_type_dict[data_type_id]
        input_data_description = expected_inputs[i][1]
        if verbose:
            print("Now converting input number %d (%s)" % (i + 1, inputs[i]))
        converted_data = convert_input_to_right_type(data_type_id, inputs[i], input_data_size, input_data_is_integer, input_data_min_value, input_data_max_value, verbose=verbose)
        if converted_data == None:
            print("Error: bad input. This input must follow this convention:")
            print("  %s: %s (size: %d unpack type: %s)" % (input_data_type_string, input_data_description, input_data_size, input_data_min_value, input_data_max_value))
            exit(1)
        concatenated_inputs += converted_data
    return concatenated_inputs

def interpret_single_response(command_id, response, verbose=True):
    parsed_response = []
    if response == None:
        if verbose:
            print("This command has no other response and there is nothing more to say")
        return parsed_response
    expected_response = registered_commands[command_id][3]
    if len(expected_response) == 0:
        if verbose:
            print("We are expecting this command to have no response whatsoever")
        if response != None:
            print("Error: there was some sort of response")
            exit(1)
        if verbose:
            print("Good. There was no response.")
    elif len(expected_response) == 1 and expected_response[0][0] == command_data_types.success_response:
        if response != b'':
            print("Error: the response was not the expected success response")
            exit(1)
        if verbose:
            print("Good. The command was successful and the payload is empty as expected")
    else:
        if verbose:
            print("The expected response for this command along with the decoded value(s) is below:"),
        for i in range(len(expected_response)):
            if verbose:
                response_text = input_or_response_to_string(expected_response[i])
                print("  ", response_text)
            data_type = expected_response[i][0]
            if data_type == command_data_types.string_null_term:
                # find the first occurance of the null terminator in the byte array response
                null_terminator_index = response.find(b'\x00')
                # if it didn't fina a null terminator then throw an error
                if null_terminator_index == -1:
                    print("Error: the response from the device is not null terminated. This is a bug in the device or some sort of communication error")
                    exit(1)
                data_type_size = null_terminator_index + 1
            elif data_type == command_data_types.general_data:
                data_type_size = len(response)
            else:
                data_type_size = data_type_to_size_dict[data_type]
            if len(response) < data_type_size:
                print("Error: the response does not contain enough bytes to decode this data type")
                exit(1)
            data_item = response[:data_type_size]
            response = response[data_type_size:]
            data_item_is_integer = data_type_is_integer_dict[data_type]
            if data_item_is_integer:
                data_item_min_value = data_type_min_value_dict[data_type]
                data_item_max_value = data_type_max_value_dict[data_type]
                if data_item_min_value < 0:
                    data_item_signed = True
                else:
                    data_item_signed = False
                from_bytes_result = int.from_bytes(data_item, byteorder = "little", signed = data_item_signed)
                parsed_response.append(from_bytes_result)
                if verbose:
                    print("   --->", from_bytes_result)
            else:
                if data_type == command_data_types.string8:
                    # remove the null terminator from the string
                    data_item = data_item[:-1]
                    data_item = data_item.decode("utf-8")
                    parsed_response.append(data_item)
                    if verbose:
                        print("   --->", data_item)
                if data_type == command_data_types.string_null_term:
                    data_item = data_item.decode("utf-8")
                    parsed_response.append(data_item)
                    if verbose:
                        print("   --->", data_item)
                elif data_type == command_data_types.u24_version_number:
                    parsed_response.append([data_item[0], data_item[1], data_item[2]])
                    if verbose:
                        print("   ---> %d.%d.%d" % (data_item[2], data_item[1], data_item[0]))
                elif data_type == command_data_types.u32_version_number:
                    parsed_response.append([data_item[0], data_item[1], data_item[2], data_item[3]])
                    if verbose:
                        print("   ---> %d.%d.%d.%d" % (data_item[3], data_item[2], data_item[1], data_item[0]))
                elif data_type == command_data_types.u64_unique_id:
                    from_bytes_result = int.from_bytes(data_item, byteorder = "little")
                    parsed_response.append(from_bytes_result)
                    if verbose:
                        print("   ---> %016X" % (from_bytes_result))
                elif data_type == command_data_types.u8_alias:
                    from_bytes_result = int.from_bytes(data_item, byteorder = "little")
                    parsed_response.append(from_bytes_result)
                    if verbose:
                        if from_bytes_result >= 33 and from_bytes_result <= 126:
                            print("   ---> the ASCII character %c (or the decimal number %d)" % (from_bytes_result, from_bytes_result))
                        else:
                            print("   ---> the single byte integer %d or 0x%02x in hex" % (from_bytes_result, from_bytes_result))
                elif data_type == command_data_types.crc32:
                    from_bytes_result = int.from_bytes(data_item, byteorder = "little")
                    parsed_response.append(from_bytes_result)
                    if verbose:
                        print("   ---> 0x%08X" % (from_bytes_result))
                elif data_type == command_data_types.buf10:
                    parsed_response.append(data_item)
                    if verbose:
                        print("   ---> %s" % (data_item))
                elif data_type == command_data_types.general_data:
                    parsed_response.append(data_item)
                    if verbose:
                        for d in data_item:
                            print("   ---> %d (0x%02x)" % (d, d))
                else:
                    print("Error: the interprettation of this data type is not iplemented:", data_type)
                    exit(1)
        if len(response) != 0:
            print("Error: there unexpected bytes left in the response after interpreting the expected response")
            exit(1)
    return parsed_response


def interpret_response(command_id, response, verbose=True):
    parsed_response = []
    if not isinstance(response, list):
        parsed_response = interpret_single_response(command_id, response, verbose=verbose)
    else:
        if len(response) == 0:
            if verbose:
                print("There was no response from any device(s)")
        elif len(response) == 1:
            parsed_response = interpret_single_response(command_id, response[0], verbose=verbose)
        else:
            if verbose:
                print("The response is a list containing multiple responses. Interpreting them:")
            for i in range(len(response)):
                partial_parsed_response = interpret_single_response(command_id, response[i], verbose=verbose)
                parsed_response.append(partial_parsed_response)
    return parsed_response
