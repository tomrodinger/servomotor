#!/usr/local/bin/python3

from .vendor.serial.serialutil import to_bytes
import shutil
from . import serial_functions
import textwrap
import sys
import os

# Import our terminal formatting module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from terminal_formatting import format_error, format_info, format_warning, format_success, format_debug, STYLE

ALL_ALIASES = 255
serial_port = None
PROTOCOL_VERSION = 20
registered_data_types = None
registered_commands = None
ser = None
alias = 255
detect_devices_command_id = None
set_device_alias_command_id = None
RESPONSE_CHARACTER = 254

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

def print_data(prefix_message, data, print_size=True, verbose=2):
    if verbose == 2:
        message = prefix_message
        for d in data:
            message += f"0x{d:02X} "
        if print_size:
            message += f"[{len(data)} bytes]"
        print(format_debug(message))

def get_human_readable_alias(alias):
    if alias >= 33 and alias <= 126:
        alias_str = f"{chr(alias)} ({alias})"
    else:
        alias_str = f"{alias} (0x{alias:02x})"
    return alias_str

def get_response(verbose=2):
    response = ser.read(3)
    if len(response) == 0:
        raise TimeoutError("Error: timeout")
    if len(response) != 3:
        print(format_error(f"Received: {response}"))
        raise CommunicationError("Error: the response is not 3 bytes long")
    if verbose == 2:
        print_data("Received a response: ", response, print_size=True, verbose=verbose)
    if response[0] != RESPONSE_CHARACTER:
        error_text = f"Error: the first byte (which should indicate a response) is not the expected {RESPONSE_CHARACTER}"
        raise CommunicationError(error_text)
    payload_size = response[2]
    if payload_size == 0xff:
        response2 = ser.read(2)
        if len(response2) == 0:
            raise TimeoutError("Error: timeout")
        if len(response2) != 2:
            raise CommunicationError("Error: the response indicated a size of 255 and then the second response is not 2 bytes long")
        payload_size = response2[0] + (response2[1] << 8)
        if verbose == 2:
            print(format_debug(f"Received an extended size: {payload_size}"))
    if payload_size == 0:
        if response[1] != 0:
            raise CommunicationError(f"Error: the second byte should be 0 if there is no payload, instead it is: {response[1]}")
    else:
        if response[1] != 1:
            raise CommunicationError(f"Error: the second byte should be 1 if there is a payload, instead it is: {response[1]}")

    if payload_size == 0:
        if verbose == 2:
            print(format_success("This response indicates SUCCESS and has no payload"))
        return b''

    payload = ser.read(payload_size)
    if len(payload) != payload_size:
        raise PayloadError(f"Error: didn't receive the right length ({payload_size} bytes) payload. Received ({len(payload)} bytes): {payload}")

    if verbose == 2:
        print(format_debug("Got a valid payload:"))
        print_data("Got a valid payload:", payload, verbose=verbose)

    return payload

def sniffer():
    n_bytes_received = 0
    while True:
        new_byte = ser.read(1)
        if len(new_byte) != 1:
            if n_bytes_received > 0:
                print(format_warning("---------------------------------------------------------------"))
                print(format_warning("Timed out after incomplete communication. Here is what we know:"))
                print(format_info(f"   Received {n_bytes_received} bytes"))
                if n_bytes_received >= 1:
                    if alias >= 33 and alias <= 126:
                        alias = chr(alias)
                    else:
                        alias = str(alias)
                    print(format_info(f"   Alias: {alias}"))
                if n_bytes_received >= 2:
                    print(format_info(f"   Command ID: {command_id}"))
                if n_bytes_received >= 3:
                    print(format_info(f"   Payload length: {payload_len}"))
                if n_bytes_received >= 4:
                    print(format_info(f"   Payload: {payload}"))
                print(format_warning("---------------------------------------------------------------"))
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

def flush_receive_buffer():
    ser.reset_input_buffer()

def send_command(command_id, gathered_inputs, verbose=2):
    command_payload_len = len(gathered_inputs)
    command = bytearray([alias, command_id, command_payload_len]) + gathered_inputs
    if verbose == 2:
        print_data("Sending command: ", command, print_size=True, verbose=verbose)
    ser.write(command)
    if (alias == 255) and (command_id != detect_devices_command_id) and (command_id != set_device_alias_command_id):
        if verbose == 2:
            print(format_info(f"Sending a command to all devices (alias {alias}) and we expect there will be no response from any of them"))
        return []
    for command_index, item in enumerate(registered_commands):
        if command_id == item["CommandEnum"]:
            break
    if registered_commands[command_index]["Output"] == []:
        if verbose == 2:
            print(format_info("This command is expected to not return any response"))
        return []
    all_response_payloads = []
    while(1):
        try:
            response_payload = get_response(verbose=verbose)
            all_response_payloads.append(response_payload)
        except TimeoutError:
            if alias == 255: # we are sending a command to all devices and so we are expecting to get no response and rather a timeout
                break
            if registered_commands[command_index]["MultipleResponses"] == True: # check if this commmand may have any number of responses (including none)
                break
            # There is no legitamate reason that we should get a timeout here, so we need to raise this Timeout error again
            raise TimeoutError("Error: timeout")
        if registered_commands[command_index]["MultipleResponses"] == False:
            break
    return all_response_payloads

def string_to_u8_alias(input):
    if input == "255":
        converted_input = 255
    elif len(input) == 1:
        converted_input = ord(input)
    else:
        try:
            converted_input = int(input)
        except ValueError:
            print(format_error("Error: it is not a single character nor is it an integer"))
            exit(1)
        if converted_input < 0 or converted_input > 255:
            print(format_error("Error: it is not within the allowed range. The allowed range is 0 to 255"))
            exit(1)
    if converted_input == RESPONSE_CHARACTER:
        print(format_error(f"Error: the alias {RESPONSE_CHARACTER} is not allowed because it is reserved to indicate a response"))
        exit(1)
    return converted_input

def string_to_u64_unique_id(input):
    try:
        converted_input = int(input, 16)
    except ValueError:
        print(format_error(f"Error: cannot convert the hexadecimal number {input} to an integer"))
        exit(1)
    if converted_input < 0 or converted_input > 0xFFFFFFFFFFFFFFFF:
        print(format_error("Error: it is not within the allowed range. The allowed range is 0 to 0xFFFFFFFFFFFFFFFF"))
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
        print(format_error("Error: you must specify an alias with the -a option. Run this command with -h to get more detailed help."))
        exit(1)

    alias = string_to_u8_alias(alias)
    print(format_info(f"The alias of the device we want to communicate with is {get_human_readable_alias(alias)}"))

def set_data_types_and_command_data(new_data_types, new_registered_commands):
    global registered_data_types
    global registered_commands
    global detect_devices_command_id
    global set_device_alias_command_id
    registered_data_types = new_data_types
    registered_commands = new_registered_commands
    detect_devices_command_id = get_command_id("Detect devices")
    set_device_alias_command_id = get_command_id("Set device alias")

def open_serial_port(timeout = 1.2):
    global ser
    ser = serial_functions.open_serial_port(serial_port, 230400, timeout = timeout)

def close_serial_port():
    ser.close
    print(format_info("Closed the serial port"))

def input_or_response_to_string(data):
    data_type_id = data.data_type_id
    data_type_str = registered_data_types[data_type_id].data_type_str
    data_description = data.description
    return f"{data_type_str}: {data_description}"

def print_wrapped_text(first_line_message, subsequent_lines_message, text, terminal_window_columns):
    wrapped_text = textwrap.wrap(text, initial_indent = first_line_message, subsequent_indent = subsequent_lines_message, width = terminal_window_columns)
    for line in wrapped_text:
        print(format_info(line))

def print_inputs_or_responses(message, inputs_or_responses, terminal_window_columns):
    subsequent_spaces = 12
    if inputs_or_responses == None or (isinstance(inputs_or_responses, list) and len(inputs_or_responses) == 0):
        print(format_info(f"{message} None"))
    elif isinstance(inputs_or_responses, list):
        for i in range(len(inputs_or_responses)):
            text = input_or_response_to_string(inputs_or_responses[i])
            print_wrapped_text(message, " " * subsequent_spaces, text, terminal_window_columns)
    elif isinstance(inputs_or_responses, tuple):
        text = input_or_response_to_string(inputs_or_responses)
        print_wrapped_text(message, " " * subsequent_spaces, text, terminal_window_columns)
    else:
        print(format_error(f"THIS IS NOT A LIST NOR A TUPLE. IT IS A: {type(inputs_or_responses)} AND THE CONTENT IS: {inputs_or_responses}"))
        print(format_error(f"Error: this is not correctly formatted: {inputs_or_responses}"))

def print_protocol_version():
    print(format_info(f"Currently using protocol version v{PROTOCOL_VERSION}"))

def print_data_type_descriptions():
    print(format_info("\nThese are the descriptions of the various input and output data types currently supported:\n"))
    header1 = "   Data type ID   "
    header2 = "Size (bytes)"
    header3 = "      Max Value      "
    header4 = "      Min Value      "
    header5 = "Description"
    header_format_string = "%%%ds | %%%ds | %%%ds | %%%ds | %%%ds" % (len(header1), len(header2), len(header3), len(header4), len(header5))
    data_format_string = "%%%ds | %%%ds | %%%ds | %%%ds | %%%ds" % (len(header1), len(header2), len(header3), len(header4), len(header5))
    header = header_format_string % (header1, header2, header3, header4, header5)
    
    # Print header with blue color
    print(format_info(header))
    header_len = len(header)
    print(format_info("-" * header_len))

    for data_type_id in registered_data_types.keys():
        data_type_str = registered_data_types[data_type_id].data_type_str
        data_size = registered_data_types[data_type_id].size
        if(data_size != None):
            data_size = str(data_size)
        else:
            data_size = "Variable"
        data_type_is_integer = registered_data_types[data_type_id].is_integer
        if(data_type_is_integer):
            data_max_value = str(registered_data_types[data_type_id].max_value)
            data_min_value = str(registered_data_types[data_type_id].min_value)
        else:
            data_max_value = "Not Applicable"
            data_min_value = "Not Applicable"
        data_description = registered_data_types[data_type_id].description
        print(format_info(data_format_string % (data_type_str, data_size, data_max_value, data_min_value, data_description)))

def print_registered_commands():
    print(format_info("\nThese are the supported commands:\n"))
    terminal_window_columns = shutil.get_terminal_size().columns
    for item in registered_commands:
        command_id = item["CommandEnum"]
        print(format_info(f"{command_id:3d}: {item['CommandString']}"))
        print_wrapped_text("     ", "     ", item["Description"], terminal_window_columns)
        inputs = item["Input"]
        print_inputs_or_responses("     Input: ", inputs, terminal_window_columns)
        response = item["Output"]
        print_inputs_or_responses("     Response: ", response, terminal_window_columns)
        print()

def lowercase_no_space_no_underscore(input):
    return input.lower().replace(" ", "").replace("_", "")

def get_command_id(command_str):
    for item in registered_commands:
        if lowercase_no_space_no_underscore(command_str) == lowercase_no_space_no_underscore(item["CommandString"]):
            return item["CommandEnum"]
    return None

def list_2d_string_to_packed_bytes(input, verbose=2):
    # convert this string representation of a python two dimensional list to a python list of lists
    try:
        input_list_2d = eval(input)
    except NameError:
        print(format_error("Error: the input could not be successfully translated using the eval function in Python"))
        exit(1)
    # check that the shape of this list is correct (ie. is a 2D list or in other words a list of lists)
    if not isinstance(input_list_2d, list):
        print(format_error("Error: the input could not be translated into a valid list object"))
        exit(1)
    if len(input_list_2d) < 1:
        print(format_error("Error: the provided input appears to be an empty list"))
        exit(1)
    is_signed = True
    input_packed = b''
    for list_item in input_list_2d:
        if verbose >= 2:
            print(format_debug(f"Here is a list item: {list_item}"))
        if not isinstance(list_item, list):
            print(format_error("Error: the sub-item in the list could not be translated into a valid list object"))
            exit(1)
        if len(list_item) != 2:
            print(format_error("Error: the sub-item in the list is not a list with two elements in it"))
            exit(1)
        for list_sub_item in list_item:
            if not isinstance(list_sub_item, int):
                print(format_error("Error: the sub-sub-item in the list is not an integer"))
                exit(1)
            try:
                input_packed = input_packed + list_sub_item.to_bytes(4, byteorder="little", signed=is_signed)
            except OverflowError:
                print(format_error("Error: the integer value is too large to be represented in the given number of bytes"))
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
        print(format_error(f"Error: the input is not a string nor is it a bytearray. It is a: {type(input)}"))
        exit(1)
    # check that the length of the bytearray is correct
    if len(input_packed) != 10:
        print(format_error("Error: the input is not 10 bytes long"))
        exit(1)
    return input_packed

def convert_input_to_right_type(data_type_id, input, input_data_size, is_integer, input_data_min_value, input_data_max_value, verbose=2):
    if is_integer:
        try:
            input_int = int(input)
        except ValueError:
            print(format_error("Error: the input is not an integer"))
            exit(1)
        if input_int < input_data_min_value or input_int > input_data_max_value:
            print(format_error("Error: the input is not within the allowed range"))
            print(format_error(f"The allowed range is: {input_data_min_value} to {input_data_max_value}"))
            exit(1)
        if input_data_min_value < 0:
            input_signed = True
        else:
            input_signed = False
        input_packed = input_int.to_bytes(input_data_size, byteorder = "little", signed = input_signed)
        if verbose:
            print_data("The converted input is:", input_packed, verbose=verbose)
    else:
        if registered_data_types[data_type_id].data_type_str == "u8_alias":
            if isinstance(input, str):
                input_packed = string_to_u8_alias(input).to_bytes(1, byteorder = "little")
            else:
                if input < 0 or input > 255:
                    print(format_error("Error: the input is not within the allowed range"))
                    print(format_error("The allowed range is: 0 to 255"))
                    exit(1)
                input_packed = input.to_bytes(1, byteorder = "little")
        elif registered_data_types[data_type_id].data_type_str == "list_2d":
            input_packed = list_2d_string_to_packed_bytes(input)
        elif registered_data_types[data_type_id].data_type_str == "buf10":
            input_packed = buf10_to_packed_bytes(input)
        elif registered_data_types[data_type_id].data_type_str == "u64_unique_id":
            # first, check if this input is a string. if it is, then it must be 16 characters long and be a hexadecimal number
            if isinstance(input, str):
                input_packed = string_to_u64_unique_id(input).to_bytes(8, byteorder = "little")
            else:
                # if is is not a string then it must be an integer and must fit into a 64-bit unsigned integer
                if input < 0 or input > 0xFFFFFFFFFFFFFFFF:
                    print(format_error(f"Error: the input ({input}) is not within the allowed range"))
                    print(format_error("The allowed range is: 0 to 0xFFFFFFFFFFFFFFFF"))
                    exit(1)
                input_packed = input.to_bytes(8, byteorder = "little")
        else:
            print(format_error(f"Error: didn't yet implement a converter to handle the input type: {registered_data_types[data_type_id].data_type_str}"))
            exit(1)
    return input_packed

def gather_inputs(command_id, inputs, verbose=2):
    if verbose == 2:
        print(format_info(f"Gathering inputs for command {command_id}"))
    for command_index, item in enumerate(registered_commands):
        if command_id == item["CommandEnum"]:
            break
    expected_inputs = registered_commands[command_index]["Input"]
    if len(expected_inputs) == 0:
        if verbose == 2:
            print(format_info("This command takes no inputs"))
    else:
        if len(expected_inputs) == 1:
            if verbose == 2:
                print(format_info("The expected input for this command is:")),
        else:
            if verbose == 2:
                print(format_info("The expected inputs for this command are:")),
        for i in range(len(expected_inputs)):
            input_text = input_or_response_to_string(expected_inputs[i])
            if verbose == 2:
                print(format_info(f"   {input_text}"))
    if verbose == 2:
        print(format_info(f"{len(inputs)} input(s) were given"))
    if len(inputs) != len(expected_inputs):
        print(format_error("Error: the number of inputs given for this command is not right. Check above to see what this command is expecting."))
        exit(1)
    if verbose == 2:
        print(format_success("You gave the correct number of inputs."))
    
    concatenated_inputs = bytearray()
    for i in range(len(inputs)):
        data_type_id = expected_inputs[i].data_type_id
        input_data_size = registered_data_types[data_type_id].size
        input_data_is_integer = registered_data_types[data_type_id].is_integer
        if input_data_is_integer:
            input_data_min_value = registered_data_types[data_type_id].min_value
            input_data_max_value = registered_data_types[data_type_id].max_value
        else:
            input_data_min_value = None
            input_data_max_value = None
        input_data_type_string = registered_data_types[data_type_id].data_type_str
        input_data_type_description = registered_data_types[data_type_id].description
        if verbose == 2:
            print(format_info(f"Converting input number {i + 1} ({inputs[i]})"))
        converted_data = convert_input_to_right_type(data_type_id, inputs[i], input_data_size, input_data_is_integer, input_data_min_value, input_data_max_value, verbose=verbose)
        if converted_data == None:
            print(format_error("Error: bad input. This input must follow this convention:"))
            print(format_error(f"  {input_data_type_string}: {input_data_type_description} (size: {input_data_size} unpack type: {input_data_min_value})"))
            exit(1)
        concatenated_inputs += converted_data
    return concatenated_inputs

def interpret_single_response(command_id, response, verbose=2):
    parsed_response = []
    if response == None:
        if verbose >= 1:
            print(format_info("This command did not return a response"))
        return parsed_response
    for command_index, item in enumerate(registered_commands):
        if command_id == item["CommandEnum"]:
            break
    expected_response = registered_commands[command_index]["Output"]
    if len(expected_response) == 0:
        if response != None:
            print(format_error("Error: We were expecting this command to have no response whatsoever, but we got a response"))
            exit(1)
        if verbose >= 1:
            print(format_success("This command produced no response, which is exactly as expected"))
    elif len(expected_response) == 1 and registered_data_types[expected_response[0].data_type_id].data_type_str == "success_response":
        if response != b'':
            raise CommunicationError("Error: the response was not the expected success response")
        if verbose >= 1:
            print(format_success("We got the success response. Good. The response payload is empty as expected."))
    else:
        if verbose >= 1:
            print(format_info("The response for this command along with the decoded value(s) is below:")),
        for i in range(len(expected_response)):
            response_text = input_or_response_to_string(expected_response[i])
            if verbose >= 1:
                print(format_info(f"   {response_text}"))
            data_type_id = expected_response[i].data_type_id
            data_type_str = registered_data_types[data_type_id].data_type_str
            if data_type_str == "string_null_term":
                # find the first occurance of the null terminator in the byte array response
                null_terminator_index = response.find(b'\x00')
                # if it didn't fina a null terminator then throw an error
                if null_terminator_index == -1:
                    raise CommunicationError("Error: the response from the device is not null terminated. This is a bug in the device or some sort of communication error")
                data_type_size = null_terminator_index + 1
            elif data_type_str == "general_data":
                data_type_size = len(response)
            else:
                data_type_size = registered_data_types[data_type_id].size
            if len(response) < data_type_size:
                raise CommunicationError("Error: the response does not contain enough bytes to decode this data type")
            data_item = response[:data_type_size]
            response = response[data_type_size:]
            data_item_is_integer = registered_data_types[data_type_id].is_integer
            if data_item_is_integer:
                data_item_min_value = registered_data_types[data_type_id].min_value
                data_item_max_value = registered_data_types[data_type_id].max_value
                if data_item_min_value < 0:
                    data_item_signed = True
                else:
                    data_item_signed = False
                from_bytes_result = int.from_bytes(data_item, byteorder = "little", signed = data_item_signed)
                parsed_response.append(from_bytes_result)
                if verbose >= 1:
                    print(format_info(f"   ---> {from_bytes_result}"))
            else:
                if data_type_str == "string8":
                    # remove the null terminator from the string
                    data_item = data_item[:-1]
                    data_item = data_item.decode("utf-8")
                    parsed_response.append(data_item)
                    if verbose >= 1:
                        print(format_info(f"   ---> {data_item}"))
                elif data_type_str == "string_null_term":
                    data_item = data_item.decode("utf-8")
                    parsed_response.append(data_item)
                    if verbose >= 1:
                        print(format_info(f"   ---> {data_item}"))
                elif data_type_str == "u24_version_number":
                    parsed_response.append([data_item[0], data_item[1], data_item[2]])
                    if verbose >= 1:
                        print(format_info(f"   ---> {data_item[2]}.{data_item[1]}.{data_item[0]}"))
                elif data_type_str == "u32_version_number":
                    parsed_response.append([data_item[0], data_item[1], data_item[2], data_item[3]])
                    if verbose >= 1:
                        print(format_info(f"   ---> {data_item[3]}.{data_item[2]}.{data_item[1]}.{data_item[0]}"))
                elif data_type_str == "u64_unique_id":
                    from_bytes_result = int.from_bytes(data_item, byteorder = "little")
                    parsed_response.append(from_bytes_result)
                    if verbose >= 1:
                        print(format_info(f"   ---> {from_bytes_result:016X}"))
                elif data_type_str == "u8_alias":
                    from_bytes_result = int.from_bytes(data_item, byteorder = "little")
                    parsed_response.append(from_bytes_result)
                    if verbose >= 1:
                        if from_bytes_result >= 33 and from_bytes_result <= 126:
                            print(format_info(f"   ---> the ASCII character {chr(from_bytes_result)} (or the decimal number {from_bytes_result})"))
                        else:
                            print(format_info(f"   ---> the single byte integer {from_bytes_result} or 0x{from_bytes_result:02x} in hex"))
                elif data_type_str == "crc32":
                    from_bytes_result = int.from_bytes(data_item, byteorder = "little")
                    parsed_response.append(from_bytes_result)
                    if verbose >= 1:
                        print(format_info(f"   ---> 0x{from_bytes_result:08X}"))
                elif data_type_str == "buf10":
                    parsed_response.append(data_item)
                    if verbose >= 1:
                        print(format_info(f"   ---> {data_item}"))
                elif data_type_str == "general_data":
                    parsed_response.append(data_item)
                    if verbose >= 1:
                        for d in data_item:
                            print(format_info(f"   ---> {d} (0x{d:02x})"))
                else:
                    raise CommunicationError(f"Error: the interpretation of this data type is not implemented: {data_type_str}")
        if len(response) != 0:
            raise CommunicationError("Error: there are unexpected bytes left in the response after interpreting the expected response")
    return parsed_response

def interpret_response(command_id, response, verbose=2):
    parsed_response = []
    if len(response) == 0:
        if verbose >= 1:
            print(format_info("There was no response from any device(s)"))
    for i in range(len(response)):
        partial_parsed_response = interpret_single_response(command_id, response[i], verbose=verbose)
        parsed_response.append(partial_parsed_response)
    return parsed_response

def execute_command(_alias, command_id_or_str, inputs, verbose=2):
    global alias
    alias = _alias
    if isinstance(command_id_or_str, int):
        command_id = command_id_or_str
        command_str = registered_commands[command_id]["CommandString"]
    else:
        command_id = get_command_id(command_str)
        command_str = command_id_or_str
    if command_id == None:
        print(format_error(f"ERROR: The command {command_id_or_str} is not supported"))
        exit(1)
    if verbose >= 1:
        print(format_info(f"The command is: {command_str} and it has ID {command_id}"))
    gathered_inputs = gather_inputs(command_id, inputs, verbose=verbose)
    response = send_command(command_id, gathered_inputs, verbose=verbose)
    parsed_response = interpret_response(command_id, response, verbose=verbose)
    return parsed_response

def read_raw_byte():
    return ser.read(1)