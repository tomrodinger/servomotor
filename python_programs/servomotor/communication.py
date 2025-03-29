#!/usr/local/bin/python3

from .vendor.serial.serialutil import to_bytes
import shutil
from . import serial_functions
import textwrap
import sys
import os
import struct

# Import our terminal formatting module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from terminal_formatting import format_error, format_info, format_warning, format_success, format_debug, STYLE

# Protocol constants for packet format
FIRST_BYTE_LSB_MASK = 0x01  # Mask for checking if LSB is set
FIRST_BYTE_SHIFT = 1        # Number of bits to shift for first byte interpretation
DECODED_FIRST_BYTE_EXTENDED_SIZE = 127  # Value indicating extended size format

# Reserved aliases with special meaning. All other values are normal device aliases that can be assigned freely to devices so that they can be individually addressed with just one byte
ALL_ALIAS = 255            # to address all devices on the bus at the same time
EXTENDED_ADDRESSING = 254  # indicates that we will use extended addressing
RESPONSE_CHARACTER_CRC32_ENABLED = 253   # indicates that the response is coming from the device being addressed and the response has a CRC32 appended
RESPONSE_CHARACTER_CRC32_DISABLED = 252   # indicates that the response is coming from the device being addressed and the response does not have a CRC32 appended

# Protocol constants
serial_port = None
PROTOCOL_VERSION = 20
registered_data_types = None
registered_commands = None
ser = None
alias = ALL_ALIAS  # Default to addressing all devices
unique_id = None   # Default to not using extended addressing
detect_devices_command_id = None
set_device_alias_command_id = None

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

# Encode a device ID by shifting left and setting LSB to 1
def encode_first_byte(device_id):
    return (device_id << FIRST_BYTE_SHIFT) | FIRST_BYTE_LSB_MASK

# Decode a device ID by shifting right by one bit
def decode_first_byte(encoded_id):
    return encoded_id >> FIRST_BYTE_SHIFT

# Check if a device ID has the correct format (LSB set to 1)
def is_valid_first_byte_format(device_id):
    return (device_id & FIRST_BYTE_LSB_MASK) == FIRST_BYTE_LSB_MASK

def print_data(prefix_message, data, print_size=True, verbose=2):
    if verbose == 2:
        message = prefix_message
        for d in data:
            message += f"0x{d:02X} "
        if print_size:
            message += f"[{len(data)} bytes]"
        print(format_debug(message))

def get_human_readable_alias(alias):
    if alias is None:
        return "The alias is not specified"
    elif alias >= 33 and alias <= 126:
        alias_str = f"{chr(alias)} ({alias})"
    else:
        alias_str = f"{alias} (0x{alias:02x})"
    return alias_str

def get_response(crc32_enabled=True, verbose=2):
    # Read the first byte (size byte)
    first_byte = ser.read(1)
    if len(first_byte) != 1:
        raise TimeoutError("Error: timeout while reading first byte")
    
    # Validate first byte format (LSB should be 1)
    if not is_valid_first_byte_format(first_byte[0]):
        raise CommunicationError(f"Error: first byte {first_byte[0]} does not have LSB set to 1")

    # Decode the size from the first byte
    packet_size = decode_first_byte(first_byte[0])
    size_bytes = first_byte
    
    # Check if we have extended size format
    if packet_size == DECODED_FIRST_BYTE_EXTENDED_SIZE:
        # Read the next two bytes for extended size
        ext_size_bytes = ser.read(2)
        if len(ext_size_bytes) != 2:
            raise CommunicationError("Error: couldn't read extended size bytes")
        
        size_bytes += ext_size_bytes
        packet_size = ext_size_bytes[0] | (ext_size_bytes[1] << 8)
        if verbose == 2:
            print(format_debug(f"Received packet with extended size: {packet_size}"))

    # Read the rest of the packet (minus the size byte(s) we already read)
    remaining_bytes = packet_size - len(size_bytes)
    if remaining_bytes <= 0:
        raise CommunicationError(f"Error: there are less bytes than expected in the response")
    packet_data = ser.read(remaining_bytes)

    if len(packet_data) != remaining_bytes:
        raise CommunicationError(f"Error: received {len(packet_data)} bytes, expected {remaining_bytes}")
    
    # Full packet for CRC calculation
    full_packet = size_bytes + packet_data

    if verbose == 2:
        print_data("Received packet: ", full_packet, print_size=True, verbose=verbose)
    
    # Check if first address byte is one of the response characters
    if packet_data[0] == RESPONSE_CHARACTER_CRC32_ENABLED:
        received_crc32_enabled = 1
    elif packet_data[0] == RESPONSE_CHARACTER_CRC32_DISABLED:
        received_crc32_enabled = 0
    else:
        error_text = f"Error: the address byte (which should indicate a response, {packet_data[0]}) is not one of the response characters: {RESPONSE_CHARACTER_CRC32_ENABLED} or {RESPONSE_CHARACTER_CRC32_DISABLED}"
        raise CommunicationError(error_text)
    
    # Extract payload (everything after address and command bytes)
    # If CRC32 is enabled, the last 4 bytes are the CRC32
    if received_crc32_enabled:
        # Verify CRC32
        if len(packet_data) < 4:
            raise CommunicationError("Error: packet too small to contain CRC32")
        
        # Extract CRC32 from the end of the packet
        received_crc32 = struct.unpack('<I', packet_data[-4:])[0]
        
        # Calculate CRC32 on all bytes except the CRC32 itself
        calculated_crc32 = calculate_crc32(size_bytes + packet_data[:-4])
        
        if calculated_crc32 != received_crc32:
            raise CommunicationError(f"CRC32 validation failed: calculated {calculated_crc32:08X}, received {received_crc32:08X}")
        
        # Payload is everything after address and command bytes, but before CRC32
        payload = packet_data[2:-4]
    else:
        # Payload is everything after address and command bytes
        payload = packet_data[2:]
    
    if verbose == 2:
        if len(payload) == 0:
            print(format_success("This response indicates SUCCESS and has no payload"))
        else:
            print(format_debug("Got a valid payload:"))
            print_data("Payload:", payload, verbose=verbose)
    if crc32_enabled == True and received_crc32_enabled == False:
        print(format_warning("The outgoing message had a CRC32 appended but the response coming back did not have a CRC32 appended"))
    if crc32_enabled == False and received_crc32_enabled == True:
        print(format_warning("The outgoing message did not have a CRC32 appended but the response coming back did have a CRC32 appended"))
    return payload

def sniffer(crc32_enabled=True):
    """Sniff RS485 traffic and decode packets according to the protocol"""
    packet_data = bytearray()
    address_byte = None
    command_byte = None
    payload = None
    packet_size = None
    extended_size = False
    
    # Read first byte (size byte)
    first_byte = ser.read(1)
    if len(first_byte) != 1:
        print(format_warning("No data received (timeout)"))
        return None, None, None
    
    # Check if first byte has LSB set to 1
    if not is_valid_first_byte_format(first_byte[0]):
        print(format_error(f"Invalid first byte format: 0x{first_byte[0]:02X} (LSB not set to 1)"))
        return None, None, None
    
    # Decode size from first byte
    packet_size = decode_first_byte(first_byte[0])
    packet_data.append(first_byte[0])
    
    # Check for extended size format
    if packet_size == DECODED_FIRST_BYTE_EXTENDED_SIZE:
        extended_size = True
        # Read two more bytes for extended size
        ext_size_bytes = ser.read(2)
        if len(ext_size_bytes) != 2:
            print(format_error("Timeout while reading extended size bytes"))
            return None, None, None
        
        packet_data.extend(ext_size_bytes)
        packet_size = ext_size_bytes[0] | (ext_size_bytes[1] << 8)
        print(format_info(f"Extended size packet: {packet_size} bytes"))
    else:
        print(format_info(f"Standard size packet: {packet_size} bytes"))
    
    # Calculate remaining bytes to read
    remaining_bytes = packet_size - len(packet_data)
    
    # Read the rest of the packet
    remaining_data = ser.read(remaining_bytes)
    if len(remaining_data) != remaining_bytes:
        print(format_error(f"Incomplete packet: expected {remaining_bytes} more bytes, got {len(remaining_data)}"))
        return None, None, None
    
    packet_data.extend(remaining_data)
    
    # Extract address byte
    address_byte = remaining_data[0]
    
    # Extract command byte
    command_byte = remaining_data[1]
    
    # Extract payload
    if crc32_enabled:
        # Last 4 bytes are CRC32
        if len(remaining_data) >= 6:  # address + command + at least 1 payload byte + 4 CRC bytes
            payload = remaining_data[2:-4]
            crc32_bytes = remaining_data[-4:]
            received_crc32 = struct.unpack('<I', crc32_bytes)[0]
            
            # Calculate CRC32 on all bytes except the CRC32 itself
            calculated_crc32 = calculate_crc32(packet_data[:-4])
            
            if calculated_crc32 != received_crc32:
                print(format_error(f"CRC32 validation failed: calculated 0x{calculated_crc32:08X}, received 0x{received_crc32:08X}"))
            else:
                print(format_success(f"CRC32 validation passed: 0x{received_crc32:08X}"))
        else:
            payload = bytearray()
            print(format_warning("Packet too small to contain CRC32"))
    else:
        # No CRC32, payload is everything after address and command
        payload = remaining_data[2:]
    
    # Print packet information
    print(format_info(f"Address: {address_byte} (0x{address_byte:02X})"))
    print(format_info(f"Command: {command_byte} (0x{command_byte:02X})"))
    print(format_info(f"Payload: {len(payload)} bytes"))
    if len(payload) > 0:
        print_data("Payload data: ", payload, print_size=True, verbose=2)
    
    return address_byte, command_byte, payload

def flush_receive_buffer():
    ser.reset_input_buffer()

def calculate_crc32(data):
    """Calculate CRC32 checksum for a byte array"""
    import zlib
    return zlib.crc32(data) & 0xffffffff

def send_command(command_id, gathered_inputs, crc32_enabled=True, verbose=2):
    # Check if we're using extended addressing
    if unique_id is not None:
        # Extended addressing mode
        # Format: address byte (EXTENDED_ADDRESSING) + 8 bytes unique ID + command byte + payload
        address_part = struct.pack('<BQ', EXTENDED_ADDRESSING, unique_id)
    else:
        # Standard addressing mode
        # Format: address byte (alias) + command byte + payload
        address_part = struct.pack('<B', alias)
    
    # Add command byte and payload
    command_part = struct.pack('<B', command_id) + gathered_inputs
    
    # Combine address and command parts
    packet_content = address_part + command_part

    # Calculate total packet size
    packet_size = 1 + len(packet_content)  # +1 for the size byte itself
    if crc32_enabled:
        packet_size += 4 # the CRC32 is 4 extra bytes

    # Check if we need extended size format (size > 127)
    if packet_size > 127:
        packet_size += 2 # since we cannot encode the size in just 1 byte, we will use extended size format, which is a total of 3 bytes (so add two more bytes)
        # Extended size format: first byte = 127 (encoded), followed by 2-byte size
        size_bytes = struct.pack('<BH', encode_first_byte(127), packet_size)
        packet = size_bytes + packet_content
    else:
        # Standard size format: first byte = encoded size
        packet = bytearray([encode_first_byte(packet_size)]) + packet_content

    # Add CRC32 if enabled
    if crc32_enabled:
        if verbose >= 2:
            print_data("Calculating CRC32 of: ", packet, print_size=True, verbose=verbose)
        crc32_value = calculate_crc32(packet)
        packet += struct.pack('<I', crc32_value)    
        
    if verbose == 2:
        print_data("Sending packet: ", packet, print_size=True, verbose=verbose)
    ser.write(packet)
    
    # Check if we should expect a response
    if (unique_id is None) and (alias == ALL_ALIAS) and (command_id != detect_devices_command_id) and (command_id != set_device_alias_command_id):
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
            response_payload = get_response(crc32_enabled=crc32_enabled, verbose=verbose)
            all_response_payloads.append(response_payload)
        except TimeoutError:
            if unique_id is None and alias == ALL_ALIAS:  # we are sending a command to all devices and so we are expecting to get no response and rather a timeout
                break
            if registered_commands[command_index]["MultipleResponses"] == True:  # check if this command may have any number of responses (including none)
                break
            # There is no legitimate reason that we should get a timeout here, so we need to raise this Timeout error again
            raise TimeoutError("Error: timeout")
        if registered_commands[command_index]["MultipleResponses"] == False:
            break
    return all_response_payloads

def string_to_u8_alias(input):
    if input == str(ALL_ALIAS):
        converted_input = ALL_ALIAS
    elif len(input) == 1:
        converted_input = ord(input)
    else:
        try:
            converted_input = int(input)
        except ValueError:
            print(format_error("Error: it is not a single character nor is it an integer"))
            exit(1)
        if converted_input < 0 or converted_input > ALL_ALIAS:
            print(format_error(f"Error: The alias (if not using extended addressing) is not within the allowed range. The allowed range is 0 to {ALL_ALIAS}"))
            exit(1)
    # Check for reserved values
    if converted_input == RESPONSE_CHARACTER_CRC32_ENABLED:
        print(format_error(f"Error: the alias {RESPONSE_CHARACTER_CRC32_ENABLED} is not allowed because it is reserved to indicate a response with a CRC32 check appended"))
        exit(1)
    if converted_input == RESPONSE_CHARACTER_CRC32_DISABLED:
        print(format_error(f"Error: the alias {RESPONSE_CHARACTER_CRC32_DISABLED} is not allowed because it is reserved to indicate a response without a CRC32 check appended"))
        exit(1)
    if converted_input == EXTENDED_ADDRESSING:
        print(format_error(f"Error: the alias {EXTENDED_ADDRESSING} is not allowed because it is reserved for extended addressing"))
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

def string_to_alias_or_unique_id(input_str):
    """Convert the input string to either an alias or a unique ID based on its format"""
    if len(input_str) >= 4:
        # The user probably has intended this to be a a unique ID (16-character hex string) given the length of the input string
        return None, string_to_u64_unique_id(input_str)
    else:
        # It's a probably a regular alias since the length of the string is short
        return string_to_u8_alias(input_str), None

def set_serial_port_from_args(args):
    global serial_port

    if args.PORT == True:
        serial_port = "MENU"
    else:
        serial_port = args.port

def set_standard_options_from_args(args):
    global alias, unique_id
    
    set_serial_port_from_args(args)
    
    # Determine verbosity level
    verbose = 1  # Default verbosity level
    if hasattr(args, 'verbose_level') and args.verbose_level is not None:
        verbose = args.verbose_level
    elif hasattr(args, 'verbose') and args.verbose:
        verbose = 2
    
    if args.alias is None:
        print(format_error("Error: you must specify an alias with the -a option. Run this command with -h to get more detailed help."))
        exit(1)
    else:
        # Determine if the alias is actually a unique ID
        alias_val, unique_id_val = string_to_alias_or_unique_id(args.alias)
        
        if unique_id_val is not None:
            # It's a unique ID
            unique_id = unique_id_val
            alias = None
            if verbose >= 1:
                print(format_info(f"Using extended addressing with unique ID: 0x{unique_id:016X}"))
        else:
            # It's a regular alias
            alias = alias_val
            unique_id = None
            if verbose >= 1:
                print(format_info(f"Using standard addressing with alias: {get_human_readable_alias(alias)}"))

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

def execute_command(_alias, command_id_or_str, inputs, crc32_enabled=True, verbose=2):
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
    response = send_command(command_id, gathered_inputs, crc32_enabled=crc32_enabled, verbose=verbose)
    parsed_response = interpret_response(command_id, response, verbose=verbose)
    return parsed_response

def read_raw_byte():
    return ser.read(1)