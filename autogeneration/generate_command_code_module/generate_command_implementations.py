#!/usr/bin/env python3
"""
Module for generating command method implementations.
This module contains a function that takes command data and returns 
the method implementations for the Servomotor class.
"""

import re


def format_command_name(command_string):
    """
    Format a command string to a camelCase function name using a simple algorithm.
    
    Args:
        command_string: The command string to format
        
    Returns:
        Formatted function name in camelCase
    """
    command_string = command_string.strip()
    command_string = re.sub(r'[^a-zA-Z0-9]+', ' ', command_string)
    words = command_string.split()
    if not words:
        return ''
    
    # First word lowercase, rest capitalized
    return words[0].lower() + ''.join(w.capitalize() for w in words[1:])


def format_response_name(command_string):
    """
    Format a command string to a response structure name.
    
    Args:
        command_string: The command string to format
        
    Returns:
        Formatted response name (e.g., "get status" -> "getStatusResponse")
    """
    return format_command_name(command_string) + 'Response'


def format_payload_name(command_string):
    """
    Format a command string to a payload structure name.
    
    Args:
        command_string: The command string to format
        
    Returns:
        Formatted payload name (e.g., "trapezoid move" -> "trapezoidMovePayload")
    """
    return format_command_name(command_string) + 'Payload'

def format_list_type_name(command_string, converted=False):
    """
    Format a command string to a list type name.
    
    Args:
        command_string: The command string to format
        converted: Whether to return the converted list type
        
    Returns:
        Formatted list type name (e.g., "multi move" -> "multiMoveList_t" with uppercase 'M')
    """
    # Process the command string into camelCase
    command_string = command_string.strip()
    command_string = re.sub(r'[^a-zA-Z0-9]+', ' ', command_string)
    words = command_string.split()
    if not words:
        return ''
    
    # Properly capitalize each word for CamelCase
    processed_name = words[0].lower()
    for word in words[1:]:
        processed_name += word.capitalize()
    
    if converted:
        return f"{processed_name}ListConverted_t"
    else:
        return f"{processed_name}List_t"
        return f"{base_name}List_t"


def generate_command_implementations(commands_data=None, data_types_data=None, **kwargs):
    """
    Generate the command method implementations for Servomotor.cpp.
    
    Args:
        commands_data: List of command dictionaries from motor_commands.json
        data_types_data: List of data type dictionaries from data_types.json
        kwargs: Additional context data (not used by this function)
        
    Returns:
        str: Formatted content for command method implementations
    """
    if commands_data is None:
        return "// No command data available"
        
    # Helper function to generate uniqueId overloaded implementations
    def generate_uniqueid_overload_implementation(cmd, cmd_str, cmd_id, func_name, has_input, has_output, return_type):
        """Generate a ByUniqueId variant implementation"""
        method_lines = []
        
        # Determine parameters with their types
        params = []
        if has_input:
            input_params = cmd.get('Input', [])
            for idx, param in enumerate(input_params):
                param_name = param.get('ParameterName')
                desc = param.get('Description', '')
                match = re.match(r'(\w+):\s*(.*)', desc)
                if match and param_name:
                    type_str = match.group(1)
                    # Get C++ type
                    cpp_type = type_map.get(type_str, 'uint8_t')
                    
                    # Handle special types
                    if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                        list_type_name = format_list_type_name(cmd_str)
                        params.append(f"{list_type_name}* {param_name}")
                        continue
                    
                    # Handle array types
                    if isinstance(cpp_type, tuple):
                        base_type, array_size = cpp_type
                        params.append(f"{base_type} {param_name}[{array_size}]")
                    else:
                        # For unit conversion in wrapper methods
                        if param.get('UnitConversion') and 'Raw' not in func_name:
                            params.append(f"float {param_name}")
                        else:
                            params.append(f"{cpp_type} {param_name}")
        
        # Add method signature - always include uniqueId as first parameter
        params_str = ", ".join(params)
        if params:
            method_lines.append(f"{return_type} Servomotor::{func_name}(uint64_t uniqueId, {params_str}) {{")
        else:
            method_lines.append(f"{return_type} Servomotor::{func_name}(uint64_t uniqueId) {{")
        
        # Add debug output
        method_lines.append(f'    Serial.println("[Motor] {func_name} with uniqueId called.");')
        
        # Add command ID
        method_lines.append(f"    const uint8_t commandID = {cmd_str.upper().replace(' ', '_')};")
        
        # Create payload if needed
        if has_input:
            payload_struct_name = format_payload_name(cmd_str)
            method_lines.append(f"    {payload_struct_name} payload;")
            
            # Copy parameters to payload
            for idx, param in enumerate(cmd.get('Input', [])):
                param_name = param.get('ParameterName')
                if not param_name:
                    continue
                    
                desc = param.get('Description', '')
                match = re.match(r'(\w+):\s*(.*)', desc)
                if match:
                    type_str = match.group(1)
                    # Handle array types
                    if type_str in ['buf10', 'string8', 'firmware_page', 'string_null_term']:
                        method_lines.append(f"    memcpy(payload.{param_name}, {param_name}, sizeof(payload.{param_name}));")
                    elif type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                        method_lines.append(f"    // Copy list data into payload")
                        method_lines.append(f"    for (int i = 0; i < moveCount; i++) {{")
                        method_lines.append(f"        payload.{param_name}[i] = {param_name}[i];")
                        method_lines.append(f"    }}")
                    else:
                        method_lines.append(f"    payload.{param_name} = {param_name};")
            
            # Send command with payload using extended addressing
            method_lines.append(f"    _comm.sendCommandByUniqueId(uniqueId, commandID, (uint8_t*)&payload, sizeof(payload));")
        else:
            # Send command with no payload using extended addressing
            method_lines.append(f"    _comm.sendCommandByUniqueId(uniqueId, commandID, nullptr, 0);")
        
        # Handle response
        if has_output:
            response_struct_name = format_response_name(cmd_str)
            method_lines.append(f"    uint8_t buffer[sizeof({response_struct_name})];")
            method_lines.append(f"    uint16_t receivedSize;")
            method_lines.append(f"    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);")
            method_lines.append(f"    if (_errno == 0) {{")
            method_lines.append(f"        if (receivedSize == sizeof({response_struct_name})) {{")
            method_lines.append(f"            {response_struct_name}* response = ({response_struct_name}*)buffer;")
            method_lines.append(f"            return *response;")
            method_lines.append(f"        }} else {{")
            method_lines.append(f"            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;")
            method_lines.append(f"        }}")
            method_lines.append(f"    }}")
            method_lines.append(f"    {response_struct_name} defaultResponse = {{0}};")
            method_lines.append(f"    return defaultResponse;")
        else:
            method_lines.append(f"    uint16_t receivedSize;")
            method_lines.append(f"    _errno = _comm.getResponse(nullptr, 0, receivedSize);")
        
        # Close method
        method_lines.append("}")
        method_lines.append("")
        
        return "\n".join(method_lines)
    
    # Create a type map for parameter conversion
    type_map = {
        # Basic types
        'i8': 'int8_t',
        'u8': 'uint8_t',
        'i16': 'int16_t',
        'u16': 'uint16_t',
        'i24': 'int32_t',
        'u24': 'uint32_t',
        'i32': 'int32_t',
        'u32': 'uint32_t',
        'i48': 'int64_t',
        'u48': 'uint64_t',
        'i64': 'int64_t',
        'u64': 'uint64_t',
        'float': 'float',
        'double': 'double',
        
        # Array types
        'buf10': ('uint8_t', 10),
        'string8': ('char', 8),
        'string_null_term': ('char', 32),
        'firmware_page': ('uint8_t', 2058),
        
        # Special types
        'u24_version_number': 'uint32_t',
        'u32_version_number': 'uint32_t',
        'u64_unique_id': 'uint64_t',
        'crc32': 'uint32_t'
    }
    
    # Map unit types to conversion function names
    unit_conversion_map = {
        'position': 'convertPosition',
        'time': 'convertTime',
        'velocity': 'convertVelocity',
        'acceleration': 'convertAcceleration',
        'temperature': 'convertTemperature',
        'voltage': 'convertVoltage',
        'current': 'convertCurrent'
    }
    
    # Map endianness conversion functions based on type
    endian_conversion_map = {
        'int8_t': '',
        'uint8_t': '',
        'int16_t': 'htole16',
        'uint16_t': 'htole16',
        'int32_t': 'htole32',
        'uint32_t': 'htole32',
        'int64_t': 'htole64',
        'uint64_t': 'htole64',
    }
    
    # Output implementations
    implementations = []
    
    # Process each command
    for cmd in commands_data:
        cmd_str = cmd['CommandString']
        cmd_id = cmd['CommandEnum']
        func_name = format_command_name(cmd_str)
        
        # Determine if this command has inputs (parameters)
        has_input = bool(cmd.get('Input'))
        
        # Determine if this command has outputs (returns a value)
        has_output = cmd.get('Output') and cmd['Output'] != 'success_response'
        
        # Check if this command has any parameters requiring unit conversion
        needs_unit_conversion = False
        has_converted_response = False
        has_unit_conversion_output = False
        has_mixed_conversion = False
        
        # Check input parameters for UnitConversion
        if cmd.get('Input'):
            for param in cmd['Input']:
                if param.get('UnitConversion'):
                    needs_unit_conversion = True
                    # Check for special mixed conversion type
                    if param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                        has_mixed_conversion = True
                    break
        
        # Check output parameters for UnitConversion
        if has_output and cmd.get('Output'):
            for param in cmd.get('Output', []):
                if param.get('UnitConversion'):
                    needs_unit_conversion = True
                    has_converted_response = True
                    has_unit_conversion_output = True
                    break
        
        # Determine the return type
        return_type = "void"
        if has_output:
            # Generate response type name
            return_type = format_response_name(cmd_str)
            
            # Check if the output is a single parameter with a built-in type
            output_params = cmd.get('Output', [])
            if len(output_params) == 1:
                desc = output_params[0].get('Description', '')
                match = re.match(r'(\w+):\s*(.*)', desc)
                if match:
                    type_str = match.group(1)
                    if type_str in type_map and not isinstance(type_map[type_str], tuple):
                        return_type = type_map[type_str]
        
        # Get wrapper return type if needed
        wrapper_return_type = return_type
        if has_unit_conversion_output:
            if len(cmd.get('Output', [])) == 1:
                # For single values with unit conversion, always return float
                wrapper_return_type = "float"
            else:
                # For multiple values in a struct, use the converted struct type
                wrapper_return_type = return_type.replace('Response', 'ResponseConverted')
        
        # Generate implementations
        if needs_unit_conversion:
            # First generate the Raw implementation
            raw_method = generate_raw_method_implementation(
                cmd, cmd_str, cmd_id, func_name, has_input, has_output,
                return_type, type_map, endian_conversion_map, by_unique_id=False
            )
            implementations.append(raw_method)
            
            # Now generate the uniqueId version of the Raw implementation
            raw_method_uniqueid = generate_raw_method_implementation(
                cmd, cmd_str, cmd_id, func_name, has_input, has_output,
                return_type, type_map, endian_conversion_map, by_unique_id=True
            )
            implementations.append(raw_method_uniqueid)
            
            # Then generate the wrapper implementation
            wrapper_method = generate_wrapper_method_implementation(
                cmd, cmd_str, func_name, has_input, has_output,
                wrapper_return_type, has_unit_conversion_output,
                unit_conversion_map, has_mixed_conversion, type_map, by_unique_id=False
            )
            implementations.append(wrapper_method)
            
            # Now generate the uniqueId version of the wrapper implementation
            wrapper_method_uniqueid = generate_wrapper_method_implementation(
                cmd, cmd_str, func_name, has_input, has_output,
                wrapper_return_type, has_unit_conversion_output,
                unit_conversion_map, has_mixed_conversion, type_map, by_unique_id=True
            )
            implementations.append(wrapper_method_uniqueid)
        else:
            # Generate a single implementation for commands without unit conversion
            simple_method = generate_simple_method_implementation(
                cmd, cmd_str, cmd_id, func_name, has_input, has_output,
                return_type, type_map, endian_conversion_map, by_unique_id=False
            )
            implementations.append(simple_method)
            
            # Generate the uniqueId overloaded version
            uniqueid_method = generate_simple_method_implementation(
                cmd, cmd_str, cmd_id, func_name, has_input, has_output,
                return_type, type_map, endian_conversion_map, by_unique_id=True
            )
            implementations.append(uniqueid_method)
        
        # Generate additional method for commands with multiple responses
        if cmd.get('MultipleResponses'):
            multi_response_method = generate_multi_response_method(cmd_str, func_name, return_type)
            implementations.append(multi_response_method)
    
    return "\n".join(implementations)


def generate_raw_method_implementation(cmd, cmd_str, cmd_id, func_name, has_input, has_output,
                                       return_type, type_map, endian_conversion_map, by_unique_id=False):
    """Generate the raw method implementation for commands that need unit conversion.
    
    Args:
        cmd: Command dictionary from JSON
        cmd_str: Command string
        cmd_id: Command enum ID
        func_name: Function name
        has_input: Whether the command has input parameters
        has_output: Whether the command has output parameters
        return_type: Return type for the function
        type_map: Type mapping dictionary
        endian_conversion_map: Endianness conversion functions
        by_unique_id: Whether to generate the uniqueId overload version
    """
    method_lines = []
    
    # Add method signature
    params = []
    
    # Process input parameters
    if has_input:
        input_params = cmd.get('Input', [])
        for idx, param in enumerate(input_params):
            desc = param.get('Description', '')
            param_name = param.get('ParameterName')
            
            # Extract type and parameter information
            match = re.match(r'(\w+):\s*(.*)', desc)
            if match:
                type_str = match.group(1)
                remainder = match.group(2)
                
                if not param_name:
                    # Simple name extraction
                    words = remainder.strip().split()
                    param_name = words[0] if words else f"param{idx}"
                    param_name = re.sub(r'\W|^(?=\d)', '_', param_name.lower())
                
                # Handle special type for list_2d
                if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                    list_type_name = format_list_type_name(cmd_str)
                    params.append(f"{list_type_name}* {param_name}")
                    continue
                
                # Get C++ type from our type map
                cpp_type = type_map.get(type_str, 'uint8_t')
                
                # Handle array types in parameters
                if isinstance(cpp_type, tuple):
                    base_type, array_size = cpp_type
                    params.append(f"{base_type} {param_name}[{array_size}]")
                else:
                    params.append(f"{cpp_type} {param_name}")
    
    # Add function signature
    params_str = ", ".join(params)
    if by_unique_id:
        if params:
            method_lines.append(f"{return_type} Servomotor::{func_name}Raw(uint64_t uniqueId, {params_str}) {{")
        else:
            method_lines.append(f"{return_type} Servomotor::{func_name}Raw(uint64_t uniqueId) {{")
        
        # Add debug output for uniqueId version
        method_lines.append(f'    Serial.println("[Motor] {func_name}Raw called (by unique ID).");')
    else:
        method_lines.append(f"{return_type} Servomotor::{func_name}Raw({params_str}) {{")
        
        # Add debug output for regular version
        method_lines.append(f'    Serial.println("[Motor] {func_name}Raw called.");')
    
    # Add debug comment about what this method does
    description = cmd.get('Description', 'No description available')
    method_lines.append(f"    // {description} (Raw version)")
    
    # Add command ID
    method_lines.append(f"    const uint8_t commandID = {cmd_str.upper().replace(' ', '_')};")
    
    # Create the payload if needed
    if has_input:
        payload_struct_name = format_payload_name(cmd_str)
        method_lines.append(f"    {payload_struct_name} payload;")
        
        # Set payload values
        for param in cmd.get('Input', []):
            param_name = param.get('ParameterName')
            # Skip parameters that don't have a clear mapping
            if not param_name:
                continue
                
            desc = param.get('Description', '')
            match = re.match(r'(\w+):\s*(.*)', desc)
            if match:
                type_str = match.group(1)
                # Handle special list_2d types
                if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                    list_type_name = format_list_type_name(cmd_str)
                    method_lines.append(f"    // Calculate the actual size of the move list data")
                    method_lines.append(f"    uint16_t {param_name}Size = moveCount * sizeof({list_type_name});")
                    method_lines.append(f"    // Copy list data into payload")
                    method_lines.append(f"    memcpy(payload.{param_name}, {param_name}, {param_name}Size);")
                    continue
                
                # Get C++ type from our type map for endianness conversion
                cpp_type = type_map.get(type_str, 'uint8_t')
                
                # Handle array types in parameters
                if isinstance(cpp_type, tuple):
                    base_type, array_size = cpp_type
                    method_lines.append(f"    // Copy array data into payload")
                    method_lines.append(f"    memcpy(payload.{param_name}, {param_name}, sizeof(payload.{param_name}));")
                else:
                    # Get endianness conversion function
                    endian_func = endian_conversion_map.get(cpp_type, '')
                    if endian_func:
                        method_lines.append(f"    payload.{param_name} = {endian_func}({param_name});")
                    else:
                        method_lines.append(f"    payload.{param_name} = {param_name};")
        
        # Check if we have list parameters that require custom payload size calculation
        has_list_param = False
        list_param_name = ""
        for param in cmd.get('Input', []):
            desc = param.get('Description', '')
            param_name = param.get('ParameterName')
            match = re.match(r'(\w+):\s*(.*)', desc)
            if match:
                type_str = match.group(1)
                if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                    has_list_param = True
                    list_param_name = param_name
                    break
                    
        # Send the command with custom payload size for list parameters
        if has_list_param:
            method_lines.append(f"    // Calculate the actual payload size (just the header plus the used entries)")
            method_lines.append(f"    uint16_t payloadSize = sizeof(payload.moveCount) + sizeof(payload.moveTypes) + {list_param_name}Size;")
            if by_unique_id:
                method_lines.append(f"    _comm.sendCommandByUniqueId(uniqueId, commandID, (uint8_t*)&payload, payloadSize);")
            else:
                method_lines.append(f"    sendCommand(commandID, (uint8_t*)&payload, payloadSize);")
        else:
            if by_unique_id:
                method_lines.append(f"    _comm.sendCommandByUniqueId(uniqueId, commandID, (uint8_t*)&payload, sizeof(payload));")
            else:
                method_lines.append(f"    sendCommand(commandID, (uint8_t*)&payload, sizeof(payload));")
    else:
        # Send command with no payload
        if by_unique_id:
            method_lines.append(f"    _comm.sendCommandByUniqueId(uniqueId, commandID, nullptr, 0);")
        else:
            method_lines.append(f"    sendCommand(commandID, nullptr, 0);")
    
    # Handle the response
    if has_output:
        response_struct_name = format_response_name(cmd_str)
        method_lines.append(f"    uint8_t buffer[sizeof({response_struct_name})];")
        method_lines.append(f"    uint16_t receivedSize;")
        method_lines.append(f"    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);")
        method_lines.append(f"    if (_errno == 0) {{")
        method_lines.append(f"        if (receivedSize == sizeof({response_struct_name})) {{")
        method_lines.append(f"            {response_struct_name}* response = ({response_struct_name}*)buffer;")
        
        # Check if return type is a primitive type and we need to extract a field
        primitive_types = ['uint8_t', 'uint16_t', 'uint32_t', 'uint64_t',
                          'int8_t', 'int16_t', 'int32_t', 'int64_t', 'float']
        if return_type in primitive_types:
            # For primitive returns we need to extract the field based on the output parameter
            if len(cmd.get('Output', [])) == 1:
                field_name = cmd.get('Output', [])[0].get('ParameterName', 'value')
                method_lines.append(f"            return response->{field_name};")
            else:
                # Fall back to returning the whole structure if we can't determine a field
                method_lines.append(f"            return *response;")
        else:
            # Return the whole structure
            method_lines.append(f"            return *response;")
            
        method_lines.append(f"        }} else {{")
        method_lines.append(f"            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;")
        method_lines.append(f"        }}")
        method_lines.append(f"    }}")
        method_lines.append(f"    {response_struct_name} defaultResponse = {{0}};")
        
        # Again, extract field for primitive types in the default case too
        if return_type in primitive_types:
            if len(cmd.get('Output', [])) == 1:
                field_name = cmd.get('Output', [])[0].get('ParameterName', 'value')
                method_lines.append(f"    return defaultResponse.{field_name};")
            else:
                method_lines.append(f"    return defaultResponse;")
        else:
            method_lines.append(f"    return defaultResponse;")
    else:
        method_lines.append(f"    uint16_t receivedSize;")
        method_lines.append(f"    _errno = _comm.getResponse(nullptr, 0, receivedSize);")
    
    # Close the method
    method_lines.append("}")
    method_lines.append("")
    
    return "\n".join(method_lines)


def generate_wrapper_method_implementation(cmd, cmd_str, func_name, has_input, has_output,
                                           return_type, has_unit_conversion_output,
                                           unit_conversion_map, has_mixed_conversion, type_map=None, by_unique_id=False):
    """Generate the wrapper method implementation for commands with unit conversion.
    
    Args:
        cmd: Command dictionary from JSON
        cmd_str: Command string
        func_name: Function name
        has_input: Whether the command has input parameters
        has_output: Whether the command has output parameters
        return_type: Return type for the function
        has_unit_conversion_output: Whether the output needs unit conversion
        unit_conversion_map: Unit conversion function map
        has_mixed_conversion: Whether mixed unit conversion is needed
        type_map: Type mapping dictionary
        by_unique_id: Whether to generate the uniqueId overload version
    """
    # Make sure we have the type_map
    if type_map is None:
        type_map = {
            # Basic types
            'i8': 'int8_t',
            'u8': 'uint8_t',
            'i16': 'int16_t',
            'u16': 'uint16_t',
            'i24': 'int32_t',
            'u24': 'uint32_t',
            'i32': 'int32_t',
            'u32': 'uint32_t',
            'i48': 'int64_t',
            'u48': 'uint64_t',
            'i64': 'int64_t',
            'u64': 'uint64_t',
            'float': 'float',
            'double': 'double',
            
            # Array types
            'buf10': ('uint8_t', 10),
            'string8': ('char', 8),
            'string_null_term': ('char', 32),
            'firmware_page': ('uint8_t', 2058),
            
            # Special types
            'u24_version_number': 'uint32_t',
            'u32_version_number': 'uint32_t',
            'u64_unique_id': 'uint64_t',
            'crc32': 'uint32_t'
        }
    method_lines = []
    
    # Process input parameters
    params = []
    if has_input:
        input_params = cmd.get('Input', [])
        for idx, param in enumerate(input_params):
            desc = param.get('Description', '')
            param_name = param.get('ParameterName')
            
            # Extract type and parameter information
            match = re.match(r'(\w+):\s*(.*)', desc)
            if match:
                type_str = match.group(1)
                remainder = match.group(2)
                
                if not param_name:
                    # Simple name extraction
                    words = remainder.strip().split()
                    param_name = words[0] if words else f"param{idx}"
                    param_name = re.sub(r'\W|^(?=\d)', '_', param_name.lower())
                
                # Handle special type for list_2d
                if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                    list_type_name = format_list_type_name(cmd_str, converted=True)
                    params.append(f"{list_type_name}* {param_name}")
                    continue
                
                # For unit conversion parameters, use float
                if param.get('UnitConversion'):
                    params.append(f"float {param_name}")
                else:
                    # For non-unit conversion parameters, keep original type
                    # This should match what's in the `Raw` method
                    cpp_type = get_cpp_type_from_desc(desc, type_str)
                    params.append(f"{cpp_type} {param_name}")
    
    # Add function signature
    params_str = ", ".join(params)
    if by_unique_id:
        if params:
            method_lines.append(f"{return_type} Servomotor::{func_name}(uint64_t uniqueId, {params_str}) {{")
        else:
            method_lines.append(f"{return_type} Servomotor::{func_name}(uint64_t uniqueId) {{")
        
        # Add debug output for uniqueId version
        method_lines.append(f'    Serial.println("[Motor] {func_name} called (by unique ID).");')
    else:
        method_lines.append(f"{return_type} Servomotor::{func_name}({params_str}) {{")
        
        # Add debug output for regular version
        method_lines.append(f'    Serial.println("[Motor] {func_name} called.");')
    
    # Add debug output for parameters with unit conversion
    if has_input:
        for param in cmd.get('Input', []):
            if param.get('UnitConversion'):
                param_name = param.get('ParameterName')
                # Special handling for list types or mixed conversion parameters
                if param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time' or param.get('Description', '').startswith('list_2d:'):
                    method_lines.append(f'    Serial.println("  {param_name} in chosen unit: [complex object - cannot display directly]");')
                else:
                    method_lines.append(f'    Serial.print("  {param_name} in chosen unit: "); Serial.println({param_name});')
    
    # Special handling for mixed conversion (like multimove)
    if has_mixed_conversion:
        # Handle special list_2d types with mixed conversions
        method_lines.append(f"    // Convert list items from user units to internal units")
        # Assuming the parameter is moveList - we should detect this from the actual param name
        list_type_name = format_list_type_name(cmd_str)
        method_lines.append(f"    {list_type_name} convertedList[32];")
        method_lines.append(f"    for (int i = 0; i < moveCount; i++) {{")
        method_lines.append(f"        // Determine conversion based on move type (velocity or acceleration)")
        method_lines.append(f"        bool isVelocity = (moveTypes & (1 << i)) != 0;")
        method_lines.append(f"        if (isVelocity) {{")
        method_lines.append(f"            convertedList[i].value = (int32_t)::convertVelocity(moveList[i].value, m_velocityUnit, ConversionDirection::TO_INTERNAL);")
        method_lines.append(f"        }} else {{")
        method_lines.append(f"            convertedList[i].value = (int32_t)::convertAcceleration(moveList[i].value, m_accelerationUnit, ConversionDirection::TO_INTERNAL);")
        method_lines.append(f"        }}")
        method_lines.append(f"        convertedList[i].timeSteps = (uint32_t)::convertTime(moveList[i].duration, m_timeUnit, ConversionDirection::TO_INTERNAL);")
        method_lines.append(f"    }}")
        
        # Call the raw method with the converted list
        raw_params = ["moveCount", "moveTypes", "convertedList"]
        raw_params_str = ", ".join(raw_params)
        if by_unique_id:
            method_lines.append(f"    {func_name}Raw(uniqueId, {raw_params_str});")
        else:
            method_lines.append(f"    {func_name}Raw({raw_params_str});")
    else:
        # Regular parameter conversion
        raw_params = []
        for param in cmd.get('Input', []) if has_input else []:
            param_name = param.get('ParameterName')
            if param.get('UnitConversion'):
                # Get unit conversion type and function
                unit_type = param.get('UnitConversion', {}).get('Type')
                conversion_func = unit_conversion_map.get(unit_type, 'convertUnknown')
                
                # Get appropriate unit enum based on type
                unit_enum = f"m_{unit_type}Unit"
                
                # Add conversion code - use the global function, not a class method
                internal_var = f"{param_name}_internal"
                method_lines.append(f"    float {internal_var} = ::{conversion_func}({param_name}, {unit_enum}, ConversionDirection::TO_INTERNAL);")
                # Find this parameter in the Input array to get its original type
                for input_param in cmd.get('Input', []):
                    if input_param.get('ParameterName') == param_name:
                        # Get the type from the description (format: "u32: The maximum velocity")
                        desc = input_param.get('Description', '')
                        match = re.match(r'(\w+):\s*(.*)', desc)
                        if match:
                            # Extract the type string (e.g., "u32", "i64")
                            json_type_str = match.group(1)
                            
                            # Look up the C++ type - if not found, this is a fatal error
                            if json_type_str not in type_map:
                                raise ValueError(f"Unknown type '{json_type_str}' for parameter '{param_name}' in command '{cmd_str}'")
                                
                            cpp_type = type_map[json_type_str]
                            # Cast the converted value to the correct C++ type
                            raw_params.append(f"({cpp_type})({internal_var})")
                            break
            else:
                # No conversion needed
                raw_params.append(param_name)
        
        # Call the raw method
        raw_params_str = ", ".join(raw_params)
        if has_output and has_unit_conversion_output:
            if by_unique_id:
                method_lines.append(f"    auto rawResult = {func_name}Raw(uniqueId, {raw_params_str});")
            else:
                method_lines.append(f"    auto rawResult = {func_name}Raw({raw_params_str});")
            
            # Convert the raw result to user units
            if return_type == "float":
                # Single value output
                output_param = cmd.get('Output', [])[0]
                unit_type = output_param.get('UnitConversion', {}).get('Type')
                conversion_func = unit_conversion_map.get(unit_type, 'convertUnknown')
                unit_enum = f"m_{unit_type}Unit"
                
                # Check the original description to determine if it's a simple value or struct
                desc = output_param.get('Description', '')
                match = re.match(r'(\w+):\s*(.*)', desc)
                type_str = match.group(1) if match else ''
                
                # If it's a simple type in our type_map, use rawResult directly
                param_name = output_param.get('ParameterName')
                primitive_types = ['i8', 'u8', 'i16', 'u16', 'i24', 'u24', 'i32', 'u32', 'i48', 'u48', 'i64', 'u64']
                if type_str in primitive_types:
                    method_lines.append(f"    float converted = ::{conversion_func}((float)rawResult, {unit_enum}, ConversionDirection::FROM_INTERNAL);")
                else:
                    # Otherwise assume it's a struct with a field name matching the parameter name
                    method_lines.append(f"    float converted = ::{conversion_func}((float)rawResult.{param_name}, {unit_enum}, ConversionDirection::FROM_INTERNAL);")
                
                method_lines.append(f"    return converted;")
            else:
                # Multiple values in struct
                method_lines.append(f"    {return_type} converted;")
                for output_param in cmd.get('Output', []):
                    if output_param.get('UnitConversion'):
                        param_name = output_param.get('ParameterName')
                        unit_type = output_param.get('UnitConversion', {}).get('Type')
                        conversion_func = unit_conversion_map.get(unit_type, 'convertUnknown')
                        unit_enum = f"m_{unit_type}Unit"
                        
                        method_lines.append(f"    converted.{param_name} = ::{conversion_func}((float)rawResult.{param_name}, {unit_enum}, ConversionDirection::FROM_INTERNAL);")
                    else:
                        # Copy non-converted fields directly
                        param_name = output_param.get('ParameterName')
                        method_lines.append(f"    converted.{param_name} = rawResult.{param_name};")
                
                method_lines.append(f"    return converted;")
        else:
            # No return value to convert
            if by_unique_id:
                method_lines.append(f"    {func_name}Raw(uniqueId, {raw_params_str});")
            else:
                method_lines.append(f"    {func_name}Raw({raw_params_str});")
    
    # Close the method
    method_lines.append("}")
    method_lines.append("")
    
    return "\n".join(method_lines)


def generate_simple_method_implementation(cmd, cmd_str, cmd_id, func_name, has_input, has_output,
                                        return_type, type_map, endian_conversion_map, by_unique_id=False):
    """Generate the implementation for commands without unit conversion.
    
    Args:
        cmd: Command dictionary from JSON
        cmd_str: Command string
        cmd_id: Command enum ID
        func_name: Function name
        has_input: Whether the command has input parameters
        has_output: Whether the command has output parameters
        return_type: Return type for the function
        type_map: Type mapping dictionary
        endian_conversion_map: Endianness conversion functions
        by_unique_id: Whether to generate the uniqueId overload version
    """
    method_lines = []
    
    # Add method signature
    params = []
    
    # Process input parameters
    if has_input:
        input_params = cmd.get('Input', [])
        for idx, param in enumerate(input_params):
            desc = param.get('Description', '')
            param_name = param.get('ParameterName')
            
            # Extract type and parameter information
            match = re.match(r'(\w+):\s*(.*)', desc)
            if match:
                type_str = match.group(1)
                remainder = match.group(2)
                
                if not param_name:
                    # Simple name extraction
                    words = remainder.strip().split()
                    param_name = words[0] if words else f"param{idx}"
                    param_name = re.sub(r'\W|^(?=\d)', '_', param_name.lower())
                
                # Get C++ type from our type map
                cpp_type = type_map.get(type_str, 'uint8_t')
                
                # Handle array types in parameters
                if isinstance(cpp_type, tuple):
                    base_type, array_size = cpp_type
                    params.append(f"{base_type} {param_name}[{array_size}]")
                else:
                    params.append(f"{cpp_type} {param_name}")
    
    # Add function signature
    params_str = ", ".join(params)
    if by_unique_id:
        if params:
            method_lines.append(f"{return_type} Servomotor::{func_name}(uint64_t uniqueId, {params_str}) {{")
        else:
            method_lines.append(f"{return_type} Servomotor::{func_name}(uint64_t uniqueId) {{")
        
        # Add debug output for uniqueId version
        method_lines.append(f'    Serial.println("[Motor] {func_name} called (by unique ID).");')
    else:
        method_lines.append(f"{return_type} Servomotor::{func_name}({params_str}) {{")
        
        # Add debug output for regular version
        method_lines.append(f'    Serial.println("[Motor] {func_name} called.");')
    
    # Add debug comment about what this method does
    description = cmd.get('Description', 'No description available')
    method_lines.append(f"    // {description}")
    
    # Add command ID
    method_lines.append(f"    const uint8_t commandID = {cmd_str.upper().replace(' ', '_')};")
    
    # Create the payload if needed
    if has_input:
        payload_struct_name = format_payload_name(cmd_str)
        method_lines.append(f"    {payload_struct_name} payload;")
        
        # Set payload values
        for param in cmd.get('Input', []):
            param_name = param.get('ParameterName')
            # Skip parameters that don't have a clear mapping
            if not param_name:
                continue
                
            desc = param.get('Description', '')
            match = re.match(r'(\w+):\s*(.*)', desc)
            if match:
                type_str = match.group(1)
                
                # Get C++ type from our type map for endianness conversion
                cpp_type = type_map.get(type_str, 'uint8_t')
                
                # Handle array types in parameters
                if isinstance(cpp_type, tuple):
                    base_type, array_size = cpp_type
                    method_lines.append(f"    // Copy array data into payload")
                    method_lines.append(f"    memcpy(payload.{param_name}, {param_name}, sizeof(payload.{param_name}));")
                else:
                    # Get endianness conversion function
                    endian_func = endian_conversion_map.get(cpp_type, '')
                    if endian_func:
                        method_lines.append(f"    payload.{param_name} = {endian_func}({param_name});")
                    else:
                        method_lines.append(f"    payload.{param_name} = {param_name};")
        
        # Send the command - use sendCommandByUniqueId for uniqueId version
        if by_unique_id:
            method_lines.append(f"    _comm.sendCommandByUniqueId(uniqueId, commandID, (uint8_t*)&payload, sizeof(payload));")
        else:
            method_lines.append(f"    sendCommand(commandID, (uint8_t*)&payload, sizeof(payload));")
    else:
        # Send command with no payload - use sendCommandByUniqueId for uniqueId version
        if by_unique_id:
            method_lines.append(f"    _comm.sendCommandByUniqueId(uniqueId, commandID, nullptr, 0);")
        else:
            method_lines.append(f"    sendCommand(commandID, nullptr, 0);")
    
    # Handle the response
    if has_output:
        response_struct_name = format_response_name(cmd_str)
        method_lines.append(f"    uint8_t buffer[sizeof({response_struct_name})];")
        method_lines.append(f"    uint16_t receivedSize;")
        method_lines.append(f"    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);")
        method_lines.append(f"    if (_errno == 0) {{")
        method_lines.append(f"        if (receivedSize == sizeof({response_struct_name})) {{")
        method_lines.append(f"            {response_struct_name}* response = ({response_struct_name}*)buffer;")
        
        # Check if return type is a primitive type and we need to extract a field
        primitive_types = ['uint8_t', 'uint16_t', 'uint32_t', 'uint64_t',
                          'int8_t', 'int16_t', 'int32_t', 'int64_t', 'float']
        if return_type in primitive_types:
            # For primitive returns we need to extract the field based on the output parameter
            if len(cmd.get('Output', [])) == 1:
                field_name = cmd.get('Output', [])[0].get('ParameterName', 'value')
                method_lines.append(f"            return response->{field_name};")
            else:
                # Fall back to returning the whole structure if we can't determine a field
                method_lines.append(f"            return *response;")
        else:
            # Return the whole structure
            method_lines.append(f"            return *response;")
            
        method_lines.append(f"        }} else {{")
        method_lines.append(f"            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;")
        method_lines.append(f"        }}")
        method_lines.append(f"    }}")
        method_lines.append(f"    {response_struct_name} defaultResponse = {{0}};")
        
        # Again, extract field for primitive types in the default case too
        if return_type in primitive_types:
            if len(cmd.get('Output', [])) == 1:
                field_name = cmd.get('Output', [])[0].get('ParameterName', 'value')
                method_lines.append(f"    return defaultResponse.{field_name};")
            else:
                method_lines.append(f"    return defaultResponse;")
        else:
            method_lines.append(f"    return defaultResponse;")
    else:
        method_lines.append(f"    uint16_t receivedSize;")
        method_lines.append(f"    _errno = _comm.getResponse(nullptr, 0, receivedSize);")
    
    # Close the method
    method_lines.append("}")
    method_lines.append("")
    
    return "\n".join(method_lines)


def generate_multi_response_method(cmd_str, func_name, return_type):
    """Generate method for commands that have multiple responses."""
    method_lines = []
    
    # Add function signature
    method_lines.append(f"{return_type} Servomotor::{func_name}GetAnotherResponse() {{")
    
    # Add debug output
    method_lines.append(f'    Serial.println("[Motor] {func_name}GetAnotherResponse called.");')
    
    # Handle the response
    response_struct_name = format_response_name(cmd_str)
    method_lines.append(f"    uint8_t buffer[sizeof({response_struct_name})];")
    method_lines.append(f"    uint16_t receivedSize;")
    method_lines.append(f"    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);")
    method_lines.append(f"    if (_errno == 0) {{")
    method_lines.append(f"        if (receivedSize == sizeof({response_struct_name})) {{")
    method_lines.append(f"            {response_struct_name}* response = ({response_struct_name}*)buffer;")
    method_lines.append(f"            return *response;")
    method_lines.append(f"        }} else {{")
    method_lines.append(f"            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;")
    method_lines.append(f"        }}")
    method_lines.append(f"    }}")
    method_lines.append(f"    {response_struct_name} defaultResponse = {{0}};")
    method_lines.append(f"    return defaultResponse;")
    
    # Close the method
    method_lines.append("}")
    method_lines.append("")
    
    return "\n".join(method_lines)


def get_cpp_type_from_desc(desc, type_str):
    """Extract C++ type from type description."""
    type_map = {
        'i8': 'int8_t',
        'u8': 'uint8_t',
        'i16': 'int16_t',
        'u16': 'uint16_t',
        'i24': 'int32_t',
        'u24': 'uint32_t',
        'i32': 'int32_t',
        'u32': 'uint32_t',
        'i48': 'int64_t',
        'u48': 'uint64_t',
        'i64': 'int64_t',
        'u64': 'uint64_t',
        'float': 'float',
        'double': 'double',
    }
    
    if type_str in type_map:
        return type_map[type_str]
    
    # Handle array types or other special cases
    if type_str == 'buf10':
        return 'uint8_t*'
    elif type_str in ['string8', 'string_null_term']:
        return 'char*'
    elif type_str == 'firmware_page':
        return 'uint8_t*'
    
    # Default to uint8_t for unknown types
    return 'uint8_t'