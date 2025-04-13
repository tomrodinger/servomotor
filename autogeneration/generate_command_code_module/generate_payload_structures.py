#!/usr/bin/env python3
"""
Module for generating payload and response structure content.
This module contains a function that takes command data and returns 
the payload and response structure declarations for Servomotor.h.
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


def format_list_type_name(command_string, converted=False):
    """
    Format a command string to a list type name.
    
    Args:
        command_string: The command string to format
        converted: Whether to return the converted list type
        
    Returns:
        Formatted list type name (e.g., "multi move" -> "multiMoveList_t")
    """
    base_name = format_command_name(command_string)
    if converted:
        return f"{base_name}ListConverted_t"
    else:
        return f"{base_name}List_t"


def generate_payload_structures(commands_data=None, data_types_data=None, **kwargs):
    """
    Generate the payload structure declarations for Servomotor.h.
    
    Args:
        commands_data: List of command dictionaries from motor_commands.json.
        data_types_data: List of data type dictionaries from data_types.json.
        kwargs: Additional context data (not used by this function)
        
    Returns:
        str: Formatted content for payload and response structure declarations.
    """
    if commands_data is None:
        return "// No command data available"
        
    if data_types_data is None:
        data_types_data = []
    
    # Built-in C/C++ types we don't want to redefine
    builtin_types = {
        "int8_t", "uint8_t", "int16_t", "uint16_t",
        "int32_t", "uint32_t", "int64_t", "uint64_t",
        "float", "double"
    }
    
    # Store list type structures for commands with list_2d parameters
    list_type_structures = []
    
    # Create a data type map for easy lookup
    data_type_map = {dt.get('data_type', ''): dt for dt in data_types_data}
    
    # Create a comprehensive type map
    # For array types, use a tuple of (base_type, array_size)
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
        # list_2d is handled specially - it needs the command name
        
        # Special types
        'u24_version_number': 'uint32_t',
        'u32_version_number': 'uint32_t',
        'u64_unique_id': 'uint64_t',
        'crc32': 'uint32_t'
    }
    # First pass: identify commands with list_2d parameters to generate list type structures
    for cmd in commands_data:
        if not cmd or not isinstance(cmd, dict):
            continue  # Skip invalid command entries
            
        cmd_str = cmd.get('CommandString', '')
        if not cmd_str:
            continue  # Skip commands without a CommandString
        
        # Check if this command has list_2d type parameters or mixed unit conversions
        has_list_2d = False
        input_params = cmd.get('Input', [])
        if input_params is None:
            input_params = []  # Ensure we have a valid iterable

        for param in input_params:
            if not param or not isinstance(param, dict):
                continue  # Skip invalid parameter entries
                
            desc = param.get('Description', '')
            match = re.match(r'(\w+):\s*(.*)', desc)
            
            if match:
                type_str = match.group(1)
                if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                    has_list_2d = True
                    break
        
        if has_list_2d:
            # Generate both raw and converted list type names
            raw_list_type = format_list_type_name(cmd_str)
            converted_list_type = format_list_type_name(cmd_str, converted=True)
            
            # Add the raw list type structure
            list_type_structures.append(f"// Structure for {cmd_str.lower()} list item with raw internal units")
            list_type_structures.append("typedef struct {")
            list_type_structures.append("    int32_t value;  // acceleration or velocity value (internal units)")
            list_type_structures.append("    uint32_t timeSteps;  // duration in time steps (internal units)")
            list_type_structures.append(f"}} {raw_list_type};")
            list_type_structures.append("")
            
            # Add the converted list type structure
            list_type_structures.append(f"// Structure for {cmd_str.lower()} list item with user-friendly units")
            list_type_structures.append("typedef struct {")
            list_type_structures.append("    float value;  // velocity or acceleration in user units")
            list_type_structures.append("    float duration;  // duration in user units (seconds, milliseconds, etc.)")
            list_type_structures.append(f"}} {converted_list_type};")
            list_type_structures.append("")
    
    # Output lines - start with list type structures followed by regular structures
    structure_lines = list_type_structures[:]
    
    # Process each command
    for cmd in commands_data:
        if not cmd or not isinstance(cmd, dict):
            continue  # Skip invalid command entries
            
        cmd_str = cmd.get('CommandString', '')
        if not cmd_str:
            continue  # Skip commands without a CommandString
            
        func_name = format_command_name(cmd_str)
        
        # Handle payload structures for commands with input parameters
        if cmd.get('Input'):
            params = []
            input_params = cmd.get('Input', [])
            if input_params is None:
                input_params = []  # Ensure we have a valid iterable
                
            for param in input_params:
                if not param or not isinstance(param, dict):
                    continue  # Skip invalid parameter entries
                    
                # Extract parameter description and type
                desc = param.get('Description', '')
                match = re.match(r'(\w+):\s*(.*)', desc)
                
                if match:
                    type_str = match.group(1)
                    remainder = match.group(2).strip()
                    
                    # Get parameter name from ParameterName field or extract from description
                    param_name = param.get('ParameterName')
                    if not param_name:
                        # Extract name from the description if possible
                        name_match = re.match(r'([\w_]+)\s*[:\-]\s*(.*)', remainder)
                        if name_match:
                            param_name = name_match.group(1)
                        else:
                            # Use the first word of the description
                            words = remainder.split()
                            param_name = words[0] if words else 'param'
                        
                        # Clean up parameter name
                        param_name = re.sub(r'\W|^(?=\d)', '_', param_name.lower())
                    
                    # Handle special types based on description patterns
                    if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                        # Generate list type name from command name
                        list_type_name = format_list_type_name(cmd_str)
                        
                        # Check if array size is specified in description
                        array_size_match = re.search(r'limit of (\d+)', desc) or re.search(r'up to (\d+)', desc)
                        if array_size_match:
                            array_size = int(array_size_match.group(1))
                            params.append(f"    {list_type_name} {param_name}[{array_size}];")
                        else:
                            # Default to pointer if no limit specified
                            params.append(f"    {list_type_name}* {param_name};")
                        continue
                    
                    # Map the type to C++ type
                    cpp_type = type_map.get(type_str, 'uint8_t')
                    
                    # Handle array types
                    if isinstance(cpp_type, tuple):
                        base_type, array_size = cpp_type
                        params.append(f"    {base_type} {param_name}[{array_size}];")
                    else:
                        params.append(f"    {cpp_type} {param_name};")
            
            # If there are parameters, create a struct
            if params:
                structure_lines.append(f"// Structure for {cmd_str} command payload")
                structure_lines.append("typedef struct __attribute__((__packed__)) {")
                structure_lines.extend(params)
                structure_lines.append(f"}} {func_name}Payload;")
                structure_lines.append("")
        
        # Handle response structures for commands with output parameters
        output_params = cmd.get('Output', [])
        if output_params and output_params != 'success_response':
            # Generate response type name algorithmically
            resp_type = format_response_name(cmd_str)
            
            # If output_params is not a list, make it one
            if not isinstance(output_params, list):
                output_params = []
                
            # Skip if the output is a single builtin type
            if len(output_params) == 1:
                desc = output_params[0].get('Description', '')
                match = re.match(r'(\w+):\s*(.*)', desc)
                if match and match.group(1) in ['u8', 'i8', 'u16', 'i16', 'u32', 'i32', 'u64', 'i64', 'float', 'double']:
                    # This is a single built-in type, skip struct creation
                    continue
            
            # Process output parameters
            params = []
            for param in output_params:
                if not param or not isinstance(param, dict):
                    continue  # Skip invalid parameter entries
                    
                # Extract parameter description and type
                desc = param.get('Description', '')
                match = re.match(r'(\w+):\s*(.*)', desc)
                
                if match:
                    type_str = match.group(1)
                    remainder = match.group(2).strip()
                    
                    # Get parameter name from ParameterName field or extract from description
                    param_name = param.get('ParameterName')
                    if not param_name:
                        # Extract name from the description if possible
                        name_match = re.match(r'([\w_]+)\s*[:\-]\s*(.*)', remainder)
                        if name_match:
                            param_name = name_match.group(1)
                        else:
                            # Use the first word of the description
                            words = remainder.split()
                            param_name = words[0] if words else 'param'
                        
                        # Clean up parameter name
                        param_name = re.sub(r'\W|^(?=\d)', '_', param_name.lower())
                    
                    # Map the type to C++ type
                    cpp_type = type_map.get(type_str, 'uint8_t')
                    
                    # Handle array types
                    if isinstance(cpp_type, tuple):
                        base_type, array_size = cpp_type
                        params.append(f"    {base_type} {param_name}[{array_size}];")
                    else:
                        params.append(f"    {cpp_type} {param_name};")
            
            # If there are parameters, create a struct
            if params:
                structure_lines.append(f"// Structure for {cmd_str} command response")
                structure_lines.append("typedef struct __attribute__((__packed__)) {")
                structure_lines.extend(params)
                structure_lines.append(f"}} {resp_type};")
                structure_lines.append("")
                
                # Check if any parameters have unit conversion defined
                has_unit_conversion = any(param.get('UnitConversion') for param in output_params)
                
                # If so, create a converted structure with float types
                # Skip if only one parameter with unit conversion - return the value directly instead
                if has_unit_conversion and not (len(output_params) == 1 and output_params[0].get('UnitConversion')):
                    converted_params = []
                    for param in output_params:
                        param_name = param.get('ParameterName')
                        if param.get('UnitConversion'):
                            # Use float for parameters with unit conversion
                            converted_params.append(f"    float {param_name};")
                        else:
                            # Keep original type for parameters without unit conversion
                            desc = param.get('Description', '')
                            match = re.match(r'(\w+):\s*(.*)', desc)
                            if match:
                                type_str = match.group(1)
                                cpp_type = type_map.get(type_str, 'uint8_t')
                                if isinstance(cpp_type, tuple):
                                    base_type, array_size = cpp_type
                                    converted_params.append(f"    {base_type} {param_name}[{array_size}];")
                                else:
                                    converted_params.append(f"    {cpp_type} {param_name};")
                            else:
                                converted_params.append(f"    uint8_t {param_name};")
                    
                    # Generate a converted response structure
                    structure_lines.append(f"// Structure for converted {cmd_str} values")
                    structure_lines.append("typedef struct {")
                    structure_lines.extend(converted_params)
                    structure_lines.append(f"}} {resp_type}Converted;")
                    structure_lines.append("")
    
    return "\n".join(structure_lines)