#!/usr/bin/env python3
"""
Module for generating command method declarations.
This module contains a function that takes command data and returns 
the method declarations for the Servomotor class.
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


def generate_command_methods(commands_data=None, data_types_data=None, **kwargs):
    """
    Generate the command method declarations for Servomotor.h.
    
    Args:
        commands_data: List of command dictionaries from motor_commands.json
        data_types_data: List of data type dictionaries from data_types.json
        kwargs: Additional context data (not used by this function)
        
    Returns:
        str: Formatted content for command method declarations
    """
    if commands_data is None:
        return "// No command data available"
    
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
    
    # Indentation for class methods (4 spaces)
    indent = "    "
    
    # Output lines with proper indentation
    method_lines = []
    
    # Process each command
    for cmd in commands_data:
        cmd_str = cmd['CommandString']
        func_name = format_command_name(cmd_str)
        
        # Determine if this command has outputs (returns a value)
        has_output = cmd.get('Output') and cmd['Output'] != 'success_response'
        
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
            
            # Also check if there's a converted response struct already defined
            if return_type.endswith('Response'):
                converted_name = return_type.replace('Response', 'ResponseConverted')
                if any(cmd_line.endswith(f"}} {converted_name};") for cmd_line in method_lines):
                    has_converted_response = True
        
        # Get input parameters
        has_input = bool(cmd.get('Input'))
        raw_params = []
        wrapper_params = []
        
        if has_input:
            # Generate parameter lists for commands with input
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
                    
                    # Handle special types based on description patterns
                    if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
                        # For list_2d types or mixed_acceleration_velocity_time, we need special handling
                        list_type_name = format_list_type_name(cmd_str)
                        converted_list_type = format_list_type_name(cmd_str, converted=True)
                        
                        # Always use pointer for these types
                        raw_params.append(f"{list_type_name}* {param_name}")
                        wrapper_params.append(f"{converted_list_type}* {param_name}")
                        continue
                    
                    # Get C++ type from our type map
                    cpp_type = type_map.get(type_str, 'uint8_t')
                    
                    # Handle array types in parameters
                    if isinstance(cpp_type, tuple):
                        base_type, array_size = cpp_type
                        raw_params.append(f"{base_type} {param_name}[{array_size}]")
                        wrapper_params.append(f"{base_type} {param_name}[{array_size}]")
                    else:
                        raw_params.append(f"{cpp_type} {param_name}")
                        
                        # For wrapper, convert numeric types to float for easier unit conversion
                        if param.get('UnitConversion'):
                            wrapper_params.append(f"float {param_name}")
                        else:
                            wrapper_params.append(f"{cpp_type} {param_name}")
        
        # Add the method declarations with proper indentation
        raw_param_str = ", ".join(raw_params)
        wrapper_param_str = ", ".join(wrapper_params)
        
        if needs_unit_conversion:
            # Add Raw and Converted methods
            method_lines.append(f"{indent}{return_type} {func_name}Raw({raw_param_str});")
            
            # For wrapper method with converted return type
            wrapper_return = return_type
            
            # Check if this is a single output parameter that needs conversion
            if has_unit_conversion_output:
                if len(cmd.get('Output', [])) == 1:
                    # For single values with unit conversion, always return float
                    wrapper_return = "float"
                else:
                    # For multiple values in a struct, use the converted struct type
                    wrapper_return = return_type.replace('Response', 'ResponseConverted')
            
            method_lines.append(f"{indent}{wrapper_return} {func_name}({wrapper_param_str});")
        else:
            # Just add the regular method for commands without unit conversion
            method_lines.append(f"{indent}{return_type} {func_name}({raw_param_str});")
        
        # Add method for multiple responses if needed
        if cmd.get('MultipleResponses'):
            method_lines.append(f"{indent}{return_type} {func_name}GetAnotherResponse();")
            
        # Add a blank line after each command group
        method_lines.append("")
    
    return "\n".join(method_lines)