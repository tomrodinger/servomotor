#!/usr/bin/env python3
"""
Module for generating Commands.h file content.
This module contains a function that takes command data and returns 
the command enumerations for Commands.h.
"""

import re


def generate_commands_header(commands_data=None, **kwargs):
    """
    Generate the commands enum entries for Commands.h file.
    
    Args:
        commands_data: List of command dictionaries from motor_commands.json.
            Each dictionary should have 'CommandString' and 'CommandEnum' keys.
        kwargs: Additional context data (not used by this function)
        
    Returns:
        str: Formatted content for command entries.
    """
    if commands_data is None:
        return "// No command data available"
        
    lines = []
    
    # Add each command to the enum
    for cmd in commands_data:
        # Format the command name by converting to uppercase and replacing non-alphanumeric with underscores
        cmd_name = cmd['CommandString'].upper()
        cmd_name = re.sub(r'[^a-zA-Z0-9]+', '_', cmd_name)
        cmd_id = cmd['CommandEnum']
        
        # Add the command to the enum
        lines.append(f"    {cmd_name} = {cmd_id},")
    
    # Join all lines with newlines
    return "\n".join(lines)