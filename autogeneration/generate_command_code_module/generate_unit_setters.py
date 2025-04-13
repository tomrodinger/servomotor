#!/usr/bin/env python3
"""
Module for generating unit setter method implementations.
This module contains a function that takes unit conversion data and returns 
the unit setter implementations for Servomotor.cpp.
"""

def capitalize_unit_type(unit_type):
    """
    Convert a unit type name to the corresponding C++ enum type name.
    For example: 'position' -> 'PositionUnit'
    
    Args:
        unit_type: The unit type string from the JSON data
        
    Returns:
        The likely enum type name based on a consistent pattern
    """
    return unit_type.capitalize() + "Unit"

def generate_unit_setters(commands_data=None, unit_conversions_data=None, **kwargs):
    """
    Generate the unit setter method implementations for Servomotor.cpp.
    
    Args:
        commands_data: List of command dictionaries from motor_commands.json (not used here)
        unit_conversions_data: Dictionary of unit conversion data from unit_conversions_M3.json
        kwargs: Additional context data (not used by this function)
        
    Returns:
        str: Formatted content for unit setter method implementations
    """
    if unit_conversions_data is None or 'units' not in unit_conversions_data:
        return "// No unit conversion data available for generating setters"
    
    # Output setter functions
    setter_functions = []
    
    # Generate a setter function for each unit type in the JSON data
    for unit_type, units in unit_conversions_data.get('units', {}).items():
        # Derive the enum type from the unit type
        enum_type = capitalize_unit_type(unit_type)
        
        # Create the setter function
        setter_functions.append(generate_setter_method(
            unit_type, enum_type
        ))
    
    return "\n".join(setter_functions)

def generate_setter_method(unit_type, enum_type):
    """Generate a single unit setter method."""
    type_name = unit_type.capitalize()
    
    method = f"""void Servomotor::set{type_name}Unit({enum_type} unit) {{
    Serial.println("[Motor] set{type_name}Unit called");
    m_{unit_type.lower()}Unit = unit;
}}
"""
    return method

if __name__ == "__main__":
    # For testing
    print(generate_unit_setters({'units': {'position': [], 'time': []}}))