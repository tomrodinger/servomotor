#!/usr/bin/env python3

"""
generate_arduino_unit_conversions.py

Generates:
  AutoGeneratedUnitConversions.h
  AutoGeneratedUnitConversions.cpp

This script generates unit conversion functions that convert directly between
user units and internal units, with a direction parameter to control the conversion.
"""

import sys
import json
import os
from datetime import datetime

import os

# Get the absolute path of the Arduino library directory
ARDUINO_LIB_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../Arduino_library/"))

# Base filenames (without paths)
H_BASENAME = "AutoGeneratedUnitConversions.h"
CPP_BASENAME = "AutoGeneratedUnitConversions.cpp"

HEADER_GUARD = "AUTO_GENERATED_UNIT_CONVERSIONS_H"
CPP_FILENAME = os.path.join(ARDUINO_LIB_DIR, CPP_BASENAME)
H_FILENAME = os.path.join(ARDUINO_LIB_DIR, H_BASENAME)

def main(json_filename):
    with open(json_filename, 'r') as f:
        data = json.load(f)

    units_dict = data["units"]              # e.g. {"time": [...], "position": [...], ...}
    factors = data["conversion_factors"]    # e.g. { "seconds": 31250.0, "milliseconds": 31.25, ... }
    
    # Check if there are conversion_offsets in the data
    conversion_offsets = data.get("conversion_offsets", {})

    # Write .h file
    with open(H_FILENAME, 'w') as hf:
        now = datetime.now().strftime('%b %-d %Y %H:%M:%S')
        hf.write(f"#ifndef {HEADER_GUARD}\n")
        hf.write(f"#define {HEADER_GUARD}\n\n")
        hf.write(f"// This file was auto-generated by generate_unit_conversion_code.py on {now}\n")
        hf.write("// Do not edit manually. If changes are needed, modify the generator program instead.\n\n")
        
        # Add COUNTS_PER_REVOLUTION definition
        counts_per_revolution = int(factors.get("shaft_rotations", 3276800))
        hf.write(f"// Define the number of encoder counts per revolution\n")
        hf.write(f"#define COUNTS_PER_REVOLUTION {counts_per_revolution}\n\n")

        # Add conversion factor definitions
        hf.write("// Conversion factors for all units\n")
        
        # Group factors by category for better organization
        category_factors = {}
        for category, unit_list in units_dict.items():
            category_factors[category] = {}
            for unit in unit_list:
                if unit in factors:
                    category_factors[category][unit] = factors[unit]
        
        # Write the #defines for each category
        for category, unit_factors in category_factors.items():
            hf.write(f"// {category.capitalize()} conversion factors\n")
            for unit, factor in unit_factors.items():
                define_name = f"CONVERSION_FACTOR_{unit.upper().replace(' ', '_')}"
                hf.write(f"#define {define_name} {factor:.9f}f\n")
            hf.write("\n")
        
        # Add conversion offset definitions if they exist
        if conversion_offsets:
            hf.write("// Conversion offsets for complex conversions\n")
            for offset_name, offset_value in conversion_offsets.items():
                define_name = f"CONVERSION_OFFSET_{offset_name.upper().replace(' ', '_')}"
                hf.write(f"#define {define_name} {offset_value:.9f}f\n")
            hf.write("\n")

        # Add ConversionDirection enum
        hf.write("// Enum to specify the direction of unit conversion\n")
        hf.write("enum class ConversionDirection {\n")
        hf.write("    TO_INTERNAL,    // Convert from user unit to internal unit\n")
        hf.write("    FROM_INTERNAL   // Convert from internal unit to user unit\n")
        hf.write("};\n\n")

        for category, unit_list in units_dict.items():
            enum_name = category.capitalize() + "Unit"  # e.g. "TimeUnit"

            # Generate an enum for the category
            hf.write(f"enum class {enum_name} {{\n")
            for i, unit_name in enumerate(unit_list):
                enumerator = unit_name.upper().replace(" ", "_")
                suffix = "," if i < len(unit_list) - 1 else ""
                hf.write(f"    {enumerator}{suffix}\n")
            hf.write("};\n\n")

            # Declare the conversion function with direction parameter
            func_name = f"convert{category.capitalize()}"
            hf.write(f"float {func_name}(float value, {enum_name} unit, ConversionDirection direction);\n\n")

        hf.write(f"#endif // {HEADER_GUARD}\n")

    # Write .cpp file
    with open(CPP_FILENAME, 'w') as cf:
        now = datetime.now().strftime('%b %-d %Y %H:%M:%S')
        cf.write(f"// This file was auto-generated by generate_unit_conversion_code.py on {now}\n")
        cf.write("// Do not edit manually. If changes are needed, modify the generator program instead.\n\n")
        cf.write(f'#include "{H_BASENAME}"\n')
        cf.write("#include <math.h>\n\n")

        # Generate a conversion function per category
        for category, unit_list in units_dict.items():
            enum_name = category.capitalize() + "Unit"
            func_name = f"convert{category.capitalize()}"
            internal_unit = unit_list[0]  # First unit in the list is the internal unit

            cf.write(f"float {func_name}(float value, {enum_name} unit, ConversionDirection direction)\n")
            cf.write("{\n")

            # Generate a unified conversion function that handles both simple and complex conversions
            cf.write("    // Handle both simple conversions (multiplication/division) and complex conversions (with offsets)\n")
            cf.write("    float result = 0.0f;\n\n")
            
            cf.write("    // Step 1: Convert from user unit to internal unit if needed\n")
            cf.write("    if (direction == ConversionDirection::TO_INTERNAL) {\n")
            cf.write("        switch (unit) {\n")
            
            # Generate cases for TO_INTERNAL direction
            for u in unit_list:
                enumerator = u.upper().replace(" ", "_")
                define_name = f"CONVERSION_FACTOR_{enumerator}"
                
                cf.write(f"            case {enum_name}::{enumerator}:\n")
                
                # Check if there's a corresponding offset for this unit to internal
                offset_name = f"{u.lower()}_to_{internal_unit.lower()}"
                offset_define = f"CONVERSION_OFFSET_{offset_name.upper().replace(' ', '_')}"
                
                # Get the factor value to check if it's 1.0
                factor_value = factors.get(u, 1.0)
                
                if offset_name in conversion_offsets:
                    # Get the offset value to check if it's 0.0
                    offset_value = conversion_offsets.get(offset_name, 0.0)
                    
                    if abs(factor_value - 1.0) < 1e-6 and abs(offset_value) < 1e-6:
                        # Both factor is 1.0 and offset is 0.0, no operation needed
                        cf.write(f"                result = value; // No conversion needed\n")
                    elif abs(factor_value - 1.0) < 1e-6:
                        # Factor is 1.0, only apply offset
                        cf.write(f"                result = value + {offset_define};\n")
                    elif abs(offset_value) < 1e-6:
                        # Offset is 0.0, only apply factor
                        cf.write(f"                result = value * {define_name};\n")
                    else:
                        # Apply both offset and factor
                        cf.write(f"                result = (value + {offset_define}) * {define_name};\n")
                    cf.write(f"                break;\n")
                else:
                    # No offset, check if factor is 1.0
                    if abs(factor_value - 1.0) < 1e-6:
                        cf.write(f"                result = value; // Factor is 1.0, no conversion needed\n")
                    else:
                        cf.write(f"                result = value * {define_name};\n")
                    cf.write(f"                break;\n")
            
            cf.write("            default:\n")
            cf.write("                return 0.0f; // Invalid unit\n")
            cf.write("        }\n")
            cf.write("    }\n")
            
            cf.write("    // Step 2: Convert from internal unit to user unit if needed\n")
            cf.write("    else { // direction == ConversionDirection::FROM_INTERNAL\n")
            cf.write("        switch (unit) {\n")
            
            # Generate cases for FROM_INTERNAL direction
            for u in unit_list:
                enumerator = u.upper().replace(" ", "_")
                define_name = f"CONVERSION_FACTOR_{enumerator}"
                
                cf.write(f"            case {enum_name}::{enumerator}:\n")
                
                # Check if there's a corresponding offset for internal to this unit
                offset_name = f"{internal_unit.lower()}_to_{u.lower()}"
                offset_define = f"CONVERSION_OFFSET_{offset_name.upper().replace(' ', '_')}"
                
                # Get the factor value to check if it's 1.0
                factor_value = factors.get(u, 1.0)
                
                if offset_name in conversion_offsets:
                    # Get the offset value to check if it's 0.0
                    offset_value = conversion_offsets.get(offset_name, 0.0)
                    
                    if abs(factor_value - 1.0) < 1e-6 and abs(offset_value) < 1e-6:
                        # Both factor is 1.0 and offset is 0.0, no operation needed
                        cf.write(f"                result = value; // No conversion needed\n")
                    elif abs(factor_value - 1.0) < 1e-6:
                        # Factor is 1.0, only apply offset
                        cf.write(f"                result = value + {offset_define};\n")
                    elif abs(offset_value) < 1e-6:
                        # Offset is 0.0, only apply factor
                        cf.write(f"                result = value / {define_name};\n")
                    else:
                        # Apply both offset and factor
                        cf.write(f"                result = (value / {define_name}) + {offset_define};\n")
                    cf.write(f"                break;\n")
                else:
                    # No offset, check if factor is 1.0
                    if abs(factor_value - 1.0) < 1e-6:
                        cf.write(f"                result = value; // Factor is 1.0, no conversion needed\n")
                    else:
                        cf.write(f"                result = value / {define_name};\n")
                    cf.write(f"                break;\n")
            
            cf.write("            default:\n")
            cf.write("                return 0.0f; // Invalid unit\n")
            cf.write("        }\n")
            cf.write("    }\n")
            
            cf.write("    return result;\n")
            cf.write("}\n\n")

    print(f"Generated {H_FILENAME} and {CPP_FILENAME} successfully.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python generate_arduino_unit_conversions.py <unit_conversions.json>")
        sys.exit(1)
    json_file = sys.argv[1]
    if not os.path.isfile(json_file):
        print(f"Error: cannot find the file {json_file}")
        sys.exit(1)
    main(json_file)
