#!/usr/bin/env python3
"""
Servomotor API Documentation Generator
Generates comprehensive API documentation in both PDF and Markdown formats
"""

import json
import os
import sys
import re
from datetime import datetime
from collections import defaultdict
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm, inch
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, Frame, PageTemplate, PageBreak, Preformatted, KeepTogether
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib import colors
from reportlab.pdfgen.canvas import Canvas

# Import existing utilities
from styles import create_title_style, create_subtitle_style, create_normal_style, create_heading_style
from utils import get_processed_image
from versioning import get_latest_version
from reportlab.platypus import Flowable, Table, TableStyle
from reportlab.lib.utils import simpleSplit

class APIDocumentationGenerator:
    def __init__(self):
        self.motor_commands_path = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/motor_commands.json"
        self.data_types_path = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/data_types.json"
        self.error_codes_path = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/error_codes.json"
        self.error_handling_text_path = "error_handling.txt"
        self.firmware_dir_path = "../../firmware/firmware_releases"
        self.valid_products_path = "VALID_PRODUCTS.txt"
        self.commands = []
        self.data_types = []
        self.error_codes = []
        self.commands_by_group = defaultdict(list)
        self.validation_errors = []
        self.latest_firmware = {}
        
    def load_commands(self):
        """Load and parse motor commands from JSON file"""
        try:
            with open(self.motor_commands_path, 'r') as f:
                self.commands = json.load(f)
            print(f"✓ Loaded {len(self.commands)} commands from motor_commands.json")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find motor_commands.json at {self.motor_commands_path}")
            return False
        except json.JSONDecodeError as e:
            print(f"\n❌ ERROR: Failed to parse motor_commands.json: {e}")
            return False
    
    def load_data_types(self):
        """Load and parse data types from JSON file"""
        try:
            with open(self.data_types_path, 'r') as f:
                self.data_types = json.load(f)
            print(f"✓ Loaded {len(self.data_types)} data types from data_types.json")
            # Sort data types by whether they are integers first, then by name
            self.data_types.sort(key=lambda x: (not x.get('is_integer', False), x['data_type']))
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find data_types.json at {self.data_types_path}")
            return False
        except json.JSONDecodeError as e:
            print(f"\n❌ ERROR: Failed to parse data_types.json: {e}")
            return False
    
    def load_error_codes(self):
        """Load and parse error codes from JSON file"""
        try:
            with open(self.error_codes_path, 'r') as f:
                data = json.load(f)
                self.error_codes = data.get('errors', [])
            print(f"✓ Loaded {len(self.error_codes)} error codes from error_codes.json")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find error_codes.json at {self.error_codes_path}")
            return False
        except json.JSONDecodeError as e:
            print(f"\n❌ ERROR: Failed to parse error_codes.json: {e}")
            return False
    
    def load_error_handling_text(self):
        """Load error handling description text"""
        try:
            with open(self.error_handling_text_path, 'r') as f:
                self.error_handling_text = f.read()
            print(f"✓ Loaded error handling text from {self.error_handling_text_path}")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find error handling text at {self.error_handling_text_path}")
            self.error_handling_text = "Error handling description not available."
            return False
    
    def load_valid_products(self):
        """Load valid product names from VALID_PRODUCTS.txt"""
        try:
            with open(self.valid_products_path, 'r') as f:
                products = [line.strip() for line in f if line.strip()]
            print(f"✓ Loaded {len(products)} valid products: {', '.join(products)}")
            return products
        except FileNotFoundError:
            print(f"⚠️  Warning: Could not find VALID_PRODUCTS.txt")
            return []
    
    def find_latest_firmware(self):
        """Find the latest firmware file for each valid product"""
        valid_products = self.load_valid_products()
        if not valid_products:
            return {}
        
        latest_firmware = {}
        
        try:
            # List all firmware files
            firmware_files = os.listdir(self.firmware_dir_path)
            
            for product in valid_products:
                # Find all firmware files for this product
                product_files = [f for f in firmware_files if f.startswith(product)]
                
                if product_files:
                    # Parse firmware versions and find the latest
                    latest_file = None
                    latest_version = None
                    
                    for filename in product_files:
                        # Extract version using regex (looking for fw followed by version numbers)
                        match = re.search(r'fw(\d+\.\d+\.\d+\.\d+)', filename)
                        if match:
                            version_str = match.group(1)
                            version_tuple = tuple(map(int, version_str.split('.')))
                            
                            if latest_version is None or version_tuple > latest_version:
                                latest_version = version_tuple
                                latest_file = filename
                    
                    if latest_file:
                        # Extract model name for display
                        model_match = re.search(r'servomotor_(M\d+)', latest_file)
                        if model_match:
                            model_name = f"Model {model_match.group(1)}"
                            latest_firmware[model_name] = latest_file
                            print(f"✓ Found latest firmware for {model_name}: {latest_file}")
            
            self.latest_firmware = latest_firmware
            return latest_firmware
            
        except FileNotFoundError:
            print(f"⚠️  Warning: Firmware directory not found at {self.firmware_dir_path}")
            return {}
        except Exception as e:
            print(f"⚠️  Warning: Error scanning firmware directory: {e}")
            return {}
    
    def validate_commands(self):
        """Validate that all commands have CommandGroup defined"""
        print("\nValidating commands...")
        commands_without_group = []
        
        for i, cmd in enumerate(self.commands):
            if 'CommandGroup' not in cmd or not cmd['CommandGroup']:
                commands_without_group.append({
                    'index': i,
                    'name': cmd.get('CommandString', 'Unknown'),
                    'enum': cmd.get('CommandEnum', 'Unknown')
                })
            else:
                # Group commands by category
                self.commands_by_group[cmd['CommandGroup']].append(cmd)
        
        if commands_without_group:
            print("\n" + "="*80)
            print("❌ VALIDATION ERROR: Commands without CommandGroup")
            print("="*80)
            print("\nThe following commands are missing the 'CommandGroup' field:")
            for cmd in commands_without_group:
                print(f"  - Index {cmd['index']}: {cmd['name']} (Enum: {cmd['enum']})")
            
            print("\n📝 TO FIX THIS ERROR:")
            print("1. Open: /Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/motor_commands.json")
            print("2. Add a 'CommandGroup' field to each command listed above")
            print("3. Valid group names include: 'Basic Control', 'Motion Control', 'Configuration', 'Status & Monitoring', 'Device Management'")
            print("4. Example: \"CommandGroup\": \"Basic Control\"")
            print("\n" + "="*80)
            return False
        
        print(f"✓ All {len(self.commands)} commands have valid CommandGroup fields")
        print(f"✓ Found {len(self.commands_by_group)} command groups:")
        for group in sorted(self.commands_by_group.keys()):
            print(f"  - {group}: {len(self.commands_by_group[group])} commands")
        return True
    
    def get_python_method_name(self, command_string):
        """Convert command string to Python method name"""
        # Handle special cases
        special_cases = {
            "Disable MOSFETs": "disable_mosfets",
            "Enable MOSFETs": "enable_mosfets",
            "Emergency stop": "emergency_stop",
            "System reset": "system_reset",
            "Get n queued items": "get_n_queued_items",
            "Zero position": "zero_position"
        }
        
        if command_string in special_cases:
            return special_cases[command_string]
        
        # General conversion: lowercase and replace spaces with underscores
        return command_string.lower().replace(' ', '_').replace('-', '_')
    
    def generate_python_example(self, command):
        """Generate Python code example for a command with explicit variable assignments"""
        method_name = self.get_python_method_name(command['CommandString'])
        
        example_lines = []
        MAX_LINE_LENGTH = 70  # Maximum characters per line for PDF display
        
        # Handle input parameters
        param_vars = []
        if command['Input'] and command['Input'] != "null":
            for param in command['Input']:
                param_name = param.get('ParameterName', 'value')
                var_name = f"{param_name}_value"
                example_lines.append(f"{var_name} = 0  # Replace with your desired value")
                param_vars.append(f"{param_name}={var_name}")
            if param_vars:
                example_lines.append("")
        
        # Handle the function call
        if command['Output'] and command['Output'] != "success_response":
            # Command returns values
            if isinstance(command['Output'], list):
                result_vars = [out.get('ParameterName', f'value{i}') for i, out in enumerate(command['Output'])]
                
                # Check if we need to wrap the return variables
                result_vars_str = ', '.join(result_vars)
                
                if len(command['Output']) == 1:
                    # Single return value
                    result_var = result_vars[0]
                    call_base = f"{result_var} = motor.{method_name}"
                else:
                    # Multiple return values - check if we need to wrap them
                    if len(result_vars_str) > 40:  # Break return values into multiple lines
                        example_lines.append("(")
                        for i, var in enumerate(result_vars):
                            if i < len(result_vars) - 1:
                                example_lines.append(f"    {var},")
                            else:
                                example_lines.append(f"    {var}")
                        call_base = f") = motor.{method_name}"
                    else:
                        call_base = f"{result_vars_str} = motor.{method_name}"
                
                # Build the function call
                if param_vars:
                    # Check if the entire line would be too long
                    full_line = f"{call_base}({', '.join(param_vars)})"
                    if len(full_line) > MAX_LINE_LENGTH or len(param_vars) > 2:
                        # Wrap parameters
                        if len(result_vars) > 1 and len(result_vars_str) > 40:
                            # Already started multi-line for return values
                            example_lines.append(f") = motor.{method_name}(")
                        else:
                            example_lines.append(f"{call_base}(")
                        for i, param in enumerate(param_vars):
                            if i < len(param_vars) - 1:
                                example_lines.append(f"    {param},")
                            else:
                                example_lines.append(f"    {param}")
                        example_lines.append(")")
                    else:
                        example_lines.append(full_line)
                else:
                    # No parameters
                    if len(result_vars) > 1 and len(result_vars_str) > 40:
                        example_lines.append(f") = motor.{method_name}()")
                    else:
                        example_lines.append(f"{call_base}()")
                
                # Add print statements
                for var in result_vars:
                    example_lines.append(f"print(f\"{var}: {{{var}}}\")")
            else:
                # Non-list output
                if param_vars:
                    full_line = f"response = motor.{method_name}({', '.join(param_vars)})"
                    if len(full_line) > MAX_LINE_LENGTH:
                        example_lines.append(f"response = motor.{method_name}(")
                        for i, param in enumerate(param_vars):
                            if i < len(param_vars) - 1:
                                example_lines.append(f"    {param},")
                            else:
                                example_lines.append(f"    {param}")
                        example_lines.append(")")
                    else:
                        example_lines.append(full_line)
                else:
                    example_lines.append(f"response = motor.{method_name}()")
                example_lines.append("print(f\"Response: {response}\")")
        else:
            # Command doesn't return values (just success response)
            if param_vars:
                # Check if the entire line would be too long
                full_line = f"motor.{method_name}({', '.join(param_vars)})"
                if len(full_line) > MAX_LINE_LENGTH or len(param_vars) > 2:
                    # Wrap parameters
                    example_lines.append(f"motor.{method_name}(")
                    for i, param in enumerate(param_vars):
                        if i < len(param_vars) - 1:
                            example_lines.append(f"    {param},")
                        else:
                            example_lines.append(f"    {param}")
                    example_lines.append(")")
                else:
                    example_lines.append(full_line)
            else:
                example_lines.append(f"motor.{method_name}()")
        
        return '\n'.join(example_lines)
    
    def generate_markdown(self):
        """Generate both Python and Arduino Markdown documentation"""
        # Generate Python documentation
        self.generate_python_markdown()
        # Generate Arduino documentation
        self.generate_arduino_markdown()
        return True
    
    def generate_arduino_example(self, command):
        """Generate Arduino code example for a command - simplified without Serial output"""
        cmd_string = command['CommandString']
        method_name = ''.join(word.capitalize() if i > 0 else word.lower()
                             for i, word in enumerate(cmd_string.split()))
        
        example_lines = []
        example_lines.append(f"// {cmd_string}")
        
        # Handle input parameters - declare variables
        param_names = []
        if command['Input'] and command['Input'] != "null":
            for param in command['Input']:
                param_name = param.get('ParameterName', 'value')
                param_desc = param.get('Description', '')
                
                # Determine data type and value based on description
                if 'float' in param_desc.lower() or 'f32' in param_desc:
                    data_type = "float"
                    default_value = "10.0"
                    if 'velocity' in param_name.lower():
                        default_value = "2.0"
                    elif 'acceleration' in param_name.lower():
                        default_value = "4.0"
                    elif 'time' in param_name.lower() or 'duration' in param_name.lower():
                        default_value = "3.0"
                elif 'bool' in param_desc.lower():
                    data_type = "bool"
                    default_value = "true"
                elif 'u32' in param_desc or 'uint32' in param_desc.lower():
                    data_type = "uint32_t"
                    default_value = "1000"
                elif 'i32' in param_desc or 'int32' in param_desc.lower():
                    data_type = "int32_t"
                    default_value = "1000"
                elif 'u16' in param_desc or 'uint16' in param_desc.lower():
                    data_type = "uint16_t"
                    default_value = "100"
                elif 'i16' in param_desc or 'int16' in param_desc.lower():
                    data_type = "int16_t"
                    default_value = "100"
                elif 'u8' in param_desc or 'uint8' in param_desc.lower():
                    data_type = "uint8_t"
                    default_value = "1"
                elif 'i8' in param_desc or 'int8' in param_desc.lower():
                    data_type = "int8_t"
                    default_value = "1"
                else:
                    data_type = "float"
                    default_value = "0"
                
                example_lines.append(f"{data_type} {param_name} = {default_value};")
                param_names.append(param_name)
            
            if param_names:
                example_lines.append("")
        
        # Handle the function call and return values
        if command['Output'] and command['Output'] != "success_response":
            # Command returns values - Always returns a structure in Arduino
            # Structure name is based on method name + Response
            struct_name = f"{method_name}Response"
            
            # Instantiate the structure and call the function
            if param_names:
                example_lines.append(f"{struct_name} response = motor.{method_name}({', '.join(param_names)});")
            else:
                example_lines.append(f"{struct_name} response = motor.{method_name}();")
        else:
            # Command doesn't return values (just success response)
            if param_names:
                example_lines.append(f"motor.{method_name}({', '.join(param_names)});")
            else:
                example_lines.append(f"motor.{method_name}();")
        
        return '\n'.join(example_lines)
    
    def generate_python_markdown(self):
        """Generate Python Markdown documentation"""
        print("\nGenerating Python Markdown documentation...")
        
        md_content = []
        md_content.append("# Servomotor Python API Documentation\n")
        md_content.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        # Add firmware information if available
        if self.latest_firmware:
            md_content.append("## Latest Firmware Versions\n")
            md_content.append("At the time of generating this API reference, the latest released firmware versions for the servomotors are:\n")
            for model, firmware_file in sorted(self.latest_firmware.items()):
                md_content.append(f"- **{model}**: `{firmware_file}`\n")
            md_content.append("\nIf you are experiencing problems, you can try to set the firmware of your product to this version and try again, and report the problem to us using the feedback page.\n")
        
        # Table of Contents
        md_content.append("## Table of Contents\n")
        md_content.append("1. [Install the Python Library](#install-the-python-library)")
        md_content.append("2. [Getting Started](#getting-started)")
        md_content.append("3. [Data Types](#data-types)")
        md_content.append("4. [Command Reference](#command-reference)")
        toc_index = 5
        for group in sorted(self.commands_by_group.keys()):
            anchor = group.lower().replace(' ', '-').replace('&', 'and')
            md_content.append(f"{toc_index}. [{group}](#{anchor})")
            toc_index += 1
        md_content.append(f"{toc_index}. [Unit Conversions](#unit-conversions)")
        toc_index += 1
        md_content.append(f"{toc_index}. [Error Handling](#error-handling)")
        toc_index += 1
        md_content.append(f"{toc_index}. [Error Codes](#error-codes)\n")
        
        # Install the Python Library Section
        md_content.append("## Install the Python Library\n")
        md_content.append("You need to install the servomotor Python library before you can use it in your code. Run this command:\n")
        md_content.append("```bash")
        md_content.append("pip3 install servomotor")
        md_content.append("```\n")
        md_content.append("If you want to work in a virtual environment then you can create it, activate it, and install the requirements. You can create the following requirements.txt file and then run the following commands:\n")
        md_content.append("**Create requirements.txt:**")
        md_content.append("```text")
        md_content.append("servomotor")
        md_content.append("```\n")
        md_content.append("**For macOS/Linux:**")
        md_content.append("```bash")
        md_content.append("# Create virtual environment")
        md_content.append("python3 -m venv venv")
        md_content.append("")
        md_content.append("# Activate virtual environment")
        md_content.append("source venv/bin/activate")
        md_content.append("")
        md_content.append("# Install requirements")
        md_content.append("pip3 install -r requirements.txt")
        md_content.append("```\n")
        md_content.append("**For Windows:**")
        md_content.append("```bash")
        md_content.append("# Create virtual environment")
        md_content.append("python -m venv venv")
        md_content.append("")
        md_content.append("# Activate virtual environment")
        md_content.append("venv\\Scripts\\activate")
        md_content.append("")
        md_content.append("# Install requirements")
        md_content.append("pip install -r requirements.txt")
        md_content.append("```\n")
        md_content.append("After installation, you can verify the servomotor library is installed correctly by running:")
        md_content.append("```python")
        md_content.append("python3 -c \"import servomotor; print('Servomotor library installed successfully!')\"")
        md_content.append("```\n")
        
        # Getting Started Section
        md_content.append("## Getting Started\n")
        md_content.append("This section provides a complete example showing how to initialize and control a servomotor.\n")
        md_content.append("### Complete Example Program\n")
        md_content.append("```python")
        
        # Read and include the example file
        try:
            with open('python_library_example.py', 'r') as f:
                example_code = f.read()
            md_content.append(example_code)
        except FileNotFoundError:
            md_content.append("# Example file not found")
        
        md_content.append("```\n")
        
        # Data Types Section
        md_content.append("## Data Types\n")
        md_content.append("This section describes the various data types used in the Servomotor API commands.\n")
        
        # Add data types information if loaded
        if self.data_types:
            # Separate integer and non-integer types
            integer_types = [dt for dt in self.data_types if dt.get('is_integer', False)]
            special_types = [dt for dt in self.data_types if not dt.get('is_integer', False)]
            
            md_content.append("### Integer Data Types\n")
            md_content.append("| Type | Size (bytes) | Range | Description |")
            md_content.append("|------|--------------|-------|-------------|")
            for dt in integer_types:
                min_val = dt.get('min_value', 'N/A')
                max_val = dt.get('max_value', 'N/A')
                if min_val != 'N/A' and max_val != 'N/A':
                    range_str = f"{min_val:,} to {max_val:,}"
                else:
                    range_str = "N/A"
                md_content.append(f"| {dt['data_type']} | {dt['size']} | {range_str} | {dt['description']} |")
            md_content.append("")
            
            md_content.append("### Special Data Types\n")
            md_content.append("| Type | Size (bytes) | Description |")
            md_content.append("|------|--------------|-------------|")
            for dt in special_types:
                size = dt.get('size', 'Variable')
                if size is None:
                    size = 'Variable'
                md_content.append(f"| {dt['data_type']} | {size} | {dt['description']} |")
            md_content.append("")
        
        # Command Reference
        md_content.append("## Command Reference\n")
        md_content.append("This section documents all available commands organized by category.\n")
        
        # Process each command group
        for group in sorted(self.commands_by_group.keys()):
            md_content.append(f"### {group}\n")
            
            for cmd in self.commands_by_group[group]:
                # Command name and description - make more prominent
                md_content.append(f"## 🔧 {cmd['CommandString']}\n")
                md_content.append(f"**Description:** {cmd['Description']}\n")
                
                # Parameters
                if cmd['Input'] and cmd['Input'] != "null":
                    md_content.append("**Parameters:**")
                    for param in cmd['Input']:
                        param_name = param.get('ParameterName', 'unknown')
                        param_desc = param.get('Description', 'No description')
                        md_content.append(f"- `{param_name}`: {param_desc}")
                        
                        # Add unit conversion info if available
                        if 'UnitConversion' in param:
                            unit_type = param['UnitConversion'].get('Type', '')
                            internal_unit = param['UnitConversion'].get('InternalUnit', '')
                            if unit_type:
                                md_content.append(f"  - Unit type: {unit_type} (internal: {internal_unit})")
                    md_content.append("")
                
                # Return values
                if cmd['Output'] and cmd['Output'] != "success_response":
                    md_content.append("**Returns:**")
                    if isinstance(cmd['Output'], list):
                        for output in cmd['Output']:
                            out_name = output.get('ParameterName', 'unknown')
                            out_desc = output.get('Description', 'No description')
                            md_content.append(f"- `{out_name}`: {out_desc}")
                    md_content.append("")
                
                # Python example
                md_content.append("**Example:**")
                md_content.append("```python")
                md_content.append(self.generate_python_example(cmd))
                md_content.append("```\n")
        
        # Unit Conversions Section
        md_content.append("## Unit Conversions\n")
        md_content.append("The servomotor library supports multiple unit systems for convenience.\n")
        md_content.append("### Position Units")
        md_content.append("- `encoder_counts`: Raw encoder counts (default)")
        md_content.append("- `shaft_rotations`: Rotations of the motor shaft")
        md_content.append("- `degrees`: Degrees of rotation")
        md_content.append("- `radians`: Radians of rotation\n")
        
        md_content.append("### Velocity Units")
        md_content.append("- `counts_per_second`: Encoder counts per second (default)")
        md_content.append("- `rotations_per_second`: Rotations per second")
        md_content.append("- `rpm`: Revolutions per minute")
        md_content.append("- `degrees_per_second`: Degrees per second")
        md_content.append("- `radians_per_second`: Radians per second\n")
        
        md_content.append("### Acceleration Units")
        md_content.append("- `counts_per_second_squared`: Encoder counts per second² (default)")
        md_content.append("- `rotations_per_second_squared`: Rotations per second²")
        md_content.append("- `rpm_per_second`: RPM per second")
        md_content.append("- `degrees_per_second_squared`: Degrees per second²")
        md_content.append("- `radians_per_second_squared`: Radians per second²\n")
        
        md_content.append("### Time Units")
        md_content.append("- `seconds`: Time in seconds")
        md_content.append("- `milliseconds`: Time in milliseconds")
        md_content.append("- `microseconds`: Time in microseconds\n")
        
        md_content.append("### Setting Units")
        md_content.append("You can set the units for a motor instance during initialization or at runtime:\n")
        md_content.append("```python")
        md_content.append("# During initialization")
        md_content.append("motor = servomotor.M3(")
        md_content.append("    alias='motor1',")
        md_content.append("    position_unit='degrees',")
        md_content.append("    velocity_unit='rpm',")
        md_content.append("    acceleration_unit='rpm_per_second'")
        md_content.append(")")
        md_content.append("")
        md_content.append("# At runtime")
        md_content.append("motor.set_position_unit('radians')")
        md_content.append("motor.set_velocity_unit('rotations_per_second')")
        md_content.append("```\n")
        
        # Error Handling Section
        md_content.append("## Error Handling\n")
        if hasattr(self, 'error_handling_text'):
            md_content.append(self.error_handling_text + "\n")
        else:
            md_content.append("Error handling description not available.\n")
        
        # Error Codes Section
        md_content.append("## Error Codes\n")
        md_content.append("This section lists all possible error codes that can be returned by the servomotor.\n")
        
        if self.error_codes:
            for error in self.error_codes:
                if error['code'] == 0:  # Skip ERROR_NONE
                    continue
                    
                md_content.append(f"### Error {error['code']}: {error['enum']}\n")
                md_content.append(f"**Short Description:** {error['short_desc']}\n")
                md_content.append(f"**Description:** {error['long_desc']}\n")
                
                if error.get('causes'):
                    md_content.append("**Possible Causes:**")
                    for cause in error['causes']:
                        md_content.append(f"- {cause}")
                    md_content.append("")
                
                if error.get('solutions'):
                    md_content.append("**Solutions:**")
                    for solution in error['solutions']:
                        md_content.append(f"- {solution}")
                    md_content.append("")
        else:
            md_content.append("Error codes not available.\n")
        
        # Write to file in parent directory
        output_file = "../M17_servomotor_Python_API_documentation.md"
        with open(output_file, 'w') as f:
            f.write('\n'.join(md_content))
        
        print(f"✓ Generated Python Markdown documentation: {output_file}")
        return True
    def generate_data_types_for_pdf(self, story, doc, heading_style, normal_style):
        """Generate Data Types section for PDF - shared by Python and Arduino"""
        story.append(Paragraph('Data Types', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This section describes the various data types used in the Servomotor API commands.', normal_style))
        story.append(Spacer(1, 12))
        
        # Separate integer and non-integer types
        integer_types = [dt for dt in self.data_types if dt.get('is_integer', False)]
        special_types = [dt for dt in self.data_types if not dt.get('is_integer', False)]
        
        # Integer Data Types
        story.append(Paragraph('<b>Integer Data Types</b>', heading_style))
        story.append(Spacer(1, 8))
        
        # Create style for table cells
        table_cell_style = ParagraphStyle(
            'TableCell',
            parent=normal_style,
            fontSize=9,
            leading=11
        )
        
        table_header_style = ParagraphStyle(
            'TableHeader',
            parent=normal_style,
            fontSize=10,
            fontName='Helvetica-Bold',
            textColor=colors.whitesmoke
        )
        
        # Create table for integer types
        int_table_data = [[
            Paragraph('Type', table_header_style),
            Paragraph('Size (bytes)', table_header_style),
            Paragraph('Range', table_header_style),
            Paragraph('Description', table_header_style)
        ]]
        
        for dt in integer_types:
            min_val = dt.get('min_value', 'N/A')
            max_val = dt.get('max_value', 'N/A')
            if min_val != 'N/A' and max_val != 'N/A':
                range_str = f"{min_val:,} to {max_val:,}"
            else:
                range_str = "N/A"
            
            int_table_data.append([
                Paragraph(dt['data_type'], table_cell_style),
                Paragraph(str(dt['size']), table_cell_style),
                Paragraph(range_str, table_cell_style),
                Paragraph(dt['description'], table_cell_style)
            ])
        
        int_table = Table(int_table_data, colWidths=[50, 50, 100, doc.width - 200])
        int_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#34a853')),
            ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
            ('GRID', (0, 0), (-1, -1), 1, colors.black),
            ('BOX', (0, 0), (-1, -1), 1, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ]))
        story.append(int_table)
        story.append(Spacer(1, 12))
        
        # Special Data Types
        story.append(Paragraph('<b>Special Data Types</b>', heading_style))
        story.append(Spacer(1, 8))
        
        # Create table for special types
        special_table_data = [[
            Paragraph('Type', table_header_style),
            Paragraph('Size (bytes)', table_header_style),
            Paragraph('Description', table_header_style)
        ]]
        
        for dt in special_types:
            size = dt.get('size', 'Variable')
            if size is None:
                size = 'Variable'
            
            special_table_data.append([
                Paragraph(dt['data_type'], table_cell_style),
                Paragraph(str(size), table_cell_style),
                Paragraph(dt['description'], table_cell_style)
            ])
        
        special_table = Table(special_table_data, colWidths=[70, 60, doc.width - 130])
        special_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#34a853')),
            ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
            ('GRID', (0, 0), (-1, -1), 1, colors.black),
            ('BOX', (0, 0), (-1, -1), 1, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ]))
        story.append(special_table)
        story.append(Spacer(1, 12))
    
    
    def generate_arduino_markdown(self):
        """Generate Arduino Markdown documentation"""
        print("\nGenerating Arduino Markdown documentation...")
        
        md_content = []
        md_content.append("# Servomotor Arduino API Documentation\n")
        md_content.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        # Add firmware information if available
        if self.latest_firmware:
            md_content.append("## Latest Firmware Versions\n")
            md_content.append("At the time of generating this API reference, the latest released firmware versions for the servomotors are:\n")
            for model, firmware_file in sorted(self.latest_firmware.items()):
                md_content.append(f"- **{model}**: `{firmware_file}`\n")
            md_content.append("\nIf you are experiencing problems, you can try to set the firmware of your product to this version and try again.\n")
        
        # Table of Contents
        md_content.append("## Table of Contents\n")
        md_content.append("1. [Getting Started](#getting-started)")
        md_content.append("2. [Data Types](#data-types)")
        md_content.append("3. [Command Reference](#command-reference)")
        toc_index = 4
        for group in sorted(self.commands_by_group.keys()):
            anchor = group.lower().replace(' ', '-').replace('&', 'and')
            md_content.append(f"{toc_index}. [{group}](#{anchor})")
            toc_index += 1
        md_content.append(f"{toc_index}. [Error Handling](#error-handling)")
        toc_index += 1
        md_content.append(f"{toc_index}. [Error Codes](#error-codes)\n")
        
        # Getting Started Section
        md_content.append("## Getting Started\n")
        md_content.append("This section provides a complete example showing how to control a servomotor with Arduino.\n")
        md_content.append("### Trapezoid Move Example\n")
        md_content.append("```cpp")
        
        # Read and include the Arduino example file
        try:
            with open('arduino_library_example.cpp', 'r') as f:
                example_code = f.read()
            md_content.append(example_code)
        except FileNotFoundError:
            md_content.append("// Example file arduino_library_example.cpp not found")
        
        md_content.append("```\n")
        
        # Data Types Section
        md_content.append("## Data Types\n")
        md_content.append("This section describes the various data types used in the Servomotor Arduino API.\n")
        
        # Add data types information if loaded
        if self.data_types:
            # Separate integer and non-integer types
            integer_types = [dt for dt in self.data_types if dt.get('is_integer', False)]
            special_types = [dt for dt in self.data_types if not dt.get('is_integer', False)]
            
            md_content.append("### Integer Data Types\n")
            md_content.append("| Type | Size (bytes) | Range | Description |")
            md_content.append("|------|--------------|-------|-------------|")
            for dt in integer_types:
                min_val = dt.get('min_value', 'N/A')
                max_val = dt.get('max_value', 'N/A')
                if min_val != 'N/A' and max_val != 'N/A':
                    range_str = f"{min_val:,} to {max_val:,}"
                else:
                    range_str = "N/A"
                md_content.append(f"| {dt['data_type']} | {dt['size']} | {range_str} | {dt['description']} |")
            md_content.append("")
            
            md_content.append("### Special Data Types\n")
            md_content.append("| Type | Size (bytes) | Description |")
            md_content.append("|------|--------------|-------------|")
            for dt in special_types:
                size = dt.get('size', 'Variable')
                if size is None:
                    size = 'Variable'
                md_content.append(f"| {dt['data_type']} | {size} | {dt['description']} |")
            md_content.append("")
        
        # Command Reference
        md_content.append("## Command Reference\n")
        md_content.append("This section documents all available commands organized by category.\n")
        
        # Generate commands by group for Arduino
        for group in sorted(self.commands_by_group.keys()):
            md_content.append(f"### {group}\n")
            
            for cmd in self.commands_by_group[group]:
                # Command name
                md_content.append(f"#### {cmd['CommandString']}\n")
                
                # Description
                md_content.append(f"**Description:** {cmd['Description']}\n")
                
                # Parameters
                if cmd['Input']:
                    md_content.append("**Parameters:**")
                    for param in cmd['Input']:
                        param_desc = param.get('Description', 'No description')
                        param_name = param.get('ParameterName', 'parameter')
                        md_content.append(f"- `{param_name}`: {param_desc}")
                    md_content.append("")
                
                # Return values
                if cmd['Output'] and cmd['Output'] != "success_response":
                    if isinstance(cmd['Output'], list):
                        md_content.append("**Returns:**")
                        for output in cmd['Output']:
                            out_name = output.get('ParameterName', 'unknown')
                            out_desc = output.get('Description', 'No description')
                            md_content.append(f"- `{out_name}`: {out_desc}")
                        md_content.append("")
                
                # Arduino example - use the proper generator function
                md_content.append("**Example:**")
                md_content.append("```cpp")
                example_code = self.generate_arduino_example(cmd)
                md_content.append(example_code)
                md_content.append("```\n")
        
        # Error Handling Section
        md_content.append("## Error Handling\n")
        if hasattr(self, 'error_handling_text'):
            for paragraph in self.error_handling_text.split('\n'):
                if paragraph.strip():
                    md_content.append(paragraph)
            md_content.append("")
        else:
            md_content.append("Error handling description not available.\n")
        
        # Error Codes Section
        md_content.append("## Error Codes\n")
        md_content.append("This section lists all possible error codes that can be returned by the servomotor.\n")
        
        if self.error_codes:
            md_content.append("| Code | Enum | Description |")
            md_content.append("|------|------|-------------|")
            for error in self.error_codes:
                if error['code'] == 0:  # Skip ERROR_NONE
                    continue
                md_content.append(f"| {error['code']} | {error['enum']} | {error['long_desc']} |")
            md_content.append("")
        
        # Write to file in parent directory
        output_file = "../M17_servomotor_Arduino_API_documentation.md"
        with open(output_file, 'w') as f:
            f.write('\n'.join(md_content))
        
        print(f"✓ Generated Arduino Markdown documentation: {output_file}")
        return True
    
    class CodeBox(Flowable):
        """A custom flowable for code blocks with light grey background and black border"""
        def __init__(self, text, width, style):
            Flowable.__init__(self)
            self.text = text
            self.width = width
            self.style = style
            self.height = 0
            self.line_height = 10  # Reduced line height for better fit
            
        def wrap(self, availWidth, availHeight):
            # Calculate height needed
            lines = self.text.split('\n')
            self.height = len(lines) * self.line_height + 16  # Line height + padding (8 top + 8 bottom)
            return (self.width, self.height)
            
        def draw(self):
            # Draw background rectangle with light grey fill
            self.canv.setFillColor(colors.HexColor('#f0f0f0'))
            self.canv.setStrokeColor(colors.black)
            self.canv.setLineWidth(0.5)
            self.canv.rect(0, 0, self.width, self.height, fill=1, stroke=1)
            
            # Draw text
            text_obj = self.canv.beginText(8, self.height - 12)  # Start 12 points from top
            text_obj.setFont('Courier', 8)
            text_obj.setFillColor(colors.black)
            
            for line in self.text.split('\n'):
                # Don't truncate, lines should already be properly wrapped
                text_obj.textLine(line)
            
            self.canv.drawText(text_obj)
    
    def generate_pdf(self):
        """Generate both Python and Arduino PDF documentation"""
        # Generate Python documentation
        self.generate_python_pdf()
        # Generate Arduino documentation
        self.generate_arduino_pdf()
        return True
    
    def generate_python_pdf(self):
        """Generate Python PDF documentation"""
        print("\nGenerating Python PDF documentation...")
        
        # Get version info for document content
        version, date_str = get_latest_version()
        # Use the specified filename format in parent directory
        output_filename = '../M17_servomotor_Python_API_documentation.pdf'
        
        # Create PDF document
        doc = SimpleDocTemplate(
            output_filename,
            pagesize=A4,
            rightMargin=18*mm,
            leftMargin=18*mm,
            topMargin=20*mm,
            bottomMargin=20*mm
        )
        
        # Initialize story
        story = []
        
        # Styles
        title_style = create_title_style()
        subtitle_style = create_subtitle_style()
        heading_style = create_heading_style()
        normal_style = create_normal_style()
        
        # Code style for examples
        code_style = ParagraphStyle(
            'Code',
            parent=getSampleStyleSheet()['Code'],
            fontName='Courier',
            fontSize=8,
            leftIndent=10,
            rightIndent=10,
            spaceBefore=6,
            spaceAfter=6,
            backColor=colors.HexColor('#f5f5f5')
        )
        
        # Command name style - larger and green (matching version/date color)
        command_style = ParagraphStyle(
            'CommandName',
            parent=normal_style,
            fontSize=14,
            textColor=colors.HexColor('#34a853'),  # Green color matching version/date
            fontName='Helvetica-Bold',
            spaceBefore=12,
            spaceAfter=6
        )
        
        # Title page
        story.append(Paragraph('Servomotor Python API Documentation', title_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph(f'Version {version}', subtitle_style))
        story.append(Paragraph(f'Generated: {datetime.now().strftime("%Y-%m-%d")}', subtitle_style))
        story.append(Spacer(1, 12))
        
        # Add firmware information if available
        if self.latest_firmware:
            firmware_style = ParagraphStyle(
                'FirmwareInfo',
                parent=normal_style,
                fontSize=10,
                textColor=colors.HexColor('#34a853'),
                spaceBefore=6,
                spaceAfter=6
            )
            
            story.append(Paragraph('<b>Latest Firmware Versions</b>', firmware_style))
            story.append(Spacer(1, 6))
            story.append(Paragraph('At the time of generating this API reference, the latest released firmware versions for the servomotors are:', normal_style))
            story.append(Spacer(1, 6))
            
            for model, firmware_file in sorted(self.latest_firmware.items()):
                story.append(Paragraph(f'• <b>{model}:</b> {firmware_file}', normal_style))
            
            story.append(Spacer(1, 8))
            story.append(Paragraph('If you are experiencing problems, you can try to set the firmware of your product to this version and try again, and report the problem to us using the feedback page.', normal_style))
        
        story.append(PageBreak())
        
        # Table of Contents
        story.append(Paragraph('Table of Contents', heading_style))
        story.append(Spacer(1, 12))
        
        # TOC style - black, bigger, and bold
        toc_style = ParagraphStyle(
            'TOCItem',
            parent=normal_style,
            fontSize=12,  # Bigger than normal (10)
            textColor=colors.black,
            fontName='Helvetica-Bold',
            leading=16,
            spaceBefore=2,
            spaceAfter=2
        )
        
        toc_items = ['1. Install the Python Library', '2. Getting Started', '3. Data Types', '4. Command Reference']
        toc_index = 5
        for group in sorted(self.commands_by_group.keys()):
            toc_items.append(f'   {toc_index}. {group}')
            toc_index += 1
        toc_items.append(f'{toc_index}. Unit Conversions')
        toc_index += 1
        toc_items.append(f'{toc_index}. Error Handling')
        toc_index += 1
        toc_items.append(f'{toc_index}. Error Codes')
        
        for item in toc_items:
            story.append(Paragraph(item, toc_style))
        story.append(PageBreak())
        
        # Install the Python Library Section
        story.append(Paragraph('Install the Python Library', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('You need to install the servomotor Python library before you can use it in your code. Run this command:', normal_style))
        story.append(Spacer(1, 8))
        
        # pip install command box
        install_cmd = "pip3 install servomotor"
        code_box = self.CodeBox(install_cmd, doc.width - 20, code_style)
        story.append(code_box)
        story.append(Spacer(1, 12))
        
        story.append(Paragraph('If you want to work in a virtual environment then you can create it, activate it, and install the requirements. You can create the following requirements.txt file and then run the following commands:', normal_style))
        story.append(Spacer(1, 8))
        
        story.append(Paragraph('<b>Create requirements.txt:</b>', normal_style))
        story.append(Spacer(1, 6))
        requirements_content = "servomotor"
        code_box = self.CodeBox(requirements_content, doc.width - 20, code_style)
        story.append(code_box)
        story.append(Spacer(1, 8))
        
        story.append(Paragraph('<b>For macOS/Linux:</b>', normal_style))
        story.append(Spacer(1, 6))
        macos_linux_cmds = """# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Install requirements
pip3 install -r requirements.txt"""
        code_box = self.CodeBox(macos_linux_cmds, doc.width - 20, code_style)
        story.append(code_box)
        story.append(Spacer(1, 8))
        
        story.append(Paragraph('<b>For Windows:</b>', normal_style))
        story.append(Spacer(1, 6))
        windows_cmds = """# Create virtual environment
python -m venv venv

# Activate virtual environment
venv\\Scripts\\activate

# Install requirements
pip install -r requirements.txt"""
        code_box = self.CodeBox(windows_cmds, doc.width - 20, code_style)
        story.append(code_box)
        story.append(Spacer(1, 12))
        
        story.append(Paragraph('After installation, you can verify the servomotor library is installed correctly by running:', normal_style))
        story.append(Spacer(1, 6))
        verify_cmd = 'python3 -c "import servomotor; print(\'Servomotor library installed successfully!\')"'
        code_box = self.CodeBox(verify_cmd, doc.width - 20, code_style)
        story.append(code_box)
        
        story.append(PageBreak())
        
        # Getting Started Section
        story.append(Paragraph('Getting Started', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This section provides a complete example showing how to initialize and control a servomotor.', normal_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('<b>Complete Example Program:</b>', normal_style))
        story.append(Spacer(1, 6))
        
        # Read and include the example file
        try:
            with open('python_library_example.py', 'r') as f:
                example_code = f.read()
            
            # Use CodeBox for the example
            code_box = self.CodeBox(example_code, doc.width - 20, code_style)
            story.append(code_box)
        except FileNotFoundError:
            story.append(Paragraph('Example file not found', normal_style))
        
        story.append(PageBreak())
        
        # Data Types Section - use the modular function
        self.generate_data_types_for_pdf(story, doc, heading_style, normal_style)
        
        story.append(PageBreak())
        
        # Command Reference
        story.append(Paragraph('Command Reference', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This section documents all available commands organized by category.', normal_style))
        story.append(Spacer(1, 12))
        
        # Subheading style - black, bigger, and bold
        subheading_style = ParagraphStyle(
            'SubHeading',
            parent=heading_style,
            fontSize=16,  # Bigger than normal headings
            textColor=colors.black,
            fontName='Helvetica-Bold',
            spaceBefore=12,
            spaceAfter=8
        )
        
        # Process each command group
        for group in sorted(self.commands_by_group.keys()):
            story.append(Paragraph(f'{group}', subheading_style))
            story.append(Spacer(1, 12))
            
            for cmd in self.commands_by_group[group]:
                # Keep command together on same page if possible
                cmd_content = []
                
                # Command name - use new prominent style
                cmd_content.append(Paragraph(cmd["CommandString"], command_style))
                cmd_content.append(Spacer(1, 6))
                
                # Description
                cmd_content.append(Paragraph(f'{cmd["Description"]}', normal_style))
                cmd_content.append(Spacer(1, 6))
                
                # Parameters
                if cmd['Input'] and cmd['Input'] != "null":
                    param_text = '<b>Parameters:</b><br/>'
                    for param in cmd['Input']:
                        param_name = param.get('ParameterName', 'unknown')
                        param_desc = param.get('Description', 'No description')
                        # Clean up description
                        param_desc = param_desc.replace('<', '&lt;').replace('>', '&gt;')
                        param_text += f'• <i>{param_name}</i>: {param_desc}<br/>'
                    cmd_content.append(Paragraph(param_text, normal_style))
                    cmd_content.append(Spacer(1, 6))
                
                # Return values
                if cmd['Output'] and cmd['Output'] != "success_response":
                    if isinstance(cmd['Output'], list):
                        output_text = '<b>Returns:</b><br/>'
                        for output in cmd['Output']:
                            out_name = output.get('ParameterName', 'unknown')
                            out_desc = output.get('Description', 'No description')
                            # Clean up description
                            out_desc = out_desc.replace('<', '&lt;').replace('>', '&gt;')
                            output_text += f'• <i>{out_name}</i>: {out_desc}<br/>'
                        cmd_content.append(Paragraph(output_text, normal_style))
                        cmd_content.append(Spacer(1, 6))
                
                # Python example with CodeBox
                cmd_content.append(Paragraph('<b>Example:</b>', normal_style))
                example_code = self.generate_python_example(cmd)
                code_box = self.CodeBox(example_code, doc.width - 20, code_style)
                cmd_content.append(code_box)
                cmd_content.append(Spacer(1, 12))
                
                # Add as KeepTogether to avoid splitting
                story.append(KeepTogether(cmd_content))
            
            story.append(PageBreak())
        
        # Unit Conversions Section
        story.append(Paragraph('Unit Conversions', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('The servomotor library supports multiple unit systems for convenience.', normal_style))
        story.append(Spacer(1, 12))
        
        units_info = [
            ('<b>Position Units:</b>', [
                'encoder_counts - Raw encoder counts (default)',
                'shaft_rotations - Rotations of the motor shaft',
                'degrees - Degrees of rotation',
                'radians - Radians of rotation'
            ]),
            ('<b>Velocity Units:</b>', [
                'counts_per_second - Encoder counts per second (default)',
                'rotations_per_second - Rotations per second',
                'rpm - Revolutions per minute',
                'degrees_per_second - Degrees per second',
                'radians_per_second - Radians per second'
            ]),
            ('<b>Acceleration Units:</b>', [
                'counts_per_second_squared - Encoder counts per second² (default)',
                'rotations_per_second_squared - Rotations per second²',
                'rpm_per_second - RPM per second',
                'degrees_per_second_squared - Degrees per second²',
                'radians_per_second_squared - Radians per second²'
            ]),
            ('<b>Time Units:</b>', [
                'seconds - Time in seconds',
                'milliseconds - Time in milliseconds',
                'microseconds - Time in microseconds'
            ])
        ]
        
        for title, units in units_info:
            story.append(Paragraph(title, normal_style))
            for unit in units:
                story.append(Paragraph(f'• {unit}', normal_style))
            story.append(Spacer(1, 12))
        
        # Setting units example
        story.append(Paragraph('<b>Setting Units:</b>', normal_style))
        story.append(Spacer(1, 6))
        story.append(Paragraph('You can set the units for a motor instance during initialization or at runtime:', normal_style))
        story.append(Spacer(1, 6))
        
        units_example = """# During initialization
motor = servomotor.M3(
    alias='motor1',
    position_unit='degrees',
    velocity_unit='rpm',
    acceleration_unit='rpm_per_second'
)

# At runtime
motor.set_position_unit('radians')
motor.set_velocity_unit('rotations_per_second')"""
        
        # Use CodeBox for the units example
        code_box = self.CodeBox(units_example, doc.width - 20, code_style)
        story.append(code_box)
        
        story.append(PageBreak())
        
        # Error Handling Section
        story.append(Paragraph('Error Handling', heading_style))
        story.append(Spacer(1, 12))
        if hasattr(self, 'error_handling_text'):
            # Split text into paragraphs for better formatting
            for paragraph in self.error_handling_text.split('\n'):
                if paragraph.strip():
                    story.append(Paragraph(paragraph, normal_style))
                    story.append(Spacer(1, 6))
        else:
            story.append(Paragraph('Error handling description not available.', normal_style))
        story.append(Spacer(1, 12))
        
        # Error Codes Section
        story.append(Paragraph('Error Codes', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This section lists all possible error codes that can be returned by the servomotor.', normal_style))
        story.append(Spacer(1, 12))
        
        if self.error_codes:
            # Create style for error code headers
            error_code_style = ParagraphStyle(
                'ErrorCode',
                parent=normal_style,
                fontSize=12,
                textColor=colors.HexColor('#d32f2f'),  # Red color for error codes
                fontName='Helvetica-Bold',
                spaceBefore=10,
                spaceAfter=6
            )
            
            error_label_style = ParagraphStyle(
                'ErrorLabel',
                parent=normal_style,
                fontSize=10,
                fontName='Helvetica-Bold',
                spaceBefore=4,
                spaceAfter=2
            )
            
            for error in self.error_codes:
                if error['code'] == 0:  # Skip ERROR_NONE
                    continue
                
                # Keep error information together
                error_content = []
                
                # Error header
                error_content.append(Paragraph(f"Error {error['code']}: {error['enum']}", error_code_style))
                
                # Short description
                if error.get('short_desc'):
                    error_content.append(Paragraph(f"<b>Short Description:</b> {error['short_desc']}", normal_style))
                    error_content.append(Spacer(1, 4))
                
                # Long description
                error_content.append(Paragraph(f"<b>Description:</b> {error['long_desc']}", normal_style))
                error_content.append(Spacer(1, 6))
                
                # Possible causes
                if error.get('causes') and len(error['causes']) > 0:
                    error_content.append(Paragraph('<b>Possible Causes:</b>', error_label_style))
                    causes_text = ''
                    for cause in error['causes']:
                        # Escape XML special characters
                        cause_escaped = cause.replace('<', '&lt;').replace('>', '&gt;')
                        causes_text += f'• {cause_escaped}<br/>'
                    error_content.append(Paragraph(causes_text, normal_style))
                    error_content.append(Spacer(1, 4))
                
                # Solutions
                if error.get('solutions') and len(error['solutions']) > 0:
                    error_content.append(Paragraph('<b>Solutions:</b>', error_label_style))
                    solutions_text = ''
                    for solution in error['solutions']:
                        # Escape XML special characters
                        solution_escaped = solution.replace('<', '&lt;').replace('>', '&gt;')
                        solutions_text += f'• {solution_escaped}<br/>'
                    error_content.append(Paragraph(solutions_text, normal_style))
                    error_content.append(Spacer(1, 8))
                
                # Add as KeepTogether to avoid splitting error codes
                story.append(KeepTogether(error_content))
        else:
            story.append(Paragraph('Error codes not available.', normal_style))
        
        # Build PDF
        doc.build(story)
        print(f"✓ Generated Python PDF documentation: {output_filename}")
        return True
    
    def generate_arduino_pdf(self):
        """Generate Arduino PDF documentation"""
        print("\nGenerating Arduino PDF documentation...")
        
        # Get version info for document content
        version, date_str = get_latest_version()
        # Output to parent directory
        output_filename = '../M17_servomotor_Arduino_API_documentation.pdf'
        
        # Create PDF document
        doc = SimpleDocTemplate(
            output_filename,
            pagesize=A4,
            rightMargin=18*mm,
            leftMargin=18*mm,
            topMargin=20*mm,
            bottomMargin=20*mm
        )
        
        # Initialize story
        story = []
        
        # Styles
        title_style = create_title_style()
        subtitle_style = create_subtitle_style()
        heading_style = create_heading_style()
        normal_style = create_normal_style()
        
        # Code style for examples
        code_style = ParagraphStyle(
            'Code',
            parent=getSampleStyleSheet()['Code'],
            fontName='Courier',
            fontSize=8,
            leftIndent=10,
            rightIndent=10,
            spaceBefore=6,
            spaceAfter=6,
            backColor=colors.HexColor('#f5f5f5')
        )
        
        # Command name style
        command_style = ParagraphStyle(
            'CommandName',
            parent=normal_style,
            fontSize=14,
            textColor=colors.HexColor('#34a853'),
            fontName='Helvetica-Bold',
            spaceBefore=12,
            spaceAfter=6
        )
        
        # Title page
        story.append(Paragraph('Servomotor Arduino API Documentation', title_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph(f'Version {version}', subtitle_style))
        story.append(Paragraph(f'Generated: {datetime.now().strftime("%Y-%m-%d")}', subtitle_style))
        story.append(Spacer(1, 12))
        
        # Add firmware information if available
        if self.latest_firmware:
            firmware_style = ParagraphStyle(
                'FirmwareInfo',
                parent=normal_style,
                fontSize=10,
                textColor=colors.HexColor('#34a853'),
                spaceBefore=6,
                spaceAfter=6
            )
            
            story.append(Paragraph('<b>Latest Firmware Versions</b>', firmware_style))
            story.append(Spacer(1, 6))
            story.append(Paragraph('At the time of generating this API reference, the latest released firmware versions for the servomotors are:', normal_style))
            story.append(Spacer(1, 6))
            
            for model, firmware_file in sorted(self.latest_firmware.items()):
                story.append(Paragraph(f'• <b>{model}:</b> {firmware_file}', normal_style))
            
            story.append(Spacer(1, 8))
        
        story.append(PageBreak())
        
        # Table of Contents
        story.append(Paragraph('Table of Contents', heading_style))
        story.append(Spacer(1, 12))
        
        # TOC style
        toc_style = ParagraphStyle(
            'TOC',
            parent=normal_style,
            fontSize=12,
            textColor=colors.black,
            fontName='Helvetica-Bold',
            spaceBefore=6,
            spaceAfter=6
        )
        
        story.append(Paragraph('1. Getting Started', toc_style))
        story.append(Paragraph('2. Data Types', toc_style))
        story.append(Paragraph('3. Command Reference', toc_style))
        
        toc_index = 4
        for group in sorted(self.commands_by_group.keys()):
            story.append(Paragraph(f'{toc_index}. {group}', toc_style))
            toc_index += 1
        
        story.append(Paragraph(f'{toc_index}. Error Handling', toc_style))
        story.append(Paragraph(f'{toc_index + 1}. Error Codes', toc_style))
        
        story.append(PageBreak())
        
        # Getting Started Section
        story.append(Paragraph('Getting Started', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This example demonstrates a trapezoid move with the servomotor:', normal_style))
        story.append(Spacer(1, 12))
        
        # Read and include the Arduino example file in a grey box
        try:
            with open('arduino_library_example.cpp', 'r') as f:
                example_code = f.read()
            # Use CodeBox to display in grey box
            code_box = self.CodeBox(example_code, doc.width - 20, code_style)
            story.append(code_box)
        except FileNotFoundError:
            story.append(Paragraph('Example file arduino_library_example.cpp not found', normal_style))
        
        story.append(PageBreak())
        
        # Data Types Section - use the same modular function as Python
        self.generate_data_types_for_pdf(story, doc, heading_style, normal_style)
        
        story.append(PageBreak())
        
        # Command Reference section
        story.append(Paragraph('Command Reference', heading_style))
        story.append(Spacer(1, 12))
        
        for group in sorted(self.commands_by_group.keys()):
            story.append(Paragraph(group, heading_style))
            story.append(Spacer(1, 12))
            
            for cmd in self.commands_by_group[group]:
                # Keep command information together
                cmd_content = []
                
                # Command name
                cmd_content.append(Paragraph(cmd['CommandString'], command_style))
                
                # Description
                cmd_content.append(Paragraph(cmd['Description'], normal_style))
                cmd_content.append(Spacer(1, 6))
                
                # Parameters
                if cmd['Input']:
                    param_text = '<b>Parameters:</b><br/>'
                    for param in cmd['Input']:
                        param_desc = param.get('Description', 'No description')
                        param_name = param.get('ParameterName', 'parameter')
                        param_desc = param_desc.replace('<', '&lt;').replace('>', '&gt;')
                        param_text += f'• <i>{param_name}</i>: {param_desc}<br/>'
                    cmd_content.append(Paragraph(param_text, normal_style))
                    cmd_content.append(Spacer(1, 6))
                
                # Arduino example - use the proper generator function
                cmd_content.append(Paragraph('<b>Example:</b>', normal_style))
                example_code = self.generate_arduino_example(cmd)
                
                code_box = self.CodeBox(example_code, doc.width - 20, code_style)
                cmd_content.append(code_box)
                cmd_content.append(Spacer(1, 12))
                
                # Add as KeepTogether
                story.append(KeepTogether(cmd_content))
            
            story.append(PageBreak())
        
        # Build PDF
        doc.build(story)
        print(f"✓ Generated Arduino PDF documentation: {output_filename}")
        return True
    
    def run(self):
        """Main execution function"""
        print("\n" + "="*80)
        print("SERVOMOTOR API DOCUMENTATION GENERATOR")
        print("="*80)
        
        # Load commands
        if not self.load_commands():
            sys.exit(1)
        
        # Load data types
        if not self.load_data_types():
            sys.exit(1)
        
        # Load error codes
        if not self.load_error_codes():
            print("⚠️  Warning: Error codes not loaded. Documentation will be generated without error codes.")
        
        # Load error handling text
        if not self.load_error_handling_text():
            print("⚠️  Warning: Error handling text not loaded. Using default text.")
        
        # Find latest firmware versions
        self.find_latest_firmware()
        
        # Validate commands
        if not self.validate_commands():
            print("\n⚠️  Documentation generation aborted due to validation errors.")
            print("Please fix the errors above and run again.")
            sys.exit(1)
        
        # Generate documentation
        success = True
        try:
            if not self.generate_markdown():
                success = False
        except Exception as e:
            print(f"❌ Error generating Markdown: {e}")
            success = False
        
        try:
            if not self.generate_pdf():
                success = False
        except Exception as e:
            print(f"❌ Error generating PDF: {e}")
            success = False
        
        if success:
            print("\n" + "="*80)
            print("✅ DOCUMENTATION GENERATION COMPLETE!")
            print("="*80)
            print("\nGenerated files (in parent directory):")
            print("  • ../M17_servomotor_Python_API_documentation.md")
            print("  • ../M17_servomotor_Python_API_documentation.pdf")
            print("  • ../M17_servomotor_Arduino_API_documentation.md")
            print("  • ../M17_servomotor_Arduino_API_documentation.pdf")
        else:
            print("\n❌ Documentation generation failed. Please check the errors above.")
            sys.exit(1)

if __name__ == "__main__":
    generator = APIDocumentationGenerator()
    generator.run()