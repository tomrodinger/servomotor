# Servomotor API Documentation Generator

## Overview
This project generates comprehensive API documentation for the Servomotor Python library. The documentation is auto-generated from the `motor_commands.json` file and produces both PDF and Markdown outputs.

## Features
- **Auto-generated Documentation**: Reads command definitions from `motor_commands.json` and data type definitions from `data_types.json`
- **Getting Started Section**: Includes complete example program from `python_library_example.py` showing how to get the motor up and running
- **Data Types Section**: Comprehensive documentation of all data types used in the API, including:
  - Integer data types (i8, i16, i32, u8, u16, u32, u48, etc.) with size and range information
  - Special data types (strings, buffers, version numbers, CRC, etc.)
- **Error Handling Documentation**: Comprehensive error handling information section explaining how the servomotor handles errors
- **Error Codes Reference**: Complete listing of all error codes with:
  - Error code number and enum name
  - Short and detailed descriptions
  - Possible causes
  - Recommended solutions
- **Command Validation**: Ensures all commands have categories defined before generating documentation
- **Validation Error Handling**: Prominent error display if any command lacks a category, with instructions on how to fix
- **Code Examples**: Provides Python code examples for each motor command
- **Dual Output Formats**:
  - PDF for printable/shareable documentation
  - Markdown for AI-friendly and version-control friendly documentation
- **Command Grouping**: Organizes commands by category (Basic Control, Motion Control, Status & Monitoring, etc.)
- **Unit Conversion Support**: Documents available unit options for position, velocity, acceleration, etc.
- **Enhanced Styling**:
  - Command names displayed in green color matching the version/date header
  - Table of contents items in black, bold, and larger font for better readability
  - Section headings in black, bold, and larger font (18pt) for clear hierarchy

## Usage
```bash
python generate_api_documentation.py
```

This will generate:
- `servomotor_api_documentation_v{version}_{date}.pdf` - PDF version
- `servomotor_api_documentation.md` - Markdown version

## Configuration Files

The generator uses several input files:
- `motor_commands.json` - Motor command definitions
- `data_types.json` - Data type definitions
- `error_codes.json` - Error code definitions with causes and solutions
- `error_handling.txt` - Error handling description text

## Validation Error Handling
If any command in `motor_commands.json` is missing a `CommandGroup` field:
- The program will display a prominent error message
- List all commands that lack categories
- Provide instructions to fix the issue (add CommandGroup to motor_commands.json)
- Exit without generating documentation

## Requirements
- Python 3.x
- ReportLab (for PDF generation)
- Motor commands definition file at: `../../python_programs/servomotor/motor_commands.json`
- Data types definition file at: `../../python_programs/servomotor/data_types.json`
- Error codes definition file at: `../../python_programs/servomotor/error_codes.json`
- Error handling text file at: `error_handling.txt`

## Documentation Contents
The generated documentation includes:
1. **Getting Started Guide** - Complete working example showing how to initialize and control the motor
2. **Data Types Reference** - Comprehensive documentation of all data types used in the API:
   - Integer types with size and range specifications
   - Special types including strings, buffers, and protocol-specific types
3. **Complete Command Reference** - All commands with descriptions and parameters
4. **Parameter Specifications** - Data types and units for all command parameters
5. **Return Value Documentation** - Output specifications for commands that return data
6. **Practical Code Examples** - Python snippets for each command
7. **Unit Conversion Information** - Available units for position, velocity, acceleration, etc.
8. **Command Grouping by Functionality** - Commands organized by categories for easy navigation
9. **Error Handling Guide** - Detailed explanation of how the servomotor handles error conditions
10. **Error Codes Reference** - Complete listing of all error codes with causes and solutions