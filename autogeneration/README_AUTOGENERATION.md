# Code Autogeneration System

This document describes the code autogeneration system used to generate command-related code from JSON definitions.

## Core Principles

1. **Data-Driven Code Generation**: All command-specific code is derived from JSON data files, not hardcoded in templates or generation scripts.
2. **No Command-Specific Logic**: Generation modules must NEVER contain ANY hardcoded command names or special-case handling for specific commands.
3. **Separation of Templates and Generation Logic**: Templates define structure, generation modules produce content.
4. **Type-Based Processing**: Processing logic should be based on data types and patterns, not on specific command names.

## File Structure

The autogeneration system consists of:

1. **JSON Data Files**:
   - `motor_commands.json`: Contains command definitions, parameters, and types
   - `data_types.json`: Contains data type definitions and sizes

2. **Generation Modules**:
   - `generate_command_methods.py`: Generates method declarations for commands
   - `generate_payload_structures.py`: Generates payload and response structure declarations
   - Other modules for generating specific parts of the code

3. **Templates**:
   - `Servomotor.h.template`: Template for Servomotor.h with insertion points
   - Other templates for different output files

4. **Main Generator Script**:
   - `generate_command_code_new2.py`: Main script that orchestrates the generation process

## How It Works

1. The main generator script loads the JSON data files.
2. It reads the templates and identifies insertion points using markers like `{{generate_command_methods}}`.
3. For each insertion point, it calls the appropriate generation module with the JSON data.
4. The generation module processes the data and returns content to be inserted.
5. The generator substitutes the content into the template.
6. The completed files are written to the output location.

## Adding New Commands

To add a new command:

1. Add the command definition to `motor_commands.json` with:
   - `CommandString`: Human-readable name for the command
   - `CommandEnum`: Numeric ID for the command
   - `Description`: Detailed description of what the command does
   - `Input`: Array of input parameters (if any)
   - `Output`: Array of output parameters or "success_response"
   - `CommandGroup`: Logical grouping for the command

2. For each parameter, specify:
   - `Description`: String in format `"type: description"` (e.g., "u32: The maximum velocity")
   - `ParameterName`: Name to use in code for this parameter
   - `UnitConversion`: (Optional) For parameters that need unit conversion

3. For unit conversion, specify:
   ```json
   "UnitConversion": {
     "Type": "position",  // Type of unit (position, time, velocity, etc.)
     "InternalUnit": "encoder_counts",  // Unit used internally
     "ConversionFactorsFile": "unit_conversions_{motor_type}.json"
   }
   ```

4. For special complex parameter types like list_2d with mixed unit conversions, use:
   ```json
   "UnitConversion": {
     "Type": "mixed_acceleration_velocity_time",
     "InternalUnit": "mixed_acceleration_velocity_time",
     "ConversionFactorsFile": "unit_conversions_{motor_type}.json"
   }
   ```

## Generator Module Rules

When creating or modifying generator modules, always follow these rules:

### 1. NO HARDCODED COMMAND NAMES OR MAPPINGS

```python
# WRONG - DO NOT DO THIS:
function_name_mappings = {
    'Disable MOSFETs': 'disableMosfets',
    'Enable MOSFETs': 'enableMosfets',
    'Get n queued items': 'getNumberOfQueuedItems',
    'Get position': 'getPosition',
    'Get status': 'getStatus',
    'System reset': 'systemReset',
    'Set PID constants': 'setPIDConstants',
    'Multi move': 'multiMove',
}
```

This approach is brittle, requires updating the mapping whenever a new command is added, and won't work for new projects with different commands.

### 2. NO SPECIAL CASE HANDLING FOR SPECIFIC COMMANDS

```python
# WRONG - DO NOT DO THIS:
if cmd_str == "Multimove":
    # Special handling for Multimove
    list_type_name = "multiMoveList_t"
    converted_list_type = "multiMoveListConverted_t"
```

Special case handling is a maintenance nightmare and won't work for new projects.

### 3. USE CONSISTENT ALGORITHMIC APPROACHES

```python
# CORRECT APPROACH:
def format_command_name(command_string):
    command_string = command_string.strip()
    command_string = re.sub(r'[^a-zA-Z0-9]+', ' ', command_string)
    words = command_string.split()
    if not words:
        return ''
    
    # First word lowercase, rest capitalized
    return words[0].lower() + ''.join(w.capitalize() for w in words[1:])
```

This algorithm consistently transforms command strings to camelCase function names.

### 4. DETECT PATTERNS IN THE DATA

```python
# CORRECT APPROACH:
for param in cmd.get('Input', []):
    if param.get('UnitConversion'):
        needs_unit_conversion = True
        if param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
            has_mixed_conversion = True
```

Use the data itself to determine behavior, not hardcoded command names.

### 5. USE TYPE-BASED PROCESSING

```python
# CORRECT APPROACH:
if type_str == 'list_2d' or param.get('UnitConversion', {}).get('Type') == 'mixed_acceleration_velocity_time':
    # Generate list type name from command name
    list_type_name = format_list_type_name(cmd_str)
    converted_list_type = format_list_type_name(cmd_str, converted=True)
```

Process based on parameter types and attributes, not command names.

## Common Mistakes to Avoid

1. **Hardcoding Response Structure Names**: Don't hardcode `getComprehensivePositionResponseConverted` - derive it algorithmically from the command name.

2. **Hardcoding List Type Names**: Don't hardcode `multiMoveList_t` - derive it algorithmically from the command name.

3. **Hardcoding Special Command Processing**: Don't add special cases for commands like `Multimove` - detect patterns in the data instead.

4. **Embedding Command-Specific Structures in Templates**: Don't embed structures like `multiMoveList_t` directly in templates - generate all structures from data.

By following these principles, the code generation system will remain flexible, maintainable, and able to handle any commands added in the future without requiring modifications to the generator code itself.