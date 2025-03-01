#!/usr/bin/env python3
import json
import re
from datetime import datetime
import sys

# Paths to JSON files
MOTOR_COMMANDS_JSON_FILE = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/motor_commands.json"
DATA_TYPES_JSON_FILE = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/data_types.json"
UNIT_CONVERSIONS_JSON_FILE = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/unit_conversions_M3.json"
COMMANDS_H_FILE = "Commands.h"
SERVOMOTOR_CPP_FILE = "ServomotorCommands.cpp"
SERVOMOTOR_H_FILE = "ServomotorCommands.h"

def get_unit_info(unit_conversions_data):
    """Generate unit information from unit_conversions_M3.json"""
    units_dict = unit_conversions_data["units"]
    conversion_mapping = {}
    default_unit = {}
    
    for unit_type, units in units_dict.items():
        # Create enum name (e.g., "time" -> "TimeUnit")
        enum_name = unit_type.capitalize() + "Unit"
        # Create member variable name (e.g., "time" -> "m_timeUnit")
        member_name = "m_" + unit_type + "Unit"
        # Create conversion function name (e.g., "time" -> "convertTime")
        conv_func = "convert" + unit_type.capitalize()
        
        conversion_mapping[unit_type] = (conv_func, conv_func, member_name, enum_name)
        default_unit[unit_type] = enum_name
    
    return conversion_mapping, default_unit

# Mapping of normalized command strings to response type names.
response_struct_mappings = {
    'get status': ('StatusResponse', ['statusFlags', 'fatalErrorCode']),
    'get position': 'getHallSensorPositionResponse',
    'get n queued items': 'uint8_t',
    'get product description': 'getProductDescriptionResponse',
    'get current time': 'getCurrentTimeResponse',
    'get hall sensor position': 'getHallSensorPositionResponse',
    'get supply voltage': 'getSupplyVoltageResponse',
    'get max pid error': 'getMaxPidErrorResponse',
    'get temperature': 'getTemperatureResponse',
    'get debug values': 'getDebugValuesResponse',
    'ping': 'pingResponse',
    'get comprehensive position': 'getComprehensivePositionResponse'
}
norm_response_map = {k.strip().lower(): v for k, v in response_struct_mappings.items()}

# Built-in types that should not be retypedef'd.
builtin_types = {"int8_t", "uint8_t", "int16_t", "uint16_t",
                 "int32_t", "uint32_t", "int64_t", "uint64_t",
                 "float", "double"}

# Set to track defined response typedefs.
defined_responses = set()

# ---------------- Helper Functions ----------------

def format_command_name(command_string, mappings):
    if command_string in mappings:
        return mappings[command_string]
    else:
        command_string = command_string.strip()
        command_string = re.sub(r'[^a-zA-Z0-9]+', ' ', command_string)
        words = command_string.strip().split()
        if not words:
            return ''
        return words[0].lower() + ''.join(word.capitalize() for word in words[1:])

def extract_parameters(param_list, command_string, data_type_map, reserved_words):
    if not param_list or param_list == 'success_response':
        return []
    params = []
    used_names = set()
    index = 1
    for item in param_list:
        description = item.get('Description', '')
        parameter_name = item.get('ParameterName')
        match = re.match(r'(\w+):\s*(.*)', description)
        if match:
            type_str = match.group(1)
            desc = match.group(2)
            if parameter_name:
                param_name = parameter_name
            else:
                param_name = extract_param_name(desc, index)
            param_name = make_unique_name(param_name, used_names)
            used_names.add(param_name)
            mapped_type = map_type(type_str, data_type_map)
            if isinstance(mapped_type, tuple):
                base_type, array_size = mapped_type
                type_name = base_type
            else:
                type_name = mapped_type
                array_size = None
            unit_conv = item.get('UnitConversion')
            params.append({'type': type_name,
                           'name': param_name,
                           'array_size': array_size,
                           'description': desc,
                           'original_type': type_str,
                           'UnitConversion': unit_conv})
            index += 1
        else:
            continue
    return params

def extract_param_name(description, index):
    stop_words = {'the', 'a', 'an', 'this', 'that', 'these', 'those', 'his', 'her', 'its', 'of', 'and', 'or', 'to', 'with', 'for', 'from'}
    words = description.strip().split()
    while words and words[0].lower() in stop_words:
        words.pop(0)
    name_match = re.match(r'([\w_]+)\s*[:\-]\s*(.*)', ' '.join(words))
    if name_match:
        param_name = name_match.group(1)
    elif words:
        param_name = words[0]
    else:
        param_name = f'param{index}'
    param_name = re.sub(r'\W|^(?=\d)', '_', param_name)
    return param_name

def make_unique_name(name, used_names):
    original_name = name
    count = 2
    while name in used_names or is_reserved_word(name):
        name = f"{original_name}_{count}"
        count += 1
    return name

def is_reserved_word(word):
    reserved = {
        'alignas', 'alignof', 'and', 'and_eq', 'asm', 'atomic_cancel', 'atomic_commit',
        'atomic_noexcept', 'auto', 'bitand', 'bitor', 'bool', 'break', 'case', 'catch',
        'char', 'char8_t', 'char16_t', 'char32_t', 'class', 'compl', 'concept', 'const',
        'consteval', 'constexpr', 'const_cast', 'continue', 'co_await', 'co_return',
        'co_yield', 'decltype', 'default', 'delete', 'do', 'double', 'dynamic_cast', 'else',
        'enum', 'explicit', 'export', 'extern', 'false', 'float', 'for', 'friend', 'goto',
        'if', 'inline', 'int', 'long', 'mutable', 'namespace', 'new', 'noexcept', 'not',
        'not_eq', 'nullptr', 'operator', 'or', 'or_eq', 'private', 'protected', 'public',
        'register', 'reinterpret_cast', 'requires', 'return', 'short', 'signed', 'sizeof',
        'static', 'static_assert', 'static_cast', 'struct', 'switch', 'template', 'this',
        'thread_local', 'throw', 'true', 'try', 'typedef', 'typeid', 'typename', 'union',
        'unsigned', 'using', 'virtual', 'void', 'volatile', 'wchar_t', 'while', 'xor',
        'xor_eq'
    }
    return word in reserved

def map_type(type_str, data_type_map):
    standard_type_map = {
        'i8': 'int8_t',
        'u8': 'uint8_t',
        'i16': 'int16_t',
        'u16': 'uint16_t',
        'i32': 'int32_t',
        'u32': 'uint32_t',
        'i64': 'int64_t',
        'u64': 'uint64_t',
        'float': 'float',
        'double': 'double',
        'string8': ('char', 8),
        'string_null_term': ('char', 32),
        'u24_version_number': 'uint32_t',
        'u32_version_number': 'uint32_t',
        'u64_unique_id': 'uint64_t',
        'crc32': 'uint32_t',
        'u48': 'uint64_t',
    }
    if type_str in standard_type_map:
        return standard_type_map[type_str]
    elif type_str in data_type_map:
        dt_info = data_type_map[type_str]
        size = dt_info['size']
        if size is not None and size > 0:
            if size == 1:
                return 'uint8_t'
            else:
                return ('uint8_t', size)
        else:
            return 'uint8_t'
    else:
        return 'uint8_t'

def format_struct_member_type(param):
    return param['type']

def format_function_param_type(param):
    if param.get('array_size'):
        return f'{param["type"]} {param["name"]}[{param["array_size"]}]'
    else:
        return f'{param["type"]} {param["name"]}'

def format_return_type(param):
    if param.get('array_size'):
        return f'{param["type"]}[{param["array_size"]}]'
    else:
        return f'{param["type"]}'

def is_integer_type(type_name):
    return type_name in {'int8_t', 'uint8_t',
                         'int16_t', 'uint16_t',
                         'int32_t', 'uint32_t',
                         'int64_t', 'uint64_t'}

def get_size_of_type(type_name, data_type_map, array_size=None):
    sizes = {
        'int8_t': 1, 'uint8_t': 1, 'int16_t': 2, 'uint16_t': 2,
        'int32_t': 4, 'uint32_t': 4, 'int64_t': 8, 'uint64_t': 8,
        'float': 4, 'double': 8, 'char': 1,
    }
    base_type = type_name.strip('*')
    base_size = sizes.get(base_type, 1)
    return base_size * array_size if array_size else base_size

def get_default_value(type_name):
    if 'int' in type_name or 'float' in type_name or 'double' in type_name:
        return '0'
    elif '*' in type_name or '[' in type_name:
        return 'nullptr'
    else:
        return f'{type_name}()'

def back_up_files(files):
    for file in files:
        backup_filename = f"{file}.{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.bak"
        with open(file, 'r') as f:
            contents = f.read()
        with open(backup_filename, 'w') as f:
            f.write(contents)
        print(f"Backed up {file} to {backup_filename}")
    sys.stdout.flush()

def command_needs_conversion(command):
    if command.get('Input'):
        for item in command['Input']:
            if item.get('UnitConversion'):
                return True
    if isinstance(command.get('Output'), list):
        for item in command['Output']:
            if item.get('UnitConversion'):
                return True
    return False

def get_wrapper_param(param):
    if param.get('UnitConversion'):
        return "float " + param['name']
    else:
        return format_function_param_type(param)

def get_wrapper_return(output_params, cmdStr, func_name):
    if cmdStr.strip().lower() in {"get current time", "get hall sensor position", "get supply voltage", "get temperature"}:
        return "float"
    for op in output_params:
        if op.get("UnitConversion"):
            return "float"
    if not output_params:
        return "void"
    norm_cmd = cmdStr.strip().lower()
    if norm_cmd in norm_response_map:
        val = norm_response_map[norm_cmd]
        if isinstance(val, tuple):
            return val[0]
        else:
            return val
    if len(output_params) == 1 and not output_params[0].get('array_size'):
        return format_return_type(output_params[0])
    else:
        return f'{func_name}Response'

def convert_internal_unit_str(unit_str, conv_type, conversion_mapping, default_unit):
    """Convert internal unit string to enum value"""
    if conversion_mapping[conv_type][3]:
        return f"{conversion_mapping[conv_type][3]}::{unit_str.upper()}"
    else:
        return f"(({default_unit[conv_type]})0)"

# ---------------- Generation Functions ----------------

def generate_commands_header(json_file, header_file):
    with open(json_file, 'r') as f:
        commands_data = json.load(f)
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(header_file, 'w') as hf:
        hf.write('// Commands.h\n')
        hf.write(f'// This file was autogenerated by generate_command_code.py on {now}\n')
        hf.write('// Do not edit this file manually.\n\n')
        hf.write('#ifndef COMMANDS_H\n')
        hf.write('#define COMMANDS_H\n\n')
        hf.write('enum CommandID {\n')
        for command in commands_data:
            command_name = command['CommandString'].upper()
            command_name = re.sub(r'[^a-zA-Z0-9]+', '_', command_name)
            command_id = command['CommandEnum']
            hf.write(f'    CMD_{command_name} = {command_id},\n')
        hf.write('};\n\n')
        hf.write('#endif // COMMANDS_H\n')
    print(f'Generated {header_file} successfully.')
    sys.stdout.flush()

def generate_servo_motor_files(json_file, data_types_file, header_file, source_file):
    with open(json_file, 'r') as f:
        commands_data = json.load(f)
    with open(data_types_file, 'r') as f:
        data_types_data = json.load(f)
    with open(UNIT_CONVERSIONS_JSON_FILE, 'r') as f:
        unit_conversions_data = json.load(f)
    commands = commands_data
    data_types = data_types_data
    data_type_map = {dt['data_type']: dt for dt in data_types}
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    function_name_mappings = {
        'Disable MOSFETs': 'disableMosfets',
        'Enable MOSFETs': 'enableMosfets',
        'Get n queued items': 'getNumberOfQueuedItems',
        'Get position': 'getPosition',
        'Get status': 'getStatus',
        'System reset': 'systemReset',
        'Set PID constants': 'setPIDConstants',
    }
    reserved_words = {
        'alignas', 'alignof', 'and', 'and_eq', 'asm', 'atomic_cancel', 'atomic_commit',
        'atomic_noexcept', 'auto', 'bitand', 'bitor', 'bool', 'break', 'case', 'catch',
        'char', 'char8_t', 'char16_t', 'char32_t', 'class', 'compl', 'concept', 'const',
        'consteval', 'constexpr', 'const_cast', 'continue', 'co_await', 'co_return',
        'co_yield', 'decltype', 'default', 'delete', 'do', 'double', 'dynamic_cast', 'else',
        'enum', 'explicit', 'export', 'extern', 'false', 'float', 'for', 'friend', 'goto',
        'if', 'inline', 'int', 'long', 'mutable', 'namespace', 'new', 'noexcept', 'not',
        'not_eq', 'nullptr', 'operator', 'or', 'or_eq', 'private', 'protected', 'public',
        'register', 'reinterpret_cast', 'requires', 'return', 'short', 'signed', 'sizeof',
        'static', 'static_assert', 'static_cast', 'struct', 'switch', 'template', 'this',
        'thread_local', 'throw', 'true', 'try', 'typedef', 'typeid', 'typename', 'union',
        'unsigned', 'using', 'virtual', 'void', 'volatile', 'wchar_t', 'while', 'xor',
        'xor_eq'
    }
    defined_responses = set()

    # Generate header file
    with open(header_file, 'w') as hf:
        hf.write(f'// {SERVOMOTOR_H_FILE}\n')
        hf.write(f'// This file was autogenerated by generate_command_code.py on {now}\n')
        hf.write('// Do not edit this file manually.\n\n')
        hf.write('#ifndef SERVOMOTOR_H\n')
        hf.write('#define SERVOMOTOR_H\n\n')
        hf.write('#ifdef ARDUINO\n#include <Arduino.h>\n#endif\n')
        hf.write('#include "Communication.h"\n')
        hf.write('#include "Commands.h"\n')
        hf.write('#include "DataTypes.h"\n')
        hf.write('#include "Utils.h"\n')
        hf.write('#include "AutoGeneratedUnitConversions.h"\n\n')
        hf.write('// Payload and Response Structures\n\n')
        for command in commands:
            func_name = format_command_name(command['CommandString'], function_name_mappings)
            input_params = extract_parameters(command.get('Input'), command['CommandString'], data_type_map, reserved_words)
            if input_params:
                struct_name = f'{func_name}Payload'
                hf.write(f'// Structure for {command["CommandString"]} command payload\n')
                hf.write('typedef struct __attribute__((__packed__)) {\n')
                for param in input_params:
                    param_type = format_struct_member_type(param)
                    param_name = param['name']
                    if param.get('array_size'):
                        hf.write(f'    {param_type} {param_name}[{param["array_size"]}];\n')
                    else:
                        hf.write(f'    {param_type} {param_name};\n')
                hf.write(f'}} {struct_name};\n\n')
            if command.get('Output') and command['Output'] != 'success_response':
                output_params = extract_parameters(command.get('Output'), command['CommandString'], data_type_map, reserved_words)
                norm_cmd = command['CommandString'].strip().lower()
                if norm_cmd in norm_response_map:
                    ret_type = norm_response_map[norm_cmd] if isinstance(norm_response_map[norm_cmd], str) else norm_response_map[norm_cmd][0]
                else:
                    if len(output_params) == 1 and output_params[0].get('array_size'):
                        ret_type = format_return_type(output_params[0])
                    elif len(output_params)==1:
                        ret_type = format_return_type(output_params[0])
                    else:
                        ret_type = f'{func_name}Response'
                if ret_type not in builtin_types and not ret_type.endswith("*") and ret_type not in defined_responses:
                    hf.write(f'// Structure for {command["CommandString"]} command response\n')
                    hf.write('typedef struct __attribute__((__packed__)) {\n')
                    for param in output_params:
                        param_type = format_struct_member_type(param)
                        param_name = param['name']
                        if param.get('array_size'):
                            hf.write(f'    {param_type} {param_name}[{param["array_size"]}];\n')
                        else:
                            hf.write(f'    {param_type} {param_name};\n')
                    hf.write(f'}} {ret_type};\n\n')
                    defined_responses.add(ret_type)
        hf.write('class ServoMotor {\npublic:\n')
        hf.write('    ServoMotor(uint8_t alias = \'X\', HardwareSerial& serialPort = Serial1);\n')
        hf.write('    void setAlias(uint8_t new_alias);\n')
        hf.write('    uint8_t getAlias();\n')
        hf.write('    void openSerialPort();\n')
        hf.write('    int getError() const;\n\n')
        # Generate unit setting methods
        hf.write('    // Unit settings\n')
        conversion_mapping, default_unit = get_unit_info(unit_conversions_data)
        for unit_type, (_, _, member_name, enum_name) in conversion_mapping.items():
            hf.write(f'    void set{unit_type.capitalize()}Unit({enum_name} unit);\n')
        hf.write('\n')
        for command in commands:
            func_name = format_command_name(command['CommandString'], function_name_mappings)
            input_params = extract_parameters(command.get('Input'), command['CommandString'], data_type_map, reserved_words)
            params = [format_function_param_type(p) for p in input_params]
            norm_cmd = command['CommandString'].strip().lower()
            if command.get('Output') and command['Output'] != 'success_response':
                output_params = extract_parameters(command.get('Output'), command['CommandString'], data_type_map, reserved_words)
                if norm_cmd in norm_response_map:
                    raw_ret = norm_response_map[norm_cmd] if isinstance(norm_response_map[norm_cmd], str) else norm_response_map[norm_cmd][0]
                else:
                    if len(output_params)==1 and output_params[0].get('array_size'):
                        raw_ret = format_return_type(output_params[0])
                    elif len(output_params)==1:
                        raw_ret = format_return_type(output_params[0])
                    else:
                        raw_ret = f'{func_name}Response'
            else:
                raw_ret = "void"
            if command_needs_conversion(command):
                wrapper_ret = get_wrapper_return(extract_parameters(command.get("Output"), command["CommandString"], data_type_map, reserved_words), command["CommandString"], func_name)
                hf.write(f'    {raw_ret} {func_name}Raw({", ".join(params)});\n')
                hf.write(f'    {wrapper_ret} {func_name}({", ".join([get_wrapper_param(p) for p in input_params])});\n')
            else:
                hf.write(f'    {raw_ret} {func_name}({", ".join(params)});\n')
            if command.get('MultipleResponses'):
                hf.write(f'    {raw_ret} {func_name}GetAnotherResponse();\n')
        hf.write('\nprivate:\n')
        hf.write('    uint8_t _alias;\n')
        hf.write('    Communication _comm;\n')
        hf.write('    int _errno;\n\n')
        # Generate unit member variables
        hf.write('    // Unit settings\n')
        for unit_type, (_, _, member_name, enum_name) in conversion_mapping.items():
            first_unit = unit_conversions_data["units"][unit_type][0].upper()
            hf.write(f'    {enum_name} {member_name} = {enum_name}::{first_unit};\n')
        hf.write('\n')
        hf.write('    // Initialization\n')
        hf.write('    bool _initialized;\n')
        hf.write('    void ensureInitialized();\n')
        hf.write('};\n\n')
        hf.write('#endif // SERVOMOTOR_H\n')
    print(f'Generated {header_file} successfully.')
    sys.stdout.flush()

    # Generate source file
    with open(source_file, 'w') as sf:
        sf.write(f'// {SERVOMOTOR_CPP_FILE}\n')
        sf.write(f'// This file was autogenerated by generate_command_code.py on {now}\n')
        sf.write('// Do not edit this file manually.\n\n')
        sf.write(f'#include "{SERVOMOTOR_H_FILE}"\n')
        sf.write('#include "Commands.h"\n')
        sf.write('#include "Utils.h"\n\n')
        sf.write('ServoMotor::ServoMotor(uint8_t alias, HardwareSerial& serialPort)\n')
        sf.write('    : _alias(alias), _comm(serialPort), _errno(0),\n')
        # Initialize unit members
        sf.write('      // Initialize unit settings\n')
        first = True
        for unit_type, (_, _, member_name, enum_name) in conversion_mapping.items():
            first_unit = unit_conversions_data["units"][unit_type][0].upper()
            if first:
                sf.write(f'      {member_name}({enum_name}::{first_unit})')
                first = False
            else:
                sf.write(f',\n      {member_name}({enum_name}::{first_unit})')
        sf.write(' {\n')
        sf.write('    openSerialPort();\n')
        sf.write('}\n\n')
        sf.write('void ServoMotor::setAlias(uint8_t new_alias) {\n')
        sf.write('    _alias = new_alias;\n')
        sf.write('}\n\n')
        sf.write('uint8_t ServoMotor::getAlias() {\n')
        sf.write('    return _alias;\n')
        sf.write('}\n\n')
        sf.write('void ServoMotor::openSerialPort() {\n')
        sf.write('    _comm.openSerialPort();\n')
        sf.write('}\n\n')
        sf.write('int ServoMotor::getError() const {\n')
        sf.write('    return _errno;\n')
        sf.write('}\n\n')
        # Generate unit setting function implementations
        sf.write('// Unit setting functions\n')
        for unit_type, (_, _, member_name, enum_name) in conversion_mapping.items():
            sf.write(f'void ServoMotor::set{unit_type.capitalize()}Unit({enum_name} unit) {{\n')
            sf.write(f'    {member_name} = unit;\n')
            sf.write(f'    Serial.print("[Motor] set{unit_type.capitalize()}Unit to ");\n')
            sf.write('    switch(unit) {\n')
            for unit_name in unit_conversions_data["units"][unit_type]:
                unit_enum = unit_name.upper()
                sf.write(f'        case {enum_name}::{unit_enum}: Serial.println("{unit_enum}"); break;\n')
            sf.write('    }\n')
            sf.write('}\n\n')
        # Command implementations
        for command in commands:
            func_name = format_command_name(command['CommandString'], function_name_mappings)
            input_params = extract_parameters(command.get('Input'), command['CommandString'], data_type_map, reserved_words)
            params = [format_function_param_type(p) for p in input_params]
            norm_cmd = command['CommandString'].strip().lower()
            if command.get('Output') and command['Output'] != 'success_response':
                output_params = extract_parameters(command.get('Output'), command['CommandString'], data_type_map, reserved_words)
                if norm_cmd in norm_response_map:
                    raw_ret = norm_response_map[norm_cmd] if isinstance(norm_response_map[norm_cmd], str) else norm_response_map[norm_cmd][0]
                else:
                    if len(output_params)==1 and output_params[0].get('array_size'):
                        raw_ret = format_return_type(output_params[0])
                    elif len(output_params)==1:
                        raw_ret = format_return_type(output_params[0])
                    else:
                        raw_ret = f'{func_name}Response'
            else:
                raw_ret = "void"
            if command_needs_conversion(command):
                sf.write(f'{raw_ret} ServoMotor::{func_name}Raw({", ".join(params)}) {{\n')
                sf.write(f'    // {command["Description"]} (Raw version)\n')
                sf.write(f'    const uint8_t commandID = {command["CommandEnum"]};\n')
                if input_params:
                    struct_name = f'{func_name}Payload'
                    sf.write(f'    {struct_name} payload;\n')
                    for param in input_params:
                        if param.get('array_size'):
                            sf.write(f'    memcpy(payload.{param["name"]}, {param["name"]}, sizeof(payload.{param["name"]}));\n')
                        elif is_integer_type(param["type"]) and get_size_of_type(param["type"], data_type_map, param.get("array_size")) > 1:
                            endian_func = f'htole{get_size_of_type(param["type"], data_type_map, param.get("array_size"))*8}'
                            sf.write(f'    payload.{param["name"]} = {endian_func}({param["name"]});\n')
                        else:
                            sf.write(f'    payload.{param["name"]} = {param["name"]};\n')
                    sf.write(f'    _comm.sendCommand(_alias, commandID, (uint8_t*)&payload, sizeof(payload));\n')
                else:
                    sf.write(f'    _comm.sendCommand(_alias, commandID, nullptr, 0);\n')
                if command.get('Output') and command['Output'] != 'success_response':
                    if norm_cmd in norm_response_map:
                        outType = norm_response_map[norm_cmd] if isinstance(norm_response_map[norm_cmd], str) else norm_response_map[norm_cmd][0]
                    else:
                        if len(output_params)==1:
                            outType = format_return_type(output_params[0])
                        else:
                            outType = f'{func_name}Response'
                    sf.write(f'    uint8_t buffer[sizeof({outType})];\n')
                    sf.write('    uint16_t receivedSize;\n')
                    sf.write('    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);\n')
                    sf.write('    if (_errno == 0) {\n')
                    sf.write(f'        if (receivedSize == sizeof({outType})) {{\n')
                    sf.write(f'            {outType}* response = ({outType}*)buffer;\n')
                    sf.write('            return *response;\n')
                    sf.write('        } else {\n')
                    sf.write('            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;\n')
                    sf.write('        }\n')
                    sf.write('    }\n')
                    sf.write(f'    {outType} defaultResponse = {{0}};\n')
                    sf.write('    return defaultResponse;\n')
                else:
                    sf.write('    uint8_t buffer[1];\n')
                    sf.write('    uint16_t receivedSize;\n')
                    sf.write('    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);\n')
                sf.write('}\n\n')
                # Wrapper function implementation
                wrapper_params = [get_wrapper_param(p) for p in input_params]
                wf_ret = get_wrapper_return(extract_parameters(command.get("Output"), command["CommandString"], data_type_map, reserved_words), command["CommandString"], func_name)
                sf.write(f'{wf_ret} ServoMotor::{func_name}({", ".join(wrapper_params)}) {{\n')
                sf.write(f'    Serial.println("[Motor] {func_name} called.");\n')
                for param in input_params:
                    if param.get('UnitConversion'):
                        sf.write(f'    Serial.print("  {param["name"]} in chosen unit: "); Serial.println({param["name"]});\n')
                sf.write('\n')
                arg_list = []
                for param in input_params:
                    if param.get('UnitConversion'):
                        conv = param['UnitConversion']
                        conv_type = conv["Type"]
                        conv_func = conversion_mapping[conv_type][0]
                        unit_member = conversion_mapping[conv_type][2]
                        sf.write(f'    float {param["name"]}_internal = {conv_func}({param["name"]}, {unit_member}, ConversionDirection::TO_INTERNAL);\n')
                        arg_list.append(f'({param["type"]})({param["name"]}_internal)')
                    else:
                        arg_list.append(param["name"])
                if wf_ret != "void":
                    sf.write(f'    auto rawResult = {func_name}Raw({", ".join(arg_list)});\n')
                    if norm_cmd == "get current time":
                        sf.write('    float convertedResult = convertTime((float)rawResult.currentTime, m_timeUnit, ConversionDirection::FROM_INTERNAL);\n')
                    elif norm_cmd == "get hall sensor position" or norm_cmd == "get position":
                        sf.write('    float convertedResult = convertPosition((float)rawResult.hallSensorPosition, m_positionUnit, ConversionDirection::FROM_INTERNAL);\n')
                    elif norm_cmd == "get supply voltage":
                        sf.write('    float convertedResult = convertVoltage((float)rawResult.supplyVoltage, m_voltageUnit, ConversionDirection::FROM_INTERNAL);\n')
                    elif norm_cmd == "get temperature":
                        sf.write('    float convertedResult = convertTemperature((float)rawResult.temperature, m_temperatureUnit, ConversionDirection::FROM_INTERNAL);\n')
                    else:
                        sf.write('    float convertedResult = 0; // Unknown conversion\n')
                    sf.write('    return convertedResult;\n')
                else:
                    sf.write(f'    {func_name}Raw({", ".join(arg_list)});\n')
                sf.write('}\n\n')
            else:
                sf.write(f'{raw_ret} ServoMotor::{func_name}({", ".join(params)}) {{\n')
                sf.write('    Serial.println("[Motor] ' + func_name + ' called.");\n')
                sf.write(f'    // {command["Description"]}\n')
                sf.write(f'    const uint8_t commandID = {command["CommandEnum"]};\n')
                if input_params:
                    struct_name = f'{func_name}Payload'
                    sf.write(f'    {struct_name} payload;\n')
                    for param in input_params:
                        if param.get('array_size'):
                            sf.write(f'    memcpy(payload.{param["name"]}, {param["name"]}, sizeof(payload.{param["name"]}));\n')
                        elif is_integer_type(param["type"]) and get_size_of_type(param["type"], data_type_map, param.get("array_size")) > 1:
                            endian_func = f'htole{get_size_of_type(param["type"], data_type_map, param.get("array_size"))*8}'
                            sf.write(f'    payload.{param["name"]} = {endian_func}({param["name"]});\n')
                        else:
                            sf.write(f'    payload.{param["name"]} = {param["name"]};\n')
                    sf.write(f'    _comm.sendCommand(_alias, commandID, (uint8_t*)&payload, sizeof(payload));\n')
                else:
                    sf.write(f'    _comm.sendCommand(_alias, commandID, nullptr, 0);\n')
                if command.get("Output") and command["Output"] != "success_response":
                    output_params = extract_parameters(command.get("Output"), command["CommandString"], data_type_map, reserved_words)
                    if norm_cmd in norm_response_map:
                        outType = norm_response_map[norm_cmd] if isinstance(norm_response_map[norm_cmd], str) else norm_response_map[norm_cmd][0]
                    else:
                        if len(output_params) == 1:
                            outType = format_return_type(output_params[0])
                        else:
                            outType = f'{func_name}Response'
                    sf.write(f'    uint8_t buffer[sizeof({outType})];\n')
                    sf.write('    uint16_t receivedSize;\n')
                    sf.write('    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);\n')
                    sf.write('    if (_errno == 0) {\n')
                    sf.write(f'        if (receivedSize == sizeof({outType})) {{\n')
                    sf.write(f'            {outType}* response = ({outType}*)buffer;\n')
                    sf.write('            return *response;\n')
                    sf.write('        } else {\n')
                    sf.write('            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;\n')
                    sf.write('        }\n')
                    sf.write('    }\n')
                    sf.write(f'    {outType} defaultResponse = {{0}};\n')
                    sf.write('    return defaultResponse;\n')
                else:
                    sf.write('    uint8_t buffer[1];\n')
                    sf.write('    uint16_t receivedSize;\n')
                    sf.write('    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);\n')
                sf.write('}\n\n')
            if command.get("MultipleResponses"):
                sf.write(f'{raw_ret} ServoMotor::{func_name}GetAnotherResponse() {{\n')
                sf.write(f'    // Attempt to receive another response from previously sent {command["CommandString"]} command\n')
                if command.get("Output") and command["Output"] != "success_response":
                    if norm_cmd in norm_response_map:
                        local_ret = norm_response_map[norm_cmd] if isinstance(norm_response_map[norm_cmd], str) else norm_response_map[norm_cmd][0]
                    else:
                        local_ret = f'{func_name}Response'
                    sf.write(f'    uint8_t buffer[sizeof({local_ret})];\n')
                    sf.write('    uint16_t receivedSize;\n')
                    sf.write('    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);\n')
                    sf.write('    if (_errno == 0) {\n')
                    sf.write(f'        if (receivedSize == sizeof({local_ret})) {{\n')
                    sf.write(f'            {local_ret}* response = ({local_ret}*)buffer;\n')
                    sf.write('            return *response;\n')
                    sf.write('        } else {\n')
                    sf.write('            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;\n')
                    sf.write('        }\n')
                    sf.write('    }\n')
                    sf.write(f'    return {local_ret}();\n')
                sf.write('}\n\n')
        print(f'Generated {header_file} and {source_file} successfully.')
        sys.stdout.flush()

if __name__ == '__main__':
    print("This script will generate the following files:")
    print(f"  {COMMANDS_H_FILE}")
    print(f"  {SERVOMOTOR_CPP_FILE}")
    print(f"  {SERVOMOTOR_H_FILE}")
    if '--no-backup' not in sys.argv:
        print("The existing files will be backed up and then overwritten. You can give the --no-backup command line option to not make any backups.")
        backup_flag = True
    else:
        print("The --no-backup command line option was given. No backups will be made.")
        backup_flag = False
    if '-y' in sys.argv:
        print("The -y command line option was given. Skipping the prompt and promptly going ahead with the generation.")
    else:
        print("You can give the -y command line option to skip the following prompt.")
        print("Do you want to continue? (y/n)")
        response = input()
        if response.lower() != 'y':
            print("Exiting.")
            exit(0)
    if backup_flag:
        back_up_files([COMMANDS_H_FILE, SERVOMOTOR_CPP_FILE, SERVOMOTOR_H_FILE])
    generate_commands_header(MOTOR_COMMANDS_JSON_FILE, COMMANDS_H_FILE)
    generate_servo_motor_files(MOTOR_COMMANDS_JSON_FILE, DATA_TYPES_JSON_FILE, SERVOMOTOR_H_FILE, SERVOMOTOR_CPP_FILE)
