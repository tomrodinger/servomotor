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
SERVOMOTOR_CPP_FILE = "Servomotor.cpp"
SERVOMOTOR_H_FILE = "Servomotor.h"

def get_unit_info(unit_conversions_data):
    """Generate unit information from unit_conversions_M3.json."""
    units_dict = unit_conversions_data["units"]
    conversion_mapping = {}
    default_unit = {}
    
    for unit_type, units in units_dict.items():
        # e.g. "time" -> "TimeUnit"
        enum_name = unit_type.capitalize() + "Unit"
        # e.g. "time" -> "m_timeUnit"
        member_name = "m_" + unit_type + "Unit"
        # e.g. "time" -> "convertTime"
        conv_func = "convert" + unit_type.capitalize()
        
        conversion_mapping[unit_type] = (conv_func, conv_func, member_name, enum_name)
        default_unit[unit_type] = enum_name
    
    return conversion_mapping, default_unit

# A known mapping of certain "get" commands to special response types in C++
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

# Built-in C/C++ types we don't want to redefine
builtin_types = {
    "int8_t", "uint8_t", "int16_t", "uint16_t",
    "int32_t", "uint32_t", "int64_t", "uint64_t",
    "float", "double"
}

def format_command_name(command_string, mappings):
    """Map 'Disable MOSFETs' => 'disableMosfets' etc."""
    if command_string in mappings:
        return mappings[command_string]
    command_string = command_string.strip()
    command_string = re.sub(r'[^a-zA-Z0-9]+', ' ', command_string)
    words = command_string.split()
    if not words:
        return ''
    return words[0].lower() + ''.join(w.capitalize() for w in words[1:])

def extract_param_name(description, index):
    stop_words = {
        'the','a','an','this','that','these','those','his','her','its',
        'of','and','or','to','with','for','from'
    }
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

def is_reserved_word(word):
    reserved = {
        'alignas','alignof','and','and_eq','asm','atomic_cancel','atomic_commit',
        'atomic_noexcept','auto','bitand','bitor','bool','break','case','catch',
        'char','char8_t','char16_t','char32_t','class','compl','concept','const',
        'consteval','constexpr','const_cast','continue','co_await','co_return',
        'co_yield','decltype','default','delete','do','double','dynamic_cast','else',
        'enum','explicit','export','extern','false','float','for','friend','goto',
        'if','inline','int','long','mutable','namespace','new','noexcept','not',
        'not_eq','nullptr','operator','or','or_eq','private','protected','public',
        'register','reinterpret_cast','requires','return','short','signed','sizeof',
        'static','static_assert','static_cast','struct','switch','template','this',
        'thread_local','throw','true','try','typedef','typeid','typename','union',
        'unsigned','using','virtual','void','volatile','wchar_t','while','xor','xor_eq'
    }
    return word in reserved

def make_unique_name(name, used_names):
    original_name = name
    count = 2
    while name in used_names or is_reserved_word(name):
        name = f"{original_name}_{count}"
        count += 1
    return name

def map_type(type_str, data_type_map):
    standard_map = {
        'i8':'int8_t','u8':'uint8_t','i16':'int16_t','u16':'uint16_t',
        'i32':'int32_t','u32':'uint32_t','i64':'int64_t','u64':'uint64_t',
        'float':'float','double':'double',
        'string8':('char',8),
        'string_null_term':('char',32),
        'u24_version_number':'uint32_t',
        'u32_version_number':'uint32_t',
        'u64_unique_id':'uint64_t',
        'crc32':'uint32_t',
        'u48':'uint64_t',
    }
    if type_str in standard_map:
        return standard_map[type_str]
    elif type_str in data_type_map:
        # Possibly a custom data type
        dt_info = data_type_map[type_str]
        size = dt_info['size']
        if size and size > 0:
            # E.g. size=10 => char[10], or something
            if size == 1:
                return 'uint8_t'
            else:
                return ('uint8_t', size)
        else:
            return 'uint8_t'
    else:
        return 'uint8_t'

def extract_parameters(param_list, command_string, data_type_map, reserved_words):
    """Extract (type, name) info from a list of param definitions."""
    if not param_list or param_list == 'success_response':
        return []
    params = []
    used_names = set()
    idx = 1
    for item in param_list:
        desc = item.get('Description','')
        param_name_given = item.get('ParameterName')
        match = re.match(r'(\w+):\s*(.*)', desc)
        if match:
            type_str = match.group(1)
            remainder = match.group(2)
            if param_name_given:
                param_name = param_name_given
            else:
                param_name = extract_param_name(remainder, idx)
            param_name = make_unique_name(param_name, used_names)
            used_names.add(param_name)
            mapped_type = map_type(type_str, data_type_map)
            if isinstance(mapped_type, tuple):
                base_t, arr_size = mapped_type
                tname = base_t
            else:
                tname = mapped_type
                arr_size = None
            unit_conv = item.get('UnitConversion')
            params.append({
                'type': tname,
                'name': param_name,
                'array_size': arr_size,
                'description': remainder,
                'original_type': type_str,
                'UnitConversion': unit_conv
            })
            idx += 1
    return params

def format_struct_member_type(param):
    return param['type']

def format_function_param_type(param):
    """Produce e.g. 'int32_t displacement' or 'char name[32]'."""
    if param.get('array_size'):
        return f'{param["type"]} {param["name"]}[{param["array_size"]}]'
    else:
        return f'{param["type"]} {param["name"]}'

def command_needs_conversion(command):
    """True if any input or output param has a 'UnitConversion'."""
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
    """Wrapper uses float for any parameter with a UnitConversion, else same type."""
    if param.get('UnitConversion'):
        return f'float {param["name"]}'
    else:
        return format_function_param_type(param)

def get_wrapper_return(output_params, cmdStr, func_name):
    """
    Determine the return type for the *wrapper* version.
    Example: if the command is 'get temperature', we return 'float' in the wrapper.
    """
    norm_cmd = cmdStr.strip().lower()
    # Hardcode special multi-float structs
    if norm_cmd == "get comprehensive position":
        return "getComprehensivePositionResponseConverted"
    if norm_cmd == "get max pid error":
        return "getMaxPidErrorResponseConverted"
    # Hardcode single-float returns
    if norm_cmd in {
        "get current time", "get hall sensor position",
        "get position", "get supply voltage", "get temperature"
    }:
        return "float"
    # If no output params => void
    if not output_params:
        return "void"
    # If command is in norm_response_map => use that type
    if norm_cmd in norm_response_map:
        val = norm_response_map[norm_cmd]
        return val[0] if isinstance(val, tuple) else val
    # If exactly 1 output param, and not an array => return that param type
    if len(output_params) == 1 and not output_params[0].get('array_size'):
        ptype = output_params[0]['type']
        return ptype
    # Otherwise define a custom struct name, e.g. {func_name}Response
    return f'{func_name}Response'

def get_size_of_type(type_name, data_type_map, array_size=None):
    sizes = {
        'int8_t':1,'uint8_t':1,'int16_t':2,'uint16_t':2,'int32_t':4,'uint32_t':4,
        'int64_t':8,'uint64_t':8,'float':4,'double':8,'char':1
    }
    base_t = type_name.strip('*')
    base_size = sizes.get(base_t,1)
    return base_size * array_size if array_size else base_size

def back_up_files(files):
    for file in files:
        try:
            with open(file,'r') as f:
                contents = f.read()
            backup_filename = f"{file}.{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.bak"
            with open(backup_filename,'w') as bf:
                bf.write(contents)
            print(f"Backed up {file} to {backup_filename}")
        except FileNotFoundError:
            pass  # If it doesn't exist, skip
    sys.stdout.flush()

def generate_commands_header(json_file, header_file):
    with open(json_file,'r') as f:
        commands_data = json.load(f)
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(header_file,'w') as hf:
        hf.write('// Commands.h\n')
        hf.write(f'// This file was autogenerated by generate_command_code.py on {now}\n')
        hf.write('// Do not edit manually.\n\n')
        hf.write('#ifndef COMMANDS_H\n#define COMMANDS_H\n\n')
        hf.write('enum CommandID {\n')
        for cmd in commands_data:
            cmd_name = cmd['CommandString'].upper()
            cmd_name = re.sub(r'[^a-zA-Z0-9]+','_',cmd_name)
            cmd_id = cmd['CommandEnum']
            hf.write(f'    CMD_{cmd_name} = {cmd_id},\n')
        hf.write('};\n\n#endif // COMMANDS_H\n')
    print(f'Generated {header_file} successfully.')

def generate_servo_motor_files(json_file, data_types_file, header_file, source_file):
    with open(json_file,'r') as f:
        commands_data = json.load(f)
    with open(data_types_file,'r') as f:
        data_types_data = json.load(f)
    with open(UNIT_CONVERSIONS_JSON_FILE,'r') as f:
        unit_conversions_data = json.load(f)

    data_type_map = {dt['data_type']: dt for dt in data_types_data}
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # Custom mappings for certain commands
    function_name_mappings = {
        'Disable MOSFETs':'disableMosfets',
        'Enable MOSFETs':'enableMosfets',
        'Get n queued items':'getNumberOfQueuedItems',
        'Get position':'getPosition',
        'Get status':'getStatus',
        'System reset':'systemReset',
        'Set PID constants':'setPIDConstants',
    }

    # Build the .h file
    with open(header_file,'w') as hf:
        hf.write(f'// {SERVOMOTOR_H_FILE}\n')
        hf.write(f'// This file was autogenerated by generate_command_code.py on {now}\n')
        hf.write('// Do not edit manually.\n\n')
        hf.write('#ifndef SERVOMOTOR_H\n#define SERVOMOTOR_H\n\n')
        hf.write('#ifdef ARDUINO\n#include <Arduino.h>\n#endif\n')
        hf.write('#include "Communication.h"\n#include "Commands.h"\n')
        hf.write('#include "DataTypes.h"\n#include "Utils.h"\n#include "AutoGeneratedUnitConversions.h"\n\n')
        hf.write('// Payload and Response Structures\n\n')

        # We track which response structs we've already typedef'd
        defined_responses = set()

        # Generate command-specific payload/response structs
        for cmd in commands_data:
            func_name = format_command_name(cmd['CommandString'], function_name_mappings)
            input_params = extract_parameters(cmd.get('Input'), cmd['CommandString'], data_type_map, set())

            # If we have input params => define {func_name}Payload
            if input_params:
                struct_name = f'{func_name}Payload'
                hf.write(f'// Structure for {cmd["CommandString"]} command payload\n')
                hf.write('typedef struct __attribute__((__packed__)) {\n')
                for p in input_params:
                    if p.get('array_size'):
                        hf.write(f'    {p["type"]} {p["name"]}[{p["array_size"]}];\n')
                    else:
                        hf.write(f'    {p["type"]} {p["name"]};\n')
                hf.write(f'}} {struct_name};\n\n')

            # If we have a real output (not "success_response"), define the response struct if needed
            if cmd.get('Output') and cmd['Output'] != 'success_response':
                output_params = extract_parameters(cmd['Output'], cmd['CommandString'], data_type_map, set())
                norm_cmd = cmd['CommandString'].strip().lower()
                if norm_cmd in norm_response_map:
                    out_type = norm_response_map[norm_cmd]
                    # Could be a tuple, e.g. ('StatusResponse',['statusFlags','fatalErrorCode'])
                    out_type_name = out_type[0] if isinstance(out_type, tuple) else out_type
                else:
                    # If there's exactly one param or a potential array
                    if len(output_params) == 1 and output_params[0].get('array_size'):
                        out_type_name = output_params[0]['type']+f'[{output_params[0]["array_size"]}]'
                    elif len(output_params) == 1:
                        out_type_name = output_params[0]['type']
                    else:
                        out_type_name = f'{func_name}Response'
                # If not built-in and not previously defined
                if (out_type_name not in builtin_types and
                    not out_type_name.endswith('*') and
                    out_type_name not in defined_responses and
                    '[' not in out_type_name):
                    hf.write(f'// Structure for {cmd["CommandString"]} command response\n')
                    hf.write('typedef struct __attribute__((__packed__)) {\n')
                    for op in output_params:
                        if op.get('array_size'):
                            hf.write(f'    {op["type"]} {op["name"]}[{op["array_size"]}];\n')
                        else:
                            hf.write(f'    {op["type"]} {op["name"]};\n')
                    hf.write(f'}} {out_type_name};\n\n')
                    defined_responses.add(out_type_name)

        # Also manually define the "converted" structs used for getComprehensivePosition and getMaxPidError
        hf.write('// Structure for converted comprehensive position values\n')
        hf.write('typedef struct {\n')
        hf.write('    float commandedPosition;\n')
        hf.write('    float hallSensorPosition;\n')
        hf.write('    float externalEncoderPosition;\n')
        hf.write('} getComprehensivePositionResponseConverted;\n\n')

        hf.write('// Structure for converted max PID error values\n')
        hf.write('typedef struct {\n')
        hf.write('    float minPidError;\n')
        hf.write('    float maxPidError;\n')
        hf.write('} getMaxPidErrorResponseConverted;\n\n')

        # Now the class definition
        hf.write('class Servomotor {\npublic:\n')
        hf.write('    Servomotor(uint8_t alias = \'X\', HardwareSerial& serialPort = Serial1);\n')
        hf.write('    void setAlias(uint8_t new_alias);\n')
        hf.write('    uint8_t getAlias();\n')
        hf.write('    void openSerialPort();\n')
        hf.write('    int getError() const;\n\n')

        # Unit setting methods
        conv_map, default_units = get_unit_info(unit_conversions_data)
        hf.write('    // Unit settings\n')
        for ut, (conv_func_in, conv_func_out, member_name, enum_name) in conv_map.items():
            hf.write(f'    void set{ut.capitalize()}Unit({enum_name} unit);\n')
        hf.write('\n')

        # For each command, declare a raw + wrapper if needed
        for cmd in commands_data:
            func_name = format_command_name(cmd['CommandString'], function_name_mappings)
            input_params = extract_parameters(cmd.get('Input'), cmd['CommandString'], data_type_map, set())
            norm_cmd = cmd['CommandString'].strip().lower()

            # Decide raw return type
            if cmd.get('Output') and cmd['Output'] != 'success_response':
                out_params = extract_parameters(cmd['Output'], cmd['CommandString'], data_type_map, set())
                if norm_cmd in norm_response_map:
                    raw_ret = norm_response_map[norm_cmd]
                    raw_ret = raw_ret[0] if isinstance(raw_ret, tuple) else raw_ret
                else:
                    if len(out_params)==1 and out_params[0].get('array_size'):
                        raw_ret = out_params[0]['type']+f'[{out_params[0]["array_size"]}]'
                    elif len(out_params)==1:
                        raw_ret = out_params[0]['type']
                    else:
                        raw_ret = f'{func_name}Response'
            else:
                raw_ret = "void"

            # The function param string for raw
            raw_params_str = ", ".join(format_function_param_type(p) for p in input_params)

            if command_needs_conversion(cmd):
                # We'll produce a raw version + wrapper
                hf.write(f'    {raw_ret} {func_name}Raw({raw_params_str});\n')
                # Wrapper return
                w_out_params = extract_parameters(cmd.get('Output'), cmd['CommandString'], data_type_map, set())
                w_ret = get_wrapper_return(w_out_params, cmd['CommandString'], func_name)
                # Wrapper param
                w_params_str = ", ".join(get_wrapper_param(p) for p in input_params)
                hf.write(f'    {w_ret} {func_name}({w_params_str});\n')
            else:
                # Single function
                hf.write(f'    {raw_ret} {func_name}({raw_params_str});\n')

            if cmd.get('MultipleResponses'):
                hf.write(f'    {raw_ret} {func_name}GetAnotherResponse();\n')

        hf.write('\nprivate:\n')
        hf.write('    uint8_t _alias;\n')
        hf.write('    Communication _comm;\n')
        hf.write('    int _errno;\n\n')
        hf.write('    // Unit settings\n')
        for ut, (cf_in, cf_out, mem_name, en_name) in conv_map.items():
            first_unit = unit_conversions_data["units"][ut][0].upper()
            hf.write(f'    {en_name} {mem_name} = {en_name}::{first_unit};\n')
        hf.write('\n    bool _initialized;\n')
        hf.write('    void ensureInitialized();\n')
        hf.write('};\n\n#endif // SERVOMOTOR_H\n')

    print(f'Generated {header_file} successfully.')

    # Build the .cpp file
    with open(source_file,'w') as sf:
        sf.write(f'// {SERVOMOTOR_CPP_FILE}\n')
        sf.write(f'// Autogenerated by generate_command_code.py on {now}\n')
        sf.write('// Do not edit manually.\n\n')
        sf.write(f'#include "{SERVOMOTOR_H_FILE}"\n')
        sf.write('#include "Commands.h"\n#include "Utils.h"\n\n')

        sf.write('Servomotor::Servomotor(uint8_t alias, HardwareSerial& serialPort)\n')
        sf.write('    : _alias(alias), _comm(serialPort), _errno(0),\n')

        conv_map, def_units = get_unit_info(unit_conversions_data)
        first_flag = True
        for ut, (cf_in, cf_out, mem_name, en_name) in conv_map.items():
            first_unit = unit_conversions_data["units"][ut][0].upper()
            if first_flag:
                sf.write(f'      {mem_name}({en_name}::{first_unit})')
                first_flag=False
            else:
                sf.write(f',\n      {mem_name}({en_name}::{first_unit})')
        sf.write(' {\n    openSerialPort();\n}\n\n')

        sf.write('void Servomotor::setAlias(uint8_t new_alias) {\n')
        sf.write('    _alias = new_alias;\n')
        sf.write('}\n\n')

        sf.write('uint8_t Servomotor::getAlias() {\n')
        sf.write('    return _alias;\n')
        sf.write('}\n\n')

        sf.write('void Servomotor::openSerialPort() {\n')
        sf.write('    _comm.openSerialPort();\n')
        sf.write('}\n\n')

        sf.write('int Servomotor::getError() const {\n')
        sf.write('    return _errno;\n}\n\n')

        # Unit setting function implementations
        sf.write('// Unit setting functions\n')
        for ut, (cf_in, cf_out, mem_name, en_name) in conv_map.items():
            capital = ut.capitalize()
            sf.write(f'void Servomotor::set{capital}Unit({en_name} unit) {{\n')
            sf.write(f'    {mem_name} = unit;\n')
            sf.write(f'    Serial.print("[Motor] set{capital}Unit to ");\n')
            sf.write('    switch(unit) {\n')
            for possible in unit_conversions_data["units"][ut]:
                sf.write(f'        case {en_name}::{possible.upper()}: Serial.println("{possible.upper()}"); break;\n')
            sf.write('    }\n}\n\n')

        # Generate out-of-line definitions for each command
        for cmd in commands_data:
            func_name = format_command_name(cmd['CommandString'], function_name_mappings)
            input_params = extract_parameters(cmd.get('Input'), cmd['CommandString'], data_type_map, set())
            norm_cmd = cmd['CommandString'].strip().lower()

            if cmd.get('Output') and cmd['Output'] != 'success_response':
                out_params = extract_parameters(cmd['Output'], cmd['CommandString'], data_type_map, set())
                if norm_cmd in norm_response_map:
                    raw_ret = norm_response_map[norm_cmd]
                    raw_ret = raw_ret[0] if isinstance(raw_ret, tuple) else raw_ret
                else:
                    if len(out_params)==1 and out_params[0].get('array_size'):
                        raw_ret = out_params[0]['type']+f'[{out_params[0]["array_size"]}]'
                    elif len(out_params)==1:
                        raw_ret = out_params[0]['type']
                    else:
                        raw_ret = f'{func_name}Response'
            else:
                raw_ret = "void"

            raw_params_str = ", ".join(format_function_param_type(p) for p in input_params)

            # If we need a separate Raw version
            if command_needs_conversion(cmd):
                sf.write(f'{raw_ret} Servomotor::{func_name}Raw({raw_params_str}) {{\n')
                sf.write(f'    // {cmd["Description"]} (Raw version)\n')
            else:
                sf.write(f'{raw_ret} Servomotor::{func_name}({raw_params_str}) {{\n')
                sf.write(f'    Serial.println("[Motor] {func_name} called.");\n')
                sf.write(f'    // {cmd["Description"]}\n')

            sf.write(f'    const uint8_t commandID = {cmd["CommandEnum"]};\n')

            if input_params:
                struct_name = f'{func_name}Payload'
                sf.write(f'    {struct_name} payload;\n')
                for p in input_params:
                    if p.get('array_size'):
                        sf.write(f'    memcpy(payload.{p["name"]}, {p["name"]}, sizeof(payload.{p["name"]}));\n')
                    else:
                        size_bytes = get_size_of_type(p["type"], data_type_map, p.get("array_size"))
                        if size_bytes>1 and ('int' in p["type"] or 'uint' in p["type"]):
                            sf.write(f'    payload.{p["name"]} = htole{size_bytes*8}({p["name"]});\n')
                        else:
                            sf.write(f'    payload.{p["name"]} = {p["name"]};\n')
                sf.write(f'    _comm.sendCommand(_alias, commandID, (uint8_t*)&payload, sizeof(payload));\n')
            else:
                sf.write(f'    _comm.sendCommand(_alias, commandID, nullptr, 0);\n')

            if cmd.get('Output') and cmd['Output'] != 'success_response':
                sf.write(f'    uint8_t buffer[sizeof({raw_ret})];\n')
                sf.write('    uint16_t receivedSize;\n')
                sf.write('    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);\n')
                sf.write('    if (_errno == 0) {\n')
                sf.write(f'        if (receivedSize == sizeof({raw_ret})) {{\n')
                sf.write(f'            {raw_ret}* response = ({raw_ret}*)buffer;\n')
                sf.write('            return *response;\n')
                sf.write('        } else {\n')
                sf.write('            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;\n')
                sf.write('        }\n')
                sf.write('    }\n')
                sf.write(f'    {raw_ret} defaultResponse = {{0}};\n')
                sf.write('    return defaultResponse;\n')
            else:
                sf.write('    uint8_t buffer[1];\n')
                sf.write('    uint16_t receivedSize;\n')
                sf.write('    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);\n')

            sf.write('}\n\n')

            # If we needed a conversion-based wrapper
            if command_needs_conversion(cmd):
                wrapper_out = get_wrapper_return(extract_parameters(cmd.get('Output'), cmd['CommandString'], data_type_map, set()),
                                                 cmd['CommandString'], func_name)
                wrapper_params = [get_wrapper_param(p) for p in input_params]
                sf.write(f'{wrapper_out} Servomotor::{func_name}({", ".join(wrapper_params)}) {{\n')
                sf.write(f'    Serial.println("[Motor] {func_name} called.");\n')
                for p in input_params:
                    if p.get('UnitConversion'):
                        sf.write(f'    Serial.print("  {p["name"]} in chosen unit: "); Serial.println({p["name"]});\n')
                sf.write('\n')
                raw_call_args = []
                for p in input_params:
                    if p.get('UnitConversion'):
                        conv_type = p['UnitConversion']['Type']
                        conv_func = conv_map[conv_type][0]
                        mem_var = conv_map[conv_type][2]
                        sf.write(f'    float {p["name"]}_internal = {conv_func}({p["name"]}, {mem_var}, ConversionDirection::TO_INTERNAL);\n')
                        raw_call_args.append(f'({p["type"]})({p["name"]}_internal)')
                    else:
                        raw_call_args.append(p["name"])

                if wrapper_out == "void":
                    sf.write(f'    {func_name}Raw({", ".join(raw_call_args)});\n')
                else:
                    sf.write(f'    auto rawResult = {func_name}Raw({", ".join(raw_call_args)});\n')
                    if norm_cmd == "get comprehensive position":
                        sf.write('    getComprehensivePositionResponseConverted converted;\n')
                        sf.write('    converted.commandedPosition = convertPosition((float)rawResult.commandedPosition, m_positionUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    converted.hallSensorPosition = convertPosition((float)rawResult.hallSensorPosition, m_positionUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    converted.externalEncoderPosition = convertPosition((float)rawResult.externalEncoderPosition, m_positionUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    return converted;\n')
                    elif norm_cmd == "get max pid error":
                        sf.write('    getMaxPidErrorResponseConverted converted;\n')
                        sf.write('    converted.minPidError = convertPosition((float)rawResult.minPidError, m_positionUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    converted.maxPidError = convertPosition((float)rawResult.maxPidError, m_positionUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    return converted;\n')
                    elif norm_cmd == "get current time":
                        sf.write('    float converted = convertTime((float)rawResult.currentTime, m_timeUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    return converted;\n')
                    elif norm_cmd in {"get hall sensor position","get position"}:
                        sf.write('    float converted = convertPosition((float)rawResult.hallSensorPosition, m_positionUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    return converted;\n')
                    elif norm_cmd == "get supply voltage":
                        sf.write('    float converted = convertVoltage((float)rawResult.supplyVoltage, m_voltageUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    return converted;\n')
                    elif norm_cmd == "get temperature":
                        sf.write('    float converted = convertTemperature((float)rawResult.temperature, m_temperatureUnit, ConversionDirection::FROM_INTERNAL);\n')
                        sf.write('    return converted;\n')
                    else:
                        sf.write('    // Not a known multi-float conversion.\n')
                        sf.write('    return rawResult;\n')

                sf.write('}\n\n')

            if cmd.get('MultipleResponses'):
                if raw_ret == 'void':
                    sf.write(f'void Servomotor::{func_name}GetAnotherResponse() {{\n')
                    sf.write(f'    // No actual output to read.\n')
                    sf.write('}\n\n')
                else:
                    sf.write(f'{raw_ret} Servomotor::{func_name}GetAnotherResponse() {{\n')
                    sf.write(f'    // Attempt to receive another response from previously sent {cmd["CommandString"]} command\n')
                    sf.write(f'    uint8_t buffer[sizeof({raw_ret})];\n')
                    sf.write('    uint16_t receivedSize;\n')
                    sf.write('    _errno = _comm.getResponse(buffer, sizeof(buffer), receivedSize);\n')
                    sf.write('    if (_errno == 0) {\n')
                    sf.write(f'        if (receivedSize == sizeof({raw_ret})) {{\n')
                    sf.write(f'            {raw_ret}* response = ({raw_ret}*)buffer;\n')
                    sf.write('            return *response;\n')
                    sf.write('        } else {\n')
                    sf.write('            _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE;\n')
                    sf.write('        }\n')
                    sf.write('    }\n')
                    sf.write(f'    return {raw_ret}();\n')
                    sf.write('}\n\n')

    print(f'Generated {header_file} and {source_file} successfully.')

if __name__ == '__main__':
    print(f"This script will generate the following files:")
    print(f"  {COMMANDS_H_FILE}")
    print(f"  {SERVOMOTOR_CPP_FILE}")
    print(f"  {SERVOMOTOR_H_FILE}")
    # The existing files will be backed up unless --no-backup is given
    if '--no-backup' not in sys.argv:
        print("The existing files will be backed up and then overwritten.")
        print("You can give the --no-backup command line option to not make any backups.")
        backup_flag = True
    else:
        print("The --no-backup command line option was given. No backups will be made.")
        backup_flag = False

    # We also handle the -y prompt logic
    if '-y' in sys.argv:
        print("The -y command line option was given. Skipping the prompt and proceeding.")
    else:
        print("You can give the -y command line option to skip the following prompt.")
        print("Do you want to continue? (y/n)")
        ans = input()
        if ans.lower() != 'y':
            print("Exiting.")
            sys.exit(0)

    if backup_flag:
        back_up_files([COMMANDS_H_FILE, SERVOMOTOR_CPP_FILE, SERVOMOTOR_H_FILE])

    generate_commands_header(MOTOR_COMMANDS_JSON_FILE, COMMANDS_H_FILE)
    generate_servo_motor_files(MOTOR_COMMANDS_JSON_FILE, DATA_TYPES_JSON_FILE,
                               SERVOMOTOR_H_FILE, SERVOMOTOR_CPP_FILE)
