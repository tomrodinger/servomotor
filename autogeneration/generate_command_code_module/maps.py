"""Centralized maps used by the command code generators.

This module is intended to be the *single source of truth* for:

- JSON type-string -> C++ type mapping used across generators
- Unit conversion function lookup
- Endianness conversion function lookup

The goal is to keep the generator modules smaller and consistent.
"""

from __future__ import annotations

import re
from typing import Dict, Tuple, Union

# For array types, values use a tuple of (base_type, array_size)
CppType = Union[str, Tuple[str, int]]


TYPE_MAP: Dict[str, CppType] = {
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

    # Special types
    'u24_version_number': 'VersionNumber24',
    'u32_version_number': 'VersionNumber32',
    'u64_unique_id': 'uint64_t',
    'crc32': 'uint32_t',
}


UNIT_CONVERSION_MAP: Dict[str, str] = {
    'position': 'convertPosition',
    'time': 'convertTime',
    'velocity': 'convertVelocity',
    'acceleration': 'convertAcceleration',
    'temperature': 'convertTemperature',
    'voltage': 'convertVoltage',
    'current': 'convertCurrent',
}


ENDIAN_CONVERSION_MAP: Dict[str, str] = {
    'int8_t': '',
    'uint8_t': '',
    'int16_t': 'htole16',
    'uint16_t': 'htole16',
    'int32_t': 'htole32',
    'uint32_t': 'htole32',
    'int64_t': 'htole64',
    'uint64_t': 'htole64',
}


def get_cpp_type(type_str: str, default: str = 'uint8_t') -> CppType:
    """Return the C++ type entry for a JSON type string.

    The returned value is either:
    - a string type name (e.g. "uint32_t")
    - a (base_type, array_size) tuple (e.g. ("uint8_t", 10))
    """
    return TYPE_MAP.get(type_str, default)


def get_variable_length_output(cmd, data_types_data):
    """Detect a variable-length (size:null) single output for a command.

    Looks up the command's sole Output parameter's data type in data_types.json.
    A data type whose "size" is None is variable length (string_null_term id 201,
    general_data id 204). Fixed types (string8=8, buf10=10, firmware_page=2058,
    all numeric types) have a concrete size and are NOT affected.

    Returns a (is_variable_length, is_string) tuple:
    - is_variable_length: True only when the single output's data type has size == None.
    - is_string: True for string_null_term (emit char* buffer, null-terminate);
                 False for general_data / any other size:null blob (emit uint8_t* buffer).
    Returns (False, False) for everything else (no output, success_response,
    multiple outputs, unknown type, or a fixed-size type).
    """
    output_params = cmd.get('Output')
    if not output_params or output_params == 'success_response':
        return (False, False)
    if not isinstance(output_params, list) or len(output_params) != 1:
        return (False, False)
    desc = output_params[0].get('Description', '')
    m = re.match(r'(\w+):\s*(.*)', desc)
    if not m:
        return (False, False)
    type_str = m.group(1)
    data_type_map = {dt.get('data_type', ''): dt for dt in (data_types_data or [])}
    dt = data_type_map.get(type_str)
    if dt is None or dt.get('size') is not None:
        return (False, False)
    return (True, type_str == 'string_null_term')

