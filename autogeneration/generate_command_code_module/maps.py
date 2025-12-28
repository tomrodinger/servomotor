"""Centralized maps used by the command code generators.

This module is intended to be the *single source of truth* for:

- JSON type-string -> C++ type mapping used across generators
- Unit conversion function lookup
- Endianness conversion function lookup

The goal is to keep the generator modules smaller and consistent.
"""

from __future__ import annotations

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

