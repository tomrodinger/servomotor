"""
Servomotor Control Module

This package provides functionality for controlling servomotors
from the company Gearotons.

Cross-platform compatible: Works on standard Python (Mac/PC/Raspberry Pi)
and MicroPython (ESP32-S3).
"""
__version__ = "0.10.0"  # Updated version for cross-platform support

# Import platform detection first
from .platform_detect import is_micropython, is_standard_python, get_platform, get_platform_info

# Import cross-platform utilities
from .platform_utils import calculate_crc32, module_path
from .serial_abstraction import create_serial_port

from .communication import (
    print_protocol_version,
    print_data_type_descriptions,
    print_registered_commands,
    get_human_readable_alias_or_unique_id,
    string_to_alias_or_unique_id,
    get_global_alias_or_unique_id,
    set_standard_options_from_args,
    set_serial_port_from_args,
    get_command_id,
    gather_inputs,
    send_command,
    interpret_response,
    flush_receive_buffer,
    RESPONSE_CHARACTER_CRC32_ENABLED,
    RESPONSE_CHARACTER_CRC32_DISABLED,
    ALL_ALIAS,
    open_serial_port,
    close_serial_port,
    read_raw_byte,
    execute_command,
    TimeoutError,
    CommunicationError,
    PayloadError,
    NoAliasOrUniqueIdSet,
    FatalError
)

# Resolve JSON paths through the platform layer (string on MicroPython,
# absolute path on CPython).
data_types_path = module_path('data_types.json')
motor_commands_path = module_path('motor_commands.json')

# Load command definitions
from . import command_loader
data_type_dict, command_dict = command_loader.load_data_types_and_commands(data_types_path, motor_commands_path, verbose=0)

# Import M3 class and define commands
from .M3 import M3, define_commands
define_commands(M3, data_type_dict, command_dict, verbose=0)

# Import device detection functions
from .device_detection import detect_devices_iteratively, Device
