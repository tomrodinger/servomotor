"""
Servomotor Control Module

This package provides functionality for controlling servomotors
from the company Gearotons.
"""

from .communication import (
    print_protocol_version,
    print_data_type_descriptions,
    print_registered_commands,
    get_human_readable_alias,
    set_standard_options_from_args,
    set_serial_port_from_args,
    get_command_id,
    gather_inputs,
    send_command,
    interpret_response,
    flush_receive_buffer,
    RESPONSE_CHARACTER,
    ENCODED_RESPONSE_CHARACTER,
    ALL_ALIAS,
    open_serial_port,
    close_serial_port,
    read_raw_byte,
    execute_command,
)

# Get the absolute path to the servomotor module directory
from pathlib import Path
module_dir = Path(__file__).parent
# Construct paths to the JSON files relative to the module directory
data_types_path = module_dir / 'data_types.json'
motor_commands_path = module_dir / 'motor_commands.json'

# Load command definitions
from . import command_loader
data_type_dict, command_dict = command_loader.load_data_types_and_commands(str(data_types_path), str(motor_commands_path), verbose=0)

# Import M3 class and define commands
from .M3 import M3, define_commands
define_commands(M3, data_type_dict, command_dict, verbose=0)
