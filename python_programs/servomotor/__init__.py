"""
Servomotor Control Module

This package provides functionality for controlling servomotors
from the company Gearotons.
"""

# Import main classes and functions that should be available when
# users import your package

from .serial_functions import (
    select_serial_port_from_menu,
    open_serial_port_or_print_detailed_error
)

from .communication import (
    set_command_data,
    set_standard_options_from_args,
    set_serial_port_from_args,
    open_serial_port,
    close_serial_port,
    get_command_id,
    gather_inputs,
    send_command,
    execute_command,
    interpret_response
)

# You can define __all__ to specify what gets imported with
# "from servomotor import *"
__all__ = [
    'set_standard_options_from_args',
    'set_serial_port_from_args',
    'open_serial_port',
    'close_serial_port',
    'get_command_id',
    'gather_inputs',
    'send_command',
    'get_response',
    'select_serial_port_from_menu',
    'open_serial_port_or_print_detailed_error',
    'set_command_data',
    'execute_command',
    'interpret_response'
]

# You can specify the version of your package here
#__version__ = "0.1.2"
# ... nope, instead, the version is in the pyproject.toml file

from . import motor_commands
set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                 motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                 motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

