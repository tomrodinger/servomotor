"""
Servomotor Control Module

This package provides functionality for controlling servomotors
from the company Gearotons.
"""

from .serial_functions import (
    select_serial_port_from_menu,
    open_serial_port_or_print_detailed_error
)

from .communication import (
    set_data_types_and_command_data,
    print_protocol_version,
    print_data_type_descriptions,
    print_registered_commands,
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
    'print_protocol_version',
    'print_data_type_descriptions',
    'print_registered_commands',
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

from . import command_loader
# we will load the motor_commands.json file, which is in the same directory as this __init__.py file, so make sure to include the right path
data_type_dict, command_dict = command_loader.load_data_types_and_commands('servomotor/data_types.json', 'servomotor/motor_commands.json')
set_data_types_and_command_data(data_type_dict, command_dict)