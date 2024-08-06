# redirect_motor_commands.py

import sys
import os
from importlib import import_module

# Print debug information
print("Redirecting motor_commands module")

# Add the path to the real motor_commands.py to the system path
real_motor_commands_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, real_motor_commands_path)

# Import the real motor_commands module dynamically
real_motor_commands = import_module('motor_commands')

# Print debug information
print("Imported motor_commands from:", real_motor_commands.__file__)

# Dynamically set the attributes in the current module's namespace
for attr in dir(real_motor_commands):
    if not attr.startswith("__"):
        setattr(sys.modules[__name__], attr, getattr(real_motor_commands, attr))
