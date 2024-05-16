# redirect_communication.py

import sys
import os
from importlib import import_module

# Print debug information
print("Redirecting communication module")

# Add the path to the real communication.py to the system path
real_communication_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../python_programs'))
sys.path.insert(0, real_communication_path)

# Import the real communication module dynamically
real_communication = import_module('communication')

# Print debug information
print("Imported communication from:", real_communication.__file__)

# Dynamically set the attributes in the current module's namespace
for attr in dir(real_communication):
    if not attr.startswith("__"):
        setattr(sys.modules[__name__], attr, getattr(real_communication, attr))
