# Cross-Platform Servomotor Library Architecture

## Overview
This document outlines the architecture for making the servomotor Python library compatible with both standard Python (Mac/PC/Raspberry Pi) and MicroPython (ESP32-S3) using a single unified codebase.

## Requirements
- **Full feature parity** across all platforms
- **Single codebase** with conditional imports and platform detection
- **ESP32-S3 Configuration**: TX=Pin(4), RX=Pin(5), baudrate=230400
- **Maintain existing API** for backward compatibility

## Architecture Components

### 1. Platform Detection Module (`servomotor/platform_detect.py`)
```python
import sys

def get_platform():
    """
    Detects the current platform
    Returns: 'micropython' | 'standard'
    """
    return 'micropython' if sys.implementation.name == 'micropython' else 'standard'

def is_micropython():
    return get_platform() == 'micropython'

def is_esp32():
    if is_micropython():
        try:
            import machine
            # Check for ESP32-specific attributes
            return hasattr(machine, 'Pin')
        except:
            return False
    return False
```

### 2. Serial Port Abstraction Layer (`servomotor/serial_abstraction.py`)

```python
class SerialPort:
    """Base class for serial port operations"""
    
    def __init__(self, port=None, baudrate=230400, timeout=0.1):
        raise NotImplementedError
    
    def write(self, data):
        raise NotImplementedError
    
    def read(self, size):
        raise NotImplementedError
    
    def read_until(self, terminator, size=None):
        raise NotImplementedError
    
    def reset_input_buffer(self):
        raise NotImplementedError
    
    def close(self):
        raise NotImplementedError
    
    @property
    def name(self):
        raise NotImplementedError

class StandardSerial(SerialPort):
    """Implementation for standard Python with pyserial"""
    
    def __init__(self, port=None, baudrate=230400, timeout=0.1):
        from .vendor import serial
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
    
    def write(self, data):
        return self.ser.write(data)
    
    def read(self, size):
        return self.ser.read(size)
    
    # ... other methods delegate to self.ser

class MicroPythonSerial(SerialPort):
    """Implementation for MicroPython on ESP32-S3"""
    
    def __init__(self, port=None, baudrate=230400, timeout=0.1):
        from machine import UART, Pin
        # Default pins for ESP32-S3
        self.uart = UART(
            1,  # UART(1) is the secondary port
            baudrate=baudrate,
            bits=8,
            parity=None,
            stop=1,
            tx=Pin(4),
            rx=Pin(5),
            timeout=int(timeout * 1000)  # Convert to milliseconds
        )
        self.timeout = timeout
    
    def write(self, data):
        return self.uart.write(data)
    
    def read(self, size):
        data = self.uart.read(size)
        return data if data else b''
    
    # ... implement other methods

def create_serial_port(port=None, baudrate=230400, timeout=0.1):
    """Factory function to create appropriate serial port"""
    from .platform_detect import is_micropython
    
    if is_micropython():
        return MicroPythonSerial(port, baudrate, timeout)
    else:
        return StandardSerial(port, baudrate, timeout)
```

### 3. Cross-Platform Utilities (`servomotor/platform_utils.py`)

```python
def calculate_crc32(data):
    """Calculate CRC32 with fallback for MicroPython"""
    try:
        import zlib
        return zlib.crc32(data) & 0xffffffff
    except ImportError:
        # MicroPython fallback - implement CRC32 algorithm
        return micropython_crc32(data)

def micropython_crc32(data):
    """Pure Python CRC32 implementation for MicroPython"""
    # CRC32 polynomial and implementation
    crc = 0xffffffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xedb88320
            else:
                crc >>= 1
    return crc ^ 0xffffffff

def get_terminal_columns():
    """Get terminal width with MicroPython fallback"""
    try:
        import shutil
        return shutil.get_terminal_size().columns
    except (ImportError, AttributeError):
        return 80  # Default width for MicroPython

def safe_print(message, style=None):
    """Print with optional styling (disabled on MicroPython)"""
    from .platform_detect import is_micropython
    
    if is_micropython() or style is None:
        print(message)
    else:
        # Apply terminal formatting for standard Python
        from terminal_formatting import STYLE
        print(f"{style}{message}{STYLE['RESET']}")

def load_json_file(filepath):
    """Load JSON with platform-appropriate method"""
    from .platform_detect import is_micropython
    
    if is_micropython():
        # MicroPython: Use ujson
        import ujson as json
    else:
        import json
    
    with open(filepath, 'r') as f:
        return json.load(f)

def save_serial_device(device_name):
    """Save serial device name with platform handling"""
    from .platform_detect import is_micropython
    
    if is_micropython():
        # On MicroPython, don't save to file or use different location
        return
    else:
        # Standard implementation
        import os
        current_dir = os.path.dirname(os.path.realpath(__file__))
        device_file_path = os.path.join(current_dir, "serial_device.txt")
        with open(device_file_path, "w") as f:
            f.write(device_name)
```

### 4. Modified Communication Module Structure

Key changes to `servomotor/communication.py`:
- Replace `ser` global with abstracted serial port
- Use platform utilities for CRC32, printing, etc.
- Conditional imports based on platform

```python
# At the top of communication.py
from .platform_detect import is_micropython, get_platform
from .serial_abstraction import create_serial_port
from .platform_utils import calculate_crc32, safe_print, get_terminal_columns

# Replace direct serial usage
def open_serial_port(timeout=1.2):
    global ser
    if is_micropython():
        # MicroPython doesn't need port selection
        ser = create_serial_port(baudrate=230400, timeout=timeout)
    else:
        # Standard Python with port selection
        from . import serial_functions
        ser = serial_functions.open_serial_port(serial_port, 230400, timeout=timeout)
```

### 5. Modified Terminal Formatting

Create fallback formatting for MicroPython:

```python
# servomotor/terminal_format_wrapper.py
from .platform_detect import is_micropython

if is_micropython():
    # Simple implementations for MicroPython
    def format_error(msg):
        return f"ERROR: {msg}"
    
    def format_info(msg):
        return f"INFO: {msg}"
    
    def format_warning(msg):
        return f"WARNING: {msg}"
    
    def format_success(msg):
        return f"SUCCESS: {msg}"
    
    def format_debug(msg):
        return f"DEBUG: {msg}"
    
    STYLE = {}
else:
    # Import from existing terminal_formatting module
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from terminal_formatting import (
        format_error, format_info, format_warning, 
        format_success, format_debug, STYLE
    )
```

### 6. Ball Juggling Demo Adaptation

Key modifications for `ball_juggling_demo.py`:
- Conditional imports
- Platform-specific initialization
- Simplified file I/O for MicroPython

```python
#!/usr/bin/env python3

import sys
import time
import math

# Platform detection
try:
    import servomotor
    PLATFORM = 'standard'
except ImportError:
    # On MicroPython, the module might be in a different location
    sys.path.append('/flash')  # Or wherever the library is stored
    import servomotor
    PLATFORM = 'micropython'

# Platform-specific imports
if PLATFORM == 'micropython':
    import random
    # MicroPython doesn't have os module in the same way
    OUTPUT_LOG_FILE_DIRECTORY = None
else:
    import os
    import random
    OUTPUT_LOG_FILE_DIRECTORY = "./logs/"

# ... rest of the constants ...

# Main code with platform adaptations
if PLATFORM == 'standard' and OUTPUT_LOG_FILE_DIRECTORY:
    # File logging only on standard Python
    if not os.path.exists(OUTPUT_LOG_FILE_DIRECTORY):
        try:
            os.makedirs(OUTPUT_LOG_FILE_DIRECTORY)
        except OSError as e:
            print(f"Could not create log directory: {e}")
            OUTPUT_LOG_FILE_DIRECTORY = None

# Initialize servomotor
servomotor.open_serial_port()

# ... rest of the demo code ...
```

## Implementation Strategy

### Phase 1: Core Infrastructure
1. Create platform detection module
2. Implement serial port abstraction layer
3. Create platform utilities for CRC32, file I/O, terminal operations

### Phase 2: Library Refactoring
1. Update `communication.py` to use abstractions
2. Modify `serial_functions.py` for conditional execution
3. Update import statements throughout the library
4. Handle terminal formatting with fallbacks

### Phase 3: Testing and Validation
1. Deploy to ESP32-S3 and test basic communication
2. Test on Mac/PC with existing hardware to make sure nothing got broken
3. Run full ball_juggling_demo.py on both platforms
4. Debug and fix platform-specific issues

### Phase 4: Documentation
1. Installation instructions for MicroPython
2. Platform-specific configuration guide
3. API documentation with platform notes
4. Troubleshooting guide

## Testing Matrix

| Feature | Mac/PC | Raspberry Pi | ESP32-S3 |
|---------|--------|--------------|----------|
| Serial Communication | ✓ | ✓ | ✓ |
| CRC32 Validation | ✓ | ✓ | ✓ |
| Command Execution | ✓ | ✓ | ✓ |
| Error Handling | ✓ | ✓ | ✓ |
| File Logging | ✓ | ✓ | ✗ |
| Port Selection Menu | ✓ | ✓ | ✗ |
| Terminal Formatting | ✓ | ✓ | Simple |
| Ball Juggling Demo | ✓ | ✓ | ✓ |