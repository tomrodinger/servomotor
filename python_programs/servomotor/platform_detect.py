"""
Platform Detection Module

Detects the current runtime environment (MicroPython or standard Python)
to enable cross-platform compatibility.
"""

import sys


def get_platform():
    """
    Detects the current platform.
    
    Returns:
        str: 'micropython' if running on MicroPython, 'standard' otherwise
    """
    try:
        # MicroPython has a different implementation structure
        if sys.implementation.name == 'micropython':
            return 'micropython'
    except AttributeError:
        # Older Python versions might not have sys.implementation
        pass
    
    return 'standard'


def is_micropython():
    """
    Check if currently running on MicroPython.
    
    Returns:
        bool: True if running on MicroPython, False otherwise
    """
    return get_platform() == 'micropython'


def is_standard_python():
    """
    Check if currently running on standard Python.
    
    Returns:
        bool: True if running on standard Python, False otherwise
    """
    return get_platform() == 'standard'


def is_esp32():
    """
    Check if currently running on an ESP32 board.
    
    Returns:
        bool: True if running on ESP32, False otherwise
    """
    if is_micropython():
        try:
            import machine
            # Check for ESP32-specific attributes
            return hasattr(machine, 'Pin') and hasattr(machine, 'UART')
        except ImportError:
            return False
    return False


def get_platform_info():
    """
    Get detailed platform information.
    
    Returns:
        dict: Dictionary containing platform details
    """
    info = {
        'platform': get_platform(),
        'is_micropython': is_micropython(),
        'is_esp32': is_esp32(),
        'python_version': sys.version,
    }
    
    if is_micropython():
        try:
            import machine
            info['machine'] = str(machine)
        except:
            pass
    
    return info