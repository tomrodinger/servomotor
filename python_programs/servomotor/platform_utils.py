"""
Platform Utilities Module

Provides cross-platform implementations for common operations that differ
between standard Python and MicroPython (CRC32, file I/O, terminal operations).
"""

from .platform_detect import is_micropython, is_standard_python


def int_to_bytes(value, length, signed=False):
    """
    Pack an integer into little-endian bytes.

    Works on both CPython and MicroPython because we never pass ``signed``
    to ``int.to_bytes`` (CPython requires it as a keyword-only argument and
    MicroPython does not accept it at all). Negative values are masked into
    the unsigned range manually.
    """
    if signed and value < 0:
        value += 1 << (8 * length)
    return value.to_bytes(length, "little")


def int_from_bytes(data, signed=False):
    """
    Unpack a little-endian byte string into an integer.

    MicroPython's ``int.from_bytes`` lacks ``signed``, so the two's-complement
    fold is done manually.
    """
    result = int.from_bytes(data, "little")
    if signed and result >= (1 << (8 * len(data) - 1)):
        result -= 1 << (8 * len(data))
    return result


def wrap_text(text, initial_indent='', subsequent_indent='', width=80):
    """
    Wrap text to a given width. Uses textwrap on standard Python and a
    naive line-splitter fallback on MicroPython.
    """
    if is_standard_python():
        import textwrap
        return textwrap.wrap(
            text,
            initial_indent=initial_indent,
            subsequent_indent=subsequent_indent,
            width=width,
        )
    lines = text.split('\n')
    return [
        (initial_indent if i == 0 else subsequent_indent) + line
        for i, line in enumerate(lines)
    ]


def module_path(filename):
    """
    Return an absolute path to ``filename`` inside the servomotor package.

    On standard Python, resolves relative to this file's directory. On
    MicroPython, returns ``'servomotor/<filename>'`` which matches how the
    package is laid out on the device's filesystem.
    """
    if is_standard_python():
        import os
        return os.path.join(os.path.dirname(__file__), filename)
    return 'servomotor/' + filename


def calculate_crc32(data):
    """
    Calculate CRC32 checksum for a byte array.
    Uses zlib on standard Python, pure Python implementation on MicroPython.
    
    Args:
        data: Bytes to calculate CRC32 for
        
    Returns:
        int: CRC32 checksum (32-bit unsigned integer)
    """
    try:
        # Try using zlib (available on standard Python and some MicroPython builds)
        import zlib
        return zlib.crc32(data) & 0xffffffff
    except ImportError:
        # Fall back to pure Python implementation for MicroPython
        return _micropython_crc32(data)


def _micropython_crc32(data):
    """
    Pure Python CRC32 implementation for MicroPython.
    Uses the standard CRC32 polynomial (0xEDB88320).
    
    Args:
        data: Bytes to calculate CRC32 for
        
    Returns:
        int: CRC32 checksum
    """
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
    """
    Get terminal width.
    Returns actual terminal width on standard Python, default 80 on MicroPython.
    
    Returns:
        int: Terminal width in columns
    """
    if is_standard_python():
        try:
            import shutil
            return shutil.get_terminal_size().columns
        except:
            return 80
    else:
        # MicroPython doesn't have shutil
        return 80


def load_json_file(filepath):
    """
    Load JSON file with platform-appropriate JSON module.
    
    Args:
        filepath: Path to JSON file
        
    Returns:
        Parsed JSON data (dict, list, etc.)
    """
    if is_micropython():
        try:
            import ujson as json
        except ImportError:
            import json
    else:
        import json
    
    with open(filepath, 'r') as f:
        return json.load(f)


def save_json_file(filepath, data):
    """
    Save data to JSON file with platform-appropriate JSON module.
    
    Args:
        filepath: Path to save JSON file
        data: Data to serialize to JSON
    """
    if is_micropython():
        try:
            import ujson as json
        except ImportError:
            import json
    else:
        import json
    
    with open(filepath, 'w') as f:
        json.dump(data, f)


def file_exists(filepath):
    """
    Check if a file exists (cross-platform).
    
    Args:
        filepath: Path to check
        
    Returns:
        bool: True if file exists, False otherwise
    """
    try:
        with open(filepath, 'r'):
            return True
    except (OSError, IOError):
        return False


def read_text_file(filepath):
    """
    Read text file contents (cross-platform).
    
    Args:
        filepath: Path to file
        
    Returns:
        str: File contents
    """
    with open(filepath, 'r') as f:
        return f.read()


def write_text_file(filepath, content):
    """
    Write text to file (cross-platform).
    
    Args:
        filepath: Path to file
        content: String to write
    """
    with open(filepath, 'w') as f:
        f.write(content)


def makedirs(path):
    """
    Create directory (cross-platform).
    
    Args:
        path: Directory path to create
    """
    if is_standard_python():
        import os
        os.makedirs(path, exist_ok=True)
    else:
        # MicroPython uses uos
        try:
            import uos as os
            try:
                os.mkdir(path)
            except OSError:
                # Directory might already exist
                pass
        except ImportError:
            import os
            try:
                os.mkdir(path)
            except OSError:
                pass


def path_exists(path):
    """
    Check if a path exists (cross-platform).
    
    Args:
        path: Path to check
        
    Returns:
        bool: True if path exists, False otherwise
    """
    if is_standard_python():
        import os
        return os.path.exists(path)
    else:
        try:
            import uos as os
            try:
                os.stat(path)
                return True
            except OSError:
                return False
        except ImportError:
            import os
            try:
                os.stat(path)
                return True
            except OSError:
                return False


def get_random_bytes(n):
    """
    Generate n random bytes (cross-platform).
    
    Args:
        n: Number of random bytes to generate
        
    Returns:
        bytearray: Random bytes
    """
    if is_standard_python():
        import random
        return bytearray(random.getrandbits(8) for _ in range(n))
    else:
        try:
            import urandom as random
        except ImportError:
            import random
        
        return bytearray(random.getrandbits(8) for _ in range(n))


def sleep(seconds):
    """
    Sleep for specified seconds (cross-platform wrapper).
    
    Args:
        seconds: Time to sleep in seconds
    """
    import time
    time.sleep(seconds)


def ticks_ms():
    """
    Get millisecond counter (cross-platform).
    
    Returns:
        int: Milliseconds since start (wraps around)
    """
    if is_micropython():
        try:
            import utime
            return utime.ticks_ms()
        except ImportError:
            import time
            return int(time.time() * 1000)
    else:
        import time
        return int(time.time() * 1000)


def ticks_diff(end, start):
    """
    Calculate difference between two tick values (handles wraparound).
    
    Args:
        end: End tick value
        start: Start tick value
        
    Returns:
        int: Difference in ticks
    """
    if is_micropython():
        try:
            import utime
            return utime.ticks_diff(end, start)
        except ImportError:
            return end - start
    else:
        return end - start