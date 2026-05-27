"""
Terminal Formatting Wrapper

Provides terminal formatting with fallback for MicroPython.
On standard Python, uses the existing terminal_formatting module.
On MicroPython, provides simple text-based formatting.
"""

from .platform_detect import is_micropython, is_standard_python


if is_micropython():
    # Simple implementations for MicroPython (no color codes)
    def format_error(msg):
        """Format error message (simple text on MicroPython)."""
        return f"ERROR: {msg}"
    
    def format_info(msg):
        """Format info message (simple text on MicroPython)."""
        return f"INFO: {msg}"
    
    def format_warning(msg):
        """Format warning message (simple text on MicroPython)."""
        return f"WARNING: {msg}"
    
    def format_success(msg):
        """Format success message (simple text on MicroPython)."""
        return f"SUCCESS: {msg}"
    
    def format_debug(msg):
        """Format debug message (simple text on MicroPython)."""
        return f"DEBUG: {msg}"
    
    # Empty style dictionary for MicroPython
    STYLE = {}

else:
    # Import from existing terminal_formatting module for standard Python
    try:
        import sys
        import os
        # Add parent directory to path to find terminal_formatting
        parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if parent_dir not in sys.path:
            sys.path.append(parent_dir)
        
        from terminal_formatting import (
            format_error,
            format_info,
            format_warning,
            format_success,
            format_debug,
            STYLE
        )
    except (ImportError, OSError):
        # Fallback if terminal_formatting module is not available
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


def safe_print(message, use_formatting=True):
    """
    Print message with optional formatting.
    
    Args:
        message: Message to print
        use_formatting: Whether to use formatting (ignored on MicroPython)
    """
    print(message)