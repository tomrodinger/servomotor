#!/usr/bin/env python3

import sys
import os

# Try to import curses for terminal capability detection
try:
    import curses
except ImportError:
    curses = None

class TerminalStyle:
    def __init__(self):
        self.has_color = False
        self.has_unicode = False
        self._detect_capabilities()
        self._setup_styles()

    def _detect_capabilities(self):
        # Check if output is a terminal
        if not sys.stdout.isatty():
            return

        # Check color support
        if curses:
            try:
                curses.setupterm()
                self.has_color = curses.tigetnum("colors") >= 8
            except:
                pass
        
        # Fallback color detection methods
        if not self.has_color:
            self.has_color = (
                'COLORTERM' in os.environ or
                'TERM' in os.environ and 'color' in os.environ['TERM']
            )

        # Check Unicode support
        self.has_unicode = sys.stdout.encoding and 'utf' in sys.stdout.encoding.lower()

    def _setup_styles(self):
        if self.has_color:
            # Colors
            self.RED = '\033[91m'
            self.GREEN = '\033[92m'
            self.YELLOW = '\033[93m'
            self.BLUE = '\033[94m'
            self.MAGENTA = '\033[95m'
            self.CYAN = '\033[96m'
            
            # Styles
            self.BOLD = '\033[1m'
            self.UNDERLINE = '\033[4m'
            self.BLINK = '\033[5m'
            
            # Backgrounds
            self.BG_RED = '\033[41m'
            self.BG_GREEN = '\033[42m'
            self.BG_YELLOW = '\033[43m'
            self.BG_BLUE = '\033[44m'
            
            # Reset
            self.RESET = '\033[0m'
        else:
            # No color support - empty strings for all styles
            self.RED = self.GREEN = self.YELLOW = self.BLUE = ''
            self.MAGENTA = self.CYAN = self.BOLD = self.UNDERLINE = ''
            self.BLINK = self.BG_RED = self.BG_GREEN = ''
            self.BG_YELLOW = self.BG_BLUE = self.RESET = ''

    def get_symbol(self, unicode_char, ascii_fallback):
        """Get appropriate symbol based on terminal capabilities"""
        return unicode_char if self.has_unicode else ascii_fallback

# Create a global style instance
STYLE = TerminalStyle()

def format_text(text, *styles):
    """Format text with multiple styles."""
    if STYLE.has_color:
        style_codes = ''.join(styles)
        return f"{style_codes}{text}{STYLE.RESET}"
    else:
        # Strip any remaining ANSI sequences for safety
        return text.replace('\033[', '').replace('\033', '')

def print_styled(text, *styles):
    """Print text with multiple styles applied."""
    print(format_text(text, *styles))

# Common formatting functions
def format_success(text):
    """Format success messages with green color and checkmark."""
    symbol = STYLE.get_symbol("✅", "+")
    return format_text(f"{symbol} {text}", STYLE.GREEN, STYLE.BOLD)

def format_error(text):
    """Format error messages with red color and X symbol."""
    symbol = STYLE.get_symbol("❌", "X")
    return format_text(f"{symbol} {text}", STYLE.RED, STYLE.BOLD)

def format_warning(text):
    """Format warning messages with yellow color and warning symbol."""
    symbol = STYLE.get_symbol("⚠️", "!")
    return format_text(f"{symbol} {text}", STYLE.YELLOW, STYLE.BOLD)

def format_info(text):
    """Format info messages with no color or formatting."""
    return text

def format_debug(text):
    """Format debug messages with no color or formatting."""
    return text
