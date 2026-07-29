#!/usr/bin/env python3

"""
Non-blocking single-keypress input, on POSIX and on Windows.

POSIX puts the terminal into raw mode and polls stdin with select(). Windows has
neither termios nor select-on-stdin, so it uses msvcrt.kbhit()/getwch() instead,
which needs no mode switching at all. The public API -- raw_mode() and getch() --
is identical on both, so callers never need to care which platform they are on.
"""

import sys
from contextlib import contextmanager

try:                      # POSIX
    import tty
    import termios
    import select
    _WINDOWS = False
except ImportError:       # Windows has no tty/termios
    import msvcrt
    _WINDOWS = True


@contextmanager
def raw_mode(file):
    """Context manager that puts the terminal in raw mode and restores it on exit.

    On Windows this is a no-op: msvcrt reads keys directly from the console
    without needing the terminal put into a special mode.
    """
    if _WINDOWS:
        yield
        return

    fd = file.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def getch():
    """Get a single character from stdin, or None if no key is waiting."""
    if _WINDOWS:
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            # Arrow/function keys arrive as a two-character sequence. Swallow the
            # second half so it is not mistaken for a separate keypress later.
            if ch in ('\x00', '\xe0'):
                msvcrt.getwch()
                return None
            return ch
        return None

    with raw_mode(sys.stdin):
        if select.select([sys.stdin], [], [], 0.01)[0]:  # Changed from 0.1s to 0.01s
            ch = sys.stdin.read(1)
            return ch
    return None
