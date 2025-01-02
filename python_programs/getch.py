#!/usr/bin/env python3

import sys
import tty
import termios
import select
from contextlib import contextmanager

@contextmanager
def raw_mode(file):
    """Context manager that puts the terminal in raw mode and restores it on exit"""
    fd = file.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def getch():
    """Get a single character from stdin"""
    print("Entering getch()")
    with raw_mode(sys.stdin):
        if select.select([sys.stdin], [], [], 0.01)[0]:  # Changed from 0.1s to 0.01s
            ch = sys.stdin.read(1)
            return ch
    return None
