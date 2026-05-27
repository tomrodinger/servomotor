#!/usr/bin/env python3
"""
Host-only test: terminal_formatting module — the format_* functions return
strings that round-trip the supplied message text. ANSI control codes may or
may not be present depending on terminal capability detection, so the test
only asserts that the user's message appears in the formatted output.

Modernized from the obsolete test_terminal_formatting.py, which only printed
sample output with no assertions.

No hardware. Uses the shared host_test_framework.
"""

import sys
from host_test_framework import TestRunner


def main():
    t = TestRunner("test_host_terminal_formatting")

    # The module should import cleanly with the expected symbols.
    try:
        from terminal_formatting import (
            STYLE, format_error, format_warning,
            format_success, format_info, format_debug,
            format_text, print_styled,
        )
        t.check("terminal_formatting module imports the expected symbols", True)
    except ImportError as e:
        t.check(f"terminal_formatting imports: {e}", False)
        t.finish()
        return

    # The TerminalStyle singleton should expose capability flags.
    t.check("STYLE.has_color is a bool", isinstance(STYLE.has_color, bool))
    t.check("STYLE.has_unicode is a bool", isinstance(STYLE.has_unicode, bool))

    # Each format_* helper returns a string and contains the input text.
    sample = "TEST_MESSAGE_123"
    for label, fn in [
        ("format_success", format_success),
        ("format_error", format_error),
        ("format_warning", format_warning),
        ("format_info", format_info),
        ("format_debug", format_debug),
    ]:
        out = fn(sample)
        t.check(f"{label} returns a str", isinstance(out, str))
        t.check(f"{label} preserves the message", sample in out)

    # format_text accepts (text, *styles) and should not crash.
    out = format_text(sample)
    t.check("format_text(sample) returns a str containing the message",
            isinstance(out, str) and sample in out)

    t.finish()


if __name__ == "__main__":
    main()
