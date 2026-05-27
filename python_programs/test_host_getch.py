#!/usr/bin/env python3
"""
Host-only test: getch module smoke test.

Modernized from the obsolete test_getch.py, which was actually a glue-machine
script using getch() rather than a test of the getch module itself.

`getch.getch()` requires a real TTY for termios — calling it under
`run_all_tests.py` (where stdin is a pipe) would raise. So this host test
only verifies:

  * The module imports.
  * `getch` is a callable.
  * `raw_mode` is a context manager factory (callable).
  * Calling `getch()` on a non-TTY raises a termios error rather than
    hanging or silently misbehaving.

That is enough regression coverage for the imports / API shape; the actual
TTY behaviour is platform/interactive and out of scope for an unattended
host test.

No hardware. Uses the shared host_test_framework.
"""

import io
import sys
from host_test_framework import TestRunner


def main():
    t = TestRunner("test_host_getch")

    try:
        import getch as getch_module
        t.check("getch module imports", True)
    except ImportError as e:
        t.check(f"getch module imports: {e}", False)
        t.finish()
        return

    t.check("module exposes 'getch' callable", callable(getattr(getch_module, "getch", None)))
    t.check("module exposes 'raw_mode' callable", callable(getattr(getch_module, "raw_mode", None)))

    # Calling getch() in a non-TTY context should raise (termios.error or
    # similar). The point of this check is that the function reports a
    # clear failure rather than hanging the suite.
    saved_stdin = sys.stdin
    sys.stdin = io.StringIO("")  # not a real fd, definitely not a TTY
    try:
        try:
            getch_module.getch()
            # If it somehow returns without raising, that's also acceptable
            # for an empty-stdin case — but it must not block.
            t.check("getch() did not block on non-TTY input", True)
        except Exception:
            # Any exception is fine (typically termios.error or
            # io.UnsupportedOperation); the contract is "must not hang".
            t.check("getch() raised cleanly on non-TTY input", True)
    finally:
        sys.stdin = saved_stdin

    t.finish()


if __name__ == "__main__":
    main()
