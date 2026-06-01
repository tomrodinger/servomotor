"""Make the existing ``servomotor`` control library importable.

The production test system *reuses* the servomotor library that lives at
``<repo>/python_programs/servomotor`` rather than duplicating any of the
command tables, unit conversions, CRC or packet-encoding logic.  This module
puts ``python_programs`` on ``sys.path`` (once) so ``import servomotor`` works
from anywhere in the backend.

Import this module before importing ``servomotor`` anywhere else.
"""

from __future__ import annotations

import os
import sys

# .../production_test_system/backend/bootstrap.py -> repo root is three up.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", ".."))
PYTHON_PROGRAMS = os.path.join(REPO_ROOT, "python_programs")


def ensure_servomotor_importable() -> str:
    """Add ``python_programs`` to ``sys.path`` if needed; return its path."""
    if PYTHON_PROGRAMS not in sys.path:
        sys.path.insert(0, PYTHON_PROGRAMS)
    return PYTHON_PROGRAMS


ensure_servomotor_importable()
