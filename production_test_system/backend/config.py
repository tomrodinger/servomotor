"""Filesystem paths and a few process-wide constants.

Everything under ``data/`` is git-ignored and disposable except the SQLite
database (which is the authoritative record of all collected raw data) and the
settings JSON.  The plots folder is fully regenerable from the database.
"""

from __future__ import annotations

import os

from . import bootstrap  # noqa: F401  (side effect: servomotor on sys.path)

# .../production_test_system/backend/config.py
BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(BACKEND_DIR, ".."))
REPO_ROOT = bootstrap.REPO_ROOT

FRONTEND_DIR = os.path.join(PROJECT_DIR, "frontend")
DATA_DIR = os.path.join(PROJECT_DIR, "data")
PLOTS_DIR = os.path.join(DATA_DIR, "plots")

DB_PATH = os.path.join(DATA_DIR, "production_test.sqlite3")
SETTINGS_PATH = os.path.join(DATA_DIR, "settings.json")

# Where released firmware images live (used by Phase 1).
FIRMWARE_RELEASES_DIR = os.path.join(REPO_ROOT, "firmware", "firmware_releases")

# RS485 / rack constants from the proposal.
BAUD_RATE = 230400
BUS_IDS = ("A", "B", "C")
MOTORS_PER_BUS = 48
POWER_LIMIT_PER_BUS = 8          # max motors at full power simultaneously per bus
SHELVES_PER_BUS = 2
SLOTS_PER_SHELF = 24

# Every system_reset (cmd 27) must be followed by this delay so the device
# leaves the bootloader before the next command is sent.
BOOTLOADER_EXIT_DELAY_S = 1.0


def ensure_data_dirs() -> None:
    """Create the data + plots directories if they do not yet exist."""
    os.makedirs(DATA_DIR, exist_ok=True)
    os.makedirs(PLOTS_DIR, exist_ok=True)
