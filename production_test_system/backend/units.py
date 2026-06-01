"""Unit conversions, sourced from the servomotor library's own factor table.

We deliberately load ``unit_conversions_M3.json`` from the library rather than
hard-coding any numbers, so the production test system always uses the same
conversion factors as the firmware/library and automatically tracks any change
to them.  The motor's *internal* position unit is encoder counts
(``shaft_rotations`` factor = counts per rotation); time is timesteps; etc.
"""

from __future__ import annotations

import json
import os
from typing import Dict

from . import bootstrap

_FACTORS_PATH = os.path.join(
    bootstrap.PYTHON_PROGRAMS, "servomotor", "unit_conversions_M3.json")

with open(_FACTORS_PATH) as _fh:
    _CF: Dict[str, float] = json.load(_fh)["conversion_factors"]

# Internal counts per shaft rotation (position).
COUNTS_PER_ROTATION = float(_CF["shaft_rotations"])
# Internal timesteps per second (time).
TIMESTEPS_PER_SECOND = float(_CF["seconds"])
# Internal velocity units per (rotation / second).
VEL_PER_ROT_PER_S = float(_CF["rotations_per_second"])
# Supply-voltage reading: raw / this = volts.
VOLTS_FACTOR = float(_CF["volts"])


def rotations_to_counts(rot: float) -> int:
    return int(round(rot * COUNTS_PER_ROTATION))


def counts_to_rotations(counts: float) -> float:
    return counts / COUNTS_PER_ROTATION


def seconds_to_timesteps(seconds: float) -> int:
    return int(round(seconds * TIMESTEPS_PER_SECOND))


def rot_per_s_to_internal(rot_per_s: float) -> int:
    return int(round(rot_per_s * VEL_PER_ROT_PER_S))


def raw_voltage_to_volts(raw: int) -> float:
    return raw / VOLTS_FACTOR
