"""Phase registry — maps phase number to its Stage-A collector instance."""

from __future__ import annotations

from typing import Dict

from .base import Phase, CollectionContext, CancelledError
from . import (
    phase01_firmware, phase02_identity, phase03_electrical, phase04_comms,
    phase05_calibration, phase06_spin_tracking, phase07_hall_linearity,
    phase08_hall_waveform, phase09_current_control, phase10_overvoltage,
    phase11_thermal, phase12_openloop_burnin, phase13_closedloop_burnin,
    phase14_set_alias, phase15_led,
)

_MODULES = [
    phase01_firmware, phase02_identity, phase03_electrical, phase04_comms,
    phase05_calibration, phase06_spin_tracking, phase07_hall_linearity,
    phase08_hall_waveform, phase09_current_control, phase10_overvoltage,
    phase11_thermal, phase12_openloop_burnin, phase13_closedloop_burnin,
    phase14_set_alias, phase15_led,
]

COLLECTORS: Dict[int, Phase] = {m.PHASE.number: m.PHASE for m in _MODULES}


def get_collector(number: int) -> Phase:
    return COLLECTORS[number]
