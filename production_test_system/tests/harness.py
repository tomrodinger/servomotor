"""Shared test harness: a simulated rack + a Runner wired to it, with fast
durations so a full 15-phase collection runs in a couple of seconds.
"""

from __future__ import annotations

import os
import tempfile
from typing import Dict, List, Optional

from backend import config
from backend.database import Database
from backend.settings import Settings
from backend.state import AppState
from backend.simulator import RackSimulator, SimMotor, MotorProfile
from backend.runner import Runner


def fast_settings(settings: Settings) -> None:
    """Shrink every long-running phase so tests finish quickly."""
    settings.set_phase_param(4, "ping_count", 10)
    settings.set_phase_param(5, "quiet_hold_s", 0.1)
    settings.set_phase_param(6, "spin_rotations", 0.3)
    settings.set_phase_param(6, "spin_velocity", 3.0)
    settings.set_phase_param(7, "n_steps", 12)
    settings.set_phase_param(7, "settle_ms", 1)
    settings.set_phase_param(9, "wait_s", 0.5)
    # Temperature is reported as an integer (deg C), so the test run must be
    # long enough to produce a measurable integer rise.  With good_profile()'s
    # 2.0 C/s slope, 4 s gives ~8 C (inside the realistic 5-20 C band).
    settings.set_phase_param(11, "duration_s", 4.0)
    settings.update_phase(11, criteria={"slope_min": 0.0, "slope_max": 10.0,
                                        "r_value_min": 0.5})
    # Phase 12 picks 8 random motors per round and spends 0.3 s setting up each
    # new one, so the window must be long enough for every motor to be selected
    # at least once.
    settings.set_phase_param(12, "duration_s", 4.0)
    settings.set_phase_param(12, "spin_interval_s", 0.1)
    # Phase 13 staggers starts over 0-1 s, so the run must outlast that plus a
    # few moves per motor.
    settings.set_phase_param(13, "duration_s", 2.5)
    settings.set_phase_param(13, "move_duration_s", 0.1)
    settings.set_phase_param(14, "reboot_delay_s", 0.05)


def good_profile(**overrides) -> MotorProfile:
    """A nominal motor whose thermal slope (1.0 C/s) gives a measurable integer
    rise over the short test thermal run.  Override fields to inject faults."""
    fields = dict(thermal_slope=2.0)
    fields.update(overrides)
    return MotorProfile(**fields)


class Harness:
    def __init__(self, fast: bool = True, fast_bootloader: bool = True):
        d = tempfile.mkdtemp(prefix="pts-test-")
        self.db = Database(os.path.join(d, "test.sqlite3"))
        self.settings = Settings(os.path.join(d, "settings.json"))
        self.state = AppState()
        self.rack = RackSimulator()
        self.logs: List[str] = []
        if fast:
            fast_settings(self.settings)
        if fast_bootloader:
            config.BOOTLOADER_EXIT_DELAY_S = 0.02

        def factory(bus_id, port):
            return self.rack.transport_for(bus_id)

        self.runner = Runner(self.db, self.settings, self.state,
                             transport_factory=factory, log_fn=self.logs.append)

    def add_motor(self, bus_id: str, unique_id: int,
                  profile: Optional[MotorProfile] = None,
                  detect_prob: float = 1.0) -> SimMotor:
        return self.rack.add_motor(
            bus_id, SimMotor(unique_id, profile=profile or MotorProfile()),
            detect_prob=detect_prob)

    def detect_and_wait(self, bus_id: str, passes: int = 1) -> None:
        for _ in range(passes):
            self.runner.detect(bus_id)
        self.runner.workers[bus_id].wait_idle()

    def run_and_wait(self, bus_id: str) -> None:
        self.runner.start_section(bus_id)
        self.runner.workers[bus_id].wait_idle()
