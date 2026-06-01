"""Optional no-hardware demo rack.

When the environment variable ``PTS_SIMULATE=1`` is set, the app wires its bus
workers to a :class:`RackSimulator` populated with a spread of good and faulty
motors, so the whole UI and both stages can be exercised without any hardware.
This is purely a convenience for development and for trying the UI while the
real RS485 wiring is being built.
"""

from __future__ import annotations

from typing import Callable

from . import config
from .simulator import RackSimulator, SimMotor, MotorProfile
from .transport import Transport


def build_demo_rack(motors_per_bus: int = 6) -> RackSimulator:
    rack = RackSimulator()
    # thermal_slope 0.8 C/s over the 10 s demo thermal run gives ~8 C rise
    # (inside the realistic 5-20 C band), so good motors pass.
    faults = [
        ("good", dict(thermal_slope=0.8)),
        ("bad_firmware", dict(firmware_version="0.14.0.0", thermal_slope=0.8)),
        ("low_voltage", dict(supply_voltage=20.0, thermal_slope=0.8)),
        ("dead_magnet", dict(dead_magnet=True, thermal_slope=0.8)),
        ("weak_current", dict(current_limit_broken=True, thermal_slope=0.8)),
        ("thermal_hot", dict(thermal_slope=3.0)),
    ]
    for bus_index, bus_id in enumerate(config.BUS_IDS):
        for i in range(motors_per_bus):
            uid = 0xD000_0000_0000_0000 + (bus_index << 16) + i
            _, overrides = faults[i % len(faults)]
            rack.add_motor(bus_id, SimMotor(uid, profile=MotorProfile(**overrides)))
    return rack


def apply_fast_demo_settings(settings) -> None:
    """Shrink the long phases so a demo run finishes in well under a minute."""
    settings.set_phase_param(4, "ping_count", 30)
    settings.set_phase_param(5, "quiet_hold_s", 2.0)
    settings.set_phase_param(6, "spin_rotations", 0.5)
    settings.set_phase_param(6, "spin_velocity", 2.0)
    settings.set_phase_param(7, "n_steps", 60)
    settings.set_phase_param(7, "settle_ms", 2)
    settings.set_phase_param(11, "duration_s", 10.0)
    settings.set_phase_param(12, "duration_s", 5.0)
    settings.set_phase_param(12, "spin_interval_s", 0.2)
    settings.set_phase_param(13, "duration_s", 6.0)
    settings.set_phase_param(13, "move_duration_s", 0.2)
    settings.set_phase_param(14, "reboot_delay_s", 0.2)


def demo_transport_factory(rack: RackSimulator) -> Callable[[str, str], Transport]:
    def factory(bus_id, port):
        return rack.transport_for(bus_id)
    return factory
