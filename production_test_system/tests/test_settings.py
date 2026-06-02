import os
import tempfile

import pytest

from backend.settings import Settings


def _tmp():
    return os.path.join(tempfile.mkdtemp(), "settings.json")


def test_default_phases_enabled():
    s = Settings(_tmp())
    assert s.phase_enabled(1)
    assert s.phase_enabled(10)                # OV firmware dep resolved in 0.15.1.0
    assert 10 in s.enabled_phase_numbers()


def test_port_uniqueness_and_ready():
    s = Settings(_tmp())
    assert not s.ports_ready()
    s.set_serial_port("A", "/dev/a")
    s.set_serial_port("B", "/dev/b")
    s.set_serial_port("C", "/dev/c")
    assert s.ports_ready()
    with pytest.raises(ValueError):
        s.set_serial_port("B", "/dev/a")      # duplicate rejected


def test_atomic_persist_and_reload():
    path = _tmp()
    s = Settings(path)
    s.set_phase_param(5, "quiet_hold_s", 7.0)
    s.set_serial_port("A", "/dev/x")
    s2 = Settings(path)
    assert s2.phase_params(5)["quiet_hold_s"] == 7.0
    assert s2.get_serial_ports()["A"] == "/dev/x"


def test_criteria_version_bumps_only_on_change():
    s = Settings(_tmp())
    v = s.criteria_version
    s.set_phase_criterion(3, "voltage_nominal", 23.0)
    assert s.criteria_version == v + 1
    s.set_phase_criterion(3, "voltage_nominal", 23.0)   # same value -> no bump
    assert s.criteria_version == v + 1


def test_corrupt_file_falls_back_to_defaults():
    path = _tmp()
    with open(path, "w") as fh:
        fh.write("{ this is not json")
    s = Settings(path)                         # must not raise
    assert s.phase_enabled(1)
