"""Persistent settings with atomic writes.

All configuration — the per-bus serial-port assignment, per-phase
enable/disable, every phase's collection parameters and pass/fail criteria, and
the run scope — lives in one JSON file on disk.  It is saved on every change and
reloaded at startup so settings survive restarts.

Writes are **atomic**: serialize to a temp file in the same directory, ``fsync``
it, then ``os.replace()`` over the real file.  A crash mid-write therefore never
corrupts or truncates the settings — the old file stays intact until the rename.

A monotonically increasing ``criteria_version`` is bumped whenever any pass/fail
criterion changes, and is recorded with every Stage-B evaluation so results are
traceable to the criteria that produced them.
"""

from __future__ import annotations

import json
import os
import tempfile
import threading
from typing import Any, Dict, List, Optional

from . import config
from . import phase_defs

_lock = threading.RLock()


def _default_settings() -> Dict[str, Any]:
    phases: Dict[str, Any] = {}
    for p in phase_defs.all_phases():
        phases[str(p.number)] = {
            "enabled": p.enabled_default,
            "params": {param.key: param.default for param in p.params},
            "criteria": {param.key: param.default for param in p.criteria},
        }
    return {
        "serial_ports": {bus: None for bus in config.BUS_IDS},
        "run_scope": "all",          # "all" | "incomplete"
        "phases": phases,
        "criteria_version": 1,
    }


def _merge_defaults(loaded: Dict[str, Any]) -> Dict[str, Any]:
    """Fill in any keys missing from a loaded file (forward/backward compat)."""
    base = _default_settings()
    base["serial_ports"].update(loaded.get("serial_ports", {}) or {})
    base["run_scope"] = loaded.get("run_scope", base["run_scope"])
    base["criteria_version"] = loaded.get("criteria_version", base["criteria_version"])
    for num, defaults in base["phases"].items():
        saved = (loaded.get("phases", {}) or {}).get(num, {})
        if "enabled" in saved:
            defaults["enabled"] = saved["enabled"]
        defaults["params"].update(saved.get("params", {}) or {})
        defaults["criteria"].update(saved.get("criteria", {}) or {})
    return base


class Settings:
    def __init__(self, path: Optional[str] = None):
        self.path = path or config.SETTINGS_PATH
        self._data: Dict[str, Any] = self._load()

    # -- persistence --------------------------------------------------------
    def _load(self) -> Dict[str, Any]:
        if os.path.exists(self.path):
            try:
                with open(self.path) as fh:
                    return _merge_defaults(json.load(fh))
            except (json.JSONDecodeError, OSError):
                # Corrupt/unreadable file: fall back to defaults rather than crash.
                pass
        return _default_settings()

    def save(self) -> None:
        with _lock:
            config.ensure_data_dirs()
            directory = os.path.dirname(self.path) or "."
            fd, tmp = tempfile.mkstemp(dir=directory, prefix=".settings-", suffix=".tmp")
            try:
                with os.fdopen(fd, "w") as fh:
                    json.dump(self._data, fh, indent=2, sort_keys=True)
                    fh.flush()
                    os.fsync(fh.fileno())
                os.replace(tmp, self.path)      # atomic on POSIX
            except BaseException:
                if os.path.exists(tmp):
                    os.unlink(tmp)
                raise

    # -- accessors ----------------------------------------------------------
    def as_dict(self) -> Dict[str, Any]:
        with _lock:
            return json.loads(json.dumps(self._data))   # deep copy

    @property
    def run_scope(self) -> str:
        return self._data["run_scope"]

    def set_run_scope(self, scope: str) -> None:
        if scope not in ("all", "incomplete", "incomplete_or_failed"):
            raise ValueError(
                "run_scope must be 'all', 'incomplete' or 'incomplete_or_failed'")
        with _lock:
            self._data["run_scope"] = scope
            self.save()

    @property
    def criteria_version(self) -> int:
        return self._data["criteria_version"]

    # serial ports ----------------------------------------------------------
    def get_serial_ports(self) -> Dict[str, Optional[str]]:
        return dict(self._data["serial_ports"])

    def set_serial_port(self, bus: str, port: Optional[str]) -> None:
        if bus not in config.BUS_IDS:
            raise ValueError("unknown bus %r" % bus)
        with _lock:
            new = dict(self._data["serial_ports"])
            new[bus] = port
            self._validate_ports_unique(new)
            self._data["serial_ports"] = new
            self.save()

    @staticmethod
    def _validate_ports_unique(ports: Dict[str, Optional[str]]) -> None:
        assigned = [p for p in ports.values() if p]
        if len(assigned) != len(set(assigned)):
            raise ValueError("the same serial port is assigned to more than one bus")

    def ports_ready(self) -> bool:
        """True iff all three buses have distinct, non-empty ports."""
        ports = self.get_serial_ports()
        assigned = [p for p in ports.values() if p]
        return len(assigned) == len(config.BUS_IDS) and len(set(assigned)) == len(assigned)

    # phases ----------------------------------------------------------------
    def phase_enabled(self, number: int) -> bool:
        return bool(self._data["phases"][str(number)]["enabled"])

    def set_phase_enabled(self, number: int, enabled: bool) -> None:
        with _lock:
            self._data["phases"][str(number)]["enabled"] = bool(enabled)
            self.save()

    def enabled_phase_numbers(self) -> List[int]:
        return [p.number for p in phase_defs.all_phases()
                if self.phase_enabled(p.number)]

    def phase_params(self, number: int) -> Dict[str, Any]:
        return dict(self._data["phases"][str(number)]["params"])

    def phase_criteria(self, number: int) -> Dict[str, Any]:
        return dict(self._data["phases"][str(number)]["criteria"])

    def set_phase_param(self, number: int, key: str, value: Any) -> None:
        with _lock:
            self._data["phases"][str(number)]["params"][key] = value
            self.save()

    def set_phase_criterion(self, number: int, key: str, value: Any) -> None:
        with _lock:
            criteria = self._data["phases"][str(number)]["criteria"]
            if key not in criteria or criteria[key] != value:
                criteria[key] = value
                self._data["criteria_version"] += 1   # criteria changed
            self.save()

    def update_phase(self, number: int, *, enabled: Optional[bool] = None,
                     params: Optional[Dict[str, Any]] = None,
                     criteria: Optional[Dict[str, Any]] = None) -> None:
        """Bulk-update a phase; bumps criteria_version if any criterion changes."""
        with _lock:
            entry = self._data["phases"][str(number)]
            if enabled is not None:
                entry["enabled"] = bool(enabled)
            if params:
                entry["params"].update(params)
            if criteria:
                changed = any(entry["criteria"].get(k) != v for k, v in criteria.items())
                entry["criteria"].update(criteria)
                if changed:
                    self._data["criteria_version"] += 1
            self.save()
