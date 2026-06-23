"""Live, in-memory application state broadcast to the browser(s).

Per the proposal the rack grid and detection lists are a *live view only* — no
slot is persisted; the database is keyed solely on the unique ID.  This module
holds that volatile state: per-bus detection (pending-pass counter + test set
with colour coding) and per-run collection status (the rack grid colours), plus
the run controls (running / paused / current phase).

It is mutated from the bus-worker threads and read from the asyncio API thread,
so every accessor is guarded by a lock.  Change notifications are delivered via
registered listeners (the app registers one that pushes a fresh snapshot to all
connected WebSockets); the listener is responsible for any thread hopping.
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, List, Optional

from . import config
from .database import uid_hex

# Test-set colours (untested-vs-tested coding).
COLOR_GREEN = "green"     # detected, no tests performed on it yet
COLOR_ORANGE = "orange"   # already has test results recorded in the DB

# Rack-grid per-phase collection/evaluation status colours.  Every detected
# motor shows a 15-cell mini-grid (one cell per phase P1..P15); each cell takes
# one of these five states:
GRID_GRAY = "gray"        # no data collected for this phase yet
GRID_YELLOW = "yellow"    # this phase is under test right now (collecting)
GRID_BLUE = "blue"        # data collected, awaiting evaluation
GRID_GREEN = "green"      # evaluated -> passed
GRID_RED = "red"          # evaluated -> failed (or data missing on a re-eval)
# (legacy alias kept so old imports/tests don't break)
GRID_ORANGE = "orange"

# The 15 test phases, in display order (3 rows x 5 columns in the UI).
PHASE_NUMBERS = list(range(1, 16))


def _blank_grid() -> Dict[int, str]:
    return {ph: GRID_GRAY for ph in PHASE_NUMBERS}


def phase_grid_from_db(db, unique_id: int) -> Dict[int, str]:
    """Derive each phase's grid colour for a motor purely from the database.

    Used to seed the live grid when a motor is (re-)detected and to refresh it
    after an evaluation run.  Per phase: no collected data -> gray; data but no
    up-to-date evaluation -> blue; evaluated -> green (pass or operator-cleared)
    or red (fail/missing).  A fresh collection (data newer than the latest eval)
    reverts the cell to blue so it reads as "pending re-evaluation".
    """
    grid = _blank_grid()
    for ph in PHASE_NUMBERS:
        data = db.latest_phase_data(unique_id, ph)
        if not data:
            continue                       # gray: nothing collected
        ev = db.latest_phase_eval(unique_id, ph)
        if ev and ev.get("evaluated_at", 0) >= data.get("collected_at", 0):
            passed = ev.get("result") == "pass" or ev.get("cleared")
            grid[ph] = GRID_GREEN if passed else GRID_RED
        else:
            grid[ph] = GRID_BLUE           # collected, awaiting (re-)evaluation
    return grid


class BusState:
    def __init__(self, bus_id: str):
        self.bus_id = bus_id
        self.pending_detections = 0
        self.detecting = False
        # test set: unique_id -> {"alias": int, "color": str, "order": int}
        self.test_set: Dict[int, Dict[str, Any]] = {}
        self._next_order = 0
        # per-phase collection/evaluation status (rack grid):
        # unique_id -> {phase_number -> GRID_* colour}
        self.phase_status: Dict[int, Dict[int, str]] = {}
        self.running = False
        self.paused = False
        self.current_phase: Optional[int] = None
        self.status_message = ""

    def add_to_set(self, unique_id: int, alias: int, color: str,
                   phase_status: Optional[Dict[int, str]] = None) -> None:
        if unique_id in self.test_set:
            # The colour was decided when this motor was first added to the set
            # this session and must NOT change on re-detection — re-detecting a
            # motor across passes (needed to beat collisions) keeps its original
            # colour (in particular an orange "already in DB" motor stays orange).
            # The live phase grid is likewise preserved (a run in progress must
            # not be clobbered by a re-detection pass).
            self.test_set[unique_id]["alias"] = alias
        else:
            self.test_set[unique_id] = {
                "alias": alias, "color": color, "order": self._next_order}
            self._next_order += 1
            grid = _blank_grid()
            if phase_status:
                grid.update(phase_status)
            self.phase_status[unique_id] = grid

    def set_phase(self, unique_id: int, phase: int, status: str) -> None:
        self.phase_status.setdefault(unique_id, _blank_grid())[phase] = status

    def clear_set(self) -> None:
        self.test_set.clear()
        self.phase_status.clear()
        self._next_order = 0

    def snapshot(self) -> Dict[str, Any]:
        motors = []
        for uid, info in sorted(self.test_set.items(), key=lambda kv: kv[1]["order"]):
            grid = self.phase_status.get(uid, {})
            motors.append({
                "unique_id": uid_hex(uid),
                "alias": info["alias"],
                "color": info["color"],
                "order": info["order"],
                # one colour per phase P1..P15, in display order
                "phase_grid": [grid.get(ph, GRID_GRAY) for ph in PHASE_NUMBERS],
            })
        return {
            "bus_id": self.bus_id,
            "pending_detections": self.pending_detections,
            "detecting": self.detecting,
            "motor_count": len(self.test_set),
            "motors": motors,
            "running": self.running,
            "paused": self.paused,
            "current_phase": self.current_phase,
            "status_message": self.status_message,
        }


class AppState:
    def __init__(self):
        self._lock = threading.RLock()
        self.buses: Dict[str, BusState] = {b: BusState(b) for b in config.BUS_IDS}
        self._listeners: List[Callable[[], None]] = []
        # LED-test (Phase 15) human-confirmation prompt, if pending.
        self.led_prompt: Optional[Dict[str, Any]] = None

    # -- change notification ------------------------------------------------
    def add_listener(self, fn: Callable[[], None]) -> None:
        with self._lock:
            self._listeners.append(fn)

    def notify(self) -> None:
        for fn in list(self._listeners):
            try:
                fn()
            except Exception:
                pass

    # -- guarded mutation helper -------------------------------------------
    def update(self, fn: Callable[["AppState"], Any]) -> Any:
        """Run ``fn`` under the lock, then notify listeners. Returns fn's result."""
        with self._lock:
            result = fn(self)
        self.notify()
        return result

    def bus(self, bus_id: str) -> BusState:
        return self.buses[bus_id]

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "buses": {b: st.snapshot() for b, st in self.buses.items()},
                "led_prompt": self.led_prompt,
            }
