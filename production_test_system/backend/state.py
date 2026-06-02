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

# Test-set colours (new-vs-known coding).
COLOR_GREEN = "green"     # new motor, or already in this session's set
COLOR_ORANGE = "orange"   # in DB but not in this session's set (or duplicate ID)

# Rack-grid collection-status colours.
GRID_GRAY = "gray"        # no data collected yet
GRID_YELLOW = "yellow"    # collecting now
GRID_BLUE = "blue"        # collected
GRID_ORANGE = "orange"    # awaiting human confirmation (LED test)


class BusState:
    def __init__(self, bus_id: str):
        self.bus_id = bus_id
        self.pending_detections = 0
        self.detecting = False
        # test set: unique_id -> {"alias": int, "color": str, "order": int}
        self.test_set: Dict[int, Dict[str, Any]] = {}
        self._next_order = 0
        # collection status per unique_id (rack grid)
        self.grid_status: Dict[int, str] = {}
        self.running = False
        self.paused = False
        self.current_phase: Optional[int] = None
        self.status_message = ""

    def add_to_set(self, unique_id: int, alias: int, color: str) -> None:
        if unique_id in self.test_set:
            # The colour was decided when this motor was first added to the set
            # this session and must NOT change on re-detection — re-detecting a
            # motor across passes (needed to beat collisions) keeps its original
            # colour (in particular an orange "already in DB" motor stays orange).
            self.test_set[unique_id]["alias"] = alias
        else:
            self.test_set[unique_id] = {
                "alias": alias, "color": color, "order": self._next_order}
            self._next_order += 1
            self.grid_status.setdefault(unique_id, GRID_GRAY)

    def clear_set(self) -> None:
        self.test_set.clear()
        self.grid_status.clear()
        self._next_order = 0

    def snapshot(self) -> Dict[str, Any]:
        motors = []
        for uid, info in sorted(self.test_set.items(), key=lambda kv: kv[1]["order"]):
            motors.append({
                "unique_id": uid_hex(uid),
                "alias": info["alias"],
                "color": info["color"],
                "order": info["order"],
                "grid_status": self.grid_status.get(uid, GRID_GRAY),
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
