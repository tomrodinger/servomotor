"""Phase base class and the collection context (Stage A).

Each phase implements :meth:`Phase.collect`, doing *only* data collection: it
runs on the hardware and stores all raw data to the database.  **No metrics, no
PNGs and no pass/fail are produced during collection** — that is Stage B.

A :class:`CollectionContext` gives a phase everything it needs for one bus:
the motors to test, this phase's parameters, a :class:`MotorClient` per motor,
the bus transport (for broadcasts), interruptible sleep/pause/cancel, the rack-
grid status setter, and a ``store`` helper that writes a ``phase_data`` row and
marks the motor "collected".
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional

from .. import config
from ..database import Database
from ..motor_client import MotorClient
from ..state import AppState, GRID_BLUE, GRID_YELLOW, GRID_ORANGE
from ..transport import Transport


class CancelledError(Exception):
    """Raised inside a phase when the run is cancelled."""


class CollectionContext:
    def __init__(self, *, bus_id: str, transport: Transport, db: Database,
                 settings, state: AppState, motors: List[int],
                 pause_event: threading.Event, cancel_event: threading.Event,
                 fw_versions: Dict[int, str], log_fn: Optional[Callable[[str], None]] = None):
        self.bus_id = bus_id
        self.transport = transport
        self.db = db
        self.settings = settings
        self.state = state
        self.motors = list(motors)
        self.power_limit = config.POWER_LIMIT_PER_BUS
        self._pause = pause_event
        self._cancel = cancel_event
        self._fw_versions = fw_versions
        self._log_fn = log_fn
        self.phase_number: int = 0
        self.params: Dict[str, Any] = {}
        self._clients: Dict[int, MotorClient] = {}

    # -- run control --------------------------------------------------------
    @property
    def cancelled(self) -> bool:
        return self._cancel.is_set()

    def check_cancel(self) -> None:
        if self._cancel.is_set():
            raise CancelledError()

    def wait_if_paused(self) -> None:
        while self._pause.is_set() and not self._cancel.is_set():
            time.sleep(0.1)
        self.check_cancel()

    def sleep(self, seconds: float) -> None:
        """Interruptible sleep: returns early (raises) if cancelled."""
        deadline = time.monotonic() + seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            if self._cancel.is_set():
                raise CancelledError()
            time.sleep(min(0.1, remaining))

    # -- helpers ------------------------------------------------------------
    def client(self, unique_id: int) -> MotorClient:
        c = self._clients.get(unique_id)
        if c is None:
            c = MotorClient(self.transport, unique_id)
            self._clients[unique_id] = c
        return c

    def log(self, message: str) -> None:
        if self._log_fn:
            self._log_fn("[Bus %s] %s" % (self.bus_id, message))

    def set_grid(self, unique_id: int, status: str) -> None:
        bus = self.state.bus(self.bus_id)
        def _do(_):
            bus.grid_status[unique_id] = status
        self.state.update(_do)

    def mark_collecting(self, unique_id: int) -> None:
        self.set_grid(unique_id, GRID_YELLOW)

    def firmware_version(self, unique_id: int) -> Optional[str]:
        return self._fw_versions.get(unique_id)

    def set_firmware_version(self, unique_id: int, version: str) -> None:
        self._fw_versions[unique_id] = version

    def calibrated_motors(self) -> List[int]:
        """Motors in this run that have calibration_done set (prerequisite).

        Logs/surfaces when motors are skipped for lacking calibration so the
        operator isn't left wondering why a motion phase did nothing (it depends
        on Phase 5 having run successfully).
        """
        ready = [uid for uid in self.motors if self.db.get_calibration_done(uid)]
        skipped = len(self.motors) - len(ready)
        if skipped:
            msg = ("Phase %d: %d of %d motors are not calibrated and will be "
                   "skipped — run Phase 5 (Calibration) first." %
                   (self.phase_number, skipped, len(self.motors)))
            self.log(msg)
            self.state.update(
                lambda _: setattr(self.state.bus(self.bus_id), "status_message", msg))
        return ready

    def batched(self, motors: Optional[Iterable[int]] = None,
                size: Optional[int] = None) -> Iterable[List[int]]:
        items = list(self.motors if motors is None else motors)
        n = size or self.power_limit
        for i in range(0, len(items), n):
            yield items[i:i + n]

    def store(self, unique_id: int, *, raw_blob: Optional[bytes] = None,
              scalar: Optional[float] = None,
              observation: Optional[Dict[str, Any]] = None,
              detail: Optional[str] = None, grid: str = GRID_BLUE) -> int:
        """Write one Stage-A phase_data row and update the rack grid."""
        data_id = self.db.insert_phase_data(
            unique_id, self.phase_number,
            firmware_version=self.firmware_version(unique_id),
            raw_blob=raw_blob, scalar=scalar, observation=observation, detail=detail)
        self.set_grid(unique_id, grid)
        return data_id

    def reset_motor(self, unique_id: int) -> None:
        """system_reset one motor + the mandatory 1 s bootloader-exit delay."""
        try:
            self.client(unique_id).system_reset()
        except Exception:
            pass
        self.sleep(config.BOOTLOADER_EXIT_DELAY_S)


class Phase:
    """Base class for a Stage-A collector. Subclasses implement ``collect``."""
    number: int = 0

    def collect(self, ctx: CollectionContext) -> None:
        raise NotImplementedError
