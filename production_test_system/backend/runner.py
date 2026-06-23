"""Coordinator over the three bus workers.

Owns one :class:`BusWorker` per bus and exposes the run controls the UI drives:
detect, clear, Start All / Start Section, Pause/Resume, Cancel, System Reset,
re-run phase, and the LED-test (Phase 15) human-confirmation reconciliation.
The workers do all the bus I/O on their own threads; the runner just routes
commands and performs the no-hardware LED bookkeeping.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional

from . import config
from .bus_worker import BusWorker
from .database import Database
from .detection import DETECT_TIMEOUT
from .settings import Settings
from .state import AppState, phase_grid_from_db
from .transport_factory import TransportFactory, serial_factory

LED_PHASE = 15

# "Flash green" locate-in-rack behaviour.  The Identify command (cmd 41) makes
# the firmware flash its green LED for ~2.4 s on its own, so we re-trigger it a
# little faster than that to get continuous flashing, and stop after a generous
# cap so a forgotten flash never ties up a bus forever.
FLASH_PERIOD_S = 2.0
FLASH_MAX_S = 300.0   # 5-minute safety auto-stop (also cancellable from the UI)


class Runner:
    def __init__(self, db: Database, settings: Settings, state: AppState,
                 transport_factory: Optional[TransportFactory] = None,
                 log_fn: Optional[Callable[[str], None]] = None):
        self.db = db
        self.settings = settings
        self.state = state
        self._factory = transport_factory or serial_factory
        self.log_fn = log_fn
        self.workers: Dict[str, BusWorker] = {
            bus: BusWorker(bus, db, settings, state, self._factory, log_fn)
            for bus in config.BUS_IDS}
        for w in self.workers.values():
            w.start()
        # Per-bus "identify" coordination.  One coordinator loop per bus services
        # *all* locate requests on that bus so any number of devices can flash at
        # once (they share the single RS485 line): ``_flashing`` is the set of
        # unique_ids to keep green-flashing, ``_pending_solid`` the ids to lock
        # solid (test mode 13) once.  ``_flash_wake`` nudges a sleeping loop when
        # the sets change; ``_flash_active`` says whether a loop is already up.
        self._flash_lock = threading.Lock()
        self._flashing: Dict[str, set] = {b: set() for b in config.BUS_IDS}
        self._pending_solid: Dict[str, set] = {b: set() for b in config.BUS_IDS}
        self._flash_active: Dict[str, bool] = {b: False for b in config.BUS_IDS}
        self._flash_wake: Dict[str, threading.Event] = {
            b: threading.Event() for b in config.BUS_IDS}

    # -- detection / test set ----------------------------------------------
    def detect(self, bus_id: str) -> None:
        self.workers[bus_id].submit_detect()

    def clear_set(self, bus_id: str) -> None:
        self.state.update(lambda _: self.state.bus(bus_id).clear_set())

    def clear_all(self) -> None:
        def _do(_):
            for bus in self.state.buses.values():
                bus.clear_set()
        self.state.update(_do)

    # -- run controls -------------------------------------------------------
    def start_all(self) -> None:
        for bus in config.BUS_IDS:
            self.workers[bus].submit_run()

    def start_section(self, bus_id: str) -> None:
        self.workers[bus_id].submit_run()

    def pause(self) -> None:
        for w in self.workers.values():
            w.pause()

    def resume(self) -> None:
        for w in self.workers.values():
            w.resume()

    def cancel(self) -> None:
        for w in self.workers.values():
            w.cancel()

    def stop(self) -> None:
        # Stop = cancel the active run, leaving motors where they are.
        self.cancel()

    def system_reset_all(self) -> None:
        for w in self.workers.values():
            w.submit_system_reset()

    def rerun_phase(self, bus_id: str, phase_number: int) -> None:
        self.workers[bus_id].submit_rerun_phase(phase_number)

    def find_bus(self, unique_id: int) -> Optional[str]:
        for bus_id, bus in self.state.buses.items():
            if unique_id in bus.test_set:
                return bus_id
        return None

    def identify_flash_start(self, unique_id: int) -> bool:
        """Start flashing a device's green LED (cmd 41, re-triggered) so it can
        be located in the rack.  Any number of devices on a bus can flash at the
        same time.  Runs until cancelled (``identify_flash_stop``), a global
        Cancel, or the ``FLASH_MAX_S`` safety cap; the motor stays responsive."""
        bus_id = self.find_bus(unique_id)
        if bus_id is None:
            return False
        with self._flash_lock:
            self._flashing[bus_id].add(unique_id)
            self._ensure_identify_loop(bus_id)
        return True

    def identify_flash_stop(self, unique_id: int) -> bool:
        """Stop flashing a device (a no-op if it wasn't flashing)."""
        with self._flash_lock:
            for bus_id in config.BUS_IDS:
                self._flashing[bus_id].discard(unique_id)
                self._flash_wake[bus_id].set()
        return True

    def identify_solid(self, unique_id: int) -> bool:
        """Light a device's LEDs solid green+red (test mode 13) so it is
        unmistakable in the rack.  This *locks up* the motor (see
        :meth:`MotorClient.leds_solid`) — it must be power-cycled afterwards.
        Works while other devices on the same bus are green-flashing."""
        bus_id = self.find_bus(unique_id)
        if bus_id is None:
            return False
        with self._flash_lock:
            self._flashing[bus_id].discard(unique_id)   # stop flashing it
            self._pending_solid[bus_id].add(unique_id)
            self._ensure_identify_loop(bus_id)
        return True

    def _ensure_identify_loop(self, bus_id: str) -> None:
        """Ensure a coordinator loop is running for ``bus_id``.  MUST be called
        while holding ``self._flash_lock``."""
        self._flash_wake[bus_id].set()
        if not self._flash_active[bus_id]:
            self._flash_active[bus_id] = True
            worker = self.workers[bus_id]
            worker._jobs.put(lambda: self._identify_loop(bus_id, worker))

    def _identify_loop(self, bus_id: str, worker: BusWorker) -> None:
        """Service every locate request on one bus from a single thread: each
        pass re-triggers Identify for all flashing devices and sends test mode 13
        to any newly-requested solid ones, then sleeps a little under the
        firmware's flash duration so the flashing looks continuous."""
        from .motor_client import MotorClient
        wake = self._flash_wake[bus_id]
        transport = None
        try:
            transport = worker._open_transport()
            deadline = time.monotonic() + FLASH_MAX_S
            while True:
                cancelled = worker._cancel.is_set()
                expired = time.monotonic() >= deadline
                with self._flash_lock:
                    wake.clear()
                    if cancelled or expired:
                        self._flashing[bus_id].clear()
                        self._pending_solid[bus_id].clear()
                    solids = list(self._pending_solid[bus_id])
                    self._pending_solid[bus_id].clear()
                    for uid in solids:
                        self._flashing[bus_id].discard(uid)
                    flashes = list(self._flashing[bus_id])
                    if not flashes and not solids:
                        self._flash_active[bus_id] = False
                        break
                for uid in solids:
                    try:
                        MotorClient(transport, uid).leds_solid()
                    except Exception:
                        pass
                for uid in flashes:
                    try:
                        MotorClient(transport, uid).identify()
                    except Exception:
                        pass
                wake.wait(FLASH_PERIOD_S)
        except Exception:
            with self._flash_lock:           # relinquish so a later start can retry
                self._flash_active[bus_id] = False
            raise
        finally:
            if transport is not None and not transport.is_simulated:
                transport.close()

    # -- LED test (Phase 15) reconciliation --------------------------------
    def led_confirm_all_pass(self) -> None:
        """Human answered Yes: mark the LED test passed for every motor."""
        for bus_id, bus in self.state.buses.items():
            for uid in list(bus.test_set.keys()):
                self.db.insert_phase_data(uid, LED_PHASE,
                                          observation={"leds_lit": True})
        self._clear_led_prompt()
        self._grade_led()

    def led_check_removed(self) -> None:
        """Ping every test-set motor; record present=pass, missing=fail.

        Runs on the worker threads (it needs the bus).  The set of missing
        (removed) motors is surfaced in ``state.led_prompt`` for the operator.
        """
        missing_lock = threading.Lock()
        missing: List[str] = []

        def make_job(bus_id):
            def _job():
                from .database import uid_hex
                from .motor_client import MotorClient
                from .transport import TimeoutError as RS485Timeout, FatalError
                worker = self.workers[bus_id]
                transport = worker._open_transport()   # workers are idle post-run
                try:
                    bus = self.state.bus(bus_id)
                    for uid in list(bus.test_set.keys()):
                        present = False
                        try:
                            echoed = MotorClient(transport, uid).ping(bytes(range(10)))
                            present = bytes(echoed) == bytes(range(10))
                        except (RS485Timeout, FatalError, Exception):
                            present = False
                        self.db.insert_phase_data(uid, LED_PHASE,
                                                  observation={"leds_lit": present})
                        if not present:
                            with missing_lock:
                                missing.append(uid_hex(uid))
                finally:
                    if not transport.is_simulated:
                        transport.close()
            return _job

        for bus_id in config.BUS_IDS:
            self.workers[bus_id]._jobs.put(make_job(bus_id))
        for bus_id in config.BUS_IDS:
            self.workers[bus_id].wait_idle()

        def _set(state):
            state.led_prompt = {"pending": False, "checked_removed": True,
                                "missing": sorted(missing)}
        self.state.update(_set)
        self._grade_led()
        return sorted(missing)

    def _clear_led_prompt(self) -> None:
        self.state.update(lambda s: setattr(s, "led_prompt", None))

    def _grade_led(self) -> None:
        """The LED (Phase 15) observation has just been recorded for every
        test-set motor — evaluate that phase per motor and colour the cell
        green/red right away (the LED test has no Stage-A ``store`` step)."""
        from . import evaluation
        for bus in self.state.buses.values():
            for uid in list(bus.test_set.keys()):
                try:
                    evaluation.evaluate_motor(self.db, self.settings, uid,
                                              [LED_PHASE])
                except Exception:
                    pass
        def _do(_):
            for bus in self.state.buses.values():
                for uid in bus.test_set:
                    bus.set_phase(uid, LED_PHASE,
                                  phase_grid_from_db(self.db, uid)[LED_PHASE])
        self.state.update(_do)

    # -- Stage-B evaluation -------------------------------------------------
    def evaluate(self, scope: str = "all") -> Dict[str, Any]:
        """Run Evaluation (Tab 2) over the motors picked by the Tab-2 Scope
        selector — ``"test_set"`` (only currently-detected motors) or ``"all"``
        (every motor in the database) — then refresh the live rack grid."""
        from . import evaluation
        if scope == "test_set":
            ids = list(dict.fromkeys(
                uid for bus in self.state.buses.values() for uid in bus.test_set))
        else:
            ids = self.db.all_motor_ids()
        summary = evaluation.evaluate_motors(self.db, self.settings, ids)
        summary["scope"] = scope
        self.refresh_grid_from_db()
        return summary

    # -- grid refresh (after Stage-B evaluation) ----------------------------
    def refresh_grid_from_db(self) -> None:
        """Recompute every test-set motor's phase grid from the database and
        push it to the browser.  Called after an evaluation run so the cells
        flip to green/red."""
        def _do(_):
            for bus in self.state.buses.values():
                for uid in list(bus.test_set.keys()):
                    bus.phase_status[uid] = phase_grid_from_db(self.db, uid)
        self.state.update(_do)
