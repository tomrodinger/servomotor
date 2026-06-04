"""Coordinator over the three bus workers.

Owns one :class:`BusWorker` per bus and exposes the run controls the UI drives:
detect, clear, Start All / Start Section, Pause/Resume, Cancel, System Reset,
re-run phase, and the LED-test (Phase 15) human-confirmation reconciliation.
The workers do all the bus I/O on their own threads; the runner just routes
commands and performs the no-hardware LED bookkeeping.
"""

from __future__ import annotations

import threading
from typing import Callable, Dict, List, Optional

from . import config
from .bus_worker import BusWorker
from .database import Database
from .detection import DETECT_TIMEOUT
from .settings import Settings
from .state import AppState, GRID_BLUE
from .transport_factory import TransportFactory, serial_factory

LED_PHASE = 15


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

    def identify(self, unique_id: int) -> bool:
        """Flash a device's LEDs (cmd 41) so it can be located in the rack."""
        bus_id = self.find_bus(unique_id)
        if bus_id is None:
            return False
        worker = self.workers[bus_id]

        def _job():
            from .motor_client import MotorClient
            transport = worker._open_transport()
            try:
                MotorClient(transport, unique_id).identify()
            finally:
                if not transport.is_simulated:
                    transport.close()
        worker._jobs.put(_job)
        return True

    # -- LED test (Phase 15) reconciliation --------------------------------
    def led_confirm_all_pass(self) -> None:
        """Human answered Yes: mark the LED test passed for every motor."""
        for bus_id, bus in self.state.buses.items():
            for uid in list(bus.test_set.keys()):
                self.db.insert_phase_data(uid, LED_PHASE,
                                          observation={"leds_lit": True})
        self._clear_led_prompt()
        self._mark_grid_blue_for_test_sets()

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
        self._mark_grid_blue_for_test_sets()
        return sorted(missing)

    def _clear_led_prompt(self) -> None:
        self.state.update(lambda s: setattr(s, "led_prompt", None))

    def _mark_grid_blue_for_test_sets(self) -> None:
        def _do(_):
            for bus in self.state.buses.values():
                for uid in bus.test_set:
                    bus.grid_status[uid] = GRID_BLUE
        self.state.update(_do)
