"""One worker thread per RS485 bus.

All test execution runs in the backend, in these worker threads — so closing or
reloading the browser never affects a running test.  A worker owns a job queue;
the API thread submits jobs (detection passes, a test run, a broadcast reset, a
single re-run phase) and the worker executes them sequentially on its bus.

Pause/Cancel are events the worker honours at safe points (between motors and
during interruptible sleeps), so a run can be paused or aborted without leaving
the bus in mid-transaction.  Closing/reloading the browser does nothing to a
run; Start/Stop/Pause/Cancel are backend commands, not page loops.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable, List, Optional

from . import config, detection, phases
from .database import Database
from .phases.base import CollectionContext, CancelledError
from .settings import Settings
from .state import AppState
from .transport import Transport
from .transport_factory import TransportFactory


class BusWorker:
    def __init__(self, bus_id: str, db: Database, settings: Settings,
                 state: AppState, transport_factory: TransportFactory,
                 log_fn: Optional[Callable[[str], None]] = None):
        self.bus_id = bus_id
        self.db = db
        self.settings = settings
        self.state = state
        self._factory = transport_factory
        self._log_fn = log_fn
        self._jobs: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._pause = threading.Event()
        self._cancel = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="bus-%s" % bus_id,
                                        daemon=True)
        self._running = False
        self._fw_versions = {}

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        self._thread.start()

    def log(self, message: str) -> None:
        if self._log_fn:
            self._log_fn(message)

    def _loop(self) -> None:
        while True:
            job = self._jobs.get()
            if job is None:
                break
            try:
                job()
            except CancelledError:
                self.log("[Bus %s] operation cancelled" % self.bus_id)
            except Exception as exc:    # never let a worker thread die
                self.log("[Bus %s] error: %s" % (self.bus_id, exc))
            finally:
                self._cancel.clear()
                self._jobs.task_done()

    # -- run controls (called from API thread) ------------------------------
    def pause(self) -> None:
        self._pause.set()
        self._set_bus_flag(paused=True)

    def resume(self) -> None:
        self._pause.clear()
        self._set_bus_flag(paused=False)

    def cancel(self) -> None:
        self._cancel.set()

    def _set_bus_flag(self, *, running: Optional[bool] = None,
                      paused: Optional[bool] = None,
                      current_phase: Optional[int] = -1,
                      message: Optional[str] = None) -> None:
        def _do(_):
            bus = self.state.bus(self.bus_id)
            if running is not None:
                bus.running = running
            if paused is not None:
                bus.paused = paused
            if current_phase != -1:
                bus.current_phase = current_phase
            if message is not None:
                bus.status_message = message
        self.state.update(_do)

    # -- transport helper ---------------------------------------------------
    def _open_transport(self) -> Transport:
        port = self.settings.get_serial_ports().get(self.bus_id)
        return self._factory(self.bus_id, port)

    # -- detection ----------------------------------------------------------
    def submit_detect(self) -> None:
        """Queue one detection pass (increments the pending counter)."""
        def _do(_):
            self.state.bus(self.bus_id).pending_detections += 1
        self.state.update(_do)
        self._jobs.put(self._run_detect_passes)

    def _run_detect_passes(self) -> None:
        bus = self.state.bus(self.bus_id)
        transport = self._open_transport()
        self._set_bus_flag(message="detecting")
        try:
            while bus.pending_detections > 0 and not self._cancel.is_set():
                self.state.update(lambda _: setattr(bus, "detecting", True))
                results = detection.run_detect_pass(
                    transport, reset_before=not transport.is_simulated)
                def _apply(_):
                    detection.classify_and_add(self.db, bus, results)
                    bus.pending_detections = max(0, bus.pending_detections - 1)
                self.state.update(_apply)
        finally:
            self.state.update(lambda _: setattr(bus, "detecting", False))
            self._set_bus_flag(message="")
            if not transport.is_simulated:
                transport.close()

    # -- runs ---------------------------------------------------------------
    def submit_run(self) -> None:
        self._jobs.put(self._run_sequence)

    def submit_system_reset(self) -> None:
        self._jobs.put(self._broadcast_reset_job)

    def submit_rerun_phase(self, phase_number: int) -> None:
        self._jobs.put(lambda: self._run_sequence(only_phase=phase_number))

    def _broadcast_reset_job(self) -> None:
        transport = self._open_transport()
        try:
            transport.transact(255, "System reset")
            time.sleep(config.BOOTLOADER_EXIT_DELAY_S)
        finally:
            if not transport.is_simulated:
                transport.close()

    def _motors_for_run(self) -> List[int]:
        bus = self.state.bus(self.bus_id)
        motors = list(bus.test_set.keys())
        scope = self.settings.run_scope
        if scope in ("incomplete", "incomplete_or_failed"):
            enabled = self.settings.enabled_phase_numbers()
            include_failures = (scope == "incomplete_or_failed")
            motors = [uid for uid in motors
                      if self._needs_rerun(uid, enabled, include_failures)]
        return motors

    def _needs_rerun(self, uid: int, enabled: List[int],
                     include_failures: bool) -> bool:
        """True if any enabled phase is missing collected data (incomplete) or,
        when ``include_failures``, has a failing/missing latest eval that has not
        been cleared."""
        for ph in enabled:
            if not self.db.has_phase_data(uid, ph):
                return True
            if include_failures:
                ev = self.db.latest_phase_eval(uid, ph)
                if ev and not ev.get("cleared") and ev["result"] in ("fail", "missing"):
                    return True
        return False

    def _run_sequence(self, only_phase: Optional[int] = None) -> None:
        motors = self._motors_for_run()
        if not motors:
            self.log("[Bus %s] no motors to run" % self.bus_id)
            return
        transport = self._open_transport()
        self._fw_versions = {}
        ctx = CollectionContext(
            bus_id=self.bus_id, transport=transport, db=self.db,
            settings=self.settings, state=self.state, motors=motors,
            pause_event=self._pause, cancel_event=self._cancel,
            fw_versions=self._fw_versions, log_fn=self._log_fn)
        self._set_bus_flag(running=True, message="running")
        try:
            phase_numbers = ([only_phase] if only_phase is not None
                             else self.settings.enabled_phase_numbers())
            for phase_num in phase_numbers:
                if self._cancel.is_set():
                    break
                ctx.wait_if_paused()
                # System reset between phases (+ 1 s bootloader-exit delay) so a
                # fault in one phase never poisons the next.
                try:
                    transport.transact(255, "System reset")
                    ctx.sleep(config.BOOTLOADER_EXIT_DELAY_S)
                    transport.flush_input()   # drain any late stragglers before reads
                except CancelledError:
                    break
                except Exception:
                    pass
                ctx.phase_number = phase_num
                ctx.params = self.settings.phase_params(phase_num)
                self._set_bus_flag(current_phase=phase_num,
                                   message="running phase %d" % phase_num)
                collector = phases.get_collector(phase_num)
                try:
                    collector.collect(ctx)
                except CancelledError:
                    break
                except Exception as exc:
                    self.log("[Bus %s] phase %d error (continuing): %s"
                             % (self.bus_id, phase_num, exc))
        finally:
            self._set_bus_flag(running=False, current_phase=None, message="")
            if not transport.is_simulated:
                transport.close()

    def wait_idle(self, timeout: Optional[float] = None) -> None:
        """Block until the job queue drains (used by tests)."""
        self._jobs.join()
