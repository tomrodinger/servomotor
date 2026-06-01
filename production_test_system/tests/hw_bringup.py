"""First real-hardware bring-up pass.

Runs the full collection sequence (phases 1-14) on all three buses with
*shortened* durations, against a SEPARATE test DB + settings so production
config/defaults are untouched.  Phase 5's 30 s calibration hold is kept (it is a
firmware requirement, not an arbitrary wait); the broadcast firmware flash is
skipped (bench-pending); Phase 15 (which locks motors) is left out of this run
and tested separately on one bus.
"""
from __future__ import annotations

import os
import time

from backend import config
from backend.database import Database
from backend.settings import Settings
from backend.state import AppState
from backend.runner import Runner
from backend import evaluation as ev

PORTS = {"A": "/dev/cu.usbserial-110", "B": "/dev/cu.usbserial-1420",
         "C": "/dev/cu.usbserial-1430"}


def configure(settings: Settings) -> None:
    for bus, port in PORTS.items():
        settings.set_serial_port(bus, port)
    settings.set_phase_param(1, "flash_enabled", False)     # verify versions only
    settings.set_phase_param(4, "ping_count", 50)
    # Phase 5 hold stays at the default 30 s (firmware calibration time).
    settings.set_phase_param(6, "spin_rotations", 0.5)
    settings.set_phase_param(6, "spin_velocity", 3.0)
    settings.set_phase_param(7, "n_steps", 40)
    settings.set_phase_param(7, "settle_ms", 5)
    settings.set_phase_param(11, "duration_s", 20.0)
    settings.set_phase_param(12, "duration_s", 60.0)
    settings.set_phase_param(12, "spin_interval_s", 5.0)
    settings.set_phase_param(13, "duration_s", 60.0)
    settings.set_phase_param(13, "move_duration_s", 2.0)
    settings.set_phase_enabled(15, False)                   # don't lock the rack here


def main():
    base = config.DATA_DIR
    db = Database(os.path.join(base, "hw_test.sqlite3"))
    settings = Settings(os.path.join(base, "hw_test_settings.json"))
    configure(settings)
    state = AppState()
    logs = []
    runner = Runner(db, settings, state, log_fn=lambda m: (logs.append(m), print(m, flush=True)))

    print("=== Detecting on all buses ===", flush=True)
    for bus in PORTS:
        runner.detect(bus)
    for bus in PORTS:
        runner.workers[bus].wait_idle()
    for bus in PORTS:
        print("Bus %s: %d motors" % (bus, len(state.bus(bus).test_set)), flush=True)

    print("=== Starting full run (phases 1-14) on all buses ===", flush=True)
    t0 = time.time()
    runner.start_all()
    # wait until all workers idle
    deadline = t0 + 1800
    while time.time() < deadline:
        runner.workers["A"].wait_idle()
        runner.workers["B"].wait_idle()
        runner.workers["C"].wait_idle()
        break
    print("=== Run complete in %.0f s ===" % (time.time() - t0), flush=True)

    print("=== Evaluating ===", flush=True)
    summary = ev.evaluate_all(db, settings)
    print("Eval summary:", summary, flush=True)
    # per-phase data coverage for one motor per bus
    for bus in PORTS:
        ids = list(state.bus(bus).test_set.keys())
        if not ids:
            continue
        uid = ids[0]
        present = [ph for ph in settings.enabled_phase_numbers() if db.has_phase_data(uid, ph)]
        print("Bus %s sample motor %016X has data for phases %s" % (bus, uid, present), flush=True)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
