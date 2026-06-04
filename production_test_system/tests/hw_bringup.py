"""Full real-hardware pass: phases 1-14 on all three buses with shortened
durations, against a SEPARATE test DB + settings (production config untouched).

Detection runs up to 6 passes per bus (the operator's usual practice) to catch
every motor.  Phase 5 calibration uses the staggered-start fix.  The broadcast
flash is skipped (already on 0.15.0.0) and Phase 15 (which locks motors until a
power cycle) is left out — run it deliberately from the UI.
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
EXPECTED = {"A": 47, "B": 48, "C": 48}
MAX_DETECT_PASSES = 6


def configure(settings: Settings) -> None:
    for bus, port in PORTS.items():
        settings.set_serial_port(bus, port)
    settings.set_phase_param(1, "flash_enabled", False)     # already on 0.15.0.0
    settings.set_phase_param(4, "ping_count", 50)
    settings.set_phase_param(5, "start_stagger_s", 0.5)     # the overflow fix
    # Phase 5 hold stays at the default 30 s (firmware calibration time).
    settings.set_phase_param(6, "spin_rotations", 0.5)
    settings.set_phase_param(6, "spin_velocity", 3.0)
    settings.set_phase_param(7, "n_steps", 40)
    settings.set_phase_param(7, "settle_ms", 5)
    settings.set_phase_param(11, "duration_s", 15.0)
    settings.set_phase_param(12, "duration_s", 30.0)
    settings.set_phase_param(12, "spin_interval_s", 5.0)
    settings.set_phase_param(13, "duration_s", 30.0)
    settings.set_phase_param(13, "move_duration_s", 2.0)
    settings.set_phase_enabled(15, False)                   # don't lock the rack here


def detect_all(runner, state):
    """Up to MAX_DETECT_PASSES passes per bus (in parallel), early-stopping a
    bus once its count stops growing."""
    counts = {b: 0 for b in PORTS}
    for p in range(MAX_DETECT_PASSES):
        active = [b for b in PORTS if not (counts[b] >= EXPECTED[b])]
        if not active:
            break
        for b in active:
            runner.detect(b)
        for b in active:
            runner.workers[b].wait_idle()
        new = {b: len(state.bus(b).test_set) for b in PORTS}
        print("  pass %d: %s" % (p + 1, new), flush=True)
        # stop a bus that didn't grow this pass (and has something)
        for b in active:
            if new[b] == counts[b] and new[b] > 0 and new[b] >= EXPECTED[b] - 1:
                pass
        counts = new
    return counts


def main():
    base = config.DATA_DIR
    db = Database(os.path.join(base, "hw_test2.sqlite3"))
    settings = Settings(os.path.join(base, "hw_test2_settings.json"))
    configure(settings)
    state = AppState()
    runner = Runner(db, settings, state, log_fn=lambda m: print(m, flush=True))

    print("=== Detecting (up to %d passes/bus) ===" % MAX_DETECT_PASSES, flush=True)
    counts = detect_all(runner, state)
    print("Detected:", counts, "expected", EXPECTED, flush=True)

    print("=== Full run (phases 1-14) on all buses ===", flush=True)
    t0 = time.time()
    runner.start_all()
    for b in PORTS:
        runner.workers[b].wait_idle()
    print("=== Run complete in %.0f s ===" % (time.time() - t0), flush=True)

    print("=== Evaluating ===", flush=True)
    print("Eval summary:", ev.evaluate_all(db, settings), flush=True)

    # coverage: how many motors have data per phase, and calibration rate
    ids = db.all_motor_ids()
    cal = sum(1 for u in ids if db.get_calibration_done(u))
    print("Motors: %d, calibration_done: %d" % (len(ids), cal), flush=True)
    for ph in settings.enabled_phase_numbers():
        n = sum(1 for u in ids if db.has_phase_data(u, ph))
        print("  phase %2d: data for %d/%d motors" % (ph, n, len(ids)), flush=True)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
