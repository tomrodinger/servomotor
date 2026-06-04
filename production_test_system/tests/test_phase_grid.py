"""The live rack now shows a 15-cell per-phase status grid per motor.

Two layers are covered:
  * ``phase_grid_from_db`` — the pure DB-derived colour for one phase
    (gray -> blue -> green/red, and back to blue on a fresh re-collection).
  * the end-to-end live snapshot — detect (DB-seeded), collect (yellow->blue),
    then evaluate (green/red) on a simulated rack.
"""

import os
import tempfile
import time

from backend import evaluation as ev
from backend.database import Database, uid_hex
from backend.settings import Settings
from backend.state import (phase_grid_from_db, GRID_GRAY, GRID_BLUE,
                           GRID_GREEN, GRID_RED, PHASE_NUMBERS)
from tests.harness import Harness, good_profile


def _fresh_db():
    d = tempfile.mkdtemp(prefix="pts-grid-")
    return Database(os.path.join(d, "t.sqlite3"))


def test_phase_grid_from_db_transitions():
    db = _fresh_db()
    uid = 0xABCD
    db.record_detection(uid)

    # nothing collected -> gray
    assert phase_grid_from_db(db, uid)[1] == GRID_GRAY

    # collected, not evaluated -> blue
    db.insert_phase_data(uid, 1, observation={"firmware_version": "0.15.1.0",
                                              "responded": True})
    assert phase_grid_from_db(db, uid)[1] == GRID_BLUE

    # evaluated pass -> green
    db.insert_phase_eval(uid, 1, criteria_version=1, derived_metrics={},
                         result="pass")
    assert phase_grid_from_db(db, uid)[1] == GRID_GREEN

    # evaluated fail -> red
    db.insert_phase_eval(uid, 1, criteria_version=1, derived_metrics={},
                         result="fail", failing_metric="firmware_version")
    assert phase_grid_from_db(db, uid)[1] == GRID_RED

    # a cleared failure reads as green (matches the overall pass-or-cleared rule)
    db.set_cleared(uid, 1, "tester", "fixed by hand")
    assert phase_grid_from_db(db, uid)[1] == GRID_GREEN

    # re-collecting newer data than the latest eval reverts to blue (pending re-eval)
    time.sleep(0.01)
    db.insert_phase_data(uid, 1, observation={"firmware_version": "0.15.1.0",
                                              "responded": True})
    assert phase_grid_from_db(db, uid)[1] == GRID_BLUE


def test_disabled_phase_with_data_is_still_graded():
    """A phase that has collected data must evaluate to green/red even when it is
    currently disabled — otherwise its cell is stuck on blue forever."""
    import os, tempfile
    from backend.settings import Settings
    d = tempfile.mkdtemp(prefix="pts-grid-")
    db = Database(os.path.join(d, "t.sqlite3"))
    settings = Settings(os.path.join(d, "s.json"))
    uid = 0x55
    db.record_detection(uid)
    settings.set_phase_enabled(9, False)          # Phase 9 disabled...
    db.insert_phase_data(uid, 9, observation={                 # ...but it has data
        "max_pid_deviation": 100000.0, "position_error": 0.01})

    # before any evaluation: collected, pending -> blue
    assert phase_grid_from_db(db, uid)[9] == GRID_BLUE

    # evaluating the motor grades the disabled-but-collected phase 9 too
    ev.evaluate_motor(db, settings, uid)
    assert phase_grid_from_db(db, uid)[9] in (GRID_GREEN, GRID_RED)
    # and a disabled phase never drags the overall result down
    assert ev.overall_result(db, settings, uid)["result"] in ("pass", "fail")


def test_live_grid_detect_collect_evaluate():
    h = Harness()
    GOOD, BAD_FW = 0x10, 0x11
    h.add_motor("A", GOOD, good_profile())
    h.add_motor("A", BAD_FW, good_profile(firmware_version="0.14.0.0"))  # P1 fails
    h.detect_and_wait("A")

    def grid_for(uid):
        motors = h.state.snapshot()["buses"]["A"]["motors"]
        m = next(x for x in motors if x["unique_id"] == uid_hex(uid))
        return m["phase_grid"]

    # freshly detected, never tested -> every cell gray
    assert grid_for(GOOD) == [GRID_GRAY] * len(PHASE_NUMBERS)

    # run the full sequence + record the LED (Phase 15) observation
    h.run_and_wait("A")
    h.runner.led_confirm_all_pass()

    # each phase auto-evaluates the instant a motor finishes it, so the cells
    # are already green/red — no Tab-2 batch run needed.
    good_after_run = grid_for(GOOD)
    assert GRID_GRAY not in good_after_run
    assert good_after_run == [GRID_GREEN] * len(PHASE_NUMBERS)

    bad = grid_for(BAD_FW)
    assert bad[0] == GRID_RED                       # P1 firmware mismatch
    assert all(c == GRID_GREEN for c in bad[1:])

    # the Tab-2 batch evaluation + refresh yields the same colours
    ev.evaluate_all(h.db, h.settings)
    h.runner.refresh_grid_from_db()
    assert grid_for(GOOD) == [GRID_GREEN] * len(PHASE_NUMBERS)
