"""Run-scope selection, including the new 'incomplete or failure' scope."""

import os
import tempfile

from backend.database import Database
from backend.settings import Settings
from backend.state import AppState
from backend.bus_worker import BusWorker


def _setup():
    d = tempfile.mkdtemp()
    db = Database(os.path.join(d, "t.sqlite3"))
    settings = Settings(os.path.join(d, "s.json"))
    state = AppState()
    worker = BusWorker("A", db, settings, state, lambda b, p: None)
    # consider only Phase 14 so the test is about eval state, not other phases
    for ph in list(settings.enabled_phase_numbers()):
        if ph != 14:
            settings.set_phase_enabled(ph, False)
    return db, settings, state, worker


def test_incomplete_or_failed_scope():
    db, settings, state, worker = _setup()
    good, bad = 0x1, 0x2
    for u in (good, bad):
        db.record_detection(u)
        state.bus("A").add_to_set(u, 88, "green")
        db.insert_phase_data(u, 14, observation={"alias": "X"})   # both have data
    db.insert_phase_eval(good, 14, criteria_version=1,
                         derived_metrics={"alias": "X"}, result="pass")
    db.insert_phase_eval(bad, 14, criteria_version=1,
                         derived_metrics={"alias": None}, result="missing",
                         failing_metric="alias")

    settings.set_run_scope("all")
    assert set(worker._motors_for_run()) == {good, bad}

    settings.set_run_scope("incomplete")        # both have data -> nothing
    assert worker._motors_for_run() == []

    settings.set_run_scope("incomplete_or_failed")
    assert worker._motors_for_run() == [bad]    # only the failing motor

    # clearing the failure removes it from the scope
    db.set_cleared(bad, 14, "tester", "fixed by hand")
    assert worker._motors_for_run() == []


def test_incomplete_scope_includes_missing_data():
    db, settings, state, worker = _setup()
    uid = 0x3
    db.record_detection(uid)
    state.bus("A").add_to_set(uid, 88, "green")
    # no phase_data for phase 14 -> incomplete
    settings.set_run_scope("incomplete")
    assert worker._motors_for_run() == [uid]
    settings.set_run_scope("incomplete_or_failed")
    assert worker._motors_for_run() == [uid]
