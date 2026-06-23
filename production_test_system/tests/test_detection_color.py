"""Test-set colour coding: a motor is green when it has only ever been detected
(no tests performed on it yet) and orange once it has test results recorded.  The
colour is decided once per session and stays stable across detection passes (the
operator runs several passes to beat collisions)."""

import os
import tempfile

from backend.database import Database
from backend.state import BusState
from backend import detection


def _db():
    return Database(os.path.join(tempfile.mkdtemp(), "t.sqlite3"))


def _give_test_result(db, uid):
    """Record a phase evaluation so the motor counts as 'tested'."""
    db.insert_phase_eval(uid, 1, criteria_version=1,
                         derived_metrics=None, result="PASS")


def test_tested_motor_stays_orange_across_passes():
    db = _db()
    bus = BusState("A")
    uid = 0x55AA
    db.record_detection(uid)        # detected in a prior session...
    _give_test_result(db, uid)      # ...and a phase ran on it

    detection.classify_and_add(db, bus, [[uid, 88]])     # pass 1
    assert bus.test_set[uid]["color"] == detection.COLOR_ORANGE
    detection.classify_and_add(db, bus, [[uid, 88]])     # pass 2
    assert bus.test_set[uid]["color"] == detection.COLOR_ORANGE
    detection.classify_and_add(db, bus, [[uid, 88]])     # pass 3
    assert bus.test_set[uid]["color"] == detection.COLOR_ORANGE


def test_detected_but_untested_motor_is_green():
    """A motor seen in a prior session but never tested must show green — this is
    the operator's case: re-running detection alone never turns a fresh motor orange."""
    db = _db()
    bus = BusState("A")
    uid = 0x99DD
    db.record_detection(uid)        # pre-existing detection row, but NO test data

    detection.classify_and_add(db, bus, [[uid, 88]])
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN
    detection.classify_and_add(db, bus, [[uid, 88]])     # another detection pass
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN


def test_brand_new_motor_stays_green_across_passes():
    db = _db()
    bus = BusState("A")
    uid = 0x66BB                     # not in DB yet

    detection.classify_and_add(db, bus, [[uid, 88]])
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN
    assert db.motor_in_db(uid)       # green motor was recorded to the DB
    detection.classify_and_add(db, bus, [[uid, 88]])
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN


def test_cleared_then_redetected_untested_is_green():
    db = _db()
    bus = BusState("A")
    uid = 0x77CC
    detection.classify_and_add(db, bus, [[uid, 88]])     # green, new
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN
    bus.clear_set()                                      # operator clicks Clear
    detection.classify_and_add(db, bus, [[uid, 88]])     # in DB but still untested
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN


def test_missing_only_eval_rows_stay_green():
    """The auto-evaluation pass writes a ``result='missing'`` eval row for every
    enabled phase even on a never-tested motor.  Those rows must NOT make the
    motor orange — only real collected data / a real (non-missing) verdict does.
    This is the operator's bug: a rack of fresh motors all showed orange because
    a prior session's evaluation had stamped 'missing' rows on every one."""
    db = _db()
    bus = BusState("A")
    uid = 0xABCD
    db.record_detection(uid)
    for ph in (1, 2, 3):             # evaluation ran but found nothing to evaluate
        db.insert_phase_eval(uid, ph, criteria_version=1,
                             derived_metrics=None, result="missing")

    assert not db.has_any_test_data(uid)
    detection.classify_and_add(db, bus, [[uid, 88]])
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN


def test_cleared_then_redetected_after_testing_is_orange():
    db = _db()
    bus = BusState("A")
    uid = 0x88EE
    detection.classify_and_add(db, bus, [[uid, 88]])     # green, new
    _give_test_result(db, uid)                           # a phase runs on it
    bus.clear_set()                                      # operator clicks Clear
    detection.classify_and_add(db, bus, [[uid, 88]])     # now has test results
    assert bus.test_set[uid]["color"] == detection.COLOR_ORANGE
