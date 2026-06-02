"""New-vs-known colour coding must be decided once per session and stay stable
across detection passes (the operator runs several passes to beat collisions)."""

import os
import tempfile

from backend.database import Database
from backend.state import BusState
from backend import detection


def _db():
    return Database(os.path.join(tempfile.mkdtemp(), "t.sqlite3"))


def test_in_db_motor_stays_orange_across_passes():
    db = _db()
    bus = BusState("A")
    uid = 0x55AA
    db.record_detection(uid)        # pre-existing in DB (a prior session)

    detection.classify_and_add(db, bus, [[uid, 88]])     # pass 1
    assert bus.test_set[uid]["color"] == detection.COLOR_ORANGE
    detection.classify_and_add(db, bus, [[uid, 88]])     # pass 2 (the bug case)
    assert bus.test_set[uid]["color"] == detection.COLOR_ORANGE
    detection.classify_and_add(db, bus, [[uid, 88]])     # pass 3
    assert bus.test_set[uid]["color"] == detection.COLOR_ORANGE


def test_brand_new_motor_stays_green_across_passes():
    db = _db()
    bus = BusState("A")
    uid = 0x66BB                     # not in DB yet

    detection.classify_and_add(db, bus, [[uid, 88]])
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN
    assert db.motor_in_db(uid)       # green motor was recorded to the DB
    detection.classify_and_add(db, bus, [[uid, 88]])
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN


def test_cleared_then_redetected_is_orange():
    db = _db()
    bus = BusState("A")
    uid = 0x77CC
    detection.classify_and_add(db, bus, [[uid, 88]])     # green, new
    assert bus.test_set[uid]["color"] == detection.COLOR_GREEN
    bus.clear_set()                                      # operator clicks Clear
    detection.classify_and_add(db, bus, [[uid, 88]])     # now in DB, not in set
    assert bus.test_set[uid]["color"] == detection.COLOR_ORANGE
