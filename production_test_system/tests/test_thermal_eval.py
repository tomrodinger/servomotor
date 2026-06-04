"""Phase 11 pass/fail: run-to-overtemperature logic.

PASS: reaches the overtemp cutoff hot enough, OR stays functional the whole run.
FAIL: any other fatal (e.g. a deviation trip from a driver cut-out / missing
thermal paste), an overtemp cutoff that isn't actually hot, or stopping mid-run
without a fatal.
"""

import os
import tempfile

from backend.database import Database
from backend.settings import Settings
from backend import blobs, evaluation as ev

OVERHEAT = 40
DEVIATION = 45


def _setup(max_time_s=1200.0):
    d = tempfile.mkdtemp()
    db = Database(os.path.join(d, "t.sqlite3"))
    settings = Settings(os.path.join(d, "s.json"))
    settings.set_phase_param(11, "max_time_s", max_time_s)
    return db, settings


def _store(db, uid, series, **obs):
    db.record_detection(uid)
    obs.setdefault("baseline", series[0][1] if series else 25)
    db.insert_phase_data(uid, 11,
                         raw_blob=blobs.pack_temperature_series(series) if series else None,
                         observation=obs)


def _ramp(n, start=25.0, slope=1.0):
    return [(float(i), start + slope * i) for i in range(n)]


def test_overtemp_hot_passes():
    db, settings = _setup()
    uid = 0x1
    _store(db, uid, _ramp(60, slope=1.0), fatal_code=OVERHEAT, max_temp=84,
           last_temp=84, ran_full=False)
    m, r, f = ev.evaluate_phase(db, settings, uid, 11)
    assert r == ev.PASS and m["outcome"] == "overtemp" and m["max_temperature"] == 84


def test_overtemp_but_not_hot_fails():
    db, settings = _setup()
    uid = 0x2
    _store(db, uid, _ramp(20), fatal_code=OVERHEAT, max_temp=60, last_temp=60,
           ran_full=False)
    m, r, f = ev.evaluate_phase(db, settings, uid, 11)
    assert r == ev.FAIL and f == "max_temperature"


def test_other_fatal_fails():
    db, settings = _setup()
    uid = 0x3
    _store(db, uid, _ramp(20), fatal_code=DEVIATION, max_temp=48, last_temp=48,
           ran_full=False)
    m, r, f = ev.evaluate_phase(db, settings, uid, 11)
    assert r == ev.FAIL and m["outcome"] == "other_fatal"


def test_functional_whole_time_passes():
    db, settings = _setup(max_time_s=120.0)
    uid = 0x4
    _store(db, uid, _ramp(120, slope=0.1), fatal_code=None, max_temp=37,
           last_temp=37, ran_full=True)
    m, r, f = ev.evaluate_phase(db, settings, uid, 11)
    assert r == ev.PASS and m["outcome"] == "functional"


def test_stopped_without_fatal_fails():
    db, settings = _setup(max_time_s=120.0)
    uid = 0x5
    _store(db, uid, _ramp(40), fatal_code=None, max_temp=65, last_temp=65,
           ran_full=False)   # stopped responding, no fatal recorded
    m, r, f = ev.evaluate_phase(db, settings, uid, 11)
    assert r == ev.FAIL and f == "incomplete_run" and m["outcome"] == "incomplete"
