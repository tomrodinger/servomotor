"""A thermal run that stops well short of the configured duration must FAIL
(the motor stopped mid-run — something is wrong and must be investigated),
not be mislabeled as a slope problem or pass."""

import os
import tempfile

from backend.database import Database
from backend.settings import Settings
from backend import blobs, evaluation as ev


def _setup(duration_s=180.0):
    d = tempfile.mkdtemp()
    db = Database(os.path.join(d, "t.sqlite3"))
    settings = Settings(os.path.join(d, "s.json"))
    settings.set_phase_param(11, "duration_s", duration_s)
    # keep the realistic criteria; a full run below is built to satisfy them
    return db, settings


def _series(n):
    # gentle linear rise: 25 C + 0.05 C/s, one sample per second 0..n-1
    return [(float(i), 25.0 + 0.05 * i) for i in range(n)]


def test_incomplete_thermal_run_fails():
    db, settings = _setup(duration_s=180.0)
    uid = 0xABCDEF01
    db.record_detection(uid)
    # only 110 s of a 180 s run (the C94F... case)
    db.insert_phase_data(uid, 11,
                         raw_blob=blobs.pack_temperature_series(_series(110)),
                         observation={"baseline": 25})
    metrics, result, failing = ev.evaluate_phase(db, settings, uid, 11)
    assert result == ev.FAIL
    assert failing == "incomplete_run"
    assert metrics["run_duration"] < 180

    # and it must count as an overall failure for the motor
    ev.evaluate_motor(db, settings, uid, [11])
    assert ev.overall_result(db, settings, uid)["result"] == ev.FAIL


def test_complete_thermal_run_not_flagged_incomplete():
    db, settings = _setup(duration_s=180.0)
    uid = 0xABCDEF02
    db.record_detection(uid)
    db.insert_phase_data(uid, 11,
                         raw_blob=blobs.pack_temperature_series(_series(180)),
                         observation={"baseline": 25})
    metrics, result, failing = ev.evaluate_phase(db, settings, uid, 11)
    assert failing != "incomplete_run"
    assert result == ev.PASS
