import os
import tempfile

from backend.database import Database


def _db():
    return Database(os.path.join(tempfile.mkdtemp(), "t.sqlite3"))


def test_detection_write_once():
    db = _db()
    uid = 0x1122334455667788
    assert db.record_detection(uid) is True          # new
    first = db.get_motor(uid)["first_detected"]
    assert db.record_detection(uid) is False          # known
    assert db.get_motor(uid)["first_detected"] == first   # never overwritten


def test_blob_roundtrip_and_latest():
    db = _db()
    uid = 0x1
    db.record_detection(uid)
    blob = bytes(range(256)) * 4
    db.insert_phase_data(uid, 8, raw_blob=blob)
    assert db.latest_phase_data(uid, 8)["raw_blob"] == blob
    # latest wins
    db.insert_phase_data(uid, 3, scalar=24.0, observation={"v": 1})
    db.insert_phase_data(uid, 3, scalar=23.0, observation={"v": 2})
    assert db.latest_phase_data(uid, 3)["observation"]["v"] == 2


def test_eval_latest_and_clear():
    db = _db()
    uid = 0x2
    db.record_detection(uid)
    db.insert_phase_eval(uid, 8, criteria_version=1, derived_metrics={"x": 1}, result="fail",
                         failing_metric="span")
    assert db.latest_phase_eval(uid, 8)["result"] == "fail"
    assert db.set_cleared(uid, 8, "alice", "rework") is True
    assert db.latest_phase_eval(uid, 8)["cleared"] is True


def test_calibration_flag_and_has_data():
    db = _db()
    uid = 0x3
    db.record_detection(uid)
    assert db.get_calibration_done(uid) is False
    db.set_calibration_done(uid, True)
    assert db.get_calibration_done(uid) is True
    assert db.has_phase_data(uid, 5) is False
    db.insert_phase_data(uid, 5, observation={"status_zero": True})
    assert db.has_phase_data(uid, 5) is True
