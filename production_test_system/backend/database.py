"""SQLite (WAL) storage — the authoritative record of all collected data.

Three tables, keyed entirely on the 8-byte hardware unique ID (no slot is ever
recorded — the rack grid is a live view only):

* ``motors``      — one row per device; identity + calibration flag + first/last
                    seen.  ``first_detected`` is written once and never changed.
* ``phase_data``  — Stage A: *all* raw data collected on the hardware.  Large
                    arrays (Phase 6/7 streams, Phase 8 waveforms, temperature
                    and PID series) go in ``raw_blob`` as binary; small
                    scalars/events go in ``scalar``/``observation``.  Never
                    overwritten — re-collecting appends new rows.
* ``phase_eval``  — Stage B: derived metrics + computed pass/fail.  Re-running
                    evaluation appends fresh rows; the *latest* row per
                    (motor, phase) is authoritative.

WAL mode lets the three bus-worker threads and the API read/write concurrently.
A single connection (``check_same_thread=False``) guarded by a lock keeps it
simple and correct for this local, single-process app.
"""

from __future__ import annotations

import json
import sqlite3
import threading
import time
from typing import Any, Dict, List, Optional

from . import config


def _u(unique_id: int) -> str:
    """Store unique IDs as 16-char uppercase hex TEXT (SQLite ints are signed 64-bit)."""
    return "%016X" % (unique_id & 0xFFFFFFFFFFFFFFFF)


def uid_hex(unique_id: int) -> str:
    return _u(unique_id)


def uid_from_hex(text: str) -> int:
    return int(text, 16)


class Database:
    def __init__(self, path: Optional[str] = None):
        config.ensure_data_dirs()
        self.path = path or config.DB_PATH
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(self.path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._conn.execute("PRAGMA foreign_keys=ON")
        self._init_schema()

    def _init_schema(self) -> None:
        with self._lock:
            self._conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS motors (
                    unique_id        TEXT PRIMARY KEY,
                    product_type     TEXT,
                    hw_version       TEXT,
                    scc              INTEGER,
                    calibration_done INTEGER DEFAULT 0,
                    first_detected   REAL,
                    last_seen        REAL
                );

                CREATE TABLE IF NOT EXISTS phase_data (
                    data_id          INTEGER PRIMARY KEY AUTOINCREMENT,
                    unique_id        TEXT NOT NULL REFERENCES motors(unique_id),
                    phase            INTEGER NOT NULL,
                    collected_at     REAL NOT NULL,
                    firmware_version TEXT,
                    raw_blob         BLOB,
                    scalar           REAL,
                    observation      TEXT,        -- JSON
                    detail           TEXT
                );
                CREATE INDEX IF NOT EXISTS idx_phase_data_motor_phase
                    ON phase_data(unique_id, phase, collected_at);

                CREATE TABLE IF NOT EXISTS phase_eval (
                    eval_id          INTEGER PRIMARY KEY AUTOINCREMENT,
                    unique_id        TEXT NOT NULL REFERENCES motors(unique_id),
                    phase            INTEGER NOT NULL,
                    evaluated_at     REAL NOT NULL,
                    criteria_version INTEGER,
                    derived_metrics  TEXT,        -- JSON
                    result           TEXT,        -- 'pass' | 'fail' | 'missing'
                    failing_metric   TEXT,
                    cleared          INTEGER DEFAULT 0,
                    cleared_by       TEXT,
                    cleared_note     TEXT
                );
                CREATE INDEX IF NOT EXISTS idx_phase_eval_motor_phase
                    ON phase_eval(unique_id, phase, evaluated_at);
                """
            )
            self._conn.commit()

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    # -- motors -------------------------------------------------------------
    def record_detection(self, unique_id: int) -> bool:
        """Insert a motor on first detection (write-once first_detected).

        Returns True if this is a brand-new unique ID (not previously in the DB).
        Always refreshes ``last_seen``.
        """
        now = time.time()
        key = _u(unique_id)
        with self._lock:
            cur = self._conn.execute("SELECT 1 FROM motors WHERE unique_id=?", (key,))
            is_new = cur.fetchone() is None
            if is_new:
                self._conn.execute(
                    "INSERT INTO motors(unique_id, first_detected, last_seen) "
                    "VALUES (?,?,?)", (key, now, now))
            else:
                self._conn.execute(
                    "UPDATE motors SET last_seen=? WHERE unique_id=?", (now, key))
            self._conn.commit()
            return is_new

    def motor_in_db(self, unique_id: int) -> bool:
        with self._lock:
            cur = self._conn.execute("SELECT 1 FROM motors WHERE unique_id=?",
                                     (_u(unique_id),))
            return cur.fetchone() is not None

    def has_any_test_data(self, unique_id: int) -> bool:
        """True if real test work (collected phase data or a non-``missing``
        evaluation) has been recorded for this motor.  Plain detection does NOT
        count — a motor that has only ever been detected returns False.

        A ``phase_eval`` row whose ``result`` is ``'missing'`` also does NOT
        count: the auto-evaluation pass writes such a row for every *enabled*
        phase even on a motor that has never been tested (it had no data to
        evaluate), so counting it would falsely mark fresh motors as "tested"."""
        key = _u(unique_id)
        with self._lock:
            cur = self._conn.execute(
                "SELECT 1 FROM phase_data WHERE unique_id=? LIMIT 1", (key,))
            if cur.fetchone() is not None:
                return True
            cur = self._conn.execute(
                "SELECT 1 FROM phase_eval WHERE unique_id=? AND result!='missing' "
                "LIMIT 1", (key,))
            return cur.fetchone() is not None

    def update_identity(self, unique_id: int, product_type: str,
                        hw_version: str, scc: int) -> None:
        with self._lock:
            self._conn.execute(
                "UPDATE motors SET product_type=?, hw_version=?, scc=? WHERE unique_id=?",
                (product_type, hw_version, int(scc), _u(unique_id)))
            self._conn.commit()

    def set_calibration_done(self, unique_id: int, done: bool) -> None:
        with self._lock:
            self._conn.execute("UPDATE motors SET calibration_done=? WHERE unique_id=?",
                               (1 if done else 0, _u(unique_id)))
            self._conn.commit()

    def get_calibration_done(self, unique_id: int) -> bool:
        with self._lock:
            cur = self._conn.execute(
                "SELECT calibration_done FROM motors WHERE unique_id=?", (_u(unique_id),))
            row = cur.fetchone()
            return bool(row["calibration_done"]) if row else False

    def get_motor(self, unique_id: int) -> Optional[Dict[str, Any]]:
        with self._lock:
            cur = self._conn.execute("SELECT * FROM motors WHERE unique_id=?",
                                     (_u(unique_id),))
            row = cur.fetchone()
            return dict(row) if row else None

    def all_motor_ids(self) -> List[int]:
        with self._lock:
            cur = self._conn.execute("SELECT unique_id FROM motors ORDER BY first_detected")
            return [uid_from_hex(r["unique_id"]) for r in cur.fetchall()]

    def all_motors(self) -> List[Dict[str, Any]]:
        with self._lock:
            cur = self._conn.execute("SELECT * FROM motors ORDER BY first_detected")
            return [dict(r) for r in cur.fetchall()]

    # -- phase_data (Stage A) ----------------------------------------------
    def insert_phase_data(self, unique_id: int, phase: int, *,
                          firmware_version: Optional[str] = None,
                          raw_blob: Optional[bytes] = None,
                          scalar: Optional[float] = None,
                          observation: Optional[Dict[str, Any]] = None,
                          detail: Optional[str] = None) -> int:
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO phase_data(unique_id, phase, collected_at, "
                "firmware_version, raw_blob, scalar, observation, detail) "
                "VALUES (?,?,?,?,?,?,?,?)",
                (_u(unique_id), int(phase), time.time(), firmware_version,
                 raw_blob, scalar,
                 json.dumps(observation) if observation is not None else None,
                 detail))
            self._conn.commit()
            return int(cur.lastrowid)

    def latest_phase_data(self, unique_id: int, phase: int) -> Optional[Dict[str, Any]]:
        with self._lock:
            cur = self._conn.execute(
                "SELECT * FROM phase_data WHERE unique_id=? AND phase=? "
                "ORDER BY collected_at DESC, data_id DESC LIMIT 1",
                (_u(unique_id), int(phase)))
            row = cur.fetchone()
            return self._decode_data_row(row) if row else None

    def has_phase_data(self, unique_id: int, phase: int) -> bool:
        with self._lock:
            cur = self._conn.execute(
                "SELECT 1 FROM phase_data WHERE unique_id=? AND phase=? LIMIT 1",
                (_u(unique_id), int(phase)))
            return cur.fetchone() is not None

    def latest_phase_data_for_phase(self, phase: int) -> List[Dict[str, Any]]:
        """Latest phase_data row per motor for a given phase (across all motors)."""
        with self._lock:
            cur = self._conn.execute(
                "SELECT pd.* FROM phase_data pd "
                "JOIN (SELECT unique_id, MAX(collected_at) AS mc, MAX(data_id) AS mid "
                "      FROM phase_data WHERE phase=? GROUP BY unique_id) latest "
                "ON pd.unique_id=latest.unique_id AND pd.data_id=latest.mid "
                "WHERE pd.phase=?", (int(phase), int(phase)))
            return [self._decode_data_row(r) for r in cur.fetchall()]

    @staticmethod
    def _decode_data_row(row: sqlite3.Row) -> Dict[str, Any]:
        d = dict(row)
        if d.get("observation"):
            try:
                d["observation"] = json.loads(d["observation"])
            except (json.JSONDecodeError, TypeError):
                pass
        return d

    # -- phase_eval (Stage B) ----------------------------------------------
    def insert_phase_eval(self, unique_id: int, phase: int, *,
                          criteria_version: int,
                          derived_metrics: Optional[Dict[str, Any]],
                          result: str,
                          failing_metric: Optional[str] = None) -> int:
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO phase_eval(unique_id, phase, evaluated_at, "
                "criteria_version, derived_metrics, result, failing_metric) "
                "VALUES (?,?,?,?,?,?,?)",
                (_u(unique_id), int(phase), time.time(), int(criteria_version),
                 json.dumps(derived_metrics) if derived_metrics is not None else None,
                 result, failing_metric))
            self._conn.commit()
            return int(cur.lastrowid)

    def latest_phase_eval(self, unique_id: int, phase: int) -> Optional[Dict[str, Any]]:
        with self._lock:
            cur = self._conn.execute(
                "SELECT * FROM phase_eval WHERE unique_id=? AND phase=? "
                "ORDER BY evaluated_at DESC, eval_id DESC LIMIT 1",
                (_u(unique_id), int(phase)))
            row = cur.fetchone()
            return self._decode_eval_row(row) if row else None

    def latest_evals_for_motor(self, unique_id: int) -> Dict[int, Dict[str, Any]]:
        """Latest eval row per phase for one motor."""
        with self._lock:
            cur = self._conn.execute(
                "SELECT pe.* FROM phase_eval pe "
                "JOIN (SELECT phase, MAX(eval_id) AS mid FROM phase_eval "
                "      WHERE unique_id=? GROUP BY phase) latest "
                "ON pe.phase=latest.phase AND pe.eval_id=latest.mid "
                "WHERE pe.unique_id=?", (_u(unique_id), _u(unique_id)))
            return {r["phase"]: self._decode_eval_row(r) for r in cur.fetchall()}

    def set_cleared(self, unique_id: int, phase: int, cleared_by: str,
                    note: str) -> bool:
        """Mark the latest eval row for (motor, phase) as cleared (operator override)."""
        with self._lock:
            row = self.latest_phase_eval(unique_id, phase)
            if not row:
                return False
            self._conn.execute(
                "UPDATE phase_eval SET cleared=1, cleared_by=?, cleared_note=? "
                "WHERE eval_id=?", (cleared_by, note, row["eval_id"]))
            self._conn.commit()
            return True

    @staticmethod
    def _decode_eval_row(row: sqlite3.Row) -> Dict[str, Any]:
        d = dict(row)
        if d.get("derived_metrics"):
            try:
                d["derived_metrics"] = json.loads(d["derived_metrics"])
            except (json.JSONDecodeError, TypeError):
                pass
        d["cleared"] = bool(d.get("cleared"))
        return d
