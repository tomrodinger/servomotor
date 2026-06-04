"""FastAPI application: REST API + WebSocket + static frontend.

All test execution lives in the backend bus workers (see ``bus_worker``); the
browser is only a view/control client, so closing or reloading it never affects
a running test.  On (re)connect the browser fetches the current state and a
WebSocket streams live updates pushed from the worker threads.

Set ``PTS_SIMULATE=1`` to run against a built-in simulated rack (no hardware).
"""

from __future__ import annotations

import asyncio
import os
import threading
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional, Set

from fastapi import FastAPI, Body, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles

from . import config, phase_defs, evaluation, png_generation
from .database import Database, uid_hex, uid_from_hex
from .settings import Settings
from .state import AppState
from .runner import Runner

config.ensure_data_dirs()

SIMULATE = os.environ.get("PTS_SIMULATE") == "1"

# -- singletons ---------------------------------------------------------------
LOGS: List[str] = []


def _log(msg: str) -> None:
    LOGS.append(msg)
    del LOGS[:-500]   # keep the last 500 lines


if SIMULATE:
    # Demo mode uses its own DB + settings file (and short durations) so it never
    # pollutes the real-hardware database or settings on the same machine.
    from .demo import (build_demo_rack, demo_transport_factory,
                       apply_fast_demo_settings)
    DB = Database(os.path.join(config.DATA_DIR, "demo.sqlite3"))
    SETTINGS = Settings(os.path.join(config.DATA_DIR, "demo_settings.json"))
    apply_fast_demo_settings(SETTINGS)
    STATE = AppState()
    _RACK = build_demo_rack(int(os.environ.get("PTS_SIM_MOTORS", "6")))
    RUNNER = Runner(DB, SETTINGS, STATE, transport_factory=demo_transport_factory(_RACK),
                    log_fn=_log)
    _log("Running in SIMULATION mode (PTS_SIMULATE=1).")
else:
    DB = Database()
    SETTINGS = Settings()
    STATE = AppState()
    RUNNER = Runner(DB, SETTINGS, STATE, log_fn=_log)

# -- WebSocket broadcast ------------------------------------------------------
_loop: Optional[asyncio.AbstractEventLoop] = None
_connections: Set["asyncio.Queue[Dict[str, Any]]"] = set()


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _loop
    _loop = asyncio.get_event_loop()
    yield


app = FastAPI(title="Servomotor Production Test System", lifespan=lifespan)


def _broadcast_from_thread() -> None:
    if _loop is None:
        return
    snapshot = STATE.snapshot()
    _loop.call_soon_threadsafe(_enqueue, snapshot)


def _enqueue(snapshot: Dict[str, Any]) -> None:
    for q in list(_connections):
        try:
            q.put_nowait(snapshot)
        except asyncio.QueueFull:
            pass


STATE.add_listener(_broadcast_from_thread)


# -- helpers ------------------------------------------------------------------
def _test_set_ids() -> List[int]:
    ids: List[int] = []
    for bus in STATE.buses.values():
        ids.extend(bus.test_set.keys())
    return ids


def _scope_ids(scope: str) -> List[int]:
    if scope == "test_set":
        return list(dict.fromkeys(_test_set_ids()))
    return DB.all_motor_ids()


def _device_row(uid: int) -> Dict[str, Any]:
    motor = DB.get_motor(uid) or {}
    overall = evaluation.overall_result(DB, SETTINGS, uid)
    in_set = RUNNER.find_bus(uid)
    return {
        "unique_id": uid_hex(uid),
        "product_type": motor.get("product_type"),
        "hw_version": motor.get("hw_version"),
        "scc": motor.get("scc"),
        "calibration_done": bool(motor.get("calibration_done")),
        "first_detected": motor.get("first_detected"),
        "last_seen": motor.get("last_seen"),
        "result": overall["result"],
        "failing": overall["failing"],
        "in_test_set": in_set is not None,
        "bus": in_set,
    }


# -- state / logs -------------------------------------------------------------
@app.get("/api/state")
def get_state():
    return STATE.snapshot()


@app.get("/api/logs")
def get_logs():
    return {"logs": LOGS[-200:]}


# -- serial ports -------------------------------------------------------------
@app.get("/api/ports")
def get_ports():
    ports: List[str] = []
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = []
    if os.environ.get("PTS_SIMULATE") == "1":
        ports = ["SIM-A", "SIM-B", "SIM-C"] + ports
    return {"available": sorted(set(ports)), "assigned": SETTINGS.get_serial_ports(),
            "ready": SETTINGS.ports_ready()}


@app.post("/api/ports/{bus}")
def set_port(bus: str, body: Dict[str, Any] = Body(...)):
    try:
        SETTINGS.set_serial_port(bus, body.get("port") or None)
    except ValueError as e:
        raise HTTPException(400, str(e))
    return get_ports()


# -- settings / phases --------------------------------------------------------
@app.get("/api/phases")
def get_phase_defs():
    return {"phases": [p.to_dict() for p in phase_defs.all_phases()]}


@app.get("/api/settings")
def get_settings():
    return SETTINGS.as_dict()


@app.post("/api/settings/run_scope")
def set_run_scope(body: Dict[str, Any] = Body(...)):
    try:
        SETTINGS.set_run_scope(body.get("scope", "all"))
    except ValueError as e:
        raise HTTPException(400, str(e))
    return {"run_scope": SETTINGS.run_scope}


@app.post("/api/phases/{number}")
def update_phase(number: int, body: Dict[str, Any] = Body(...)):
    if number not in phase_defs.PHASES_BY_NUMBER:
        raise HTTPException(404, "no such phase")
    SETTINGS.update_phase(number, enabled=body.get("enabled"),
                          params=body.get("params"), criteria=body.get("criteria"))
    return {"phase": SETTINGS.as_dict()["phases"][str(number)],
            "criteria_version": SETTINGS.criteria_version}


# -- detection / test set -----------------------------------------------------
@app.post("/api/detect/{bus}")
def detect(bus: str):
    if bus not in config.BUS_IDS:
        raise HTTPException(404, "no such bus")
    RUNNER.detect(bus)
    return {"ok": True}


@app.post("/api/clear/{bus}")
def clear(bus: str):
    RUNNER.clear_set(bus)
    return {"ok": True}


@app.post("/api/clear_all")
def clear_all():
    RUNNER.clear_all()
    return {"ok": True}


# -- run controls -------------------------------------------------------------
@app.post("/api/run/start_all")
def start_all():
    RUNNER.start_all()
    return {"ok": True}


@app.post("/api/run/start/{bus}")
def start_section(bus: str):
    RUNNER.start_section(bus)
    return {"ok": True}


@app.post("/api/run/pause")
def pause():
    RUNNER.pause(); return {"ok": True}


@app.post("/api/run/resume")
def resume():
    RUNNER.resume(); return {"ok": True}


@app.post("/api/run/cancel")
def cancel():
    RUNNER.cancel(); return {"ok": True}


@app.post("/api/run/stop")
def stop():
    RUNNER.stop(); return {"ok": True}


@app.post("/api/system_reset")
def system_reset():
    RUNNER.system_reset_all(); return {"ok": True}


@app.post("/api/run/rerun/{bus}/{phase}")
def rerun(bus: str, phase: int):
    RUNNER.rerun_phase(bus, phase); return {"ok": True}


# -- LED test (Phase 15) ------------------------------------------------------
@app.post("/api/led/confirm")
def led_confirm():
    RUNNER.led_confirm_all_pass(); return {"ok": True}


@app.post("/api/led/check_removed")
def led_check_removed():
    missing = RUNNER.led_check_removed()
    return {"missing": missing}


# -- Stage B ------------------------------------------------------------------
@app.post("/api/evaluate")
def evaluate(body: Dict[str, Any] = Body(default={})):
    # Evaluate the motors picked by the Tab-2 Scope selector, then refresh the grid.
    return RUNNER.evaluate(body.get("scope", "all"))


PNG_PROGRESS: Dict[str, Any] = {"running": False, "done": 0, "total": 0, "pngs": 0}
_png_lock = threading.Lock()


@app.post("/api/generate_pngs")
def generate_pngs(body: Dict[str, Any] = Body(default={})):
    """Start PNG generation in the background; poll /api/png_progress for status.
    Returns immediately so the UI can show progress (the whole rack can take a
    few minutes)."""
    scope = body.get("scope", "all")
    with _png_lock:
        if PNG_PROGRESS["running"]:
            return {"started": False, **PNG_PROGRESS}
        ids = _scope_ids(scope)
        PNG_PROGRESS.update(running=True, done=0, total=len(ids), pngs=0)

    def _work():
        def prog(done, total):
            PNG_PROGRESS["done"] = done
            PNG_PROGRESS["total"] = total
        try:
            res = png_generation.generate_all(DB, SETTINGS, ids, progress=prog)
            PNG_PROGRESS["pngs"] = res["pngs"]
        finally:
            PNG_PROGRESS["running"] = False

    threading.Thread(target=_work, name="png-gen", daemon=True).start()
    return {"started": True, "total": len(ids)}


@app.get("/api/png_progress")
def png_progress():
    return dict(PNG_PROGRESS)


# -- database / Tab 2 ---------------------------------------------------------
@app.get("/api/db/devices")
def db_devices(scope: str = "all", result: Optional[str] = None,
               failed_phase: Optional[int] = None):
    rows = [_device_row(uid) for uid in _scope_ids(scope)]
    if result in ("pass", "fail"):
        rows = [r for r in rows if r["result"] == result]
    if failed_phase is not None:
        rows = [r for r in rows
                if any(f["phase"] == failed_phase for f in r["failing"])]
    n_pass = sum(1 for r in rows if r["result"] == "pass")
    return {"devices": rows, "total": len(rows), "pass": n_pass,
            "fail": len(rows) - n_pass,
            "yield": (100.0 * n_pass / len(rows)) if rows else 0.0,
            "criteria_version": SETTINGS.criteria_version}


@app.get("/api/db/device/{uid_hex_str}")
def db_device(uid_hex_str: str):
    uid = uid_from_hex(uid_hex_str)
    row = _device_row(uid)
    evals = DB.latest_evals_for_motor(uid)
    data = {}
    for ph in phase_defs.PHASES_BY_NUMBER:
        d = DB.latest_phase_data(uid, ph)
        if d:
            # don't ship big blobs over the API
            data[ph] = {"collected_at": d["collected_at"],
                        "observation": d.get("observation"),
                        "scalar": d.get("scalar"),
                        "has_blob": d.get("raw_blob") is not None,
                        "firmware_version": d.get("firmware_version")}
    row["evals"] = {ph: {"result": e["result"], "failing_metric": e.get("failing_metric"),
                         "cleared": e.get("cleared"), "metrics": e.get("derived_metrics")}
                    for ph, e in evals.items()}
    row["data"] = data
    return row


@app.post("/api/db/clear_failure/{uid_hex_str}/{phase}")
def clear_failure(uid_hex_str: str, phase: int, body: Dict[str, Any] = Body(...)):
    ok = DB.set_cleared(uid_from_hex(uid_hex_str), phase,
                        body.get("cleared_by", ""), body.get("note", ""))
    return {"ok": ok}


@app.post("/api/db/identify/{uid_hex_str}")
def identify(uid_hex_str: str):
    ok = RUNNER.identify(uid_from_hex(uid_hex_str))
    return {"ok": ok}


# -- histograms (per-phase tabs) ---------------------------------------------
@app.get("/api/histogram/{phase}/{metric}")
def histogram(phase: int, metric: str, scope: str = "test_set"):
    motor_ids = _scope_ids(scope)
    values = evaluation.gather_metric(DB, motor_ids, phase, metric)
    counts = evaluation.gather_categorical(DB, motor_ids, phase, metric)
    pdef = phase_defs.PHASES_BY_NUMBER.get(phase)
    thresholds: Dict[str, float] = {}
    unit = None
    kind = "histogram"
    if pdef:
        crit = SETTINGS.phase_criteria(phase)
        for item in pdef.measured:
            if item.key == metric:
                unit = item.unit
                kind = item.kind
                for tk in item.threshold_keys:
                    if tk in crit:
                        thresholds[tk] = crit[tk]
    return {"phase": phase, "metric": metric, "kind": kind, "values": values,
            "counts": counts, "thresholds": thresholds, "unit": unit,
            "n": len(values), "n_categorical": sum(counts.values())}


@app.get("/api/pngs")
def pngs(scope: str = "test_set"):
    return {"groups": png_generation.list_pngs_by_type(_scope_ids(scope))}


# -- WebSocket ----------------------------------------------------------------
@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    q: "asyncio.Queue[Dict[str, Any]]" = asyncio.Queue(maxsize=32)
    _connections.add(q)
    try:
        await websocket.send_json(STATE.snapshot())
        while True:
            snapshot = await q.get()
            await websocket.send_json(snapshot)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        _connections.discard(q)


# -- static frontend + plots --------------------------------------------------
app.mount("/plots", StaticFiles(directory=config.PLOTS_DIR), name="plots")
if os.path.isdir(config.FRONTEND_DIR):
    app.mount("/", StaticFiles(directory=config.FRONTEND_DIR, html=True), name="frontend")
