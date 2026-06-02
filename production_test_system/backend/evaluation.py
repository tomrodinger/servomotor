"""Stage B — metric derivation + criteria -> pass/fail.

`Run Evaluation` reads the raw ``phase_data`` (never altering it), derives each
phase's metrics, applies the current criteria, and writes fresh ``phase_eval``
rows tagged with the current ``criteria_version``.  It uses no hardware and is
fully re-runnable: change a criterion, re-evaluate, and watch which units flip.

The *latest* eval row per (motor, phase) is authoritative.  A motor's overall
result is pass only if every enabled phase's latest eval is pass-or-cleared.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from . import blobs, units, phase_defs
from .analysis import hall, thermal
from .database import Database
from .settings import Settings

# Result strings.
PASS = "pass"
FAIL = "fail"
MISSING = "missing"

EvalResult = Tuple[Dict[str, Any], str, Optional[str]]


def _obs(row: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if row and isinstance(row.get("observation"), dict):
        return row["observation"]
    return {}


# ---------------------------------------------------------------------------
# Per-phase evaluators: (data_row, params, criteria) -> (metrics, result, fail)
# ---------------------------------------------------------------------------

def _eval_firmware(row, params, crit) -> EvalResult:
    obs = _obs(row)
    target = str(params.get("target_firmware_version", "0.15.0.0"))
    version = obs.get("firmware_version")
    responded = bool(obs.get("responded"))
    metrics = {"firmware_version": version, "responded": responded, "target": target}
    if not responded or version is None:
        return metrics, FAIL, "responded"
    return (metrics, PASS, None) if version == target else (metrics, FAIL, "firmware_version")


def _eval_identity(row, params, crit) -> EvalResult:
    obs = _obs(row)
    metrics = {"product_type": obs.get("product_type"),
               "hw_version": obs.get("hw_version"), "scc": obs.get("scc")}
    if not obs.get("responded"):
        return metrics, FAIL, "responded"
    if obs.get("product_type") != params.get("expected_product_type"):
        return metrics, FAIL, "product_type"
    if str(obs.get("hw_version")) != str(params.get("expected_hw_version")):
        return metrics, FAIL, "hw_version"
    if int(obs.get("scc", -1)) != int(params.get("expected_scc")):
        return metrics, FAIL, "scc"
    return metrics, PASS, None


def _eval_electrical(row, params, crit) -> EvalResult:
    obs = _obs(row)
    v = obs.get("supply_voltage")
    t = obs.get("temperature")
    fatal = obs.get("fatal_code")
    status_clean = (fatal == 0)
    metrics = {"supply_voltage": v, "temperature": t, "status_clean": status_clean}
    if not obs.get("responded"):
        return metrics, FAIL, "responded"
    nominal = float(crit["voltage_nominal"])
    tol = nominal * float(crit["voltage_tolerance_pct"]) / 100.0
    if v is None or not (nominal - tol <= v <= nominal + tol):
        return metrics, FAIL, "supply_voltage"
    if t is None or not (float(crit["temp_min"]) <= t <= float(crit["temp_max"])):
        return metrics, FAIL, "temperature"
    if not status_clean:
        return metrics, FAIL, "status_clean"
    return metrics, PASS, None


def _eval_comms(row, params, crit) -> EvalResult:
    obs = _obs(row)
    rate = obs.get("success_rate")
    crc = obs.get("crc_errors", 0)
    metrics = {"success_rate": rate, "crc_errors": crc}
    if rate is None:
        return metrics, FAIL, "success_rate"
    if rate < float(crit["required_success_rate"]):
        return metrics, FAIL, "success_rate"
    if crc > int(crit["max_crc_errors"]):
        return metrics, FAIL, "crc_errors"
    return metrics, PASS, None


def _eval_calibration(row, params, crit) -> EvalResult:
    obs = _obs(row)
    ok = bool(obs.get("status_zero"))
    return {"status_zero": ok}, (PASS if ok else FAIL), (None if ok else "status_zero")


def _eval_position_deviation(row, params, crit) -> EvalResult:
    """Phases 6 & 7: max |commanded - hall| in rotations."""
    if not row or not row.get("raw_blob"):
        return {}, MISSING, "max_deviation"
    samples = blobs.unpack_position_stream(row["raw_blob"])
    max_dev = 0.0
    for _, commanded, hall_pos in samples:
        dev = abs(commanded - hall_pos)
        if dev > max_dev:
            max_dev = dev
    max_dev_rot = units.counts_to_rotations(max_dev)
    metrics = {"max_deviation": max_dev_rot, "n_samples": len(samples)}
    threshold = float(crit["deviation_threshold_rot"])
    return (metrics, PASS, None) if max_dev_rot < threshold else (metrics, FAIL, "max_deviation")


def _eval_hall_waveform(row, params, crit) -> EvalResult:
    if not row or not row.get("raw_blob"):
        return {}, MISSING, "extrema_count"
    threshold = int(crit["peak_find_hysteresis"])
    analysis = hall.analyze_capture(row["raw_blob"], threshold)
    # collect per-channel metric lists for histograms
    keys = ["extrema_count", "hall_min", "hall_max", "span",
            "max_peak_from_avg", "max_valley_from_avg",
            "max_adjacent_peak_dev", "max_adjacent_valley_dev",
            "peak_spacing_min", "peak_spacing_max"]
    metrics: Dict[str, Any] = {k: [] for k in keys}
    for ch in range(3):
        cm = analysis["channels"][ch]
        for k in keys:
            metrics[k].append(cm.get(k))

    failing: Optional[str] = None
    for ch in range(3):
        cm = analysis["channels"][ch]
        ec = cm.get("extrema_count")
        if ec is None or not (int(crit["extrema_count_min"]) <= ec <= int(crit["extrema_count_max"])):
            failing = "extrema_count"; break
        if cm.get("hall_min") is not None and cm["hall_min"] < int(crit["saturation_min"]):
            failing = "hall_min"; break
        if cm.get("hall_max") is not None and cm["hall_max"] > int(crit["saturation_max"]):
            failing = "hall_max"; break
        if cm.get("span") is not None and cm["span"] < int(crit["span_min"]):
            failing = "span"; break
        if cm.get("max_peak_from_avg", 0) > int(crit["peak_max_dev_from_avg"]):
            failing = "max_peak_from_avg"; break
        if cm.get("max_valley_from_avg", 0) > int(crit["valley_max_dev_from_avg"]):
            failing = "max_valley_from_avg"; break
        if cm.get("max_adjacent_peak_dev", 0) > int(crit["max_adjacent_peak_dev"]):
            failing = "max_adjacent_peak_dev"; break
        if cm.get("max_adjacent_valley_dev", 0) > int(crit["max_adjacent_valley_dev"]):
            failing = "max_adjacent_valley_dev"; break
        psn = cm.get("peak_spacing_min")
        psx = cm.get("peak_spacing_max")
        if psn is not None and psn < float(crit["peak_spacing_min"]):
            failing = "peak_spacing_min"; break
        if psx is not None and psx > float(crit["peak_spacing_max"]):
            failing = "peak_spacing_max"; break
    return (metrics, PASS, None) if failing is None else (metrics, FAIL, failing)


def _eval_current_control(row, params, crit) -> EvalResult:
    obs = _obs(row)
    dev = obs.get("max_pid_deviation")
    pos_err = obs.get("position_error")
    metrics = {"max_pid_deviation": dev, "min_pid": obs.get("min_pid"),
               "max_pid": obs.get("max_pid"), "position_error": pos_err}
    if dev is None:
        # No reading was collected (the motor did not respond during the read
        # phase) — report that, not a band failure, so widening the band is not
        # mistaken for a fix.
        return metrics, MISSING, "no_reading"
    lo = float(crit["pid_error_min"])
    hi = float(crit["pid_error_max"])
    # Band: too small => current limit not limiting; too large => other fault.
    if dev < lo or dev > hi:
        return metrics, FAIL, "max_pid_deviation"
    # And the motor must actually have reached the final commanded position.
    if pos_err is None:
        return metrics, FAIL, "position_error"
    if pos_err > float(crit["position_tolerance_rot"]):
        return metrics, FAIL, "position_error"
    return metrics, PASS, None


def _eval_overvoltage(row, params, crit) -> EvalResult:
    obs = _obs(row)
    if obs.get("missing") or obs.get("tripped_low") is None:
        return {"tripped_low": obs.get("tripped_low"),
                "tripped_high": obs.get("tripped_high")}, MISSING, "tripped_low"
    metrics = {"tripped_low": obs.get("tripped_low"),
               "tripped_high": obs.get("tripped_high")}
    ok = bool(obs.get("tripped_low")) and not bool(obs.get("tripped_high"))
    return (metrics, PASS, None) if ok else (metrics, FAIL, "tripped_low")


def _eval_thermal(row, params, crit) -> EvalResult:
    if not row or not row.get("raw_blob"):
        return {}, MISSING, "temp_rise"
    baseline = _obs(row).get("baseline")
    series = [list(s) for s in blobs.unpack_temperature_series(row["raw_blob"])]
    res = thermal.analyze(baseline, series)
    metrics = {
        "temp_rise": res["temp_rise"], "fit_slope": res["fit_slope"],
        "fit_start_temp": res["fit_start_temp"], "fit_r": res["fit_r"],
    }
    rise = res["temp_rise"]
    if rise is None or not (float(crit["rise_min"]) <= rise <= float(crit["rise_max"])):
        return metrics, FAIL, "temp_rise"
    st = res["fit_start_temp"]
    if st is None or not (float(crit["start_temp_min"]) <= st <= float(crit["start_temp_max"])):
        return metrics, FAIL, "fit_start_temp"
    sl = res["fit_slope"]
    if sl is None or not (float(crit["slope_min"]) <= sl <= float(crit["slope_max"])):
        return metrics, FAIL, "fit_slope"
    if res["fit_r"] is None or res["fit_r"] < float(crit["r_value_min"]):
        return metrics, FAIL, "fit_r"
    return metrics, PASS, None


def _eval_openloop_burnin(row, params, crit) -> EvalResult:
    obs = _obs(row)
    ok = obs.get("no_fatal_error")
    metrics = {"no_fatal_error": ok, "fatal_error": obs.get("fatal_error")}
    if ok is None:
        return metrics, MISSING, "no_fatal_error"
    return (metrics, PASS, None) if ok else (metrics, FAIL, "no_fatal_error")


def _eval_closedloop_burnin(row, params, crit) -> EvalResult:
    obs = _obs(row)
    no_fatal = obs.get("no_fatal_error")
    series = blobs.unpack_pid_series(row["raw_blob"]) if row and row.get("raw_blob") else []
    overall_max = 0
    for mn, mx in series:
        overall_max = max(overall_max, abs(mn), abs(mx))
    metrics = {"max_pid_deviation": overall_max, "no_fatal_error": no_fatal,
               "n_moves": len(series), "pid_series": [max(abs(a), abs(b)) for a, b in series]}
    if no_fatal is None:
        return metrics, MISSING, "no_fatal_error"
    if not no_fatal:
        return metrics, FAIL, "no_fatal_error"
    if overall_max >= float(crit["max_pid_deviation_threshold"]):
        return metrics, FAIL, "max_pid_deviation"
    return metrics, PASS, None


def _eval_set_alias(row, params, crit) -> EvalResult:
    obs = _obs(row)
    alias = obs.get("alias")
    metrics = {"alias": alias}
    if obs.get("missing"):
        return metrics, MISSING, "alias"
    expected = str(params.get("factory_alias", "X"))
    return (metrics, PASS, None) if alias == expected else (metrics, FAIL, "alias")


def _eval_led(row, params, crit) -> EvalResult:
    obs = _obs(row)
    lit = obs.get("leds_lit")
    metrics = {"leds_lit": lit}
    if lit is None:
        return metrics, MISSING, "leds_lit"
    return (metrics, PASS, None) if lit else (metrics, FAIL, "leds_lit")


_EVALUATORS = {
    1: _eval_firmware, 2: _eval_identity, 3: _eval_electrical, 4: _eval_comms,
    5: _eval_calibration, 6: _eval_position_deviation, 7: _eval_position_deviation,
    8: _eval_hall_waveform, 9: _eval_current_control, 10: _eval_overvoltage,
    11: _eval_thermal, 12: _eval_openloop_burnin, 13: _eval_closedloop_burnin,
    14: _eval_set_alias, 15: _eval_led,
}


def evaluate_phase(db: Database, settings: Settings, unique_id: int,
                   phase: int) -> EvalResult:
    row = db.latest_phase_data(unique_id, phase)
    params = settings.phase_params(phase)
    crit = settings.phase_criteria(phase)
    if row is None:
        return {}, MISSING, None
    return _EVALUATORS[phase](row, params, crit)


def evaluate_motor(db: Database, settings: Settings, unique_id: int,
                   phases: Optional[List[int]] = None) -> Dict[int, str]:
    """Evaluate the given (or all enabled) phases for one motor; write eval rows."""
    if phases is None:
        phases = settings.enabled_phase_numbers()
    cv = settings.criteria_version
    results: Dict[int, str] = {}
    for ph in phases:
        metrics, result, failing = evaluate_phase(db, settings, unique_id, ph)
        db.insert_phase_eval(unique_id, ph, criteria_version=cv,
                             derived_metrics=metrics, result=result,
                             failing_metric=failing)
        results[ph] = result
    return results


def evaluate_all(db: Database, settings: Settings) -> Dict[str, int]:
    """Re-evaluate every motor in the DB; returns a small summary."""
    phases = settings.enabled_phase_numbers()
    motors = db.all_motor_ids()
    n_pass = 0
    for uid in motors:
        evaluate_motor(db, settings, uid, phases)
        if overall_result(db, settings, uid)["result"] == PASS:
            n_pass += 1
    return {"motors": len(motors), "pass": n_pass, "fail": len(motors) - n_pass,
            "criteria_version": settings.criteria_version}


def overall_result(db: Database, settings: Settings, unique_id: int) -> Dict[str, Any]:
    """Overall pass/fail = pass only if every enabled phase is pass-or-cleared."""
    enabled = settings.enabled_phase_numbers()
    evals = db.latest_evals_for_motor(unique_id)
    failing: List[Dict[str, Any]] = []
    for ph in enabled:
        ev = evals.get(ph)
        if ev is None:
            failing.append({"phase": ph, "result": MISSING, "metric": None})
            continue
        if ev["result"] == PASS or ev.get("cleared"):
            continue
        failing.append({"phase": ph, "result": ev["result"],
                        "metric": ev.get("failing_metric")})
    return {"result": PASS if not failing else FAIL, "failing": failing}


def gather_metric(db: Database, motor_ids: List[int], phase: int,
                  metric_key: str) -> List[float]:
    """Collect a derived metric across motors (flattening per-channel lists),
    for the phase-tab histograms.  Reads the latest eval row per motor."""
    values: List[float] = []
    for uid in motor_ids:
        ev = db.latest_phase_eval(uid, phase)
        if not ev or not isinstance(ev.get("derived_metrics"), dict):
            continue
        val = ev["derived_metrics"].get(metric_key)
        if val is None:
            continue
        if isinstance(val, list):
            values.extend(v for v in val if isinstance(v, (int, float)))
        elif isinstance(val, (int, float)) and not isinstance(val, bool):
            values.append(val)
    return values


def gather_categorical(db: Database, motor_ids: List[int], phase: int,
                       metric_key: str) -> Dict[str, int]:
    """Count occurrences of each value of a categorical/boolean metric across
    motors (latest eval row), for the phase-tab count summary.  ``None`` (a
    missing observation) is bucketed as ``"(none)"``."""
    counts: Dict[str, int] = {}
    for uid in motor_ids:
        ev = db.latest_phase_eval(uid, phase)
        if not ev or not isinstance(ev.get("derived_metrics"), dict):
            continue
        metrics = ev["derived_metrics"]
        if metric_key not in metrics:
            continue
        val = metrics[metric_key]
        key = "(none)" if val is None else str(val)
        counts[key] = counts.get(key, 0) + 1
    return counts
