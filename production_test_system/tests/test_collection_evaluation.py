"""End-to-end: collect on a simulated rack, evaluate, and assert each injected
fault is caught by the correct phase (and good motors pass)."""

import pytest

from tests.harness import Harness, good_profile
from backend import evaluation as ev, png_generation as png

FAULTS = {
    "good": (0x10, good_profile(), None),
    "bad_firmware": (0x11, good_profile(firmware_version="0.14.0.0"), 1),
    "bad_identity": (0x12, good_profile(scc=4), 2),
    "low_voltage": (0x13, good_profile(supply_voltage=20.0), 3),
    "comm_errors": (0x14, good_profile(ping_drop_rate=0.5, crc_errors=5), 4),
    "calib_fail": (0x15, good_profile(calibration_fails=True), 5),
    "hall_track_bad": (0x16, good_profile(tracking_error_rot=0.02), 6),
    "dead_magnet": (0x17, good_profile(dead_magnet=True), 8),
    "weak_curr_broken": (0x18, good_profile(current_limit_broken=True), 9),
    "ov_no_trip_low": (0x1C, good_profile(ov_no_trip_low=True), 10),
    "ov_false_trip_high": (0x1D, good_profile(ov_false_trip_high=True), 10),
    "thermal_hot": (0x19, good_profile(thermal_slope=8.0), 11),
    "openloop_fatal": (0x1A, good_profile(openloop_skips=True), 12),
    "closedloop_fatal": (0x1B, good_profile(closedloop_fatal=True), 13),
}


@pytest.fixture(scope="module")
def run_results():
    h = Harness()
    for name, (uid, prof, _) in FAULTS.items():
        h.add_motor("A", uid, prof)
    h.detect_and_wait("A")
    h.run_and_wait("A")
    h.runner.led_confirm_all_pass()
    ev.evaluate_all(h.db, h.settings)
    return h


def test_all_phases_collected_for_good(run_results):
    h = run_results
    uid = 0x10
    for ph in h.settings.enabled_phase_numbers():
        assert h.db.has_phase_data(uid, ph), "phase %d not collected" % ph


def test_good_motor_passes(run_results):
    h = run_results
    assert ev.overall_result(h.db, h.settings, 0x10)["result"] == "pass"


@pytest.mark.parametrize("name", list(FAULTS.keys()))
def test_fault_caught_by_expected_phase(run_results, name):
    h = run_results
    uid, _, expected_phase = FAULTS[name]
    res = ev.overall_result(h.db, h.settings, uid)
    if expected_phase is None:
        assert res["result"] == "pass"
    else:
        failing_phases = {f["phase"] for f in res["failing"]}
        assert expected_phase in failing_phases, \
            "%s: expected phase %d in %s" % (name, expected_phase, sorted(failing_phases))


def test_reevaluation_flips_on_criteria_change(run_results):
    h = run_results
    # tighten voltage tolerance so the 24 V good motor now fails phase 3
    v0 = h.settings.criteria_version
    h.settings.update_phase(3, criteria={"voltage_nominal": 30.0})
    assert h.settings.criteria_version > v0
    ev.evaluate_motor(h.db, h.settings, 0x10)
    assert h.db.latest_phase_eval(0x10, 3)["result"] == "fail"
    # restore and confirm it flips back
    h.settings.update_phase(3, criteria={"voltage_nominal": 24.0})
    ev.evaluate_motor(h.db, h.settings, 0x10)
    assert h.db.latest_phase_eval(0x10, 3)["result"] == "pass"


def test_clear_failure_makes_overall_pass(run_results):
    h = run_results
    uid = 0x11   # bad firmware -> phase 1 fail
    assert ev.overall_result(h.db, h.settings, uid)["result"] == "fail"
    assert h.db.set_cleared(uid, 1, "tester", "reflashed by hand")
    assert ev.overall_result(h.db, h.settings, uid)["result"] == "pass"


def test_png_generation_and_grouping(run_results):
    h = run_results
    res = png.generate_all(h.db, h.settings, [0x10, 0x17])
    assert res["pngs"] > 0
    groups = png.list_pngs_by_type([0x10, 0x17])
    assert groups["hall_waveform"] and groups["temperature"]
