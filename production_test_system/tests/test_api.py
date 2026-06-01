"""API smoke tests via Starlette TestClient against the simulated app."""

import os
import time

os.environ["PTS_SIMULATE"] = "1"
os.environ["PTS_SIM_MOTORS"] = "3"

import pytest
from fastapi.testclient import TestClient

from backend import app as app_module

client = TestClient(app_module.app)


def test_phases_and_settings():
    phases = client.get("/api/phases").json()["phases"]
    assert len(phases) == 15
    settings = client.get("/api/settings").json()
    assert settings["phases"]["10"]["enabled"] is False


def test_ports_and_assignment():
    ports = client.get("/api/ports").json()
    assert "SIM-A" in ports["available"]
    r = client.post("/api/ports/A", json={"port": "SIM-A"}).json()
    assert r["assigned"]["A"] == "SIM-A"
    # duplicate rejected
    client.post("/api/ports/B", json={"port": "SIM-B"})
    bad = client.post("/api/ports/C", json={"port": "SIM-A"})
    assert bad.status_code == 400


def test_detect_then_devices():
    client.post("/api/detect/A")
    deadline = time.time() + 5
    count = 0
    while time.time() < deadline:
        st = client.get("/api/state").json()
        count = st["buses"]["A"]["motor_count"]
        if count >= 3 and st["buses"]["A"]["pending_detections"] == 0:
            break
        time.sleep(0.2)
    assert count == 3


def test_phase_update_bumps_criteria_version():
    before = client.get("/api/settings").json()["criteria_version"]
    r = client.post("/api/phases/3", json={"criteria": {"voltage_nominal": 23.5}})
    assert r.json()["criteria_version"] > before


def test_histogram_endpoint_shape():
    h = client.get("/api/histogram/3/supply_voltage?scope=all").json()
    assert "values" in h and "thresholds" in h
