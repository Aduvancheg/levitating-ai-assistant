import asyncio
import pytest
from fastapi.testclient import TestClient
from src.main_server import app, system_state, CalibrationState

client = TestClient(app)

@pytest.fixture(autouse=True)
def reset_system_state():
    system_state.calib_state = CalibrationState.PARKED
    system_state.bist_unlocked = True
    system_state.bist_bypass_active = False
    system_state.calib_timer_task = None
    system_state.target_z = 30.0

def test_flight_rejected_if_not_unlocked():
    system_state.calib_state = CalibrationState.PARKED
    response = client.post("/api/manage/execute", json={"command": "SPATIAL_HELIX"})
    assert response.status_code == 403
    assert "Калибровка на стенде не завершена" in response.json()["detail"]

def test_calibration_success_workflow():
    resp = client.post("/api/calibration/start")
    assert resp.status_code == 200
    assert system_state.calib_state == CalibrationState.WAIT_REMOVAL
    assert system_state.target_z == 33.0
    
    resp = client.post("/api/calibration/confirm_stand_removed")
    assert resp.status_code == 200
    assert system_state.calib_state == CalibrationState.UNLOCKED_FLIGHT
    assert system_state.target_z == 25.0
    
    resp = client.post("/api/manage/execute", json={"command": "SPATIAL_HELIX"})
    assert resp.status_code == 200

def test_calibration_cancel():
    resp = client.post("/api/calibration/start")
    assert resp.status_code == 200
    
    resp = client.post("/api/calibration/cancel")
    assert resp.status_code == 200
    assert system_state.calib_state == CalibrationState.PARKED
    assert system_state.target_z == 30.0
