"""
test_main_server.py — API tests for the FastAPI levitation server.

Tests cover:
  - GET /api/state — full state snapshot
  - POST /api/pid/set_tunings — on-the-fly PID tuning
  - POST /api/manage/execute — pipeline start + collision lock (WEB-CL-2)
  - GET /api/wifi/scan — Wi-Fi scan mock
  - POST /api/wifi/connect — Wi-Fi handover mock
  - WebSocket /ws/telemetry — telemetry reception + Agent data ingestion

Uses httpx.AsyncClient + FastAPI TestClient for synchronous HTTP tests
and the built-in WebSocket test client for WS tests.

References:
  - quality-v4.md: RPI-CL-1, WEB-CL-1, WEB-CL-2
"""

import json
import pytest
from fastapi.testclient import TestClient

from src.main_server import app, system_state, SystemState


@pytest.fixture(autouse=True)
def reset_system_state():
    """Reset system state before each test to avoid cross-contamination."""
    # Re-initialise all fields to defaults
    system_state.__init__()
    yield
    # Cleanup: reset pipeline state to prevent collision lock
    system_state.active_pipeline = None
    system_state.pipeline_status = "Idle"
    system_state.pipeline_steps = []


@pytest.fixture
def client():
    """Synchronous TestClient — does NOT start background tasks."""
    return TestClient(app, raise_server_exceptions=False)


# =====================================================================
# GET /api/state
# =====================================================================

class TestGetState:
    def test_returns_all_required_fields(self, client: TestClient) -> None:
        """GET /api/state must return all telemetry fields."""
        response = client.get("/api/state")
        assert response.status_code == 200
        data = response.json()

        # Top-level coordinates
        assert "x" in data
        assert "y" in data
        assert "z" in data
        assert "target_z" in data

        # PID parameters
        assert "kp" in data
        assert "ki" in data
        assert "kd" in data

        # Thermal
        assert "temp" in data
        assert "throttling" in data

        # Pipeline
        assert "active_pipeline" in data

        # Wi-Fi nested object
        assert "wifi" in data
        wifi = data["wifi"]
        assert "mode" in wifi
        assert "ssid" in wifi
        assert "ip" in wifi
        assert "agent_rssi" in wifi

    def test_returns_default_values(self, client: TestClient) -> None:
        """Default state should have Z=0, target_z=25, no pipeline."""
        response = client.get("/api/state")
        data = response.json()
        assert data["target_z"] == 25.0
        assert data["active_pipeline"] is None
        assert data["throttling"] is False
        assert data["wifi"]["mode"] == "AP"


# =====================================================================
# POST /api/pid/set_tunings
# =====================================================================

class TestSetTunings:
    def test_updates_kp_only(self, client: TestClient) -> None:
        """Partial update: only Kp should change, others untouched."""
        original_ki = system_state.ki
        response = client.post(
            "/api/pid/set_tunings",
            json={"kp": 99.9},
        )
        assert response.status_code == 200
        assert system_state.kp == 99.9
        assert system_state.ki == original_ki  # unchanged

    def test_updates_all_fields(self, client: TestClient) -> None:
        """Full update: all PID parameters + target_z."""
        response = client.post(
            "/api/pid/set_tunings",
            json={"kp": 10.0, "ki": 0.5, "kd": 3.0, "target_z": 30.0},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "SUCCESS"
        assert system_state.kp == 10.0
        assert system_state.ki == 0.5
        assert system_state.kd == 3.0
        assert system_state.target_z == 30.0


# =====================================================================
# POST /api/manage/execute
# =====================================================================

class TestExecuteCommand:
    def test_starts_known_pipeline(self, client: TestClient) -> None:
        """Starting a valid scenario should return 'started'."""
        response = client.post(
            "/api/manage/execute",
            json={"command": "BENCH_CALIBRATE"},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "started"
        assert data["pipeline"] == "BENCH_CALIBRATE"

    def test_collision_lock_blocks_second_pipeline(self, client: TestClient) -> None:
        """
        WEB-CL-2: If a pipeline is already running, reject the new one
        with HTTP 400.
        """
        # Simulate an active pipeline
        system_state.active_pipeline = "SPATIAL_HELIX"

        response = client.post(
            "/api/manage/execute",
            json={"command": "SMOOTH_TAKEOFF"},
        )
        assert response.status_code == 400
        data = response.json()
        assert "SPATIAL_HELIX" in data["detail"]

    def test_unknown_command_returns_404(self, client: TestClient) -> None:
        """Unknown scenario name should yield HTTP 404."""
        response = client.post(
            "/api/manage/execute",
            json={"command": "NONEXISTENT_SCENARIO"},
        )
        assert response.status_code == 404

    def test_all_8_commands_accepted(self, client: TestClient) -> None:
        """Every documented Use Case command should be accepted."""
        commands = [
            "BENCH_CALIBRATE", "SMOOTH_TAKEOFF", "SPATIAL_HELIX",
            "LISSAJOUS_PATROL", "EMO_REACTION", "FORCE_FEEDBACK",
            "SAFE_LANDING", "RTH_ROUTINE",
        ]
        for cmd in commands:
            # Reset pipeline state between attempts
            system_state.active_pipeline = None
            system_state.bist_unlocked = True  # Bypass interlock for test
            response = client.post(
                "/api/manage/execute",
                json={"command": cmd},
            )
            assert response.status_code == 200, f"Failed for command: {cmd}"
            assert response.json()["pipeline"] == cmd


# =====================================================================
# GET /api/wifi/scan
# =====================================================================

class TestWifiScan:
    def test_returns_network_list(self, client: TestClient) -> None:
        """Wi-Fi scan should return a list of networks."""
        response = client.get("/api/wifi/scan")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "SUCCESS"
        assert isinstance(data["networks"], list)
        assert len(data["networks"]) >= 1

        # Each network should have ssid, signal, secure
        for net in data["networks"]:
            assert "ssid" in net
            assert "signal" in net
            assert "secure" in net


# =====================================================================
# POST /api/wifi/connect
# =====================================================================

class TestWifiConnect:
    def test_updates_wifi_state(self, client: TestClient) -> None:
        """Wi-Fi connect should switch from AP to STA mode."""
        assert system_state.base_wifi_mode == "AP"

        response = client.post(
            "/api/wifi/connect",
            json={"ssid": "MyHomeWiFi", "password": "secret123"},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "SUCCESS"

        # Verify state was updated
        assert system_state.base_wifi_mode == "STA"
        assert system_state.base_wifi_ssid == "MyHomeWiFi"


# =====================================================================
# WebSocket /ws/telemetry
# =====================================================================

class TestWebSocket:
    def test_telemetry_connection_and_agent_data(self, client: TestClient) -> None:
        """
        WebSocket should accept connections and process Agent data.
        """
        with client.websocket_connect("/ws/telemetry") as ws:
            # Send Agent orientation data
            agent_data = json.dumps({
                "source": "agent",
                "pitch": 5.5,
                "roll": -3.2,
                "yaw": 180.0,
                "battery": 87.5,
                "coil_pwm": 512,
                "relay": "Isolated",
            })
            ws.send_text(agent_data)

            # Give the server a moment to process
            # (In TestClient, this is synchronous)

        # Verify Agent data was ingested into system_state
        assert system_state.agent_pitch == 5.5
        assert system_state.agent_roll == -3.2
        assert system_state.agent_yaw == 180.0
        assert system_state.agent_battery == 87.5
        assert system_state.agent_coil_pwm == 512
        assert system_state.agent_relay_status == "Isolated"

    def test_malformed_json_does_not_crash(self, client: TestClient) -> None:
        """
        WEB-CL-3: Malformed JSON should be silently ignored,
        not crash the WebSocket handler.
        """
        with client.websocket_connect("/ws/telemetry") as ws:
            ws.send_text("this is not valid json {{{")
            # Connection should stay alive — no exception
            ws.send_text("{}")
            # Still alive
