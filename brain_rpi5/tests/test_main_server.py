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

from unittest.mock import AsyncMock, patch, MagicMock
import json
import pytest
from fastapi.testclient import TestClient

from src.main_server import (
    app,
    system_state,
    SystemState,
    CalibrationState,
    get_api_token,
    get_agent_token,
    AgentTelemetryUDPProtocol,
    AgentTelemetryMessage,
    process_agent_telemetry,
    PIPELINE_TIME_SCALE,
    pipeline_sleep,
)


@pytest.fixture(autouse=True)
def reset_system_state():
    """Reset system state before each test to avoid cross-contamination."""
    # Re-initialise all fields to defaults
    system_state.__init__()
    yield
    # Cleanup: reset pipeline state to prevent collision lock
    if system_state.pipeline_task is not None and not system_state.pipeline_task.done():
        system_state.pipeline_task.cancel()
    system_state.pipeline_task = None
    system_state.active_pipeline = None
    system_state.pipeline_status = "Idle"
    system_state.pipeline_steps = []


@pytest.fixture
def client():
    """Synchronous TestClient with operator Bearer token — does NOT start background tasks."""
    return TestClient(
        app,
        headers={"Authorization": f"Bearer {get_api_token()}"},
        raise_server_exceptions=False,
    )


@pytest.fixture
def unauth_client():
    """Unauthenticated TestClient to verify API auth enforcement (review A1)."""
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
        """Full update: all PID parameters + target_z (requires unlocked interlocks)."""
        system_state.bist_unlocked = True
        system_state.calib_state = CalibrationState.UNLOCKED_FLIGHT
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

    def test_rejects_out_of_range_target_z_and_tunings(self, client: TestClient) -> None:
        """Review A4: Pydantic bounds reject out-of-range target_z or negative Kp."""
        system_state.bist_unlocked = True
        system_state.calib_state = CalibrationState.UNLOCKED_FLIGHT
        resp_z = client.post("/api/pid/set_tunings", json={"target_z": 99.0})
        assert resp_z.status_code == 422
        resp_kp = client.post("/api/pid/set_tunings", json={"kp": -1.0})
        assert resp_kp.status_code == 422


# =====================================================================
# POST /api/manage/execute
# =====================================================================

class TestExecuteCommand:
    def test_starts_known_pipeline(self, client: TestClient) -> None:
        """Starting a valid scenario should return 'started'."""
        system_state.bist_unlocked = True
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
            system_state.calib_state = CalibrationState.UNLOCKED_FLIGHT  # Bypass calibration FSM
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
        """Wi-Fi connect should validate credentials, schedule handover, and switch to STA mode."""
        assert system_state.base_wifi_mode == "AP"

        with patch(
            "src.main_server.NetworkManager.handover_network",
            new=AsyncMock(return_value=True),
        ) as mock_handover:
            response = client.post(
                "/api/wifi/connect",
                json={"ssid": "MyHomeWiFi", "password": "secret123"},
            )
            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "ACCEPTED"
            mock_handover.assert_awaited_once()

        # Verify state was updated after background handover completed
        assert system_state.base_wifi_mode == "STA"
        assert system_state.base_wifi_ssid == "MyHomeWiFi"


# =====================================================================
# WebSocket /ws/telemetry & /ws/agent (review A1)
# =====================================================================

class TestWebSocket:
    def test_unauthenticated_api_and_ws_rejected(self, unauth_client: TestClient) -> None:
        """Review A1: requests without Bearer token return 401; WS without auth closes with 1008."""
        resp = unauth_client.get("/api/state")
        assert resp.status_code == 401

        with unauth_client.websocket_connect("/ws/telemetry") as ws:
            ws.send_text(json.dumps({"type": "auth", "token": "wrong-token"}))
            with pytest.raises(Exception):
                ws.receive_text()

    def test_telemetry_connection_and_agent_data(self, client: TestClient) -> None:
        """
        Agent telemetry is ingested via dedicated /ws/agent with agent token,
        while /ws/telemetry authenticates with operator token (review A1).
        """
        with client.websocket_connect("/ws/telemetry") as ws_tel:
            ws_tel.send_text(json.dumps({"type": "auth", "token": get_api_token()}))
            auth_reply = json.loads(ws_tel.receive_text())
            assert auth_reply == {"type": "auth_ok"}

        with client.websocket_connect("/ws/agent") as ws:
            ws.send_text(json.dumps({"type": "auth", "token": get_agent_token()}))
            auth_reply = json.loads(ws.receive_text())
            assert auth_reply == {"type": "auth_ok"}

            # Send Agent orientation data
            agent_data = json.dumps({
                "pitch": 5.5,
                "roll": -3.2,
                "yaw": 180.0,
                "battery": 87.5,
                "coil_pwm": 512,
                "relay": "Isolated",
            })
            ws.send_text(agent_data)

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
        with client.websocket_connect("/ws/agent") as ws:
            ws.send_text(json.dumps({"type": "auth", "token": get_agent_token()}))
            assert json.loads(ws.receive_text()) == {"type": "auth_ok"}
            ws.send_text("this is not valid json {{{")
            # Connection should stay alive — no exception
            ws.send_text("{}")
            # Still alive


# =====================================================================
# StaticFiles & Web GUI Tabs (review §5, WEB-CL-10)
# =====================================================================

class TestWebGuiStaticAndTabs:
    def test_serves_index_html_with_ai_and_post_assembly_tabs(
        self, unauth_client: TestClient
    ) -> None:
        """Root '/' serves index.html without Bearer header and includes AI Assistant + Post-Assembly tabs."""
        resp = unauth_client.get("/")
        assert resp.status_code == 200
        html = resp.text
        assert "tab-content-ai" in html
        assert "tab-content-pa" in html
        assert "AI Assistant" in html
        assert "Post-Assembly" in html
        assert "TEST-AI-01" in html
        assert "TEST-PA-01" in html

    def test_post_assembly_and_bypass_blocked_during_calibration_and_preempted_by_shutdown(
        self, client: TestClient
    ) -> None:
        """
        Review regression:
          1. /api/bist/post_assembly_test and /api/bist/bypass must be rejected
             while stand calibration is in WAIT_REMOVAL.
          2. Preempting with SAFE_SHUTDOWN must cancel calib_timer_task and
             reset calib_state to PARKED so the 30s timeout cannot re-energize target_z.
        """
        system_state.bist_unlocked = True
        resp_cal = client.post("/api/calibration/start")
        assert resp_cal.status_code == 200
        assert system_state.calib_state == CalibrationState.WAIT_REMOVAL

        # Post-Assembly BIST must be blocked by guard_command while calibration is active
        resp_pa = client.post("/api/bist/post_assembly_test")
        assert resp_pa.status_code == 400
        assert "калибровка" in resp_pa.json()["detail"]

        # BIST bypass toggle must be blocked while calibration is active
        resp_byp = client.post(
            "/api/bist/bypass",
            json={"override": False},
        )
        assert resp_byp.status_code == 409

        # SAFE_SHUTDOWN preempts calibration and resets calib_state to PARKED
        resp_sd = client.post(
            "/api/manage/execute",
            json={"command": "SAFE_SHUTDOWN"},
        )
        assert resp_sd.status_code == 200
        assert system_state.calib_state == CalibrationState.PARKED
        assert system_state.calib_timer_task is None


class TestAgentUDPTelemetry:
    """Tests for ESP32 UDP Telemetry datagram listener (port 4211, review F4/N-F2)."""

    def test_udp_valid_telemetry_datagram(self):
        proto = AgentTelemetryUDPProtocol()
        transport = MagicMock()
        proto.connection_made(transport)

        # Datagram formatted identically to bno085_format_telemetry_json
        packet = json.dumps({
            "pitch": 12.34,
            "roll": -5.67,
            "yaw": 180.25,
            "battery": 88.5,
            "coil_pwm": 320,
            "relay": "Isolated",
            "rssi": -55
        }).encode("utf-8")

        proto.datagram_received(packet, ("192.168.42.105", 4211))

        assert system_state.agent_wifi_ip == "192.168.42.105"
        assert system_state.agent_wifi_status == "Connected"
        assert system_state.agent_pitch == 12.34
        assert system_state.agent_roll == -5.67
        assert system_state.agent_yaw == 180.25
        assert system_state.agent_orientation == [12.34, -5.67, 180.25]
        assert system_state.agent_battery == 88.5
        assert system_state.agent_coil_pwm == 320
        assert system_state.agent_relay_status == "Isolated"
        assert system_state.agent_wifi_rssi == -55

    def test_udp_garbage_datagram_ignored(self):
        proto = AgentTelemetryUDPProtocol()
        initial_ip = system_state.agent_wifi_ip

        # Send invalid non-JSON data
        proto.datagram_received(b"\xde\xad\xbe\xef\x00\x01\x02", ("192.168.42.199", 4211))

        # State should remain unchanged
        assert system_state.agent_wifi_ip == initial_ip

    def test_udp_out_of_range_datagram_rejected(self):
        proto = AgentTelemetryUDPProtocol()
        initial_battery = system_state.agent_battery

        # Battery 150% is out of Pydantic bounds [0..100]
        packet = json.dumps({"battery": 150.0}).encode("utf-8")
        proto.datagram_received(packet, ("192.168.42.105", 4211))

        assert system_state.agent_battery == initial_battery

    def test_udp_error_received_does_not_crash(self):
        proto = AgentTelemetryUDPProtocol()
        proto.error_received(RuntimeError("Socket error simulation"))
        proto.connection_lost(None)


class TestPipelineTimeScale:
    """Tests for pipeline sleep time scaling (N-H3)."""

    @pytest.mark.asyncio
    async def test_pipeline_sleep_scales_accurately(self, monkeypatch):
        import time
        import src.main_server as ms

        monkeypatch.setattr(ms, "PIPELINE_TIME_SCALE", 0.01)
        t0 = time.perf_counter()
        await ms.pipeline_sleep(1.0)
        elapsed = time.perf_counter() - t0

        # 1.0s scaled down to 0.01s should take well under 0.1s
        assert elapsed < 0.15
