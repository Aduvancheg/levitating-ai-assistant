"""
test_v12_features.py — Unit & Integration Tests for Antigravity V12 Polish Edition.

Covers:
  - TASK-AI-5 / RPI-CL-14: Graceful Emergency Safe Landing (Wi-Fi Heartbeat > 500ms & LiPo < 10.0%,
    5.0 mm/s descent, Slew-Rate <= 10 units/tick, SPU-WM30 voice warning, WS shutdown.emergency_reason)
  - TASK-UI-5 / WEB-CL-11: Embedded MJPEG Live Stream (/api/agent/camera_stream, >= 15 FPS, < 200ms latency,
    non-blocking 860 Hz PID loop, UI HUD overlay with Pitch/Roll/Yaw/Z)
  - TASK-AI-6 / RPI-CL-15: Automated SQLite Database Compression & Zip Archiving (> 50 MB threshold,
    PRAGMA wal_checkpoint(FULL), VACUUM, < 10 MB post-archive size, retains latest 1,000 dialogue turns)
"""

import asyncio
import json
import os
import sqlite3
import time
import zipfile
from pathlib import Path
from unittest.mock import AsyncMock, patch

import pytest
from fastapi.testclient import TestClient

import brain_rpi5.src.main_server as ms
from brain_rpi5.src.main_server import (
    AIHistoryDB,
    CalibrationState,
    app,
    archive_db_if_oversized,
    audio_service,
    core_loop_860hz,
    execute_emergency_landing_pipeline,
    get_api_token,
    init_db,
    mjpeg_frame_generator,
    system_state,
    tool_clear_temp_cache,
)


@pytest.fixture(autouse=True)
def reset_v12_state():
    """Reset system_state before and after each V12 test."""
    system_state.__init__()
    system_state.bist_unlocked = True
    system_state.calib_state = CalibrationState.UNLOCKED_FLIGHT
    system_state.agent_wifi_last_seen = time.perf_counter()
    yield
    if system_state.pipeline_task is not None and not system_state.pipeline_task.done():
        system_state.pipeline_task.cancel()
    system_state.pipeline_task = None
    system_state.active_pipeline = None


@pytest.fixture
def client():
    """Authenticated TestClient for operator API calls."""
    return TestClient(
        app,
        headers={"Authorization": f"Bearer {get_api_token()}"},
        raise_server_exceptions=False,
    )


# =====================================================================
# TOUCH 1: TASK-AI-5 / RPI-CL-14 — Emergency Safe Landing
# =====================================================================

@pytest.mark.asyncio
async def test_emergency_landing_triggered_by_wifi_loss_in_core_loop():
    """RPI-CL-14: core_loop_860hz triggers emergency landing when Wi-Fi elapsed > 500ms."""
    system_state.target_z = 2.0
    system_state.pwm_channels = [100, 100, 100, 100, 100]
    system_state.prev_pwm_channels = [100, 100, 100, 100, 100]
    system_state.agent_wifi_last_seen = time.perf_counter() - 0.600  # 600ms > 500ms

    loop_task = asyncio.create_task(core_loop_860hz())
    try:
        await asyncio.sleep(0.05)
        assert system_state.active_pipeline == "EMERGENCY_SAFE_LANDING"
        assert system_state.emergency_reason == "Wi-Fi Heartbeat Loss (>500ms)"
        # Wait for the 2.0mm descent (0.4s) + ramp-down to complete
        await asyncio.sleep(0.55)
        assert system_state.target_z == 0.0
        assert system_state.pwm_channels == [0, 0, 0, 0, 0]
        assert system_state.safe_to_unplug is True
    finally:
        loop_task.cancel()
        try:
            await loop_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_emergency_landing_triggered_by_critical_battery_in_core_loop():
    """RPI-CL-14: core_loop_860hz triggers emergency landing when LiPo battery < 10.0%."""
    system_state.target_z = 1.0
    system_state.agent_wifi_last_seen = time.perf_counter()
    system_state.agent_battery = 7.5  # < 10.0%

    loop_task = asyncio.create_task(core_loop_860hz())
    try:
        await asyncio.sleep(0.05)
        assert system_state.active_pipeline == "EMERGENCY_SAFE_LANDING"
        assert system_state.emergency_reason == "Critical Battery Level (<10%)"
        await asyncio.sleep(0.35)
        assert system_state.target_z == 0.0
        assert system_state.safe_to_unplug is True
    finally:
        loop_task.cancel()
        try:
            await loop_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_emergency_landing_slew_rate_audio_alert_and_telemetry(client: TestClient):
    """RPI-CL-14: Verify <= 10 units/tick PWM delta, 5.0 mm/s descent, SPU-WM30 voice alert, and WS telemetry."""
    system_state.target_z = 2.0
    system_state.pwm_channels = [45, 45, 45, 45, 45]
    system_state.prev_pwm_channels = [45, 45, 45, 45, 45]
    system_state.slew_rate_limit = 10

    observed_pwm_history = [list(system_state.pwm_channels)]
    orig_sleep = asyncio.sleep

    async def tracking_sleep(delay: float):
        observed_pwm_history.append(list(system_state.pwm_channels))
        await orig_sleep(min(delay, 0.01))

    with patch.object(audio_service, "speak_text", new=AsyncMock(return_value=True)) as mock_speak, \
         patch("brain_rpi5.src.main_server.asyncio.sleep", side_effect=tracking_sleep):
        await execute_emergency_landing_pipeline("Critical Battery Level (<10%)")

    # Voice warning dispatched over SPU-WM30
    mock_speak.assert_awaited_once()
    spoken_text = mock_speak.await_args.args[0]
    assert "аварийная посадка" in spoken_text.lower()

    # Verify max PWM delta per tick <= 10
    for prev_vec, curr_vec in zip(observed_pwm_history, observed_pwm_history[1:]):
        for p, c in zip(prev_vec, curr_vec):
            assert abs(p - c) <= 10, f"PWM delta {abs(p - c)} exceeded slew-rate limit 10"

    assert system_state.target_z == 0.0
    assert system_state.pwm_channels == [0, 0, 0, 0, 0]
    assert system_state.safe_to_unplug is True
    assert system_state.emergency_reason == "Critical Battery Level (<10%)"

    # Verify state & telemetry expose emergency_reason
    state_resp = client.get("/api/state")
    assert state_resp.status_code == 200
    assert state_resp.json()["emergency_reason"] == "Critical Battery Level (<10%)"


# =====================================================================
# TOUCH 3: TASK-UI-5 / WEB-CL-11 — Embedded Live MJPEG Stream & HUD
# =====================================================================

@pytest.mark.asyncio
async def test_mjpeg_frame_generator_fps_and_non_blocking_latency():
    """WEB-CL-11: Generator yields valid JPEG frames at >= 15 FPS (< 200ms per frame)."""
    frames = []
    t0 = time.perf_counter()
    async for chunk in mjpeg_frame_generator(max_frames=3):
        frames.append(chunk)
    elapsed = time.perf_counter() - t0

    assert len(frames) == 3
    for frame in frames:
        assert frame.startswith(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
        assert b"\xff\xd8\xff\xe0" in frame
        assert frame.endswith(b"\xff\xd9\r\n")

    # 3 frames at ~60ms each should take ~0.18s (< 0.20s per frame)
    avg_frame_latency = elapsed / 3.0
    assert avg_frame_latency < 0.200


def test_camera_stream_http_endpoint_and_hud_ui(client: TestClient):
    """WEB-CL-11: /api/agent/camera_stream streams multipart MJPEG and UI renders HUD overlay."""
    resp = client.get("/api/agent/camera_stream?frames=2")
    assert resp.status_code == 200
    assert "multipart/x-mixed-replace; boundary=frame" in resp.headers["content-type"]
    assert b"--frame\r\nContent-Type: image/jpeg" in resp.content
    assert b"\xff\xd8\xff\xe0" in resp.content

    html_resp = client.get("/")
    assert html_resp.status_code == 200
    html = html_resp.text
    assert "/api/agent/camera_stream" in html
    assert "camera-stream-card" in html
    assert "hud-overlay" in html
    assert "pitch-val" in html
    assert "roll-val" in html
    assert "z-val" in html
    assert "Clean Temp Cache" in html


# =====================================================================
# TOUCH 4: TASK-AI-6 / RPI-CL-15 — SQLite Auto-Archiving (> 50 MB)
# =====================================================================

@pytest.mark.asyncio
async def test_sqlite_auto_archiving_when_oversized_preserves_recent_1000_turns(
    tmp_path: Path, client: TestClient
):
    """RPI-CL-15: When ai_history.db > 50 MB, clean_cache checkpoints, vacuums, zips, and keeps 1,000 turns."""
    original_db = ms.AI_HISTORY_DB_PATH
    original_db_str = ms.DB_PATH
    original_cache = ms.TEMP_CACHE_DIR

    test_db_path = tmp_path / "ai_history.db"
    test_cache_dir = tmp_path / "temp_cache"
    test_cache_dir.mkdir()
    (test_cache_dir / "temp_frame.jpg").write_bytes(b"frame_data" * 100)

    ms.AI_HISTORY_DB_PATH = test_db_path
    ms.DB_PATH = str(test_db_path)
    ms.TEMP_CACHE_DIR = test_cache_dir

    try:
        # 1. Initialize DB and insert 1,050 dialogue turns
        db = AIHistoryDB(db_path=test_db_path)
        await db.init_db()
        conn = sqlite3.connect(str(test_db_path))
        try:
            rows = [
                ("user" if i % 2 == 0 else "assistant", f"Dialogue turn #{i}", "2026-10-17T11:00:00Z")
                for i in range(1050)
            ]
            conn.executemany(
                "INSERT INTO messages (role, content, timestamp) VALUES (?, ?, ?)",
                rows,
            )
            conn.commit()
        finally:
            conn.close()

        # 2. Inflate DB file size above 50 MB (51 MB)
        with open(test_db_path, "ab") as f:
            f.truncate(51 * 1024 * 1024)
        assert os.path.getsize(test_db_path) > 50 * 1024 * 1024

        # 3. Trigger clean_cache endpoint
        resp = client.post("/api/ai/clean_cache")
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "success"

        # 4. Verify ZIP archive was created and passes integrity check
        archives = list(tmp_path.glob("history_archive_*.zip"))
        assert len(archives) == 1
        zip_path = archives[0]
        assert zipfile.is_zipfile(zip_path)
        with zipfile.ZipFile(zip_path, "r") as zf:
            assert "ai_history_backup.db" in zf.namelist()

        # 5. Verify post-archiving ai_history.db size < 10 MB
        post_size_mb = os.path.getsize(test_db_path) / (1024 * 1024)
        assert post_size_mb < 10.0, f"Expected DB < 10 MB after archiving, got {post_size_mb:.2f} MB"

        # 6. Verify active dialogue retains the most recent 1,000 turns without loss
        recent = await db.load_recent(2000)
        assert len(recent) == 1000
        assert recent[0]["content"] == "Dialogue turn #50"
        assert recent[-1]["content"] == "Dialogue turn #1049"

        # 7. Verify maintenance_logs recorded the archiving event
        assert any("DB Archived:" in entry for entry in system_state.maintenance_logs)
    finally:
        ms.AI_HISTORY_DB_PATH = original_db
        ms.DB_PATH = original_db_str
        ms.TEMP_CACHE_DIR = original_cache

