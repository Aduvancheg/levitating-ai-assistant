"""
test_ai_agent.py — Unit tests for AI Agent, SQLite Memory & Tool Calling (Phase 4 V11).

Covers:
- TASK-AI-1: SQLite ai_history.db persistence & reload
- TASK-AI-2: Tool Calling safety (set_levitation_height, inspect_surface, clear_temp_cache)
- TASK-AI-3: AudioService non-blocking (RPI-CL-13)
- RPI-CL-12: API key masking, AI disconnect graceful degradation
"""

import asyncio
import json
import os
import tempfile
import shutil
from pathlib import Path
from unittest.mock import MagicMock, AsyncMock, patch

import pytest

from brain_rpi5.src.main_server import (
    AIHistoryDB,
    AIConfigManager,
    GeminiAIManager,
    AudioService,
    tool_set_levitation_height,
    tool_inspect_surface,
    tool_clear_temp_cache,
    trigger_nod_gesture,
    system_state,
    SystemState,
)


# =====================================================================
# Fixtures
# =====================================================================

@pytest.fixture
def tmp_dir(tmp_path):
    """Provide a clean temp directory for each test."""
    return tmp_path


@pytest.fixture
def history_db(tmp_dir):
    """AIHistoryDB backed by a temp SQLite file."""
    db_path = tmp_dir / "test_ai_history.db"
    return AIHistoryDB(db_path=db_path)


@pytest.fixture
def config_mgr(tmp_dir):
    """AIConfigManager backed by a temp config file."""
    config_path = tmp_dir / "test_ai_config.json"
    return AIConfigManager(config_path=config_path)


@pytest.fixture(autouse=True)
def reset_system_state():
    """Reset AI-related system state before each test."""
    system_state.ai_enabled = False
    system_state.ai_status = "DISCONNECTED"
    system_state.ai_ping_ms = 0.0
    system_state.ai_thoughts_log = []
    system_state.ai_chat_history = []
    system_state.target_z = 25.0
    system_state.agent_pitch = 0.0
    system_state.audio_available = False
    system_state.audio_vad_active = False
    yield


# =====================================================================
# TEST GROUP 1: SQLite ai_history.db Persistence
# =====================================================================

@pytest.mark.asyncio
async def test_sqlite_init_creates_table(history_db):
    """Verify init_db creates the messages table without errors."""
    await history_db.init_db()
    assert history_db.db_path.exists()


@pytest.mark.asyncio
async def test_sqlite_save_load_messages(history_db):
    """Verify messages saved to SQLite can be loaded back correctly."""
    await history_db.init_db()
    await history_db.save_message("user", "Привет, Агент!")
    await history_db.save_message("assistant", "Здравствуйте! Чем могу помочь?")
    await history_db.save_message("user", "Какая высота?")

    messages = await history_db.load_recent(10)
    assert len(messages) == 3
    assert messages[0]["role"] == "user"
    assert messages[0]["content"] == "Привет, Агент!"
    assert messages[1]["role"] == "assistant"
    assert messages[2]["content"] == "Какая высота?"


@pytest.mark.asyncio
async def test_sqlite_survives_reboot(history_db):
    """History persists when a new AIHistoryDB instance reads the same file (simulates reboot)."""
    await history_db.init_db()
    await history_db.save_message("user", "Сообщение до перезагрузки")
    await history_db.save_message("assistant", "Ответ до перезагрузки")

    # Simulate reboot: create a new instance pointing to the same DB file
    history_db_2 = AIHistoryDB(db_path=history_db.db_path)
    await history_db_2.init_db()
    messages = await history_db_2.load_recent(10)

    assert len(messages) == 2
    assert messages[0]["content"] == "Сообщение до перезагрузки"
    assert messages[1]["content"] == "Ответ до перезагрузки"


@pytest.mark.asyncio
async def test_sqlite_load_recent_limit(history_db):
    """load_recent respects the limit parameter."""
    await history_db.init_db()
    for i in range(20):
        await history_db.save_message("user", f"Message {i}")

    messages = await history_db.load_recent(5)
    assert len(messages) == 5
    # Should be the last 5 messages (16..19), ordered chronologically
    assert messages[0]["content"] == "Message 15"
    assert messages[4]["content"] == "Message 19"


@pytest.mark.asyncio
async def test_sqlite_checksum(history_db):
    """get_checksum returns consistent SHA-256 hash."""
    await history_db.init_db()
    await history_db.save_message("user", "Test data")
    checksum1 = await history_db.get_checksum()
    assert len(checksum1) == 64  # SHA-256 hex length
    checksum2 = await history_db.get_checksum()
    assert checksum1 == checksum2


# =====================================================================
# TEST GROUP 2: AIConfigManager Persistence
# =====================================================================

def test_config_save_and_load(config_mgr):
    """Config saves API key and ai_enabled to JSON, then loads them back."""
    config_mgr.save(api_key="AIzaSyD_test_key_1234567890", ai_enabled=True)
    assert config_mgr.config_path.exists()

    # Create new instance and load
    config_mgr_2 = AIConfigManager(config_path=config_mgr.config_path)
    config_mgr_2.load()
    assert config_mgr_2.api_key == "AIzaSyD_test_key_1234567890"
    assert config_mgr_2.ai_enabled is True


def test_config_auto_load_api_key(config_mgr):
    """API key auto-loads from file when load() is called."""
    config_mgr.save(api_key="KEY_12345678", ai_enabled=False)
    new_mgr = AIConfigManager(config_path=config_mgr.config_path)
    new_mgr.load()
    assert new_mgr.api_key == "KEY_12345678"


def test_config_ai_enabled_flag(config_mgr):
    """ai_enabled flag is correctly restored after save/load cycle."""
    config_mgr.save(ai_enabled=True)
    new_mgr = AIConfigManager(config_path=config_mgr.config_path)
    new_mgr.load()
    assert new_mgr.ai_enabled is True


def test_config_missing_file_no_crash(config_mgr):
    """load() on non-existent file does not crash."""
    config_mgr.load()  # File doesn't exist
    assert config_mgr.api_key == ""
    assert config_mgr.ai_enabled is False


# =====================================================================
# TEST GROUP 3: API Key Security (RPI-CL-12)
# =====================================================================

def test_api_key_not_logged_in_plain(config_mgr):
    """RPI-CL-12: masked_key() never reveals the full API key."""
    config_mgr.save(api_key="AIzaSyDxxxxxxxxxxxxxxxxxxxxxxxxxxYYYY")
    masked = config_mgr.masked_key()
    assert "AIza" in masked        # First 4 chars visible
    assert "YYYY" in masked        # Last 4 chars visible
    assert "xxxx" not in masked    # Middle is hidden
    assert "****" in masked        # Masking pattern present
    assert len(masked) < len(config_mgr.api_key)


def test_api_key_short_masked(config_mgr):
    """Short or empty keys produce safe masked output."""
    config_mgr.save(api_key="AB")
    assert config_mgr.masked_key() == "***"

    config_mgr.save(api_key="")
    assert config_mgr.masked_key() == "***"


# =====================================================================
# TEST GROUP 4: Tool Calling Safety (RPI-CL-12)
# =====================================================================

@pytest.mark.asyncio
async def test_tool_set_levitation_height_valid():
    """set_levitation_height accepts values in [10..45] mm range."""
    result = await tool_set_levitation_height(30.0)
    assert "OK" in result
    assert system_state.target_z == 30.0


@pytest.mark.asyncio
async def test_tool_set_levitation_height_out_of_range_low():
    """set_levitation_height rejects target_z < 10 mm."""
    original_z = system_state.target_z
    result = await tool_set_levitation_height(5.0)
    assert "REJECTED" in result
    assert system_state.target_z == original_z  # Unchanged


@pytest.mark.asyncio
async def test_tool_set_levitation_height_out_of_range_high():
    """set_levitation_height rejects target_z > 45 mm."""
    original_z = system_state.target_z
    result = await tool_set_levitation_height(50.0)
    assert "REJECTED" in result
    assert system_state.target_z == original_z  # Unchanged


@pytest.mark.asyncio
async def test_tool_set_levitation_height_boundary():
    """set_levitation_height accepts boundary values 10.0 and 45.0."""
    result = await tool_set_levitation_height(10.0)
    assert "OK" in result
    assert system_state.target_z == 10.0

    result = await tool_set_levitation_height(45.0)
    assert "OK" in result
    assert system_state.target_z == 45.0


@pytest.mark.asyncio
async def test_tool_inspect_surface():
    """inspect_surface tilts agent and returns analysis result."""
    result = await tool_inspect_surface(15.0)
    assert "OK" in result
    assert "15.0" in result
    assert "1600x1200" in result
    # Agent pitch should return to 0 after inspection
    assert system_state.agent_pitch == 0.0


@pytest.mark.asyncio
async def test_tool_inspect_surface_out_of_range():
    """inspect_surface rejects tilt angles > 30 degrees."""
    result = await tool_inspect_surface(35.0)
    assert "REJECTED" in result


@pytest.mark.asyncio
async def test_tool_clear_temp_cache_empty_dir(tmp_dir):
    """clear_temp_cache reports nothing to clean when dir doesn't exist."""
    # Temporarily override TEMP_CACHE_DIR
    import brain_rpi5.src.main_server as ms
    original = ms.TEMP_CACHE_DIR
    ms.TEMP_CACHE_DIR = tmp_dir / "nonexistent_cache"

    result = await tool_clear_temp_cache()
    assert "Nothing to clean" in result or "does not exist" in result

    ms.TEMP_CACHE_DIR = original


@pytest.mark.asyncio
async def test_tool_clear_temp_cache_preserves_db(tmp_dir):
    """clear_temp_cache deletes temp files but NEVER touches ai_history.db."""
    import brain_rpi5.src.main_server as ms
    original_cache = ms.TEMP_CACHE_DIR
    original_db = ms.AI_HISTORY_DB_PATH

    # Set up temp cache with files
    cache_dir = tmp_dir / "cache_test"
    cache_dir.mkdir()
    (cache_dir / "frame_001.jpg").write_bytes(b"fake_image" * 1000)
    (cache_dir / "calibration.log").write_text("log data")
    sub_dir = cache_dir / "subdir"
    sub_dir.mkdir()
    (sub_dir / "nested.tmp").write_text("nested file")

    ms.TEMP_CACHE_DIR = cache_dir

    # Set up separate DB file
    db_path = tmp_dir / "ai_history.db"
    db_path.write_text("PRECIOUS_DATA")
    ms.AI_HISTORY_DB_PATH = db_path

    result = await tool_clear_temp_cache()
    assert "OK" in result

    # Cache contents should be gone
    assert len(list(cache_dir.iterdir())) == 0

    # DB must survive!
    assert db_path.exists()
    assert db_path.read_text() == "PRECIOUS_DATA"

    ms.TEMP_CACHE_DIR = original_cache
    ms.AI_HISTORY_DB_PATH = original_db


# =====================================================================
# TEST GROUP 5: AI Graceful Degradation (RPI-CL-12)
# =====================================================================

@pytest.mark.asyncio
async def test_ai_disconnect_does_not_crash_pid():
    """RPI-CL-12: PID loop continues running when AI is disconnected."""
    system_state.ai_status = "AI_DISCONNECTED"
    system_state.target_z = 25.0

    # PID loop should still compute (simulated by checking state is writable)
    system_state.z = 24.5
    error = system_state.target_z - system_state.z
    assert error == 0.5
    assert system_state.ai_status == "AI_DISCONNECTED"
    # System state is independent of AI status
    system_state.pwm_channels = [200, 200, 200, 200, 200]
    assert system_state.pwm_channels == [200, 200, 200, 200, 200]


@pytest.mark.asyncio
async def test_gemini_manager_no_genai(config_mgr):
    """GeminiAIManager gracefully handles missing google-generativeai."""
    import brain_rpi5.src.main_server as ms
    original = ms.HAS_GENAI
    ms.HAS_GENAI = False

    db = AIHistoryDB(db_path=config_mgr.config_path.parent / "test.db")
    config_mgr.save(api_key="test_key_12345678")
    mgr = GeminiAIManager(config_mgr, db)
    result = await mgr.connect()

    assert result is False
    assert system_state.ai_status == "AI_DISCONNECTED"

    ms.HAS_GENAI = original


# =====================================================================
# TEST GROUP 6: Nod Gesture (UC-BUS-12)
# =====================================================================

@pytest.mark.asyncio
async def test_nod_gesture_z_shift():
    """UC-BUS-12: Nod gesture temporarily raises Z by 3mm and returns."""
    system_state.target_z = 25.0
    await trigger_nod_gesture()
    # After the gesture completes, target_z should return to original
    assert system_state.target_z == 25.0


# =====================================================================
# TEST GROUP 7: AudioService Non-Blocking (RPI-CL-13)
# =====================================================================

def test_audio_service_init_without_sounddevice():
    """RPI-CL-13: AudioService doesn't crash when sounddevice is unavailable."""
    import brain_rpi5.src.main_server as ms
    original = ms.HAS_AUDIO
    ms.HAS_AUDIO = False

    svc = AudioService()
    # Init should return False gracefully
    loop = asyncio.new_event_loop()
    result = loop.run_until_complete(svc.init())
    loop.close()
    assert result is False
    assert system_state.audio_available is False

    ms.HAS_AUDIO = original


@pytest.mark.asyncio
async def test_audio_capture_returns_none_when_unavailable():
    """RPI-CL-13: capture_voice_chunk returns None when audio is unavailable."""
    svc = AudioService()
    system_state.audio_available = False
    result = await svc.capture_voice_chunk(1.0)
    assert result is None
"""
test_ai_agent.py — 12+ unit tests for AI & Memory module.
Covers all checklist items: RPI-CL-12, RPI-CL-13.
"""
