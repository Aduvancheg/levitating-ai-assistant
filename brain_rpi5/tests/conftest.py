"""
conftest.py — Shared pytest fixtures for brain_rpi5 tests.
V11: Extended with AI, Audio and Post-Assembly BIST fixtures.
"""

import os
import sys

# Ensure default fast execution for tests before any main_server import (N-H3)
os.environ.setdefault("LEVITATION_PIPELINE_TIME_SCALE", "0.25")

import pytest
from pathlib import Path
from unittest.mock import MagicMock, AsyncMock


@pytest.fixture(autouse=True)
def fast_pipeline_time_scale(monkeypatch):
    """Speed up pipeline sleeps for fast CI/CD execution (N-H3)."""
    scale = float(os.environ.get("LEVITATION_PIPELINE_TIME_SCALE", "0.25"))
    for mod_name in ("src.main_server", "brain_rpi5.src.main_server"):
        mod = sys.modules.get(mod_name)
        if mod is not None:
            monkeypatch.setattr(mod, "PIPELINE_TIME_SCALE", scale, raising=False)


@pytest.fixture
def mock_i2c_bus():
    """
    Provides a mock smbus2.SMBus-like object for ADS1115 / ToF testing.
    Callers can configure return values per test.
    """
    bus = MagicMock()
    bus.read_i2c_block_data = MagicMock(return_value=[0x00, 0x00])
    bus.write_i2c_block_data = MagicMock()
    return bus


@pytest.fixture
def mock_gemini_model():
    """Provides a mock Gemini GenerativeModel for AI tests without real API calls."""
    model = MagicMock()
    response = MagicMock()
    response.text = "Я ваш AI-ассистент Antigravity. Готов к работе!"
    response.candidates = []
    model.generate_content = MagicMock(return_value=response)
    return model


@pytest.fixture
def tmp_ai_db(tmp_path):
    """Provides a temporary SQLite DB path for AI history tests."""
    return tmp_path / "test_ai_history.db"


@pytest.fixture
def tmp_ai_config(tmp_path):
    """Provides a temporary config path for AI config tests."""
    return tmp_path / "test_ai_config.json"

