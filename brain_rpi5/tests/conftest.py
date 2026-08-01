"""
conftest.py — Shared pytest fixtures for brain_rpi5 tests.
"""

import pytest
from unittest.mock import MagicMock


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
