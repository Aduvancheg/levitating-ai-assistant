"""
test_hall_ads1115.py — Unit tests for Hall sensor ADS1115 driver.

Covers:
  - Raw ADC → voltage conversion
  - Voltage → distance polynomial calculation
  - Negative / anomalous voltage handling
  - NaN / inf rejection (quality.md §4)
  - I2C config register writing before read (backlog LS-5)
"""

import math
import pytest
from unittest.mock import MagicMock, call
from brain_rpi5.src.hall_ads1115 import HallADS1115Driver


class TestRawToVoltage:

    def test_zero_raw_gives_zero_volts(self):
        driver = HallADS1115Driver()
        assert abs(driver.raw_to_voltage(0) - 0.0) < 1e-4

    def test_max_positive_raw(self):
        driver = HallADS1115Driver()
        v = driver.raw_to_voltage(32767)
        assert abs(v - 4.096) < 1e-3

    def test_unsigned_overflow_wrapping(self):
        """Values > 32767 should wrap to negative (signed 16-bit)."""
        driver = HallADS1115Driver()
        # 65535 → -1 → -1/32768 * 4.096 ≈ -0.000125
        v = driver.raw_to_voltage(65535)
        assert v < 0.0

    def test_midpoint_raw(self):
        driver = HallADS1115Driver()
        # 16384 → 16384/32768 * 4.096 = 2.048
        v = driver.raw_to_voltage(16384)
        assert abs(v - 2.048) < 1e-3


class TestVoltageToDistance:

    def test_typical_voltage(self):
        driver = HallADS1115Driver()
        # poly: 1.2 * 2² + (-5.4) * 2 + 25 = 4.8 - 10.8 + 25 = 19.0
        dist = driver.voltage_to_distance(2.0)
        assert abs(dist - 19.0) < 1e-4

    def test_negative_voltage_is_invalid(self):
        """Review S2: an inverted / broken input must not become a distance."""
        driver = HallADS1115Driver()
        assert driver.voltage_to_distance(-1.5) is None

    def test_nan_input_returns_none(self):
        driver = HallADS1115Driver()
        assert driver.voltage_to_distance(float("nan")) is None

    def test_inf_input_returns_none(self):
        driver = HallADS1115Driver()
        assert driver.voltage_to_distance(float("inf")) is None

    def test_negative_inf_returns_none(self):
        driver = HallADS1115Driver()
        assert driver.voltage_to_distance(float("-inf")) is None

    def test_none_input_returns_none(self):
        driver = HallADS1115Driver()
        assert driver.voltage_to_distance(None) is None

    def test_zero_voltage_is_invalid_not_working_height(self):
        """
        Review S2: 0 V used to map to the constant term c = 25 mm — right at
        the working height — hiding a dead sensor. It must be rejected.
        """
        driver = HallADS1115Driver()
        assert driver.voltage_to_distance(0.0) is None

    def test_saturated_voltage_is_invalid(self):
        driver = HallADS1115Driver()
        assert driver.voltage_to_distance(4.09) is None

    def test_negative_polynomial_result_is_invalid(self):
        driver = HallADS1115Driver(poly_coeffs=[0.0, -10.0, 5.0])
        # 5 - 10*2 = -15 mm → outside calibration
        assert driver.voltage_to_distance(2.0) is None


class TestI2CConfigRegisterWrite:

    def test_config_written_before_read(self):
        """
        The ADS1115 must have its config register written (to select
        channel and start conversion) before reading the result.
        """
        mock_bus = MagicMock()
        # Mock read_i2c_block_data to return: config ready + conversion result
        mock_bus.read_i2c_block_data = MagicMock(
            side_effect=[
                [0x80, 0x00],   # Config poll: OS bit set (conversion done)
                [0x40, 0x00],   # Conversion result: 0x4000 = 16384
            ]
        )
        mock_bus.write_i2c_block_data = MagicMock()

        driver = HallADS1115Driver(i2c_bus=mock_bus, address=0x48)

        raw = driver.read_channel_raw(0)

        # Verify config was WRITTEN before read
        mock_bus.write_i2c_block_data.assert_called_once()
        write_args = mock_bus.write_i2c_block_data.call_args
        assert write_args[0][0] == 0x48  # address
        assert write_args[0][1] == 0x01  # config register

    def test_all_four_channels_read(self):
        """read_all_channels_voltage must return 4 values via config writes."""
        mock_bus = MagicMock()
        # Each channel: 1 config write + 1 poll read + 1 conversion read
        mock_bus.read_i2c_block_data = MagicMock(
            return_value=[0x80, 0x00]  # OS ready + zero result
        )
        mock_bus.write_i2c_block_data = MagicMock()

        driver = HallADS1115Driver(i2c_bus=mock_bus)
        voltages = driver.read_all_channels_voltage()

        assert len(voltages) == 4
        # 4 channels × 1 config write each = 4 calls
        assert mock_bus.write_i2c_block_data.call_count == 4

    def test_exception_during_read_reports_invalid(self):
        """Review S2: an I2C error must surface as None, not as 0.0 V."""
        mock_bus = MagicMock()
        mock_bus.write_i2c_block_data = MagicMock(side_effect=IOError("Bus error"))

        driver = HallADS1115Driver(i2c_bus=mock_bus)
        voltages = driver.read_all_channels_voltage()

        assert len(voltages) == 4
        assert all(v is None for v in voltages)
        assert all(d is None for d in driver.read_all_distances())

    def test_conversion_timeout_does_not_return_stale_sample(self):
        """OS bit never set → must raise, not read the previous conversion."""
        mock_bus = MagicMock()
        mock_bus.read_i2c_block_data = MagicMock(return_value=[0x00, 0x00])
        mock_bus.write_i2c_block_data = MagicMock()

        driver = HallADS1115Driver(i2c_bus=mock_bus)
        with pytest.raises(TimeoutError):
            driver.read_channel_raw(0)
        # Conversion register (0x00) must never have been read
        read_regs = [c.args[1] for c in mock_bus.read_i2c_block_data.call_args_list]
        assert 0x00 not in read_regs

        assert driver.read_all_channels_voltage() == [None, None, None, None]
