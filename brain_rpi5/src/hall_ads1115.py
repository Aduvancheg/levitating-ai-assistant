"""
hall_ads1115.py — ADS1115 ADC Driver for SS49E Hall Sensors

Reads 4 analog Hall-effect sensors via the ADS1115 16-bit ADC over I2C.
Converts raw ADC readings → voltage → distance in mm using a calibration
polynomial.

Conforms to:
  - backlog LS-5 (860 SPS, async operable, mock-friendly)
  - quality.md §2 (type hints, NaN/inf rejection, exception handling)
  - quality.md §4 (boundary testing for anomalous values)
"""

from __future__ import annotations

import math
import logging
from typing import List, Optional

logger = logging.getLogger(__name__)

# ADS1115 register addresses
_REG_CONVERSION = 0x00
_REG_CONFIG     = 0x01

# ADS1115 configuration bit-fields
_OS_START       = 0x8000   # Start single conversion
_PGA_4_096V     = 0x0200   # ±4.096 V full-scale range
_MODE_SINGLE    = 0x0100   # Single-shot mode
_DR_860_SPS     = 0x00E0   # 860 samples per second
_COMP_DISABLE   = 0x0003   # Disable comparator

# Multiplexer settings for single-ended channels A0..A3
_MUX_SINGLE = [0x4000, 0x5000, 0x6000, 0x7000]


class HallADS1115Driver:
    """
    Driver for reading 4 SS49E Hall sensors connected to ADS1115 ADC.
    Configured for 860 SPS (samples per second) operation.
    """

    # Full-scale voltage for PGA = ±4.096 V
    _FULL_SCALE_V: float = 4.096

    def __init__(
        self,
        i2c_bus: Optional[object] = None,
        address: int = 0x48,
        poly_coeffs: Optional[List[float]] = None,
    ) -> None:
        """
        Args:
            i2c_bus:      smbus2.SMBus instance (or mock for testing).
            address:      I2C address of ADS1115 (default 0x48).
            poly_coeffs:  Calibration polynomial [a, b, c] for
                          distance_mm = a*V² + b*V + c.
        """
        self.i2c_bus = i2c_bus
        self.address = address
        self.poly_coeffs: List[float] = poly_coeffs or [1.2, -5.4, 25.0]

    # -----------------------------------------------------------------
    # Conversion helpers
    # -----------------------------------------------------------------

    def raw_to_voltage(self, raw_val: int) -> float:
        """
        Convert signed 16-bit ADC value to voltage.

        Range: -4.096 V to +4.096 V (PGA = ±4.096 V).
        Handles unsigned overflow wrapping (values > 32767).
        Rejects NaN and inf — returns 0.0 as safe fallback.
        """
        # Wrap unsigned to signed 16-bit
        if raw_val > 32767:
            raw_val -= 65536

        voltage = (raw_val / 32768.0) * self._FULL_SCALE_V

        # Reject non-finite results (quality.md §4: boundary testing)
        if math.isnan(voltage) or math.isinf(voltage):
            logger.warning("ADC produced non-finite voltage: raw=%d", raw_val)
            return 0.0

        return voltage

    def voltage_to_distance(self, voltage: float) -> float:
        """
        Translate sensor voltage to distance in mm using the
        calibration polynomial: distance = a·V² + b·V + c.

        Guards against:
          - Negative / inverted magnetic field → clamped to 0 V.
          - NaN / inf input → returns 0.0 mm.
        """
        # Reject non-finite input BEFORE any arithmetic
        if math.isnan(voltage) or math.isinf(voltage):
            logger.warning("Non-finite voltage input: %s", voltage)
            return 0.0

        # Clamp negative voltage anomaly to baseline
        safe_voltage: float = max(0.0, float(voltage))

        a, b, c = self.poly_coeffs
        distance = a * (safe_voltage ** 2) + b * safe_voltage + c

        # Physical distance cannot be negative
        result = max(0.0, float(distance))

        # Final NaN guard on the polynomial result
        if math.isnan(result) or math.isinf(result):
            logger.warning("Polynomial produced non-finite distance")
            return 0.0

        return result

    # -----------------------------------------------------------------
    # I2C communication
    # -----------------------------------------------------------------

    def read_channel_raw(self, channel: int) -> int:
        """
        Perform a single-shot ADC conversion on the specified channel.

        Steps:
          1. Write config register to select channel + start conversion.
          2. Wait for conversion to complete (poll config register OS bit).
          3. Read the conversion result register.

        Returns:
            Raw 16-bit signed ADC value.
        """
        if self.i2c_bus is None:
            return 0

        if channel < 0 or channel > 3:
            raise ValueError(f"Channel must be 0..3, got {channel}")

        # Build config word
        config = (
            _OS_START
            | _MUX_SINGLE[channel]
            | _PGA_4_096V
            | _MODE_SINGLE
            | _DR_860_SPS
            | _COMP_DISABLE
        )

        # Write config to start conversion
        config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
        self.i2c_bus.write_i2c_block_data(self.address, _REG_CONFIG, config_bytes)

        # Poll for conversion complete (OS bit goes HIGH when done).
        # At 860 SPS, conversion takes ~1.2 ms.  Max 10 polls.
        for _ in range(10):
            status = self.i2c_bus.read_i2c_block_data(self.address, _REG_CONFIG, 2)
            if status[0] & 0x80:
                break

        # Read conversion result (2 bytes, big-endian)
        raw_data = self.i2c_bus.read_i2c_block_data(self.address, _REG_CONVERSION, 2)
        raw_val = (raw_data[0] << 8) | raw_data[1]

        return raw_val

    def read_all_channels_voltage(self) -> List[float]:
        """Read voltages from all 4 channels (A0..A3)."""
        if self.i2c_bus is None:
            # Mock / unit-test mode — return safe midpoint values
            return [2.5, 2.5, 2.5, 2.5]

        voltages: List[float] = []
        for ch in range(4):
            try:
                raw_val = self.read_channel_raw(ch)
                voltages.append(self.raw_to_voltage(raw_val))
            except Exception as exc:
                logger.error("Failed to read Hall channel %d: %s", ch, exc)
                voltages.append(0.0)  # Safe fallback — not 2.5V
        return voltages

    def read_all_distances(self) -> List[float]:
        """Returns distance in mm for each of the 4 Hall sensors."""
        voltages = self.read_all_channels_voltage()
        return [self.voltage_to_distance(v) for v in voltages]
