"""
hall_ads1115.py — ADS1115 ADC Driver for SS49E Hall Sensors

Reads 4 analog Hall-effect sensors via the ADS1115 16-bit ADC over I2C.
Converts raw ADC readings → voltage → distance in mm using a calibration
polynomial.

Conforms to:
  - backlog LS-5 (860 SPS, async operable, mock-friendly)
  - quality.md §2 (type hints, NaN/inf rejection, exception handling)
  - quality.md §4 (boundary testing for anomalous values)

Invalid readings (I2C error, conversion timeout, NaN, voltage outside the
plausible SS49E range) are reported as None — never as a substitute number.
A fake 0 V would map to the polynomial constant term, i.e. a plausible
working height, and hide the failure from the control loop (review S2).
"""

from __future__ import annotations

import math
import time
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

    Calibration polynomial:
      distance_mm = a·V² + b·V + c
      Recommended monotonic coefficients `MONOTONIC_DEFAULT_COEFFS = [0.0, -10.0, 39.0]`
      are strictly decreasing on the valid SS49E operating window [1.0 V, 3.8 V]:
      higher magnetic flux density (closer sphere → higher voltage above the
      ~2.5 V quiescent point) maps monotonically to smaller distance Z (e.g.
      2.0 V → 19.0 mm, 2.5 V → 14.0 mm, 3.5 V → 4.0 mm). Custom polynomials can
      be loaded from a JSON file and verified for monotonicity via
      `is_polynomial_monotonic()` (review S3).
    """

    # Full-scale voltage for PGA = ±4.096 V
    _FULL_SCALE_V: float = 4.096

    # Plausible SS49E output window. The sensor's rail-to-rail swing is
    # roughly 0.86..4.21 V at 5 V supply; anything outside this window
    # means a broken wire, a shorted input or ADC saturation.
    DEFAULT_VALID_VOLTAGE_RANGE = (0.2, 4.0)

    # Monotonicity check window (active sensing range for approaching pole).
    DEFAULT_MONOTONIC_RANGE = (1.0, 3.8)
    MONOTONIC_DEFAULT_COEFFS = [0.0, -10.0, 39.0]

    # OS-bit polling: one conversion at 860 SPS takes ~1.2 ms.
    _POLL_INTERVAL_S: float = 0.0002
    _POLL_MAX_ATTEMPTS: int = 25  # ~5 ms budget

    def __init__(
        self,
        i2c_bus: Optional[object] = None,
        address: int = 0x48,
        poly_coeffs: Optional[List[float]] = None,
        valid_voltage_range: Optional[tuple] = None,
        coil_ff_gains: Optional[List[float]] = None,
    ) -> None:
        """
        Args:
            i2c_bus:      smbus2.SMBus instance (or mock for testing).
            address:      I2C address of ADS1115 (default 0x48).
            poly_coeffs:  Calibration polynomial [a, b, c] for
                          distance_mm = a*V² + b*V + c.
            valid_voltage_range: (min_v, max_v) accepted as a real reading.
            coil_ff_gains: Optional feed-forward coefficients (V per PWM unit)
                           to subtract the Base coils' own magnetic field from
                           measured Hall voltage: V_net = V_meas - Σ k_i * duty_i
                           (review S4).
        """
        self.i2c_bus = i2c_bus
        self.address = address
        self.poly_coeffs: List[float] = list(poly_coeffs) if poly_coeffs is not None else [1.2, -5.4, 25.0]
        self.valid_voltage_range = valid_voltage_range or self.DEFAULT_VALID_VOLTAGE_RANGE
        self.coil_ff_gains: List[float] = list(coil_ff_gains) if coil_ff_gains is not None else [0.0] * 5

    @staticmethod
    def is_polynomial_monotonic(
        poly_coeffs: List[float],
        voltage_range: tuple = DEFAULT_MONOTONIC_RANGE,
    ) -> bool:
        """
        Check whether `d(V) = a·V² + b·V + c` is strictly monotonic on
        `voltage_range = (v_min, v_max)` (review S3).

        The derivative `d'(V) = 2·a·V + b` is linear, so `d(V)` is strictly
        monotonic on `[v_min, v_max]` iff `d'(v_min)` and `d'(v_max)` have the
        same non-zero sign (or one endpoint is 0 while the other is non-zero).
        """
        if len(poly_coeffs) != 3:
            return False
        a, b, _ = (float(poly_coeffs[0]), float(poly_coeffs[1]), float(poly_coeffs[2]))
        v_min, v_max = float(voltage_range[0]), float(voltage_range[1])
        if v_min >= v_max:
            return False
        d_min = 2.0 * a * v_min + b
        d_max = 2.0 * a * v_max + b
        if abs(d_min) < 1e-12 and abs(d_max) < 1e-12:
            return False
        return (d_min * d_max) >= -1e-12

    def load_calibration_file(
        self,
        path: str,
        monotonic_range: tuple = DEFAULT_MONOTONIC_RANGE,
    ) -> None:
        """
        Load calibration polynomial `[a, b, c]` (and optional `coil_ff_gains`)
        from a JSON file and validate monotonicity on `monotonic_range` (review S3).

        Raises:
            ValueError: If `poly_coeffs` is not a 3-element monotonic polynomial.
        """
        import json
        from pathlib import Path

        data = json.loads(Path(path).read_text(encoding="utf-8"))
        coeffs = data.get("poly_coeffs")
        if not isinstance(coeffs, list) or len(coeffs) != 3:
            raise ValueError("Calibration file must contain 'poly_coeffs' with 3 floats [a, b, c]")
        coeffs_f = [float(x) for x in coeffs]
        if not self.is_polynomial_monotonic(coeffs_f, monotonic_range):
            raise ValueError(
                f"Calibration polynomial {coeffs_f} is non-monotonic on {monotonic_range} V"
            )
        self.poly_coeffs = coeffs_f
        if "coil_ff_gains" in data and isinstance(data["coil_ff_gains"], list):
            self.coil_ff_gains = [float(k) for k in data["coil_ff_gains"]]

    def compensate_coil_field(
        self,
        voltage: Optional[float],
        coil_duties: Optional[List[int]] = None,
    ) -> Optional[float]:
        """
        Subtract the Base coils' self-field contribution from a measured Hall
        voltage: `V_net = V_meas - Σ k_i · duty_i` (review S4).
        """
        if voltage is None:
            return None
        if not coil_duties or not self.coil_ff_gains:
            return voltage
        offset_v = sum(
            k * float(d) for k, d in zip(self.coil_ff_gains, coil_duties)
        )
        return voltage - offset_v

    # -----------------------------------------------------------------
    # Conversion helpers
    # -----------------------------------------------------------------

    def raw_to_voltage(self, raw_val: int) -> Optional[float]:
        """
        Convert signed 16-bit ADC value to voltage.

        Range: -4.096 V to +4.096 V (PGA = ±4.096 V).
        Handles unsigned overflow wrapping (values > 32767).
        Rejects NaN and inf — returns None.
        """
        # Wrap unsigned to signed 16-bit
        if raw_val > 32767:
            raw_val -= 65536

        voltage = (raw_val / 32768.0) * self._FULL_SCALE_V

        # Reject non-finite results (quality.md §4: boundary testing)
        if math.isnan(voltage) or math.isinf(voltage):
            logger.warning("ADC produced non-finite voltage: raw=%d", raw_val)
            return None

        return voltage

    def voltage_to_distance(self, voltage: Optional[float]) -> Optional[float]:
        """
        Translate sensor voltage to distance in mm using the
        calibration polynomial: distance = a·V² + b·V + c.

        Returns None (sensor invalid) for:
          - None / NaN / inf input.
          - Voltage outside `valid_voltage_range` (broken wire, short,
            inverted field, saturation).
          - Polynomial result that is negative or non-finite (outside
            the calibrated range).
        """
        if voltage is None:
            return None

        # Reject non-finite input BEFORE any arithmetic
        if math.isnan(voltage) or math.isinf(voltage):
            logger.warning("Non-finite voltage input: %s", voltage)
            return None

        v_min, v_max = self.valid_voltage_range
        if not (v_min <= voltage <= v_max):
            logger.warning(
                "Hall voltage %.3f V outside plausible range [%.2f, %.2f] V",
                voltage, v_min, v_max,
            )
            return None

        a, b, c = self.poly_coeffs
        distance = a * (voltage ** 2) + b * voltage + c

        if math.isnan(distance) or math.isinf(distance):
            logger.warning("Polynomial produced non-finite distance")
            return None

        # A negative distance means the reading is outside calibration.
        if distance < 0.0:
            logger.warning("Polynomial produced negative distance %.2f mm", distance)
            return None

        return float(distance)

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
        # At 860 SPS, conversion takes ~1.2 ms. Wait between polls and
        # fail loudly on timeout — reading the conversion register before
        # the OS bit is set would silently return the PREVIOUS sample.
        ready = False
        for _ in range(self._POLL_MAX_ATTEMPTS):
            status = self.i2c_bus.read_i2c_block_data(self.address, _REG_CONFIG, 2)
            if status[0] & 0x80:
                ready = True
                break
            time.sleep(self._POLL_INTERVAL_S)
        if not ready:
            raise TimeoutError(f"ADS1115 conversion timeout on channel {channel}")

        # Read conversion result (2 bytes, big-endian)
        raw_data = self.i2c_bus.read_i2c_block_data(self.address, _REG_CONVERSION, 2)
        raw_val = (raw_data[0] << 8) | raw_data[1]

        return raw_val

    def read_all_channels_voltage(self) -> List[Optional[float]]:
        """
        Read voltages from all 4 channels (A0..A3).

        A channel that fails to read is reported as None.
        """
        if self.i2c_bus is None:
            # Mock / unit-test mode — return safe midpoint values
            return [2.5, 2.5, 2.5, 2.5]

        voltages: List[Optional[float]] = []
        for ch in range(4):
            try:
                raw_val = self.read_channel_raw(ch)
                voltages.append(self.raw_to_voltage(raw_val))
            except Exception as exc:
                logger.error("Failed to read Hall channel %d: %s", ch, exc)
                voltages.append(None)  # Invalid — never a fake voltage
        return voltages

    def read_all_distances(self) -> List[Optional[float]]:
        """Returns distance in mm for each of the 4 Hall sensors (None = invalid)."""
        voltages = self.read_all_channels_voltage()
        return [self.voltage_to_distance(v) for v in voltages]
