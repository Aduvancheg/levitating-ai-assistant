"""
core_loop.py — Levitation Core Loop (Raspberry Pi 5 Brain)

Central control module implementing:
  - Sigmoidal Complementary Filter (Hall ↔ ToF sensor fusion)
  - 3-axis PID controller (X, Y, Z) with anti-windup
  - Coil Mapper (XYZ error → 5 PWM duty cycles for 5 base coils)
  - UART Packet Builder / Parser with CRC-8
  - Async main loop targeting ≥ 800 Hz control rate
  - Dry-Run Mode for safe debugging without energising coils

Conforms to:
  - architecture.md §5 (sigmoidal blending, Gimbal compensation)
  - quality.md §2 (asyncio, type hints, exception handling)
  - quality.md §3 (≥ 800 Hz control loop rate)
  - quality.md §4 (Dry-Run Mode)
  - backlog LS-7
"""

from __future__ import annotations

import asyncio
import math
import struct
import time
import logging
from typing import List, Tuple, Dict, Optional

logger = logging.getLogger(__name__)


# =====================================================================
# CRC-8 (polynomial 0x07, init 0x00) — matches RP2040 implementation
# =====================================================================

def calculate_crc8(data: bytes) -> int:
    """Calculate CRC-8 (polynomial 0x07) over a byte payload."""
    crc: int = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# =====================================================================
# Complementary Filter — Sigmoidal blending (architecture §5)
# =====================================================================

class ComplementaryFilter:
    """
    Sensor Fusion filter for Z-axis distance measurement.

    Uses a sigmoidal (logistic) blending curve instead of linear
    interpolation to prevent discontinuities that the PID D-term
    would interpret as sudden drops / spikes.

    Transition zone: 49 mm (100% Hall) ↔ 61 mm (100% ToF).
    """

    HALL_MAX_Z_MM: float = 49.0
    TOF_MIN_Z_MM: float = 61.0

    def __init__(self, steepness: float = 0.5) -> None:
        """
        Args:
            steepness: Controls sigmoid slope. Higher = sharper transition.
                       0.5 provides smooth blending across the 12 mm zone.
        """
        self._steepness = steepness
        self._midpoint = (self.HALL_MAX_Z_MM + self.TOF_MIN_Z_MM) / 2.0

    def _sigmoid(self, z: float) -> float:
        """Logistic sigmoid: returns 0..1 (0 = all Hall, 1 = all ToF)."""
        exponent = -self._steepness * (z - self._midpoint)
        # Clamp exponent to prevent overflow
        exponent = max(-20.0, min(20.0, exponent))
        return 1.0 / (1.0 + math.exp(exponent))

    def compute_weights(self, estimated_z_mm: float) -> Tuple[float, float]:
        """
        Returns (hall_weight, tof_weight) based on estimated height.

        Below HALL_MAX_Z_MM: pure Hall (1.0, 0.0)
        Above TOF_MIN_Z_MM: pure ToF  (0.0, 1.0)
        Between: sigmoidal blend.
        """
        z = float(estimated_z_mm)
        if z <= self.HALL_MAX_Z_MM:
            return 1.0, 0.0
        if z >= self.TOF_MIN_Z_MM:
            return 0.0, 1.0

        tof_w = self._sigmoid(z)
        hall_w = 1.0 - tof_w
        return hall_w, tof_w

    def fuse(self, hall_z_mm: float, tof_z_mm: float) -> float:
        """Combine Hall and ToF Z measurements via sigmoidal blending."""
        approx_z = (hall_z_mm + tof_z_mm) / 2.0
        hall_w, tof_w = self.compute_weights(approx_z)
        return hall_w * hall_z_mm + tof_w * tof_z_mm


# =====================================================================
# PID Controller — single axis, with anti-windup
# =====================================================================

class PIDController:
    """
    PID controller for a single axis with:
      - Configurable gains (tunable on-the-fly)
      - Integral anti-windup (clamping)
      - Derivative filtering (optional)
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.1,
        integral_limit: float = 500.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self._integral: float = 0.0
        self._prev_error: float = 0.0

    def set_tunings(self, kp: float, ki: float, kd: float) -> None:
        """Update P, I, D gains on-the-fly without resetting state."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self) -> None:
        """Clear accumulated integral and derivative history."""
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, setpoint: float, measurement: float, dt: float = 0.01) -> float:
        """
        Compute PID output for one time step.

        Args:
            setpoint:    Target value.
            measurement: Current sensor value.
            dt:          Time delta in seconds (must be > 0).

        Returns:
            Control output (unbounded — caller should clamp to PWM range).
        """
        if dt <= 0.0:
            return 0.0

        error = setpoint - measurement

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamp
        self._integral += error * dt
        if self._integral > self.integral_limit:
            self._integral = self.integral_limit
        elif self._integral < -self.integral_limit:
            self._integral = -self.integral_limit
        i_term = self.ki * self._integral

        # Derivative
        d_term = self.kd * ((error - self._prev_error) / dt)
        self._prev_error = error

        return p_term + i_term + d_term


# =====================================================================
# 3-Axis PID — bundles X, Y, Z controllers
# =====================================================================

class TriAxisPID:
    """Three independent PID controllers for X, Y, Z position regulation."""

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.1,
        integral_limit: float = 500.0,
    ) -> None:
        self.pid_x = PIDController(kp, ki, kd, integral_limit)
        self.pid_y = PIDController(kp, ki, kd, integral_limit)
        self.pid_z = PIDController(kp, ki, kd, integral_limit)

    def compute(
        self,
        setpoint: Tuple[float, float, float],
        measurement: Tuple[float, float, float],
        dt: float = 0.01,
    ) -> Tuple[float, float, float]:
        """Returns (output_x, output_y, output_z)."""
        out_x = self.pid_x.compute(setpoint[0], measurement[0], dt)
        out_y = self.pid_y.compute(setpoint[1], measurement[1], dt)
        out_z = self.pid_z.compute(setpoint[2], measurement[2], dt)
        return out_x, out_y, out_z

    def reset_all(self) -> None:
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()


# =====================================================================
# Coil Mapper — XYZ PID outputs → 5 PWM duty cycles
# =====================================================================

class CoilMapper:
    """
    Maps 3-axis PID control outputs to 5 independent coil duty cycles.

    The 5 base coils are arranged as:
      - Coil 0: Center (primary Z-lift)
      - Coil 1: +X direction
      - Coil 2: -X direction
      - Coil 3: +Y direction
      - Coil 4: -Y direction

    The mixing matrix defines how each PID axis contributes to each coil.
    This is a simplified topology — update the matrix to match the
    actual physical coil arrangement.

    All outputs are clamped to [0, max_duty].
    """

    def __init__(self, max_duty: int = 460) -> None:
        """
        Args:
            max_duty: Ceiling from RP2040 MAX_DUTY_LIMIT (≈45 % of 1023).
        """
        self.max_duty = max_duty

        # Mixing matrix: rows = coils (0..4), columns = [X, Y, Z]
        # Signs: positive PID output → more current in the corresponding coil
        self._mix: List[Tuple[float, float, float]] = [
            ( 0.0,  0.0,  1.0),   # Coil 0: pure Z-lift (center)
            ( 1.0,  0.0,  0.5),   # Coil 1: +X correction + half Z
            (-1.0,  0.0,  0.5),   # Coil 2: -X correction + half Z
            ( 0.0,  1.0,  0.5),   # Coil 3: +Y correction + half Z
            ( 0.0, -1.0,  0.5),   # Coil 4: -Y correction + half Z
        ]

    def map(self, pid_x: float, pid_y: float, pid_z: float) -> List[int]:
        """
        Compute 5 duty-cycle values from PID outputs.

        Returns:
            List of 5 integers, each in [0, max_duty].
        """
        duties: List[int] = []
        for mx, my, mz in self._mix:
            raw = mx * pid_x + my * pid_y + mz * pid_z
            clamped = max(0, min(self.max_duty, int(raw)))
            duties.append(clamped)
        return duties


# =====================================================================
# UART Packet Builder — binary frame with CRC-8
# =====================================================================

class UARTPacketBuilder:
    """
    Serialises 5 PWM duty-cycle values into the binary protocol frame
    expected by the RP2040 coprocessor.

    Frame: [0xAA 0x55] [5× uint16_t LE payload] [CRC8] [0x55 0xAA]
    Total: 15 bytes.
    """

    HEADER: bytes = bytes([0xAA, 0x55])
    FOOTER: bytes = bytes([0x55, 0xAA])

    def pack_pwm_packet(self, duties: List[int]) -> bytes:
        """
        Pack 5 duty values into a 15-byte binary frame.

        Raises:
            ValueError: If not exactly 5 values are provided.
        """
        if len(duties) != 5:
            raise ValueError("Must provide exactly 5 PWM duty cycle values")

        payload = bytearray()
        for duty in duties:
            clamped = max(0, min(1023, int(duty)))
            payload.append(clamped & 0xFF)
            payload.append((clamped >> 8) & 0xFF)

        crc = calculate_crc8(bytes(payload))
        return self.HEADER + bytes(payload) + bytes([crc]) + self.FOOTER

    @staticmethod
    def unpack_ack(data: bytes) -> Optional[int]:
        """
        Parse a 3-byte ACK response from RP2040.

        Returns:
            Status byte (0x06 = ACK, 0x15 = NAK) or None if invalid.
        """
        if len(data) < 3:
            return None
        if data[0] != 0xAA:
            return None
        status = data[1]
        expected_crc = calculate_crc8(bytes([status]))
        if data[2] != expected_crc:
            return None
        return status


# =====================================================================
# Async Levitation Core Loop
# =====================================================================

class LevitationCoreLoop:
    """
    Asynchronous main control loop for the Raspberry Pi 5 brain.

    Orchestrates: Sensor read → Fusion → PID → Coil Map → UART send.
    Targets ≥ 800 Hz (1.25 ms per iteration).

    Attributes:
        dry_run: When True, all math runs normally but UART packets
                 are NOT sent to the coprocessor. Safe for debugging
                 LLM integrations without energising coils.
    """

    TARGET_LOOP_HZ: int = 800
    TARGET_DT_S: float = 1.0 / TARGET_LOOP_HZ

    def __init__(
        self,
        serial_port: Optional[object] = None,
        dry_run: bool = False,
        setpoint: Tuple[float, float, float] = (0.0, 0.0, 30.0),
    ) -> None:
        """
        Args:
            serial_port: An opened pyserial Serial object (or None for dry-run).
            dry_run:     If True, suppress UART output (quality.md §4).
            setpoint:    Target (X, Y, Z) position in mm.
        """
        self.serial_port = serial_port
        self.dry_run = dry_run
        self.setpoint = setpoint

        self.filter = ComplementaryFilter(steepness=0.5)
        self.pid = TriAxisPID(kp=2.0, ki=0.1, kd=0.5, integral_limit=500.0)
        self.mapper = CoilMapper(max_duty=460)
        self.packer = UARTPacketBuilder()

        self._running: bool = False
        self._iteration_count: int = 0
        self._last_duties: List[int] = [0, 0, 0, 0, 0]

    async def run(
        self,
        read_sensors_callback,
        max_iterations: Optional[int] = None,
    ) -> None:
        """
        Start the async control loop.

        Args:
            read_sensors_callback: An async callable returning a dict with:
                {
                    "hall_z_mm": float,
                    "tof_xyz": (float, float, float),
                }
            max_iterations: Stop after N iterations (None = run forever).
        """
        self._running = True
        self._iteration_count = 0

        try:
            while self._running:
                loop_start = time.monotonic()

                # 1. Read sensors
                try:
                    sensor_data = await read_sensors_callback()
                except Exception as exc:
                    logger.error("Sensor read failed: %s", exc)
                    # Safe mode: zero all outputs
                    self._last_duties = [0, 0, 0, 0, 0]
                    self._send_packet(self._last_duties)
                    await asyncio.sleep(self.TARGET_DT_S)
                    continue

                # 2. Sensor fusion (Z-axis)
                hall_z = float(sensor_data.get("hall_z_mm", 0.0))
                tof_x, tof_y, tof_z = sensor_data.get("tof_xyz", (0.0, 0.0, 0.0))
                fused_z = self.filter.fuse(hall_z, tof_z)

                measurement = (tof_x, tof_y, fused_z)

                # 3. PID compute
                dt = time.monotonic() - loop_start
                if dt <= 0:
                    dt = self.TARGET_DT_S
                pid_x, pid_y, pid_z = self.pid.compute(
                    self.setpoint, measurement, dt
                )

                # 4. Map to 5 coil duties
                duties = self.mapper.map(pid_x, pid_y, pid_z)
                self._last_duties = duties

                # 5. Send UART packet (or suppress in dry-run)
                self._send_packet(duties)

                self._iteration_count += 1

                if max_iterations is not None and self._iteration_count >= max_iterations:
                    break

                # 6. Pace to target rate
                elapsed = time.monotonic() - loop_start
                sleep_time = self.TARGET_DT_S - elapsed
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)

        except asyncio.CancelledError:
            logger.info("Core loop cancelled — safe shutdown")
            self._safe_shutdown()

        finally:
            self._running = False

    def stop(self) -> None:
        """Request a graceful stop of the control loop."""
        self._running = False

    def _send_packet(self, duties: List[int]) -> None:
        """Serialise and send a PWM packet, respecting dry-run mode."""
        packet = self.packer.pack_pwm_packet(duties)

        if self.dry_run:
            logger.debug("[DRY-RUN] Packet suppressed: duties=%s", duties)
            return

        if self.serial_port is not None:
            try:
                self.serial_port.write(packet)
            except Exception as exc:
                logger.error("UART write failed: %s", exc)

    def _safe_shutdown(self) -> None:
        """Send all-zero command to safely de-energise coils."""
        self._last_duties = [0, 0, 0, 0, 0]
        self._send_packet(self._last_duties)
        logger.info("Safe shutdown: all coils set to 0")
