"""
test_core_loop.py — Unit tests for the Levitation Core Loop.

Covers:
  - Sigmoidal Complementary Filter (boundary conditions)
  - PID Controller math + anti-windup
  - 3-axis PID
  - Coil Mapper (XYZ → 5 PWM)
  - UART Packet Builder (pack + unpack round-trip)
  - Dry-Run Mode
"""

import asyncio
import math
import pytest
from brain_rpi5.src.core_loop import (
    ComplementaryFilter,
    PIDController,
    TriAxisPID,
    CoilMapper,
    UARTPacketBuilder,
    LevitationCoreLoop,
    calculate_crc8,
)


# =================================================================
# Complementary Filter — Sigmoidal blending
# =================================================================

class TestComplementaryFilter:

    def test_below_hall_max_returns_pure_hall(self):
        filt = ComplementaryFilter()
        hw, tw = filt.compute_weights(30.0)
        assert abs(hw - 1.0) < 1e-4
        assert abs(tw - 0.0) < 1e-4

    def test_at_hall_max_boundary_returns_pure_hall(self):
        filt = ComplementaryFilter()
        hw, tw = filt.compute_weights(49.0)
        assert abs(hw - 1.0) < 1e-4
        assert abs(tw - 0.0) < 1e-4

    def test_at_tof_min_boundary_returns_pure_tof(self):
        filt = ComplementaryFilter()
        hw, tw = filt.compute_weights(61.0)
        assert abs(hw - 0.0) < 1e-4
        assert abs(tw - 1.0) < 1e-4

    def test_above_tof_min_returns_pure_tof(self):
        filt = ComplementaryFilter()
        hw, tw = filt.compute_weights(80.0)
        assert abs(hw - 0.0) < 1e-4
        assert abs(tw - 1.0) < 1e-4

    def test_midpoint_sigmoidal_approximately_50_50(self):
        """At the midpoint (55 mm), sigmoidal should give ~50/50 blend."""
        filt = ComplementaryFilter(steepness=0.5)
        hw, tw = filt.compute_weights(55.0)
        # Sigmoid at midpoint = 0.5 exactly
        assert abs(hw - 0.5) < 0.01
        assert abs(tw - 0.5) < 0.01

    def test_weights_always_sum_to_one_in_transition(self):
        filt = ComplementaryFilter()
        for z in [50.0, 52.0, 55.0, 58.0, 60.0]:
            hw, tw = filt.compute_weights(z)
            assert abs(hw + tw - 1.0) < 1e-6

    def test_fuse_uses_blended_values(self):
        filt = ComplementaryFilter()
        # With midpoint input, should return average-ish
        result = filt.fuse(hall_z_mm=50.0, tof_z_mm=60.0)
        assert 50.0 <= result <= 60.0


# =================================================================
# PID Controller
# =================================================================

class TestPIDController:

    def test_proportional_only(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        out = pid.compute(setpoint=50.0, measurement=40.0, dt=1.0)
        # P = 2.0 * 10 = 20.0
        assert abs(out - 20.0) < 1e-4

    def test_full_pid_math(self):
        pid = PIDController(kp=2.0, ki=0.5, kd=0.1)
        out = pid.compute(setpoint=50.0, measurement=40.0, dt=1.0)
        # P = 20.0, I = 0.5*(10*1) = 5.0, D = 0.1*(10/1) = 1.0 → 26.0
        assert abs(out - 26.0) < 1e-4

    def test_anti_windup_clamps_integral(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, integral_limit=10.0)
        # Pump integral beyond limit
        for _ in range(100):
            pid.compute(setpoint=100.0, measurement=0.0, dt=1.0)
        # Integral should be clamped at 10.0
        out = pid.compute(setpoint=100.0, measurement=0.0, dt=1.0)
        # I-term = 1.0 * 10.0 = 10.0 (clamped)
        assert out <= 10.0 + 1e-4

    def test_zero_dt_returns_zero(self):
        pid = PIDController(kp=2.0, ki=0.5, kd=0.1)
        assert pid.compute(setpoint=50.0, measurement=40.0, dt=0.0) == 0.0

    def test_set_tunings_preserves_state(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        pid.compute(setpoint=10.0, measurement=5.0, dt=1.0)
        pid.set_tunings(kp=3.0, ki=0.0, kd=0.0)
        # prev_error should still be 5.0 from the previous call
        out = pid.compute(setpoint=10.0, measurement=5.0, dt=1.0)
        # P = 3.0 * 5 = 15.0, D = 0.0*(5-5)/1 = 0
        assert abs(out - 15.0) < 1e-4

    def test_reset_clears_state(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        pid.compute(setpoint=10.0, measurement=0.0, dt=1.0)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._prev_error == 0.0


# =================================================================
# 3-Axis PID
# =================================================================

class TestTriAxisPID:

    def test_independent_axes(self):
        tri = TriAxisPID(kp=1.0, ki=0.0, kd=0.0)
        ox, oy, oz = tri.compute(
            setpoint=(10.0, 20.0, 30.0),
            measurement=(5.0, 15.0, 25.0),
            dt=1.0,
        )
        assert abs(ox - 5.0) < 1e-4
        assert abs(oy - 5.0) < 1e-4
        assert abs(oz - 5.0) < 1e-4


# =================================================================
# Coil Mapper
# =================================================================

class TestCoilMapper:

    def test_pure_z_maps_to_center_coil(self):
        mapper = CoilMapper(max_duty=460)
        duties = mapper.map(pid_x=0.0, pid_y=0.0, pid_z=200.0)
        assert len(duties) == 5
        # Coil 0 (center): Z=200 → 200
        assert duties[0] == 200
        # Coils 1-4: 0.5*Z = 100
        assert duties[1] == 100
        assert duties[2] == 100
        assert duties[3] == 100
        assert duties[4] == 100

    def test_x_correction_differentiates_coils_1_2(self):
        mapper = CoilMapper(max_duty=460)
        duties = mapper.map(pid_x=100.0, pid_y=0.0, pid_z=0.0)
        # Coil 1 (+X): 100, Coil 2 (-X): max(0, -100) = 0
        assert duties[1] == 100
        assert duties[2] == 0

    def test_clamping_to_max_duty(self):
        mapper = CoilMapper(max_duty=460)
        duties = mapper.map(pid_x=0.0, pid_y=0.0, pid_z=1000.0)
        # Coil 0: min(1000, 460) = 460
        assert duties[0] == 460

    def test_negative_clamped_to_zero(self):
        mapper = CoilMapper(max_duty=460)
        duties = mapper.map(pid_x=0.0, pid_y=0.0, pid_z=-100.0)
        # All negative → clamped to 0
        assert all(d == 0 for d in duties)


# =================================================================
# UART Packet Builder
# =================================================================

class TestUARTPacketBuilder:

    def test_packet_structure_and_crc(self):
        builder = UARTPacketBuilder()
        duties = [100, 250, 512, 800, 1023]
        packet = builder.pack_pwm_packet(duties)

        # Total: Header(2) + Payload(10) + CRC(1) + Footer(2) = 15
        assert len(packet) == 15
        assert packet[0] == 0xAA and packet[1] == 0x55
        assert packet[13] == 0x55 and packet[14] == 0xAA

        # Verify CRC matches payload
        payload = packet[2:12]
        assert packet[12] == calculate_crc8(payload)

    def test_pack_unpack_roundtrip(self):
        """Verify pack → unpack preserves duty values (backlog LS-7)."""
        builder = UARTPacketBuilder()
        original = [100, 250, 512, 800, 1023]
        packet = builder.pack_pwm_packet(original)

        # Manual unpack matching RP2040 parser logic
        payload = packet[2:12]
        unpacked = []
        for i in range(5):
            val = payload[i * 2] | (payload[i * 2 + 1] << 8)
            unpacked.append(val)
        assert unpacked == original

    def test_duty_clamping(self):
        builder = UARTPacketBuilder()
        duties = [0, 0, 0, 0, 2000]  # 2000 > 1023
        packet = builder.pack_pwm_packet(duties)
        # Last duty should be clamped to 1023
        payload = packet[2:12]
        last_val = payload[8] | (payload[9] << 8)
        assert last_val == 1023

    def test_wrong_duty_count_raises(self):
        builder = UARTPacketBuilder()
        with pytest.raises(ValueError):
            builder.pack_pwm_packet([100, 200, 300])

    def test_ack_unpack_valid(self):
        # Build a valid ACK: [0xAA, 0x06, CRC8(0x06)]
        status = 0x06
        crc = calculate_crc8(bytes([status]))
        ack_data = bytes([0xAA, status, crc])
        result = UARTPacketBuilder.unpack_ack(ack_data)
        assert result == 0x06

    def test_ack_unpack_invalid_crc(self):
        ack_data = bytes([0xAA, 0x06, 0xFF])  # wrong CRC
        result = UARTPacketBuilder.unpack_ack(ack_data)
        assert result is None

    def test_ack_unpack_too_short(self):
        result = UARTPacketBuilder.unpack_ack(bytes([0xAA]))
        assert result is None


# =================================================================
# Dry-Run Mode
# =================================================================

class TestDryRunMode:

    @pytest.mark.asyncio
    async def test_dry_run_does_not_write_serial(self):
        """In dry-run mode, UART packets must NOT be sent."""
        from unittest.mock import MagicMock

        mock_serial = MagicMock()
        loop = LevitationCoreLoop(serial_port=mock_serial, dry_run=True)

        async def fake_sensors():
            return {
                "hall_z_mm": 30.0,
                "tof_xyz": (0.0, 0.0, 30.0),
            }

        await loop.run(read_sensors_callback=fake_sensors, max_iterations=3)

        # Serial.write should never have been called
        mock_serial.write.assert_not_called()

    @pytest.mark.asyncio
    async def test_non_dry_run_writes_serial(self):
        """Without dry-run, packets must be sent to serial."""
        from unittest.mock import MagicMock

        mock_serial = MagicMock()
        loop = LevitationCoreLoop(serial_port=mock_serial, dry_run=False)

        async def fake_sensors():
            return {
                "hall_z_mm": 30.0,
                "tof_xyz": (0.0, 0.0, 30.0),
            }

        await loop.run(read_sensors_callback=fake_sensors, max_iterations=3)

        # Serial.write should have been called at least once
        assert mock_serial.write.call_count >= 1
