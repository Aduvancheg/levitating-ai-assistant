"""
test_core_loop_regressions.py — Regression tests for project review package P1.

Each test targets one finding from the review and fails on the
pre-refactoring code:
  - C1: PID dt must be the period between iterations, not the iteration time.
  - C2: realistic motion must not trigger PHYSICAL_LOCK; lock is resettable.
  - C3: fusion weights are continuous at the 49 / 61 mm zone edges.
  - C4: thermal throttling accumulates and recovers.
  - C6: weights are chosen from the previous fused estimate.
  - S2: missing sensor data leads to safe state, never to a fake 0 mm.

Timing tests use a virtual clock: `time.perf_counter` and `asyncio.sleep`
are replaced so that time advances only during the read (50 µs) and the
pacing sleep — like on the real target.
"""

import asyncio
from unittest.mock import MagicMock

import pytest

import brain_rpi5.src.core_loop as core_loop_mod
from brain_rpi5.src.core_loop import (
    ComplementaryFilter,
    HoldLastValid,
    LevitationCoreLoop,
)

READ_LATENCY_S = 0.00005  # 50 µs spent inside the sensor read


class VirtualClock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def clock(monkeypatch):
    clk = VirtualClock()

    async def fake_sleep(seconds, *args, **kwargs):
        clk.t += max(0.0, seconds)

    monkeypatch.setattr(core_loop_mod.time, "perf_counter", clk)
    monkeypatch.setattr(core_loop_mod.asyncio, "sleep", fake_sleep)
    monkeypatch.setattr(LevitationCoreLoop, "setup_posix_realtime", lambda self: None)
    return clk


def make_sensor(clock, z_values=None, z=30.0):
    """Sensor callback that advances the clock by the read latency."""
    seq = list(z_values) if z_values is not None else None

    async def read():
        clock.t += READ_LATENCY_S
        value = seq.pop(0) if seq else z
        return {"hall_z_mm": value, "tof_xyz": (0.0, 0.0, value)}

    return read


# =================================================================
# C1 — dt between iterations
# =================================================================

class TestLoopDt:

    @pytest.mark.asyncio
    async def test_pid_receives_period_between_iterations(self, clock):
        loop = LevitationCoreLoop(dry_run=True)
        dts = []
        original = loop.pid.compute

        def spy(setpoint, measurement, dt):
            dts.append(dt)
            return original(setpoint, measurement, dt)

        loop.pid.compute = spy
        await loop.run(make_sensor(clock), max_iterations=20)

        period = LevitationCoreLoop.TARGET_DT_S
        assert len(dts) == 20
        for dt in dts[1:]:
            # Old code produced ~READ_LATENCY_S here (25x too small).
            assert abs(dt - period) < 1e-9

    @pytest.mark.asyncio
    async def test_pid_dt_is_clamped_after_stall(self, clock):
        loop = LevitationCoreLoop(dry_run=True)
        dts = []
        original = loop.pid.compute

        def spy(setpoint, measurement, dt):
            dts.append(dt)
            return original(setpoint, measurement, dt)

        loop.pid.compute = spy
        sensor = make_sensor(clock)
        await loop.run(sensor, max_iterations=1)
        clock.t += 1.0  # 1 s scheduling stall
        await loop.run(sensor, max_iterations=1)

        assert loop.last_dt_s > 0.9
        assert dts[-1] == LevitationCoreLoop.MAX_PID_DT_S


# =================================================================
# C2 — PHYSICAL_LOCK false positives and reset
# =================================================================

class TestPhysicalLock:

    @pytest.mark.asyncio
    async def test_realistic_motion_does_not_lock(self, clock):
        """100 mm/s is ordinary motion; old dt bug saw it as 2500 mm/s."""
        step = 100.0 * LevitationCoreLoop.TARGET_DT_S  # 0.125 mm per tick
        z_values = [20.0 + i * step for i in range(100)]
        loop = LevitationCoreLoop(dry_run=True)
        await loop.run(make_sensor(clock, z_values), max_iterations=100)
        assert loop.state_physical_lock is False

    @pytest.mark.asyncio
    async def test_impossible_jump_still_locks(self, clock):
        loop = LevitationCoreLoop(dry_run=True)
        await loop.run(make_sensor(clock, [30.0, 30.0, 60.0]), max_iterations=3)
        assert loop.state_physical_lock is True
        assert loop._last_duties == [0, 0, 0, 0, 0]

    def test_reset_requires_confirmation(self):
        loop = LevitationCoreLoop(dry_run=True)
        loop.state_physical_lock = True
        loop._prev_z = 42.0

        assert loop.reset_physical_lock() is False
        assert loop.state_physical_lock is True

        assert loop.reset_physical_lock(confirm=True) is True
        assert loop.state_physical_lock is False
        assert loop._prev_z is None

    @pytest.mark.asyncio
    async def test_loop_resumes_after_reset(self, clock):
        loop = LevitationCoreLoop(dry_run=True)
        await loop.run(make_sensor(clock, [30.0, 90.0]), max_iterations=2)
        assert loop.state_physical_lock is True

        loop.reset_physical_lock(confirm=True)
        await loop.run(make_sensor(clock, z=20.0), max_iterations=5)
        assert loop.state_physical_lock is False
        assert any(d > 0 for d in loop._last_duties)


# =================================================================
# C3 / C6 — fusion weights
# =================================================================

class TestFusionWeights:

    def test_weights_continuous_at_hall_edge(self):
        filt = ComplementaryFilter()
        _, tof_w = filt.compute_weights(49.0 + 1e-6)
        assert tof_w < 1e-5  # old code: jumped to ~0.047

    def test_weights_continuous_at_tof_edge(self):
        filt = ComplementaryFilter()
        _, tof_w = filt.compute_weights(61.0 - 1e-6)
        assert tof_w > 1.0 - 1e-5  # old code: stopped at ~0.953

    def test_weights_monotonic_and_without_steps(self):
        filt = ComplementaryFilter()
        prev = 0.0
        z = 48.0
        while z <= 62.0:
            _, tof_w = filt.compute_weights(z)
            assert tof_w >= prev - 1e-12
            assert tof_w - prev < 0.01  # no step larger than 1% per 0.01 mm
            prev = tof_w
            z += 0.01

    def test_saturated_hall_does_not_bias_weights(self):
        """
        Sphere at 70 mm: Hall is saturated near its limit (50 mm),
        ToF reads correctly. The old mean-based reference got stuck at
        ~68.5 mm; the previous-fused reference converges to the ToF value.
        """
        filt = ComplementaryFilter()
        fused = None
        for _ in range(20):
            fused = filt.fuse(hall_z_mm=50.0, tof_z_mm=70.0)
        assert abs(fused - 70.0) < 0.01


# =================================================================
# S2 — invalid sensor data
# =================================================================

class TestInvalidSensorData:

    def test_hold_last_valid_bridges_then_gives_up(self):
        hold = HoldLastValid(max_stale=3)
        assert hold.update(None) is None  # nothing to hold yet
        assert hold.update(10.0) == 10.0
        for _ in range(3):
            assert hold.update(float("nan")) == 10.0
        assert hold.update(None) is None

    def test_fuse_uses_remaining_sensor(self):
        filt = ComplementaryFilter()
        assert filt.fuse(hall_z_mm=None, tof_z_mm=40.0) == 40.0
        filt2 = ComplementaryFilter()
        assert filt2.fuse(hall_z_mm=25.0, tof_z_mm=float("nan")) == 25.0

    def test_fuse_without_any_valid_data_returns_none(self):
        """Old code returned 0.0 mm here, making the PID push full power."""
        filt = ComplementaryFilter()
        assert filt.fuse(float("nan"), float("nan")) is None

    def test_fuse_gives_up_after_stale_window(self):
        filt = ComplementaryFilter(max_stale_frames=5)
        filt.fuse(30.0, 30.0)
        for _ in range(5):
            assert filt.fuse(None, None) == 30.0
        assert filt.fuse(None, None) is None

    @pytest.mark.asyncio
    async def test_missing_data_from_start_enters_safe_state(self, clock):
        mock_serial = MagicMock()
        loop = LevitationCoreLoop(serial_port=mock_serial, dry_run=False)

        async def dead_sensors():
            clock.t += READ_LATENCY_S
            return {}

        await loop.run(dead_sensors, max_iterations=5)

        assert loop.state_sensor_fault is True
        assert loop._last_duties == [0, 0, 0, 0, 0]
        # Every packet sent must be the all-zero command
        zero_packet = loop.packer.pack_pwm_packet([0, 0, 0, 0, 0])
        for call in mock_serial.write.call_args_list:
            assert call.args[0] == zero_packet

    @pytest.mark.asyncio
    async def test_short_dropout_bridged_long_dropout_faults_then_recovers(self, clock):
        loop = LevitationCoreLoop(dry_run=True)
        script = (
            [{"hall_z_mm": 30.0, "tof_xyz": (0.0, 0.0, 30.0)}] * 3
            + [{"hall_z_mm": None, "tof_xyz": None}] * 3
        )

        async def scripted():
            clock.t += READ_LATENCY_S
            return script.pop(0)

        await loop.run(scripted, max_iterations=6)
        assert loop.state_sensor_fault is False  # 3 frames bridged

        script.extend([{"hall_z_mm": float("nan"), "tof_xyz": None}] * 10)
        await loop.run(scripted, max_iterations=10)
        assert loop.state_sensor_fault is True
        assert loop._last_duties == [0, 0, 0, 0, 0]

        await loop.run(make_sensor(clock, z=30.0), max_iterations=3)
        assert loop.state_sensor_fault is False
        assert loop.state_physical_lock is False

    @pytest.mark.asyncio
    async def test_sensor_exception_is_bridged_then_faults(self, clock):
        loop = LevitationCoreLoop(dry_run=True)
        await loop.run(make_sensor(clock), max_iterations=2)

        async def broken():
            clock.t += READ_LATENCY_S
            raise IOError("I2C bus stuck")

        await loop.run(broken, max_iterations=10)
        assert loop.state_sensor_fault is True
        assert loop._last_duties == [0, 0, 0, 0, 0]


# =================================================================
# C4 — thermal throttling accumulates
# =================================================================

class TestThermalOffset:

    @pytest.mark.asyncio
    async def test_throttling_accumulates_and_recovers(self, clock, monkeypatch):
        loop = LevitationCoreLoop(dry_run=True, setpoint=(0.0, 0.0, 30.0))
        throttling = {"on": True}
        monkeypatch.setattr(loop.thermal_model, "update", lambda dt, d: throttling["on"])

        await loop.run(make_sensor(clock), max_iterations=100)
        # Old code: always 29.99 (offset re-applied to a fresh copy each tick)
        assert loop.active_setpoint[2] == pytest.approx(29.0, abs=1e-6)
        assert loop.setpoint[2] == 30.0  # user setpoint untouched

        throttling["on"] = False
        await loop.run(make_sensor(clock), max_iterations=60)
        assert loop.active_setpoint[2] == pytest.approx(29.6, abs=1e-6)

        await loop.run(make_sensor(clock), max_iterations=100)
        assert loop.thermal_z_offset_mm == 0.0

    @pytest.mark.asyncio
    async def test_throttling_respects_min_height(self, clock, monkeypatch):
        loop = LevitationCoreLoop(dry_run=True, setpoint=(0.0, 0.0, 5.3))
        monkeypatch.setattr(loop.thermal_model, "update", lambda dt, d: True)

        await loop.run(make_sensor(clock, z=5.3), max_iterations=100)
        assert loop.active_setpoint[2] == pytest.approx(
            LevitationCoreLoop.THERMAL_MIN_Z_MM, abs=1e-9
        )
