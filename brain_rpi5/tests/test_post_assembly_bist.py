"""
test_post_assembly_bist.py — Unit & Integration Tests for Post-Assembly BIST Suite v1.0.

Covers 5 async hardware diagnostic scenarios from post_assembly_testing_guide_v1.md:
  - TEST-PA-01: Channel Isolation (Hall sensor cross-talk)
  - TEST-PA-02: Winding Symmetry & Polarity (12-winding 3x4, all North)
  - TEST-PA-03: ToF Optical Path Clearance (X = +45.0 mm)
  - TEST-PA-04: EMC Ground Bounce Stress (Star ground RMS)
  - TEST-PA-05: Thermal Airflow Clearance (3-quadrant gill fans)
"""

import asyncio
import pytest

from brain_rpi5.src.main_server import (
    system_state,
    run_post_assembly_test,
    execute_post_assembly_bist_pipeline,
)


@pytest.fixture(autouse=True)
def reset_state():
    """Reset post-assembly BIST state before each test."""
    system_state.active_pipeline = None
    system_state.pipeline_status = "Idle"
    system_state.pipeline_steps = []
    system_state.bist_unlocked = True
    system_state.pwm_channels = [0, 0, 0, 0, 0]
    system_state.coil_temp_model = 25.0
    system_state.post_assembly_results = {
        "TEST-PA-01": "UNTESTED",
        "TEST-PA-02": "UNTESTED",
        "TEST-PA-03": "UNTESTED",
        "TEST-PA-04": "UNTESTED",
        "TEST-PA-05": "UNTESTED",
    }
    system_state.post_assembly_logs = {
        "TEST-PA-01": "Тест не запускался.",
        "TEST-PA-02": "Тест не запускался.",
        "TEST-PA-03": "Тест не запускался.",
        "TEST-PA-04": "Тест не запускался.",
        "TEST-PA-05": "Тест не запускался.",
    }
    yield


# =====================================================================
# Individual Test Scenarios
# =====================================================================

@pytest.mark.asyncio
async def test_pa_01_channel_isolation():
    """TEST-PA-01: Isolated PWM sweep verifies dV >= 0.12V on active channel,
    dV <= 0.03V on inactive channels. No cross-talk between MOSFET gates.
    """
    success = await run_post_assembly_test("TEST-PA-01")

    assert success is True
    assert system_state.post_assembly_results["TEST-PA-01"] == "PASS"
    assert "Изоляция" in system_state.post_assembly_logs["TEST-PA-01"]
    assert "dV[active]" in system_state.post_assembly_logs["TEST-PA-01"]
    # PWM channels should be zeroed after test
    assert system_state.pwm_channels == [0, 0, 0, 0, 0]


@pytest.mark.asyncio
async def test_pa_02_winding_symmetry():
    """TEST-PA-02: All 5 coils generate North pole (dV > 0),
    spread between coils <= 50 mV (90% symmetry).
    """
    success = await run_post_assembly_test("TEST-PA-02")

    assert success is True
    assert system_state.post_assembly_results["TEST-PA-02"] == "PASS"
    log = system_state.post_assembly_logs["TEST-PA-02"]
    assert "North" in log or "SUCCESS" in log
    assert "50мВ" in log or "30мВ" in log  # Spread within threshold


@pytest.mark.asyncio
async def test_pa_03_tof_optical_clearance():
    """TEST-PA-03: ToF optical window at X = +45.0mm is unobstructed.
    All 64 zones report min >= 15.0mm, std <= 3.0mm.
    """
    success = await run_post_assembly_test("TEST-PA-03")

    assert success is True
    assert system_state.post_assembly_results["TEST-PA-03"] == "PASS"
    log = system_state.post_assembly_logs["TEST-PA-03"]
    assert "ToF" in log or "прозрачно" in log


@pytest.mark.asyncio
async def test_pa_04_emi_ground_bounce():
    """TEST-PA-04: Hall sensor RMS noise <= 12.0 mV at full PWM=460 stress.
    Star ground wiring integrity confirmed.
    """
    success = await run_post_assembly_test("TEST-PA-04")

    assert success is True
    assert system_state.post_assembly_results["TEST-PA-04"] == "PASS"
    log = system_state.post_assembly_logs["TEST-PA-04"]
    assert "RMS" in log
    assert "12.0" in log or "8.4" in log
    # PWM should be zeroed after stress test
    assert system_state.pwm_channels == [0, 0, 0, 0, 0]


@pytest.mark.asyncio
async def test_pa_05_thermal_airflow():
    """TEST-PA-05: Heat rate <= 0.25°C/s at 30% PWM with active fan cooling.
    3-quadrant gill airflow verified.
    """
    success = await run_post_assembly_test("TEST-PA-05")

    assert success is True
    assert system_state.post_assembly_results["TEST-PA-05"] == "PASS"
    log = system_state.post_assembly_logs["TEST-PA-05"]
    assert "0.25" in log or "Продуваемость" in log
    # PWM should be zeroed after thermal test
    assert system_state.pwm_channels == [0, 0, 0, 0, 0]


# =====================================================================
# Full Pipeline Integration
# =====================================================================

@pytest.mark.asyncio
async def test_full_post_assembly_pipeline_all_pass():
    """Full Post-Assembly BIST pipeline: all 5 tests pass sequentially."""
    await execute_post_assembly_bist_pipeline()

    # All 5 tests should PASS
    for tid in ["TEST-PA-01", "TEST-PA-02", "TEST-PA-03", "TEST-PA-04", "TEST-PA-05"]:
        assert system_state.post_assembly_results[tid] == "PASS", \
            f"{tid} expected PASS, got {system_state.post_assembly_results[tid]}"

    # Pipeline should complete and clean up
    assert system_state.active_pipeline is None
    assert "ALL PASS" in system_state.pipeline_status

    # Pipeline steps should reflect completion
    assert len(system_state.pipeline_steps) == 5
    for step in system_state.pipeline_steps:
        assert step["status"] == "DONE"


@pytest.mark.asyncio
async def test_pipeline_steps_have_correct_ids():
    """Pipeline step IDs match the expected PA-01 through PA-05."""
    await execute_post_assembly_bist_pipeline()

    step_ids = [s["step_id"] for s in system_state.pipeline_steps]
    assert "PA-01" in step_ids
    assert "PA-02" in step_ids
    assert "PA-03" in step_ids
    assert "PA-04" in step_ids
    assert "PA-05" in step_ids


@pytest.mark.asyncio
async def test_pipeline_sets_active_pipeline_during_execution():
    """Pipeline correctly sets active_pipeline to POST_ASSEMBLY_BIST."""
    # Start pipeline as background task
    task = asyncio.create_task(execute_post_assembly_bist_pipeline())

    # Give it a moment to start
    await asyncio.sleep(0.1)
    assert system_state.active_pipeline == "POST_ASSEMBLY_BIST"

    # Wait for completion
    await task
    assert system_state.active_pipeline is None


@pytest.mark.asyncio
async def test_post_assembly_does_not_affect_hw_bist():
    """Post-Assembly BIST results are independent from HW BIST results."""
    await execute_post_assembly_bist_pipeline()

    # HW BIST should remain untouched
    assert system_state.bist_results["BIST-HW-01"] == "UNTESTED"
    assert system_state.bist_results["BIST-HW-05"] == "UNTESTED"

    # Post-Assembly should all be PASS
    assert system_state.post_assembly_results["TEST-PA-01"] == "PASS"
