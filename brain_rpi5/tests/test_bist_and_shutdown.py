"""
test_bist_and_shutdown.py — Unit tests for Phase 4 BIST and Safe Shutdown logic.

Covers:
- TEST-6: BIST Diagnostics failures, retry, bypass logic
- TEST-7: Safe Shutdown (Turn Off) pipeline
- BIST-HW-05: Lenz Law & Slew Rate Validation Test
"""

import asyncio
import pytest
from brain_rpi5.src.main_server import (
    system_state,
    run_single_bist_test,
    execute_bist_pipeline,
    execute_shutdown_pipeline,
    bist_retry,
    bist_bypass,
    BypassRequest,
)

@pytest.fixture(autouse=True)
def reset_state():
    """Reset the global system state before each test."""
    system_state.bist_unlocked = False
    system_state.bist_bypass_active = False
    system_state.simulate_fault_on_test_id = None
    system_state.active_pipeline = None
    system_state.pipeline_status = "Idle"
    system_state.safe_to_unplug = False
    system_state.bist_results = {
        "BIST-HW-01": "UNTESTED",
        "BIST-HW-02": "UNTESTED",
        "BIST-HW-03": "UNTESTED",
        "BIST-HW-04": "UNTESTED",
        "BIST-HW-05": "UNTESTED"
    }
    system_state.bist_logs = {
        "BIST-HW-01": "Тест не запускался.",
        "BIST-HW-02": "Тест не запускался.",
        "BIST-HW-03": "Тест не запускался.",
        "BIST-HW-04": "Тест не запускался.",
        "BIST-HW-05": "Тест не запускался."
    }

@pytest.mark.asyncio
async def test_bist_pipeline_success():
    """TEST-6: Happy path for BIST pipeline (all 5 HW tests)."""
    system_state.simulate_fault_on_test_id = None
    
    await execute_bist_pipeline()
    
    assert system_state.bist_unlocked is True
    assert all(res == "PASS" for res in system_state.bist_results.values())
    assert system_state.pipeline_status == "System Unlocked (All Tests PASS)"

@pytest.mark.asyncio
async def test_bist_pipeline_failure():
    """TEST-6: Fault simulation blocks unlock."""
    system_state.simulate_fault_on_test_id = "BIST-HW-02"
    
    await execute_bist_pipeline()
    
    assert system_state.bist_unlocked is False
    assert system_state.bist_results["BIST-HW-01"] == "PASS"
    assert system_state.bist_results["BIST-HW-02"] == "FAILED"
    # Execution stops on first failure
    assert system_state.bist_results["BIST-HW-03"] == "UNTESTED"

@pytest.mark.asyncio
async def test_bist_retry():
    """TEST-6: Test retry endpoint."""
    system_state.simulate_fault_on_test_id = "BIST-HW-01"
    await execute_bist_pipeline()
    assert system_state.bist_unlocked is False
    
    # Remove fault simulation
    system_state.simulate_fault_on_test_id = None
    
    # Call retry API logic directly
    await bist_retry({"test_id": "BIST-HW-01"})
    
    assert system_state.bist_results["BIST-HW-01"] == "PASS"
    # But because 02, 03, 04, 05 are UNTESTED, unlocked is still False
    assert system_state.bist_unlocked is False

@pytest.mark.asyncio
async def test_bist_bypass():
    """TEST-6: Bypass overrides interlock."""
    system_state.bist_results["BIST-HW-01"] = "FAILED"
    
    await bist_bypass(BypassRequest(override=True))
    
    assert system_state.bist_unlocked is True
    assert system_state.bist_bypass_active is True
    assert system_state.bist_results["BIST-HW-01"] == "BYPASSED"
    
    # Turn off bypass
    await bist_bypass(BypassRequest(override=False))
    
    assert system_state.bist_unlocked is False
    assert system_state.bist_bypass_active is False
    assert system_state.bist_results["BIST-HW-01"] == "UNTESTED"

@pytest.mark.asyncio
async def test_safe_shutdown_pipeline():
    """TEST-7: Safe Shutdown (UC-9) Pipeline Execution."""
    system_state.target_z = 25.0
    system_state.pwm_channels = [100, 100, 100, 100, 100]
    system_state.agent_relay_status = "Isolated"
    
    await execute_shutdown_pipeline()
    
    assert system_state.target_z == 0.0
    assert system_state.z == 0.0
    assert system_state.pwm_channels == [0, 0, 0, 0, 0]
    assert system_state.agent_relay_status == "Connected"
    assert system_state.safe_to_unplug is True
    assert system_state.bist_unlocked is False
    assert system_state.active_pipeline == "SAFE_SHUTDOWN"

# =====================================================================
# BIST-HW-05: Lenz Law & Slew Rate Validation Tests
# =====================================================================

@pytest.mark.asyncio
async def test_bist_hw05_lenz_test_success():
    """BIST-HW-05: Lenz Law test passes with valid slew-rate limiter."""
    system_state.simulate_fault_on_test_id = None
    
    success = await run_single_bist_test("BIST-HW-05")
    
    assert success is True
    assert system_state.bist_results["BIST-HW-05"] == "PASS"
    assert "Lenz Law" in system_state.bist_logs["BIST-HW-05"]
    assert "45.0" in system_state.bist_logs["BIST-HW-05"]
    assert "Ramp-down: 30" in system_state.bist_logs["BIST-HW-05"]

@pytest.mark.asyncio
async def test_bist_hw05_lenz_test_failure_simulation():
    """BIST-HW-05: Fault simulation triggers FAILED status."""
    system_state.simulate_fault_on_test_id = "BIST-HW-05"
    
    success = await run_single_bist_test("BIST-HW-05")
    
    assert success is False
    assert system_state.bist_results["BIST-HW-05"] == "FAILED"
    assert "Lenz Protection" in system_state.bist_logs["BIST-HW-05"]
    assert "SMBJ12A" in system_state.bist_logs["BIST-HW-05"]

@pytest.mark.asyncio
async def test_bist_pipeline_with_hw05():
    """Full BIST pipeline with all 5 tests (HW-01 through HW-05) passes and unlocks."""
    system_state.simulate_fault_on_test_id = None
    
    await execute_bist_pipeline()
    
    # All 5 hardware tests must PASS
    assert system_state.bist_results["BIST-HW-01"] == "PASS"
    assert system_state.bist_results["BIST-HW-02"] == "PASS"
    assert system_state.bist_results["BIST-HW-03"] == "PASS"
    assert system_state.bist_results["BIST-HW-04"] == "PASS"
    assert system_state.bist_results["BIST-HW-05"] == "PASS"
    
    # System must be fully unlocked
    assert system_state.bist_unlocked is True
    assert system_state.pipeline_status == "System Unlocked (All Tests PASS)"
    
    # Pipeline steps should include LENZ step
    step_ids = [s["step_id"] for s in system_state.pipeline_steps]
    assert "BIST-05-LENZ" in step_ids
