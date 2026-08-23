import pytest
import asyncio
from brain_rpi5.src.main_server import (
    system_state,
    execute_takeoff_pipeline,
    execute_landing_pipeline,
)

@pytest.mark.asyncio
async def test_takeoff_scenario():
    """
    TEST-2: Unity E2E Scenarios (Takeoff)
    Simulate Takeoff -> Calibration.
    """
    system_state.target_z = 0.0
    
    # Run the takeoff pipeline
    task = asyncio.create_task(execute_takeoff_pipeline())
    
    # Let it yield
    await asyncio.sleep(0.5)
    
    assert system_state.active_pipeline == "SMOOTH_TAKEOFF"
    
    # Cancel the task so we don't wait 10 seconds
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
        
    system_state.active_pipeline = None

@pytest.mark.asyncio
async def test_safe_landing_scenario():
    """
    TEST-2: Unity E2E Scenarios (Safe Landing)
    Simulate Safe Landing.
    """
    system_state.target_z = 50.0
    
    task = asyncio.create_task(execute_landing_pipeline())
    
    # Yield
    await asyncio.sleep(0.5)
    
    assert system_state.active_pipeline == "SAFE_LANDING"
    
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
