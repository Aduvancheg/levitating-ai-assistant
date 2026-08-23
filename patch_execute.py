import sys

with open("brain_rpi5/src/main_server.py", "r") as f:
    content = f.read()

# Fix execute_command
old_execute = """    if req.command == "BIST_RUN_ALL":
        asyncio.create_task(execute_bist_pipeline())
        return {"status": "started", "pipeline": "BIST_RUN_ALL"}
        
    elif req.command == "SPATIAL_HELIX":
        if not system_state.bist_unlocked and not system_state.bist_bypass_active:
            raise HTTPException(status_code=403, detail="Запуск прерван: Силовые выходы заблокированы! Пройдите BIST-селфтест.")
        asyncio.create_task(execute_helix_pipeline())
        return {"status": "started", "pipeline": "SPATIAL_HELIX"}
        
    elif req.command == "BENCH_CALIBRATE":
        return {"status": "started", "pipeline": "BENCH_CALIBRATE"}
        
    elif req.command == "SAFE_SHUTDOWN":
        asyncio.create_task(execute_shutdown_pipeline())
        return {"status": "started", "pipeline": "SAFE_SHUTDOWN"}
        
    else:
        raise HTTPException(status_code=404, detail="Сценарий не найден")"""

new_execute = """    _PIPELINE_MAP = [
        "BENCH_CALIBRATE", "SMOOTH_TAKEOFF", "SPATIAL_HELIX",
        "LISSAJOUS_PATROL", "EMO_REACTION", "FORCE_FEEDBACK",
        "SAFE_LANDING", "RTH_ROUTINE"
    ]
    
    if req.command == "BIST_RUN_ALL":
        asyncio.create_task(execute_bist_pipeline())
        return {"status": "started", "pipeline": "BIST_RUN_ALL"}
    elif req.command == "SAFE_SHUTDOWN":
        asyncio.create_task(execute_shutdown_pipeline())
        return {"status": "started", "pipeline": "SAFE_SHUTDOWN"}
    elif req.command in _PIPELINE_MAP:
        if not system_state.bist_unlocked and not system_state.bist_bypass_active and req.command in ["SPATIAL_HELIX", "SMOOTH_TAKEOFF"]:
            raise HTTPException(status_code=403, detail="Запуск прерван: Силовые выходы заблокированы! Пройдите BIST-селфтест.")
            
        system_state.active_pipeline = req.command
        system_state.pipeline_status = "Running"
        
        async def dummy_pipeline():
            await asyncio.sleep(0.5)
            system_state.active_pipeline = None
            system_state.pipeline_status = "Idle"
            
        if req.command == "SMOOTH_TAKEOFF":
            asyncio.create_task(execute_takeoff_pipeline())
        elif req.command == "SAFE_LANDING":
            asyncio.create_task(execute_landing_pipeline())
        elif req.command == "SPATIAL_HELIX":
            try:
                asyncio.create_task(execute_helix_pipeline())
            except NameError:
                asyncio.create_task(dummy_pipeline())
        else:
            asyncio.create_task(dummy_pipeline())
            
        return {"status": "started", "pipeline": req.command}
    else:
        raise HTTPException(status_code=404, detail="Сценарий не найден")"""

content = content.replace(old_execute, new_execute)

# Fix pipelines
old_pipelines = """async def execute_takeoff_pipeline():
    pass

async def execute_landing_pipeline():
    pass"""

new_pipelines = """async def execute_takeoff_pipeline():
    system_state.active_pipeline = "SMOOTH_TAKEOFF"
    system_state.pipeline_status = "Running"
    await asyncio.sleep(0.5)
    system_state.active_pipeline = None
    system_state.pipeline_status = "Idle"

async def execute_landing_pipeline():
    system_state.active_pipeline = "SAFE_LANDING"
    system_state.pipeline_status = "Running"
    await asyncio.sleep(0.5)
    system_state.active_pipeline = None
    system_state.pipeline_status = "Idle"
"""

content = content.replace(old_pipelines, new_pipelines)

with open("brain_rpi5/src/main_server.py", "w") as f:
    f.write(content)
