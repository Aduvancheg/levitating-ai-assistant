import re

with open('brain_rpi5/src/main_server.py', 'r') as f:
    orig_content = f.read()

with open('requirements/main_server-v8.py', 'r') as f:
    v8_content = f.read()

# Extract pipelines from orig_content (from execute_calibration_pipeline to _PIPELINE_MAP)
pipelines_match = re.search(r'(async def execute_calibration_pipeline.*?)_PIPELINE_MAP = \{', orig_content, re.DOTALL)
if pipelines_match:
    pipelines_code = pipelines_match.group(1)
    
    # Replace the single helix pipeline in v8_content with all pipelines
    v8_content = re.sub(r'async def execute_helix_pipeline.*?(?=@app\.on_event)', pipelines_code + '\n_PIPELINE_MAP = {\n' + 
    '    "BENCH_CALIBRATE": execute_calibration_pipeline,\n' +
    '    "SMOOTH_TAKEOFF": execute_takeoff_pipeline,\n' +
    '    "SPATIAL_HELIX": execute_helix_pipeline,\n' +
    '    "LISSAJOUS_PATROL": execute_lissajous_pipeline,\n' +
    '    "EMO_REACTION": execute_emotional_pipeline,\n' +
    '    "FORCE_FEEDBACK": execute_force_pipeline,\n' +
    '    "SAFE_LANDING": execute_landing_pipeline,\n' +
    '    "RTH_ROUTINE": execute_rth_pipeline,\n' +
    '    "SAFE_SHUTDOWN": execute_shutdown_pipeline,\n' +
    '}\n\n', v8_content, flags=re.DOTALL)

# Also fix the `NetworkManager` logic
v8_content = "from .network_manager import NetworkManager\nfrom contextlib import asynccontextmanager\n" + v8_content

# Add lifespan
lifespan_code = """
@asynccontextmanager
async def lifespan(app: FastAPI):
    await NetworkManager.auto_configure()
    task_core = asyncio.create_task(core_loop_860hz())
    task_telemetry = asyncio.create_task(telemetry_broadcaster_10hz())
    try:
        yield
    finally:
        task_core.cancel()
        task_telemetry.cancel()

"""

v8_content = re.sub(r'app = FastAPI\(', lifespan_code + 'app = FastAPI(', v8_content)
v8_content = v8_content.replace('version="8.0.0"\n)', 'version="8.0.0",\n    lifespan=lifespan\n)')
v8_content = re.sub(r'@app\.on_event\("startup"\)\nasync def startup_event\(\):\n.*?logger\.info\(".*?запущены\."\)', '', v8_content, flags=re.DOTALL)

# Add wifi endpoints
wifi_code = """
@app.get("/api/wifi/scan")
async def wifi_scan():
    return {
        "status": "SUCCESS",
        "networks": [
            {"ssid": "MyHomeWiFi", "signal": -50, "secure": True},
            {"ssid": "Lab_Antigravity", "signal": -35, "secure": True},
            {"ssid": "Guest_Net", "signal": -80, "secure": False},
        ],
    }

class WiFiConnectRequest(BaseModel):
    ssid: str
    password: str

@app.post("/api/wifi/connect")
async def wifi_connect(req: WiFiConnectRequest):
    asyncio.create_task(NetworkManager.handover_network(req.ssid, req.password))
    system_state.base_wifi_ssid = req.ssid
    system_state.base_wifi_mode = "STA"
    return {"status": "SUCCESS"}
"""

v8_content = v8_content.replace('@app.websocket("/ws/telemetry")', wifi_code + '\n@app.websocket("/ws/telemetry")')

# Change execute_command to use _PIPELINE_MAP
exec_code = """
@app.post("/api/manage/execute")
async def execute_command(req: CommandRequest):
    if system_state.active_pipeline is not None and system_state.active_pipeline != "SAFE_SHUTDOWN":
        raise HTTPException(
            status_code=400, 
            detail=f"Cannot start '{req.command}', pipeline '{system_state.active_pipeline}' is running"
        )
    
    if req.command == "BIST_RUN_ALL":
        asyncio.create_task(execute_bist_pipeline())
        return {"status": "started", "pipeline": "BIST_RUN_ALL"}
        
    pipeline_fn = _PIPELINE_MAP.get(req.command)
    if pipeline_fn is None:
        raise HTTPException(status_code=404, detail="Сценарий не найден")
        
    if req.command == "SPATIAL_HELIX":
        if not system_state.bist_unlocked and not system_state.bist_bypass_active:
            raise HTTPException(status_code=403, detail="Запуск прерван: Силовые выходы заблокированы!")
            
    asyncio.create_task(pipeline_fn())
    return {"status": "started", "pipeline": req.command}
"""
v8_content = re.sub(r'@app\.post\("/api/manage/execute"\).*?raise HTTPException\(status_code=404, detail="Сценарий не найден"\)', exec_code.strip(), v8_content, flags=re.DOTALL)

with open('brain_rpi5/src/main_server.py', 'w') as f:
    f.write(v8_content)

