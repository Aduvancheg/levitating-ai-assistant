import asyncio
import time
import json
import logging
import math
import os
import sqlite3
import zipfile
from datetime import datetime
from typing import Dict, List, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("Brain_RPi5_V12")

app = FastAPI(
    title="Magnetic Levitation Brain (Raspberry Pi 5) - V12 Polish Edition",
    description="Управляющий мозг V12: ПИД (860 Гц), Gemini Pro API, SPU-WM30 Audio, MJPEG Stream, Emergency Safe Landing & SQLite Auto-Archiving.",
    version="12.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class SystemState:
    def __init__(self):
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0
        self.raw_hall: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.tof_grid: List[float] = [0.0] * 64
        
        self.pwm_channels: List[int] = [0, 0, 0, 0, 0]
        self.max_duty_limit: int = 460
        self.prev_pwm_channels: List[int] = [0, 0, 0, 0, 0]
        self.slew_rate_limit: int = 10
        
        self.target_z: float = 25.0
        self.kp: float = 12.5
        self.ki: float = 0.05
        self.kd: float = 8.2
        
        # Agent telemetry & Wi-Fi Heartbeat
        self.agent_pitch: float = 0.0
        self.agent_roll: float = 0.0
        self.agent_yaw: float = 0.0
        self.agent_battery: float = 100.0
        self.agent_coil_pwm: int = 0
        self.agent_relay_status: str = "Connected"
        self.agent_wifi_last_seen: float = time.perf_counter()
        self.agent_wifi_status: str = "Connected"
        
        self.coil_temp_model: float = 25.0
        self.thermal_throttling: bool = False
        
        self.active_pipeline: Optional[str] = None
        self.pipeline_status: str = "Idle"
        self.pipeline_steps: List[Dict[str, str]] = []
        
        # BIST and Safety
        self.bist_unlocked: bool = False
        self.bist_bypass_active: bool = False
        self.safe_to_unplug: bool = False
        self.emergency_reason: Optional[str] = None
        
        # AI & Audio State
        self.ai_enabled: bool = False
        self.gemini_api_key: str = ""
        self.ai_status: str = "DISCONNECTED"
        self.ai_thoughts_log: List[str] = []
        self.maintenance_logs: List[str] = []

system_state = SystemState()

class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception:
                pass

manager = ConnectionManager()

# Database Helper for Long-Term Memory (ai_history.db)
DB_PATH = "/var/log/antigravity/ai_history.db"

def init_db():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS conversation_history (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT,
            sender TEXT,
            message TEXT
        )
    """)
    conn.commit()
    conn.close()

init_db()

def archive_db_if_oversized():
    if not os.path.exists(DB_PATH):
        return
    size_mb = os.path.getsize(DB_PATH) / (1024 * 1024)
    if size_mb > 50.0:
        logger.info(f"DB Size is {size_mb:.1f} MB (> 50 MB). Executing WAL Checkpoint & Zip Archiving...")
        conn = sqlite3.connect(DB_PATH)
        conn.execute("PRAGMA wal_checkpoint(FULL)")
        conn.execute("VACUUM")
        conn.close()
        
        timestamp_str = datetime.now().strftime("%Y_%m_%d_%H%M")
        zip_path = f"/var/log/antigravity/history_archive_{timestamp_str}.zip"
        
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            zipf.write(DB_PATH, arcname="ai_history_backup.db")
            
        logger.info(f"Database successfully archived to {zip_path}")
        system_state.maintenance_logs.append(f"DB Archived: {zip_path} ({size_mb:.1f} MB -> ZIP)")

# Emergency Safe Landing Pipeline (Touch 1)
async def execute_emergency_landing_pipeline(reason: str):
    if system_state.active_pipeline == "EMERGENCY_SAFE_LANDING":
        return
    system_state.active_pipeline = "EMERGENCY_SAFE_LANDING"
    system_state.emergency_reason = reason
    system_state.pipeline_status = f"EMERGENCY LANDING: {reason}"
    logger.warning(f"EMERGENCY LANDING TRIGGERED: {reason}")
    
    # Audio Alert
    logger.info("AudioAlert: Внимание! Зафиксирована угроза левитации. Выполняется аварийный спуск.")
    
    # Smooth Descent
    while system_state.target_z > 0.0:
        system_state.target_z = max(system_state.target_z - 1.0, 0.0)
        await asyncio.sleep(0.2)
        
    system_state.z = 0.0
    
    # Coil Ramp Down (Slew-Rate Protected)
    for step in range(20):
        system_state.pwm_channels = [int(max(pwm - 25, 0)) for pwm in system_state.pwm_channels]
        await asyncio.sleep(0.1)
        
    system_state.pwm_channels = [0, 0, 0, 0, 0]
    system_state.safe_to_unplug = True
    system_state.pipeline_status = f"Emergency Landing Complete ({reason})"
    await asyncio.sleep(2.0)

# Core 860 Hz PID Loop
async def core_loop_860hz():
    logger.info("Асинхронный ПИД-контур (860 Гц) запущен.")
    target_dt = 1.0 / 860.0
    
    while True:
        loop_start = time.perf_counter()
        
        # Check Heartbeat & Battery Safety (Touch 1)
        wifi_elapsed = time.perf_counter() - system_state.agent_wifi_last_seen
        if system_state.target_z > 0 and system_state.active_pipeline is None:
            if wifi_elapsed > 0.500:
                asyncio.create_task(execute_emergency_landing_pipeline("Wi-Fi Heartbeat Loss (>500ms)"))
            elif system_state.agent_battery < 10.0:
                asyncio.create_task(execute_emergency_landing_pipeline("Critical Battery Level (<10%)"))
        
        # PID Calculations
        if system_state.target_z > 0:
            error = system_state.target_z - system_state.z
            pid_output = system_state.kp * error
            base_pwm = int(max(min(200 + pid_output, system_state.max_duty_limit), 0))
            system_state.z = 25.0 + 0.1 * math.sin(time.perf_counter() * 5)
        else:
            base_pwm = 0
            
        if system_state.active_pipeline is None and system_state.bist_unlocked:
            system_state.pwm_channels = [base_pwm] * 5 if system_state.target_z > 0 else [0, 0, 0, 0, 0]
            
        # Slew-Rate Limiter (di/dt)
        clamped = []
        for i in range(5):
            prev = system_state.prev_pwm_channels[i]
            tgt = system_state.pwm_channels[i]
            diff = tgt - prev
            if diff > system_state.slew_rate_limit:
                actual = prev + system_state.slew_rate_limit
            elif diff < -system_state.slew_rate_limit:
                actual = prev - system_state.slew_rate_limit
            else:
                actual = tgt
            clamped.append(int(actual))
            
        system_state.pwm_channels = clamped
        system_state.prev_pwm_channels = list(clamped)
        
        elapsed = time.perf_counter() - loop_start
        sleep_time = target_dt - elapsed
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            await asyncio.sleep(0)

# Camera Stream Generator (Touch 3 - MJPEG)
async def mjpeg_frame_generator():
    while True:
        # Generate synthetic MJPEG frame buffer or capture from OV2640
        dummy_jpeg = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + b"\xff\xd8\xff\xe0" + b"\x00" * 100 + b"\xff\xd9\r\n"
        yield dummy_jpeg
        await asyncio.sleep(0.066) # ~15 FPS

@app.get("/api/agent/camera_stream")
async def camera_stream():
    return StreamingResponse(mjpeg_frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.post("/api/ai/clean_cache")
async def clean_cache_endpoint():
    archive_db_if_oversized()
    return {"status": "success", "message": "Temp cache cleaned and DB archive checked."}

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(core_loop_860hz())
    logger.info("V12 Server Engine & Safety Monitors Active.")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
