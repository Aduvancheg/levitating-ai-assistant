import asyncio
import time
import json
import logging
import math
from enum import Enum
from typing import Dict, List, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("Brain_RPi5_V10")

app = FastAPI(
    title="Magnetic Levitation Brain (Raspberry Pi 5) - V10 Calibration & Takeoff FSM Edition",
    description="Управляющий сервер с интегрированным конечным автоматом калибровки на разъёмном стенде.",
    version="10.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class CalibrationState(str, Enum):
    PARKED = "PARKED"
    ZEROING = "ZEROING"
    TAKEOFF_HOVER = "TAKEOFF_HOVER"
    WAIT_REMOVAL = "WAIT_REMOVAL"
    UNLOCKED_FLIGHT = "UNLOCKED_FLIGHT"

class SystemState:
    def __init__(self):
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0
        self.raw_hall: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.tof_grid: List[float] = [0.0] * 64
        
        self.pwm_channels: List[int] = [0, 0, 0, 0, 0]
        self.prev_pwm_channels: List[int] = [0, 0, 0, 0, 0]
        self.max_duty_limit: int = 460
        self.slew_rate_limit: int = 10
        
        self.target_z: float = 30.0  # Height while parked in cradle (mm)
        self.kp: float = 12.5
        self.ki: float = 0.05
        self.kd: float = 8.2
        
        self.offset_x_mm: float = 45.0
        self.offset_y_mm: float = 0.0
        
        # Calibration FSM
        self.calib_state: CalibrationState = CalibrationState.PARKED
        self.calib_timer_task: Optional[asyncio.Task] = None
        self.calib_log: str = "Агент установлен на калибровочный стенд."
        
        self.bist_unlocked: bool = False
        self.bist_bypass_active: bool = False
        self.bist_results: Dict[str, str] = {
            "BIST-HW-01": "PASS", "BIST-HW-02": "PASS",
            "BIST-HW-03": "PASS", "BIST-HW-04": "PASS", "BIST-HW-05": "PASS"
        }
        self.bist_logs: Dict[str, str] = {k: "PASS" for k in self.bist_results}
        
        self.active_pipeline: Optional[str] = None
        self.pipeline_status: str = "Idle"
        self.safe_to_unplug: bool = False

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

async def core_loop_860hz():
    target_dt = 1.0 / 860.0
    last_time = time.perf_counter()
    last_error = 0.0
    
    while True:
        loop_start = time.perf_counter()
        
        # Read simulated sensors
        system_state.raw_hall = [1.25, 1.25, 1.25, 1.25]
        system_state.z = system_state.target_z
        
        # Calculate PID if active
        if system_state.calib_state in [CalibrationState.TAKEOFF_HOVER, CalibrationState.WAIT_REMOVAL, CalibrationState.UNLOCKED_FLIGHT]:
            error = system_state.target_z - system_state.z
            dt = target_dt
            p_term = system_state.kp * error
            d_term = system_state.kd * ((error - last_error) / dt)
            base_pwm = int(max(min(200 + p_term + d_term, system_state.max_duty_limit), 0))
            last_error = error
        else:
            base_pwm = 0
            
        system_state.pwm_channels = [base_pwm] * 5 if base_pwm > 0 else [0, 0, 0, 0, 0]
        
        # Apply Slew-Rate Limiter (di/dt)
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

async def telemetry_broadcaster_10hz():
    while True:
        if manager.active_connections:
            payload = {
                "base": {
                    "position_xyz": [system_state.x, system_state.y, system_state.z],
                    "coils_pwm": system_state.pwm_channels,
                    "target_z": system_state.target_z
                },
                "calibration_fsm": {
                    "state": system_state.calib_state.value,
                    "log": system_state.calib_log,
                    "unlocked_flight": system_state.calib_state == CalibrationState.UNLOCKED_FLIGHT
                }
            }
            await manager.broadcast(json.dumps(payload))
        await asyncio.sleep(0.1)

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(core_loop_860hz())
    asyncio.create_task(telemetry_broadcaster_10hz())

@app.post("/api/calibration/start")
async def start_calibration():
    if system_state.calib_state != CalibrationState.PARKED:
        raise HTTPException(status_code=400, detail="Калибровка уже запущена или завершена.")
        
    system_state.calib_state = CalibrationState.ZEROING
    system_state.calib_log = "ШАГ 1/5: Снятие нулей датчиков Холла и ToF на высоте Z = 30.0мм..."
    await asyncio.sleep(1.0)
    
    system_state.calib_state = CalibrationState.TAKEOFF_HOVER
    system_state.calib_log = "ШАГ 2/5: Плавный приподъем сферы на +3.0мм (Z = 33.0мм)..."
    system_state.target_z = 33.0
    await asyncio.sleep(1.5)
    
    system_state.calib_state = CalibrationState.WAIT_REMOVAL
    system_state.calib_log = "ШАГ 3/5: Сфера зависла! Раздвиньте половинки подставки и нажмите CONFIRM."
    
    # Timeout task: 30s auto-descent
    async def timeout_handler():
        await asyncio.sleep(30.0)
        if system_state.calib_state == CalibrationState.WAIT_REMOVAL:
            logger.warning("Calibration timeout! Auto-landing sphere...")
            system_state.calib_state = CalibrationState.PARKED
            system_state.target_z = 30.0
            system_state.calib_log = "ТАЙМАУТ (30с)! Сфера безопасно опустилась обратно в чашу."
            
    system_state.calib_timer_task = asyncio.create_task(timeout_handler())
    return {"status": "success", "state": system_state.calib_state.value}

@app.post("/api/calibration/confirm_stand_removed")
async def confirm_stand_removed():
    if system_state.calib_state != CalibrationState.WAIT_REMOVAL:
        raise HTTPException(status_code=400, detail="Ожидание удаления подставки не активно.")
        
    if system_state.calib_timer_task:
        system_state.calib_timer_task.cancel()
        
    system_state.calib_state = CalibrationState.UNLOCKED_FLIGHT
    system_state.target_z = 25.0  # Transition to operational levitation height
    system_state.calib_log = "ШАГ 5/5: Подставка удалена! Переход на рабочую высоту 25.0мм. Полетные режимы разблокированы!"
    return {"status": "success", "state": system_state.calib_state.value}

@app.post("/api/calibration/cancel")
async def cancel_calibration():
    if system_state.calib_timer_task:
        system_state.calib_timer_task.cancel()
    system_state.calib_state = CalibrationState.PARKED
    system_state.target_z = 30.0
    system_state.calib_log = "Калибровка отменена. Сфера вернулась на подставку."
    return {"status": "cancelled", "state": system_state.calib_state.value}

@app.websocket("/ws/telemetry")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
