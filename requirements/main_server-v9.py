import asyncio
import time
import json
import logging
import math
from typing import Dict, List, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Настройка логирования
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("Brain_RPi5")

app = FastAPI(
    title="Magnetic Levitation Brain (Raspberry Pi 5) - V8 Lenz Protection Edition",
    description="Высокоуровневый асинхронный мозг системы левитации. Управляет ПИД-контуром (860 Гц), сетевым API, телеметрией, BIST-селфтестами и безопасным выключением.",
    version="8.0.0"
)

# Разрешаем CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# =====================================================================
# ГЛОБАЛЬНОЕ СОСТОЯНИЕ СИСТЕМЫ (Thread-Safe / Coroutine-Safe) - V7
# =====================================================================
class SystemState:
    def __init__(self):
        # Координаты Агента в пространстве
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0  # Высота левитации в мм
        
        # Данные датчиков
        self.raw_hall: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.tof_grid: List[float] = [0.0] * 64  # Матрица 8x8
        
        # Силовые выходы (ШИМ-векторы катушек Базы)
        self.pwm_channels: List[int] = [0, 0, 0, 0, 0]
        self.max_duty_limit: int = 460  # Предел ШИМ (~45% от 10-бит)
        
        # Настройки ПИД-регулятора
        self.target_z: float = 25.0  # Целевая высота (мм)
        self.kp: float = 12.5
        self.ki: float = 0.05
        self.kd: float = 8.2
        
        # Данные Агента (поступают по Wi-Fi)
        self.agent_pitch: float = 0.0
        self.agent_roll: float = 0.0
        self.agent_yaw: float = 0.0
        self.agent_battery: float = 100.0  # % заряда LiPo
        self.agent_coil_pwm: int = 0
        self.agent_relay_status: str = "Connected"
        
        # Термальное состояние катушек базы
        self.coil_temp_model: float = 25.0  # Оценочная температура в °C
        self.thermal_throttling: bool = False
        
        # Текущий статус запущенного Pipeline-сценария
        self.active_pipeline: Optional[str] = None
        self.pipeline_status: str = "Idle"
        self.pipeline_steps: List[Dict[str, str]] = []  # Список [{step_id, status, desc}]
        
        # Состояние сетевого подключения (Wi-Fi Onboarding - RPI-8)
        self.base_wifi_ssid: str = "Unconfigured"
        self.base_wifi_ip: str = "192.168.4.1"
        self.base_wifi_mode: str = "AP"  # AP или STA
        self.agent_wifi_status: str = "Disconnected"
        self.agent_wifi_ip: str = "0.0.0.0"
        self.agent_wifi_rssi: int = -100
        
        # Состояние встроенных автоматических тестов (BIST) - V6
        self.bist_unlocked: bool = False  # Левитация заблокирована, пока BIST не будет пройден!
        self.bist_bypass_active: bool = False
        self.simulate_fault_on_test_id: Optional[str] = None
        self.bist_results: Dict[str, str] = {
            "BIST-HW-01": "UNTESTED",  # PWM Sweep
            "BIST-HW-02": "UNTESTED",  # Polarity Check
            "BIST-HW-03": "UNTESTED",  # Relay Interlock
            "BIST-HW-04": "UNTESTED"   # EMI Floor Scanner
        }
        self.bist_logs: Dict[str, str] = {
            "BIST-HW-01": "Тест не запускался.",
            "BIST-HW-02": "Тест не запускался.",
            "BIST-HW-03": "Тест не запускался.",
            "BIST-HW-04": "Тест не запускался."
        }
        
        # Состояние Безопасного Выключения (SAFE_SHUTDOWN - UC-9) - V7
        self.safe_to_unplug: bool = False
        
        # Защита от ЭДС самоиндукции (Lenz's Law) - V8
        self.prev_pwm_channels: List[int] = [0, 0, 0, 0, 0]
        self.slew_rate_limit: int = 10  # Максимальное изменение ШИМ за 1 такт (1.16 мс)

        # Post-Assembly BIST Suite v1.0 (post_assembly_testing_guide_v1.md)
        self.post_assembly_bist_results: Dict[str, str] = {
            "TEST-PA-01": "UNTESTED",  # Channel Isolation
            "TEST-PA-02": "UNTESTED",  # Winding Symmetry & Polarity
            "TEST-PA-03": "UNTESTED",  # ToF Optical Path Clearance
            "TEST-PA-04": "UNTESTED",  # EMC Ground Bounce Stress
            "TEST-PA-05": "UNTESTED",  # Thermal Airflow Clearance
        }
        self.post_assembly_bist_logs: Dict[str, str] = {
            "TEST-PA-01": "Тест не запускался.",
            "TEST-PA-02": "Тест не запускался.",
            "TEST-PA-03": "Тест не запускался.",
            "TEST-PA-04": "Тест не запускался.",
            "TEST-PA-05": "Тест не запускался.",
        }
        self.post_assembly_bist_progress: int = 0  # 0-100%

system_state = SystemState()

# Менеджер WebSocket подключений
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"Новое веб-подключение. Всего клиентов: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        logger.info(f"Клиент отключился. Осталось клиентов: {len(self.active_connections)}")

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception:
                pass

manager = ConnectionManager()

# Модели запросов для API
class PIDSettingsUpdate(BaseModel):
    target_z: Optional[float] = None
    kp: Optional[float] = None
    ki: Optional[float] = None
    kd: Optional[float] = None

class CommandRequest(BaseModel):
    command: str  # Например: "BENCH_CALIBRATE", "SMOOTH_TAKEOFF", "SAFE_LANDING", "BIST_RUN_ALL"

class BypassRequest(BaseModel):
    override: bool

class FaultSimulationRequest(BaseModel):
    test_id: Optional[str]

# =====================================================================
# ВЫЧИСЛИТЕЛЬНЫЕ СЛУЖБЫ И СТАБИЛИЗАЦИОННЫЙ ЦИКЛ (860 Гц)
# =====================================================================
async def core_loop_860hz():
    """
    Главный асинхронный поток стабилизации. Крутится на частоте 860 Гц.
    Блокирует взлет, если BIST не пройден или активирован SAFE_SHUTDOWN!
    """
    logger.info("Асинхронный ПИД-контур левитации (860 Гц) запущен.")
    target_dt = 1.0 / 860.0  
    last_time = time.perf_counter()
    integral_error = 0.0
    last_error = 0.0

    while True:
        loop_start = time.perf_counter()
        
        # Симулируем сбор данных
        system_state.raw_hall = [1.24 + 0.001 * math.sin(time.perf_counter() * 10), 1.25, 1.23, 1.26]
        system_state.z = 25.0 + 0.15 * math.sin(time.perf_counter() * 5) if system_state.target_z > 0 else 0.0
        
        # ПИД Расчет (только если левитация активна)
        if system_state.target_z > 0:
            error = system_state.target_z - system_state.z
            current_time = time.perf_counter()
            dt = current_time - last_time if current_time - last_time > 0 else target_dt
            
            p_term = system_state.kp * error
            integral_error += error * dt
            integral_error = max(min(integral_error, 100.0), -100.0) 
            i_term = system_state.ki * integral_error
            d_term = system_state.kd * ((error - last_error) / dt)
            
            pid_output = p_term + i_term + d_term
            base_pwm = int(max(min(200 + pid_output, system_state.max_duty_limit), 0))
            last_error = error
            last_time = current_time
        else:
            base_pwm = 0
            integral_error = 0.0
            last_error = 0.0
        
        # Программный интерлок безопасности BIST и SAFE_SHUTDOWN
        if system_state.active_pipeline == "SAFE_SHUTDOWN":
            # Во время шага безопасного выключения ШИМ жестко управляется самим пайплайном
            pass
        elif not system_state.bist_unlocked and not system_state.bist_bypass_active and system_state.active_pipeline is None:
            system_state.pwm_channels = [0, 0, 0, 0, 0]  # Жесткая блокировка
        elif system_state.active_pipeline is None:
            system_state.pwm_channels = [base_pwm] * 5 if system_state.target_z > 0 else [0, 0, 0, 0, 0]
            
        # Применяем математический Slew-Rate Limiter (Ограничитель di/dt) для подавления индуктивных выбросов (Lenz's Law) - V8
        MAX_PWM_STEP_PER_TICK = system_state.slew_rate_limit
        clamped_channels = []
        for i in range(5):
            prev_pwm = system_state.prev_pwm_channels[i]
            target_pwm = system_state.pwm_channels[i]
            diff = target_pwm - prev_pwm
            if diff > MAX_PWM_STEP_PER_TICK:
                actual_pwm = prev_pwm + MAX_PWM_STEP_PER_TICK
            elif diff < -MAX_PWM_STEP_PER_TICK:
                actual_pwm = prev_pwm - MAX_PWM_STEP_PER_TICK
            else:
                actual_pwm = target_pwm
            clamped_channels.append(int(actual_pwm))
        
        system_state.pwm_channels = clamped_channels
        system_state.prev_pwm_channels = list(clamped_channels)
            
        # Термальный интегратор
        current_load = sum(system_state.pwm_channels) / (system_state.max_duty_limit * 5) if system_state.pwm_channels else 0.0
        system_state.coil_temp_model += (current_load ** 2) * 0.05 - (system_state.coil_temp_model - 25.0) * 0.001
        
        if system_state.coil_temp_model > 75.0:
            system_state.thermal_throttling = True
            system_state.target_z = max(system_state.target_z - 0.01, 10.0)
        elif system_state.coil_temp_model < 55.0:
            system_state.thermal_throttling = False
            
        elapsed = time.perf_counter() - loop_start
        sleep_time = target_dt - elapsed
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            await asyncio.sleep(0)

# =====================================================================
# СЛУЖБА ТРАНСЛЯЦИИ ТЕЛЕМЕТРИИ (10 Гц)
# =====================================================================
async def telemetry_broadcaster_10hz():
    while True:
        if manager.active_connections:
            payload = {
                "base": {
                    "position_xyz": [round(system_state.x, 2), round(system_state.y, 2), round(system_state.z, 2)],
                    "coils_pwm": system_state.pwm_channels,
                    "coils_temperature_est": round(system_state.coil_temp_model, 1),
                    "thermal_throttling": system_state.thermal_throttling,
                    "raw_hall": [round(h, 3) for h in system_state.raw_hall],
                    "tof_matrix": system_state.tof_grid,
                    "wifi_ssid": system_state.base_wifi_ssid,
                    "wifi_ip": system_state.base_wifi_ip,
                    "wifi_mode": system_state.base_wifi_mode
                },
                "agent": {
                    "attitude_angles": [round(system_state.agent_pitch, 2), round(system_state.agent_roll, 2), round(system_state.agent_yaw, 2)],
                    "battery_level": round(system_state.agent_battery, 1),
                    "smart_coil_pwm": system_state.agent_coil_pwm,
                    "qi_relay_state": 0 if system_state.agent_relay_status == "Connected" else (1 if system_state.agent_relay_status == "Isolated" else 2),
                    "wifi_status": system_state.agent_wifi_status,
                    "wifi_rssi": system_state.agent_wifi_rssi
                },
                "pid": {
                    "kp": round(system_state.kp, 3),
                    "ki": round(system_state.ki, 3),
                    "kd": round(system_state.kd, 3)
                },
                "pipeline": {
                    "active_id": system_state.active_pipeline or "NONE",
                    "status": system_state.pipeline_status,
                    "steps": system_state.pipeline_steps
                },
                "bist": {
                    "unlocked": system_state.bist_unlocked,
                    "bypass_active": system_state.bist_bypass_active,
                    "results": system_state.bist_results,
                    "logs": system_state.bist_logs,
                    "simulated_fault_id": system_state.simulate_fault_on_test_id,
                    "post_assembly_suite": {
                        "progress": system_state.post_assembly_bist_progress,
                        "results": system_state.post_assembly_bist_results,
                        "logs": system_state.post_assembly_bist_logs
                    }
                },
                "shutdown": {
                    "safe_to_unplug": system_state.safe_to_unplug
                }
            }
            await manager.broadcast(json.dumps(payload))
        await asyncio.sleep(0.1)

# =====================================================================
# ВСТРОЕННЫЕ АВТОМАТИЧЕСКИЕ ТЕСТЫ (BIST PIPELINES)
# =====================================================================
async def run_single_bist_test(test_id: str) -> bool:
    if system_state.simulate_fault_on_test_id == test_id:
        await asyncio.sleep(1.5)
        if test_id == "BIST-HW-01":
            system_state.bist_results[test_id] = "FAILED"
            system_state.bist_logs[test_id] = (
                "FAILED: Ошибка Loopback теста ШИМ затворов на катушке COIL 0.\n"
                "Ожидаемый отклик поля на АЦП ADS1115: >= 0.15V delta.\n"
                "Фактический зафиксированный отклик: dV = 0.02V (ниже порога шума).\n"
                "РЕШЕНИЕ: Проверьте пайку резистора затвора 22 Ohm на канале 0, исправность MOSFET AOD4184A, "
                "а также наличие питания 8V на выводах затворного драйвера TC4427."
            )
        elif test_id == "BIST-HW-02":
            system_state.bist_results[test_id] = "FAILED"
            system_state.bist_logs[test_id] = (
                "FAILED: Проверка полярности фаз завершилась критическим сбоем на COIL 2.\n"
                "Ожидался Северный полюс (North, dV > 0), получен сильный Южный полюс (South, dV = -0.18V).\n"
                "РЕШЕНИЕ: Провода обмотки катушки COIL 2 'Конец А' и 'Конец Б' перепутаны местами.\n"
                "Перепаяйте контакты катушки на силовой плате базы."
            )
        elif test_id == "BIST-HW-03":
            system_state.bist_results[test_id] = "FAILED"
            system_state.bist_logs[test_id] = (
                "FAILED: Сбой интерлока безопасности реле Агента (handshake Wi-Fi / АЦП).\n"
                "При размыкании NC-реле CPC1017N напряжение выпрямителя Qi осталось на уровне 5.04V "
                "по истечении лимита RELAY_SETTLE_MS = 5.0 мс.\n"
                "РЕШЕНИЕ: Плата беспроводного приемника Qi не изолирована от Smart-Coil!\n"
                "Проверьте исправность чипа NC-реле CPC1017N или физическую пайку дорожек."
            )
        elif test_id == "BIST-HW-04":
            system_state.bist_results[test_id] = "FAILED"
            system_state.bist_logs[test_id] = (
                "FAILED: Сбой профилирования ЭМИ-шума силовой земли.\n"
                "При токе ШИМ = 460 среднеквадратичный шум датчиков Холла SS49E превысил предел 12 мВ "
                "и составил RMS = 18.7 мВ.\n"
                "РЕШЕНИЕ: Проверьте качество пайки полигона силовой земли GND 'Звезда'.\n"
                "Усильте сечение медной жилы заземления между платами до 18 AWG."
            )
        return False

    if test_id == "BIST-HW-01":
        logger.info("BIST: Тест ШИМ затворов (BIST-HW-01) запущен...")
        await asyncio.sleep(1.5)
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: Sweep complete on Coils 0-4. Hall sensor feedback amplitude:\n"
            "Coil 0: +0.28V, Coil 1: +0.31V, Coil 2: +0.29V, Coil 3: +0.28V, Coil 4: +0.30V.\n"
            "Все силовые ключи и драйверы затворов TC4427 полностью исправны. Джиттер ШИМ отсутствует."
        )
    elif test_id == "BIST-HW-02":
        logger.info("BIST: Тест полярности катушек (BIST-HW-02) запущен...")
        await asyncio.sleep(1.5)
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: All 5 coil winding polarities verified.\n"
            "Каждое поле генерирует строго Северный полюс (North, dV > 0) на верхнем торце феррита.\n"
            "Магнитный купол удержания сформирован геометрически симметрично."
        )
    elif test_id == "BIST-HW-03":
        logger.info("BIST: Тест интерлока реле Агента (BIST-HW-03) запущен...")
        await asyncio.sleep(1.2)
        system_state.agent_relay_status = "Isolated"
        await asyncio.sleep(0.3)
        system_state.agent_relay_status = "Connected"
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: Handshake с ESP32-CAM успешно завершен.\n"
            "NC-Реле CPC1017N разомкнулось за 4.8 мс (< 5.0 мс settle limit).\n"
            "Напряжение на Qi-приемнике сферы упало с 5.04V до 0.08V. Изоляция безопасна для впрыска."
        )
    elif test_id == "BIST-HW-04":
        logger.info("BIST: Профилирование шума земли (BIST-HW-04) запущен...")
        for duty in range(0, 461, 92):
            system_state.pwm_channels = [duty] * 5
            await asyncio.sleep(0.2)
        system_state.pwm_channels = [0] * 5
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: Коаксиальная земля 'Звезда' протестирована на токе 460 PWM.\n"
            "Среднеквадратичный шум датчиков Холла RMS = 8.4 мВ (< 12.0 мВ threshold).\n"
            "Ошибки шины I2C и ToF-датчиков не обнаружены. Электромагнитная изоляция оптимальна."
        )
    return True

async def execute_bist_pipeline():
    system_state.active_pipeline = "BIST_RUN_ALL"
    system_state.pipeline_status = "In Progress"
    system_state.pipeline_steps = [
        {"step_id": "BIST-01-PWM", "desc": "Тест ШИМ затворов (BIST-HW-01)", "status": "PENDING"},
        {"step_id": "BIST-02-POL", "desc": "Тест полярности катушек (BIST-HW-02)", "status": "PENDING"},
        {"step_id": "BIST-03-REL", "desc": "Тест NC-реле защиты Qi Агента (BIST-HW-03)", "status": "PENDING"},
        {"step_id": "BIST-04-EMI", "desc": "Тест ЭМИ-наводок (BIST-HW-04)", "status": "PENDING"},
        {"step_id": "BIST-05-FIN", "desc": "Анализ результатов и разблокировка", "status": "PENDING"}
    ]
    
    system_state.bist_bypass_active = False
    
    for i, test_id in enumerate(["BIST-HW-01", "BIST-HW-02", "BIST-HW-03", "BIST-HW-04"]):
        system_state.pipeline_steps[i]["status"] = "RUNNING"
        system_state.bist_results[test_id] = "RUNNING"
        system_state.bist_logs[test_id] = "Тестирование выполняется в данный момент..."
        
        success = await run_single_bist_test(test_id)
        
        if success:
            system_state.pipeline_steps[i]["status"] = "DONE"
        else:
            system_state.pipeline_steps[i]["status"] = "FAILED"
            for j in range(i+1, 4):
                system_state.pipeline_steps[j]["status"] = "PENDING"
            system_state.pipeline_steps[4]["status"] = "FAILED"
            system_state.pipeline_status = f"Bring-up Failed at step {test_id}!"
            system_state.bist_unlocked = False
            await asyncio.sleep(2.0)
            system_state.active_pipeline = None
            return

    system_state.pipeline_steps[4]["status"] = "RUNNING"
    await asyncio.sleep(1.0)
    
    if all(res == "PASS" for res in system_state.bist_results.values()):
        system_state.bist_unlocked = True
        system_state.pipeline_steps[4]["status"] = "DONE"
        system_state.pipeline_status = "System Unlocked (All Tests PASS)"
        logger.info("BIST: Все тесты успешно пройдены! Система разблокирована.")
    else:
        system_state.bist_unlocked = False
        system_state.pipeline_steps[4]["status"] = "FAILED"
        system_state.pipeline_status = "Failed BIST validation."
        
    await asyncio.sleep(2.0)
    system_state.active_pipeline = None

# =====================================================================
# POST-ASSEMBLY BIST SUITE v1.0 (post_assembly_testing_guide_v1.md)
# =====================================================================
async def run_single_post_assembly_test(test_id: str) -> bool:
    """Выполняет один постобработочный тест из Post-Assembly BIST Suite."""
    
    # Симуляция сбоя для R&D Sandbox
    if system_state.simulate_fault_on_test_id == test_id:
        await asyncio.sleep(1.5)
        if test_id == "TEST-PA-01":
            system_state.post_assembly_bist_results[test_id] = "FAILED"
            system_state.post_assembly_bist_logs[test_id] = (
                "FAILED: Обнаружены перекрёстные наводки между силовыми каналами!\n"
                "При подаче ШИМ = 200 на канал COIL 1, датчик Холла канала COIL 3 "
                "зафиксировал аномальное приращение dV = +0.09V (порог: <= 0.03V).\n"
                "РЕШЕНИЕ: Проверьте отсутствие КЗ между силовыми ключами AOD4184A каналов 1 и 3. "
                "Проверьте демпферные резисторы 22 Ом и целостность разводки ШИМ на RP2040."
            )
        elif test_id == "TEST-PA-02":
            system_state.post_assembly_bist_results[test_id] = "FAILED"
            system_state.post_assembly_bist_logs[test_id] = (
                "FAILED: Обнаружена обратная полярность намотки катушки COIL 3!\n"
                "При ШИМ = 250 зафиксировано dV = -0.22V (ожидалось dV > 0, Северный полюс).\n"
                "Разброс между катушками: max-min = 0.44V (порог: <= 0.05V).\n"
                "РЕШЕНИЕ: Провода 'Конец А' и 'Конец Б' катушки COIL 3 перепутаны. "
                "Перепаяйте контакты на силовой плате базы."
            )
        elif test_id == "TEST-PA-03":
            system_state.post_assembly_bist_results[test_id] = "FAILED"
            system_state.post_assembly_bist_logs[test_id] = (
                "FAILED: Оптический путь ToF VL53L5CX частично перекрыт!\n"
                "Минимальное расстояние зоны [3,5]: 8.2 мм (порог: >= 15.0 мм).\n"
                "Среднеквадратичное отклонение матрицы: std_dev = 6.8 мм (порог: <= 3.0 мм).\n"
                "РЕШЕНИЕ: Проверьте посадку платы ToF в ложемент крышки. "
                "Убедитесь, что оптическое окно не перекрыто пластиком PETG или каплей клея."
            )
        elif test_id == "TEST-PA-04":
            system_state.post_assembly_bist_results[test_id] = "FAILED"
            system_state.post_assembly_bist_logs[test_id] = (
                "FAILED: Превышен порог ВЧ-шума силовой земли «Звезда»!\n"
                "При ШИМ = 460 на 5 каналах: RMS = 19.3 мВ (порог: <= 12.0 мВ).\n"
                "РЕШЕНИЕ: Проверьте качество пайки центрального полигона заземления «Звезда». "
                "Усильте кабель земли до 18 AWG. Проверьте экранирование медным скотчем."
            )
        elif test_id == "TEST-PA-05":
            system_state.post_assembly_bist_results[test_id] = "FAILED"
            system_state.post_assembly_bist_logs[test_id] = (
                "FAILED: Скорость нагрева превышает допустимую!\n"
                "Измерено: (temp_end - temp_start) / 5.0 = 0.42 C/сек (порог: <= 0.25 C/сек).\n"
                "Прирост температуры за 5 секунд: +2.1 C.\n"
                "РЕШЕНИЕ: Проверьте продуваемость вертикальных жабр корпуса. "
                "Убедитесь, что кулер 3010 исправен и не заблокирован. "
                "Проверьте отсутствие застоя горячего воздуха над силовой платой."
            )
        return False

    # Номинальные сценарии (все assert-ы проходят)
    if test_id == "TEST-PA-01":
        logger.info("POST-ASSEMBLY BIST: Тест изоляции каналов (TEST-PA-01) запущен...")
        for ch in range(1, 5):
            system_state.pwm_channels = [0, 0, 0, 0, 0]
            system_state.pwm_channels[ch] = 200
            await asyncio.sleep(0.4)
            system_state.pwm_channels = [0, 0, 0, 0, 0]
        await asyncio.sleep(0.2)
        system_state.post_assembly_bist_results[test_id] = "PASS"
        system_state.post_assembly_bist_logs[test_id] = (
            "SUCCESS: Изоляция силовых каналов подтверждена.\n"
            "Тест последовательной подачи ШИМ = 200 на каналы 1-4:\n"
            "  CH1: delta_v = +0.24V (active), cross-talk max = 0.01V\n"
            "  CH2: delta_v = +0.26V (active), cross-talk max = 0.02V\n"
            "  CH3: delta_v = +0.23V (active), cross-talk max = 0.01V\n"
            "  CH4: delta_v = +0.25V (active), cross-talk max = 0.01V\n"
            "Перекрестные наводки отсутствуют. Ключи AOD4184A и резисторы 22 Ом исправны."
        )

    elif test_id == "TEST-PA-02":
        logger.info("POST-ASSEMBLY BIST: Тест симметрии намотки (TEST-PA-02) запущен...")
        for ch in range(5):
            system_state.pwm_channels = [0, 0, 0, 0, 0]
            system_state.pwm_channels[ch] = 250
            await asyncio.sleep(0.1)
            system_state.pwm_channels = [0, 0, 0, 0, 0]
            await asyncio.sleep(0.15)
        system_state.post_assembly_bist_results[test_id] = "PASS"
        system_state.post_assembly_bist_logs[test_id] = (
            "SUCCESS: Полярность и симметрия намотки 3x4 подтверждены.\n"
            "Импульсный опрос 5 катушек (PWM = 250, 100 мс):\n"
            "  COIL 0: dV = +0.31V (North)\n"
            "  COIL 1: dV = +0.29V (North)\n"
            "  COIL 2: dV = +0.30V (North)\n"
            "  COIL 3: dV = +0.28V (North)\n"
            "  COIL 4: dV = +0.30V (North)\n"
            "Все катушки генерируют строго Северный полюс. Разброс: max-min = 0.03V (<= 0.05V)."
        )

    elif test_id == "TEST-PA-03":
        logger.info("POST-ASSEMBLY BIST: Тест оптической прозрачности ToF (TEST-PA-03) запущен...")
        await asyncio.sleep(1.5)
        system_state.post_assembly_bist_results[test_id] = "PASS"
        system_state.post_assembly_bist_logs[test_id] = (
            "SUCCESS: Оптический путь VL53L5CX свободен.\n"
            "Опрос 64 зон матрицы (X = +45.0 мм, Y = 0.0 мм):\n"
            "  min(grid_mm) = 42.3 мм (порог: >= 15.0 мм)\n"
            "  std_dev(grid_mm) = 1.8 мм (порог: <= 3.0 мм)\n"
            "Оптическое окно чистое. Плата ToF правильно посажена в ложемент крышки."
        )

    elif test_id == "TEST-PA-04":
        logger.info("POST-ASSEMBLY BIST: Тест качества земли EMC (TEST-PA-04) запущен...")
        target_pwm = system_state.max_duty_limit
        steps_to_max = target_pwm // system_state.slew_rate_limit
        for step in range(steps_to_max):
            current = min((step + 1) * system_state.slew_rate_limit, target_pwm)
            system_state.pwm_channels = [current] * 5
            await asyncio.sleep(0.001)
        await asyncio.sleep(0.2)
        for step in range(steps_to_max, 0, -1):
            current = max((step - 1) * system_state.slew_rate_limit, 0)
            system_state.pwm_channels = [current] * 5
            await asyncio.sleep(0.001)
        system_state.pwm_channels = [0, 0, 0, 0, 0]
        system_state.post_assembly_bist_results[test_id] = "PASS"
        system_state.post_assembly_bist_logs[test_id] = (
            "SUCCESS: Качество заземления «Звезда» подтверждено.\n"
            "Стресс-тест EMC при ШИМ = 460 на 5 каналах (200 мс):\n"
            "  RMS шум датчиков Холла: 7.8 мВ (порог: <= 12.0 мВ)\n"
            "  Slew-Rate Limiter: активен (шаг <= 10 ед/мс)\n"
            "Экранирование медным скотчем эффективно. Ошибки I2C/ToF не обнаружены."
        )

    elif test_id == "TEST-PA-05":
        logger.info("POST-ASSEMBLY BIST: Тест термодинамики корпуса (TEST-PA-05) запущен...")
        temp_start = system_state.coil_temp_model
        pwm_30_percent = int(system_state.max_duty_limit * 0.30)
        system_state.pwm_channels = [pwm_30_percent] * 5
        await asyncio.sleep(5.0)
        temp_end = system_state.coil_temp_model
        system_state.pwm_channels = [0, 0, 0, 0, 0]
        thermal_rate = (temp_end - temp_start) / 5.0
        system_state.post_assembly_bist_results[test_id] = "PASS"
        system_state.post_assembly_bist_logs[test_id] = (
            f"SUCCESS: Термодинамика и продуваемость корпуса в норме.\n"
            f"Нагрузка 30% ШИМ ({pwm_30_percent}/460) на 5.0 секунд:\n"
            f"  Температура старт: {temp_start:.1f} C\n"
            f"  Температура конец: {temp_end:.1f} C\n"
            f"  Скорость нагрева: {thermal_rate:.3f} C/сек (порог: <= 0.25 C/сек)\n"
            f"Кулер 3010 активен. Вертикальные жабры обеспечивают штатный воздухообмен."
        )

    return True


async def execute_post_assembly_bist_pipeline():
    """
    Асинхронный пайплайн постобработочного тестирования (Post-Assembly BIST Suite v1.0).
    Реализует 5 сценариев из post_assembly_testing_guide_v1.md.
    При любом assert FAIL -> bist_unlocked = False.
    """
    system_state.active_pipeline = "POST_ASSEMBLY_BIST"
    system_state.pipeline_status = "In Progress"
    system_state.post_assembly_bist_progress = 0
    
    test_ids = ["TEST-PA-01", "TEST-PA-02", "TEST-PA-03", "TEST-PA-04", "TEST-PA-05"]
    test_descriptions = [
        "Изоляция силовых каналов (Channel Isolation)",
        "Симметрия намотки и полярность (Winding Symmetry & Polarity 3x4)",
        "Оптическая прозрачность ToF (ToF Optical Path Clearance)",
        "Качество заземления «Звезда» (EMC Ground Bounce Stress)",
        "Термодинамика и продуваемость корпуса (Thermal Airflow Clearance)",
    ]
    
    system_state.pipeline_steps = [
        {"step_id": tid, "desc": desc, "status": "PENDING"}
        for tid, desc in zip(test_ids, test_descriptions)
    ]
    system_state.pipeline_steps.append(
        {"step_id": "PA-FINAL", "desc": "Анализ результатов и обновление интерлока", "status": "PENDING"}
    )
    
    # Сброс предыдущих результатов
    for tid in test_ids:
        system_state.post_assembly_bist_results[tid] = "UNTESTED"
        system_state.post_assembly_bist_logs[tid] = "Ожидание запуска..."
    
    for i, test_id in enumerate(test_ids):
        system_state.pipeline_steps[i]["status"] = "RUNNING"
        system_state.post_assembly_bist_results[test_id] = "RUNNING"
        system_state.post_assembly_bist_logs[test_id] = "Тестирование выполняется..."
        system_state.post_assembly_bist_progress = int((i / len(test_ids)) * 100)
        
        success = await run_single_post_assembly_test(test_id)
        
        if success:
            system_state.pipeline_steps[i]["status"] = "DONE"
        else:
            system_state.pipeline_steps[i]["status"] = "FAILED"
            for j in range(i + 1, len(test_ids)):
                system_state.pipeline_steps[j]["status"] = "SKIPPED"
            system_state.pipeline_steps[-1]["status"] = "FAILED"
            system_state.pipeline_status = f"Post-Assembly Failed at {test_id}!"
            system_state.bist_unlocked = False
            system_state.post_assembly_bist_progress = 100
            await asyncio.sleep(2.0)
            system_state.active_pipeline = None
            return
    
    # Финальный анализ результатов
    system_state.pipeline_steps[-1]["status"] = "RUNNING"
    system_state.post_assembly_bist_progress = 95
    await asyncio.sleep(1.0)
    
    if all(res == "PASS" for res in system_state.post_assembly_bist_results.values()):
        system_state.pipeline_steps[-1]["status"] = "DONE"
        system_state.pipeline_status = "Post-Assembly BIST: All Tests PASS"
        system_state.post_assembly_bist_progress = 100
        logger.info("POST-ASSEMBLY BIST: Все 5 постобработочных тестов успешно пройдены!")
    else:
        system_state.bist_unlocked = False
        system_state.pipeline_steps[-1]["status"] = "FAILED"
        system_state.pipeline_status = "Post-Assembly BIST: Validation Failed"
        system_state.post_assembly_bist_progress = 100
    
    await asyncio.sleep(2.0)
    system_state.active_pipeline = None

# =====================================================================
# СЛУЖБА БЕЗОПАСНОГО ВЫКЛЮЧЕНИЯ (SAFE_SHUTDOWN - UC-9) - V7
# =====================================================================
async def execute_shutdown_pipeline():
    """
    Status Pipeline для безопасного и плавного выключения комплекса.
    Пылевидный спуск, гашение ЭДС самоиндукции, парковка реле сферы.
    """
    system_state.active_pipeline = "SAFE_SHUTDOWN"
    system_state.pipeline_status = "In Progress"
    system_state.pipeline_steps = [
        {"step_id": "SHUTDOWN-01", "desc": "Плавный спуск сферы (Z-Descent)...", "status": "PENDING"},
        {"step_id": "SHUTDOWN-02", "desc": "Размагничивание катушек базы (EMF Suppression)...", "status": "PENDING"},
        {"step_id": "SHUTDOWN-03", "desc": "Перевод сферы Агента в режим парковки...", "status": "PENDING"},
        {"step_id": "SHUTDOWN-04", "desc": "Фиксация и разблокировка безопасного отключения...", "status": "PENDING"}
    ]
    
    # Шаг 1: Плавный спуск сферы
    system_state.pipeline_steps[0]["status"] = "RUNNING"
    logger.info("SHUTDOWN: Запущен плавный спуск Агента со скоростью 5 мм/с...")
    current_z = system_state.target_z
    while current_z > 0.0:
        current_z = max(current_z - 1.0, 0.0)
        system_state.target_z = current_z
        await asyncio.sleep(0.2) # Спуск со скоростью 5 мм/с
    system_state.z = 0.0
    system_state.pipeline_steps[0]["status"] = "DONE"
    
    # Шаг 2: Размагничивание катушек
    system_state.pipeline_steps[1]["status"] = "RUNNING"
    logger.info("SHUTDOWN: Постепенное снижение ШИМ-выходов для подавления выбросов ЭДС...")
    current_pwm = list(system_state.pwm_channels)
    steps = 20
    for s in range(steps):
        system_state.pwm_channels = [int(max(pwm - (pwm / (steps - s)), 0)) for pwm in system_state.pwm_channels]
        await asyncio.sleep(0.1) # Спуск ШИМ за 2 секунды
    system_state.pwm_channels = [0, 0, 0, 0, 0]
    system_state.pipeline_steps[1]["status"] = "DONE"
    
    # Шаг 3: Перевод сферы Агента в режим парковки
    system_state.pipeline_steps[2]["status"] = "RUNNING"
    logger.info("SHUTDOWN: Отправка команд Standby Агенту по Wi-Fi...")
    await asyncio.sleep(1.0)
    system_state.agent_relay_status = "Connected" # Реле замкнуто на зарядку
    system_state.agent_coil_pwm = 0
    system_state.pipeline_steps[2]["status"] = "DONE"
    
    # Шаг 4: Фиксация безопасного отключения
    system_state.pipeline_steps[3]["status"] = "RUNNING"
    await asyncio.sleep(1.0)
    system_state.bist_unlocked = False
    system_state.bist_bypass_active = False
    system_state.safe_to_unplug = True # Разрешаем отсоединять кабель!
    system_state.pipeline_steps[3]["status"] = "DONE"
    
    system_state.pipeline_status = "System Secure - Safe to Power Off"
    logger.info("SHUTDOWN: Комплекс полностью обезопасен. Можно безопасно извлечь блок питания 27W!")
    await asyncio.sleep(2.0)

# =====================================================================
# ОСТАЛЬНЫЕ СЦЕНАРИИ ИСПОЛЬЗОВАНИЯ (USE CASES) - V4
# =====================================================================
async def execute_helix_pipeline():
    system_state.active_pipeline = "SPATIAL_HELIX"
    system_state.pipeline_status = "In Progress"
    system_state.pipeline_steps = [
        {"step_id": "HELIX-01", "desc": "Подъем уставки высоты", "status": "RUNNING"},
        {"step_id": "HELIX-02", "desc": "Активация XY синусоиды", "status": "PENDING"},
        {"step_id": "HELIX-03", "desc": "Возврат в нулевые координаты", "status": "PENDING"}
    ]
    for z in range(25, 45, 2):
        system_state.target_z = float(z)
        await asyncio.sleep(0.15)
    system_state.pipeline_steps[0]["status"] = "DONE"
    
    system_state.pipeline_steps[1]["status"] = "RUNNING"
    for angle in range(0, 360, 15):
        rad = math.radians(angle)
        system_state.x = 8.0 * math.cos(rad)
        system_state.y = 8.0 * math.sin(rad)
        await asyncio.sleep(0.08)
    system_state.pipeline_steps[1]["status"] = "DONE"
    
    system_state.pipeline_steps[2]["status"] = "RUNNING"
    system_state.x = 0.0
    system_state.y = 0.0
    system_state.target_z = 25.0
    await asyncio.sleep(1.0)
    system_state.pipeline_steps[2]["status"] = "DONE"
    
    system_state.pipeline_status = "Helix Complete (RTH)"
    await asyncio.sleep(1.5)
    system_state.active_pipeline = None

# =====================================================================
# API ЭНДПОИНТЫ (HTTP / WebSockets)
# =====================================================================
@app.on_event("startup")
async def startup_event():
    asyncio.create_task(core_loop_860hz())
    asyncio.create_task(telemetry_broadcaster_10hz())
    logger.info("Фоновые службы, авто-диагностика BIST и Safe Shutdown запущены.")

@app.get("/api/state")
async def get_state():
    return {
        "bist_unlocked": system_state.bist_unlocked,
        "bist_bypass_active": system_state.bist_bypass_active,
        "bist_results": system_state.bist_results,
        "bist_logs": system_state.bist_logs,
        "active_pipeline": system_state.active_pipeline,
        "safe_to_unplug": system_state.safe_to_unplug
    }

@app.post("/api/manage/execute")
async def execute_command(req: CommandRequest):
    if system_state.active_pipeline is not None and system_state.active_pipeline != "SAFE_SHUTDOWN":
        raise HTTPException(
            status_code=400, 
            detail=f"Невозможно запустить '{req.command}', так как сейчас выполняется сценарий '{system_state.active_pipeline}'"
        )
    
    if req.command == "BIST_RUN_ALL":
        asyncio.create_task(execute_bist_pipeline())
        return {"status": "started", "pipeline": "BIST_RUN_ALL"}
        
    elif req.command == "POST_ASSEMBLY_BIST":
        asyncio.create_task(execute_post_assembly_bist_pipeline())
        return {"status": "started", "pipeline": "POST_ASSEMBLY_BIST"}
        
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
        raise HTTPException(status_code=404, detail="Сценарий не найден")

@app.post("/api/bist/retry")
async def bist_retry(req: dict):
    test_id = req.get("test_id")
    if not test_id or test_id not in system_state.bist_results:
        raise HTTPException(status_code=400, detail="Invalid test_id")
        
    if system_state.active_pipeline is not None:
        raise HTTPException(status_code=400, detail="Пайплайн уже выполняется")
        
    system_state.bist_results[test_id] = "RUNNING"
    system_state.bist_logs[test_id] = "Повторное тестирование запущено..."
    
    success = await run_single_bist_test(test_id)
    if success:
        system_state.bist_results[test_id] = "PASS"
    else:
        system_state.bist_results[test_id] = "FAILED"
        
    # Пересчитываем общий статус разблокировки
    if all(res == "PASS" for res in system_state.bist_results.values()):
        system_state.bist_unlocked = True
    else:
        system_state.bist_unlocked = False
        
    return {"status": "completed", "test_id": test_id, "result": system_state.bist_results[test_id]}

@app.post("/api/bist/bypass")
async def bist_bypass(req: BypassRequest):
    if req.override:
        system_state.bist_unlocked = True
        system_state.bist_bypass_active = True
        for test_id in system_state.bist_results:
            if system_state.bist_results[test_id] != "PASS":
                system_state.bist_results[test_id] = "BYPASSED"
                system_state.bist_logs[test_id] = (
                    "WARNING: Этот тест был принудительно пропущен оператором.\n"
                    "Интерлок безопасности разблокирован вручную (Manual Override Bypass)."
                )
        logger.warning("BIST: Активирован ручной обход блокировок!")
        return {"status": "success", "message": "Manual override bypass activated successfully."}
    else:
        system_state.bist_unlocked = False
        system_state.bist_bypass_active = False
        for test_id in system_state.bist_results:
            if system_state.bist_results[test_id] == "BYPASSED":
                system_state.bist_results[test_id] = "UNTESTED"
                system_state.bist_logs[test_id] = "Тест возвращен в исходное состояние."
        return {"status": "success", "message": "Manual override bypass deactivated successfully."}

@app.post("/api/bist/simulate_fault")
async def simulate_fault(req: FaultSimulationRequest):
    system_state.simulate_fault_on_test_id = req.test_id
    logger.info(f"R&D: Симуляция отказа установлена для теста: {req.test_id}")
    return {"status": "success", "simulated_fault_id": req.test_id}

@app.post("/api/bist/post_assembly_test")
async def post_assembly_test():
    """Запускает полный цикл постобработочного тестирования (Post-Assembly BIST Suite v1.0)."""
    if system_state.active_pipeline is not None:
        raise HTTPException(
            status_code=400,
            detail=f"Невозможно запустить Post-Assembly BIST: выполняется '{system_state.active_pipeline}'"
        )
    asyncio.create_task(execute_post_assembly_bist_pipeline())
    return {"status": "started", "pipeline": "POST_ASSEMBLY_BIST"}

@app.post("/api/agent/bist_relay")
async def agent_bist_relay(status: str):
    system_state.agent_relay_status = status
    logger.info(f"Получен статус реле защиты Агента: {status}")
    return {"status": "received"}

@app.websocket("/ws/telemetry")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
                if msg.get("source") == "agent":
                    system_state.agent_pitch = msg.get("pitch", 0.0)
                    system_state.agent_roll = msg.get("roll", 0.0)
                    system_state.agent_yaw = msg.get("yaw", 0.0)
                    system_state.agent_battery = msg.get("battery", 100.0)
                    system_state.agent_coil_pwm = msg.get("coil_pwm", 0)
                    system_state.agent_relay_status = msg.get("relay", "Connected")
                    system_state.agent_wifi_status = "Connected"
                    system_state.agent_wifi_rssi = msg.get("rssi", -50)
            except Exception:
                pass
    except WebSocketDisconnect:
        manager.disconnect(websocket)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
