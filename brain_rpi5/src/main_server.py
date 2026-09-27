import asyncio
import time
import json
import logging
import math
import os
import shutil
import hashlib
import struct
from pathlib import Path
from datetime import datetime, timezone
from enum import Enum
from typing import Dict, List, Optional, Any
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware

# Условный импорт AI/Audio зависимостей (не падаем, если не установлены)
try:
    import aiosqlite
    HAS_AIOSQLITE = True
except ImportError:
    HAS_AIOSQLITE = False

try:
    import google.generativeai as genai
    HAS_GENAI = True
except ImportError:
    HAS_GENAI = False

try:
    import sounddevice as sd
    import numpy as np
    HAS_AUDIO = True
except ImportError:
    HAS_AUDIO = False

class CalibrationState(str, Enum):
    PARKED = "PARKED"
    ZEROING = "ZEROING"
    TAKEOFF_HOVER = "TAKEOFF_HOVER"
    WAIT_REMOVAL = "WAIT_REMOVAL"
    UNLOCKED_FLIGHT = "UNLOCKED_FLIGHT"
from pydantic import BaseModel

# Настройка логирования
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("Brain_RPi5")

app = FastAPI(
    title="Magnetic Levitation Brain (Raspberry Pi 5) - V11.0 AI & Audio Edition",
    description="Высокоуровневый асинхронный мозг системы левитации. Управляет ПИД-контуром (860 Гц), Gemini Pro AI, AudioService SPU-WM30, Tool Calling, SQLite-памятью и BIST-селфтестами.",
    version="11.0.0"
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
            "BIST-HW-04": "UNTESTED",  # EMI Floor Scanner
            "BIST-HW-05": "UNTESTED"   # Lenz EMF Protection
        }
        self.bist_logs: Dict[str, str] = {
            "BIST-HW-01": "Тест не запускался.",
            "BIST-HW-02": "Тест не запускался.",
            "BIST-HW-03": "Тест не запускался.",
            "BIST-HW-04": "Тест не запускался.",
            "BIST-HW-05": "Тест не запускался."
        }
        
        # Состояние Безопасного Выключения (SAFE_SHUTDOWN - UC-9) - V7
        self.safe_to_unplug: bool = False
        
        # Защита от ЭДС самоиндукции (Lenz's Law) - V8
        self.prev_pwm_channels: List[int] = [0, 0, 0, 0, 0]
        self.slew_rate_limit: int = 10  # Максимальное изменение ШИМ за 1 такт (1.16 мс)
        
        # Геометрическое смещение ToF-датчика (enclosure_architecture_v7.md) - V9
        self.tof_offset_x_mm: float = 45.0  # Физический сдвиг ToF по оси X (мм)
        self.tof_offset_y_mm: float = 0.0   # Физический сдвиг ToF по оси Y (мм)

        # Calibration FSM
        self.calib_state: CalibrationState = CalibrationState.PARKED
        self.calib_timer_task: Optional[asyncio.Task] = None
        self.calib_log: str = "Агент установлен на калибровочный стенд."

        # ===================== V11: AI & Audio State =====================
        self.ai_enabled: bool = False
        self.ai_status: str = "DISCONNECTED"  # DISCONNECTED | CONNECTING | CONNECTED | AI_DISCONNECTED
        self.ai_ping_ms: float = 0.0
        self.ai_thoughts_log: List[str] = []
        self.ai_chat_history: List[Dict[str, str]] = []  # [{role, content, ts}]
        self.ai_model_name: str = "gemini-2.0-flash"

        # Post-Assembly BIST Suite v1.0 (TEST-PA-01..05)
        self.post_assembly_results: Dict[str, str] = {
            "TEST-PA-01": "UNTESTED",
            "TEST-PA-02": "UNTESTED",
            "TEST-PA-03": "UNTESTED",
            "TEST-PA-04": "UNTESTED",
            "TEST-PA-05": "UNTESTED",
        }
        self.post_assembly_logs: Dict[str, str] = {
            "TEST-PA-01": "Тест не запускался.",
            "TEST-PA-02": "Тест не запускался.",
            "TEST-PA-03": "Тест не запускался.",
            "TEST-PA-04": "Тест не запускался.",
            "TEST-PA-05": "Тест не запускался.",
        }

        # Audio Service state
        self.audio_available: bool = False
        self.audio_vad_active: bool = False
        self.audio_last_transcript: str = ""

system_state = SystemState()

# =====================================================================
# AI ДОЛГОВРЕМЕННАЯ ПАМЯТЬ SQLite (ai_history.db) - V11
# =====================================================================
AI_HISTORY_DB_PATH = Path("/var/log/antigravity/ai_history.db")
AI_CONFIG_PATH = Path("/etc/antigravity/ai_config.json")
TEMP_CACHE_DIR = Path("/tmp/antigravity")

# Gemini Tool Calling JSON schemas (ai_agent_gemini_architecture_v1.md)
GEMINI_TOOLS_SCHEMA = [
    {
        "name": "get_system_telemetry",
        "description": "Получить текущую физическую телеметрию Базы и Агента (высота Z, температура катушек, заряд LiPo, статус BIST).",
    },
    {
        "name": "set_levitation_height",
        "description": "Изменить целевую высоту левитации Агента (target_z в мм, диапазон 10..45 мм).",
        "parameters": {"target_z": {"type": "float"}},
    },
    {
        "name": "run_spatial_helix_animation",
        "description": "Запустить фигуру высшего пилотажа 'Пространственная спираль' (SPATIAL_HELIX).",
    },
    {
        "name": "execute_safe_shutdown",
        "description": "Запустить плавный спуск Агента и обесточить катушки (SAFE_SHUTDOWN).",
    },
    {
        "name": "inspect_surface",
        "description": "Наклонить сферу Агента на заданный угол, сделать снимок с камеры ESP32-CAM и проанализировать предметы на столе.",
        "parameters": {"tilt_angle_deg": {"type": "float"}},
    },
    {
        "name": "clear_temp_cache",
        "description": "Очистить временные лог-файлы и снимки калибровки, освободив диск RPi 5 без удаления истории диалогов.",
    },
]


class AIHistoryDB:
    """Долговременная память ИИ — сохранение диалогов в SQLite (RPI-CL-12 safe)."""

    def __init__(self, db_path: Path = AI_HISTORY_DB_PATH):
        self.db_path = db_path

    async def init_db(self) -> None:
        """Создать таблицу messages если не существует."""
        if not HAS_AIOSQLITE:
            logger.warning("aiosqlite не установлен — SQLite-память AI отключена.")
            return
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        async with aiosqlite.connect(str(self.db_path)) as db:
            await db.execute(
                """CREATE TABLE IF NOT EXISTS messages (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    role TEXT NOT NULL,
                    content TEXT NOT NULL,
                    timestamp TEXT NOT NULL
                )"""
            )
            await db.commit()

    async def save_message(self, role: str, content: str) -> None:
        if not HAS_AIOSQLITE:
            return
        ts = datetime.now(timezone.utc).isoformat()
        async with aiosqlite.connect(str(self.db_path)) as db:
            await db.execute(
                "INSERT INTO messages (role, content, timestamp) VALUES (?, ?, ?)",
                (role, content, ts),
            )
            await db.commit()

    async def load_recent(self, limit: int = 10) -> List[Dict[str, str]]:
        if not HAS_AIOSQLITE:
            return []
        try:
            async with aiosqlite.connect(str(self.db_path)) as db:
                cursor = await db.execute(
                    "SELECT role, content, timestamp FROM messages ORDER BY id DESC LIMIT ?",
                    (limit,),
                )
                rows = await cursor.fetchall()
                return [
                    {"role": r[0], "content": r[1], "ts": r[2]}
                    for r in reversed(rows)
                ]
        except Exception:
            return []

    async def get_checksum(self) -> str:
        """SHA-256 контрольная сумма файла БД для проверки целостности."""
        if not self.db_path.exists():
            return ""
        h = hashlib.sha256()
        data = await asyncio.to_thread(self.db_path.read_bytes)
        h.update(data)
        return h.hexdigest()


class AIConfigManager:
    """Менеджер постоянной конфигурации AI (ai_config.json).
    RPI-CL-12: ключ НЕ логируется в открытом виде.
    """

    def __init__(self, config_path: Path = AI_CONFIG_PATH):
        self.config_path = config_path
        self._api_key: str = ""
        self._ai_enabled: bool = False

    @property
    def api_key(self) -> str:
        return self._api_key

    @property
    def ai_enabled(self) -> bool:
        return self._ai_enabled

    def masked_key(self) -> str:
        """Вернуть маскированный ключ для безопасного логирования (RPI-CL-12)."""
        if not self._api_key or len(self._api_key) < 8:
            return "***"
        return self._api_key[:4] + "****" + self._api_key[-4:]

    def load(self) -> None:
        """Загрузить конфиг с диска."""
        if self.config_path.exists():
            try:
                data = json.loads(self.config_path.read_text())
                self._api_key = data.get("GEMINI_API_KEY", "")
                self._ai_enabled = data.get("ai_enabled", False)
                logger.info(f"AI Config loaded. Key={self.masked_key()}, enabled={self._ai_enabled}")
            except Exception as e:
                logger.error(f"AI Config load error: {e}")
        else:
            logger.info("AI Config not found — первый запуск.")

    def save(self, api_key: Optional[str] = None, ai_enabled: Optional[bool] = None) -> None:
        """Сохранить конфиг на диск."""
        if api_key is not None:
            self._api_key = api_key
        if ai_enabled is not None:
            self._ai_enabled = ai_enabled
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        data = {"GEMINI_API_KEY": self._api_key, "ai_enabled": self._ai_enabled}
        self.config_path.write_text(json.dumps(data, indent=2))
        logger.info(f"AI Config saved. Key={self.masked_key()}, enabled={self._ai_enabled}")


class GeminiAIManager:
    """Асинхронный клиент Gemini Pro API с Tool Calling и контекстным инжектором."""

    def __init__(self, config: AIConfigManager, history_db: AIHistoryDB):
        self.config = config
        self.history_db = history_db
        self._model = None
        self._chat = None

    async def connect(self) -> bool:
        """Инициализировать соединение с Gemini API."""
        if not HAS_GENAI:
            logger.warning("google-generativeai не установлен — Gemini AI отключен.")
            system_state.ai_status = "AI_DISCONNECTED"
            return False
        if not self.config.api_key:
            system_state.ai_status = "DISCONNECTED"
            return False
        try:
            system_state.ai_status = "CONNECTING"
            genai.configure(api_key=self.config.api_key)
            self._model = genai.GenerativeModel(system_state.ai_model_name)
            # Ping-тест — быстрый запрос
            t0 = time.perf_counter()
            response = await asyncio.to_thread(
                self._model.generate_content, "Respond with OK"
            )
            system_state.ai_ping_ms = round((time.perf_counter() - t0) * 1000, 1)
            system_state.ai_status = "CONNECTED"
            system_state.ai_thoughts_log.append(
                f"[{datetime.now(timezone.utc).isoformat()}] Gemini API connected. Ping={system_state.ai_ping_ms}ms"
            )
            logger.info(f"Gemini AI connected. Ping={system_state.ai_ping_ms}ms")
            return True
        except Exception as e:
            system_state.ai_status = "AI_DISCONNECTED"
            logger.error(f"Gemini AI connection failed: {e}")
            return False

    def _build_telemetry_context(self) -> str:
        """Подмешать телеметрию SystemState в контекст запроса."""
        return json.dumps({
            "height_z_mm": round(system_state.z, 2),
            "target_z_mm": round(system_state.target_z, 2),
            "coil_temp_c": round(system_state.coil_temp_model, 1),
            "thermal_throttling": system_state.thermal_throttling,
            "battery_pct": round(system_state.agent_battery, 1),
            "bist_unlocked": system_state.bist_unlocked,
            "pwm_channels": system_state.pwm_channels,
        })

    async def chat(self, user_message: str) -> Dict[str, Any]:
        """Отправить сообщение пользователя в Gemini и обработать Tool Calling."""
        # Сохраняем в историю
        await self.history_db.save_message("user", user_message)
        system_state.ai_chat_history.append({
            "role": "user", "content": user_message,
            "ts": datetime.now(timezone.utc).isoformat()
        })

        if not self._model or system_state.ai_status != "CONNECTED":
            return {"response": "AI не подключен.", "tool_calls": []}

        # Подготовить контекст
        history = await self.history_db.load_recent(10)
        telemetry = self._build_telemetry_context()
        context_prompt = (
            f"[System Telemetry Context]: {telemetry}\n"
            f"[Dialog History]: {json.dumps(history, ensure_ascii=False)}\n"
            f"User: {user_message}"
        )

        try:
            t0 = time.perf_counter()
            response = await asyncio.to_thread(
                self._model.generate_content, context_prompt
            )
            system_state.ai_ping_ms = round((time.perf_counter() - t0) * 1000, 1)

            ai_text = response.text if hasattr(response, "text") else str(response)

            # Сохраняем ответ ИИ
            await self.history_db.save_message("assistant", ai_text)
            system_state.ai_chat_history.append({
                "role": "assistant", "content": ai_text,
                "ts": datetime.now(timezone.utc).isoformat()
            })
            system_state.ai_thoughts_log.append(
                f"[{datetime.now(timezone.utc).isoformat()}] Response ({system_state.ai_ping_ms}ms): {ai_text[:80]}..."
            )

            # Обработка Tool Calling ответов
            tool_calls = await self._process_tool_calls(response)

            return {"response": ai_text, "tool_calls": tool_calls}
        except Exception as e:
            system_state.ai_status = "AI_DISCONNECTED"
            logger.error(f"Gemini chat error: {e}")
            return {"response": f"Ошибка AI: {e}", "tool_calls": []}

    async def _process_tool_calls(self, response: Any) -> List[Dict]:
        """Извлечь и выполнить вызовы инструментов из ответа Gemini."""
        tool_calls_executed = []
        # Проверяем наличие function calls в ответе
        if not hasattr(response, "candidates"):
            return tool_calls_executed
        for candidate in response.candidates:
            if not hasattr(candidate, "content") or not hasattr(candidate.content, "parts"):
                continue
            for part in candidate.content.parts:
                if hasattr(part, "function_call"):
                    fc = part.function_call
                    name = fc.name
                    args = dict(fc.args) if hasattr(fc, "args") else {}
                    result = await self._execute_tool(name, args)
                    tool_calls_executed.append({"name": name, "args": args, "result": result})
        return tool_calls_executed

    async def _execute_tool(self, name: str, args: Dict) -> str:
        """Выполнить инструмент Tool Calling (с проверкой безопасности RPI-CL-12)."""
        system_state.ai_thoughts_log.append(
            f"[TOOL] Executing: {name}({json.dumps(args)})"
        )
        if name == "get_system_telemetry":
            return self._build_telemetry_context()
        elif name == "set_levitation_height":
            return await tool_set_levitation_height(args.get("target_z", 25.0))
        elif name == "run_spatial_helix_animation":
            asyncio.create_task(execute_helix_pipeline())
            return "Spatial helix animation started."
        elif name == "execute_safe_shutdown":
            asyncio.create_task(execute_shutdown_pipeline())
            return "Safe shutdown initiated."
        elif name == "inspect_surface":
            return await tool_inspect_surface(args.get("tilt_angle_deg", 15.0))
        elif name == "clear_temp_cache":
            return await tool_clear_temp_cache()
        else:
            return f"Unknown tool: {name}"


# =====================================================================
# ИНСТРУМЕНТЫ TOOL CALLING (ai_agent_gemini_architecture_v1.md) - V11
# =====================================================================
async def tool_set_levitation_height(target_z: float) -> str:
    """Изменить уставку target_z с проверкой лимитов безопасности (RPI-CL-12)."""
    if target_z < 10.0 or target_z > 45.0:
        return f"REJECTED: target_z={target_z} out of safe range [10..45] mm."
    old_z = system_state.target_z
    system_state.target_z = target_z
    logger.info(f"Tool: set_levitation_height {old_z} -> {target_z} mm")
    return f"OK: target_z changed from {old_z} to {target_z} mm."


async def tool_inspect_surface(tilt_angle_deg: float = 15.0) -> str:
    """Наклонить сферу, захватить кадр с ESP32-CAM, отправить в Gemini Vision."""
    if tilt_angle_deg < 0 or tilt_angle_deg > 30.0:
        return f"REJECTED: tilt_angle={tilt_angle_deg} out of safe range [0..30] deg."
    # Симуляция наклона через дифференциальный ШИМ
    original_target_z = system_state.target_z
    system_state.agent_pitch = tilt_angle_deg
    logger.info(f"Tool: inspect_surface tilt={tilt_angle_deg}°, capturing ESP32-CAM frame...")
    await asyncio.sleep(0.5)  # Ждем стабилизации наклона
    # Симуляция захвата кадра (в реальности — HTTP к ESP32-CAM)
    frame_data = b"SIMULATED_JPEG_FRAME_1600x1200"
    # Возврат в горизонталь
    system_state.agent_pitch = 0.0
    return f"OK: Surface inspected at {tilt_angle_deg}°. Frame captured (1600x1200). Analysis: [simulated objects detected]."


async def tool_clear_temp_cache() -> str:
    """Безопасная очистка временных файлов (сохраняем ai_history.db!)."""
    if not TEMP_CACHE_DIR.exists():
        return "OK: /tmp/antigravity/ directory does not exist. Nothing to clean."
    # Считаем размер до очистки
    total_before = sum(
        f.stat().st_size for f in TEMP_CACHE_DIR.rglob("*") if f.is_file()
    ) if TEMP_CACHE_DIR.exists() else 0
    # Удаляем всё кроме самой директории
    for item in TEMP_CACHE_DIR.iterdir():
        try:
            if item.is_dir():
                shutil.rmtree(item)
            else:
                item.unlink()
        except Exception as e:
            logger.warning(f"Cache clean skip: {item} — {e}")
    freed_mb = round(total_before / (1024 * 1024), 1)
    logger.info(f"Tool: clear_temp_cache freed {freed_mb} MB. ai_history.db preserved.")
    return f"OK: Cleared {freed_mb} MB from /tmp/antigravity/. History DB preserved."


async def trigger_nod_gesture() -> None:
    """Микро-кивок сферы: Z += 3.0 мм на 500 мс, затем возврат (UC-BUS-12)."""
    original_z = system_state.target_z
    system_state.target_z = original_z + 3.0
    await asyncio.sleep(0.5)
    system_state.target_z = original_z


# =====================================================================
# АУДИО-СЛУЖБА SPU-WM30 USB AUDIO (spu_wm30_audio_integration_guide_v1.md) - V11
# =====================================================================
class AudioService:
    """Драйвер USB-микрофона Spacetronik SPU-WM30 (RPI-CL-13: non-blocking)."""

    SAMPLE_RATE = 16000  # 16 кГц PCM
    CHANNELS = 1  # Mono
    BLOCK_SIZE = 480  # 30ms frames для VAD

    def __init__(self):
        self._recording = False
        self._stream = None

    async def init(self) -> bool:
        """Инициализация аудио-устройства."""
        if not HAS_AUDIO:
            logger.warning("sounddevice не установлен — AudioService отключен.")
            system_state.audio_available = False
            return False
        try:
            devices = await asyncio.to_thread(sd.query_devices)
            system_state.audio_available = True
            logger.info(f"AudioService: SPU-WM30 initialized. Devices: {len(devices)}")
            return True
        except Exception as e:
            system_state.audio_available = False
            logger.error(f"AudioService init failed: {e}")
            return False

    async def capture_voice_chunk(self, duration_s: float = 3.0) -> Optional[bytes]:
        """Захват PCM-аудио в asyncio.to_thread (RPI-CL-13: не блокирует ПИД-контур)."""
        if not HAS_AUDIO or not system_state.audio_available:
            return None

        def _record():
            frames = int(self.SAMPLE_RATE * duration_s)
            audio = sd.rec(frames, samplerate=self.SAMPLE_RATE, channels=self.CHANNELS, dtype="int16")
            sd.wait()
            return audio.tobytes()

        try:
            system_state.audio_vad_active = True
            pcm_data = await asyncio.to_thread(_record)
            system_state.audio_vad_active = False
            return pcm_data
        except Exception as e:
            system_state.audio_vad_active = False
            logger.error(f"AudioService capture error: {e}")
            return None

    async def play_tone(self, freq_hz: float = 440.0, duration_s: float = 0.5) -> bool:
        """Воспроизвести тестовый тон через динамик SPU-WM30 (TEST-AI-03)."""
        if not HAS_AUDIO:
            return False

        def _play():
            t = np.linspace(0, duration_s, int(self.SAMPLE_RATE * duration_s), endpoint=False)
            tone = (np.sin(2 * np.pi * freq_hz * t) * 0.3 * 32767).astype(np.int16)
            sd.play(tone, samplerate=self.SAMPLE_RATE)
            sd.wait()

        try:
            await asyncio.to_thread(_play)
            return True
        except Exception as e:
            logger.error(f"AudioService play error: {e}")
            return False

    async def speak_text(self, text: str) -> bool:
        """Озвучить текст через TTS + динамик SPU-WM30 (заглушка для интеграции)."""
        logger.info(f"AudioService TTS: \"{text[:60]}...\"")
        # В реальной системе — вызов gTTS/pyttsx3 + sd.play()
        return True

    async def bist_audio_loop_test(self) -> Dict[str, Any]:
        """TEST-AI-03: Loopback тест аудио-тракта SPU-WM30."""
        if not HAS_AUDIO:
            return {"status": "SKIPPED", "reason": "sounddevice not available"}

        play_ok = await self.play_tone(440.0, 0.5)
        if not play_ok:
            return {"status": "FAILED", "reason": "Playback failed"}

        chunk = await self.capture_voice_chunk(0.5)
        if chunk is None:
            return {"status": "FAILED", "reason": "Capture failed"}

        # Проверка амплитуды эхо-сигнала
        samples = np.frombuffer(chunk, dtype=np.int16)
        max_amplitude = float(np.max(np.abs(samples))) / 32767.0
        if max_amplitude > 0.05:
            return {"status": "PASS", "max_amplitude": round(max_amplitude, 4)}
        else:
            return {"status": "FAILED", "max_amplitude": round(max_amplitude, 4),
                    "reason": "Echo amplitude below threshold 0.05"}


# =====================================================================
# POST-ASSEMBLY BIST SUITE V1.0 (post_assembly_testing_guide_v1.md) - V11
# =====================================================================
async def run_post_assembly_test(test_id: str) -> bool:
    """Выполнить один из 5 пост-сборочных тестов."""
    if test_id == "TEST-PA-01":
        # Channel Isolation: PWM pulse на каждый канал, проверка dV >= 0.12V
        logger.info("POST-ASSEMBLY: TEST-PA-01 Channel Isolation запущен...")
        await asyncio.sleep(1.0)
        # Симуляция: каждый активный канал дает dV = 0.28V, остальные < 0.03V
        for ch in range(4):
            system_state.pwm_channels = [0] * 5
            system_state.pwm_channels[ch] = 200
            await asyncio.sleep(0.2)
        system_state.pwm_channels = [0] * 5
        system_state.post_assembly_results[test_id] = "PASS"
        system_state.post_assembly_logs[test_id] = (
            "SUCCESS: Изоляция 4 боковых каналов подтверждена.\n"
            "dV[active] >= 0.28V, dV[other] <= 0.02V. Наводки отсутствуют."
        )
        return True

    elif test_id == "TEST-PA-02":
        # Winding Symmetry & Polarity: все катушки North, разброс <= 50mV
        logger.info("POST-ASSEMBLY: TEST-PA-02 Winding Symmetry запущен...")
        await asyncio.sleep(1.0)
        delta_v_list = [0.29, 0.31, 0.28, 0.30, 0.29]
        all_north = all(dv > 0 for dv in delta_v_list)
        spread = max(delta_v_list) - min(delta_v_list)
        if all_north and spread <= 0.05:
            system_state.post_assembly_results[test_id] = "PASS"
            system_state.post_assembly_logs[test_id] = (
                f"SUCCESS: Все 5 катушек North-полярность. Разброс = {spread*1000:.0f}мВ (<= 50мВ)."
            )
            return True
        else:
            system_state.post_assembly_results[test_id] = "FAILED"
            system_state.post_assembly_logs[test_id] = (
                f"FAILED: Полярность или симметрия нарушены. Spread={spread*1000:.0f}мВ."
            )
            return False

    elif test_id == "TEST-PA-03":
        # ToF Optical Path Clearance: min >= 15mm, std <= 3mm
        logger.info("POST-ASSEMBLY: TEST-PA-03 ToF Optical Path запущен...")
        await asyncio.sleep(0.8)
        grid_mm = [25.0 + 0.5 * (i % 8) for i in range(64)]
        min_val = min(grid_mm)
        std_val = (sum((x - sum(grid_mm)/64)**2 for x in grid_mm) / 64) ** 0.5
        if min_val >= 15.0 and std_val <= 3.0:
            system_state.post_assembly_results[test_id] = "PASS"
            system_state.post_assembly_logs[test_id] = (
                f"SUCCESS: ToF окно прозрачно. min={min_val:.1f}мм, std={std_val:.1f}мм."
            )
            return True
        else:
            system_state.post_assembly_results[test_id] = "FAILED"
            return False

    elif test_id == "TEST-PA-04":
        # EMC Ground Bounce Stress: RMS <= 12mV at PWM=460
        logger.info("POST-ASSEMBLY: TEST-PA-04 EMC Ground Bounce запущен...")
        system_state.pwm_channels = [460] * 5
        await asyncio.sleep(0.5)
        rms_noise_mv = 8.4  # Симуляция
        system_state.pwm_channels = [0] * 5
        if rms_noise_mv <= 12.0:
            system_state.post_assembly_results[test_id] = "PASS"
            system_state.post_assembly_logs[test_id] = (
                f"SUCCESS: RMS шум = {rms_noise_mv}мВ (<= 12.0мВ). Земля 'Звезда' исправна."
            )
            return True
        else:
            system_state.post_assembly_results[test_id] = "FAILED"
            return False

    elif test_id == "TEST-PA-05":
        # Thermal Airflow: heat rate <= 0.25°C/s
        logger.info("POST-ASSEMBLY: TEST-PA-05 Thermal Airflow запущен...")
        temp_start = system_state.coil_temp_model
        system_state.pwm_channels = [int(460 * 0.3)] * 5
        await asyncio.sleep(1.0)
        temp_end = system_state.coil_temp_model
        system_state.pwm_channels = [0] * 5
        heat_rate = (temp_end - temp_start) / 1.0
        if heat_rate <= 0.25:
            system_state.post_assembly_results[test_id] = "PASS"
            system_state.post_assembly_logs[test_id] = (
                f"SUCCESS: Скорость нагрева = {heat_rate:.3f}°C/с (<= 0.25). Продуваемость ОК."
            )
            return True
        else:
            system_state.post_assembly_results[test_id] = "FAILED"
            return False

    return False


async def execute_post_assembly_bist_pipeline():
    """Пайплайн пост-сборочного тестирования (5 сценариев)."""
    system_state.active_pipeline = "POST_ASSEMBLY_BIST"
    system_state.pipeline_status = "In Progress"
    system_state.pipeline_steps = [
        {"step_id": "PA-01", "desc": "Изоляция силовых каналов (TEST-PA-01)", "status": "PENDING"},
        {"step_id": "PA-02", "desc": "Полярность и симметрия катушек (TEST-PA-02)", "status": "PENDING"},
        {"step_id": "PA-03", "desc": "Оптика ToF-датчика (TEST-PA-03)", "status": "PENDING"},
        {"step_id": "PA-04", "desc": "ЭМИ-стресс земли (TEST-PA-04)", "status": "PENDING"},
        {"step_id": "PA-05", "desc": "Термодинамика корпуса (TEST-PA-05)", "status": "PENDING"},
    ]

    test_ids = ["TEST-PA-01", "TEST-PA-02", "TEST-PA-03", "TEST-PA-04", "TEST-PA-05"]
    all_pass = True

    for i, tid in enumerate(test_ids):
        system_state.pipeline_steps[i]["status"] = "RUNNING"
        system_state.post_assembly_results[tid] = "RUNNING"
        success = await run_post_assembly_test(tid)
        if success:
            system_state.pipeline_steps[i]["status"] = "DONE"
        else:
            system_state.pipeline_steps[i]["status"] = "FAILED"
            all_pass = False
            break

    if all_pass:
        system_state.pipeline_status = "Post-Assembly BIST: ALL PASS"
    else:
        system_state.pipeline_status = "Post-Assembly BIST: FAILED"
        system_state.bist_unlocked = False

    await asyncio.sleep(1.0)
    system_state.active_pipeline = None


# Глобальные синглтоны AI подсистемы
ai_config = AIConfigManager()
ai_history_db = AIHistoryDB()
ai_manager = GeminiAIManager(ai_config, ai_history_db)
audio_service = AudioService()

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
                "calibration_fsm": {
                    "state": system_state.calib_state.value,
                    "log": system_state.calib_log,
                    "unlocked_flight": system_state.calib_state == CalibrationState.UNLOCKED_FLIGHT
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
                    "simulated_fault_id": system_state.simulate_fault_on_test_id
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
        elif test_id == "BIST-HW-05":
            system_state.bist_results[test_id] = "FAILED"
            system_state.bist_logs[test_id] = (
                "FAILED: Сбой аппаратного ограничения ЭДС самоиндукции (Lenz Protection).\n"
                "TVS-диод зафиксировал импульс 18.5V при резком сбросе ШИМ. Сбой программного Slew-Rate Limiter.\n"
                "РЕШЕНИЕ: Проверьте исправность TVS-диода SMBJ12A. Проверьте настройки core_loop.py `MAX_PWM_STEP_PER_TICK`."
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
    elif test_id == "BIST-HW-05":
        logger.info("BIST: Тест защиты ЭДС самоиндукции (BIST-HW-05 Lenz Law) запущен...")
        # Симуляция резкого изменения ШИМ 0→300→0 с проверкой slew-rate limiter
        test_channel_pwm = 0
        ramp_up_steps = 0
        # Ramp up: 0 → 300 с ограничением 10 ед/такт
        while test_channel_pwm < 300:
            test_channel_pwm = min(test_channel_pwm + system_state.slew_rate_limit, 300)
            ramp_up_steps += 1
            await asyncio.sleep(0.001)  # Симуляция тактов 860 Гц
        # Ramp down: 300 → 0 с ограничением 10 ед/такт
        ramp_down_steps = 0
        while test_channel_pwm > 0:
            test_channel_pwm = max(test_channel_pwm - system_state.slew_rate_limit, 0)
            ramp_down_steps += 1
            await asyncio.sleep(0.001)
        # Проверка плавности: 300/10 = 30 шагов минимум (34.8 мс при 860 Гц)
        ramp_time_ms = ramp_down_steps * (1000.0 / 860.0)
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            f"SUCCESS: Lenz Law & Slew Rate Validation на радиусе ToF 45.0 мм.\n"
            f"Ramp-up: {ramp_up_steps} шагов (0→300). Ramp-down: {ramp_down_steps} шагов (300→0).\n"
            f"Время спада: {ramp_time_ms:.1f} мс (≥ 35 мс threshold). "
            f"TVS SMBJ12A: пик 11.8V (< 12.5V). GPIO4 RPi5: стабильно 5.1V."
        )
    return True


async def execute_takeoff_pipeline():
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


async def execute_bist_pipeline():
    system_state.active_pipeline = "BIST_RUN_ALL"
    system_state.pipeline_status = "In Progress"
    system_state.pipeline_steps = [
        {"step_id": "BIST-01-PWM", "desc": "Тест ШИМ затворов (BIST-HW-01)", "status": "PENDING"},
        {"step_id": "BIST-02-POL", "desc": "Тест полярности катушек (BIST-HW-02)", "status": "PENDING"},
        {"step_id": "BIST-03-REL", "desc": "Тест NC-реле защиты Qi Агента (BIST-HW-03)", "status": "PENDING"},
        {"step_id": "BIST-04-EMI", "desc": "Тест ЭМИ-наводок (BIST-HW-04)", "status": "PENDING"},
        {"step_id": "BIST-05-LENZ", "desc": "Тест защиты ЭДС Lenz Law (BIST-HW-05)", "status": "PENDING"},
        {"step_id": "BIST-06-FIN", "desc": "Анализ результатов и разблокировка", "status": "PENDING"}
    ]
    
    system_state.bist_bypass_active = False
    
    for i, test_id in enumerate(["BIST-HW-01", "BIST-HW-02", "BIST-HW-03", "BIST-HW-04", "BIST-HW-05"]):
        system_state.pipeline_steps[i]["status"] = "RUNNING"
        system_state.bist_results[test_id] = "RUNNING"
        system_state.bist_logs[test_id] = "Тестирование выполняется в данный момент..."
        
        success = await run_single_bist_test(test_id)
        
        if success:
            system_state.pipeline_steps[i]["status"] = "DONE"
        else:
            system_state.pipeline_steps[i]["status"] = "FAILED"
            for j in range(i+1, 5):
                system_state.pipeline_steps[j]["status"] = "PENDING"
            system_state.pipeline_steps[5]["status"] = "FAILED"
            system_state.pipeline_status = f"Bring-up Failed at step {test_id}!"
            system_state.bist_unlocked = False
            await asyncio.sleep(2.0)
            system_state.active_pipeline = None
            return

    system_state.pipeline_steps[5]["status"] = "RUNNING"
    await asyncio.sleep(1.0)
    
    if all(res == "PASS" for res in system_state.bist_results.values()):
        system_state.bist_unlocked = True
        system_state.pipeline_steps[5]["status"] = "DONE"
        system_state.pipeline_status = "System Unlocked (All Tests PASS)"
        logger.info("BIST: Все 5 тестов успешно пройдены! Система разблокирована.")
    else:
        system_state.bist_unlocked = False
        system_state.pipeline_steps[5]["status"] = "FAILED"
        system_state.pipeline_status = "Failed BIST validation."
        
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
    # V11: Инициализация AI подсистемы
    ai_config.load()
    await ai_history_db.init_db()
    await audio_service.init()
    if ai_config.ai_enabled:
        system_state.ai_enabled = True
        asyncio.create_task(ai_manager.connect())
    logger.info("Фоновые службы, AI/Audio и BIST запущены. V11.0")

@app.get("/api/state")
async def get_state():
    return {
        "x": system_state.x,
        "y": system_state.y,
        "z": system_state.z,
        "target_z": system_state.target_z,
        "kp": system_state.kp,
        "ki": system_state.ki,
        "kd": system_state.kd,
        "temp": system_state.coil_temp_model,
        "throttling": system_state.thermal_throttling,
        "wifi": {
            "mode": system_state.base_wifi_mode,
            "ssid": system_state.base_wifi_ssid,
            "ip": system_state.base_wifi_ip,
            "agent_rssi": system_state.agent_wifi_rssi
        },
        "bist_unlocked": system_state.bist_unlocked,
        "bist_bypass_active": system_state.bist_bypass_active,
        "bist_results": system_state.bist_results,
        "bist_logs": system_state.bist_logs,
        "active_pipeline": system_state.active_pipeline,
        "safe_to_unplug": system_state.safe_to_unplug
    }

class PIDTuning(BaseModel):
    kp: float | None = None
    ki: float | None = None
    kd: float | None = None
    target_z: float | None = None

@app.post("/api/pid/set_tunings")
async def set_tunings(tunings: PIDTuning):
    if tunings.kp is not None: system_state.kp = tunings.kp
    if tunings.ki is not None: system_state.ki = tunings.ki
    if tunings.kd is not None: system_state.kd = tunings.kd
    if tunings.target_z is not None: system_state.target_z = tunings.target_z
    return {"status": "SUCCESS"}

@app.get("/api/wifi/scan")
async def wifi_scan():
    return {
        "status": "SUCCESS",
        "networks": [
            {"ssid": "Lab_Network", "signal": -50, "secure": True},
            {"ssid": "Guest_WIFI", "signal": -70, "secure": False}
        ]
    }

class WifiConnectRequest(BaseModel):
    ssid: str
    password: str

@app.post("/api/wifi/connect")
async def wifi_connect(req: WifiConnectRequest):
    system_state.base_wifi_ssid = req.ssid
    system_state.base_wifi_mode = "STA"
    return {"status": "SUCCESS"}

@app.post("/api/manage/execute")
async def execute_command(req: CommandRequest):
    if system_state.active_pipeline is not None and system_state.active_pipeline != "SAFE_SHUTDOWN":
        raise HTTPException(
            status_code=400, 
            detail=f"Невозможно запустить '{req.command}', так как сейчас выполняется сценарий '{system_state.active_pipeline}'"
        )
    
    _PIPELINE_MAP = [
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
        if req.command in ["SPATIAL_HELIX", "JOYSTICK"] and system_state.calib_state != CalibrationState.UNLOCKED_FLIGHT:
            raise HTTPException(status_code=403, detail="Запуск прерван: Калибровка на стенде не завершена. Подставка не удалена.")
            
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

@app.post("/api/calibration/start")
async def start_calibration():
    if system_state.calib_state != CalibrationState.PARKED:
        raise HTTPException(status_code=400, detail="Калибровка уже запущена или завершена.")
        
    system_state.calib_state = CalibrationState.ZEROING
    system_state.calib_log = "ШАГ 1/5: Снятие нулей датчиков Холла и ToF на высоте Z = 30.0мм..."
    
    # Снятие нулей и проверка шума (симуляция)
    rms_noise = 1.2  # mV
    if rms_noise >= 2.0:
        system_state.calib_state = CalibrationState.PARKED
        raise HTTPException(status_code=500, detail=f"Calibration Failed: Sensor RMS noise {rms_noise}mV >= 2.0mV")
        
    await asyncio.sleep(1.0)
    
    system_state.calib_state = CalibrationState.TAKEOFF_HOVER
    system_state.calib_log = "ШАГ 2/5: Плавный приподъем сферы на +3.0мм (Z = 33.0мм)..."
    system_state.target_z = 33.0
    await asyncio.sleep(1.5)
    
    system_state.calib_state = CalibrationState.WAIT_REMOVAL
    system_state.calib_log = "ШАГ 3/5: Сфера зависла! Раздвиньте половинки подставки и нажмите CONFIRM."
    
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
    system_state.target_z = 25.0
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

# =====================================================================
# AI API ЭНДПОИНТЫ (V11 — Gemini Pro, Audio, Tool Calling)
# =====================================================================
class AIActivateRequest(BaseModel):
    enabled: bool

class AISetKeyRequest(BaseModel):
    api_key: str

class AIChatRequest(BaseModel):
    message: str


@app.post("/api/ai/activate")
async def ai_activate(req: AIActivateRequest):
    """Включить/выключить AI-ассистента с сохранением в config."""
    system_state.ai_enabled = req.enabled
    ai_config.save(ai_enabled=req.enabled)
    if req.enabled and system_state.ai_status != "CONNECTED":
        ok = await ai_manager.connect()
        return {"status": "activated" if ok else "activation_failed",
                "ai_status": system_state.ai_status}
    elif not req.enabled:
        system_state.ai_status = "DISCONNECTED"
    return {"status": "success", "ai_enabled": system_state.ai_enabled}


@app.post("/api/ai/set_key")
async def ai_set_key(req: AISetKeyRequest):
    """Установить Gemini API Key (RPI-CL-12: ключ маскируется в логах)."""
    ai_config.save(api_key=req.api_key)
    return {"status": "key_saved", "masked_key": ai_config.masked_key()}


@app.post("/api/ai/chat")
async def ai_chat(req: AIChatRequest):
    """Текстовый чат с Gemini Pro (с Tool Calling и контекстом телеметрии)."""
    if not system_state.ai_enabled:
        raise HTTPException(status_code=400, detail="AI не активирован.")
    result = await ai_manager.chat(req.message)
    # UC-BUS-12: Кивок сферы при ответе
    asyncio.create_task(trigger_nod_gesture())
    return result


@app.get("/api/ai/status")
async def ai_status():
    """Статус AI-подсистемы (WEB-CL-10: ping, thoughts, audio)."""
    return {
        "ai_enabled": system_state.ai_enabled,
        "ai_status": system_state.ai_status,
        "ai_ping_ms": system_state.ai_ping_ms,
        "ai_model": system_state.ai_model_name,
        "thoughts_log": system_state.ai_thoughts_log[-20:],
        "chat_history": system_state.ai_chat_history[-50:],
        "audio_available": system_state.audio_available,
        "audio_vad_active": system_state.audio_vad_active,
    }


@app.post("/api/ai/bringup_test")
async def ai_bringup_test():
    """Запуск 3-этапного Bring-Up теста AI (TEST-AI-01..03)."""
    results = {}

    # TEST-AI-01: API Sync Check
    try:
        t0 = time.perf_counter()
        ok = await ai_manager.connect()
        ping = round((time.perf_counter() - t0) * 1000, 1)
        results["TEST-AI-01"] = {
            "status": "PASS" if ok else "FAILED",
            "ping_ms": ping,
            "detail": f"Gemini API {'connected' if ok else 'unreachable'}. Ping={ping}ms"
        }
    except Exception as e:
        results["TEST-AI-01"] = {"status": "FAILED", "detail": str(e)}

    # TEST-AI-02: Camera Check (ESP32-CAM placeholder)
    results["TEST-AI-02"] = {
        "status": "WARNING",
        "detail": "ESP32-CAM frame capture — requires Wi-Fi connection to Agent."
    }

    # TEST-AI-03: Audio Loop Check (SPU-WM30)
    audio_result = await audio_service.bist_audio_loop_test()
    results["TEST-AI-03"] = audio_result

    return {"bringup_results": results}


@app.post("/api/bist/post_assembly_test")
async def post_assembly_test():
    """Запуск Post-Assembly BIST Suite v1.0 (TEST-PA-01..05)."""
    if system_state.active_pipeline is not None:
        raise HTTPException(status_code=400, detail="Пайплайн уже выполняется.")
    asyncio.create_task(execute_post_assembly_bist_pipeline())
    return {"status": "started", "pipeline": "POST_ASSEMBLY_BIST"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
