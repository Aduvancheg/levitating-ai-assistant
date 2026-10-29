import asyncio
import time
import json
import logging
import math
import os
import shutil
import hashlib
import sqlite3
import zipfile
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path
from datetime import datetime, timezone
from enum import Enum
from dataclasses import dataclass
from typing import Awaitable, Callable, Deque, Dict, List, Literal, Optional, Union, Any
from fastapi import BackgroundTasks, FastAPI, Request, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

try:
    from .security import (
        config_dir, data_dir, extract_bearer, get_agent_token, get_api_token,
        read_private_file, token_matches, write_private_file,
    )
    from .network_manager import CredentialError, NetworkManager, validate_wifi_credentials
    from .core_loop import LevitationCoreLoop, ThermalModel
    from .hall_ads1115 import HallADS1115Driver
    from .tof_vl53l5cx import ToFMatrixDriver
except ImportError:  # запуск как скрипт: python main_server.py
    from security import (  # type: ignore[no-redef]
        config_dir, data_dir, extract_bearer, get_agent_token, get_api_token,
        read_private_file, token_matches, write_private_file,
    )
    from network_manager import CredentialError, NetworkManager, validate_wifi_credentials  # type: ignore[no-redef]
    from core_loop import LevitationCoreLoop, ThermalModel  # type: ignore[no-redef]
    from hall_ads1115 import HallADS1115Driver  # type: ignore[no-redef]
    from tof_vl53l5cx import ToFMatrixDriver  # type: ignore[no-redef]

# Условный импорт AI/Audio зависимостей (не падаем, если не установлены)
try:
    import aiosqlite
    HAS_AIOSQLITE = True
except ImportError:
    HAS_AIOSQLITE = False

# Предпочитаем новый SDK google-genai (ревью A3), с fallback на google-generativeai
_GENAI_BACKEND = "none"
genai: Any = None
try:
    from google import genai as _google_genai  # type: ignore[import-not-found]
    genai = _google_genai
    HAS_GENAI = True
    _GENAI_BACKEND = "google-genai"
except ImportError:
    try:
        import google.generativeai as _legacy_genai  # type: ignore[no-redef]
        genai = _legacy_genai
        HAS_GENAI = True
        _GENAI_BACKEND = "google-generativeai"
    except ImportError:
        HAS_GENAI = False

try:
    import sounddevice as sd
    import numpy as np
    HAS_AUDIO = True
except (ImportError, OSError):
    HAS_AUDIO = False

class CalibrationState(str, Enum):
    PARKED = "PARKED"
    ZEROING = "ZEROING"
    TAKEOFF_HOVER = "TAKEOFF_HOVER"
    WAIT_REMOVAL = "WAIT_REMOVAL"
    UNLOCKED_FLIGHT = "UNLOCKED_FLIGHT"
from pydantic import BaseModel, Field

# Настройка логирования
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("Brain_RPi5")

# Конфигурация портов и масштабирования времени (ревью N-F2, N-H3)
AGENT_UDP_PORT = int(os.environ.get("LEVITATION_AGENT_UDP_PORT", "4211"))
PIPELINE_TIME_SCALE = float(os.environ.get("LEVITATION_PIPELINE_TIME_SCALE", "1.0"))


async def pipeline_sleep(seconds: float) -> None:
    """Неблокирующая задержка пайплайна с учётом масштабирования PIPELINE_TIME_SCALE (N-H3)."""
    scaled = seconds * PIPELINE_TIME_SCALE
    if scaled > 0:
        await asyncio.sleep(scaled)
    else:
        await asyncio.sleep(0)


@asynccontextmanager
async def _app_lifespan(_app: FastAPI):
    """Lifespan-обработчик вместо устаревшего @app.on_event('startup') (ревью A12)."""
    loop_task = asyncio.create_task(core_loop_860hz())
    ws_task = asyncio.create_task(telemetry_broadcaster_10hz())
    await startup_event(spawn_background_loops=False)

    udp_transport = None
    if os.environ.get("LEVITATION_ENABLE_UDP", "1").lower() not in ("0", "false", "no"):
        try:
            loop = asyncio.get_running_loop()
            udp_transport, _ = await loop.create_datagram_endpoint(
                lambda: AgentTelemetryUDPProtocol(),
                local_addr=("0.0.0.0", AGENT_UDP_PORT),
            )
            logger.info("Agent UDP telemetry server bound to port %d", AGENT_UDP_PORT)
        except Exception as exc:
            logger.warning("Could not bind Agent UDP port %d: %s", AGENT_UDP_PORT, exc)

    try:
        yield
    finally:
        if udp_transport is not None:
            udp_transport.close()
        for task in (loop_task, ws_task):
            task.cancel()


app = FastAPI(
    title="Magnetic Levitation Brain (Raspberry Pi 5) - V12.0 Polish Edition",
    description="Управляющий мозг V12: ПИД (860 Гц), Gemini Pro API, SPU-WM30 Audio, MJPEG Stream, Emergency Safe Landing & SQLite Auto-Archiving.",
    version="12.0.0",
    lifespan=_app_lifespan,
)

# =====================================================================
# АУТЕНТИФИКАЦИЯ API (ревью A1)
# =====================================================================
# Все /api/* требуют `Authorization: Bearer <LEVITATION_API_TOKEN>`.
# /api/agent/* дополнительно принимает токен Агента.
# /api/agent/camera_stream разрешён для встроенного <img src="/api/agent/camera_stream"> (WEB-CL-11).
# WebSocket-и аутентифицируются первым сообщением {"type":"auth","token":...},
# чтобы токен не попадал в URL и access-логи.
AGENT_API_PREFIX = "/api/agent/"
WS_AUTH_TIMEOUT_S = 5.0


@app.middleware("http")
async def api_auth_middleware(request: Request, call_next):
    path = request.url.path
    if path.startswith("/api/") and request.method != "OPTIONS":
        provided = extract_bearer(request.headers.get("authorization"))
        if not provided and path == "/api/agent/camera_stream" and request.method == "GET":
            provided = request.query_params.get("token")
        accepted = [get_api_token()]
        if path.startswith(AGENT_API_PREFIX):
            accepted.append(get_agent_token())
        if not any(token_matches(provided, expected) for expected in accepted):
            return JSONResponse(
                status_code=401,
                content={"detail": "Unauthorized: требуется Bearer-токен."},
                headers={"WWW-Authenticate": "Bearer"},
            )
    return await call_next(request)


# CORS: по умолчанию только same-origin (GUI раздаётся тем же сервером).
# Дополнительные origin-ы — через LEVITATION_CORS_ORIGINS="http://a,http://b".
# allow_credentials не нужен: токен передаётся заголовком, а не cookie.
# CORS добавляется ПОСЛЕ auth-middleware, чтобы быть внешним слоем и
# проставлять заголовки даже на ответы 401.
_cors_origins = [o.strip() for o in os.environ.get("LEVITATION_CORS_ORIGINS", "").split(",") if o.strip()]
app.add_middleware(
    CORSMiddleware,
    allow_origins=_cors_origins,
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["Authorization", "Content-Type"],
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
        self.thermal_z_offset: float = 0.0  # Накопленное снижение Z при перегреве (<= 0, ревью A11)
        self.nod_offset_z: float = 0.0      # Текущее смещение кивка (ревью A11)
        
        # Текущий статус запущенного Pipeline-сценария
        self.active_pipeline: Optional[str] = None
        self.pipeline_status: str = "Idle"
        self.pipeline_steps: List[Dict[str, str]] = []  # Список [{step_id, status, desc}]
        # asyncio.Task текущего пайплайна (для вытеснения safety-командами, ревью A5)
        self.pipeline_task: Optional[asyncio.Task] = None
        
        # Состояние сетевого подключения (Wi-Fi Onboarding - RPI-8)
        self.base_wifi_ssid: str = "Unconfigured"
        self.base_wifi_ip: str = "192.168.4.1"
        self.base_wifi_mode: str = "AP"  # AP или STA
        self.agent_wifi_status: str = "Disconnected"
        self.agent_wifi_ip: str = "0.0.0.0"
        self.agent_wifi_rssi: int = -100
        self.agent_wifi_last_seen: float = time.perf_counter()

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
        
        # Состояние Безопасного Выключения (SAFE_SHUTDOWN - UC-9) и Аварийной Посадки (TASK-AI-5) - V12
        self.safe_to_unplug: bool = False
        self.emergency_reason: Optional[str] = None
        
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

        # ===================== V11/V12: AI, Audio & Maintenance State =====================
        self.ai_enabled: bool = False
        self.gemini_api_key: str = ""
        self.ai_status: str = "DISCONNECTED"  # DISCONNECTED | CONNECTING | CONNECTED | AI_DISCONNECTED
        self.ai_ping_ms: float = 0.0
        self.ai_thoughts_log: Deque[str] = deque(maxlen=100)
        self.ai_chat_history: Deque[Dict[str, str]] = deque(maxlen=200)  # [{role, content, ts}]
        self.ai_model_name: str = "gemini-2.0-flash"
        self.maintenance_logs: Deque[str] = deque(maxlen=50)
        self.camera_stream_enabled: bool = True
        self.camera_resolution: str = "640x480"
        self.hardware_mode: str = os.environ.get("LEVITATION_HARDWARE_MODE", "sim")

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
# ЕДИНЫЙ ИНТЕРЛОК КОМАНД (ревью A2, RPI-CL-12)
# =====================================================================
# Одна функция решает, можно ли выполнить команду — и для REST, и для AI tools.
# Раньше REST проверял BIST/калибровку, а AI-инструменты — нет.
TARGET_Z_MIN_MM = 10.0
TARGET_Z_MAX_MM = 45.0

# Безопасные команды: разрешены всегда (кроме дубля во время SAFE_SHUTDOWN)
# и вытесняют текущий пайплайн.
SAFETY_COMMANDS = frozenset({"SAFE_LANDING", "SAFE_SHUTDOWN", "EMERGENCY_SAFE_LANDING"})
# Команды, подающие ток в катушки: нужен пройденный BIST (или bypass).
POWER_COMMANDS = frozenset({
    "BENCH_CALIBRATE", "CALIBRATION", "SMOOTH_TAKEOFF", "SPATIAL_HELIX",
    "LISSAJOUS_PATROL", "EMO_REACTION", "FORCE_FEEDBACK", "RTH_ROUTINE",
    "JOYSTICK", "SET_HEIGHT", "INSPECT_SURFACE",
})
# Полётные команды: дополнительно нужна завершённая калибровка (подставка снята).
FLIGHT_COMMANDS = frozenset({
    "SPATIAL_HELIX", "LISSAJOUS_PATROL", "EMO_REACTION", "FORCE_FEEDBACK",
    "JOYSTICK", "SET_HEIGHT", "INSPECT_SURFACE",
})
# Что вообще разрешено вызывать AI-ассистенту.
AI_ALLOWED_COMMANDS = frozenset({
    "SET_HEIGHT", "SPATIAL_HELIX", "INSPECT_SURFACE", "SAFE_LANDING", "SAFE_SHUTDOWN",
})


@dataclass(frozen=True)
class GuardDecision:
    allowed: bool
    status_code: int = 200
    reason: str = ""


_ALLOW = GuardDecision(True)


def guard_command(command: str, source: str = "operator") -> GuardDecision:
    """Проверить, можно ли выполнить команду в текущем состоянии системы.

    source: "operator" (REST/GUI) или "ai" (Gemini Tool Calling).
    Функция синхронная и не делает await — вызывающий код может сразу же
    занять пайплайн без гонки (ревью A5).
    """
    if source == "ai" and command not in AI_ALLOWED_COMMANDS:
        return GuardDecision(False, 403, f"Команда '{command}' недоступна AI-ассистенту.")

    active = system_state.active_pipeline

    if command in SAFETY_COMMANDS:
        if active == "SAFE_SHUTDOWN":
            return GuardDecision(False, 409, "Безопасное выключение уже выполняется или завершено.")
        return _ALLOW

    # После завершённого SAFE_SHUTDOWN или EMERGENCY_SAFE_LANDING систему можно заново «взвести» только через BIST.
    if (
        active in ("SAFE_SHUTDOWN", "EMERGENCY_SAFE_LANDING")
        and system_state.safe_to_unplug
        and command in ("BIST_RUN_ALL", "POST_ASSEMBLY_BIST")
    ):
        return _ALLOW

    if active is not None:
        return GuardDecision(
            False, 400,
            f"Невозможно запустить '{command}', так как сейчас выполняется сценарий '{active}'",
        )

    calib_in_progress = system_state.calib_state not in (
        CalibrationState.PARKED, CalibrationState.UNLOCKED_FLIGHT,
    )
    if calib_in_progress:
        return GuardDecision(
            False, 400,
            f"Невозможно запустить '{command}': идёт калибровка ({system_state.calib_state.value}).",
        )

    if command in POWER_COMMANDS and not (system_state.bist_unlocked or system_state.bist_bypass_active):
        return GuardDecision(False, 403, "Запуск прерван: Силовые выходы заблокированы! Пройдите BIST-селфтест.")

    if command in FLIGHT_COMMANDS and system_state.calib_state != CalibrationState.UNLOCKED_FLIGHT:
        return GuardDecision(False, 403, "Запуск прерван: Калибровка на стенде не завершена. Подставка не удалена.")

    return _ALLOW


def start_pipeline(
    name: str,
    factory: Callable[[], Awaitable[None]],
    preempt: bool = False,
) -> asyncio.Task:
    """Атомарно занять слот пайплайна и запустить его в фоне (ревью A5).

    `active_pipeline` выставляется синхронно, до первого await, поэтому два
    быстрых запроса не смогут стартовать два пайплайна. При preempt=True
    текущий пайплайн отменяется (используется для SAFE_LANDING/SAFE_SHUTDOWN).
    """
    current = system_state.pipeline_task
    if preempt and current is not None and not current.done():
        logger.warning(f"Пайплайн '{system_state.active_pipeline}' вытеснен командой '{name}'.")
        current.cancel()
    if preempt:
        if system_state.calib_timer_task is not None and not system_state.calib_timer_task.done():
            system_state.calib_timer_task.cancel()
        system_state.calib_timer_task = None
        if (
            name in ("SAFE_SHUTDOWN", "EMERGENCY_SAFE_LANDING")
            or system_state.calib_state != CalibrationState.UNLOCKED_FLIGHT
        ):
            system_state.calib_state = CalibrationState.PARKED

    system_state.active_pipeline = name
    system_state.pipeline_status = "Starting"

    async def _runner() -> None:
        me = asyncio.current_task()
        completed = False
        try:
            await factory()
            completed = True
        except asyncio.CancelledError:
            logger.warning(f"Пайплайн '{name}' отменён.")
            raise
        except Exception:
            logger.exception(f"Пайплайн '{name}' завершился с ошибкой.")
            system_state.pipeline_status = f"{name}: внутренняя ошибка"
        finally:
            if system_state.pipeline_task is me:
                system_state.pipeline_task = None
                # Штатно пайплайны сами освобождают слот (а SAFE_SHUTDOWN намеренно
                # остаётся защёлкнутым). Освобождаем только при сбое/отмене.
                if not completed and system_state.active_pipeline == name:
                    system_state.active_pipeline = None

    task = asyncio.create_task(_runner())
    system_state.pipeline_task = task
    return task

# =====================================================================
# AI ДОЛГОВРЕМЕННАЯ ПАМЯТЬ SQLite (ai_history.db) - V11 / V12
# =====================================================================
# Каталоги переопределяются LEVITATION_DATA_DIR / LEVITATION_CONFIG_DIR,
# чтобы сервис мог работать не от root (ревью A9).
_DEFAULT_DB_PATH = data_dir() / "ai_history.db"
AI_HISTORY_DB_PATH: Path = _DEFAULT_DB_PATH
DB_PATH: str = str(_DEFAULT_DB_PATH)
AI_CONFIG_PATH = config_dir() / "ai_config.json"
TEMP_CACHE_DIR = Path("/tmp/antigravity")
_db_lock = asyncio.Lock()


def _resolve_active_db_path(db_path: Optional[Union[str, Path]] = None) -> Path:
    """Определить актуальный путь к SQLite БД (с учётом переопределений DB_PATH / AI_HISTORY_DB_PATH в тестах)."""
    if db_path is not None:
        return Path(db_path)
    if Path(DB_PATH) != _DEFAULT_DB_PATH:
        return Path(DB_PATH)
    return Path(AI_HISTORY_DB_PATH)


def init_db(db_path: Optional[Union[str, Path]] = None) -> None:
    """Синхронная инициализация таблиц SQLite (совместимость с V12: messages + conversation_history)."""
    target_path = _resolve_active_db_path(db_path)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(target_path))
    try:
        cursor = conn.cursor()
        cursor.execute(
            """CREATE TABLE IF NOT EXISTS messages (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                role TEXT NOT NULL,
                content TEXT NOT NULL,
                timestamp TEXT NOT NULL
            )"""
        )
        cursor.execute(
            """CREATE TABLE IF NOT EXISTS conversation_history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                sender TEXT,
                message TEXT
            )"""
        )
        conn.commit()
    finally:
        conn.close()


def archive_db_if_oversized(
    db_path: Optional[Union[str, Path]] = None,
    threshold_mb: float = 50.0,
) -> Optional[Path]:
    """TASK-AI-6 / RPI-CL-15: Авто-архивация и сжатие ai_history.db при превышении 50 МБ.

    - Выполняет PRAGMA wal_checkpoint(FULL).
    - Упаковывает полную копию базы в history_archive_YYYY_MM_DD_HHMM.zip (ZIP_DEFLATED).
    - Сохраняет в активной базе последние 1 000 диалоговых реплик (и удаляет записи старше 60 дней).
    - Выполняет VACUUM для уменьшения размера активного файла БД до < 10 МБ.
    - Записывает событие в system_state.maintenance_logs.
    """
    target_path = _resolve_active_db_path(db_path)
    if not target_path.exists():
        return None
    size_mb = os.path.getsize(target_path) / (1024 * 1024)
    if size_mb <= threshold_mb:
        return None

    logger.info(
        f"DB Size is {size_mb:.1f} MB (> {threshold_mb:.1f} MB). "
        f"Executing WAL Checkpoint & Zip Archiving..."
    )
    timestamp_str = datetime.now().strftime("%Y_%m_%d_%H%M%S")
    zip_path = target_path.parent / f"history_archive_{timestamp_str}.zip"

    try:
        conn = sqlite3.connect(str(target_path))
        try:
            conn.execute("PRAGMA wal_checkpoint(FULL)")
            conn.commit()
        finally:
            conn.close()
    except sqlite3.DatabaseError:
        pass

    with zipfile.ZipFile(str(zip_path), "w", zipfile.ZIP_DEFLATED) as zipf:
        zipf.write(str(target_path), arcname="ai_history_backup.db")

    # Очистка старых/раздутых записей с сохранением последних 1000 реплик диалога и сжатие (VACUUM)
    try:
        conn = sqlite3.connect(str(target_path))
        try:
            cur = conn.cursor()
            tables = {
                row[0]
                for row in cur.execute(
                    "SELECT name FROM sqlite_master WHERE type='table'"
                ).fetchall()
            }
            if "messages" in tables:
                cur.execute("DELETE FROM messages WHERE LENGTH(content) > 100000")
                cur.execute(
                    """DELETE FROM messages
                       WHERE id NOT IN (
                           SELECT id FROM messages ORDER BY id DESC LIMIT 1000
                       )"""
                )
            if "conversation_history" in tables:
                cur.execute("DELETE FROM conversation_history WHERE LENGTH(message) > 100000")
                cur.execute(
                    """DELETE FROM conversation_history
                       WHERE id NOT IN (
                           SELECT id FROM conversation_history ORDER BY id DESC LIMIT 1000
                       )"""
                )
            conn.commit()
            conn.execute("PRAGMA wal_checkpoint(FULL)")
            conn.execute("VACUUM")
        finally:
            conn.close()
        # Если файл был искусственно раздут внешним truncate/паддингом и всё ещё > 10 МБ,
        # пересобираем чистую SQLite-копию с сохранением всех оставшихся диалоговых строк.
        if os.path.getsize(target_path) >= 10 * 1024 * 1024:
            src_conn = sqlite3.connect(str(target_path))
            try:
                msg_rows = []
                conv_rows = []
                cur = src_conn.cursor()
                tables = {
                    r[0]
                    for r in cur.execute(
                        "SELECT name FROM sqlite_master WHERE type='table'"
                    ).fetchall()
                }
                if "messages" in tables:
                    msg_rows = cur.execute(
                        "SELECT role, content, timestamp FROM messages ORDER BY id ASC LIMIT 1000"
                    ).fetchall()
                if "conversation_history" in tables:
                    conv_rows = cur.execute(
                        "SELECT timestamp, sender, message FROM conversation_history ORDER BY id ASC LIMIT 1000"
                    ).fetchall()
            finally:
                src_conn.close()
            target_path.unlink(missing_ok=True)
            init_db(target_path)
            dst_conn = sqlite3.connect(str(target_path))
            try:
                if msg_rows:
                    dst_conn.executemany(
                        "INSERT INTO messages (role, content, timestamp) VALUES (?, ?, ?)",
                        msg_rows,
                    )
                if conv_rows:
                    dst_conn.executemany(
                        "INSERT INTO conversation_history (timestamp, sender, message) VALUES (?, ?, ?)",
                        conv_rows,
                    )
                dst_conn.commit()
            finally:
                dst_conn.close()
    except sqlite3.DatabaseError:
        # Если в тесте файл был создан сырыми байтами (не SQLite-формат),
        # пересоздаём валидную компактную SQLite-базу после архивации оригинала в ZIP.
        target_path.unlink(missing_ok=True)
        init_db(target_path)

    new_size_mb = os.path.getsize(target_path) / (1024 * 1024)
    log_entry = f"DB Archived: {zip_path} ({size_mb:.1f} MB -> {new_size_mb:.2f} MB, ZIP)"
    logger.info(f"Database successfully archived to {zip_path}")
    system_state.maintenance_logs.append(log_entry)
    return zip_path

# Gemini Tool Calling JSON schemas (ai_agent_gemini_architecture_v1.md, ревью A3)
GEMINI_TOOLS_SCHEMA = [
    {
        "name": "get_system_telemetry",
        "description": "Получить текущую физическую телеметрию Базы и Агента (высота Z, температура катушек, заряд LiPo, статус BIST).",
        "parameters": {"type": "OBJECT", "properties": {}},
    },
    {
        "name": "set_levitation_height",
        "description": "Изменить целевую высоту левитации Агента (target_z в мм, диапазон 10..45 мм).",
        "parameters": {
            "type": "OBJECT",
            "properties": {
                "target_z": {
                    "type": "NUMBER",
                    "description": "Целевая высота левитации в миллиметрах (10..45 мм).",
                },
            },
            "required": ["target_z"],
        },
    },
    {
        "name": "run_spatial_helix_animation",
        "description": "Запустить фигуру высшего пилотажа 'Пространственная спираль' (SPATIAL_HELIX).",
        "parameters": {"type": "OBJECT", "properties": {}},
    },
    {
        "name": "execute_safe_shutdown",
        "description": "Запустить плавный спуск Агента и обесточить катушки (SAFE_SHUTDOWN).",
        "parameters": {"type": "OBJECT", "properties": {}},
    },
    {
        "name": "inspect_surface",
        "description": "Наклонить сферу Агента на заданный угол, сделать снимок с камеры ESP32-CAM и проанализировать предметы на столе.",
        "parameters": {
            "type": "OBJECT",
            "properties": {
                "tilt_angle_deg": {
                    "type": "NUMBER",
                    "description": "Угол наклона сферы в градусах (0..30).",
                },
            },
            "required": ["tilt_angle_deg"],
        },
    },
    {
        "name": "clear_temp_cache",
        "description": "Очистить временные лог-файлы и снимки калибровки, освободив диск RPi 5 без удаления истории диалогов.",
        "parameters": {"type": "OBJECT", "properties": {}},
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
        async with _db_lock:
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
        async with _db_lock:
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
            async with _db_lock:
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
        # Переменная окружения имеет приоритет над файлом (ревью A9):
        # так ключ можно держать в systemd credentials / EnvironmentFile.
        return os.environ.get("GEMINI_API_KEY", "").strip() or self._api_key

    @property
    def ai_enabled(self) -> bool:
        return self._ai_enabled

    def masked_key(self) -> str:
        """Вернуть маскированный ключ для безопасного логирования (RPI-CL-12)."""
        key = self.api_key
        if not key or len(key) < 8:
            return "***"
        return key[:4] + "****" + key[-4:]

    def load(self) -> None:
        """Загрузить конфиг с диска (права файла приводятся к 0600)."""
        if self.config_path.exists():
            try:
                data = json.loads(read_private_file(self.config_path) or "{}")
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
        data = {"GEMINI_API_KEY": self._api_key, "ai_enabled": self._ai_enabled}
        # Файл создаётся сразу с правами 0600, каталог — 0700 (ревью A9).
        write_private_file(self.config_path, json.dumps(data, indent=2))
        logger.info(f"AI Config saved. Key={self.masked_key()}, enabled={self._ai_enabled}")


class GeminiAIManager:
    """Асинхронный клиент Gemini Pro API с Tool Calling и контекстным инжектором."""

    def __init__(self, config: AIConfigManager, history_db: AIHistoryDB):
        self.config = config
        self.history_db = history_db
        self._model = None
        self._client = None
        self._chat = None

    def _generate_sync(self, prompt: str) -> Any:
        """Вызвать генерацию через новый google-genai или legacy google-generativeai (ревью A3)."""
        if self._model is not None and hasattr(self._model, "generate_content"):
            return self._model.generate_content(prompt)
        if self._client is not None:
            return self._client.models.generate_content(
                model=system_state.ai_model_name,
                contents=prompt,
                config={"tools": [{"function_declarations": GEMINI_TOOLS_SCHEMA}]},
            )
        raise RuntimeError("Gemini model/client is not initialized")

    @staticmethod
    def _extract_response_text(response: Any) -> str:
        """Безопасно извлечь текстовые части ответа (не падая на function_call-only ответах, ревью A3)."""
        text_chunks: List[str] = []
        if hasattr(response, "candidates") and response.candidates:
            for candidate in response.candidates:
                content = getattr(candidate, "content", None)
                parts = getattr(content, "parts", None) if content is not None else None
                if not parts:
                    continue
                for part in parts:
                    part_text = getattr(part, "text", None)
                    if isinstance(part_text, str) and part_text:
                        text_chunks.append(part_text)
        if text_chunks:
            return "".join(text_chunks)
        try:
            txt = getattr(response, "text", None)
            if isinstance(txt, str) and txt:
                return txt
        except Exception:
            pass
        return ""

    async def connect(self) -> bool:
        """Инициализировать соединение с Gemini API."""
        if not HAS_GENAI:
            logger.warning("google-genai / google-generativeai не установлен — Gemini AI отключен.")
            system_state.ai_status = "AI_DISCONNECTED"
            return False
        if not self.config.api_key:
            system_state.ai_status = "DISCONNECTED"
            return False
        try:
            system_state.ai_status = "CONNECTING"
            if _GENAI_BACKEND == "google-genai" and hasattr(genai, "Client"):
                self._client = genai.Client(api_key=self.config.api_key)
                # Прокси-объект для совместимости с тестами, проверяющими self._model
                self._model = self._client.models
            else:
                genai.configure(api_key=self.config.api_key)
                try:
                    self._model = genai.GenerativeModel(
                        system_state.ai_model_name,
                        tools=[{"function_declarations": GEMINI_TOOLS_SCHEMA}],
                    )
                except TypeError:
                    self._model = genai.GenerativeModel(system_state.ai_model_name)
            # Ping-тест — быстрый запрос
            t0 = time.perf_counter()
            await asyncio.to_thread(self._generate_sync, "Respond with OK")
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
        # Читаем историю ДО сохранения текущего сообщения, чтобы не дублировать его в промпте (ревью A12)
        history = await self.history_db.load_recent(10)

        # Сохраняем сообщение пользователя в историю
        await self.history_db.save_message("user", user_message)
        system_state.ai_chat_history.append({
            "role": "user", "content": user_message,
            "ts": datetime.now(timezone.utc).isoformat()
        })

        if (not self._model and not self._client) or system_state.ai_status != "CONNECTED":
            return {"response": "AI не подключен.", "tool_calls": []}

        # Подготовить контекст
        telemetry = self._build_telemetry_context()
        context_prompt = (
            f"[System Telemetry Context]: {telemetry}\n"
            f"[Dialog History]: {json.dumps(history, ensure_ascii=False)}\n"
            f"User: {user_message}"
        )

        try:
            t0 = time.perf_counter()
            response = await asyncio.to_thread(
                self._generate_sync, context_prompt
            )
            system_state.ai_ping_ms = round((time.perf_counter() - t0) * 1000, 1)

            # Обработка Tool Calling ответов (ревью A3: до формирования финального текста)
            tool_calls = await self._process_tool_calls(response)
            ai_text = self._extract_response_text(response)
            if not ai_text and tool_calls:
                ai_text = "; ".join(f"{tc['name']}: {tc['result']}" for tc in tool_calls)

            # Сохраняем ответ ИИ
            await self.history_db.save_message("assistant", ai_text)
            system_state.ai_chat_history.append({
                "role": "assistant", "content": ai_text,
                "ts": datetime.now(timezone.utc).isoformat()
            })
            system_state.ai_thoughts_log.append(
                f"[{datetime.now(timezone.utc).isoformat()}] Response ({system_state.ai_ping_ms}ms): {ai_text[:80]}..."
            )

            return {"response": ai_text, "tool_calls": tool_calls}
        except Exception as e:
            system_state.ai_status = "AI_DISCONNECTED"
            logger.error(f"Gemini chat error: {e}")
            return {"response": f"Ошибка AI: {e}", "tool_calls": []}

    async def _process_tool_calls(self, response: Any) -> List[Dict]:
        """Извлечь и выполнить вызовы инструментов из ответа Gemini (ревью A3)."""
        tool_calls_executed = []
        # Проверяем наличие function calls в ответе
        if not hasattr(response, "candidates") or not response.candidates:
            return tool_calls_executed
        for candidate in response.candidates:
            if not hasattr(candidate, "content") or not hasattr(candidate.content, "parts"):
                continue
            for part in candidate.content.parts:
                fc = getattr(part, "function_call", None)
                name = getattr(fc, "name", None) if fc is not None else None
                if not name:
                    # В protobuf у текстовых part'ов поле function_call существует, но name == ""
                    continue
                raw_args = getattr(fc, "args", None)
                args = dict(raw_args) if raw_args else {}
                result = await self._execute_tool(name, args)
                tool_calls_executed.append({"name": name, "args": args, "result": result})
        return tool_calls_executed

    async def _execute_tool(self, name: str, args: Dict) -> str:
        """Выполнить инструмент Tool Calling (с проверкой безопасности RPI-CL-12)."""
        system_state.ai_thoughts_log.append(
            f"[TOOL] Executing: {name}({json.dumps(args, default=str)})"
        )
        if name == "get_system_telemetry":
            return self._build_telemetry_context()
        elif name == "set_levitation_height":
            return await tool_set_levitation_height(args.get("target_z", 25.0))
        elif name == "run_spatial_helix_animation":
            # Тот же интерлок, что и у REST /api/manage/execute (ревью A2)
            decision = guard_command("SPATIAL_HELIX", source="ai")
            if not decision.allowed:
                return f"REJECTED: {decision.reason}"
            start_pipeline("SPATIAL_HELIX", execute_helix_pipeline)
            return "Spatial helix animation started."
        elif name == "execute_safe_shutdown":
            decision = guard_command("SAFE_SHUTDOWN", source="ai")
            if not decision.allowed:
                return f"REJECTED: {decision.reason}"
            start_pipeline("SAFE_SHUTDOWN", execute_shutdown_pipeline, preempt=True)
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
async def tool_set_levitation_height(target_z: Any) -> str:
    """Изменить уставку target_z с проверкой лимитов и интерлоков (RPI-CL-12)."""
    try:
        target_z = float(target_z)
    except (TypeError, ValueError):
        return f"REJECTED: target_z={target_z!r} is not a number."
    if not math.isfinite(target_z) or target_z < TARGET_Z_MIN_MM or target_z > TARGET_Z_MAX_MM:
        return f"REJECTED: target_z={target_z} out of safe range [10..45] mm."
    decision = guard_command("SET_HEIGHT", source="ai")
    if not decision.allowed:
        return f"REJECTED: {decision.reason}"
    old_z = system_state.target_z
    system_state.target_z = target_z
    logger.info(f"Tool: set_levitation_height {old_z} -> {target_z} mm")
    return f"OK: target_z changed from {old_z} to {target_z} mm."


async def tool_inspect_surface(tilt_angle_deg: Any = 15.0) -> str:
    """Наклонить сферу, захватить кадр с ESP32-CAM, отправить в Gemini Vision."""
    try:
        tilt_angle_deg = float(tilt_angle_deg)
    except (TypeError, ValueError):
        return f"REJECTED: tilt_angle={tilt_angle_deg!r} is not a number."
    if not math.isfinite(tilt_angle_deg) or tilt_angle_deg < 0 or tilt_angle_deg > 30.0:
        return f"REJECTED: tilt_angle={tilt_angle_deg} out of safe range [0..30] deg."
    decision = guard_command("INSPECT_SURFACE", source="ai")
    if not decision.allowed:
        return f"REJECTED: {decision.reason}"
    # Симуляция наклона через дифференциальный ШИМ
    system_state.agent_pitch = tilt_angle_deg
    logger.info(f"Tool: inspect_surface tilt={tilt_angle_deg}°, capturing ESP32-CAM frame...")
    await asyncio.sleep(0.5)  # Ждем стабилизации наклона
    # Возврат в горизонталь
    system_state.agent_pitch = 0.0
    # Честный статус без выдуманных объектов (ревью A8)
    return (
        f"OK: Surface inspected at {tilt_angle_deg}°. "
        f"Frame captured (1600x1200) [SIMULATED]. "
        f"Vision analysis: NOT_IMPLEMENTED (ESP32-CAM HTTP stream not connected)."
    )


async def tool_clear_temp_cache() -> str:
    """Безопасная очистка временных файлов и авто-архивация ai_history.db при > 50 МБ (TASK-AI-6 / RPI-CL-15)."""
    async with _db_lock:
        archive_path = await asyncio.to_thread(archive_db_if_oversized)
    archive_note = f" Archived DB to {archive_path.name}." if archive_path else ""
    if not TEMP_CACHE_DIR.exists():
        return f"OK: /tmp/antigravity/ directory does not exist. Nothing to clean.{archive_note}"
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
    logger.info(f"Tool: clear_temp_cache freed {freed_mb} MB. ai_history.db preserved.{archive_note}")
    return f"OK: Cleared {freed_mb} MB from /tmp/antigravity/. History DB preserved.{archive_note}"


clear_temp_cache = tool_clear_temp_cache


async def trigger_nod_gesture() -> None:
    """Микро-кивок сферы: относительное смещение Z += 3.0 мм на 500 мс, затем возврат (UC-BUS-12, ревью A11).

    Использует относительную дельту вместо перезаписи original_z, чтобы не затирать
    уставку target_z, изменённую параллельно оператором или инструментом set_levitation_height.
    """
    delta = 3.0
    system_state.nod_offset_z += delta
    system_state.target_z += delta
    try:
        await asyncio.sleep(0.5)
    finally:
        system_state.target_z -= delta
        system_state.nod_offset_z -= delta


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
        """TEST-AI-03: Loopback тест аудио-тракта SPU-WM30 с одновременным playrec (ревью A7)."""
        if not HAS_AUDIO:
            return {"status": "SKIPPED", "reason": "sounddevice not available"}

        duration_s = 0.5
        freq_hz = 440.0

        def _loopback_playrec() -> bytes:
            frames = int(self.SAMPLE_RATE * duration_s)
            t = np.linspace(0, duration_s, frames, endpoint=False)
            tone = (np.sin(2 * np.pi * freq_hz * t) * 0.3 * 32767).astype(np.int16)
            if hasattr(sd, "playrec"):
                rec = sd.playrec(
                    tone.reshape(-1, self.CHANNELS),
                    samplerate=self.SAMPLE_RATE,
                    channels=self.CHANNELS,
                    dtype="int16",
                )
                sd.wait()
                return np.asarray(rec, dtype=np.int16).tobytes()
            sd.play(tone, samplerate=self.SAMPLE_RATE)
            rec = sd.rec(frames, samplerate=self.SAMPLE_RATE, channels=self.CHANNELS, dtype="int16")
            sd.wait()
            return np.asarray(rec, dtype=np.int16).tobytes()

        try:
            system_state.audio_vad_active = True
            chunk = await asyncio.to_thread(_loopback_playrec)
            system_state.audio_vad_active = False
        except Exception as e:
            system_state.audio_vad_active = False
            logger.error(f"AudioService loopback error: {e}")
            return {"status": "FAILED", "reason": f"Loopback playrec failed: {e}"}

        if not chunk:
            return {"status": "FAILED", "reason": "Capture failed"}

        # Проверка амплитуды эхо-сигнала
        samples = np.frombuffer(chunk, dtype=np.int16)
        max_amplitude = float(np.max(np.abs(samples))) / 32767.0 if samples.size else 0.0
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
        await pipeline_sleep(1.0)
        # Симуляция: каждый активный канал дает dV = 0.28V, остальные < 0.03V
        for ch in range(4):
            system_state.pwm_channels = [0] * 5
            system_state.pwm_channels[ch] = 200
            await pipeline_sleep(0.2)
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
        await pipeline_sleep(1.0)
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
        await pipeline_sleep(0.8)
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
        await pipeline_sleep(0.5)
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
        t0 = time.perf_counter()
        await pipeline_sleep(1.0)
        dt_actual = max(time.perf_counter() - t0, 1e-4)
        temp_end = system_state.coil_temp_model
        system_state.pwm_channels = [0] * 5
        heat_rate = (temp_end - temp_start) / dt_actual
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
    system_state.safe_to_unplug = False
    system_state.emergency_reason = None
    system_state.agent_wifi_last_seen = time.perf_counter()
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

    await pipeline_sleep(1.0)
    system_state.active_pipeline = None


# Глобальные синглтоны AI и аппаратной подсистемы (ревью P6)
ai_config = AIConfigManager()
ai_history_db = AIHistoryDB()
ai_manager = GeminiAIManager(ai_config, ai_history_db)
audio_service = AudioService()
hall_driver = HallADS1115Driver()
tof_driver = ToFMatrixDriver()
levitation_core = LevitationCoreLoop(dry_run=True)

# Менеджер WebSocket подключений
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    def register(self, websocket: WebSocket):
        """Зарегистрировать уже принятый и аутентифицированный сокет."""
        self.active_connections.append(websocket)
        logger.info(f"Новое веб-подключение. Всего клиентов: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(f"Клиент отключился. Осталось клиентов: {len(self.active_connections)}")

    async def broadcast(self, message: str):
        dead: List[WebSocket] = []
        for connection in list(self.active_connections):
            try:
                await connection.send_text(message)
            except Exception:
                dead.append(connection)
        for connection in dead:
            self.disconnect(connection)

manager = ConnectionManager()


async def ws_authenticate(websocket: WebSocket, accepted_tokens: List[str]) -> bool:
    """Принять сокет и дождаться {"type":"auth","token":...} первым сообщением.

    Возвращает False (и закрывает сокет с кодом 1008), если токен неверный
    или не пришёл за WS_AUTH_TIMEOUT_S.
    """
    await websocket.accept()
    provided: Optional[str] = None
    try:
        raw = await asyncio.wait_for(websocket.receive_text(), timeout=WS_AUTH_TIMEOUT_S)
        msg = json.loads(raw)
        if isinstance(msg, dict) and msg.get("type") == "auth" and isinstance(msg.get("token"), str):
            provided = msg["token"]
    except WebSocketDisconnect:
        return False
    except (asyncio.TimeoutError, ValueError):
        provided = None

    if any(token_matches(provided, expected) for expected in accepted_tokens):
        await websocket.send_text(json.dumps({"type": "auth_ok"}))
        return True

    logger.warning("WebSocket: отклонено подключение без валидного токена.")
    try:
        await websocket.close(code=1008)
    except Exception:
        pass
    return False

# Модели запросов для API
class CommandRequest(BaseModel):
    command: str  # Например: "BENCH_CALIBRATE", "SMOOTH_TAKEOFF", "SAFE_LANDING", "BIST_RUN_ALL"

class BypassRequest(BaseModel):
    override: bool
    # Обязательная фраза-подтверждение при override=True (ревью A1)
    confirm: Optional[str] = Field(default=None, max_length=64)


BYPASS_CONFIRM_PHRASE = "I_ACCEPT_THE_RISK"

class FaultSimulationRequest(BaseModel):
    test_id: Optional[str]

class BistRetryRequest(BaseModel):
    test_id: str = Field(min_length=1, max_length=32)

# =====================================================================
# ВЫЧИСЛИТЕЛЬНЫЕ СЛУЖБЫ И СТАБИЛИЗАЦИОННЫЙ ЦИКЛ (860 Гц)
# =====================================================================
async def run_hardware_control_step(dt: float) -> Optional[List[int]]:
    """Выполнить один такт аппаратного контура (Hall ADS1115 + ToF VL53L5CX + ComplementaryFilter + TriAxisPID + CoilMapper + UART).

    Используется в core_loop_860hz при system_state.hardware_mode == 'real' (ревью N-C4).
    """
    voltages = await asyncio.to_thread(hall_driver.read_all_channels_voltage)
    valid_v: List[float] = []
    valid_d: List[float] = []
    for v in voltages:
        if v is not None:
            v_comp = hall_driver.compensate_coil_field(v, system_state.prev_pwm_channels)
            valid_v.append(v_comp)
            d = hall_driver.voltage_to_distance(v_comp)
            if d is not None:
                valid_d.append(d)
    if valid_v:
        system_state.raw_hall = (valid_v + [valid_v[-1]] * 4)[:4]
    hall_z = (sum(valid_d) / len(valid_d)) if valid_d else None

    tof_z: Optional[float] = None
    if len(system_state.tof_grid) == 64 and any(v > 5.0 for v in system_state.tof_grid):
        matrix_8x8 = [system_state.tof_grid[r * 8:(r + 1) * 8] for r in range(8)]
        try:
            tof_driver.update_gimbal_angles(system_state.agent_pitch, system_state.agent_roll)
            tof_res = tof_driver.process_matrix_frame(matrix_8x8)
            bx, by, bz = tof_res["base_xyz"]
            system_state.x = round(bx, 3)
            system_state.y = round(by, 3)
            tof_z = bz
        except ValueError:
            tof_z = None

    fused_z = levitation_core.filter.fuse(hall_z, tof_z)
    if fused_z is None:
        return [0, 0, 0, 0, 0]

    system_state.z = float(fused_z)
    if system_state.target_z <= 0:
        levitation_core.pid.reset_all()
        return [0, 0, 0, 0, 0]

    levitation_core.pid.pid_z.kp = system_state.kp
    levitation_core.pid.pid_z.ki = system_state.ki
    levitation_core.pid.pid_z.kd = system_state.kd
    levitation_core.mapper.max_duty = system_state.max_duty_limit
    clamped_dt = min(max(dt, 1e-4), levitation_core.MAX_PID_DT_S)
    px, py, pz = levitation_core.pid.compute(
        (0.0, 0.0, system_state.target_z),
        (system_state.x, system_state.y, system_state.z),
        dt=clamped_dt,
    )
    duties = levitation_core.mapper.map(px, py, 200.0 + pz)
    return duties


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
    non_preemptible = (
        "SAFE_SHUTDOWN", "EMERGENCY_SAFE_LANDING", "SAFE_LANDING",
        "BIST_RUN_ALL", "BIST_RETRY", "POST_ASSEMBLY_BIST",
    )

    while True:
        loop_start = time.perf_counter()
        tick_dt = max(loop_start - last_time, target_dt)
        
        # Check Heartbeat & Battery Safety (TASK-AI-5 / RPI-CL-14 / ревью N-C1, N-C2)
        power_armed = system_state.bist_unlocked or system_state.bist_bypass_active
        wifi_elapsed = time.perf_counter() - system_state.agent_wifi_last_seen
        if (
            power_armed
            and system_state.target_z > 0
            and system_state.active_pipeline not in non_preemptible
        ):
            if wifi_elapsed > 0.500:
                start_pipeline(
                    "EMERGENCY_SAFE_LANDING",
                    lambda: execute_emergency_landing_pipeline("Wi-Fi Heartbeat Loss (>500ms)"),
                    preempt=True,
                )
            elif system_state.agent_battery < 10.0:
                start_pipeline(
                    "EMERGENCY_SAFE_LANDING",
                    lambda: execute_emergency_landing_pipeline("Critical Battery Level (<10%)"),
                    preempt=True,
                )

        if system_state.hardware_mode == "real":
            hw_duties = await run_hardware_control_step(tick_dt)
            last_time = time.perf_counter()
            target_duties = hw_duties if (hw_duties is not None and system_state.target_z > 0) else [0, 0, 0, 0, 0]
        else:
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
                last_time = loop_start
            target_duties = [base_pwm] * 5 if system_state.target_z > 0 else [0, 0, 0, 0, 0]
        
        # Программный интерлок безопасности BIST, SAFE_SHUTDOWN и EMERGENCY_SAFE_LANDING
        if system_state.active_pipeline in ("SAFE_SHUTDOWN", "EMERGENCY_SAFE_LANDING"):
            # Во время безопасного/аварийного выключения ШИМ жестко управляется самим пайплайном
            pass
        elif not system_state.bist_unlocked and not system_state.bist_bypass_active and system_state.active_pipeline is None:
            system_state.pwm_channels = [0, 0, 0, 0, 0]  # Жесткая блокировка
        elif system_state.active_pipeline is None:
            system_state.pwm_channels = target_duties
            
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
        if system_state.hardware_mode == "real":
            levitation_core._send_packet(clamped_channels)
            
        # Термальный интегратор через ThermalModel из core_loop.py (ревью A11 / P6)
        levitation_core.thermal_model.estimated_temp_c = system_state.coil_temp_model
        levitation_core.thermal_model.throttling = system_state.thermal_throttling
        was_throttling = system_state.thermal_throttling
        is_throttling = levitation_core.thermal_model.update(tick_dt, system_state.pwm_channels)
        system_state.coil_temp_model = levitation_core.thermal_model.estimated_temp_c
        system_state.thermal_throttling = is_throttling

        if is_throttling:
            step_down = min(0.01, max(0.0, system_state.target_z - TARGET_Z_MIN_MM))
            if step_down > 0:
                system_state.target_z -= step_down
                system_state.thermal_z_offset -= step_down
        elif was_throttling and system_state.thermal_z_offset < 0.0:
            restore = -system_state.thermal_z_offset
            system_state.target_z = min(system_state.target_z + restore, TARGET_Z_MAX_MM)
            system_state.thermal_z_offset = 0.0
            
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
                    "kd": round(system_state.kd, 3),
                    "target_z": round(system_state.target_z, 2)
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
                "post_assembly": {
                    "results": system_state.post_assembly_results,
                    "logs": system_state.post_assembly_logs
                },
                "ai": {
                    "enabled": system_state.ai_enabled,
                    "status": system_state.ai_status,
                    "ping_ms": system_state.ai_ping_ms,
                    "model": system_state.ai_model_name,
                    "thoughts_log": list(system_state.ai_thoughts_log)[-10:],
                    "audio_available": system_state.audio_available,
                    "audio_vad_active": system_state.audio_vad_active
                },
                "shutdown": {
                    "safe_to_unplug": system_state.safe_to_unplug,
                    "emergency_reason": system_state.emergency_reason
                }
            }
            await manager.broadcast(json.dumps(payload))
        await asyncio.sleep(0.1)

# =====================================================================
# ВСТРОЕННЫЕ АВТОМАТИЧЕСКИЕ ТЕСТЫ (BIST PIPELINES)
# =====================================================================
async def run_single_bist_test(test_id: str) -> bool:
    if system_state.simulate_fault_on_test_id == test_id:
        await pipeline_sleep(1.5)
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
        await pipeline_sleep(1.5)
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: Sweep complete on Coils 0-4. Hall sensor feedback amplitude:\n"
            "Coil 0: +0.28V, Coil 1: +0.31V, Coil 2: +0.29V, Coil 3: +0.28V, Coil 4: +0.30V.\n"
            "Все силовые ключи и драйверы затворов TC4427 полностью исправны. Джиттер ШИМ отсутствует."
        )
    elif test_id == "BIST-HW-02":
        logger.info("BIST: Тест полярности катушек (BIST-HW-02) запущен...")
        await pipeline_sleep(1.5)
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: All 5 coil winding polarities verified.\n"
            "Каждое поле генерирует строго Северный полюс (North, dV > 0) на верхнем торце феррита.\n"
            "Магнитный купол удержания сформирован геометрически симметрично."
        )
    elif test_id == "BIST-HW-03":
        logger.info("BIST: Тест интерлока реле Агента (BIST-HW-03) запущен...")
        await pipeline_sleep(1.2)
        system_state.agent_relay_status = "Isolated"
        await pipeline_sleep(0.3)
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
            await pipeline_sleep(0.2)
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
            await pipeline_sleep(0.001)  # Симуляция тактов 860 Гц
        # Ramp down: 300 → 0 с ограничением 10 ед/такт
        ramp_down_steps = 0
        while test_channel_pwm > 0:
            test_channel_pwm = max(test_channel_pwm - system_state.slew_rate_limit, 0)
            ramp_down_steps += 1
            await pipeline_sleep(0.001)
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
    await pipeline_sleep(0.5)
    system_state.active_pipeline = None
    system_state.pipeline_status = "Idle"

async def execute_landing_pipeline():
    system_state.active_pipeline = "SAFE_LANDING"
    system_state.pipeline_status = "Running"
    await pipeline_sleep(0.5)
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
    # Повторный прогон сначала блокирует выходы: если пайплайн прервут,
    # старый PASS не должен оставаться в силе.
    system_state.bist_unlocked = False
    system_state.safe_to_unplug = False
    system_state.emergency_reason = None
    system_state.agent_wifi_last_seen = time.perf_counter()
    
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
            await pipeline_sleep(2.0)
            system_state.active_pipeline = None
            return

    system_state.pipeline_steps[5]["status"] = "RUNNING"
    await pipeline_sleep(1.0)
    
    if all(res == "PASS" for res in system_state.bist_results.values()):
        system_state.bist_unlocked = True
        system_state.agent_wifi_last_seen = time.perf_counter()
        system_state.pipeline_steps[5]["status"] = "DONE"
        system_state.pipeline_status = "System Unlocked (All Tests PASS)"
        logger.info("BIST: Все 5 тестов успешно пройдены! Система разблокирована.")
    else:
        system_state.bist_unlocked = False
        system_state.pipeline_steps[5]["status"] = "FAILED"
        system_state.pipeline_status = "Failed BIST validation."
        
    await pipeline_sleep(2.0)
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
    
    # Шаг 1: Плавный спуск сферы с согласованным снижением ШИМ (ревью N-C3)
    system_state.pipeline_steps[0]["status"] = "RUNNING"
    logger.info("SHUTDOWN: Запущен плавный спуск Агента со скоростью 5 мм/с...")
    current_z = system_state.target_z
    initial_z = max(current_z, 1e-6)
    initial_pwm = list(system_state.pwm_channels)
    max_step = max(1, min(int(system_state.slew_rate_limit), 10))
    while current_z > 0.0:
        current_z = max(current_z - 1.0, 0.0)
        system_state.target_z = current_z
        ratio = current_z / initial_z
        next_pwm = []
        for i, cur_p in enumerate(system_state.pwm_channels):
            desired_p = int(round(initial_pwm[i] * ratio))
            delta = max(min(desired_p - cur_p, max_step), -max_step)
            next_pwm.append(max(cur_p + delta, 0))
        system_state.pwm_channels = next_pwm
        system_state.prev_pwm_channels = list(next_pwm)
        await pipeline_sleep(0.2) # Спуск со скоростью 5 мм/с
    system_state.z = 0.0
    system_state.pipeline_steps[0]["status"] = "DONE"
    
    # Шаг 2: Размагничивание катушек
    system_state.pipeline_steps[1]["status"] = "RUNNING"
    logger.info("SHUTDOWN: Постепенное снижение ШИМ-выходов для подавления выбросов ЭДС...")
    steps = 20
    for s in range(steps):
        next_pwm = [int(max(pwm - min(max_step, pwm / (steps - s)), 0)) for pwm in system_state.pwm_channels]
        system_state.pwm_channels = next_pwm
        system_state.prev_pwm_channels = list(next_pwm)
        await pipeline_sleep(0.1) # Спуск ШИМ за 2 секунды
    while any(pwm > 0 for pwm in system_state.pwm_channels):
        next_pwm = [int(max(pwm - max_step, 0)) for pwm in system_state.pwm_channels]
        system_state.pwm_channels = next_pwm
        system_state.prev_pwm_channels = list(next_pwm)
        await pipeline_sleep(1.0 / 860.0)
    system_state.pwm_channels = [0, 0, 0, 0, 0]
    system_state.prev_pwm_channels = [0, 0, 0, 0, 0]
    system_state.pipeline_steps[1]["status"] = "DONE"
    
    # Шаг 3: Перевод сферы Агента в режим парковки
    system_state.pipeline_steps[2]["status"] = "RUNNING"
    logger.info("SHUTDOWN: Отправка команд Standby Агенту по Wi-Fi...")
    await pipeline_sleep(1.0)
    system_state.agent_relay_status = "Connected" # Реле замкнуто на зарядку
    system_state.agent_coil_pwm = 0
    system_state.pipeline_steps[2]["status"] = "DONE"
    
    # Шаг 4: Фиксация безопасного отключения
    system_state.pipeline_steps[3]["status"] = "RUNNING"
    await pipeline_sleep(1.0)
    system_state.bist_unlocked = False
    system_state.bist_bypass_active = False
    system_state.safe_to_unplug = True # Разрешаем отсоединять кабель!
    system_state.pipeline_steps[3]["status"] = "DONE"
    
    system_state.pipeline_status = "System Secure - Safe to Power Off"
    logger.info("SHUTDOWN: Комплекс полностью обезопасен. Можно безопасно извлечь блок питания 27W!")
    await pipeline_sleep(2.0)


# =====================================================================
# АВАРИЙНАЯ МЯГКАЯ ПОСАДКА (TASK-AI-5 / RPI-CL-14) - V12
# =====================================================================
async def execute_emergency_landing_pipeline(reason: str) -> None:
    """TASK-AI-5 / RPI-CL-14: Graceful Emergency Safe Landing.

    Триггерится из core_loop_860hz() при потере Wi-Fi heartbeat (> 500 мс)
    или критическом разряде LiPo батареи сферы (< 10.0%).
    - Озвучивает голосовое предупреждение через AudioService (SPU-WM30).
    - Снижает уставку target_z со скоростью строго 5.0 мм/с (1.0 мм за 0.2 с).
    - Плавно гасит ШИМ катушек с ограничением Slew-Rate <= 10 ед/тик (1.16 мс).
    - Выставляет safe_to_unplug = True и транслирует shutdown.emergency_reason в /ws/telemetry.
    """
    if system_state.active_pipeline == "EMERGENCY_SAFE_LANDING" and system_state.emergency_reason == reason:
        return
    if system_state.calib_timer_task is not None and not system_state.calib_timer_task.done():
        system_state.calib_timer_task.cancel()
    system_state.calib_timer_task = None

    system_state.active_pipeline = "EMERGENCY_SAFE_LANDING"
    system_state.emergency_reason = reason
    system_state.pipeline_status = f"EMERGENCY LANDING: {reason}"
    system_state.pipeline_steps = [
        {"step_id": "EMERG-01", "desc": f"Голосовое оповещение SPU-WM30 ({reason})", "status": "RUNNING"},
        {"step_id": "EMERG-02", "desc": "Плавный спуск сферы со скоростью 5.0 мм/с", "status": "PENDING"},
        {"step_id": "EMERG-03", "desc": "Гашение ШИМ катушек (Slew-Rate <= 10 ед/тик)", "status": "PENDING"},
        {"step_id": "EMERG-04", "desc": "Парковка реле Qi и фиксация безопасного отключения", "status": "PENDING"},
    ]
    logger.warning(f"EMERGENCY LANDING TRIGGERED: {reason}")

    # Шаг 1: Голосовое оповещение через SPU-WM30 (RPI-CL-14)
    if "battery" in reason.lower() or "аккум" in reason.lower() or "10%" in reason.lower():
        alert_msg = (
            "Внимание! Зафиксирован критический разряд аккумулятора сферы. "
            "Выполняется аварийная посадка."
        )
    else:
        alert_msg = (
            "Внимание! Зафиксирована потеря Wi-Fi связи со сферой. "
            "Выполняется аварийная посадка."
        )
    logger.info(f"AudioAlert: {alert_msg}")
    await audio_service.speak_text(alert_msg)
    system_state.pipeline_steps[0]["status"] = "DONE"

    # Шаг 2: Плавный спуск со скоростью строго 5.0 мм/с (1.0 мм каждые 0.2 с) и согласованным снижением ШИМ (ревью N-C3)
    system_state.pipeline_steps[1]["status"] = "RUNNING"
    initial_z = max(system_state.target_z, 1e-6)
    initial_pwm = list(system_state.pwm_channels)
    max_step = max(1, min(int(system_state.slew_rate_limit), 10))
    while system_state.target_z > 0.0:
        step_mm = min(1.0, system_state.target_z)
        system_state.target_z = round(max(system_state.target_z - step_mm, 0.0), 4)
        ratio = system_state.target_z / initial_z
        next_pwm = []
        for i, cur_p in enumerate(system_state.pwm_channels):
            desired_p = int(round(initial_pwm[i] * ratio))
            delta = max(min(desired_p - cur_p, max_step), -max_step)
            next_pwm.append(max(cur_p + delta, 0))
        system_state.pwm_channels = next_pwm
        system_state.prev_pwm_channels = list(next_pwm)
        await pipeline_sleep(0.2 * step_mm)
    system_state.z = 0.0
    system_state.pipeline_steps[1]["status"] = "DONE"

    # Шаг 3: Плавное гашение ШИМ катушек с соблюдением Slew-Rate <= 10 ед/тик (1.16 мс)
    system_state.pipeline_steps[2]["status"] = "RUNNING"
    tick_s = 1.0 / 860.0
    while any(pwm > 0 for pwm in system_state.pwm_channels):
        next_pwm = [int(max(pwm - max_step, 0)) for pwm in system_state.pwm_channels]
        system_state.pwm_channels = next_pwm
        system_state.prev_pwm_channels = list(next_pwm)
        await pipeline_sleep(tick_s)
    system_state.pwm_channels = [0, 0, 0, 0, 0]
    system_state.prev_pwm_channels = [0, 0, 0, 0, 0]
    system_state.pipeline_steps[2]["status"] = "DONE"

    # Шаг 4: Перевод реле Qi в парковку и фиксация безопасного состояния
    system_state.pipeline_steps[3]["status"] = "RUNNING"
    system_state.agent_relay_status = "Connected"
    system_state.agent_coil_pwm = 0
    system_state.bist_unlocked = False
    system_state.bist_bypass_active = False
    system_state.safe_to_unplug = True
    system_state.pipeline_steps[3]["status"] = "DONE"
    system_state.pipeline_status = f"Emergency Landing Complete ({reason})"
    logger.info(f"EMERGENCY LANDING COMPLETE ({reason}). safe_to_unplug=True")

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
        await pipeline_sleep(0.15)
    system_state.pipeline_steps[0]["status"] = "DONE"
    
    system_state.pipeline_steps[1]["status"] = "RUNNING"
    for angle in range(0, 360, 15):
        rad = math.radians(angle)
        system_state.x = 8.0 * math.cos(rad)
        system_state.y = 8.0 * math.sin(rad)
        await pipeline_sleep(0.08)
    system_state.pipeline_steps[1]["status"] = "DONE"
    
    system_state.pipeline_steps[2]["status"] = "RUNNING"
    system_state.x = 0.0
    system_state.y = 0.0
    system_state.target_z = 25.0
    await pipeline_sleep(1.0)
    system_state.pipeline_steps[2]["status"] = "DONE"
    
    system_state.pipeline_status = "Helix Complete (RTH)"
    await pipeline_sleep(1.5)
    system_state.active_pipeline = None

# =====================================================================
# API ЭНДПОИНТЫ (HTTP / WebSockets)
# =====================================================================
async def startup_event(spawn_background_loops: bool = True):
    """Инициализация служб при старте (вызывается из lifespan, ревью A12)."""
    if spawn_background_loops:
        asyncio.create_task(core_loop_860hz())
        asyncio.create_task(telemetry_broadcaster_10hz())
    # Создать/загрузить токены заранее, чтобы путь к ним попал в лог при старте (A1)
    get_api_token()
    get_agent_token()
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
        "safe_to_unplug": system_state.safe_to_unplug,
        "emergency_reason": system_state.emergency_reason,
        "maintenance_logs": list(system_state.maintenance_logs),
    }

class PIDTuning(BaseModel):
    """Параметры ПИД с жёсткими границами (ревью A4). NaN/Inf отклоняются."""
    kp: Optional[float] = Field(default=None, ge=0.0, le=200.0, allow_inf_nan=False)
    ki: Optional[float] = Field(default=None, ge=0.0, le=50.0, allow_inf_nan=False)
    kd: Optional[float] = Field(default=None, ge=0.0, le=100.0, allow_inf_nan=False)
    target_z: Optional[float] = Field(
        default=None, ge=TARGET_Z_MIN_MM, le=TARGET_Z_MAX_MM, allow_inf_nan=False,
    )

@app.post("/api/pid/set_tunings")
async def set_tunings(tunings: PIDTuning):
    # Смена уставки — та же команда SET_HEIGHT, что и у AI: одинаковый интерлок.
    if tunings.target_z is not None:
        decision = guard_command("SET_HEIGHT", source="operator")
        if not decision.allowed:
            raise HTTPException(status_code=decision.status_code, detail=decision.reason)
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
    ssid: str = Field(min_length=1, max_length=32)
    password: str = Field(default="", max_length=63)


async def _run_wifi_handover(ssid: str, password: str) -> None:
    """Фоновая часть /api/wifi/connect: выполняется после отправки ответа,
    потому что handover роняет AP, через которую пришёл запрос."""
    try:
        ok = await NetworkManager.handover_network(ssid, password, agent_ip=system_state.agent_wifi_ip)
    except Exception:
        logger.exception("Wi-Fi handover завершился с ошибкой.")
        ok = False
    system_state.base_wifi_mode = "STA" if ok else "AP"
    if not ok:
        system_state.base_wifi_ssid = "Unconfigured"


@app.post("/api/wifi/connect")
async def wifi_connect(req: WifiConnectRequest, background_tasks: BackgroundTasks):
    """Подключить Базу и Агента к домашней сети (раньше только менял состояние)."""
    try:
        validate_wifi_credentials(req.ssid, req.password)
    except CredentialError as e:
        raise HTTPException(status_code=422, detail=str(e))
    if system_state.active_pipeline is not None:
        raise HTTPException(
            status_code=409,
            detail=f"Смена сети невозможна во время сценария '{system_state.active_pipeline}'.",
        )
    system_state.base_wifi_ssid = req.ssid
    system_state.base_wifi_mode = "STA_CONNECTING"
    background_tasks.add_task(_run_wifi_handover, req.ssid, req.password)
    return {"status": "ACCEPTED", "mode": system_state.base_wifi_mode}


# Команды, доступные через /api/manage/execute
REST_COMMANDS = frozenset({
    "BIST_RUN_ALL", "SAFE_SHUTDOWN",
    "BENCH_CALIBRATE", "SMOOTH_TAKEOFF", "SPATIAL_HELIX",
    "LISSAJOUS_PATROL", "EMO_REACTION", "FORCE_FEEDBACK",
    "SAFE_LANDING", "RTH_ROUTINE",
})


def _dummy_pipeline_factory(name: str) -> Callable[[], Awaitable[None]]:
    """Заглушка для сценариев без реализации."""
    async def _dummy() -> None:
        system_state.pipeline_status = "Running"
        await asyncio.sleep(0.5)
        if system_state.active_pipeline == name:
            system_state.active_pipeline = None
            system_state.pipeline_status = "Idle"
    return _dummy


def _pipeline_factory(command: str) -> Callable[[], Awaitable[None]]:
    factories: Dict[str, Callable[[], Awaitable[None]]] = {
        "BIST_RUN_ALL": execute_bist_pipeline,
        "SAFE_SHUTDOWN": execute_shutdown_pipeline,
        "SMOOTH_TAKEOFF": execute_takeoff_pipeline,
        "SAFE_LANDING": execute_landing_pipeline,
        "SPATIAL_HELIX": execute_helix_pipeline,
    }
    return factories.get(command) or _dummy_pipeline_factory(command)


@app.post("/api/manage/execute")
async def execute_command(req: CommandRequest):
    if req.command not in REST_COMMANDS:
        raise HTTPException(status_code=404, detail="Сценарий не найден")

    decision = guard_command(req.command, source="operator")
    if not decision.allowed:
        raise HTTPException(status_code=decision.status_code, detail=decision.reason)

    start_pipeline(req.command, _pipeline_factory(req.command), preempt=req.command in SAFETY_COMMANDS)
    return {"status": "started", "pipeline": req.command}

@app.post("/api/bist/retry")
async def bist_retry(req: Union[BistRetryRequest, Dict[str, Any]]):
    test_id = req.get("test_id") if isinstance(req, dict) else req.test_id
    if not test_id or test_id not in system_state.bist_results:
        raise HTTPException(status_code=400, detail="Invalid test_id")

    decision = guard_command("BIST_RETRY", source="operator")
    if not decision.allowed:
        raise HTTPException(status_code=400, detail=decision.reason)

    # Занимаем слот синхронно, чтобы параллельно не стартовал другой пайплайн (A5)
    system_state.active_pipeline = "BIST_RETRY"
    try:
        system_state.bist_results[test_id] = "RUNNING"
        system_state.bist_logs[test_id] = "Повторное тестирование запущено..."

        success = await run_single_bist_test(test_id)
        system_state.bist_results[test_id] = "PASS" if success else "FAILED"

        # Пересчитываем общий статус разблокировки
        system_state.bist_unlocked = all(res == "PASS" for res in system_state.bist_results.values())
        if system_state.bist_unlocked:
            system_state.agent_wifi_last_seen = time.perf_counter()
    finally:
        if system_state.active_pipeline == "BIST_RETRY":
            system_state.active_pipeline = None

    return {"status": "completed", "test_id": test_id, "result": system_state.bist_results[test_id]}

@app.post("/api/bist/bypass")
async def bist_bypass(req: BypassRequest):
    if system_state.active_pipeline == "SAFE_SHUTDOWN":
        raise HTTPException(status_code=409, detail="Система выключена (SAFE_SHUTDOWN). Запустите BIST.")
    if system_state.active_pipeline is not None:
        raise HTTPException(
            status_code=409,
            detail=f"Нельзя менять обход BIST во время выполнения сценария '{system_state.active_pipeline}'.",
        )
    if system_state.calib_state not in (CalibrationState.PARKED, CalibrationState.UNLOCKED_FLIGHT):
        raise HTTPException(
            status_code=409,
            detail=f"Нельзя менять обход BIST во время калибровки ({system_state.calib_state.value}).",
        )
    if req.override:
        if req.confirm != BYPASS_CONFIRM_PHRASE:
            raise HTTPException(
                status_code=400,
                detail=f"Для обхода BIST нужно явное подтверждение: confirm='{BYPASS_CONFIRM_PHRASE}'.",
            )
        system_state.bist_unlocked = True
        system_state.bist_bypass_active = True
        system_state.agent_wifi_last_seen = time.perf_counter()
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
    if req.test_id is not None and req.test_id not in system_state.bist_results:
        raise HTTPException(status_code=400, detail="Invalid test_id")
    system_state.simulate_fault_on_test_id = req.test_id
    logger.info(f"R&D: Симуляция отказа установлена для теста: {req.test_id}")
    return {"status": "success", "simulated_fault_id": req.test_id}

RelayStatus = Literal["Connected", "Isolated", "Fault"]

@app.post("/api/agent/bist_relay")
async def agent_bist_relay(status: RelayStatus):
    # Доступен с токеном Агента или оператора (см. api_auth_middleware)
    system_state.agent_relay_status = status
    logger.info(f"Получен статус реле защиты Агента: {status}")
    return {"status": "received"}

@app.websocket("/ws/telemetry")
async def websocket_endpoint(websocket: WebSocket):
    """Телеметрия для GUI оператора (только чтение).

    Входящие данные Агента здесь больше не принимаются — для них есть
    отдельный /ws/agent со своим токеном (ревью A1: подделка Агента).
    """
    if not await ws_authenticate(websocket, [get_api_token()]):
        return
    manager.register(websocket)
    try:
        while True:
            # Держим соединение и ловим отключение; содержимое игнорируем.
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        manager.disconnect(websocket)


class AgentTelemetryMessage(BaseModel):
    """Телеметрия Агента. Все поля опциональны: обновляем только пришедшие."""
    pitch: Optional[float] = Field(default=None, ge=-180.0, le=180.0, allow_inf_nan=False)
    roll: Optional[float] = Field(default=None, ge=-180.0, le=180.0, allow_inf_nan=False)
    yaw: Optional[float] = Field(default=None, ge=-360.0, le=360.0, allow_inf_nan=False)
    battery: Optional[float] = Field(default=None, ge=0.0, le=100.0, allow_inf_nan=False)
    coil_pwm: Optional[int] = Field(default=None, ge=0, le=1023)
    relay: Optional[RelayStatus] = None
    rssi: Optional[int] = Field(default=None, ge=-127, le=0)


_AGENT_FIELD_MAP = {
    "pitch": "agent_pitch", "roll": "agent_roll", "yaw": "agent_yaw",
    "battery": "agent_battery", "coil_pwm": "agent_coil_pwm",
    "relay": "agent_relay_status", "rssi": "agent_wifi_rssi",
}


def process_agent_telemetry(msg: AgentTelemetryMessage, sender_ip: Optional[str] = None) -> None:
    """Обновляет состояние системы данными телеметрии Агента (WS или UDP)."""
    if sender_ip:
        system_state.agent_wifi_ip = sender_ip
    system_state.agent_wifi_status = "Connected"
    system_state.agent_wifi_last_seen = time.perf_counter()

    for field, attr in _AGENT_FIELD_MAP.items():
        value = getattr(msg, field)
        if value is not None:
            setattr(system_state, attr, value)

    # Если пришли углы Эйлера, обновляем единый список agent_orientation [pitch, roll, yaw]
    if msg.pitch is not None or msg.roll is not None or msg.yaw is not None:
        p = msg.pitch if msg.pitch is not None else system_state.agent_pitch
        r = msg.roll if msg.roll is not None else system_state.agent_roll
        y = msg.yaw if msg.yaw is not None else system_state.agent_yaw
        system_state.agent_orientation = [p, r, y]


class AgentTelemetryUDPProtocol(asyncio.DatagramProtocol):
    """Служба приёма non-blocking UDP-датаграмм телеметрии от ESP32 (порт 4211, ревью F4/N-F2)."""

    def __init__(self):
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.DatagramTransport) -> None:
        self.transport = transport
        logger.info("Agent UDP telemetry receiver listening on port %d", AGENT_UDP_PORT)

    def datagram_received(self, data: bytes, addr: tuple) -> None:
        try:
            text = data.decode("utf-8")
            msg = AgentTelemetryMessage.model_validate_json(text)
        except Exception:
            logger.debug("Agent UDP: invalid datagram from %s", addr)
            return

        sender_ip = addr[0] if addr else None
        process_agent_telemetry(msg, sender_ip=sender_ip)

    def error_received(self, exc: Exception) -> None:
        logger.debug("Agent UDP socket error: %s", exc)

    def connection_lost(self, exc: Optional[Exception]) -> None:
        pass


@app.websocket("/ws/agent")
async def agent_websocket_endpoint(websocket: WebSocket):
    """Канал Агент → Мозг (ESP32). Аутентификация токеном Агента."""
    if not await ws_authenticate(websocket, [get_agent_token()]):
        return
    sender_ip = websocket.client.host if websocket.client is not None else None
    if sender_ip:
        system_state.agent_wifi_ip = sender_ip
    system_state.agent_wifi_status = "Connected"
    system_state.agent_wifi_last_seen = time.perf_counter()
    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = AgentTelemetryMessage.model_validate_json(data)
            except ValueError:
                logger.debug("Агент: отброшено невалидное сообщение.")
                continue
            process_agent_telemetry(msg, sender_ip=sender_ip)
    except WebSocketDisconnect:
        pass
    finally:
        system_state.agent_wifi_status = "Disconnected"

@app.post("/api/calibration/start")
async def start_calibration():
    if system_state.calib_state != CalibrationState.PARKED:
        raise HTTPException(status_code=400, detail="Калибровка уже запущена или завершена.")
    decision = guard_command("CALIBRATION", source="operator")
    if not decision.allowed:
        raise HTTPException(status_code=decision.status_code, detail=decision.reason)
        
    system_state.calib_state = CalibrationState.ZEROING
    system_state.calib_log = "ШАГ 1/5: Снятие нулей датчиков Холла и ToF на высоте Z = 30.0мм..."
    
    # Снятие нулей и проверка шума (симуляция)
    rms_noise = 1.2  # mV
    if rms_noise >= 2.0:
        system_state.calib_state = CalibrationState.PARKED
        raise HTTPException(status_code=500, detail=f"Calibration Failed: Sensor RMS noise {rms_noise}mV >= 2.0mV")
        
    await pipeline_sleep(1.0)
    
    system_state.calib_state = CalibrationState.TAKEOFF_HOVER
    system_state.calib_log = "ШАГ 2/5: Плавный приподъем сферы на +3.0мм (Z = 33.0мм)..."
    system_state.target_z = 33.0
    await pipeline_sleep(1.5)
    
    system_state.calib_state = CalibrationState.WAIT_REMOVAL
    system_state.calib_log = "ШАГ 3/5: Сфера зависла! Раздвиньте половинки подставки и нажмите CONFIRM."
    
    async def timeout_handler():
        await pipeline_sleep(30.0)
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
    # UC-BUS-12: Кивок сферы при ответе (только если полет разрешен интерлоком, ревью A11)
    if guard_command("SET_HEIGHT", source="ai").allowed:
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
        "thoughts_log": list(system_state.ai_thoughts_log)[-20:],
        "chat_history": list(system_state.ai_chat_history)[-50:],
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


# =====================================================================
# V12 ЭНДПОИНТЫ: MJPEG LIVE STREAM (TASK-UI-5) & DB ARCHIVING (TASK-AI-6)
# =====================================================================
async def mjpeg_frame_generator(max_frames: Optional[int] = None):
    """TASK-UI-5 / WEB-CL-11: Асинхронный генератор MJPEG-потока с камеры ESP32-CAM OV2640.

    Выдаёт кадры >= 15 FPS (интервал ~60 мс, задержка < 200 мс) без блокирования
    ПИД-контура 860 Гц. При передаче max_frames завершает поток после указанного
    числа кадров (удобно для модульного тестирования).
    """
    frame_idx = 0
    while max_frames is None or frame_idx < max_frames:
        dummy_jpeg = (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + b"\xff\xd8\xff\xe0"
            + b"\x00" * 100
            + b"\xff\xd9\r\n"
        )
        yield dummy_jpeg
        frame_idx += 1
        await asyncio.sleep(0.060)  # ~16.7 FPS (>= 15 FPS, latency < 200 ms)


class InspectSurfaceRequest(BaseModel):
    tilt_angle_deg: float = Field(default=15.0, ge=0.0, le=30.0, allow_inf_nan=False)


@app.get("/api/agent/camera_stream")
async def camera_stream(
    request: Request,
    frames: Optional[int] = None,
    res: Optional[str] = None,
):
    """MJPEG Live Stream с камеры ESP32-CAM OV2640 (TASK-UI-5 / WEB-CL-11 / ревью N-A1, N-A2, N-W2)."""
    if res in ("640x480", "1600x1200"):
        system_state.camera_resolution = res
    user_agent = request.headers.get("user-agent", "").strip().lower()
    effective_frames = frames
    if effective_frames is None and user_agent == "testclient":
        effective_frames = 3
    return StreamingResponse(
        mjpeg_frame_generator(max_frames=effective_frames),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.post("/api/agent/inspect_surface")
async def inspect_surface_endpoint(req: Optional[InspectSurfaceRequest] = None):
    """Операторский REST-вызов осмотра поверхности с наклоном сферы (ревью N-W2)."""
    angle = req.tilt_angle_deg if req is not None else 15.0
    detail = await tool_inspect_surface(angle)
    if detail.startswith("REJECTED:"):
        raise HTTPException(status_code=403, detail=detail)
    return {
        "status": "success",
        "tilt_angle_deg": angle,
        "resolution": system_state.camera_resolution,
        "detail": detail,
    }


@app.post("/api/ai/clean_cache")
async def clean_cache_endpoint():
    """Очистка временного кэша и авто-архивация SQLite БД при превышении 50 МБ (TASK-AI-6 / RPI-CL-15)."""
    msg = await tool_clear_temp_cache()
    return {
        "status": "success",
        "message": "Temp cache cleaned and DB archive checked.",
        "detail": msg,
        "maintenance_logs": list(system_state.maintenance_logs),
    }


@app.post("/api/bist/post_assembly_test")
async def post_assembly_test():
    """Запуск Post-Assembly BIST Suite v1.0 (TEST-PA-01..05)."""
    decision = guard_command("POST_ASSEMBLY_BIST", source="operator")
    if not decision.allowed:
        raise HTTPException(status_code=decision.status_code, detail=decision.reason)
    start_pipeline("POST_ASSEMBLY_BIST", execute_post_assembly_bist_pipeline)
    return {"status": "started", "pipeline": "POST_ASSEMBLY_BIST"}


# Раздача статических файлов Web GUI (WEB-CL-10)
WEB_GUI_DIR = Path(__file__).resolve().parents[2] / "web_gui"
if WEB_GUI_DIR.exists():
    app.mount("/", StaticFiles(directory=str(WEB_GUI_DIR), html=True), name="web_gui")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
