"""
security.py — Аутентификация API и безопасное хранение секретов (ревью A1, A9).

Модель доступа:
  * Оператор (Web GUI, REST, WS /ws/telemetry) — токен `LEVITATION_API_TOKEN`.
  * Агент (ESP32, WS /ws/agent) — отдельный токен `LEVITATION_AGENT_TOKEN`,
    чтобы скомпрометированная сфера не получила доступ к API оператора.

Источник каждого токена (по приоритету):
  1. Переменная окружения.
  2. Файл в каталоге конфигурации (`LEVITATION_CONFIG_DIR`, по умолчанию
     /etc/antigravity) с правами 0600.
  3. Если файла нет — генерируется случайный токен и сохраняется туда же.

Секреты никогда не пишутся в лог целиком.
"""

from __future__ import annotations

import hmac
import logging
import os
import secrets
from pathlib import Path
from typing import Dict, Optional

logger = logging.getLogger("Brain_RPi5.security")

DEFAULT_CONFIG_DIR = Path("/etc/antigravity")
DEFAULT_DATA_DIR = Path("/var/log/antigravity")


def _can_use_dir(path: Path) -> bool:
    """Проверить, может ли процесс писать в каталог (или создать его в родителе)."""
    try:
        if path.exists():
            return os.access(path, os.W_OK | os.X_OK)
        parent = path.parent
        while not parent.exists() and parent != parent.parent:
            parent = parent.parent
        return os.access(parent, os.W_OK | os.X_OK)
    except OSError:
        return False


def config_dir() -> Path:
    """Каталог конфигурации (переопределяется `LEVITATION_CONFIG_DIR`).

    При запуске без root-прав и без явной переменной окружения автоматически
    откатывается в `$XDG_CONFIG_HOME/antigravity` (`~/.config/antigravity`),
    чтобы сервис и тесты работали от непривилегированного пользователя (A9).
    """
    env_dir = os.environ.get("LEVITATION_CONFIG_DIR", "").strip()
    if env_dir:
        return Path(env_dir)
    if _can_use_dir(DEFAULT_CONFIG_DIR):
        return DEFAULT_CONFIG_DIR
    xdg_config = Path(os.environ.get("XDG_CONFIG_HOME", str(Path.home() / ".config")))
    return xdg_config / "antigravity"


def data_dir() -> Path:
    """Каталог данных/логов (переопределяется `LEVITATION_DATA_DIR`)."""
    env_dir = os.environ.get("LEVITATION_DATA_DIR", "").strip()
    if env_dir:
        return Path(env_dir)
    if _can_use_dir(DEFAULT_DATA_DIR):
        return DEFAULT_DATA_DIR
    xdg_state = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local" / "state")))
    return xdg_state / "antigravity"


def mask_secret(value: str) -> str:
    """Маскировать секрет для логов: показываем только 4+4 символа."""
    if not value or len(value) < 12:
        return "***"
    return value[:4] + "****" + value[-4:]


def write_private_file(path: Path, content: str) -> None:
    """Атомарно записать файл с правами 0600 (каталог — 0700).

    Файл создаётся сразу с нужными правами, поэтому секрет ни в какой момент
    не лежит на диске с правами по умолчанию.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.chmod(path.parent, 0o700)
    except PermissionError:
        # Каталог может принадлежать другому пользователю (например, /etc).
        pass
    tmp = path.with_name(path.name + ".tmp")
    fd = os.open(str(tmp), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(content)
    except Exception:
        tmp.unlink(missing_ok=True)
        raise
    os.replace(tmp, path)
    os.chmod(path, 0o600)


def read_private_file(path: Path) -> Optional[str]:
    """Прочитать секрет из файла. Предупредить, если права шире 0600."""
    if not path.exists():
        return None
    mode = path.stat().st_mode & 0o777
    if mode & 0o077:
        logger.warning(
            "Файл секрета %s доступен группе/всем (mode=%o). Исправляю на 0600.",
            path, mode,
        )
        try:
            os.chmod(path, 0o600)
        except PermissionError:
            logger.error("Не удалось сменить права на %s.", path)
    value = path.read_text(encoding="utf-8").strip()
    return value or None


def _load_or_create_secret(env_var: str, file_name: str, label: str) -> str:
    env_value = os.environ.get(env_var, "").strip()
    if env_value:
        return env_value

    path = config_dir() / file_name
    try:
        existing = read_private_file(path)
        if existing:
            return existing
        new_value = secrets.token_urlsafe(32)
        write_private_file(path, new_value + "\n")
        logger.warning(
            "%s не задан: сгенерирован новый и сохранён в %s (0600). "
            "Скопируйте его оттуда для подключения.", label, path,
        )
        return new_value
    except OSError as e:
        # Нет прав на каталог конфигурации: токен живёт только в памяти.
        # Это единственный случай, когда токен выводится в лог — иначе
        # оператор не сможет подключиться вообще.
        new_value = secrets.token_urlsafe(32)
        logger.error(
            "Не удалось сохранить %s в %s (%s). Временный токен до перезапуска: %s",
            label, path, e, new_value,
        )
        return new_value


_cache: Dict[str, str] = {}


def get_api_token() -> str:
    """Токен оператора (REST + WS телеметрии)."""
    if "api" not in _cache:
        _cache["api"] = _load_or_create_secret("LEVITATION_API_TOKEN", "api_token", "API-токен оператора")
    return _cache["api"]


def get_agent_token() -> str:
    """Токен Агента (WS /ws/agent, отчёт реле)."""
    if "agent" not in _cache:
        _cache["agent"] = _load_or_create_secret("LEVITATION_AGENT_TOKEN", "agent_token", "Токен Агента")
    return _cache["agent"]


def reset_token_cache() -> None:
    """Сбросить кэш токенов (для тестов и ротации без перезапуска)."""
    _cache.clear()


def token_matches(provided: Optional[str], expected: str) -> bool:
    """Сравнение в постоянное время (защита от timing-атак)."""
    if not provided or not expected:
        return False
    return hmac.compare_digest(provided.encode("utf-8"), expected.encode("utf-8"))


def extract_bearer(authorization: Optional[str]) -> Optional[str]:
    """Достать токен из заголовка `Authorization: Bearer <token>`."""
    if not authorization:
        return None
    scheme, _, value = authorization.partition(" ")
    if scheme.lower() != "bearer":
        return None
    return value.strip() or None
