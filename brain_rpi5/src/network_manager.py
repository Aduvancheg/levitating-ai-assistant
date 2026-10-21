import asyncio
import logging
import os
import socket
from typing import List, Optional, Sequence, Tuple

try:
    from .security import config_dir, read_private_file, write_private_file
except ImportError:  # запуск как скрипт из каталога src/
    from security import config_dir, read_private_file, write_private_file

logger = logging.getLogger(__name__)

# Ограничения IEEE 802.11 / WPA2-PSK
SSID_MAX_BYTES = 32
WPA_PASS_MIN = 8
WPA_PASS_MAX = 63


class CredentialError(ValueError):
    """Недопустимые SSID/пароль Wi-Fi."""


def validate_wifi_credentials(ssid: str, password: str) -> None:
    """Проверить SSID и пароль до передачи в nmcli / Агенту.

    Аргументы передаются в nmcli списком (без shell), поэтому инъекция
    невозможна в принципе. Валидация нужна, чтобы отсечь мусор и символы,
    которые сломают UDP-формат `ssid\\npassword` на стороне Агента.
    """
    if not ssid:
        raise CredentialError("SSID пустой.")
    if len(ssid.encode("utf-8")) > SSID_MAX_BYTES:
        raise CredentialError(f"SSID длиннее {SSID_MAX_BYTES} байт.")
    for value, name in ((ssid, "SSID"), (password, "Пароль")):
        if any(ch in value for ch in ("\x00", "\n", "\r")):
            raise CredentialError(f"{name} содержит управляющие символы.")
    if password and not (WPA_PASS_MIN <= len(password) <= WPA_PASS_MAX):
        raise CredentialError(
            f"Пароль WPA2 должен быть {WPA_PASS_MIN}..{WPA_PASS_MAX} символов (или пустой для открытой сети)."
        )


def _load_ap_password() -> str:
    """Пароль точки доступа: env → файл 0600 → сгенерированный (ревью A10).

    Раньше пароль был захардкожен в исходниках ("antigravity").
    """
    env_value = os.environ.get("LEVITATION_AP_PASSWORD", "").strip()
    if env_value:
        return env_value
    path = config_dir() / "ap_password"
    try:
        existing = read_private_file(path)
        if existing:
            return existing
        import secrets
        alphabet = "abcdefghjkmnpqrstuvwxyz23456789"  # без похожих символов: удобно вводить с наклейки
        new_value = "".join(secrets.choice(alphabet) for _ in range(12))
        write_private_file(path, new_value + "\n")
        logger.warning("Пароль AP сгенерирован и сохранён в %s (0600).", path)
        return new_value
    except OSError as e:
        logger.error("Не удалось прочитать/сохранить пароль AP (%s). Нужна переменная LEVITATION_AP_PASSWORD.", e)
        raise


class NetworkManager:
    """
    Handles RPi5 network autocutover and Wi-Fi provisioning.
    Requires NetworkManager (nmcli) to be installed and active on the host OS.
    """

    AP_SSID = "Antigravity_Base_AP"
    AP_IP = "192.168.4.1"
    AP_BROADCAST = "192.168.4.255"
    HANDOVER_PORT = 4242
    _ap_password: Optional[str] = None

    @classmethod
    def ap_password(cls) -> str:
        if cls._ap_password is None:
            cls._ap_password = _load_ap_password()
        return cls._ap_password

    @classmethod
    async def run_cmd(cls, args: Sequence[str]) -> Tuple[int, str, str]:
        """Run a command asynchronously and return (exit_code, stdout, stderr).

        Аргументы передаются списком через exec — без shell (ревью A10:
        раньше SSID/пароль подставлялись в строку для `create_subprocess_shell`).
        """
        try:
            process = await asyncio.create_subprocess_exec(
                *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
        except FileNotFoundError as e:
            return 127, "", str(e)
        stdout, stderr = await process.communicate()
        return process.returncode or 0, stdout.decode().strip(), stderr.decode().strip()

    @classmethod
    async def is_connected(cls) -> bool:
        """Check if currently connected to a Wi-Fi network (STA mode)."""
        code, stdout, _ = await cls.run_cmd(["nmcli", "-t", "-f", "TYPE,STATE", "dev"])
        for line in stdout.split('\n'):
            if line.startswith("wifi:connected"):
                return True
        return False

    @classmethod
    async def start_ap(cls) -> bool:
        """Create and start the fallback Wi-Fi hotspot."""
        logger.info(f"Starting fallback AP: {cls.AP_SSID}")
        # Check if connection already exists
        code, stdout, _ = await cls.run_cmd(["nmcli", "-t", "-f", "NAME", "con", "show"])
        if cls.AP_SSID in stdout.split('\n'):
            # Just bring it up
            code, _, stderr = await cls.run_cmd(["nmcli", "con", "up", cls.AP_SSID])
        else:
            # Create new hotspot
            # con-name = AP_SSID, иначе nmcli назовёт профиль "Hotspot",
            # и проверка "профиль уже существует" выше никогда не сработает.
            code, _, stderr = await cls.run_cmd([
                "nmcli", "dev", "wifi", "hotspot", "ifname", "wlan0",
                "con-name", cls.AP_SSID, "ssid", cls.AP_SSID, "password", cls.ap_password(),
            ])
            if code == 0:
                # Фиксированный адрес шлюза AP. `nmcli dev wifi hotspot` не принимает
                # ipv4.* аргументы, поэтому задаём их отдельно и переподнимаем профиль.
                await cls.run_cmd([
                    "nmcli", "con", "modify", cls.AP_SSID,
                    "ipv4.method", "shared", "ipv4.addresses", f"{cls.AP_IP}/24",
                ])
                await cls.run_cmd(["nmcli", "con", "up", cls.AP_SSID])

        if code == 0:
            logger.info("AP started successfully.")
            return True
        else:
            logger.error(f"Failed to start AP: {stderr}")
            return False

    @classmethod
    def _send_credentials_to_agent(cls, payload: bytes, agent_ip: Optional[str]) -> None:
        """Передать креды Агенту по UDP.

        Если IP Агента известен — unicast, иначе broadcast по подсети AP.
        TODO(F4): payload пока открытым текстом (защищён только WPA2 самой AP).
        Нужна аутентификация/шифрование на стороне прошивки ESP32.
        """
        target = agent_ip if agent_ip and agent_ip not in ("0.0.0.0", "") else cls.AP_BROADCAST
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            if target == cls.AP_BROADCAST:
                logger.warning("IP Агента неизвестен — креды уходят broadcast'ом по подсети AP.")
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(payload, (target, cls.HANDOVER_PORT))
        finally:
            sock.close()

    @classmethod
    async def handover_network(cls, target_ssid: str, target_pass: str, agent_ip: Optional[str] = None) -> bool:
        """
        Execute the network handover procedure:
        1. Send credentials to the Agent via UDP (port 4242).
        2. Attempt to connect the RPi5 to the target network.

        Raises CredentialError on invalid input (before anything is sent).
        """
        validate_wifi_credentials(target_ssid, target_pass)
        logger.info("Executing handover to SSID=%r", target_ssid)

        # 1. Send UDP packet to Agent
        try:
            payload = f"{target_ssid}\n{target_pass}".encode('utf-8')
            cls._send_credentials_to_agent(payload, agent_ip)
            logger.info("Sent credentials to Agent via UDP.")
        except Exception as e:
            logger.error(f"UDP send failed: {e}")

        # Give the agent a moment to receive it before we drop the AP
        await asyncio.sleep(2.0)

        # 2. Connect to the new network
        logger.info("Connecting RPi5 to SSID=%r...", target_ssid)
        args: List[str] = ["nmcli", "dev", "wifi", "connect", target_ssid]
        if target_pass:
            args += ["password", target_pass]
        code, stdout, stderr = await cls.run_cmd(args)

        if code == 0:
            logger.info("Successfully connected to the new network.")
            return True
        else:
            logger.error("Failed to connect to SSID=%r: %s", target_ssid, stderr)
            # Fallback to AP if connection failed
            await cls.start_ap()
            return False

    @classmethod
    async def auto_configure(cls) -> None:
        """
        Check connectivity. If offline, start AP automatically.
        Called during main server startup.
        """
        if not await cls.is_connected():
            logger.warning("No Wi-Fi connection detected. Booting fallback AP.")
            await cls.start_ap()
        else:
            logger.info("Wi-Fi connection is active.")
