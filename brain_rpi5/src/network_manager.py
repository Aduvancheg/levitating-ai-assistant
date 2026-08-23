import asyncio
import logging
from typing import Tuple

logger = logging.getLogger(__name__)

class NetworkManager:
    """
    Handles RPi5 network autocutover and Wi-Fi provisioning.
    Requires NetworkManager (nmcli) to be installed and active on the host OS.
    """

    AP_SSID = "Antigravity_Base_AP"
    AP_PASS = "antigravity"
    AP_IP = "192.168.4.1"

    @classmethod
    async def run_cmd(cls, cmd: str) -> Tuple[int, str, str]:
        """Run a shell command asynchronously and return (exit_code, stdout, stderr)."""
        process = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await process.communicate()
        return process.returncode or 0, stdout.decode().strip(), stderr.decode().strip()

    @classmethod
    async def is_connected(cls) -> bool:
        """Check if currently connected to a Wi-Fi network (STA mode)."""
        code, stdout, _ = await cls.run_cmd("nmcli -t -f TYPE,STATE dev")
        for line in stdout.split('\n'):
            if line.startswith("wifi:connected"):
                return True
        return False

    @classmethod
    async def start_ap(cls) -> bool:
        """Create and start the fallback Wi-Fi hotspot."""
        logger.info(f"Starting fallback AP: {cls.AP_SSID}")
        # Check if connection already exists
        code, stdout, _ = await cls.run_cmd(f"nmcli -t -f NAME con show")
        if cls.AP_SSID in stdout.split('\n'):
            # Just bring it up
            code, _, stderr = await cls.run_cmd(f"nmcli con up {cls.AP_SSID}")
        else:
            # Create new hotspot
            cmd = (
                f"nmcli dev wifi hotspot ifname wlan0 "
                f"ssid {cls.AP_SSID} password {cls.AP_PASS} "
                f"ipv4.method shared ipv4.address {cls.AP_IP}/24"
            )
            code, _, stderr = await cls.run_cmd(cmd)
            
        if code == 0:
            logger.info("AP started successfully.")
            return True
        else:
            logger.error(f"Failed to start AP: {stderr}")
            return False

    @classmethod
    async def handover_network(cls, target_ssid: str, target_pass: str) -> bool:
        """
        Execute the network handover procedure:
        1. Broadcast credentials to the Agent via UDP (port 4242).
        2. Attempt to connect the RPi5 to the target network.
        """
        logger.info(f"Executing handover to {target_ssid}")
        
        # 1. Send UDP packet to Agent (broadcast on AP subnet)
        try:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            payload = f"{target_ssid}\n{target_pass}".encode('utf-8')
            # 192.168.4.255 is the broadcast address for the AP
            sock.sendto(payload, ("192.168.4.255", 4242))
            sock.close()
            logger.info("Broadcasted credentials to Agent via UDP.")
        except Exception as e:
            logger.error(f"UDP Broadcast failed: {e}")

        # Give the agent a moment to receive it before we drop the AP
        await asyncio.sleep(2.0)

        # 2. Connect to the new network
        logger.info(f"Connecting RPi5 to {target_ssid}...")
        cmd = f"nmcli dev wifi connect '{target_ssid}' password '{target_pass}'"
        code, stdout, stderr = await cls.run_cmd(cmd)
        
        if code == 0:
            logger.info("Successfully connected to the new network.")
            return True
        else:
            logger.error(f"Failed to connect to {target_ssid}: {stderr}")
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
