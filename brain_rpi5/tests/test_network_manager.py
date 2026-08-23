import pytest
import asyncio
from unittest.mock import patch, AsyncMock, MagicMock
from brain_rpi5.src.network_manager import NetworkManager
from brain_rpi5.src.main_server import system_state

@pytest.mark.asyncio
async def test_wifi_loss_handover():
    """
    TEST-5: Wi-Fi Loss Safe Landing & Handover Test
    Simulate Wi-Fi drop -> AP fallback -> UDP handover -> reconnect.
    """
    nm = NetworkManager()
    
    with patch("asyncio.create_subprocess_shell") as mock_run:
        # 1. Simulate Home Wi-Fi is lost, auto_configure should start AP
        mock_process = AsyncMock()
        mock_process.returncode = 1 # ping fails
        mock_process.communicate.return_value = (b"", b"")
        mock_run.return_value = mock_process
        
        # Test auto_configure
        await nm.auto_configure()
        
        # Should have tried to start the AP
        assert mock_run.call_count >= 1
        # The state should be updated
        assert system_state.base_wifi_mode == "AP"
        
        # 2. Simulate Handover
        # Mock the synchronous socket
        with patch("socket.socket") as mock_socket:
            mock_sock_instance = MagicMock()
            mock_socket.return_value = mock_sock_instance
            
            # Handover to new network (make mock return success)
            mock_process.returncode = 0
            result = await nm.handover_network("NewHomeNetwork", "supersecret")
            
            # Should have broadcasted the UDP packet
            mock_sock_instance.sendto.assert_called_once()
            
            # Should return True for success
            assert result is True
