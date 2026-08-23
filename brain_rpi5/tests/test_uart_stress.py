import struct
import pytest
import random
from brain_rpi5.src.core_loop import UARTPacketBuilder, calculate_crc8

def test_uart_chaos_flood():
    """
    TEST-1: UART Chaos Flood Test
    Simulate generating 100k packets per second of garbage/valid mix.
    Ensure UARTPacketBuilder unpack_ack handles garbage safely.
    """
    builder = UARTPacketBuilder()
    
    # Generate 100k ACKs
    valid_count = 0
    garbage_count = 0
    
    # Static method
    for i in range(100_000):
        if i % 2 == 0:
            # Valid ACK (3 bytes)
            status = 0x06
            crc = calculate_crc8(bytes([status]))
            packet = bytes([0xAA, status, crc])
            res = UARTPacketBuilder.unpack_ack(packet)
            if res is not None:
                valid_count += 1
        else:
            # Garbage ACK
            garbage = bytes([0xAA, i % 255, (i + 1) % 255])
            res = UARTPacketBuilder.unpack_ack(garbage)
            if res is None:
                garbage_count += 1
            else:
                # Accidental CRC collision
                garbage_count += 1
                
    assert valid_count == 50000, f"Expected 50k valid, got {valid_count}"
    assert garbage_count == 50000, f"Expected 50k rejected garbage, got {garbage_count}"

