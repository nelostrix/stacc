# ══════════════════════════════════════════
# I2C BUS
# ══════════════════════════════════════════
from __future__ import annotations

import os
import sys
import time
from typing import List, Optional


class I2CBus:
    """Universal I2C communication.

    Uses smbus2 on Linux (RPi, Jetson).

    Usage:
        bus = I2CBus(bus_number=1)
        bus.open()
        value = bus.read_byte(address=0x68, register=0x00)
        bus.write_byte(address=0x68, register=0x6B, value=0x00)
    """

    def __init__(self, bus_number: int = 1):
        self.bus_number = bus_number
        self._bus = None

    def open(self) -> bool:
        try:
            import smbus2

            self._bus = smbus2.SMBus(self.bus_number)
            return True
        except Exception as e:
            print(f"  [i2c] Failed to open bus {self.bus_number}: {e}")
            return False

    def close(self):
        if self._bus:
            self._bus.close()

    def read_byte(self, address: int, register: int) -> int:
        if self._bus:
            return self._bus.read_byte_data(address, register)
        return 0

    def read_word(self, address: int, register: int) -> int:
        if self._bus:
            return self._bus.read_word_data(address, register)
        return 0

    def read_block(self, address: int, register: int, length: int) -> List[int]:
        if self._bus:
            return self._bus.read_i2c_block_data(address, register, length)
        return [0] * length

    def write_byte(self, address: int, register: int, value: int):
        if self._bus:
            self._bus.write_byte_data(address, register, value)

    def write_word(self, address: int, register: int, value: int):
        if self._bus:
            self._bus.write_word_data(address, register, value)

    def write_block(self, address: int, register: int, data: List[int]):
        if self._bus:
            self._bus.write_i2c_block_data(address, register, data)

    def scan(self) -> List[int]:
        """Scan for devices on the bus. Returns list of addresses."""
        devices = []
        if self._bus:
            for addr in range(0x03, 0x78):
                try:
                    self._bus.read_byte(addr)
                    devices.append(addr)
                except Exception:
                    pass
        return devices
