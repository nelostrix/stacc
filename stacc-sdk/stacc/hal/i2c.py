# ══════════════════════════════════════════
# I2C BUS
# ══════════════════════════════════════════
from __future__ import annotations

import os
import sys
import time
from typing import Any, List, Optional


class I2CBus:
    """Universal I2C communication.

    Uses smbus2 on Linux (RPi, Jetson). Supports context manager usage.

    Usage:
        bus = I2CBus(bus_number=1)
        bus.open()
        value = bus.read_byte(address=0x68, register=0x00)
        bus.write_byte(address=0x68, register=0x6B, value=0x00)
        bus.close()

    Or as a context manager:
        with I2CBus(1) as bus:
            value = bus.read_byte(0x68, 0x00)
    """

    def __init__(self, bus_number: int = 1):
        if bus_number < 0:
            raise ValueError(f"bus_number must be >= 0, got {bus_number}")

        self.bus_number = bus_number
        self._bus: Any = None

    # ── Public API ──────────────────────

    def open(self) -> bool:
        """Open the I2C bus.

        Returns:
            True if the bus opened successfully
        """
        try:
            import smbus2

            self._bus = smbus2.SMBus(self.bus_number)
            return True
        except Exception as e:
            print(f"  [i2c] Failed to open bus {self.bus_number}: {e}")
            return False

    def close(self) -> None:
        """Close the I2C bus."""
        if self._bus:
            self._bus.close()
            self._bus = None

    def read_byte(self, address: int, register: int) -> int:
        """Read a single byte from a register."""
        if self._bus:
            return self._bus.read_byte_data(address, register)
        return 0

    def read_word(self, address: int, register: int) -> int:
        """Read a word (2 bytes) from a register."""
        if self._bus:
            return self._bus.read_word_data(address, register)
        return 0

    def read_block(self, address: int, register: int, length: int) -> List[int]:
        """Read a block of bytes from a register.

        Args:
            address: I2C device address
            register: Register address
            length: Number of bytes to read

        Returns:
            List of byte values
        """
        if self._bus:
            return self._bus.read_i2c_block_data(address, register, length)
        return [0] * length

    def write_byte(self, address: int, register: int, value: int) -> None:
        """Write a single byte to a register."""
        if self._bus:
            self._bus.write_byte_data(address, register, value)

    def write_word(self, address: int, register: int, value: int) -> None:
        """Write a word (2 bytes) to a register."""
        if self._bus:
            self._bus.write_word_data(address, register, value)

    def write_block(self, address: int, register: int, data: List[int]) -> None:
        """Write a block of bytes to a register."""
        if self._bus:
            self._bus.write_i2c_block_data(address, register, data)

    def scan(self) -> List[int]:
        """Scan for devices on the bus.

        Returns:
            List of detected I2C device addresses
        """
        devices: List[int] = []
        if self._bus:
            for addr in range(0x03, 0x78):
                try:
                    self._bus.read_byte(addr)
                    devices.append(addr)
                except Exception:
                    pass
        return devices

    # ── Context manager ─────────────────

    def __enter__(self) -> I2CBus:
        self.open()
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"I2CBus(bus={self.bus_number}, open={self._bus is not None})"
