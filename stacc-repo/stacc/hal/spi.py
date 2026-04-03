# ══════════════════════════════════════════
# SPI BUS
# ══════════════════════════════════════════
from __future__ import annotations

import os
import sys
import time
from typing import Any, Dict, List, Optional


class SPIBus:
    """Universal SPI communication.

    Usage:
        spi = SPIBus(bus=0, device=0, speed=1000000)
        spi.open()
        response = spi.transfer([0x01, 0x02])
    """

    def __init__(
        self, bus: int = 0, device: int = 0, speed: int = 1000000, mode: int = 0
    ):
        self.bus = bus
        self.device = device
        self.speed = speed
        self.mode = mode
        self._spi = None

    def open(self) -> bool:
        try:
            import spidev

            self._spi = spidev.SpiDev()
            self._spi.open(self.bus, self.device)
            self._spi.max_speed_hz = self.speed
            self._spi.mode = self.mode
            return True
        except Exception as e:
            print(f"  [spi] Failed to open SPI {self.bus}.{self.device}: {e}")
            return False

    def close(self):
        if self._spi:
            self._spi.close()

    def transfer(self, data: List[int]) -> List[int]:
        if self._spi:
            return self._spi.xfer2(data)
        return [0] * len(data)

    def write(self, data: List[int]):
        if self._spi:
            self._spi.writebytes(data)

    def read(self, length: int) -> List[int]:
        if self._spi:
            return self._spi.readbytes(length)
        return [0] * length
