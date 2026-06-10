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

    Uses spidev on Linux. Supports context manager usage.

    Args:
        bus: SPI bus number (default 0)
        device: SPI device number / chip select (default 0)
        speed: SPI clock speed in Hz (default 1 MHz)
        mode: SPI mode 0-3 (default 0)

    Usage:
        spi = SPIBus(bus=0, device=0, speed=1000000)
        spi.open()
        response = spi.transfer([0x01, 0x02])

    Or as a context manager:
        with SPIBus(0, 0) as spi:
            response = spi.transfer([0x01, 0x02])
    """

    def __init__(
        self,
        bus: int = 0,
        device: int = 0,
        speed: int = 1000000,
        mode: int = 0,
    ):
        if bus < 0:
            raise ValueError(f"bus must be >= 0, got {bus}")
        if device < 0:
            raise ValueError(f"device must be >= 0, got {device}")
        if speed <= 0:
            raise ValueError(f"speed must be > 0, got {speed}")
        if mode not in (0, 1, 2, 3):
            raise ValueError(f"mode must be 0-3, got {mode}")

        self.bus = bus
        self.device = device
        self.speed = speed
        self.mode = mode
        self._spi: Any = None

    # ── Public API ──────────────────────

    def open(self) -> bool:
        """Open the SPI bus.

        Returns:
            True if the bus opened successfully
        """
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

    def close(self) -> None:
        """Close the SPI bus."""
        if self._spi:
            self._spi.close()
            self._spi = None

    def transfer(self, data: List[int]) -> List[int]:
        """Perform a full-duplex SPI transfer.

        Args:
            data: Bytes to send

        Returns:
            Received bytes
        """
        if self._spi:
            return self._spi.xfer2(data)
        return [0] * len(data)

    def write(self, data: List[int]) -> None:
        """Write bytes to the SPI bus (simplex).

        Args:
            data: Bytes to send
        """
        if self._spi:
            self._spi.writebytes(data)

    def read(self, length: int) -> List[int]:
        """Read bytes from the SPI bus (simplex).

        Args:
            length: Number of bytes to read

        Returns:
            Received bytes
        """
        if self._spi:
            return self._spi.readbytes(length)
        return [0] * length

    # ── Context manager ─────────────────

    def __enter__(self) -> SPIBus:
        self.open()
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return (
            f"SPIBus(bus={self.bus}, device={self.device}, "
            f"speed={self.speed}, mode={self.mode}, open={self._spi is not None})"
        )
