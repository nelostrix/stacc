from __future__ import annotations

import os
import sys
import time
from typing import Any, Dict


class Board:
    """Auto-detect and configure the hardware platform.

    Usage:
        board = Board.detect()
        print(board.platform)  # "raspberry_pi", "jetson", "desktop", etc.
    """

    PLATFORMS = {
        "raspberry_pi": "Raspberry Pi (BCM GPIO)",
        "jetson": "NVIDIA Jetson (Jetson.GPIO)",
        "desktop": "Desktop (no GPIO, serial only)",
        "unknown": "Unknown platform",
    }

    def __init__(self, platform: str = "unknown"):
        self.platform = platform
        self.gpio_available = False
        self.i2c_available = False
        self.spi_available = False
        self.serial_available = True  # serial works everywhere

    @classmethod
    def detect(cls) -> "Board":
        board = cls()

        # Check for Raspberry Pi
        if os.path.exists("/proc/device-tree/model"):
            try:
                with open("/proc/device-tree/model") as f:
                    model = f.read().lower()
                if "raspberry" in model:
                    board.platform = "raspberry_pi"
                    board.gpio_available = True
                    board.i2c_available = True
                    board.spi_available = True
                elif "jetson" in model:
                    board.platform = "jetson"
                    board.gpio_available = True
                    board.i2c_available = True
            except Exception:
                pass

        if board.platform == "unknown":
            board.platform = "desktop"

        return board

    def info(self) -> Dict[str, Any]:
        return {
            "platform": self.platform,
            "description": self.PLATFORMS.get(self.platform, "Unknown"),
            "gpio": self.gpio_available,
            "i2c": self.i2c_available,
            "spi": self.spi_available,
            "serial": self.serial_available,
        }

    def __repr__(self):
        return f"Board(platform='{self.platform}')"
