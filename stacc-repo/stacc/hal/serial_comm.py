# ══════════════════════════════════════════
# SERIAL COMMUNICATION
# ══════════════════════════════════════════
from __future__ import annotations

import os
import sys
import time
from typing import Any, Dict, List, Optional


class SerialPort:
    """Universal serial/UART communication.

    Works on any platform with pyserial.

    Usage:
        port = SerialPort("/dev/ttyUSB0", baudrate=115200)
        port.open()
        port.write(b"hello")
        data = port.readline()
        port.close()
    """

    def __init__(
        self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, timeout: float = 1.0
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None

    def open(self) -> bool:
        try:
            import serial

            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(0.5)  # wait for connection
            return True
        except Exception as e:
            print(f"  [serial] Failed to open {self.port}: {e}")
            return False

    def close(self):
        if self._serial and self._serial.is_open:
            self._serial.close()

    def write(self, data: bytes) -> int:
        if self._serial and self._serial.is_open:
            return self._serial.write(data)
        return 0

    def write_string(self, text: str):
        self.write(text.encode("utf-8"))

    def readline(self) -> str:
        if self._serial and self._serial.is_open:
            return self._serial.readline().decode("utf-8", errors="ignore").strip()
        return ""

    def read(self, size: int = 1) -> bytes:
        if self._serial and self._serial.is_open:
            return self._serial.read(size)
        return b""

    def read_all(self) -> bytes:
        if self._serial and self._serial.is_open:
            return self._serial.read_all()
        return b""

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def available(self) -> int:
        if self._serial and self._serial.is_open:
            return self._serial.in_waiting
        return 0

    @classmethod
    def list_ports(cls) -> List[str]:
        try:
            import serial.tools.list_ports

            return [p.device for p in serial.tools.list_ports.comports()]
        except ImportError:
            return []
