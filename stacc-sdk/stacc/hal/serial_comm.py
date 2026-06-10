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

    Works on any platform with pyserial. Supports context manager usage.

    Usage:
        port = SerialPort("/dev/ttyUSB0", baudrate=115200)
        port.open()
        port.write(b"hello")
        data = port.readline()
        port.close()

    Or as a context manager:
        with SerialPort("/dev/ttyUSB0") as port:
            port.write(b"hello")
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout: float = 1.0,
    ):
        if baudrate <= 0:
            raise ValueError(f"baudrate must be > 0, got {baudrate}")
        if timeout < 0:
            raise ValueError(f"timeout must be >= 0, got {timeout}")

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Any = None

    # ── Public API ──────────────────────

    def open(self) -> bool:
        """Open the serial port.

        Returns:
            True if the port opened successfully
        """
        try:
            import serial

            self._serial = serial.Serial(
                self.port, self.baudrate, timeout=self.timeout
            )
            time.sleep(0.5)
            return True
        except Exception as e:
            print(f"  [serial] Failed to open {self.port}: {e}")
            return False

    def close(self) -> None:
        """Close the serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None

    def write(self, data: bytes) -> int:
        """Write bytes to the serial port.

        Args:
            data: Bytes to send

        Returns:
            Number of bytes written
        """
        if self._serial and self._serial.is_open:
            return self._serial.write(data)
        return 0

    def write_string(self, text: str) -> None:
        """Write a string to the serial port (UTF-8 encoded)."""
        self.write(text.encode("utf-8"))

    def readline(self) -> str:
        """Read a line from the serial port.

        Returns:
            Decoded string, stripped of trailing whitespace
        """
        if self._serial and self._serial.is_open:
            return self._serial.readline().decode("utf-8", errors="ignore").strip()
        return ""

    def read(self, size: int = 1) -> bytes:
        """Read bytes from the serial port.

        Args:
            size: Number of bytes to read (default 1)

        Returns:
            Read bytes
        """
        if self._serial and self._serial.is_open:
            return self._serial.read(size)
        return b""

    def read_all(self) -> bytes:
        """Read all available bytes from the serial port."""
        if self._serial and self._serial.is_open:
            return self._serial.read_all()
        return b""

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def available(self) -> int:
        """Number of bytes waiting to be read."""
        if self._serial and self._serial.is_open:
            return self._serial.in_waiting
        return 0

    @classmethod
    def list_ports(cls) -> List[str]:
        """List available serial port device paths.

        Returns:
            List of port paths (e.g. ["/dev/ttyUSB0", "/dev/ttyACM0"])
        """
        try:
            import serial.tools.list_ports

            return [p.device for p in serial.tools.list_ports.comports()]
        except ImportError:
            return []

    # ── Context manager ─────────────────

    def __enter__(self) -> SerialPort:
        self.open()
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return (
            f"SerialPort(port={self.port!r}, "
            f"baudrate={self.baudrate}, "
            f"open={self.is_open})"
        )
