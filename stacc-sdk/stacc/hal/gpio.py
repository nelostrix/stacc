from __future__ import annotations

import os
import sys
import time
import importlib
import math
from typing import Any, Dict, Optional

# ══════════════════════════════════════════
# GPIO
# ══════════════════════════════════════════


class GPIO:
    """Universal GPIO interface.

    Works on Raspberry Pi (RPi.GPIO or gpiod), Jetson (Jetson.GPIO),
    or in simulation mode (logs actions without hardware).

    Usage:
        gpio = GPIO()
        gpio.setup(18, GPIO.OUTPUT)
        gpio.write(18, GPIO.HIGH)

        gpio.setup(23, GPIO.INPUT, pull=GPIO.PULL_UP)
        value = gpio.read(23)
    """

    OUTPUT = "output"
    INPUT = "input"
    HIGH = 1
    LOW = 0
    PULL_UP = "pull_up"
    PULL_DOWN = "pull_down"
    PULL_NONE = "none"

    def __init__(self, simulation: bool = False):
        self.simulation = simulation
        self._backend: Optional[str] = None
        self._pin_states: Dict[int, int] = {}
        self._pin_modes: Dict[int, str] = {}

        if not simulation:
            self._backend = self._detect_backend()
            self._gpio_module()

    def _gpio_module(self):
        """Resolve an implemented provider; never substitute cached simulation I/O."""
        if self.simulation:
            return None
        modules = {"rpigpio": "RPi.GPIO", "jetson": "Jetson.GPIO"}
        if self._backend not in modules:
            raise RuntimeError(
                "No supported real GPIO backend is available. Install RPi.GPIO or "
                "Jetson.GPIO on the matching board, or explicitly use GPIO(simulation=True). "
                "The gpiod backend is not implemented."
            )
        return importlib.import_module(modules[self._backend])

    # ── Backend detection ────────────────

    @staticmethod
    def _detect_backend() -> Optional[str]:
        try:
            import RPi.GPIO as rpi_gpio  # noqa: F401

            rpi_gpio.setmode(rpi_gpio.BCM)
            rpi_gpio.setwarnings(False)
            return "rpigpio"
        except ImportError:
            pass
        try:
            import Jetson.GPIO as jetson_gpio  # noqa: F401

            jetson_gpio.setmode(jetson_gpio.BCM)
            return "jetson"
        except ImportError:
            pass
        try:
            import gpiod  # noqa: F401

            return "gpiod"
        except ImportError:
            pass
        return None

    # ── Public API ──────────────────────

    def setup(self, pin: int, mode: str, pull: str = "none") -> None:
        """Configure a GPIO pin.

        Args:
            pin: Pin number (BCM numbering)
            mode: GPIO.OUTPUT or GPIO.INPUT
            pull: GPIO.PULL_UP, GPIO.PULL_DOWN, or GPIO.PULL_NONE
        """
        if mode not in (self.INPUT, self.OUTPUT):
            raise ValueError(f"Unknown GPIO mode: {mode}")
        if pull not in (self.PULL_NONE, self.PULL_UP, self.PULL_DOWN):
            raise ValueError(f"Unknown GPIO pull: {pull}")
        rpi_gpio = self._gpio_module()
        if rpi_gpio is not None:

            direction = rpi_gpio.OUT if mode == self.OUTPUT else rpi_gpio.IN
            pud = (
                rpi_gpio.PUD_UP
                if pull == self.PULL_UP
                else (rpi_gpio.PUD_DOWN if pull == self.PULL_DOWN else rpi_gpio.PUD_OFF)
            )
            if mode == self.INPUT:
                rpi_gpio.setup(pin, direction, pull_up_down=pud)
            else:
                rpi_gpio.setup(pin, direction)
        self._pin_modes[pin] = mode
        self._pin_states[pin] = 0

    def write(self, pin: int, value: int) -> None:
        """Set a GPIO pin HIGH or LOW.

        Args:
            pin: Pin number
            value: GPIO.HIGH (1) or GPIO.LOW (0)
        """
        rpi_gpio = self._gpio_module()
        if rpi_gpio is not None:
            rpi_gpio.output(pin, value)
        self._pin_states[pin] = value

    def read(self, pin: int) -> int:
        """Read the current value of a GPIO pin.

        Args:
            pin: Pin number

        Returns:
            GPIO.HIGH (1) or GPIO.LOW (0)
        """
        rpi_gpio = self._gpio_module()
        if rpi_gpio is not None:
            val = rpi_gpio.input(pin)
            self._pin_states[pin] = val
            return val
        return self._pin_states.get(pin, 0)

    def pwm_start(
        self, pin: int, frequency: float = 1000, duty_cycle: float = 0.0
    ) -> Any:
        """Start PWM on a pin.

        Args:
            pin: Pin number
            frequency: PWM frequency in Hz (default 1000)
            duty_cycle: Duty cycle 0-100 (default 0)

        Returns:
            PWM object, or None in simulation mode
        """
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError("PWM frequency must be finite and greater than zero")
        if not math.isfinite(duty_cycle) or not 0 <= duty_cycle <= 100:
            raise ValueError("PWM duty cycle must be finite and between 0 and 100")
        rpi_gpio = self._gpio_module()
        if rpi_gpio is not None:
            self.setup(pin, self.OUTPUT)
            pwm = rpi_gpio.PWM(pin, frequency)
            pwm.start(duty_cycle)
            return pwm
        return None

    def cleanup(self) -> None:
        """Clean up GPIO resources. Call on shutdown."""
        rpi_gpio = self._gpio_module()
        if rpi_gpio is not None:
            rpi_gpio.cleanup()
        self._pin_states.clear()
        self._pin_modes.clear()

    def __repr__(self) -> str:
        backend = "simulation" if self.simulation else self._backend or "none"
        return f"GPIO(backend={backend!r}, pins={len(self._pin_states)})"
