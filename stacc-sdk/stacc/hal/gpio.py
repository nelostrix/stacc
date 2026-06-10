from __future__ import annotations

import os
import sys
import time
from typing import Any, Dict

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
        self._backend = None
        self._pin_states: Dict[int, int] = {}
        self._pin_modes: Dict[int, str] = {}

        if not simulation:
            self._backend = self._detect_backend()

    def _detect_backend(self) -> Optional[str]:
        try:
            import RPi.GPIO as rpi_gpio

            rpi_gpio.setmode(rpi_gpio.BCM)
            rpi_gpio.setwarnings(False)
            return "rpigpio"
        except ImportError:
            pass
        try:
            import Jetson.GPIO as jetson_gpio

            jetson_gpio.setmode(jetson_gpio.BCM)
            return "jetson"
        except ImportError:
            pass
        try:
            import gpiod

            return "gpiod"
        except ImportError:
            pass
        return None

    def setup(self, pin: int, mode: str, pull: str = "none"):
        self._pin_modes[pin] = mode
        self._pin_states[pin] = 0

        if self._backend == "rpigpio":
            import RPi.GPIO as rpi_gpio

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

    def write(self, pin: int, value: int):
        self._pin_states[pin] = value
        if self._backend == "rpigpio":
            import RPi.GPIO as rpi_gpio

            rpi_gpio.output(pin, value)

    def read(self, pin: int) -> int:
        if self._backend == "rpigpio":
            import RPi.GPIO as rpi_gpio

            val = rpi_gpio.input(pin)
            self._pin_states[pin] = val
            return val
        return self._pin_states.get(pin, 0)

    def pwm_start(self, pin: int, frequency: float = 1000, duty_cycle: float = 0.0):
        """Start PWM on a pin. duty_cycle: 0-100."""
        if self._backend == "rpigpio":
            import RPi.GPIO as rpi_gpio

            self.setup(pin, self.OUTPUT)
            pwm = rpi_gpio.PWM(pin, frequency)
            pwm.start(duty_cycle)
            return pwm
        return None

    def cleanup(self):
        if self._backend == "rpigpio":
            import RPi.GPIO as rpi_gpio

            rpi_gpio.cleanup()
