from __future__ import annotations

import os
import sys
import time
from typing import Any, Dict, List, Optional

from stacc.hal.gpio import GPIO

# ══════════════════════════════════════════
# MOTOR DRIVER
# ══════════════════════════════════════════


class MotorDriver:
    """Universal motor driver — controls DC motors, servos, steppers.

    Supports L298N, L293D, PCA9685 (via I2C), and direct PWM.

    Usage:
        motor = MotorDriver(pin_a=17, pin_b=27, pin_enable=22, gpio=gpio)
        motor.set_speed(0.5)   # 50% forward
        motor.set_speed(-0.3)  # 30% reverse
        motor.stop()
    """

    def __init__(
        self,
        pin_a: int,
        pin_b: int,
        pin_enable: Optional[int] = None,
        gpio: Optional[GPIO] = None,
        pwm_freq: float = 1000,
    ):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pin_enable = pin_enable
        self.gpio = gpio or GPIO(simulation=True)
        self.pwm_freq = pwm_freq
        self._speed = 0.0
        self._pwm = None

        self.gpio.setup(pin_a, GPIO.OUTPUT)
        self.gpio.setup(pin_b, GPIO.OUTPUT)
        if pin_enable is not None:
            self.gpio.setup(pin_enable, GPIO.OUTPUT)
            self._pwm = self.gpio.pwm_start(pin_enable, pwm_freq, 0)

    def set_speed(self, speed: float):
        """Set motor speed. -1.0 to 1.0 (negative = reverse)."""
        speed = max(-1.0, min(1.0, speed))
        self._speed = speed

        if speed > 0:
            self.gpio.write(self.pin_a, GPIO.HIGH)
            self.gpio.write(self.pin_b, GPIO.LOW)
        elif speed < 0:
            self.gpio.write(self.pin_a, GPIO.LOW)
            self.gpio.write(self.pin_b, GPIO.HIGH)
        else:
            self.gpio.write(self.pin_a, GPIO.LOW)
            self.gpio.write(self.pin_b, GPIO.LOW)

        if self._pwm:
            self._pwm.ChangeDutyCycle(abs(speed) * 100)

    def stop(self):
        self.set_speed(0)

    @property
    def speed(self) -> float:
        return self._speed


class ServoDriver:
    """PWM servo driver — controls standard hobby servos.

    Usage:
        servo = ServoDriver(pin=18, gpio=gpio)
        servo.set_angle(90)    # center
        servo.set_angle(0)     # full left
        servo.set_angle(180)   # full right
    """

    def __init__(
        self,
        pin: int,
        gpio: Optional[GPIO] = None,
        min_pulse: float = 0.5,
        max_pulse: float = 2.5,
        min_angle: float = 0,
        max_angle: float = 180,
        frequency: float = 50,
    ):
        self.pin = pin
        self.gpio = gpio or GPIO(simulation=True)
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.frequency = frequency
        self._angle = 90.0
        self._pwm = None

        self.gpio.setup(pin, GPIO.OUTPUT)
        self._pwm = self.gpio.pwm_start(pin, frequency, 0)

    def set_angle(self, angle: float):
        angle = max(self.min_angle, min(self.max_angle, angle))
        self._angle = angle

        pulse = self.min_pulse + (angle - self.min_angle) / (
            self.max_angle - self.min_angle
        ) * (self.max_pulse - self.min_pulse)
        duty = pulse / (1000.0 / self.frequency) * 100

        if self._pwm:
            self._pwm.ChangeDutyCycle(duty)

    @property
    def angle(self) -> float:
        return self._angle
