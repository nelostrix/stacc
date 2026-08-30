"""STACC HAL: Low-Cost H-Bridge Dual Motor Drivers (L298N, TB6612FNG, DRV8833)."""

from typing import Dict, Tuple, Optional
import numpy as np


class HBridgeDriver:
    """
    Standard Dual DC Motor H-Bridge Driver (L298N, TB6612FNG, DRV8833).
    Supports PWM duty cycle [-1.0 (full reverse) .. +1.0 (full forward)] with deadband & braking.
    """

    def __init__(
        self,
        driver_type: str = "TB6612FNG",
        pin_in1: int = 12,
        pin_in2: int = 14,
        pin_pwm_a: int = 13,
        pin_in3: int = 27,
        pin_in4: int = 26,
        pin_pwm_b: int = 25,
        deadband: float = 0.05,
    ):
        self.driver_type = driver_type
        self.pins = {
            "motor_a": {"in1": pin_in1, "in2": pin_in2, "pwm": pin_pwm_a},
            "motor_b": {"in3": pin_in3, "in4": pin_in4, "pwm": pin_pwm_b},
        }
        self.deadband = deadband
        self.motor_a_speed = 0.0
        self.motor_b_speed = 0.0

    def set_speeds(self, speed_left: float, speed_right: float) -> Dict[str, Dict[str, int]]:
        """
        Set speed for Left (Motor A) and Right (Motor B) in [-1.0, 1.0].
        Returns digital pin states (HIGH/LOW) and PWM duties [0..255].
        """
        self.motor_a_speed = float(np.clip(speed_left, -1.0, 1.0))
        self.motor_b_speed = float(np.clip(speed_right, -1.0, 1.0))

        def _calc_pins(spd: float):
            if abs(spd) < self.deadband:
                return {"in1": 0, "in2": 0, "pwm": 0}  # Coast
            elif spd > 0:
                return {"in1": 1, "in2": 0, "pwm": int(abs(spd) * 255)}  # Forward
            else:
                return {"in1": 0, "in2": 1, "pwm": int(abs(spd) * 255)}  # Reverse

        return {
            "motor_left": _calc_pins(self.motor_a_speed),
            "motor_right": _calc_pins(self.motor_b_speed),
        }

    def emergency_brake(self) -> Dict[str, Dict[str, int]]:
        """Short-circuit dynamic brake."""
        self.motor_a_speed = 0.0
        self.motor_b_speed = 0.0
        return {
            "motor_left": {"in1": 1, "in2": 1, "pwm": 255},
            "motor_right": {"in1": 1, "in2": 1, "pwm": 255},
        }
