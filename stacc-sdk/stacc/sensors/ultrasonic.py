from __future__ import annotations

from typing import Dict

from stacc.sensors.base import BaseSensor


class Ultrasonic(BaseSensor):
    """Ultrasonic range finder (e.g., HC-SR04).

    Single-ray distance measurement along the sensor's forward axis.

    Args:
        range_max: Maximum range in meters (typical: 2-4m)
        range_min: Minimum range in meters (typical: 0.02m)
        beam_angle: Beam spread angle in degrees (typical: 15-30)
        rate: Measurement rate in Hz
    """

    def __init__(
        self,
        range_max: float = 4.0,
        range_min: float = 0.02,
        beam_angle: float = 15.0,
        rate: float = 40.0,
        noise_stddev: float = 0.02,
        **kwargs,
    ):
        super().__init__(
            name="ultrasonic", rate=rate, noise_stddev=noise_stddev, **kwargs
        )
        self.range_max = range_max
        self.range_min = range_min
        self.beam_angle = beam_angle
        self._distance = range_max

    def read(self) -> Dict[str, float]:
        return {
            "distance": self.add_noise(self._distance),
            "valid": self.range_min <= self._distance <= self.range_max,
        }

    def get_distance(self) -> float:
        return self.add_noise(self._distance)
