from __future__ import annotations

from typing import Dict

from stacc.sensors.base import BaseSensor


class Ultrasonic(BaseSensor):
    """Ultrasonic range finder (e.g., HC-SR04).

    Single-ray distance measurement along the sensor's forward axis.

    Args:
        range_max: Maximum range in meters (must be > 0)
        range_min: Minimum range in meters (must be >= 0, < range_max)
        beam_angle: Beam spread angle in degrees (must be > 0)
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
        if range_max <= 0:
            raise ValueError(f"range_max must be > 0, got {range_max}")
        if range_min < 0 or range_min >= range_max:
            raise ValueError(
                f"range_min ({range_min}) must be >= 0 and < range_max ({range_max})"
            )
        if beam_angle <= 0:
            raise ValueError(f"beam_angle must be > 0, got {beam_angle}")

        self.range_max = range_max
        self.range_min = range_min
        self.beam_angle = beam_angle
        self._distance = range_max

    def reset(self):
        """Reset distance to range_max."""
        self._distance = self.range_max

    def read(self) -> Dict[str, float]:
        return {
            "distance": self.add_noise(self._distance),
            "valid": self.range_min <= self._distance <= self.range_max,
        }

    def get_distance(self) -> float:
        return self.add_noise(self._distance)

    def __repr__(self) -> str:
        return (
            f"Ultrasonic(name='{self.name}', range=[{self.range_min}, {self.range_max}]m, "
            f"beam_angle={self.beam_angle}°, rate={self.rate}Hz)"
        )
