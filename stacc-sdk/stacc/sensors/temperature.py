from __future__ import annotations

from typing import Dict

from stacc.sensors.base import BaseSensor


class TemperatureSensor(BaseSensor):
    """Temperature sensor.

    Args:
        range_min: Minimum measurable temperature in °C
        range_max: Maximum measurable temperature in °C (must be > range_min)
        rate: Sampling rate in Hz
    """

    def __init__(
        self,
        range_min: float = -40.0,
        range_max: float = 125.0,
        rate: float = 1.0,
        noise_stddev: float = 0.5,
        **kwargs,
    ):
        super().__init__(
            name="temperature", rate=rate, noise_stddev=noise_stddev, **kwargs
        )
        if range_max <= range_min:
            raise ValueError(
                f"range_max ({range_max}) must be > range_min ({range_min})"
            )

        self.range_min = range_min
        self.range_max = range_max
        self._temperature = 25.0

    def reset(self):
        """Reset temperature to 25°C."""
        self._temperature = 25.0

    def read(self) -> Dict[str, float]:
        return {
            "temperature": self.add_noise(self._temperature),
            "unit": "celsius",
        }

    def __repr__(self) -> str:
        return (
            f"TemperatureSensor(name='{self.name}', range=[{self.range_min}, {self.range_max}]°C, "
            f"rate={self.rate}Hz)"
        )
