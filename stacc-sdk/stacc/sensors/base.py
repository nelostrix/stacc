"""Sensor base class and common sensor implementations.

All sensors in stacc inherit from BaseSensor and provide a uniform
interface for reading data, configuring noise models, and calibration.
"""

from __future__ import annotations

import math
import random
from typing import Any, Dict, List, Optional


class BaseSensor:
    """Base class for all stacc sensors.

    Args:
        name: Sensor identifier
        rate: Sampling rate in Hz
        noise_stddev: Gaussian noise standard deviation (0 = perfect)
        enabled: Whether the sensor is active
    """

    def __init__(
        self,
        name: str = "sensor",
        rate: float = 100.0,
        noise_stddev: float = 0.0,
        enabled: bool = True,
    ):
        self.name = name
        self.rate = rate
        self.noise_stddev = noise_stddev
        self.enabled = enabled

        self._mount_position: str = "body"
        self._mount_kwargs: Dict[str, Any] = {}
        self._last_reading: Any = None
        self._last_time: float = 0.0

    def read(self) -> Any:
        """Read the current sensor value. Override in subclasses."""
        raise NotImplementedError

    def add_noise(self, value: float) -> float:
        """Add Gaussian noise to a reading."""
        if self.noise_stddev > 0:
            return value + random.gauss(0, self.noise_stddev)
        return value

    def add_noise_vector(self, values: List[float]) -> List[float]:
        """Add noise to a vector of readings."""
        return [self.add_noise(v) for v in values]

    @property
    def period(self) -> float:
        """Sampling period in seconds."""
        return 1.0 / self.rate if self.rate > 0 else float("inf")

    def should_update(self, current_time: float) -> bool:
        """Check if enough time has passed for a new reading."""
        if current_time - self._last_time >= self.period:
            self._last_time = current_time
            return True
        return False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "type": self.__class__.__name__,
            "rate": self.rate,
            "mount": self._mount_position,
        }

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name='{self.name}', rate={self.rate}Hz)"
