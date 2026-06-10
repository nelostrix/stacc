from __future__ import annotations

from typing import Dict

from stacc.sensors.base import BaseSensor


class Barometer(BaseSensor):
    """Barometric pressure sensor for altitude estimation.

    Converts altitude to pressure using the barometric formula.

    Args:
        rate: Sampling rate in Hz
        noise_stddev: Pressure noise in Pa (must be >= 0)
    """

    def __init__(self, rate: float = 50.0, noise_stddev: float = 10.0, **kwargs):
        super().__init__(
            name="barometer", rate=rate, noise_stddev=noise_stddev, **kwargs
        )
        self._sea_level_pressure = 101325.0
        self._altitude = 0.0

    def reset(self):
        """Reset altitude to zero."""
        self._altitude = 0.0

    def read(self) -> Dict[str, float]:
        pressure = self._altitude_to_pressure(self._altitude)
        return {
            "pressure": self.add_noise(pressure),
            "altitude_estimate": self._pressure_to_altitude(self.add_noise(pressure)),
            "temperature": 15.0 - 0.0065 * self._altitude,
        }

    def _altitude_to_pressure(self, alt: float) -> float:
        return self._sea_level_pressure * (1 - 2.25577e-5 * alt) ** 5.25588

    def _pressure_to_altitude(self, pressure: float) -> float:
        if pressure <= 0:
            return 0.0
        return (1 - (pressure / self._sea_level_pressure) ** (1 / 5.25588)) / 2.25577e-5

    def __repr__(self) -> str:
        return f"Barometer(name='{self.name}', rate={self.rate}Hz)"
