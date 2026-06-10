from __future__ import annotations

import math
import random
from typing import Any, Dict, Tuple

from stacc.sensors.base import BaseSensor


class GPS(BaseSensor):
    """GPS/GNSS sensor for global positioning.

    In simulation, converts the robot's world-frame position to
    latitude/longitude/altitude using a configurable origin point.

    Args:
        origin_lat: Latitude of the simulation origin (degrees)
        origin_lon: Longitude of the simulation origin (degrees)
        origin_alt: Altitude of the simulation origin (meters)
        rate: Update rate in Hz
        noise_stddev: Position noise in meters (typical GPS: 2-5m)
    """

    def __init__(
        self,
        origin_lat: float = 0.0,
        origin_lon: float = 0.0,
        origin_alt: float = 0.0,
        rate: float = 10.0,
        noise_stddev: float = 2.0,
        **kwargs,
    ):
        super().__init__(name="gps", rate=rate, noise_stddev=noise_stddev, **kwargs)
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt = origin_alt
        self._position = [0.0, 0.0, 0.0]

    def reset(self):
        """Reset GPS position to origin."""
        self._position = [0.0, 0.0, 0.0]

    def read(self) -> Dict[str, Any]:
        noisy_position = self.add_noise_vector(self._position)
        lat, lon = self._xy_to_latlon(noisy_position[0], noisy_position[1])
        alt = self.origin_alt + noisy_position[2]

        return {
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "position_local": noisy_position,
            "hdop": 1.0 + random.random() * 0.5,
        }

    def _xy_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        lat = self.origin_lat + (y / 111320.0)
        lon = self.origin_lon + (
            x / (111320.0 * math.cos(math.radians(self.origin_lat)))
        )
        return lat, lon

    def __repr__(self) -> str:
        return (
            f"GPS(name='{self.name}', origin=({self.origin_lat}, {self.origin_lon}, {self.origin_alt}), "
            f"rate={self.rate}Hz)"
        )
