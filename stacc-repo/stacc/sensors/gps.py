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

        self._position = [0.0, 0.0, 0.0]  # [x, y, z] in world frame

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
        """Convert local XY (meters) to lat/lon (degrees)."""
        # Approximate conversion: 1 degree lat ≈ 111,320m
        lat = self.origin_lat + (y / 111320.0)
        lon = self.origin_lon + (
            x / (111320.0 * math.cos(math.radians(self.origin_lat)))
        )
        return lat, lon


# ══════════════════════════════════════════
# ULTRASONIC SENSOR
# ══════════════════════════════════════════


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
