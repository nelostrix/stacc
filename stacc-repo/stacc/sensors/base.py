"""Sensor base class and common sensor implementations.

All sensors in stacc inherit from BaseSensor and provide a uniform
interface for reading data, configuring noise models, and calibration.
"""
from __future__ import annotations
from typing import Optional, List, Dict, Any
import math
import random


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


class IMU(BaseSensor):
    """Inertial Measurement Unit — accelerometer + gyroscope + magnetometer.

    Args:
        rate: Sampling rate (default 100Hz, flight controllers use 400Hz)
        accel_noise: Accelerometer noise (m/s², default 0.01)
        gyro_noise: Gyroscope noise (rad/s, default 0.001)
        mag_noise: Magnetometer noise (μT, default 0.1)
    """

    def __init__(
        self,
        rate: float = 100.0,
        accel_noise: float = 0.01,
        gyro_noise: float = 0.001,
        mag_noise: float = 0.1,
        **kwargs,
    ):
        super().__init__(name="imu", rate=rate, **kwargs)
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.mag_noise = mag_noise

        # Current readings
        self.acceleration = [0.0, 0.0, 9.81]  # m/s² (gravity)
        self.angular_velocity = [0.0, 0.0, 0.0]  # rad/s
        self.magnetic_field = [20.0, 0.0, 40.0]  # μT

        # Fused orientation (Euler angles)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def read(self) -> Dict[str, List[float]]:
        """Read all IMU axes."""
        return {
            "acceleration": self.add_noise_vector(self.acceleration),
            "angular_velocity": [
                self.acceleration[i] + random.gauss(0, self.gyro_noise)
                for i in range(3)
            ],
            "magnetic_field": [
                self.magnetic_field[i] + random.gauss(0, self.mag_noise)
                for i in range(3)
            ],
            "orientation": [self.roll, self.pitch, self.yaw],
        }

    def read_acceleration(self) -> List[float]:
        return [a + random.gauss(0, self.accel_noise) for a in self.acceleration]

    def read_gyroscope(self) -> List[float]:
        return [g + random.gauss(0, self.gyro_noise) for g in self.angular_velocity]


class IRArray(BaseSensor):
    """Infrared sensor array for line following.

    Args:
        count: Number of IR sensors in the array
        spacing: Distance between sensors (meters)
    """

    def __init__(self, count: int = 5, spacing: float = 0.01, **kwargs):
        super().__init__(name="ir_array", rate=200.0, **kwargs)
        self.count = count
        self.spacing = spacing
        self._values = [0.0] * count

    def read(self) -> List[float]:
        """Read raw IR values (0.0 = white/no line, 1.0 = black/line)."""
        return self.add_noise_vector(self._values)

    def line_position(self) -> float:
        """Compute line position as weighted average.

        Returns:
            -1.0 (far left) to 1.0 (far right), 0.0 = centered
        """
        values = self.read()
        total = sum(values)
        if total < 0.01:
            return 0.0

        positions = [
            (i - (self.count - 1) / 2) / ((self.count - 1) / 2)
            for i in range(self.count)
        ]
        weighted = sum(p * v for p, v in zip(positions, values))
        return weighted / total


class Encoder(BaseSensor):
    """Rotary encoder for measuring shaft position/velocity.

    Args:
        ticks_per_rev: Encoder resolution (CPR)
        gear_ratio: Gear ratio (output/input)
    """

    def __init__(self, ticks_per_rev: int = 1024, gear_ratio: float = 1.0, **kwargs):
        super().__init__(name="encoder", rate=1000.0, **kwargs)
        self.ticks_per_rev = ticks_per_rev
        self.gear_ratio = gear_ratio
        self._ticks: int = 0
        self._prev_ticks: int = 0

    @property
    def angle(self) -> float:
        """Current angle in radians."""
        return (self._ticks / self.ticks_per_rev) * 2 * math.pi / self.gear_ratio

    @property
    def velocity(self) -> float:
        """Angular velocity in rad/s (computed from tick difference)."""
        dticks = self._ticks - self._prev_ticks
        return (dticks / self.ticks_per_rev) * 2 * math.pi * self.rate / self.gear_ratio

    def read(self) -> Dict[str, float]:
        return {"angle": self.angle, "velocity": self.velocity, "ticks": self._ticks}
