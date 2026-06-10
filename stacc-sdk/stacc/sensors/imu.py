from __future__ import annotations

import random
from typing import Dict, List

from stacc.sensors.base import BaseSensor


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
        if accel_noise < 0:
            raise ValueError(f"accel_noise must be >= 0, got {accel_noise}")
        if gyro_noise < 0:
            raise ValueError(f"gyro_noise must be >= 0, got {gyro_noise}")
        if mag_noise < 0:
            raise ValueError(f"mag_noise must be >= 0, got {mag_noise}")

        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.mag_noise = mag_noise

        self.reset()

    def reset(self):
        """Reset IMU readings to default values."""
        self.acceleration = [0.0, 0.0, 9.81]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.magnetic_field = [20.0, 0.0, 40.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def read(self) -> Dict[str, List[float]]:
        return {
            "acceleration": self.add_noise_vector(self.acceleration),
            "angular_velocity": self.add_noise_vector(self.angular_velocity),
            "magnetic_field": self.add_noise_vector(self.magnetic_field),
            "orientation": [self.roll, self.pitch, self.yaw],
        }

    def read_acceleration(self) -> List[float]:
        return [a + random.gauss(0, self.accel_noise) for a in self.acceleration]

    def read_gyroscope(self) -> List[float]:
        return [g + random.gauss(0, self.gyro_noise) for g in self.angular_velocity]

    def __repr__(self) -> str:
        return (
            f"IMU(name='{self.name}', rate={self.rate}Hz, "
            f"accel_noise={self.accel_noise}, gyro_noise={self.gyro_noise})"
        )
