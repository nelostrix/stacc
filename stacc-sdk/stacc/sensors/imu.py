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
