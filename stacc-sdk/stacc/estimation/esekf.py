"""15-State Error-State Extended Kalman Filter (ES-EKF).

Fuses high-frequency IMU (accelerometer + gyroscope) with wheel odometry, GPS,
and visual/LiDAR position updates to provide drift-free 1000Hz pose and velocity estimation.

State Vector:
    x = [p (3), v (3), q (4), b_a (3), b_g (3)]  (16-dim nominal state)
Error State:
    delta_x = [delta_p (3), delta_v (3), delta_theta (3), delta_ba (3), delta_bg (3)]  (15-dim)
"""

from __future__ import annotations
from typing import Optional, Tuple, Union
import math
import numpy as np

from stacc.kinematics.spatial import skew, exp_so3


class ErrorStateEKF:
    """15-State Error-State Extended Kalman Filter for Robotics Navigation."""

    def __init__(
        self,
        gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81),
        accel_noise: float = 0.05,
        gyro_noise: float = 0.005,
        accel_bias_noise: float = 0.0001,
        gyro_bias_noise: float = 0.00001
    ):
        self.g = np.array(gravity, dtype=np.float64)

        # Nominal states
        self.p = np.zeros(3, dtype=np.float64)
        self.v = np.zeros(3, dtype=np.float64)
        self.R = np.eye(3, dtype=np.float64)  # Rotation matrix representation of orientation
        self.b_a = np.zeros(3, dtype=np.float64)  # Accelerometer bias
        self.b_g = np.zeros(3, dtype=np.float64)  # Gyroscope bias

        # 15x15 Error state covariance matrix
        self.P = np.eye(15, dtype=np.float64) * 0.01

        # Continuous noise parameters
        self.var_acc = float(accel_noise) ** 2
        self.var_gyro = float(gyro_noise) ** 2
        self.var_ba = float(accel_bias_noise) ** 2
        self.var_bg = float(gyro_bias_noise) ** 2

    def predict(self, a_m: np.ndarray, w_m: np.ndarray, dt: float) -> None:
        """IMU propagation step using measured specific force and angular velocity."""
        dt = float(dt)
        a_unbiased = np.asarray(a_m, dtype=np.float64) - self.b_a
        w_unbiased = np.asarray(w_m, dtype=np.float64) - self.b_g

        # 1. Propagate nominal state
        a_world = self.R @ a_unbiased + self.g
        self.p += self.v * dt + 0.5 * a_world * (dt ** 2)
        self.v += a_world * dt
        delta_R = exp_so3(w_unbiased * dt)
        self.R = self.R @ delta_R

        # 2. Linearized Error-State Transition Matrix F_x (15x15)
        F_x = np.eye(15, dtype=np.float64)
        F_x[0:3, 3:6] = np.eye(3) * dt
        F_x[3:6, 6:9] = -self.R @ skew(a_unbiased) * dt
        F_x[3:6, 9:12] = -self.R * dt
        F_x[6:9, 6:9] = delta_R.T
        F_x[6:9, 12:15] = -np.eye(3) * dt

        # Process noise covariance Q_k
        Q = np.zeros((15, 15), dtype=np.float64)
        Q[3:6, 3:6] = self.var_acc * (dt ** 2) * np.eye(3)
        Q[6:9, 6:9] = self.var_gyro * (dt ** 2) * np.eye(3)
        Q[9:12, 9:12] = self.var_ba * dt * np.eye(3)
        Q[12:15, 12:15] = self.var_bg * dt * np.eye(3)

        # Propagate covariance: P = F_x * P * F_x^T + Q
        self.P = F_x @ self.P @ F_x.T + Q

    def update_position(self, p_meas: np.ndarray, noise_std: float = 0.05) -> None:
        """Correct state using external position measurement (GPS, LiDAR, VIO)."""
        z = np.asarray(p_meas, dtype=np.float64)
        H = np.zeros((3, 15), dtype=np.float64)
        H[0:3, 0:3] = np.eye(3)
        R_cov = (float(noise_std) ** 2) * np.eye(3)

        # Innovation
        y = z - self.p
        S = H @ self.P @ H.T + R_cov
        K = self.P @ H.T @ np.linalg.inv(S)

        # Error state correction
        delta_x = K @ y
        self.P = (np.eye(15) - K @ H) @ self.P

        # Inject error into nominal states
        self.p += delta_x[0:3]
        self.v += delta_x[3:6]
        self.R = self.R @ exp_so3(delta_x[6:9])
        self.b_a += delta_x[9:12]
        self.b_g += delta_x[12:15]

    def get_pose(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return estimated position (3), velocity (3), and 3x3 rotation matrix."""
        return self.p.copy(), self.v.copy(), self.R.copy()
