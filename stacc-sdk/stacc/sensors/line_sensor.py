"""STACC Sensors: Multi-Channel IR Reflectance Line Follower Array & Auto-Calibration."""

import numpy as np
from typing import Dict, List, Tuple


class LineSensorArray:
    """
    5 to 8 Channel Infrared (IR) Reflectance Line Sensor Array (e.g. QTR-8A / QTR-8RC).
    Includes automatic threshold calibration over black/white surfaces and normalized centroid calculation.
    """

    def __init__(self, num_channels: int = 5, spacing_mm: float = 10.0):
        self.num_channels = num_channels
        self.spacing_mm = spacing_mm
        self.min_values = np.zeros(num_channels, dtype=np.float64) + 1023.0
        self.max_values = np.zeros(num_channels, dtype=np.float64)
        self.is_calibrated = False

    def calibrate_step(self, raw_readings: np.ndarray) -> None:
        """Feed sensor readings while sweeping over line to find min/max reflectance."""
        if len(raw_readings) != self.num_channels:
            raise ValueError(f"Expected {self.num_channels} sensor values")

        self.min_values = np.minimum(self.min_values, raw_readings)
        self.max_values = np.maximum(self.max_values, raw_readings)
        self.is_calibrated = True

    def read_normalized(self, raw_readings: np.ndarray) -> np.ndarray:
        """Normalize raw readings to [0.0 (white/surface), 1.0 (black line)]."""
        if not self.is_calibrated:
            # Fallback range [0, 1023]
            return np.clip(raw_readings / 1023.0, 0.0, 1.0)

        denom = np.maximum(1e-3, self.max_values - self.min_values)
        normalized = (raw_readings - self.min_values) / denom
        return np.clip(normalized, 0.0, 1.0)

    def compute_line_position(self, raw_readings: np.ndarray) -> Tuple[float, bool]:
        """
        Compute line position in range [-1.0 (far left) .. +1.0 (far right)].
        Returns (position, line_detected).
        """
        norm = self.read_normalized(raw_readings)
        total_weight = np.sum(norm)

        if total_weight < 0.15:
            # No line detected (surface is uniform white)
            return 0.0, False

        # Channel coordinates from -1.0 to +1.0
        coords = np.linspace(-1.0, 1.0, self.num_channels)
        position = np.sum(coords * norm) / total_weight

        return float(position), True

    def compute_pid_steering(
        self,
        line_position: float,
        last_error: float,
        integral_error: float,
        kp: float = 1.2,
        ki: float = 0.0,
        kd: float = 0.35,
        dt: float = 0.01,
    ) -> Tuple[float, float, float]:
        """
        Compute differential steering adjustment (turn rate) using PID control.
        """
        error = line_position
        integral = integral_error + error * dt
        derivative = (error - last_error) / dt if dt > 0 else 0.0

        steering_output = kp * error + ki * integral + kd * derivative
        return float(steering_output), error, float(integral)
