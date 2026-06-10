from __future__ import annotations

import math
from typing import Optional


class LowPassFilter:
    """Exponential moving average low-pass filter.

    Args:
        alpha: Smoothing factor (0.0 = no smoothing, 1.0 = no filtering).
            Typical values: 0.1-0.3 for heavy smoothing, 0.5-0.8 for light.
    """

    def __init__(self, alpha: float):
        self.alpha = alpha
        self.prev: Optional[float] = None

    def compute(self, value: float) -> float:
        if self.prev is None:
            self.prev = value
        self.prev = self.alpha * value + (1 - self.alpha) * self.prev
        return self.prev

    def reset(self):
        self.prev = None


class MovingAverageFilter:
    """Windowed moving average filter.

    Averages the last N samples. Good for smoothing noisy sensors
    without the lag characteristic of exponential filters.

    Args:
        window_size: Number of samples to average.
    """

    def __init__(self, window_size: int = 10):
        self.window_size = window_size
        self._buffer: list[float] = []

    def compute(self, value: float) -> float:
        self._buffer.append(value)
        if len(self._buffer) > self.window_size:
            self._buffer.pop(0)
        return sum(self._buffer) / len(self._buffer)

    def reset(self):
        self._buffer.clear()


class ComplementaryFilter:
    """Complementary filter — fuses low-frequency and high-frequency signals.

    The classic use is IMU attitude estimation: gyroscope (high-rate, drifty)
    + accelerometer (noisy, no drift). The filter blends them:

        output = alpha * (output + gyro * dt) + (1 - alpha) * accel_estimate

    Args:
        alpha: Gyroscope weight (0-1). Higher = trust gyro more.
            Typical: 0.95-0.99 for IMU attitude.
    """

    def __init__(self, alpha: float = 0.96):
        self.alpha = alpha
        self._filtered: Optional[float] = None

    def compute(self, gyro_rate: float, accel_estimate: float, dt: float) -> float:
        if self._filtered is None:
            self._filtered = accel_estimate
            return self._filtered

        # Integrate gyro, then blend with accel estimate
        gyro_integral = self._filtered + gyro_rate * dt
        self._filtered = self.alpha * gyro_integral + (1 - self.alpha) * accel_estimate
        return self._filtered

    def reset(self, value: Optional[float] = None):
        self._filtered = value


class KalmanFilter1D:
    """1D Kalman filter — optimal recursive estimation.

    Tracks a single state variable (e.g., position, temperature) with
    Gaussian noise on both the process model and measurements.

    Args:
        process_noise: Process noise variance (how much the state can change
            between steps). Higher = more responsive but noisier.
        measurement_noise: Measurement noise variance (sensor trust).
            Higher = smoother but laggier.
        initial_value: Starting state estimate.
        initial_covariance: Initial uncertainty.
    """

    def __init__(
        self,
        process_noise: float = 0.1,
        measurement_noise: float = 1.0,
        initial_value: float = 0.0,
        initial_covariance: float = 1.0,
    ):
        self.Q = process_noise
        self.R = measurement_noise
        self.x = initial_value
        self.P = initial_covariance

    def predict(self, dt: float = 1.0, control: float = 0.0):
        """Predict step — advance state estimate.

        Uses a simple velocity model: x = x + control * dt.
        Override this for different process models.

        Args:
            dt: Time step.
            control: Control input (e.g., velocity).
        """
        self.x += control * dt
        self.P += self.Q * dt

    def update(self, measurement: float) -> float:
        """Update step — incorporate a measurement.

        Args:
            measurement: Observed value.

        Returns:
            Updated state estimate.
        """
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P = (1 - K) * self.P
        return self.x

    def compute(self, measurement: float, dt: float = 1.0, control: float = 0.0) -> float:
        """Convenience: predict then update in one call.

        Args:
            measurement: Observed value.
            dt: Time step.
            control: Control input.

        Returns:
            Filtered state estimate.
        """
        self.predict(dt, control)
        return self.update(measurement)

    def reset(self, value: float = 0.0, covariance: float = 1.0):
        self.x = value
        self.P = covariance


class NotchFilter:
    """Notch (band-stop) filter — removes a specific frequency.

    Useful for removing mechanical resonance frequencies or power-line
    noise (50/60 Hz) from sensor signals.

    Uses a simple biquad implementation.

    Args:
        sample_rate: Sampling frequency in Hz.
        notch_freq: Frequency to remove in Hz.
        Q: Quality factor (higher = narrower notch). Typical: 1-30.
    """

    def __init__(self, sample_rate: float, notch_freq: float, Q: float = 10.0):
        self.sample_rate = sample_rate
        self.notch_freq = notch_freq
        self.Q = Q

        # Biquad coefficients
        w0 = 2 * math.pi * notch_freq / sample_rate
        alpha = math.sin(w0) / (2 * Q)

        b0 = 1.0
        b1 = -2.0 * math.cos(w0)
        b2 = 1.0
        a0 = 1.0 + alpha
        a1 = -2.0 * math.cos(w0)
        a2 = 1.0 - alpha

        self._b0 = b0 / a0
        self._b1 = b1 / a0
        self._b2 = b2 / a0
        self._a1 = a1 / a0
        self._a2 = a2 / a0

        self._x1 = 0.0
        self._x2 = 0.0
        self._y1 = 0.0
        self._y2 = 0.0

    def compute(self, value: float) -> float:
        y = (
            self._b0 * value
            + self._b1 * self._x1
            + self._b2 * self._x2
            - self._a1 * self._y1
            - self._a2 * self._y2
        )
        self._x2, self._x1 = self._x1, value
        self._y2, self._y1 = self._y1, y
        return y

    def reset(self):
        self._x1 = self._x2 = self._y1 = self._y2 = 0.0
