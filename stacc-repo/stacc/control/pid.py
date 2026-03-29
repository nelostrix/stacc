"""PID controller — single-axis, cascaded, and adaptive variants.

The most fundamental control algorithm in robotics. Used for motor speed,
joint position, balance, line following, altitude hold, and more.
"""
from __future__ import annotations
from typing import Optional


class PIDController:
    """Proportional-Integral-Derivative controller.

    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        output_limits: (min, max) output clamp
        integral_limits: (min, max) anti-windup clamp for integral term
        setpoint: Target value (can also be set later)
        deadband: Ignore errors smaller than this
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limits: Optional[tuple] = None,
        integral_limits: Optional[tuple] = None,
        setpoint: float = 0.0,
        deadband: float = 0.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.deadband = deadband

        self.output_limits = output_limits or (-float("inf"), float("inf"))
        self.integral_limits = integral_limits or (-1000.0, 1000.0)

        self._integral: float = 0.0
        self._prev_error: Optional[float] = None
        self._prev_output: float = 0.0

    def compute(self, measurement_or_error: float, dt: float, is_error: bool = True) -> float:
        """Compute PID output.

        Args:
            measurement_or_error: Either the error directly, or the current
                measurement (if is_error=False, error = setpoint - measurement)
            dt: Time step in seconds
            is_error: If True, first arg is error. If False, error is computed
                from setpoint.

        Returns:
            Control output value
        """
        if dt <= 0:
            return self._prev_output

        if is_error:
            error = measurement_or_error
        else:
            error = self.setpoint - measurement_or_error

        # Apply deadband
        if abs(error) < self.deadband:
            error = 0.0

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(
            self.integral_limits[0],
            min(self.integral_limits[1], self._integral),
        )
        i = self.ki * self._integral

        # Derivative (on error, with filtering)
        if self._prev_error is not None:
            d = self.kd * (error - self._prev_error) / dt
        else:
            d = 0.0
        self._prev_error = error

        # Total output
        output = p + i + d

        # Clamp output
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        self._prev_output = output
        return output

    def reset(self):
        """Reset integral and derivative state."""
        self._integral = 0.0
        self._prev_error = None
        self._prev_output = 0.0

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update gains (useful for adaptive PID or gain scheduling)."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def __repr__(self) -> str:
        return f"PID(kp={self.kp}, ki={self.ki}, kd={self.kd})"


class CascadedPID:
    """Two-loop cascaded PID controller.

    Outer loop sets the setpoint for the inner loop.
    Common use: outer = position, inner = velocity.
    """

    def __init__(self, outer: PIDController, inner: PIDController):
        self.outer = outer
        self.inner = inner

    def compute(self, outer_measurement: float, inner_measurement: float, dt: float) -> float:
        """Compute cascaded output.

        Args:
            outer_measurement: Measurement for the outer loop (e.g., position)
            inner_measurement: Measurement for the inner loop (e.g., velocity)
            dt: Time step

        Returns:
            Control output from inner loop
        """
        inner_setpoint = self.outer.compute(outer_measurement, dt, is_error=False)
        self.inner.setpoint = inner_setpoint
        return self.inner.compute(inner_measurement, dt, is_error=False)

    def reset(self):
        self.outer.reset()
        self.inner.reset()
