from __future__ import annotations

import math
from typing import Optional


class RateLimiter:
    """Limits the rate of change of a signal.

    Ensures the output does not change faster than max_rate per second.
    Useful for smoothing step inputs to prevent actuator jerk.

    Args:
        max_rate: Maximum change per second (same units as the signal).
    """

    def __init__(self, max_rate: float):
        self.max_rate = max_rate
        self.prev = 0.0

    def compute(self, value: float, dt: float) -> float:
        delta = value - self.prev
        max_delta = self.max_rate * dt
        delta = max(-max_delta, min(max_delta, delta))
        self.prev += delta
        return self.prev

    def reset(self, value: float = 0.0):
        self.prev = value


class AccelerationLimiter:
    """Limits both velocity and acceleration of a signal.

    Two-stage: first clamp velocity to v_max, then clamp acceleration to a_max.
    Useful for generating smooth trajectories from step inputs.

    Args:
        max_velocity: Maximum absolute velocity.
        max_acceleration: Maximum absolute acceleration.
    """

    def __init__(self, max_velocity: float, max_acceleration: float):
        self.max_vel = max_velocity
        self.max_accel = max_acceleration
        self._prev_output = 0.0
        self._prev_velocity = 0.0

    def compute(self, target: float, dt: float) -> float:
        """Compute limited output.

        Args:
            target: Desired position/value.
            dt: Time step in seconds.

        Returns:
            Smoothly limited output value.
        """
        if dt <= 0:
            return self._prev_output

        # Velocity limit
        max_step = self.max_vel * dt
        desired = target
        if desired - self._prev_output > max_step:
            desired = self._prev_output + max_step
        elif desired - self._prev_output < -max_step:
            desired = self._prev_output - max_step

        # Acceleration limit
        velocity = (desired - self._prev_output) / dt
        max_accel_step = self.max_accel * dt
        if velocity - self._prev_velocity > max_accel_step:
            velocity = self._prev_velocity + max_accel_step
        elif velocity - self._prev_velocity < -max_accel_step:
            velocity = self._prev_velocity - max_accel_step

        output = self._prev_output + velocity * dt
        self._prev_output = output
        self._prev_velocity = velocity
        return output

    def reset(self, value: float = 0.0):
        self._prev_output = value
        self._prev_velocity = 0.0


class SCurveProfile:
    """Analytic seven-segment, rest-to-rest jerk-limited position profile.

    Limits are finite positive magnitudes. Retargeting a moving profile is
    rejected: finish the current move or explicitly reset before planning a
    new rest-to-rest move. Sampling evaluates the analytic trajectory rather
    than accumulating integration error or snapping an overshoot to target.
    """

    def __init__(self, max_velocity: float = 1.0,
                 max_acceleration: float = 1.0, max_jerk: float = 10.0):
        for name, value in (("max_velocity", max_velocity),
                            ("max_acceleration", max_acceleration),
                            ("max_jerk", max_jerk)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and greater than zero")
        self.max_vel = float(max_velocity)
        self.max_accel = float(max_acceleration)
        self.max_jerk = float(max_jerk)
        self.reset()

    def set_target(self, target: float):
        if not math.isfinite(target):
            raise ValueError("target must be finite")
        if not self.is_idle:
            if target == self._target:
                return
            raise RuntimeError("Finish or reset the moving profile before retargeting")
        self._target = float(target)
        self._start = self._pos
        distance = abs(self._target - self._start)
        if distance == 0:
            return
        self._sign = 1.0 if self._target > self._start else -1.0
        jerk = self.max_jerk
        ramp = min(self.max_accel / jerk, math.sqrt(self.max_vel / jerk))
        plateau = max(0.0, self.max_vel / (jerk * ramp) - ramp)
        peak_velocity = jerk * ramp * (ramp + plateau)
        ramp_distance = peak_velocity * (2 * ramp + plateau)
        cruise = 0.0
        if distance >= ramp_distance:
            cruise = (distance - ramp_distance) / peak_velocity
        else:
            ramp = self.max_accel / jerk
            if distance < 2 * jerk * ramp ** 3:
                ramp = (distance / (2 * jerk)) ** (1.0 / 3.0)
                plateau = 0.0
            else:
                plateau = max(0.0, (math.sqrt(ramp ** 2 + 4 * distance / self.max_accel) - 3 * ramp) / 2)
        self._segments = list(zip(
            (ramp, plateau, ramp, cruise, ramp, plateau, ramp),
            (jerk, 0.0, -jerk, 0.0, -jerk, 0.0, jerk),
        ))
        self._total_time = sum(duration for duration, _ in self._segments)
        self._elapsed = 0.0
        self._phase = "moving"

    def compute(self, dt: float) -> float:
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        if self.is_idle or dt == 0:
            return self._pos
        self._elapsed = min(self._total_time, self._elapsed + dt)
        remaining = self._elapsed
        position = velocity = acceleration = 0.0
        for duration, jerk in self._segments:
            t = min(duration, remaining)
            position += velocity * t + acceleration * t * t / 2 + jerk * t ** 3 / 6
            velocity += acceleration * t + jerk * t * t / 2
            acceleration += jerk * t
            remaining = max(0.0, remaining - t)
            if remaining == 0:
                break
        self._pos = self._start + self._sign * position
        self._vel = self._sign * velocity
        self._accel = self._sign * acceleration
        if self._elapsed >= self._total_time:
            self._pos = self._target
            self._vel = self._accel = 0.0
            self._phase = "idle"
        return self._pos

    @property
    def position(self) -> float:
        return self._pos

    @property
    def velocity(self) -> float:
        return self._vel

    @property
    def acceleration(self) -> float:
        return self._accel

    @property
    def is_idle(self) -> bool:
        return self._phase == "idle"

    def reset(self, position: float = 0.0):
        if not math.isfinite(position):
            raise ValueError("position must be finite")
        self._pos = self._start = float(position)
        self._vel = self._accel = self._elapsed = self._total_time = 0.0
        self._target: Optional[float] = None
        self._segments = []
        self._phase = "idle"
