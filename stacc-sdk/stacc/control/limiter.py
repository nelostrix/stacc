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
    """S-curve velocity profile generator.

    Generates a smooth trajectory from current position to target with
    bounded velocity, acceleration, and jerk. Produces a continuous
    acceleration profile (trapezoidal acceleration, not just trapezoidal velocity).

    Args:
        max_velocity: Maximum velocity.
        max_acceleration: Maximum acceleration.
        max_jerk: Maximum jerk (rate of change of acceleration).
    """

    def __init__(
        self,
        max_velocity: float = 1.0,
        max_acceleration: float = 1.0,
        max_jerk: float = 10.0,
    ):
        self.max_vel = max_velocity
        self.max_accel = max_acceleration
        self.max_jerk = max_jerk
        self._pos: float = 0.0
        self._vel: float = 0.0
        self._accel: float = 0.0
        self._target: Optional[float] = None
        self._t_accel: float = max_acceleration / max_jerk if max_jerk > 0 else 0
        self._t_const: float = 0.0
        self._t_decel: float = self._t_accel
        self._total_time: float = 0.0
        self._phase: str = "idle"

    def set_target(self, target: float):
        """Set a new target position and compute the profile.

        Args:
            target: Desired position.
        """
        self._target = target
        distance = target - self._pos

        if abs(distance) < 1e-10:
            self._phase = "idle"
            return

        sign = 1 if distance > 0 else -1
        d = abs(distance)

        # Time to reach max velocity with S-curve acceleration
        t_ramp = self._t_accel * 2  # ramp up + ramp down of acceleration
        v_max_reachable = self.max_accel * self._t_accel
        dist_during_ramp = v_max_reachable * t_ramp

        if d <= dist_during_ramp:
            # Triangle profile (never reaches full velocity)
            # Solve for peak velocity
            peak_v = math.sqrt(d * self.max_jerk * self._t_accel)
            t_ramp = peak_v / self.max_accel if self.max_accel > 0 else 0
            self._t_accel = t_ramp
            self._t_const = 0.0
            self._t_decel = t_ramp
        else:
            # Trapezoidal profile
            const_dist = d - dist_during_ramp
            self._t_const = const_dist / v_max_reachable if v_max_reachable > 0 else 0

        self._total_time = self._t_accel + self._t_const + self._t_decel
        self._phase = "accelerating"
        self._elapsed = 0.0
        self._sign = sign

    def compute(self, dt: float) -> float:
        """Step the profile forward by dt seconds.

        Args:
            dt: Time step.

        Returns:
            Current position after stepping.
        """
        if self._phase == "idle" or self._target is None:
            return self._pos

        self._elapsed += dt
        if self._elapsed >= self._total_time:
            self._pos = self._target
            self._vel = 0.0
            self._accel = 0.0
            self._phase = "idle"
            return self._pos

        t = self._elapsed
        t1 = self._t_accel
        t2 = t1 + self._t_const
        t3 = self._total_time
        s = self._sign

        if t <= t1:
            # Accelerating phase: jerk-limited increase
            j = self.max_jerk * s
            self._accel = j * t
            self._vel = 0.5 * j * t * t
            self._pos += self._vel * dt
        elif t <= t2:
            # Constant velocity phase
            self._accel = self.max_accel * s
            self._vel = self.max_accel * self._t_accel * s
            self._pos += self._vel * dt
        else:
            # Decelerating phase: jerk-limited decrease
            t_dec = t - t2
            j = -self.max_jerk * s
            self._accel = self.max_accel * s + j * t_dec
            self._vel = (self.max_accel * self._t_accel - 0.5 * j * t_dec * t_dec) * s
            self._pos += self._vel * dt

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
        self._pos = position
        self._vel = 0.0
        self._accel = 0.0
        self._target = None
        self._phase = "idle"
        self._elapsed = 0.0
