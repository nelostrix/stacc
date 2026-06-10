from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple


class GaitGenerator:
    """Base class for all gait generators.

    Generates foot placement trajectories for legged robots.
    Supports configurable swing/stance ratio, direction turning,
    and both circular and rectangular foot distributions.

    Args:
        num_legs: Number of legs
        step_height: Maximum foot lift height in meters
        step_length: Forward step length in meters
        frequency: Step frequency in Hz
        body_height: Body height above ground in meters
        swing_ratio: Fraction of cycle spent in swing (0.0-1.0, default 0.5)
    """

    def __init__(
        self,
        num_legs: int,
        step_height: float = 0.04,
        step_length: float = 0.08,
        frequency: float = 2.0,
        body_height: float = 0.15,
        swing_ratio: float = 0.5,
    ):
        if num_legs < 1:
            raise ValueError(f"num_legs must be >= 1, got {num_legs}")
        if not 0 < swing_ratio < 1:
            raise ValueError(f"swing_ratio must be in (0, 1), got {swing_ratio}")

        self.num_legs = num_legs
        self.step_height = step_height
        self.step_length = step_length
        self.frequency = frequency
        self.body_height = body_height
        self.swing_ratio = swing_ratio
        self._phase_offsets: List[float] = [0.0] * num_legs
        self._heading: float = 0.0

    def compute(self, time: float) -> List[List[float]]:
        """Compute foot target positions at given time.

        Args:
            time: Current simulation time in seconds

        Returns:
            List of [x, y, z] foot positions, one per leg,
            relative to the body frame.
        """
        targets: List[List[float]] = []
        period = 1.0 / self.frequency if self.frequency > 0 else 1.0

        for leg_id in range(self.num_legs):
            phase = (time / period + self._phase_offsets[leg_id]) % 1.0
            x, y, z = self._foot_trajectory(leg_id, phase)
            # Apply heading rotation
            if self._heading != 0.0:
                cos_h = math.cos(self._heading)
                sin_h = math.sin(self._heading)
                rx = x * cos_h - y * sin_h
                ry = x * sin_h + y * cos_h
                x, y = rx, ry
            targets.append([x, y, z])

        return targets

    def _foot_trajectory(
        self, leg_id: int, phase: float
    ) -> Tuple[float, float, float]:
        """Compute single foot position from phase (0-1).

        Phase 0.0-swing_ratio = swing (foot in air)
        Phase swing_ratio-1.0 = stance (foot on ground)

        Override in subclasses for custom foot paths.
        """
        default_x, default_y = self._default_foot_pos(leg_id)
        s = self.swing_ratio

        if phase < s:
            swing_t = phase / s
            x = default_x + self.step_length * (swing_t - 0.5)
            z = -self.body_height + self.step_height * math.sin(swing_t * math.pi)
        else:
            stance_t = (phase - s) / (1.0 - s)
            x = default_x + self.step_length * (0.5 - stance_t)
            z = -self.body_height

        return x, default_y, z

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        """Default foot X, Y position for each leg in body frame.

        Distributes legs evenly in a circle. Override in subclasses
        for specific robot geometries (e.g. rectangular quadruped).
        """
        if self.num_legs == 0:
            return (0.0, 0.0)
        angle = (2.0 * math.pi * leg_id / self.num_legs) + math.pi / self.num_legs
        radius = 0.15
        return radius * math.cos(angle), radius * math.sin(angle)

    def set_speed(self, speed: float) -> None:
        """Adjust gait speed. Scales frequency and step length."""
        self.frequency = max(0.5, speed * 3.0)
        self.step_length = speed * 0.15

    def set_direction(self, heading: float) -> None:
        """Set walking direction in radians (0 = forward)."""
        self._heading = heading

    def reset(self) -> None:
        """Reset gait internal state. Useful after a fall or gait switch."""
        self._phase_offsets = [0.0] * self.num_legs
        self._heading = 0.0
