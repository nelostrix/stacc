from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple


class GaitGenerator:
    """Base class for all gait generators."""

    def __init__(
        self,
        num_legs: int,
        step_height: float = 0.04,
        step_length: float = 0.08,
        frequency: float = 2.0,
        body_height: float = 0.15,
    ):
        self.num_legs = num_legs
        self.step_height = step_height
        self.step_length = step_length
        self.frequency = frequency
        self.body_height = body_height
        self._phase_offsets: List[float] = [0.0] * num_legs

    def compute(self, time: float) -> List[List[float]]:
        """Compute foot target positions at given time.

        Returns:
            List of [x, y, z] foot positions, one per leg.
            Positions are relative to the body frame.
        """
        targets = []
        period = 1.0 / self.frequency if self.frequency > 0 else 1.0

        for leg_id in range(self.num_legs):
            phase = (time * self.frequency + self._phase_offsets[leg_id]) % 1.0
            x, y, z = self._foot_trajectory(leg_id, phase)
            targets.append([x, y, z])

        return targets

    def _foot_trajectory(self, leg_id: int, phase: float) -> Tuple[float, float, float]:
        """Compute single foot position from phase (0-1).

        Phase 0.0-0.5 = swing (foot in air)
        Phase 0.5-1.0 = stance (foot on ground)
        """
        default_x, default_y = self._default_foot_pos(leg_id)

        if phase < 0.5:
            # Swing phase: foot moves forward in the air
            swing_phase = phase / 0.5  # normalize to 0-1
            x = default_x + self.step_length * (swing_phase - 0.5)
            z = -self.body_height + self.step_height * math.sin(swing_phase * math.pi)
            y = default_y
        else:
            # Stance phase: foot moves backward on ground
            stance_phase = (phase - 0.5) / 0.5  # normalize to 0-1
            x = default_x + self.step_length * (0.5 - stance_phase)
            z = -self.body_height
            y = default_y

        return x, y, z

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        """Default foot X, Y position for each leg (body frame)."""
        # Distribute legs evenly around body
        angle = (2 * math.pi * leg_id / self.num_legs) + math.pi / self.num_legs
        radius = 0.15
        return radius * math.cos(angle), radius * math.sin(angle)

    def set_speed(self, speed: float):
        """Adjust gait speed (frequency scales with speed)."""
        self.frequency = max(0.5, speed * 3.0)
        self.step_length = speed * 0.15

    def set_direction(self, heading: float):
        """Set walking direction in radians (0 = forward)."""
        pass  # override in subclasses for turning
