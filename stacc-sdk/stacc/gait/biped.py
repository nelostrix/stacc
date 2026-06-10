from __future__ import annotations

import math
from typing import Tuple

from stacc.gait.base import GaitGenerator


class BipedWalkGait(GaitGenerator):
    """Biped walking gait — human-like walking.

    Two legs alternate with hip sway for lateral balance.
    Foot lifts with an elliptical trajectory during swing.

    Args:
        step_height: Foot lift height (default 0.03)
        step_length: Forward step length (default 0.06)
        frequency: Step frequency in Hz (default 1.5)
        body_height: Body height above ground (default 0.15)
        hip_width: Lateral distance from center to each hip (default 0.08)
    """

    def __init__(
        self,
        step_height: float = 0.03,
        step_length: float = 0.06,
        frequency: float = 1.5,
        body_height: float = 0.15,
        hip_width: float = 0.08,
    ):
        super().__init__(
            num_legs=2,
            step_height=step_height,
            step_length=step_length,
            frequency=frequency,
            body_height=body_height,
        )
        self.hip_width = hip_width
        self._phase_offsets = [0.0, 0.5]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        return (0.0, self.hip_width) if leg_id == 0 else (0.0, -self.hip_width)

    def _foot_trajectory(
        self, leg_id: int, phase: float
    ) -> Tuple[float, float, float]:
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
