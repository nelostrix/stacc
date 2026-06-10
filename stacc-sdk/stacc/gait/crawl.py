from __future__ import annotations

import math
from typing import Tuple

from stacc.gait.base import GaitGenerator


class CrawlGait(GaitGenerator):
    """Crawl gait — very slow, maximum stability.

    Only one foot lifts at a time while the other three form a stable
    triangular support polygon. Each foot has a 25 % swing / 75 %
    stance ratio. Ideal for rough or uneven terrain.

    Args:
        step_height: Foot lift height (default 0.03)
        step_length: Forward step length (default 0.06)
        frequency: Step frequency in Hz (default 1.0)
        body_height: Body height above ground (default 0.15)
    """

    _SWING_FRACTION = 0.25

    def __init__(
        self,
        step_height: float = 0.03,
        step_length: float = 0.06,
        frequency: float = 1.0,
        body_height: float = 0.15,
    ):
        super().__init__(
            num_legs=4,
            step_height=step_height,
            step_length=step_length,
            frequency=frequency,
            body_height=body_height,
            swing_ratio=self._SWING_FRACTION,
        )
        self._phase_offsets = [0.0, 0.25, 0.5, 0.75]

    def _foot_trajectory(
        self, leg_id: int, phase: float
    ) -> Tuple[float, float, float]:
        default_x, default_y = self._default_foot_pos(leg_id)
        s = self._SWING_FRACTION

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
        positions = [
            (0.12,  0.08),  # front-left
            (0.12, -0.08),  # front-right
            (-0.12,  0.08),  # back-left
            (-0.12, -0.08),  # back-right
        ]
        return positions[leg_id] if leg_id < 4 else (0.0, 0.0)
