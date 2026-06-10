from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from stacc.gait.base import GaitGenerator


class CrawlGait(GaitGenerator):
    """Crawl gait — very slow, maximum stability.

    Only one foot lifts at a time, other three form a stable triangle.
    Used for rough terrain.
    """

    def __init__(self, **kwargs):
        kwargs.setdefault("frequency", 1.0)
        kwargs.setdefault("step_height", 0.03)
        super().__init__(num_legs=4, **kwargs)
        self._phase_offsets = [0.0, 0.25, 0.5, 0.75]

    def _foot_trajectory(self, leg_id: int, phase: float) -> Tuple[float, float, float]:
        default_x, default_y = self._default_foot_pos(leg_id)
        # Only 25% of cycle is swing (75% stance)
        swing_fraction = 0.25

        if phase < swing_fraction:
            swing_phase = phase / swing_fraction
            x = default_x + self.step_length * (swing_phase - 0.5)
            z = -self.body_height + self.step_height * math.sin(swing_phase * math.pi)
        else:
            stance_phase = (phase - swing_fraction) / (1.0 - swing_fraction)
            x = default_x + self.step_length * (0.5 - stance_phase)
            z = -self.body_height

        return x, default_y, z

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.12, 0.08),
            (0.12, -0.08),
            (-0.12, 0.08),
            (-0.12, -0.08),
        ]
        return positions[leg_id] if leg_id < 4 else (0, 0)
