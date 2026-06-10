from __future__ import annotations

import math
from typing import Tuple

from stacc.gait.base import GaitGenerator


class TrotGait(GaitGenerator):
    """Trot gait — diagonal pairs move together.

    The most common quadruped gait. Front-left + back-right swing
    together, then front-right + back-left. Stable at medium speeds.

    Leg ordering (body frame):
      0 = front-left,  1 = front-right,
      2 = back-left,   3 = back-right

    Args:
        step_height: Foot lift height (default 0.04)
        step_length: Forward step length (default 0.08)
        frequency: Step frequency in Hz (default 2.0)
        body_height: Body height above ground (default 0.15)
    """

    def __init__(
        self,
        step_height: float = 0.04,
        step_length: float = 0.08,
        frequency: float = 2.0,
        body_height: float = 0.15,
    ):
        super().__init__(
            num_legs=4,
            step_height=step_height,
            step_length=step_length,
            frequency=frequency,
            body_height=body_height,
        )
        self._phase_offsets = [0.0, 0.5, 0.5, 0.0]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.12,  0.08),  # front-left  (FL)
            (0.12, -0.08),  # front-right (FR)
            (-0.12,  0.08),  # back-left   (BL)
            (-0.12, -0.08),  # back-right  (BR)
        ]
        return positions[leg_id] if leg_id < 4 else (0.0, 0.0)
