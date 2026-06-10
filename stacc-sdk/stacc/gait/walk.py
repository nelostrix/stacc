from __future__ import annotations

import math
from typing import Tuple

from stacc.gait.base import GaitGenerator


class WalkGait(GaitGenerator):
    """Walk gait — one foot at a time, 3 always on ground.

    Slowest but most stable quadruped gait. Each foot lifts
    independently in sequence around the body.

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
        self._phase_offsets = [0.0, 0.25, 0.5, 0.75]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.12,  0.08),  # front-left
            (0.12, -0.08),  # front-right
            (-0.12,  0.08),  # back-left
            (-0.12, -0.08),  # back-right
        ]
        return positions[leg_id] if leg_id < 4 else (0.0, 0.0)
