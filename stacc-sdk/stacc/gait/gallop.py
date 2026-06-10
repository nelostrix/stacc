from __future__ import annotations

import math
from typing import Tuple

from stacc.gait.base import GaitGenerator


class GallopGait(GaitGenerator):
    """Gallop gait — fastest quadruped gait.

    Front pair lifts close together, then back pair lifts close together,
    with an aerial phase where all four feet are off the ground.

    Args:
        step_height: Foot lift height (default 0.05)
        step_length: Forward step length (default 0.15)
        frequency: Step frequency in Hz (default 3.0)
        body_height: Body height above ground (default 0.15)
    """

    def __init__(
        self,
        step_height: float = 0.05,
        step_length: float = 0.15,
        frequency: float = 3.0,
        body_height: float = 0.15,
    ):
        super().__init__(
            num_legs=4,
            step_height=step_height,
            step_length=step_length,
            frequency=frequency,
            body_height=body_height,
        )
        self._phase_offsets = [0.0, 0.1, 0.5, 0.6]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.15,  0.08),  # front-left
            (0.15, -0.08),  # front-right
            (-0.15,  0.08),  # back-left
            (-0.15, -0.08),  # back-right
        ]
        return positions[leg_id] if leg_id < 4 else (0.0, 0.0)
