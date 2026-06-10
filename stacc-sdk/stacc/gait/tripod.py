from __future__ import annotations

import math
from typing import Tuple

from stacc.gait.base import GaitGenerator


class TripodGait(GaitGenerator):
    """Tripod gait — for hexapod robots (6 legs).

    Two alternating groups of three legs. At any time three feet are
    on the ground forming a stable support triangle.

    Leg groups:
      Group A: leg 0 (front-left), leg 2 (middle-left), leg 4 (rear-left)
      Group B: leg 1 (front-right), leg 3 (middle-right), leg 5 (rear-right)

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
            num_legs=6,
            step_height=step_height,
            step_length=step_length,
            frequency=frequency,
            body_height=body_height,
        )
        self._phase_offsets = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.10,  0.12),  # front-left
            (0.10, -0.12),  # front-right
            (0.00,  0.14),  # middle-left
            (0.00, -0.14),  # middle-right
            (-0.10,  0.12),  # rear-left
            (-0.10, -0.12),  # rear-right
        ]
        return positions[leg_id] if leg_id < 6 else (0.0, 0.0)
