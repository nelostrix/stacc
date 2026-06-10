from __future__ import annotations

import math
from typing import Tuple

from stacc.gait.base import GaitGenerator


class BoundGait(GaitGenerator):
    """Bound gait — both front legs then both rear legs.

    Symmetrical left-right, alternating front-back pairs.
    Medium-speed gait for quadruped robots (e.g. dogs, cheetahs).

    Args:
        step_height: Foot lift height (default 0.04)
        step_length: Forward step length (default 0.10)
        frequency: Step frequency in Hz (default 2.5)
        body_height: Body height above ground (default 0.15)
    """

    def __init__(
        self,
        step_height: float = 0.04,
        step_length: float = 0.10,
        frequency: float = 2.5,
        body_height: float = 0.15,
    ):
        super().__init__(
            num_legs=4,
            step_height=step_height,
            step_length=step_length,
            frequency=frequency,
            body_height=body_height,
        )
        self._phase_offsets = [0.0, 0.0, 0.5, 0.5]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.12,  0.08),  # front-left
            (0.12, -0.08),  # front-right
            (-0.12,  0.08),  # back-left
            (-0.12, -0.08),  # back-right
        ]
        return positions[leg_id] if leg_id < 4 else (0.0, 0.0)
