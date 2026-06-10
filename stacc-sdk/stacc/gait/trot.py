from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from stacc.gait.base import GaitGenerator


class TrotGait(GaitGenerator):
    """Trot gait — diagonal pairs move together.

    The most common quadruped gait. Front-left + back-right move
    together, then front-right + back-left. Stable at medium speeds.
    """

    def __init__(self, **kwargs):
        super().__init__(num_legs=4, **kwargs)
        # Diagonal pairs have opposite phase
        self._phase_offsets = [0.0, 0.5, 0.5, 0.0]  # FL, FR, BL, BR

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.12, 0.08),  # front-left
            (0.12, -0.08),  # front-right
            (-0.12, 0.08),  # back-left
            (-0.12, -0.08),  # back-right
        ]
        return positions[leg_id] if leg_id < 4 else (0, 0)
