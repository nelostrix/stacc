from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from stacc.gait.base import GaitGenerator


class TripodGait(GaitGenerator):
    """Tripod gait — for hexapod robots (6 legs).

    Two groups of 3 legs alternate. Always 3 feet on ground = very stable.
    """

    def __init__(self, **kwargs):
        super().__init__(num_legs=6, **kwargs)
        self._phase_offsets = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.10, 0.12),
            (0.10, -0.12),  # front
            (0.00, 0.14),
            (0.00, -0.14),  # middle
            (-0.10, 0.12),
            (-0.10, -0.12),  # rear
        ]
        return positions[leg_id] if leg_id < 6 else (0, 0)
