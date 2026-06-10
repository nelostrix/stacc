from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from stacc.gait.base import GaitGenerator


class WalkGait(GaitGenerator):
    """Walk gait — one foot at a time, 3 always on ground.

    Slowest but most stable quadruped gait. Each foot lifts
    independently in sequence.
    """

    def __init__(self, **kwargs):
        super().__init__(num_legs=4, **kwargs)
        self._phase_offsets = [0.0, 0.25, 0.5, 0.75]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.12, 0.08),
            (0.12, -0.08),
            (-0.12, 0.08),
            (-0.12, -0.08),
        ]
        return positions[leg_id] if leg_id < 4 else (0, 0)
