from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from stacc.gait.base import GaitGenerator


class GallopGait(GaitGenerator):
    """Gallop gait — fastest quadruped gait.

    Front pair lifts together, then back pair. Unstable but fast.
    There's an aerial phase where all feet are off the ground.
    """

    def __init__(self, **kwargs):
        kwargs.setdefault("frequency", 3.0)
        kwargs.setdefault("step_length", 0.15)
        super().__init__(num_legs=4, **kwargs)
        self._phase_offsets = [0.0, 0.1, 0.5, 0.6]  # front pair close, back pair close

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.15, 0.08),
            (0.15, -0.08),
            (-0.15, 0.08),
            (-0.15, -0.08),
        ]
        return positions[leg_id] if leg_id < 4 else (0, 0)
