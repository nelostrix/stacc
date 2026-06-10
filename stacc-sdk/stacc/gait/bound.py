from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from stacc.gait.base import GaitGenerator


class BoundGait(GaitGenerator):
    """Bound gait — both fronts then both rears.

    Symmetrical left-right, alternating front-back. Medium speed.
    """

    def __init__(self, **kwargs):
        kwargs.setdefault("frequency", 2.5)
        super().__init__(num_legs=4, **kwargs)
        self._phase_offsets = [0.0, 0.0, 0.5, 0.5]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        positions = [
            (0.12, 0.08),
            (0.12, -0.08),
            (-0.12, 0.08),
            (-0.12, -0.08),
        ]
        return positions[leg_id] if leg_id < 4 else (0, 0)
