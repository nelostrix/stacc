from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from stacc.gait.base import GaitGenerator


class BipedWalkGait(GaitGenerator):
    """Biped walking gait — human-like walking.

    Two legs alternate with weight shifting and foot placement.
    """

    def __init__(self, **kwargs):
        kwargs.setdefault("frequency", 1.5)
        kwargs.setdefault("step_height", 0.03)
        super().__init__(num_legs=2, **kwargs)
        self._phase_offsets = [0.0, 0.5]

    def _default_foot_pos(self, leg_id: int) -> Tuple[float, float]:
        return (0, 0.08) if leg_id == 0 else (0, -0.08)
