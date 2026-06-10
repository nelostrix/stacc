from __future__ import annotations

import math
from typing import Dict

from stacc.sensors.base import BaseSensor


class Encoder(BaseSensor):
    """Rotary encoder for measuring shaft position/velocity.

    Args:
        ticks_per_rev: Encoder resolution (CPR)
        gear_ratio: Gear ratio (output/input)
    """

    def __init__(self, ticks_per_rev: int = 1024, gear_ratio: float = 1.0, **kwargs):
        super().__init__(name="encoder", rate=1000.0, **kwargs)
        self.ticks_per_rev = ticks_per_rev
        self.gear_ratio = gear_ratio
        self._ticks: int = 0
        self._prev_ticks: int = 0

    @property
    def angle(self) -> float:
        """Current angle in radians."""
        return (self._ticks / self.ticks_per_rev) * 2 * math.pi / self.gear_ratio

    @property
    def velocity(self) -> float:
        """Angular velocity in rad/s (computed from tick difference)."""
        dticks = self._ticks - self._prev_ticks
        return (dticks / self.ticks_per_rev) * 2 * math.pi * self.rate / self.gear_ratio

    def read(self) -> Dict[str, float]:
        return {"angle": self.angle, "velocity": self.velocity, "ticks": self._ticks}
