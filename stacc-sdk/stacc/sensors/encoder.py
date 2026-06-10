from __future__ import annotations

import math
from typing import Dict

from stacc.sensors.base import BaseSensor


class Encoder(BaseSensor):
    """Rotary encoder for measuring shaft position/velocity.

    Args:
        ticks_per_rev: Encoder resolution in counts per revolution (must be > 0)
        gear_ratio: Gear ratio output/input (must be > 0)
    """

    def __init__(self, ticks_per_rev: int = 1024, gear_ratio: float = 1.0, **kwargs):
        super().__init__(name="encoder", rate=1000.0, **kwargs)
        if ticks_per_rev <= 0:
            raise ValueError(f"ticks_per_rev must be > 0, got {ticks_per_rev}")
        if gear_ratio <= 0:
            raise ValueError(f"gear_ratio must be > 0, got {gear_ratio}")

        self.ticks_per_rev = ticks_per_rev
        self.gear_ratio = gear_ratio
        self._ticks: int = 0
        self._prev_ticks: int = 0

    def reset(self):
        """Reset encoder ticks to zero."""
        self._ticks = 0
        self._prev_ticks = 0

    @property
    def angle(self) -> float:
        return (self._ticks / self.ticks_per_rev) * 2 * math.pi / self.gear_ratio

    @property
    def velocity(self) -> float:
        dticks = self._ticks - self._prev_ticks
        return (dticks / self.ticks_per_rev) * 2 * math.pi * self.rate / self.gear_ratio

    def read(self) -> Dict[str, float]:
        return {"angle": self.angle, "velocity": self.velocity, "ticks": self._ticks}

    def __repr__(self) -> str:
        return (
            f"Encoder(name='{self.name}', ticks_per_rev={self.ticks_per_rev}, "
            f"gear_ratio={self.gear_ratio})"
        )
