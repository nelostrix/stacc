from __future__ import annotations

from typing import List

from stacc.sensors.base import BaseSensor


class IRArray(BaseSensor):
    """Infrared sensor array for line following.

    Args:
        count: Number of IR sensors (must be >= 1)
        spacing: Distance between sensors in meters (must be > 0)
    """

    def __init__(self, count: int = 5, spacing: float = 0.01, **kwargs):
        super().__init__(name="ir_array", rate=200.0, **kwargs)
        if count < 1:
            raise ValueError(f"count must be >= 1, got {count}")
        if spacing <= 0:
            raise ValueError(f"spacing must be > 0, got {spacing}")

        self.count = count
        self.spacing = spacing
        self._values = [0.0] * count

    def reset(self):
        """Clear all IR readings."""
        self._values = [0.0] * self.count

    def read(self) -> List[float]:
        return self.add_noise_vector(self._values)

    def line_position(self) -> float:
        values = self.read()
        total = sum(values)
        if total < 0.01:
            return 0.0

        positions = [
            (i - (self.count - 1) / 2) / ((self.count - 1) / 2)
            for i in range(self.count)
        ]
        weighted = sum(p * v for p, v in zip(positions, values))
        return weighted / total

    def __repr__(self) -> str:
        return (
            f"IRArray(name='{self.name}', count={self.count}, "
            f"spacing={self.spacing}m, rate={self.rate}Hz)"
        )
