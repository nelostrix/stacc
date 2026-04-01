from __future__ import annotations

from typing import List

from stacc.sensors.base import BaseSensor


class IRArray(BaseSensor):
    """Infrared sensor array for line following.

    Args:
        count: Number of IR sensors in the array
        spacing: Distance between sensors (meters)
    """

    def __init__(self, count: int = 5, spacing: float = 0.01, **kwargs):
        super().__init__(name="ir_array", rate=200.0, **kwargs)
        self.count = count
        self.spacing = spacing
        self._values = [0.0] * count

    def read(self) -> List[float]:
        """Read raw IR values (0.0 = white/no line, 1.0 = black/line)."""
        return self.add_noise_vector(self._values)

    def line_position(self) -> float:
        """Compute line position as weighted average.

        Returns:
            -1.0 (far left) to 1.0 (far right), 0.0 = centered
        """
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
