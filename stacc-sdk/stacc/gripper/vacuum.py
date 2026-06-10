from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from stacc.gripper.base import Gripper


class VacuumGripper(Gripper):
    """Vacuum suction gripper — holds objects with negative pressure.

    Uses vacuum suction cups to pick up flat or smooth objects.
    Common in pick-and-place applications (boxes, PCBs, glass, panels).

    Holding force is computed from vacuum pressure, cup area, and
    number of cups using: F = P * A * N * 1000 (kPa to Pa conversion).

    Args:
        num_cups: Number of suction cups (default 1)
        cup_diameter: Diameter of each cup in meters (default 0.03)
        max_vacuum: Maximum vacuum pressure in kPa (default 80.0)
        seal_threshold: Minimum vacuum in kPa for reliable seal (default 40.0)
    """

    def __init__(
        self,
        num_cups: int = 1,
        cup_diameter: float = 0.03,
        max_vacuum: float = 80.0,
        seal_threshold: float = 40.0,
    ):
        if num_cups < 1:
            raise ValueError(f"num_cups must be >= 1, got {num_cups}")
        if cup_diameter <= 0:
            raise ValueError(f"cup_diameter must be > 0, got {cup_diameter}")
        if not 0 < seal_threshold <= max_vacuum:
            raise ValueError(
                f"seal_threshold ({seal_threshold}) must be in "
                f"(0, max_vacuum={max_vacuum}]"
            )

        super().__init__("vacuum_gripper")
        self.num_cups = num_cups
        self.cup_diameter = cup_diameter
        self.max_vacuum = max_vacuum
        self.seal_threshold = seal_threshold
        self._vacuum_level: float = 0.0
        self._sealed: bool = False

    # ── Properties ──────────────────────

    @property
    def vacuum_level(self) -> float:
        return self._vacuum_level

    @property
    def is_sealed(self) -> bool:
        return self._sealed

    @property
    def holding_force(self) -> float:
        """Compute the current holding force from vacuum and cup geometry.

        F = P * A * N, where:
          P = vacuum pressure in kPa (*1000 to convert to Pa)
          A = pi * (d/2)^2  (cup area in m^2)
          N = number of cups

        Returns:
            Holding force in Newtons
        """
        if self._vacuum_level <= 0:
            return 0.0
        area = math.pi * (self.cup_diameter / 2.0) ** 2
        return self._vacuum_level * area * self.num_cups * 1000.0

    # ── Public API ──────────────────────

    def open(self, speed: float = 1.0) -> None:
        self._vacuum_level = 0.0
        self._sealed = False
        self._grasp_detected = False
        self._state = "open"

    def close(
        self, force: Optional[float] = None, speed: float = 1.0
    ) -> None:
        target_vacuum = force if force is not None else self.max_vacuum
        self._vacuum_level = min(target_vacuum, self.max_vacuum)
        self._state = "closed"

    def release(self) -> None:
        """Release vacuum and drop the held object."""
        self._vacuum_level = 0.0
        self._sealed = False
        self._grasp_detected = False
        self._state = "open"

    def update(self, dt: float) -> None:
        if self._state == "closed" and self._vacuum_level >= self.seal_threshold:
            self._sealed = True
            self._grasp_detected = True
            self._current_force = self.holding_force
        else:
            self._sealed = False
            self._grasp_detected = False
            self._current_force = 0.0
