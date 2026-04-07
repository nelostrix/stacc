from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


class VacuumGripper(Gripper):
    """Vacuum suction gripper — holds objects with negative pressure.

    Uses vacuum suction cups to pick up flat or smooth objects.
    Common in pick-and-place applications with boxes, PCBs, glass.

    Args:
        num_cups: Number of suction cups
        cup_diameter: Diameter of each cup in meters
        max_vacuum: Maximum vacuum pressure in kPa
        seal_threshold: Minimum vacuum for reliable grasp (kPa)
    """

    def __init__(
        self,
        num_cups: int = 1,
        cup_diameter: float = 0.03,
        max_vacuum: float = 80.0,
        seal_threshold: float = 40.0,
    ):
        super().__init__("vacuum_gripper")
        self.num_cups = num_cups
        self.cup_diameter = cup_diameter
        self.max_vacuum = max_vacuum
        self.seal_threshold = seal_threshold
        self._vacuum_level = 0.0
        self._sealed = False

    @property
    def vacuum_level(self) -> float:
        return self._vacuum_level

    @property
    def is_sealed(self) -> bool:
        return self._sealed

    def open(self, speed: float = 1.0):
        self._vacuum_level = 0.0
        self._sealed = False
        self._grasp_detected = False
        self._state = "open"

    def close(self, force: Optional[float] = None, speed: float = 1.0):
        target_vacuum = force if force is not None else self.max_vacuum
        self._vacuum_level = min(target_vacuum, self.max_vacuum)
        self._state = "closed"

    def update(self, dt: float):
        if self._state == "closed" and self._vacuum_level >= self.seal_threshold:
            self._sealed = True
            self._grasp_detected = True
            self._current_force = (
                self._vacuum_level
                * math.pi
                * (self.cup_diameter / 2) ** 2
                * self.num_cups
                * 1000
            )
        else:
            self._sealed = False
            self._grasp_detected = False
            self._current_force = 0.0

    def release(self):
        self._vacuum_level = 0.0
        self._sealed = False
        self._grasp_detected = False
        self._state = "open"
