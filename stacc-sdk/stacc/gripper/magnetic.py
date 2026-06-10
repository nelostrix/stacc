from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


class MagneticGripper(Gripper):
    """Electromagnetic gripper — holds ferromagnetic objects with a magnet.

    Instant engage/disengage. No moving parts. Only works on
    ferromagnetic materials (steel, iron).

    Args:
        max_force: Maximum holding force in Newtons
        diameter: Magnet face diameter in meters
    """

    def __init__(self, max_force: float = 50.0, diameter: float = 0.05):
        super().__init__("magnetic_gripper")
        self.max_force = max_force
        self.diameter = diameter
        self._energized = False

    @property
    def is_energized(self) -> bool:
        return self._energized

    def open(self, speed: float = 1.0):
        self._energized = False
        self._current_force = 0.0
        self._grasp_detected = False
        self._state = "open"

    def close(self, force: Optional[float] = None, speed: float = 1.0):
        self._energized = True
        self._current_force = min(force or self.max_force, self.max_force)
        self._grasp_detected = True
        self._state = "closed"

    def update(self, dt: float):
        if not self._energized:
            self._current_force = 0.0
            self._grasp_detected = False
