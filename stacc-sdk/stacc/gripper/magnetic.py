from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from stacc.gripper.base import Gripper


class MagneticGripper(Gripper):
    """Electromagnetic gripper — holds ferromagnetic objects with a magnet.

    Instant engage/disengage with no moving parts. Only works on
    ferromagnetic materials (steel, iron, nickel).

    Args:
        max_force: Maximum holding force in Newtons (default 50.0)
        diameter: Magnet face diameter in meters (default 0.05)
    """

    def __init__(self, max_force: float = 50.0, diameter: float = 0.05):
        if max_force <= 0:
            raise ValueError(f"max_force must be > 0, got {max_force}")
        if diameter <= 0:
            raise ValueError(f"diameter must be > 0, got {diameter}")

        super().__init__("magnetic_gripper")
        self.max_force = max_force
        self.diameter = diameter
        self._energized: bool = False

    # ── Properties ──────────────────────

    @property
    def is_energized(self) -> bool:
        return self._energized

    # ── Public API ──────────────────────

    def open(self, speed: float = 1.0) -> None:
        self._energized = False
        self._current_force = 0.0
        self._grasp_detected = False
        self._state = "open"

    def close(
        self, force: Optional[float] = None, speed: float = 1.0
    ) -> None:
        self._energized = True
        self._current_force = min(force or self.max_force, self.max_force)
        self._grasp_detected = True
        self._state = "closed"

    def release(self) -> None:
        """De-energize the magnet, releasing any held object."""
        self.open()

    def update(self, dt: float) -> None:
        if not self._energized:
            self._current_force = 0.0
            self._grasp_detected = False
