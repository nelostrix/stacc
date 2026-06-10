from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from stacc.gripper.base import Gripper


class ParallelGripper(Gripper):
    """Parallel jaw gripper — two flat fingers that move linearly.

    The most common industrial gripper. Two jaws open/close
    symmetrically to grasp objects between them.

    Args:
        max_width: Maximum opening width in meters (default 0.08)
        min_width: Minimum closing width in meters (default 0.0)
        max_force: Maximum gripping force in Newtons (default 20.0)
        speed: Opening/closing speed in m/s (default 0.1)
    """

    def __init__(
        self,
        max_width: float = 0.08,
        min_width: float = 0.0,
        max_force: float = 20.0,
        speed: float = 0.1,
    ):
        if min_width < 0:
            raise ValueError(f"min_width must be >= 0, got {min_width}")
        if max_width <= min_width:
            raise ValueError(
                f"max_width ({max_width}) must be > min_width ({min_width})"
            )

        super().__init__("parallel_gripper")
        self.max_width = max_width
        self.min_width = min_width
        self.max_force = max_force
        self.speed = speed
        self._current_width = max_width
        self._target_width = max_width

    def open(self, speed: float = 1.0) -> None:
        self._target_width = self.max_width
        self._target_force = 0.0
        self._state = "opening"
        self._grasp_detected = False

    def close(
        self,
        force: Optional[float] = None,
        width: Optional[float] = None,
        speed: float = 1.0,
    ) -> None:
        """Close the gripper to a target width with a given force.

        Args:
            force: Gripping force in Newtons (defaults to max_force)
            width: Target jaw width in meters (defaults to min_width)
            speed: Relative speed multiplier
        """
        self._target_width = width if width is not None else self.min_width
        self._target_force = min(force or self.max_force, self.max_force)
        self._state = "closing"

    def set_width(self, width: float) -> None:
        """Set target jaw width, clamped to [min_width, max_width]."""
        self._target_width = max(self.min_width, min(self.max_width, width))

    def update(self, dt: float) -> None:
        move = self.speed * dt

        if self._state == "closing":
            self._current_width = max(self._target_width, self._current_width - move)
            if self._current_width <= self._target_width + 0.001:
                self._state = "closed"
                self._current_force = self._target_force
                if self._current_width > self.min_width + 0.002:
                    self._grasp_detected = True

        elif self._state == "opening":
            self._current_width = min(self.max_width, self._current_width + move)
            self._current_force = 0.0
            if self._current_width >= self.max_width - 0.001:
                self._state = "open"
                self._grasp_detected = False
