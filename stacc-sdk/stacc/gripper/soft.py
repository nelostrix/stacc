from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from stacc.gripper.base import Gripper


class SoftGripper(Gripper):
    """Soft/compliant gripper — deformable fingers conform to object shape.

    Uses pneumatic or tendon-driven soft actuators. Inherently safe
    for human interaction and suitable for fragile objects.

    Args:
        num_fingers: Number of soft fingers (default 3)
        max_curl: Maximum finger curl angle in radians (default 2.0)
        max_force: Maximum gentle gripping force in Newtons (default 5.0)
        compliance: Finger compliance 0=rigid, 1=very soft (default 0.7)
    """

    def __init__(
        self,
        num_fingers: int = 3,
        max_curl: float = 2.0,
        max_force: float = 5.0,
        compliance: float = 0.7,
    ):
        if num_fingers < 1:
            raise ValueError(f"num_fingers must be >= 1, got {num_fingers}")
        if not 0 <= compliance <= 1:
            raise ValueError(f"compliance must be in [0, 1], got {compliance}")

        super().__init__("soft_gripper")
        self.num_fingers = num_fingers
        self.max_curl = max_curl
        self.max_force = max_force
        self.compliance = compliance
        self._curl_angles: List[float] = [0.0] * num_fingers
        self._target_curl: float = 0.0

    # ── Properties ──────────────────────

    @property
    def curl_angles(self) -> List[float]:
        return list(self._curl_angles)

    # ── Public API ──────────────────────

    def open(self, speed: float = 1.0) -> None:
        self._target_curl = 0.0
        self._state = "opening"
        self._grasp_detected = False

    def close(
        self, force: Optional[float] = None, speed: float = 1.0
    ) -> None:
        self._target_curl = self.max_curl
        self._target_force = min(force or self.max_force, self.max_force)
        self._state = "closing"

    def set_curl(self, curl: float) -> None:
        """Set target finger curl, clamped to [0, max_curl]."""
        self._target_curl = max(0.0, min(self.max_curl, curl))

    def reset(self) -> None:
        self._curl_angles = [0.0] * self.num_fingers
        self._target_curl = 0.0
        super().reset()

    def update(self, dt: float) -> None:
        rate = 2.0 * dt

        if self._state == "closing":
            for i in range(self.num_fingers):
                self._curl_angles[i] = min(
                    self._target_curl, self._curl_angles[i] + rate
                )
            if all(a >= self._target_curl - 0.01 for a in self._curl_angles):
                self._state = "closed"
                self._current_force = self._target_force * self.compliance
                self._grasp_detected = True

        elif self._state == "opening":
            for i in range(self.num_fingers):
                self._curl_angles[i] = max(0.0, self._curl_angles[i] - rate)
            self._current_force = 0.0
            if all(a <= 0.01 for a in self._curl_angles):
                self._state = "open"
                self._grasp_detected = False
