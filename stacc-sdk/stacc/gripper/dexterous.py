from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from stacc.gripper.base import Gripper


class DexterousHand(Gripper):
    """Multi-finger dexterous hand — human-like manipulation.

    Each finger has multiple joints for complex grasps.
    Supports power grasp, pinch grasp, and individual finger control.

    Args:
        num_fingers: Number of fingers, typically 3-5 (default 5)
        joints_per_finger: Joints per finger, typically 2-4 (default 3)
        max_force_per_finger: Maximum force per finger in Newtons (default 5.0)
        curl_power_grasp: Finger curl for power grasp in radians (default 1.5)
        curl_pinch_grasp: Finger curl for pinch grasp in radians (default 1.2)
    """

    def __init__(
        self,
        num_fingers: int = 5,
        joints_per_finger: int = 3,
        max_force_per_finger: float = 5.0,
        curl_power_grasp: float = 1.5,
        curl_pinch_grasp: float = 1.2,
    ):
        if num_fingers < 1:
            raise ValueError(f"num_fingers must be >= 1, got {num_fingers}")
        if joints_per_finger < 1:
            raise ValueError(
                f"joints_per_finger must be >= 1, got {joints_per_finger}"
            )

        super().__init__("dexterous_hand")
        self.num_fingers = num_fingers
        self.joints_per_finger = joints_per_finger
        self.max_force_per_finger = max_force_per_finger
        self.max_force = max_force_per_finger * num_fingers
        self._curl_power_grasp = curl_power_grasp
        self._curl_pinch_grasp = curl_pinch_grasp
        self._finger_angles = [
            [0.0] * joints_per_finger for _ in range(num_fingers)
        ]
        self._finger_targets = [
            [0.0] * joints_per_finger for _ in range(num_fingers)
        ]

    # ── Properties ──────────────────────

    @property
    def finger_angles(self) -> List[List[float]]:
        return [list(f) for f in self._finger_angles]

    @property
    def total_dof(self) -> int:
        return self.num_fingers * self.joints_per_finger

    # ── Public API ──────────────────────

    def set_finger(
        self, finger_index: int, joint_angles: List[float]
    ) -> None:
        """Set target angles for a specific finger.

        Args:
            finger_index: Index of the finger (0-based)
            joint_angles: Target angles for each joint

        Raises:
            IndexError: If finger_index is out of range
        """
        if finger_index < 0 or finger_index >= self.num_fingers:
            raise IndexError(
                f"finger_index {finger_index} out of range "
                f"[0, {self.num_fingers})"
            )
        for j in range(min(len(joint_angles), self.joints_per_finger)):
            self._finger_targets[finger_index][j] = joint_angles[j]

    def power_grasp(
        self, force: Optional[float] = None
    ) -> None:
        """Close all fingers in a power grasp (full curl).

        Args:
            force: Gripping force (defaults to max_force)
        """
        curl = self._curl_power_grasp
        for i in range(self.num_fingers):
            self._finger_targets[i] = [curl] * self.joints_per_finger
        self._target_force = force or self.max_force
        self._state = "closing"

    def pinch_grasp(
        self, force: Optional[float] = None
    ) -> None:
        """Close first two fingers for a precision pinch.

        Args:
            force: Gripping force (defaults to 30% of max_force)
        """
        curl = self._curl_pinch_grasp
        for i in range(self.num_fingers):
            if i < 2:
                self._finger_targets[i] = [curl] * self.joints_per_finger
            else:
                self._finger_targets[i] = [0.0] * self.joints_per_finger
        self._target_force = force or (self.max_force * 0.3)
        self._state = "closing"

    def open(self, speed: float = 1.0) -> None:
        for i in range(self.num_fingers):
            self._finger_targets[i] = [0.0] * self.joints_per_finger
        self._state = "opening"
        self._grasp_detected = False

    def close(
        self, force: Optional[float] = None, speed: float = 1.0
    ) -> None:
        self.power_grasp(force)

    def reset(self) -> None:
        self._finger_angles = [
            [0.0] * self.joints_per_finger for _ in range(self.num_fingers)
        ]
        self._finger_targets = [
            [0.0] * self.joints_per_finger for _ in range(self.num_fingers)
        ]
        super().reset()

    def update(self, dt: float) -> None:
        rate = 3.0 * dt
        all_reached = True

        for i in range(self.num_fingers):
            for j in range(self.joints_per_finger):
                current = self._finger_angles[i][j]
                target = self._finger_targets[i][j]
                if current < target:
                    self._finger_angles[i][j] = min(target, current + rate)
                elif current > target:
                    self._finger_angles[i][j] = max(target, current - rate)
                if abs(self._finger_angles[i][j] - target) > 0.01:
                    all_reached = False

        if all_reached:
            if self._state == "closing":
                self._state = "closed"
                self._current_force = self._target_force
                self._grasp_detected = True
            elif self._state == "opening":
                self._state = "open"
                self._current_force = 0.0
                self._grasp_detected = False
