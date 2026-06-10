from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


class DexterousHand(Gripper):
    """Multi-finger dexterous hand — human-like manipulation.

    Each finger has multiple joints for complex grasps.
    Supports power grasp, pinch grasp, and individual finger control.

    Args:
        num_fingers: Number of fingers (typically 3-5)
        joints_per_finger: Joints per finger (typically 2-4)
        max_force_per_finger: Maximum force per finger in Newtons
    """

    def __init__(
        self,
        num_fingers: int = 5,
        joints_per_finger: int = 3,
        max_force_per_finger: float = 5.0,
    ):
        super().__init__("dexterous_hand")
        self.num_fingers = num_fingers
        self.joints_per_finger = joints_per_finger
        self.max_force_per_finger = max_force_per_finger
        self.max_force = max_force_per_finger * num_fingers
        self._finger_angles = [[0.0] * joints_per_finger for _ in range(num_fingers)]
        self._finger_targets = [[0.0] * joints_per_finger for _ in range(num_fingers)]

    @property
    def finger_angles(self) -> List[List[float]]:
        return [list(f) for f in self._finger_angles]

    @property
    def total_dof(self) -> int:
        return self.num_fingers * self.joints_per_finger

    def set_finger(self, finger_index: int, joint_angles: List[float]):
        if 0 <= finger_index < self.num_fingers:
            for j in range(min(len(joint_angles), self.joints_per_finger)):
                self._finger_targets[finger_index][j] = joint_angles[j]

    def power_grasp(self, force: Optional[float] = None):
        target = 1.5  # curl all fingers
        for i in range(self.num_fingers):
            self._finger_targets[i] = [target] * self.joints_per_finger
        self._target_force = force or self.max_force
        self._state = "closing"

    def pinch_grasp(self, force: Optional[float] = None):
        for i in range(self.num_fingers):
            if i < 2:
                self._finger_targets[i] = [1.2] * self.joints_per_finger
            else:
                self._finger_targets[i] = [0.0] * self.joints_per_finger
        self._target_force = force or (self.max_force * 0.3)
        self._state = "closing"

    def open(self, speed: float = 1.0):
        for i in range(self.num_fingers):
            self._finger_targets[i] = [0.0] * self.joints_per_finger
        self._state = "opening"
        self._grasp_detected = False

    def close(self, force: Optional[float] = None, speed: float = 1.0):
        self.power_grasp(force)

    def update(self, dt: float):
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
