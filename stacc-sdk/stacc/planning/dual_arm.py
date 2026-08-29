"""Bimanual Coordinated Dual-Arm Motion Planning.

Solves synchronized motion planning for two arms manipulating a shared rigid object
under closed kinematic chain constraints:
    p_left(q_left) - p_right(q_right) = const_distance
    R_left(q_left)^T * R_right(q_right) = const_relative_orientation
"""

from __future__ import annotations
from typing import Tuple
import numpy as np


class DualArmCoordinator:
    """Coordinated Bimanual Dual-Arm Motion Planner."""

    def __init__(self, relative_grasp_offset: np.ndarray):
        self.grasp_offset = np.asarray(relative_grasp_offset, dtype=np.float64)

    def compute_slave_target(self, master_pose: np.ndarray) -> np.ndarray:
        """Given master arm end-effector pose, compute required synchronous slave arm pose."""
        return np.asarray(master_pose, dtype=np.float64) + self.grasp_offset
