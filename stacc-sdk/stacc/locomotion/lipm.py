"""Linear Inverted Pendulum Model (LIPM) & Zero Moment Point (ZMP) Dynamics.

Provides fundamental balance and walking trajectory generation for bipedal humanoid robots:
- 3D Linear Inverted Pendulum Model (LIPM)
- Zero Moment Point (ZMP) calculation: x_zmp = x - (z_c / g) * x_ddot
- Instantaneous Capture Point (ICP) & Divergent Component of Motion (DCM): xi = x + x_dot / omega_0
- Footstep placement adjustment for push recovery
"""

from __future__ import annotations
from typing import List, Tuple
import math
import numpy as np


class LinearInvertedPendulum:
    """3D Linear Inverted Pendulum Model (LIPM) for Bipedal Humanoids.

    Args:
        com_height: Constant Center of Mass (COM) height z_c in meters (default 0.80m)
        gravity: Gravitational acceleration g in m/s^2 (default 9.81)
    """

    def __init__(self, com_height: float = 0.80, gravity: float = 9.81):
        self.z_c = float(com_height)
        self.g = float(gravity)
        # Natural pendulum frequency: omega_0 = sqrt(g / z_c)
        self.omega_0 = math.sqrt(self.g / self.z_c)

    def compute_zmp(self, com_pos: Tuple[float, float], com_acc: Tuple[float, float]) -> Tuple[float, float]:
        """Compute the Zero Moment Point (ZMP) from COM position and acceleration."""
        px = com_pos[0] - (self.z_c / self.g) * com_acc[0]
        py = com_pos[1] - (self.z_c / self.g) * com_acc[1]
        return px, py

    def compute_capture_point(self, com_pos: Tuple[float, float], com_vel: Tuple[float, float]) -> Tuple[float, float]:
        """Compute the Instantaneous Capture Point (ICP) xi = x + x_dot / omega_0.

        If the robot places its foot at the capture point, it will come to a complete stop.
        """
        xi_x = com_pos[0] + com_vel[0] / self.omega_0
        xi_y = com_pos[1] + com_vel[1] / self.omega_0
        return xi_x, xi_y

    def plan_recovery_footstep(
        self,
        com_pos: Tuple[float, float],
        com_vel: Tuple[float, float],
        step_duration: float = 0.35
    ) -> Tuple[float, float]:
        """Predict the required footstep placement point to recover from a disturbance push."""
        xi_x, xi_y = self.compute_capture_point(com_pos, com_vel)
        # Projected capture point at touchdown: xi(T) = xi(0) * e^(omega_0 * T)
        decay = math.exp(self.omega_0 * float(step_duration))
        step_x = xi_x * (1.0 - 1.0 / decay)
        step_y = xi_y * (1.0 - 1.0 / decay)
        return step_x, step_y
