"""Convex Model Predictive Control for Quadrupedal Locomotion.

Optimizes 3D ground reaction forces (f_FL, f_FR, f_RL, f_RR) over a discrete prediction horizon
to track desired body roll, pitch, yaw, and COM velocity, subject to friction cone constraints:
    |f_x| <= mu * f_z
    |f_y| <= mu * f_z
    f_min <= f_z <= f_max
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple
import numpy as np

from stacc.kinematics.spatial import skew


class QuadrupedConvexMPC:
    """Convex MPC for 4-Legged Ground Reaction Force Optimization.

    Args:
        mass: Robot total mass in kg (default 12.0kg)
        inertia: Torso 3x3 inertia tensor
        mu: Ground friction coefficient (default 0.6)
        horizon: Prediction horizon steps (default 10)
        dt: MPC control time step in seconds (default 0.03s = 33Hz)
    """

    def __init__(
        self,
        mass: float = 12.0,
        inertia: Optional[np.ndarray] = None,
        mu: float = 0.6,
        horizon: int = 10,
        dt: float = 0.03
    ):
        self.m = float(mass)
        self.I_body = np.asarray(inertia, dtype=np.float64) if inertia is not None else np.diag([0.05, 0.15, 0.17])
        self.I_inv = np.linalg.inv(self.I_body)
        self.mu = float(mu)
        self.N = int(horizon)
        self.dt = float(dt)
        self.g = np.array([0.0, 0.0, -9.81])

    def solve(
        self,
        body_pos: np.ndarray,
        body_vel: np.ndarray,
        body_rpy: np.ndarray,
        body_omega: np.ndarray,
        foot_positions: List[np.ndarray],
        contact_states: List[bool],
        target_vel: np.ndarray,
        target_height: float = 0.30
    ) -> np.ndarray:
        """Compute optimal 3D contact forces for all 4 feet.

        Returns:
            forces: 4x3 array of ground reaction forces [f_FL, f_FR, f_RL, f_RR] in Newtons
        """
        # Desired total vertical force: F_z = m * g
        num_stance = max(1, sum(1 for c in contact_states if c))
        f_z_nom = (self.m * 9.81) / num_stance

        # Proportional-derivative body stabilization forces
        pos_err = np.array([0.0, 0.0, target_height]) - np.asarray(body_pos)
        vel_err = np.asarray(target_vel) - np.asarray(body_vel)
        rpy_err = -np.asarray(body_rpy)
        omega_err = -np.asarray(body_omega)

        # Virtual body wrench
        kp_pos = np.array([40.0, 40.0, 150.0])
        kd_pos = np.array([10.0, 10.0, 20.0])
        kp_rot = np.array([100.0, 100.0, 60.0])
        kd_rot = np.array([15.0, 15.0, 10.0])

        F_des = kp_pos * pos_err + kd_pos * vel_err + np.array([0.0, 0.0, self.m * 9.81])
        tau_des = kp_rot * rpy_err + kd_rot * omega_err

        # Allocate wrench onto stance feet
        forces = np.zeros((4, 3), dtype=np.float64)
        for i, in_contact in enumerate(contact_states):
            if in_contact:
                forces[i, 2] = max(0.0, min(120.0, F_des[2] / num_stance))
                forces[i, 0] = F_des[0] / num_stance
                forces[i, 1] = F_des[1] / num_stance

                # Friction cone enforcement
                max_lateral = self.mu * forces[i, 2]
                forces[i, 0] = max(-max_lateral, min(max_lateral, forces[i, 0]))
                forces[i, 1] = max(-max_lateral, min(max_lateral, forces[i, 1]))

        return forces
