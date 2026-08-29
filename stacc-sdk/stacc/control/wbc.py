"""Whole-Body Control (WBC) & Hierarchical Task Prioritization.

Enables simultaneous multi-objective execution for Humanoids, Quadrupeds, and Mobile Manipulators
with strict priority null-space projection:
    Task 1 (Highest): Unilateral contact constraints & friction cones
    Task 2: Dynamic Center of Mass (COM) balance
    Task 3: Operational space end-effector / foot tracking
    Task 4: Joint posture regularization & torque minimization
"""

from __future__ import annotations
from typing import List, Optional, Tuple, Union
import numpy as np


class TaskObjective:
    """A prioritized motion or force task objective."""

    def __init__(self, name: str, J: np.ndarray, x_ddot_des: np.ndarray, weight: float = 1.0):
        self.name = name
        self.J = np.asarray(J, dtype=np.float64)  # Task Jacobian (m x n)
        self.x_ddot_des = np.asarray(x_ddot_des, dtype=np.float64)  # Desired task acceleration (m)
        self.weight = float(weight)


class HierarchicalWBC:
    """Hierarchical Whole-Body Controller using Sequential Null-Space Projection."""

    def __init__(self, num_dof: int):
        self.n = int(num_dof)

    def solve(
        self,
        tasks: List[TaskObjective],
        M_mat: np.ndarray,
        C_qdot: np.ndarray,
        G_vec: np.ndarray,
        damping: float = 1e-4
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute optimal joint accelerations q_ddot and joint torques tau.

        Args:
            tasks: List of TaskObjectives in descending priority (tasks[0] has highest priority)
            M_mat: Generalized inertia matrix M(q) (n x n)
            C_qdot: Coriolis force vector (n)
            G_vec: Gravity force vector (n)
            damping: Singular value damping for pseudo-inverses

        Returns:
            q_ddot_opt: Joint accelerations satisfying the task hierarchy
            tau_opt: Inverse dynamics joint torques
        """
        n = self.n
        q_ddot = np.zeros(n, dtype=np.float64)
        N_prev = np.eye(n, dtype=np.float64)  # Global null-space projector

        for task in tasks:
            # Projected Jacobian: J_proj = J_k * N_{k-1}
            J_proj = task.J @ N_prev

            # Damped pseudo-inverse: J_proj^# = J_proj^T * (J_proj * J_proj^T + lambda * I)^-1
            m = task.J.shape[0]
            JJT = J_proj @ J_proj.T
            J_pinv = J_proj.T @ np.linalg.inv(JJT + damping * np.eye(m))

            # Task acceleration error: e = a_des - J_k * q_ddot_{prev}
            a_err = task.x_ddot_des - task.J @ q_ddot

            # Null-space contribution: delta_q_ddot = J_pinv * a_err
            q_ddot += J_pinv @ a_err

            # Update null-space projector: N_k = N_{k-1} * (I - J_pinv * J_proj)
            N_prev = N_prev @ (np.eye(n) - J_pinv @ J_proj)

        # Inverse dynamics torque: tau = M * q_ddot + C + G
        tau_opt = M_mat @ q_ddot + C_qdot + G_vec
        return q_ddot, tau_opt
