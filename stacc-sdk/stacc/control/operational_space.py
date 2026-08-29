"""Operational Space Control (OSC) & Dynamic Decoupling.

Formulation:
    Lambda(q) = (J * M(q)^-1 * J^T)^-1  (Task-space apparent inertia)
    mu(q, q_dot) = Lambda * J * M^-1 * C * q_dot - Lambda * J_dot * q_dot (Task-space Coriolis)
    p(q) = Lambda * J * M^-1 * G (Task-space gravity)
    J_bar = M^-1 * J^T * Lambda (Dynamically consistent generalized inverse)

Torque synthesis:
    tau = J^T * F_task + (I - J^T * J_bar^T) * tau_null
where tau_null does not accelerate the end-effector frame.
"""

from __future__ import annotations
from typing import Optional, Union
import numpy as np


class OperationalSpaceController:
    """Operational Space Decoupled Dynamics Controller.

    Args:
        kp: Task-space proportional gain
        kd: Task-space derivative gain
        kp_null: Null-space secondary posture gain
        kd_null: Null-space secondary damping gain
    """

    def __init__(
        self,
        kp: Union[float, np.ndarray] = 100.0,
        kd: Union[float, np.ndarray] = 20.0,
        kp_null: float = 10.0,
        kd_null: float = 2.0
    ):
        self.kp = float(kp) if np.isscalar(kp) else np.asarray(kp)
        self.kd = float(kd) if np.isscalar(kd) else np.asarray(kd)
        self.kp_null = float(kp_null)
        self.kd_null = float(kd_null)

    def compute(
        self,
        M: np.ndarray,
        C_qdot: np.ndarray,
        G: np.ndarray,
        J: np.ndarray,
        x_des: np.ndarray,
        x_cur: np.ndarray,
        v_des: Optional[np.ndarray] = None,
        v_cur: Optional[np.ndarray] = None,
        q: Optional[np.ndarray] = None,
        q_dot: Optional[np.ndarray] = None,
        q_nominal: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """Compute operational space joint torques."""
        n = M.shape[0]
        M_inv = np.linalg.inv(M + 1e-6 * np.eye(n))

        # Task-space mass matrix: Lambda = (J * M^-1 * J^T)^-1
        JMJ = J @ M_inv @ J.T
        Lambda = np.linalg.inv(JMJ + 1e-6 * np.eye(J.shape[0]))

        # Dynamically consistent inverse: J_bar = M^-1 * J^T * Lambda
        J_bar = M_inv @ J.T @ Lambda

        # Task-space control acceleration
        pos_err = np.asarray(x_des) - np.asarray(x_cur)
        v_d = np.asarray(v_des) if v_des is not None else np.zeros_like(x_des)
        v_c = np.asarray(v_cur) if v_cur is not None else np.zeros_like(x_cur)
        vel_err = v_d - v_c

        a_task = self.kp * pos_err + self.kd * vel_err
        F_task = Lambda @ a_task

        # Primary task torque
        tau_task = J.T @ F_task

        # Secondary posture task in null-space
        tau_null = np.zeros(n)
        if q is not None and q_nominal is not None:
            q_err = np.asarray(q_nominal) - np.asarray(q)
            q_d = np.asarray(q_dot) if q_dot is not None else np.zeros(n)
            tau_posture = self.kp_null * q_err - self.kd_null * q_d

            # Dynamically consistent null-space projector: N = I - J^T * J_bar^T
            N = np.eye(n) - J.T @ J_bar.T
            tau_null = N @ (M @ tau_posture)

        # Full torque with Coriolis and gravity compensation
        tau_total = tau_task + tau_null + C_qdot + G
        return tau_total
