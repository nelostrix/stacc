"""Cartesian Task-Space Impedance and Admittance Control.

Provides virtual spring-damper-mass mechanical compliance for safe human-robot interaction,
contact-rich surface following, and force-controlled assembly.

Formulation:
    M_d * (x_ddot_d - x_ddot) + D_d * (x_dot_d - x_dot) + K_d * (x_d - x) = F_ext
Joint torque synthesis:
    tau = J(q)^T * [ K_d * (x_d - x) + D_d * (x_dot_d - x_dot) ] + G(q)
"""

from __future__ import annotations
from typing import List, Optional, Tuple, Union
import numpy as np


class CartesianImpedanceController:
    """Cartesian 6-DOF (or 3-DOF) Impedance Controller.

    Args:
        stiffness: 6-vector (or 6x6 matrix) of translation/rotation stiffnesses K_d [N/m, Nm/rad]
        damping: 6-vector (or 6x6 matrix) of damping coefficients D_d [Ns/m, Nms/rad]
        inertia: 6-vector (or 6x6 matrix) of desired apparent inertia M_d (optional)
        null_space_stiffness: Stiffness for secondary joint-space posture attractor
        null_space_damping: Damping for secondary joint-space posture attractor
    """

    def __init__(
        self,
        stiffness: Union[float, List[float], np.ndarray] = 400.0,
        damping: Union[float, List[float], np.ndarray] = 40.0,
        inertia: Optional[Union[float, List[float], np.ndarray]] = None,
        null_space_stiffness: float = 10.0,
        null_space_damping: float = 2.0
    ):
        if np.isscalar(stiffness):
            self.K_d = np.eye(6) * float(stiffness)
        elif np.ndim(stiffness) == 1:
            self.K_d = np.diag(stiffness) if len(stiffness) == 6 else np.diag(np.concatenate([stiffness, stiffness]))
        else:
            self.K_d = np.asarray(stiffness, dtype=np.float64)

        if np.isscalar(damping):
            self.D_d = np.eye(6) * float(damping)
        elif np.ndim(damping) == 1:
            self.D_d = np.diag(damping) if len(damping) == 6 else np.diag(np.concatenate([damping, damping]))
        else:
            self.D_d = np.asarray(damping, dtype=np.float64)

        self.null_space_kp = float(null_space_stiffness)
        self.null_space_kd = float(null_space_damping)

    def compute_task_wrench(
        self,
        target_pos: np.ndarray,
        current_pos: np.ndarray,
        target_vel: Optional[np.ndarray] = None,
        current_vel: Optional[np.ndarray] = None,
        target_rot: Optional[np.ndarray] = None,
        current_rot: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """Compute the virtual Cartesian impedance wrench F_task = [F_trans, F_rot].

        Returns:
            6D Cartesian wrench [fx, fy, fz, tx, ty, tz]
        """
        # Translation error
        pos_err = np.asarray(target_pos, dtype=np.float64) - np.asarray(current_pos, dtype=np.float64)
        
        # Velocity error
        v_des = np.asarray(target_vel, dtype=np.float64) if target_vel is not None else np.zeros(3)
        v_cur = np.asarray(current_vel, dtype=np.float64) if current_vel is not None else np.zeros(3)
        vel_err = v_des - v_cur

        # Orientation error (using small angle approximation: 0.5 * (R_cur.T * R_des - R_des.T * R_cur))
        if target_rot is not None and current_rot is not None:
            R_des = np.asarray(target_rot, dtype=np.float64)
            R_cur = np.asarray(current_rot, dtype=np.float64)
            R_err = R_cur.T @ R_des
            rot_err = 0.5 * np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1]
            ])
            rot_err = R_cur @ rot_err  # Transform back to base frame
        else:
            rot_err = np.zeros(3)

        delta_x = np.concatenate([pos_err, rot_err])
        delta_v = np.concatenate([vel_err, np.zeros(3)])

        # Impedance law: F = K_d * delta_x + D_d * delta_v
        F_task = self.K_d @ delta_x + self.D_d @ delta_v
        return F_task

    def compute_joint_torques(
        self,
        J: np.ndarray,
        target_pos: np.ndarray,
        current_pos: np.ndarray,
        target_vel: Optional[np.ndarray] = None,
        current_vel: Optional[np.ndarray] = None,
        q: Optional[np.ndarray] = None,
        q_dot: Optional[np.ndarray] = None,
        q_nominal: Optional[np.ndarray] = None,
        gravity_torques: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """Compute joint torques tau = J^T * F_task + (I - J^T * J^#T) * tau_null + G(q)."""
        F_task = self.compute_task_wrench(target_pos, current_pos, target_vel, current_vel)
        
        # Primary task torque: tau = J^T * F_task
        if J.shape[0] == 3:
            tau_task = J.T @ F_task[:3]
        else:
            tau_task = J.T @ F_task

        # Null-space posture regularization for redundant manipulators (n > 6)
        n = J.shape[1]
        tau_null = np.zeros(n)
        if q is not None and q_nominal is not None:
            q_err = np.asarray(q_nominal) - np.asarray(q)
            q_d = np.asarray(q_dot) if q_dot is not None else np.zeros(n)
            tau_posture = self.null_space_kp * q_err - self.null_space_kd * q_d

            # Null-space projector: N = I - J^T * J_pinv^T
            J_pinv = np.linalg.pinv(J)
            N = np.eye(n) - J.T @ J_pinv.T
            tau_null = N @ tau_posture

        tau_total = tau_task + tau_null
        if gravity_torques is not None:
            tau_total += np.asarray(gravity_torques, dtype=np.float64)

        return tau_total
