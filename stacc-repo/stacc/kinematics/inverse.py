"""Inverse kinematics solvers.

Compute joint angles that achieve a desired end-effector pose.
Supports Jacobian transpose, damped least squares (Levenberg-Marquardt),
and Cyclic Coordinate Descent (CCD).
"""
from __future__ import annotations
from typing import List, Optional, Tuple
import math

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

from stacc.kinematics.transforms import DH
from stacc.kinematics.forward import (
    forward_kinematics_dh,
    end_effector_position,
    jacobian,
)


class InverseKinematics:
    """IK solver for serial manipulators.

    Args:
        robot: Robot or list of DH parameters
        method: "damped_least_squares" (default), "jacobian_transpose", or "ccd"
        max_iterations: Maximum solver iterations
        tolerance: Position error tolerance in meters
        damping: Damping factor for DLS method (lambda)
    """

    METHODS = ("damped_least_squares", "jacobian_transpose", "ccd")

    def __init__(
        self,
        robot=None,
        dh_params: Optional[List[DH]] = None,
        method: str = "damped_least_squares",
        max_iterations: int = 100,
        tolerance: float = 1e-3,
        damping: float = 0.1,
    ):
        if method not in self.METHODS:
            raise ValueError(f"Unknown IK method '{method}'. Use one of {self.METHODS}")

        if dh_params is not None:
            self.dh_params = dh_params
        elif robot is not None and hasattr(robot, "dh_parameters"):
            self.dh_params = robot.dh_parameters
        else:
            raise ValueError("Provide either robot with dh_parameters or dh_params directly")

        self.method = method
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.damping = damping
        self.n_joints = len(self.dh_params)

    def solve(
        self,
        target_position: List[float],
        orientation: Optional[str] = None,
        initial_guess: Optional[List[float]] = None,
        joint_limits: Optional[List[Tuple[float, float]]] = None,
    ) -> List[float]:
        """Solve IK for a target end-effector position.

        Args:
            target_position: Desired [x, y, z] position in meters
            orientation: Optional orientation hint ("top_down", "horizontal", etc.)
            initial_guess: Starting joint angles (default: zeros)
            joint_limits: List of (min, max) per joint

        Returns:
            List of joint angles in radians
        """
        if not HAS_NUMPY:
            raise ImportError("numpy required for IK")

        target = np.array(target_position, dtype=float)
        q = np.array(initial_guess if initial_guess else [0.0] * self.n_joints, dtype=float)

        if self.method == "damped_least_squares":
            q = self._solve_dls(target, q, joint_limits)
        elif self.method == "jacobian_transpose":
            q = self._solve_jt(target, q, joint_limits)
        elif self.method == "ccd":
            q = self._solve_ccd(target, q, joint_limits)

        return q.tolist()

    def _solve_dls(
        self,
        target: "np.ndarray",
        q: "np.ndarray",
        limits: Optional[List[Tuple[float, float]]],
    ) -> "np.ndarray":
        """Damped Least Squares (Levenberg-Marquardt)."""
        lam = self.damping

        for iteration in range(self.max_iterations):
            current_pos = end_effector_position(self.dh_params, q.tolist())
            error = target - current_pos
            err_norm = np.linalg.norm(error)

            if err_norm < self.tolerance:
                break

            J = jacobian(self.dh_params, q.tolist())
            J_pos = J[:3, :]  # only position rows

            # DLS: dq = J^T (J J^T + λ²I)^{-1} e
            JJT = J_pos @ J_pos.T
            dq = J_pos.T @ np.linalg.solve(JJT + lam**2 * np.eye(3), error)

            q = q + dq

            # Clamp to limits
            if limits:
                for i, (lo, hi) in enumerate(limits):
                    q[i] = max(lo, min(hi, q[i]))

        return q

    def _solve_jt(
        self,
        target: "np.ndarray",
        q: "np.ndarray",
        limits: Optional[List[Tuple[float, float]]],
    ) -> "np.ndarray":
        """Jacobian Transpose method (simpler, slower convergence)."""
        alpha = 0.1  # step size

        for iteration in range(self.max_iterations):
            current_pos = end_effector_position(self.dh_params, q.tolist())
            error = target - current_pos

            if np.linalg.norm(error) < self.tolerance:
                break

            J = jacobian(self.dh_params, q.tolist())
            J_pos = J[:3, :]

            dq = alpha * J_pos.T @ error
            q = q + dq

            if limits:
                for i, (lo, hi) in enumerate(limits):
                    q[i] = max(lo, min(hi, q[i]))

        return q

    def _solve_ccd(
        self,
        target: "np.ndarray",
        q: "np.ndarray",
        limits: Optional[List[Tuple[float, float]]],
    ) -> "np.ndarray":
        """Cyclic Coordinate Descent — iteratively rotates each joint toward target."""
        from stacc.kinematics.forward import forward_kinematics_chain

        for iteration in range(self.max_iterations):
            current_pos = end_effector_position(self.dh_params, q.tolist())
            if np.linalg.norm(target - current_pos) < self.tolerance:
                break

            # Iterate joints from tip to base
            for i in range(self.n_joints - 1, -1, -1):
                frames = forward_kinematics_chain(self.dh_params, q.tolist())
                ee_pos = frames[-1][:3, 3]

                if i == 0:
                    joint_pos = np.zeros(3)
                else:
                    joint_pos = frames[i - 1][:3, 3]

                # Vectors from joint to EE and joint to target
                to_ee = ee_pos - joint_pos
                to_target = target - joint_pos

                to_ee_norm = np.linalg.norm(to_ee)
                to_target_norm = np.linalg.norm(to_target)

                if to_ee_norm < 1e-10 or to_target_norm < 1e-10:
                    continue

                to_ee = to_ee / to_ee_norm
                to_target = to_target / to_target_norm

                cos_angle = np.clip(np.dot(to_ee, to_target), -1, 1)
                angle = math.acos(cos_angle)

                cross = np.cross(to_ee, to_target)
                if np.linalg.norm(cross) > 1e-10:
                    # Determine sign from cross product
                    if i == 0:
                        z_axis = np.array([0, 0, 1])
                    else:
                        z_axis = frames[i - 1][:3, 2]
                    sign = 1 if np.dot(cross, z_axis) > 0 else -1
                    q[i] += sign * angle

                if limits:
                    lo, hi = limits[i]
                    q[i] = max(lo, min(hi, q[i]))

        return q
