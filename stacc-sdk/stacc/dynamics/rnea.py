"""Recursive Newton-Euler Algorithm (RNEA) for Inverse Dynamics.

Computes joint torques tau = M(q) * q_ddot + C(q, q_dot) * q_dot + G(q) + J^T * F_ext
in linear O(n) computational complexity.
"""

from __future__ import annotations
from typing import List, Optional, Tuple, Union
import math

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

from stacc.kinematics.spatial import (
    exp_se3,
    adjoint_transform,
    spatial_cross_twist,
    spatial_cross_wrench,
)


class SpatialInertia:
    """Spatial 6x6 Inertia Tensor for a rigid link.

    I_spatial = [[ I_c - m * [c]^2,   m * [c] ],
                 [ -m * [c],          m * eye(3) ]]

    where m is mass, c is center of mass vector, I_c is rotational inertia about COM.
    """

    def __init__(
        self,
        mass: float,
        com: Union[List[float], "np.ndarray"],
        inertia_tensor: Union[List[List[float]], "np.ndarray"]
    ):
        if not HAS_NUMPY:
            raise ImportError("numpy required for SpatialInertia")
        self.mass = float(mass)
        self.com = np.asarray(com, dtype=np.float64)
        self.I_c = np.asarray(inertia_tensor, dtype=np.float64)

        c_x, c_y, c_z = self.com
        c_skew = np.array([
            [0.0, -c_z, c_y],
            [c_z, 0.0, -c_x],
            [-c_y, c_x, 0.0]
        ], dtype=np.float64)

        # 6x6 Spatial Inertia Matrix
        self.matrix = np.zeros((6, 6), dtype=np.float64)
        self.matrix[:3, :3] = self.I_c - self.mass * (c_skew @ c_skew)
        self.matrix[:3, 3:6] = self.mass * c_skew
        self.matrix[3:6, :3] = -self.mass * c_skew
        self.matrix[3:6, 3:6] = self.mass * np.eye(3, dtype=np.float64)


def rnea_inverse_dynamics(
    q: Union[List[float], "np.ndarray"],
    q_dot: Union[List[float], "np.ndarray"],
    q_ddot: Union[List[float], "np.ndarray"],
    screw_axes_body: List["np.ndarray"],
    M_list: List["np.ndarray"],
    spatial_inertias: List[SpatialInertia],
    gravity: Optional[Union[List[float], "np.ndarray"]] = None,
    f_ext: Optional[List["np.ndarray"]] = None
) -> "np.ndarray":
    """Compute joint torques using Recursive Newton-Euler Algorithm (RNEA).

    Args:
        q: Joint positions (n-vector)
        q_dot: Joint velocities (n-vector)
        q_ddot: Joint accelerations (n-vector)
        screw_axes_body: List of 6D screw axes in body frame (n items)
        M_list: List of 4x4 home configuration transforms from link (i-1) to link i (n+1 items)
        spatial_inertias: SpatialInertia for each link (n items)
        gravity: Base gravitational acceleration vector [gx, gy, gz] (default [0, 0, -9.81])
        f_ext: External spatial wrenches applied on each link (optional list of n 6D vectors)

    Returns:
        tau: Joint torques (n-vector)
    """
    if not HAS_NUMPY:
        raise ImportError("numpy required for RNEA")

    q = np.asarray(q, dtype=np.float64)
    q_dot = np.asarray(q_dot, dtype=np.float64)
    q_ddot = np.asarray(q_ddot, dtype=np.float64)
    n = len(q)

    if gravity is None:
        g = np.array([0.0, 0.0, 0.0, 0.0, 0.0, -9.81], dtype=np.float64)
    else:
        grav = np.asarray(gravity, dtype=np.float64)
        if len(grav) == 3:
            g = np.concatenate([np.zeros(3), grav])
        else:
            g = grav

    # Pre-allocate velocities and accelerations
    V = [np.zeros(6, dtype=np.float64) for _ in range(n + 1)]
    V_dot = [np.zeros(6, dtype=np.float64) for _ in range(n + 1)]
    V_dot[0] = -g  # Base acceleration set to fictitious -g to handle gravity

    # Precompute relative transformations
    T_rel = []
    for i in range(n):
        # T_{i-1, i} = M_list[i] * exp(B_i * q_i)
        T_i = M_list[i] @ exp_se3(screw_axes_body[i], q[i])
        T_rel.append(T_i)

    # 1. Forward Recursion: Kinematics (Link 1 to n)
    for i in range(n):
        idx = i + 1
        A_inv = adjoint_transform(np.linalg.inv(T_rel[i]))
        A_inv_V = A_inv @ V[idx - 1]
        S_i = screw_axes_body[i]

        # Velocity: V_i = Ad_{T_{i-1, i}^-1} * V_{i-1} + S_i * q_dot_i
        V[idx] = A_inv_V + S_i * q_dot[i]

        # Acceleration: V_dot_i = Ad_{T_{i-1, i}^-1} * V_dot_{i-1} + S_i * q_ddot_i + ad_{V_i} * S_i * q_dot_i
        ad_V_S = spatial_cross_twist(V[idx]) @ S_i
        V_dot[idx] = A_inv @ V_dot[idx - 1] + S_i * q_ddot[i] + ad_V_S * q_dot[i]

    # 2. Backward Recursion: Forces and Torques (Link n down to 1)
    F = [np.zeros(6, dtype=np.float64) for _ in range(n + 1)]
    tau = np.zeros(n, dtype=np.float64)

    # Last frame transform T_{n, n+1} if provided, otherwise identity
    T_n_plus_1 = M_list[n] if len(M_list) > n else np.eye(4)
    if f_ext is not None and len(f_ext) > n:
        F[n] = f_ext[n]

    for i in range(n - 1, -1, -1):
        idx = i + 1
        I_i = spatial_inertias[i].matrix

        # Net body spatial wrench: F_i = I_i * V_dot_i - ad_{V_i}^* * I_i * V_i
        ad_V_star = spatial_cross_wrench(V[idx])
        F_body = I_i @ V_dot[idx] - ad_V_star @ (I_i @ V[idx])

        # Add external wrench if present
        if f_ext is not None and i < len(f_ext) and f_ext[i] is not None:
            F_body += np.asarray(f_ext[i], dtype=np.float64)

        # Transmitted wrench from child link (i+1)
        if idx < n:
            A_next = adjoint_transform(np.linalg.inv(T_rel[idx]))
            F[idx] = F_body + A_next.T @ F[idx + 1]
        else:
            F[idx] = F_body

        # Joint torque projected onto screw axis: tau_i = F_i^T * S_i
        tau[i] = np.dot(F[idx], screw_axes_body[i])

    return tau


def compute_gravity_vector(
    q: Union[List[float], "np.ndarray"],
    screw_axes_body: List["np.ndarray"],
    M_list: List["np.ndarray"],
    spatial_inertias: List[SpatialInertia],
    gravity: Optional[Union[List[float], "np.ndarray"]] = None
) -> "np.ndarray":
    """Compute the gravity torque vector G(q) by calling RNEA with zero velocity and acceleration."""
    n = len(q)
    zeros = np.zeros(n, dtype=np.float64)
    return rnea_inverse_dynamics(
        q=q,
        q_dot=zeros,
        q_ddot=zeros,
        screw_axes_body=screw_axes_body,
        M_list=M_list,
        spatial_inertias=spatial_inertias,
        gravity=gravity
    )


def compute_coriolis_vector(
    q: Union[List[float], "np.ndarray"],
    q_dot: Union[List[float], "np.ndarray"],
    screw_axes_body: List["np.ndarray"],
    M_list: List["np.ndarray"],
    spatial_inertias: List[SpatialInertia]
) -> "np.ndarray":
    """Compute Coriolis and Centrifugal forces C(q, q_dot) * q_dot by running RNEA with zero gravity."""
    n = len(q)
    zeros = np.zeros(n, dtype=np.float64)
    return rnea_inverse_dynamics(
        q=q,
        q_dot=q_dot,
        q_ddot=zeros,
        screw_axes_body=screw_axes_body,
        M_list=M_list,
        spatial_inertias=spatial_inertias,
        gravity=np.zeros(3)
    )
