"""Articulated Body Algorithm (ABA) for Forward Dynamics.

Computes joint accelerations q_ddot = M(q)^-1 * (tau - C(q, q_dot)*q_dot - G(q))
in O(n) computational time without explicitly forming or inverting the mass matrix M(q).
"""

from __future__ import annotations
from typing import List, Optional, Union
import numpy as np

from stacc.kinematics.spatial import (
    exp_se3,
    adjoint_transform,
    spatial_cross_twist,
    spatial_cross_wrench,
)
from stacc.dynamics.rnea import SpatialInertia


def aba_forward_dynamics(
    q: Union[List[float], np.ndarray],
    q_dot: Union[List[float], np.ndarray],
    tau: Union[List[float], np.ndarray],
    screw_axes_body: List[np.ndarray],
    M_list: List[np.ndarray],
    spatial_inertias: List[SpatialInertia],
    gravity: Optional[Union[List[float], np.ndarray]] = None,
    f_ext: Optional[List[np.ndarray]] = None
) -> np.ndarray:
    """Compute joint accelerations q_ddot in O(n) using Articulated Body Algorithm.

    Args:
        q: Joint positions (n items)
        q_dot: Joint velocities (n items)
        tau: Applied joint torques (n items)
        screw_axes_body: List of 6D screw axes in body frame (n items)
        M_list: List of 4x4 relative home configuration transforms (n items)
        spatial_inertias: SpatialInertia objects for each link (n items)
        gravity: Base gravity acceleration [gx, gy, gz] (default [0, 0, -9.81])
        f_ext: External wrenches on each link (optional)

    Returns:
        q_ddot: Joint accelerations (n items)
    """
    q = np.asarray(q, dtype=np.float64)
    q_dot = np.asarray(q_dot, dtype=np.float64)
    tau = np.asarray(tau, dtype=np.float64)
    n = len(q)

    if gravity is None:
        g = np.array([0.0, 0.0, 0.0, 0.0, 0.0, -9.81], dtype=np.float64)
    else:
        grav = np.asarray(gravity, dtype=np.float64)
        g = np.concatenate([np.zeros(3), grav]) if len(grav) == 3 else grav

    # 1. Forward Pass: compute body velocities V_i and bias accelerations c_i
    V = [np.zeros(6, dtype=np.float64) for _ in range(n + 1)]
    c = [np.zeros(6, dtype=np.float64) for _ in range(n + 1)]
    T_rel = []

    for i in range(n):
        idx = i + 1
        T_i = M_list[i] @ exp_se3(screw_axes_body[i], q[i])
        T_rel.append(T_i)
        A_inv = adjoint_transform(np.linalg.inv(T_i))

        S_i = screw_axes_body[i]
        V[idx] = A_inv @ V[idx - 1] + S_i * q_dot[i]
        c[idx] = spatial_cross_twist(V[idx]) @ S_i * q_dot[i]

    # 2. Backward Pass: compute Articulated Body Inertias I_A and bias forces p_A
    I_A = [spatial_inertias[i].matrix.copy() for i in range(n)]
    p_A = [np.zeros(6, dtype=np.float64) for _ in range(n)]

    for i in range(n):
        idx = i + 1
        ad_star = spatial_cross_wrench(V[idx])
        p_A[i] = -ad_star @ (I_A[i] @ V[idx])
        if f_ext is not None and i < len(f_ext) and f_ext[i] is not None:
            p_A[i] -= np.asarray(f_ext[i], dtype=np.float64)

    U = [np.zeros(6, dtype=np.float64) for _ in range(n)]
    d = np.zeros(n, dtype=np.float64)
    u = np.zeros(n, dtype=np.float64)

    for i in range(n - 1, -1, -1):
        S_i = screw_axes_body[i]
        U[i] = I_A[i] @ S_i
        d[i] = np.dot(S_i, U[i])
        u[i] = tau[i] - np.dot(S_i, p_A[i])

        if i > 0:
            A_inv = adjoint_transform(np.linalg.inv(T_rel[i]))
            I_a_link = I_A[i] - np.outer(U[i], U[i]) / (d[i] if abs(d[i]) > 1e-12 else 1e-12)
            p_a_link = p_A[i] + I_a_link @ c[i + 1] + U[i] * (u[i] / (d[i] if abs(d[i]) > 1e-12 else 1e-12))
            I_A[i - 1] += A_inv.T @ I_a_link @ A_inv
            p_A[i - 1] += A_inv.T @ p_a_link

    # 3. Second Forward Pass: compute joint accelerations q_ddot
    V_dot = [np.zeros(6, dtype=np.float64) for _ in range(n + 1)]
    V_dot[0] = -g
    q_ddot = np.zeros(n, dtype=np.float64)

    for i in range(n):
        idx = i + 1
        A_inv = adjoint_transform(np.linalg.inv(T_rel[i]))
        a_prime = A_inv @ V_dot[idx - 1] + c[idx]
        d_val = d[i] if abs(d[i]) > 1e-12 else 1e-12
        q_ddot[i] = (u[i] - np.dot(U[i], a_prime)) / d_val
        V_dot[idx] = a_prime + screw_axes_body[i] * q_ddot[i]

    return q_ddot
