"""Composite Rigid Body Algorithm (CRBA) for Joint Space Inertia Matrix M(q).

Efficiently calculates the symmetric positive-definite generalized mass matrix M(q)
for serial multi-body robotic kinematic chains.
"""

from __future__ import annotations
from typing import List, Union
import numpy as np

from stacc.kinematics.spatial import exp_se3, adjoint_transform
from stacc.dynamics.rnea import SpatialInertia


def crba_mass_matrix(
    q: Union[List[float], np.ndarray],
    screw_axes_body: List[np.ndarray],
    M_list: List[np.ndarray],
    spatial_inertias: List[SpatialInertia]
) -> np.ndarray:
    """Compute the joint-space inertia matrix M(q) using Composite Rigid Body Algorithm.

    Args:
        q: Joint angle vector (n items)
        screw_axes_body: List of 6D screw axes in body frame (n items)
        M_list: List of 4x4 relative home configuration transforms (n items)
        spatial_inertias: SpatialInertia objects for each link (n items)

    Returns:
        M: n x n symmetric, positive-definite inertia matrix
    """
    q = np.asarray(q, dtype=np.float64)
    n = len(q)

    # 1. Initialize composite spatial inertias I_c with individual link inertias
    I_c = [spatial_inertias[i].matrix.copy() for i in range(n)]

    # Compute relative transformations T_{i-1, i}
    T_rel = []
    for i in range(n):
        T_i = M_list[i] @ exp_se3(screw_axes_body[i], q[i])
        T_rel.append(T_i)

    # 2. Backward accumulation of composite inertias
    for i in range(n - 1, 0, -1):
        # Transform inertia of body i into body (i-1) frame
        A_inv = adjoint_transform(np.linalg.inv(T_rel[i]))
        I_c[i - 1] += A_inv.T @ I_c[i] @ A_inv

    # 3. Compute M(q) entries
    M = np.zeros((n, n), dtype=np.float64)

    for i in range(n):
        S_i = screw_axes_body[i]
        # Diagonal term: M_ii = S_i^T * I_c[i] * S_i
        F = I_c[i] @ S_i
        M[i, i] = np.dot(S_i, F)

        # Off-diagonal terms M_ij = M_ji
        for j in range(i + 1, n):
            # Accumulate adjoint transform from body j to body i
            T_i_to_j = np.eye(4)
            for k in range(i + 1, j + 1):
                T_i_to_j = T_i_to_j @ T_rel[k]
            A_ij = adjoint_transform(np.linalg.inv(T_i_to_j))
            
            # F_j projected back into body i frame
            M[i, j] = np.dot(S_i, A_ij.T @ (I_c[j] @ screw_axes_body[j]))
            M[j, i] = M[i, j]

    return M
