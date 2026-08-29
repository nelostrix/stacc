"""Manipulability and Singularity Diagnostics.

Calculates Yoshikawa manipulability index, velocity/force manipulability ellipsoids,
and condition numbers to detect kinematic singularities and optimize workspace trajectories.
"""

from __future__ import annotations
from typing import Dict, Tuple, Union
import math
import numpy as np


def yoshikawa_manipulability(J: np.ndarray) -> float:
    """Compute the Yoshikawa scalar manipulability measure w = sqrt(det(J * J^T)).

    Args:
        J: 6xn (or 3xn) geometric or spatial Jacobian

    Returns:
        w: non-negative scalar measure (0 indicates a singular configuration)
    """
    JJT = J @ J.T
    det = np.linalg.det(JJT)
    return math.sqrt(max(0.0, float(det)))


def manipulability_ellipsoid(
    J: np.ndarray,
    space: str = "translation"
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute the principal axes and radii of the velocity manipulability ellipsoid.

    Args:
        J: 6xn Jacobian
        space: "translation" (uses linear velocity top 3 rows) or "rotation" (bottom 3 rows)

    Returns:
        axes: 3x3 matrix whose columns are the principal ellipsoid directions
        radii: 3-vector of singular values (lengths of principal semi-axes)
    """
    if space == "translation":
        J_sub = J[3:6, :] if J.shape[0] == 6 else J[:3, :]
    elif space == "rotation":
        J_sub = J[:3, :]
    else:
        J_sub = J

    # SVD of sub-Jacobian: J_sub = U * S * V^T
    U, S, Vt = np.linalg.svd(J_sub)
    return U, S


def jacobian_condition_number(J: np.ndarray) -> float:
    """Compute the condition number kappa = sigma_max / sigma_min of the Jacobian.

    Value of 1 indicates isotropic velocity transmission; infinity indicates singularity.
    """
    U, S, Vt = np.linalg.svd(J)
    sigma_max = S[0]
    sigma_min = S[-1] if len(S) > 0 else 0.0
    if sigma_min < 1e-12:
        return float("inf")
    return float(sigma_max / sigma_min)


def singularity_robust_inverse(
    J: np.ndarray,
    damping_max: float = 0.05,
    threshold: float = 0.02
) -> np.ndarray:
    """Compute the Singularity-Robust Damped Least Squares inverse J* of a Jacobian.

    J* = J^T * (J * J^T + lambda^2 * I)^-1
    where lambda smoothly ramps up when approaching kinematic singularities.
    """
    m, n = J.shape
    w = yoshikawa_manipulability(J)

    if w < threshold:
        lam = damping_max * (1.0 - (w / threshold) ** 2)
    else:
        lam = 0.0

    JJT = J @ J.T
    return J.T @ np.linalg.inv(JJT + (lam ** 2) * np.eye(m))
