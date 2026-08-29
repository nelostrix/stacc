"""Spatial Vector Algebra and Lie Group SE(3)/SO(3) kinematics.

Provides mathematically rigorous Lie Group formulations for modern robotics:
- Skew-symmetric matrices so(3) and se(3)
- Matrix exponential and logarithm on SO(3) and SE(3) (Rodrigues formula)
- Spatial Twists (linear & angular velocities) and Wrenches (forces & torques)
- Adjoint transforms Ad_T and spatial cross products ad_V, ad_V^*
- Product of Exponentials (PoE) forward kinematics for spatial and body frames
- Spatial and Body Jacobians
"""

from __future__ import annotations
import math
from typing import List, Optional, Tuple, Union

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


def skew(v: Union[List[float], "np.ndarray"]) -> "np.ndarray":
    """Convert a 3D vector into a 3x3 skew-symmetric matrix [v] in so(3).

    Args:
        v: 3D vector [x, y, z]

    Returns:
        3x3 numpy array
    """
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    x, y, z = v[0], v[1], v[2]
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0]
    ], dtype=np.float64)


def unskew(omega_hat: "np.ndarray") -> "np.ndarray":
    """Extract a 3D vector from a 3x3 skew-symmetric matrix.

    Args:
        omega_hat: 3x3 skew-symmetric matrix

    Returns:
        3D numpy array [x, y, z]
    """
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    return np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]], dtype=np.float64)


def vec_to_se3(twist: Union[List[float], "np.ndarray"]) -> "np.ndarray":
    """Convert a 6D spatial twist V = [omega, v]^T into a 4x4 se(3) matrix."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    omega = np.asarray(twist[:3], dtype=np.float64)
    v = np.asarray(twist[3:6], dtype=np.float64)
    se3 = np.zeros((4, 4), dtype=np.float64)
    se3[:3, :3] = skew(omega)
    se3[:3, 3] = v
    return se3


def se3_to_vec(se3: "np.ndarray") -> "np.ndarray":
    """Convert a 4x4 se(3) matrix into a 6D twist vector [omega, v]."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    omega = unskew(se3[:3, :3])
    v = se3[:3, 3]
    return np.concatenate([omega, v])


def exp_so3(omega: Union[List[float], "np.ndarray"], theta: Optional[float] = None) -> "np.ndarray":
    """Compute the matrix exponential exp([omega] * theta) in SO(3) using Rodrigues formula."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    omega = np.asarray(omega, dtype=np.float64)
    if theta is None:
        norm = np.linalg.norm(omega)
        if norm < 1e-12:
            return np.eye(3, dtype=np.float64)
        theta = norm
        omega_unit = omega / norm
    else:
        norm = np.linalg.norm(omega)
        omega_unit = omega / norm if norm > 1e-12 else omega

    w_hat = skew(omega_unit)
    return np.eye(3, dtype=np.float64) + math.sin(theta) * w_hat + (1.0 - math.cos(theta)) * (w_hat @ w_hat)


def log_so3(R: "np.ndarray") -> Tuple["np.ndarray", float]:
    """Compute the matrix logarithm of R in SO(3), returning (axis, angle)."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    trace = np.trace(R)
    cos_theta = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    theta = math.acos(cos_theta)

    if theta < 1e-10:
        return np.array([0.0, 0.0, 1.0]), 0.0
    elif math.isclose(theta, math.pi, abs_tol=1e-5):
        diag = np.diag(R)
        k = np.argmax(diag)
        if k == 0:
            axis = np.array([R[0, 0] + 1.0, R[1, 0], R[2, 0]])
        elif k == 1:
            axis = np.array([R[0, 1], R[1, 1] + 1.0, R[2, 1]])
        else:
            axis = np.array([R[0, 2], R[1, 2], R[2, 2] + 1.0])
        norm = np.linalg.norm(axis)
        return axis / norm if norm > 1e-12 else np.array([0.0, 0.0, 1.0]), theta
    else:
        w_hat = (R - R.T) / (2.0 * math.sin(theta))
        return unskew(w_hat), theta


def exp_se3(twist: Union[List[float], "np.ndarray"], theta: Optional[float] = None) -> "np.ndarray":
    """Compute matrix exponential exp([S] * theta) in SE(3)."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    twist = np.asarray(twist, dtype=np.float64)
    omega = twist[:3]
    v = twist[3:6]

    if theta is None:
        norm = np.linalg.norm(omega)
        if norm > 1e-12:
            theta = norm
            omega = omega / norm
            v = v / norm
        else:
            v_norm = np.linalg.norm(v)
            if v_norm < 1e-12:
                return np.eye(4, dtype=np.float64)
            theta = v_norm
            v = v / v_norm

    w_norm = np.linalg.norm(omega)
    T = np.eye(4, dtype=np.float64)

    if w_norm < 1e-12:
        T[:3, 3] = v * theta
        return T

    w_hat = skew(omega)
    R = exp_so3(omega, theta)
    G = np.eye(3) * theta + (1.0 - math.cos(theta)) * w_hat + (theta - math.sin(theta)) * (w_hat @ w_hat)
    p = G @ v

    T[:3, :3] = R
    T[:3, 3] = p
    return T


def log_se3(T: "np.ndarray") -> Tuple["np.ndarray", float]:
    """Compute the matrix logarithm of T in SE(3), returning (screw_axis, theta)."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    R = T[:3, :3]
    p = T[:3, 3]

    omega, theta = log_so3(R)

    if theta < 1e-10:
        p_norm = np.linalg.norm(p)
        if p_norm < 1e-12:
            return np.zeros(6), 0.0
        return np.concatenate([np.zeros(3), p / p_norm]), p_norm

    w_hat = skew(omega)
    G_inv = (np.eye(3) / theta - 0.5 * w_hat +
             (1.0 / theta - 0.5 / math.tan(theta / 2.0)) * (w_hat @ w_hat))
    v = G_inv @ p
    return np.concatenate([omega, v]), theta


def adjoint_transform(T: "np.ndarray") -> "np.ndarray":
    """Compute the 6x6 Adjoint representation Ad_T of a homogeneous transformation matrix T."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    R = T[:3, :3]
    p = T[:3, 3]
    Ad = np.zeros((6, 6), dtype=np.float64)
    Ad[:3, :3] = R
    Ad[3:6, 3:6] = R
    Ad[3:6, :3] = skew(p) @ R
    return Ad


def spatial_cross_twist(V1: "np.ndarray") -> "np.ndarray":
    """Compute the 6x6 Lie bracket / spatial cross product matrix [ad_V1] for twists."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    w1 = V1[:3]
    v1 = V1[3:6]
    ad = np.zeros((6, 6), dtype=np.float64)
    ad[:3, :3] = skew(w1)
    ad[3:6, 3:6] = skew(w1)
    ad[3:6, :3] = skew(v1)
    return ad


def spatial_cross_wrench(V: "np.ndarray") -> "np.ndarray":
    """Compute the 6x6 co-adjoint spatial cross product matrix [ad_V^*] for wrenches."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    return -spatial_cross_twist(V).T


def product_of_exponentials_spatial(
    screw_axes: List["np.ndarray"],
    joint_angles: List[float],
    M: "np.ndarray"
) -> "np.ndarray":
    """Compute Forward Kinematics in Space Frame using Product of Exponentials (PoE)."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    T = np.eye(4, dtype=np.float64)
    for S, q in zip(screw_axes, joint_angles):
        T = T @ exp_se3(S, q)
    return T @ M


def product_of_exponentials_body(
    screw_axes_body: List["np.ndarray"],
    joint_angles: List[float],
    M: "np.ndarray"
) -> "np.ndarray":
    """Compute Forward Kinematics in Body Frame using Product of Exponentials (PoE)."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    T = M.copy()
    for B, q in zip(screw_axes_body, joint_angles):
        T = T @ exp_se3(B, q)
    return T


def spatial_jacobian(
    screw_axes: List["np.ndarray"],
    joint_angles: List[float]
) -> "np.ndarray":
    """Compute the 6xn Space Jacobian J_s(q) using Product of Exponentials."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    n = len(joint_angles)
    Js = np.zeros((6, n), dtype=np.float64)
    Js[:, 0] = screw_axes[0]

    T = np.eye(4, dtype=np.float64)
    for i in range(1, n):
        T = T @ exp_se3(screw_axes[i - 1], joint_angles[i - 1])
        Js[:, i] = adjoint_transform(T) @ screw_axes[i]

    return Js


def body_jacobian(
    screw_axes_body: List["np.ndarray"],
    joint_angles: List[float]
) -> "np.ndarray":
    """Compute the 6xn Body Jacobian J_b(q) using Product of Exponentials."""
    if not HAS_NUMPY:
        raise ImportError("numpy is required for spatial kinematics")
    n = len(joint_angles)
    Jb = np.zeros((6, n), dtype=np.float64)
    Jb[:, -1] = screw_axes_body[-1]

    T = np.eye(4, dtype=np.float64)
    for i in range(n - 2, -1, -1):
        T = T @ exp_se3(-screw_axes_body[i + 1], joint_angles[i + 1])
        Jb[:, i] = adjoint_transform(T) @ screw_axes_body[i]

    return Jb
