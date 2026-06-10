"""Forward kinematics — compute end-effector pose from joint angles.

Supports DH parameter chains, product of exponentials, and general
serial kinematic chains.
"""
from __future__ import annotations
from typing import List, Optional, TYPE_CHECKING

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

from stacc.kinematics.transforms import DH


def forward_kinematics_dh(dh_params: List[DH], joint_angles: List[float]) -> "np.ndarray":
    """Compute FK using DH parameters.

    Args:
        dh_params: List of DH parameter objects, one per joint
        joint_angles: Current joint angles in radians

    Returns:
        4x4 homogeneous transform of the end-effector frame
    """
    if not HAS_NUMPY:
        raise ImportError("numpy required for FK")
    if len(dh_params) != len(joint_angles):
        raise ValueError(
            f"DH params ({len(dh_params)}) and joint angles ({len(joint_angles)}) must match"
        )

    T = np.eye(4)
    for dh, theta in zip(dh_params, joint_angles):
        T = T @ dh.transform(theta)
    return T


def forward_kinematics_chain(dh_params: List[DH], joint_angles: List[float]) -> List["np.ndarray"]:
    """Compute FK for all frames in the chain (not just end-effector).

    Returns:
        List of 4x4 transforms, one per joint (T_0_1, T_0_2, ..., T_0_n)
    """
    if not HAS_NUMPY:
        raise ImportError("numpy required for FK")

    frames = []
    T = np.eye(4)
    for dh, theta in zip(dh_params, joint_angles):
        T = T @ dh.transform(theta)
        frames.append(T.copy())
    return frames


def end_effector_position(dh_params: List[DH], joint_angles: List[float]) -> "np.ndarray":
    """Get just the xyz position of the end effector."""
    T = forward_kinematics_dh(dh_params, joint_angles)
    return T[:3, 3]


def end_effector_rotation(dh_params: List[DH], joint_angles: List[float]) -> "np.ndarray":
    """Get the 3x3 rotation matrix of the end effector."""
    T = forward_kinematics_dh(dh_params, joint_angles)
    return T[:3, :3]


def jacobian(dh_params: List[DH], joint_angles: List[float]) -> "np.ndarray":
    """Compute the 6×n geometric Jacobian matrix.

    The Jacobian maps joint velocities to end-effector spatial velocity:
        [v_x, v_y, v_z, w_x, w_y, w_z]^T = J * [dq_1, ..., dq_n]^T

    Returns:
        6×n numpy array
    """
    if not HAS_NUMPY:
        raise ImportError("numpy required for Jacobian")

    n = len(joint_angles)
    frames = forward_kinematics_chain(dh_params, joint_angles)
    T_ee = frames[-1]
    p_ee = T_ee[:3, 3]

    J = np.zeros((6, n))

    for i in range(n):
        # Get transform to frame i
        if i == 0:
            T_i = np.eye(4)
        else:
            T_i = frames[i - 1]

        z_i = T_i[:3, 2]  # z-axis of frame i
        p_i = T_i[:3, 3]  # origin of frame i

        # For revolute joints
        J[:3, i] = np.cross(z_i, p_ee - p_i)  # linear velocity
        J[3:, i] = z_i                          # angular velocity

    return J
