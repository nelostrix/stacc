"""SE(3) spatial transformations.

Provides rotation matrices, quaternions, Euler angles, and homogeneous
transformation matrices for robotics kinematics.

All angles in radians. Rotation order is ZYX (yaw-pitch-roll) by default.
"""
from __future__ import annotations
import math
from typing import List, Tuple, Optional

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


def _require_numpy() -> None:
    if not HAS_NUMPY:
        raise ImportError(
            "numpy is required for transforms. Install with: pip install numpy"
        )


# ── Rotation matrices ──

def rot_x(angle: float) -> "np.ndarray":
    """Rotation matrix about X axis.

    Args:
        angle: Rotation angle in radians

    Returns:
        3x3 rotation matrix
    """
    _require_numpy()
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c],
    ])


def rot_y(angle: float) -> "np.ndarray":
    """Rotation matrix about Y axis."""
    _require_numpy()
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c],
    ])


def rot_z(angle: float) -> "np.ndarray":
    """Rotation matrix about Z axis."""
    _require_numpy()
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ])


def euler_to_rotation(roll: float, pitch: float, yaw: float) -> "np.ndarray":
    """Convert Euler angles (ZYX convention) to 3x3 rotation matrix."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def rotation_to_euler(R: "np.ndarray") -> Tuple[float, float, float]:
    """Extract Euler angles (roll, pitch, yaw) from rotation matrix."""
    _require_numpy()
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


# ── Quaternions (w, x, y, z convention) ──

def euler_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """Convert Euler angles to quaternion (w, x, y, z)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z


def quaternion_to_euler(
    w: float, x: float, y: float, z: float
) -> Tuple[float, float, float]:
    """Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_to_rotation(
    w: float, x: float, y: float, z: float
) -> "np.ndarray":
    """Convert quaternion to 3x3 rotation matrix."""
    _require_numpy()
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),       1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x**2 + y**2)],
    ])


def quaternion_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Hamilton product of two quaternions (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


def quaternion_normalize(
    w: float, x: float, y: float, z: float
) -> Tuple[float, float, float, float]:
    """Normalize quaternion to unit length."""
    norm = math.sqrt(w**2 + x**2 + y**2 + z**2)
    if norm < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


# ── Homogeneous transforms (4x4) ──

def homogeneous(R: "np.ndarray", t: "np.ndarray") -> "np.ndarray":
    """Build 4x4 homogeneous transform from rotation and translation.

    Args:
        R: 3x3 rotation matrix
        t: 3-element translation vector

    Returns:
        4x4 homogeneous transformation matrix
    """
    _require_numpy()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def transform_from_euler(
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
) -> "np.ndarray":
    """Build 4x4 transform from position + Euler angles."""
    _require_numpy()
    R = euler_to_rotation(roll, pitch, yaw)
    return homogeneous(R, np.array([x, y, z]))


def transform_inverse(T: "np.ndarray") -> "np.ndarray":
    """Compute inverse of a homogeneous transform (faster than np.linalg.inv).

    Uses the property that [R t; 0 1]⁻¹ = [Rᵀ  -Rᵀt; 0 1].
    """
    _require_numpy()
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def transform_point(T: "np.ndarray", point: "np.ndarray") -> "np.ndarray":
    """Apply a homogeneous transform to a 3D point.

    Args:
        T: 4x4 homogeneous transformation matrix
        point: 3-element point [x, y, z]

    Returns:
        Transformed 3-element point
    """
    _require_numpy()
    p = np.append(point, 1.0)
    return (T @ p)[:3]


# ── DH (Denavit-Hartenberg) ──

class DH:
    """Denavit-Hartenberg parameters for a single joint.

    Standard DH convention:
        a:     link length (along x_{i-1})
        alpha: link twist (about x_{i-1})
        d:     link offset (along z_i)
        theta_offset: joint angle offset

    Usage:
        dh = DH(a=1.0, alpha=0.5, d=0.2, theta_offset=0.1)
        T = dh.transform(theta=0.5)
    """

    def __init__(
        self,
        a: float = 0.0,
        alpha: float = 0.0,
        d: float = 0.0,
        theta_offset: float = 0.0,
    ):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta_offset = theta_offset

    @classmethod
    def from_param_list(
        cls, params: List[Tuple[float, float, float, float]]
    ) -> List[DH]:
        """Create a list of DH parameters from a list of (a, alpha, d, theta_offset) tuples.

        Args:
            params: List of (a, alpha, d, theta_offset) tuples

        Returns:
            List of DH objects
        """
        return [cls(a, alpha, d, offset) for a, alpha, d, offset in params]

    def transform(self, theta: float) -> "np.ndarray":
        """Compute the 4x4 transform for this DH frame at joint angle theta.

        Args:
            theta: Joint angle in radians

        Returns:
            4x4 homogeneous transformation matrix
        """
        _require_numpy()
        t = theta + self.theta_offset
        ct, st = math.cos(t), math.sin(t)
        ca, sa = math.cos(self.alpha), math.sin(self.alpha)

        return np.array([
            [ct, -st * ca,  st * sa, self.a * ct],
            [st,  ct * ca, -ct * sa, self.a * st],
            [0,   sa,       ca,      self.d],
            [0,   0,        0,       1],
        ])

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, DH):
            return NotImplemented
        return (
            math.isclose(self.a, other.a)
            and math.isclose(self.alpha, other.alpha)
            and math.isclose(self.d, other.d)
            and math.isclose(self.theta_offset, other.theta_offset)
        )

    def __repr__(self) -> str:
        return (
            f"DH(a={self.a}, alpha={self.alpha:.3f}, "
            f"d={self.d}, offset={self.theta_offset:.3f})"
        )


# ── Axis-angle ──

def axis_angle_to_rotation(axis: List[float], angle: float) -> "np.ndarray":
    """Convert axis-angle representation to rotation matrix (Rodrigues formula)."""
    _require_numpy()
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-10:
        return np.eye(3)
    axis = axis / norm
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def rotation_to_axis_angle(R: "np.ndarray") -> Tuple[List[float], float]:
    """Extract axis-angle from rotation matrix."""
    _require_numpy()
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if abs(angle) < 1e-10:
        return [0.0, 0.0, 1.0], 0.0
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2.0 * math.sin(angle))
    return axis.tolist(), angle
