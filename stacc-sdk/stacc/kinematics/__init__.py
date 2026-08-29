"""stacc.kinematics — Forward/inverse kinematics, dynamics, Lie group transforms, and trajectory generation.

Provides:
- DH parameter transforms and SE(3) spatial transformations
- Lie Group SE(3)/SO(3) algebra, twists, wrenches, and adjoint transforms
- Product of Exponentials (PoE) spatial and body kinematics
- Forward kinematics (FK) and Inverse kinematics (IK) solvers (DLS, JT, CCD)
- Smooth trajectory generation (quintic, cubic, min-jerk, trapezoidal, linear, waypoints)
- Geometric, analytic, space, and body Jacobians
"""

from stacc.kinematics.transforms import (
    DH,
    rot_x, rot_y, rot_z,
    euler_to_rotation, rotation_to_euler,
    euler_to_quaternion, quaternion_to_euler,
    quaternion_to_rotation, quaternion_multiply, quaternion_normalize,
    homogeneous, transform_from_euler, transform_inverse, transform_point,
    axis_angle_to_rotation, rotation_to_axis_angle,
)
from stacc.kinematics.spatial import (
    skew, unskew,
    vec_to_se3, se3_to_vec,
    exp_so3, log_so3,
    exp_se3, log_se3,
    adjoint_transform,
    spatial_cross_twist, spatial_cross_wrench,
    product_of_exponentials_spatial, product_of_exponentials_body,
    spatial_jacobian, body_jacobian,
)
from stacc.kinematics.forward import (
    forward_kinematics_dh, forward_kinematics_chain,
    end_effector_position, end_effector_rotation,
    jacobian,
)
from stacc.kinematics.inverse import InverseKinematics
from stacc.kinematics.trajectory import (
    QuinticTrajectory, CubicTrajectory, MinJerkTrajectory,
    TrapezoidalProfile, LinearTrajectory,
    WaypointPath, TrajectoryExecutor,
)

__all__ = [
    "DH",
    "rot_x", "rot_y", "rot_z",
    "euler_to_rotation", "rotation_to_euler",
    "euler_to_quaternion", "quaternion_to_euler",
    "quaternion_to_rotation", "quaternion_multiply", "quaternion_normalize",
    "homogeneous", "transform_from_euler", "transform_inverse", "transform_point",
    "axis_angle_to_rotation", "rotation_to_axis_angle",
    "skew", "unskew",
    "vec_to_se3", "se3_to_vec",
    "exp_so3", "log_so3",
    "exp_se3", "log_se3",
    "adjoint_transform",
    "spatial_cross_twist", "spatial_cross_wrench",
    "product_of_exponentials_spatial", "product_of_exponentials_body",
    "spatial_jacobian", "body_jacobian",
    "forward_kinematics_dh", "forward_kinematics_chain",
    "end_effector_position", "end_effector_rotation",
    "jacobian",
    "InverseKinematics",
    "QuinticTrajectory", "CubicTrajectory", "MinJerkTrajectory",
    "TrapezoidalProfile", "LinearTrajectory",
    "WaypointPath", "TrajectoryExecutor",
]
