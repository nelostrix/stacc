"""stacc.kinematics — Forward/inverse kinematics, dynamics, transforms, and trajectory generation.

Provides:
- DH parameter transforms and SE(3) spatial transformations
- Forward kinematics (FK) for serial manipulators
- Inverse kinematics (IK) solvers (DLS, JT, CCD)
- Smooth trajectory generation (quintic, cubic, min-jerk, trapezoidal, linear, waypoints)
- Geometric and analytic Jacobians

Usage:
    from stacc.kinematics import (
        DH, forward_kinematics_dh, InverseKinematics,
        QuinticTrajectory, WaypointPath,
        rot_x, rot_y, rot_z, homogeneous,
    )

    # FK
    dh_params = [DH(a=1.0), DH(a=1.0)]
    T = forward_kinematics_dh(dh_params, [0.5, -0.3])

    # IK
    ik = InverseKinematics(dh_params=dh_params)
    q = ik.solve([1.5, 0.0, 0.0])

    # Trajectory
    traj = QuinticTrajectory([0, 0], [1, 1], duration=2.0)
    pos, vel, acc = traj.evaluate(t=1.0)
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
    "forward_kinematics_dh", "forward_kinematics_chain",
    "end_effector_position", "end_effector_rotation",
    "jacobian",
    "InverseKinematics",
    "QuinticTrajectory", "CubicTrajectory", "MinJerkTrajectory",
    "TrapezoidalProfile", "LinearTrajectory",
    "WaypointPath", "TrajectoryExecutor",
]
