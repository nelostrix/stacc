"""Serial manipulator — robotic arm with serial kinematic chain.

Supports 3 to 7+ DOF arms defined by DH parameters. Includes
forward kinematics, inverse kinematics, and joint-space control.
"""
from __future__ import annotations
from typing import List, Optional, Tuple
import math
from stacc.core.robot_base import Robot
from stacc.core.joint import Joint
from stacc.core.link import Link
from stacc.kinematics.transforms import DH


class SerialManipulator(Robot):
    """N-DOF serial robotic arm.

    Args:
        name: Robot name
        dh_parameters: List of DH parameter objects defining the arm geometry
        joint_limits: Optional list of (min, max) per joint in radians
    """

    def __init__(
        self,
        name: str = "arm",
        dh_parameters: Optional[List[DH]] = None,
        joint_limits: Optional[List[Tuple[float, float]]] = None,
    ):
        super().__init__(name=name)
        self.dh_parameters = dh_parameters or []
        self.joint_limits = joint_limits

        # Build joints from DH parameters
        for i, dh in enumerate(self.dh_parameters):
            limits = joint_limits[i] if joint_limits and i < len(joint_limits) else None
            joint = Joint(
                name=f"joint_{i + 1}",
                type="revolute",
                limits=limits,
                parent_link=f"link_{i}" if i > 0 else "base",
                child_link=f"link_{i + 1}",
            )
            self.add_joint(joint)
            self.add_link(Link(f"link_{i + 1}", mass=1.0))

    @property
    def n_joints(self) -> int:
        return len(self.dh_parameters)

    def forward_kinematics(self, joint_angles: Optional[List[float]] = None):
        """Compute end-effector pose from current or given joint angles."""
        from stacc.kinematics.forward import forward_kinematics_dh
        angles = joint_angles or self.get_joint_positions()
        return forward_kinematics_dh(self.dh_parameters, angles)

    def end_effector_position(self, joint_angles: Optional[List[float]] = None) -> List[float]:
        """Get end-effector [x, y, z] position."""
        from stacc.kinematics.forward import end_effector_position
        angles = joint_angles or self.get_joint_positions()
        return end_effector_position(self.dh_parameters, angles).tolist()

    def inverse_kinematics(
        self,
        target_position: List[float],
        method: str = "damped_least_squares",
        **kwargs,
    ) -> List[float]:
        """Solve IK for a target position."""
        from stacc.kinematics.inverse import InverseKinematics
        ik = InverseKinematics(
            dh_params=self.dh_parameters,
            method=method,
            **kwargs,
        )
        return ik.solve(
            target_position,
            initial_guess=self.get_joint_positions(),
            joint_limits=self.joint_limits,
        )

    def move_joints(self, target_angles: List[float], speed: float = 1.0):
        """Command the arm to move to target joint angles.

        Args:
            target_angles: Target angles in radians for each joint
            speed: Speed factor (0.0 to 1.0)
        """
        self.set_joint_positions(target_angles)

    def get_jacobian(self, joint_angles: Optional[List[float]] = None):
        """Compute the geometric Jacobian at current/given configuration."""
        from stacc.kinematics.forward import jacobian
        angles = joint_angles or self.get_joint_positions()
        return jacobian(self.dh_parameters, angles)
