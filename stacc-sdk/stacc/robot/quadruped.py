"""Quadruped Robot Topology.

12-DOF quadrupedal locomotion model (Spot / Unitree style) with 4 legs:
- Front Left (FL), Front Right (FR), Rear Left (RL), Rear Right (RR)
- Each leg has 3 active revolute joints: Hip Roll, Hip Pitch, Knee Pitch
- Floating-base state representation, foot Jacobians, and contact state.
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple
import math
import numpy as np

from stacc.core.robot_base import Robot
from stacc.core.joint import Joint
from stacc.core.link import Link, Inertia, Geometry


class QuadrupedRobot(Robot):
    """12-DOF Quadruped Walker Robot.

    Args:
        name: Robot name
        body_length: Length of main chassis in meters (default 0.40m)
        body_width: Width of main chassis in meters (default 0.20m)
        body_mass: Mass of torso in kg (default 12.0kg)
        hip_length: Upper leg / thigh length in meters (default 0.20m)
        calf_length: Lower leg / shank length in meters (default 0.20m)
    """

    LEG_NAMES = ("FL", "FR", "RL", "RR")

    def __init__(
        self,
        name: str = "quadruped",
        body_length: float = 0.40,
        body_width: float = 0.20,
        body_mass: float = 12.0,
        hip_length: float = 0.20,
        calf_length: float = 0.20
    ):
        super().__init__(name=name)
        self.body_length = float(body_length)
        self.body_width = float(body_width)
        self.body_mass = float(body_mass)
        self.l1 = float(hip_length)
        self.l2 = float(calf_length)

        # Base Torso Link
        self.add_link(Link(name="base_link", mass=self.body_mass, visual=Geometry(shape="box", size=[self.body_length, self.body_width, 0.12])))

        # Build 4 legs (3 joints each = 12 joints total)
        for leg in self.LEG_NAMES:
            is_front = 1 if "F" in leg else -1
            is_left = 1 if "L" in leg else -1

            hip_offset = [is_front * self.body_length / 2.0, is_left * self.body_width / 2.0, 0.0]

            # Hip Roll Joint (Abduction/Adduction)
            j_roll = Joint(
                name=f"{leg}_hip_roll",
                type="revolute",
                axis=[1, 0, 0],
                origin=hip_offset,
                limits=(-math.radians(45), math.radians(45)),
                parent_link="base_link",
                child_link=f"{leg}_hip"
            )
            self.add_joint(j_roll)
            self.add_link(Link(name=f"{leg}_hip", mass=0.8, visual=Geometry(shape="cylinder", radius=0.03, length=0.08)))

            # Hip Pitch Joint (Thigh flexion/extension)
            j_pitch = Joint(
                name=f"{leg}_hip_pitch",
                type="revolute",
                axis=[0, 1, 0],
                origin=[0.0, 0.0, 0.0],
                limits=(-math.radians(60), math.radians(120)),
                parent_link=f"{leg}_hip",
                child_link=f"{leg}_thigh"
            )
            self.add_joint(j_pitch)
            self.add_link(Link(name=f"{leg}_thigh", mass=1.2, visual=Geometry(shape="cylinder", radius=0.03, length=self.l1)))

            # Knee Pitch Joint (Shank flexion/extension)
            j_knee = Joint(
                name=f"{leg}_knee",
                type="revolute",
                axis=[0, 1, 0],
                origin=[0.0, 0.0, -self.l1],
                limits=(-math.radians(150), -math.radians(30)),
                parent_link=f"{leg}_thigh",
                child_link=f"{leg}_calf"
            )
            self.add_joint(j_knee)
            self.add_link(Link(name=f"{leg}_calf", mass=0.4, visual=Geometry(shape="cylinder", radius=0.02, length=self.l2)))

    def foot_forward_kinematics(self, leg: str, q_leg: List[float]) -> np.ndarray:
        """Compute foot position relative to the hip mount for a given leg [q_roll, q_pitch, q_knee]."""
        q_roll, q_pitch, q_knee = q_leg[0], q_leg[1], q_leg[2]
        l1, l2 = self.l1, self.l2

        x = -l1 * math.sin(q_pitch) - l2 * math.sin(q_pitch + q_knee)
        z_plane = -l1 * math.cos(q_pitch) - l2 * math.cos(q_pitch + q_knee)
        y = -z_plane * math.sin(q_roll)
        z = z_plane * math.cos(q_roll)

        return np.array([x, y, z], dtype=np.float64)

    def foot_jacobian(self, leg: str, q_leg: List[float]) -> np.ndarray:
        """Compute 3x3 foot contact Jacobian J_foot = d(p_foot)/d(q_leg)."""
        eps = 1e-6
        q0 = np.asarray(q_leg, dtype=np.float64)
        p0 = self.foot_forward_kinematics(leg, q0)
        J = np.zeros((3, 3), dtype=np.float64)

        for i in range(3):
            q_plus = q0.copy()
            q_plus[i] += eps
            p_plus = self.foot_forward_kinematics(leg, q_plus)
            J[:, i] = (p_plus - p0) / eps

        return J
