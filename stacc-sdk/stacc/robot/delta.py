"""Delta Parallel Robot Manipulator Kinematics."""

from __future__ import annotations
from typing import Optional, Tuple
import math
import numpy as np

from stacc.core.robot_base import Robot
from stacc.core.link import Link, Geometry


class DeltaRobot(Robot):
    """3-DOF Delta Parallel Manipulator."""

    def __init__(
        self,
        name: str = "delta_robot",
        base_radius: float = 0.25,
        end_effector_radius: float = 0.05,
        upper_arm_length: float = 0.25,
        forearm_length: float = 0.55
    ):
        super().__init__(name=name)
        self.R_b = float(base_radius)
        self.r_e = float(end_effector_radius)
        self.L1 = float(upper_arm_length)
        self.L2 = float(forearm_length)

        self.add_link(Link(name="base_platform", mass=5.0, visual=Geometry(shape="cylinder", radius=self.R_b, length=0.03)))
        self.add_link(Link(name="end_effector", mass=0.8, visual=Geometry(shape="cylinder", radius=self.r_e, length=0.02)))

        self.phi = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]

    def inverse_kinematics(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        theta = []
        for i in range(3):
            cos_phi = math.cos(self.phi[i])
            sin_phi = math.sin(self.phi[i])
            x_rot = x * cos_phi + y * sin_phi
            y_rot = -x * sin_phi + y * cos_phi

            d = self.R_b - self.r_e
            x_eff = x_rot - d

            a = 2.0 * self.L1 * z
            b = 2.0 * self.L1 * x_eff
            c = (x_eff ** 2) + (y_rot ** 2) + (z ** 2) + (self.L1 ** 2) - (self.L2 ** 2)

            discriminant = (a ** 2) + (b ** 2) - (c ** 2)
            if discriminant < 0:
                raise ValueError(f"Target point ({x:.3f}, {y:.3f}, {z:.3f}) is outside Delta workspace (arm {i+1})")

            t1 = (a + math.sqrt(discriminant)) / (b + c + 1e-12)
            theta_i = 2.0 * math.atan(t1)
            theta.append(theta_i)

        return theta[0], theta[1], theta[2]
