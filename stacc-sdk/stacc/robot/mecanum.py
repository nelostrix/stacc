"""Mecanum and Omnidirectional Mobile Base Kinematics."""

from __future__ import annotations
from typing import List, Tuple, Union
import numpy as np

from stacc.core.robot_base import Robot
from stacc.core.link import Link, Geometry


class MecanumDrive(Robot):
    """4-Wheel 45-Degree Mecanum Mobile Robot Platform."""

    def __init__(
        self,
        name: str = "mecanum_base",
        wheel_radius: float = 0.075,
        track_width: float = 0.40,
        wheel_base: float = 0.35,
        mass: float = 18.0
    ):
        super().__init__(name=name)
        self.r = float(wheel_radius)
        self.lx = float(wheel_base) / 2.0
        self.ly = float(track_width) / 2.0
        self.mass = float(mass)

        self.add_link(Link(name="chassis", mass=self.mass, visual=Geometry(shape="box", size=[wheel_base, track_width, 0.10])))

        k = self.lx + self.ly
        self.ik_matrix = (1.0 / self.r) * np.array([
            [1.0, -1.0, -k],
            [1.0,  1.0,  k],
            [1.0,  1.0, -k],
            [1.0, -1.0,  k]
        ], dtype=np.float64)
        self.fk_matrix = np.linalg.pinv(self.ik_matrix)

    def inverse_kinematics(self, vx: float, vy: float, omega_z: float) -> np.ndarray:
        body_vel = np.array([vx, vy, omega_z], dtype=np.float64)
        return self.ik_matrix @ body_vel

    def forward_kinematics(self, wheel_speeds: Union[List[float], np.ndarray]) -> Tuple[float, float, float]:
        w = np.asarray(wheel_speeds, dtype=np.float64)
        body_vel = self.fk_matrix @ w
        return float(body_vel[0]), float(body_vel[1]), float(body_vel[2])
