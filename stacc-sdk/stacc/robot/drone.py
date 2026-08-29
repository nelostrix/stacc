"""Multirotor Aerial Drone & Quadrotor Dynamics."""

from __future__ import annotations
from typing import List, Optional, Tuple, Union
import math
import numpy as np

from stacc.core.robot_base import Robot
from stacc.core.link import Link, Geometry


class MultirotorDrone(Robot):
    """6-DOF Multirotor Drone (Quadrotor / Hexacopter)."""

    def __init__(
        self,
        name: str = "quadrotor",
        arm_length: float = 0.22,
        mass: float = 1.20,
        num_rotors: int = 4,
        thrust_coefficient: float = 1.5e-5,
        torque_coefficient: float = 2.5e-7
    ):
        super().__init__(name=name)
        self.arm_length = float(arm_length)
        self.mass = float(mass)
        self.num_rotors = int(num_rotors)
        self.c_T = float(thrust_coefficient)
        self.c_Q = float(torque_coefficient)

        # Base Frame
        self.add_link(Link(name="drone_chassis", mass=self.mass, visual=Geometry(shape="box", size=[self.arm_length * 2, self.arm_length * 2, 0.05])))

        # Inertia tensor
        Ixx = 0.5 * self.mass * (self.arm_length ** 2)
        Iyy = Ixx
        Izz = 2.0 * Ixx
        self.inertia = np.diag([Ixx, Iyy, Izz])

        self._build_mixer_matrix()

    def _build_mixer_matrix(self) -> None:
        d = self.arm_length
        c = self.c_Q / self.c_T

        if self.num_rotors == 4:
            s45 = math.sin(math.radians(45))
            self.mixer = np.array([
                [ 1.0,      1.0,      1.0,      1.0     ],
                [-d * s45,  d * s45,  d * s45, -d * s45 ],
                [ d * s45, -d * s45,  d * s45, -d * s45 ],
                [-c,       -c,        c,        c       ]
            ], dtype=np.float64)
            self.mixer_inv = np.linalg.pinv(self.mixer)

    def allocate_thrusts(self, total_thrust: float, torques: Union[List[float], np.ndarray]) -> np.ndarray:
        desired_wrench = np.array([total_thrust, torques[0], torques[1], torques[2]], dtype=np.float64)
        rotor_thrusts = self.mixer_inv @ desired_wrench
        return np.maximum(0.0, rotor_thrusts)
