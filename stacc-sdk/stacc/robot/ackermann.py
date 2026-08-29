"""Ackermann Steering Vehicle and Autonomous Car Kinematics."""

from __future__ import annotations
from typing import Optional, Tuple
import math
import numpy as np

from stacc.core.robot_base import Robot
from stacc.core.link import Link, Geometry


class AckermannVehicle(Robot):
    """Ackermann-Steered Autonomous Vehicle (Car / Rover)."""

    def __init__(
        self,
        name: str = "ackermann_car",
        wheel_base: float = 0.65,
        track_width: float = 0.45,
        max_steer_angle: float = 0.61,
        mass: float = 25.0
    ):
        super().__init__(name=name)
        self.L = float(wheel_base)
        self.W = float(track_width)
        self.max_steer = float(max_steer_angle)
        self.mass = float(mass)

        self.add_link(Link(name="chassis", mass=self.mass, visual=Geometry(shape="box", size=[self.L, self.W, 0.15])))

    def steering_angles(self, delta_center: float) -> Tuple[float, float]:
        delta = max(-self.max_steer, min(self.max_steer, delta_center))
        if abs(delta) < 1e-5:
            return 0.0, 0.0

        R = self.L / math.tan(delta)
        delta_inner = math.atan(self.L / (R - self.W / 2.0))
        delta_outer = math.atan(self.L / (R + self.W / 2.0))
        return delta_inner, delta_outer

    def pure_pursuit_steering(
        self,
        vehicle_pose: Tuple[float, float, float],
        target_waypoint: Tuple[float, float],
        lookahead_distance: float
    ) -> float:
        x, y, yaw = vehicle_pose
        tx, ty = target_waypoint

        dx = tx - x
        dy = ty - y
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        L_d = max(0.05, float(lookahead_distance))
        curvature = 2.0 * local_y / (L_d ** 2)
        delta = math.atan(curvature * self.L)
        return float(max(-self.max_steer, min(self.max_steer, delta)))
