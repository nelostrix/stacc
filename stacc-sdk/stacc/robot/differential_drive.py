"""Differential drive robot — 2-wheel mobile robot.

The most common robot type: two independently driven wheels for
steering via speed difference. Used for line followers, small mobile
bots, and educational platforms.
"""
from __future__ import annotations
from typing import Optional
import math
from stacc.core.robot_base import Robot
from stacc.core.joint import Joint
from stacc.core.link import Link, Inertia, Geometry


class DifferentialDrive(Robot):
    """Two-wheel differential drive mobile robot.

    Args:
        name: Robot name
        wheel_radius: Wheel radius in meters
        wheel_base: Distance between wheels in meters
        max_rpm: Maximum motor RPM
        mass: Total robot mass in kg
    """

    def __init__(
        self,
        name: str = "diff_drive",
        wheel_radius: float = 0.033,
        wheel_base: float = 0.15,
        max_rpm: int = 200,
        mass: float = 0.5,
    ):
        super().__init__(name=name)
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.max_rpm = max_rpm
        self.max_speed = (max_rpm * 2 * math.pi * wheel_radius) / 60  # m/s

        # Build kinematic structure
        body = Link(
            "body",
            mass=mass * 0.8,
            visual=Geometry("box", size=[wheel_base, wheel_base * 0.6, 0.03]),
        )
        left_wheel = Link(
            "left_wheel",
            mass=mass * 0.1,
            visual=Geometry("cylinder", radius=wheel_radius, length=0.01),
        )
        right_wheel = Link(
            "right_wheel",
            mass=mass * 0.1,
            visual=Geometry("cylinder", radius=wheel_radius, length=0.01),
        )

        self.add_link(body)
        self.add_link(left_wheel)
        self.add_link(right_wheel)

        self.add_joint(Joint(
            "left_wheel_joint",
            type="continuous",
            parent_link="body",
            child_link="left_wheel",
            axis=[0, 1, 0],
            origin=[0, -wheel_base / 2, 0, 0, 0, 0],
        ))
        self.add_joint(Joint(
            "right_wheel_joint",
            type="continuous",
            parent_link="body",
            child_link="right_wheel",
            axis=[0, 1, 0],
            origin=[0, wheel_base / 2, 0, 0, 0, 0],
        ))

        # Speed state
        self._left_speed: float = 0.0
        self._right_speed: float = 0.0

    def set_speed(self, left: float, right: float):
        """Set wheel speeds as fraction of max (-1.0 to 1.0)."""
        self._left_speed = max(-1.0, min(1.0, left))
        self._right_speed = max(-1.0, min(1.0, right))

    def set_velocity(self, linear: float, angular: float):
        """Set linear (m/s) and angular (rad/s) velocity."""
        left_vel = linear - (angular * self.wheel_base / 2)
        right_vel = linear + (angular * self.wheel_base / 2)
        self._left_speed = left_vel / self.max_speed
        self._right_speed = right_vel / self.max_speed

    @property
    def linear_velocity(self) -> float:
        """Current linear velocity in m/s."""
        vl = self._left_speed * self.max_speed
        vr = self._right_speed * self.max_speed
        return (vl + vr) / 2

    @property
    def angular_velocity(self) -> float:
        """Current angular velocity in rad/s."""
        vl = self._left_speed * self.max_speed
        vr = self._right_speed * self.max_speed
        return (vr - vl) / self.wheel_base

    @property
    def heading(self) -> float:
        """Current heading angle in radians."""
        return self.orientation[2]
