"""STACC Teleop: Universal Gamepad & 6-DOF SpaceMouse Teleoperation Mapper."""

import numpy as np
from typing import Dict, Tuple, Any


class UniversalTeleopMapper:
    """
    Maps Gamepad (Xbox, PlayStation) and 6-DOF SpaceMouse controllers
    to robot joint rates or Cartesian twist velocities [vx, vy, vz, wx, wy, wz].
    """

    def __init__(
        self,
        deadband: float = 0.08,
        max_linear_speed_m_s: float = 1.0,
        max_angular_speed_rad_s: float = 1.5,
    ):
        self.deadband = deadband
        self.max_lin = max_linear_speed_m_s
        self.max_ang = max_angular_speed_rad_s

    def process_joystick_axis(self, raw_value: float, exponent: float = 1.5) -> float:
        """Apply deadband and non-linear response curve for fine control."""
        if abs(raw_value) < self.deadband:
            return 0.0
        # Re-scale from [deadband, 1.0] -> [0.0, 1.0]
        sign = np.sign(raw_value)
        val = (abs(raw_value) - self.deadband) / (1.0 - self.deadband)
        return float(sign * (val ** exponent))

    def map_gamepad_to_differential_drive(self, stick_left_y: float, stick_right_x: float) -> Tuple[float, float]:
        """Maps Dual-Stick gamepad inputs to [linear_velocity, angular_velocity]."""
        lin = -self.process_joystick_axis(stick_left_y) * self.max_lin
        ang = -self.process_joystick_axis(stick_right_x) * self.max_ang
        return lin, ang

    def map_spacemouse_to_6dof_twist(self, axes_6d: np.ndarray) -> np.ndarray:
        """Maps 6-axis 3D mouse (Tx, Ty, Tz, Rx, Ry, Rz) to Cartesian end-effector velocity."""
        if len(axes_6d) != 6:
            raise ValueError("Expected 6 SpaceMouse axes")

        twist = np.zeros(6, dtype=np.float64)
        for i in range(3):
            twist[i] = self.process_joystick_axis(axes_6d[i]) * self.max_lin
        for i in range(3, 6):
            twist[i] = self.process_joystick_axis(axes_6d[i]) * self.max_ang

        return twist
