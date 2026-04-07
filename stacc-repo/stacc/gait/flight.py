# ══════════════════════════════════════════
# FLIGHT CONTROLLER PRIMITIVES
# ══════════════════════════════════════════
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple


class FlightController:
    """Basic flight controller primitives for aerial robots.

    Computes motor commands from desired attitude and thrust.
    Works for any multirotor configuration (quad, hex, octo).

    Args:
        num_rotors: Number of rotors
        arm_length: Distance from center to rotor in meters
        mass: Vehicle mass in kg
    """

    def __init__(
        self, num_rotors: int = 4, arm_length: float = 0.175, mass: float = 1.2
    ):
        self.num_rotors = num_rotors
        self.arm_length = arm_length
        self.mass = mass
        self.gravity = 9.81

        # Motor mixing matrix (for quadrotor X configuration)
        if num_rotors == 4:
            self._mix = [
                [1, -1, -1, -1],  # front-left  (CCW)
                [1, -1, 1, 1],  # front-right (CW)
                [1, 1, 1, -1],  # back-right  (CCW)
                [1, 1, -1, 1],  # back-left   (CW)
            ]
        else:
            # Generic even distribution
            self._mix = []
            for i in range(num_rotors):
                angle = 2 * math.pi * i / num_rotors
                self._mix.append([1, math.sin(angle), math.cos(angle), (-1) ** i])

    def hover_thrust(self) -> float:
        """Thrust per motor to hover."""
        return self.mass * self.gravity / self.num_rotors

    def compute_motor_commands(
        self, thrust: float, roll: float, pitch: float, yaw: float
    ) -> List[float]:
        """Compute individual motor thrusts from desired attitude.

        Args:
            thrust: Total thrust (normalized, 1.0 = hover)
            roll: Roll torque command (-1 to 1)
            pitch: Pitch torque command (-1 to 1)
            yaw: Yaw torque command (-1 to 1)

        Returns:
            List of motor thrust values
        """
        base = thrust * self.hover_thrust()
        commands = [
            base * self._mix[i][0]
            + roll * self._mix[i][1]
            + pitch * self._mix[i][2]
            + yaw * self._mix[i][3]
            for i in range(self.num_rotors)
        ]
        return [max(0, c) for c in commands]

    def attitude_control(
        self,
        current_rpy: List[float],
        target_rpy: List[float],
        angular_vel: List[float],
        kp: float = 5.0,
        kd: float = 1.0,
    ) -> Tuple[float, float, float]:
        """PD attitude controller.

        Args:
            current_rpy: Current [roll, pitch, yaw] in radians
            target_rpy: Target [roll, pitch, yaw]
            angular_vel: Current angular velocities [p, q, r]
            kp: Proportional gain
            kd: Derivative gain

        Returns:
            (roll_cmd, pitch_cmd, yaw_cmd) torque commands
        """
        roll_cmd = kp * (target_rpy[0] - current_rpy[0]) - kd * angular_vel[0]
        pitch_cmd = kp * (target_rpy[1] - current_rpy[1]) - kd * angular_vel[1]
        yaw_cmd = kp * (target_rpy[2] - current_rpy[2]) - kd * angular_vel[2]
        return roll_cmd, pitch_cmd, yaw_cmd
