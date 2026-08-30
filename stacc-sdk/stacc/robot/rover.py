"""STACC Planetary Rover: 6-Wheel Rocker-Bogie Kinematics & Suspension System."""

import numpy as np
from typing import Dict, Tuple, List, Optional
from dataclasses import dataclass


@dataclass
class RoverState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    left_rocker_deg: float = 0.0
    right_rocker_deg: float = 0.0
    left_bogie_deg: float = 0.0
    right_bogie_deg: float = 0.0
    diff_bar_deg: float = 0.0
    wheel_speeds: np.ndarray = None  # 6 wheels (rad/s)
    steer_angles: np.ndarray = None  # 4 corner steer wheels (deg)


class RockerBogieRover:
    """
    6-Wheel Rocker-Bogie Planetary Rover Kinematic Model (Curiosity / Perseverance class).
    Features:
      - Passive rocker-bogie differential suspension averaging
      - 4-wheel independent corner steering (FL, FR, RL, RR) + 2 fixed center wheels (ML, MR)
      - Individual in-hub BLDC wheel drive motors
      - Ground clearance: 0.45m, Rock climbing capability: up to 1.5x wheel diameter
    """

    def __init__(
        self,
        wheel_radius: float = 0.25,
        track_width: float = 1.60,
        wheelbase_front: float = 1.10,
        wheelbase_rear: float = 1.10,
        rocker_ratio: float = 1.5,
    ):
        self.r = wheel_radius
        self.W = track_width
        self.L_f = wheelbase_front
        self.L_r = wheelbase_rear
        self.rocker_ratio = rocker_ratio
        
        # 6 wheels: [FL, ML, RL, FR, MR, RR]
        self.wheel_names = ["FL", "ML", "RL", "FR", "MR", "RR"]
        self.corner_steer_indices = [0, 2, 3, 5]  # FL, RL, FR, RR

    def forward_kinematics(
        self,
        wheel_rotations_rad: np.ndarray,
        steer_angles_rad: np.ndarray,
        dt: float = 0.01,
    ) -> Tuple[float, float, float]:
        """
        Compute forward chassis velocity [vx, vy, omega_z] from wheel encoders.
        """
        if wheel_rotations_rad.shape[0] != 6:
            raise ValueError("Expected 6 wheel velocity values")

        linear_velocities = wheel_rotations_rad * self.r
        # Average longitudinal velocity
        vx = float(np.mean(linear_velocities))
        
        # Turn rate from differential corner steer angles
        fl_steer = steer_angles_rad[0] if len(steer_angles_rad) >= 4 else 0.0
        fr_steer = steer_angles_rad[2] if len(steer_angles_rad) >= 4 else 0.0
        avg_steer = (fl_steer + fr_steer) / 2.0
        
        omega_z = (vx / (self.L_f + self.L_r)) * np.tan(avg_steer) if abs(np.cos(avg_steer)) > 1e-4 else 0.0
        vy = 0.0  # Non-holonomic constraint

        return vx, vy, omega_z

    def inverse_kinematics(
        self,
        linear_vel: float,
        angular_vel: float,
        turn_radius: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute 6 wheel drive speeds (rad/s) and 4 corner steering angles (deg).
        """
        wheel_speeds = np.zeros(6, dtype=np.float64)
        steer_angles = np.zeros(4, dtype=np.float64)  # FL, RL, FR, RR

        if abs(angular_vel) < 1e-4:
            # Straight drive
            wheel_speeds[:] = linear_vel / self.r
            steer_angles[:] = 0.0
            return wheel_speeds, steer_angles

        # Compute turning radius R
        R = linear_vel / angular_vel if turn_radius is None else turn_radius
        
        # Ackermann-style geometry for 6 wheels
        # Left side distance from Instantaneous Center of Rotation (ICR): R - W/2
        # Right side distance from ICR: R + W/2
        R_L = R - self.W / 2.0
        R_R = R + self.W / 2.0

        # Steering angles (FL, FR, RL, RR)
        delta_FL = np.arctan2(self.L_f, R_L)
        delta_FR = np.arctan2(self.L_f, R_R)
        delta_RL = np.arctan2(-self.L_r, R_L)
        delta_RR = np.arctan2(-self.L_r, R_R)

        steer_angles[0] = np.degrees(delta_FL)
        steer_angles[1] = np.degrees(delta_RL)
        steer_angles[2] = np.degrees(delta_FR)
        steer_angles[3] = np.degrees(delta_RR)

        # Wheel speeds
        speed_FL = angular_vel * np.sqrt(R_L**2 + self.L_f**2)
        speed_ML = angular_vel * abs(R_L)
        speed_RL = angular_vel * np.sqrt(R_L**2 + self.L_r**2)

        speed_FR = angular_vel * np.sqrt(R_R**2 + self.L_f**2)
        speed_MR = angular_vel * abs(R_R)
        speed_RR = angular_vel * np.sqrt(R_R**2 + self.L_r**2)

        wheel_speeds[0] = speed_FL / self.r
        wheel_speeds[1] = speed_ML / self.r
        wheel_speeds[2] = speed_RL / self.r
        wheel_speeds[3] = speed_FR / self.r
        wheel_speeds[4] = speed_MR / self.r
        wheel_speeds[5] = speed_RR / self.r

        return wheel_speeds, steer_angles

    def compute_suspension_differential(
        self,
        left_rocker_angle_rad: float,
        right_rocker_angle_rad: float,
    ) -> float:
        """
        Compute top differential averaging bar pitch angle.
        Differential mechanism couples the left and right rockers so chassis pitch
        is the average of left and right rocker angles.
        """
        chassis_pitch = (left_rocker_angle_rad + right_rocker_angle_rad) / 2.0
        return float(chassis_pitch)
