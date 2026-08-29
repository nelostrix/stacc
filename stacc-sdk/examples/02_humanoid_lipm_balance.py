#!/usr/bin/env python3
"""Example 02: Bipedal Humanoid Balance & Push Recovery using LIPM and Capture Point."""

import numpy as np
from stacc.robot import HumanoidRobot
from stacc.locomotion import LinearInvertedPendulum


def main():
    print("=" * 60)
    print("STACC ROBOTICS: Humanoid Bipedal LIPM Balance & Push Recovery")
    print("=" * 60)

    # 1. Instantiate 24-DOF Humanoid
    humanoid = HumanoidRobot(name="stacc_humanoid", total_height=1.65, total_mass=55.0)
    print(f"Humanoid Robot: {humanoid.name} (Height: {humanoid.total_height}m, Mass: {humanoid.total_mass}kg)")
    print(f"Total Joints: {len(humanoid.joints)}")

    # 2. Linear Inverted Pendulum Model
    lipm = LinearInvertedPendulum(com_height=0.85)
    print(f"Natural Frequency omega_0: {lipm.omega_0:.3f} rad/s")

    # 3. Simulate sudden disturbance push to the COM (delta_vx = +0.6 m/s)
    com_pos = (0.0, 0.0)
    com_vel = (0.60, 0.0)

    # Compute Instantaneous Capture Point (ICP)
    icp_x, icp_y = lipm.compute_capture_point(com_pos, com_vel)
    print(f"Instantaneous Capture Point: ({icp_x:.3f}m, {icp_y:.3f}m)")

    # Plan push recovery footstep placement
    step_x, step_y = lipm.plan_recovery_footstep(com_pos, com_vel, step_duration=0.32)
    print(f"Push Recovery Step Target:  ({step_x:.3f}m, {step_y:.3f}m)")


if __name__ == "__main__":
    main()
