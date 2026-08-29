#!/usr/bin/env python3
"""Example 01: 12-DOF Quadruped Walker with Convex MPC Stance Force Optimization."""

import numpy as np
from stacc.robot import QuadrupedRobot
from stacc.locomotion import QuadrupedConvexMPC


def main():
    print("=" * 60)
    print("STACC ROBOTICS: 12-DOF Quadruped Convex MPC Locomotion")
    print("=" * 60)

    # 1. Instantiate 12-DOF Quadruped
    robot = QuadrupedRobot(name="stacc_spot", body_mass=12.0)
    print(f"Robot Name: {robot.name}")
    print(f"Total Links: {len(robot.links)} | Total Joints: {len(robot.joints)}")

    # 2. Foot Kinematics
    fl_foot = robot.foot_forward_kinematics("FL", [0.0, 0.4, -0.8])
    print(f"FL Foot Position relative to Hip: {fl_foot}")

    # 3. Convex MPC Ground Reaction Force QP Solver
    mpc = QuadrupedConvexMPC(mass=12.0, mu=0.6)
    foot_pos = [
        np.array([0.20, 0.12, 0.0]),
        np.array([0.20, -0.12, 0.0]),
        np.array([-0.20, 0.12, 0.0]),
        np.array([-0.20, -0.12, 0.0])
    ]
    contact_states = [True, True, True, True]  # 4-foot stance

    forces = mpc.solve(
        body_pos=np.array([0.0, 0.0, 0.28]),
        body_vel=np.array([0.0, 0.0, 0.0]),
        body_rpy=np.array([0.0, 0.0, 0.0]),
        body_omega=np.array([0.0, 0.0, 0.0]),
        foot_positions=foot_pos,
        contact_states=contact_states,
        target_vel=np.array([0.8, 0.0, 0.0]),  # Move forward at 0.8 m/s
        target_height=0.30
    )

    print("\nOptimal 3D Ground Reaction Forces [Fx, Fy, Fz] (N):")
    for leg, f in zip(QuadrupedRobot.LEG_NAMES, forces):
        print(f"  Leg {leg}: Fx={f[0]:+6.2f} N, Fy={f[1]:+6.2f} N, Fz={f[2]:+6.2f} N")
    
    total_fz = np.sum(forces[:, 2])
    print(f"\nTotal Vertical Support: {total_fz:.2f} N (Weight = {12.0 * 9.81:.2f} N)")


if __name__ == "__main__":
    main()
