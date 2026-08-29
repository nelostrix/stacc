#!/usr/bin/env python3
"""Example 03: 6-DOF Multirotor Drone Flight Dynamics & Motor Thrust Allocation."""

import numpy as np
from stacc.robot import MultirotorDrone


def main():
    print("=" * 60)
    print("STACC ROBOTICS: 6-DOF Drone Thrust/Torque Allocation")
    print("=" * 60)

    drone = MultirotorDrone(name="stacc_quadrotor", mass=1.4, arm_length=0.25)
    print(f"Drone Mass: {drone.mass} kg | Arm Radius: {drone.arm_length} m")

    # Command: Hover + Pitch Right Turn
    hover_thrust = drone.mass * 9.81
    torques = [0.15, -0.08, 0.05]  # [Roll, Pitch, Yaw]

    rotor_thrusts = drone.allocate_thrusts(total_thrust=hover_thrust, torques=torques)
    print(f"\nDesired Total Thrust: {hover_thrust:.2f} N")
    print(f"Desired Torques:      [Roll={torques[0]:.2f}, Pitch={torques[1]:.2f}, Yaw={torques[2]:.2f}] Nm")

    print("\nAllocated Individual Rotor Thrusts:")
    for i, t in enumerate(rotor_thrusts):
        print(f"  Rotor {i+1}: {t:.3f} N")


if __name__ == "__main__":
    main()
