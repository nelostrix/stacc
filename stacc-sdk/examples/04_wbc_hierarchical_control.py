#!/usr/bin/env python3
"""Example 04: Hierarchical Whole-Body Control (WBC) with Strict-Priority Null-Space Tasks."""

import numpy as np
from stacc.control import HierarchicalWBC, TaskObjective


def main():
    print("=" * 60)
    print("STACC ROBOTICS: Hierarchical Whole-Body Control (WBC)")
    print("=" * 60)

    wbc = HierarchicalWBC(num_dof=6)

    # Priority 1: Keep Center of Mass Stable (2D task)
    J_com = np.array([
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    ])
    task_balance = TaskObjective(name="COM_Balance", J=J_com, x_ddot_des=np.array([0.0, 0.0]))

    # Priority 2: End-Effector Trajectory Tracking (3D task)
    J_ee = np.array([
        [0.0, 0.0, 1.0, 0.0, 0.5, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.5],
        [0.0, 0.0, 0.5, 0.0, 1.0, 0.0]
    ])
    task_tracking = TaskObjective(name="EE_Tracking", J=J_ee, x_ddot_des=np.array([1.2, -0.4, 0.8]))

    # Priority 3: Posture Damping / Energy Minimization (6D task)
    J_posture = np.eye(6)
    task_posture = TaskObjective(name="Posture_Damping", J=J_posture, x_ddot_des=np.zeros(6))

    # Mass matrix M(q)
    M_mat = np.diag([3.0, 3.0, 2.0, 1.5, 0.8, 0.5])
    C_qdot = np.zeros(6)
    G_vec = np.array([0.0, 0.0, 9.81, 4.2, 1.1, 0.2])

    q_ddot, tau = wbc.solve([task_balance, task_tracking, task_posture], M_mat, C_qdot, G_vec)

    print("\nOptimal Joint Accelerations (rad/s^2):")
    print(np.round(q_ddot, 3))
    print("\nRequired Actuator Torques (Nm):")
    print(np.round(tau, 3))


if __name__ == "__main__":
    main()
