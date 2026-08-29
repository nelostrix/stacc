#!/usr/bin/env python3
"""Example 05: CHOMP Covariant Trajectory Optimization around 3D Obstacles."""

import numpy as np
from stacc.planning import CHOMPPlanner


def main():
    print("=" * 60)
    print("STACC ROBOTICS: CHOMP Covariant Trajectory Optimization")
    print("=" * 60)

    planner = CHOMPPlanner(num_waypoints=20, num_dof=3, weight_obstacle=15.0)

    q_start = np.array([0.0, 0.0, 0.0])
    q_goal = np.array([1.5, 1.0, 0.8])

    # Obstacle sphere at q_obs = [0.75, 0.5, 0.4] with radius r = 0.3
    q_obs = np.array([0.75, 0.5, 0.4])
    r_obs = 0.30

    def obstacle_potential(q):
        dist = np.linalg.norm(q - q_obs)
        if dist < r_obs:
            cost = 0.5 * ((dist - r_obs) ** 2)
            grad = (dist - r_obs) * (q - q_obs) / (dist + 1e-12)
            return cost, grad
        return 0.0, np.zeros_like(q)

    traj = planner.optimize(q_start, q_goal, obstacle_cost_fn=obstacle_potential, iterations=60)

    print(f"Generated {len(traj)} collision-free waypoints.")
    print("Waypoint 0 (Start):", traj[0])
    print("Waypoint 10 (Mid):  ", np.round(traj[10], 3))
    print("Waypoint 19 (Goal): ", traj[-1])


if __name__ == "__main__":
    main()
