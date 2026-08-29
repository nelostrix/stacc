"""Covariant Hamiltonian Optimization for Motion Planning (CHOMP).

Gradient-based trajectory optimization that transforms an initial naive trajectory
into a smooth, dynamically feasible, collision-free path by minimizing:
    U(xi) = f_smooth(xi) + lambda * f_obs(xi)
"""

from __future__ import annotations
from typing import Callable, List, Optional, Tuple, Union
import numpy as np


class CHOMPPlanner:
    """CHOMP Trajectory Optimizer for Robotic Arms & Mobile Bases.

    Args:
        num_waypoints: Number of trajectory discretization points N (default 25)
        num_dof: Degrees of freedom n (default 6)
        weight_obstacle: Multiplier for obstacle collision cost lambda (default 10.0)
        learning_rate: Gradient descent step size eta (default 0.01)
    """

    def __init__(
        self,
        num_waypoints: int = 25,
        num_dof: int = 6,
        weight_obstacle: float = 10.0,
        learning_rate: float = 0.01
    ):
        self.N = int(num_waypoints)
        self.n = int(num_dof)
        self.lambda_obs = float(weight_obstacle)
        self.eta = float(learning_rate)

        self._build_metric_matrix()

    def _build_metric_matrix(self) -> None:
        """Construct the finite-difference acceleration operator K and smoothness metric A = K^T * K."""
        N = self.N
        K = np.zeros((N - 1, N), dtype=np.float64)
        for i in range(N - 1):
            K[i, i] = -1.0
            K[i, i + 1] = 1.0

        A = K.T @ K
        # Regularize to guarantee positive definiteness for inversion
        A += 1e-4 * np.eye(N)
        self.A_inv = np.linalg.inv(A)

    def optimize(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        obstacle_cost_fn: Optional[Callable[[np.ndarray], Tuple[float, np.ndarray]]] = None,
        iterations: int = 50
    ) -> np.ndarray:
        """Optimize trajectory from q_start to q_goal.

        Args:
            q_start: Initial joint configuration (n)
            q_goal: Final target joint configuration (n)
            obstacle_cost_fn: Function taking q -> (cost_scalar, grad_vector)
            iterations: Number of gradient descent iterations

        Returns:
            trajectory: Optimized N x n joint trajectory
        """
        q_start = np.asarray(q_start, dtype=np.float64)
        q_goal = np.asarray(q_goal, dtype=np.float64)

        # 1. Initialize with linear interpolation
        trajectory = np.zeros((self.N, self.n), dtype=np.float64)
        for i in range(self.N):
            alpha = i / (self.N - 1)
            trajectory[i] = (1.0 - alpha) * q_start + alpha * q_goal

        if obstacle_cost_fn is None:
            return trajectory

        # 2. Covariant Gradient Descent iterations
        for _ in range(iterations):
            grad_obs = np.zeros_like(trajectory)
            for i in range(1, self.N - 1):
                cost, g = obstacle_cost_fn(trajectory[i])
                grad_obs[i] = g

            # Smoothness gradient: grad_smooth = A * xi
            # Covariant update: delta_xi = - eta * A^-1 * (A * xi + lambda * grad_obs)
            for j in range(self.n):
                total_grad = self.lambda_obs * grad_obs[:, j]
                delta = -self.eta * (self.A_inv @ total_grad)
                trajectory[1:-1, j] += delta[1:-1]  # Keep start and goal fixed

        return trajectory
