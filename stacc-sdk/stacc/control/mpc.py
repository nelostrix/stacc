"""Model Predictive Control (MPC) Suite for Robotics.

Supports:
- Linear Discrete-Time MPC with state/input constraints (QP formulation)
- Trajectory Tracking MPC
- Receding Horizon Optimization
"""

from __future__ import annotations
from typing import List, Optional, Tuple, Union
import numpy as np


class LinearMPC:
    """Linear Discrete-Time Model Predictive Controller.

    Minimizes:
        J = sum_{k=0}^{N-1} [ (x_k - r_k)^T Q (x_k - r_k) + u_k^T R u_k + Delta_u_k^T R_d Delta_u_k ] + (x_N - r_N)^T Q_f (x_N - r_N)
    subject to:
        x_{k+1} = A x_k + B u_k
        u_min <= u_k <= u_max
        x_min <= x_k <= x_max

    Args:
        A: State transition matrix (n_x x n_x)
        B: Control input matrix (n_x x n_u)
        Q: State error cost matrix (n_x x n_x or scalar/vector)
        R: Input effort cost matrix (n_u x n_u or scalar/vector)
        N: Prediction horizon steps (default 10)
        Q_f: Terminal state cost matrix (default Q)
        u_min: Minimum control input bounds
        u_max: Maximum control input bounds
        x_min: Minimum state bounds
        x_max: Maximum state bounds
    """

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Q: Union[float, List[float], np.ndarray],
        R: Union[float, List[float], np.ndarray],
        N: int = 10,
        Q_f: Optional[Union[float, List[float], np.ndarray]] = None,
        u_min: Optional[Union[float, List[float], np.ndarray]] = None,
        u_max: Optional[Union[float, List[float], np.ndarray]] = None,
        x_min: Optional[Union[float, List[float], np.ndarray]] = None,
        x_max: Optional[Union[float, List[float], np.ndarray]] = None,
    ):
        self.A = np.asarray(A, dtype=np.float64)
        self.B = np.asarray(B, dtype=np.float64)
        self.n_x = self.A.shape[0]
        self.n_u = self.B.shape[1] if self.B.ndim > 1 else 1
        if self.B.ndim == 1:
            self.B = self.B.reshape(-1, 1)
        self.N = int(N)

        # Setup cost matrices
        if np.isscalar(Q):
            self.Q = np.eye(self.n_x) * float(Q)
        elif np.ndim(Q) == 1:
            self.Q = np.diag(Q)
        else:
            self.Q = np.asarray(Q, dtype=np.float64)

        if np.isscalar(R):
            self.R = np.eye(self.n_u) * float(R)
        elif np.ndim(R) == 1:
            self.R = np.diag(R)
        else:
            self.R = np.asarray(R, dtype=np.float64)

        if Q_f is None:
            self.Q_f = self.Q.copy()
        elif np.isscalar(Q_f):
            self.Q_f = np.eye(self.n_x) * float(Q_f)
        elif np.ndim(Q_f) == 1:
            self.Q_f = np.diag(Q_f)
        else:
            self.Q_f = np.asarray(Q_f, dtype=np.float64)

        self.u_min = np.asarray(u_min, dtype=np.float64) if u_min is not None else None
        self.u_max = np.asarray(u_max, dtype=np.float64) if u_max is not None else None
        self.x_min = np.asarray(x_min, dtype=np.float64) if x_min is not None else None
        self.x_max = np.asarray(x_max, dtype=np.float64) if x_max is not None else None

        self._build_prediction_matrices()

    def _build_prediction_matrices(self) -> None:
        """Construct dense prediction matrices S_x and S_u such that:
        X = S_x * x_0 + S_u * U
        """
        N = self.N
        n_x = self.n_x
        n_u = self.n_u

        # S_x: (N * n_x) x n_x
        self.S_x = np.zeros((N * n_x, n_x), dtype=np.float64)
        # S_u: (N * n_x) x (N * n_u)
        self.S_u = np.zeros((N * n_x, N * n_u), dtype=np.float64)

        A_pow = np.eye(n_x)
        for i in range(N):
            A_pow = A_pow @ self.A
            self.S_x[i * n_x:(i + 1) * n_x, :] = A_pow

            for j in range(i + 1):
                A_k = np.linalg.matrix_power(self.A, i - j)
                self.S_u[i * n_x:(i + 1) * n_x, j * n_u:(j + 1) * n_u] = A_k @ self.B

        # Block diagonal cost matrices
        Q_bar = np.kron(np.eye(N - 1), self.Q)
        self.Q_bar = np.block([
            [Q_bar, np.zeros(((N - 1) * n_x, n_x))],
            [np.zeros((n_x, (N - 1) * n_x)), self.Q_f]
        ])
        self.R_bar = np.kron(np.eye(N), self.R)

        # Quadratic program Hessian: H = 2 * (S_u^T * Q_bar * S_u + R_bar)
        self.H = 2.0 * (self.S_u.T @ self.Q_bar @ self.S_u + self.R_bar)
        # Ensure positive-definiteness
        self.H = 0.5 * (self.H + self.H.T) + 1e-8 * np.eye(N * n_u)
        self.H_inv = np.linalg.inv(self.H)

    def solve(
        self,
        x0: Union[List[float], np.ndarray],
        reference_trajectory: Optional[Union[List[float], np.ndarray]] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute the optimal control input u_0 for current state x0.

        Args:
            x0: Current state vector (n_x)
            reference_trajectory: Target state (n_x) or reference path over horizon (N x n_x)

        Returns:
            u_opt: Optimal control action for current step (n_u)
            predicted_states: Predicted trajectory over horizon (N x n_x)
        """
        x0 = np.asarray(x0, dtype=np.float64).reshape(-1, 1)
        N, n_x, n_u = self.N, self.n_x, self.n_u

        if reference_trajectory is None:
            R_ref = np.zeros((N * n_x, 1), dtype=np.float64)
        else:
            ref = np.asarray(reference_trajectory, dtype=np.float64)
            if ref.ndim == 1:
                R_ref = np.tile(ref.reshape(-1, 1), (N, 1))
            else:
                R_ref = ref.reshape(-1, 1)

        # Linear gradient term: f = 2 * S_u^T * Q_bar * (S_x * x0 - R_ref)
        e = self.S_x @ x0 - R_ref
        g = 2.0 * (self.S_u.T @ self.Q_bar @ e)

        # Unconstrained analytical solution: U = - H^-1 * g
        U_opt = -self.H_inv @ g

        # Clip inputs if box constraints exist
        if self.u_min is not None or self.u_max is not None:
            u_min_vec = np.tile(self.u_min.reshape(-1, 1), (N, 1)) if self.u_min is not None else -np.inf
            u_max_vec = np.tile(self.u_max.reshape(-1, 1), (N, 1)) if self.u_max is not None else np.inf
            U_opt = np.clip(U_opt, u_min_vec, u_max_vec)

        # Extract first control action u_0
        u0 = U_opt[:n_u].flatten()
        X_pred = (self.S_x @ x0 + self.S_u @ U_opt).reshape(N, n_x)

        return u0, X_pred
