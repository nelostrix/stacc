"""Linear Quadratic Regulator (LQR) and Riccati Equation Solvers."""

from __future__ import annotations
from typing import Tuple, Union
import numpy as np


def solve_discrete_are(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray, max_iter: int = 150, eps: float = 1e-9) -> np.ndarray:
    """Solve the Discrete Algebraic Riccati Equation (DARE) using value iteration.

    P = A^T * P * A - (A^T * P * B) * (R + B^T * P * B)^-1 * (B^T * P * A) + Q
    """
    P = Q.copy()
    for _ in range(max_iter):
        BPB_R = R + B.T @ P @ B
        P_next = A.T @ P @ A - (A.T @ P @ B) @ np.linalg.solve(BPB_R, B.T @ P @ A) + Q
        if np.max(np.abs(P_next - P)) < eps:
            return P_next
        P = P_next
    return P


def lqr(
    A: np.ndarray,
    B: np.ndarray,
    Q: Union[float, np.ndarray],
    R: Union[float, np.ndarray]
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute the discrete-time LQR optimal gain matrix K such that u = -K * x.

    Returns:
        K: Optimal state-feedback gain matrix (n_u x n_x)
        P: Solution to the Riccati equation (n_x x n_x)
    """
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    n_x = A.shape[0]
    n_u = B.shape[1] if B.ndim > 1 else 1
    if B.ndim == 1:
        B = B.reshape(-1, 1)

    Q_mat = np.eye(n_x) * float(Q) if np.isscalar(Q) else np.asarray(Q, dtype=np.float64)
    R_mat = np.eye(n_u) * float(R) if np.isscalar(R) else np.asarray(R, dtype=np.float64)

    P = solve_discrete_are(A, B, Q_mat, R_mat)
    K = np.linalg.solve(R_mat + B.T @ P @ B, B.T @ P @ A)
    return K, P


class LQRController:
    """Infinite-Horizon State-Feedback LQR Controller."""

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Q: Union[float, np.ndarray],
        R: Union[float, np.ndarray]
    ):
        self.K, self.P = lqr(A, B, Q, R)

    def compute(self, x: np.ndarray, x_ref: Optional[np.ndarray] = None) -> np.ndarray:
        """Compute optimal control action u = -K * (x - x_ref)."""
        x_val = np.asarray(x, dtype=np.float64)
        if x_ref is not None:
            err = x_val - np.asarray(x_ref, dtype=np.float64)
        else:
            err = x_val
        u = -self.K @ err
        return u.flatten() if u.size == 1 else u
