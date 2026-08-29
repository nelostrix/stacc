"""Control Barrier Functions (CBF) & Real-Time Safety Filters.

Guarantees forward invariance of a safe set C = {x : h(x) >= 0}
by solving a minimally invasive Quadratic Program (QP) filter:
    min_u  0.5 * ||u - u_nom||^2
    s.t.   L_f h(x) + L_g h(x) * u + gamma * h(x) >= 0
           u_min <= u <= u_max
"""

from __future__ import annotations
from typing import Callable, List, Optional, Tuple, Union
import numpy as np


class ControlBarrierFilter:
    """Control Barrier Function (CBF) Safety Filter.

    Args:
        gamma: Linear class-K function gain (higher values allow operating closer to boundary)
        u_min: Minimum control action bounds
        u_max: Maximum control action bounds
    """

    def __init__(
        self,
        gamma: float = 5.0,
        u_min: Optional[Union[float, np.ndarray]] = None,
        u_max: Optional[Union[float, np.ndarray]] = None
    ):
        self.gamma = float(gamma)
        self.u_min = np.asarray(u_min, dtype=np.float64) if u_min is not None else None
        self.u_max = np.asarray(u_max, dtype=np.float64) if u_max is not None else None

    def filter_control(
        self,
        u_nominal: np.ndarray,
        h_val: float,
        grad_h: np.ndarray,
        f_x: np.ndarray,
        g_x: np.ndarray
    ) -> np.ndarray:
        """Filter a nominal control action u_nom to ensure safety h(x) >= 0.

        Args:
            u_nominal: Desired control action (n_u)
            h_val: Current barrier function value h(x)
            grad_h: Gradient of barrier function dh/dx (n_x)
            f_x: Autonomous drift vector f(x) (n_x)
            g_x: Input vector field g(x) (n_x x n_u)

        Returns:
            u_safe: Safe control input closest to u_nominal
        """
        u_nom = np.asarray(u_nominal, dtype=np.float64)
        n_u = len(u_nom)

        # Lie derivatives: L_f h = (dh/dx) * f(x), L_g h = (dh/dx) * g(x)
        Lf_h = float(np.dot(grad_h, f_x))
        Lg_h = np.dot(grad_h, g_x)  # 1 x n_u

        # Constraint: Lg_h * u >= -Lf_h - gamma * h(x)  =>  -Lg_h * u <= Lf_h + gamma * h(x)
        b_cbf = Lf_h + self.gamma * h_val

        # Check if nominal control already satisfies the CBF condition: Lg_h * u_nom >= -b_cbf
        cbf_eval = np.dot(Lg_h, u_nom) + b_cbf
        if cbf_eval >= 0:
            u_safe = u_nom
        else:
            # Analytical projection onto the half-space constraint
            # u_safe = u_nom - (Lg_h * u_nom + b_cbf) / ||Lg_h||^2 * Lg_h^T
            Lg_norm_sq = np.dot(Lg_h, Lg_h)
            if Lg_norm_sq > 1e-12:
                correction = (-cbf_eval / Lg_norm_sq) * Lg_h
                u_safe = u_nom + correction
            else:
                u_safe = u_nom

        # Apply box bounds
        if self.u_min is not None:
            u_safe = np.maximum(u_safe, self.u_min)
        if self.u_max is not None:
            u_safe = np.minimum(u_safe, self.u_max)

        return u_safe
