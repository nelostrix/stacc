"""Generalized Momentum Disturbance Observer for Contact & Collision Detection.

Estimates external joint contact torques tau_ext without requiring expensive multi-axis force/torque sensors:
    r(t) = K_I * [ p(t) - integral_0^t (tau + C(q, q_dot)^T * q_dot - G(q) + r) dt - p(0) ]
where p(t) = M(q) * q_dot is the generalized momentum of the robot.
"""

from __future__ import annotations
from typing import Optional, Union
import numpy as np


class MomentumObserver:
    """Generalized Momentum Disturbance Observer.

    Args:
        num_joints: Number of robot joints
        gain_diag: Observer gain K_I for each joint (default 20.0)
    """

    def __init__(self, num_joints: int, gain_diag: Union[float, np.ndarray] = 25.0):
        self.n = int(num_joints)
        self.K_I = np.eye(self.n) * float(gain_diag) if np.isscalar(gain_diag) else np.diag(gain_diag)
        self.integral_term = np.zeros(self.n, dtype=np.float64)
        self.p_0: Optional[np.ndarray] = None
        self.r = np.zeros(self.n, dtype=np.float64)  # Estimated external torque

    def update(
        self,
        q: np.ndarray,
        q_dot: np.ndarray,
        tau_applied: np.ndarray,
        M_mat: np.ndarray,
        C_qdot: np.ndarray,
        G_vec: np.ndarray,
        dt: float
    ) -> np.ndarray:
        """Update observer state and return estimated external contact torque tau_ext."""
        q_d = np.asarray(q_dot, dtype=np.float64)
        tau = np.asarray(tau_applied, dtype=np.float64)
        p = M_mat @ q_d

        if self.p_0 is None:
            self.p_0 = p.copy()
            return np.zeros(self.n)

        # Dynamic residual integrand: tau - C(q, q_dot)*q_dot - G(q) + r
        # Note: using Coriolis matrix property dM/dt = C + C^T
        integrand = tau + C_qdot - G_vec + self.r
        self.integral_term += integrand * float(dt)

        # Observer law: r = K_I * (p - integral - p_0)
        self.r = self.K_I @ (p - self.integral_term - self.p_0)
        return self.r.copy()

    def is_collision(self, threshold_nm: float = 8.0) -> bool:
        """Check if any joint exceeds the external impact threshold."""
        return bool(np.any(np.abs(self.r) > float(threshold_nm)))
