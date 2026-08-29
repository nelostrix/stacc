"""stacc.locomotion — Advanced Legged Locomotion, Convex MPC, and Bipedal Balance."""

from stacc.locomotion.convex_mpc import QuadrupedConvexMPC
from stacc.locomotion.lipm import LinearInvertedPendulum

__all__ = [
    "QuadrupedConvexMPC",
    "LinearInvertedPendulum",
]
