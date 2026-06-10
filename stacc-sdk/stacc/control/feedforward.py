from __future__ import annotations

import math
from typing import Optional


class Feedforward:
    """Velocity + acceleration feedforward.

    Computes: u = kv * velocity + ka * acceleration + ks

    Args:
        kv: Velocity gain (counteracts back-EMF / damping).
        ka: Acceleration gain (counteracts inertia).
        ks: Static offset (gravity compensation bias, friction breakaway).
    """

    def __init__(self, kv: float = 0.0, ka: float = 0.0, ks: float = 0.0):
        self.kv = kv
        self.ka = ka
        self.ks = ks

    def compute(self, velocity: float = 0.0, acceleration: float = 0.0) -> float:
        return self.kv * velocity + self.ka * acceleration + self.ks

    def set_gains(self, kv: float, ka: float, ks: float):
        self.kv = kv
        self.ka = ka
        self.ks = ks


class GravityCompensation:
    """Gravity compensation for serial manipulators.

    Computes torque required to hold joint positions against gravity.
    For a simple pendulum model: tau = m * g * l * sin(q)
    For full arms, load a dynamics model instead.

    Args:
        masses: Link masses [kg].
        lengths: Center-of-mass distances from each joint [m].
        g: Gravity magnitude (m/s^2). Default 9.81.
    """

    def __init__(self, masses: list[float], lengths: list[float], g: float = 9.81):
        if len(masses) != len(lengths):
            raise ValueError("masses and lengths must have the same length")
        self.masses = masses
        self.lengths = lengths
        self.g = g

    def compute(self, joint_angles: list[float]) -> list[float]:
        """Compute gravity compensation torques for each joint.

        Uses a simplified pendulum model per joint. For accurate results
        on complex arms, use the full dynamics from kinematics.dynamics.

        Args:
            joint_angles: Joint positions in radians.

        Returns:
            Gravity torques [Nm] for each joint.
        """
        n = len(joint_angles)
        torques = [0.0] * n
        cumulative = 0.0
        for i in range(n):
            cumulative += self.masses[i] * self.g * self.lengths[i] * math.sin(joint_angles[i])
            torques[i] = cumulative
        return torques

    def set_gravity(self, g: float):
        self.g = g


class FrictionModel:
    """Coulomb + viscous friction model.

    friction = Coulomb_coeff * sign(velocity) + viscous_coeff * velocity

    Args:
        coulomb_coefficient: Coulomb (dry) friction magnitude.
        viscous_coefficient: Viscous friction coefficient.
        stiction_threshold: Velocity below which stiction applies (rad/s or m/s).
    """

    def __init__(
        self,
        coulomb_coefficient: float = 0.1,
        viscous_coefficient: float = 0.01,
        stiction_threshold: float = 0.01,
    ):
        self.coulomb = coulomb_coefficient
        self.viscous = viscous_coefficient
        self.stiction_threshold = stiction_threshold

    def compute(self, velocity: float) -> float:
        """Compute friction force/torque.

        Args:
            velocity: Joint velocity (rad/s or m/s).

        Returns:
            Friction torque/force.
        """
        if abs(velocity) < self.stiction_threshold:
            return 0.0
        return self.coulomb * (1 if velocity > 0 else -1) + self.viscous * velocity

    def compute_array(self, velocities: list[float]) -> list[float]:
        """Compute friction for multiple joints."""
        return [self.compute(v) for v in velocities]
