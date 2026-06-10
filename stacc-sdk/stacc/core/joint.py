"""Joint types for robot definitions.

Supports revolute, prismatic, continuous, fixed, and spherical joints
with configurable limits, damping, friction, and effort bounds.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple, Dict, Any


class Joint:
    """A joint connecting two links.

    Args:
        name: Unique joint identifier
        type: Joint type — "revolute", "prismatic", "continuous", "fixed", "spherical"
        parent_link: Name of the parent link
        child_link: Name of the child link
        axis: Joint axis of rotation/translation [x, y, z]
        limits: (lower, upper) bounds in radians or meters
        max_velocity: Maximum joint velocity (rad/s or m/s)
        max_effort: Maximum joint torque/force (Nm or N)
        damping: Viscous damping coefficient
        friction: Coulomb friction
    """

    VALID_TYPES = ("revolute", "prismatic", "continuous", "fixed", "spherical")

    def __init__(
        self,
        name: str,
        type: str = "revolute",
        parent_link: Optional[str] = None,
        child_link: Optional[str] = None,
        axis: Optional[list] = None,
        limits: Optional[Tuple[float, float]] = None,
        max_velocity: float = 10.0,
        max_effort: float = 100.0,
        damping: float = 0.0,
        friction: float = 0.0,
        origin: Optional[list] = None,
    ):
        if type not in self.VALID_TYPES:
            raise ValueError(f"Invalid joint type '{type}'. Must be one of {self.VALID_TYPES}")

        self.name = name
        self.type = type
        self.parent_link = parent_link
        self.child_link = child_link
        self.axis = axis or [0, 0, 1]
        self.max_velocity = max_velocity
        self.max_effort = max_effort
        self.damping = damping
        self.friction = friction
        self.origin = origin or [0, 0, 0, 0, 0, 0]  # x,y,z,roll,pitch,yaw

        # Set default limits based on type
        if limits is not None:
            self.limits = limits
        elif type == "revolute":
            self.limits = (-math.pi, math.pi)
        elif type == "prismatic":
            self.limits = (-1.0, 1.0)
        elif type == "continuous":
            self.limits = (-math.inf, math.inf)
        elif type == "fixed":
            self.limits = (0.0, 0.0)
        elif type == "spherical":
            self.limits = (-math.pi, math.pi)

        # Current state
        self.position: float = 0.0
        self.velocity: float = 0.0
        self.effort: float = 0.0

    def clamp(self, value: float) -> float:
        """Clamp a value to this joint's limits."""
        if self.type in ("continuous", "fixed"):
            return value
        return max(self.limits[0], min(self.limits[1], value))

    def clamp_velocity(self, vel: float) -> float:
        """Clamp velocity to max."""
        return max(-self.max_velocity, min(self.max_velocity, vel))

    def clamp_effort(self, effort: float) -> float:
        """Clamp effort/torque to max."""
        return max(-self.max_effort, min(self.max_effort, effort))

    @property
    def is_actuated(self) -> bool:
        return self.type != "fixed"

    @property
    def range(self) -> float:
        """Total range of motion."""
        if self.type == "continuous":
            return math.inf
        return self.limits[1] - self.limits[0]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "type": self.type,
            "parent_link": self.parent_link,
            "child_link": self.child_link,
            "axis": self.axis,
            "limits": list(self.limits),
            "max_velocity": self.max_velocity,
            "max_effort": self.max_effort,
            "damping": self.damping,
            "friction": self.friction,
        }

    def __repr__(self) -> str:
        lim = f"[{self.limits[0]:.2f}, {self.limits[1]:.2f}]" if self.type != "continuous" else "[-inf, inf]"
        return f"Joint('{self.name}', type='{self.type}', limits={lim})"
