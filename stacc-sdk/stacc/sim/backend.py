"""The physics backend contract — this is what makes simulation agnostic.

Today `SimulationEngine._init_physics` picks a backend through a hardcoded
if/elif chain, which means adding a new engine (PhysX, a custom solver)
means editing engine.py itself. That's the opposite of agnostic: the whole
point of one simulation API over multiple physics engines is that a new
backend is something you *register*, not something you patch the core
loop for.

Every backend — MuJoCo, the internal fallback, or a future one — must:
    - accept (robot, config) in its constructor
    - implement step(dt): advance physics by dt, read robot._state
      joint_torques in, write joint_positions/joint_velocities out
    - implement is_available(): a classmethod/staticmethod cheap check
      (usually "can I import my dependency") used before construction
    - optionally implement close(): release resources (viewers, sim handles)

`register_backend`/`get_backend` are the plug-in point: a new backend
calls register_backend("physx", PhysXBackend) once, anywhere, and
SimulationEngine picks it up with zero changes to engine.py.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Dict, Type, Any

if TYPE_CHECKING:
    from stacc.core.robot_base import Robot
    from stacc.sim.engine import SimulationConfig


class PhysicsBackend(ABC):
    """Contract every physics backend must satisfy."""

    def __init__(self, robot: "Robot", config: "SimulationConfig"):
        self.robot = robot
        self.config = config

    @classmethod
    def is_available(cls) -> bool:
        """Cheap check for whether this backend's dependency is usable.

        Default: assume available. Backends with an optional dependency
        (mujoco, pybullet, ...) should override this with a try/import.
        """
        return True

    @abstractmethod
    def step(self, dt: float) -> None:
        """Advance physics by dt, syncing joint state with the robot."""
        raise NotImplementedError

    def close(self) -> None:
        """Release backend resources. No-op by default."""

    def reset(self) -> None:
        """Reset backend state. No-op by default."""

    def capabilities(self) -> Dict[str, bool]:
        """Declared engine capabilities for feature negotiation."""
        return {
            "step": True,
            "reset": True,
            "snapshot": False,
            "restore": False,
            "read_state": False,
            "write_controls": False,
        }

    def snapshot(self) -> bytes:
        """Capture opaque bit-identical state snapshot."""
        raise NotImplementedError("Snapshot not supported by this backend")

    def restore(self, snapshot_bytes: bytes) -> None:
        """Restore backend state from an opaque snapshot."""
        raise NotImplementedError("Restore not supported by this backend")

    def read_state(self) -> Dict[str, Any]:
        """Read engine-level state representation."""
        raise NotImplementedError("read_state not supported by this backend")

    def write_controls(self, controls: Any) -> None:
        """Write low-level actuator or torque controls."""
        raise NotImplementedError("write_controls not supported by this backend")


_REGISTRY: Dict[str, Type[PhysicsBackend]] = {}


def register_backend(name: str, backend_cls: Type[PhysicsBackend]) -> None:
    """Register a physics backend under a name used by SimulationConfig.physics_engine."""
    _REGISTRY[name.lower()] = backend_cls


def get_backend(name: str) -> Type[PhysicsBackend]:
    """Look up a registered backend class. Raises KeyError if not registered."""
    return _REGISTRY[name.lower()]


def available_backends() -> Dict[str, bool]:
    """Every registered backend name mapped to whether it's actually usable right now."""
    return {name: cls.is_available() for name, cls in _REGISTRY.items()}
