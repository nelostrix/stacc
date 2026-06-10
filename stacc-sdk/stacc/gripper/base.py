from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


class Gripper:
    """Base class for all end effectors.

    Provides the core state machine (open / closing / closed / opening),
    force/width tracking, and grasp detection. Subclasses implement the
    specific open/close/update logic.

    Args:
        name: Gripper identifier (default "gripper")
    """

    def __init__(self, name: str = "gripper"):
        self.name = name
        self._state: str = "open"
        self._current_width: float = 0.0
        self._current_force: float = 0.0
        self._target_force: float = 0.0
        self._grasp_detected: bool = False
        self._mount_link: Optional[str] = None

    # ── Properties ──────────────────────

    @property
    def is_open(self) -> bool:
        return self._state == "open"

    @property
    def is_closed(self) -> bool:
        return self._state == "closed"

    @property
    def is_grasping(self) -> bool:
        return self._grasp_detected and self._state == "closed"

    @property
    def state(self) -> str:
        return self._state

    @property
    def width(self) -> float:
        return self._current_width

    @property
    def force(self) -> float:
        return self._current_force

    # ── Public API ──────────────────────

    def open(self, speed: float = 1.0) -> None:
        """Open the gripper. Subclasses must override."""
        raise NotImplementedError

    def close(
        self, force: Optional[float] = None, speed: float = 1.0
    ) -> None:
        """Close the gripper. Subclasses must override."""
        raise NotImplementedError

    def stop(self) -> None:
        """Stop any in-progress motion immediately."""
        if self._state == "opening":
            self._state = "open"
        elif self._state == "closing":
            self._state = "closed"

    def reset(self) -> None:
        """Reset gripper to fully open state with zero force."""
        self._state = "open"
        self._current_width = 0.0
        self._current_force = 0.0
        self._target_force = 0.0
        self._grasp_detected = False

    def update(self, dt: float) -> None:
        """Update gripper state. Called by the simulation loop.

        Args:
            dt: Time step in seconds
        """
        pass

    def to_dict(self) -> Dict[str, Any]:
        """Serialize gripper state to a dictionary."""
        return {
            "name": self.name,
            "type": self.__class__.__name__,
            "state": self._state,
            "width": self._current_width,
            "force": self.force,
            "grasping": self._grasp_detected,
        }

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}("
            f"name={self.name!r}, "
            f"state={self._state!r}, "
            f"width={self._current_width:.3f}, "
            f"force={self._current_force:.1f}, "
            f"grasping={self._grasp_detected})"
        )
