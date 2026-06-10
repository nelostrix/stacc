from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


class Gripper:
    # "Base class for all end effectors."

    def __init__(self, name: str = "gripper"):
        self.name = name
        self._state = "open"

        self._current_width = 0.0
        self._current_force = 0.0
        self._target_force = 0.0
        self._grasp_detected = False
        self._mount_link = None

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

    def open(self, speed: float = 1.0):
        raise NotImplementedError

    def close(self, force: Optional[float] = None, speed: float = 1.0):
        raise NotImplementedError

    def stop(self):
        self._state = (
            self._state.replace("ing", "ed") if "ing" in self._state else self._state
        )

    def update(self, dt: float):
        "update gripper state. Called by simulation loop."
        pass

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "type": self.__class__.__name__,
            "state": self._state,
            "width": self._current_width,
            "force": self.force,
            "grasping": self._grasp_detected,
        }
