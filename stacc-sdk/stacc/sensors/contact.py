from __future__ import annotations

from typing import Any, Dict

from stacc.sensors.base import BaseSensor


class ContactSensor(BaseSensor):
    """Binary or analog contact/touch sensor.

    Detects contact forces on a specific link.

    Args:
        threshold: Force threshold for binary contact detection in N (must be > 0)
        rate: Sampling rate in Hz
    """

    def __init__(
        self,
        threshold: float = 0.1,
        rate: float = 1000.0,
        **kwargs,
    ):
        super().__init__(name="contact", rate=rate, **kwargs)
        if threshold <= 0:
            raise ValueError(f"threshold must be > 0, got {threshold}")

        self.threshold = threshold
        self._force = 0.0
        self._in_contact = False

    def reset(self):
        """Clear contact state."""
        self._force = 0.0
        self._in_contact = False

    def read(self) -> Dict[str, Any]:
        return {
            "in_contact": self._force > self.threshold,
            "force": self._force,
        }

    @property
    def is_touching(self) -> bool:
        return self._force > self.threshold

    def __repr__(self) -> str:
        return (
            f"ContactSensor(name='{self.name}', threshold={self.threshold}N, "
            f"rate={self.rate}Hz)"
        )
