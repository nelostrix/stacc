# ══════════════════════════════════════════
# CONTACT SENSOR
# ══════════════════════════════════════════
from __future__ import annotations

from typing import Any, Dict

from stacc.sensors.base import BaseSensor


class ContactSensor(BaseSensor):
    """Binary or analog contact/touch sensor.

    Detects contact forces on a specific link.

    Args:
        threshold: Force threshold for binary contact detection (N)
        rate: Sampling rate in Hz
    """

    def __init__(
        self,
        threshold: float = 0.1,
        rate: float = 1000.0,
        **kwargs,
    ):
        super().__init__(name="contact", rate=rate, **kwargs)
        self.threshold = threshold
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
