from __future__ import annotations

from typing import Any, Dict, List

from stacc.sensors.base import BaseSensor


class ForceTorqueSensor(BaseSensor):
    """6-axis force/torque sensor.

    Measures forces (Fx, Fy, Fz) and torques (Tx, Ty, Tz)
    at a specific joint or link.

    Args:
        range_force: Maximum measurable force in Newtons
        range_torque: Maximum measurable torque in Nm
        rate: Sampling rate in Hz
    """

    def __init__(
        self,
        range_force: float = 100.0,
        range_torque: float = 10.0,
        rate: float = 1000.0,
        noise_stddev: float = 0.1,
        **kwargs,
    ):
        super().__init__(
            name="force_torque", rate=rate, noise_stddev=noise_stddev, **kwargs
        )
        self.range_force = range_force
        self.range_torque = range_torque
        self._wrench = [0.0] * 6  # [Fx, Fy, Fz, Tx, Ty, Tz]

    def read(self) -> Dict[str, Any]:
        return {
            "force": self.add_noise_vector(self._wrench[:3]),
            "torque": self.add_noise_vector(self._wrench[3:]),
            "wrench": self.add_noise_vector(self._wrench),
        }

    def get_force(self) -> List[float]:
        return self.add_noise_vector(self._wrench[:3])

    def get_torque(self) -> List[float]:
        return self.add_noise_vector(self._wrench[3:])
