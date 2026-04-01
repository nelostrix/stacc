# ══════════════════════════════════════════
# STEREO CAMERA
# ══════════════════════════════════════════
from __future__ import annotations

from typing import Any, Dict, Tuple

from stacc.sensors.base import BaseSensor
from stacc.sensors.camera import Camera


class StereoCamera(BaseSensor):
    """Stereo camera pair for depth estimation.

    Args:
        baseline: Distance between left and right cameras in meters
        width: Image width per camera
        height: Image height per camera
        fov: Field of view in degrees
    """

    def __init__(
        self,
        baseline: float = 0.12,
        width: int = 640,
        height: int = 480,
        fov: float = 90.0,
        rate: float = 30.0,
        **kwargs,
    ):
        super().__init__(name="stereo_camera", rate=rate, **kwargs)
        self.baseline = baseline
        self.left = Camera(width=width, height=height, fov=fov, rate=rate)
        self.right = Camera(width=width, height=height, fov=fov, rate=rate)

    def set_mujoco_backend(self, model, data):
        self.left.set_mujoco_backend(model, data)
        self.right.set_mujoco_backend(model, data)

    def read(self) -> Dict[str, Any]:
        left_data = self.left.read()
        right_data = self.right.read()
        return {
            "left": left_data["image"],
            "right": right_data["image"],
            "baseline": self.baseline,
        }

    @property
    def resolution(self) -> Tuple[int, int]:
        return self.left.resolution
