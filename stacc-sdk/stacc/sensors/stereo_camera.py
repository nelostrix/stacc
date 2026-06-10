from __future__ import annotations

from typing import Any, Dict, Tuple

from stacc.sensors.base import BaseSensor
from stacc.sensors.camera import Camera


class StereoCamera(BaseSensor):
    """Stereo camera pair for depth estimation.

    Args:
        baseline: Distance between left and right cameras in meters (must be > 0)
        width: Image width per camera (must be > 0)
        height: Image height per camera (must be > 0)
        fov: Field of view in degrees (must be in (0, 180))
        rate: Frame rate in Hz
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
        if baseline <= 0:
            raise ValueError(f"baseline must be > 0, got {baseline}")

        self.baseline = baseline
        self.left = Camera(width=width, height=height, fov=fov, rate=rate)
        self.right = Camera(width=width, height=height, fov=fov, rate=rate)

    def reset(self):
        """Reset both cameras."""
        self.left.reset()
        self.right.reset()

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

    def __repr__(self) -> str:
        return (
            f"StereoCamera(name='{self.name}', baseline={self.baseline}m, "
            f"resolution=({self.left.width}x{self.left.height}), rate={self.rate}Hz)"
        )
