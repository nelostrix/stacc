from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from stacc.sensors.base import BaseSensor


class Camera(BaseSensor):
    """RGB camera sensor.

    When connected to MuJoCo, renders the scene from the camera viewpoint.
    Otherwise returns a blank image.

    Args:
        width: Image width in pixels (must be > 0)
        height: Image height in pixels (must be > 0)
        fov: Field of view in degrees (must be > 0)
        rate: Frame rate in Hz
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fov: float = 60.0,
        rate: float = 30.0,
        **kwargs,
    ):
        super().__init__(name="camera", rate=rate, **kwargs)
        if width <= 0:
            raise ValueError(f"width must be > 0, got {width}")
        if height <= 0:
            raise ValueError(f"height must be > 0, got {height}")
        if fov <= 0 or fov >= 180:
            raise ValueError(f"fov must be in (0, 180), got {fov}")

        self.width = width
        self.height = height
        self.fov = fov

        self._image = None
        self._mj_model = None
        self._mj_data = None
        self._mj_renderer = None

    def reset(self):
        """Clear camera image and backend references."""
        self._image = None
        self._mj_model = None
        self._mj_data = None
        self._mj_renderer = None

    def set_mujoco_backend(self, model, data):
        self._mj_model = model
        self._mj_data = data
        try:
            import mujoco

            self._mj_renderer = mujoco.Renderer(
                model, height=self.height, width=self.width
            )
        except Exception:
            self._mj_renderer = None

    def read(self) -> Dict[str, Any]:
        if self._mj_renderer is not None:
            self._capture_mujoco()
        else:
            self._capture_synthetic()

        return {
            "image": self._image,
            "width": self.width,
            "height": self.height,
            "channels": 3,
        }

    def _capture_mujoco(self):
        try:
            import mujoco

            self._mj_renderer.update_scene(self._mj_data)
            self._image = self._mj_renderer.render()
        except Exception:
            self._capture_synthetic()

    def _capture_synthetic(self):
        try:
            import numpy as np

            self._image = np.zeros((self.height, self.width, 3), dtype="uint8")
        except ImportError:
            self._image = None

    def get_image(self):
        return self._image

    @property
    def resolution(self) -> Tuple[int, int]:
        return (self.width, self.height)

    def __repr__(self) -> str:
        return (
            f"Camera(name='{self.name}', resolution=({self.width}x{self.height}), "
            f"fov={self.fov}, rate={self.rate}Hz)"
        )
