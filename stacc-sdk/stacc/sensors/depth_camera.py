from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from stacc.sensors.base import BaseSensor


class DepthCamera(BaseSensor):
    """Depth camera sensor (RGB-D).

    Produces both RGB image and depth map.

    Args:
        width: Image width in pixels (must be > 0)
        height: Image height in pixels (must be > 0)
        fov: Field of view in degrees (must be in (0, 180))
        range_max: Maximum depth in meters (must be > 0)
        rate: Frame rate in Hz
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fov: float = 60.0,
        range_max: float = 10.0,
        rate: float = 30.0,
        noise_stddev: float = 0.005,
        **kwargs,
    ):
        super().__init__(
            name="depth_camera", rate=rate, noise_stddev=noise_stddev, **kwargs
        )
        if width <= 0:
            raise ValueError(f"width must be > 0, got {width}")
        if height <= 0:
            raise ValueError(f"height must be > 0, got {height}")
        if fov <= 0 or fov >= 180:
            raise ValueError(f"fov must be in (0, 180), got {fov}")
        if range_max <= 0:
            raise ValueError(f"range_max must be > 0, got {range_max}")

        self.width = width
        self.height = height
        self.fov = fov
        self.range_max = range_max

        self._rgb = None
        self._depth = None
        self._mj_model = None
        self._mj_data = None
        self._mj_renderer = None
        self._renderer_attempted = False

    def reset(self):
        """Clear images and backend references."""
        self.close_renderer()
        self._rgb = None
        self._depth = None
        self._mj_model = None
        self._mj_data = None
        self._renderer_attempted = False

    def set_mujoco_backend(self, model, data):
        self.close_renderer()
        self._mj_model = model
        self._mj_data = data
        self._renderer_attempted = False

    def close_renderer(self):
        """Release the GL context on the thread that owns this renderer."""
        renderer = self._mj_renderer
        self._mj_renderer = None
        if renderer is not None:
            renderer.close()

    def read(self) -> Dict[str, Any]:
        if self._mj_model is not None and self._mj_data is not None:
            self._capture_mujoco()
        else:
            self._capture_synthetic()

        return {
            "rgb": self._rgb,
            "depth": self._depth,
            "width": self.width,
            "height": self.height,
        }

    def _capture_mujoco(self):
        try:
            import mujoco

            if self._mj_renderer is None and not self._renderer_attempted:
                self._renderer_attempted = True
                self._mj_renderer = mujoco.Renderer(
                    self._mj_model, height=self.height, width=self.width
                )
            if self._mj_renderer is None:
                self._capture_synthetic()
                return

            self._mj_renderer.update_scene(self._mj_data)
            self._rgb = self._mj_renderer.render()
            self._mj_renderer.enable_depth_rendering(True)
            self._depth = self._mj_renderer.render()
            self._mj_renderer.enable_depth_rendering(False)
        except Exception:
            self._capture_synthetic()

    def _capture_synthetic(self):
        try:
            import numpy as np

            self._rgb = np.zeros((self.height, self.width, 3), dtype="uint8")
            self._depth = np.full(
                (self.height, self.width), self.range_max, dtype="float32"
            )
        except ImportError:
            self._rgb = None
            self._depth = None

    def get_rgb(self):
        return self._rgb

    def get_depth(self):
        return self._depth

    def get_point_cloud(self) -> Optional[List[List[float]]]:
        if self._depth is None:
            return None

        try:
            import numpy as np
        except ImportError:
            return None

        points = []
        fx = self.width / (2 * math.tan(math.radians(self.fov / 2)))
        fy = fx
        cx, cy = self.width / 2, self.height / 2

        step = max(1, min(self.width, self.height) // 64)

        for v in range(0, self.height, step):
            for u in range(0, self.width, step):
                z = float(self._depth[v, u])
                if z <= 0 or z >= self.range_max:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])

        return points

    def __repr__(self) -> str:
        return (
            f"DepthCamera(name='{self.name}', resolution=({self.width}x{self.height}), "
            f"fov={self.fov}, range_max={self.range_max}m, rate={self.rate}Hz)"
        )
