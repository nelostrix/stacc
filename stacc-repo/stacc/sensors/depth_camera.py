from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from stacc.sensors.base import BaseSensor


class DepthCamera(BaseSensor):
    """Depth camera sensor (RGB-D).

    Produces both RGB image and depth map.

    Args:
        width: Image width in pixels
        height: Image height in pixels
        fov: Field of view in degrees
        range_max: Maximum depth in meters
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
        self.width = width
        self.height = height
        self.fov = fov
        self.range_max = range_max

        self._rgb = None  # (H, W, 3) uint8
        self._depth = None  # (H, W) float32 in meters
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
            "rgb": self._rgb,
            "depth": self._depth,
            "width": self.width,
            "height": self.height,
        }

    def _capture_mujoco(self):
        try:
            import mujoco

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
        """Convert depth map to 3D point cloud."""
        if self._depth is None:
            return None

        try:
            import numpy as np
        except ImportError:
            return None

        points = []
        fx = self.width / (2 * math.tan(math.radians(self.fov / 2)))
        fy = fx  # square pixels
        cx, cy = self.width / 2, self.height / 2

        # Subsample for performance
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
