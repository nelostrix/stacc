from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from stacc.sensors.base import BaseSensor


class LiDAR(BaseSensor):
    """LiDAR sensor — 2D or 3D laser scanner.

    Generates range measurements by raycasting into the scene.
    When connected to MuJoCo, uses mj_ray for accurate raycasting.
    Otherwise returns configurable default/noise values.

    Args:
        channels: Number of vertical channels (must be >= 1; 1 = 2D, 16/32/64 = 3D)
        points_per_channel: Horizontal resolution per channel (must be >= 1)
        range_max: Maximum detection range in meters (must be > 0)
        range_min: Minimum detection range in meters (must be >= 0, < range_max)
        fov_h: Horizontal field of view in degrees (must be > 0)
        fov_v: Vertical field of view in degrees (must be >= 0)
        rate: Scan rate in Hz
        noise_stddev: Range noise standard deviation in meters (must be >= 0)
    """

    def __init__(
        self,
        channels: int = 1,
        points_per_channel: int = 360,
        range_max: float = 30.0,
        range_min: float = 0.1,
        fov_h: float = 360.0,
        fov_v: float = 30.0,
        rate: float = 10.0,
        noise_stddev: float = 0.01,
        **kwargs,
    ):
        super().__init__(name="lidar", rate=rate, noise_stddev=noise_stddev, **kwargs)
        if channels < 1:
            raise ValueError(f"channels must be >= 1, got {channels}")
        if points_per_channel < 1:
            raise ValueError(
                f"points_per_channel must be >= 1, got {points_per_channel}"
            )
        if range_max <= 0:
            raise ValueError(f"range_max must be > 0, got {range_max}")
        if range_min < 0 or range_min >= range_max:
            raise ValueError(
                f"range_min ({range_min}) must be >= 0 and < range_max ({range_max})"
            )
        if fov_h <= 0:
            raise ValueError(f"fov_h must be > 0, got {fov_h}")
        if fov_v < 0:
            raise ValueError(f"fov_v must be >= 0, got {fov_v}")

        self.channels = channels
        self.points_per_channel = points_per_channel
        self.range_max = range_max
        self.range_min = range_min
        self.fov_h = fov_h
        self.fov_v = fov_v

        self._ray_directions = self._compute_ray_directions()

        self._ranges = []
        self._points = []
        self._intensities = []

        self._mj_model = None
        self._mj_data = None

    def reset(self):
        """Clear scan data."""
        self._ranges = []
        self._points = []
        self._intensities = []

    def _compute_ray_directions(self) -> List[Tuple[float, float, float]]:
        directions = []
        h_step = math.radians(self.fov_h) / max(self.points_per_channel, 1)
        v_step = (
            math.radians(self.fov_v) / max(self.channels, 1) if self.channels > 1 else 0
        )

        v_start = -math.radians(self.fov_v) / 2

        for ch in range(self.channels):
            v_angle = v_start + ch * v_step if self.channels > 1 else 0
            cos_v = math.cos(v_angle)
            sin_v = math.sin(v_angle)

            for pt in range(self.points_per_channel):
                h_angle = -math.radians(self.fov_h) / 2 + pt * h_step
                dx = cos_v * math.cos(h_angle)
                dy = cos_v * math.sin(h_angle)
                dz = sin_v
                directions.append((dx, dy, dz))

        return directions

    def set_mujoco_backend(self, model, data):
        self._mj_model = model
        self._mj_data = data

    def read(self) -> Dict[str, Any]:
        if self._mj_model is not None and self._mj_data is not None:
            self._scan_mujoco()
        else:
            self._scan_synthetic()

        return {
            "ranges": self._ranges,
            "points": self._points,
            "num_points": len(self._points),
            "channels": self.channels,
        }

    def _scan_mujoco(self):
        try:
            import mujoco
            import numpy as np
        except ImportError:
            self._scan_synthetic()
            return

        self._ranges = []
        self._points = []

        mount_pos = np.array([0.0, 0.0, 1.0])

        for dx, dy, dz in self._ray_directions:
            direction = np.array([dx, dy, dz])
            geom_id = np.array([-1], dtype=np.int32)

            dist = mujoco.mj_ray(
                self._mj_model,
                self._mj_data,
                mount_pos,
                direction,
                None,
                1,
                -1,
                geom_id,
            )

            if dist < 0 or dist > self.range_max:
                dist = self.range_max
            elif dist < self.range_min:
                dist = self.range_min

            dist = self.add_noise(dist)
            self._ranges.append(dist)

            if dist < self.range_max:
                px = mount_pos[0] + dx * dist
                py = mount_pos[1] + dy * dist
                pz = mount_pos[2] + dz * dist
                self._points.append([px, py, pz])

    def _scan_synthetic(self):
        self._ranges = []
        self._points = []

        for dx, dy, dz in self._ray_directions:
            if dz < -0.01:
                mount_height = 1.0
                dist = mount_height / abs(dz)
                dist = min(dist, self.range_max)
            else:
                dist = self.range_max

            dist = self.add_noise(dist)
            self._ranges.append(dist)

            if dist < self.range_max:
                self._points.append([dx * dist, dy * dist, dz * dist])

    def get_ranges(self) -> List[float]:
        return list(self._ranges)

    def get_point_cloud(self) -> List[List[float]]:
        return [list(p) for p in self._points]

    @property
    def num_rays(self) -> int:
        return self.channels * self.points_per_channel

    def __repr__(self) -> str:
        return (
            f"LiDAR(name='{self.name}', channels={self.channels}, "
            f"points_per_channel={self.points_per_channel}, "
            f"range=[{self.range_min}, {self.range_max}]m, rate={self.rate}Hz)"
        )
