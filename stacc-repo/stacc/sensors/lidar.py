from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from stacc.sensors.base import BaseSensor

# ══════════════════════════════════════════
# LIDAR SENSOR
# ══════════════════════════════════════════


class LiDAR(BaseSensor):
    """LiDAR sensor — 2D or 3D laser scanner.

    Generates range measurements by raycasting into the scene.
    When connected to MuJoCo, uses mj_ray for accurate raycasting.
    Otherwise returns configurable default/noise values.

    Args:
        channels: Number of vertical channels (1 = 2D LiDAR, 16/32/64 = 3D)
        points_per_channel: Horizontal resolution per channel
        range_max: Maximum detection range in meters
        range_min: Minimum detection range in meters
        fov_h: Horizontal field of view in degrees (360 for full rotation)
        fov_v: Vertical field of view in degrees (0 for 2D)
        rate: Scan rate in Hz
        noise_stddev: Range noise standard deviation in meters
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
        self.channels = channels
        self.points_per_channel = points_per_channel
        self.range_max = range_max
        self.range_min = range_min
        self.fov_h = fov_h
        self.fov_v = fov_v

        # Precompute ray directions
        self._ray_directions = self._compute_ray_directions()

        # Latest scan data
        self._ranges = []  # flat list of range values
        self._points = []  # list of [x, y, z] points
        self._intensities = []  # reflection intensity per point

        # MuJoCo backend reference (set by simulation engine)
        self._mj_model = None
        self._mj_data = None

    def _compute_ray_directions(self) -> List[Tuple[float, float, float]]:
        """Precompute unit direction vectors for all rays."""
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
        """Connect to MuJoCo for raycasting (called by simulation engine)."""
        self._mj_model = model
        self._mj_data = data

    def read(self) -> Dict[str, Any]:
        """Perform a scan and return range data."""
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
        """Raycast into MuJoCo scene for accurate range data."""
        try:
            import mujoco
            import numpy as np
        except ImportError:
            self._scan_synthetic()
            return

        self._ranges = []
        self._points = []

        # Get sensor mount position from robot
        # Default to origin if not determinable
        mount_pos = np.array([0.0, 0.0, 1.0])  # default height

        for dx, dy, dz in self._ray_directions:
            direction = np.array([dx, dy, dz])
            geom_id = np.array([-1], dtype=np.int32)

            dist = mujoco.mj_ray(
                self._mj_model,
                self._mj_data,
                mount_pos,
                direction,
                None,  # geom group filter (None = all)
                1,  # exclude static geoms flag
                -1,  # bodyexclude
                geom_id,
            )

            if dist < 0 or dist > self.range_max:
                dist = self.range_max
            elif dist < self.range_min:
                dist = self.range_min

            # Add noise
            dist = self.add_noise(dist)

            self._ranges.append(dist)

            if dist < self.range_max:
                px = mount_pos[0] + dx * dist
                py = mount_pos[1] + dy * dist
                pz = mount_pos[2] + dz * dist
                self._points.append([px, py, pz])

    def _scan_synthetic(self):
        """Generate synthetic scan data (no physics backend)."""
        self._ranges = []
        self._points = []

        for dx, dy, dz in self._ray_directions:
            # Simulate a flat ground plane at z=0
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
        """Get the latest range measurements."""
        return list(self._ranges)

    def get_point_cloud(self) -> List[List[float]]:
        """Get the latest 3D point cloud."""
        return [list(p) for p in self._points]

    @property
    def num_rays(self) -> int:
        return self.channels * self.points_per_channel
