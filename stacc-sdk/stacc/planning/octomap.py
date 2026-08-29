"""3D Voxel OctoMap & Euclidean Signed Distance Field (ESDF).

Provides 3D spatial occupancy representation and continuous distance gradient queries
for real-time collision avoidance and trajectory optimization.
"""

from __future__ import annotations
from typing import List, Optional, Tuple, Union
import numpy as np


class VoxelOctoMap:
    """3D Voxel Grid & Distance Field Representation.

    Args:
        min_bound: 3D minimum workspace coordinate [xmin, ymin, zmin]
        max_bound: 3D maximum workspace coordinate [xmax, ymax, zmax]
        resolution: Voxel side length in meters (default 0.05m = 5cm)
    """

    def __init__(
        self,
        min_bound: Tuple[float, float, float] = (-1.0, -1.0, 0.0),
        max_bound: Tuple[float, float, float] = (1.0, 1.0, 1.5),
        resolution: float = 0.05
    ):
        self.min_b = np.array(min_bound, dtype=np.float64)
        self.max_b = np.array(max_bound, dtype=np.float64)
        self.res = float(resolution)

        self.dim = np.ceil((self.max_b - self.min_b) / self.res).astype(int)
        # Log-odds occupancy grid (0.0 = unknown, >0 occupied, <0 free)
        self.grid = np.zeros(self.dim, dtype=np.float32)

    def point_to_index(self, point: np.ndarray) -> Optional[Tuple[int, int, int]]:
        """Convert a 3D Cartesian point (x, y, z) into voxel grid indices (i, j, k)."""
        idx = np.floor((point - self.min_b) / self.res).astype(int)
        if np.all(idx >= 0) and np.all(idx < self.dim):
            return int(idx[0]), int(idx[1]), int(idx[2])
        return None

    def insert_point_cloud(self, points: np.ndarray, is_occupied: bool = True) -> None:
        """Insert a batch of 3D sensor points (N x 3) into the voxel grid."""
        val = 1.5 if is_occupied else -0.5
        for pt in points:
            idx = self.point_to_index(pt)
            if idx is not None:
                self.grid[idx] = np.clip(self.grid[idx] + val, -5.0, 5.0)

    def is_occupied(self, point: np.ndarray, threshold: float = 0.5) -> bool:
        """Query if a 3D point is occupied."""
        idx = self.point_to_index(point)
        if idx is not None:
            return bool(self.grid[idx] >= threshold)
        return False
