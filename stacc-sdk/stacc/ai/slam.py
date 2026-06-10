"""Universal SLAM — Simultaneous Localization and Mapping.

Provides multiple SLAM algorithms that work with any sensor
configuration (LiDAR, camera, depth, odometry) on any robot.

Algorithms:
- EKF-SLAM: Extended Kalman Filter — optimal for small maps, landmark-based
- Particle Filter SLAM (FastSLAM): Monte Carlo — scales to large maps
- ICP: Iterative Closest Point — scan matching for LiDAR-based SLAM
- Occupancy Grid Mapping: Build 2D maps from range data

Usage:
    from stacc.ai.slam import EKFSLAM, ParticleFilterSLAM, SLAMNavigator

    slam = SLAMNavigator(method="ekf")
    slam.initialize(robot)

    @robot.on_update
    def control(dt):
        slam.update(dt)
        pose = slam.get_pose()
        grid = slam.get_map()
"""

from __future__ import annotations

import math
import random
from copy import deepcopy
from typing import Any, Dict, List, Optional, Tuple

try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


# ══════════════════════════════════════════
# EKF-SLAM
# ══════════════════════════════════════════


class EKFSLAM:
    """Extended Kalman Filter SLAM.

    State vector: [x, y, theta, lm1_x, lm1_y, lm2_x, lm2_y, ...]
    Robot pose (3) + landmark positions (2 per landmark).

    Prediction: motion model with odometry
    Update: range-bearing observations of landmarks

    Args:
        motion_noise: [alpha1, alpha2, alpha3, alpha4] motion noise params
        observation_noise: [range_noise, bearing_noise] in meters/radians
        max_landmarks: Maximum number of landmarks to track
        association_threshold: Mahalanobis distance for data association
    """

    def __init__(
        self,
        motion_noise: Optional[List[float]] = None,
        observation_noise: Optional[List[float]] = None,
        max_landmarks: int = 100,
        association_threshold: float = 5.0,
    ):
        if not HAS_NUMPY:
            raise ImportError("numpy required for EKF-SLAM")

        self.motion_noise = motion_noise or [0.1, 0.01, 0.01, 0.1]
        self.obs_noise = observation_noise or [0.1, 0.05]
        self.max_landmarks = max_landmarks
        self.assoc_threshold = association_threshold

        # State: [x, y, theta]
        self._state = np.zeros(3)
        # Covariance
        self._cov = np.eye(3) * 0.001
        # Number of landmarks in state
        self._n_landmarks = 0
        # Landmark IDs
        self._landmark_ids = []

        # Observation noise covariance
        self._Q = np.diag([self.obs_noise[0] ** 2, self.obs_noise[1] ** 2])

    @property
    def pose(self) -> List[float]:
        return self._state[:3].tolist()

    @property
    def x(self) -> float:
        return float(self._state[0])

    @property
    def y(self) -> float:
        return float(self._state[1])

    @property
    def theta(self) -> float:
        return float(self._state[2])

    @property
    def landmarks(self) -> List[List[float]]:
        lms = []
        for i in range(self._n_landmarks):
            idx = 3 + 2 * i
            lms.append([float(self._state[idx]), float(self._state[idx + 1])])
        return lms

    @property
    def num_landmarks(self) -> int:
        return self._n_landmarks

    def predict(self, v: float, w: float, dt: float):
        """Predict step — apply motion model.

        Args:
            v: Linear velocity (m/s)
            w: Angular velocity (rad/s)
            dt: Time step
        """
        theta = self._state[2]
        n = len(self._state)

        if abs(w) > 1e-6:
            dx = -v / w * math.sin(theta) + v / w * math.sin(theta + w * dt)
            dy = v / w * math.cos(theta) - v / w * math.cos(theta + w * dt)
            dtheta = w * dt
        else:
            dx = v * math.cos(theta) * dt
            dy = v * math.sin(theta) * dt
            dtheta = 0.0

        self._state[0] += dx
        self._state[1] += dy
        self._state[2] = self._normalize_angle(self._state[2] + dtheta)

        # Jacobian of motion model w.r.t. state
        G = np.eye(n)
        if abs(w) > 1e-6:
            G[0, 2] = -v / w * math.cos(theta) + v / w * math.cos(theta + w * dt)
            G[1, 2] = -v / w * math.sin(theta) + v / w * math.sin(theta + w * dt)

        # Motion noise
        a = self.motion_noise
        M = np.diag(
            [
                a[0] * v * v + a[1] * w * w,
                a[2] * v * v + a[3] * w * w,
            ]
        )

        V = np.zeros((n, 2))
        if abs(w) > 1e-6:
            V[0, 0] = (-math.sin(theta) + math.sin(theta + w * dt)) / w
            V[0, 1] = (
                v * (math.sin(theta) - math.sin(theta + w * dt)) / (w * w)
                + v * math.cos(theta + w * dt) * dt / w
            )
            V[1, 0] = (math.cos(theta) - math.cos(theta + w * dt)) / w
            V[1, 1] = (
                -v * (math.cos(theta) - math.cos(theta + w * dt)) / (w * w)
                + v * math.sin(theta + w * dt) * dt / w
            )
        else:
            V[0, 0] = math.cos(theta) * dt
            V[1, 0] = math.sin(theta) * dt
        V[2, 1] = dt

        self._cov = G @ self._cov @ G.T + V @ M @ V.T

    def update(self, observations: List[Tuple[float, float, int]]):
        """Update step — incorporate range-bearing observations.

        Args:
            observations: List of (range, bearing, landmark_id).
                          landmark_id=-1 for new/unknown landmarks.
        """
        for z_range, z_bearing, lm_id in observations:
            if lm_id < 0 or lm_id >= self._n_landmarks:
                # Try data association or add new landmark
                lm_id = self._associate(z_range, z_bearing)
                if lm_id < 0:
                    if self._n_landmarks < self.max_landmarks:
                        lm_id = self._add_landmark(z_range, z_bearing)
                    else:
                        continue

            if lm_id < 0 or lm_id >= self._n_landmarks:
                continue

            self._update_landmark(z_range, z_bearing, lm_id)

    def _associate(self, z_range: float, z_bearing: float) -> int:
        """Find which existing landmark this observation matches."""
        best_id = -1
        best_dist = self.assoc_threshold

        rx, ry, rt = self._state[0], self._state[1], self._state[2]

        for i in range(self._n_landmarks):
            idx = 3 + 2 * i
            lx = self._state[idx]
            ly = self._state[idx + 1]

            dx = lx - rx
            dy = ly - ry
            pred_range = math.sqrt(dx * dx + dy * dy)
            pred_bearing = self._normalize_angle(math.atan2(dy, dx) - rt)

            innovation = np.array(
                [z_range - pred_range, self._normalize_angle(z_bearing - pred_bearing)]
            )

            # Simplified Mahalanobis distance
            dist = float(np.sqrt(innovation @ innovation))
            if dist < best_dist:
                best_dist = dist
                best_id = i

        return best_id

    def _add_landmark(self, z_range: float, z_bearing: float) -> int:
        """Add a new landmark to the state vector."""
        rx, ry, rt = self._state[0], self._state[1], self._state[2]
        lx = rx + z_range * math.cos(rt + z_bearing)
        ly = ry + z_range * math.sin(rt + z_bearing)

        n = len(self._state)
        new_state = np.zeros(n + 2)
        new_state[:n] = self._state
        new_state[n] = lx
        new_state[n + 1] = ly
        self._state = new_state

        new_cov = np.eye(n + 2) * 1000
        new_cov[:n, :n] = self._cov
        self._cov = new_cov

        lm_id = self._n_landmarks
        self._n_landmarks += 1
        self._landmark_ids.append(lm_id)

        return lm_id

    def _update_landmark(self, z_range: float, z_bearing: float, lm_id: int):
        """EKF update for a single landmark observation."""
        n = len(self._state)
        idx = 3 + 2 * lm_id

        rx, ry, rt = self._state[0], self._state[1], self._state[2]
        lx = self._state[idx]
        ly = self._state[idx + 1]

        dx = lx - rx
        dy = ly - ry
        q = dx * dx + dy * dy
        sq = math.sqrt(q)

        if sq < 1e-10:
            return

        pred_range = sq
        pred_bearing = self._normalize_angle(math.atan2(dy, dx) - rt)

        innovation = np.array(
            [z_range - pred_range, self._normalize_angle(z_bearing - pred_bearing)]
        )

        # Jacobian H
        H = np.zeros((2, n))
        H[0, 0] = -dx / sq
        H[0, 1] = -dy / sq
        H[0, idx] = dx / sq
        H[0, idx + 1] = dy / sq
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1
        H[1, idx] = -dy / q
        H[1, idx + 1] = dx / q

        S = H @ self._cov @ H.T + self._Q
        K = self._cov @ H.T @ np.linalg.inv(S)

        self._state = self._state + K @ innovation
        self._state[2] = self._normalize_angle(self._state[2])
        self._cov = (np.eye(n) - K @ H) @ self._cov

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def reset(self, pose: Optional[List[float]] = None):
        self._state = np.array(pose or [0.0, 0.0, 0.0], dtype=float)
        self._cov = np.eye(3) * 0.001
        self._n_landmarks = 0
        self._landmark_ids = []


# ══════════════════════════════════════════
# PARTICLE FILTER SLAM (FastSLAM)
# ══════════════════════════════════════════


class Particle:
    """Single particle for FastSLAM."""

    __slots__ = ("x", "y", "theta", "weight", "landmarks")

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 1.0
        self.landmarks: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}


class ParticleFilterSLAM:
    """FastSLAM — particle filter based SLAM.

    Each particle carries its own pose estimate and landmark map.
    Scales better than EKF-SLAM for large environments.

    Args:
        num_particles: Number of particles
        motion_noise: [x_noise, y_noise, theta_noise]
        observation_noise: [range_noise, bearing_noise]
        resample_threshold: Effective sample size ratio for resampling
    """

    def __init__(
        self,
        num_particles: int = 100,
        motion_noise: Optional[List[float]] = None,
        observation_noise: Optional[List[float]] = None,
        resample_threshold: float = 0.5,
    ):
        if not HAS_NUMPY:
            raise ImportError("numpy required for ParticleFilterSLAM")

        self.num_particles = num_particles
        self.motion_noise = motion_noise or [0.05, 0.05, 0.03]
        self.obs_noise = observation_noise or [0.1, 0.05]
        self.resample_threshold = resample_threshold

        self.particles = [Particle() for _ in range(num_particles)]

    @property
    def pose(self) -> List[float]:
        best = max(self.particles, key=lambda p: p.weight)
        return [best.x, best.y, best.theta]

    @property
    def mean_pose(self) -> List[float]:
        total_w = sum(p.weight for p in self.particles) or 1.0
        x = sum(p.x * p.weight for p in self.particles) / total_w
        y = sum(p.y * p.weight for p in self.particles) / total_w
        sin_t = sum(math.sin(p.theta) * p.weight for p in self.particles) / total_w
        cos_t = sum(math.cos(p.theta) * p.weight for p in self.particles) / total_w
        theta = math.atan2(sin_t, cos_t)
        return [x, y, theta]

    def predict(self, v: float, w: float, dt: float):
        mn = self.motion_noise
        for p in self.particles:
            vn = v + random.gauss(0, mn[0])
            wn = w + random.gauss(0, mn[2])
            if abs(wn) > 1e-6:
                p.x += -vn / wn * math.sin(p.theta) + vn / wn * math.sin(
                    p.theta + wn * dt
                )
                p.y += vn / wn * math.cos(p.theta) - vn / wn * math.cos(
                    p.theta + wn * dt
                )
            else:
                p.x += vn * math.cos(p.theta) * dt
                p.y += vn * math.sin(p.theta) * dt
            p.theta += wn * dt + random.gauss(0, mn[1])

    def update(self, observations: List[Tuple[float, float, int]]):
        for p in self.particles:
            for z_range, z_bearing, lm_id in observations:
                if lm_id not in p.landmarks:
                    lx = p.x + z_range * math.cos(p.theta + z_bearing)
                    ly = p.y + z_range * math.sin(p.theta + z_bearing)
                    mean = np.array([lx, ly])
                    cov = np.eye(2) * self.obs_noise[0] ** 2
                    p.landmarks[lm_id] = (mean, cov)
                    continue

                mean, cov = p.landmarks[lm_id]
                dx = mean[0] - p.x
                dy = mean[1] - p.y
                pred_range = math.sqrt(dx * dx + dy * dy)
                pred_bearing = math.atan2(dy, dx) - p.theta

                innovation = np.array(
                    [
                        z_range - pred_range,
                        EKFSLAM._normalize_angle(z_bearing - pred_bearing),
                    ]
                )

                Q = np.diag([self.obs_noise[0] ** 2, self.obs_noise[1] ** 2])
                w = math.exp(
                    -0.5
                    * float(
                        innovation @ np.linalg.inv(Q + np.eye(2) * 0.01) @ innovation
                    )
                )
                p.weight *= max(w, 1e-300)

        self._normalize_weights()
        if self._effective_sample_size() < self.resample_threshold * self.num_particles:
            self._resample()

    def _normalize_weights(self):
        total = sum(p.weight for p in self.particles)
        if total > 0:
            for p in self.particles:
                p.weight /= total

    def _effective_sample_size(self) -> float:
        return 1.0 / sum(p.weight**2 for p in self.particles)

    def _resample(self):
        weights = [p.weight for p in self.particles]
        indices = random.choices(
            range(self.num_particles), weights=weights, k=self.num_particles
        )
        new_particles = []
        for i in indices:
            p = self.particles[i]
            new_p = Particle(p.x, p.y, p.theta)
            new_p.landmarks = {
                k: (v[0].copy(), v[1].copy()) for k, v in p.landmarks.items()
            }
            new_p.weight = 1.0 / self.num_particles
            new_particles.append(new_p)
        self.particles = new_particles

    def reset(self, pose: Optional[List[float]] = None):
        p = pose or [0, 0, 0]
        self.particles = [Particle(p[0], p[1], p[2]) for _ in range(self.num_particles)]


# ══════════════════════════════════════════
# ICP SCAN MATCHING
# ══════════════════════════════════════════


class ICPMatcher:
    """Iterative Closest Point — align two point clouds / LiDAR scans.

    Used for scan-to-scan matching in LiDAR SLAM.

    Args:
        max_iterations: Maximum ICP iterations
        tolerance: Convergence threshold (meters)
        max_correspondence_distance: Maximum distance for point pairing
    """

    def __init__(
        self,
        max_iterations: int = 50,
        tolerance: float = 1e-4,
        max_correspondence_distance: float = 1.0,
    ):
        if not HAS_NUMPY:
            raise ImportError("numpy required for ICP")
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.max_dist = max_correspondence_distance

    def match(
        self, source: np.ndarray, target: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Align source point cloud to target.

        Args:
            source: Nx2 or Nx3 source points
            target: Mx2 or Mx3 target points

        Returns:
            (rotation_matrix, translation_vector, fitness_score)
        """
        src = source.copy()
        dim = src.shape[1]
        R_total = np.eye(dim)
        t_total = np.zeros(dim)

        for iteration in range(self.max_iterations):
            correspondences = self._find_correspondences(src, target)
            if len(correspondences) < 3:
                break

            src_matched = src[correspondences[:, 0]]
            tgt_matched = target[correspondences[:, 1]]

            R, t = self._compute_transform(src_matched, tgt_matched)

            src = (R @ src.T).T + t
            R_total = R @ R_total
            t_total = R @ t_total + t

            if np.linalg.norm(t) < self.tolerance:
                break

        fitness = self._compute_fitness(src, target)
        return R_total, t_total, fitness

    def _find_correspondences(self, source, target) -> np.ndarray:
        pairs = []
        for i, sp in enumerate(source):
            dists = np.linalg.norm(target - sp, axis=1)
            j = np.argmin(dists)
            if dists[j] < self.max_dist:
                pairs.append([i, j])
        return np.array(pairs) if pairs else np.zeros((0, 2), dtype=int)

    def _compute_transform(self, source, target):
        centroid_s = np.mean(source, axis=0)
        centroid_t = np.mean(target, axis=0)
        s_centered = source - centroid_s
        t_centered = target - centroid_t

        H = s_centered.T @ t_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_t - R @ centroid_s
        return R, t

    def _compute_fitness(self, source, target) -> float:
        total = 0.0
        count = 0
        for sp in source:
            dists = np.linalg.norm(target - sp, axis=1)
            min_d = np.min(dists)
            if min_d < self.max_dist:
                total += min_d
                count += 1
        return total / max(count, 1)


# ══════════════════════════════════════════
# OCCUPANCY GRID MAPPER
# ══════════════════════════════════════════


class OccupancyGridMapper:
    """Build occupancy grid maps from range sensor data.

    Uses log-odds representation for probabilistic updates.

    Args:
        width, height: Grid dimensions in cells
        resolution: Meters per cell
        origin: World position of grid origin
        p_hit: Probability of occupied given hit
        p_miss: Probability of occupied given miss
    """

    def __init__(
        self,
        width: int = 200,
        height: int = 200,
        resolution: float = 0.05,
        origin: Optional[List[float]] = None,
        p_hit: float = 0.7,
        p_miss: float = 0.4,
    ):
        if not HAS_NUMPY:
            raise ImportError("numpy required")

        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin or [-width * resolution / 2, -height * resolution / 2]

        self.l_hit = math.log(p_hit / (1 - p_hit))
        self.l_miss = math.log(p_miss / (1 - p_miss))
        self.l_max = 5.0
        self.l_min = -5.0

        self.grid = np.zeros((height, width), dtype=np.float32)

    def update(
        self,
        robot_pose: List[float],
        ranges: List[float],
        angle_min: float = -math.pi,
        angle_max: float = math.pi,
        range_max: float = 30.0,
    ):
        """Update map with a single range scan.

        Args:
            robot_pose: [x, y, theta]
            ranges: List of range measurements
            angle_min, angle_max: Scan angular bounds
            range_max: Maximum valid range
        """
        rx, ry, rt = robot_pose
        n_beams = len(ranges)
        if n_beams == 0:
            return

        angle_step = (angle_max - angle_min) / n_beams

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_step + rt
            if r <= 0 or r >= range_max:
                continue

            # Ray endpoint
            ex = rx + r * math.cos(angle)
            ey = ry + r * math.sin(angle)

            # Bresenham ray trace
            cells = self._ray_trace(rx, ry, ex, ey)
            for cx, cy in cells[:-1]:
                if 0 <= cx < self.width and 0 <= cy < self.height:
                    self.grid[cy, cx] = max(self.l_min, self.grid[cy, cx] + self.l_miss)

            # Endpoint = occupied
            if cells:
                cx, cy = cells[-1]
                if 0 <= cx < self.width and 0 <= cy < self.height:
                    self.grid[cy, cx] = min(self.l_max, self.grid[cy, cx] + self.l_hit)

    def _ray_trace(self, x0, y0, x1, y1) -> List[Tuple[int, int]]:
        gx0, gy0 = self._world_to_grid(x0, y0)
        gx1, gy1 = self._world_to_grid(x1, y1)

        cells = []
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        cx, cy = gx0, gy0

        while True:
            cells.append((cx, cy))
            if cx == gx1 and cy == gy1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy
            if len(cells) > 1000:
                break

        return cells

    def _world_to_grid(self, wx, wy) -> Tuple[int, int]:
        gx = int((wx - self.origin[0]) / self.resolution)
        gy = int((wy - self.origin[1]) / self.resolution)
        return gx, gy

    def get_probability_map(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.grid))

    def get_binary_map(self, threshold: float = 0.6) -> np.ndarray:
        prob = self.get_probability_map()
        return (prob > threshold).astype(np.uint8)


# ══════════════════════════════════════════
# SLAM NAVIGATOR (high-level interface)
# ══════════════════════════════════════════


class SLAMNavigator:
    """High-level SLAM interface — connects sensors, SLAM algorithm, and map.

    Usage:
        slam = SLAMNavigator(method="ekf")
        slam.initialize(robot)

        @robot.on_update
        def control(dt):
            slam.update(dt)
            pose = slam.get_pose()
            occupancy = slam.get_occupancy_grid()
    """

    METHODS = {"ekf": EKFSLAM, "particle": ParticleFilterSLAM}

    def __init__(self, method: str = "ekf", **kwargs):
        if method not in self.METHODS:
            raise ValueError(
                f"Unknown SLAM method: {method}. Use: {list(self.METHODS.keys())}"
            )

        self._slam = self.METHODS[method](**kwargs)
        self._mapper = OccupancyGridMapper()
        self._robot = None
        self._lidar = None
        self._imu = None
        self._prev_time = 0.0
        self._initialized = False

    def initialize(self, robot, pose: Optional[List[float]] = None):
        self._robot = robot
        self._slam.reset(pose)
        self._initialized = True

        for sensor in robot.sensors:
            stype = type(sensor).__name__
            if stype == "LiDAR" and self._lidar is None:
                self._lidar = sensor
            elif stype == "IMU" and self._imu is None:
                self._imu = sensor

    def update(self, dt: float):
        if not self._initialized or not self._robot:
            return

        velocities = self._robot.get_joint_velocities()
        v = velocities[0] if velocities else 0.0
        w = velocities[1] if len(velocities) > 1 else 0.0

        self._slam.predict(v, w, dt)

        if self._lidar:
            data = self._lidar.read()
            ranges = data.get("ranges", [])
            if ranges:
                observations = self._extract_landmarks(ranges)
                self._slam.update(observations)
                pose = self._slam.pose
                self._mapper.update(pose, ranges)

    def _extract_landmarks(self, ranges: List[float]) -> List[Tuple[float, float, int]]:
        observations = []
        n = len(ranges)
        if n == 0:
            return observations

        angle_step = 2 * math.pi / n
        threshold = 0.3

        i = 0
        lm_id = 0
        while i < n:
            if ranges[i] < threshold * max(ranges):
                cluster_start = i
                while i < n and ranges[i] < threshold * max(ranges):
                    i += 1
                mid = (cluster_start + i) // 2
                angle = -math.pi + mid * angle_step
                observations.append((ranges[mid], angle, -1))
                lm_id += 1
            else:
                i += 1

        return observations

    @property
    def pose(self) -> List[float]:
        return self._slam.pose

    @property
    def current_pose(self) -> List[float]:
        return self._slam.pose

    @property
    def map(self):
        return self._mapper

    def get_pose(self) -> List[float]:
        return self._slam.pose

    def get_occupancy_grid(self) -> Any:
        return self._mapper.get_binary_map()

    def get_landmarks(self) -> List[List[float]]:
        if hasattr(self._slam, "landmarks"):
            return self._slam.landmarks
        return []
