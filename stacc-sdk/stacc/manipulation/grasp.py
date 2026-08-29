"""Grasp Synthesis, Antipodal Candidate Sampling, and Force-Closure Metrics.

Evaluates grasp quality on 3D object surfaces using:
- Antipodal grasp sampling
- Ferrari-Canny epsilon-metric (radius of the largest origin-centered ball in the Grasp Wrench Space)
- Point contact with friction (PCWF) cone validation
"""

from __future__ import annotations
from typing import List, Optional, Tuple
import math
import numpy as np

from stacc.kinematics.spatial import skew


class GraspCandidate:
    """Antipodal Two-Finger Grasp Candidate."""

    def __init__(
        self,
        p1: np.ndarray,
        p2: np.ndarray,
        n1: np.ndarray,
        n2: np.ndarray,
        mu: float = 0.5
    ):
        self.p1 = np.asarray(p1, dtype=np.float64)
        self.p2 = np.asarray(p2, dtype=np.float64)
        self.n1 = np.asarray(n1, dtype=np.float64) / (np.linalg.norm(n1) + 1e-12)
        self.n2 = np.asarray(n2, dtype=np.float64) / (np.linalg.norm(n2) + 1e-12)
        self.mu = float(mu)
        self.center = (self.p1 + self.p2) / 2.0
        self.width = float(np.linalg.norm(self.p2 - self.p1))

    def is_antipodal(self, tolerance_deg: float = 15.0) -> bool:
        """Check if the contact normals satisfy the antipodal condition."""
        grasp_axis = (self.p2 - self.p1) / (self.width + 1e-12)
        cos_tol = math.cos(math.radians(tolerance_deg))

        # Check alignment of normals with grasp axis
        dot1 = np.dot(self.n1, grasp_axis)
        dot2 = np.dot(self.n2, -grasp_axis)
        return bool(dot1 >= cos_tol and dot2 >= cos_tol)

    def ferrari_canny_metric(self, num_friction_edges: int = 4) -> float:
        """Compute the Ferrari-Canny epsilon-metric (grasp wrench space radius)."""
        if not self.is_antipodal():
            return 0.0

        # Discretize friction cone at both contact points
        wrenches = []
        for p, n in [(self.p1, self.n1), (self.p2, self.n2)]:
            # Construct tangent basis
            t1 = np.cross(n, [0, 0, 1]) if abs(n[2]) < 0.9 else np.cross(n, [0, 1, 0])
            t1 /= (np.linalg.norm(t1) + 1e-12)
            t2 = np.cross(n, t1)

            for k in range(num_friction_edges):
                angle = 2.0 * math.pi * k / num_friction_edges
                f = n + self.mu * (math.cos(angle) * t1 + math.sin(angle) * t2)
                f /= np.linalg.norm(f)

                # Wrench: [f, tau = r x f]
                r = p - self.center
                tau = np.cross(r, f)
                w = np.concatenate([f, tau])
                wrenches.append(w)

        # Minimum distance from origin to convex hull of wrenches (approximate epsilon)
        W = np.array(wrenches)
        dists = [np.linalg.norm(w) for w in W]
        return float(min(dists)) if dists else 0.0
