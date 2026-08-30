"""Rigid body link definition.

A link is a rigid body in the robot's kinematic chain.
Each link has mass, inertia, and optional visual/collision geometry.
"""
from __future__ import annotations
from typing import Optional, List, Dict, Any
import math


class Inertia:
    """3x3 inertia tensor (symmetric, stored as 6 unique values).

    The inertia tensor is:
        [ixx, ixy, ixz]
        [ixy, iyy, iyz]
        [ixz, iyz, izz]
    """

    def __init__(
        self,
        ixx: float = 0.001,
        iyy: float = 0.001,
        izz: float = 0.001,
        ixy: float = 0.0,
        ixz: float = 0.0,
        iyz: float = 0.0,
    ):
        self.ixx = ixx
        self.iyy = iyy
        self.izz = izz
        self.ixy = ixy
        self.ixz = ixz
        self.iyz = iyz

    @classmethod
    def from_box(cls, mass: float, x: float, y: float, z: float) -> "Inertia":
        """Compute inertia for a uniform box."""
        return cls(
            ixx=mass * (y**2 + z**2) / 12,
            iyy=mass * (x**2 + z**2) / 12,
            izz=mass * (x**2 + y**2) / 12,
        )

    @classmethod
    def from_cylinder(cls, mass: float, radius: float, length: float) -> "Inertia":
        """Compute inertia for a uniform cylinder (along Z axis)."""
        return cls(
            ixx=mass * (3 * radius**2 + length**2) / 12,
            iyy=mass * (3 * radius**2 + length**2) / 12,
            izz=mass * radius**2 / 2,
        )

    @classmethod
    def from_sphere(cls, mass: float, radius: float) -> "Inertia":
        """Compute inertia for a uniform sphere."""
        i = 2 * mass * radius**2 / 5
        return cls(ixx=i, iyy=i, izz=i)

    def as_matrix(self) -> List[List[float]]:
        return [
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz],
        ]

    def to_dict(self) -> Dict[str, float]:
        return {
            "ixx": self.ixx, "iyy": self.iyy, "izz": self.izz,
            "ixy": self.ixy, "ixz": self.ixz, "iyz": self.iyz,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, float]) -> "Inertia":
        return cls(
            ixx=d.get("ixx", 0.001), iyy=d.get("iyy", 0.001), izz=d.get("izz", 0.001),
            ixy=d.get("ixy", 0.0), ixz=d.get("ixz", 0.0), iyz=d.get("iyz", 0.0),
        )


class Geometry:
    """Visual or collision geometry for a link.

    Supports: box, cylinder, sphere, and mesh.
    """

    def __init__(
        self,
        shape: str = "box",
        size: Optional[List[float]] = None,
        radius: Optional[float] = None,
        length: Optional[float] = None,
        mesh_path: Optional[str] = None,
        mesh_scale: Optional[List[float]] = None,
        origin: Optional[List[float]] = None,
        color: Optional[List[float]] = None,
        contype: Optional[int] = None,
        conaffinity: Optional[int] = None,
    ):
        self.shape = shape  # "box", "cylinder", "sphere", "mesh"
        self.size = size or [0.1, 0.1, 0.1]
        self.radius = radius or 0.05
        self.length = length or 0.1
        self.mesh_path = mesh_path
        self.mesh_scale = mesh_scale or [1.0, 1.0, 1.0]
        self.origin = origin or [0, 0, 0, 0, 0, 0]
        self.color = color or [0.7, 0.7, 0.7, 1.0]  # RGBA
        # MuJoCo collision filtering (None = default, contacts enabled).
        self.contype = contype
        self.conaffinity = conaffinity

    def to_dict(self) -> Dict[str, Any]:
        d = {"shape": self.shape, "origin": self.origin, "color": self.color}
        if self.shape == "box":
            d["size"] = self.size
        elif self.shape == "cylinder":
            d["radius"] = self.radius
            d["length"] = self.length
        elif self.shape == "sphere":
            d["radius"] = self.radius
        elif self.shape == "mesh":
            d["mesh_path"] = self.mesh_path
            d["mesh_scale"] = self.mesh_scale
        if self.contype is not None:
            d["contype"] = self.contype
        if self.conaffinity is not None:
            d["conaffinity"] = self.conaffinity
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Geometry":
        return cls(
            shape=d.get("shape", "box"),
            size=d.get("size"),
            radius=d.get("radius"),
            length=d.get("length"),
            mesh_path=d.get("mesh_path"),
            mesh_scale=d.get("mesh_scale"),
            origin=d.get("origin"),
            color=d.get("color"),
            contype=d.get("contype"),
            conaffinity=d.get("conaffinity"),
        )


class Link:
    """A rigid body in the robot's kinematic chain.

    Args:
        name: Unique link identifier
        mass: Mass in kg
        inertia: Inertia tensor (auto-computed if not provided)
        visual: Visual geometry for rendering
        collision: Collision geometry for physics (defaults to visual if not set)
        origin: Link frame origin relative to parent [x,y,z,r,p,y]
    """

    def __init__(
        self,
        name: str,
        mass: float = 1.0,
        inertia: Optional[Inertia] = None,
        visual: Optional[Geometry] = None,
        collision: Optional[Geometry] = None,
        origin: Optional[List[float]] = None,
    ):
        self.name = name
        self.mass = mass
        self.inertia = inertia or Inertia.from_box(mass, 0.1, 0.1, 0.1)
        self.visual = visual
        self.collision = collision or visual
        self.origin = origin or [0, 0, 0, 0, 0, 0]

    @property
    def center_of_mass(self) -> List[float]:
        """Center of mass in link frame (defaults to origin)."""
        return self.origin[:3]

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "name": self.name,
            "mass": self.mass,
            "inertia": self.inertia.to_dict(),
            "origin": self.origin,
        }
        if self.visual:
            d["visual"] = self.visual.to_dict()
        if self.collision:
            d["collision"] = self.collision.to_dict()
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Link":
        visual = Geometry.from_dict(d["visual"]) if d.get("visual") else None
        collision = Geometry.from_dict(d["collision"]) if d.get("collision") else None
        return cls(
            name=d["name"],
            mass=d.get("mass", 1.0),
            inertia=Inertia.from_dict(d["inertia"]) if d.get("inertia") else None,
            visual=visual,
            collision=collision,
            origin=d.get("origin"),
        )

    def __repr__(self) -> str:
        return f"Link('{self.name}', mass={self.mass}kg)"
