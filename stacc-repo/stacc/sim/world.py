"""Universal world/terrain system — simulation environments for any scenario.

Load pre-built worlds or create custom environments with obstacles,
terrain heightmaps, lighting, and physics properties.

Usage:
    from stacc.sim.world import World, Terrain

    world = World()
    world.add_box("table", pos=[1, 0, 0.4], size=[0.8, 0.5, 0.02], mass=0)
    world.add_sphere("ball", pos=[1, 0, 1.0], radius=0.05, mass=0.1)
    world.set_terrain(Terrain.heightmap(width=10, depth=10, resolution=0.1))

    stacc.simulate(robot, duration=10, world=world)
"""

from __future__ import annotations

import math
import random
from typing import Any, Dict, List, Optional


class WorldObject:
    """A static or dynamic object in the simulation world."""

    __slots__ = (
        "name",
        "shape",
        "pos",
        "size",
        "radius",
        "length",
        "mass",
        "color",
        "friction",
        "euler",
    )

    def __init__(
        self,
        name: str,
        shape: str,
        pos: List[float],
        size: Optional[List[float]] = None,
        radius: float = 0.1,
        length: float = 0.1,
        mass: float = 0.0,
        color: Optional[List[float]] = None,
        friction: float = 1.0,
        euler: Optional[List[float]] = None,
    ):
        self.name = name
        self.shape = shape
        self.pos = pos
        self.size = size or [0.1, 0.1, 0.1]
        self.radius = radius
        self.length = length
        self.mass = mass
        self.color = color or [0.5, 0.5, 0.5, 1.0]
        self.friction = friction
        self.euler = euler or [0, 0, 0]


class Terrain:
    """Terrain configuration for the simulation ground."""

    def __init__(self, terrain_type: str = "flat", **kwargs):
        self.terrain_type = terrain_type
        self.params = kwargs

    @classmethod
    def flat(cls, size: float = 10.0, color: Optional[List[float]] = None) -> "Terrain":
        return cls("flat", size=size, color=color or [0.9, 0.9, 0.9, 1.0])

    @classmethod
    def heightmap(
        cls,
        width: float = 10.0,
        depth: float = 10.0,
        resolution: float = 0.1,
        max_height: float = 0.5,
        seed: int = 42,
    ) -> "Terrain":
        return cls(
            "heightmap",
            width=width,
            depth=depth,
            resolution=resolution,
            max_height=max_height,
            seed=seed,
        )

    @classmethod
    def slopes(cls, angle_deg: float = 10.0, size: float = 5.0) -> "Terrain":
        return cls("slopes", angle_deg=angle_deg, size=size)

    @classmethod
    def stairs(
        cls,
        num_steps: int = 10,
        step_height: float = 0.05,
        step_depth: float = 0.3,
        width: float = 2.0,
    ) -> "Terrain":
        return cls(
            "stairs",
            num_steps=num_steps,
            step_height=step_height,
            step_depth=step_depth,
            width=width,
        )


class World:
    """Simulation world — contains terrain, objects, lighting, and physics settings.

    Usage:
        world = World()
        world.add_box("table", pos=[1, 0, 0.4], size=[0.8, 0.5, 0.02])
        world.add_sphere("ball", pos=[1, 0, 1.0], radius=0.05, mass=0.1)
        world.add_cylinder("pole", pos=[2, 0, 0.5], radius=0.05, length=1.0)
        world.set_terrain(Terrain.stairs(num_steps=5))
    """

    def __init__(self, name: str = "world"):
        self.name = name
        self.objects: List[WorldObject] = []
        self.terrain: Terrain = Terrain.flat()
        self.lights: List[Dict[str, Any]] = [
            {"pos": [0, 0, 4], "dir": [0, 0, -1], "diffuse": [1, 1, 1]}
        ]
        self.ambient_light: List[float] = [0.3, 0.3, 0.3]
        self.skybox: bool = True

    def add_box(
        self,
        name: str,
        pos: List[float],
        size: List[float],
        mass: float = 0.0,
        color: Optional[List[float]] = None,
        friction: float = 1.0,
        euler: Optional[List[float]] = None,
    ) -> "World":
        self.objects.append(
            WorldObject(
                name,
                "box",
                pos,
                size=size,
                mass=mass,
                color=color,
                friction=friction,
                euler=euler,
            )
        )
        return self

    def add_sphere(
        self,
        name: str,
        pos: List[float],
        radius: float = 0.1,
        mass: float = 0.0,
        color: Optional[List[float]] = None,
    ) -> "World":
        self.objects.append(
            WorldObject(name, "sphere", pos, radius=radius, mass=mass, color=color)
        )
        return self

    def add_cylinder(
        self,
        name: str,
        pos: List[float],
        radius: float = 0.05,
        length: float = 0.5,
        mass: float = 0.0,
        color: Optional[List[float]] = None,
    ) -> "World":
        self.objects.append(
            WorldObject(
                name,
                "cylinder",
                pos,
                radius=radius,
                length=length,
                mass=mass,
                color=color,
            )
        )
        return self

    def add_wall(
        self,
        name: str,
        start: List[float],
        end: List[float],
        height: float = 1.0,
        thickness: float = 0.05,
        color: Optional[List[float]] = None,
    ) -> "World":
        cx = (start[0] + end[0]) / 2
        cy = (start[1] + end[1]) / 2
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx * dx + dy * dy)
        angle = math.atan2(dy, dx)
        self.objects.append(
            WorldObject(
                name,
                "box",
                [cx, cy, height / 2],
                size=[length / 2, thickness / 2, height / 2],
                color=color or [0.7, 0.7, 0.7, 1.0],
                euler=[0, 0, angle],
            )
        )
        return self

    def set_terrain(self, terrain: Terrain) -> "World":
        self.terrain = terrain
        return self

    def add_light(
        self,
        pos: List[float],
        dir: Optional[List[float]] = None,
        diffuse: Optional[List[float]] = None,
    ) -> "World":
        self.lights.append(
            {
                "pos": pos,
                "dir": dir or [0, 0, -1],
                "diffuse": diffuse or [1, 1, 1],
            }
        )
        return self

    def to_mjcf_elements(self) -> List[Dict[str, Any]]:
        """Convert world objects to MJCF-compatible element descriptions.
        Used by mjcf_builder to inject into the worldbody.
        """
        elements = []
        for obj in self.objects:
            el = {
                "tag": "body",
                "name": obj.name,
                "pos": f"{obj.pos[0]} {obj.pos[1]} {obj.pos[2]}",
                "geom_type": obj.shape,
                "mass": obj.mass,
                "color": f"{obj.color[0]} {obj.color[1]} {obj.color[2]} {obj.color[3]}",
                "friction": obj.friction,
            }
            if obj.shape == "box":
                el["size"] = f"{obj.size[0]} {obj.size[1]} {obj.size[2]}"
            elif obj.shape == "sphere":
                el["size"] = str(obj.radius)
            elif obj.shape == "cylinder":
                el["size"] = f"{obj.radius} {obj.length / 2}"
            if obj.euler and any(e != 0 for e in obj.euler):
                el["euler"] = f"{obj.euler[0]} {obj.euler[1]} {obj.euler[2]}"
            elements.append(el)

        # Terrain objects (stairs, etc.)
        if self.terrain.terrain_type == "stairs":
            p = self.terrain.params
            for i in range(p.get("num_steps", 5)):
                h = (i + 1) * p.get("step_height", 0.05)
                d = i * p.get("step_depth", 0.3)
                w = p.get("width", 2.0) / 2
                elements.append(
                    {
                        "tag": "geom",
                        "name": f"stair_{i}",
                        "pos": f"{d} 0 {h / 2}",
                        "geom_type": "box",
                        "size": f"{p.get('step_depth', 0.3) / 2} {w} {h / 2}",
                        "color": "0.6 0.6 0.65 1",
                        "mass": 0,
                    }
                )

        return elements


# ── Pre-built worlds ──


def warehouse_world() -> World:
    w = World("warehouse")
    w.add_box("floor_mark_1", [2, 0, 0.001], [2, 0.05, 0.001], color=[1, 0.8, 0, 1])
    w.add_box("shelf_1", [3, -2, 0.75], [0.4, 1.0, 0.75], color=[0.4, 0.3, 0.2, 1])
    w.add_box("shelf_2", [3, 2, 0.75], [0.4, 1.0, 0.75], color=[0.4, 0.3, 0.2, 1])
    w.add_box("pallet", [5, 0, 0.05], [0.6, 0.6, 0.05], color=[0.6, 0.5, 0.3, 1])
    w.add_box(
        "crate_1", [5, 0, 0.2], [0.3, 0.3, 0.15], mass=2.0, color=[0.3, 0.5, 0.3, 1]
    )
    w.add_wall("wall_left", [-1, -4], [8, -4], height=2.5)
    w.add_wall("wall_right", [-1, 4], [8, 4], height=2.5)
    w.add_wall("wall_back", [8, -4], [8, 4], height=2.5)
    return w


def obstacle_course() -> World:
    w = World("obstacle_course")
    w.add_box("block_1", [1, 0.5, 0.15], [0.15, 0.15, 0.15], color=[0.8, 0.2, 0.2, 1])
    w.add_box("block_2", [2, -0.3, 0.15], [0.15, 0.15, 0.15], color=[0.2, 0.8, 0.2, 1])
    w.add_cylinder(
        "pole_1", [1.5, 0, 0.4], radius=0.03, length=0.8, color=[0.2, 0.2, 0.8, 1]
    )
    w.add_cylinder(
        "pole_2", [3, 0.5, 0.4], radius=0.03, length=0.8, color=[0.2, 0.2, 0.8, 1]
    )
    w.add_sphere("ball", [2.5, 0, 0.5], radius=0.08, mass=0.2, color=[0.9, 0.6, 0.1, 1])
    w.add_box(
        "ramp",
        [4, 0, 0.1],
        [0.5, 0.5, 0.1],
        euler=[0, 0.2, 0],
        color=[0.5, 0.5, 0.5, 1],
    )
    return w


def tabletop_world() -> World:
    w = World("tabletop")
    w.add_box("table", [0.5, 0, 0.35], [0.4, 0.3, 0.02], color=[0.6, 0.4, 0.2, 1])
    w.add_box(
        "leg_1", [0.15, -0.25, 0.17], [0.02, 0.02, 0.17], color=[0.5, 0.3, 0.15, 1]
    )
    w.add_box(
        "leg_2", [0.85, -0.25, 0.17], [0.02, 0.02, 0.17], color=[0.5, 0.3, 0.15, 1]
    )
    w.add_box(
        "leg_3", [0.15, 0.25, 0.17], [0.02, 0.02, 0.17], color=[0.5, 0.3, 0.15, 1]
    )
    w.add_box(
        "leg_4", [0.85, 0.25, 0.17], [0.02, 0.02, 0.17], color=[0.5, 0.3, 0.15, 1]
    )
    w.add_box(
        "cube_red",
        [0.4, 0.1, 0.39],
        [0.02, 0.02, 0.02],
        mass=0.05,
        color=[0.9, 0.2, 0.2, 1],
    )
    w.add_box(
        "cube_blue",
        [0.6, -0.05, 0.39],
        [0.02, 0.02, 0.02],
        mass=0.05,
        color=[0.2, 0.2, 0.9, 1],
    )
    w.add_cylinder(
        "cup",
        [0.5, 0.15, 0.4],
        radius=0.025,
        length=0.06,
        mass=0.03,
        color=[0.8, 0.8, 0.3, 1],
    )
    return w


def mars_world() -> World:
    w = World("mars")
    w.set_terrain(Terrain.flat(size=20.0, color=[0.7, 0.4, 0.2, 1.0]))
    w.add_sphere("rock_1", [3, 1, 0.15], radius=0.15, color=[0.5, 0.3, 0.2, 1])
    w.add_sphere("rock_2", [5, -2, 0.1], radius=0.1, color=[0.6, 0.35, 0.2, 1])
    w.add_sphere("rock_3", [7, 0.5, 0.2], radius=0.2, color=[0.55, 0.3, 0.15, 1])
    w.add_box("boulder", [4, -1, 0.25], [0.3, 0.25, 0.25], color=[0.45, 0.25, 0.15, 1])
    w.add_cylinder(
        "sample_tube",
        [6, 0, 0.02],
        radius=0.01,
        length=0.08,
        mass=0.01,
        color=[0.9, 0.9, 0.9, 1],
    )
    return w


def empty_world() -> World:
    return World("empty")
