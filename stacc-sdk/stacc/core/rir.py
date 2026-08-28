"""Robotics Intermediate Representation (RIR).

RIR is the portable, engine-independent serialization of a stacc `Robot`
graph — the same Link/Joint/Connection structure regardless of whether the
robot originated from URDF, SDF, or hand-written code, and regardless of
which simulator or hardware target consumes it downstream.

    URDF ─┐
    SDF  ─┼─► RIR ─┬─► MuJoCo / Bullet (simulation)
    code ─┘        └─► HAL / deployment (real hardware)

RIR captures the graph (links, joints, connections), attached sensors, and
attached actuators. It does not capture Python callbacks (on_update/
on_start/on_stop) or live runtime state (joint positions/velocities at a
given instant) — those belong to a running Robot instance, not to its
portable description.

This is the seam everything else (compiler, runtime, multi-backend sim)
is meant to build on: one shared representation instead of each backend
speaking directly to robot-specific code.
"""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

from stacc.core.joint import Joint
from stacc.core.link import Link
from stacc.core.robot_base import Robot

RIR_VERSION = "0.1"


class Actuator:
    """The physical device driving one joint.

    Joint carries kinematic/dynamic limits (range, max velocity/effort).
    Actuator carries the deployment-relevant question Joint doesn't answer:
    what hardware drives this joint and how is it addressed. The HAL and
    compiler layers need this; the simulator and kinematics code don't.
    """

    VALID_TYPES = ("servo", "dc_motor", "stepper", "bldc", "linear")
    VALID_INTERFACES = ("pwm", "can", "i2c", "serial", "spi")

    def __init__(
        self,
        joint_name: str,
        type: str = "servo",
        interface: str = "pwm",
        gear_ratio: float = 1.0,
        current_limit: Optional[float] = None,
        channel: Optional[str] = None,
    ):
        if type not in self.VALID_TYPES:
            raise ValueError(f"Invalid actuator type '{type}'. Must be one of {self.VALID_TYPES}")
        if interface not in self.VALID_INTERFACES:
            raise ValueError(f"Invalid interface '{interface}'. Must be one of {self.VALID_INTERFACES}")

        self.joint_name = joint_name
        self.type = type
        self.interface = interface
        self.gear_ratio = gear_ratio
        self.current_limit = current_limit
        self.channel = channel

    def to_dict(self) -> Dict[str, Any]:
        return {
            "joint_name": self.joint_name,
            "type": self.type,
            "interface": self.interface,
            "gear_ratio": self.gear_ratio,
            "current_limit": self.current_limit,
            "channel": self.channel,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Actuator":
        return cls(
            joint_name=d["joint_name"],
            type=d.get("type", "servo"),
            interface=d.get("interface", "pwm"),
            gear_ratio=d.get("gear_ratio", 1.0),
            current_limit=d.get("current_limit"),
            channel=d.get("channel"),
        )

    def __repr__(self) -> str:
        return f"Actuator(joint='{self.joint_name}', type='{self.type}', interface='{self.interface}')"


def robot_to_rir(robot: Robot, actuators: Optional[List[Actuator]] = None) -> Dict[str, Any]:
    """Serialize a Robot's graph into RIR.

    Walks the tree from the root using `get_all_children`, which already
    merges both ways a Robot can be built (serial add_link/add_joint chains,
    and explicit connect() topology) into one traversal — so RIR always
    ends up with a single canonical explicit graph regardless of how the
    source Robot was constructed.
    """
    links: Dict[str, Any] = {}
    connections: List[Dict[str, Any]] = []
    visited = set()

    def walk(link_name: str) -> None:
        if link_name in visited:
            return
        visited.add(link_name)
        link = robot._links_by_name.get(link_name)
        if link is not None:
            links[link_name] = link.to_dict()
        for joint, child_link, offset in robot.get_all_children(link_name):
            connections.append(
                {
                    "parent": link_name,
                    "child": child_link.name,
                    "joint": joint.to_dict(),
                    "offset": offset,
                }
            )
            walk(child_link.name)

    root = robot.get_root_link()
    if root is not None:
        walk(root.name)

    # Links unreachable from the detected root (shouldn't happen for a
    # well-formed robot, but don't silently drop data if it does).
    for link in robot.links:
        if link.name not in links:
            links[link.name] = link.to_dict()

    return {
        "rir_version": RIR_VERSION,
        "name": robot.name,
        "fixed_base": robot.fixed_base,
        "links": links,
        "connections": connections,
        "sensors": [s.to_dict() for s in robot.sensors],
        "actuators": [a.to_dict() for a in (actuators or [])],
    }


def rir_from_dict(data: Dict[str, Any]) -> Robot:
    """Reconstruct a Robot from RIR.

    Sensors are restored as metadata only (name/type/rate/mount) — full
    reconstruction of a live sensor object needs a type registry mapping
    each sensor class name back to its constructor kwargs, which doesn't
    exist yet. That's the next gap to close, not solved here.

    Actuators round-trip fully and are attached as `robot.actuators`
    (Robot itself has no built-in concept of actuators today).
    """
    robot = Robot(name=data.get("name", "robot"))
    robot.fixed_base = data.get("fixed_base", True)

    for link_data in data.get("links", {}).values():
        robot.add_link(Link.from_dict(link_data))

    for conn in data.get("connections", []):
        joint = Joint.from_dict(conn["joint"])
        robot.connect(conn["parent"], conn["child"], joint, offset=conn.get("offset"))

    robot.actuators = [Actuator.from_dict(a) for a in data.get("actuators", [])]
    robot._rir_sensor_metadata = list(data.get("sensors", []))

    return robot


def save_rir(robot: Robot, filepath: str, actuators: Optional[List[Actuator]] = None) -> None:
    with open(filepath, "w") as f:
        json.dump(robot_to_rir(robot, actuators=actuators), f, indent=2)


def load_rir(filepath: str) -> Robot:
    with open(filepath) as f:
        data = json.load(f)
    return rir_from_dict(data)
