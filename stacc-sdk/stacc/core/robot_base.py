"""Base Robot class — universal robot definition for any topology.

Supports:
- Serial chains (arms, snakes)
- Branching trees (humanoids, multi-arm)
- Mobile bases (wheeled, legged, aerial)
- Any number of joints, links, sensors, end-effectors

Users define robots by adding links and connecting them with joints.
The MJCF builder reads this structure and generates simulation XML.

Usage:
    robot = Robot("humanoid")

    # Add links (rigid bodies)
    robot.add_link(Link("pelvis", mass=8.0, ...))
    robot.add_link(Link("torso", mass=5.0, ...))
    robot.add_link(Link("head", mass=2.0, ...))
    robot.add_link(Link("l_upper_arm", mass=1.5, ...))

    # Connect them with joints (specify parent → child)
    robot.connect("pelvis", "torso", Joint("waist", ...))
    robot.connect("torso", "head", Joint("neck", ...))
    robot.connect("torso", "l_upper_arm", Joint("l_shoulder", ...))

    # Attach sensors
    robot.attach_sensor(IMU(), position="pelvis")

    # Define control
    @robot.on_update
    def control(dt):
        ...

    stacc.simulate(robot, duration=30)
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from stacc.core.joint import Joint
from stacc.core.link import Link


class Connection:
    """A connection between two links via a joint."""

    __slots__ = ("parent_name", "child_name", "joint", "offset")

    def __init__(
        self,
        parent_name: str,
        child_name: str,
        joint: Joint,
        offset: Optional[List[float]] = None,
    ):
        self.parent_name = parent_name
        self.child_name = child_name
        self.joint = joint
        self.offset = offset  # [x, y, z] manual offset, or None for auto


class Robot:
    """Universal robot class — supports any kinematic topology.

    Two ways to build a robot:

    Method 1 — Simple serial chain (quick, for arms/snakes):
        robot.add_link(Link("base", ...))
        robot.add_joint(Joint("j1", ...))     # connects to previous link
        robot.add_link(Link("link1", ...))
        robot.add_joint(Joint("j2", ...))
        robot.add_link(Link("link2", ...))

    Method 2 — Explicit tree topology (for humanoids, multi-arm):
        robot.add_link(Link("pelvis", ...))
        robot.add_link(Link("torso", ...))
        robot.add_link(Link("l_arm", ...))
        robot.add_link(Link("r_arm", ...))
        robot.connect("pelvis", "torso", Joint("waist", ...))
        robot.connect("torso", "l_arm", Joint("l_shoulder", ...))
        robot.connect("torso", "r_arm", Joint("r_shoulder", ...))

    Both methods can be mixed. connect() overrides serial chain order.
    """

    def __init__(self, name: str = "robot", **kwargs):
        self.name = name
        self.fixed_base: bool = True  # False for mobile robots/drones
        self.links: List[Link] = []
        self.joints: List[Joint] = []
        self.sensors: List[Any] = []
        self.end_effectors: List[Any] = []
        self.actuators: List[Any] = []
        self.time: float = 0.0
        self.dt: float = 0.001

        # ── Tree topology ──
        self._links_by_name: Dict[str, Link] = {}
        self._connections: List[Connection] = []  # all parent→child connections
        self._children_of: Dict[str, List[Connection]] = {}  # parent → [connections]
        self._parent_of: Dict[str, str] = {}  # child → parent
        self._serial_chain: List[str] = []  # link names in serial add order

        # ── Callbacks ──
        self._update_callbacks: List[Callable] = []
        self._start_callbacks: List[Callable] = []
        self._stop_callbacks: List[Callable] = []

        # ── Runtime state ──
        self._state: Dict[str, Any] = {
            "joint_positions": [],
            "joint_velocities": [],
            "joint_torques": [],
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }

        # ── Joint ordering (maps joint name → index in state arrays) ──
        self._joint_index: Dict[str, int] = {}

        for key, value in kwargs.items():
            setattr(self, key, value)

    # ══════════════════════════════════════════
    # BUILDING THE ROBOT
    # ══════════════════════════════════════════

    def add_link(self, link: Link) -> "Robot":
        """Add a rigid body link. For serial chains, order matters."""
        self.links.append(link)
        self._links_by_name[link.name] = link
        self._serial_chain.append(link.name)
        return self

    def add_joint(self, joint: Joint) -> "Robot":
        """Add a joint to the serial chain (connects previous link to next).

        In serial chain mode, joints are placed between consecutive links.
        Joint 0 goes between link 0 and link 1, etc.
        """
        self.joints.append(joint)
        idx = len(self._state["joint_positions"])
        self._state["joint_positions"].append(0.0)
        self._state["joint_velocities"].append(0.0)
        self._state["joint_torques"].append(0.0)
        self._joint_index[joint.name] = idx
        return self

    def connect(
        self,
        parent_link_name: str,
        child_link_name: str,
        joint: Joint,
        offset: Optional[List[float]] = None,
    ) -> "Robot":
        """Connect two links with a joint — for any topology.

        Args:
            parent_link_name: Name of the parent link
            child_link_name: Name of the child link
            joint: Joint connecting them
            offset: Optional [x, y, z] position of child relative to parent.
                    If None, auto-computed from geometry.

        Usage:
            robot.connect("torso", "l_upper_arm", Joint("l_shoulder", ...))
            robot.connect("torso", "r_upper_arm", Joint("r_shoulder", ...))
            robot.connect("pelvis", "l_thigh", Joint("l_hip", ...), offset=[0.1, 0, -0.06])
        """
        conn = Connection(parent_link_name, child_link_name, joint, offset)
        self._connections.append(conn)

        # Track parent-child relationships
        if parent_link_name not in self._children_of:
            self._children_of[parent_link_name] = []
        self._children_of[parent_link_name].append(conn)
        self._parent_of[child_link_name] = parent_link_name

        # Add joint to state arrays if not already there
        if joint.name not in self._joint_index:
            if joint not in self.joints:
                self.joints.append(joint)
            idx = len(self._state["joint_positions"])
            self._state["joint_positions"].append(0.0)
            self._state["joint_velocities"].append(0.0)
            self._state["joint_torques"].append(0.0)
            self._joint_index[joint.name] = idx

        return self

    def attach(
        self,
        parent_link_name: str,
        joint: Joint,
        child_link: Link,
        offset: Optional[List[float]] = None,
    ) -> "Robot":
        """Shorthand: add a link and connect it in one call.

        Args:
            parent_link_name: Name of the parent link
            joint: Joint connecting them
            child_link: The new child link to add
            offset: Optional [x, y, z] offset

        Usage:
            robot.attach("torso",
                Joint("l_shoulder", type="revolute", axis=[1,0,0]),
                Link("l_upper_arm", mass=1.5, ...),
                offset=[-0.18, 0, 0.08])
        """
        self.add_link(child_link)
        self.connect(parent_link_name, child_link.name, joint, offset)
        return self

    # ══════════════════════════════════════════
    # TOPOLOGY QUERIES (used by mjcf_builder)
    # ══════════════════════════════════════════

    def get_root_link(self) -> Optional[Link]:
        """Get the root link (first link, or the one with no parent)."""
        if not self.links:
            return None

        # Find links that are never children
        child_names = set(self._parent_of.keys())
        for link in self.links:
            if link.name not in child_names:
                return link
        return self.links[0]

    def get_children(self, link_name: str) -> List[Tuple[Joint, Link]]:
        """Get all children of a link as (joint, child_link) pairs."""
        result = []

        # From explicit connections
        if link_name in self._children_of:
            for conn in self._children_of[link_name]:
                child_link = self._links_by_name.get(conn.child_name)
                if child_link:
                    result.append((conn.joint, child_link, conn.offset))

        return result

    def get_serial_children(self, link_name: str) -> List[Tuple[Joint, Link]]:
        """Get serial chain children (from add_link/add_joint pattern)."""
        result = []
        chain = self._serial_chain
        try:
            idx = chain.index(link_name)
        except ValueError:
            return result

        # Next link in serial chain
        next_idx = idx + 1
        if next_idx < len(chain):
            next_name = chain[next_idx]
            # Only if it's not already connected via explicit connect()
            if next_name not in self._parent_of:
                next_link = self._links_by_name.get(next_name)
                # Joint index is idx (joint 0 between link 0 and link 1)
                serial_joints = [
                    j
                    for j in self.joints
                    if j.name not in {c.joint.name for c in self._connections}
                ]
                joint_idx = idx
                if next_link and joint_idx < len(serial_joints):
                    result.append((serial_joints[joint_idx], next_link, None))

        return result

    def get_all_children(self, link_name: str) -> List[Tuple[Joint, Link, Any]]:
        """Get ALL children — both serial and explicit connections."""
        serial = self.get_serial_children(link_name)
        explicit = self.get_children(link_name)

        # Serial children come as (joint, link) — add None offset
        all_children = [(j, l, o) for j, l, o in explicit]
        all_children += [(j, l, o) for j, l, o in serial]

        return all_children

    def get_connection_offset(
        self, parent_name: str, child_name: str
    ) -> Optional[List[float]]:
        """Get explicit offset for a connection, if set."""
        if parent_name in self._children_of:
            for conn in self._children_of[parent_name]:
                if conn.child_name == child_name:
                    return conn.offset
        return None

    def get_joint_index(self, joint_name: str) -> int:
        """Get the state array index for a joint."""
        return self._joint_index.get(joint_name, -1)

    # ══════════════════════════════════════════
    # SENSORS & END EFFECTORS
    # ══════════════════════════════════════════

    def attach_sensor(self, sensor, position: str = "body", **kwargs) -> "Robot":
        """Attach a sensor to the robot."""
        sensor._mount_position = position
        sensor._mount_kwargs = kwargs
        self.sensors.append(sensor)
        return self

    def attach_end_effector(self, effector) -> "Robot":
        """Attach an end effector (gripper, tool, etc.)."""
        self.end_effectors.append(effector)
        return self

    # ══════════════════════════════════════════
    # STATE ACCESS
    # ══════════════════════════════════════════

    @property
    def num_joints(self) -> int:
        return len(self.joints)

    @property
    def num_links(self) -> int:
        return len(self.links)

    @property
    def dof(self) -> int:
        return sum(1 for j in self.joints if j.type != "fixed")

    def get_joint_positions(self) -> List[float]:
        return list(self._state["joint_positions"])

    def get_joint_velocities(self) -> List[float]:
        return list(self._state["joint_velocities"])

    def get_joint_position(self, joint_name: str) -> float:
        """Get position of a specific joint by name."""
        idx = self._joint_index.get(joint_name, -1)
        if idx >= 0 and idx < len(self._state["joint_positions"]):
            return self._state["joint_positions"][idx]
        return 0.0

    def set_joint_positions(self, positions: List[float]):
        for i, pos in enumerate(positions):
            if i < len(self._state["joint_positions"]):
                if i < len(self.joints):
                    self._state["joint_positions"][i] = self.joints[i].clamp(pos)
                else:
                    self._state["joint_positions"][i] = pos

    def set_joint_torques(self, torques: List[float]):
        for i, torque in enumerate(torques):
            if i < len(self._state["joint_torques"]):
                self._state["joint_torques"][i] = torque

    def set_joint_torque(self, joint_name: str, torque: float):
        """Set torque on a specific joint by name."""
        idx = self._joint_index.get(joint_name, -1)
        if idx >= 0 and idx < len(self._state["joint_torques"]):
            self._state["joint_torques"][idx] = torque

    @property
    def position(self) -> List[float]:
        return self._state["pose"][:3]

    @position.setter
    def position(self, pos: List[float]):
        self._state["pose"][:3] = pos[:3]

    @property
    def orientation(self) -> List[float]:
        return self._state["pose"][3:]

    # ══════════════════════════════════════════
    # EVENT CALLBACKS
    # ══════════════════════════════════════════

    def on_update(self, func: Callable) -> Callable:
        """Decorator: called every simulation step with dt."""
        self._update_callbacks.append(func)
        return func

    def on_start(self, func: Callable) -> Callable:
        """Decorator: called once before simulation starts."""
        self._start_callbacks.append(func)
        return func

    def on_stop(self, func: Callable) -> Callable:
        """Decorator: called once after simulation ends."""
        self._stop_callbacks.append(func)
        return func

    def _tick(self, dt: float):
        self.time += dt
        self.dt = dt
        for cb in self._update_callbacks:
            cb(dt)

    def _startup(self):
        for cb in self._start_callbacks:
            cb()

    def _shutdown(self):
        for cb in self._stop_callbacks:
            cb()

    # ══════════════════════════════════════════
    # SERIALIZATION
    # ══════════════════════════════════════════

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "fixed_base": self.fixed_base,
            "num_links": self.num_links,
            "num_joints": self.num_joints,
            "dof": self.dof,
            "links": [l.name for l in self.links],
            "joints": [j.name for j in self.joints],
            "connections": [
                (c.parent_name, c.child_name, c.joint.name) for c in self._connections
            ],
            "sensors": [s.__class__.__name__ for s in self.sensors],
        }

    def __repr__(self) -> str:
        return (
            f"Robot('{self.name}', links={self.num_links}, "
            f"joints={self.num_joints}, dof={self.dof})"
        )
