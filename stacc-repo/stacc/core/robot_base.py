"""Base Robot class — all robots in stacc inherit from this.

Provides the core interface for defining, simulating, and controlling robots.
Every robot has joints, links, sensors, and a control loop.
"""
from __future__ import annotations
import time
from typing import Callable, Optional, List, Dict, Any
from stacc.core.joint import Joint
from stacc.core.link import Link


class Robot:
    """Base class for all stacc robots.

    Usage:
        robot = Robot(name="my_robot")
        robot.add_link(Link("base", mass=1.0))
        robot.add_joint(Joint("joint1", type="revolute"))

        @robot.on_update
        def control(dt):
            pass

        stacc.simulate(robot, duration=30)
    """

    def __init__(self, name: str = "robot", **kwargs):
        self.name = name
        self.links: List[Link] = []
        self.joints: List[Joint] = []
        self.sensors: List[Any] = []
        self.end_effectors: List[Any] = []
        self.time: float = 0.0
        self.dt: float = 0.001  # default 1kHz

        self._update_callbacks: List[Callable] = []
        self._start_callbacks: List[Callable] = []
        self._stop_callbacks: List[Callable] = []

        self._state: Dict[str, Any] = {
            "joint_positions": [],
            "joint_velocities": [],
            "joint_torques": [],
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # x,y,z,roll,pitch,yaw
        }

        # Apply any extra kwargs as attributes
        for key, value in kwargs.items():
            setattr(self, key, value)

    # ── Link & joint management ──

    def add_link(self, link: Link) -> "Robot":
        """Add a rigid body link to the robot."""
        self.links.append(link)
        return self

    def add_joint(self, joint: Joint) -> "Robot":
        """Add a joint connecting two links."""
        self.joints.append(joint)
        self._state["joint_positions"].append(0.0)
        self._state["joint_velocities"].append(0.0)
        self._state["joint_torques"].append(0.0)
        return self

    @property
    def num_joints(self) -> int:
        return len(self.joints)

    @property
    def num_links(self) -> int:
        return len(self.links)

    @property
    def dof(self) -> int:
        """Degrees of freedom (number of actuated joints)."""
        return sum(1 for j in self.joints if j.type != "fixed")

    # ── Sensor management ──

    def attach_sensor(self, sensor, position: str = "body", **kwargs) -> "Robot":
        """Attach a sensor to the robot.

        Args:
            sensor: Sensor instance (IMU, LiDAR, Camera, etc.)
            position: Mount position string (e.g., "mast_top", "front_bottom")
        """
        sensor._mount_position = position
        sensor._mount_kwargs = kwargs
        self.sensors.append(sensor)
        return self

    def attach_end_effector(self, effector) -> "Robot":
        """Attach an end effector (gripper, tool, etc.)."""
        self.end_effectors.append(effector)
        return self

    # ── State access ──

    def get_joint_positions(self) -> List[float]:
        """Get current joint positions (radians for revolute, meters for prismatic)."""
        return list(self._state["joint_positions"])

    def get_joint_velocities(self) -> List[float]:
        return list(self._state["joint_velocities"])

    def set_joint_positions(self, positions: List[float]):
        """Command joint positions directly."""
        for i, pos in enumerate(positions):
            if i < len(self._state["joint_positions"]):
                joint = self.joints[i]
                self._state["joint_positions"][i] = joint.clamp(pos)

    def set_joint_torques(self, torques: List[float]):
        """Command joint torques."""
        for i, torque in enumerate(torques):
            if i < len(self._state["joint_torques"]):
                self._state["joint_torques"][i] = torque

    @property
    def position(self) -> List[float]:
        """Robot base position [x, y, z]."""
        return self._state["pose"][:3]

    @position.setter
    def position(self, pos: List[float]):
        self._state["pose"][:3] = pos[:3]

    @property
    def orientation(self) -> List[float]:
        """Robot base orientation [roll, pitch, yaw]."""
        return self._state["pose"][3:]

    # ── Event callbacks ──

    def on_update(self, func: Callable) -> Callable:
        """Decorator to register a control loop callback.

        Called every simulation step with dt as argument.

        Usage:
            @robot.on_update
            def control_loop(dt):
                error = sensor.read()
                robot.set_joint_torques([...])
        """
        self._update_callbacks.append(func)
        return func

    def on_start(self, func: Callable) -> Callable:
        """Decorator for one-time startup logic (runs before simulation loop)."""
        self._start_callbacks.append(func)
        return func

    def on_stop(self, func: Callable) -> Callable:
        """Decorator for cleanup logic (runs after simulation ends)."""
        self._stop_callbacks.append(func)
        return func

    def _tick(self, dt: float):
        """Internal: advance one simulation step."""
        self.time += dt
        self.dt = dt
        for cb in self._update_callbacks:
            cb(dt)

    def _startup(self):
        """Internal: run start callbacks."""
        for cb in self._start_callbacks:
            cb()

    def _shutdown(self):
        """Internal: run stop callbacks."""
        for cb in self._stop_callbacks:
            cb()

    # ── Serialization ──

    def to_dict(self) -> Dict[str, Any]:
        """Serialize robot configuration to dict (for stacc.json)."""
        return {
            "name": self.name,
            "num_links": self.num_links,
            "num_joints": self.num_joints,
            "dof": self.dof,
            "sensors": [s.__class__.__name__ for s in self.sensors],
            "links": [l.to_dict() for l in self.links],
            "joints": [j.to_dict() for j in self.joints],
        }

    def __repr__(self) -> str:
        return (
            f"Robot(name='{self.name}', links={self.num_links}, "
            f"joints={self.num_joints}, dof={self.dof}, "
            f"sensors={len(self.sensors)})"
        )
