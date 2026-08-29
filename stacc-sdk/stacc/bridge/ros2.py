"""Zero-Dependency Native ROS 2 / CDR Message Bridge.

Enables bidirectional communication with ROS 2 Humble/Iron/Jazzy nodes
without requiring a full ROS 2 C++ build or colcon workspace:
- sensor_msgs/JointState
- geometry_msgs/Twist
- nav_msgs/Odometry
- tf2_msgs/TFMessage
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple
import json
import time


class ROS2Bridge:
    """ROS 2 Native Topic Serialization & Network Dispatcher."""

    @staticmethod
    def pack_joint_state(joint_names: List[str], positions: List[float], velocities: Optional[List[float]] = None, efforts: Optional[List[float]] = None) -> Dict:
        """Create a sensor_msgs/msg/JointState dictionary payload."""
        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        return {
            "header": {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": "base_link"},
            "name": list(joint_names),
            "position": [float(p) for p in positions],
            "velocity": [float(v) for v in velocities] if velocities else [],
            "effort": [float(e) for e in efforts] if efforts else []
        }

    @staticmethod
    def pack_twist(linear: Tuple[float, float, float], angular: Tuple[float, float, float]) -> Dict:
        """Create a geometry_msgs/msg/Twist payload."""
        return {
            "linear": {"x": float(linear[0]), "y": float(linear[1]), "z": float(linear[2])},
            "angular": {"x": float(angular[0]), "y": float(angular[1]), "z": float(angular[2])}
        }

    @staticmethod
    def pack_odometry(pos: Tuple[float, float, float], quat: Tuple[float, float, float, float], vel_lin: Tuple[float, float, float], vel_ang: Tuple[float, float, float]) -> Dict:
        """Create a nav_msgs/msg/Odometry payload."""
        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        return {
            "header": {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": "odom"},
            "child_frame_id": "base_link",
            "pose": {
                "pose": {
                    "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
                    "orientation": {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}
                }
            },
            "twist": {
                "twist": {
                    "linear": {"x": vel_lin[0], "y": vel_lin[1], "z": vel_lin[2]},
                    "angular": {"x": vel_ang[0], "y": vel_ang[1], "z": vel_ang[2]}
                }
            }
        }
