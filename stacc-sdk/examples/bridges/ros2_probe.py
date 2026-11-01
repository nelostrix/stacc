#!/usr/bin/env python3
"""One-shot ROS 2 probe: read an observation, then publish a command.

Used by tests/domain/ros2Integration.test.ts to prove AC10 (the simulated robot
exchanges commands and observations with ROS 2) without holding a long-lived
node: it subscribes to the observation topic until one `sensor_msgs/JointState`
arrives, then publishes a command `JointState` on the command topic for ~1s so
the bridge (`examples/bridges/ros2_runtime_bridge.py`) can forward it to the
runtime. Emits a single JSON object on stdout.

Run inside the ROS 2 environment:
    ~/eyantra/ros2/run.sh python3 stacc-sdk/examples/bridges/ros2_probe.py \
        --joint joint_2 --angle 0.5
"""

from __future__ import annotations

import argparse
import json
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError as exc:  # pragma: no cover - environment dependent
    print(json.dumps({"error": f"rclpy unavailable: {exc}"}))
    raise SystemExit(2)


class Probe(Node):
    def __init__(self, state_topic: str, command_topic: str):
        super().__init__("nelo_ros2_probe")
        self.observed = None
        self.create_subscription(JointState, state_topic, self.on_state, 10)
        self.publisher = self.create_publisher(JointState, command_topic, 10)

    def on_state(self, msg: JointState):
        if self.observed is None:
            self.observed = {
                "names": [str(name) for name in msg.name],
                "positions": [float(value) for value in msg.position],
                "velocities": [float(value) for value in msg.velocity],
                "frame_id": str(msg.header.frame_id),
            }


def main() -> int:
    parser = argparse.ArgumentParser(description="Read one JointState and publish one command.")
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--command-topic", default="/nelo/joint_commands")
    parser.add_argument("--joint", default="joint_2")
    parser.add_argument("--angle", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=20.0, help="seconds to wait for an observation")
    args = parser.parse_args()

    rclpy.init()
    node = Probe(args.state_topic, args.command_topic)
    deadline = time.time() + args.timeout
    while node.observed is None and time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.observed is None:
        print(json.dumps({"error": "no observation received", "state_topic": args.state_topic}))
        node.destroy_node()
        rclpy.shutdown()
        return 3

    command = JointState()
    command.name = [args.joint]
    command.position = [args.angle]
    deadline = time.time() + 1.0
    while time.time() < deadline and rclpy.ok():
        command.header.stamp = node.get_clock().now().to_msg()
        node.publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)

    print(json.dumps({"observed": node.observed, "published": {"name": [str(n) for n in command.name], "position": [float(p) for p in command.position]}}))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
