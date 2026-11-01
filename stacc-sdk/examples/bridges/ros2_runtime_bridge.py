#!/usr/bin/env python3
"""NELO sim <-> ROS 2 bridge: exchange observations and commands with a live sim.

Runs INSIDE the ROS 2 environment (the `ubuntu-ros` Distrobox via
`~/eyantra/ros2/run.sh`) as an `rclpy` node and bridges it to the STACC
controller runtime API (`/v1/runtime/*`, docs/API_CONTRACTS.md §5) which it
reaches over loopback. This is the concrete realisation of acceptance criterion
10 — the simulated robot exchanges *observations* and *commands* with ROS 2:

  observations  sim -> ROS : `sensor_msgs/JointState` on `/joint_states`
                             (name/position/velocity/effort from telemetry)
  commands      ROS -> sim : `sensor_msgs/JointState` on `/nelo/joint_commands`
                             (each name/position pair becomes a joint command;
                              positions-only falls back to positional order)

The bridge owns no physics and no robot structure: it is a transport adapter, so
the robot description stays independent of any controller or ROS node.

Run:
    ~/eyantra/ros2/run.sh python3 stacc-sdk/examples/bridges/ros2_runtime_bridge.py \
        --ros-args -p runtime_url:=http://127.0.0.1:8766 -p publish_hz:=10.0

Flags: --runtime-url, --publish-hz, --command-topic, --state-topic, --once-every
(seconds between "bridged N observations" log lines; 0 disables the log).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError as exc:  # pragma: no cover - environment dependent
    print(
        f"[ros2-bridge] rclpy is not available: {exc}. Run this inside the ROS 2 "
        "environment, e.g. ~/eyantra/ros2/run.sh python3 <this file>.",
        file=sys.stderr,
    )
    raise SystemExit(1)


def _get_json(url: str, timeout: float = 5.0):
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8") or "{}")


def _post_json(url: str, payload: dict, timeout: float = 5.0):
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = resp.read().decode("utf-8")
        return json.loads(body) if body else {}


class RuntimeRosBridge(Node):
    """Bridges `/joint_states` <-> the STACC runtime for one live simulation."""

    def __init__(self, runtime_url: str, publish_hz: float, state_topic: str, command_topic: str):
        super().__init__("nelo_runtime_bridge")
        self.runtime_url = runtime_url.rstrip("/")
        self.observations = 0
        self.commands = 0
        self.errors = 0
        self.publisher = self.create_publisher(JointState, state_topic, 10)
        self.create_subscription(JointState, command_topic, self.on_command, 10)
        self.create_timer(1.0 / max(publish_hz, 0.1), self.publish_state)
        self.get_logger().info(f"bridging {self.runtime_url} -> {state_topic}, {command_topic} -> runtime")

    # ── observations: sim -> ROS ──

    def publish_state(self):
        try:
            telemetry = _get_json(f"{self.runtime_url}/v1/runtime/telemetry")
        except (urllib.error.URLError, urllib.error.HTTPError, OSError, ValueError) as exc:
            self.errors += 1
            if self.errors % 20 == 1:
                self.get_logger().warn(f"runtime unreachable: {exc}")
            return
        joints = telemetry.get("joints") or []
        msg = JointState()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "base_link"
        msg.name = [str(j.get("name", j.get("id", f"joint_{i}"))) for i, j in enumerate(joints)]
        msg.position = [float(j.get("position", 0.0)) for j in joints]
        msg.velocity = [float(j.get("velocity", 0.0)) for j in joints]
        msg.effort = [float(j.get("torque", 0.0)) for j in joints]
        self.publisher.publish(msg)
        self.observations += 1

    # ── commands: ROS -> sim ──

    def on_command(self, msg: JointState):
        try:
            if msg.name:
                for name, position in zip(msg.name, msg.position):
                    _post_json(f"{self.runtime_url}/v1/runtime/joints/command", {"joint": name, "angle_rad": float(position)})
            elif msg.position:
                _post_json(f"{self.runtime_url}/v1/runtime/joints/command", {"positions": [float(p) for p in msg.position]})
            else:
                return
            self.commands += 1
            self.get_logger().info(f"command #{self.commands}: {list(msg.name) or 'positional'} -> runtime")
        except (urllib.error.URLError, urllib.error.HTTPError, OSError, ValueError) as exc:
            self.errors += 1
            self.get_logger().error(f"command failed: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Bridge a live NELO/STACC simulation to ROS 2.")
    parser.add_argument("--runtime-url", default="http://127.0.0.1:8766")
    parser.add_argument("--publish-hz", type=float, default=10.0)
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--command-topic", default="/nelo/joint_commands")
    parser.add_argument("--once-every", type=float, default=10.0, help="seconds between activity logs (0 disables)")
    args = parser.parse_args()

    rclpy.init()
    node = RuntimeRosBridge(args.runtime_url, args.publish_hz, args.state_topic, args.command_topic)
    last_log = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.once_every > 0 and time.time() - last_log >= args.once_every:
                node.get_logger().info(f"observations={node.observations} commands={node.commands} errors={node.errors}")
                last_log = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
