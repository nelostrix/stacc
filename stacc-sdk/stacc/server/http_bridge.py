"""STACC HTTP/UI bridge — stdlib-only REST server for driving a live simulation.

This preserves the legacy ``/api/stacc/*`` contract that UIs already consume
(the NELO-Studio-style state/joint/gains/step/reset endpoints) but backs it
with a real :class:`StaccRuntime` — actual robot, real physics (MuJoCo), and
live joint/sensor telemetry — instead of the old hardcoded mock state.

Zero third-party dependencies: built on ``http.server``. For the richer
FastAPI + WebSocket API see ``stacc.server.fastapi_bridge`` and ``stacc serve
--fastapi``.

Run directly:
    python -m stacc.server.http_bridge --robot arm --physics mujoco --port 8002
"""

from __future__ import annotations

import json
import math
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from typing import Any, Dict, Optional

from stacc.server.runtime import StaccRuntime, create_robot, available_robots

_NO_ROBOT = {"error": "No robot loaded"}


def legacy_state(runtime: StaccRuntime) -> Dict[str, Any]:
    """Render the runtime snapshot in the legacy UI JSON shape (angles in degrees)."""
    if runtime.robot is None:
        return _NO_ROBOT
    snapshot = runtime.snapshot()
    joints = []
    for i, j in enumerate(snapshot["joints"]):
        info = runtime.joint_info()[i] if i < len(runtime.joint_info()) else {}
        lo, hi = info.get("min"), info.get("max")
        cmd_pos = j.get("command_position")
        joints.append({
            "id": j["id"],
            "name": j["name"],
            "angle": round(math.degrees(j["position"]), 4),
            "vel": round(math.degrees(j["velocity"]), 4),
            "torque": round(j["torque"], 4),
            "min": round(math.degrees(lo), 2) if lo is not None and math.isfinite(lo) else -360.0,
            "max": round(math.degrees(hi), 2) if hi is not None and math.isfinite(hi) else 360.0,
            "limit_torque": info.get("max_effort", 100.0),
            "command_angle": round(math.degrees(cmd_pos), 4) if cmd_pos is not None else None,
        })
    ee = snapshot.get("end_effector")
    end_effector = {}
    if ee:
        end_effector = {
            "x": round(ee.get("x", 0.0), 4),
            "y": round(ee.get("y", 0.0), 4),
            "z": round(ee.get("z", 0.0), 4),
            "roll": round(math.degrees(ee.get("roll", 0.0)), 2),
            "pitch": round(math.degrees(ee.get("pitch", 0.0)), 2),
            "yaw": round(math.degrees(ee.get("yaw", 0.0)), 2),
        }
    return {
        "robot_name": snapshot["robot_name"],
        "dof": snapshot["dof"],
        "frequency_hz": round(snapshot["hz"], 2),
        "jitter_ms": snapshot["jitter_ms"],
        "sim_time": round(snapshot["sim_time"], 3),
        "step": snapshot["step"],
        "is_simulating": snapshot["running"] and not snapshot["paused"],
        "joints": joints,
        "end_effector": end_effector,
        "control": snapshot["control"],
    }


class STACCRequestHandler(BaseHTTPRequestHandler):
    """HTTP handler backed by a global StaccRuntime (set via serve())."""

    runtime: StaccRuntime

    # ── plumbing ──

    def _send_json(self, data: Any, status: int = 200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, data: bytes, content_type: str, status: int = 200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> Dict[str, Any]:
        content_len = int(self.headers.get("Content-Length", 0))
        post_body = self.rfile.read(content_len) if content_len > 0 else b"{}"
        try:
            return json.loads(post_body.decode("utf-8"))
        except Exception:
            return {}

    def log_message(self, fmt, *args):
        pass  # keep the console quiet while polling

    # ── verbs ──

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/api/stacc/status", "/api/stacc/state"):
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/robots":
            self._send_json({"robots": available_robots()})
        elif path == "/api/stacc/robot":
            if self.runtime.robot is None:
                self._send_json(_NO_ROBOT)
            else:
                self._send_json(self.runtime.robot_info())
        elif path.startswith("/api/stacc/camera/"):
            sensor = path.rsplit("/", 1)[-1].removesuffix(".jpg")
            try:
                jpeg = self.runtime.camera_jpeg(sensor)
                self._send_bytes(jpeg, "image/jpeg")
            except RuntimeError as e:
                self._send_json({"error": str(e)}, status=409)
        else:
            self._send_json({"error": "Not Found"}, status=404)

    def do_POST(self):
        path = self.path.split("?")[0]
        payload = self._body()

        if path == "/api/stacc/start":
            try:
                self.runtime.start()
            except RuntimeError as e:
                self._send_json({"error": str(e)}, status=409)
            else:
                self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/pause":
            self.runtime.pause()
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/resume":
            self.runtime.resume()
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/stop":
            self.runtime.stop()
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/emergency-stop":
            self.runtime.emergency_stop(str(payload.get("reason", "operator request")))
            self._send_json(self.runtime.snapshot())
        elif path == "/api/stacc/emergency-stop/clear":
            self.runtime.clear_emergency_stop()
            self._send_json(self.runtime.snapshot())
        elif path == "/api/stacc/step":
            self.runtime.step_once()
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/reset":
            self.runtime.reset()
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/joint":
            joint_id = payload.get("joint_id", "J1")
            angle = float(payload.get("angle", 0.0))  # degrees
            try:
                self.runtime.set_joint_angle(joint_id, math.radians(angle))
            except ValueError as e:
                self._send_json({"error": str(e)}, status=400)
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/set_joints":
            positions = payload.get("positions") or payload.get("angles") or []
            try:
                self.runtime.set_joint_positions([math.radians(float(a)) for a in positions])
            except (ValueError, TypeError) as e:
                self._send_json({"error": str(e)}, status=400)
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/torques":
            torques = payload.get("torques") or []
            try:
                self.runtime.set_joint_torques([float(t) for t in torques])
            except (ValueError, TypeError) as e:
                self._send_json({"error": str(e)}, status=400)
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/gains":
            kp = payload.get("kp")
            kd = payload.get("kd")
            gc = payload.get("gravity_comp")
            self.runtime.set_gains(
                kp=float(kp) if kp is not None else None,
                kd=float(kd) if kd is not None else None,
                gravity_comp=gc if gc is not None else None,
            )
            self._send_json(legacy_state(self.runtime))
        elif path == "/api/stacc/robot":
            kind = payload.get("type", "arm")
            kwargs = payload.get("kwargs", {}) or {}
            try:
                self.runtime.set_robot(create_robot(kind, **kwargs))
            except ValueError as e:
                self._send_json({"error": str(e)}, status=400)
            self._send_json(self.runtime.robot_info())
        else:
            self._send_json({"error": "Not Found"}, status=404)


def build_runtime(
    robot_type: str = "arm",
    physics: str = "mujoco",
    hz: float = 60.0,
    timestep: float = 0.001,
    **kwargs,
) -> StaccRuntime:
    """Create a runtime preloaded with a robot from the factory registry."""
    runtime = StaccRuntime(physics=physics, hz=hz, timestep=timestep, **kwargs)
    runtime.set_robot(create_robot(robot_type))
    return runtime


def run_stacc_server(
    runtime: Optional[StaccRuntime] = None,
    host: str = "0.0.0.0",
    port: int = 8002,
    robot: str = "arm",
    physics: str = "mujoco",
    hz: float = 60.0,
    timestep: float = 0.001,
    command_timeout: Optional[float] = None,
):
    """Start the stdlib REST bridge (blocks until interrupted)."""
    if runtime is None:
        runtime = build_runtime(robot_type=robot, physics=physics, hz=hz,
                                timestep=timestep, command_timeout=command_timeout)
    if not runtime.running:
        runtime.start()  # live by default; /api/stacc/start|stop can toggle it
    STACCRequestHandler.runtime = runtime
    server = ThreadingHTTPServer((host, port), STACCRequestHandler)
    url = f"http://{host}:{port}"
    print(f"stacc bridge: {url}  (robot={runtime.robot.name if runtime.robot else None}, "
          f"engine={runtime.config.physics_engine}, hz={runtime.hz:g})")
    print(f"  REST: {url}/api/stacc/state")
    print(f"  View robots: {url}/api/stacc/robots")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop()
        server.server_close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="stacc live UI bridge (stdlib REST)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8002)
    parser.add_argument("--robot", default="arm",
                        choices=sorted(r["type"] for r in available_robots()))
    parser.add_argument("--physics", default="mujoco", choices=["mujoco", "internal"])
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--command-timeout", type=float, default=None,
                        help="Stop actuator commands after this many seconds without a fresh command")
    args = parser.parse_args()
    run_stacc_server(host=args.host, port=args.port, robot=args.robot, physics=args.physics,
                     hz=args.hz, timestep=args.timestep, command_timeout=args.command_timeout)
