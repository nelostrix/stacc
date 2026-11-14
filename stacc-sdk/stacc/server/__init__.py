"""stacc.server — live UI/server adapter for the stacc robotics SDK.

Two bridges share one thread-safe runtime:

- ``StaccRuntime`` drives a real robot + physics engine from a background
  loop, exposing snapshots and joint/torque commands to any UI.
- The stdlib REST bridge (``http_bridge.run_stacc_server``) preserves the
  legacy ``/api/stacc/*`` contract — zero dependencies.
- The optional FastAPI bridge (``fastapi_bridge.create_app``) adds a full
  OpenAPI surface, a WebSocket live-telemetry stream, and camera JPEGs.

Usage (programmatic):

    from stacc.server import StaccRuntime, create_robot

    rt = StaccRuntime(physics="mujoco", hz=120)
    rt.set_robot(create_robot("arm"))
    rt.start()
    state = rt.snapshot()
    rt.set_joint_angle("J2", 0.6)   # radians, servo-driven
    rt.stop()

CLI:

    stacc serve --robot arm --physics mujoco --port 8002
    stacc serve --fastapi --robot quadruped
"""

from __future__ import annotations

from typing import Optional

from stacc.server.runtime import (
    StaccRuntime,
    create_robot,
    available_robots,
    ROBOT_FACTORIES,
)
from stacc.server.http_bridge import run_stacc_server, build_runtime, legacy_state
from stacc.server.fastapi_bridge import create_app, serve as serve_fastapi
from stacc.server.runtime_client import RuntimeClient, RuntimeApiError

__all__ = [
    "StaccRuntime",
    "create_robot",
    "available_robots",
    "ROBOT_FACTORIES",
    "run_stacc_server",
    "build_runtime",
    "legacy_state",
    "create_app",
    "serve_fastapi",
    "RuntimeClient",
    "RuntimeApiError",
]


def serve(
    robot: str = "arm",
    physics: str = "mujoco",
    host: str = "0.0.0.0",
    port: int = 8002,
    hz: float = 60.0,
    timestep: float = 0.001,
    fastapi: bool = False,
    command_timeout: Optional[float] = None,
):
    """Launch either the stdlib REST bridge or the FastAPI+WebSocket server.

    Blocks until interrupted. Both share the same StaccRuntime underneath.
    """
    if fastapi:
        serve_fastapi(host=host, port=port, robot=robot, physics=physics,
                      hz=hz, timestep=timestep, command_timeout=command_timeout)
    else:
        run_stacc_server(host=host, port=port, robot=robot, physics=physics,
                         hz=hz, timestep=timestep, command_timeout=command_timeout)
