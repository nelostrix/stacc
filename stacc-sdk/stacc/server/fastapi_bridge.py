"""FastAPI + WebSocket bridge — the richer live API for UIs.

Same :class:`StaccRuntime` underneath as the stdlib bridge, but with a proper
OpenAPI surface, a live WebSocket telemetry stream, and camera JPEG frames.

Requires the optional ``server`` extra (fastapi, uvicorn, websockets):

    pip install stacc[server]

Run:
    stacc serve --fastapi --host 0.0.0.0 --port 8002
"""

from __future__ import annotations

import asyncio
import json
from typing import Any, Optional

from stacc.server.runtime import StaccRuntime, create_robot, available_robots

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
    from fastapi.responses import Response

    HAS_FASTAPI = True
except ImportError:  # pragma: no cover
    HAS_FASTAPI = False


def _require_fastapi():
    if not HAS_FASTAPI:
        raise ImportError(
            "fastapi is not installed. Install it with: pip install 'stacc[server]'"
        )


def _ws_json(obj: Any) -> str:
    return json.dumps(obj, default=lambda o: None)


def create_app(runtime: Optional[StaccRuntime] = None) -> "FastAPI":
    """Build the FastAPI app around a StaccRuntime."""
    _require_fastapi()
    rt = runtime if runtime is not None else StaccRuntime()
    app = FastAPI(title="stacc", version="0.3.0", description="stacc live robotics API")

    # ── robots ──
    @app.get("/api/robots")
    def list_robots():
        return {"robots": available_robots()}

    @app.get("/api/robot")
    def get_robot():
        if rt.robot is None:
            raise HTTPException(status_code=404, detail="No robot loaded")
        return rt.robot_info()

    @app.post("/api/robot")
    def load_robot(payload: dict):
        kind = payload.get("type", "arm")
        kwargs = payload.get("kwargs", {}) or {}
        try:
            rt.set_robot(create_robot(kind, **kwargs))
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        return rt.robot_info()

    # ── sim controls ──
    @app.get("/api/state")
    def get_state():
        return rt.snapshot()

    @app.post("/api/sim/start")
    def sim_start():
        rt.start()
        return rt.snapshot()

    @app.post("/api/sim/pause")
    def sim_pause():
        rt.pause()
        return rt.snapshot()

    @app.post("/api/sim/resume")
    def sim_resume():
        rt.resume()
        return rt.snapshot()

    @app.post("/api/sim/stop")
    def sim_stop():
        rt.stop()
        return rt.snapshot()

    @app.post("/api/sim/reset")
    def sim_reset():
        rt.reset()
        return rt.snapshot()

    @app.post("/api/safety/emergency-stop")
    def emergency_stop(payload: dict | None = None):
        rt.emergency_stop(str((payload or {}).get("reason", "operator request")))
        return rt.snapshot()

    @app.post("/api/safety/emergency-stop/clear")
    def clear_emergency_stop():
        rt.clear_emergency_stop()
        return rt.snapshot()

    @app.post("/api/sim/step")
    def sim_step(payload: dict | None = None):
        n = int(payload.get("n", 1)) if payload else 1
        for _ in range(max(1, n)):
            rt.step_once()
        return rt.snapshot()

    # ── actuation ──
    @app.post("/api/joints")
    def command_joints(payload: dict):
        positions = payload.get("positions")
        if positions is not None:
            rt.set_joint_positions([float(p) for p in positions])
        else:
            for joint_id, angle in payload.get("joints", {}).items():
                rt.set_joint_angle(joint_id, float(angle))
        return rt.snapshot()

    @app.post("/api/torques")
    def command_torques(payload: dict):
        torques = payload.get("torques")
        if torques is None:
            raise HTTPException(status_code=400, detail="'torques' list required")
        rt.set_joint_torques([float(t) for t in torques])
        return rt.snapshot()

    @app.post("/api/gains")
    def set_gains(payload: dict):
        rt.set_gains(
            kp=payload.get("kp"),
            kd=payload.get("kd"),
            gravity_comp=payload.get("gravity_comp"),
        )
        return rt.robot_info()

    # ── sensors / camera ──
    @app.get("/api/camera/{sensor_name}.jpg")
    def camera_frame(sensor_name: str):
        try:
            jpeg = rt.camera_jpeg(sensor_name)
        except RuntimeError as e:
            raise HTTPException(status_code=404, detail=str(e))
        return Response(content=jpeg, media_type="image/jpeg")

    # ── live telemetry ──
    @app.websocket("/api/ws")
    async def telemetry_ws(ws: WebSocket):
        await ws.accept()
        period = 1.0 / max(rt.hz, 1e-9)
        try:
            while True:
                await ws.send_text(_ws_json(rt.snapshot()))
                await asyncio.sleep(period)
        except WebSocketDisconnect:
            pass

    return app


def serve(runtime: Optional[StaccRuntime] = None, host: str = "0.0.0.0", port: int = 8002,
          robot: str = "arm", physics: str = "mujoco", hz: float = 60.0,
          timestep: float = 0.001, command_timeout: Optional[float] = None):
    """Run the FastAPI app via uvicorn (blocks until killed)."""
    _require_fastapi()
    import uvicorn

    if runtime is None:
        rt = StaccRuntime(physics=physics, hz=hz, timestep=timestep,
                          command_timeout=command_timeout)
        rt.set_robot(create_robot(robot))
    else:
        rt = runtime
    app = create_app(rt)
    if not rt.running:
        rt.start()  # live by default; POST /api/sim/start|stop toggles it
    print(f"stacc fastapi: http://{host}:{port}  (robot={rt.robot.name if rt.robot else None}, hz={rt.hz:g})")
    print(f"  Docs:  http://{host}:{port}/docs")
    print(f"  State: http://{host}:{port}/api/state")
    print(f"  Live:  ws://{host}:{port}/api/ws")
    uvicorn.run(app, host=host, port=port)  # reload/lifespan handled by uvicorn


if __name__ == "__main__":
    import argparse
    from stacc.server import available_robots

    parser = argparse.ArgumentParser(description="stacc live UI bridge (FastAPI + WebSocket)")
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
    serve(host=args.host, port=args.port, robot=args.robot, physics=args.physics,
          hz=args.hz, timestep=args.timestep, command_timeout=args.command_timeout)
