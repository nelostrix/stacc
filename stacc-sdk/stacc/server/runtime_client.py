"""STACC controller runtime client — the stable API for controller processes.

Controllers do not import the physics engine: they talk to the STACC daemon's
versioned ``/v1/runtime/*`` surface over loopback HTTP (see
docs/API_CONTRACTS.md §5 in NELO Studio). This module is the Python binding for
that contract; the C++ controller SDK binds to the identical JSON over HTTP, so
a controller written for one language is structurally portable to the other.

Units at this surface are SI: radians, seconds, newton-metres, metres. (The
legacy ``/v1/robot/simulate_step`` route keeps degrees for the UI; controllers
never use it.)

Example::

    from stacc.server.runtime_client import RuntimeClient

    rt = RuntimeClient()                       # 127.0.0.1:8766
    print(rt.capabilities()["joints"])         # capability check before use
    rt.load_project(open("robot.rss").read())
    rt.start()
    rt.command_joint("J2", 0.5)                # radians
    for _ in range(200):
        rt.step()
    print(rt.joints()[1]["position"])
    rt.stop()

Zero third-party dependencies (stdlib only).
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional

DEFAULT_BASE = "http://127.0.0.1:8766"
RUNTIME_API_VERSION = 1


class RuntimeApiError(RuntimeError):
    """Raised when the runtime API rejects a call or the daemon is unreachable."""

    def __init__(self, message: str, status: Optional[int] = None, kind: Optional[str] = None):
        super().__init__(message)
        self.status = status
        self.kind = kind


class RuntimeClient:
    """Thin, typed client for the STACC ``/v1/runtime/*`` controller API."""

    def __init__(self, base: str = DEFAULT_BASE, timeout: float = 10.0):
        self.base = base.rstrip("/")
        self.timeout = timeout

    # ── transport ──

    def _request(self, method: str, path: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        url = f"{self.base}{path}"
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8")
                return json.loads(body) if body else {}
        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", "replace")
            try:
                parsed = json.loads(body)
            except json.JSONDecodeError:
                parsed = {"error": body or f"HTTP {e.code}"}
            raise RuntimeApiError(
                parsed.get("error", f"HTTP {e.code}"), status=e.code, kind=parsed.get("kind")
            ) from e
        except urllib.error.URLError as e:
            raise RuntimeApiError(f"STACC daemon unreachable at {self.base}: {e.reason}") from e

    def _get(self, path: str) -> Dict[str, Any]:
        return self._request("GET", path)

    def _post(self, path: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return self._request("POST", path, payload or {})

    # ── capabilities ──

    def capabilities(self) -> Dict[str, Any]:
        """Engine + API declaration. A controller SHOULD verify api_version here.

        Raises RuntimeApiError if the daemon speaks a newer breaking API version.
        """
        caps = self._get("/v1/runtime/capabilities")
        version = caps.get("api_version")
        if isinstance(version, int) and version > RUNTIME_API_VERSION:
            raise RuntimeApiError(
                f"Runtime API v{version} is newer than this client supports (v{RUNTIME_API_VERSION})",
                kind="version",
            )
        return caps

    # ── lifecycle ──

    def load_project(self, source: str, format: str = "rss") -> Dict[str, Any]:
        return self._post("/v1/runtime/load_project", {"format": format, "source": source})

    def start(self) -> Dict[str, Any]:
        return self._post("/v1/runtime/start")

    def pause(self) -> Dict[str, Any]:
        return self._post("/v1/runtime/pause")

    def resume(self) -> Dict[str, Any]:
        return self._post("/v1/runtime/resume")

    def reset(self) -> Dict[str, Any]:
        return self._post("/v1/runtime/reset")

    def step(self, count: int = 1) -> Dict[str, Any]:
        return self._post("/v1/runtime/step", {"count": int(count)})

    def speed(self, hz: Optional[float] = None, realtime_factor: Optional[float] = None) -> Dict[str, Any]:
        payload: Dict[str, Any] = {}
        if hz is not None:
            payload["hz"] = float(hz)
        if realtime_factor is not None:
            payload["realtime_factor"] = float(realtime_factor)
        return self._post("/v1/runtime/speed", payload)

    def stop(self) -> Dict[str, Any]:
        return self._post("/v1/runtime/stop")

    def kill(self) -> Dict[str, Any]:
        return self._post("/v1/runtime/kill")

    # ── reading ──

    def joints(self) -> List[Dict[str, Any]]:
        return self._get("/v1/runtime/joints").get("joints", [])

    def sensors(self) -> Dict[str, Any]:
        return self._get("/v1/runtime/sensors").get("sensors", {})

    def telemetry(self) -> Dict[str, Any]:
        return self._get("/v1/runtime/telemetry")

    def joint_by_name(self, name: str) -> Optional[Dict[str, Any]]:
        for joint in self.joints():
            if joint.get("name") == name or joint.get("id") == name:
                return joint
        return None

    # ── commanding ──

    def command_joints(self, positions: List[float]) -> Dict[str, Any]:
        return self._post("/v1/runtime/joints/command", {"positions": [float(p) for p in positions]})

    def command_joint(self, joint: str, angle_rad: float) -> Dict[str, Any]:
        return self._post("/v1/runtime/joints/command", {"joint": joint, "angle_rad": float(angle_rad)})

    def command_torques(self, torques: List[float]) -> Dict[str, Any]:
        return self._post("/v1/runtime/joints/command", {"torques": [float(t) for t in torques]})

    def gains(self, kp: Optional[float] = None, kd: Optional[float] = None,
              gravity_comp: Optional[bool] = None) -> Dict[str, Any]:
        payload: Dict[str, Any] = {}
        if kp is not None:
            payload["kp"] = float(kp)
        if kd is not None:
            payload["kd"] = float(kd)
        if gravity_comp is not None:
            payload["gravity_comp"] = bool(gravity_comp)
        return self._post("/v1/runtime/control/gains", payload)


def _demo() -> None:
    """AC5 smoke: command one joint through the runtime and report it moved."""
    import sys

    rt = RuntimeClient()
    caps = rt.capabilities()
    print(f"engine={caps['engine']} dof={caps['joints']} api=v{caps['api_version']}")
    if caps["joints"] < 2:
        print("No second joint to command")
        sys.exit(1)
    rt.command_joint("J2", 0.5)
    for _ in range(300):
        rt.step()
    moved = rt.joints()[1]["position"]
    print(f"J2 position now {moved:.4f} rad")
    sys.exit(0 if abs(moved) > 1e-3 else 2)


if __name__ == "__main__":
    _demo()
