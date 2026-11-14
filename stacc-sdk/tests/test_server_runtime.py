"""Tests for the stacc.server live runtime and its HTTP bridge.

They use the zero-dependency "internal" physics backend so they run anywhere
(MuJoCo optional). The HTTP bridge is exercised over a real socket with the
stdlib http.client — no third-party server framework needed.
"""
import json
import math
import threading
import time
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer

import pytest

try:
    import mujoco  # noqa: F401

    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False

from stacc.server.runtime import StaccRuntime, create_robot, available_robots
from stacc.sim.engine import _InternalPhysics


@pytest.fixture()
def runtime():
    rt = StaccRuntime(physics="internal", hz=200.0, timestep=0.001)
    rt.set_robot(create_robot("arm"))
    yield rt
    rt.stop()


def test_arm_robot_factory():
    robot = create_robot("arm")
    assert robot.name == "arm_6dof"
    assert robot.dof == 6
    assert [j.name for j in robot.joints][:2] == ["joint_1", "joint_2"]
    assert any(r["type"] == "arm" for r in available_robots())


def test_set_robot_with_unknown_type():
    with pytest.raises(ValueError):
        create_robot("mars_rover_9000")


def test_engine_reports_actual_backend_when_requested_backend_falls_back():
    rt = StaccRuntime(physics="mujoco", hz=200.0, timestep=0.001)
    rt.set_robot(create_robot("arm"))
    try:
        # Force the same unavailable-backend fallback without depending on whether
        # MuJoCo happens to be installed in this test environment.
        rt.engine._physics_backend = _InternalPhysics(rt.robot, rt.config)
        assert rt.engine.config.physics_engine == "mujoco"
        assert rt.engine.actual_physics_engine == "internal"
    finally:
        rt.stop()


def test_snapshot_shape(runtime):
    snap = runtime.snapshot()
    assert snap["robot_name"] == "arm_6dof"
    assert snap["dof"] == 6
    assert len(snap["joints"]) == 6
    assert snap["joints"][0]["name"] == "joint_1"
    assert {"position", "velocity", "torque"} <= set(snap["joints"][0])
    assert snap["running"] is False
    assert snap["end_effector"] is not None  # DH FK available


def test_start_stop_runs_real_physics(runtime):
    runtime.start()
    try:
        deadline = time.time() + 3.0
        while runtime.engine.state.step == 0 and time.time() < deadline:
            time.sleep(0.01)
        assert runtime.is_simulating
        assert runtime.engine.state.step > 0
        assert runtime.engine.state.time > 0.0
        assert runtime.engine._physics_backend is not None
    finally:
        runtime.stop()
    assert not runtime.is_simulating


def test_step_once_advances_time(runtime):
    t0 = runtime.engine.state.time
    runtime.step_once()
    assert runtime.engine.state.time == pytest.approx(t0 + 0.001)
    assert runtime.engine.state.step == 1


def test_set_joint_angle_clamped_to_limits(runtime):
    # Arm J2 limits are (-110, 110) degrees.
    runtime.set_joint_angle("J2", math.radians(500.0))
    assert runtime._cmd_positions[1] == pytest.approx(math.radians(110.0))
    runtime.set_joint_angle("joint_3", math.radians(-9999.0))
    assert runtime._cmd_positions[2] == pytest.approx(math.radians(-140.0))


def test_set_joint_positions_servo_torque(runtime):
    runtime.set_joint_angle("J5", 0.25)
    runtime._apply_command()
    torques = runtime.robot._state["joint_torques"]
    # J5 at rest (vel 0), commanded 0.25 rad -> tau = kp * err.
    assert torques[4] == pytest.approx(runtime.kp * 0.25, rel=1e-6)


def test_set_joint_torques_clears_command(runtime):
    runtime.set_joint_angle("J1", 0.5)
    runtime.set_joint_torques([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert runtime._cmd_positions is None
    assert runtime.robot._state["joint_torques"] == pytest.approx([1, 2, 3, 4, 5, 6])


def test_gains_and_robot_info(runtime):
    runtime.set_gains(kp=50.0, kd=2.0, gravity_comp=False)
    info = runtime.robot_info()
    assert info["control"]["kp"] == 50.0
    assert info["control"]["kd"] == 2.0
    assert info["control"]["gravity_comp"] is False
    assert info["joints"][0]["id"] == "J1"
    assert info["joints"][0]["min"] < info["joints"][0]["max"]


def test_snapshot_is_json_serializable(runtime):
    runtime.start()
    try:
        snap = runtime.snapshot()
    finally:
        runtime.stop()
    body = json.dumps(snap)
    assert body
    assert json.loads(body)["robot_name"] == "arm_6dof"


# ── HTTP bridge contract ───────────────────────────────────────────────────


class BridgeServer:
    def __init__(self, runtime):
        handler = make_handler(runtime)
        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self.httpd.shutdown()
        self.thread.join(timeout=2)
        self.httpd.server_close()


def make_handler(runtime):
    from stacc.server.http_bridge import STACCRequestHandler

    class _Handler(STACCRequestHandler):
        pass

    _Handler.runtime = runtime
    return _Handler


@pytest.fixture()
def bridge(runtime):
    server = BridgeServer(runtime).start()
    yield server
    server.stop()


def _request(port, method, path, body=None, headers=None):
    conn = HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        payload = json.dumps(body) if body is not None else None
        conn.request(method, path, body=payload, headers=headers or {})
        resp = conn.getresponse()
        data = resp.read()
        ctype = resp.getheader("Content-Type", "")
        if "json" in ctype:
            return resp.status, json.loads(data.decode("utf-8"))
        return resp.status, data
    finally:
        conn.close()


def test_bridge_contract_get_state(bridge, runtime):
    status, data = _request(bridge.port, "GET", "/api/stacc/state")
    assert status == 200
    assert data["robot_name"] == "arm_6dof"
    assert data["dof"] == 6
    assert len(data["joints"]) == 6
    assert data["joints"][0]["id"] == "J1"
    assert "angle" in data["joints"][0]
    assert "end_effector" in data
    assert data["control"]["mode"] == "JointImpedance"


def test_bridge_contract_status_alias(bridge):
    status, data = _request(bridge.port, "GET", "/")
    assert status == 200
    assert data["robot_name"] == "arm_6dof"


def test_bridge_joint_angle_degrees(bridge, runtime):
    status, data = _request(bridge.port, "POST", "/api/stacc/joint", {"joint_id": "J2", "angle": 45.0})
    assert status == 200
    assert data["joints"][1]["command_angle"] == pytest.approx(45.0, abs=0.01)
    assert runtime._cmd_positions[1] == pytest.approx(math.radians(45.0))


def test_bridge_reset_zeroes_state(bridge, runtime):
    _request(bridge.port, "POST", "/api/stacc/joint", {"joint_id": "J1", "angle": 33.0})
    _, before = _request(bridge.port, "GET", "/api/stacc/state")
    assert before["joints"][0]["command_angle"] == pytest.approx(33.0, abs=0.01)
    status, after = _request(bridge.port, "POST", "/api/stacc/reset")
    assert status == 200
    assert after["joints"][0]["angle"] == 0.0
    assert after["joints"][0]["command_angle"] is None
    assert runtime._cmd_positions is None


def test_bridge_start_stop(bridge, runtime):
    status, data = _request(bridge.port, "POST", "/api/stacc/start")
    assert status == 200
    assert data["is_simulating"] is True
    runtime.stop()
    status, data = _request(bridge.port, "POST", "/api/stacc/stop")
    assert status == 200
    assert data["is_simulating"] is False


def test_bridge_camera_404(bridge):
    status, data = _request(bridge.port, "GET", "/api/stacc/camera/none.jpg")
    assert status == 409  # RuntimeError -> conflict
    assert "error" in data


def test_factory_arm_physics_is_servo_stable():
    """The arm factory must ship plausible passive dynamics so a PD servo
    can actually converge (regression: frictionless point-mass chain used to
    flail at the joint-range limits)."""
    import pytest as _pytest  # noqa: F401 (keeps import style local)
    robot = create_robot("arm")
    for j in robot.joints:
        assert j.damping >= 10.0
        assert j.max_effort >= 400.0
    assert robot.links[-1].name == "flange"


def test_arm_servo_tracks_multijoint_pose():
    """A commanded multi-joint pose must be reached by the physics, not just
    echoed back (uses the fast internal integrator backend)."""
    rt = StaccRuntime(physics="internal", hz=500.0, timestep=0.001)
    rt.set_robot(create_robot("arm"))
    rt.set_joint_positions([math.radians(42), 0.0, math.radians(42), 0.0,
                            math.radians(30), math.radians(60)])
    rt.start()
    try:
        deadline = time.time() + 3.0
        q = rt.robot.get_joint_positions()
        while time.time() < deadline:
            q = rt.robot.get_joint_positions()
            errs = [abs(a - cmd) for a, cmd in zip(q, rt._cmd_positions)]
            if max(errs) < math.radians(1.0):
                break
            time.sleep(0.05)
        goal = [math.radians(42), 0.0, math.radians(42), 0.0,
                math.radians(30), math.radians(60)]
        assert max(abs(a - g) for a, g in zip(q, goal)) < math.radians(2.0)
    finally:
        rt.stop()


@pytest.mark.skipif(not HAS_MUJOCO, reason="MuJoCo not installed")
def test_mujoco_snapshot_restore_replays_identically():
    """A restored full-physics snapshot reproduces the same subsequent state."""
    import hashlib

    rt = StaccRuntime(physics="mujoco", hz=1000.0, timestep=0.001, headless=True)
    rt.set_robot(create_robot("arm"))
    rt.set_joint_positions([0.15, -0.2, 0.1, 0.05, -0.1, 0.2])
    try:
        for _ in range(10):
            rt.step_once()
        backend = rt.engine._physics_backend
        initial = backend.snapshot()
        for _ in range(100):
            rt.step_once()
        first = hashlib.sha256(backend.snapshot()).hexdigest()

        backend.restore(initial)
        for _ in range(100):
            rt.step_once()
        replay = hashlib.sha256(backend.snapshot()).hexdigest()
        assert replay == first
    finally:
        rt.stop()


@pytest.mark.skipif(not HAS_MUJOCO, reason="MuJoCo not installed")
def test_arm_servo_tracks_pose_mujoco():
    """MuJoCo-backed arm must serve a 6-DOF pose without limit-range flail."""
    rt = StaccRuntime(physics="mujoco", hz=1000.0, timestep=0.001, headless=True)
    rt.set_robot(create_robot("arm"))
    rt.set_joint_positions([math.radians(42), -math.radians(20),
                            math.radians(42), 0.0, math.radians(30), math.radians(60)])
    rt.start()
    try:
        time.sleep(2.0)
        q = rt.robot.get_joint_positions()
        goal = [math.radians(42), -math.radians(20), math.radians(42), 0.0,
                math.radians(30), math.radians(60)]
        assert max(abs(a - g) for a, g in zip(q, goal)) < math.radians(3.0)
        assert max(abs(v) for v in rt.robot.get_joint_velocities()) < 0.5
    finally:
        rt.stop()