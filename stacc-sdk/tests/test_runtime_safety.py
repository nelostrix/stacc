"""Software safety and timing contract for the live STACC runtime.

These tests use the internal simulator and do not connect to hardware.
"""
import math
import time

import pytest

from stacc.server.runtime import StaccRuntime, create_robot


def make_runtime(**kwargs):
    config = {"physics": "internal", "hz": 100.0, "timestep": 0.001}
    config.update(kwargs)
    rt = StaccRuntime(**config)
    rt.set_robot(create_robot("arm"))
    return rt


@pytest.mark.parametrize("kwargs", [
    {"hz": 0}, {"hz": -1}, {"hz": math.inf}, {"hz": 10001},
    {"timestep": 0}, {"timestep": math.nan}, {"command_timeout": 0},
])
def test_rejects_invalid_runtime_timing(kwargs):
    with pytest.raises(ValueError):
        StaccRuntime(**kwargs)


def test_commands_are_dimension_checked_finite_and_bounded():
    rt = make_runtime()
    with pytest.raises(ValueError, match="expected 6"):
        rt.set_joint_positions([0.1])
    with pytest.raises(ValueError, match="finite"):
        rt.set_joint_positions([math.nan] * 6)
    rt.set_joint_torques([1e9] * 6)
    assert all(abs(t) <= j.max_effort for t, j in zip(rt.robot._state["joint_torques"], rt.robot.joints))
    with pytest.raises(ValueError, match="expected 6"):
        rt.set_joint_torques([0.0])
    with pytest.raises(ValueError, match="finite"):
        rt.set_joint_angle("J1", math.inf)
    rt.stop()


def test_emergency_stop_latches_clears_outputs_and_blocks_commands():
    rt = make_runtime()
    rt.set_joint_positions([0.2] * 6)
    rt._apply_command()
    rt.emergency_stop("operator")
    snap = rt.snapshot()
    assert snap["safety"]["emergency_stopped"] is True
    assert snap["safety"]["emergency_stop_reason"] == "operator"
    assert rt._cmd_positions is None
    assert "servo" not in rt.robot._state
    assert rt.robot._state["joint_torques"] == [0.0] * 6
    with pytest.raises(RuntimeError, match="emergency stop"):
        rt.set_joint_positions([0.0] * 6)
    with pytest.raises(RuntimeError, match="latched"):
        rt.start()
    rt.clear_emergency_stop()
    rt.set_joint_positions([0.0] * 6)
    rt.stop()


def test_command_watchdog_drops_stale_command():
    rt = make_runtime(command_timeout=0.04)
    rt.set_joint_torques([20.0] * 6)
    rt.start()
    try:
        deadline = time.monotonic() + 1.0
        while rt._watchdog_trips == 0 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert rt._watchdog_trips == 1
        assert rt._cmd_positions is None
        assert rt.robot._state["joint_torques"] == [0.0] * 6
        assert rt.snapshot()["safety"]["watchdog_trips"] == 1
    finally:
        rt.stop()


def test_shared_memory_commands_buffered_while_stopped_are_discarded(tmp_path):
    from stacc.server.rt_channel import ControlChannel

    rt = make_runtime(hz=200.0)
    path = str(tmp_path / "stacc-rt")
    rt.enable_channel(path)
    controller = ControlChannel.open(path)
    try:
        controller.command_positions([0.25] * 6, answers_step=0)
        rt.start()
        time.sleep(0.03)
        assert rt._cmd_positions is None

        state = controller.read_state()
        controller.command_positions([0.1] * 6, answers_step=state.step)
        deadline = time.monotonic() + 1.0
        while rt._cmd_positions is None and time.monotonic() < deadline:
            time.sleep(0.005)
        assert rt._cmd_positions == pytest.approx([0.1] * 6)
    finally:
        rt.stop()
        controller.close()
        rt.disable_channel()


def test_software_stop_clears_actuator_outputs():
    rt = make_runtime()
    rt.start()
    rt.set_joint_positions([0.2] * 6)
    time.sleep(0.03)
    rt.stop()
    assert rt._cmd_positions is None
    assert "servo" not in rt.robot._state
    assert rt.robot._state["joint_torques"] == [0.0] * 6


def test_fractional_physics_substeps_track_control_period():
    rt = StaccRuntime(physics="internal", hz=60.0, timestep=0.001)
    rt.set_robot(create_robot("arm"))
    rt.start()
    started = time.monotonic()
    time.sleep(0.25)
    elapsed = time.monotonic() - started
    rt.stop()
    assert rt.engine.state.time == pytest.approx(elapsed, abs=0.025)


def test_single_step_is_rejected_while_live_and_reset_restarts_cleanly():
    rt = make_runtime()
    rt.start()
    try:
        with pytest.raises(RuntimeError, match="stopped or paused"):
            rt.step_once()
        rt.reset()
        assert rt.running is True
        assert rt.engine.state.time >= 0.0
    finally:
        rt.stop()


def test_fastapi_bridge_exposes_latched_emergency_stop():
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient
    from stacc.server.fastapi_bridge import create_app

    rt = make_runtime()
    client = TestClient(create_app(rt))
    stopped = client.post("/api/safety/emergency-stop", json={"reason": "api-test"})
    assert stopped.status_code == 200
    assert stopped.json()["safety"]["emergency_stopped"] is True
    cleared = client.post("/api/safety/emergency-stop/clear")
    assert cleared.status_code == 200
    assert cleared.json()["safety"]["emergency_stopped"] is False
    rt.stop()


def test_physics_loop_fault_is_reported_and_stops_running_state():
    rt = make_runtime()
    rt.start()
    try:
        class BrokenBackend:
            def step(self, _dt):
                raise RuntimeError("simulated backend fault")

            def close(self):
                pass

        rt.engine._physics_backend = BrokenBackend()
        deadline = time.monotonic() + 1.0
        while rt.running and time.monotonic() < deadline:
            time.sleep(0.005)
        assert rt.running is False
        assert rt.paused is True
        assert rt.snapshot()["safety"]["runtime_fault"] == "RuntimeError: simulated backend fault"
        assert rt.robot._state["joint_torques"] == [0.0] * 6
    finally:
        rt.stop()
