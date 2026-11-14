"""Shared-memory control channel (stacc/server/rt_channel.py) and its runtime loop."""

import math
import os
import struct
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stacc.server.rt_channel import (  # noqa: E402
    KIND_POSITION,
    MODE_LOCKSTEP,
    MODE_REALTIME,
    ControlChannel,
)

ARM = "robot arm { body { shape: box; dimensions: [200mm, 200mm, 80mm]; mass: 5kg; } arm main { joints: 3; reach: 600mm; mass: 0.6kg; } }"


@pytest.fixture
def shm_path():
    path = f"/dev/shm/stacc_rt_test_{os.getpid()}_{time.monotonic_ns()}"
    yield path
    if os.path.exists(path):
        os.unlink(path)


def test_state_and_command_round_trip(shm_path):
    owner = ControlChannel.create(shm_path, n_joints=2, n_thrusters=1, mode=MODE_LOCKSTEP)
    client = ControlChannel.open(shm_path)
    try:
        assert (client.n_joints, client.n_thrusters, client.mode) == (2, 1, MODE_LOCKSTEP)
        owner.write_state(7, 0.007, [1, 2, 3, 1, 0, 0, 0], [0.1, 0.2], [0.3, 0.4], [0.5, 0.6], [2.5])
        s = client.read_state()
        assert (s.step, s.sim_time, s.position, s.torque, s.thrust) == (7, 0.007, [0.1, 0.2], [0.5, 0.6], [2.5])
        assert client.read_command() is None  # nothing commanded yet
        cid = client.command_positions([0.5, -0.5], answers_step=7, thrust=[3.0])
        c = owner.read_command()
        assert (c.cmd_id, c.answers_step, c.kind, c.values, c.thrust) == (cid, 7, KIND_POSITION, [0.5, -0.5], [3.0])
        assert client.command_positions([0, 0], answers_step=8) == cid + 1
    finally:
        client.close()
        owner.close()
    assert not os.path.exists(shm_path)


def test_channel_rejects_invalid_modes_and_nonfinite_commands(shm_path):
    with pytest.raises(ValueError, match="mode"):
        ControlChannel.create(shm_path, n_joints=1, mode=7)
    channel = ControlChannel.create(shm_path, n_joints=1, n_thrusters=1)
    try:
        with pytest.raises(ValueError, match="finite"):
            channel.command_positions([math.nan], answers_step=0)
        with pytest.raises(ValueError, match="finite"):
            channel.command_torques([0.0], answers_step=0, thrust=[math.nan])
    finally:
        channel.close()


def test_reader_never_returns_a_half_written_block(shm_path):
    owner = ControlChannel.create(shm_path, n_joints=1)
    try:
        owner.write_state(1, 0.001, [0] * 7, [1.0], [0.0], [0.0])
        # Simulate a writer caught mid-update: seq odd.
        off = owner._state_off
        seq = struct.unpack_from("<Q", owner._mm, off)[0]
        struct.pack_into("<Q", owner._mm, off, seq + 1)
        assert owner._read_consistent(off, 64, retries=50) is None
        struct.pack_into("<Q", owner._mm, off, seq + 2)
        assert owner.read_state().position == [1.0]
    finally:
        owner.close()


def _runtime(robot_src: str):
    from stacc.rss import compile_document, parse
    from stacc.server.runtime import StaccRuntime

    rt = StaccRuntime(physics="mujoco", hz=1000.0)
    rt.set_robot(compile_document(parse(robot_src)))
    return rt


def _lockstep_run(path: str, steps: int):
    """A controller closing the loop over shared memory: each step, read the
    state and command a target that depends on it. Returns the trajectory."""
    rt = _runtime(ARM)
    rt.enable_channel(path, mode=MODE_LOCKSTEP, timeout_s=2.0)
    ch = ControlChannel.open(path)
    trajectory = []
    try:
        rt.start()
        state = ch.wait_state(after_step=-1, timeout_s=5.0)
        for _ in range(steps):
            q = state.position
            target = [0.5 * math.sin(state.sim_time * 4.0), 0.6 - 0.2 * q[0], -0.4]
            ch.command_positions(target, answers_step=state.step)
            nxt = ch.wait_state(after_step=state.step, timeout_s=5.0)
            assert nxt is not None, "runtime stopped publishing"
            state = nxt
            trajectory.append((state.step, tuple(round(v, 12) for v in state.position)))
        status = rt.channel_status()
    finally:
        rt.stop()
        ch.close()
        rt.disable_channel()
    return trajectory, status


def test_lockstep_is_deterministic_and_never_misses(shm_path):
    pytest.importorskip("mujoco")
    first, status = _lockstep_run(shm_path, 300)
    second, _ = _lockstep_run(shm_path, 300)
    assert first == second  # same controller, same run, bit for bit
    assert status["missed_deadlines"] == 0
    assert status["late_commands"] == 0
    assert first[-1][0] - first[0][0] == 299  # exactly one physics step per command


def test_realtime_mode_tracks_an_async_controller(shm_path):
    pytest.importorskip("mujoco")
    rt = _runtime(ARM)
    rt.enable_channel(shm_path, mode=MODE_REALTIME)
    ch = ControlChannel.open(shm_path)
    stop = threading.Event()

    def controller():
        while not stop.is_set():
            s = ch.read_state()
            if s is not None:
                ch.command_positions([0.4, 0.3, -0.5], answers_step=s.step)
            time.sleep(0.002)  # a slower controller: the runtime does not wait for it

    t = threading.Thread(target=controller)
    try:
        rt.start()
        t.start()
        time.sleep(2.0)
        state = ch.read_state()
        status = rt.channel_status()
    finally:
        stop.set()
        t.join()
        rt.stop()
        ch.close()
        rt.disable_channel()
    assert state.position[0] == pytest.approx(0.4, abs=0.02)
    assert status["commands"] > 100
    assert status["missed_deadlines"] == 0  # realtime never waits, so never misses
