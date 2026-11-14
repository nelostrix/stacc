"""Real-time control channel: STACC runtime <-> controller over shared memory.

The HTTP API (/v1/runtime/*) is the control plane: load a robot, start, set
gains, enable this channel. This module is the data plane: one file in
/dev/shm with a fixed binary layout, read and written every control cycle
with no sockets, no JSON and no allocation.

Layout (little-endian; every field 8-byte aligned)::

    header   (64 B)  magic u32 'STRT', version u32, n_joints u32, n_thrusters u32,
                     mode u32 (0 realtime, 1 lockstep), reserved u32,
                     state_offset u64, command_offset u64, reserved 16 B
    state            seq u64, step u64, sim_time f64,
                     pose f64[7]           x, y, z, qw, qx, qy, qz (base, world)
                     position f64[n_joints], velocity f64[n_joints], torque f64[n_joints],
                     thrust f64[n_thrusters]
    command          seq u64, cmd_id u64, answers_step u64, kind u32, flags u32,
                     values f64[n_joints]  rad (kind 1, position) or N·m (kind 2, torque)
                     thrust f64[n_thrusters]   N, applied when flags & 1

Each block is a seqlock: its writer makes `seq` odd, writes the payload, then
makes it even. A reader copies the payload between two reads of `seq` and
retries if either was odd or they differ, so it never sees a torn frame and
neither side ever blocks the other. The runtime is the only writer of the
state block and a controller the only writer of the command block.

Joint order is the runtime's (`GET /v1/runtime/joints`); rotor order is
telemetry's `thrusters`.

Modes:
  realtime  the runtime steps on its own clock and applies the newest command
            each cycle; commands that arrive late are counted, never waited for.
  lockstep  after publishing step k the runtime waits until a command answering
            step k arrives (or `timeout_s` passes, counted as a missed
            deadline). Deterministic: the same controller gives the same run.

Python's mmap writes are plain stores; the seqlock is exact on x86-64 (total
store order). The C++ header uses acquire/release atomics on `seq` for
weakly-ordered CPUs.
"""

from __future__ import annotations

import mmap
import math
import os
import struct
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

MAGIC = 0x54525453  # 'STRT'
VERSION = 1
HEADER_SIZE = 64
MODE_REALTIME = 0
MODE_LOCKSTEP = 1
KIND_NONE = 0
KIND_POSITION = 1
KIND_TORQUE = 2
FLAG_THRUST = 1

_HEADER = struct.Struct("<IIIIII QQ 16x")
_STATE_HEAD = struct.Struct("<QQd")  # seq, step, sim_time
_CMD_HEAD = struct.Struct("<QQQII")  # seq, cmd_id, answers_step, kind, flags
_U64 = struct.Struct("<Q")


def _state_size(n_joints: int, n_thrusters: int) -> int:
    return _STATE_HEAD.size + 8 * (7 + 3 * n_joints + n_thrusters)


def _command_size(n_joints: int, n_thrusters: int) -> int:
    return _CMD_HEAD.size + 8 * (n_joints + n_thrusters)


@dataclass
class State:
    step: int
    sim_time: float
    pose: List[float]  # x, y, z, qw, qx, qy, qz
    position: List[float]
    velocity: List[float]
    torque: List[float]
    thrust: List[float]


@dataclass
class Command:
    cmd_id: int
    answers_step: int
    kind: int
    values: List[float]
    thrust: Optional[List[float]]


class ControlChannel:
    """One shared-memory control channel. The runtime calls `create`, a
    controller calls `open`; both then use the methods for their side."""

    def __init__(self, path: str, mm: mmap.mmap, fd: int, owner: bool):
        self.path = path
        self._mm = mm
        self._fd = fd
        self._owner = owner
        magic, version, nj, nt, mode, _r, state_off, cmd_off = _HEADER.unpack_from(mm, 0)
        if magic != MAGIC:
            raise ValueError(f"{path} is not a STACC control channel")
        if version != VERSION:
            raise ValueError(f"{path} uses channel layout v{version}; this client speaks v{VERSION}")
        self.n_joints, self.n_thrusters, self.mode = nj, nt, mode
        self._state_off, self._cmd_off = state_off, cmd_off
        self._state_fmt = struct.Struct(f"<{7 + 3 * nj + nt}d")
        self._cmd_fmt = struct.Struct(f"<{nj + nt}d")
        self._next_cmd_id = 0

    # ── lifecycle ──

    @classmethod
    def create(cls, path: str, n_joints: int, n_thrusters: int = 0, mode: int = MODE_REALTIME) -> "ControlChannel":
        if int(n_joints) < 0 or int(n_thrusters) < 0:
            raise ValueError("joint and thruster counts cannot be negative")
        if int(mode) not in (MODE_REALTIME, MODE_LOCKSTEP):
            raise ValueError("mode must be MODE_REALTIME or MODE_LOCKSTEP")
        state_off = HEADER_SIZE
        cmd_off = state_off + _state_size(n_joints, n_thrusters)
        size = cmd_off + _command_size(n_joints, n_thrusters)
        fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_TRUNC, 0o600)
        os.ftruncate(fd, size)
        mm = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        mm[:] = b"\0" * size
        _HEADER.pack_into(mm, 0, MAGIC, VERSION, n_joints, n_thrusters, mode, 0, state_off, cmd_off)
        return cls(path, mm, fd, owner=True)

    @classmethod
    def open(cls, path: str) -> "ControlChannel":
        fd = os.open(path, os.O_RDWR)
        size = os.fstat(fd).st_size
        mm = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        return cls(path, mm, fd, owner=False)

    def close(self) -> None:
        try:
            self._mm.close()
        finally:
            os.close(self._fd)
            if self._owner:
                try:
                    os.unlink(self.path)
                except FileNotFoundError:
                    pass

    def set_mode(self, mode: int) -> None:
        self.mode = mode
        struct.pack_into("<I", self._mm, 16, mode)

    # ── seqlock primitives ──

    def _begin_write(self, off: int) -> int:
        seq = _U64.unpack_from(self._mm, off)[0] + 1  # odd: writing
        _U64.pack_into(self._mm, off, seq)
        return seq

    def _end_write(self, off: int, seq: int) -> None:
        _U64.pack_into(self._mm, off, seq + 1)  # even: consistent

    def _read_consistent(self, off: int, size: int, retries: int = 1000) -> Optional[bytes]:
        for _ in range(retries):
            s1 = _U64.unpack_from(self._mm, off)[0]
            if s1 & 1:
                continue
            data = self._mm[off:off + size]
            if _U64.unpack_from(self._mm, off)[0] == s1:
                return data
        return None

    # ── runtime side ──

    def write_state(self, step: int, sim_time: float, pose: Sequence[float], position: Sequence[float],
                    velocity: Sequence[float], torque: Sequence[float], thrust: Sequence[float] = ()) -> None:
        off = self._state_off
        seq = self._begin_write(off)
        struct.pack_into("<Qd", self._mm, off + 8, step, sim_time)
        values = list(pose[:7]) + list(position) + list(velocity) + list(torque) + list(thrust)
        self._state_fmt.pack_into(self._mm, off + _STATE_HEAD.size, *values)
        self._end_write(off, seq)

    def read_command(self) -> Optional[Command]:
        size = _command_size(self.n_joints, self.n_thrusters)
        data = self._read_consistent(self._cmd_off, size)
        if data is None:
            return None
        _seq, cmd_id, answers, kind, flags = _CMD_HEAD.unpack_from(data, 0)
        if cmd_id == 0:
            return None  # nothing written yet
        values = list(self._cmd_fmt.unpack_from(data, _CMD_HEAD.size))
        nj = self.n_joints
        return Command(
            cmd_id=cmd_id,
            answers_step=answers,
            kind=kind,
            values=values[:nj],
            thrust=values[nj:] if flags & FLAG_THRUST else None,
        )

    # ── controller side ──

    def read_state(self) -> Optional[State]:
        size = _state_size(self.n_joints, self.n_thrusters)
        data = self._read_consistent(self._state_off, size)
        if data is None:
            return None
        _seq, step, sim_time = _STATE_HEAD.unpack_from(data, 0)
        v = self._state_fmt.unpack_from(data, _STATE_HEAD.size)
        nj, nt = self.n_joints, self.n_thrusters
        return State(
            step=step,
            sim_time=sim_time,
            pose=list(v[0:7]),
            position=list(v[7:7 + nj]),
            velocity=list(v[7 + nj:7 + 2 * nj]),
            torque=list(v[7 + 2 * nj:7 + 3 * nj]),
            thrust=list(v[7 + 3 * nj:7 + 3 * nj + nt]),
        )

    def wait_state(self, after_step: int, timeout_s: float = 1.0) -> Optional[State]:
        """Spin until the runtime publishes a step newer than `after_step`."""
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            state = self.read_state()
            if state is not None and state.step > after_step:
                return state
        return None

    def write_command(self, kind: int, values: Sequence[float], answers_step: int,
                      thrust: Optional[Sequence[float]] = None) -> int:
        if kind not in (KIND_POSITION, KIND_TORQUE):
            raise ValueError("kind must be KIND_POSITION or KIND_TORQUE")
        if int(answers_step) < 0:
            raise ValueError("answers_step cannot be negative")
        if len(values) != self.n_joints:
            raise ValueError(f"expected {self.n_joints} joint values, got {len(values)}")
        if thrust is not None and len(thrust) != self.n_thrusters:
            raise ValueError(f"expected {self.n_thrusters} thrust values, got {len(thrust)}")
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("command values must be finite")
        if thrust is not None and not all(math.isfinite(float(value)) for value in thrust):
            raise ValueError("thrust values must be finite")
        off = self._cmd_off
        self._next_cmd_id = max(self._next_cmd_id, _U64.unpack_from(self._mm, off + 8)[0]) + 1
        seq = self._begin_write(off)
        _CMD_HEAD.pack_into(self._mm, off, seq, self._next_cmd_id, answers_step, kind,
                            FLAG_THRUST if thrust is not None else 0)
        payload = list(values) + (list(thrust) if thrust is not None else [0.0] * self.n_thrusters)
        self._cmd_fmt.pack_into(self._mm, off + _CMD_HEAD.size, *payload)
        self._end_write(off, seq)
        return self._next_cmd_id

    def command_positions(self, positions: Sequence[float], answers_step: int, thrust=None) -> int:
        return self.write_command(KIND_POSITION, positions, answers_step, thrust)

    def command_torques(self, torques: Sequence[float], answers_step: int, thrust=None) -> int:
        return self.write_command(KIND_TORQUE, torques, answers_step, thrust)
