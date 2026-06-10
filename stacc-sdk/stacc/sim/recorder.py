"""Simulation recording and replay — save, load, and replay simulation runs.

Records joint states, sensor data, and timing at every step.
Replay plays back the recording through MuJoCo viewer.
Export to CSV or JSON for analysis.

Usage:
    from stacc.sim.recorder import SimRecorder, replay_recording

    recorder = SimRecorder(robot)

    @robot.on_update
    def control(dt):
        recorder.record(dt)
        # ... your control logic

    stacc.simulate(robot, duration=10)
    recorder.save("my_recording.json")

    # Later: replay
    replay_recording("my_recording.json")
"""

from __future__ import annotations

import csv
import json
import os
import time
from typing import Any, Dict, List, Optional


class SimFrame:
    """Single frame of recorded simulation data."""

    __slots__ = (
        "time",
        "joint_positions",
        "joint_velocities",
        "joint_torques",
        "pose",
        "sensor_data",
    )

    def __init__(
        self,
        t: float,
        positions: List[float],
        velocities: List[float],
        torques: List[float],
        pose: List[float],
        sensor_data: Optional[Dict[str, Any]] = None,
    ):
        self.time = t
        self.joint_positions = list(positions)
        self.joint_velocities = list(velocities)
        self.joint_torques = list(torques)
        self.pose = list(pose)
        self.sensor_data = sensor_data or {}

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time": self.time,
            "joint_positions": self.joint_positions,
            "joint_velocities": self.joint_velocities,
            "joint_torques": self.joint_torques,
            "pose": self.pose,
            "sensor_data": self.sensor_data,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "SimFrame":
        return cls(
            t=d["time"],
            positions=d["joint_positions"],
            velocities=d["joint_velocities"],
            torques=d["joint_torques"],
            pose=d.get("pose", [0, 0, 0, 0, 0, 0]),
            sensor_data=d.get("sensor_data", {}),
        )

    def __repr__(self) -> str:
        return (
            f"SimFrame(t={self.time:.3f}s, "
            f"joints={len(self.joint_positions)})"
        )


class SimRecording:
    """A complete simulation recording."""

    def __init__(self, robot_name: str = ""):
        self.robot_name = robot_name
        self.frames: List[SimFrame] = []
        self.metadata: Dict[str, Any] = {}

    @property
    def duration(self) -> float:
        if not self.frames:
            return 0.0
        return self.frames[-1].time - self.frames[0].time

    @property
    def num_frames(self) -> int:
        return len(self.frames)

    @property
    def fps(self) -> float:
        if self.num_frames < 2:
            return 0.0
        return self.num_frames / max(self.duration, 1e-10)

    def get_frame(self, index: int) -> Optional[SimFrame]:
        if 0 <= index < len(self.frames):
            return self.frames[index]
        return None

    def get_frame_at_time(self, t: float) -> Optional[SimFrame]:
        if not self.frames:
            return None
        for frame in self.frames:
            if frame.time >= t:
                return frame
        return self.frames[-1]

    def get_joint_trajectory(self, joint_index: int) -> List[float]:
        return [
            f.joint_positions[joint_index]
            for f in self.frames
            if joint_index < len(f.joint_positions)
        ]

    def get_time_series(self) -> List[float]:
        return [f.time for f in self.frames]

    def __repr__(self) -> str:
        return (
            f"SimRecording(robot='{self.robot_name}', "
            f"frames={self.num_frames}, duration={self.duration:.1f}s)"
        )


class SimRecorder:
    """Records simulation data during a run.

    Attach to a robot and call record() in the on_update callback.

    Usage:
        recorder = SimRecorder(robot, record_every=10)

        @robot.on_update
        def control(dt):
            recorder.record(dt)

        stacc.simulate(robot, duration=10)
        recorder.save("recording.json")
    """

    def __init__(
        self,
        robot,
        record_every: int = 1,
        record_sensors: bool = False,
    ):
        if record_every < 1:
            raise ValueError(
                f"record_every must be >= 1, got {record_every}"
            )

        self.robot = robot
        self.record_every = record_every
        self.record_sensors = record_sensors
        self.recording = SimRecording(robot.name)
        self._step_count = 0

    def record(self, dt: float = 0.0):
        """Record current robot state. Call in on_update."""
        self._step_count += 1
        if self._step_count % self.record_every != 0:
            return

        sensor_data = {}
        if self.record_sensors:
            for sensor in self.robot.sensors:
                try:
                    data = sensor.read()
                    clean = {}
                    for k, v in data.items():
                        if isinstance(v, (int, float, str, bool)):
                            clean[k] = v
                        elif isinstance(v, list):
                            clean[k] = v
                    sensor_data[sensor.name] = clean
                except Exception:
                    pass

        frame = SimFrame(
            t=self.robot.time,
            positions=list(self.robot._state["joint_positions"]),
            velocities=list(self.robot._state["joint_velocities"]),
            torques=list(self.robot._state["joint_torques"]),
            pose=list(self.robot._state["pose"]),
            sensor_data=sensor_data,
        )
        self.recording.frames.append(frame)

    def save(self, filepath: str):
        """Save recording to JSON file."""
        data = {
            "robot_name": self.recording.robot_name,
            "num_frames": self.recording.num_frames,
            "duration": self.recording.duration,
            "fps": round(self.recording.fps, 1),
            "metadata": self.recording.metadata,
            "frames": [f.to_dict() for f in self.recording.frames],
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)
        print(f"  [recorder] Saved {self.recording.num_frames} frames to {filepath}")

    def save_csv(self, filepath: str):
        """Save joint positions to CSV file."""
        if not self.recording.frames:
            return

        n_joints = len(self.recording.frames[0].joint_positions)
        headers = (
            ["time"]
            + [f"q{i}" for i in range(n_joints)]
            + [f"v{i}" for i in range(n_joints)]
        )

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for frame in self.recording.frames:
                row = [round(frame.time, 4)]
                row += [round(p, 6) for p in frame.joint_positions]
                row += [round(v, 6) for v in frame.joint_velocities]
                writer.writerow(row)
        print(f"  [recorder] Saved CSV to {filepath}")

    def reset(self):
        self.recording = SimRecording(self.robot.name)
        self._step_count = 0

    def __repr__(self) -> str:
        return (
            f"SimRecorder(robot='{self.robot.name}', "
            f"frames={self.recording.num_frames}, "
            f"record_every={self.record_every})"
        )


def load_recording(filepath: str) -> SimRecording:
    """Load a recording from JSON file."""
    with open(filepath) as f:
        data = json.load(f)

    recording = SimRecording(data.get("robot_name", ""))
    recording.metadata = data.get("metadata", {})
    for fd in data.get("frames", []):
        recording.frames.append(SimFrame.from_dict(fd))

    print(
        f"  [recorder] Loaded {recording.num_frames} frames, {recording.duration:.1f}s"
    )
    return recording


def replay_recording(filepath: str, speed: float = 1.0):
    """Replay a recorded simulation (prints states — viewer replay requires MuJoCo).

    Args:
        filepath: Path to recording JSON file
        speed: Playback speed multiplier (2.0 = double speed)
    """
    recording = load_recording(filepath)

    if recording.num_frames == 0:
        print("  [replay] Empty recording")
        return

    print(f"  [replay] Playing {recording.num_frames} frames at {speed}x speed")

    start_wall = time.time()
    start_sim = recording.frames[0].time

    for i, frame in enumerate(recording.frames):
        sim_elapsed = (frame.time - start_sim) / speed
        wall_elapsed = time.time() - start_wall

        if sim_elapsed > wall_elapsed:
            time.sleep(sim_elapsed - wall_elapsed)

        if i % max(1, recording.num_frames // 20) == 0:
            pos_str = ", ".join(f"{p:.3f}" for p in frame.joint_positions[:4])
            print(f"  [replay] t={frame.time:.2f}s  joints=[{pos_str}...]")

    print(f"  [replay] Done ({recording.duration:.1f}s simulated)")
