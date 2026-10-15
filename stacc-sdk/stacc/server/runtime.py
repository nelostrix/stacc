"""Live simulation runtime — thread-safe adapter between UIs/servers and a real
stacc robot + physics engine.

``SimulationEngine.run()`` blocks for a fixed duration, which doesn't fit live
control. This runtime drives the physics backend itself from a background
thread instead: UI-facing methods (``snapshot``, ``set_joint_positions``,
``step_once``, ...) can be called at any time and are serialized against the
step loop with a lock.

It also provides a joint-position servo (``control_mode="JointImpedance"``) so
a UI can command joint angles through PD gains while the physics engine keeps
the dynamics real. Concrete robot types are created from ``ROBOT_FACTORIES``,
so a UI can pick one by name.

Usage:
    rt = StaccRuntime(physics="mujoco", hz=120.0)
    rt.set_robot(create_robot("arm"))
    rt.start()
    print(rt.snapshot())
    rt.set_joint_angle("J2", 0.5)
    rt.stop()
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any, Callable, Dict, List, Optional

from stacc.sim.engine import SimulationEngine, SimulationConfig


def _jsonable(value: Any) -> Any:
    """Recursively convert a possibly-numpy sensor/state value to JSON-safe python."""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, (int, float)):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    item = getattr(value, "item", None)
    if callable(item):
        try:
            return _jsonable(item())
        except Exception:
            pass
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        try:
            return _jsonable(tolist())
        except Exception:
            return None
    return str(value)


def _sensor_readout(robot: Any) -> Dict[str, Any]:
    """Read all attached sensors into a JSON-safe dict (skipping image arrays)."""
    out: Dict[str, Any] = {}
    for sensor in getattr(robot, "sensors", []):
        try:
            sensor.reset()
            data = sensor.read()
        except Exception:
            continue
        if isinstance(data, dict):
            cleaned = {}
            for k, v in data.items():
                if isinstance(v, dict) and "image" in v:
                    v = {key: val for key, val in v.items() if key != "image"}
                if isinstance(v, list) and v and hasattr(v[0], "shape"):
                    continue
                cleaned[k] = _jsonable(v)
            out[sensor.name] = cleaned
        else:
            out[sensor.name] = _jsonable(data)
    return out


def _sensor_images(robot: Any) -> Dict[str, Any]:
    """Return {sensor_name: numpy image} for image-producing sensors."""
    images: Dict[str, Any] = {}
    for sensor in getattr(robot, "sensors", []):
        try:
            data = sensor.read()
        except Exception:
            continue
        if isinstance(data, dict):
            img = data.get("image")
            if img is None:
                img = data.get("rgb")
            if img is not None and hasattr(img, "shape"):
                images[sensor.name] = img
    return images


# ─────────────────────────────────────────────────────────────────────────────
# Concrete robot factories — used by the UI to pick a topology by name.
# Robot classes import numpy at module level, so they are imported lazily.
# ─────────────────────────────────────────────────────────────────────────────

_DEFAULT_ARM_DH = [
    (0.0, -math.pi / 2, 0.089, 0.0),
    (0.425, 0.0, 0.0, -math.pi / 2),
    (0.392, 0.0, 0.0, 0.0),
    (0.0, -math.pi / 2, 0.109, 0.0),
    (0.0, math.pi / 2, 0.095, 0.0),
    (0.0, 0.0, 0.082, 0.0),
]

# Legacy UI joint ranges (degrees) used by the NELO-Studio bridge for the arm.
_ARM_LIMITS_DEG = [(-180, 180), (-110, 110), (-140, 140), (-180, 180), (-120, 120), (-360, 360)]


def make_arm(name: str = "arm_6dof", **kwargs: Any):
    """6-DOF UR5e-style serial manipulator (the default demo robot)."""
    from stacc.robot import SerialManipulator
    from stacc.kinematics.transforms import DH
    from stacc.core.link import Geometry, Inertia, Link

    dh = kwargs.get("dh_parameters")
    if dh is None:
        dh = [DH(a=a, alpha=alpha, d=d, theta_offset=off) for a, alpha, d, off in _DEFAULT_ARM_DH]
    limits = kwargs.get("joint_limits")
    if limits is None and len(dh) == len(_ARM_LIMITS_DEG):
        limits = [(math.radians(lo), math.radians(hi)) for lo, hi in _ARM_LIMITS_DEG]
    robot = SerialManipulator(name=name, dh_parameters=dh, joint_limits=limits)

    # Realistic arm physics: the bare SerialManipulator chain is a set of
    # point-mass links on frictionless joints, which free-spins under any
    # closed-loop servo effort. Give each link plausible inertia (stiffish
    # against shoulder altitude, light about its own axis), strong passive
    # viscous joint damping (gear-train drive resistance) and a torque headroom
    # wider than the servo ever needs, so PD servo control tracks smoothly
    # instead of saturating into a limit-range flail.
    robot_link_damping = kwargs.get("damping", 25.0)
    max_effort = kwargs.get("max_effort", 800.0)
    max_velocity = kwargs.get("max_velocity", 20.0)
    for joint in robot.joints:
        joint.damping = robot_link_damping
        joint.max_effort = max_effort
        joint.max_velocity = max_velocity
    link_mass = kwargs.get("link_mass", 2.0)
    for i, link in enumerate(robot.links):
        link.mass = link_mass
        link.inertia.ixx = link.inertia.iyy = 0.09
        link.inertia.izz = 0.015
        if link.visual is None:
            seg = 0.30 if i < len(robot.links) - 1 else 0.12
            link.visual = Geometry(shape="box", size=[0.045, 0.05, seg],
                                   color=[0.65, 0.74, 0.82, 1.0])
        # Self-collision-free serial chain: auto-placed links are stacked with
        # zero clearance, so any small rotation jams against a neighbour.
        link.visual.contype = 0
        link.visual.conaffinity = 0

    # Materialise the final DH axis in the physics model. The serial-chain
    # builder emits one hinge per link transition, so the 6th joint must hang
    # off a small terminal flange body instead of being silently dropped.
    flange = Link(
        "flange",
        mass=0.25,
        inertia=Inertia.from_sphere(0.25, 0.03),
        visual=Geometry(shape="sphere", radius=0.035, contype=0, conaffinity=0,
                        color=[0.5, 0.5, 0.5, 1.0]),
    )
    j6 = next(j for j in robot.joints if j.name == "joint_6")
    j6.parent_link = "link_6"
    j6.child_link = "flange"
    robot.add_link(flange)
    robot.connect("link_6", "flange", j6, offset=[0.0, 0.0, 0.12])

    return robot


def make_quadruped(name: str = "quadruped", **kwargs: Any):
    """12-DOF quadruped walker."""
    from stacc.robot import QuadrupedRobot

    return QuadrupedRobot(name=name, **kwargs)


def make_diff_drive(name: str = "diff_drive", **kwargs: Any):
    """Two-wheel differential drive mobile base."""
    from stacc.robot import DifferentialDrive

    return DifferentialDrive(name=name, **kwargs)


def make_humanoid(name: str = "humanoid", **kwargs: Any):
    """20-DOF humanoid biped."""
    from stacc.robot import HumanoidRobot

    return HumanoidRobot(name=name, **kwargs)


def make_drone(name: str = "drone", **kwargs: Any):
    """Quadcopter."""
    from stacc.robot import MultirotorDrone

    return MultirotorDrone(name=name, **kwargs)


ROBOT_FACTORIES: Dict[str, Callable[..., Any]] = {
    "arm": make_arm,
    "quadruped": make_quadruped,
    "diff_drive": make_diff_drive,
    "humanoid": make_humanoid,
    "drone": make_drone,
}


def available_robots() -> List[Dict[str, str]]:
    """Describe every registered robot type for the UI picker."""
    return [{"type": name, "factory": fn.__name__} for name, fn in ROBOT_FACTORIES.items()]


def create_robot(kind: str = "arm", **kwargs: Any):
    """Build a robot instance from the factory registry."""
    kind = kind or "arm"
    if kind not in ROBOT_FACTORIES:
        raise ValueError(
            f"Unknown robot type '{kind}'. Available: {sorted(ROBOT_FACTORIES)}"
        )
    return ROBOT_FACTORIES[kind](**kwargs)


# ─────────────────────────────────────────────────────────────────────────────
# StaccRuntime
# ─────────────────────────────────────────────────────────────────────────────


class StaccRuntime:
    """Owns a robot + physics engine and steps it live from a background thread.

    Args:
        robot: A stacc Robot instance, or None to set one later with set_robot().
        physics: Backend name ("mujoco" or "internal"). Falls back to "internal".
        hz: Telemetry/control loop rate for the background thread.
        timestep: Physics integration timestep in seconds.
        gravity: Gravity vector [x, y, z].
        headless: Run without any display.
        render: Open a MuJoCo passive viewer (requires display; implies not headless).
    """

    def __init__(
        self,
        robot: Any = None,
        physics: str = "mujoco",
        hz: float = 60.0,
        timestep: float = 0.001,
        gravity: Optional[List[float]] = None,
        headless: bool = True,
        render: bool = False,
    ):
        self._lock = threading.RLock()
        self.robot = robot
        self.hz = float(hz)
        self.config = SimulationConfig(
            duration=1e9,  # unbounded — the runtime drives the loop, not run()
            timestep=timestep,
            render=render,
            headless=headless,
            physics_engine=physics,
            gravity=gravity,
            realtime=False,
        )
        self.engine = (
            SimulationEngine(self.robot, self.config) if robot is not None else None
        )
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._running = False
        self._paused = False

        # Joint-position servo gains (JointImpedance mode).
        self.kp = 120.0
        self.kd = 25.0
        self.gravity_comp = True
        self.control_mode = "JointImpedance"
        self._cmd_positions: Optional[List[float]] = None

        # Loop timing. `step_time_ms` is how long one cycle's physics took;
        # `jitter_ms` is the real timing jitter: the standard deviation of the time
        # between consecutive cycle starts. Both are measured, never configured.
        self.step_time_ms: float = 0.0
        self.jitter_ms: float = 0.0
        self.fps: float = 0.0
        self._period_lock = threading.Lock()
        self._periods: "deque[float]" = deque(maxlen=2000)  # seconds between cycle starts

    # ── lifecycle ──

    def set_robot(self, robot: Any):
        """Swap in a new robot, stopping the loop if it is running."""
        was_running = self._running
        if was_running:
            self.stop()
        with self._lock:
            self.robot = robot
            self.engine = SimulationEngine(robot, self.config) if robot is not None else None
            self._cmd_positions = None
        if was_running and robot is not None:
            self.start()

    def start(self):
        """Start the live background loop (calls robot._startup once)."""
        with self._lock:
            if self.robot is None:
                raise RuntimeError("No robot set — call set_robot() or pass one to the constructor")
            if self._running:
                return
            self.engine.reset()
            self.engine._init_physics()
            self.robot._startup()
            self._running = True
            self._paused = False
            self._stop_event = threading.Event()
            self._thread = threading.Thread(
                target=self._loop, name="stacc-runtime", daemon=True
            )
            self._thread.start()

    def stop(self):
        """Stop the loop and release the physics backend."""
        self._running = False
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        with self._lock:
            if self.engine is not None and self.engine._physics_backend is not None:
                try:
                    self.robot._shutdown()
                finally:
                    self.engine._physics_backend.close()
            self._thread = None

    def pause(self):
        """Pause stepping (the loop idles, state stays readable)."""
        with self._lock:
            self._paused = True

    def resume(self):
        """Resume stepping."""
        with self._lock:
            self._paused = False

    def reset(self):
        """Reset physics and state back to the initial configuration."""
        with self._lock:
            self._paused = True
            if self.engine is not None:
                self.engine.reset()
                self.engine._init_physics()
            if self.robot is not None:
                n = len(self.robot._state["joint_positions"])
                self.robot.set_joint_positions([0.0] * n)
                self.robot._state["joint_velocities"] = [0.0] * n
                self.robot.set_joint_torques([0.0] * n)
                self.robot._state["pose"] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self._cmd_positions = None
            self._paused = False

    @property
    def running(self) -> bool:
        return self._running

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def is_simulating(self) -> bool:
        return self._running and not self._paused

    # ── stepping ──

    def _apply_command(self):
        """Convert commanded joint positions into servo torques (JointImpedance)."""
        if self._cmd_positions is None or self.robot is None:
            return
        positions = self.robot.get_joint_positions()
        velocities = self.robot.get_joint_velocities()
        torques = [0.0] * len(positions)
        for i in range(len(positions)):
            err = self._cmd_positions[i] - positions[i]
            torques[i] = self.kp * err - self.kd * velocities[i]
        self.robot.set_joint_torques(torques)

    def _substep(self, dt: float):
        self._apply_command()
        self.engine._physics_backend.step(dt)
        for sensor in self.robot.sensors:
            if sensor.enabled and sensor.should_update(self.engine.state.time):
                sensor.read()
        self.robot._tick(dt)
        self.engine.state.time += dt

    def step_once(self):
        """Advance a single physics timestep (usable while paused or stopped)."""
        with self._lock:
            if self.robot is None or self.engine is None:
                return
            if self.engine._physics_backend is None:
                self.engine._init_physics()
            self._substep(self.config.timestep)
            self.engine.state.step += 1

    def _loop(self):
        cycle = 1.0 / max(self.hz, 1e-9)
        substeps = max(1, int(round(1.0 / (max(self.hz, 1e-9) * self.config.timestep))))
        backend = self.engine._physics_backend
        last_start: Optional[float] = None
        while not self._stop_event.is_set():
            start = time.perf_counter()
            if last_start is not None:
                with self._period_lock:
                    self._periods.append(start - last_start)
            last_start = start
            with self._lock:
                if not self._stop_event.is_set() and self._running and not self._paused and backend is not None:
                    for _ in range(substeps):
                        self._substep(self.config.timestep)
                    self.engine.state.step += substeps
            elapsed = time.perf_counter() - start
            self.step_time_ms = elapsed * 1000.0
            time.sleep(max(0.0, cycle - elapsed))

    # ── joint command API ──

    def joint_names(self) -> List[str]:
        """Ordered joint names from the robot."""
        if self.robot is None:
            return []
        return [j.name for j in self.robot.joints]

    def joint_index(self, joint_id: str) -> int:
        """Resolve a joint id ("J2", "joint_2", "2" or the raw name) to an index."""
        names = self.joint_names()
        if joint_id.isdigit() or (joint_id[:1] in "Jj" and joint_id[1:].isdigit()):
            idx = int(joint_id.lstrip("Jj")) - 1
            if 0 <= idx < len(names):
                return idx
        if joint_id in names:
            return names.index(joint_id)
        raise ValueError(f"Unknown joint '{joint_id}'. Have: {names}")

    def set_joint_positions(self, positions: List[float]):
        """Command all joints to target positions (radians) via the servo."""
        with self._lock:
            if self.robot is None:
                raise RuntimeError("No robot set")
            n = len(self.robot.get_joint_positions())
            cmd = [0.0] * n
            for i in range(min(n, len(positions))):
                cmd[i] = self.robot.joints[i].clamp(positions[i])
            self._cmd_positions = cmd

    def set_joint_angle(self, joint_id: str, angle_rad: float):
        """Command a single joint to a target angle (radians)."""
        with self._lock:
            if self.robot is None:
                raise RuntimeError("No robot set")
            idx = self.joint_index(joint_id)
            if self._cmd_positions is None:
                self._cmd_positions = list(self.robot.get_joint_positions())
            self._cmd_positions[idx] = self.robot.joints[idx].clamp(angle_rad)

    def set_joint_torques(self, torques: List[float]):
        """Apply direct joint torques (clears any position command)."""
        with self._lock:
            if self.robot is None:
                raise RuntimeError("No robot set")
            self._cmd_positions = None
            self.robot.set_joint_torques([float(t) for t in torques])

    def clear_commands(self):
        """Drop servo commands and stop applying torques."""
        with self._lock:
            self._cmd_positions = None
            if self.robot is not None:
                self.robot.set_joint_torques([0.0] * len(self.robot._state["joint_torques"]))

    def set_gains(self, kp: Optional[float] = None, kd: Optional[float] = None,
                  gravity_comp: Optional[bool] = None):
        """Update servo gains."""
        with self._lock:
            if kp is not None:
                self.kp = float(kp)
            if kd is not None:
                self.kd = float(kd)
            if gravity_comp is not None:
                self.gravity_comp = bool(gravity_comp)

    # ── readout ──

    def joint_info(self) -> List[Dict[str, Any]]:
        """Joint metadata (types, limits, effort) for the UI."""
        if self.robot is None:
            return []
        out = []
        for i, joint in enumerate(self.robot.joints):
            lo, hi = joint.limits
            out.append({
                "id": f"J{i + 1}",
                "name": joint.name,
                "type": joint.type,
                "index": i,
                "min": _jsonable(lo),
                "max": _jsonable(hi),
                "max_velocity": joint.max_velocity,
                "max_effort": joint.max_effort,
            })
        return out

    def robot_info(self) -> Dict[str, Any]:
        """Summary of the loaded robot."""
        if self.robot is None:
            return {"name": None, "dof": 0, "num_links": 0, "num_joints": 0, "sensors": []}
        info = self.robot.to_dict() if hasattr(self.robot, "to_dict") else {}
        info["sensors"] = [
            {"name": s.name, "type": s.__class__.__name__, "rate": getattr(s, "rate", 0.0)}
            for s in self.robot.sensors
        ]
        info["joints"] = self.joint_info()
        info["control"] = {
            "mode": self.control_mode,
            "kp": self.kp,
            "kd": self.kd,
            "gravity_comp": self.gravity_comp,
        }
        return info

    def end_effector_pose(self) -> Optional[Dict[str, float]]:
        """FK of the end effector as {x,y,z,roll,pitch,yaw} (radians), if the robot has DH."""
        if self.robot is None:
            return None
        q = list(self.robot.get_joint_positions())
        dh_params = getattr(self.robot, "dh_parameters", None)
        if not dh_params:
            return None
        try:
            from stacc.kinematics.forward import end_effector_position, end_effector_rotation
            from stacc.kinematics.transforms import rotation_to_euler

            pos = [float(v) for v in end_effector_position(dh_params, q)]
            roll, pitch, yaw = (float(v) for v in rotation_to_euler(end_effector_rotation(dh_params, q)))
            return {"x": pos[0], "y": pos[1], "z": pos[2], "roll": roll, "pitch": pitch, "yaw": yaw}
        except Exception:
            return None

    def loop_stats(self) -> Dict[str, Any]:
        """Measured timing of the background loop over its last ~2000 cycles.

        Returns zeros until at least two cycles have run. The target period is
        1/hz; `measured_hz` is what the loop actually achieves (time.sleep
        granularity means this is usually a little under the target).
        """
        with self._period_lock:
            periods = list(self._periods)
        target_ms = 1000.0 / max(self.hz, 1e-9)
        if len(periods) < 2:
            return {
                "target_hz": self.hz, "measured_hz": 0.0, "period_mean_ms": 0.0,
                "jitter_ms": 0.0, "period_p99_ms": 0.0, "period_max_ms": 0.0,
                "step_time_ms": round(self.step_time_ms, 4), "deadline_misses": 0,
                "samples": len(periods),
            }
        ms = sorted(p * 1000.0 for p in periods)
        mean = sum(ms) / len(ms)
        var = sum((v - mean) ** 2 for v in ms) / len(ms)
        return {
            "target_hz": self.hz,
            "measured_hz": round(1000.0 / mean, 1) if mean > 0 else 0.0,
            "period_mean_ms": round(mean, 4),
            "jitter_ms": round(var ** 0.5, 4),
            "period_p99_ms": round(ms[min(len(ms) - 1, int(len(ms) * 0.99))], 4),
            "period_max_ms": round(ms[-1], 4),
            "step_time_ms": round(self.step_time_ms, 4),
            # a cycle that took more than 1.5x its budget counts as a missed deadline
            "deadline_misses": sum(1 for v in ms if v > target_ms * 1.5),
            "samples": len(ms),
        }

    def snapshot(self) -> Dict[str, Any]:
        """Full current state in SI units — the payload the UI polls/streams."""
        with self._lock:
            if self.robot is None or self.engine is None:
                return {"robot_name": None, "sim_time": 0.0, "running": False, "paused": False}
            state = self.engine.state
            loop = self.loop_stats()
            joints = []
            positions = self.robot.get_joint_positions()
            velocities = self.robot.get_joint_velocities()
            torques = self.robot._state["joint_torques"]
            names = self.joint_names()
            for i, name in enumerate(names):
                joints.append({
                    "id": f"J{i + 1}",
                    "name": name,
                    "position": _jsonable(positions[i]),
                    "velocity": _jsonable(velocities[i]),
                    "torque": _jsonable(torques[i]),
                    "command_position": _jsonable(
                        self._cmd_positions[i] if self._cmd_positions is not None else None
                    ),
                })
            try:
                pose = _jsonable(self.robot._state.get("pose", [0, 0, 0, 0, 0, 0]))
            except Exception:
                pose = [0, 0, 0, 0, 0, 0]
            return {
                "robot_name": self.robot.name,
                "dof": self.robot.dof,
                "hz": self.hz,
                "timestep": self.config.timestep,
                "sim_time": round(state.time, 6),
                "step": state.step,
                "running": self._running,
                "paused": self._paused,
                "fps": round(self.fps, 1),
                "jitter_ms": loop["jitter_ms"],
                "loop": loop,
                "joints": joints,
                "pose": pose,
                "end_effector": self.end_effector_pose(),
                "sensors": _sensor_readout(self.robot),
                "control": {
                    "mode": self.control_mode,
                    "kp": self.kp,
                    "kd": self.kd,
                    "gravity_comp": self.gravity_comp,
                },
                "command_positions": _jsonable(self._cmd_positions),
            }

    def camera_jpeg(self, sensor_name: str, quality: int = 85) -> bytes:
        """Encode one camera sensor's latest frame as JPEG bytes.

        Requires opencv-python (fast) or Pillow (fallback). Raises RuntimeError
        if the sensor is not an image camera or no encoder is installed.
        """
        images = _sensor_images(self.robot)
        if sensor_name not in images:
            raise RuntimeError(f"No image camera named '{sensor_name}'. Have: {sorted(images)}")
        img = images[sensor_name]
        try:
            import cv2
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                raise RuntimeError("cv2 JPEG encode failed")
            return buf.tobytes()
        except ImportError:
            from PIL import Image
            rgb = img[..., :3]
            pil = Image.fromarray(rgb)
            import io
            buf = io.BytesIO()
            pil.save(buf, format="JPEG", quality=quality)
            return buf.getvalue()

    def __repr__(self) -> str:
        name = self.robot.name if self.robot is not None else None
        return (
            f"StaccRuntime(robot='{name}', engine='{self.config.physics_engine}', "
            f"hz={self.hz}, running={self._running}, paused={self._paused})"
        )