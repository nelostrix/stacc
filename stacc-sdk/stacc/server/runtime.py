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

from stacc.server.rt_channel import (
    KIND_POSITION,
    KIND_TORQUE,
    MODE_LOCKSTEP,
    VERSION as RT_CHANNEL_VERSION,
    ControlChannel,
)
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


_MAX_INLINE_ARRAY = 64


def _compact(value: Any) -> Any:
    """Large arrays (camera frames, depth maps, dense scans) are summarized as
    their shape instead of being expanded element by element. Expanding a
    single camera frame takes ~1 s under the runtime lock, which starves the
    physics loop and stalls every other request while telemetry is polled."""
    size = getattr(value, "size", None)
    if hasattr(value, "shape") and isinstance(size, int) and size > _MAX_INLINE_ARRAY:
        return {"shape": list(value.shape), "dtype": str(getattr(value, "dtype", ""))}
    if isinstance(value, (list, tuple)) and len(value) > _MAX_INLINE_ARRAY:
        return {"length": len(value)}
    if isinstance(value, dict):
        return {k: _compact(v) for k, v in value.items()}
    return value


def _sensor_readout(robot: Any) -> Dict[str, Any]:
    """The latest sample of every sensor, as taken by the physics loop at the
    sensor's own rate, in a JSON-safe dict (large arrays summarized).

    This never reads or resets a sensor itself: reading here re-rendered
    cameras inside the runtime lock on every telemetry poll (stalling the
    physics loop and robot loads), and reset() restarted each sensor's
    sampling clock."""
    out: Dict[str, Any] = {}
    for sensor in getattr(robot, "sensors", []):
        data = getattr(sensor, "_last_reading", None)
        if data is None:
            continue
        if isinstance(data, dict):
            cleaned = {}
            for k, v in data.items():
                if isinstance(v, dict) and "image" in v:
                    v = {key: val for key, val in v.items() if key != "image"}
                if isinstance(v, list) and v and hasattr(v[0], "shape"):
                    continue
                cleaned[k] = _jsonable(_compact(v))
            out[sensor.name] = cleaned
        else:
            out[sensor.name] = _jsonable(_compact(data))
    return out


def _sensor_images(robot: Any) -> Dict[str, Any]:
    """Return the latest sampled image without crossing sensor/GL threads."""
    images: Dict[str, Any] = {}
    for sensor in getattr(robot, "sensors", []):
        data = getattr(sensor, "_last_reading", None)
        if isinstance(data, dict):
            img = data.get("image", data.get("rgb"))
            if img is None and data.get("left") is not None:
                img = data.get("left")
                if isinstance(img, dict):
                    img = img.get("image")
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
        command_timeout: Optional[float] = None,
    ):
        if not math.isfinite(float(hz)) or float(hz) <= 0 or float(hz) > 10_000:
            raise ValueError("hz must be finite and in (0, 10000]")
        if not math.isfinite(float(timestep)) or float(timestep) <= 0:
            raise ValueError("timestep must be finite and positive")
        if command_timeout is not None and (not math.isfinite(float(command_timeout)) or float(command_timeout) <= 0):
            raise ValueError("command_timeout must be finite and positive, or None")
        self._lock = threading.RLock()
        self.robot = robot
        self.hz = float(hz)
        self.command_timeout = float(command_timeout) if command_timeout is not None else None
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
        self._step_requested = threading.Event()
        self._step_completed = threading.Event()
        # Shared-memory control channel (stacc/server/rt_channel.py), off until enabled.
        self._channel = None
        self._channel_cfg: Optional[Dict[str, Any]] = None
        self._channel_stats = {"cycles": 0, "commands": 0, "missed_deadlines": 0, "late_commands": 0, "invalid_commands": 0}
        self._channel_last_cmd = 0
        self._channel_published_step = -1
        self._running = False
        self._paused = False
        self._emergency_stopped = False
        self._emergency_stop_reason: Optional[str] = None
        self._fault: Optional[str] = None
        self._last_command_at: Optional[float] = None
        self._watchdog_trips = 0
        self._deadline_misses = 0
        self._sensor_errors: Dict[str, str] = {}

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
            if self.engine is not None and self.engine._physics_backend is not None:
                self.engine._physics_backend.close()
                self.engine._physics_backend = None
            self.robot = robot
            self.engine = SimulationEngine(robot, self.config) if robot is not None else None
            self._cmd_positions = None
            self._fault = None
            self._last_command_at = None
            # Bumped on every load, even of a same-named robot, so viewers know
            # to re-fetch the structure after an edit.
            self.robot_revision = getattr(self, "robot_revision", 0) + 1
            # A new robot means new joint/rotor counts: rebuild the channel layout.
            if self._channel_cfg is not None:
                self._open_channel()
        if was_running and robot is not None:
            self.start()

    def start(self):
        """Start the live background loop (calls robot._startup once)."""
        with self._lock:
            if self.robot is None:
                raise RuntimeError("No robot set — call set_robot() or pass one to the constructor")
            if self._emergency_stopped:
                raise RuntimeError("Emergency stop is latched; clear it explicitly before starting")
            if self._fault is not None:
                raise RuntimeError(f"Runtime is faulted: {self._fault}; reset or load a robot before restarting")
            if self._running:
                return
            if self.engine._physics_backend is not None:
                self.engine._physics_backend.close()
                self.engine._physics_backend = None
            self.engine.reset()
            self.engine._init_physics()
            try:
                # Commands written while stopped are stale. Controllers can
                # publish a fresh command after observing the new run state.
                self._discard_pending_channel_command()
                self.robot._startup()
                self._running = True
                self._paused = False
                self._stop_event = threading.Event()
                self._thread = threading.Thread(
                    target=self._loop, name="stacc-runtime", daemon=True
                )
                self._thread.start()
            except Exception:
                self._running = False
                if self.engine._physics_backend is not None:
                    self.engine._physics_backend.close()
                    self.engine._physics_backend = None
                raise

    def stop(self, timeout: float = 2.0):
        """Stop the loop and release the physics backend."""
        if not math.isfinite(float(timeout)) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        self._running = False
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=float(timeout))
            if thread.is_alive():
                raise TimeoutError("STACC runtime loop did not stop before timeout; physics backend remains open")
        self._step_requested.clear()
        with self._lock:
            self._cmd_positions = None
            self._last_command_at = None
            self._discard_pending_channel_command()
            if self.engine is not None and self.engine._physics_backend is not None:
                try:
                    if self.robot is not None:
                        self.robot._state.pop("servo", None)
                        self.robot.set_joint_torques([0.0] * len(self.robot._state.get("joint_torques", [])))
                        if getattr(self.robot, "thrusters", None):
                            self.robot._state["thrusts"] = [0.0] * len(self.robot.thrusters)
                    self.robot._shutdown()
                finally:
                    self.engine._physics_backend.close()
            elif self.robot is not None:
                self.robot._state.pop("servo", None)
                self.robot.set_joint_torques([0.0] * len(self.robot._state.get("joint_torques", [])))
                if getattr(self.robot, "thrusters", None):
                    self.robot._state["thrusts"] = [0.0] * len(self.robot.thrusters)
            self._thread = None

    def pause(self):
        """Pause stepping (the loop idles, state stays readable)."""
        with self._lock:
            self._paused = True

    def resume(self):
        """Resume stepping."""
        with self._lock:
            if self._emergency_stopped:
                raise RuntimeError("Emergency stop is latched; clear it explicitly before resuming")
            self._paused = False

    def emergency_stop(self, reason: str = "operator request") -> None:
        """Latch software stop and command zero output.

        This is not a safety-rated drive disable or replacement for a physical
        emergency-stop circuit.
        """
        with self._lock:
            self._emergency_stopped = True
            self._emergency_stop_reason = str(reason)[:256] or "operator request"
            self._paused = True
            self._cmd_positions = None
            self._step_requested.clear()
            self._discard_pending_channel_command()
            if self.robot is not None:
                self.robot._state.pop("servo", None)
                self.robot.set_joint_torques([0.0] * len(self.robot._state.get("joint_torques", [])))
                if getattr(self.robot, "thrusters", None):
                    self.robot._state["thrusts"] = [0.0] * len(self.robot.thrusters)

    def clear_emergency_stop(self) -> None:
        """Explicitly clear the stop latch; actuator commands stay cleared."""
        with self._lock:
            self._emergency_stopped = False
            self._emergency_stop_reason = None
            self._cmd_positions = None
            self._discard_pending_channel_command()
            if self.robot is not None:
                self.robot._state.pop("servo", None)
                self.robot.set_joint_torques([0.0] * len(self.robot._state.get("joint_torques", [])))

    def _trip_watchdog(self) -> None:
        self._watchdog_trips += 1
        self._cmd_positions = None
        if self.robot is not None:
            self.robot._state.pop("servo", None)
            self.robot.set_joint_torques([0.0] * len(self.robot._state.get("joint_torques", [])))
            if getattr(self.robot, "thrusters", None):
                self.robot._state["thrusts"] = [0.0] * len(self.robot.thrusters)
        self._last_command_at = None

    def _discard_pending_channel_command(self) -> None:
        """Prevent a command buffered during stop/E-stop from executing later."""
        if self._channel is not None:
            command = self._channel.read_command()
            if command is not None:
                self._channel_last_cmd = max(self._channel_last_cmd, command.cmd_id)

    def _record_fault(self, exc: BaseException) -> None:
        with self._lock:
            self._fault = f"{type(exc).__name__}: {exc}"[:512]
            self._running = False
            self._paused = True
            self._cmd_positions = None
            try:
                if self.robot is not None:
                    self.robot._state.pop("servo", None)
                    self.robot.set_joint_torques([0.0] * len(self.robot._state.get("joint_torques", [])))
                    if getattr(self.robot, "thrusters", None):
                        self.robot._state["thrusts"] = [0.0] * len(self.robot.thrusters)
            except Exception:
                pass

    def reset(self):
        """Reset physics and state back to the initial configuration."""
        was_running = self._running
        if self._thread is not None:
            self.stop()
        with self._lock:
            self._paused = True
            if self.engine is not None:
                self.engine.reset()
            if self.robot is not None:
                n = len(self.robot._state["joint_positions"])
                self.robot.set_joint_positions([0.0] * n)
                self.robot._state["joint_velocities"] = [0.0] * n
                self.robot.set_joint_torques([0.0] * n)
                self.robot._state["pose"] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.robot._state.pop("base_quat", None)
                self.robot._state.pop("thrusts", None)
            self._cmd_positions = None
            self._fault = None
            self._last_command_at = None
            self._sensor_errors.clear()
            self._paused = self._emergency_stopped
        if was_running and not self._emergency_stopped:
            self.start()

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
        """Hand commanded joint positions to the physics backend's servo
        (JointImpedance). MuJoCo runs it as an actuator, which is stable for any
        gains; other backends get the equivalent PD torques."""
        if self.robot is None:
            return
        if self._cmd_positions is None:
            self.robot._state.pop("servo", None)
            return
        backend = getattr(self.engine, "_physics_backend", None)
        if backend is not None and hasattr(backend, "_actuator_of"):
            self.robot._state["servo"] = {"kp": self.kp, "kd": self.kd, "targets": self._cmd_positions}
            return
        positions = self.robot.get_joint_positions()
        velocities = self.robot.get_joint_velocities()
        torques = [0.0] * len(positions)
        for i in range(len(positions)):
            if i < len(self.robot.joints) and self.robot.joints[i].type == "fixed":
                continue
            err = self._cmd_positions[i] - positions[i]
            torques[i] = self.robot.joints[i].clamp_effort(self.kp * err - self.kd * velocities[i])
        self.robot.set_joint_torques(torques)

    def _substep(self, dt: float):
        self._apply_command()
        self.engine._physics_backend.step(dt)
        for sensor in self.robot.sensors:
            if sensor.enabled and sensor.should_update(self.engine.state.time):
                # Keep the sample: telemetry reports it instead of re-reading.
                try:
                    sensor._last_reading = sensor.read()
                except Exception as exc:
                    self._sensor_errors[sensor.name] = f"{type(exc).__name__}: {exc}"[:256]
        self.robot._tick(dt)
        self.engine.state.time += dt

    def step_once(self):
        """Advance a single physics timestep (usable while paused or stopped)."""
        with self._lock:
            if self.robot is None or self.engine is None:
                return
            if self._running and not self._paused:
                raise RuntimeError("single-step is only available while stopped or paused")
            if self._emergency_stopped:
                raise RuntimeError("cannot single-step while emergency stop is latched")
            if self._running:
                # Keep MuJoCo renderer and physics operations on the runtime
                # thread even while paused; the caller waits for one cycle.
                self._step_completed.clear()
                self._step_requested.set()
            else:
                if self.engine._physics_backend is None:
                    self.engine._init_physics()
                self._substep(self.config.timestep)
                self.engine.state.step += 1
                if self._channel is not None:
                    self._publish_channel_state(self._channel)
                return
        if not self._step_completed.wait(timeout=2.0):
            self._step_requested.clear()
            raise TimeoutError("runtime thread did not complete the paused single-step")

    def _loop(self):
        cycle = 1.0 / max(self.hz, 1e-9)
        backend = self.engine._physics_backend
        last_start: Optional[float] = None
        next_deadline = time.perf_counter()
        step_remainder = 0.0
        try:
            while not self._stop_event.is_set():
                start = time.perf_counter()
                if last_start is not None:
                    with self._period_lock:
                        self._periods.append(start - last_start)
                last_start = start
                channel = self._channel
                active = self._running and not self._paused and backend is not None
                paused_step = self._running and self._paused and backend is not None and self._step_requested.is_set()
                if channel is not None and active and channel.mode == MODE_LOCKSTEP:
                    self._await_lockstep_command(channel)
                with self._lock:
                    if not self._stop_event.is_set() and self._running and not self._paused and backend is not None:
                        if self._channel is not None:
                            self._apply_channel_command(self._channel)
                        if (self.command_timeout is not None and self._last_command_at is not None
                                and time.monotonic() - self._last_command_at >= self.command_timeout):
                            self._trip_watchdog()
                        # Distribute fractional physics steps over control periods,
                        # avoiding systematic time error at rates such as 60 Hz.
                        step_remainder += cycle
                        substeps = 0
                        while step_remainder + 1e-15 >= self.config.timestep:
                            self._substep(self.config.timestep)
                            step_remainder -= self.config.timestep
                            substeps += 1
                        self.engine.state.step += substeps
                        if self._channel is not None:
                            self._publish_channel_state(self._channel)
                    elif not self._stop_event.is_set() and paused_step and self._step_requested.is_set():
                        self._step_requested.clear()
                        self._substep(self.config.timestep)
                        self.engine.state.step += 1
                        if self._channel is not None:
                            self._publish_channel_state(self._channel)
                        self._step_completed.set()
                self.step_time_ms = (time.perf_counter() - start) * 1000.0
                next_deadline += cycle
                now = time.perf_counter()
                if now > next_deadline:
                    self._deadline_misses += 1
                    # Skip missed wall-clock slots instead of an unbounded catch-up burst.
                    missed = int((now - next_deadline) // cycle) + 1
                    next_deadline += missed * cycle
                self._stop_event.wait(max(0.0, next_deadline - time.perf_counter()))
        except BaseException as exc:
            if not self._stop_event.is_set():
                self._record_fault(exc)
        finally:
            self._step_completed.set()
            # MuJoCo GL contexts belong to the thread that created them. Sensor
            # renderers are lazy-created during sampling and must be closed here
            # before the backend is released by stop() on the caller thread.
            for sensor in getattr(self.robot, "sensors", []):
                close_renderer = getattr(sensor, "close_renderer", None)
                if callable(close_renderer):
                    try:
                        close_renderer()
                    except Exception as exc:
                        self._sensor_errors[sensor.name] = f"renderer close: {type(exc).__name__}: {exc}"[:256]

    # ── shared-memory control channel ──

    def enable_channel(self, path: str, mode: int = 0, timeout_s: float = 1.0) -> Dict[str, Any]:
        """Create (or re-create) the channel at `path` and start serving it."""
        if int(mode) not in (0, MODE_LOCKSTEP):
            raise ValueError("mode must be realtime (0) or lockstep (1)")
        if not math.isfinite(float(timeout_s)) or float(timeout_s) <= 0:
            raise ValueError("timeout_s must be finite and positive")
        with self._lock:
            self._channel_cfg = {"path": path, "mode": int(mode), "timeout_s": float(timeout_s)}
            self._channel_stats = {"cycles": 0, "commands": 0, "missed_deadlines": 0, "late_commands": 0, "invalid_commands": 0}
            self._open_channel()
            return self.channel_status()

    def disable_channel(self) -> None:
        with self._lock:
            self._channel_cfg = None
            if self._channel is not None:
                self._channel.close()
                self._channel = None

    def channel_status(self) -> Dict[str, Any]:
        ch = self._channel
        if ch is None:
            return {"enabled": False}
        return {
            "enabled": True,
            "path": ch.path,
            "layout_version": RT_CHANNEL_VERSION,
            "mode": "lockstep" if ch.mode == MODE_LOCKSTEP else "realtime",
            "timeout_s": self._channel_cfg["timeout_s"] if self._channel_cfg else None,
            "command_timeout_s": self.command_timeout,
            "n_joints": ch.n_joints,
            "n_thrusters": ch.n_thrusters,
            "joint_names": self.joint_names(),
            "thrusters": list(getattr(self.robot, "thrusters", []) or []) if self.robot is not None else [],
            "published_step": self._channel_published_step,
            **self._channel_stats,
        }

    def _open_channel(self) -> None:
        """(Re)build the channel for the current robot. Caller holds the lock."""
        if self._channel is not None:
            self._channel.close()
            self._channel = None
        cfg = self._channel_cfg
        if cfg is None or self.robot is None:
            return
        n_joints = len(self.robot._state["joint_positions"])
        n_thrusters = len(getattr(self.robot, "thrusters", []) or [])
        self._channel = ControlChannel.create(cfg["path"], n_joints, n_thrusters, cfg["mode"])
        self._channel_last_cmd = 0
        self._channel_published_step = -1
        if self.engine is not None:
            self._publish_channel_state(self._channel)

    def _await_lockstep_command(self, channel) -> None:
        """Lockstep: block until the controller answers the last published step."""
        want = self._channel_published_step
        timeout = self._channel_cfg["timeout_s"] if self._channel_cfg else 1.0
        deadline = time.perf_counter() + timeout
        while not self._stop_event.is_set():
            cmd = channel.read_command()
            if cmd is not None and cmd.answers_step == want:
                return
            if time.perf_counter() >= deadline:
                self._channel_stats["missed_deadlines"] += 1
                return
            time.sleep(0)  # yield, but stay responsive

    def _apply_channel_command(self, channel) -> None:
        cmd = channel.read_command()
        if cmd is None or cmd.cmd_id == self._channel_last_cmd:
            return
        self._channel_last_cmd = cmd.cmd_id
        self._channel_stats["commands"] += 1
        valid_kind = cmd.kind in (KIND_POSITION, KIND_TORQUE)
        valid_values = len(cmd.values) == len(self.robot.joints) and all(math.isfinite(float(v)) for v in cmd.values)
        valid_thrust = cmd.thrust is None or (len(cmd.thrust) == len(self.robot.thrusters) and all(math.isfinite(float(v)) for v in cmd.thrust))
        if not valid_kind or not valid_values or not valid_thrust:
            self._channel_stats["invalid_commands"] += 1
            return
        if cmd.answers_step < self._channel_published_step:
            self._channel_stats["late_commands"] += 1  # computed from an older state
        if cmd.kind == KIND_POSITION:
            self._cmd_positions = [0.0 if j.type == "fixed" else j.clamp(v) for j, v in zip(self.robot.joints, cmd.values)]
            self._last_command_at = time.monotonic()
        elif cmd.kind == KIND_TORQUE:
            self._cmd_positions = None
            self.robot.set_joint_torques([0.0 if j.type == "fixed" else j.clamp_effort(float(v)) for j, v in zip(self.robot.joints, cmd.values)])
            self._last_command_at = time.monotonic()
        if cmd.thrust is not None:
            self.robot._state["thrusts"] = [max(0.0, min(float(v), self.robot._links_by_name[name].thruster["max_thrust"])) for name, v in zip(self.robot.thrusters, cmd.thrust)]
        self._last_command_at = time.monotonic()

    def _publish_channel_state(self, channel) -> None:
        state = self.robot._state
        pose = state.get("pose", [0.0] * 6)
        quat = state.get("base_quat") or [1.0, 0.0, 0.0, 0.0]
        step = self.engine.state.step
        channel.write_state(
            step,
            self.engine.state.time,
            [pose[0], pose[1], pose[2], *quat],
            state["joint_positions"],
            state["joint_velocities"],
            state["joint_torques"],
            state.get("thrusts") or [0.0] * channel.n_thrusters,
        )
        self._channel_published_step = step
        self._channel_stats["cycles"] += 1

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
            if len(positions) != n:
                raise ValueError(f"expected {n} joint positions, got {len(positions)}")
            values = [float(v) for v in positions]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("joint positions must all be finite")
            if self._emergency_stopped:
                raise RuntimeError("Cannot command joints while emergency stop is latched")
            self._cmd_positions = [0.0 if joint.type == "fixed" else joint.clamp(v) for joint, v in zip(self.robot.joints, values)]
            self._last_command_at = time.monotonic()

    def set_joint_angle(self, joint_id: str, angle_rad: float):
        """Command a single joint to a target angle (radians)."""
        with self._lock:
            if self.robot is None:
                raise RuntimeError("No robot set")
            idx = self.joint_index(joint_id)
            angle_rad = float(angle_rad)
            if not math.isfinite(angle_rad):
                raise ValueError("joint position must be finite")
            if self._emergency_stopped:
                raise RuntimeError("Cannot command joints while emergency stop is latched")
            if self._cmd_positions is None:
                self._cmd_positions = list(self.robot.get_joint_positions())
            joint = self.robot.joints[idx]
            self._cmd_positions[idx] = 0.0 if joint.type == "fixed" else joint.clamp(angle_rad)
            self._last_command_at = time.monotonic()

    def set_joint_torques(self, torques: List[float]):
        """Apply direct joint torques (clears any position command)."""
        with self._lock:
            if self.robot is None:
                raise RuntimeError("No robot set")
            if len(torques) != len(self.robot.joints):
                raise ValueError(f"expected {len(self.robot.joints)} joint torques, got {len(torques)}")
            values = [float(t) for t in torques]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("joint torques must all be finite")
            if self._emergency_stopped:
                raise RuntimeError("Cannot command joints while emergency stop is latched")
            self._cmd_positions = None
            self.robot.set_joint_torques([0.0 if j.type == "fixed" else j.clamp_effort(v) for j, v in zip(self.robot.joints, values)])
            self._last_command_at = time.monotonic()

    def _thruster_readout(self) -> List[Dict[str, Any]]:
        thrusts = self.robot._state.get("thrusts") or []
        return [
            {
                "name": name,
                "thrust": float(thrusts[i]) if i < len(thrusts) else 0.0,
                "max_thrust": self.robot._links_by_name[name].thruster["max_thrust"],
            }
            for i, name in enumerate(getattr(self.robot, "thrusters", []))
        ]

    def set_thrusts(self, thrusts: List[float]):
        """Rotor thrust in newtons, in robot.thrusters order (clamped to each
        rotor's 0..max by the actuator)."""
        with self._lock:
            if self.robot is None:
                raise RuntimeError("No robot set")
            n = len(getattr(self.robot, "thrusters", []))
            if n == 0:
                raise ValueError("This robot has no rotors")
            if len(thrusts) != n:
                raise ValueError(f"expected {n} rotor thrusts, got {len(thrusts)}")
            values = [float(t) for t in thrusts]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("rotor thrusts must all be finite")
            if self._emergency_stopped:
                raise RuntimeError("Cannot command thrusters while emergency stop is latched")
            self.robot._state["thrusts"] = [max(0.0, min(v, self.robot._links_by_name[name].thruster["max_thrust"])) for name, v in zip(self.robot.thrusters, values)]
            self._last_command_at = time.monotonic()

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
                kp = float(kp)
                if not math.isfinite(kp) or kp < 0:
                    raise ValueError("kp must be finite and non-negative")
                self.kp = kp
            if kd is not None:
                kd = float(kd)
                if not math.isfinite(kd) or kd < 0:
                    raise ValueError("kd must be finite and non-negative")
                self.kd = kd
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
                "step_time_ms": round(self.step_time_ms, 4), "deadline_misses": self._deadline_misses,
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
            "deadline_misses": self._deadline_misses,
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
                "robot_revision": getattr(self, "robot_revision", 0),
                "dof": self.robot.dof,
                "hz": self.hz,
                "timestep": self.config.timestep,
                "sim_time": round(state.time, 6),
                "step": state.step,
                "running": self._running,
                "paused": self._paused,
                "safety": {
                    "emergency_stopped": self._emergency_stopped,
                    "emergency_stop_reason": self._emergency_stop_reason,
                    "runtime_fault": self._fault,
                    "command_timeout_s": self.command_timeout,
                    "watchdog_trips": self._watchdog_trips,
                    "sensor_errors": dict(self._sensor_errors),
                },
                "fps": round(self.fps, 1),
                "jitter_ms": loop["jitter_ms"],
                "loop": loop,
                "joints": joints,
                "pose": pose,
                "thrusters": self._thruster_readout(),
                "base_quat": _jsonable(self.robot._state.get("base_quat")),
                "fixed_base": bool(getattr(self.robot, "fixed_base", True)),
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
