"""Universal trajectory generation for any robot.

Generates smooth joint-space and task-space trajectories for moving
robots between configurations. Every trajectory type takes a start
state, end state, and duration, and produces position/velocity/acceleration
at any time t.

Supports:
- Quintic polynomial (5th order) — smooth pos, vel, accel at endpoints
- Cubic polynomial (3rd order) — smooth pos, vel at endpoints
- Minimum jerk — minimizes jerk for natural motion
- Trapezoidal velocity profile — constant accel/decel with cruise
- Linear interpolation — simplest, for non-critical moves
- Multi-waypoint paths — chain segments through a sequence of configurations

Usage:
    from stacc.kinematics.trajectory import QuinticTrajectory, WaypointPath, TrapezoidalProfile

    # Single segment: move joint from 0 to 1.5 rad in 2 seconds
    traj = QuinticTrajectory(q_start=0.0, q_end=1.5, duration=2.0)
    pos, vel, acc = traj.evaluate(t=1.0)

    # Multi-joint: move 6 joints simultaneously
    traj = QuinticTrajectory(
        q_start=[0, 0, 0, 0, 0, 0],
        q_end=[1.0, -0.5, 0.8, -1.2, 0.3, 0.7],
        duration=3.0
    )
    positions, velocities, accelerations = traj.evaluate(t=1.5)

    # Multi-waypoint path through multiple configurations
    path = WaypointPath(
        waypoints=[[0,0,0], [1,0.5,-0.5], [0.5,1.0,-1.0], [0,0,0]],
        durations=[2.0, 1.5, 2.5],
        method="quintic"
    )
    pos, vel, acc = path.evaluate(t=3.0)
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple, Union

State = Union[float, List[float]]


def _to_list(val: State) -> List[float]:
    if isinstance(val, (int, float)):
        return [float(val)]
    return [float(v) for v in val]


def _scalar_or_list(vals: List[float], was_scalar: bool) -> State:
    if was_scalar and len(vals) == 1:
        return vals[0]
    return vals


# ══════════════════════════════════════════
# QUINTIC POLYNOMIAL TRAJECTORY
# ══════════════════════════════════════════


class QuinticTrajectory:
    """5th-order polynomial trajectory.

    Guarantees smooth position, velocity, AND acceleration at both
    start and end points. The gold standard for robot arm motion.

    Boundary conditions:
        q(0) = q_start,  q(T) = q_end
        q'(0) = v_start, q'(T) = v_end
        q''(0) = a_start, q''(T) = a_end

    q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    """

    def __init__(
        self,
        q_start: State,
        q_end: State,
        duration: float,
        v_start: Optional[State] = None,
        v_end: Optional[State] = None,
        a_start: Optional[State] = None,
        a_end: Optional[State] = None,
    ):
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")

        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.T = duration
        self.n = len(self.q_start)

        v_start = _to_list(v_start) if v_start is not None else [0.0] * self.n
        v_end = _to_list(v_end) if v_end is not None else [0.0] * self.n
        a_start = _to_list(a_start) if a_start is not None else [0.0] * self.n
        a_end = _to_list(a_end) if a_end is not None else [0.0] * self.n

        self._coeffs: List[Tuple[float, float, float, float, float, float]] = []
        T = self.T
        T2, T3, T4, T5 = T**2, T**3, T**4, T**5

        for i in range(self.n):
            a0 = self.q_start[i]
            a1 = v_start[i]
            a2 = a_start[i] / 2.0

            if abs(T3) > 1e-15:
                a3 = (
                    10.0 * (self.q_end[i] - self.q_start[i]) / T3
                    - 6.0 * v_start[i] / T2
                    - 4.0 * v_end[i] / T2
                    - 1.5 * a_start[i] / T
                    + 0.5 * a_end[i] / T
                )
                a4 = (
                    -15.0 * (self.q_end[i] - self.q_start[i]) / T4
                    + 8.0 * v_start[i] / T3
                    + 7.0 * v_end[i] / T3
                    + 1.5 * a_start[i] / T2
                    - a_end[i] / T2
                )
                a5 = (
                    6.0 * (self.q_end[i] - self.q_start[i]) / T5
                    - 3.0 * (v_start[i] + v_end[i]) / T4
                    - 0.5 * (a_start[i] - a_end[i]) / T3
                )
            else:
                a3 = a4 = a5 = 0.0

            self._coeffs.append((a0, a1, a2, a3, a4, a5))

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        """Evaluate trajectory at time t.

        Args:
            t: Time in seconds (clamped to [0, duration])

        Returns:
            (position, velocity, acceleration) — each is scalar or list
        """
        t = max(0.0, min(t, self.T))

        pos: List[float] = []
        vel: List[float] = []
        acc: List[float] = []
        for a0, a1, a2, a3, a4, a5 in self._coeffs:
            t2, t3, t4, t5 = t**2, t**3, t**4, t**5
            pos.append(a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5)
            vel.append(a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 + 5.0 * a5 * t4)
            acc.append(2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3)

        return (
            _scalar_or_list(pos, self._scalar),
            _scalar_or_list(vel, self._scalar),
            _scalar_or_list(acc, self._scalar),
        )

    @property
    def duration(self) -> float:
        return self.T

    def __repr__(self) -> str:
        return f"QuinticTrajectory(n={self.n}, T={self.T:.2f}s)"


# ══════════════════════════════════════════
# CUBIC POLYNOMIAL TRAJECTORY
# ══════════════════════════════════════════


class CubicTrajectory:
    """3rd-order polynomial trajectory.

    Smooth position and velocity at endpoints.
    Simpler than quintic, but acceleration is discontinuous at boundaries.

    q(t) = a0 + a1*t + a2*t² + a3*t³
    """

    def __init__(
        self,
        q_start: State,
        q_end: State,
        duration: float,
        v_start: Optional[State] = None,
        v_end: Optional[State] = None,
    ):
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")

        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.T = duration
        self.n = len(self.q_start)

        v_start = _to_list(v_start) if v_start is not None else [0.0] * self.n
        v_end = _to_list(v_end) if v_end is not None else [0.0] * self.n

        self._coeffs: List[Tuple[float, float, float, float]] = []
        T = self.T
        T2, T3 = T**2, T**3

        for i in range(self.n):
            a0 = self.q_start[i]
            a1 = v_start[i]

            if abs(T2) > 1e-15:
                a2 = (
                    3.0 * (self.q_end[i] - self.q_start[i]) / T2
                    - 2.0 * v_start[i] / T
                    - v_end[i] / T
                )
                a3 = (
                    -2.0 * (self.q_end[i] - self.q_start[i]) / T3
                    + (v_start[i] + v_end[i]) / T2
                )
            else:
                a2 = a3 = 0.0

            self._coeffs.append((a0, a1, a2, a3))

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        t = max(0.0, min(t, self.T))
        pos: List[float] = []
        vel: List[float] = []
        acc: List[float] = []

        for a0, a1, a2, a3 in self._coeffs:
            t2, t3 = t**2, t**3
            pos.append(a0 + a1 * t + a2 * t2 + a3 * t3)
            vel.append(a1 + 2.0 * a2 * t + 3.0 * a3 * t2)
            acc.append(2.0 * a2 + 6.0 * a3 * t)

        return (
            _scalar_or_list(pos, self._scalar),
            _scalar_or_list(vel, self._scalar),
            _scalar_or_list(acc, self._scalar),
        )

    @property
    def duration(self) -> float:
        return self.T

    def __repr__(self) -> str:
        return f"CubicTrajectory(n={self.n}, T={self.T:.2f}s)"


# ══════════════════════════════════════════
# MINIMUM JERK TRAJECTORY
# ══════════════════════════════════════════


class MinJerkTrajectory:
    """Minimum jerk trajectory — produces the most natural-looking motion.

    Minimizes the integral of jerk² (derivative of acceleration).
    This is what human arm movements approximate.

    q(t) = q0 + (q1-q0) * (10τ³ - 15τ⁴ + 6τ⁵)  where τ = t/T
    """

    def __init__(self, q_start: State, q_end: State, duration: float):
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")

        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.T = duration
        self.n = len(self.q_start)

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        t = max(0.0, min(t, self.T))
        tau = t / self.T if self.T > 1e-12 else 1.0
        tau2, tau3, tau4, tau5 = tau**2, tau**3, tau**4, tau**5

        s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
        sd = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / self.T if self.T > 1e-12 else 0.0
        sdd = (
            (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (self.T**2)
            if self.T > 1e-12
            else 0.0
        )

        pos: List[float] = []
        vel: List[float] = []
        acc: List[float] = []
        for i in range(self.n):
            dq = self.q_end[i] - self.q_start[i]
            pos.append(self.q_start[i] + dq * s)
            vel.append(dq * sd)
            acc.append(dq * sdd)

        return (
            _scalar_or_list(pos, self._scalar),
            _scalar_or_list(vel, self._scalar),
            _scalar_or_list(acc, self._scalar),
        )

    @property
    def duration(self) -> float:
        return self.T

    def __repr__(self) -> str:
        return f"MinJerkTrajectory(n={self.n}, T={self.T:.2f}s)"


# ══════════════════════════════════════════
# TRAPEZOIDAL VELOCITY PROFILE
# ══════════════════════════════════════════


class TrapezoidalProfile:
    """Trapezoidal velocity profile — accelerate, cruise, decelerate.

    The most common industrial robot motion profile.
    Constant acceleration to max velocity, cruise, then decelerate.

    Args:
        q_start: Start position(s)
        q_end: End position(s)
        max_velocity: Maximum velocity (per joint or scalar)
        max_acceleration: Maximum acceleration (per joint or scalar)
    """

    def __init__(
        self,
        q_start: State,
        q_end: State,
        max_velocity: float = 1.0,
        max_acceleration: float = 5.0,
    ):
        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.n = len(self.q_start)

        self._profiles: List[Tuple[float, float, float, float, float]] = []
        max_duration = 0.0

        for i in range(self.n):
            dist = abs(self.q_end[i] - self.q_start[i])
            sign = 1.0 if self.q_end[i] >= self.q_start[i] else -1.0

            if dist < 1e-10:
                self._profiles.append((0.0, 0.0, 0.0, 0.0, sign))
                continue

            v_max = max_velocity
            a_max = max_acceleration

            t_accel = v_max / a_max
            d_accel = 0.5 * a_max * t_accel**2

            if 2.0 * d_accel >= dist:
                t_accel = math.sqrt(dist / a_max)
                t_cruise = 0.0
                t_total = 2.0 * t_accel
                v_peak = a_max * t_accel
            else:
                d_cruise = dist - 2.0 * d_accel
                t_cruise = d_cruise / v_max
                t_total = 2.0 * t_accel + t_cruise
                v_peak = v_max

            self._profiles.append((t_accel, t_cruise, t_total, v_peak, sign))
            max_duration = max(max_duration, t_total)

        self.T = max_duration

        # Synchronize: scale all joints to finish at the same time
        for i in range(self.n):
            t_accel, t_cruise, t_total, v_peak, sign = self._profiles[i]
            if t_total > 1e-10 and self.T > t_total:
                dist = abs(self.q_end[i] - self.q_start[i])
                if dist > 1e-10:
                    t_a = self.T / 3.0
                    v_p = dist / (self.T - t_a)
                    t_a = v_p / max_acceleration
                    t_c = self.T - 2.0 * t_a
                    if t_c < 0:
                        t_a = self.T / 2.0
                        t_c = 0.0
                        v_p = dist / t_a
                    self._profiles[i] = (t_a, t_c, self.T, v_p, sign)

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        t = max(0.0, min(t, self.T))
        pos: List[float] = []
        vel: List[float] = []
        acc: List[float] = []

        for i in range(self.n):
            t_accel, t_cruise, t_total, v_peak, sign = self._profiles[i]

            if t_total < 1e-10:
                pos.append(self.q_start[i])
                vel.append(0.0)
                acc.append(0.0)
                continue

            a = v_peak / t_accel if t_accel > 1e-10 else 0.0

            if t <= t_accel:
                p = self.q_start[i] + sign * 0.5 * a * t**2
                v = sign * a * t
                ac = sign * a
            elif t <= t_accel + t_cruise:
                dt = t - t_accel
                d_accel_ = 0.5 * a * t_accel**2
                p = self.q_start[i] + sign * (d_accel_ + v_peak * dt)
                v = sign * v_peak
                ac = 0.0
            else:
                dt = t - t_accel - t_cruise
                d_accel_ = 0.5 * a * t_accel**2
                d_cruise = v_peak * t_cruise
                p = self.q_start[i] + sign * (
                    d_accel_ + d_cruise + v_peak * dt - 0.5 * a * dt**2
                )
                v = sign * (v_peak - a * dt)
                ac = -sign * a

            pos.append(p)
            vel.append(v)
            acc.append(ac)

        return (
            _scalar_or_list(pos, self._scalar),
            _scalar_or_list(vel, self._scalar),
            _scalar_or_list(acc, self._scalar),
        )

    @property
    def duration(self) -> float:
        return self.T

    def __repr__(self) -> str:
        return f"TrapezoidalProfile(n={self.n}, T={self.T:.2f}s)"


# ══════════════════════════════════════════
# LINEAR INTERPOLATION
# ══════════════════════════════════════════


class LinearTrajectory:
    """Simple linear interpolation — constant velocity, instant acceleration.

    Use only for non-critical moves or as a baseline.
    """

    def __init__(self, q_start: State, q_end: State, duration: float):
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")

        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.T = duration
        self.n = len(self.q_start)

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        t = max(0.0, min(t, self.T))
        s = t / self.T if self.T > 1e-12 else 1.0

        pos: List[float] = []
        vel: List[float] = []
        acc: List[float] = []
        for i in range(self.n):
            dq = self.q_end[i] - self.q_start[i]
            pos.append(self.q_start[i] + dq * s)
            vel.append(dq / self.T if self.T > 1e-12 else 0.0)
            acc.append(0.0)

        return (
            _scalar_or_list(pos, self._scalar),
            _scalar_or_list(vel, self._scalar),
            _scalar_or_list(acc, self._scalar),
        )

    @property
    def duration(self) -> float:
        return self.T

    def __repr__(self) -> str:
        return f"LinearTrajectory(n={self.n}, T={self.T:.2f}s)"


# ══════════════════════════════════════════
# MULTI-WAYPOINT PATH
# ══════════════════════════════════════════


class WaypointPath:
    """Chain multiple trajectory segments through waypoints.

    Takes a list of joint configurations and durations between them,
    and produces a smooth path through all waypoints.

    Args:
        waypoints: List of joint configurations [[q1,...], [q1,...], ...]
        durations: Time for each segment (len = len(waypoints) - 1)
        method: ``"quintic"``, ``"cubic"``, ``"minjerk"``, ``"linear"``

    Usage:
        path = WaypointPath(
            waypoints=[[0,0], [1,0.5], [0.5,1], [0,0]],
            durations=[2.0, 1.5, 2.5],
            method="quintic"
        )
        pos, vel, acc = path.evaluate(t=3.0)
    """

    METHODS = {
        "quintic": QuinticTrajectory,
        "cubic": CubicTrajectory,
        "minjerk": MinJerkTrajectory,
        "linear": LinearTrajectory,
    }

    def __init__(
        self,
        waypoints: List[List[float]],
        durations: Optional[List[float]] = None,
        method: str = "quintic",
    ):
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")

        self.waypoints = waypoints
        self.n_segments = len(waypoints) - 1

        if durations is None:
            durations = [2.0] * self.n_segments
        if len(durations) != self.n_segments:
            raise ValueError(
                f"Need {self.n_segments} durations, got {len(durations)}"
            )

        for d in durations:
            if d <= 0:
                raise ValueError(f"All durations must be > 0, got {d}")

        traj_class = self.METHODS.get(method)
        if traj_class is None:
            raise ValueError(
                f"Unknown method {method!r}. "
                f"Available: {list(self.METHODS)}"
            )

        self.durations = durations
        self.T = sum(durations)

        self._segments: List = []
        self._segment_starts: List[float] = []
        cumulative = 0.0

        for i in range(self.n_segments):
            self._segment_starts.append(cumulative)
            seg = traj_class(
                q_start=waypoints[i],
                q_end=waypoints[i + 1],
                duration=durations[i],
            )
            self._segments.append(seg)
            cumulative += durations[i]

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        """Evaluate the path at time t (clamped to [0, total_duration])."""
        t = max(0.0, min(t, self.T))

        for i in range(self.n_segments):
            seg_end = self._segment_starts[i] + self.durations[i]
            if t <= seg_end or i == self.n_segments - 1:
                local_t = t - self._segment_starts[i]
                return self._segments[i].evaluate(local_t)

        return self._segments[-1].evaluate(self.durations[-1])

    @property
    def duration(self) -> float:
        return self.T

    @property
    def num_segments(self) -> int:
        return self.n_segments

    def __repr__(self) -> str:
        return (
            f"WaypointPath(segments={self.n_segments}, "
            f"T={self.T:.2f}s)"
        )


# ══════════════════════════════════════════
# CONVENIENCE: Trajectory executor for robot control loops
# ══════════════════════════════════════════


class TrajectoryExecutor:
    """Executes a trajectory on a robot inside the control loop.

    Wraps any trajectory/path and provides a simple interface for
    the robot's on_update callback.

    Usage:
        traj = QuinticTrajectory([0,0,0], [1,0.5,-1], duration=3.0)
        executor = TrajectoryExecutor(robot, traj, pids)

        @robot.on_update
        def control(dt):
            executor.step(dt)
    """

    def __init__(
        self,
        robot,
        trajectory,
        pid_controllers: List,
        start_time: float = 0.0,
    ):
        self.robot = robot
        self.trajectory = trajectory
        self.pids = pid_controllers
        self.start_time = start_time
        self._started = False
        self.done = False

    def step(self, dt: float) -> None:
        """Call this from the robot's control loop callback.

        Args:
            dt: Time step in seconds
        """
        if not self._started:
            self.start_time = self.robot.time
            self._started = True

        elapsed = self.robot.time - self.start_time

        if elapsed >= self.trajectory.duration:
            self.done = True
            elapsed = self.trajectory.duration

        target_pos, target_vel, _ = self.trajectory.evaluate(elapsed)
        current_pos = self.robot.get_joint_positions()

        if isinstance(target_pos, (int, float)):
            target_pos = [target_pos]
        if isinstance(target_vel, (int, float)):
            target_vel = [target_vel]

        torques = []
        for i in range(min(len(self.pids), len(current_pos))):
            error = target_pos[i] - current_pos[i] if i < len(target_pos) else 0.0
            torque = self.pids[i].compute(error, dt)
            torques.append(torque)

        self.robot.set_joint_torques(torques)

    def __repr__(self) -> str:
        return (
            f"TrajectoryExecutor(done={self.done}, "
            f"trajectory={type(self.trajectory).__name__})"
        )
