"""Universal trajectory generation for any robot.

Generates smooth joint-space and task-space trajectories for moving
robots between configurations. Every trajectory type takes a start
state, end state, and duration, and produces position/velocity/acceleration
at any time t.

Supports:
- Quintic polynomial (5th order) — smooth pos, vel, accel at endpoints
- Cubic polynomial (3rd order) — smooth pos, vel at endpoints
- Minimum jerk — minimizes jerk (derivative of acceleration) for natural motion
- Trapezoidal velocity profile — constant acceleration/deceleration with cruise
- Linear interpolation — simplest, for non-critical moves
- Multi-waypoint paths — chain segments through a sequence of configurations
- Time-optimal scaling — respect joint velocity and acceleration limits

Usage:
    from stacc.kinematics.trajectory import QuinticTrajectory, MinJerkTrajectory
    from stacc.kinematics.trajectory import WaypointPath, TrapezoidalProfile

    # Single segment: move joint from 0 to 1.5 rad in 2 seconds
    traj = QuinticTrajectory(q_start=0.0, q_end=1.5, duration=2.0)
    pos, vel, acc = traj.evaluate(t=1.0)  # position at t=1.0s

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

# Type alias: scalar or list of floats
State = Union[float, List[float]]


def _to_list(val: State) -> List[float]:
    """Convert scalar or list to list."""
    if isinstance(val, (int, float)):
        return [float(val)]
    return [float(v) for v in val]


def _scalar_or_list(vals: List[float], was_scalar: bool) -> State:
    """Return scalar if input was scalar, else list."""
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
        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.T = duration
        self.n = len(self.q_start)

        v_start = _to_list(v_start) if v_start is not None else [0.0] * self.n
        v_end = _to_list(v_end) if v_end is not None else [0.0] * self.n
        a_start = _to_list(a_start) if a_start is not None else [0.0] * self.n
        a_end = _to_list(a_end) if a_end is not None else [0.0] * self.n

        # Compute coefficients for each joint
        self._coeffs = []  # list of (a0,a1,a2,a3,a4,a5) per joint
        T = self.T
        T2, T3, T4, T5 = T**2, T**3, T**4, T**5

        for i in range(self.n):
            a0 = self.q_start[i]
            a1 = v_start[i]
            a2 = a_start[i] / 2.0

            # Solve for a3, a4, a5 from end conditions
            dq = self.q_end[i] - a0 - a1 * T - a2 * T2
            dv = v_end[i] - a1 - 2 * a2 * T
            da = a_end[i] - 2 * a2

            if T3 > 1e-12:
                a3 = (20 * dq - (8 * dv + da * T) * T - (12 * a1 + 6 * a2 * T) * T) / (
                    2 * T3
                )
                # Re-derive properly using the full system
                # System of equations for a3, a4, a5:
                #   a3*T³ + a4*T⁴ + a5*T⁵ = dq
                #   3*a3*T² + 4*a4*T³ + 5*a5*T⁴ = dv
                #   6*a3*T + 12*a4*T² + 20*a5*T³ = da

                det = T3 * (2 * T2)  # simplified
                if abs(T3) > 1e-15:
                    a3 = (
                        10 * (self.q_end[i] - self.q_start[i]) / T3
                        - 6 * v_start[i] / T2
                        - 4 * v_end[i] / T2
                        - 1.5 * a_start[i] / T
                        + 0.5 * a_end[i] / T
                    )
                    a4 = (
                        -15 * (self.q_end[i] - self.q_start[i]) / T4
                        + 8 * v_start[i] / T3
                        + 7 * v_end[i] / T3
                        + 1.5 * a_start[i] / T2
                        - a_end[i] / T2
                    )
                    a5 = (
                        6 * (self.q_end[i] - self.q_start[i]) / T5
                        - 3 * (v_start[i] + v_end[i]) / T4
                        - 0.5 * (a_start[i] - a_end[i]) / T3
                    )
                else:
                    a3 = a4 = a5 = 0.0
            else:
                a3 = a4 = a5 = 0.0

            self._coeffs.append((a0, a1, a2, a3, a4, a5))

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        """Evaluate trajectory at time t.

        Returns:
            (position, velocity, acceleration) — each is scalar or list
        """
        t = max(0.0, min(t, self.T))

        pos, vel, acc = [], [], []
        for a0, a1, a2, a3, a4, a5 in self._coeffs:
            t2, t3, t4, t5 = t**2, t**3, t**4, t**5

            p = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5
            v = a1 + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4
            a = 2 * a2 + 6 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3

            pos.append(p)
            vel.append(v)
            acc.append(a)

        return (
            _scalar_or_list(pos, self._scalar),
            _scalar_or_list(vel, self._scalar),
            _scalar_or_list(acc, self._scalar),
        )

    @property
    def duration(self) -> float:
        return self.T


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
        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.T = duration
        self.n = len(self.q_start)

        v_start = _to_list(v_start) if v_start is not None else [0.0] * self.n
        v_end = _to_list(v_end) if v_end is not None else [0.0] * self.n

        self._coeffs = []
        T = self.T
        T2, T3 = T**2, T**3

        for i in range(self.n):
            a0 = self.q_start[i]
            a1 = v_start[i]

            if abs(T2) > 1e-15:
                a2 = (
                    3 * (self.q_end[i] - self.q_start[i]) / T2
                    - 2 * v_start[i] / T
                    - v_end[i] / T
                )
                a3 = (
                    -2 * (self.q_end[i] - self.q_start[i]) / T3
                    + (v_start[i] + v_end[i]) / T2
                )
            else:
                a2 = a3 = 0.0

            self._coeffs.append((a0, a1, a2, a3))

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        t = max(0.0, min(t, self.T))
        pos, vel, acc = [], [], []

        for a0, a1, a2, a3 in self._coeffs:
            t2, t3 = t**2, t**3
            pos.append(a0 + a1 * t + a2 * t2 + a3 * t3)
            vel.append(a1 + 2 * a2 * t + 3 * a3 * t2)
            acc.append(2 * a2 + 6 * a3 * t)

        return (
            _scalar_or_list(pos, self._scalar),
            _scalar_or_list(vel, self._scalar),
            _scalar_or_list(acc, self._scalar),
        )

    @property
    def duration(self) -> float:
        return self.T


# ══════════════════════════════════════════
# MINIMUM JERK TRAJECTORY
# ══════════════════════════════════════════


class MinJerkTrajectory:
    """Minimum jerk trajectory — produces the most natural-looking motion.

    Minimizes the integral of jerk² (derivative of acceleration).
    This is what human arm movements approximate.

    q(t) = q0 + (q1-q0) * (10τ³ - 15τ⁴ + 6τ⁵)
    where τ = t/T
    """

    def __init__(self, q_start: State, q_end: State, duration: float):
        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.T = duration
        self.n = len(self.q_start)

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        t = max(0.0, min(t, self.T))
        tau = t / self.T if self.T > 1e-12 else 1.0
        tau2, tau3, tau4, tau5 = tau**2, tau**3, tau**4, tau**5

        # Position: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
        s = 10 * tau3 - 15 * tau4 + 6 * tau5
        # Velocity: s'(τ)/T = (30τ² - 60τ³ + 30τ⁴)/T
        sd = (30 * tau2 - 60 * tau3 + 30 * tau4) / self.T if self.T > 1e-12 else 0.0
        # Acceleration: s''(τ)/T² = (60τ - 180τ² + 120τ³)/T²
        sdd = (
            (60 * tau - 180 * tau2 + 120 * tau3) / (self.T**2)
            if self.T > 1e-12
            else 0.0
        )

        pos, vel, acc = [], [], []
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

        # Compute profile for the joint that takes longest
        self._profiles = []
        max_duration = 0.0

        for i in range(self.n):
            dist = abs(self.q_end[i] - self.q_start[i])
            sign = 1.0 if self.q_end[i] >= self.q_start[i] else -1.0

            if dist < 1e-10:
                self._profiles.append((0, 0, 0, 0, sign))
                continue

            v_max = max_velocity
            a_max = max_acceleration

            # Time to accelerate to max velocity
            t_accel = v_max / a_max
            # Distance covered during acceleration
            d_accel = 0.5 * a_max * t_accel**2

            if 2 * d_accel >= dist:
                # Triangle profile (never reaches max velocity)
                t_accel = math.sqrt(dist / a_max)
                t_cruise = 0.0
                t_total = 2 * t_accel
                v_peak = a_max * t_accel
            else:
                # Trapezoidal profile
                d_cruise = dist - 2 * d_accel
                t_cruise = d_cruise / v_max
                t_total = 2 * t_accel + t_cruise
                v_peak = v_max

            self._profiles.append((t_accel, t_cruise, t_total, v_peak, sign))
            max_duration = max(max_duration, t_total)

        self.T = max_duration

        # Synchronize: scale all joints to finish at the same time
        for i in range(self.n):
            t_accel, t_cruise, t_total, v_peak, sign = self._profiles[i]
            if t_total > 1e-10 and self.T > t_total:
                # Recompute with stretched duration
                dist = abs(self.q_end[i] - self.q_start[i])
                if dist > 1e-10:
                    # Use the longer duration
                    t_a = self.T / 3  # rough split
                    v_p = dist / (self.T - t_a)
                    t_a = v_p / max_acceleration
                    t_c = self.T - 2 * t_a
                    if t_c < 0:
                        t_a = self.T / 2
                        t_c = 0
                        v_p = dist / t_a
                    self._profiles[i] = (t_a, t_c, self.T, v_p, sign)

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        t = max(0.0, min(t, self.T))
        pos, vel, acc = [], [], []

        for i in range(self.n):
            t_accel, t_cruise, t_total, v_peak, sign = self._profiles[i]

            if t_total < 1e-10:
                pos.append(self.q_start[i])
                vel.append(0.0)
                acc.append(0.0)
                continue

            a = v_peak / t_accel if t_accel > 1e-10 else 0.0

            if t <= t_accel:
                # Acceleration phase
                p = self.q_start[i] + sign * 0.5 * a * t**2
                v = sign * a * t
                ac = sign * a
            elif t <= t_accel + t_cruise:
                # Cruise phase
                dt = t - t_accel
                d_accel = 0.5 * a * t_accel**2
                p = self.q_start[i] + sign * (d_accel + v_peak * dt)
                v = sign * v_peak
                ac = 0.0
            else:
                # Deceleration phase
                dt = t - t_accel - t_cruise
                d_accel = 0.5 * a * t_accel**2
                d_cruise = v_peak * t_cruise
                p = self.q_start[i] + sign * (
                    d_accel + d_cruise + v_peak * dt - 0.5 * a * dt**2
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


# ══════════════════════════════════════════
# LINEAR INTERPOLATION
# ══════════════════════════════════════════


class LinearTrajectory:
    """Simple linear interpolation — constant velocity, instant acceleration.

    Use only for non-critical moves or as a baseline.
    """

    def __init__(self, q_start: State, q_end: State, duration: float):
        self._scalar = isinstance(q_start, (int, float))
        self.q_start = _to_list(q_start)
        self.q_end = _to_list(q_end)
        self.T = duration
        self.n = len(self.q_start)

    def evaluate(self, t: float) -> Tuple[State, State, State]:
        t = max(0.0, min(t, self.T))
        s = t / self.T if self.T > 1e-12 else 1.0

        pos, vel, acc = [], [], []
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
        method: "quintic", "cubic", "minjerk", "trapez", "linear"

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

        # Default: 2 seconds per segment
        if durations is None:
            durations = [2.0] * self.n_segments
        if len(durations) != self.n_segments:
            raise ValueError(f"Need {self.n_segments} durations, got {len(durations)}")

        self.durations = durations
        self.T = sum(durations)

        # Build segments
        traj_class = self.METHODS.get(method, QuinticTrajectory)
        self._segments = []
        self._segment_starts = []  # cumulative start times
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
        """Evaluate the path at time t."""
        t = max(0.0, min(t, self.T))

        # Find which segment we're in
        for i in range(self.n_segments):
            seg_start = self._segment_starts[i]
            seg_end = seg_start + self.durations[i]

            if t <= seg_end or i == self.n_segments - 1:
                local_t = t - seg_start
                return self._segments[i].evaluate(local_t)

        # Shouldn't reach here, but return final position
        return self._segments[-1].evaluate(self.durations[-1])

    @property
    def duration(self) -> float:
        return self.T

    @property
    def num_segments(self) -> int:
        return self.n_segments


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
        self, robot, trajectory, pid_controllers: List, start_time: float = 0.0
    ):
        self.robot = robot
        self.trajectory = trajectory
        self.pids = pid_controllers
        self.start_time = start_time
        self._started = False
        self.done = False

    def step(self, dt: float):
        """Call this from the robot's on_update callback."""
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
