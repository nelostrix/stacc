"""Simulation engine — the core loop that steps physics, sensors, and control.

This is the orchestrator that ties together the physics backend (MuJoCo/Bullet),
sensor simulation, and the user's control logic.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Dict, Optional, List

if TYPE_CHECKING:
    from stacc.core.robot_base import Robot


_VALID_ENGINES = ("mujoco", "bullet", "internal")


class SimulationConfig:
    """Configuration for a simulation run.

    Args:
        duration: Simulation duration in seconds (must be > 0)
        timestep: Physics timestep in seconds (must be > 0)
        render: Whether to render the simulation visually
        headless: Run without display (overrides render if True)
        physics_engine: One of "mujoco", "bullet", "internal"
        gravity: Gravity vector [x, y, z]
        terrain: Terrain type string
        realtime: Whether to sync with real time
        record: Whether to record the simulation
        record_path: Path for recording file
    """

    def __init__(
        self,
        duration: float = 30.0,
        timestep: float = 0.001,
        render: bool = True,
        headless: bool = False,
        physics_engine: str = "mujoco",
        gravity: Optional[List[float]] = None,
        terrain: str = "flat_ground",
        realtime: bool = True,
        record: bool = False,
        record_path: Optional[str] = None,
        **kwargs,
    ):
        if duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")
        if timestep <= 0:
            raise ValueError(f"timestep must be > 0, got {timestep}")
        pe = physics_engine.lower()
        if pe not in _VALID_ENGINES:
            raise ValueError(
                f"physics_engine must be one of {_VALID_ENGINES}, got '{physics_engine}'"
            )

        self.duration = duration
        self.timestep = timestep
        self.render = render and not headless
        self.headless = headless
        self.physics_engine = pe
        self.gravity = gravity or [0, 0, -9.81]
        self.terrain = terrain
        self.realtime = realtime
        self.record = record
        self.record_path = record_path
        self.extra = kwargs

    def __repr__(self) -> str:
        return (
            f"SimulationConfig(duration={self.duration}s, timestep={self.timestep}s, "
            f"engine='{self.physics_engine}')"
        )


class SimulationState:
    """Tracks the current state of a running simulation."""

    def __init__(self):
        self.time: float = 0.0
        self.step: int = 0
        self.running: bool = False
        self.paused: bool = False
        self.fps: float = 0.0
        self.physics_time_ms: float = 0.0
        self.render_time_ms: float = 0.0

    def reset(self):
        """Reset simulation state to initial values."""
        self.time = 0.0
        self.step = 0
        self.running = False
        self.paused = False
        self.fps = 0.0
        self.physics_time_ms = 0.0
        self.render_time_ms = 0.0

    def __repr__(self) -> str:
        return (
            f"SimulationState(time={self.time:.1f}s, step={self.step}, "
            f"running={self.running}, paused={self.paused})"
        )


class SimulationEngine:
    """Main simulation loop.

    Coordinates physics stepping, sensor updates, control callbacks,
    and rendering for a single robot in a world.

    Usage:
        engine = SimulationEngine(robot, config)
        engine.run()
    """

    def __init__(self, robot: "Robot", config: Optional[SimulationConfig] = None):
        self.robot = robot
        self.config = config or SimulationConfig()
        self.state = SimulationState()
        self._physics_backend = None
        self._renderer = None

    def reset(self):
        """Reset engine state and clear physics backend."""
        self.state.reset()
        self._physics_backend = None
        self._renderer = None

    def _init_physics(self):
        engine = self.config.physics_engine

        if engine == "mujoco":
            try:
                from stacc.sim.mujoco_backend import MuJoCoBackend

                self._physics_backend = MuJoCoBackend(self.robot, self.config)
            except ImportError:
                print(f"  [sim] MuJoCo not available, falling back to internal physics")
                self._physics_backend = _InternalPhysics(self.robot, self.config)
        elif engine == "bullet":
            try:
                from stacc.sim.bullet_backend import BulletBackend

                self._physics_backend = BulletBackend(self.robot, self.config)
            except ImportError:
                print(
                    f"  [sim] PyBullet not available, falling back to internal physics"
                )
                self._physics_backend = _InternalPhysics(self.robot, self.config)
        else:
            self._physics_backend = _InternalPhysics(self.robot, self.config)

    def run(self):
        """Run the simulation for the configured duration."""
        print(
            f"  [sim] Starting simulation: {self.config.physics_engine}, "
            f"dt={self.config.timestep}s, duration={self.config.duration}s"
        )

        self._init_physics()
        self.state.running = True
        self.robot._startup()

        total_steps = int(self.config.duration / self.config.timestep)
        wall_start = time.time()

        try:
            for step in range(total_steps):
                if not self.state.running:
                    break
                if self.state.paused:
                    time.sleep(0.01)
                    continue

                step_start = time.time()

                self._physics_backend.step(self.config.timestep)

                for sensor in self.robot.sensors:
                    if sensor.enabled and sensor.should_update(self.state.time):
                        sensor.read()

                self.robot._tick(self.config.timestep)

                self.state.time += self.config.timestep
                self.state.step = step
                self.state.physics_time_ms = (time.time() - step_start) * 1000

                if self.config.realtime and not self.config.headless:
                    elapsed = time.time() - wall_start
                    sim_time = self.state.time
                    if sim_time > elapsed:
                        time.sleep(sim_time - elapsed)

                if step > 0 and step % 1000 == 0:
                    elapsed = time.time() - wall_start
                    self.state.fps = step / elapsed if elapsed > 0 else 0
                    print(
                        f"  [sim] t={self.state.time:.1f}s, "
                        f"step={step}/{total_steps}, "
                        f"fps={self.state.fps:.0f}"
                    )

        except KeyboardInterrupt:
            print("\n  [sim] Interrupted by user")

        finally:
            self.state.running = False
            self.robot._shutdown()
            if hasattr(self._physics_backend, "close"):
                self._physics_backend.close()
            elapsed = time.time() - wall_start
            ratio = self.state.time / elapsed if elapsed > 0 else 0
            print(
                f"  [sim] Completed: {self.state.time:.1f}s simulated "
                f"in {elapsed:.1f}s wall time ({ratio:.1f}x realtime)"
            )

    def pause(self):
        self.state.paused = True

    def resume(self):
        self.state.paused = False

    def stop(self):
        self.state.running = False

    def __repr__(self) -> str:
        return (
            f"SimulationEngine(robot='{self.robot.name}', "
            f"engine='{self.config.physics_engine}', "
            f"state={'running' if self.state.running else 'idle'})"
        )


class _InternalPhysics:
    """Minimal internal physics engine (used when MuJoCo/Bullet unavailable).

    This is a simple Euler integrator for basic testing. Not physically
    accurate — use MuJoCo or Bullet for real simulation.
    """

    def __init__(self, robot: "Robot", config: SimulationConfig):
        self.robot = robot
        self.gravity = config.gravity

    def reset(self):
        """No state to reset for internal physics."""

    def step(self, dt: float):
        state = self.robot._state
        for i in range(len(state["joint_positions"])):
            accel = state["joint_torques"][i]
            state["joint_velocities"][i] += accel * dt
            state["joint_positions"][i] += state["joint_velocities"][i] * dt

            if i < len(self.robot.joints):
                joint = self.robot.joints[i]
                state["joint_positions"][i] = joint.clamp(state["joint_positions"][i])

    def __repr__(self) -> str:
        return f"_InternalPhysics(robot='{self.robot.name}', gravity={self.gravity})"


def simulate(
    robot: "Robot",
    duration: float = 30.0,
    render: bool = True,
    headless: bool = False,
    physics: str = "mujoco",
    **kwargs,
):
    """Convenience function to simulate a robot.

    Usage:
        import stacc
        stacc.simulate(robot, duration=30, render=True)
    """
    config = SimulationConfig(
        duration=duration,
        render=render,
        headless=headless,
        physics_engine=physics,
        **kwargs,
    )
    engine = SimulationEngine(robot, config)
    engine.run()
    return engine
