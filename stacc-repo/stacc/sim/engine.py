"""Simulation engine — the core loop that steps physics, sensors, and control.

This is the orchestrator that ties together the physics backend (MuJoCo/Bullet),
sensor simulation, and the user's control logic.
"""
from __future__ import annotations
import time
from typing import Optional, Dict, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from stacc.core.robot_base import Robot


class SimulationConfig:
    """Configuration for a simulation run."""

    def __init__(
        self,
        duration: float = 30.0,
        timestep: float = 0.001,
        render: bool = True,
        headless: bool = False,
        physics_engine: str = "mujoco",
        gravity: Optional[list] = None,
        terrain: str = "flat_ground",
        realtime: bool = True,
        record: bool = False,
        record_path: Optional[str] = None,
        **kwargs,
    ):
        self.duration = duration
        self.timestep = timestep
        self.render = render and not headless
        self.headless = headless
        self.physics_engine = physics_engine
        self.gravity = gravity or [0, 0, -9.81]
        self.terrain = terrain
        self.realtime = realtime
        self.record = record
        self.record_path = record_path
        self.extra = kwargs


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

    def _init_physics(self):
        """Initialize the physics backend."""
        engine = self.config.physics_engine.lower()

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
                print(f"  [sim] PyBullet not available, falling back to internal physics")
                self._physics_backend = _InternalPhysics(self.robot, self.config)
        else:
            self._physics_backend = _InternalPhysics(self.robot, self.config)

    def run(self):
        """Run the simulation for the configured duration."""
        print(f"  [sim] Starting simulation: {self.config.physics_engine}, "
              f"dt={self.config.timestep}s, duration={self.config.duration}s")

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

                # Step physics
                self._physics_backend.step(self.config.timestep)

                # Update sensors
                for sensor in self.robot.sensors:
                    if sensor.enabled and sensor.should_update(self.state.time):
                        sensor.read()

                # Run user control loop
                self.robot._tick(self.config.timestep)

                # Update state
                self.state.time += self.config.timestep
                self.state.step = step
                self.state.physics_time_ms = (time.time() - step_start) * 1000

                # Real-time sync
                if self.config.realtime and not self.config.headless:
                    elapsed = time.time() - wall_start
                    sim_time = self.state.time
                    if sim_time > elapsed:
                        time.sleep(sim_time - elapsed)

                # FPS calculation
                if step > 0 and step % 1000 == 0:
                    elapsed = time.time() - wall_start
                    self.state.fps = step / elapsed if elapsed > 0 else 0
                    print(f"  [sim] t={self.state.time:.1f}s, "
                          f"step={step}/{total_steps}, "
                          f"fps={self.state.fps:.0f}")

        except KeyboardInterrupt:
            print("\n  [sim] Interrupted by user")

        finally:
            self.state.running = False
            self.robot._shutdown()
            elapsed = time.time() - wall_start
            ratio = self.state.time / elapsed if elapsed > 0 else 0
            print(f"  [sim] Completed: {self.state.time:.1f}s simulated "
                  f"in {elapsed:.1f}s wall time ({ratio:.1f}x realtime)")

    def pause(self):
        self.state.paused = True

    def resume(self):
        self.state.paused = False

    def stop(self):
        self.state.running = False


class _InternalPhysics:
    """Minimal internal physics engine (used when MuJoCo/Bullet unavailable).

    This is a simple Euler integrator for basic testing. Not physically
    accurate — use MuJoCo or Bullet for real simulation.
    """

    def __init__(self, robot: "Robot", config: SimulationConfig):
        self.robot = robot
        self.gravity = config.gravity

    def step(self, dt: float):
        """Euler integration step."""
        state = self.robot._state
        for i in range(len(state["joint_positions"])):
            # Simple: apply torque as acceleration (mass=1 assumption)
            accel = state["joint_torques"][i]
            state["joint_velocities"][i] += accel * dt
            state["joint_positions"][i] += state["joint_velocities"][i] * dt

            # Clamp to joint limits
            if i < len(self.robot.joints):
                joint = self.robot.joints[i]
                state["joint_positions"][i] = joint.clamp(state["joint_positions"][i])


# ── Module-level convenience function ──

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
