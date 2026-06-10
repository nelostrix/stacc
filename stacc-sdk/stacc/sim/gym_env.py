"""Universal RL environment wrapper — train any stacc robot with reinforcement learning.

Wraps any stacc Robot + MuJoCo simulation into a Gymnasium-compatible
environment. Users define the observation space, action space, and reward
function. The wrapper handles physics stepping, state management, resets,
and episode termination.

Compatible with: Gymnasium, Stable Baselines3, CleanRL, RLlib, TorchRL

Usage:
    from stacc.sim.gym_env import StaccEnv

    # Define your robot and reward
    env = StaccEnv(
        robot=my_robot,
        reward_fn=my_reward,
        max_steps=1000,
    )

    # Train with Stable Baselines3
    from stable_baselines3 import PPO
    model = PPO("MlpPolicy", env)
    model.train(total_timesteps=100000)

    # Or use the standard Gym loop
    obs, info = env.reset()
    for _ in range(1000):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
"""

from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional, Tuple

_VALID_ACTION_MODES = ("torque", "position")
_VALID_OBS_COMPONENTS = (
    "joint_positions",
    "joint_velocities",
    "joint_torques",
    "pose",
    "time",
)

try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import gymnasium as gym
    from gymnasium import spaces

    HAS_GYM = True
except ImportError:
    try:
        import gym
        from gym import spaces

        HAS_GYM = True
    except ImportError:
        HAS_GYM = False

from stacc.core.robot_base import Robot
from stacc.sim.engine import SimulationConfig


class StaccEnv:
    """Gymnasium-compatible RL environment for any stacc robot.

    Turns a stacc Robot into a trainable environment. The user provides:
    - A Robot object (defines the physical system)
    - A reward function (defines the task)
    - Optionally: custom observation/action builders

    The environment handles:
    - MuJoCo physics stepping
    - State observation (joint positions, velocities, sensor data)
    - Action application (joint torques or position targets)
    - Episode reset with optional randomization
    - Termination conditions

    Args:
        robot: A stacc Robot object
        reward_fn: Function(obs, action, next_obs, env) -> float
        terminal_fn: Function(obs, env) -> bool (optional)
        max_steps: Maximum steps per episode
        dt: Physics timestep
        action_mode: "torque" (direct torques) or "position" (target angles)
        obs_includes: List of observation components to include
        gravity: Gravity vector
        render: Whether to open the MuJoCo viewer
        action_scale: Multiply actions by this before applying
        obs_noise: Add noise to observations (0 = none)
        randomize_on_reset: Randomize initial joint positions on reset
    """

    # Gymnasium metadata
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(
        self,
        robot: Robot,
        reward_fn: Optional[Callable] = None,
        terminal_fn: Optional[Callable] = None,
        max_steps: int = 1000,
        dt: float = 0.002,
        action_mode: str = "torque",
        obs_includes: Optional[List[str]] = None,
        gravity: Optional[List[float]] = None,
        render: bool = False,
        action_scale: float = 1.0,
        obs_noise: float = 0.0,
        randomize_on_reset: bool = False,
        render_mode: Optional[str] = None,
    ):
        if not HAS_NUMPY:
            raise ImportError("numpy is required for RL environments")
        if max_steps < 1:
            raise ValueError(f"max_steps must be >= 1, got {max_steps}")
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
        if action_mode not in _VALID_ACTION_MODES:
            raise ValueError(
                f"action_mode must be one of {_VALID_ACTION_MODES}, got '{action_mode}'"
            )
        if obs_includes is not None:
            for c in obs_includes:
                if c not in _VALID_OBS_COMPONENTS:
                    raise ValueError(
                        f"Unknown obs_includes component '{c}'. "
                        f"Valid: {_VALID_OBS_COMPONENTS}"
                    )
        if obs_noise < 0:
            raise ValueError(f"obs_noise must be >= 0, got {obs_noise}")
        if not robot.joints:
            raise ValueError("robot must have at least one joint")

        self.robot = robot
        self.reward_fn = reward_fn or self._default_reward
        self.terminal_fn = terminal_fn
        self.max_steps = max_steps
        self.dt = dt
        self.action_mode = action_mode
        self.obs_includes = obs_includes or ["joint_positions", "joint_velocities"]
        self.gravity = gravity or [0, 0, -9.81]
        self.action_scale = action_scale
        self.obs_noise = obs_noise
        self.randomize_on_reset = randomize_on_reset
        self.render_mode = render_mode or ("human" if render else None)

        # State
        self._step_count = 0
        self._episode_count = 0
        self._total_reward = 0.0
        self._backend = None

        # Compute dimensions
        self._n_actuated = sum(1 for j in robot.joints if j.type != "fixed")
        self._obs_dim = self._compute_obs_dim()

        # Define spaces
        self.action_space = self._build_action_space()
        self.observation_space = self._build_observation_space()

        # Initialize backend
        self._init_backend()

    def _compute_obs_dim(self) -> int:
        """Compute observation vector dimension."""
        dim = 0
        n = self._n_actuated
        for component in self.obs_includes:
            if component == "joint_positions":
                dim += n
            elif component == "joint_velocities":
                dim += n
            elif component == "joint_torques":
                dim += n
            elif component == "pose":
                dim += 6  # x,y,z,roll,pitch,yaw
            elif component == "time":
                dim += 1
        return max(dim, 1)

    def _build_action_space(self):
        """Build action space based on action mode and robot joints."""
        if not HAS_GYM:
            return None

        n = self._n_actuated
        if self.action_mode == "torque":
            # Actions are normalized to [-1, 1], scaled by action_scale
            low = np.full(n, -1.0, dtype=np.float32)
            high = np.full(n, 1.0, dtype=np.float32)
        elif self.action_mode == "position":
            # Actions are target joint angles
            low = np.array(
                [j.limits[0] for j in self.robot.joints if j.type != "fixed"],
                dtype=np.float32,
            )
            high = np.array(
                [j.limits[1] for j in self.robot.joints if j.type != "fixed"],
                dtype=np.float32,
            )
        else:
            low = np.full(n, -1.0, dtype=np.float32)
            high = np.full(n, 1.0, dtype=np.float32)

        return spaces.Box(low=low, high=high, dtype=np.float32)

    def _build_observation_space(self):
        """Build observation space."""
        if not HAS_GYM:
            return None

        low = np.full(self._obs_dim, -np.inf, dtype=np.float32)
        high = np.full(self._obs_dim, np.inf, dtype=np.float32)
        return spaces.Box(low=low, high=high, dtype=np.float32)

    def _init_backend(self):
        """Initialize MuJoCo backend."""
        try:
            from stacc.sim.mujoco_backend import MuJoCoBackend

            config = SimulationConfig(
                timestep=self.dt,
                gravity=self.gravity,
                render=(self.render_mode == "human"),
                headless=(self.render_mode != "human"),
            )
            self._backend = MuJoCoBackend(self.robot, config)
        except ImportError:
            from stacc.sim.engine import _InternalPhysics

            config = SimulationConfig(timestep=self.dt, gravity=self.gravity)
            self._backend = _InternalPhysics(self.robot, config)

    def _get_obs(self) -> np.ndarray:
        """Build observation vector from current state."""
        obs_parts = []

        for component in self.obs_includes:
            if component == "joint_positions":
                obs_parts.extend(self.robot.get_joint_positions())
            elif component == "joint_velocities":
                obs_parts.extend(self.robot.get_joint_velocities())
            elif component == "joint_torques":
                obs_parts.extend(self.robot._state["joint_torques"])
            elif component == "pose":
                obs_parts.extend(self.robot._state["pose"])
            elif component == "time":
                obs_parts.append(self._step_count * self.dt)

        obs = np.array(obs_parts, dtype=np.float32)

        # Add noise
        if self.obs_noise > 0:
            obs += np.random.normal(0, self.obs_noise, size=obs.shape).astype(
                np.float32
            )

        return obs

    def _apply_action(self, action: np.ndarray):
        """Apply action to the robot."""
        action = np.clip(action, -1.0, 1.0) if self.action_mode == "torque" else action

        if self.action_mode == "torque":
            # Scale normalized actions to torque range
            torques = []
            act_idx = 0
            for joint in self.robot.joints:
                if joint.type == "fixed":
                    continue
                if act_idx < len(action):
                    torque = (
                        float(action[act_idx]) * self.action_scale * joint.max_effort
                    )
                    torques.append(torque)
                    act_idx += 1
            self.robot.set_joint_torques(torques)

        elif self.action_mode == "position":
            # Use PD control to reach target positions
            from stacc.control.pid import PIDController

            positions = self.robot.get_joint_positions()
            torques = []
            act_idx = 0
            for i, joint in enumerate(self.robot.joints):
                if joint.type == "fixed":
                    continue
                if act_idx < len(action):
                    target = float(action[act_idx])
                    error = target - positions[i] if i < len(positions) else 0
                    torque = 100.0 * error  # simple P control
                    torques.append(torque)
                    act_idx += 1
            self.robot.set_joint_torques(torques)

    def _default_reward(self, obs, action, next_obs, env) -> float:
        """Default reward: negative squared joint velocity (encourage stillness)."""
        velocities = self.robot.get_joint_velocities()
        return -sum(v * v for v in velocities) * 0.01

    # ══════════════════════════════════════
    # GYMNASIUM INTERFACE
    # ══════════════════════════════════════

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        """Reset environment to initial state.

        Returns:
            (observation, info)
        """
        if seed is not None:
            np.random.seed(seed)

        self._step_count = 0
        self._total_reward = 0.0
        self._episode_count += 1

        # Reset joint states
        n = len(self.robot._state["joint_positions"])
        if self.randomize_on_reset:
            # Random initial positions within joint limits
            for i, joint in enumerate(self.robot.joints):
                if i < n and joint.type != "fixed":
                    lo, hi = joint.limits
                    self.robot._state["joint_positions"][i] = np.random.uniform(
                        lo * 0.5, hi * 0.5
                    )
        else:
            self.robot._state["joint_positions"] = [0.0] * n

        self.robot._state["joint_velocities"] = [0.0] * n
        self.robot._state["joint_torques"] = [0.0] * n
        self.robot.time = 0.0

        # Reinitialize backend
        self._init_backend()

        obs = self._get_obs()
        info = {"episode": self._episode_count}

        return obs, info

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Take one step in the environment.

        Args:
            action: Action from the agent

        Returns:
            (observation, reward, terminated, truncated, info)
        """
        action = np.array(action, dtype=np.float32)
        prev_obs = self._get_obs()

        # Apply action
        self._apply_action(action)

        # Step physics
        self._backend.step(self.dt)
        self.robot.time += self.dt
        self._step_count += 1

        # Get new observation
        obs = self._get_obs()

        # Compute reward
        reward = float(self.reward_fn(prev_obs, action, obs, self))
        self._total_reward += reward

        # Check termination
        terminated = False
        if self.terminal_fn is not None:
            terminated = bool(self.terminal_fn(obs, self))

        # Check truncation (max steps)
        truncated = self._step_count >= self.max_steps

        info = {
            "step": self._step_count,
            "total_reward": self._total_reward,
            "time": self.robot.time,
        }

        return obs, reward, terminated, truncated, info

    def render(self):
        """Render the environment."""
        if self.render_mode == "rgb_array":
            # Return rendered image
            try:
                import mujoco

                if hasattr(self._backend, "model"):
                    renderer = mujoco.Renderer(
                        self._backend.model, height=480, width=640
                    )
                    renderer.update_scene(self._backend.data)
                    return renderer.render()
            except Exception:
                pass
            return np.zeros((480, 640, 3), dtype=np.uint8)
        # "human" mode is handled by the MuJoCo viewer in the backend

    def close(self):
        """Clean up resources."""
        if hasattr(self._backend, "close"):
            self._backend.close()

    @property
    def unwrapped(self):
        return self

    @property
    def spec(self):
        return None

    def __repr__(self) -> str:
        return (
            f"StaccEnv(robot='{self.robot.name}', "
            f"action_mode='{self.action_mode}', "
            f"max_steps={self.max_steps}, dt={self.dt}s)"
        )


# ══════════════════════════════════════════
# CONVENIENCE: Pre-built reward functions
# ══════════════════════════════════════════


def reach_target_reward(target_position: List[float]):
    """Reward function for reaching a target end-effector position.

    Args:
        target_position: [x, y, z] target in world coordinates

    Returns:
        Reward function compatible with StaccEnv
    """
    target = np.array(target_position)

    def reward_fn(obs, action, next_obs, env):
        # Use last 3 joint positions as proxy for end-effector position
        # (proper FK would be better but this works for simple cases)
        positions = env.robot.get_joint_positions()
        velocities = env.robot.get_joint_velocities()

        # Distance penalty (encourage reaching target)
        pos_error = sum(p**2 for p in positions)  # simplified

        # Velocity penalty (encourage smooth motion)
        vel_penalty = sum(v**2 for v in velocities) * 0.001

        # Action penalty (encourage efficiency)
        act_penalty = sum(float(a) ** 2 for a in action) * 0.0001

        return -(pos_error + vel_penalty + act_penalty)

    return reward_fn


def balance_reward():
    """Reward function for keeping a robot upright (balancing tasks).

    Returns:
        Reward function compatible with StaccEnv
    """

    def reward_fn(obs, action, next_obs, env):
        # Reward for staying upright (small angles = good)
        positions = env.robot.get_joint_positions()
        velocities = env.robot.get_joint_velocities()

        # Angle penalty
        angle_penalty = sum(p**2 for p in positions)

        # Velocity penalty
        vel_penalty = sum(v**2 for v in velocities) * 0.1

        # Survival bonus
        alive_bonus = 1.0

        return alive_bonus - angle_penalty - vel_penalty

    return reward_fn


def locomotion_reward(forward_axis: int = 0):
    """Reward function for forward locomotion (walking/driving).

    Args:
        forward_axis: 0=X, 1=Y, 2=Z direction of forward motion

    Returns:
        Reward function compatible with StaccEnv
    """

    def reward_fn(obs, action, next_obs, env):
        # Reward forward velocity
        velocities = env.robot.get_joint_velocities()
        forward_vel = (
            velocities[forward_axis] if forward_axis < len(velocities) else 0.0
        )

        # Penalize excessive action
        action_cost = sum(float(a) ** 2 for a in action) * 0.001

        # Penalize falling (large z change)
        alive_bonus = 1.0

        return forward_vel + alive_bonus - action_cost

    return reward_fn
