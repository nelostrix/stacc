"""stacc.sim — Simulation engine, physics backends, worlds, recording, and RL."""

from stacc.sim.engine import SimulationEngine, SimulationConfig, SimulationState, simulate
from stacc.sim.world import World, WorldObject, Terrain
from stacc.sim.recorder import SimRecorder, SimRecording, SimFrame, load_recording, replay_recording
from stacc.sim.gym_env import StaccEnv, reach_target_reward, balance_reward, locomotion_reward

__all__ = [
    "SimulationEngine",
    "SimulationConfig",
    "SimulationState",
    "simulate",
    "World",
    "WorldObject",
    "Terrain",
    "SimRecorder",
    "SimRecording",
    "SimFrame",
    "load_recording",
    "replay_recording",
    "StaccEnv",
    "reach_target_reward",
    "balance_reward",
    "locomotion_reward",
]
